#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  if (is_initialized) {
    return;
  }

  // Initializing the number of particles.
  num_particles = 100;

  // Estimates of GPS position with gaussian noise
  normal_distribution<double> gps_x_dist(x, std[0]);
  normal_distribution<double> gps_y_dist(y, std[1]);
  normal_distribution<double> gps_theta_dist(theta, std[2]);


  // Initialize all particles' to the first GPS measurement with random Gaussian noise.
  for (int i=0; i<num_particles; ++i) {
    Particle particle = {i,
                         x + gps_x_dist(gen),
                         y + gps_y_dist(gen),
                         theta + gps_theta_dist(gen),
                         1};

    particles.push_back(particle);
  }
  is_initialized = true;
}

/**
 * Predict each particle's pose based on motion measurements of velocity and yaw-rate.
 * @param delta_t  Time difference between the motion step update.
 * @param std_pos  Noise associated with pose.
 * @param velocity Observed velocity.
 * @param yaw_rate Observed yaw-rate.
 */
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // Update particle's position and yaw-angle based on yaw rate and velocity.
  for (int i=0; i<num_particles; ++i) {
    double theta = particles[i].theta;
    // Yaw rate close to 0 degrees; no change in yaw.
    if (fabs(yaw_rate) < 0.00001) {
      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);
    }
    else {  // Take into account the heading, as the car moves.
      double yawrate_delt = yaw_rate * delta_t;
      double vel_by_yawrate = velocity / yaw_rate;
      particles[i].x += vel_by_yawrate * (sin(theta + yawrate_delt) - sin(theta));
      particles[i].y += vel_by_yawrate * (cos(theta) - cos(theta + yawrate_delt));
      particles[i].theta += yawrate_delt;
    }

    // Add random Gaussian noise.
    normal_distribution<double> x_noise_dist(particles[i].x, std_pos[0]);
    normal_distribution<double> y_noise_dist(particles[i].y, std_pos[1]);
    normal_distribution<double> theta_noise_dist(particles[i].theta, std_pos[2]);

    particles[i].x += x_noise_dist(gen);
    particles[i].y += y_noise_dist(gen);
    particles[i].theta += theta_noise_dist(gen);
  }
}

/**
 * Associates a closest landmark to each observation.
 * @param predicted  Landmarks within range of a particle's predicted pose.
 * @param observations  Transformed observations sensed by vehicle.
 */
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  // For each observation, associate the predicted landmark.
  for (int i=0; i<observations.size(); ++i) {
    double minDistance = numeric_limits<double>::max();
    int mapId = -1;

    for (int j=0; j<predicted.size(); ++j) {
      double px = predicted[j].x;
      double py = predicted[j].y;
      double dx = observations[i].x - px;
      double dy = observations[i].y - py;
      double distance = dx*dx + dy*dy;
      if (distance < minDistance) {
        mapId = predicted[j].id;
        minDistance = distance;
      }
    }
    observations[i].id = mapId;
  }
}

/**
 * Update the weights of each particle using a multi-variate Gaussian distribution
 * @param sensor_range  Maximum range of a sensor.
 * @param std_landmark  Standard deviation associated with a landmark.
 * @param observations  Set of observations in vehicle's coordinate system.
 * @param map_landmarks Landmark features in the map.
 */
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  vector<LandmarkObs> transformedObservations;

  double std_x = std_landmark[0];
  double std_y = std_landmark[1];

  for (int i=0; i<num_particles; ++i) {
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    // Filter for landmarks in range for each particle.
    vector<LandmarkObs> inRangeLandmarks;
    double sensor_range_sq = sensor_range*sensor_range;
    for (int j=0; j<map_landmarks.landmark_list.size(); ++j) {
      int id = map_landmarks.landmark_list[j].id_i;
      float lx = map_landmarks.landmark_list[j].x_f;
      float ly = map_landmarks.landmark_list[j].y_f;
      double dx = x-lx;
      double dy = y-ly;
      if (dx*dx + dy*dy <= sensor_range_sq) {
        inRangeLandmarks.push_back(LandmarkObs{id, lx, ly});
      }
    }

    // Transform observations to map's coordinate system.
    vector<LandmarkObs> transformedObservations;
    for (int j=0; j<observations.size(); ++j) {
      double trans_obsx = observations[j].x*cos(theta) - observations[j].y*sin(theta) + x;
      double trans_obsy = observations[j].x*sin(theta) + observations[j].y*cos(theta) + y;
      transformedObservations.push_back(LandmarkObs{observations[j].id, trans_obsx, trans_obsy});
    }

    // Associate observations with in-range landmarks.
    dataAssociation(inRangeLandmarks, transformedObservations);

    // Reset particle weight.
    particles[i].weight = 1.;
    for (int j=0; j<transformedObservations.size(); ++j) {
      int closestLandmarkId = transformedObservations[j].id;
      double landmarkX, landmarkY;
      int numInRangeLandmarks = inRangeLandmarks.size();
      int k=0;
      bool landmarkFound = false;
      while (!landmarkFound && k < numInRangeLandmarks) {
        if (inRangeLandmarks[k].id == closestLandmarkId)  {  // landmark found
          landmarkX = inRangeLandmarks[k].x;
          landmarkY = inRangeLandmarks[k].y;
          landmarkFound = true;
        }
        ++k;
      }

      // Assign particle weight based on multi-variate Gaussian having a mean of associated landmark position
      // and standard-deviation set to uncertainty in x and y ranges.
      double dx = transformedObservations[j].x - landmarkX;
      double dy = transformedObservations[j].y - landmarkY;

      double exponent = -0.5 * (((dx*dx) / (std_x*std_x)) + ((dy*dy) / (std_y*std_y)));
      double weight = (-1 / 2 * M_PI * std_x * std_y) * exp(exponent);
      if (weight == 0) {
        particles[i].weight *= 0.00001;
      }
      else {
        particles[i].weight *= weight;
      }
    }
  }
}

/**
 * Resample particles with replacement with probability proportional to their weight.
 */
void ParticleFilter::resample() {
  vector<double> weights;
  double maxWeight = numeric_limits<double>::min();

  for_each(particles.begin(), particles.end(), [&weights, &maxWeight](Particle particle)
  {
    weights.push_back(particle.weight);
    if (particle.weight > maxWeight) maxWeight = particle.weight;
  });

  // Generate distributions for sampling-wheel, including one for picking the first particle randomly.
  uniform_real_distribution<double> realDist(0., maxWeight);
  uniform_int_distribution<int> intDist(0, num_particles);

  // Random index for the sampling wheel.
  int index = intDist(gen);
  double beta = 0.0;

  // Sampling wheel.
  vector<Particle> resampledParticles;
  for (int i=0; i<num_particles; ++i) {
    beta += realDist(gen) * 2.0;
    while(beta > weights[index]) {
      beta -= weights[index];
      index = (index+1) % num_particles;
    }
    resampledParticles.push_back(particles[index]);
  }
  particles = resampledParticles;
}

/**
 * @param particle  The particle to assign each listed association, and association's (x,y) world coordinates mapping to
 * @param associations  The landmark id that goes along with each listed association
 * @param sense_x  The associations x mapping already converted to world coordinates
 * @param sense_y  the associations y mapping already converted to world coordinates
 * @return
 */
Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
