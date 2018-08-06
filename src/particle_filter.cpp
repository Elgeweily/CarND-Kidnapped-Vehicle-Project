/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */
# define M_PI 3.14159265359

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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;

	// create Gaussian distributions around GPS mean location and heading
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	default_random_engine gen;

	// initialize each particle by sampling from the Gaussian distributions
	for (unsigned int i = 0; i < num_particles; i++) {
		Particle particle;
		particle.id = i;
		particle.weight = 1.0;
		weights.push_back(1.0);
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particles.push_back(particle);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	
	for (unsigned int i = 0; i < num_particles; i++) {
		if (abs(yaw_rate) < 0.0001) {
			// predict mean location and heading
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
			// theta stays the same
		}

		else {
			particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// create Gaussian distributions around predicted mean location and heading
		normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);

		// randomly sample from the Gaussian distributions to arrive at final predicted location and heading
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (unsigned int i = 0; i < observations.size(); i++) {
		double min_dist = numeric_limits<float>::max(); // initialize min_dist to a very large value
		for (unsigned int j = 0; j < predicted.size(); j++) {
			double cur_dist = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
			if (cur_dist < min_dist) {
				min_dist = cur_dist;
				observations[i].id = predicted[j].id;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	for (unsigned int i = 0; i < particles.size(); i++) {
		double xp = particles[i].x;
		double yp = particles[i].y;
		double theta = particles[i].theta;
		
		// make a list of all landmarks within each particle sensor range
		vector<LandmarkObs> in_range_landmarks;
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			if ((abs(map_landmarks.landmark_list[j].x_f - xp) <= sensor_range) && (abs(map_landmarks.landmark_list[j].y_f - yp) <= sensor_range)) {
				LandmarkObs in_range_landmark;
				in_range_landmark.id = map_landmarks.landmark_list[j].id_i;
				in_range_landmark.x = map_landmarks.landmark_list[j].x_f;
				in_range_landmark.y = map_landmarks.landmark_list[j].y_f;
				in_range_landmarks.push_back(in_range_landmark);
			}
		}

		// transform observations to map coordinates
		vector<LandmarkObs> trans_observations;
		for (unsigned int k = 0; k < observations.size(); k++) {
			double xc = observations[k].x;
			double yc = observations[k].y;
			double xm = xp + cos(theta) * xc - sin(theta) * yc;
			double ym = yp + sin(theta) * xc + cos(theta) * yc;
			LandmarkObs trans_obs;
			trans_obs.x = xm;
			trans_obs.y = ym;
			trans_observations.push_back(trans_obs);
		}

		// associate transformed observations with nearest landmarks (within sensor range)
		dataAssociation(in_range_landmarks, trans_observations);

		double particle_weight = 1.0;
		for (unsigned int l = 0; l < trans_observations.size(); l++) {
			double obs_x = trans_observations[l].x;
			double obs_y = trans_observations[l].y;
			double landmark_x;
			double landmark_y;
			for (unsigned int m = 0; m < in_range_landmarks.size(); m++) {
				if (trans_observations[l].id == in_range_landmarks[m].id) {
					landmark_x = in_range_landmarks[m].x;
					landmark_y = in_range_landmarks[m].y;
				}
			}

			// update weights using multivariate Gaussian distribution equation
			const double denom = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
			const double x_denom = 2 * std_landmark[0] * std_landmark[0];
			const double y_denom = 2 * std_landmark[1] * std_landmark[1];
			particle_weight *= denom * exp(-((pow((obs_x - landmark_x), 2) / x_denom) + (pow((obs_y - landmark_y), 2) / y_denom)));
		}
		particles[i].weight = particle_weight;
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;

	vector<Particle> new_particles;

	discrete_distribution<int> weights_dist(weights.begin(), weights.end());
	for (unsigned int i = 0; i < num_particles; i++) {
		new_particles.push_back(particles[weights_dist(gen)]);
	}
	particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
