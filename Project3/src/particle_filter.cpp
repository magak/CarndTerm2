/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

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
	num_particles = 50;
	// random engine
	default_random_engine gen;

	// normal distributions for x,y and theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for(int i = 0; i < num_particles; i++){
		Particle prtcl;
		prtcl.weight = 1;
		prtcl.id = i;
		prtcl.x = dist_x(gen);
		prtcl.y = dist_y(gen);
		prtcl.theta = dist_theta(gen);
		particles.push_back(prtcl);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// random engine
	default_random_engine gen;

	// normal distributions for x,y and theta
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);

	//avoid division by zero
	if (fabs(yaw_rate) > 0.0001) {
		for(int i = 0; i < num_particles; i++){
			particles[i].x = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta)) + dist_x(gen);
			particles[i].y = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t)) + dist_y(gen);
			particles[i].theta = particles[i].theta + yaw_rate*delta_t + dist_theta(gen);
		}
	}
	else {
		for(int i = 0; i < num_particles; i++){
			particles[i].x = particles[i].x + velocity*delta_t*cos(particles[i].theta) + dist_x(gen);
			particles[i].y = particles[i].y + velocity*delta_t*sin(particles[i].theta) + dist_y(gen);
			particles[i].theta = particles[i].theta + dist_theta(gen);
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for(int i = 0; i < observations.size(); i++)
	{
		if(predicted.size() == 0){
			continue;
		}

		observations[i].id = 0;
		double min_distance = dist(observations[i].x, observations[i].y, predicted[0].x, predicted[0].y);

		for(int j = 1; j < predicted.size(); j++){
			double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if(distance < min_distance){
				min_distance = distance;
				observations[i].id = j;//predicted[j].id;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

	//
	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];
	double gauss_norm = 1/(2*M_PI*sig_x*sig_y);

	// loop through particles;
	for(int i = 0; i < num_particles; i++){
		std::vector<LandmarkObs> predicted;

		// keep the landmarks within the sensor range with respect to particular particle
		for(int j = 0; j < map_landmarks.landmark_list.size(); j++){
			if(dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f) < sensor_range){
				LandmarkObs lm;
				lm.id = map_landmarks.landmark_list[j].id_i;
				lm.x = map_landmarks.landmark_list[j].x_f;
				lm.y = map_landmarks.landmark_list[j].y_f;
				predicted.push_back(lm);
			}
		}

		// transform observations
		std::vector<LandmarkObs> transfd_obsrvs;
		for(int k = 0; k < observations.size(); k++){
			LandmarkObs obs;
			obs.x = particles[i].x + observations[k].x*cos(particles[i].theta)-observations[k].y*sin(particles[i].theta);
			obs.y = particles[i].y + observations[k].x*sin(particles[i].theta)+observations[k].y*cos(particles[i].theta);
			obs.id = -1;
			transfd_obsrvs.push_back(obs);
		}

		// Data association
		dataAssociation(predicted, transfd_obsrvs);

		// Setting associations
		std::vector<int> associations;
		std::vector<double> sense_x;
		std::vector<double> sense_y;
		for(int k = 0; k < transfd_obsrvs.size(); k++){
			if(transfd_obsrvs[k].id == -1)
				continue;

			associations.push_back(predicted[transfd_obsrvs[k].id].id);
			sense_x.push_back(transfd_obsrvs[k].x);
			sense_y.push_back(transfd_obsrvs[k].y);
		}

		particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);

		// Updating the weights
		if(associations.size()>0 && transfd_obsrvs.size()>0){
			double weight = 1;
			for(int k = 0; k < transfd_obsrvs.size(); k++){
				double mu_x = predicted[transfd_obsrvs[k].id].x;
				double mu_y = predicted[transfd_obsrvs[k].id].y;
				double x_obs = transfd_obsrvs[k].x;
				double y_obs = transfd_obsrvs[k].y;
				double exponent = ((x_obs - mu_x)*(x_obs - mu_x))/(2 * sig_x*sig_x) + ((y_obs - mu_y)*(y_obs - mu_y))/(2 * sig_y*sig_y);

				weight *= gauss_norm*exp(-exponent);
			}

			particles[i].weight = weight;
		}
		else{
			particles[i].weight = 0;
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::default_random_engine gen;

	std::vector<double> distr;
	for(int i = 0; i < num_particles; i++){
		distr.push_back(particles[i].weight);
	}
	std::discrete_distribution<int> distribution(distr.begin(), distr.end());

	std::vector<Particle> new_particles;
	for(int i = 0; i < num_particles; i++){
		int index = distribution(gen);

		new_particles.push_back(particles[index]);
	}

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
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
