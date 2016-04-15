#include <particle_filter/ParticleFilter.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <time.h>

namespace particle_filter {

/**
 * \brief Calculate the probability phi(d, stdev) of a measurement according to a Gaussian distribution.
 * \param[in] d The difference between the measurement and the mean
 * \param[in] stdev The standard deviation of the Gaussian.
 * \return Probability of the measurement.
 */
double ParticleFilter::gaussianProbability(const double& d, const double& stdev) {
	double probability = 0.0;
	double pi = 3.1415926535897;
	double exp = 2.7182818284;

	/*TODO: Calculate the probability of the measurement for a Gaussian distribution with
	  the given mean and standard deviation */
	probability = (1.0/(stdev*pow(2.0*pi, 0.5)))*pow(exp, -d*d/(2.0*stdev*stdev));

	return probability;
}

/**
 * \brief Draw a sample from a Gaussian distribution.
 * \param[in] mean The mean of the Gaussian.
 * \param[in] stdev The standard deviation of the Gaussian.
 * \return A random sample drawn from the given Gaussian distribution.
 */
double ParticleFilter::sampleFromGaussian(const double& mean, const double& stdev) {
	double result = 0.0;
	double sum = 0.0;
	double onePieceOfRand = (2.0*stdev)/(double(RAND_MAX));
	double temp;

	//srand(time(NULL));

	//TODO: draw a sample from a 1D Gaussian
	for(int i=0; i<12; i++) {
		std::cout<<"double(rand())*onePieceOfRand-stdev"<<double(rand())*onePieceOfRand-stdev<<std::endl;
		temp = double(rand())*onePieceOfRand;

		if(temp > 2.0*stdev)
			temp = 2.0*stdev;
		sum += (temp-stdev);
	}

	result = mean + 0.5*sum;

	return result;
}


/**
 * \brief Initializes the position and weights of the particles.
 * \param[in,out] particles The list of particles.
 *
 * The positions should be distributed uniformly in the interval [0, 10].
 * The weights should be equal and sum up to 1.
 */
void ParticleFilter::initParticles(std::vector<Particle>& particles) {
	//TODO: Distribute the particles randomly between [0, 10] with equal weights
	int size = particles.size();
	double interval = 10.0/double(size);

	for(int i=0; i<size; i++) {
		particles[i].x = double(i)*interval;
		particles[i].weight = 1.0/size;
	}
}

/**
 * \brief Normalizes the weights of the particle set so that they sum up to 1.
 * \param[in,out] particles The list of particles.
 */
void ParticleFilter::normalizeWeights(std::vector<Particle>& particles) {
	//TODO: normalize the particles' weights so that they sum up to 1.
	double oriWeightSum = 0.0;
	double norOnePieceWeight = 0.0;

	for(int i=0; i<particles.size(); i++) {
		oriWeightSum += particles[i].weight;
	}

	norOnePieceWeight = 1.0/oriWeightSum;

	for(int i=0; i<particles.size(); i++) {
		particles[i].weight *= norOnePieceWeight;
	}
}

/**
 * \brief Displace the particles according to the robot's movements.
 * \param[in,out] particles The list of particles.
 * \param[in] ux The odometry (displacement) of the robot along the x axis.
 * \param[in] stdev The standard deviation of the motion model.
 */
void ParticleFilter::integrateMotion(std::vector<Particle>& particles, const double& ux, const double& stdev) {
	//TODO: Prediction step: Update each sample by drawing the a pose from the motion model.
	double move;
	//std::cout<<"move:"<<move<<" ux:"<<ux<<" stdev:"<<stdev<<std::endl;

	for(int i=0; i<particles.size(); i++) {
		//std::cout<<"particles["<<"i"<<"].x:"<<particles[i].x<<std::endl;
		particles[i].x = sampleFromGaussian(ux+particles[i].x, stdev);
		//particles[i].x += move;
	}
}


/**
 * \brief Returns the distance between the given x position and the nearest light source.
 * \param[in] x The position on the x axis.
 * \return The distance to the nearest light source.
 */
double ParticleFilter::getDistanceToNearestLight(const double& x) {
	double dist = 0.0;
	double distance[3] = {2.0, 6.0, 8.0};
	double minimum;

	distance[0] = fabs(distance[0]-x);
	distance[1] = fabs(distance[1]-x);
	distance[2] = fabs(distance[2]-x);

	minimum = distance[0];

	if(distance[1] < minimum) {
		minimum = distance[1];
	}

	if(distance[2] < minimum) {
		minimum = distance[2];
	}

	dist = minimum;

//	std::cout<<"+++++++++++++"<<std::endl;
//	for(int i=0; i<=2; i++) {
//		std::cout<<"distance["<<i<<"]:"<<distance[i]<<std::endl;
//	}
//	std::cout<<"x:"<<x<<" dist:"<<dist<<std::endl;
//	std::cout<<"-------------"<<std::endl;

	//TODO Return the distance from the robot's position x to the nearest light source.
	return dist;
}

/**
 * \brief Updates the particle weights according to the measured distance to the nearest light source.
 * \param[in,out] particles The list of particles.
 * \param[in] measurement The measured distance between the robot and the nearest light source.
 * \param[in] stdev The standard deviation of the observation model.
 */
void ParticleFilter::integrateObservation(std::vector<Particle>& particles, const double measurement,
		const double& stdev) {
	//TODO: Correction step: weight the samples according to the observation model.
	for(int i=0; i < particles.size(); i++) {
		particles[i].weight = gaussianProbability(getDistanceToNearestLight(particles[i].x)-measurement, stdev);
	}

	// Normalize the weights after updating so that they sum up to 1 again:
	normalizeWeights(particles);
}

/**
 * \brief Resamples the particle set by throwing out unlikely particles and duplicating more likely ones.
 * \param[in] particles The old list of particles.
 * \return The new list of particles after resampling.
 */
std::vector<ParticleFilter::Particle> ParticleFilter::resample(const std::vector<Particle>& particles) {
	std::vector<Particle> newParticles;
	/*TODO: Use stochastic universal resampling (also called low variance resampling)
	 * to draw a new set of particles according to the old particles' weights */
	double JtoMinusOne = 1.0/double(particles.size());
	double r = double(rand())*(JtoMinusOne/double(RAND_MAX));
	if (r > JtoMinusOne)
		r = JtoMinusOne;

	double c = particles[0].weight;
	int i = 0;

	for(int j=0; j < particles.size(); j++) {
		double U = r + j*JtoMinusOne;

		while(U > c) {
			i++;
			if (i < particles.size())
				c += particles[i].weight;
		}

		if(i < particles.size())
			newParticles.push_back(particles[i]);
	}

	return newParticles;
}

}  // namespace particle_filter

