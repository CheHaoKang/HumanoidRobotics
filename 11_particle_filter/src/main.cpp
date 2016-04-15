#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <particle_filter/ParticleFilter.h>
#include <particle_filter/FileIO.h>

using namespace particle_filter;

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "particle_filter");
	srand(time(0));  //initialize the random number generator

	const std::string packagePath = ros::package::getPath("particle_filter");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package particle_filter. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	FileIO fileIO(packagePath + "/data/data", packagePath + "/data/result");
	if (!fileIO.isOK()) {
		return 2;
	}

	const double odom_stdev = 0.5;
	const double measurement_stdev = 0.1;

	std::vector<ParticleFilter::Particle> particles(250);
	ParticleFilter::initParticles(particles);
	fileIO.writeMap(particles);

	for (int i = 0; i < fileIO.odom.size(); ++i) {
		ParticleFilter::integrateMotion(particles, fileIO.odom[i], odom_stdev);
		ParticleFilter::integrateObservation(particles, fileIO.measurement[i], measurement_stdev);
		particles = ParticleFilter::resample(particles);
		fileIO.writeMap(particles);
	}	

	std::cout << "Wrote particle positions to 11_particle_filter/data/result." << std::endl;

	return 0;
}
