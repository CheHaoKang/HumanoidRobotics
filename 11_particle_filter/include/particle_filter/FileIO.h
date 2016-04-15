#ifndef PARTICLE_FILTER_FILEIO_H_
#define PARTICLE_FILTER_FILEIO_H_

#include <vector>
#include <particle_filter/ParticleFilter.h>
#include <string>
#include <fstream>
#include <iostream>

namespace particle_filter {

class FileIO {
private:
	std::ofstream ofs;
	bool ok;

public:
	FileIO(const std::string & inputfilename,const std::string & outputfilename);
	~FileIO();
	std::vector<double> odom;
	std::vector<double> measurement;

	bool isOK() const { return ok; };

	void writeMap(const std::vector<ParticleFilter::Particle>& particles);
};

}  // namespace particle_filter

#endif  //PARTICLE_FILTER_FILEIO_H_
