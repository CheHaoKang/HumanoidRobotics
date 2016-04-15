#include <particle_filter/ParticleFilter.h>
#include <particle_filter/FileIO.h>


namespace particle_filter {

FileIO::FileIO(const std::string & inputfilename,const std::string & outputfilename)
    : ok(false), ofs(outputfilename.c_str())
{
	if (!ofs.good()) {
		std::cerr << "Could not open file " << outputfilename << " for writing" << std::endl;
		return;
	}

	std::ifstream ifs(inputfilename.c_str());
	if (!ifs.good()) {
		std::cerr << "Could not open file " << inputfilename << " for reading." << std::endl;
		return;
	}

	double o, m;
	ifs >> o >> m;
	while (ifs.good()) {
		odom.push_back(o);
		measurement.push_back(m);
		ifs >> o >> m;
	}
	ifs.close();
	ok = true;
}

FileIO::~FileIO() {
	if (ofs.is_open()) {
		ofs.close();
	}
}

void FileIO::writeMap(const std::vector<ParticleFilter::Particle>& particles) {
	if (!ofs.good()) {
		return;
	}

	for (size_t i = 0; i < particles.size(); ++i) {
		ofs << particles[i].x << std::endl;
	}
	ofs << std::endl << std::endl;
}

}  // namespace particle_filter
