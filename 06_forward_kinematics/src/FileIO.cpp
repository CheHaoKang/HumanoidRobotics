#include <forward_kinematics/FileIO.h>
#include <fstream>
#include <iostream>

namespace forward_kinematics {

FileIO::FileIO(const std::string& filename) {
	std::ifstream ifs(filename.c_str());
	if (!ifs.good()) {
		std::cerr << "Could not open file " << filename << std::endl;
		return;
	}
	while (ifs.good()) {
		std::vector<double> encoderReadings(5), groundTruth(3);
		ifs >> encoderReadings[0]
			>> encoderReadings[1]
	        >> encoderReadings[2]
			>> encoderReadings[3]
			>> encoderReadings[4]
            >> groundTruth[0]
		    >> groundTruth[1]
			>> groundTruth[2];
		if (groundTruth[0] != 0.0 || groundTruth[1] != 0.0 || groundTruth[2] != 0.0) {
			this->jointAngles.push_back(encoderReadings);
			this->groundTruth.push_back(groundTruth);
		}
	}

}

bool FileIO::writeToFile(const std::string& filename) {
	std::ofstream ofs(filename.c_str());
	if (!ofs.good()) {
		std::cerr << "Could not open file " << filename << " for writing results" << std::endl;
		return false;
	}
	for (VectorOfTransforms::const_iterator it = results.begin(); it != results.end(); ++it) {
		ofs << it->topRightCorner(3, 1).transpose();
		ofs << std::endl;
	}
	ofs.close();
	return true;
}

}
