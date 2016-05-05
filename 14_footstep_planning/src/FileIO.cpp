#include <footstep_planning/FileIO.h>
#include <fstream>
#include <iostream>
#include <cstdio>

namespace footstep_planning {

FileIO::FileIO(const std::string& package_path) {
	const size_t angleStep = 15;
	std::string filename = package_path + "/data/map.pbm";
	std::ifstream ifs(filename.c_str());
	if (!ifs.good()) {
		std::cerr << "Could not open map file " << filename << std::endl;
		return;
	}
	std::string header;
	ifs >> header;
	if (header != "P1") {
		std::cerr << "Error: map file is not in PBM format" << std::endl;
		return;
	}

	int width = 0, height = 0;
	ifs >> width >> height;
	if (width > 1000 || height > 1000) {
		throw std::runtime_error("Map is too big");
	}

	std::vector<bool> data;
	data.reserve(width * height);
	char c;
	while (ifs.get(c)) {
		switch (c) {
		case '0': data.push_back(false); break;
		case '1': data.push_back(true); break;
		default: //ignore
			break;
		}
	}
	ifs.close();

	std::vector<std::vector<unsigned char> > distanceMaps(180 / angleStep);

	char buffer[package_path.length() + 25];
	for (size_t angle = 0; angle < 180; angle += angleStep) {
		size_t i = angle / angleStep;
		snprintf(buffer, sizeof(buffer), "%s/data/distance_%zu.pgm", package_path.c_str(), angle);
		ifs.open(buffer);
		if (!ifs.good()) {
			std::cerr << "Could not load " << buffer << " for reading distance map." << std::endl;
			return;
		}
		ifs >> header;
		if (header != "P5") {
			std::cerr << "Error: distance map file is not in PGM format" << std::endl;
			return;
		}

		int nwidth, nheight, depth;
		ifs >> nwidth >> nheight >> depth;
		if (width != nwidth || height != nheight) {
			std::cerr << "Error: distance map does not have same dimensions as map" << std::endl;
			return;
		}
		if (depth != 255) {
			std::cerr << "Error: distance map has invalid depth " << depth << std::endl;
			return;
		}

		distanceMaps[i].reserve(width * height);
		ifs.get(c); // line break
		while (ifs.get(c)) {
			distanceMaps[i].push_back(c);
		}
		ifs.close();
		if (distanceMaps[i].size() != width * height) {
			std::cerr << "Error: distance map has " << distanceMaps[i].size() << " elements, but expected " << width << " x " << height << " = " << width * height << " elements." << std::endl;
			return;
		}
	}

	map = new FootstepMap(width, height, data, distanceMaps);
}

FileIO::~FileIO() {
}

} /* namespace footstep_planning */
