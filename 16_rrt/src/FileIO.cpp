/*
 * FileIO.cpp
 *
 *  Created on: 10.06.2015
 *      Author: sosswald
 */

#include <rrt/FileIO.h>

namespace rrt {

FileIO::FileIO() {
}

FileIO::~FileIO() {
	if (logfile.is_open()) {
		logfile.close();
	}
}


bool FileIO::openLogfile(const std::string& filename) {
	logfile.open(filename.c_str());
	if (!logfile.is_open()) {
		std::cerr << "Error: Could not open " << filename << " for writing the log file." << std::endl;
	}
	return logfile.is_open();
}

void FileIO::logPath(const std::string& filename, const std::deque<AbstractNode*>& path) {
	std::ofstream ofs(filename.c_str());
	if (ofs.good()) {
		for (std::deque<AbstractNode *>::const_iterator it = path.begin(); it != path.end(); ++it) {
			ofs << (*it)->toLogString() << std::endl;
		}
	} else {
		std::cerr << "Error: Could not open " << filename << " for writing the path." << std::endl;
	}
}

void FileIO::logExtend(const GridNode * const from, const GridNode * const to, const GridNode * const randomNode, const std::string * const listName) {
	if (logfile.good()) {
		logfile << ((GridNode *) from)->toLogString() << " " << ((GridNode *) to)->toLogString() << " " << ((GridNode *) randomNode)->toLogString() << " " << (*listName) << std::endl;
	}
}
void FileIO::logFailedExtend(const GridNode * const from, const GridNode * const randomNode, const std::string * const listName) {
	if (logfile.good()) {
		logfile << ((GridNode *) from)->toLogString() << " " << 1./0. << " " << 1./0. << " " << ((GridNode *) randomNode)->toLogString() << " " << (*listName) << std::endl;
	}
}

GridMap* FileIO::loadMap(const std::string& filename) {
	std::ifstream ifs(filename.c_str());
	if (!ifs.good()) {
		std::cerr << "Could not open map file " << filename << std::endl;
		return NULL;
	}
	std::string header;
	ifs >> header;
	if (header != "P1") {
		std::cerr << "Error: map file is not in PBM format" << std::endl;
		return NULL;
	}

	size_t width = 0, height = 0;
	ifs >> width >> height;
	if (width > 100 || height > 100) {
		throw std::runtime_error("Map is too big");
	}

	std::vector<bool> data;
	data.reserve(width * height);
	char c;
	while (ifs.get(c)) {
		switch (c) {
		case '0':
			data.push_back(false);
			break;
		case '1':
			data.push_back(true);
			break;
		default: //ignore
			break;
		}
	}

	ifs.close();

	return new GridMap(width, height, data);
}

} /* namespace rrt */

