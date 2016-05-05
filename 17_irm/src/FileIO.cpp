#include <irm/FileIO.h>
#include <fstream>

namespace irm {

FileIO::FileIO(const std::string& packagePath) : packagePath(packagePath) {

}

FileIO::~FileIO() {
}

template<typename MapType>
void FileIO::writeHelper(const std::vector<IRM::MapConfig>& mapConfig, const IRM& irm, const MapType& rm, std::ofstream& ofs) {
	Eigen::Vector3d eef = Eigen::Vector3d::Zero();
	for (eef[1] = mapConfig[1].min + mapConfig[1].res / 2; eef[1] <= mapConfig[1].max; eef[1] += mapConfig[1].res) {
		for (eef[0] = mapConfig[0].min + mapConfig[1].res / 2; eef[0] <= mapConfig[0].max; eef[0] += mapConfig[0].res) {
			const size_t idx = irm.getIndex(mapConfig, eef, true);
			if (idx != IRM::INVALID) {
				const typename MapType::const_iterator it = rm.find(idx);
				if (it != rm.end()) {
					ofs << it->second->manipulability << " ";
				} else {
					ofs << "-1 ";
				}
			} else {
				ofs << "-1 ";
			}
		}
		ofs << std::endl;
	}
}

void FileIO::writeRM(const IRM& irm) {
	const IRM::RMMapType& rm = irm.get2DReachabilityMap();
	const std::vector<IRM::MapConfig>& mapConfig = irm.getRMMapConfig();
	const std::string filename = packagePath + "/data/rm.txt";
	std::ofstream ofs(filename.c_str());
	if (!ofs.good()) {
		std::cerr << "Error: Could not open " << filename << " for writing the reachability map." << std::endl;
		return;
	}
	writeHelper(mapConfig, irm, rm, ofs);
	ofs.close();
	std::cout << "Wrote 2D projection of the reachability map with " << rm.size() << " samples to " << filename << std::endl;
}

void FileIO::writeIRM(const IRM& irm) {
	const IRM::IRMMapType& rm = irm.get2DInverseReachabilityMap();
	const std::vector<IRM::MapConfig>& mapConfig = irm.getIRMMapConfig();
	const std::string filename = packagePath + "/data/irm.txt";
	std::ofstream ofs(filename.c_str());
	if (!ofs.good()) {
		std::cerr << "Error: Could not open " << filename << " for writing the inverse reachability map." << std::endl;
		return;
	}
	writeHelper(mapConfig, irm, rm, ofs);
	ofs.close();
	std::cout << "Wrote 2D projection of the inverse reachability map with " << rm.size() << " samples to " << filename << std::endl;
}

} /* namespace irm */
