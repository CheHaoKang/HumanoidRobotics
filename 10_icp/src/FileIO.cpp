#include <icp/FileIO.h>
#include <fstream>
#include <iostream>

namespace icp {

FileIO::FileIO(const std::string& filename)
{
	std::ifstream ifs(filename.c_str());
	if (!ifs.good()) {
		std::cerr << "Could not open file " << filename << " for reading." << std::endl;
		return;
	}

	double qx,qy,px,py;
	ifs >> qx>>qy>> px >> py;
	while (ifs.good())
	{
		Eigen::Vector2d q(qx,qy); 
		Eigen::Vector2d p(px,py); 
		Q.push_back(q);
		P.push_back(p);
		ifs >> qx>>qy>> px >> py;
	}
	ifs.close();

}
		
		



void FileIO::writeMap(const StdVectorOfVector2d & P, const std::string& filename) 
{
	std::ofstream ofs(filename.c_str());
	if (!ofs.good()) {
		std::cerr << "Could not open file " << filename << " for writing" << std::endl;
		return;
	}


	for (size_t i = 0; i < P.size(); ++i) 
	{
		ofs << P[i](0) << " "<<P[i](1) <<"\n";
	}
	ofs.close();
}

}  // namespace icp
