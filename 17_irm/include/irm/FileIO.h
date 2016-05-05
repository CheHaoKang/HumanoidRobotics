#ifndef IRM_FILEIO_H_
#define IRM_FILEIO_H_

#include <string>
#include <irm/IRM.h>

namespace irm {

class FileIO {
public:
	FileIO(const std::string& packagePath);
	virtual ~FileIO();

	void writeRM(const IRM& irm);
	void writeIRM(const IRM& irm);

protected:
	const std::string& packagePath;

private:
	template<typename MapType>
	void writeHelper(const std::vector<IRM::MapConfig>& mapConfig, const IRM& irm, const MapType& rm, std::ofstream& ofs);
};

} /* namespace irm */

#endif /* IRM_FILEIO_H_ */
