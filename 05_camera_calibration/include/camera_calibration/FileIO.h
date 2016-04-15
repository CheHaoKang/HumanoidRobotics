#ifndef CAMERA_CALIBRATION_FILE_IO_H_
#define CAMERA_CALIBRATION_FILE_IO_H_

#include <opencv/cv.h>
#include <Eigen/Core>
#include <camera_calibration/CameraCalibration.h>
#include<Eigen/StdVector>

namespace camera_calibration {

class FileIO {
public:
	FileIO();
	~FileIO();

	struct Image {
		cv::Mat *image;
		std::string filename;
		Eigen::Matrix3d homography;
		Eigen::MatrixXd extrinsic;
		Eigen::Vector3d worldCoordinates[4];
		Eigen::Vector2d imageCoordinates[4];
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	typedef std::vector<Image, Eigen::aligned_allocator<Image> > ImageVectorType;
	ImageVectorType images;

	void loadImage(const std::string& filename);
	void showImages(const Eigen::Matrix3d& intrinsic, const char *outdir) const;

private:
	const cv::Size patternsize;   ///< Number of checkerboard corners (= number of squares minus 1) for each dimension
	const double squaresize;      ///< Size of the checkerboard squares in meters
};

}  // namespace camera_calibration


#endif // CAMERA_CALIBRATION_FILE_IO_H_
