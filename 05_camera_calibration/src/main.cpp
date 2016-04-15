#include <camera_calibration/CameraCalibration.h>
#include <camera_calibration/FileIO.h>
#include <ros/package.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace camera_calibration;

int main(int argc, char *argv[]) {
	FileIO fileIO;
	std::string packagePath = ros::package::getPath("camera_calibration");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package camera_calibration. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	try {
		fileIO.loadImage(packagePath + "/data/image1.jpg");
		fileIO.loadImage(packagePath + "/data/image2.jpg");
		fileIO.loadImage(packagePath + "/data/image3.jpg");
		fileIO.loadImage(packagePath + "/data/image4.jpg");
		fileIO.loadImage(packagePath + "/data/image5.jpg");
		fileIO.loadImage(packagePath + "/data/image6.jpg");
		fileIO.loadImage(packagePath + "/data/image7.jpg");
	} catch(const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		return 2;
	}

	HomographyVectorType homographies;
	for (size_t i = 0; i < fileIO.images.size(); ++i) {
		std::cout << "Computing homography for image " << i << " ..." << std::endl;
		fileIO.images[i].homography = CameraCalibration::homographyFromPoints(
				fileIO.images[i].worldCoordinates,
				fileIO.images[i].imageCoordinates);
		homographies.push_back(fileIO.images[i].homography);
	}

	std::cout << "Calculating the calibration matrix ..." << std::endl;
	const Eigen::Matrix3d B = CameraCalibration::calculateB(homographies);
	const Eigen::Matrix3d K = CameraCalibration::calibrationMatrix(B);

	std::cout << std::endl;
	std::cout << "The calibration matrix is: K = " << std::endl << K << std::endl;
	std::cout << std::endl;
	std::cout << "Hence the intrinsic camera parameters are: " << std::endl;
	std::cout << "     focal length fx = " << K(0, 0); std::cout << std::endl;
	std::cout << "     focal length fy = " << K(1, 1); std::cout << std::endl;
	std::cout << "             sheer s = " << K(0, 1); std::cout << std::endl;
	std::cout << "  principal point xH = " << K(0, 2); std::cout << std::endl;
	std::cout << "  principal point yH = " << K(1, 2); std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "Camera positions:" << std::endl;
	for (size_t i = 0; i < fileIO.images.size(); ++i) {
		fileIO.images[i].extrinsic = CameraCalibration::cameraPose(homographies[i], K);
		std::cout << "    " << fileIO.images[i].filename << ": " << fileIO.images[i].extrinsic.col(3).transpose();
		std::cout << std::endl;
	}


	char *outdir = NULL;
	for (size_t i = 1; i < argc - 1; ++i) {
		if (strcmp(argv[i], "--outdir") == 0) {
			outdir = argv[i + 1];
		}
	}
	fileIO.showImages(K, outdir);
	if (!outdir) {
		cv::waitKey(0);
	}
	return 0;
}
