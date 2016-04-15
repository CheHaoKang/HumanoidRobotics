#include <camera_calibration/FileIO.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace camera_calibration {

FileIO::FileIO()
: patternsize(9, 6), squaresize(0.0226) {

}

FileIO::~FileIO() {
	for (ImageVectorType::const_iterator it = images.begin();
			it != images.end(); ++it) {
		delete it->image;
	}
	images.clear();
}

void FileIO::loadImage(const std::string& filename) {
	Image image;
	size_t found = filename.rfind("/");
	image.filename = found == std::string::npos ? filename : filename.substr(found + 1);
	std::cout << "Loading image " << image.filename << " and searching corners ... " << std::endl;

	image.image = new cv::Mat();
	*image.image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (!image.image->data) {
		throw std::runtime_error("Could not open image file " + filename);
	}

	cv::Mat grayImage;
	cv::cvtColor(*image.image, grayImage, cv::COLOR_BGR2GRAY);

	cv::vector<cv::Point2f> corners;
	bool patternfound = cv::findChessboardCorners(grayImage, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
	if (patternfound) {
		cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	}

	if (corners.size() != patternsize.height * patternsize.width) {
		std::cerr << "Could not find the expected number of corners in image " << filename << std::endl;
		return;
	}

	size_t indices[] = {
		0,
		patternsize.width - 1,
		(patternsize.height - 1) * patternsize.width,
		patternsize.height * patternsize.width - 1
	};
	for (size_t i = 0; i < 4; ++i) {
		image.imageCoordinates[i] << corners[indices[i]].x, corners[indices[i]].y;
	}
	image.worldCoordinates[0] << patternsize.width * squaresize, squaresize, 0;
	image.worldCoordinates[1] << squaresize, squaresize, 0;
	image.worldCoordinates[2] << patternsize.width * squaresize, patternsize.height * squaresize, 0;
	image.worldCoordinates[3] << squaresize, patternsize.height * squaresize, 0;

	images.push_back(image);
}


void drawCoordinateSystem(cv::Mat& debug, const Eigen::Matrix3d& homography, const Eigen::Matrix3d& intrinsic, const Eigen::MatrixXd& extrinsic) {
	const double scale = 0.1;
	Eigen::Vector3d points[3], transformed[3];
	cv::Point2i imagePoints[3];
	points[0] <<     0,     0, 1;
	points[1] << scale,     0, 1;
	points[2] <<     0, scale, 1;

	for (size_t i = 0; i < 3; ++i ) {
		transformed[i] = homography * points[i];
		imagePoints[i].x = transformed[i][0] / transformed[i][2];
		imagePoints[i].y = transformed[i][1] / transformed[i][2];
	}

	cv::line(debug, imagePoints[0], imagePoints[1], CV_RGB(255, 0, 0), 2);
	cv::line(debug, imagePoints[0], imagePoints[2], CV_RGB(0, 255, 0), 2);

	Eigen::Vector4d zWorld;
	zWorld << 0, 0, scale, 1;
	Eigen::Vector3d zImageH = intrinsic * extrinsic * zWorld;

	cv::Point2i zImageE(zImageH(0) / zImageH(2), zImageH(1) / zImageH(2));

	cv::line(debug, imagePoints[0], zImageE, CV_RGB(0, 0, 255), 2);
}

void FileIO::showImages(const Eigen::Matrix3d& intrinsic, const char *outdir) const {
	for (ImageVectorType::const_iterator it = images.begin(); it != images.end(); ++it) {
		cv::Mat debug(*it->image);
		//cv::drawChessboardCorners(debug, patternsize, cv::Mat(corners), patternfound);
		for (size_t i = 0; i < 4; ++i) {
			cv::circle(debug, cv::Point2i(it->imageCoordinates[i].x(), it->imageCoordinates[i].y()), 5 + 3 * i, CV_RGB(200, 0, 200), 2);
		}
		drawCoordinateSystem(debug, it->homography, intrinsic, it->extrinsic);

		if (outdir) {
			cv::imwrite(std::string(outdir) + "/calibration_" + it->filename, debug);
		} else {
			cv::namedWindow(it->filename, cv::WINDOW_NORMAL);
			cv::imshow(it->filename, debug);
		}
	}

}

}  // namespace camera_calibration
