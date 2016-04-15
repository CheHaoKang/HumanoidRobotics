#include <signed_distance_function/SignedDistanceFunction.h>
#include <iostream>
#include <stdexcept>
#include <sstream>

namespace signed_distance_function {

/**
 * \brief Calculates the Euclidean distance between two 2D points.
 * \param[in] pointA The first point.
 * \param[in] pointB The second point.
 * \return The Euclidean distance
 */
double SignedDistanceFunction::calculateDistance(const Eigen::Vector2d& pointA, const Eigen::Vector2d& pointB) {
	double distance = 0.0;
	//TODO: Implement the Euclidean distance.
	distance = sqrt((((pointA.x()-pointB.x())*(pointA.x()-pointB.x())))+((pointA.y()-pointB.y())*(pointA.y()-pointB.y())));

	return distance;
}

/**
 * \brief Truncates the signed distance.
 * \param[in] signedDistance The signed distance in cells (negative outside the object, positive inside the object).
 * \param[in] delta The truncating limit in cells.
 * \return The truncated signed distance.
 */
double SignedDistanceFunction::truncateDistance(const double& signedDistance, const double& delta) {
	double truncatedDistance = 0.0;
	//TODO: Implement the truncated signedDistance function.
	// Note that signedDistance can be negative here.
	truncatedDistance = signedDistance;

	if(signedDistance < 0) {
		if (-signedDistance > delta)
			truncatedDistance = -delta;
	} else {
		if (signedDistance > delta)
			truncatedDistance = delta;
	}

	return truncatedDistance;
}

/**
 * \brief Calculates the weight according to the signed distance.
 * \param[in] signedDistance The signed distance in cells (negative outside the object, positive inside the object).
 * \param[in] delta The truncation limit.
 * \param[in] epsilon The lower limit.
 * \re
 */
double SignedDistanceFunction::calculateWeight(const double& signedDistance, const double& delta, const double& epsilon) {
	double weight = 0.0;
	//TODO calculate the weight according to the current measurement
	if (signedDistance <= epsilon)
		weight = 1;
	else if (signedDistance >= delta)
		weight = 0;
	else
		weight = (signedDistance-delta)/(epsilon-delta);
		//weight = (signedDistance-epsilon)/(delta-epsilon);

	return weight;
}

/**
 * \brief Update the signed distance value in the map
 * \param[in] signedDistance The new signed distance of the current measurement.
 * \param[in] weight The new weight of the current measurement.
 * \param[in] oldSignedDistance The old signed distance contained in the map.
 * \param[in] oldWeight The old weight of the map cell.
 * \return The new signed distance entry of the map cell.
 */
double SignedDistanceFunction::updateMap(
		const double& signedDistance, const double& weight,
		const double& oldSignedDistance, const double& oldWeight) {
	double newSignedDistance = 0.0;
	//TODO calculate the new signed distance that will be written into the map
	if((oldWeight+weight) > 0)
		newSignedDistance = (oldWeight*oldSignedDistance + weight*signedDistance) / (oldWeight + weight);
	else
		newSignedDistance = oldSignedDistance;

	return newSignedDistance;
}

/**
 * \brief Update the weight of the map cell
 * \param[in] weight The new weight of the current measurement.
 * \param[in] oldWeight The old weight of the map cell.
 * \return The new weight of the map cell.
 */
double SignedDistanceFunction::updateWeight(const double& weight, const double& oldWeight) {
	double newWeight = 0.0;
	//TODO calculate the new weight of the map cell
	newWeight = oldWeight + weight;

	return newWeight;
}

/**
 * \brief Adds the information from the new laser scan to the map.
 * \param[in,out] map The map containing the signed distance value for each cell
 * \param[in,out] weights The weight of each map cell.
 * \param[in] measurement The current laser measurement.
 */
void SignedDistanceFunction::integrateLaserScan(Eigen::MatrixXd& map, Eigen::MatrixXd& weights, const Measurement& measurement) {
	const double delta = 5;
	const double epsilon = 1;
	double signedDistance, weight;
	double distanceRobotLaser, distanceCellRobot, distanceCellLaser;

	//TODO: Add the information from the new laser scan to the map.
	for(int j=0; j<measurement.laserPoints.size(); j++) {
		distanceRobotLaser = calculateDistance(measurement.robotPose, measurement.laserPoints.at(j));
		VectorOfPoints linePoints = SignedDistanceFunction::bresenham(measurement.robotPose, measurement.laserPoints.at(j), map.rows(), map.cols());

		for(int i=0; i < linePoints.size(); i++) {
			distanceCellRobot = calculateDistance(linePoints[i], measurement.robotPose);
			distanceCellLaser = calculateDistance(linePoints[i], measurement.laserPoints.at(j));

			if(distanceCellRobot < distanceRobotLaser)
				signedDistance = truncateDistance(-distanceCellLaser, delta);
			else
				signedDistance = truncateDistance(distanceCellLaser, delta);

			weight = calculateWeight(signedDistance, delta, epsilon);

			map(int(linePoints[i].y()), int(linePoints[i].x())) = updateMap(signedDistance, weight, double(map(int(linePoints[i].y()), int(linePoints[i].x()))), double(weights(int(linePoints[i].y()), int(linePoints[i].x()))));

			weights(int(linePoints[i].y()), int(linePoints[i].x())) = updateWeight(weight, double(weights(int(linePoints[i].y()), int(linePoints[i].x()))));
		}
	}

//	for (std::vector<Eigen::Vector2d>::iterator it = linePoints.begin(); it != linePoints.end(); ++it) {
	//}


	//double distanceRobotLaser, distanceCellLaser;
	//double signedDistance, weight;
	//Eigen::Vector2d vectorTemp;


	/*distanceRobotLaser = calculateDistance(measurement.robotPose, measurement.laserPoints.at(0));

	for(int i=0; i<map.rows(); i++) {
		for(int j=0; j<map.cols(); j++) {
			vectorTemp << i, j;

			distanceCellLaser = calculateDistance(vectorTemp, measurement.laserPoints.at(0));
			if(distanceRobotLaser > distanceCellLaser)
				signedDistance = truncateDistance(-distanceCellLaser, delta);
			else
				signedDistance = truncateDistance(distanceCellLaser, delta);

			weight = calculateWeight(signedDistance, delta, epsilon);

			map(i, j) = updateMap(signedDistance, weight, double(map(i, j)), double(weights(i, j)));

			weights(i, j) = updateWeight(weight, double(weights(i, j)));
		}
	}*/
}

/**
 * \brief Modified version of the Bresenham algorithm for calculating points on a straight line.
 * \param[in] pointA The first end point of the line
 * \param[in] pointB The second end point of the line
 * \return Vector of all points between A and B+(B-A)
 *
 * This method is based on the code on http://de.wikipedia.org/wiki/Bresenham-Algorithmus
 */
VectorOfPoints SignedDistanceFunction::bresenham(const Eigen::Vector2d& pointA, const Eigen::Vector2d& pointB,
		const size_t& numRows, const size_t& numCols) {

	if (pointA.x() < 0 || pointA.x() > numCols || pointA.y() < 0 || pointA.y() > numRows) {
		std::stringstream ss;
		ss << "Error in bresenham(): pointA with coordinates (" << pointA.x() << ", " << pointA.y() << ") is outside the map";
		throw std::invalid_argument(ss.str());
	}
	if (pointB.x() < 0 || pointB.x() > numCols || pointB.y() < 0 || pointB.y() > numRows) {
		std::stringstream ss;
		ss << "Error in bresenham(): pointB with coordinates (" << pointB.x() << ", " << pointB.y() << ") is outside the map";
		throw std::invalid_argument(ss.str());
	}

	VectorOfPoints pointsOnLine;

	const Eigen::Vector2d pointC = pointB + (pointB - pointA);

	int x0 = pointA.x();
	int y0 = pointA.y();
	int x1 = pointC.x();
	int y1 = pointC.y();
	int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
	int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
	int err = dx+dy, e2; /* error value e_xy */

	Eigen::Vector2d p;
	while(true) {
		p << x0, y0;
		if (x0 >= 0 && x0 < numCols && y0 >= 0 && y0 < numRows) {
			pointsOnLine.push_back(p);
		}
		if (x0==x1 && y0==y1) break;
		e2 = 2*err;
		if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
		if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
	}

	return pointsOnLine;
}



}  // namespace signed_distance_function
