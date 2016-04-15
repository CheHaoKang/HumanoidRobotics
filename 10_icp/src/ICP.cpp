#include <icp/ICP.h>
#include <iostream>
#include <stdexcept>
#include <math.h>
#include <Eigen/SVD> 

using namespace std;
namespace icp
{
//	struct vecXcomp{
//		bool operator()(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs){
//			return lhs.x() < rhs.x();
//			//return lhs.x() < rhs.x() && lhs.y() < rhs.y() && lhs.z() < rhs.z();
//		}
//	} mycomp;

/**
 * \brief Compute the Euclidean distance between a couple of 2D points.
 * \param[in] p1: The first 2D point.
 * \param[in] p2: The second 2D point.
 * \return The Euclidean distance between the two input 2D points.
 */
	double ICP::distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
	{
		double result;
		//TODO
		result = sqrt((((p1.x()-p2.x())*(p1.x()-p2.x())))+((p1.y()-p2.y())*(p1.y()-p2.y())));

		return result;
	}
/**
 * \brief Compute the closest point that lies on a given line to a given 2D point.
 * \param[in] pX: The given 2D point, to which we need to compute the closest point that lies on a line.
 * \param[in] pL1: A point that lies on the line.
 * \param[in] pL2: A point that lies on the line.
 * \return The closest point on the line.
 */
	Eigen::Vector2d ICP::closestPointOnLine(const Eigen::Vector2d& pX, const Eigen::Vector2d& pL1, const Eigen::Vector2d& pL2)
	{
		Eigen::Vector2d result;
		//TODO
		double a, b, c, t;

		if(pL1.x()==pL2.x()) {
			result.x() = pL1.x();
			result.y() = pX.y();
		} else {
			// x=t, y=(c-at)/b;
			// Line: (q2-q1)/(p2-p1)x-y=(q2-q1)/(p2-p1)p1-q1
			// t=(ac-abq+b^2p)/(a^2+b^2)
			a = (pL2.y()-pL1.y())/(pL2.x()-pL1.x());
			b = -1;
			c = ((pL2.y()-pL1.y())/(pL2.x()-pL1.x()))*pL1.x()-pL1.y();
			t = (a*c-a*b*pX.y()+b*b*pX.x())/(a*a+b*b);

			result.x() = t;
			result.y() = (c-a*t)/b;
		}

		return result;
	}
/**
 * \brief Get the minimum value within vector.
 * \param[in] dist: A vector of values.
 * \return The minimum value.
 */
	double ICP::min(const std::vector<double>& dist)
	{
		double result;
		double smallest = dist.at(0);
		//TODO
		for(int i=1; i < dist.size(); i++) {
			if(dist.at(i) < smallest)
				smallest = dist.at(i);
		}

		result = smallest;

		return result;
	}
/**
 * \brief Compute the corresponding points in list P to those points in list Q, using the 'closest point' matching method .
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points.
 * \return A vector of the corresponding 2D points matched to points of list Q in list P.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::euclideanCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d result;
		double smallest;
		int smallestIndex;

		//TODO
		for(int i=0; i<Q.size(); i++) {
			smallest = distance(Q[i], P[0]);
			smallestIndex = 0;
			//cout<<"++++++++++++++++++"<<endl<<endl;
			//cout<<"START smallest:"<<smallest<<"  smallestIndex:"<<smallestIndex<<endl;

			for(int j=1; j<P.size(); j++) {
				//cout<<"Q("<<Q[i].x()<<", "<<Q[i].y()<<")"<<endl;
				if(distance(Q[i], P[j]) < smallest) {
					//cout<<"P("<<P[j].x()<<", "<<P[j].y()<<")"<<endl;
					smallest = distance(Q[i], P[j]);
					smallestIndex = j;

					//cout<<"smallest:"<<smallest<<"  smallestIndex:"<<smallestIndex<<endl;
				}
			}
			//cout<<"END smallest:"<<smallest<<"  smallestIndex:"<<smallestIndex<<endl;
			//cout<<"-------------------"<<endl<<endl;

			result.push_back(P[smallestIndex]);
		}

		return result;
	}
/**
 * \brief Compute the corresponding points in list P to those points in list Q, using the 'point-to-line' matching method .
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points.
 * \return A vector of the corresponding 2D points matched to points of list Q in list P.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::closestPointToLineCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d result;
		StdVectorOfVector2d euclideanC;
		double leftPointDistance, rightPointDistance;
		int i, j;

//		//std::sort(P.data(), P.data()+P.size(), mycomp);
//		std::sort(P.begin(), P.end());
//		for(int i=0; i<P.size(); i++) {
//			cout<<"P["<<i<<"]=("<<P[i].x()<<", "<<P[i].y()<<endl;
//		}

		euclideanC = euclideanCorrespondences(Q, P);

		for(i=0; i < euclideanC.size(); i++) {
			leftPointDistance = rightPointDistance = std::numeric_limits<double>::max() ;

			for(j=0; j<P.size(); j++) {
				if(euclideanC[i].x()==P[j].x() && euclideanC[i].y()==P[j].y())
					break;
			}

			//cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>"<<endl;
			if(j-1 >= 0) {
				//cout<<"i:"<<i<<"  "<<"euclideanC["<<i<<"]=("<<euclideanC[i].x()<<", "<<euclideanC[i].y()<<")"<<endl;
				//cout<<"i-1:"<<i-1<<"  "<<"euclideanC["<<i-1<<"]=("<<euclideanC[i-1].x()<<", "<<euclideanC[i-1].y()<<")"<<endl;
				leftPointDistance = distance(euclideanC[i], P[j-1]);
			}

			if(j+1 < P.size()) {
				//cout<<"i:"<<i<<"  "<<"euclideanC["<<i<<"]=("<<euclideanC[i].x()<<", "<<euclideanC[i].y()<<")"<<endl;
				//cout<<"i+1:"<<i+1<<"  "<<"euclideanC["<<i+1<<"]=("<<euclideanC[i+1].x()<<", "<<euclideanC[i+1].y()<<")"<<endl;
				rightPointDistance = distance(euclideanC[i], P[j+1]);
			}
			//cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;

			if(leftPointDistance > rightPointDistance) {
				result.push_back(closestPointOnLine(Q[i], euclideanC[i], P[j+1]));
			} else {
				result.push_back(closestPointOnLine(Q[i], P[j-1], euclideanC[i]));
			}
		}

		return result;
	}
/**
 * \brief Compute the affine transformation matrix needed to allign the previously computed corresponding points (list C) to the points of list Q.
 * \param[in] Q: A vector of 2D points.
 * \param[in] C: A vector of 2D points, that corresponds to points in list Q.
 * \return An Affine transformation matrix.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	Eigen::Matrix3d ICP::calculateAffineTransformation(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C)
	{
		Eigen::Matrix3d result;
		Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
		Eigen::Matrix2d R;
		Eigen::Vector2d t;
		Eigen::Vector2d centerMassQ, centerMassC;
		//Eigen::Vector2d qDelta, cDelta;
		centerMassQ << 0, 0;
		centerMassC << 0, 0;

		//cout<<"W "<<W(0, 0)<<", "<<W(1, 1)<<endl;
		cout<<"++++++++++++++++++++++"<<endl;

//		double centerMassQX=0, centerMassCX=0;
//		double centerMassQY=0, centerMassCY=0;
		//TODO		
		for(int i=0; i<Q.size(); i++) {
			centerMassQ += Q[i];
		}
		centerMassQ /= double(Q.size());
		cout<<"centerMassQ:"<<centerMassQ<<endl;
//		centerMassQ.x() /= double(Q.size());
//		centerMassQ.y() /= double(Q.size());

		for(int i=0; i<C.size(); i++) {
			centerMassC += C[i];
		}
		centerMassC /= double(C.size());
		cout<<"centerMassC:"<<centerMassC<<endl;
//		centerMassC.x() /= double(C.size());
//		centerMassC.y() /= double(C.size());

		for(int i=0; i<C.size(); i++) {
			W += (Q[i]-centerMassQ)*((C[i]-centerMassC).transpose());
		}
		cout<<"W:"<<W<<endl;

		//http://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html
		//Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		R = svd.matrixU()*(svd.matrixV().transpose());
		t = centerMassQ - R*centerMassC;

		result << 	R(0, 0), R(0, 1), t.x(),
					R(1, 0), R(1, 1), t.y(),
					0, 0, 1;

		cout<<"Affine transformation matrix: "<<result<<endl;
		cout<<"-------------------------"<<endl;

		return result;
	}
/**
 * \brief Apply the affine transformation matrix on the points in list P.
 * \param[in] A: Affine transformation matrix.
 * \param[in] P: A vector of 2D points, on which the affine transformation will be applied.
 * \return The vector of transformed points.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::applyTransformation(const Eigen::Matrix3d& A, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d  result;
		Eigen::Vector3d vector3D;
		Eigen::Vector2d vector2D;

		//TODO
		for(int i=0; i<P.size(); i++){
			vector3D << P[i], 1;
			vector3D = A*vector3D;
			vector2D << vector3D.x(), vector3D.y();
			result.push_back(vector2D);
		}

		return result;
	}
/**
 * \brief Compute the error between the points in Q list and the transformed corresponding points .
 * \param[in] Q: A vector of 2D points.
 * \param[in] C: A vector of 2D points corresponding to point in Q.
 * \param[in] A: Affine transformation matrix.
 * \return The error of applying the transformation.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	double ICP::computeError(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C, const Eigen::Matrix3d& A)
	{
		double result;
		Eigen::Matrix2d R;
		Eigen::Vector2d t, temp;
		//TODO
		R << 	A(0, 0), A(0, 1),
				A(1, 0), A(1, 1);
		t <<	A(0, 2), A(1, 2);

		result = 0.0;
		for(int i=0; i<C.size(); i++) {
			temp = (Q[i]-R*C[i]-t);
			result += temp.x()*temp.x()+temp.y()*temp.y();
		}

		return result;		
	}
/**
 * \brief Perform one iteration of ICP and prints the error of that iteration.
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points, that we need to transform them to be alligned with points in Q list.
 * \param[out] convergenceFlag: A flag passed by reference to determine whether the allignment error has crossed the convergence threshold or not. The flag should be set to '1' in case of convergence.
 * \param[in] pointToLineFlag: A flag that states the matching method to be used. Its value is set to 'true' when point-to-line method is needed, and to 'false' when closest point method is needed.
 * \param[in] threshold: The maximum value of acceptable error.
 * \return The vector of transformed points.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::iterateOnce(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P, int & convergenceFlag,bool pointToLineFlag, double threshold)
	{
		StdVectorOfVector2d result;
		StdVectorOfVector2d correspondences;
		//StdVectorOfVector2d PTransformedPoints;
		Eigen::Matrix3d affineTransformation;
		double error;
		
		//TODO
		if(pointToLineFlag == true) {
			correspondences = closestPointToLineCorrespondences(Q, P);
		} else {
			correspondences = euclideanCorrespondences(Q, P);
		}

		affineTransformation = calculateAffineTransformation(Q, correspondences);
		result = applyTransformation(affineTransformation, P);
		error = computeError(Q, correspondences, affineTransformation);

		if (error <= threshold) {
			convergenceFlag = 1;
		}

		return result;
	}
}
