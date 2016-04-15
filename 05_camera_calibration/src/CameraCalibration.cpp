#include <camera_calibration/CameraCalibration.h>
#include <iostream>
#include <stdexcept>
#include <math.h>

namespace camera_calibration {

/**
 * \brief Compute a 2D homography H from 4 point correspondances.
 * \param[in] worldCoordinates The world coordinates of 4 checkerboard corners in Euclidean coordinates (X, Y, Z).
 * \param[in] imageCoordinates The image coordinates of the same corners in Euclidean pixel coordinates (sx, sy).
 * \return The 2D homography that transforms points from the checkerboard plane to the image plane.
 */
Eigen::Matrix3d CameraCalibration::homographyFromPoints(const Eigen::Vector3d worldCoordinates[4], const Eigen::Vector2d imageCoordinates[4]) {
	Eigen::Matrix3d H;

	/*TODO: Calculate the homography by building Q and r (see exercise sheet)
	 * and solving the linear system Qh = r for h. Then fill the coefficients
	 * of h into the homography matrix H.
	 */
	Eigen::MatrixXd Q(8, 8);
	Eigen::VectorXd r(8);

	for(int i=0; i<4; i++) {
		Q(i*2, 0) = worldCoordinates[i].x();
		Q(i*2, 1) = worldCoordinates[i].y();
		Q(i*2, 2) = 1;
		Q(i*2, 3) = 0;
		Q(i*2, 4) = 0;
		Q(i*2, 5) = 0;
		Q(i*2, 6) = -imageCoordinates[i].x()*worldCoordinates[i].x();
		Q(i*2, 7) = -imageCoordinates[i].x()*worldCoordinates[i].y();

		Q(i*2+1, 0) = 0;
		Q(i*2+1, 1) = 0;
		Q(i*2+1, 2) = 0;
		Q(i*2+1, 3) = worldCoordinates[i].x();
		Q(i*2+1, 4) = worldCoordinates[i].y();
		Q(i*2+1, 5) = 1;
		Q(i*2+1, 6) = -imageCoordinates[i].y()*worldCoordinates[i].x();
		Q(i*2+1, 7) = -imageCoordinates[i].y()*worldCoordinates[i].y();

		r(i*2) = imageCoordinates[i].x();
		r(i*2+1) = imageCoordinates[i].y();
	}

	Eigen::VectorXd h(8);
	h = Q.fullPivLu().solve(r);

	H << 	h(0), h(1), h(2),
			h(3), h(4), h(5),
			h(6), h(7), 1;

	return H;
}

/**
 * \brief Auxiliary function defining the vector v as a function of the column indices i and j.
 * \param[in] i First column index of H
 * \param[in] j Second column index of H
 * \return Vector v.
 */
Eigen::VectorXd CameraCalibration::getV(const Eigen::Matrix3d& h, const size_t& i, const size_t& j) {
	Eigen::VectorXd v(6);
	// TODO: Fill the elements of the vector v.
	// vij=
	// (h1ih1j, h1ih2j+h2ih1j, h3ih1j+h1ih3j,
	// h2ih2j, h3ih2j+h2ih3j, h3ih3j)T
	v(0) = h(0, i)*h(0, j);
	v(1) = h(0, i)*h(1, j)+h(1, i)*h(0, j);
	v(2) = h(2, i)*h(0, j)+h(0, i)*h(2, j);
	v(3) = h(1, i)*h(1, j);
	v(4) = h(2, i)*h(1, j)+h(1, i)*h(2, j);
	v(5) = h(2, i)*h(2, j);

	return v;
}

/**
 * \brief Auxiliary function for computing B from multiple homographies.
 * \param[in] homography A list of 2D homographies calculated using the homographyFromPoints method.
 * \return Matrix B.
 */
Eigen::Matrix3d CameraCalibration::calculateB(const HomographyVectorType& homography) {
	Eigen::Matrix3d B;
	Eigen::MatrixXd V = Eigen::MatrixXd::Zero(2 * homography.size(), 6);
	int i;

	/* TODO: Fill the matrix V according to slide 57. Every homography will contribute
	 * two rows to V.
	 */
	for(i=0; i<homography.size(); i++) {
		Eigen::VectorXd v12(6);
		Eigen::VectorXd v11(6);
		Eigen::VectorXd v22(6);

		v12 = getV(homography[i], 0, 1);
		v11 = getV(homography[i], 0, 0);
		v22 = getV(homography[i], 1, 1);

		V(2*i, 0) = v12(0);
		V(2*i, 1) = v12(1);
		V(2*i, 2) = v12(2);
		V(2*i, 3) = v12(3);
		V(2*i, 4) = v12(4);
		V(2*i, 5) = v12(5);

		V(2*i+1, 0) = v11(0)-v22(0);
		V(2*i+1, 1) = v11(1)-v22(1);
		V(2*i+1, 2) = v11(2)-v22(2);
		V(2*i+1, 3) = v11(3)-v22(3);
		V(2*i+1, 4) = v11(4)-v22(4);
		V(2*i+1, 5) = v11(5)-v22(5);
		//V(2*i, 1) = getV(homography[i], 1, 1)-getV(homography[i], 2, 2);
	}

	const Eigen::VectorXd b = solveV(V);
	B << b(0), b(1), b(2),
	     b(1), b(3), b(4),
		 b(2), b(4), b(5);

	return B;
}

/**
 * \brief Decompose B into the camera matrix K using Cholesky factorization.
 * \param[in] B Matrix B calculated using the calculateB method.
 * \return The normalized calibration matrix K.
 */
Eigen::Matrix3d CameraCalibration::calibrationMatrix(const Eigen::Matrix3d& B) {
	Eigen::Matrix3d K;

	/* TODO: Calculate K from B using a Cholesky factorization and normalize
	 * K so that the bottom right entry is 1.
	 */
	Eigen::LLT<Eigen::MatrixXd> lltOfB(B);
	Eigen::MatrixXd L = lltOfB.matrixL();
	Eigen::MatrixXd LInverseTranspose = L.transpose().inverse();

	K << 	LInverseTranspose(0, 0)/LInverseTranspose(2, 2), LInverseTranspose(0, 1)/LInverseTranspose(2, 2), LInverseTranspose(0, 2)/LInverseTranspose(2, 2),
			LInverseTranspose(1, 0)/LInverseTranspose(2, 2), LInverseTranspose(1, 1)/LInverseTranspose(2, 2), LInverseTranspose(1, 2)/LInverseTranspose(2, 2),
			LInverseTranspose(2, 0)/LInverseTranspose(2, 2), LInverseTranspose(2, 1)/LInverseTranspose(2, 2), LInverseTranspose(2, 2)/LInverseTranspose(2, 2);

	return K;
}

/**
 * \brief Calculate the camera pose from where an image was taken relative to the checkerboard.
 * \param[in] homography The 2D homography of a single image calculated using the homographyFromPoints method.
 * \param[in] K The calibration matrix returned by the calibrationMatrix method.
 * \return The camera pose matrix (r1 r2 r3 t).
 */
Eigen::MatrixXd CameraCalibration::cameraPose(const Eigen::Matrix3d& homography, const Eigen::Matrix3d& K) {
	Eigen::MatrixXd cameraPoseMatrix(3,4);
	/* TODO: Compute the camera position and orientation matrix [r1 r2 r3 t]
	 * (i.e., the extrinsic parameters) using the equation H = K [r1 r2 t].
	 * Reconstruct the third column r3 of the rotation matrix using the fact that
	 * r3 is orthogonal to r1 and r2.
	 *
	 * The homography H is a 2D homography and is scale invariant, whereas
	 * [r1 r2 r3 t] is not scale invariant. You can reconstruct the missing
	 * scale factor by making sure that [r1 r2 r3] forms an orthonormal basis.
	 *
	 * Determine the scale factor that scales the norm of r1 to 1
	 * (the same scaling factor will also scale the norms of r2 and r3 to 1).
	 * Then scale the whole matrix [r1 r2 r3 t] with that scaling factor.
	 */
	Eigen::Vector3d r1;
	Eigen::Vector3d r2;
	Eigen::Vector3d r3;
	Eigen::MatrixXd r1r2t(3, 3);
	double r31, r32, r33, r1Sqrt, r2Sqrt;

	r1r2t = K.inverse()*homography;
	r1(0) = r1r2t(0, 0);
	r1(1) = r1r2t(1, 0);
	r1(2) = r1r2t(2, 0);
	r2(0) = r1r2t(0, 1);
	r2(1) = r1r2t(1, 1);
	r2(2) = r1r2t(2, 1);

    r31 = (-r1(2)*r2(1)+r1(1)*r2(2))/(r1(0)*r2(1)-r1(1)*r2(0));
    r32 = (-r1(0)*r2(2)+r1(2)*r2(0))/(r1(0)*r2(1)-r1(1)*r2(0));

    r33 = sqrt(1/(1+r31*r31+r32*r32));
    r31 = r31*r33;
    r32 = r32*r33;

    std::cout<<"r3: "<<r31<<" "<<r32<<" "<<r33<<std::endl;

	r1Sqrt = sqrt(1/(r1(0)*r1(0)+r1(1)*r1(1)+r1(2)*r1(2)));
	r2Sqrt = sqrt(1/(r2(0)*r2(0)+r2(1)*r2(1)+r2(2)*r2(2)));

	r1(0) *= r1Sqrt;
	r1(1) *= r1Sqrt;
	r1(2) *= r1Sqrt;
	r2(0) *= r2Sqrt;
	r2(1) *= r2Sqrt;
	r2(2) *= r2Sqrt;

	std::cout<<"r1Sqrt: "<<r1Sqrt<<std::endl;
	std::cout<<"r2Sqrt: "<<r2Sqrt<<std::endl;
	std::cout<<"r1: "<<r1<<std::endl;
	std::cout<<"r2: "<<r2<<std::endl;


    cameraPoseMatrix(0, 0) = r1(0);
    cameraPoseMatrix(1, 0) = r1(1);
    cameraPoseMatrix(2, 0) = r1(2);
    cameraPoseMatrix(0, 1) = r2(0);
    cameraPoseMatrix(1, 1) = r2(1);
    cameraPoseMatrix(2, 1) = r2(2);
    cameraPoseMatrix(0, 2) = r31;
    cameraPoseMatrix(1, 2) = r32;
    cameraPoseMatrix(2, 2) = r33;
    cameraPoseMatrix(0, 3) = r1r2t(0, 2);
    cameraPoseMatrix(1, 3) = r1r2t(1, 2);
    cameraPoseMatrix(2, 3) = r1r2t(2, 2);

	return cameraPoseMatrix;
}

/**
 * \brief Find a least-squares approximation of the overdetermined homogeneous linear system V.
 * \param[in] The matrix V calculated in calculateB
 * \return The least-squares solution
 *
 * This method uses the Moore-Penrose pseudo inverse to find a least squares
 * solution of the overdetermined homogeneous linear system V with the
 * side condition b.sum() = 1.
 */
Eigen::VectorXd CameraCalibration::solveV(const Eigen::MatrixXd& V) {
	// There's nothing to change in this method.
	if (V.cols() != 6) {
		std::stringstream ss;
		ss << "V is expected to have 6 columns, but is has " << V.cols() << " columns";
		throw std::invalid_argument(ss.str());
	}
	Eigen::MatrixXd V_(V.rows() + 1, 6);
	V_ << V, Eigen::VectorXd::Ones(6).transpose();

	Eigen::VectorXd r(V_.rows());
	r << Eigen::VectorXd::Zero(V.rows()), 1;

	Eigen::MatrixXd V_TV_ = V_.transpose() * V_;
	return V_TV_.inverse() * V_.transpose() * r;
}

}  // namespace camera_calibration
