#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include <Eigen/Dense>
#include <stdexcept>

namespace inverse_kinematics {

typedef Eigen::VectorXd EndeffectorPose;
typedef Eigen::Matrix3d Cartesian2DTransform;
typedef Eigen::VectorXd JointAngles;
typedef Eigen::MatrixXd Jacobian;


Eigen::MatrixXd pseudoInverse(const Jacobian &a)
{
  const double epsilon = std::numeric_limits<Jacobian::Scalar>::epsilon();

  if(a.rows() < a.cols()) {
      throw std::runtime_error("cannot compute pseudoinverse because the system is underdetermined");
  }

  Eigen::JacobiSVD< Jacobian > svd = a.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

  Jacobian::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

  return svd.matrixV() * Jacobian( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().
      array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
}

class InverseKinematics
{
public:
	InverseKinematics(const double& alpha) : alpha(alpha) {};
   	virtual ~InverseKinematics() {};

   	virtual Cartesian2DTransform rotation(const double& angle) const;
   	virtual Cartesian2DTransform translation(const double& distance_x, const double& distance_y) const;
   	virtual EndeffectorPose chooseStep(const EndeffectorPose& e, const EndeffectorPose& g) const;
   	virtual JointAngles computeJointChange(const EndeffectorPose& delta_e, const Jacobian& jacobian) const;
   	virtual JointAngles applyChangeToJoints(const JointAngles& q, const JointAngles& delta_q) const;
   	virtual void oneIteration(JointAngles& q, EndeffectorPose& e, const EndeffectorPose& g) const;

   	virtual Jacobian jacobian(const JointAngles& q) const = 0;
   	virtual EndeffectorPose forwardKinematic(const JointAngles& q) const = 0;

   	const double alpha;
};

class InverseKinematics_2Links : public InverseKinematics {
public:
	InverseKinematics_2Links(const double& a0, const double& a1, const double& h, const double& alpha) : InverseKinematics(alpha), a0(a0), a1(a1), h(h) {};
	virtual ~InverseKinematics_2Links() {};

	virtual JointAngles computeIK(EndeffectorPose& g, const double& maxTranslationalError) const;
	virtual Jacobian jacobian(const JointAngles& q) const;
	virtual EndeffectorPose forwardKinematic(const JointAngles& q) const;

   	const double a0;
   	const double a1;
   	const double h;
};

class InverseKinematics_3Links : public InverseKinematics {
public:
	InverseKinematics_3Links(const double& a0, const double& a1, const double& a2, const double& h, const double& alpha) : InverseKinematics(alpha), a0(a0), a1(a1), a2(a2), h(h) {};
	virtual ~InverseKinematics_3Links() {};

	virtual JointAngles computeIK(EndeffectorPose& g, const double& maxTranslationalError, const double& maxAngularError) const;
	virtual Jacobian jacobian(const JointAngles& q) const;
	virtual EndeffectorPose forwardKinematic(const JointAngles& q) const;

   	const double a0;
   	const double a1;
   	const double a2;
   	const double h;
};

}  // namespace inverse_kinematics

#endif  // INVERSE_KINEMATICS_H_
