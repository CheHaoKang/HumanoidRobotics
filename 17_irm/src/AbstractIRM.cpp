#include <irm/AbstractIRM.h>
#include <angles/angles.h>

namespace irm {

void AbstractIRM::doComputeIRM() {
	std::vector<RMVoxel> voxels;
	voxels.reserve(rm.size());
	for (RMMapType::const_iterator it = rm.begin(); it != rm.end(); ++it) {
		RMVoxel voxel;
		voxel.configurations.reserve(it->second->configurations.size());
		voxel.configurations.insert(voxel.configurations.end(), it->second->configurations.begin(), it->second->configurations.end());
		voxels.push_back(voxel);
	}
	computeIRM(voxels);
}

bool AbstractIRM::forwardKinematics(const Eigen::Vector3d& jointAngles, Eigen::Vector3d& endeffectorPose) const {
	//Creation of jntarrays:
	KDL::JntArray q(chain->getNrOfJoints());
	q.data << jointAngles;

	KDL::Frame fkPos;
	const int ret = fksolver1->JntToCart(q, fkPos);
	double roll, pitch, yaw;
	fkPos.	M.GetRPY(roll, pitch, yaw);
	endeffectorPose << fkPos.p.z(), -fkPos.p.y(), angles::normalize_angle(roll);
	return ret == 0;
}

void AbstractIRM::addToIRM(const Eigen::Vector3d& basePose, const Eigen::Vector3d& jointAngles, const double& manipulability) {
	size_t idx = getIndex(irmMapConfig, basePose, true);
	if (idx == INVALID) {
		return;
	}
	std::pair<IRMMapType::iterator, bool> result = irm2d.emplace(idx, new IRMEntry(basePose));
	IRMEntry * entry = result.first->second;

	if (entry->manipulability > manipulability) {
		entry->jointAngles = jointAngles;
		entry->basePose = basePose;
		entry->manipulability = manipulability;
	}
	JointConfiguration el;
	el.jointAngles = jointAngles;
	el.manipulability = manipulability;
	entry->configurations.push_back(el);
}


bool AbstractIRM::inverseKinematics(const Eigen::Vector3d& endeffectorPose, Eigen::Vector3d& jointAngles) const {
	//Set destination frame
	KDL::Frame F_dest;
	F_dest.p.Set2DYZ(KDL::Vector2(-endeffectorPose[1], endeffectorPose[0]));
	F_dest.M = KDL::Rotation::RPY((double) endeffectorPose[2], 0., 0.);

	KDL::JntArray q(chain->getNrOfJoints());
	KDL::JntArray q_init(chain->getNrOfJoints());
	q_init.data << 0.1, 0.1, 0.1;

	const int ret = iksolver1->CartToJnt(q_init,F_dest,q);
	jointAngles << angles::normalize_angle((double) q.data[0]),
			angles::normalize_angle((double) q.data[1]),
			angles::normalize_angle((double) q.data[2]);
	return ret == KDL::ChainIkSolverPos_NR::E_NOERROR;
}

void AbstractIRM::addToRM(const Eigen::Vector3d& jointAngles, const Eigen::Vector3d& endeffectorPose, const double& manipulability) {
	std::pair<size_t, RMMapType *> c[2] = {
			std::make_pair(getIndex(rmMapConfig, endeffectorPose, false), &rm),
			std::make_pair(getIndex(rmMapConfig, endeffectorPose, true), &rm2d),
	};
	for (size_t i = 0; i < 2; ++i) {
		if (c[i].first == INVALID) {
			return;
		}
		std::pair<RMMapType::iterator, bool> result = c[i].second->emplace(c[i].first, new RMEntry(endeffectorPose));
		RMEntry * entry = result.first->second;

		if (entry->manipulability > manipulability) {
			entry->jointAngles = jointAngles;
			entry->endeffectorPose = endeffectorPose;
			entry->manipulability = manipulability;
		}
		JointConfiguration el;
		el.jointAngles = jointAngles;
		el.manipulability = manipulability;
		entry->configurations.push_back(el);
	}
}

size_t AbstractIRM::getIndex(const std::vector<MapConfig>& mapConfig, const Eigen::Vector3d& pose, const bool& use2d) const {
	size_t idx = 0;
	size_t mult = 1;
	for (size_t i = 0; i < (use2d ? 2 : 3); ++i) {
		const double v = i < 2 ? pose[i] : angles::normalize_angle((double) pose[i]);
		const size_t j = (v - mapConfig[i].min) / mapConfig[i].res;
		if (j > mapConfig[i].numCells) {
			return INVALID;
		}
		idx = idx * mult + j;
		mult += mapConfig[i].numCells;
	}
	return idx;
}

AbstractIRM::AbstractIRM() {
	chain = new KDL::Chain();
	chain->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,3.0))));
	chain->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,2.0))));
	chain->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.5))));

	//Creation of the solvers:
	fksolver1 = new KDL::ChainFkSolverPos_recursive(*chain);//Forward position solver
	iksolver1v = new KDL::ChainIkSolverVel_pinv(*chain);//Inverse velocity solver
	iksolver1 = new KDL::ChainIkSolverPos_NR(*chain,*fksolver1,*iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	rmMapConfig.resize(3);
	rmMapConfig[0].min = -6.5;
	rmMapConfig[0].max = 6.5;
	rmMapConfig[0].res = 0.1;
	rmMapConfig[1].min = -0.5;
	rmMapConfig[1].max = 6.5;
	rmMapConfig[1].res = 0.1;
	rmMapConfig[2].min = -M_PI;
	rmMapConfig[2].max = M_PI;
	rmMapConfig[2].res = angles::from_degrees(10.0);

	irmMapConfig.resize(3);
	irmMapConfig[0].min = -6.0;
	irmMapConfig[0].max =  6.0;
	irmMapConfig[0].res =  0.1;
	irmMapConfig[1].min = -6.0;
	irmMapConfig[1].max =  6.0;
	irmMapConfig[1].res =  0.1;
	irmMapConfig[2].min = -M_PI;
	irmMapConfig[2].max =  M_PI;
	irmMapConfig[2].res = angles::from_degrees(10.0);
	for (size_t i = 0; i < 3; ++i) {
		rmMapConfig[i].numCells = (rmMapConfig[0].max - rmMapConfig[0].min) / rmMapConfig[0].res + 1;
		irmMapConfig[i].numCells = (irmMapConfig[0].max - irmMapConfig[0].min) / irmMapConfig[0].res + 1;
	}
}

AbstractIRM::~AbstractIRM() {
	delete iksolver1;
	delete fksolver1;
	delete iksolver1v;
	fksolver1 = NULL;
	iksolver1 = NULL;
	iksolver1v = NULL;
	RMMapType::iterator it = rm.begin();
	while (it != rm.end()) {
		delete it->second;
		it = rm.erase(it);
	}
	rm.clear();
	it = rm2d.begin();
	while (it != rm2d.end()) {
		delete it->second;
		it = rm2d.erase(it);
	}
	rm2d.clear();
	IRMMapType::iterator it2 = irm2d.begin();
	while (it2 != irm2d.end()) {
		delete it2->second;
		it2 = irm2d.erase(it2);
	}
	irm2d.clear();

}

}  // namespace irm
