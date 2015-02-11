#include "stdafx.h"
#include "DirectionConstraint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/optimizer/Var.h"
#include "dart/math/Geometry.h"


DirectionConstraint::DirectionConstraint(
	std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel, dart::dynamics::BodyNode* node,
	const Eigen::Vector3d& offset1, const Eigen::Vector3d& offset2, const Eigen::Vector3d& val, double weight)
	: Constraint(var), mSkel(skel), mNode(node), mTargetDir(val), mOffset1(offset1), mOffset2(offset2)
{
	mNumRows = 1;

	mWeight = weight * VectorXd::Ones(mNumRows);
	mConstTerm = VectorXd::Zero(mNumRows);
	mCompletion = VectorXd::Zero(mNumRows);
}

VectorXd DirectionConstraint::evalCon() {
	Vector3d wp1 = mNode->getTransform() * mOffset1;
	Vector3d wp2 = mNode->getTransform() * mOffset2;
	double C = mWeight[0] * (0.1 - (wp2 - wp1).dot(mTargetDir));
	VectorXd ret = VectorXd::Zero(1);
	ret[0] = C;
	// cout << "mNode = " << mNode->getModelIndex() << " : "
	//      << ret.transpose() << endl;
	return ret;
}

void DirectionConstraint::fillJac(VVD jEntry, VVB jMap, int index) {

	Eigen::MatrixXd jacobian1 = mNode->getWorldLinearJacobian(mOffset1);
	Eigen::MatrixXd jacobian2 = mNode->getWorldLinearJacobian(mOffset2);
	for (int dofIndex = 0; dofIndex < mNode->getNumDependentGenCoords(); dofIndex++)
	{
		// VLOG(1) << "dofIndex = " << dofIndex;
		int i = mNode->getDependentGenCoordIndex(dofIndex);

		// VLOG(1) << "w = " << w;
		VectorXd J1 = jacobian1.col(dofIndex);
		VectorXd J2 = jacobian2.col(dofIndex);

		jEntry->at(index + 0)->at(i) = mWeight[0] * (J1 - J2).dot(mTargetDir);
		jMap->at(index + 0)->at(i) = true;

	}
}



void DirectionConstraint::setTarget(const Eigen::Vector3d& target) {
	this->mTargetDir = target;
}

Vector3d DirectionConstraint::getTarget() const {
	return mTargetDir;
}

