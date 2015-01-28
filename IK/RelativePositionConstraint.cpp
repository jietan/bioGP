#include "stdafx.h"
#include "RelativePositionConstraint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/optimizer/Var.h"
#include "dart/math/Geometry.h"

    
RelativePositionConstraint::RelativePositionConstraint(
    std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel, 
	dart::dynamics::BodyNode* node1, const Eigen::Vector3d& offset1,
	dart::dynamics::BodyNode* node2, const Eigen::Vector3d& offset2,
	double distanceSqr)
	: Constraint(var), mSkel(skel), mNode1(node1), mOffset1(offset1), mNode2(node2), mOffset2(offset2), mDistanceSqr(distanceSqr)
{
	mEquality = 1;
    mNumRows = 1;

    mWeight = 10 * VectorXd::Ones(mNumRows);
    mConstTerm = VectorXd::Zero(mNumRows);
    mCompletion = VectorXd::Zero(mNumRows);
}

VectorXd RelativePositionConstraint::evalCon() {
    Vector3d wp1 = mNode1->getTransform() * mOffset1;
	Vector3d wp2 = mNode2->getTransform() * mOffset2;
    double distSqr = (wp1 - wp2).dot(wp1 - wp2);
	VectorXd ret = VectorXd::Zero(1);
	ret[0] = mWeight[0] * (distSqr - mDistanceSqr);
    return ret;
}

void RelativePositionConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
	
	Eigen::Vector3d pointDiffDoubled = mWeight[0] * 2 * (mNode1->getTransform() * mOffset1 - mNode2->getTransform() * mOffset2);
	Eigen::MatrixXd jacobian1 = mNode1->getWorldLinearJacobian(mOffset1);
	Eigen::MatrixXd jacobian2 = mNode2->getWorldLinearJacobian(mOffset2);
	for(int dofIndex = 0; dofIndex < mNode1->getNumDependentGenCoords(); dofIndex++)
	{
		// VLOG(1) << "dofIndex = " << dofIndex;
		int i = mNode1->getDependentGenCoordIndex(dofIndex);

		// VLOG(1) << "w = " << w;
		VectorXd J = jacobian1.col(dofIndex);
		
		jEntry->at(index)->at(i) = pointDiffDoubled.dot(J);
		jMap->at(index)->at(i) = true;
	}
	for (int dofIndex = 0; dofIndex < mNode2->getNumDependentGenCoords(); dofIndex++)
	{
		// VLOG(1) << "dofIndex = " << dofIndex;
		int i = mNode2->getDependentGenCoordIndex(dofIndex);

		// VLOG(1) << "w = " << w;
		VectorXd J = jacobian2.col(dofIndex);

		jEntry->at(index)->at(i) -= pointDiffDoubled.dot(J);
		jMap->at(index)->at(i) = true;
	}
}

