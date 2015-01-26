#include "stdafx.h"
#include "headConstraint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/optimizer/Var.h"
#include "dart/math/Geometry.h"


HeadConstraint::HeadConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel, dart::dynamics::BodyNode* node,
								const Eigen::Vector3d& offset,
								const Eigen::Vector3d& globalUp,
								const Eigen::Vector3d& globalFront)
: Constraint(var), mSkel(skel), mNode(node), mOffset(offset)
{
	mTargetPos = mNode->getTransform() * offset;
	mTargetUp = globalUp.normalized();
	mTargetFront = globalFront.normalized();
	mLocalUp = mNode->getTransform().inverse().linear() * mTargetUp;
	mLocalFront = mNode->getTransform().inverse().linear() * mTargetFront;
	mNumRows = 9;
	mWeight = VectorXd::Ones(mNumRows);
	mWeight.tail(3) *= 10;
	mConstTerm = VectorXd::Zero(mNumRows);
	mCompletion = VectorXd::Zero(mNumRows);
}

VectorXd HeadConstraint::evalCon() {
	Vector3d wPos = mNode->getTransform() * mOffset;
	Vector3d wUp = mNode->getTransform().linear() * mLocalUp;
	Vector3d wFront = mNode->getTransform().linear() * mLocalFront;

	VectorXd ret = VectorXd::Zero(mNumRows);
	ret.head(3) = wPos - mTargetPos;
	ret.segment(3, 3) = wUp - mTargetUp;
	ret.tail(3) = wFront - mTargetFront;
	ret = mWeight.cwiseProduct(ret);
	return ret;
}

void HeadConstraint::fillJac(VVD jEntry, VVB jMap, int index) {


	for (int dofIndex = 0; dofIndex < mNode->getNumDependentGenCoords(); dofIndex++)
	{
		// VLOG(1) << "dofIndex = " << dofIndex;
		int i = mNode->getDependentGenCoordIndex(dofIndex);

		// VLOG(1) << "w = " << w;
		VectorXd J = dart::math::xformHom(mNode->getDerivWorldTransform(dofIndex), mOffset);
		jEntry->at(index + 0)->at(i) = J[0];
		jEntry->at(index + 1)->at(i) = J[1];
		jEntry->at(index + 2)->at(i) = J[2];
		jMap->at(index + 0)->at(i) = true;
		jMap->at(index + 1)->at(i) = true;
		jMap->at(index + 2)->at(i) = true;

		VectorXd JUp = dart::math::xformHomDir(mNode->getDerivWorldTransform(dofIndex), mLocalUp);
		jEntry->at(index + 3)->at(i) = JUp[0];
		jEntry->at(index + 4)->at(i) = JUp[1];
		jEntry->at(index + 5)->at(i) = JUp[2];
		jMap->at(index + 3)->at(i) = true;
		jMap->at(index + 4)->at(i) = true;
		jMap->at(index + 5)->at(i) = true;

		VectorXd JFront = dart::math::xformHomDir(mNode->getDerivWorldTransform(dofIndex), mLocalFront);
		jEntry->at(index + 6)->at(i) = JFront[0];
		jEntry->at(index + 7)->at(i) = JFront[1];
		jEntry->at(index + 8)->at(i) = JFront[2];
		jMap->at(index + 6)->at(i) = true;
		jMap->at(index + 7)->at(i) = true;
		jMap->at(index + 8)->at(i) = true;

	}
}

void HeadConstraint::fillObjGrad(std::vector<double>& dG) {
	VectorXd dP = evalCon();
	int startIdx = 0;
	int offsetIdx = 0;

	for (int dofIndex = startIdx; dofIndex < mNode->getNumDependentGenCoords(); dofIndex++) {
		// VLOG(1) << "dofIndex = " << dofIndex;
		int i = mNode->getDependentGenCoordIndex(dofIndex);
		// VLOG(1) << "i = " << i;

		const dart::optimizer::Var* v = mVariables[i];
		double w = v->mWeight;
		// VLOG(1) << "w = " << w;

		VectorXd J = VectorXd::Zero(mNumRows);

		VectorXd J0 = dart::math::xformHom(mNode->getDerivWorldTransform(dofIndex), mOffset);
		VectorXd J1 = dart::math::xformHomDir(mNode->getDerivWorldTransform(dofIndex), mLocalUp);
		VectorXd J2 = dart::math::xformHomDir(mNode->getDerivWorldTransform(dofIndex), mLocalFront);

		J.head(3) = J0;
		J.segment(3,3) = J1;
		J.tail(3) = J2;
		J /= w;
		J = mWeight.cwiseProduct(J);
		// VLOG(1) << "J = " << J.transpose();
		dG.at(i + offsetIdx) += dP.dot(J);
	}
}

void HeadConstraint::setPositionTarget(const Eigen::Vector3d& target)
{
	mTargetPos = target;
}
void HeadConstraint::setFrontTarget(const Eigen::Vector3d& target)
{
	mTargetFront = target;
}
void HeadConstraint::SetUpTarget(const Eigen::Vector3d& target)
{
	mTargetUp = target;
}

