#include "stdafx.h"
#include "poseConstraint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/optimizer/Var.h"

PoseConstraint::PoseConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel)
	: Constraint(var), mSkel(skel)
{
	
	mNumRows = static_cast<int>(var.size());
	mWeight = VectorXd::Ones(mNumRows);
	mWeight.head(6) = Eigen::VectorXd::Zero(6);
	//mWeight.segment(3, 3) = Eigen::VectorXd::Zero(3);
	mConstTerm = VectorXd::Zero(mNumRows);
	mCompletion = VectorXd::Zero(mNumRows);

	Eigen::VectorXd pose = mSkel->getPositions();
	mTarget = pose;
}

VectorXd PoseConstraint::evalCon() {
	Eigen::VectorXd ret = VectorXd::Zero(mNumRows);

	Eigen::VectorXd pose = mSkel->getPositions();
	ret = mWeight.cwiseProduct((pose - mTarget).tail(mSkel->getNumDofs()));

	return ret;
}

void PoseConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
}

void PoseConstraint::fillObjGrad(std::vector<double>& dG) {
	VectorXd dP = evalCon();
	int numVar = static_cast<int>(mVariables.size());
	for(int dofIndex = 0; dofIndex < numVar; dofIndex++) {
		dG.at(dofIndex) += mWeight[dofIndex] * dP[dofIndex];
	}
}

void PoseConstraint::setTarget(const Eigen::VectorXd& target) {
	this->mTarget = target;
}

VectorXd PoseConstraint::getTarget() const {
	return mTarget;
}

