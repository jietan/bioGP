#include "stdafx.h"
#include "RootConstraint.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"
#include "optimizer/Var.h"
#include "utils/UtilsMath.h"

namespace optimizer {

	RootConstraint::RootConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel)
		: Constraint(var), mSkel(skel)
	{
		mNumRows = 6;

		mWeight = VectorXd::Ones(mNumRows);
		mConstTerm = VectorXd::Zero(mNumRows);
		mCompletion = VectorXd::Zero(mNumRows);

		Eigen::VectorXd pose;
		mSkel->getPose(pose);
		mTarget = pose.head(mNumRows);
	}

	VectorXd RootConstraint::evalCon() {
		Eigen::VectorXd ret = VectorXd::Zero(mNumRows);

		Eigen::VectorXd pose;
		mSkel->getPose(pose);
		ret = (pose.head(mNumRows) - mTarget);
		return ret;
	}

	void RootConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
		for(int dofIndex = 0; dofIndex < mNumRows; dofIndex++)
		{
			jEntry->at(index + dofIndex)->at(dofIndex) = 1;
			jMap->at(index + dofIndex)->at(dofIndex) = true;
		}
	}

	void RootConstraint::fillObjGrad(std::vector<double>& dG) {
		VectorXd dP = evalCon();
		for(int dofIndex = 0; dofIndex < mNumRows; dofIndex++) {
			dG.at(dofIndex) += dP[dofIndex];
		}
	}

	void RootConstraint::setTarget(const Eigen::VectorXd& target) {
		this->mTarget = target;
	}

	VectorXd RootConstraint::getTarget() const {
		return mTarget;
	}
} // namespace optimizer
