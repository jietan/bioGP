#include "stdafx.h"
#include "PositionConstraint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/optimizer/Var.h"
#include "dart/math/Geometry.h"

    
PositionConstraint::PositionConstraint(
    std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel, dart::dynamics::BodyNode* node,
    const Eigen::Vector3d& offset, const Eigen::Vector3d& val, double weight)
    : Constraint(var), mSkel(skel), mNode(node), mTarget(val), mOffset(offset)
{
    mNumRows = 3;

    mWeight = weight * VectorXd::Ones(mNumRows);
    mConstTerm = VectorXd::Zero(mNumRows);
    mCompletion = VectorXd::Zero(mNumRows);
}

VectorXd PositionConstraint::evalCon() {
    Vector3d wp = mNode->getTransform() * mOffset;
	Vector3d C = mWeight.cwiseProduct(wp - mTarget);
    VectorXd ret(C);
    // cout << "mNode = " << mNode->getModelIndex() << " : "
    //      << ret.transpose() << endl;
    return ret;
}

void PositionConstraint::fillJac(VVD jEntry, VVB jMap, int index) {

	Eigen::MatrixXd jacobian = mNode->getWorldLinearJacobian(mOffset);
	for(int dofIndex = 0; dofIndex < mNode->getNumDependentGenCoords(); dofIndex++)
	{
		// VLOG(1) << "dofIndex = " << dofIndex;
		int i = mNode->getDependentGenCoordIndex(dofIndex);

		// VLOG(1) << "w = " << w;
		VectorXd J = jacobian.col(dofIndex);
		
		jEntry->at(index + 0)->at(i) = mWeight[0] * J[0];
		jEntry->at(index + 1)->at(i) = mWeight[1] * J[1];
		jEntry->at(index + 2)->at(i) = mWeight[2] * J[2];
		jMap->at(index + 0)->at(i) = true;
		jMap->at(index + 1)->at(i) = true;
		jMap->at(index + 2)->at(i) = true;
	}
}

void PositionConstraint::fillObjGrad(std::vector<double>& dG) {
    VectorXd dP = evalCon();
	Eigen::MatrixXd jacobian = mNode->getWorldLinearJacobian(mOffset);
	for (int dofIndex = 0; dofIndex < mNode->getNumDependentGenCoords(); dofIndex++) {
        // VLOG(1) << "dofIndex = " << dofIndex;
		int i = mNode->getDependentGenCoordIndex(dofIndex);
        // VLOG(1) << "i = " << i;
            
        const dart::optimizer::Var* v = mVariables[i];
        double w = v->mWeight;
        // VLOG(1) << "w = " << w;

		VectorXd J = jacobian.col(dofIndex);
        J /= w;
        // VLOG(1) << "J = " << J.transpose();
        dG.at(i) += dP.dot(J);
    }
}

void PositionConstraint::setTarget(const Eigen::Vector3d& target) {
    this->mTarget = target;
}

Vector3d PositionConstraint::getTarget() const {
    return mTarget;
}

