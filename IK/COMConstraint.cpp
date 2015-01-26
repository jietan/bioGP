#include "stdafx.h"
#include "COMConstraint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/optimizer/Var.h"
//#include "utils/UtilsMath.h"



COMConstraint::COMConstraint(
	std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel)
	: Constraint(var), mSkel(skel)
{
	mAxisMask = 0;
	
	mNumRows = 3;

	mWeight = 10 * VectorXd::Ones(mNumRows);
	mConstTerm = VectorXd::Zero(mNumRows);
	mCompletion = VectorXd::Zero(mNumRows);
}

VectorXd COMConstraint::evalCon() {
	int numAxis = getNumControlAxis();
	if (!numAxis)
		return VectorXd::Zero(1);

	Vector3d com = mSkel->getWorldCOM();

	Vector3d C = mWeight.cwiseProduct(com - mTarget);

	VectorXd ret = VectorXd::Zero(numAxis);
	vector<int> axisId = getControlAxis();
	for (int i = 0; i < numAxis; ++i)
		ret[i] = C[axisId[i]];
	return ret;
}

void COMConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
	int numBodies = mSkel->getNumBodyNodes();

	int startId = 0;

	int numAxis = getNumControlAxis();
	if (!numAxis)
		return;
	vector<int> axisId = getControlAxis();

	for (int bodyIndex = 0; bodyIndex < numBodies; ++bodyIndex)
	{
		dart::dynamics::BodyNode* node = mSkel->getBodyNode(bodyIndex);
		MatrixXd bodyCOMDeriv = node->getWorldLinearJacobian();

		for (int dofIndex = startId; dofIndex < node->getNumDependentGenCoords(); dofIndex++)
		{
			Vector3d comDeriv = (node->getMass() / mSkel->getMass()) * bodyCOMDeriv.col(dofIndex);

			int i = node->getDependentGenCoordIndex(dofIndex);

			for (int ithAxis = 0; ithAxis < numAxis; ++ithAxis)
			{
				jEntry->at(index + ithAxis)->at(i - startId) += mWeight[axisId[ithAxis]] * comDeriv[axisId[ithAxis]];
				jMap->at(index + ithAxis)->at(i - startId) = true;
			}

			// VLOG(1) << "dofIndex = " << dofIndex;

		}

	}


}

void COMConstraint::fillObjGrad(std::vector<double>& dG) {

	int startId = 0;

	int numBodies = mSkel->getNumBodyNodes();
	int numDofs = mSkel->getNumDofs();

	int numAxis = getNumControlAxis();
	if (!numAxis)
	{
		for (int i = startId; i < numDofs; ++i)
		{
			dG.at(i - startId) = 0;
		}
		return;
	}
	vector<int> axisId = getControlAxis();

	VectorXd dP = evalCon();

	MatrixXd Jac = MatrixXd::Zero(3, numDofs - startId);
	for (int bodyIndex = 0; bodyIndex < numBodies; ++bodyIndex)
	{
		dart::dynamics::BodyNode* node = mSkel->getBodyNode(bodyIndex);
		MatrixXd bodyCOMDeriv = node->getWorldLinearJacobian();

		for (int dofIndex = startId; dofIndex < node->getNumDependentGenCoords(); dofIndex++) {
			Vector3d comDeriv = (node->getMass() / mSkel->getMass()) * bodyCOMDeriv.col(dofIndex);
				
			int i = node->getDependentGenCoordIndex(dofIndex);
			Jac.col(i - startId) += comDeriv;
		}
	}
	VectorXd reducedWeight = VectorXd::Zero(numAxis);
	for (int ithAxis = 0; ithAxis < numAxis; ++ithAxis)
	{
		reducedWeight[ithAxis] = mWeight[axisId[ithAxis]];
	}
	for (int i = startId; i < numDofs; ++i)
	{
		VectorXd reducedJacCol = VectorXd::Zero(numAxis);
		for (int ithAxis = 0; ithAxis < numAxis; ++ithAxis)
		{
			reducedJacCol[ithAxis] = Jac.col(i - startId)[axisId[ithAxis]];
		}
		dG.at(i - startId) += reducedWeight.cwiseProduct(dP).dot(reducedJacCol);
	}


}

void COMConstraint::setTarget(const Eigen::Vector3d& target, int axisMask) {
	this->mTarget = target;
	mAxisMask = axisMask;
}

Vector3d COMConstraint::getTarget() const {
	return mTarget;
}

int COMConstraint::getNumControlAxis() const
{
	int ret = 0;
	ret += mAxisMask & COM_CONSTRAINT_X ? 1 : 0;
	ret += mAxisMask & COM_CONSTRAINT_Y ? 1 : 0;
	ret += mAxisMask & COM_CONSTRAINT_Z ? 1 : 0;
	return ret;
}
vector<int> COMConstraint::getControlAxis() const
{
	vector<int> ret;
	if (mAxisMask & COM_CONSTRAINT_X)
		ret.push_back(0);
	if (mAxisMask & COM_CONSTRAINT_Y)
		ret.push_back(1);
	if (mAxisMask & COM_CONSTRAINT_Z)
		ret.push_back(2);
	return ret;
}

