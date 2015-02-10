#include "SupportStateDouble2Right.h"
#include "dart/dynamics/Skeleton.h"
#include "IKProblem.h"

SupportStateDouble2Right::SupportStateDouble2Right() 
{
	mType = SST_DOUBLE2RIGHT;
}
SupportStateDouble2Right::~SupportStateDouble2Right()
{
}
void SupportStateDouble2Right::AddConstraint(int frameNum, IKProblem* ik)
{
	snapshotInitialFootLocations(frameNum);
	addDoubleFootConstraint(frameNum, mInitialLeftFoot, mInitialRightFoot, ik);
	Eigen::Vector3d currentCOM = mTarget->getWorldCOM();
	Eigen::Vector3d endCOM = mInitialRightFoot;
	Eigen::Vector3d comTarget = endCOM;// currentCOM + (endCOM - currentCOM) / 2;
	addCOMObjective(frameNum, comTarget, ik);
}