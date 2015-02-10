#include "SupportStateDouble2Left.h"
#include "dart/dynamics/Skeleton.h"
#include "IKProblem.h"

SupportStateDouble2Left::SupportStateDouble2Left()
{
	mType = SST_DOUBLE2LEFT;
}
SupportStateDouble2Left::~SupportStateDouble2Left()
{
}

void SupportStateDouble2Left::AddConstraint(int frameNum, IKProblem* ik)
{
	snapshotInitialFootLocations(frameNum);
	addDoubleFootConstraint(frameNum, mInitialLeftFoot, mInitialRightFoot, ik);
	Eigen::Vector3d currentCOM = mTarget->getWorldCOM();
	Eigen::Vector3d endCOM = mInitialLeftFoot;
	Eigen::Vector3d comTarget = endCOM;//  currentCOM + (endCOM - currentCOM) / (mEndFrame - frameNum);
	addCOMObjective(frameNum, comTarget, ik);
}