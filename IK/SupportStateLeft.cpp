#include "SupportStateLeft.h"
#include "dart/dynamics/Skeleton.h"
#include "IKProblem.h"

SupportStateLeft::SupportStateLeft() 
{
	mType = SST_LEFT;
}
SupportStateLeft::~SupportStateLeft()
{
}
void SupportStateLeft::AddConstraint(int frameNum, IKProblem* ik)
{
	snapshotInitialFootLocations(frameNum);
	addLeftFootConstraint(frameNum, mInitialLeftFoot, true, ik);

	Eigen::Vector3d rightFootTarget = mOrig->getMarker("rfoot")->getWorldPosition();
	if (rightFootTarget[1] < 0)
		rightFootTarget[1] = 0;
	addRightFootObjective(frameNum, rightFootTarget, false, ik);

	Eigen::Vector3d comTarget = mInitialLeftFoot;
	addCOMObjective(frameNum, comTarget, ik);
}