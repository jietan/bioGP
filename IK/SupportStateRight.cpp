#include "SupportStateRight.h"
#include "dart/dynamics/Skeleton.h"
#include "IKProblem.h"

SupportStateRight::SupportStateRight()
{
	mType = SST_RIGHT;
}
SupportStateRight::~SupportStateRight()
{
}

void SupportStateRight::AddConstraint(int frameNum, IKProblem* ik)
{
	snapshotInitialFootLocations(frameNum);
	addRightFootConstraint(frameNum, mInitialRightFoot, true, ik);

	Eigen::Vector3d leftFootTarget = mOrig->getMarker("lfoot")->getWorldPosition();
	if (leftFootTarget[1] < 0)
		leftFootTarget[1] = 0;
	addLeftFootObjective(frameNum, leftFootTarget, false, ik);

	Eigen::Vector3d comTarget = mInitialRightFoot;
	addCOMObjective(frameNum, comTarget, ik);
}