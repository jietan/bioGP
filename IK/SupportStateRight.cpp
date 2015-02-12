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
	Eigen::Vector3d realRightFootConstraint = mInitialRightFoot;
	realRightFootConstraint[1] = 0;
	addRightFootConstraint(frameNum, realRightFootConstraint, true, ik);

	Eigen::Vector3d leftFootTarget = mOrig->getMarker("lfoot")->getWorldPosition();
	//leftFootTarget[1] = leftFootTarget[1] - mInitialRightFoot[1];
	//if (leftFootTarget[1] < 0)
	//	leftFootTarget[1] = 0;
	double stepHeight = 0;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "StepHeight", stepHeight);
	leftFootTarget[1] = stepHeight * sin(static_cast<double>(frameNum - mStartFrame) / (mEndFrame - mStartFrame) * (M_PI + 0.1));
	if (leftFootTarget[1] < 0)
		leftFootTarget[1] = 0;
	addLeftFootConstraint(frameNum, leftFootTarget, false, ik);
	//addLeftFootConstraint(frameNum, leftFootTarget, false, ik);

	double comOffsetX = 0;
	double comOffsetZ = 0;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "COMOffsetX", comOffsetX);
	DecoConfig::GetSingleton()->GetDouble("Mocap", "COMOffsetZ", comOffsetZ);
	Eigen::Vector3d currentCOM = mTarget->getWorldCOM();

	Eigen::Vector3d comTarget = mInitialRightFoot + Eigen::Vector3d(-comOffsetX, 0, comOffsetZ);
	addCOMObjective(frameNum, comTarget, ik);

	if (frameNum == mEndFrame - 1 && mNext)
	{
		mNext->SetInitialCOMLocation(comTarget);
	}
}