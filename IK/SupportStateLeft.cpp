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
	Eigen::Vector3d realLeftFootConstraint = mInitialLeftFoot;
	realLeftFootConstraint[1] = 0;
	addLeftFootConstraint(frameNum, realLeftFootConstraint, true, ik);

	Eigen::Vector3d rightFootTarget = mOrig->getMarker("rfoot")->getWorldPosition();
	//rightFootTarget[1] -= mInitialLeftFoot[1];
	//if (rightFootTarget[1] < 0)
	//	rightFootTarget[1] = 0;
	double stepHeight = 0;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "StepHeight", stepHeight);
	rightFootTarget[1] = stepHeight * sin(static_cast<double>(frameNum - mStartFrame) / (mEndFrame - mStartFrame) * M_PI);
	addRightFootConstraint(frameNum, rightFootTarget, false, ik);
	//addRightFootConstraint(frameNum, rightFootTarget, false, ik);

	double comOffsetX = 0;
	double comOffsetZ = 0;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "COMOffsetX", comOffsetX);
	DecoConfig::GetSingleton()->GetDouble("Mocap", "COMOffsetZ", comOffsetZ);
	Eigen::Vector3d comTarget = mInitialLeftFoot + Eigen::Vector3d(comOffsetX, 0, comOffsetZ);
	addCOMObjective(frameNum, comTarget, ik);

	if (frameNum == mEndFrame - 1 && mNext)
	{
		mNext->SetInitialCOMLocation(comTarget);
	}
}