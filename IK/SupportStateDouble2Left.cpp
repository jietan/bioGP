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
	
	double comOffsetX = 0;
	double comOffsetZ = 0;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "COMOffsetX", comOffsetX);
	DecoConfig::GetSingleton()->GetDouble("Mocap", "COMOffsetZ", comOffsetZ);
	Eigen::Vector3d endCOM = mInitialLeftFoot + Eigen::Vector3d(comOffsetX, 0, comOffsetZ);
	Eigen::Vector3d comTarget = mInitialCOM + (endCOM - mInitialCOM) / (mEndFrame - mStartFrame) * (frameNum - mStartFrame);//  currentCOM + (endCOM - currentCOM) / (mEndFrame - frameNum);
	addCOMObjective(frameNum, comTarget, ik);

	if (frameNum == mEndFrame - 1 && mNext)
	{
		mNext->SetInitialCOMLocation(comTarget);
	}
}