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

	double comOffsetX = 0;
	double comOffsetZ = 0;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "COMOffsetX", comOffsetX);
	DecoConfig::GetSingleton()->GetDouble("Mocap", "COMOffsetZ", comOffsetZ);
	Eigen::Vector3d endCOM = mInitialRightFoot + Eigen::Vector3d(-comOffsetX, 0, comOffsetZ);
	Eigen::Vector3d comTarget = mInitialCOM + (endCOM - mInitialCOM) / (mEndFrame - mStartFrame) * (frameNum - mStartFrame);//  currentCOM + (endCOM - currentCOM) / (mEndFrame - frameNum);

	addCOMObjective(frameNum, comTarget, ik);
}