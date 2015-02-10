#include "SupportStateDouble.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Marker.h"
#include "IKProblem.h"
#include "PositionConstraint.h"


SupportStateDouble::SupportStateDouble()
{
	mType = SST_DOUBLE;
}
SupportStateDouble::~SupportStateDouble()
{
}

void SupportStateDouble::AddConstraint(int frameNum, IKProblem* ik)
{
	snapshotInitialFootLocations(frameNum);
	addDoubleFootConstraint(frameNum, mInitialLeftFoot, mInitialRightFoot, ik);
	Eigen::Vector3d currentCOM = mTarget->getWorldCOM();
	Eigen::Vector3d endCOM = (mInitialLeftFoot + mInitialRightFoot) / 2;
	Eigen::Vector3d comTarget = endCOM;// currentCOM + (endCOM - currentCOM) / (mEndFrame - frameNum);
	addCOMObjective(frameNum, comTarget, ik);
}