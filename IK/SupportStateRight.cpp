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

}