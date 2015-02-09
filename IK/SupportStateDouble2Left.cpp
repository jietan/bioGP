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

}