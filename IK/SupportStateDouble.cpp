#include "SupportStateDouble.h"
#include "dart/dynamics/Skeleton.h"
#include "IKProblem.h"

SupportStateDouble::SupportStateDouble()
{
	mType = SST_DOUBLE;
}
SupportStateDouble::~SupportStateDouble()
{
}

void SupportStateDouble::AddConstraint(int frameNum, IKProblem* ik)
{

}