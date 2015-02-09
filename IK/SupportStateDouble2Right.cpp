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

}