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

}