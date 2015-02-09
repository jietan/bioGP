#include "SupportState.h"
#include "dart/dynamics/Skeleton.h"
#include "IKProblem.h"

SupportState::SupportState() : mOrig(NULL), mTarget(NULL)
{
}
SupportState::~SupportState()
{
}
SupportStateType SupportState::GetType() const
{
	return mType;
}
void SupportState::SetFrameRange(int start, int end)
{
	mStartFrame = start;
	mEndFrame = end;
}
void SupportState::SetSkeletons(dart::dynamics::Skeleton* origSkel, dart::dynamics::Skeleton* targetSkel)
{
	mOrig = origSkel;
	mTarget = targetSkel;
}
void SupportState::AddConstraint(int frameNum, IKProblem* ik)
{

}