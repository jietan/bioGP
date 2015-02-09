#ifndef _SUPPORT_STATE_H
#define _SUPPORT_STATE_H


class IKProblem;
namespace dart
{
	namespace dynamics
	{
		class Skeleton;
	}
}

enum SupportStateType
{
	SST_LEFT = 0,
	SST_RIGHT = 1,
	SST_DOUBLE = 2,
	SST_DOUBLE2LEFT = 3,
	SST_DOUBLE2RIGHT = 4,
	SST_MAX = 5
};


class SupportState
{
public:
	SupportState();
	virtual ~SupportState();
	SupportStateType GetType() const;
	void SetFrameRange(int start, int end);
	void SetSkeletons(dart::dynamics::Skeleton* origSkel, dart::dynamics::Skeleton* targetSkel);
	virtual void AddConstraint(int frameNum, IKProblem* ik);
protected:
	SupportStateType mType;
	int mStartFrame;
	int mEndFrame;
	dart::dynamics::Skeleton* mOrig;
	dart::dynamics::Skeleton* mTarget;
};
#endif