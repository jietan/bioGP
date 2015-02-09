#ifndef _SUPPORT_STATE_DOUBLE2RIGHT_H
#define _SUPPORT_STATE_DOUBLE2RIGHT_H

#include "SupportState.h"

class SupportStateDouble2Right : public SupportState
{
public:
	SupportStateDouble2Right();
	virtual ~SupportStateDouble2Right();
	
	virtual void AddConstraint(int frameNum, IKProblem* ik);
};
#endif