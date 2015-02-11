#ifndef _SUPPORT_STATE_DOUBLE2LEFT_H
#define _SUPPORT_STATE_DOUBLE2LEFT_H

#include "SupportState.h"

class SupportStateDouble2Left : public SupportState
{
public:
	SupportStateDouble2Left();
	virtual ~SupportStateDouble2Left();
	virtual void AddConstraint(int frameNum, IKProblem* ik);
	
};
#endif