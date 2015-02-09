#ifndef _SUPPORT_STATE_RIGHT_H
#define _SUPPORT_STATE_RIGHT_H

#include "SupportState.h"

class SupportStateRight : public SupportState
{
public:
	SupportStateRight();
	virtual ~SupportStateRight();
	virtual void AddConstraint(int frameNum, IKProblem* ik);
};
#endif