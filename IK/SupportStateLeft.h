#ifndef _SUPPORT_STATE_LEFT_H
#define _SUPPORT_STATE_LEFT_H

#include "SupportState.h"
class SupportStateLeft : public SupportState
{
public:
	SupportStateLeft();
	virtual ~SupportStateLeft();
	virtual void AddConstraint(int frameNum, IKProblem* ik);
};
#endif