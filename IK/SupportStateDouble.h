#ifndef _SUPPORT_STATE_DOUBLE_H
#define _SUPPORT_STATE_DOUBLE_H

#include "SupportState.h"
#include <Eigen/Dense>

class SupportStateDouble : public SupportState
{
public:
	SupportStateDouble();
	virtual ~SupportStateDouble();
	virtual void AddConstraint(int frameNum, IKProblem* ik);


	
};
#endif