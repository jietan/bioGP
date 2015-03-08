#ifndef _SIM_FRAME_H
#define _SIM_FRAME_H


#include <Eigen/Dense>
#include <iostream>
#include <glog/logging.h>
using namespace google;
using namespace std;

class SimFrame
{
public:
	double mTime;
	Eigen::VectorXd mPose;	
	SimFrame()
	{}
	SimFrame(int nDofs)
	{
		mPose = Eigen::VectorXd::Zero(nDofs);
	}
};

istream& operator>> (istream& in, SimFrame& rhs);
ostream& operator<< (ostream& out, const SimFrame& rhs);

#endif