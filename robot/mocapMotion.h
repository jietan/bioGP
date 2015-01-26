#ifndef _MOCAP_MOTION_H
#define _MOCAP_MOTION_H

#include <Eigen/Dense>
#include <string>
#include <vector>
using namespace std;

class MocapMotion
{
public:
	void Read(const string& filename);
	const Eigen::VectorXd& GetPose(double time) const;
	const Eigen::VectorXd& GetPose(int ithFrame) const;
private:
	vector<Eigen::VectorXd> mMotion;
};

#endif