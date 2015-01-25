#ifndef _MOCAP_READER_H
#define _MOCAP_READER_H

#include <vector>
#include <Eigen/Dense>
#include <string>
using namespace std;

enum CMUMocapJointType
{
	CMU_JointName_root,
	CMU_JointName_lowerback,
	CMU_JointName_upperback,
	CMU_JointName_thorax,
	CMU_JointName_lowerneck,
	CMU_JointName_upperneck,
	CMU_JointName_head,
	CMU_JointName_rclavicle,
	CMU_JointName_rhumerus,
	CMU_JointName_rradius,
	CMU_JointName_rwrist,
	CMU_JointName_rhand,
	CMU_JointName_rfinger,
	CMU_JointName_rthumb,
	CMU_JointName_lclavicle,
	CMU_JointName_lhumerus,
	CMU_JointName_lradius,
	CMU_JointName_lwrist,
	CMU_JointName_lhand,
	CMU_JointName_lfingers,
	CMU_JointName_lthumb,
	CMU_JointName_rfemur,
	CMU_JointName_rtibia,
	CMU_JointName_rfoot,
	CMU_JointName_rtoes,
	CMU_JointName_lfemur,
	CMU_JointName_ltibia,
	CMU_JointName_lfoot,
	CMU_JointName_ltoes,
	CMU_JointName_max
};
class CMUMocapJointDof
{
public:
	string mName;
	vector<double> mValues;
};

class CMUMocapFrame
{
public:
	Eigen::VectorXd GetRobotPose() const;
	vector<CMUMocapJointDof> mDofs;
};
class MocapReader
{
public:
	void Read(const string& filename);
	const CMUMocapFrame& GetFrame(int ithFrame);
	const CMUMocapFrame& GetFrame(double time);

private:
	void readHeader(ifstream& inFile);

	string mFileName;
	vector<CMUMocapFrame> mMotion;
};

#endif