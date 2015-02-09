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
	CMU_JointName_lfinger,
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

enum DARTJointSequence
{
	DART_JointName_root,
	DART_JointName_lowerback,
	DART_JointName_lfemur,
	DART_JointName_rfemur,
	DART_JointName_upperback,
	DART_JointName_ltibia,
	DART_JointName_rtibia,
	DART_JointName_thorax,
	DART_JointName_lfoot,
	DART_JointName_rfoot,
	DART_JointName_lowerneck,
	DART_JointName_lclavicle,
	DART_JointName_rclavicle,
	DART_JointName_ltoes,
	DART_JointName_rtoes,
	DART_JointName_upperneck,
	DART_JointName_lhumerus,
	DART_JointName_rhumerus,
	DART_JointName_head,
	DART_JointName_lradius,
	DART_JointName_rradius,
	DART_JointName_lwrist,
	DART_JointName_rwrist,
	DART_JointName_lhand,
	DART_JointName_rhand,
	DART_JointName_lfinger,
	DART_JointName_rfinger,
	DART_JointName_lthumb,
	DART_JointName_rthumb,
	DART_JointName_max
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
	Eigen::VectorXd GetCharacterPose() const;
	vector<CMUMocapJointDof> mDofs;
	static int msJointMapping[62];
};
class MocapReader
{
public:
	void Read(const string& filename);
	void Save(const string& filename);
	void SaveMotionAfterIK(const string& name);
	const CMUMocapFrame& GetFrame(int ithFrame) const;
	const CMUMocapFrame& GetFrame(double time) const;
	void SetFrameAfterIK(int ithFrame, const Eigen::VectorXd& pose);
private:
	void readHeader(ifstream& inFile);
	void buildJointMapping();

	string mFileName;
	vector<CMUMocapFrame> mMotion;
	vector<Eigen::VectorXd> mMotionAfterIK;

};

#endif