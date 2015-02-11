#include "MocapReader.h"
#include <fstream>
#include <algorithm>
#include "utils/CppCommon.h"
#include <glog/logging.h>
using namespace google;
#include "dart/math/Geometry.h"
#include "myUtils/ConfigManager.h"

const int numMotors = 18;

int CMUMocapFrame::msJointMapping[] = 
{
	CMU_JointName_root,
	CMU_JointName_lowerback,
	CMU_JointName_lfemur,
	CMU_JointName_rfemur,
	CMU_JointName_upperback,
	CMU_JointName_ltibia,
	CMU_JointName_rtibia,
	CMU_JointName_thorax,
	CMU_JointName_lfoot,
	CMU_JointName_rfoot,
	CMU_JointName_lowerneck,
	CMU_JointName_lclavicle,
	CMU_JointName_rclavicle,
	CMU_JointName_ltoes,
	CMU_JointName_rtoes,
	CMU_JointName_upperneck,
	CMU_JointName_lhumerus,
	CMU_JointName_rhumerus,
	CMU_JointName_head,
	CMU_JointName_lradius,
	CMU_JointName_rradius,
	CMU_JointName_lwrist,
	CMU_JointName_rwrist,
	CMU_JointName_lhand,
	CMU_JointName_rhand,
	CMU_JointName_lfinger,
	CMU_JointName_rfinger,
	CMU_JointName_lthumb,
	CMU_JointName_rthumb
};

Eigen::VectorXd CMUMocapFrame::GetCharacterPose() const
{
	Eigen::VectorXd ret;
	int count = 0;
	ret = Eigen::VectorXd::Zero(62);
	for (int i = 0; i < DART_JointName_max; ++i)
	{
		int jointIdx = msJointMapping[i];
		int nValues = static_cast<int>(mDofs[jointIdx].mValues.size());
		for (int j = nValues - 1; j >= 0; --j)
			ret[count++] = mDofs[jointIdx].mValues[j];
	}

	ret.tail(56) = UTILS_PI / 180 * ret.tail(56);
	//ret.head(6) = Eigen::VectorXd::Zero(6);

	Eigen::Matrix3d rotRoot;
	rotRoot = Eigen::AngleAxisd(ret[0] / 180.0 * M_PI, Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(ret[1] / 180.0 * M_PI, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(ret[2] / 180.0 * M_PI, Eigen::Vector3d::UnitX());
	
	ret.head(3) = dart::math::logMap(rotRoot);
	double mocapScale = 0.06;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "Scale", mocapScale);
	ret.segment(3, 3) = mocapScale * Eigen::Vector3d(ret[5], ret[4], ret[3]);
	
	return ret;
}

Eigen::VectorXd CMUMocapFrame::GetRobotPose() const
{
	Eigen::VectorXd ret;
	ret = Eigen::VectorXd::Zero(numMotors);

	Eigen::Matrix3d rHumerusRot;
	double rx = mDofs[CMU_JointName_rhumerus].mValues[0] * UTILS_PI / 180;
	double ry = mDofs[CMU_JointName_rhumerus].mValues[1] * UTILS_PI / 180;
	double rz = mDofs[CMU_JointName_rhumerus].mValues[2] * UTILS_PI / 180;
	rHumerusRot = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
	Eigen::Vector3d rEulerAngle = rHumerusRot.eulerAngles(1, 2, 0) ;

	Eigen::Matrix3d lHumerusRot;
	rx = mDofs[CMU_JointName_lhumerus].mValues[0] * UTILS_PI / 180;
	ry = mDofs[CMU_JointName_lhumerus].mValues[1] * UTILS_PI / 180;
	rz = mDofs[CMU_JointName_lhumerus].mValues[2] * UTILS_PI / 180;
	rHumerusRot = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(ry, -Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(rx, -Eigen::Vector3d::UnitX());
	Eigen::Vector3d lEulerAngle = rHumerusRot.eulerAngles(1, 2, 0);
	//ret[0] = -rEulerAngle[0];
	//ret[1] = -lEulerAngle[0];
	//ret[2] = rEulerAngle[1]; //rShoulder2
	//ret[3] = lEulerAngle[1];
	//ret[0] = mDofs[CMU_JointName_rhumerus].mValues[2]; //rShoulder1
	//ret[1] = -mDofs[CMU_JointName_lhumerus].mValues[2]; //lShoulder1
	//ret[2] = mDofs[CMU_JointName_rhumerus].mValues[1]; //rShoulder2
	//ret[3] = -mDofs[CMU_JointName_lhumerus].mValues[1]; //lShoulder2
	ret[0] -= UTILS_PI / 6;
	ret[1] -= UTILS_PI / 6;
	ret[2] -= UTILS_PI / 2 - 0.2;
	ret[3] -= -UTILS_PI / 2 + 0.2;
	ret[4] = mDofs[CMU_JointName_rradius].mValues[0]; //rElbow
	ret[5] = mDofs[CMU_JointName_lradius].mValues[0]; //lElbow
	ret[8] = -mDofs[CMU_JointName_rfemur].mValues[2]; //rHip1
	ret[9] = -mDofs[CMU_JointName_lfemur].mValues[2]; //lHip1
	ret[10] = -mDofs[CMU_JointName_rfemur].mValues[0]; //rHip2
	ret[11] = -mDofs[CMU_JointName_lfemur].mValues[0]; //lHip2
	ret[12] = -mDofs[CMU_JointName_rtibia].mValues[0]; //rKnee
	ret[13] = -mDofs[CMU_JointName_ltibia].mValues[0]; //lKnee	
	ret[14] = -mDofs[CMU_JointName_rfoot].mValues[0]; //rAnkel1
	ret[15] = -mDofs[CMU_JointName_lfoot].mValues[0]; //lAnkel1
	ret[16] = -mDofs[CMU_JointName_rfoot].mValues[1]; //rAnkel2
	ret[17] = -mDofs[CMU_JointName_lfoot].mValues[1]; //lAnkel2
	ret.tail(numMotors - 4) = UTILS_PI / 180 * ret.tail(numMotors - 4);
	//ret.tail(numMotors) = UTILS_PI / 180 * ret.tail(numMotors);

	//offset between DART and CMU mocap data
	double collisionAvoidanceOffset = -0.0; // jie hack
	double hip1Offset = atan2(0.34202, 0.939693);
	ret[0] += UTILS_PI / 6;
	ret[1] += UTILS_PI / 6;
	ret[2] += UTILS_PI / 2;
	ret[3] += -UTILS_PI / 2;
	ret[4] += -UTILS_PI / 2;
	ret[5] += -UTILS_PI / 2;
	ret[8] += hip1Offset + collisionAvoidanceOffset;
	ret[9] += -hip1Offset;
	ret[10] += -0.5;
	ret[11] += -0.5;
	ret[12] += 1;
	ret[13] += 1;
	ret[14] += -0.5;
	ret[15] += -0.5;
	return ret;
}

const CMUMocapFrame& MocapReader::GetFrame(int ithFrame) const
{
	if (ithFrame < 0)
		ithFrame = 0;
	else if (ithFrame >= mMotion.size())
		ithFrame = mMotion.size() - 1;
	return mMotion[ithFrame];
}
const CMUMocapFrame& MocapReader::GetFrame(double time) const
{
	int ithFrame = static_cast<int>(time * 120);
	return GetFrame(ithFrame);
}

void MocapReader::Read(const string& filename)
{
	mFileName = filename;
	ifstream inFile(filename.c_str());
	readHeader(inFile);
	while (inFile.good())
	{
		string line;
		getline(inFile, line);
		CMUMocapFrame frame;
		frame.mDofs.resize(CMU_JointName_max);
		for (int ithJoint = 0; ithJoint < CMU_JointName_max; ++ithJoint)
		{
			getline(inFile, line);
			stringstream sstream(line.c_str());
			double value = 0;

			sstream >> frame.mDofs[ithJoint].mName;
			while (sstream.good() && line != "")
			{
				sstream >> skipws >> value;
				frame.mDofs[ithJoint].mValues.push_back(value);
			}
		}
		// jie hack to make to feet separate
		if (!frame.mDofs[CMU_JointName_ltibia].mValues.empty())
		{
			frame.mDofs[CMU_JointName_ltibia].mValues[0] += 40;
			frame.mDofs[CMU_JointName_rtibia].mValues[0] += 40;
			//frame.mDofs[CMU_JointName_lfemur].mValues[0] -= 10;
			//frame.mDofs[CMU_JointName_rfemur].mValues[0] -= 10;
			frame.mDofs[CMU_JointName_lfemur].mValues[2] += 11;
			frame.mDofs[CMU_JointName_rfemur].mValues[2] -= 11;
		}


		mMotion.push_back(frame);
	}
	mMotion.erase(mMotion.end() - 1);

	int numFrames = static_cast<int>(mMotion.size());
	mMotionAfterIK.resize(numFrames);
	for (int i = 0; i < numFrames; ++i)
	{
		mMotionAfterIK[i] = GetFrame(i).GetRobotPose();
	}
	LOG(INFO) << "Read " << mMotion.size() << " frames of mocap data.";
}


void MocapReader::SaveMotionAfterIK(const string& filename)
{
	ofstream outFile(filename.c_str());
	int numFrames = static_cast<int>(mMotionAfterIK.size());
	outFile << numFrames << endl;

	for (int i = 0; i < numFrames; ++i)
	{
		int numDofs = static_cast<int>(mMotion[i].mDofs.size());
		for (int j = 0; j < numMotors; ++j)
			outFile << mMotionAfterIK[i][j] << " ";
		outFile << endl;
	}
}

void MocapReader::Save(const string& filename)
{
	ofstream outFile(filename.c_str());
	int numFrames = static_cast<int>(mMotion.size());
	outFile << numFrames << endl;
	
	for (int i = 0; i < numFrames; ++i)
	{
		int numDofs = static_cast<int>(mMotion[i].mDofs.size());
		for (int j = 0; j < numMotors; ++j)
			outFile << mMotion[i].GetRobotPose()[j] << " ";
		outFile << endl;
	}
		
}

void MocapReader::readHeader(ifstream& inFile)
{
	string line;
	getline(inFile, line);
	getline(inFile, line);
	getline(inFile, line);
}


void MocapReader::SetFrameAfterIK(int ithFrame, const Eigen::VectorXd& pose)
{
	int numFrames = static_cast<int>(mMotion.size());
	if (ithFrame >= numFrames)
		return;
	mMotionAfterIK[ithFrame] = pose;
}
