#include "MocapReader.h"
#include <fstream>
#include <algorithm>
#include "utils/CppCommon.h"
#include <glog/logging.h>
using namespace google;

const int numMotors = 18;


Eigen::VectorXd CMUMocapFrame::GetRobotPose() const
{
	Eigen::VectorXd ret;
	ret = Eigen::VectorXd::Zero(numMotors);
	ret[0] = mDofs[CMU_JointName_rhumerus].mValues[2]; //rShoulder1
	ret[1] = -mDofs[CMU_JointName_lhumerus].mValues[2]; //lShoulder1
	ret[2] = mDofs[CMU_JointName_rhumerus].mValues[1]; //rShoulder2
	ret[3] = -mDofs[CMU_JointName_lhumerus].mValues[1]; //lShoulder2
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
	ret = UTILS_PI / 180 * ret;

	//offset between DART and CMU mocap data
	double collisionAvoidanceOffset = 0.2; // jie hack
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

const CMUMocapFrame& MocapReader::GetFrame(int ithFrame)
{
	if (ithFrame < 0)
		ithFrame = 0;
	else if (ithFrame >= mMotion.size())
		ithFrame = mMotion.size() - 1;
	return mMotion[ithFrame];
}
const CMUMocapFrame& MocapReader::GetFrame(double time)
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
		mMotion.push_back(frame);
	}
	mMotion.erase(mMotion.end() - 1);
	LOG(INFO) << "Read " << mMotion.size() << " frames of mocap data.";
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