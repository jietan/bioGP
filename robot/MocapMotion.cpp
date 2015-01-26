#include "mocapMotion.h"
#include <fstream>

const int numMotors = 18;

void MocapMotion::Read(const string& filename)
{
	ifstream inFile(filename.c_str());
	int numFrames;
	inFile >> numFrames;
	mMotion.resize(numFrames);
	for (int i = 0; i < numFrames; ++i)
	{
		mMotion[i] = Eigen::VectorXd::Zero(numMotors);
		for (int j = 0; j < numMotors; ++j)
		{
			inFile >> mMotion[i][j];
		}
	}
}
const Eigen::VectorXd& MocapMotion::GetPose(double time) const
{
	int ithFrame = static_cast<int>(time * 120);
	if (ithFrame < 0) ithFrame = 0;
	if (ithFrame >= mMotion.size()) ithFrame = mMotion.size() - 1;
	return GetPose(ithFrame);
}
const Eigen::VectorXd& MocapMotion::GetPose(int ithFrame) const
{
	return mMotion[ithFrame];
}
