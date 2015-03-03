#include "ControllerData.h"
#include <fstream>
using namespace std;

void ControllerData::ReadFromFile(const string& filename)
{
	ifstream inFile(filename.c_str());
	int nFrames;
	inFile >> nFrames;
	mKeyFrameId.resize(nFrames);
	mDofValues.resize(nFrames);
	mKeyFrameDuration.resize(nFrames);

	for (int i = 0; i < nFrames; ++i)
	{
		inFile >> mKeyFrameId[i];

		int nDofs;
		inFile >> nDofs;
		mDofValues[i].resize(nDofs);
		for (int j = 0; j < nDofs; ++j)
		{
			inFile >> mDofValues[i][j].mId;
			inFile >> mDofValues[i][j].mValue;
			inFile >> mDofValues[i][j].mMin;
			inFile >> mDofValues[i][j].mMax;
		}
		inFile >> mKeyFrameDuration[i].mKeyFrameDuration >> mKeyFrameDuration[i].mMin >> mKeyFrameDuration[i].mMax;
	}
}

int ControllerData::GetNumParameters() const
{
	int n = 0;
	int nFrames = static_cast<int>(mKeyFrameId.size());
	for (int i = 0; i < nFrames; ++i)
	{
		n += mDofValues[i].size();
		n++;
	}
	return n;
}
void ControllerData::GetParameterLowerBounds(double* lb)
{
	int count = 0;
	int nFrames = static_cast<int>(mKeyFrameId.size());
	for (int i = 0; i < nFrames; ++i)
	{
		int nDofs = static_cast<int>(mDofValues[i].size());
		for (int j = 0; j < nDofs; ++j)
		{
			lb[count++] = mDofValues[i][j].mMin;
		}
		lb[count++] = mKeyFrameDuration[i].mMin;
	}
}
void ControllerData::GetParameterUpperBounds(double* ub)
{
	int count = 0;
	int nFrames = static_cast<int>(mKeyFrameId.size());
	for (int i = 0; i < nFrames; ++i)
	{
		int nDofs = static_cast<int>(mDofValues[i].size());
		for (int j = 0; j < nDofs; ++j)
		{
			ub[count++] = mDofValues[i][j].mMax;
		}
		ub[count++] = mKeyFrameDuration[i].mMax;
	}
}
void ControllerData::FromParameterSetting(double* param)
{
	int count = 0;
	int nFrames = static_cast<int>(mKeyFrameId.size());
	for (int i = 0; i < nFrames; ++i)
	{
		int nDofs = static_cast<int>(mDofValues[i].size());
		for (int j = 0; j < nDofs; ++j)
		{
			mDofValues[i][j].mValue = param[count++];
		}
		mKeyFrameDuration[i].mKeyFrameDuration = param[count++];
	}
}