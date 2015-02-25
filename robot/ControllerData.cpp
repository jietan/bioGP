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
	mKeyFrameDuration.resize(nFrames, -1);

	for (int i = 0; i < nFrames; ++i)
	{
		inFile >> mKeyFrameId[i];

		int nDofs;
		inFile >> nDofs;
		mDofValues[i].resize(nDofs);
		for (int j = 0; j < nDofs; ++j)
		{
			inFile >> mDofValues[i][j].first;
			inFile >> mDofValues[i][j].second;
		}
		inFile >> mKeyFrameDuration[i];
	}
}