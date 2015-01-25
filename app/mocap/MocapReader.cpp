#include "MocapReader.h"
#include <fstream>
#include <algorithm>
#include <glog/logging.h>
using namespace google;

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

void MocapReader::readHeader(ifstream& inFile)
{
	string line;
	getline(inFile, line);
	getline(inFile, line);
	getline(inFile, line);
}