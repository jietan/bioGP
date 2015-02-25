#ifndef _CONTROLLER_DATA_H
#define _CONTROLLER_DATA_H
#include <vector>
using namespace std;

class ControllerData
{
public:
	vector<int> mKeyFrameId;
	vector<vector<pair<int, int> > >  mDofValues;
	vector<double> mKeyFrameDuration;

	void ReadFromFile(const string& filename);
};

#endif