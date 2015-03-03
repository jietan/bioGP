#ifndef _CONTROLLER_DATA_H
#define _CONTROLLER_DATA_H
#include <vector>
using namespace std;

struct ControllerDof
{
	int mId;
	int mValue;
	int mMin;
	int mMax;
};

struct ControllerDuration
{
	double mKeyFrameDuration;
	double mMin;
	double mMax;
};
class ControllerData
{
public:
	vector<int> mKeyFrameId;
	vector<vector<ControllerDof> >  mDofValues;
	vector<ControllerDuration> mKeyFrameDuration;

	void ReadFromFile(const string& filename);
	int GetNumParameters() const;
	void GetParameterLowerBounds(double* lb);
	void GetParameterUpperBounds(double* ub);
	void FromParameterSetting(double* param);
};

#endif