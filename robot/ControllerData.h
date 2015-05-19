#ifndef _CONTROLLER_DATA_H
#define _CONTROLLER_DATA_H
#include "CMAData.h"
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
class ControllerData : public CMAData
{
public:
	vector<int> mKeyFrameId;
	vector<vector<ControllerDof> >  mDofValues;
	vector<ControllerDuration> mKeyFrameDuration;

	virtual ~ControllerData();
	virtual void ReadFromFile(const string& filename);
	virtual int GetNumParameters() const;
	virtual void GetParameterLowerBounds(double* lb);
	virtual void GetParameterUpperBounds(double* ub);
	virtual void FromParameterSetting(double* param);
	virtual void ApplyToController(bioloidgp::robot::HumanoidController* controller);
};

#endif