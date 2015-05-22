#ifndef _SYSTEM_IDENTIFICATION_DATA
#define _SYSTEM_IDENTIFICATION_DATA

#include "CMAData.h"
#include <Eigen/Dense>
#include <string>
#include <vector>
using namespace std;

class SystemIdentificationData : public CMAData
{
public:
	Eigen::VectorXd mMassRatio;
	Eigen::VectorXd mGainRatio;
	Eigen::VectorXd mCOMOffset;
	vector<double> mLowerBound;
	vector<double> mUpperBound;
	int mbSearchMass;
	int mbSearchGain;
	int mbSearchCOM;

	SystemIdentificationData();
	virtual ~SystemIdentificationData();
	virtual void ReadFromFile(const string& filename);
	virtual int GetNumParameters() const;
	virtual void GetParameterLowerBounds(double* lb);
	virtual void GetParameterUpperBounds(double* ub);
	virtual void FromParameterSetting(double* param);
	virtual void ApplyToController(bioloidgp::robot::HumanoidController* controller);
};

#endif