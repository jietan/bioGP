#ifndef _SYSTEM_IDENTIFICATION_DATA
#define _SYSTEM_IDENTIFICATION_DATA

#include "CMAData.h"
#include <Eigen/Dense>
#include <string>
using namespace std;

class SystemIdentificationData : public CMAData
{
public:
	Eigen::VectorXd mMassRatio;

	virtual ~SystemIdentificationData();
	virtual void ReadFromFile(const string& filename);
	virtual int GetNumParameters() const;
	virtual void GetParameterLowerBounds(double* lb);
	virtual void GetParameterUpperBounds(double* ub);
	virtual void FromParameterSetting(double* param);
	virtual void ApplyToController(bioloidgp::robot::HumanoidController* controller);
};

#endif