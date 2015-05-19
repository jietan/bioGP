#include "SystemIdentificationData.h"
#include <fstream>
#include "HumanoidController.h"

#define MIN_VALUE 0.9
#define MAX_VALUE 1.5

SystemIdentificationData::~SystemIdentificationData()
{

}
void SystemIdentificationData::ReadFromFile(const string& filename)
{
	ifstream inFile(filename.c_str());
	int dim;
	inFile >> dim;
	mMassRatio = Eigen::VectorXd::Zero(dim);
	for (int i = 0; i < dim; ++i)
		inFile >> mMassRatio[i];
}


int SystemIdentificationData::GetNumParameters() const
{
	return static_cast<int>(mMassRatio.size());
}
void SystemIdentificationData::GetParameterLowerBounds(double* lb)
{
	int nParameters = GetNumParameters();
	for (int i = 0; i < nParameters; ++i)
		lb[i] = MIN_VALUE;
}
void SystemIdentificationData::GetParameterUpperBounds(double* ub)
{
	int nParameters = GetNumParameters();
	for (int i = 0; i < nParameters; ++i)
		ub[i] = MAX_VALUE;
}
void SystemIdentificationData::FromParameterSetting(double* param)
{
	int nParameters = GetNumParameters();
	for (int i = 0; i < nParameters; ++i)
	{
		mMassRatio[i] = param[i] * (MAX_VALUE - MIN_VALUE) + MIN_VALUE;
	}
}


void SystemIdentificationData::ApplyToController(bioloidgp::robot::HumanoidController* controller)
{
	controller->setSystemIdData(*this);
}