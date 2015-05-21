#include "SystemIdentificationData.h"
#include <fstream>
#include "HumanoidController.h"
#include "myUtils/ConfigManager.h"

#define MIN_MASSRAITO 0.5
#define MAX_MASSRATIO 1.5
#define MIN_GAINRATIO 0.1
#define MAX_GAINRATIO 2


SystemIdentificationData::SystemIdentificationData() : mbSearchMass(1), mbSearchGain(1)
{

}
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
	inFile >> dim;
	mGainRatio = Eigen::VectorXd::Zero(dim);
	for (int i = 0; i < dim; ++i)
		inFile >> mGainRatio[i];

	DecoConfig::GetSingleton()->GetInt("CMA", "isSearchMass", mbSearchMass);
	DecoConfig::GetSingleton()->GetInt("CMA", "isSearchGain", mbSearchGain);
	mLowerBound.clear();
	mUpperBound.clear();
	if (mbSearchMass)
	{
		int nMassParameters = static_cast<int>(mMassRatio.size());
		for (int i = 0; i < nMassParameters; ++i)
		{
			mLowerBound.push_back(MIN_MASSRAITO);
			mUpperBound.push_back(MAX_MASSRATIO);
		}
	}

	if (mbSearchGain)
	{
		int nGainParameters = static_cast<int>(mGainRatio.size());
		for (int i = 0; i < nGainParameters; ++i)
		{
			mLowerBound.push_back(MIN_GAINRATIO);
			mUpperBound.push_back(MAX_GAINRATIO);
		}
	}
}


int SystemIdentificationData::GetNumParameters() const
{
	int n = 0;
	if (mbSearchMass)
		n += static_cast<int>(mMassRatio.size());
	if (mbSearchGain)
		n += static_cast<int>(mGainRatio.size());
	return n;
}
void SystemIdentificationData::GetParameterLowerBounds(double* lb)
{
	int n = GetNumParameters();
	for (int i = 0; i < n; ++i)
	{
		lb[i] = mLowerBound[i];
	}

}
void SystemIdentificationData::GetParameterUpperBounds(double* ub)
{
	int n = GetNumParameters();
	for (int i = 0; i < n; ++i)
	{
		ub[i] = mUpperBound[i];
	}
}
void SystemIdentificationData::FromParameterSetting(double* param)
{
	int nMassRatios = 0;
	if (mbSearchMass)
	{
		nMassRatios = static_cast<int>(mMassRatio.size());
		for (int i = 0; i < nMassRatios; ++i)
		{
			mMassRatio[i] = param[i] * (mUpperBound[i] - mLowerBound[i]) + mLowerBound[i];
		}
		
	}
	if (mbSearchGain)
	{
		int nGainRatios = static_cast<int>(mGainRatio.size());
		for (int i = 0; i < nGainRatios; ++i)
		{
			mGainRatio[i] = param[i + nMassRatios] * (mUpperBound[i + nMassRatios] - mLowerBound[i + nMassRatios]) + mLowerBound[i + nMassRatios];
		}
	}

}


void SystemIdentificationData::ApplyToController(bioloidgp::robot::HumanoidController* controller)
{
	//controller->setSystemIdData(*this);
	if (mbSearchMass)
	{
		controller->setBodyMassesByRatio(mMassRatio);
		controller->setBodyInertiaByRatio(mMassRatio);
	}
	if (mbSearchGain)
		controller->setActuatorGains(mGainRatio);
}