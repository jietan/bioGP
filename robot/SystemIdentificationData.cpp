#include "SystemIdentificationData.h"
#include <fstream>
#include "HumanoidController.h"
#include "myUtils/ConfigManager.h"

#define MIN_MASSRAITO 0.9
#define MAX_MASSRATIO 1.5
#define MIN_GAINRATIO 0.1
#define MAX_GAINRATIO 2
#define MIN_COMRATIO -0.2
#define MAX_COMRATIO 0.2


SystemIdentificationData::SystemIdentificationData() : mbSearchMass(1), mbSearchGain(1), mbSearchCOM(1)
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
	inFile >> dim;
	mCOMOffsetRatio = Eigen::VectorXd::Zero(dim);
	for (int i = 0; i < dim; ++i)
	{
		inFile >> mCOMOffsetRatio[i];
	}

	DecoConfig::GetSingleton()->GetInt("CMA", "isSearchMass", mbSearchMass);
	DecoConfig::GetSingleton()->GetInt("CMA", "isSearchGain", mbSearchGain);
	DecoConfig::GetSingleton()->GetInt("CMA", "isSearchCOM", mbSearchCOM);
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

	if (mbSearchCOM)
	{
		int nCOMParameters = static_cast<int>(mCOMOffsetRatio.size());
		for (int i = 0; i < nCOMParameters; ++i)
		{
			mLowerBound.push_back(MIN_COMRATIO);
			mUpperBound.push_back(MAX_COMRATIO);
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
	if (mbSearchCOM)
		n += static_cast<int>(mCOMOffsetRatio.size());
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
	int nGainRatios = 0;
	int nCOM = 0;
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
		nGainRatios = static_cast<int>(mGainRatio.size());
		for (int i = 0; i < nGainRatios; ++i)
		{
			mGainRatio[i] = param[i + nMassRatios] * (mUpperBound[i + nMassRatios] - mLowerBound[i + nMassRatios]) + mLowerBound[i + nMassRatios];
		}
	}
	if (mbSearchCOM)
	{
		nCOM = static_cast<int>(mCOMOffsetRatio.size());
		for (int i = 0; i < nCOM; ++i)
		{
			mCOMOffsetRatio[i] = param[i + nMassRatios + nGainRatios] * (mUpperBound[i + nMassRatios + nGainRatios] - mLowerBound[i + nMassRatios + nGainRatios]) + mLowerBound[i + nMassRatios + nGainRatios];
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
	if (mbSearchCOM)
		controller->setCenterOfMassOffsetByRatio(mCOMOffsetRatio);
}