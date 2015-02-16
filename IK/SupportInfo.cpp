#include "SupportInfo.h"
#include <fstream>
#include "myUtils/mathlib.h"
#include "SupportStateLeft.h"
#include "SupportStateRight.h"
#include "SupportStateDouble2Right.h"
#include "SupportStateDouble.h"
#include "SupportStateDouble2Left.h"

SupportInfo::SupportInfo() : mOrig(NULL), mTarget(NULL)
{
}
SupportInfo::SupportInfo(const string& fileName) : mOrig(NULL), mTarget(NULL)
{
	loadFromFile(fileName);
}


SupportInfo::~SupportInfo()
{
	int nStates = static_cast<int>(mStates.size());
	for (int i = 0; i < nStates; ++i)
	{
		if (mStates[i])
			delete mStates[i];
	}
	mStates.clear();
}

void SupportInfo::SetLeftGlobal(const Eigen::Vector3d& left)
{
	int nStates = static_cast<int>(mStates.size());
	for (int i = 0; i < nStates; ++i)
	{
		mStates[i]->SetLeftGlobal(left);
	}

}
void SupportInfo::SetSkeletons(dart::dynamics::Skeleton* origSkel, dart::dynamics::Skeleton* targetSkel)
{
	mOrig = origSkel;
	mTarget = targetSkel;
	int nStates = static_cast<int>(mStates.size());
	for (int i = 0; i < nStates; ++i)
	{
		mStates[i]->SetSkeletons(mOrig, mTarget);
	}
}

SupportStateType SupportInfo::GetSupportType(int frameNumber)
{
	return getSupportState(frameNumber)->GetType();
}

SupportState* SupportInfo::getSupportState(int frameNumber)
{
	frameNumber = Clamp<int>(frameNumber, 0, mNumFrames - 1);
	int nStates = static_cast<int>(mStates.size());
	int i = 0;
	for (i = 0; i < nStates; ++i)
	{
		if (frameNumber >= mStartFrames[i] && frameNumber < mStartFrames[i + 1])
			return mStates[i];
	}
	LOG(FATAL) << frameNumber << " cannot be found in supportInfo.";
	return NULL;
}
void SupportInfo::AddConstraints(int frameNumber, IKProblem* ik)
{
	SupportState* ss = getSupportState(frameNumber);

	ss->AddConstraint(frameNumber, ik);
}

void SupportInfo::loadFromFile(const string& fileName)
{
	std::ifstream inFile(fileName.c_str());
	int nStates;
	inFile >> nStates >> mNumFrames;
	
	int startFrame, stateType;
	mStates.resize(nStates);
	mStartFrames.resize(nStates + 1);
	for (int i = 0; i < nStates; ++i)
	{
		inFile >> startFrame >> stateType;
		mStartFrames[i] = startFrame;
		switch (stateType)
		{
		case SST_LEFT:
			mStates[i] = new SupportStateLeft();
			break;
		case SST_RIGHT:
			mStates[i] = new SupportStateRight();
			break;
		case SST_DOUBLE:
			mStates[i] = new SupportStateDouble();
			break;
		case SST_DOUBLE2LEFT:
			mStates[i] = new SupportStateDouble2Left();
			break;
		case SST_DOUBLE2RIGHT:
			mStates[i] = new SupportStateDouble2Right();
			break;
		default:
			LOG(FATAL) << "Not supported support state.";
		}
	}
	mStartFrames[nStates] = mNumFrames;
	for (int i = 0; i < nStates; ++i)
	{
		mStates[i]->SetFrameRange(mStartFrames[i], mStartFrames[i + 1]);
		mStates[i]->SetSkeletons(mOrig, mTarget);
		if (i < nStates - 1)
			mStates[i]->mNext = mStates[i + 1];
		else
			mStates[i]->mNext = NULL;
	}
}

