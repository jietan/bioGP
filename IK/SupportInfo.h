#ifndef _SUPPORT_INFO
#define _SUPPORT_INFO

#include "SupportState.h"
#include "IKProblem.h"
#include "dart/dynamics/Skeleton.h"
#include <vector>
#include <glog/logging.h>
using namespace google;


class SupportInfo
{
public:
	SupportInfo();
	SupportInfo(const string& fileName);
	~SupportInfo();
	void SetSkeletons(dart::dynamics::Skeleton* origSkel, dart::dynamics::Skeleton* targetSkel);
	void AddConstraints(int frameNumber, IKProblem* ik);
	void SetLeftGlobal(const Eigen::Vector3d& left);
	SupportStateType GetSupportType(int frameNumber);
private:
	void loadFromFile(const string& fileName);
	SupportState* getSupportState(int frameNumber);

	vector<SupportState*> mStates;
	vector<int> mStartFrames;
	int mNumFrames;
	dart::dynamics::Skeleton* mOrig;
	dart::dynamics::Skeleton* mTarget;
};

#endif