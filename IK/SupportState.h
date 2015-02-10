#ifndef _SUPPORT_STATE_H
#define _SUPPORT_STATE_H

#include <Eigen/Dense>
#include "dart/dynamics/Skeleton.h"
#include "IKProblem.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Marker.h"
#include "IKProblem.h"
#include "PositionConstraint.h"
#include "COMConstraint.h"
class IKProblem;
namespace dart
{
	namespace dynamics
	{
		class Skeleton;
	}
}

enum SupportStateType
{
	SST_LEFT = 0,
	SST_RIGHT = 1,
	SST_DOUBLE = 2,
	SST_DOUBLE2LEFT = 3,
	SST_DOUBLE2RIGHT = 4,
	SST_MAX = 5
};


class SupportState
{
public:
	SupportState();
	virtual ~SupportState();
	SupportStateType GetType() const;
	void SetFrameRange(int start, int end);
	void SetSkeletons(dart::dynamics::Skeleton* origSkel, dart::dynamics::Skeleton* targetSkel);
	void SetLeftGlobal(const Eigen::Vector3d& left);
	virtual void AddConstraint(int frameNum, IKProblem* ik);
protected:
	void addDoubleFootConstraint(int frameNum, const Eigen::Vector3d& leftFootConstraint, const Eigen::Vector3d& rightFootConstraint, IKProblem* ik);
	void addLeftFootConstraint(int frameNum, const Eigen::Vector3d& footConstraint, bool bUseLeftDir, IKProblem* ik);
	void addRightFootConstraint(int frameNum, const Eigen::Vector3d& footConstraint, bool bUseLeftDir, IKProblem* ik);
	void addLeftFootObjective(int frameNum, const Eigen::Vector3d& footConstraint, bool bUseLeftDir, IKProblem* ik);
	void addRightFootObjective(int frameNum, const Eigen::Vector3d& footConstraint, bool bUseLeftDir, IKProblem* ik);
	void addCOMObjective(int frameNum, const Eigen::Vector3d& comTarget, IKProblem* ik);
	void snapshotInitialFootLocations(int frameNum);

	SupportStateType mType;
	int mStartFrame;
	int mEndFrame;
	dart::dynamics::Skeleton* mOrig;
	dart::dynamics::Skeleton* mTarget;
	Eigen::Vector3d mLeftGlobal;
	Eigen::Vector3d mInitialLeftFoot;
	Eigen::Vector3d mInitialRightFoot;


};
#endif