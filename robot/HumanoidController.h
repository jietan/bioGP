/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef ROBOT_HUMANOIDCONTROLLER_H
#define ROBOT_HUMANOIDCONTROLLER_H

#include <vector>
#include <Eigen/Dense>
#include "utils/HppCommon.h"
#include "MocapMotion.h"
#include "MotorMap.h"
#include "CollisionSphere.h"
#include "myUtils/SimFrame.h"

namespace dart {
namespace dynamics {
class BodyNode;
class Skeleton;
class Marker;
}  // namespace dynamics
namespace constraint {
class ConstraintSolver;
}  // namespace constraint
}  // namespace dart

namespace bioloidgp {
namespace robot {
class MotorMap;
struct Motion;
} // namespace robot
} // namespace bioloidgp

namespace bioloidgp {
namespace robot {


class HumanoidController {
public:
    HumanoidController(
        dart::dynamics::Skeleton* _robot,
        dart::constraint::ConstraintSolver* _collisionSolver
        );
    virtual ~HumanoidController();
	void setFreeDofs(const Eigen::VectorXd& q6);
	
    void setMotorMapPose(const Eigen::VectorXd& mtv);
	void setMotorMapPoseRad(const Eigen::VectorXd& mtv);
    void setMotionTargetPose(int index); // Debug Purpose
	const Eigen::VectorXd& getCurrentTargetPose() const;
	void reset();
	void setInitialPose(const Eigen::VectorXd& init);
	void setInitialFirst6Dofs(const Eigen::VectorXd& init6Dofs);
    virtual void update(double _currentTime);
    virtual void printDebugInfo() const;
	Eigen::Vector3d getCOMChangeFromInitial() const;
	Eigen::Vector3d getCOMVelocity();
	const std::vector<std::vector<CollisionSphere> >& getCollisionSpheres() const;
	Eigen::Vector3d getUpDir() const;
	Eigen::Vector3d getLeftDir() const;
	Eigen::Vector3d getForwardDir() const;
	void setMarkers(const vector<dart::dynamics::Marker*> markers);
	const vector<dart::dynamics::Marker*> getMarkers() const;
	Eigen::VectorXd useAnkelStrategy(const Eigen::VectorXd& refPose, double currentTime, bool bSim = false);
	void keepFeetLevel();
    void keyboard(unsigned char _key, int _x, int _y, double _currentTime);

protected:
	Eigen::VectorXd computeTorque(const Eigen::VectorXd& qhat);
    void setJointDamping(double _damping = 80.0);
	void readMovieFile(const string& fileName);
	Eigen::VectorXd computeDesiredVelocity(double time);
protected:
    MEMBER_PTR(dart::dynamics::Skeleton*, robot);
    MEMBER_PTR(dart::constraint::ConstraintSolver*, collisionSolver);
    MEMBER_PTR(MotorMap*, motormap);
    MEMBER_PTR(Motion*, motion);
	MEMBER_PTR(MocapMotion*, mocap);
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
	Eigen::Vector3d mInitialCOM;
	bool mIsCOMInitialized;
	Eigen::VectorXd mPrevPos;
	double mAnkelOffset;
	double mLastControlTime;
	double mLatency;
	std::vector<double> mAnkelOffsetQueue;
	std::vector<std::vector<CollisionSphere> > mCollisionSpheres;
	Eigen::VectorXd mMotor_qHat;
	Eigen::VectorXd mInitFirst6Dofs;
	int mIsHybridDynamics;
	vector<SimFrame> mRecordedFrames;
	vector<dart::dynamics::Marker*> mMarkers;
}; // class Humanoidcontroller

} // namespace robot
} // namespace bioloidgp



#endif // #ifndef ROBOT_HUMANOIDCONTROLLER_H

