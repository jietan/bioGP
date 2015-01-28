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

namespace dart {
namespace dynamics {
class BodyNode;
class Skeleton;
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
	void reset();
	void setInitialPose(const Eigen::VectorXd& init);
    virtual void update(double _currentTime);
    virtual void printDebugInfo() const;
	Eigen::Vector3d getCOMChangeFromInitial() const;
	Eigen::Vector3d getCOMVelocity();
	const std::vector<std::vector<CollisionSphere> >& getCollisionSpheres() const;
	
	Eigen::VectorXd useAnkelStrategy(const Eigen::VectorXd& refPose, double currentTime, bool bSim = false);
	void keepFeetLevel();
    void keyboard(unsigned char _key, int _x, int _y, double _currentTime);

protected:
    void setJointDamping(double _damping = 80.0);

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
	
}; // class Humanoidcontroller

} // namespace robot
} // namespace bioloidgp



#endif // #ifndef ROBOT_HUMANOIDCONTROLLER_H

