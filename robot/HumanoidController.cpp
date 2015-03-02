/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "HumanoidController.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "utils/CppCommon.h"

#include "Motion.h"
#include "myUtils/ConfigManager.h"

namespace bioloidgp {
namespace robot {
////////////////////////////////////////////////////////////
// class HumanoidController implementation
HumanoidController::HumanoidController(
    dart::dynamics::Skeleton* _robot,
    dart::constraint::ConstraintSolver* _collisionSolver
    )
    : MEMBER_INIT(robot, _robot)
    , MEMBER_INIT(collisionSolver, _collisionSolver)

{
    const int NDOFS = robot()->getNumDofs();
    const int NMOTORS = 18;

    //setJointDamping(0.15);

	setJointDamping(0.0);
    set_motormap( new MotorMap(NMOTORS, NDOFS) );
    motormap()->load(DATA_DIR"/urdf/BioloidGP/BioloidGPMotorMap.xml");
	set_mocap(new MocapMotion());
	mInitFirst6Dofs = Eigen::VectorXd::Zero(6);
	//string mocapFileName;
	//DecoConfig::GetSingleton()->GetString("Mocap", "FileName", mocapFileName);
	//mocap()->Read(mocapFileName.c_str());

	//Eigen::VectorXd mtvInitPose = Eigen::VectorXd::Ones(NMOTORS) * 512;
	////mtvInitPose << 512, 512, 512, 512, 200, 824, 512, 512, 512, 512, 200, 512, 512, 512, 512, 200, 512, 512; //for motorTest
	////mtvInitPose <<
	////    342, 681, 572, 451, 762, 261,
	////    358, 666,
	////    515, 508, 741, 282, 857, 166, 684, 339, 515, 508;
	//vector<int> initialPose;
	//if (DecoConfig::GetSingleton()->GetIntVector("Sim", "InitialPose", initialPose))
	//{
	//	for (int i = 0; i < NMOTORS; ++i)
	//	{
	//		mtvInitPose[i] = static_cast<double>(initialPose[i]);
	//	}
	//}
	//

 //   set_motion( new Motion(NMOTORS, mtvInitPose) );
	//setInitialPose(mtvInitPose);

	//string motionFileName;
	//string motionPageName;
	//DecoConfig::GetSingleton()->GetString("Ctrl", "MotionFileName", motionFileName);
	//DecoConfig::GetSingleton()->GetString("Ctrl", "MotionPageName", motionPageName);
	//motion()->loadMTN(motionFileName.c_str(), motionPageName.c_str());
 //   motion()->printSteps();


    mKp = Eigen::VectorXd::Zero(NDOFS);
    mKd = Eigen::VectorXd::Zero(NDOFS);
    for (int i = 6; i < NDOFS; ++i) {
		mKp(i) = 9.272;
		mKd(i) = 0.3069;
    }
	int numBodies = static_cast<int>(robot()->getNumBodyNodes());
    for (int i = 0; i < numBodies; i++) {
        LOG(INFO) << "Joint " << i + 5 << " : name = " << robot()->getJoint(i)->getName();
    }
	//mCollisionSpheres.resize(numBodies);
	//for (int i = 0; i < numBodies; ++i)
	//{
	//	dart::dynamics::BodyNode* body = robot()->getBodyNode(i);
	//	if (body->getName() == "r_foot")
	//	{
	//		mCollisionSpheres[i].push_back(CollisionSphere(Eigen::Vector3d(-0.00, 0.0276, -0.02042), 0.025));
	//		mCollisionSpheres[i].push_back(CollisionSphere(Eigen::Vector3d(-0.035, 0.0276, -0.02042), 0.025));
	//		mCollisionSpheres[i].push_back(CollisionSphere(Eigen::Vector3d(0.035, 0.0276, -0.02042), 0.025));
	//	}
	//	else if (body->getName() == "l_thigh")
	//	{
	//		mCollisionSpheres[i].push_back(CollisionSphere(Eigen::Vector3d(-0.001, -0.0637, -0.0115), 0.03));
	//	}
	//	else if (body->getName() == "l_shin")
	//	{
	//		mCollisionSpheres[i].push_back(CollisionSphere(Eigen::Vector3d(-0.0127, 0.0341, 0.0156), 0.03));
	//	}

	//}
	mIsCOMInitialized = false;
	mAnkelOffset = 0;
	mLastControlTime = 0;
	mLatency = 0;
	//DecoConfig::GetSingleton()->GetDouble("Sim", "Latency", mLatency);
	//robot()->getBodyNode("l_foot")->setRestitutionCoeff(1.0);
	//robot()->getBodyNode("r_foot")->setRestitutionCoeff(1.0);
	//robot()->getBodyNode("l_foot")->setFrictionCoeff(0.5);
	//robot()->getBodyNode("r_foot")->setFrictionCoeff(0.5);
}

HumanoidController::~HumanoidController() {
}


void HumanoidController::reset()
{
	setInitialPose(motion()->getInitialPose());
	setFreeDofs(mInitFirst6Dofs);
	mIsCOMInitialized = false;
	mAnkelOffset = 0;
	mLastControlTime = 0;

	double timeStep = robot()->getTimeStep();
	int numLatencySteps = static_cast<int>(mLatency / timeStep);
	mAnkelOffsetQueue.clear();
	mAnkelOffsetQueue.insert(mAnkelOffsetQueue.end(), numLatencySteps, 0);
}
void HumanoidController::setFreeDofs(const Eigen::VectorXd& q6)
{
	Eigen::VectorXd q = robot()->getPositions();
	q.head(6) = q6;
	robot()->setPositions(q);
	robot()->computeForwardKinematics(true, true, false);
	mInitialCOM = robot()->getWorldCOM();
}

void HumanoidController::setInitialFirst6Dofs(const Eigen::VectorXd& init6Dofs)
{
	mInitFirst6Dofs = init6Dofs;
	setFreeDofs(mInitFirst6Dofs);
}
void HumanoidController::setInitialPose(const Eigen::VectorXd& init)
{
	const int NMOTORS = 18;
	
	//mtvInitPose <<
	//    342, 681, 572, 451, 762, 261,
	//    358, 666,
	//    515, 508, 741, 282, 857, 166, 684, 339, 515, 508;
	setMotorMapPose(init);
	motion()->setInitialPose(init);

	Eigen::VectorXd q = robot()->getPositions();
	Eigen::VectorXd noise = 0.0 * Eigen::VectorXd::Random(q.size());
	noise.head<6>().setZero();
	robot()->setPositions(q + noise);
	mPrevPos = q + noise;
	robot()->setVelocities(Eigen::VectorXd::Zero(q.size()));
	// Update the dynamics
	robot()->computeForwardKinematics(true, true, false);
}

void HumanoidController::setMotorMapPose(const Eigen::VectorXd& mtv) {
	int n = robot()->getNumDofs();
    Eigen::VectorXd q = robot()->getPositions();
    Eigen::VectorXd pose = motormap()->fromMotorMapVector( mtv );
    CHECK_EQ(q.size(), pose.size());
    q.tail(n - 6) = pose.tail(n - 6);
    //LOG(INFO) << "q = " << q.transpose();

    robot()->setPositions(q);
    robot()->computeForwardKinematics(true, true, false);
    
}

void HumanoidController::setMotorMapPoseRad(const Eigen::VectorXd& mtv) 
{
	int n = robot()->getNumDofs();
	Eigen::VectorXd q = robot()->getPositions();
	Eigen::VectorXd pose = motormap()->fromMotorMapVectorRad(mtv);
	CHECK_EQ(q.size(), pose.size());
	q.tail(n - 6) = pose.tail(n - 6);
	//LOG(INFO) << "q = " << q.transpose();

	robot()->setPositions(q);
	robot()->computeForwardKinematics(true, true, false);

}

void HumanoidController::setMotionTargetPose(int index) {
    Eigen::VectorXd mtv = motion()->targetPoseAtIndex(index);
    LOG(INFO) << FUNCTION_NAME();
    LOG(INFO) << index << " : " << mtv.transpose();
    this->setMotorMapPose(mtv);
}


Eigen::Vector3d HumanoidController::getCOMChangeFromInitial() const
{
	Eigen::Vector3d com = robot()->getWorldCOM();
	return (com - mInitialCOM);
}
Eigen::Vector3d HumanoidController::getCOMVelocity()
{
	Eigen::Vector3d comV = robot()->getWorldCOMVelocity();
	return comV;
}

Eigen::VectorXd HumanoidController::useAnkelStrategy(const Eigen::VectorXd& refPose, double currentTime, bool bSim)
{
	double kp = 1500;
	double kd = 0;
	DecoConfig::GetSingleton()->GetDouble("Ankel", "kp", kp);
	DecoConfig::GetSingleton()->GetDouble("Ankel", "kd", kd);

	Eigen::VectorXd ret = refPose;
	if (!mIsCOMInitialized)
	{
		mInitialCOM = robot()->getWorldCOM();
		mIsCOMInitialized = true;
		return ret;
	}

	const int forwardBackwardIndex = 2;
	double deltaCOM = getCOMChangeFromInitial()[forwardBackwardIndex];
	double deltaCOMV = getCOMVelocity()[forwardBackwardIndex];

	if (bSim)
	{

		double controlInteval = 0, comNoiseLevel = 0, comVNoiseLevel = 0;
		DecoConfig::GetSingleton()->GetDouble("Sim", "ControlInteval", controlInteval);
		DecoConfig::GetSingleton()->GetDouble("Sim", "COMNoise", comNoiseLevel);
		DecoConfig::GetSingleton()->GetDouble("Sim", "COMVNoise", comVNoiseLevel);

		double comNoise = comNoiseLevel * Eigen::VectorXd::Random(1)[0];
		double comVNoise = comVNoiseLevel * Eigen::VectorXd::Random(1)[0];

		deltaCOM += comNoise; 
		deltaCOMV += comVNoise; 
		double ankelOffset = -kp * deltaCOM - kd * deltaCOMV;
		mAnkelOffsetQueue.push_back(ankelOffset);

		if ((currentTime - mLastControlTime) > controlInteval)
		{
			mLastControlTime = currentTime;
			double timeStep = robot()->getTimeStep();
			int offset = static_cast<int>(mLatency / timeStep);
			int idx = static_cast<int>(mAnkelOffsetQueue.size()) - offset - 1;
			if (idx < 0) idx = 0;
			mAnkelOffset = mAnkelOffsetQueue[idx];
		}
	}
	else
	{
		mAnkelOffset = -kp * deltaCOM - kd * deltaCOMV;;
	}
	int nMotors = motormap()->numMotors();

	ret[nMotors - 1 - 2] -= mAnkelOffset;
	ret[nMotors - 1 - 3] += mAnkelOffset;
	return ret;
}

void HumanoidController::keepFeetLevel()
{
	
	Eigen::VectorXd q = robot()->getPositions();
	mPrevPos = q;
	q.head(6) = Eigen::VectorXd::Zero(6);
	q[0] = -0.5 * DART_PI;
	robot()->setPositions(q);
	robot()->computeForwardKinematics(true, true, false);

	dart::dynamics::BodyNode* b = robot()->getBodyNode("l_foot");
	const Eigen::Isometry3d& trans = b->getTransform();
	Eigen::Isometry3d targetTrans;
	Eigen::Matrix4d internalTrans;
	internalTrans << 0, 0, -1, 0,
		0, -1, 0, 0,
		-1, 0, 0, 0,
		0.0281, -0.26867, -0.000717, 1;
	targetTrans.matrix() = internalTrans.transpose();
	Eigen::Isometry3d diffTrans = targetTrans * trans.inverse();

	Eigen::Matrix3d initialRootRot;
	initialRootRot = Eigen::AngleAxisd(-0.5 * DART_PI, Eigen::Vector3d::UnitX());
	q.head<3>() = dart::math::logMap(diffTrans.linear() * initialRootRot);
	q.segment(3, 3) = diffTrans.translation();
	robot()->setPositions(q);
	robot()->computeForwardKinematics(true, true, false);
	Eigen::VectorXd v = (q - mPrevPos) / robot()->getTimeStep();
	robot()->setVelocities(v);
}

const Eigen::VectorXd& HumanoidController::getCurrentTargetPose() const
{
	return mMotor_qHat;
}

void HumanoidController::update(double _currentTime) {
	const int NDOFS = robot()->getNumDofs();
    Eigen::VectorXd q    = robot()->getPositions();
    Eigen::VectorXd dq   = robot()->getVelocities();
    // Eigen::VectorXd qhat = Eigen::VectorXd::Zero(NDOFS);
    Eigen::VectorXd tau  = Eigen::VectorXd::Zero(NDOFS);

    // Eigen::VectorXd motor_qhat(18);
    // motor_qhat <<
    //     1.0, 1.0, -0.5, 0.5, 0.0, 0.0,
    //     0.0, 0.0,
    //     0.0, 0.0,  0.6, 0.6,  -1.0, -1.0,  0.5, 0.5,  0.0, 0.0;


    // int m = motormap()->numMotors();
    // Eigen::VectorXd mtvInitPose = Eigen::VectorXd::Ones(m) * 512;
    // mtvInitPose <<
    //     342, 681, 572, 451, 762, 261,
    //     358, 666,
    //     515, 508, 741, 282, 857, 166, 684, 339, 515, 508;
    // Eigen::VectorXd qhat = motormap()->fromMotorMapVector( mtvInitPose );
	double controlInteval = 0;
	DecoConfig::GetSingleton()->GetDouble("Sim", "ControlInteval", controlInteval);
	if (_currentTime < robot()->getTimeStep() || (_currentTime - mLastControlTime) > controlInteval)
	{
		mLastControlTime = _currentTime;
		mMotor_qHat = motion()->targetPose(_currentTime);
	}
	Eigen::VectorXd motor_qhat = mMotor_qHat;

	//Eigen::VectorXd mocapPose = mocap()->GetPose(_currentTime);
	//Eigen::VectorXd motor_qhat = motormap()->toMotorMapVectorSameDim(mocapPose);
	//motor_qhat = useAnkelStrategy(motor_qhat, _currentTime, true);

    Eigen::VectorXd qhat = motormap()->fromMotorMapVector( motor_qhat );

	const double MAX_TORQUE = 0.5 * 1.5;
	const double friction = 0.03;
    tau.head<6>() = Eigen::Vector6d::Zero();
    for (int i = 6; i < NDOFS; ++i) {
		tau(i) = -mKp(i) * (q(i) - qhat(i)) - mKd(i) * dq(i);
		if (abs(dq(i) > 1e-6))
			tau(i) += -friction * abs(dq(i)) / dq(i);

    }

    // Confine within the limit: 25% of stall torque
    // Reference: http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm
    for (int i = 6; i < NDOFS; i++) {
        tau(i) = CONFINE(tau(i), -MAX_TORQUE, MAX_TORQUE);
    }
 	//LOG(INFO) << q(20);
    // cout << _currentTime << " : " << tau.transpose() << endl;
    robot()->setForces(tau);

}

Eigen::Vector3d HumanoidController::getUpDir() const
{
	Eigen::Isometry3d rootTransform = robot()->getRootBodyNode()->getTransform();
	Eigen::Matrix3d rot = rootTransform.linear();
	return rot.col(2);
}
Eigen::Vector3d HumanoidController::getLeftDir() const
{
	Eigen::Isometry3d rootTransform = robot()->getRootBodyNode()->getTransform();
	Eigen::Matrix3d rot = rootTransform.linear();
	return rot.col(0);

}
Eigen::Vector3d HumanoidController::getForwardDir() const
{
	Eigen::Isometry3d rootTransform = robot()->getRootBodyNode()->getTransform();
	Eigen::Matrix3d rot = rootTransform.linear();
	return -rot.col(1);
}

void HumanoidController::setJointDamping(double _damping) {
	int numBodies = static_cast<int>(robot()->getNumBodyNodes());
    for (int i = 1; i < numBodies; ++i) {
        dart::dynamics::Joint* joint = robot()->getJoint(i);
		int numDofs = static_cast<int>(joint->getNumDofs());
        for (int j = 0; j < numDofs; ++j) 
		{
            joint->setDampingCoefficient(j, _damping);
        }
        
    }
}

void HumanoidController::printDebugInfo() const {
}

void HumanoidController::keyboard(unsigned char _key, int _x, int _y, double _currentTime) {
}

const std::vector<std::vector<CollisionSphere> >& HumanoidController::getCollisionSpheres() const
{
	return mCollisionSpheres;
}

// class HumanoidController ends
////////////////////////////////////////////////////////////


} // namespace robot
} // namespace bioloidgp







