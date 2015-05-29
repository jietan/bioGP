#include "WorldConstructor.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/BodyNode.h"
#include "MotorMap.h"

World* WorldConstructor::msWorld = NULL;
bioloidgp::robot::HumanoidController* WorldConstructor::msHumanoid = NULL;
double WorldConstructor::msTimeStep = 0.0002;
ControllerData WorldConstructor::msCData;
SystemIdentificationData WorldConstructor::msIdData;
int WorldConstructor::mWorldId = 0;

void WorldConstructor::Debug()
{
	return;
	// Create a humanoid controller
	//msHumanoid = new bioloidgp::robot::HumanoidController(robot, msWorld->getConstraintSolver());
	DartLoader urdfLoader;
	Skeleton* robot
		= urdfLoader.parseSkeleton(
		DATA_DIR"/urdf/BioloidGP/BioloidGP.URDF");
	robot->enableSelfCollision();
	msWorld->withdrawSkeleton(msHumanoid->robot());
	msWorld->addSkeleton(robot);

	msHumanoid->reset();

	Skeleton* tmp = msHumanoid->robot();
	msHumanoid->robot() = robot;
	msHumanoid->setJointDamping(0.0);
	msHumanoid->reset();

	Eigen::VectorXd qOrig = tmp->getPositions();
	Eigen::VectorXd q = msHumanoid->robot()->getPositions();
	Eigen::VectorXd err = qOrig - q;

	Eigen::VectorXd qDotOrig = tmp->getVelocities();
	Eigen::VectorXd qDot = msHumanoid->robot()->getVelocities();
	Eigen::VectorXd errDot = qDotOrig - qDot;


}


int WorldConstructor::GetWorldId()
{
	return mWorldId;
}

void WorldConstructor::Construct()
{
	msWorld = new World();
	int worldId = 0;
	DecoConfig::GetSingleton()->GetInt("Sim", "World", worldId);
	mWorldId = worldId;
	switch (worldId)
	{
	case 0:
		constructWallWorld(msWorld);
		break;
	case 1:
		constructChairWorld(msWorld);
		break;
	case 2:
		constructKneelWorld(msWorld);
		break;
	default:
		LOG(FATAL) << "Unsupported world.";
		break;
	}
	
}

void WorldConstructor::Destroy()
{
	if (msWorld)
		delete msWorld;
	if (msHumanoid)
		delete msHumanoid;
}

void WorldConstructor::commonConstruction(World* world)
{
	DecoConfig::GetSingleton()->GetDouble("Sim", "TimeStep", msTimeStep);
	world->setTimeStep(msTimeStep);
	// Set gravity of the world
	world->setGravity(Eigen::Vector3d(0.0, -9.81, 0));
	dart::constraint::ContactConstraint::setErrorReductionParameter(0.0);
	dart::constraint::ContactConstraint::setMaxErrorReductionVelocity(0.1);

	// // Load ground and Atlas robot and add them to the world
	DartLoader urdfLoader;
	Skeleton* ground = urdfLoader.parseSkeleton(
		DATA_DIR"/sdf/ground.urdf");
	Skeleton* robot 
		= urdfLoader.parseSkeleton(
		DATA_DIR"/urdf/BioloidGP/BioloidGP.URDF");

	int isHybridDynamics = 0;
	DecoConfig::GetSingleton()->GetInt("Sim", "HybridDynamics", isHybridDynamics);
	if (isHybridDynamics)
	{
		dart::dynamics::Joint* joint0 = robot->getJoint(0);
		joint0->setActuatorType(dart::dynamics::Joint::PASSIVE);
		for (size_t i = 1; i < robot->getNumBodyNodes(); ++i)
		{
			dart::dynamics::Joint* joint = robot->getJoint(i);
			joint->setActuatorType(dart::dynamics::Joint::VELOCITY);
		}

	}
	

	robot->enableSelfCollision();

	world->addSkeleton(robot);
	world->addSkeleton(ground);

	// Create a humanoid controller
	msHumanoid = new bioloidgp::robot::HumanoidController(robot, world->getConstraintSolver());


}

void WorldConstructor::constructWallWorld(World* world)
{
	commonConstruction(world);

	DartLoader urdfLoader;
	Skeleton* wall = urdfLoader.parseSkeleton(
		DATA_DIR"/sdf/wall.urdf");
	world->addSkeleton(wall);

	const int NMOTORS = 18;
	Eigen::VectorXd mtvInitPose = Eigen::VectorXd::Ones(NMOTORS) * 512;
	mtvInitPose << 512, 512, 512, 512, 312, 712, 512, 512, 512, 512, 632, 392, 512, 512, 512, 512, 512, 512;

	msHumanoid->set_motion(new bioloidgp::robot::Motion(NMOTORS, mtvInitPose));
	msHumanoid->setInitialPose(mtvInitPose);

	Eigen::VectorXd first6Dofs = Eigen::VectorXd::Zero(6);
	first6Dofs << -1.535210908621381, -0.03100735580890254, -0.008932248690586354, -0.0001811591801725731, 0.2953401869229447, -0.02281130177915284;
	//first6Dofs << -1.57, 0, 0, 0, 0.295, -0.02;
	msHumanoid->setInitialFirst6Dofs(first6Dofs);

	msHumanoid->motion()->loadMTN("../../data/mtn/sitPose.mtn", "lean-to-stand-quickStart");


	msHumanoid->reset();
	
	msCData.ReadFromFile("../../data/controller/lean-to-stand.txt");
	msHumanoid->motion()->setControllerData(msCData);

	double totalMass = msHumanoid->robot()->getMass();
	LOG(INFO) << "The total mass: " << totalMass;
	Eigen::Vector3d com = msHumanoid->robot()->getCOM();
	LOG(INFO) << "The COM: " << com[0] << " " << com[1] << " " << com[2];

	int isUseSystemId = 0;
	DecoConfig::GetSingleton()->GetInt("Sim", "IsUseSystemId", isUseSystemId);
	if (isUseSystemId)
	{
		msIdData.ReadFromFile("../../data/systemIdentification/lean-to-stand/0.13.txt");
		msHumanoid->setSystemIdData(msIdData);
		msHumanoid->robot()->computeForwardKinematics(true, true, false);
	}

	
	LOG(INFO) << "The total mass: " << totalMass;
	com = msHumanoid->robot()->getCOM();
	LOG(INFO) << "The COM: " << com[0] << " " << com[1] << " " << com[2];

}
void WorldConstructor::constructChairWorld(World* world)
{
	commonConstruction(world);
	DartLoader urdfLoader;
	Skeleton* chair = urdfLoader.parseSkeleton(
		DATA_DIR"/sdf/chair.urdf");
	world->addSkeleton(chair);

	const int NMOTORS = 18;
	Eigen::VectorXd mtvInitPose = Eigen::VectorXd::Ones(NMOTORS) * 512;
	mtvInitPose << 512,	512, 512, 512, 512, 512, 512, 512, 512, 512, 762, 262, 812, 212, 512, 512, 512, 512;
	msHumanoid->set_motion(new bioloidgp::robot::Motion(NMOTORS, mtvInitPose));
	msHumanoid->setInitialPose(mtvInitPose);

	Eigen::VectorXd first6Dofs = Eigen::VectorXd::Zero(6);
	first6Dofs << -1.85, 0, 0, 0, 0.235, -0.02;
	msHumanoid->setInitialFirst6Dofs(first6Dofs);
	msHumanoid->motion()->loadMTN("../../data/mtn/sitPose.mtn", "sit-to-stand");
	msHumanoid->reset();
	
	msCData.ReadFromFile("../../data/controller/sit-to-stand.txt");
	msHumanoid->motion()->setControllerData(msCData);
}
void WorldConstructor::constructKneelWorld(World* world)
{
	commonConstruction(world);
	world->getConstraintSolver()->getCollisionDetector()->disablePair(msHumanoid->robot()->getBodyNode("torso"), msHumanoid->robot()->getBodyNode("l_heel"));
	world->getConstraintSolver()->getCollisionDetector()->disablePair(msHumanoid->robot()->getBodyNode("torso"), msHumanoid->robot()->getBodyNode("r_heel"));
	world->getConstraintSolver()->getCollisionDetector()->disablePair(msHumanoid->robot()->getBodyNode("l_hip"), msHumanoid->robot()->getBodyNode("l_heel"));
	world->getConstraintSolver()->getCollisionDetector()->disablePair(msHumanoid->robot()->getBodyNode("r_hip"), msHumanoid->robot()->getBodyNode("r_heel"));
	world->getConstraintSolver()->getCollisionDetector()->disablePair(msHumanoid->robot()->getBodyNode("l_thigh"), msHumanoid->robot()->getBodyNode("l_heel"));
	world->getConstraintSolver()->getCollisionDetector()->disablePair(msHumanoid->robot()->getBodyNode("r_thigh"), msHumanoid->robot()->getBodyNode("r_heel"));
	const int NMOTORS = 18;
	Eigen::VectorXd mtvInitPose = Eigen::VectorXd::Ones(NMOTORS) * 512;
	mtvInitPose << 412, 612, 562, 462, 512, 512, 512, 512, 512, 512, 612, 412, 1023, 1, 842, 182, 512, 512;

	msHumanoid->set_motion(new bioloidgp::robot::Motion(NMOTORS, mtvInitPose));
	msHumanoid->setInitialPose(mtvInitPose);

	Eigen::VectorXd first6Dofs = Eigen::VectorXd::Zero(6);
	//DecoConfig::GetSingleton()->GetVectorXd("Sim", "Initial6Dofs", first6Dofs);
	//first6Dofs << -1.4, 0, 0, 0, 0.206, 0;
	first6Dofs << -1.328514851378874, -0.01304727359099908, 0.01469150902990003, 0.0009582401251999403, 0.2044851807667352, -0.002224987479213158;
	msHumanoid->setInitialFirst6Dofs(first6Dofs);
	msHumanoid->motion()->loadMTN("../../data/mtn/sitPose.mtn", "kneel-to-stand");
	msHumanoid->reset();
	
	msCData.ReadFromFile("../../data/controller/kneel-to-stand.txt");
	msHumanoid->motion()->setControllerData(msCData);

	double totalMass = msHumanoid->robot()->getMass();
	LOG(INFO) << "The total mass: " << totalMass;
	Eigen::Vector3d com = msHumanoid->robot()->getCOM();
	LOG(INFO) << "The COM: " << com[0] << " " << com[1] << " " << com[2];

	int isUseSystemId = 0;
	DecoConfig::GetSingleton()->GetInt("Sim", "IsUseSystemId", isUseSystemId);
	if (isUseSystemId)
	{
		msIdData.ReadFromFile("../../data/systemIdentification/kneel-to-stand/sim1.txt");
		msHumanoid->setSystemIdData(msIdData);
		msHumanoid->robot()->computeForwardKinematics(true, true, false);
	}

	LOG(INFO) << "The total mass: " << totalMass;
	com = msHumanoid->robot()->getCOM();
	LOG(INFO) << "The COM: " << com[0] << " " << com[1] << " " << com[2];

}