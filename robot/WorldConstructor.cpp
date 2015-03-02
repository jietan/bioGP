#include "WorldConstructor.h"


bioloidgp::robot::HumanoidController* WorldConstructor::msHumanoid = NULL;
double WorldConstructor::msTimeStep = 0.0002;

void WorldConstructor::Construct(World* world)
{
	int worldId = 0;
	DecoConfig::GetSingleton()->GetInt("Sim", "World", worldId);
	switch (worldId)
	{
	case 0:
		constructWallWorld(world);
		break;
	case 1:
		constructChairWorld(world);
		break;
	case 2:
		constructKneelWorld(world);
		break;
	default:
		LOG(FATAL) << "Unsupported world.";
		break;
	}
}

void WorldConstructor::commonConstruction(World* world)
{
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
	first6Dofs << -1.57, 0, 0, 0, 0.295, -0.02;
	msHumanoid->setInitialFirst6Dofs(first6Dofs);

	msHumanoid->motion()->loadMTN("../../data/mtn/sitPose.mtn", "lean-to-stand-quickStart");
	msHumanoid->reset();
	ControllerData cData;
	cData.ReadFromFile("../../data/controller/lean-to-stand.txt");
	msHumanoid->motion()->setControllerData(cData);
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
	mtvInitPose << 512,	512, 512, 512, 512, 512, 512, 512, 512, 512, 762, 262, 912, 112, 612, 412, 512, 512;
	msHumanoid->set_motion(new bioloidgp::robot::Motion(NMOTORS, mtvInitPose));
	msHumanoid->setInitialPose(mtvInitPose);

	Eigen::VectorXd first6Dofs = Eigen::VectorXd::Zero(6);
	first6Dofs << -1.85, 0, 0, 0, 0.235, -0.02;
	msHumanoid->setInitialFirst6Dofs(first6Dofs);
	msHumanoid->motion()->loadMTN("../../data/mtn/sitPose.mtn", "sit-to-stand");
	msHumanoid->reset();
	//ControllerData cData;
	//cData.ReadFromFile("../../data/controller/lean-to-stand.txt");
	//msHumanoid->motion()->setControllerData(cData);
}
void WorldConstructor::constructKneelWorld(World* world)
{
	commonConstruction(world);

	const int NMOTORS = 18;
	Eigen::VectorXd mtvInitPose = Eigen::VectorXd::Ones(NMOTORS) * 512;
	mtvInitPose << 412, 612, 562, 462, 512, 512, 512, 512, 512, 512, 612, 412, 1023, 1, 842, 182, 512, 512;

	msHumanoid->set_motion(new bioloidgp::robot::Motion(NMOTORS, mtvInitPose));
	msHumanoid->setInitialPose(mtvInitPose);

	Eigen::VectorXd first6Dofs = Eigen::VectorXd::Zero(6);
	DecoConfig::GetSingleton()->GetVectorXd("Sim", "Initial6Dofs", first6Dofs);
	//first6Dofs << -1.85, 0, 0, 0, 0.235, -0.02;
	msHumanoid->setInitialFirst6Dofs(first6Dofs);
	msHumanoid->motion()->loadMTN("../../data/mtn/sitPose.mtn", "kneel-to-stand");
	msHumanoid->reset();
	//ControllerData cData;
	//cData.ReadFromFile("../../data/controller/lean-to-stand.txt");
	//msHumanoid->motion()->setControllerData(cData);
}