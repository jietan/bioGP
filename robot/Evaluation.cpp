#include "Evaluation.h"

#include "HumanoidController.h"
#include "CMAData.h"
#include "Motion.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "WorldConstructor.h"


double evalController(dart::simulation::World* world, bioloidgp::robot::HumanoidController* controller, CMAData* cData, int pop_id, double* timerPerStep)
{
	double reward = 0;
	double motionTime = controller->motion()->getMotionLength();
	const double testTime = motionTime + 1.0;
	while (world->getTime() < testTime)
	{
		controller->update(world->getTime());
		world->step();

		Eigen::Vector3d up = controller->getUpDir();
		Eigen::Vector3d left = controller->getLeftDir();

		double asinNegativeIfForward = up.cross(Eigen::Vector3d::UnitY()).dot(left);
		double angleFromUp = asin(asinNegativeIfForward);
		double threshold = 60.0 * M_PI / 180.0;
		if (abs(angleFromUp) > threshold)
			break;
		//LOG(INFO) << gWorld->getTime() << " " << angleFromUp;;

		if (world->getTime() > motionTime)
		{
			reward += -1.0 / (abs(angleFromUp) + 0.1);
		}
		else
		{
			reward += -1;
		}
	}
	return reward;

}
double evalSystemId(dart::simulation::World* world, bioloidgp::robot::HumanoidController* controller, CMAData* cData, int pop_id, double* timerPerStep)
{
	double reward = 0;
	double testTime = 1.4;
	if (WorldConstructor::GetWorldId() == 2)
		testTime = 1.1;
	while (world->getTime() < testTime)
	{
		double t = world->getTime();
		Eigen::VectorXd q = controller->robot()->getPositions();
		double globalAngle = q.head(3).norm();
		//LOG(INFO) << globalAngle;
		controller->update(world->getTime());
		world->step();
		reward += controller->compareGlobalRotationWithReferenceTrajectories(t, globalAngle);
	}

	return reward;
}