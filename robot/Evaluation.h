#ifndef _EVALUATION_H
#define _EVALUATION_H

namespace dart
{
	namespace simulation
	{
		class World;
	}
}
namespace bioloidgp
{
	namespace robot
	{
		class HumanoidController;
	}
}

class CMAData;

double evalController(dart::simulation::World* world, bioloidgp::robot::HumanoidController* controller, CMAData* cData, int pop_id, double* timerPerStep);
double evalSystemId(dart::simulation::World* world, bioloidgp::robot::HumanoidController* controller, CMAData* cData, int pop_id, double* timerPerStep);

#endif // !_EVALUATION_H
