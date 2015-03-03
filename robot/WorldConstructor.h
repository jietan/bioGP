#ifndef _WORLD_CONSTRUCTOR_H
#define _WORLD_CONSTRUCTOR_H

#include <iostream>

#include "utils/CppCommon.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/sdf/SoftSdfParser.h"
#include "dart/utils/urdf/DartLoader.h"

#include "dart/constraint/ConstraintSolver.h"
#include "dart/constraint/ContactConstraint.h"
//#include "dart/collision/bullet/BulletCollisionDetector.h"

#include "HumanoidController.h"
#include "MotorMap.h"
#include "Motion.h"
#include "ControllerData.h"

#include "myUtils/ConfigManager.h"

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

class WorldConstructor
{
public:
	static void Construct(World* world);
	static bioloidgp::robot::HumanoidController* msHumanoid;
	static ControllerData msCData;
	static double msTimeStep;
private:
	static void commonConstruction(World* world);
	static void constructWallWorld(World* world);
	static void constructChairWorld(World* world);
	static void constructKneelWorld(World* world);
};


#endif