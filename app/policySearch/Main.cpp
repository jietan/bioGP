/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <iomanip>
#include "utils/CppCommon.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/sdf/SoftSdfParser.h"
#include "dart/utils/urdf/DartLoader.h"

#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/constraint/WeldJointConstraint.h"
#include "dart/constraint/ContactConstraint.h"
//#include "dart/collision/bullet/BulletCollisionDetector.h"

#include "robot/HumanoidController.h"
#include "robot/MotorMap.h"
#include "robot/Motion.h"
#include "robot/ControllerData.h"
#include "robot/CMASearcher.h"
#include "robot/WorldConstructor.h"

#include "myUtils/ConfigManager.h"
#include "myUtils/mathlib.h"

#include <boost/interprocess/ipc/message_queue.hpp>
using namespace boost::interprocess;

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

World* gWorld = NULL;
bioloidgp::robot::HumanoidController* gController = NULL;
CMASearcher* gPolicySearch = NULL;

double gTimeStep = 0.0002;


double eval(const ControllerData& cData, int pop_id, double* timerPerStep)
{
	double reward = 0;

	gController->reset();
	gController->motion()->setControllerData(cData);
	gWorld->setTime(0);
	double motionTime = gController->motion()->getMotionLength();
	const double testTime = motionTime + 1.0;
	while (gWorld->getTime() < testTime)
	{
		gController->update(gWorld->getTime());
		gWorld->step();

		Eigen::Vector3d up = gController->getUpDir();
		Eigen::Vector3d left = gController->getLeftDir();

		double asinNegativeIfForward = up.cross(Eigen::Vector3d::UnitY()).dot(left);
		double angleFromUp = asin(asinNegativeIfForward);
		double threshold = 35.0 * M_PI / 180.0;

		//LOG(INFO) << gWorld->getTime() << " " << angleFromUp;;

		if (gWorld->getTime() > motionTime)
		{
			if (abs(angleFromUp) > threshold)
				break;
			reward += -1.0 / (abs(angleFromUp) + 0.1);
		}
	}
	return reward;
}

static void buildPolicy()
{
	int numParams = WorldConstructor::msCData.GetNumParameters();

	double* params = new double[numParams];


	gPolicySearch = new CMASearcher;
	gPolicySearch->SetDimension(numParams);
	gPolicySearch->SetEvaluatorFunc(eval);
	gPolicySearch->Search(WorldConstructor::msCData, params, 50);
	LOG(INFO) << setprecision(16) << "params " << params[0] << endl;

	delete[] params;
}

int main(int argc, char* argv[])
{
   // google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging((const char*)argv[0]);
    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = google::INFO;
#ifndef _DEBUG
    FLAGS_log_dir = "./glog/";
#endif
    LOG(INFO) << "BioloidGP program begins...";

    
    srand( (unsigned int) time (NULL) );

    gWorld = new World;
	WorldConstructor::Construct(gWorld);
	gController = WorldConstructor::msHumanoid;

	buildPolicy();

    return 0;
}
