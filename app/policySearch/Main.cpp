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
#include "robot/Evaluation.h"
#include "robot/UniformSearcher.h"

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
Searcher* gPolicySearch = NULL;
int gIsSystemId = 0;
double gTimeStep = 0.0002;


double eval(CMAData* cData, int pop_id, double* timerPerStep)
{
	//WorldConstructor::Destroy();
	//WorldConstructor::Construct();

	gController->reset();
	//SystemIdentificationData* sData = static_cast<SystemIdentificationData*>(cData);
	//sData->mMassRatio[0] = 1.0221644765835582;
	//sData->mMassRatio[1] = 1.0625016548173554;
	//sData->mMassRatio[2] = 1.0559031762912450;
	//sData->mMassRatio[3] = 1.0836126286505545;
	//sData->mMassRatio[4] = 1.0893469339813033;
	//sData->mMassRatio[5] = 1.1480225306013023;
	//sData->mMassRatio[6] = 1.1459299660452369;
	//sData->mMassRatio[7] = 1.1092851276684312;
	//sData->mMassRatio[8] = 1.0601630886808593;

	cData->ApplyToController(gController);

	if (gIsSystemId)
	{
		return evalSystemId(gWorld, gController, cData, pop_id, timerPerStep);
	}
	else
	{
		return evalController(gWorld, gController, cData, pop_id, timerPerStep);
	}
}

static void buildPolicy()
{
	CMAData* cmaData = NULL;

	if (gIsSystemId)
	{
		cmaData = &WorldConstructor::msIdData;
	}
	else
		cmaData = &WorldConstructor::msCData;

	int numParams = cmaData->GetNumParameters();
	double* params = new double[numParams];

	if (!gIsSystemId && WorldConstructor::GetWorldId() == 0)
	{
		gPolicySearch = new UniformSearcher;
	}
	else
	{
		gPolicySearch = new CMASearcher;
	}
	gPolicySearch->SetDimension(numParams);
	gPolicySearch->SetEvaluatorFunc(eval);
	gPolicySearch->Search(cmaData, params, 50);
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
	
	DecoConfig::GetSingleton()->GetInt("CMA", "isSystemId", gIsSystemId);
    
    srand( (unsigned int) time (NULL) );

    
	WorldConstructor::Construct();
	gWorld = WorldConstructor::msWorld;
	gController = WorldConstructor::msHumanoid;
	if (gIsSystemId)
		gController->ReadReferenceTrajectories();
	buildPolicy();

    return 0;
}
