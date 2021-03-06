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
#include "robot/WorldConstructor.h"
#include "robot/Evaluation.h"
#include "myUtils/ConfigManager.h"
// #include "robot/Controller.h"
#include "myUtils/mathlib.h"
#include <sstream>
#include <iomanip>
#include <boost/interprocess/ipc/message_queue.hpp>
using namespace boost::interprocess;

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

World* gWorld = NULL;
bioloidgp::robot::HumanoidController* gController = NULL;
int gIsSystemId = 0;
int gIsRobotData = 0;

double gTimeStep = 0.0002;

double eval(CMAData* cData, int pop_id, double* timerPerStep)
{
	if (gIsRobotData)
	{
		return evalLeanToStandRobotData(gWorld, gController, cData, pop_id, timerPerStep);
	}
	else if (gIsSystemId)
	{
		return evalSystemId(gWorld, gController, cData, pop_id, timerPerStep);
	}
	else
	{
		return evalController(gWorld, gController, cData, pop_id, timerPerStep);
	}
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

    
    //srand( (unsigned int) time (NULL) );

    
	WorldConstructor::Construct();
	gWorld = WorldConstructor::msWorld;
	gController = WorldConstructor::msHumanoid;

	DecoConfig::GetSingleton()->GetInt("CMA", "isSystemId", gIsSystemId);
	DecoConfig::GetSingleton()->GetInt("CMA", "isRobotData", gIsRobotData);
	//if (gIsSystemId)
	//	gController->ReadReferenceTrajectories();

	CMAData* cmaData = NULL;

	if (gIsSystemId)
	{
		cmaData = &WorldConstructor::msIdData;
	}
	else
		cmaData = &WorldConstructor::msCData;

	int gProcessId = 0;
	if (argc == 2)
	{
		gProcessId = atoi(argv[1]);
	}
	else
	{
		double reward = eval(cmaData, 0, NULL);
		LOG(INFO) << "reward is " << reward;
		return 0;
	}
	
	int policyDim = cmaData->GetNumParameters();

	vector<vector<double> > parameters;
	while (true)
	{
		try {
			LOG(INFO) << gProcessId << "th process.";
			// creating a message queue
			char queueName[128];

			sprintf(queueName, "mqToSlave%d", gProcessId);
			message_queue mqIn(open_only, queueName);
			size_t recvd_size;
			unsigned int priority;
			double numEval;
			LOG(INFO) << "waiting to receive.";
			mqIn.receive((void*)&numEval, sizeof(double), recvd_size, priority);
			int numEvaluations = static_cast<int>(numEval + 0.5);
			if (numEvaluations <= 0)
				break;


			parameters.resize(numEvaluations);
			for (int ithEvaluation = 0; ithEvaluation < numEvaluations; ++ithEvaluation)
			{
				parameters[ithEvaluation].resize(policyDim);

				for (int i = 0; i < policyDim; ++i)
				{
					double p;
					mqIn.receive((void*)&p, sizeof(double), recvd_size, priority);
					parameters[ithEvaluation][i] = p;
				}
			}

			LOG(INFO) << "calculating.";

			vector<double> values;
			const int numRetPerEvaluation = 4;
			values.resize(numEvaluations * numRetPerEvaluation, 0);

			for (int i = 0; i < numEvaluations; ++i)
			{
				double timePerStep = 0;
				cmaData->FromParameterSetting(&(parameters[i][0]));

				ostringstream out;
				for (int ithDim = 0; ithDim < policyDim; ++ithDim)
				{
					out << setprecision(16) << parameters[i][ithDim] << " ";
				}
				LOG(INFO) << "[" << out.str() << "]:";

				values[numRetPerEvaluation * i + 0] = eval(cmaData, gProcessId, &timePerStep);

				double elapsedTime = 0;
				values[numRetPerEvaluation * i + 1] = gProcessId;
				values[numRetPerEvaluation * i + 2] = elapsedTime;
				values[numRetPerEvaluation * i + 3] = timePerStep;
				LOG(INFO) << values[numRetPerEvaluation * i + 0];
			}

			LOG(INFO) << "ready to send.";
			sprintf(queueName, "mqFromSlave%d", gProcessId);
			message_queue mqOut(open_only, queueName);
			for (int i = 0; i < numEvaluations; ++i)
			{
				for (int j = 0; j < numRetPerEvaluation; ++j)
				{
					mqOut.send(&(values[numRetPerEvaluation * i + j]), sizeof(double), 0);
				}
			}
			LOG(INFO) << "done.";

		}
		catch (interprocess_exception& e) {
			std::cout << e.what() << std::endl;
		}
	}



    return 0;
}
