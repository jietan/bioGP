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
#define STRICT
#include <tchar.h>
#include <stdio.h>
#include <string.h>
#include <vector>


#include <iostream>

#include "utils/CppCommon.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Marker.h"
#include "dart/simulation/World.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/sdf/SoftSdfParser.h"
#include "dart/utils/urdf/DartLoader.h"
#include "dart/constraint/WeldJointConstraint.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
//#include "dart/collision/bullet/BulletCollisionDetector.h"

#include "MyWindow.h"
#include "robot/HumanoidController.h"
#include "robot/MotorMap.h"
#include "robot/Motion.h"
#include "robot/WorldConstructor.h"
#include "myUtils/ConfigManager.h"
#include <windows.h>



using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

void AddMarkers(dart::dynamics::Skeleton* robotSkel)
{
	dart::dynamics::BodyNode* root = robotSkel->getBodyNode("torso");
	dart::dynamics::Marker* leftFrontMarker = new dart::dynamics::Marker("torso_lf", Eigen::Vector3d(0.037, -0.019, 0.02), root);
	dart::dynamics::Marker* rightFrontMarker = new dart::dynamics::Marker("torso_rf", Eigen::Vector3d(-0.037, -0.019, 0.02), root);
	dart::dynamics::Marker* rightBackMarker = new dart::dynamics::Marker("torso_rb", Eigen::Vector3d(-0.042, 0.025, 0.02), root);
	dart::dynamics::Marker* leftBackMarker = new dart::dynamics::Marker("torso_lb", Eigen::Vector3d(0.052, 0.03, 0.02), root);
	root->addMarker(leftFrontMarker);
	root->addMarker(rightFrontMarker);
	root->addMarker(rightBackMarker);
	root->addMarker(leftBackMarker);

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
	DecoConfig::GetSingleton()->Init("../config.ini");

 //   World* myWorld = new World;
 //   myWorld->setTimeStep(0.017);



 //   // // Load ground and Atlas robot and add them to the world
 //   DartLoader urdfLoader;
 //   Skeleton* ground = urdfLoader.parseSkeleton(
 //       DATA_DIR"/sdf/ground.urdf");
 //   Skeleton* robot
 //       = urdfLoader.parseSkeleton(
 //           DATA_DIR"/urdf/BioloidGP/BioloidGP.URDF");
 //   robot->enableSelfCollision();

	//
 //   myWorld->addSkeleton(robot);
 //   myWorld->addSkeleton(ground);


 //   // Print some info
 //   LOG(INFO) << "robot.mass = " << robot->getMass();

 //   // Set gravity of the world
 //   myWorld->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
	////AddWeldConstraint(myWorld);

 //   // Create a humanoid controller
 //   bioloidgp::robot::HumanoidController* con =
 //       new bioloidgp::robot::HumanoidController(robot, myWorld->getConstraintSolver());
	
	//AddMarkers(con->robot());

	//WorldConstructor::Construct(myWorld);
	
	WorldConstructor::Construct();
	World* myWorld = WorldConstructor::msWorld;
	myWorld->setTimeStep(0.017);
	MyWindow window(WorldConstructor::msHumanoid);
    window.setWorld(myWorld);

	int isRobot2 = 0;
	DecoConfig::GetSingleton()->GetInt("Player", "IsRobot2", isRobot2);
	if (isRobot2)
	{
		DartLoader urdfLoader;
		Skeleton* robot2 = urdfLoader.parseSkeleton(DATA_DIR"/urdf/BioloidGP/BioloidGP.URDF");
		robot2->enableSelfCollision();
		myWorld->addSkeleton(robot2);
		bioloidgp::robot::HumanoidController* con2 =
			new bioloidgp::robot::HumanoidController(robot2, myWorld->getConstraintSolver());
		window.setController2(con2);
	}

    // Print manual
    LOG(INFO) << "space bar: simulation on/off";
    LOG(INFO) << "'p': playback/stop";
    LOG(INFO) << "'[' and ']': play one frame backward and forward";
    LOG(INFO) << "'v': visualization on/off";
    LOG(INFO) << endl;

    // Run glut loop
    glutInit(&argc, argv);
    window.initWindow(1280, 720, "BioloidGP Robot - with Dart4.0");

    glutMainLoop();

    return 0;
}
