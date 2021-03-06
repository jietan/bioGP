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
#include "myUtils/ConfigManager.h"
#include <windows.h>
#include "IK/MocapReader.h"
#include "IK/CMUSkeletonReader.h"
#include "IK/SupportInfo.h"


using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

dart::constraint::WeldJointConstraint* gWeldJoint;

void AddWeldConstraint(World* myWorld)
{
	BodyNode* bd = myWorld->getSkeleton(0)->getBodyNode("torso");
	gWeldJoint = new dart::constraint::WeldJointConstraint(bd);
	myWorld->getConstraintSolver()->addConstraint(gWeldJoint);
}


//int ShowError(LONG lError, LPCTSTR lptszMessage)
//{
//	// Generate a message text
//	TCHAR tszMessage[256];
//	wsprintf(tszMessage, _T("%s\n(error code %d)"), lptszMessage, lError);
//
//	// Display message-box and return with an error-code
//	::MessageBox(0, tszMessage, _T("Listener"), MB_ICONSTOP | MB_OK);
//	return 1;
//}

void AddMarkers(dart::dynamics::Skeleton* mocapSkel, dart::dynamics::Skeleton* robotSkel)
{
	double scale = 0.06;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "Scale", scale);
	dart::dynamics::BodyNode* mocapLFoot = mocapSkel->getBodyNode("lfoot");
	dart::dynamics::Marker* lFootMarker = new dart::dynamics::Marker("lfoot", Eigen::Vector3d(0, -2.13962 * scale, 0), mocapLFoot);
	mocapLFoot->addMarker(lFootMarker);

	dart::dynamics::BodyNode* mocapRFoot = mocapSkel->getBodyNode("rfoot");
	dart::dynamics::Marker* rFootMarker = new dart::dynamics::Marker("rfoot", Eigen::Vector3d(0, -2.13962 * scale, 0), mocapRFoot);
	
	mocapRFoot->addMarker(rFootMarker);

	dart::dynamics::BodyNode* robotLFoot = robotSkel->getBodyNode("l_foot");
	dart::dynamics::Marker* robotLFootMarker = new dart::dynamics::Marker("l_foot", Eigen::Vector3d(0, 0.0275947483062579, -0.005), robotLFoot);
	dart::dynamics::Marker* robotLFootMarkerUp = new dart::dynamics::Marker("l_footUp", Eigen::Vector3d(0, 0.0275947483062579 - 0.1, -0.005), robotLFoot);
	dart::dynamics::Marker* robotLFootMarkerLeft = new dart::dynamics::Marker("l_footLeft", Eigen::Vector3d(0, 0.0275947483062579, -0.005 - 0.1), robotLFoot);
	robotLFoot->addMarker(robotLFootMarker);
	robotLFoot->addMarker(robotLFootMarkerUp);
	robotLFoot->addMarker(robotLFootMarkerLeft);

	dart::dynamics::BodyNode* robotRFoot = robotSkel->getBodyNode("r_foot");
	dart::dynamics::Marker* robotRFootMarker = new dart::dynamics::Marker("r_foot", Eigen::Vector3d(0, 0.0275947483062579, 0.005), robotRFoot);
	dart::dynamics::Marker* robotRFootMarkerUp = new dart::dynamics::Marker("r_footUp", Eigen::Vector3d(0, 0.0275947483062579 - 0.1, 0.005), robotRFoot);
	dart::dynamics::Marker* robotRFootMarkerLeft = new dart::dynamics::Marker("r_footLeft", Eigen::Vector3d(0, 0.0275947483062579, 0.005 - 0.1), robotRFoot);
	robotRFoot->addMarker(robotRFootMarker);
	robotRFoot->addMarker(robotRFootMarkerUp);
	robotRFoot->addMarker(robotRFootMarkerLeft);
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

    World* myWorld = new World;
    myWorld->setTimeStep(0.017);



    // // Load ground and Atlas robot and add them to the world
    DartLoader urdfLoader;
    Skeleton* ground = urdfLoader.parseSkeleton(
        DATA_DIR"/sdf/ground.urdf");
    Skeleton* robot
        = urdfLoader.parseSkeleton(
            DATA_DIR"/urdf/BioloidGP/BioloidGP.URDF");
    robot->enableSelfCollision();

	//dart::simulation::World* testWorld
	//	= dart::utils::SkelParser::readWorld("../../data/skel/fullbody1.skel");

	Skeleton* mocapSkel = ReadCMUSkeleton("../../mocap/oneFootBalance.asf");
	
    myWorld->addSkeleton(robot);
    myWorld->addSkeleton(ground);
	myWorld->addSkeleton(mocapSkel);
	AddMarkers(mocapSkel, robot);

    // Print some info
    LOG(INFO) << "robot.mass = " << robot->getMass();

    // Set gravity of the world
    myWorld->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
	//AddWeldConstraint(myWorld);

    // Create a humanoid controller
    bioloidgp::robot::HumanoidController* con =
        new bioloidgp::robot::HumanoidController(robot, myWorld->getConstraintSolver());
	
	//Eigen::VectorXd q6 = Eigen::VectorXd::Zero(6);
	//q6[4] = 0;
	//con->setFreeDofs(q6);

    // Create a window and link it to the world
    // MyWindow window(new Controller(robot, myWorld->getConstraintSolver()));

	MocapReader mocapReader;
	string fileName;
	DecoConfig::GetSingleton()->GetString("Mocap", "OriginalFileName", fileName);
	mocapReader.Read(fileName);

	fileName.replace(fileName.length() - 4, 4, ".state");
	SupportInfo supportInfo(fileName);
	supportInfo.SetSkeletons(mocapSkel, con->robot());
	//supportInfo.SetLeftGlobal(0.1 * Eigen::Vector3d(0, 0, 1));
	supportInfo.SetLeftGlobal( 0.1 * Eigen::Vector3d(-1, 0, 0));
    MyWindow window(con);
    window.setWorld(myWorld);
	window.setMocap(&mocapReader);
	window.setSupportInfo(&supportInfo);
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
