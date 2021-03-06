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
#include "dart/constraint/ContactConstraint.h"
//#include "dart/collision/bullet/BulletCollisionDetector.h"

#include "MyWindow.h"
#include "robot/HumanoidController.h"
#include "robot/MotorMap.h"
#include "robot/Motion.h"
#include "robot/WorldConstructor.h"
#include "myUtils/ConfigManager.h"
#include <windows.h>
#include "Serial.h"
#include "Client.h"
using namespace ViconDataStreamSDK::CPP;

#ifdef WIN32
#include <conio.h>   // For _kbhit()
#include <cstdio>   // For getchar()
#endif // WIN32

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

dart::constraint::WeldJointConstraint* gWeldJoint;
bool gTransmitMulticast = false;


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



void SetupMocapClient(Client* MyClient)
{
	int isUseMocap = 0;
	DecoConfig::GetSingleton()->GetInt("Ctrl", "UseMocap", isUseMocap);
	if (!isUseMocap) return;

	std::string HostName = "localhost:801";
	// Make a new client
		

	for (int i = 0; i != 3; ++i) // repeat to check disconnecting doesn't wreck next connect
	{
		// Connect to a server
		std::cout << "Connecting to " << HostName << " ..." << std::flush;
		while (!MyClient->IsConnected().Connected)
		{
			// Direct connection
			MyClient->Connect(HostName);

			// Multicast connection
			// MyClient.ConnectToMulticast( HostName, "224.0.0.0" );

			std::cout << ".";
#ifdef WIN32
			Sleep(10);
#else
			sleep(1);
#endif
		}
		std::cout << std::endl;

		// Enable some different data types
		//MyClient.EnableSegmentData();
		//MyClient->EnableMarkerData();
		MyClient->EnableUnlabeledMarkerData();
		//MyClient.EnableDeviceData();

		//std::cout << "Segment Data Enabled: "          << Adapt( MyClient.IsSegmentDataEnabled().Enabled )         << std::endl;
		//std::cout << "Marker Data Enabled: " << (MyClient->IsMarkerDataEnabled().Enabled) << std::endl;
		std::cout << "Unlabeled Marker Data Enabled: " << (MyClient->IsUnlabeledMarkerDataEnabled().Enabled) << std::endl;
		//std::cout << "Device Data Enabled: "           << Adapt( MyClient.IsDeviceDataEnabled().Enabled )          << std::endl;

		// Set the streaming mode
		MyClient->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
		// MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
		// MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

		// Set the global up axis
		MyClient->SetAxisMapping(Direction::Forward,
			Direction::Left,
			Direction::Up); // Z-up
		// MyClient.SetGlobalUpAxis( Direction::Forward, 
		//                           Direction::Up, 
		//                           Direction::Right ); // Y-up

		Output_GetAxisMapping _Output_GetAxisMapping = MyClient->GetAxisMapping();
		std::cout << "Axis Mapping: X-" << (_Output_GetAxisMapping.XAxis)
			<< " Y-" << (_Output_GetAxisMapping.YAxis)
			<< " Z-" << (_Output_GetAxisMapping.ZAxis) << std::endl;

		// Discover the version number
		Output_GetVersion _Output_GetVersion = MyClient->GetVersion();
		std::cout << "Version: " << _Output_GetVersion.Major << "."
			<< _Output_GetVersion.Minor << "."
			<< _Output_GetVersion.Point << std::endl;

		if (gTransmitMulticast)
		{
			MyClient->StartTransmittingMulticast("localhost", "224.0.0.0");
		}
	}
		
}
void DestroyMocapClient(Client* MyClient)
{
	int isUseMocap = 0;
	DecoConfig::GetSingleton()->GetInt("Ctrl", "UseMocap", isUseMocap);
	if (!isUseMocap) return;

	if (gTransmitMulticast)
	{
		MyClient->StopTransmittingMulticast();
	}
	//MyClient.DisableSegmentData();
	MyClient->DisableMarkerData();
	//MyClient.DisableUnlabeledMarkerData();
	//MyClient.DisableDeviceData();

	// Disconnect and dispose
		
	std::cout << " Disconnecting..." << std::endl;
	MyClient->Disconnect();
}

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
	//serial communication setup
	CSerial serial;
	LONG    lLastError = ERROR_SUCCESS;
	std::string portName = "";
	DecoConfig::GetSingleton()->GetString("Server", "clientPort", portName);
	lLastError = serial.Open(_T("COM2"), 0, 0, false);
	//lLastError = serial.Open(LPCTSTR(portName.c_str()), 0, 0, false);
	if (lLastError != ERROR_SUCCESS)
		LOG(FATAL) << serial.GetLastError() << " Unable to open COM-port.";
	lLastError = serial.Setup(CSerial::EBaud57600, CSerial::EData8, CSerial::EParNone, CSerial::EStop1);
	if (lLastError != ERROR_SUCCESS)
		LOG(FATAL) << serial.GetLastError() << " Unable to set COM-port setting.";
	lLastError = serial.SetMask(CSerial::EEventBreak |
		CSerial::EEventCTS |
		CSerial::EEventDSR |
		CSerial::EEventError |
		CSerial::EEventRing |
		CSerial::EEventRLSD |
		CSerial::EEventRecv);
	if (lLastError != ERROR_SUCCESS)
		LOG(FATAL) << serial.GetLastError() << " Unable to set COM-port event mask.";
	lLastError = serial.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
	if (lLastError != ERROR_SUCCESS)
		LOG(FATAL) << serial.GetLastError() << " Unable to set COM-port read timeout.";
	//lLastError = serial.Write("w");
	//if (lLastError != ERROR_SUCCESS)
	//	LOG(FATAL) << serial.GetLastError() << " Unable to send data.";
	// end serial communication setup
	Client myClient;
	SetupMocapClient(&myClient);

	WorldConstructor::Construct();
	World* myWorld = WorldConstructor::msWorld;
	double playBackSpeed = 1.0;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "PlaybackSpeed", playBackSpeed);
	WorldConstructor::msTimeStep = playBackSpeed * 0.017;

	AddMarkers(WorldConstructor::msHumanoid->robot());
	//Eigen::VectorXd q6 = Eigen::VectorXd::Zero(6);
	//q6[4] = 0;
	//con->setFreeDofs(q6);

    // Create a window and link it to the world
    // MyWindow window(new Controller(robot, myWorld->getConstraintSolver()));
    MyWindow window(WorldConstructor::msHumanoid);
    window.setWorld(myWorld);
	window.setSerial(&serial);
	window.setMocapClient(&myClient);
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
	DestroyMocapClient(&myClient);
    return 0;
}
