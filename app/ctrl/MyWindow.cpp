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
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "MyWindow.h"
#include "dart/common/Timer.h"
#include "dart/math/Helpers.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/gui/SimWindow.h"
#include "dart/gui/GLFuncs.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/math/Geometry.h"
#include "utils/GLObjects.h"
#include "utils/CppCommon.h"
#include "myUtils/ConfigManager.h"
#include "robot/HumanoidController.h"
#include "robot/Motion.h"
#include "Serial.h"



#define NUM_MOTORS 16
#define NUM_BYTES_PER_MOTOR 3



//int MakeWord(int hex0, int hex1, int hex2, int hex3)
//{
//	unsigned int word;
//
//	word = ((hex0 << 12) + (hex1 << 8) + (hex2 << 4) + hex3);
//	return (int)word;
//}
int MakeWord(unsigned char lowbyte, unsigned char highbyte)
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}
void SeparateWord(int word, unsigned char* highByte, unsigned char* lowByte)
{
	unsigned short temp;

	temp = word & 0xff;
	*lowByte = static_cast<unsigned char>(temp);

	temp = word & 0xff00;
	temp = temp >> 8;
	*highByte = static_cast<unsigned char>(temp);
}

//int Decipher(char c)
//{
//	int ret = c;
//	if (ret > '9')
//	{
//		ret -= 7;
//	}
//	ret -= '0';
//	return ret;
//}

bool ProcessFrame(const string& frame, Eigen::VectorXd& motorAngle)
{
	std::vector<int> tmpPos;
	tmpPos.resize(NUM_MOTORS);
	for (int i = 0; i < NUM_MOTORS; ++i)
	{
		//int mPos = MakeWord(Decipher(frame[NUM_BYTES_PER_MOTOR * i]), Decipher(frame[NUM_BYTES_PER_MOTOR * i + 1]), Decipher(frame[NUM_BYTES_PER_MOTOR * i + 2]), Decipher(frame[NUM_BYTES_PER_MOTOR * i + 3]));
		int mPos = MakeWord(frame[NUM_BYTES_PER_MOTOR * i], frame[NUM_BYTES_PER_MOTOR * i + 1]);
		tmpPos[i] = mPos;
		if (mPos < 0 || mPos >= 1024)
		{
			std::cout << "Warning! Joint angle abnormal." << std::endl;
			return false;
		}

	}
	for (int i = 0; i < NUM_MOTORS; ++i)
	{
		motorAngle[i] = tmpPos[i];
	}
	return true;
}

int FindLastOf(const string& buff, const string& target, int startPos)
{
	int buffLen = static_cast<int>(buff.size());
	int targetLen = static_cast<int>(target.length());
	for (int i = startPos - 1; i >= targetLen; --i)
	{
		bool found = true;
		for (int j = 0; j < targetLen; ++j)
		{
			if (buff[i - targetLen + j + 1] != target[j])
			{
				found = false;
				break;
			}
		}
		if (found)
			return i;
	}
	return buffLen + 1;
}

bool ProcessBuffer(string& buff, Eigen::VectorXd& motorAngle)
{
	bool ret = false;
	size_t endPos = FindLastOf(buff, "\t\n", buff.size());
	if (endPos >= buff.size())
	{
		std::cout << "Warning! End symbol not found!" << std::endl;
		return false;
	}
	else if (endPos == NUM_BYTES_PER_MOTOR * NUM_MOTORS + 2 - 1)
	{
		ret = ProcessFrame(buff, motorAngle);
	}
	else if (endPos < NUM_BYTES_PER_MOTOR * NUM_MOTORS + 2 - 1)
	{
		std::cout << "End symbol too early!" << std::endl;
	}
	else
	{
		size_t newEndPos = FindLastOf(buff, "\t\n", endPos - 2);
		if (endPos - newEndPos == NUM_BYTES_PER_MOTOR * NUM_MOTORS + 2)
		{
			string frame = buff.substr(newEndPos + 1, endPos);
			ret = ProcessFrame(frame, motorAngle);
			endPos = newEndPos;
		}
		else
		{
			std::cout << "Unknow reason." << std::endl;
		}
	}
	buff = buff.substr(endPos + 1, buff.size());

	return ret;
}
//==============================================================================
MyWindow::MyWindow(bioloidgp::robot::HumanoidController* _controller)
    : SimWindow(),
      mController(_controller),
	  mSerial(NULL),
	  mTmpBuffer(""),
	  mTime(0)
{
    mForce = Eigen::Vector3d::Zero();
    mImpulseDuration = 0.0;
	
	mDisplayTimeout = 10;
	DecoConfig::GetSingleton()->GetDouble("Server", "timeout", mDisplayTimeout);
    // 0.166622 0.548365 0.118241 0.810896
    Eigen::Quaterniond q(0.810896, 0.166622, 0.548365, 0.118241);
    mTrackBall.setQuaternion(q);
    mTrans = Eigen::Vector3d(-32.242,  212.85, 21.7107);
}

//==============================================================================
MyWindow::~MyWindow()
{
    delete mController;
}

int g_cnt = 0;

void MyWindow::setSerial(CSerial* serial)
{
	mSerial = serial;
}
void MyWindow::displayTimer(int _val) 
{
	timeStepping();
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}
//==============================================================================
void MyWindow::timeStepping()
{
	static dart::common::Timer t;

	
	// Wait for an event
	Eigen::VectorXd motorAngle;
	motorAngle = Eigen::VectorXd::Zero(NUM_MOTORS);
	//Eigen::VectorXd motor_qhat = mController->motion()->targetPose(mTime);
	//if (mTime < 3)
	//	motor_qhat = mController->useAnkelStrategy(motor_qhat, mTime);

	Eigen::VectorXd mocapPose = mController->mocap()->GetPose(mTime - 1.0);
	Eigen::VectorXd motor_qhat = mController->motormap()->toMotorMapVectorSameDim(mocapPose);
	Eigen::VectorXd motor_qhat_noGriper = Eigen::VectorXd::Zero(NUM_MOTORS);
	motor_qhat_noGriper.head(6) = motor_qhat.head(6);
	motor_qhat_noGriper.tail(10) = motor_qhat.tail(10);
	unsigned char commands[256];
	for (int i = 0; i < NUM_MOTORS; ++i)
	{
		int command = static_cast<int>(motor_qhat_noGriper[i]);
		unsigned char highByte, lowByte;
		SeparateWord(command, &highByte, &lowByte);
		commands[2 * i] = lowByte;
		commands[2 * i + 1] = highByte;
		//commands[3 * i + 2] = ' ';
	}
	commands[2 * 16] = '\t';
	commands[2 * 16 + 1] = '\n';
	mSerial->Write(commands, 2 * 16 + 2);
//	commands[3 * 16] = '\n';
//	mSerial->Write(commands, 3 * 16 + 1);
	//LOG(INFO) << motor_qhat_noGriper[14];
	//LONG lLastError = mSerial->WaitEvent();

	//if (lLastError != ERROR_SUCCESS)
	//	LOG(FATAL) << mSerial->GetLastError() << " Unable to wait for a COM-port event.";

	//// Save event
	//const CSerial::EEvent eEvent = mSerial->GetEventType();

	//// Handle break event
	//if (eEvent & CSerial::EEventBreak)
	//{
	//	printf("\n### BREAK received ###\n");
	//}

	//// Handle CTS event
	//if (eEvent & CSerial::EEventCTS)
	//{
	//	printf("\n### Clear to send %s ###\n", mSerial->GetCTS() ? "on" : "off");
	//}

	//// Handle DSR event
	//if (eEvent & CSerial::EEventDSR)
	//{
	//	printf("\n### Data set ready %s ###\n", mSerial->GetDSR() ? "on" : "off");
	//}

	//// Handle error event
	//if (eEvent & CSerial::EEventError)
	//{
	//	printf("\n### ERROR: ");
	//	switch (mSerial->GetError())
	//	{
	//	case CSerial::EErrorBreak:		printf("Break condition");			break;
	//	case CSerial::EErrorFrame:		printf("Framing error");			break;
	//	case CSerial::EErrorIOE:		printf("IO device error");			break;
	//	case CSerial::EErrorMode:		printf("Unsupported mode");			break;
	//	case CSerial::EErrorOverrun:	printf("Buffer overrun");			break;
	//	case CSerial::EErrorRxOver:		printf("Input buffer overflow");	break;
	//	case CSerial::EErrorParity:		printf("Input parity error");		break;
	//	case CSerial::EErrorTxFull:		printf("Output buffer full");		break;
	//	default:						printf("Unknown");					break;
	//	}
	//	printf(" ###\n");
	//}

	//// Handle ring event
	//if (eEvent & CSerial::EEventRing)
	//{
	//	printf("\n### RING ###\n");
	//}

	//// Handle RLSD/CD event
	//if (eEvent & CSerial::EEventRLSD)
	//{
	//	printf("\n### RLSD/CD %s ###\n", mSerial->GetRLSD() ? "on" : "off");
	//}

	//// Handle data receive event
	//if (eEvent & CSerial::EEventRecv)
	//{
		// Read data, until there is nothing left
		DWORD dwBytesRead = 0;
		char szBuffer[101];
		bool bUpdated = false;
		do
		{
			// Read data from the COM-port
			LONG lLastError = mSerial->Read(szBuffer, sizeof(szBuffer) - 1, &dwBytesRead);

			if (lLastError != ERROR_SUCCESS)
				LOG(FATAL) << mSerial->GetLastError() << " Unable to read from COM-port.";

			if (dwBytesRead > 0)
			{
				mTmpBuffer.insert(mTmpBuffer.size(), szBuffer, dwBytesRead);

				if (mTmpBuffer.size() >= NUM_BYTES_PER_MOTOR * NUM_MOTORS + 1)
				{

					bUpdated = ProcessBuffer(mTmpBuffer, motorAngle);

					if (bUpdated)
					{
						Eigen::VectorXd fullMotorAngle = Eigen::VectorXd::Zero(NUM_MOTORS + 2);
						fullMotorAngle.head(6) = motorAngle.head(6);
						fullMotorAngle.tail(10) = motorAngle.tail(10);
						mController->setMotorMapPose(fullMotorAngle);						
					}
					//for (int i = 0; i < 3; ++i)
					//{
					//	std::cout << motorAngle[i] << " "; //<< motor_qhat_noGriper[14];
					//}
					//std::cout << std::endl;
				}
			}
			else
			{
				std::cout << "Noting received." << endl;
			}
		} while (dwBytesRead == sizeof(szBuffer) - 1);
	//}
		mController->keepFeetLevel();
		double elaspedTime = t.getElapsedTime();
		//LOG(INFO) << elaspedTime;
		std::cout << elaspedTime << endl;
		//mTime += elaspedTime;
		mTime += mController->robot()->getTimeStep();
		t.start();
}

void MyWindow::draw()
{
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	drawSkels();

	// display the frame count in 2D text
	char buff[64];
	if (!mSimulating)
#ifdef WIN32
		_snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#else
		std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#endif
	else
#ifdef WIN32
		_snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#else
		std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#endif
	std::string frame(buff);
	glColor3f(0.0, 0.0, 0.0);
	dart::gui::drawStringOnScreen(0.02f, 0.02f, frame);
	glEnable(GL_LIGHTING);

}
//==============================================================================
void MyWindow::drawSkels()
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
        if (i == 1) {
            glPushMatrix();
            glTranslated(0, -0.301, 0);
            bioloidgp::utils::renderChessBoard(100, 100, 50.0, 50.0);
            glPopMatrix();

            glPushMatrix();
            glTranslated(0, -0.001, 0);
            mWorld->getSkeleton(i)->draw(mRI);
            glPopMatrix();
            
            continue;
        }
        mWorld->getSkeleton(i)->draw(mRI);
        // {
        //     Eigen::Vector3d C = mWorld->getSkeleton(i)->getWorldCOM();
        //     glPushMatrix();
        //     glTranslated(C(0), C(1), C(2));
        //     bioloidgp::utils::renderAxis(1.0);
        //     glPopMatrix();
        // }
        
    }
}

void MyWindow::calculateInertia() {
    dart::dynamics::Skeleton* robot = mWorld->getSkeleton(0);
    int n = robot->getNumBodyNodes();

    double I = 0.0;
    double m = 0;
    double OFFSET = -0.30;
    for (int i = 0; i < n; i++) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        m += bn->getMass();
        Eigen::Vector3d p = bn->getWorldCOM();
        double y = p.y() - (OFFSET); // Subtract the feet height
        double z = p.z();
        // cout << bn->getName() << " " << y << " " << z << endl;
        I += bn->getMass() * (y * y + z * z);
    }
    Eigen::Vector3d C = robot->getWorldCOM();
    cout << m << ", " << I << ", " << C.y() - OFFSET << ", " << C.z() << endl;
    
}


//==============================================================================
int temp = 0;
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    switch (_key)
    {
    case ' ':  // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if (mSimulating)
        {
            mPlay = false;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p':  // playBack
        mPlay = !mPlay;
        if (mPlay)
        {
            mSimulating = false;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[':  // step backward
        if (!mSimulating)
        {
            mPlayFrame-=10;
            if (mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']':  // step forwardward
        if (!mSimulating)
        {
            mPlayFrame+=10;
            if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case 'I':  // print debug information
        calculateInertia();
        break;
    case 'i':  // print debug information
        mController->printDebugInfo();
        break;
    case 'v':  // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
    case 'a':  // upper right force
        mForce[0] = 5;
        mImpulseDuration = 100;
        std::cout << "push forward" << std::endl;
        break;
    case 's':  // upper right force
        mForce[0] = -5;
        mImpulseDuration = 100;
        std::cout << "push backward" << std::endl;
        break;
    case 'd':  // upper right force
        mForce[2] = 5;
        mImpulseDuration = 100;
        std::cout << "push right" << std::endl;
        break;
    case 'f':  // upper right force
        mForce[2] = -5;
        mImpulseDuration = 100;
        std::cout << "push left" << std::endl;
        break;
    case 'n':  // upper right force
        mController->setMotionTargetPose(temp);
        temp++;
	case 'r':
		mController->reset();
		
		mTime = 0;
		break;
    default:
        Win3D::keyboard(_key, _x, _y);
    }

    // Keyboard control for Controller
    mController->keyboard(_key, _x, _y, mWorld->getTime());

    glutPostRedisplay();
}

