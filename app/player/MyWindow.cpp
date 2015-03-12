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
#include "dart/optimizer/snopt/SnoptInterface.h"
#include "dart/optimizer/snopt/SnoptSolver.h"
#include "utils/GLObjects.h"
#include "utils/CppCommon.h"
#include "myUtils/ConfigManager.h"
#include "myUtils/mathlib.h"
#include "robot/HumanoidController.h"
#include "robot/Motion.h"
#include "IK/IKProblem.h"
#include "IK/MocapReader.h"
#include "IK/SupportInfo.h"
#include <algorithm>

//==============================================================================
MyWindow::MyWindow(bioloidgp::robot::HumanoidController* _controller)
    : SimWindow(),
      mController(_controller),
	  mController2(NULL),
	  mTime(0)
	  
{

	mDisplayTimeout = 10;
	DecoConfig::GetSingleton()->GetDouble("Server", "timeout", mDisplayTimeout);
    // 0.166622 0.548365 0.118241 0.810896
	//Eigen::Quaterniond q(0.15743952233047084, 0.53507160411429699, 0.10749289301287825, 0.82301687360383402);
 //   mTrackBall.setQuaternion(q);
    //mTrans = Eigen::Vector3d(-32.242,  212.85, 21.7107);
	
	mTrans = Eigen::Vector3d(0, -212.85, 0); 
	mFrameCount = 0;
	
	string fileName;
	mOffsetTransform = Eigen::Isometry3d::Identity();
	DecoConfig::GetSingleton()->GetString("Player", "FileName", fileName);
	readMovieFile(fileName);
	mController->robot()->setPositions(mMovie[0].mPose);

	int isRobot2 = 0;
	DecoConfig::GetSingleton()->GetInt("Player", "IsRobot2", isRobot2);
	if (isRobot2)
	{
		DecoConfig::GetSingleton()->GetString("Player", "FileName2", fileName);
		readMovieFile2(fileName);
		Eigen::VectorXd rootPos = mMovie[0].mPose.head(6);
		Eigen::VectorXd rootPos2 = mMovie2[0].mPose.head(6);
		Eigen::Matrix3d rootR = dart::math::expMapRot(rootPos.head(3));
		Eigen::Matrix3d rootR2 = dart::math::expMapRot(rootPos2.head(3));
		mOffsetTransform.linear() = rootR * rootR2.inverse();
	}
	glutTimerFunc(mDisplayTimeout, refreshTimer, 0);

}

void MyWindow::setController2(bioloidgp::robot::HumanoidController* _controller)
{
	mController2 = _controller;
	mController2->robot()->setPositions(mMovie2[0].mPose);
}

void MyWindow::readMovieFile(const string& fileName)
{
	mMovie = ReadSimFrames(fileName);
}

void MyWindow::readMovieFile2(const string& fileName)
{
	mMovie2 = ReadSimFrames(fileName);
}

//==============================================================================
MyWindow::~MyWindow()
{
    delete mController;
}

int g_cnt = 0;


void MyWindow::displayTimer(int _val) 
{
	timeStepping();
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}
//==============================================================================

Eigen::VectorXd MyWindow::samplePose(const vector<SimFrame>& frames, double time)
{
	int nFrames = static_cast<int>(frames.size());
	int i = 0;
	if (time <= frames[i].mTime)
	{
		return frames[0].mPose;
	}
	else
	{
		for (i = 0; i < nFrames - 1; ++i)
		{
			if (time > frames[i].mTime && time <= frames[i + 1].mTime)
			{
				break;
			}
		}
	}

	if (i + 1 >= nFrames)
	{
		return frames[nFrames - 1].mPose;
	}
	else
	{
		double alpha = (time - frames[i].mTime) / (frames[i + 1].mTime - frames[i].mTime);
		Eigen::VectorXd result = LinearInterpolate(frames[i].mPose, frames[i + 1].mPose, alpha);
		return result;
	}
}


void MyWindow::timeStepping()
{
	if (!mSimulating) return;
	int numFrames = static_cast<int>(mMovie.size());

	if (mController2)
	{
		numFrames = std::min(numFrames, static_cast<int>(mMovie2.size()));
		Eigen::VectorXd qhat2 = samplePose(mMovie2, mTime);// mMovie2[mFrameCount % numFrames].mPose;
		mController2->robot()->setPositions(qhat2);
		mController2->robot()->computeForwardKinematics(true, true, false);
	}
	Eigen::VectorXd qhat = samplePose(mMovie, mTime); //  mMovie[mFrameCount % numFrames].mPose;

	mController->robot()->setPositions(qhat);
	mController->robot()->computeForwardKinematics(true, true, false);



	mTime = mFrameCount * mController->robot()->getTimeStep();
	LOG(INFO) << mFrameCount << ": " << qhat[7];
	mFrameCount++;
	LOG(INFO) << mFrameCount;

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

	for (unsigned int ithSkel = 0; ithSkel < mWorld->getNumSkeletons(); ithSkel++) 
	{

		if (ithSkel == 1) {
            glPushMatrix();
            glTranslated(0, -0.035, 0);
            bioloidgp::utils::renderChessBoard(100, 100, 50.0, 50.0);
            glPopMatrix();

            //glPushMatrix();
            //glTranslated(0, -0.001, 0);
            //mWorld->getSkeleton(i)->draw(mRI);
            //glPopMatrix();
        }
		else if (ithSkel == 0)
		{
			mWorld->getSkeleton(ithSkel)->draw(mRI);
			//mWorld->getSkeleton(ithSkel)->drawMarkers(mRI);
		}
		else
		{
			glPushMatrix();
			Eigen::Vector3d offsetTranslation = mOffsetTransform.translation();
			Eigen::AngleAxisd rot;
			rot = mOffsetTransform.linear();
			//glTranslated(offsetTranslation[0], offsetTranslation[1], offsetTranslation[2]);
			glTranslated(mMovie[0].mPose[3], mMovie[0].mPose[4], mMovie[0].mPose[5]);


			glRotated(rot.angle() * 180.0 / M_PI, rot.axis()[0], rot.axis()[1], rot.axis()[2]);
			glTranslated(-mMovie2[0].mPose[3], -mMovie2[0].mPose[4], -mMovie2[0].mPose[5]);
			//glTranslated(offsetTranslation[0], offsetTranslation[1], offsetTranslation[2]);
			

			Eigen::Vector4d col;
			col << 1, 1, 0, 1;
			mWorld->getSkeleton(ithSkel)->draw(mRI, col, false);
			glPopMatrix();
		}
		
    }
			
	Eigen::Vector3d C = mWorld->getSkeleton(0)->getCOM();
	glPushMatrix();
	glTranslated(C(0), C(1), C(2));
	bioloidgp::utils::renderAxis(1.0);
	glPopMatrix();
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
    Eigen::Vector3d C = robot->getCOM();
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
        break;

    case 'p':  // step backward
		mSimulating = true;
		mFrameCount -= 2;
		timeStepping();
		mSimulating = false;
        glutPostRedisplay();

        break;
    case 'n':  // step forwardward
		mSimulating = true;
		timeStepping();
		mSimulating = false;
		glutPostRedisplay();

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

	case 'r':	
		mFrameCount = 0;
		mTime = 0;
		break;
    default:
        Win3D::keyboard(_key, _x, _y);
    }

    // Keyboard control for Controller
    mController->keyboard(_key, _x, _y, mWorld->getTime());

    glutPostRedisplay();
}

