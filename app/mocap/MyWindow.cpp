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
#include "robot/HumanoidController.h"
#include "robot/Motion.h"
#include "IK/IKProblem.h"
#include "IK/MocapReader.h"
#include "IK/SupportInfo.h"

//==============================================================================
MyWindow::MyWindow(bioloidgp::robot::HumanoidController* _controller)
    : SimWindow(),
      mController(_controller),
	  mSerial(NULL),
	  mTmpBuffer(""),
	  mTime(0),
	  mDisplayMode(3)
{
    mForce = Eigen::Vector3d::Zero();
    mImpulseDuration = 0.0;
	
	mDisplayTimeout = 10;
	DecoConfig::GetSingleton()->GetDouble("Server", "timeout", mDisplayTimeout);
    // 0.166622 0.548365 0.118241 0.810896
	//Eigen::Quaterniond q(0.15743952233047084, 0.53507160411429699, 0.10749289301287825, 0.82301687360383402);
 //   mTrackBall.setQuaternion(q);
    //mTrans = Eigen::Vector3d(-32.242,  212.85, 21.7107);
	
	mTrans = Eigen::Vector3d(0, -212.85, 0); 
	mFrameCount = 0;

	glutTimerFunc(mDisplayTimeout, refreshTimer, 0);

}

//==============================================================================
MyWindow::~MyWindow()
{
    delete mController;
}

int g_cnt = 0;


void MyWindow::setSupportInfo(SupportInfo* support)
{
	mSupportInfo = support;
}
void MyWindow::setMocap(MocapReader* mocap)
{
	mMocapReader = mocap;
}

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
	if (!mSimulating) return;
	static dart::common::Timer t;
	//if (mFrameCount < 560)
	//	mFrameCount = 560;
	/*frameCount = 0;*/
	//Eigen::VectorXd motor_qhat = mController->motion()->targetPose(mTime);
	//mController->setMotorMapPose(motor_qhat);
	//Eigen::VectorXd motor_qhat = Eigen::VectorXd::Zero(18);
	Eigen::VectorXd motor_qhat = mMocapReader->GetFrame(mFrameCount).GetRobotPose();
	mController->setMotorMapPoseRad(motor_qhat);

	//mFrameCount = 0;
	dart::dynamics::Skeleton* character = mWorld->getSkeleton(2);
	Eigen::VectorXd q = character->getPositions();
	Eigen::VectorXd qhat = mMocapReader->GetFrame(mFrameCount).GetCharacterPose();

	//Eigen::VectorXd::Zero(37);//
	character->setPositions(qhat);
	character->computeForwardKinematics(true, true, false);

	int isUseIK = 0;
	DecoConfig::GetSingleton()->GetInt("Mocap", "IsUseIK", isUseIK);

	if (isUseIK)
	{
		Eigen::VectorXd prevPose = mController->robot()->getPositions();
		int isUseCOM = 0, isAvoidCollision = 0;
		DecoConfig::GetSingleton()->GetInt("Mocap", "IsUseCOMControl", isUseCOM);
		DecoConfig::GetSingleton()->GetInt("Mocap", "IsAvoidCollision", isAvoidCollision);
		IKProblem ik(mController, isUseCOM, isAvoidCollision);
		SupportStateType supportType = mSupportInfo->GetSupportType(mFrameCount);
		LOG(INFO) << "Current Support Type: " << supportType;
		mSupportInfo->AddConstraints(mFrameCount, &ik);
		dart::optimizer::snopt::SnoptSolver solver(&ik);
		bool ret = solver.solve();
		if (!ret)
		{
			LOG(WARNING) << "IK solve failed.";
			//ik.getSkel()->setPositions(prevPose);
			//CHECK(0);
		}
		ik.verifyConstraint();
		Eigen::VectorXd poseAfterIK = ik.getSkel()->getPositions();
		Eigen::VectorXd motorPoseAfterIK = mController->motormap()->toMotorMapVectorRad(poseAfterIK);
		mMocapReader->SetFrameAfterIK(mFrameCount, motorPoseAfterIK);
	}
	//mController->keepFeetLevel();
	double elaspedTime = t.getElapsedTime();
	//LOG(INFO) << elaspedTime;
	mTime += mController->robot()->getTimeStep();
	mFrameCount++;
	LOG(INFO) << mFrameCount;
	//mTime += elaspedTime;
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

	for (unsigned int ithSkel = 0; ithSkel < mWorld->getNumSkeletons(); ithSkel++) 
	{
		if (ithSkel != mDisplayMode && mDisplayMode < mWorld->getNumSkeletons())
			continue;
		if (ithSkel == 1) {
            glPushMatrix();
            glTranslated(0, -0.003, 0);
            bioloidgp::utils::renderChessBoard(100, 100, 50.0, 50.0);
            glPopMatrix();

            //glPushMatrix();
            //glTranslated(0, -0.001, 0);
            //mWorld->getSkeleton(i)->draw(mRI);
            //glPopMatrix();
        }
		else
		{
			mWorld->getSkeleton(ithSkel)->draw(mRI);
			mWorld->getSkeleton(ithSkel)->drawMarkers(mRI);
			int isShowCollisionSphere = 0;
			DecoConfig::GetSingleton()->GetInt("Display", "IsShowCollisionSphere", isShowCollisionSphere);

			
			//if (ithSkel == 2)
			//{
			//	int numBodies = mWorld->getSkeleton(ithSkel)->getNumBodyNodes();
			//	const double axisScale = 0.03;
			//	for (int i = 0; i < numBodies; ++i)
			//	{
			//		Eigen::Isometry3d transform = mWorld->getSkeleton(ithSkel)->getBodyNode(i)->getTransform();

			//		glPushMatrix();
			//		glTranslated(transform.translation()[0], transform.translation()[1], transform.translation()[2]);
			//		glDisable(GL_LIGHTING);

			//		glPushMatrix();
			//		// const double LEN = 10.0;

			//		glColor3d(1.0, 0.0, 0.0);
			//		glBegin(GL_LINES);
			//		glVertex3d(0, 0.0, 0.0);
			//		glVertex3d(axisScale * transform.linear()(0, 0), axisScale * transform.linear()(1, 0), axisScale * transform.linear()(2, 0));
			//		glEnd();

			//		glColor3d(0.0, 1.0, 0.0);
			//		glBegin(GL_LINES);
			//		glVertex3d(0.0, 0, 0.0);
			//		glVertex3d(axisScale * transform.linear()(0, 1), axisScale * transform.linear()(1, 1), axisScale * transform.linear()(2, 1));
			//		glEnd();

			//		glColor3d(0.0, 0.0, 1.0);
			//		glBegin(GL_LINES);
			//		glVertex3d(0.0, 0.0, 0);
			//		glVertex3d(axisScale * transform.linear()(0, 2), axisScale * transform.linear()(1, 2), axisScale * transform.linear()(2, 2));
			//		glEnd();
			//		glPopMatrix();

			//		glEnable(GL_LIGHTING);
			//		glPopMatrix();
			//	}
			//}

			if (isShowCollisionSphere && mWorld->getSkeleton(ithSkel) == mController->robot())
			{
				const std::vector<std::vector<CollisionSphere> >& cspheres = mController->getCollisionSpheres();
				int numBodies = mWorld->getSkeleton(ithSkel)->getNumBodyNodes();
				for (int i = 0; i < numBodies; ++i)
				{
					int numSpheres = static_cast<int>(cspheres[i].size());
					Eigen::Isometry3d transform = mWorld->getSkeleton(ithSkel)->getBodyNode(i)->getTransform();

					for (int j = 0; j < numSpheres; ++j)
					{
						Eigen::Vector3d centerWorld = transform * cspheres[i][j].mOffset;
						glPushMatrix();
						glTranslated(centerWorld[0], centerWorld[1], centerWorld[2]);
						glutSolidSphere(cspheres[i][j].mRadius, 16, 16);
						glPopMatrix();
					}
				}
			}
		}
		
    }
			
	Eigen::Vector3d C = mWorld->getSkeleton(0)->getWorldCOM();
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
        break;
	case '1':
		mDisplayMode = (mDisplayMode + 1) % 4;
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
	case 's':
		mMocapReader->SaveMotionAfterIK("../../mocap/stepRight.mocap");
		break;
	case 'r':
		mController->reset();
		
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

