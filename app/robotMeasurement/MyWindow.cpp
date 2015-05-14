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
#include "dart/dynamics/Marker.h"
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
#include "myUtils/JetColorMap.h"
#include "myUtils/mathlib.h"
#include "robot/HumanoidController.h"
#include "robot/Motion.h"
#include "IK/IKProblemRobotState.h"


//==============================================================================
MyWindow::MyWindow(bioloidgp::robot::HumanoidController* _controller)
    : SimWindow(),
      mController(_controller),
	  mTmpBuffer(""),
	  mTime(0),
	  mDisplayMode(3)
{
	mDisplayTimeout = 10;
	DecoConfig::GetSingleton()->GetDouble("Server", "timeout", mDisplayTimeout);
    // 0.166622 0.548365 0.118241 0.810896
	//Eigen::Quaterniond q(0.15743952233047084, 0.53507160411429699, 0.10749289301287825, 0.82301687360383402);
 //   mTrackBall.setQuaternion(q);
    //mTrans = Eigen::Vector3d(-32.242,  212.85, 21.7107);
	
	mTrans = Eigen::Vector3d(0, -212.85, 0); 
	mFrameCount = 0;
	readMeasurementFile();
	glutTimerFunc(mDisplayTimeout, refreshTimer, 0);

}

//==============================================================================
MyWindow::~MyWindow()
{
    delete mController;
}

int g_cnt = 0;


void MyWindow::readMeasurementFile()
{
	string measurementFileName = "";
	DecoConfig::GetSingleton()->GetString("Measurement", "MeasurementFileName", measurementFileName);
	mMeasuredFrames = ReadMocapFrames(measurementFileName);
	mConvertedFrames.resize(mMeasuredFrames.size());
	
}


void MyWindow::saveProcessedMeasurement()
{
	string fileName = "";
	DecoConfig::GetSingleton()->GetString("Measurement", "ProcessedDofFileName", fileName);
	SaveSimFrames(fileName, mConvertedFrames);

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


	int nFrames = static_cast<int>(mMeasuredFrames.size());
	if (mFrameCount < 0)
		mFrameCount += nFrames;
	
	int ithFrame = mFrameCount % nFrames;
	if (ithFrame == nFrames - 1)
	{
		saveProcessedMeasurement();
	}
	const MocapFrame& frame = mMeasuredFrames[ithFrame];
	Eigen::VectorXd pose = frame.mMotorAngle;
	Eigen::VectorXd first6Dofs = Eigen::VectorXd::Zero(6);
	const int nMarkers = static_cast<int>(mMeasuredFrames[ithFrame].mMarkerPos.size());
	mMarkersMapping.resize(nMarkers);
	CHECK(nMarkers == 10) << "Weird number of markers";
	mMarkersMapping[0] = 5;
	mMarkersMapping[1] = 0;
	mMarkersMapping[2] = 7;
	mMarkersMapping[3] = 3;
	const int nTopMarkers = 4;
	vector<Eigen::Vector3d> topMarkersPos(nTopMarkers);
	vector<int> topMarkesOcculuded(nTopMarkers);
	for (int i = 0; i < nTopMarkers; ++i)
	{
		topMarkersPos[i] = mMeasuredFrames[ithFrame].mMarkerPos[mMarkersMapping[i]];
		topMarkesOcculuded[i] = mMeasuredFrames[ithFrame].mMarkerOccluded[mMarkersMapping[i]];
	}
	bool result = fromMarkersTo6Dofs(topMarkersPos, topMarkesOcculuded, first6Dofs);
	if (result)
	{
		pose.head(6) = first6Dofs;
	}
	mController->robot()->setPositions(pose);
	
	for (int i = 0; i < nMarkers; ++i)
	{
		mMarkersMapping[i] = -1;
	}
	for (int i = 0; i < nMarkers; ++i)
	{
		if (frame.mMarkerOccluded[i])
			continue;
		
		int nearestBodyMarkId = findNearestBodyMarker(frame.mMarkerPos[i]);
		mMarkersMapping[nearestBodyMarkId] = i;
	}
	int isUseIK = 0;
	DecoConfig::GetSingleton()->GetInt("Measurement", "IsUseIK", isUseIK);
	if (isUseIK)
	{
		IKProblemRobotState ik(mController, frame.mMarkerPos, frame.mMarkerOccluded, mMarkersMapping);

		dart::optimizer::snopt::SnoptSolver solver(&ik);
		bool ret = solver.solve();
		if (!ret)
		{
			LOG(WARNING) << "IK solve failed.";
		}

		pose = ik.getSkel()->getPositions();

	}
	mConvertedFrames[ithFrame].mTime = frame.mTime;
	mConvertedFrames[ithFrame].mPose = pose;

	double elaspedTime = t.getElapsedTime();
	//LOG(INFO) << elaspedTime;
	mTime += mController->robot()->getTimeStep();
	mFrameCount++;
	LOG(INFO) << mFrameCount;
	//mTime += elaspedTime;
	t.start();
}

int MyWindow::findNearestBodyMarker(const Eigen::Vector3d& markerPos)
{
	const vector<dart::dynamics::Marker*> robotMarkers = mController->getMarkers();
	int nMarkers = static_cast<int>(robotMarkers.size());
	double minDist = DBL_MAX;
	int nearestId = -1;
	for (int i = 0; i < nMarkers; ++i)
	{
		double dist = (robotMarkers[i]->getWorldPosition() - markerPos).norm();
		if (dist < minDist)
		{
			minDist = dist;
			nearestId = i;
		}
	}
	return nearestId;
}
int MyWindow::numUnocculudedMarkers(const vector<Eigen::Vector3d>& topMarkerPos, const vector<int>& topMarkerOcculuded) const
{
	int numMarkers = static_cast<int>(topMarkerPos.size());
	int ret = 0;
	for (int i = 0; i < numMarkers; ++i)
	{
		if (!topMarkerOcculuded[i])
			ret++;
	}
	return ret;
}

Eigen::Vector3d MyWindow::computeYFromMarkers(const vector<Eigen::Vector3d>& topMarkerPos, const vector<int>& topMarkerOcculuded) const
{
	int numUnOcculudedMarkers = numUnocculudedMarkers(topMarkerPos, topMarkerOcculuded);
	int numMarkers = static_cast<int>(topMarkerPos.size());
	std::vector<Eigen::Vector3d> unocculudedMarkers;
	for (int i = 0; i < numMarkers; ++i)
	{
		if (!topMarkerOcculuded[i])
			unocculudedMarkers.push_back(topMarkerPos[i]);
	}
	std::vector<Eigen::Vector3d> axis;
	Eigen::Vector3d mean;
	PCAOnPoints(unocculudedMarkers, mean, axis);
	Eigen::Vector3d y = axis[0];
	if (y[1] < 0)
	{
		y = -y;
	}
	return y.normalized();
}
Eigen::Vector3d MyWindow::computeXFromMarkers(const vector<Eigen::Vector3d>& topMarkerPos, const vector<int>& topMarkerOcculuded, const Eigen::Vector3d& y) const
{
	Eigen::Vector3d x;
	if (!topMarkerOcculuded[0] && !topMarkerOcculuded[1])
	{
		x = (topMarkerPos[1] - topMarkerPos[0]).normalized();
	}
	else if (!topMarkerOcculuded[2] && !topMarkerOcculuded[3])
	{
		x = (topMarkerPos[2] - topMarkerPos[3]).normalized();
		Eigen::Matrix3d rot;
		rot = Eigen::AngleAxisd(mMarker3To2AngleToXAxis, y);
		x = rot * x;
	}
	else
	{
		LOG(FATAL) << "Should not reach here.";
	}
	return x.normalized();
}

Eigen::Vector3d MyWindow::computeZFromMarkers(const vector<Eigen::Vector3d>& topMarkerPos, const vector<int>& topMarkerOcculuded, const Eigen::Vector3d& x, const Eigen::Vector3d& y) const
{
	Eigen::Vector3d z = x.cross(y);
	return z.normalized();
}

void MyWindow::buildMarker3To2AngleToXAxis(const vector<Eigen::Vector3d>& topMarkerPos, const vector<int>& topMarkerOcculuded)
{
	int numMarkers = static_cast<int>(topMarkerPos.size());
	if (numMarkers != numUnocculudedMarkers(topMarkerPos, topMarkerOcculuded)) return;
	LOG(INFO) << "Num of Markers: " << numMarkers;
	Eigen::Vector3d marker0To1Dir = (topMarkerPos[1] - topMarkerPos[0]).normalized();
	Eigen::Vector3d marker3To2Dir = (topMarkerPos[2] - topMarkerPos[3]).normalized();
	Eigen::Vector3d y = computeYFromMarkers(topMarkerPos, topMarkerOcculuded);
	mMarker3To2AngleToXAxis = asin(marker3To2Dir.cross(marker0To1Dir).dot(y));
	mIsInitialMarkersCaptured = true;
}

bool MyWindow::fromMarkersTo6Dofs(const vector<Eigen::Vector3d>& topMarkerPos, const vector<int>& topMarkerOcculuded, Eigen::VectorXd& first6Dofs)
{
	if (!mIsInitialMarkersCaptured)
	{
		buildMarker3To2AngleToXAxis(topMarkerPos, topMarkerOcculuded);
	}
	Eigen::Vector3d y = computeYFromMarkers(topMarkerPos, topMarkerOcculuded);
	Eigen::Vector3d x = computeXFromMarkers(topMarkerPos, topMarkerOcculuded, y);
	Eigen::Vector3d z = computeZFromMarkers(topMarkerPos, topMarkerOcculuded, x, y);
	Eigen::Matrix3d rot;
	//rot.col(0) = x;
	//rot.col(1) = y;
	//rot.col(2) = z;
	rot = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
	Eigen::Matrix3d rot1;
	rot1.col(0) = x;
	rot1.col(1) = y;
	rot1.col(2) = z;
	rot = rot1 * rot;
	first6Dofs = Eigen::VectorXd::Zero(6);
	first6Dofs.head(3) = dart::math::logMap(rot);

	mController->setFreeDofs(first6Dofs);
	Eigen::Vector3d centerOfMocapMarkers = Eigen::Vector3d::Zero();
	Eigen::Vector3d centerOfRobotMarkers = Eigen::Vector3d::Zero();
	Eigen::Vector3d translation;
	dart::dynamics::BodyNode* root = mController->robot()->getRootBodyNode();
	int nMarkers = root->getNumMarkers();
	for (int i = 0; i < nMarkers; ++i)
	{
		if (topMarkerOcculuded[i]) continue;
		centerOfMocapMarkers += topMarkerPos[i];
		centerOfRobotMarkers += root->getMarker(i)->getWorldPosition();
	}
	int numUnOcculudedMarkers = numUnocculudedMarkers(topMarkerPos, topMarkerOcculuded);
	centerOfMocapMarkers /= numUnOcculudedMarkers;
	centerOfRobotMarkers /= numUnOcculudedMarkers;
	translation = centerOfMocapMarkers - centerOfRobotMarkers;
	first6Dofs.tail(3) = translation;
	return true;
}


void MyWindow::drawMocapMarkers()
{
	int nFrames = static_cast<int>(mMeasuredFrames.size());
	MocapFrame frame = mMeasuredFrames[0];
	if (mFrameCount)
		frame = mMeasuredFrames[(mFrameCount - 1) % nFrames];
	int nMarkers = static_cast<int>(frame.mMarkerPos.size());
	
	for (int i = 0; i < nMarkers; ++i)
	{
		if (frame.mMarkerOccluded[i]) continue;
		Eigen::Vector3f col = GetColour(i, 0, nMarkers);// Eigen::Vector3f::Zero();
		//for (int j = 0; j < nMarkers; ++j)
		//{
		//	if (j < mMarkersMapping.size() && mMarkersMapping[j] == i)
		//		col = GetColour(j, 0, nMarkers);
		//}
		glPushMatrix();
		glTranslated(frame.mMarkerPos[i][0], frame.mMarkerPos[i][1], frame.mMarkerPos[i][2]);
		glColor3f(col[0], col[1], col[2]);
		glutSolidSphere(0.007, 16, 16);
		glPopMatrix();
	}
	
}

void MyWindow::draw()
{
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	drawSkels();
	drawMocapMarkers();
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
            glTranslated(0, -0.035, 0);
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
			
	Eigen::Vector3d C = mWorld->getSkeleton(0)->getCOM();
	glPushMatrix();
	glTranslated(C(0), C(1), C(2));
	bioloidgp::utils::renderAxis(1.0);
	glPopMatrix();
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
    case 'v':  // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
	case 's':
		
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

