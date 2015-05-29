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

#include "dart/math/Helpers.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/Marker.h"
#include "dart/gui/SimWindow.h"
#include "dart/gui/GLFuncs.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/math/Geometry.h"
#include "utils/GLObjects.h"
#include "utils/CppCommon.h"
#include "myUtils/ConfigManager.h"
#include "myUtils/mathlib.h"
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
	mTime(0),
	mIsInitialMarkersCaptured(false),
	mIsTimerRefresherStarted(false)
{
    mForce = Eigen::Vector3d::Zero();
    mImpulseDuration = 0.0;
	
	mDisplayTimeout = 10;
	DecoConfig::GetSingleton()->GetDouble("Server", "timeout", mDisplayTimeout);
    //// 0.166622 0.548365 0.118241 0.810896
    //Eigen::Quaterniond q(0.810896, 0.166622, 0.548365, 0.118241);
    //mTrackBall.setQuaternion(q);
    //mTrans = Eigen::Vector3d(-32.242,  212.85, 21.7107);

	mTrans = Eigen::Vector3d(0, -212.85, 0);
}

//==============================================================================
MyWindow::~MyWindow()
{
    delete mController;
}

int g_cnt = 0;

void MyWindow::setMocapClient(Client* client)
{
	mMocapClient = client;
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

int MyWindow::numUnocculudedMarkers() const
{
	int numMarkers = static_cast<int>(mMarkerPos.size());
	int ret = 0;
	for (int i = 0; i < numMarkers; ++i)
	{
		if (!mMarkerOccluded[i])
			ret++;
	}
	return ret;
}

void MyWindow::buildMarkerDistanceField()
{
	int numMarkers = static_cast<int>(mMarkerPos.size());
	if (numMarkers != numUnocculudedMarkers()) return;
	LOG(INFO) << "Num of Markers: " << numMarkers;
	mMarkerDistances = computeMarkerDistances();
	Eigen::Vector3d marker0To1Dir = (mMarkerPos[1] - mMarkerPos[0]).normalized();
	Eigen::Vector3d marker3To2Dir = (mMarkerPos[2] - mMarkerPos[3]).normalized();
	Eigen::Vector3d y = computeYFromMarkers();
	mMarker3To2AngleToXAxis = asin(marker3To2Dir.cross(marker0To1Dir).dot(y));
	mIsInitialMarkersCaptured = true;
}

Eigen::Vector3d MyWindow::computeYFromMarkers() const
{
	int numUnOcculudedMarkers = numUnocculudedMarkers();
	int numMarkers = static_cast<int>(mMarkerPos.size());
	std::vector<Eigen::Vector3d> unocculudedMarkers;
	for (int i = 0; i < numMarkers; ++i)
	{
		if (!mMarkerOccluded[i])
			unocculudedMarkers.push_back(mMarkerPos[i]);
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
Eigen::Vector3d MyWindow::computeXFromMarkers(const Eigen::Vector3d& y) const
{
	Eigen::Vector3d x;
	if (!mMarkerOccluded[0] && !mMarkerOccluded[1])
	{
		x = (mMarkerPos[1] - mMarkerPos[0]).normalized();
	}
	else if (!mMarkerOccluded[2] && !mMarkerOccluded[3])
	{
		x = (mMarkerPos[2] - mMarkerPos[3]).normalized();
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

Eigen::Vector3d MyWindow::computeZFromMarkers(const Eigen::Vector3d& x, const Eigen::Vector3d& y) const
{
	Eigen::Vector3d z = x.cross(y);
	return z.normalized();
}

vector<vector<double> > MyWindow::computeMarkerDistances() const
{
	int numMarkers = static_cast<int>(mMarkerPos.size());
	vector<vector<double> > ret;
	ret.resize(numMarkers);
	for (int i = 0; i < numMarkers; ++i)
	{
		ret[i].resize(numMarkers, 0);
		if (mMarkerOccluded[i]) continue;
		for (int j = 0; j < numMarkers; ++j)
		{
			if (mMarkerOccluded[j]) continue;
			ret[i][j] = (mMarkerPos[i] - mMarkerPos[j]).norm();
		}
	}
	return ret;
}

double MyWindow::computeDistanceOfMarkerDistances(const vector<int>& labelCandidate, const vector<vector<double> >& currentMarkerDistance)
{
	
	int numMarkers = static_cast<int>(labelCandidate.size());
	double distAccum = 0;
	int unOcculudedCount = 0;
	for (int i = 0; i < numMarkers; ++i)
	{
		int realI = labelCandidate[i];
		if (mMarkerOccluded[i]) continue;

		for (int j = 0; j < numMarkers; ++j)
		{
			int realJ = labelCandidate[j];
			if (mMarkerOccluded[j]) continue;

			distAccum += abs(currentMarkerDistance[i][j] - mMarkerDistances[realI][realJ]);
			unOcculudedCount += 1;
		}

	}
	return distAccum / unOcculudedCount;
}
void MyWindow::reorderMarkers()
{

	//based on the assumption that only one marker can be occuluded.
	int numMarkers = static_cast<int>(mMarkerPos.size());
	vector<vector<int> > candidateLabels = GetPermutation(0, numMarkers);
	int numCandidates = static_cast<int>(candidateLabels.size());
	vector<double> candidateDistances;
	candidateDistances.resize(numCandidates);
	vector<vector<double> > currentMarkerDistance = computeMarkerDistances();
	for (int i = 0; i < numCandidates; ++i)
	{
		candidateDistances[i] = computeDistanceOfMarkerDistances(candidateLabels[i], currentMarkerDistance);
	}
	vector<double>::const_iterator labelIt = std::min_element(candidateDistances.begin(), candidateDistances.end());
	vector<int> newLabel = candidateLabels[labelIt - candidateDistances.begin()];

	vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > orderedMarkerPos;
	orderedMarkerPos.resize(numMarkers);
	vector<int> orderedMarkerOcculuded;
	orderedMarkerOcculuded.resize(numMarkers);

	for (int i = 0; i < numMarkers; ++i)
	{
		if (i != newLabel[i])
			LOG(WARNING) << "The markers are relabeled.";
		orderedMarkerPos[newLabel[i]] = mMarkerPos[i];
		orderedMarkerOcculuded[newLabel[i]] = mMarkerOccluded[i];


	}
	mMarkerPos = orderedMarkerPos;
	mMarkerOccluded = orderedMarkerOcculuded;
}

bool MyWindow::fromMarkersTo6Dofs()
{
	int isUseMocap = 0;
	DecoConfig::GetSingleton()->GetInt("Ctrl", "UseMocap", isUseMocap);
	if (!isUseMocap) return false;

	if (!mIsInitialMarkersCaptured)
	{
		buildMarkerDistanceField();
	}
	if (!mIsInitialMarkersCaptured)
		return false;
	int numUnOcculudedMarkers = numUnocculudedMarkers();
	if (numUnOcculudedMarkers <= 2) return false;
	reorderMarkers();
	Eigen::Vector3d y = computeYFromMarkers();
	Eigen::Vector3d x = computeXFromMarkers(y);
	Eigen::Vector3d z = computeZFromMarkers(x, y);
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
	mFirst6DofsFromMocap = Eigen::VectorXd::Zero(6);
	mFirst6DofsFromMocap.head(3) = dart::math::logMap(rot);

	mController->setFreeDofs(mFirst6DofsFromMocap);
	Eigen::Vector3d centerOfMocapMarkers = Eigen::Vector3d::Zero();
	Eigen::Vector3d centerOfRobotMarkers = Eigen::Vector3d::Zero();
	Eigen::Vector3d translation;
	dart::dynamics::BodyNode* root = mController->robot()->getRootBodyNode();
	int nMarkers = root->getNumMarkers();
	for (int i = 0; i < nMarkers; ++i)
	{
		if (mMarkerOccluded[i]) continue;
		centerOfMocapMarkers += mMarkerPos[i];
		centerOfRobotMarkers += root->getMarker(i)->getWorldPosition();
	}
	centerOfMocapMarkers /= numUnOcculudedMarkers;
	centerOfRobotMarkers /= numUnOcculudedMarkers;
	translation = centerOfMocapMarkers - centerOfRobotMarkers;
	mFirst6DofsFromMocap.tail(3) = translation;
	return true;
}

void MyWindow::processMocapData()
{
	int isUseMocap = 0;
	DecoConfig::GetSingleton()->GetInt("Ctrl", "UseMocap", isUseMocap);
	if (!isUseMocap) return;
	while (mMocapClient->GetFrame().Result != Result::Success)
	{
		// Sleep a little so that we don't lumber the CPU with a busy poll
#ifdef WIN32
		Sleep(10);
#else
		sleep(1);
#endif

		std::cout << ".";
	}
	//std::cout << std::endl;

	// Get the frame number
	Output_GetFrameNumber _Output_GetFrameNumber = mMocapClient->GetFrameNumber();
	//std::cout << "Frame Number: " << _Output_GetFrameNumber.FrameNumber << std::endl;

	// Get the timecode
	Output_GetTimecode _Output_GetTimecode = mMocapClient->GetTimecode();


	for (unsigned int LatencySampleIndex = 0; LatencySampleIndex < mMocapClient->GetLatencySampleCount().Count; ++LatencySampleIndex)
	{
		std::string SampleName = mMocapClient->GetLatencySampleName(LatencySampleIndex).Name;
		double      SampleValue = mMocapClient->GetLatencySampleValue(SampleName).Value;
	}

	const int numMarkers = 10;
	// Count the number of subjects
	unsigned int SubjectCount = mMocapClient->GetSubjectCount().SubjectCount;
	for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
	{

		std::string SubjectName = mMocapClient->GetSubjectName(SubjectIndex).SubjectName;

		// Count the number of markers
		//unsigned int MarkerCount = mMocapClient->GetMarkerCount(SubjectName).MarkerCount;

		// Get the unlabeled markers
		unsigned int MarkerCount = mMocapClient->GetUnlabeledMarkerCount().MarkerCount;

		// Count the number of devices
		mMarkerPos.resize(numMarkers);
		mMarkerOccluded.resize(numMarkers);
		for (int i = 0; i < numMarkers; ++i)
		{
			mMarkerPos[i] = Eigen::Vector3d::Zero();
			mMarkerOccluded[i] = true;
		}
		for (unsigned int MarkerIndex = 0; MarkerIndex < MarkerCount; ++MarkerIndex)
		{
			// Get the marker name
			//std::string MarkerName = mMocapClient->GetMarkerName(SubjectName, MarkerIndex).MarkerName;

			// Get the marker parent
			//std::string MarkerParentName = mMocapClient->GetMarkerParentName(SubjectName, MarkerName).SegmentName;

			// Get the global marker translation
			//Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation = mMocapClient->GetMarkerGlobalTranslation(SubjectName, MarkerName);
			Output_GetUnlabeledMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
				mMocapClient->GetUnlabeledMarkerGlobalTranslation(MarkerIndex);
			if (MarkerIndex >= numMarkers)
				continue;
			mMarkerPos[MarkerIndex] = Eigen::Vector3d(_Output_GetMarkerGlobalTranslation.Translation[0], _Output_GetMarkerGlobalTranslation.Translation[1], _Output_GetMarkerGlobalTranslation.Translation[2]);
			mMarkerPos[MarkerIndex] /= 1000.0; //convert unit from mm to m
			mMarkerOccluded[MarkerIndex] = false;// _Output_GetMarkerGlobalTranslation.Occluded;
			//std::cout << "      Marker #" << MarkerIndex << ": "
			//	//<< MarkerName 
			//	<< " ("
			//	<< _Output_GetMarkerGlobalTranslation.Translation[0] << ", "
			//	<< _Output_GetMarkerGlobalTranslation.Translation[1] << ", "
			//	<< _Output_GetMarkerGlobalTranslation.Translation[2] << ") "
			//	<< mMarkerOccluded[MarkerIndex] << std::endl;
		}
	}

}
//==============================================================================
void MyWindow::timeStepping()
{
	processMocapData();
	bool isFirst6DofsValid = false;// fromMarkersTo6Dofs();
	Eigen::VectorXd motor_qhat = mController->motion()->targetPose(mTime);
	Eigen::VectorXd fullMotorAngle = Eigen::VectorXd::Zero(NUM_MOTORS + 2);

	if (!mSimulating)
	{
		motor_qhat = mController->motion()->getInitialPose();
	}
	
	// Wait for an event
	Eigen::VectorXd motorAngle;
	motorAngle = Eigen::VectorXd::Zero(NUM_MOTORS);
	//if (mTime < 3)
	//	motor_qhat = mController->useAnkelStrategy(motor_qhat, mTime);

	//Eigen::VectorXd mocapPose = mController->mocap()->GetPose(mTime - 1.0);
	//Eigen::VectorXd motor_qhat = mController->motormap()->toMotorMapVectorSameDim(mocapPose);
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
	
	if (isFirst6DofsValid)
		mController->setFreeDofs(mFirst6DofsFromMocap);
	if (mSimulating)
	{
		Eigen::VectorXd poseToRecord = mController->robot()->getPositions();
		MocapFrame frame;
		frame.mTime = mTime;
		frame.mMarkerPos = mMarkerPos;
		frame.mMarkerOccluded = mMarkerOccluded;
		frame.mMotorAngle = poseToRecord;
		mRecordedFrames.push_back(frame);
	}
	//mController->keepFeetLevel();
	if (mTimer.isStarted())
	{
		double elaspedTime = mTimer.getElapsedTime();
		//LOG(INFO) << elaspedTime;
		LOG(INFO) << mTime << " " << motor_qhat[10] << " " << fullMotorAngle[10] << endl;
	
		mTime += elaspedTime;// mController->robot()->getTimeStep();
	}
	mTimer.start();
}


void MyWindow::drawMocapMarkers()
{
	int nMarkers = static_cast<int>(mMarkerPos.size());
	{
		for (int i = 0; i < nMarkers; ++i)
		{
			if (mMarkerOccluded[i]) continue;
			glPushMatrix();
			glTranslated(mMarkerPos[i][0], mMarkerPos[i][1], mMarkerPos[i][2]);
			glColor3f(1.0, 0.0, 0.0);
			glutSolidSphere(0.007, 16, 16);
			glPopMatrix();

		}
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

    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
        if (i == 1) {
            glPushMatrix();
            glTranslated(0, -0.035, 0);
            bioloidgp::utils::renderChessBoard(100, 100, 50.0, 50.0);
            glPopMatrix();

            //glPushMatrix();
            //glTranslated(0, -0.001, 0);
            //mWorld->getSkeleton(i)->draw(mRI);
            //glPopMatrix();
            
            continue;
        }
        mWorld->getSkeleton(i)->draw(mRI);
		mWorld->getSkeleton(i)->drawMarkers(mRI);

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
		Eigen::Vector3d p = bn->getCOM();
        double y = p.y() - (OFFSET); // Subtract the feet height
        double z = p.z();
        // cout << bn->getName() << " " << y << " " << z << endl;
        I += bn->getMass() * (y * y + z * z);
    }
	Eigen::Vector3d C = robot->getCOM();
    cout << m << ", " << I << ", " << C.y() - OFFSET << ", " << C.z() << endl;
    
}

void MyWindow::saveRecordedFrames()
{
	string fileName;
	DecoConfig::GetSingleton()->GetString("Ctrl", "RecordingFileName", fileName);
	SaveMocapFrames(fileName, mRecordedFrames);

}
//==============================================================================
int temp = 0;
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    switch (_key)
    {
    case ' ':  // use space key to play or stop the motion
		if (mIsTimerRefresherStarted)
		{
			mSimulating = !mSimulating;
			mTime = 0;
		}
		else
		{
			glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
			mIsTimerRefresherStarted = true;
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
    case 's':  // upper right force
		saveRecordedFrames();
        break;
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

