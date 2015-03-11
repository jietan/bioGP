#include "stdafx.h"
#include "IKProblemRobotState.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"

using namespace dart::dynamics;

#include "dart/optimizer/Var.h"
#include "dart/optimizer/ObjectiveBox.h"
#include "dart/optimizer/ConstraintBox.h"
#include "dart/dynamics/Marker.h"
using namespace dart::optimizer;

#include "poseConstraint.h"
#include "PositionConstraint.h"


#include "myUtils/ConfigManager.h"
#include "robot/HumanoidController.h"


IKProblemRobotState::IKProblemRobotState(bioloidgp::robot::HumanoidController *skel, const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& markerPos, const vector<int>& isMarkerOccluded, const vector<int>& markersMapping)
	: dart::optimizer::Problem()  {
	mController = skel;
	initProblem(skel, markerPos, isMarkerOccluded, markersMapping);
}

IKProblemRobotState::~IKProblemRobotState() {
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		delete mConstraints[i];
	}
	mConstraints.clear();

}

void IKProblemRobotState::initProblem(bioloidgp::robot::HumanoidController *skel, const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& markerPos, const vector<int>& isMarkerOccluded, const vector<int>& markersMapping)
{

	// add variables
	int startDofId = 0;
	double oneDegree = 1.0 * M_PI / 180;
	Eigen::VectorXd pos = getSkel()->getPositions();
	for (int i = 0; i < 6; ++i)
	{
		addVariable(pos[i], getSkel()->getPositionLowerLimit(i), getSkel()->getPositionUpperLimit(i));
	}
	for (int i = 6; i < getSkel()->getNumDofs(); i++) {
		addVariable(pos[i], std::max(pos[i] - oneDegree, getSkel()->getPositionLowerLimit(i)), std::min(pos[i] + oneDegree, getSkel()->getPositionUpperLimit(i)));
	}
	//LOG(INFO) << "add # " << getNumVariables() << " Variables";

	// Create Con and Obj Boxes
	createBoxes();

	// add positional constraints
	mConstraints.clear();

	PoseConstraint* p = new PoseConstraint(this->vars(), getSkel());
	
	p->setTarget(pos);
	objBox()->add(p);
	mConstraints.push_back(p);

	int nMarkers = static_cast<int>(markerPos.size());
	const vector<dart::dynamics::Marker*> markers = mController->getMarkers();
	for (int i = 0; i < nMarkers; ++i)
	{
		const dart::dynamics::Marker* m = markers[i];
		int markerIdx = markersMapping[i];
		if (markerIdx == -1 || isMarkerOccluded[markerIdx])
			continue;
		double weight = 50;
		dart::dynamics::BodyNode* node = const_cast<dart::dynamics::BodyNode*>(m->getBodyNode());
		Eigen::Vector3d offset = m->getLocalPosition();
		Eigen::Vector3d target = markerPos[markerIdx];
		PositionConstraint* p = new PositionConstraint(vars(), getSkel(), node, offset, target, weight);
		addObjective(p);
	}



}

void IKProblemRobotState::update(double* coefs) {
	Eigen::VectorXd pose = getSkel()->getPositions();

	for (unsigned int i = 0; i < mVariables.size(); ++i) {
		pose(i) = mVariables[i]->mVal;
	}

	// cout << "IKProblem::update()" << endl;
	// cout << pose.transpose() << endl;
	getSkel()->setPositions(pose);
	getSkel()->computeForwardKinematics(true, false, false);
}

void IKProblemRobotState::addObjective(dart::optimizer::Constraint* o)
{
	objBox()->add(o);
	mConstraints.push_back(o);
}
void IKProblemRobotState::addConstraint(dart::optimizer::Constraint* c)
{
	conBox()->add(c);
	mConstraints.push_back(c);
}
void IKProblemRobotState::verifyConstraint() const
{
	int numConstraints = conBox()->getNumConstraints();
	for (int i = 0; i < numConstraints; ++i)
	{
		dart::optimizer::Constraint* c = conBox()->getConstraint(i);
		Eigen::VectorXd cValue = c->evalCon();
		if (c->mEquality == 0 && cValue.norm() > 1e-3)
			LOG(WARNING) << i << "th constraint (equality) verification failed.";
		if (c->mEquality == 1 && cValue[0] < -1e-6)
			LOG(WARNING) << i << "th constraint (inequality) verification failed:" << cValue[0];

	}

}
Skeleton* IKProblemRobotState::getSkel() const {
	return mController->robot();
}

dart::optimizer::Constraint* IKProblemRobotState::getConstraint(int index) const {
	if (index >= mConstraints.size())
		return NULL;
	return mConstraints[index];
}


