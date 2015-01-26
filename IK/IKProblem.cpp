#include "stdafx.h"
#include "IKProblem.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"

using namespace dart::dynamics;

#include "dart/optimizer/Var.h"
#include "dart/optimizer/ObjectiveBox.h"
#include "dart/optimizer/ConstraintBox.h"
using namespace dart::optimizer;

#include "poseConstraint.h"
#include "PositionConstraint.h"
#include "COMConstraint.h"



IKProblem::IKProblem(dart::dynamics::Skeleton *skel, bool bCOMControl)
    : dart::optimizer::Problem()  {
	mSkel = skel;
    initProblem(skel, bCOMControl);
}

IKProblem::~IKProblem() {
       
}

void IKProblem::initProblem(dart::dynamics::Skeleton *skel, bool bCOMControl) {

    // add variables
	int startDofId = 0;
    for (int i = startDofId; i < getSkel()->getNumDofs(); i++) {
        addVariable(getSkel()->getPosition(i), getSkel()->getPositionLowerLimit(i), getSkel()->getPositionUpperLimit(i));
    }
    //LOG(INFO) << "add # " << getNumVariables() << " Variables";

    // Create Con and Obj Boxes
    createBoxes();

    // add positional constraints
    mConstraints.clear();

	PoseConstraint* p = new PoseConstraint(this->vars(), getSkel());
	mInitialPose = getSkel()->getPositions();
	p->setTarget(mInitialPose);
	objBox()->add(p);
	mConstraints.push_back(p);

	if (bCOMControl)
	{
		COMConstraint* pCOM = new COMConstraint(this->vars(), getSkel());
		Eigen::Vector3d targetCOM = getSkel()->getWorldCOM();
		targetCOM[0] = 0.0281 + 0.01;
		//targetCOM[1] = -0.26867;
		targetCOM[2] = 0 + 0.005;
		pCOM->setTarget(targetCOM, COM_CONSTRAINT_X | COM_CONSTRAINT_Z);
		objBox()->add(pCOM);
		mConstraints.push_back(pCOM);
	}

	int numMarkerConstraints = 3;
	vector<Eigen::Vector3d> offsets;
	vector<Eigen::Vector3d> targets;
	offsets.push_back(Eigen::Vector3d(0, 0, 0));
	offsets.push_back(Eigen::Vector3d(0, 1, 0));
	offsets.push_back(Eigen::Vector3d(0, 0, 1));

	targets.push_back(Eigen::Vector3d(0.0281, -0.26867, 0));
	targets.push_back(Eigen::Vector3d(0.0281, -1.2687,	0));
	targets.push_back(Eigen::Vector3d(-0.9719, -0.2687,	0));
	for (int i = 0; i < numMarkerConstraints; i++) {
		
		BodyNode* node = mSkel->getBodyNode("l_foot");
		
		PositionConstraint* p = new PositionConstraint(this->vars(), getSkel(), node, offsets[i], targets[i]);
		conBox()->add(p);
		mConstraints.push_back(p);
	}



    //LOG(INFO) << "# Constraints = " << conBox()->getNumConstraints();
    //LOG(INFO) << "# Objectives = " << objBox()->getNumConstraints();

    //LOG(INFO) << "initProblem OK";
}

void IKProblem::update(double* coefs) {
	Eigen::VectorXd pose = getSkel()->getPositions();

	for (unsigned int i = 0; i < mVariables.size(); ++i) {
		pose(i) = mVariables[i]->mVal;
	}

    // cout << "IKProblem::update()" << endl;
    // cout << pose.transpose() << endl;
    getSkel()->setPositions(pose);
	getSkel()->computeForwardKinematics(true, false, false);
}
void IKProblem::verifyConstraint() const
{
	int numConstraints = conBox()->getNumConstraints();
	for (int i = 0; i < numConstraints; ++i)
	{
		dart::optimizer::Constraint* c = conBox()->getConstraint(i);
		Eigen::VectorXd cValue = c->evalCon();
		if (cValue.norm() > 1e-3)
			LOG(WARNING) << i << "th constraint verification failed.";
	}

}
Skeleton* IKProblem::getSkel() const {
    return mSkel;
}

dart::optimizer::Constraint* IKProblem::getConstraint(int index) const {
    if (index >= mConstraints.size())
        return NULL;
    return mConstraints[index];
}


