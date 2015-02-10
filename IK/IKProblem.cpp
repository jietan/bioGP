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
#include "RelativePositionConstraint.h"

#include "myUtils/ConfigManager.h"
#include "robot/HumanoidController.h"


IKProblem::IKProblem(bioloidgp::robot::HumanoidController *skel, bool bCOMControl, bool bCollisionAvoidance)
    : dart::optimizer::Problem()  {
	mController = skel;
    initProblem(skel, bCOMControl, bCollisionAvoidance);
}

IKProblem::~IKProblem() {
	int numConstraints = static_cast<int>(mConstraints.size());
	for (int i = 0; i < numConstraints; ++i)
	{
		delete mConstraints[i];
	}
	mConstraints.clear();
       
}

void IKProblem::initProblem(bioloidgp::robot::HumanoidController *skel, bool bCOMControl, bool bCollisionAvoidance) {

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
	
	if (bCollisionAvoidance)
	{
		vector<int> bodyPairs;
		DecoConfig::GetSingleton()->GetIntVector("Mocap", "CollisionBodyPair", bodyPairs);
		const std::vector<std::vector<CollisionSphere> >& cSpheres = mController->getCollisionSpheres();
		int numPairs = static_cast<int>(bodyPairs.size() / 2);
		for (int i = 0; i < numPairs; ++i)
		{
			int jthBody = bodyPairs[2 * i + 0];
			int kthBody = bodyPairs[2 * i + 1];
			dart::dynamics::Skeleton* skel = getSkel();
			int numSpheresJ = static_cast<int>(cSpheres[jthBody].size());
			int numSpheresK = static_cast<int>(cSpheres[kthBody].size());
			for (int j = 0; j < numSpheresJ; ++j)
			{
				for (int k = 0; k < numSpheresK; ++k)
				{
					RelativePositionConstraint* pRPC = new RelativePositionConstraint(this->vars(), skel,
						skel->getBodyNode(jthBody), cSpheres[jthBody][j].mOffset,
						skel->getBodyNode(kthBody), cSpheres[kthBody][k].mOffset,
						(cSpheres[jthBody][j].mRadius + cSpheres[kthBody][k].mRadius) * (cSpheres[jthBody][j].mRadius + cSpheres[kthBody][k].mRadius));
					conBox()->add(pRPC);
					mConstraints.push_back(pRPC);
				}
			}
			
		}
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

void IKProblem::addObjective(dart::optimizer::Constraint* o)
{
	objBox()->add(o);
	mConstraints.push_back(o);
}
void IKProblem::addConstraint(dart::optimizer::Constraint* c)
{
	conBox()->add(c);
	mConstraints.push_back(c);
}
void IKProblem::verifyConstraint() const
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
Skeleton* IKProblem::getSkel() const {
    return mController->robot();
}

dart::optimizer::Constraint* IKProblem::getConstraint(int index) const {
    if (index >= mConstraints.size())
        return NULL;
    return mConstraints[index];
}


