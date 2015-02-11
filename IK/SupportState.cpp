#include "SupportState.h"
#include "myUtils/ConfigManager.h"
#include "DirectionConstraint.h"

SupportState::SupportState() : mOrig(NULL), mTarget(NULL)
{
}
SupportState::~SupportState()
{
}
SupportStateType SupportState::GetType() const
{
	return mType;
}
void SupportState::SetFrameRange(int start, int end)
{
	mStartFrame = start;
	mEndFrame = end;
}
void SupportState::SetSkeletons(dart::dynamics::Skeleton* origSkel, dart::dynamics::Skeleton* targetSkel)
{
	mOrig = origSkel;
	mTarget = targetSkel;
}

void SupportState::SetLeftGlobal(const Eigen::Vector3d& left)
{
	mLeftGlobal = left;
}
void SupportState::AddConstraint(int frameNum, IKProblem* ik)
{

}

void SupportState::addCOMObjective(int frameNum, const Eigen::Vector3d& comTarget, IKProblem* ik)
{
	int bCOMControl = 0;
	DecoConfig::GetSingleton()->GetInt("Mocap", "IsUseCOMControl", bCOMControl);
	if (bCOMControl)
	{
		COMConstraint* pCOM = new COMConstraint(ik->vars(), mTarget);

		pCOM->setTarget(comTarget, COM_CONSTRAINT_X | COM_CONSTRAINT_Z);
		//ik->addObjective(pCOM);
		ik->addConstraint(pCOM);
	}
}

void SupportState::addLeftFootObjective(int frameNum, const Eigen::Vector3d& footConstraint, bool bUseLeftDir, IKProblem* ik)
{
	Eigen::Vector3d offset;
	Eigen::Vector3d target;
	double weight;
	dart::dynamics::BodyNode* node = NULL;
	PositionConstraint* p = NULL;

	weight = 50;
	node = mTarget->getBodyNode("l_foot");
	offset = mTarget->getMarker("l_foot")->getLocalPosition();
	p = new PositionConstraint(ik->vars(), mTarget, node, offset, footConstraint, weight);
	ik->addObjective(p);

	weight = 50;
	node = mTarget->getBodyNode("l_foot");
	offset = mTarget->getMarker("l_footUp")->getLocalPosition();
	target = footConstraint;
	target[1] = 0.1;
	p = new PositionConstraint(ik->vars(), mTarget, node, offset, target, weight);
	ik->addObjective(p);

	if (bUseLeftDir)
	{
		weight = 1e-3;
		node = mTarget->getBodyNode("l_foot");
		offset = mTarget->getMarker("l_footLeft")->getLocalPosition();
		target = footConstraint + mLeftGlobal;;
		p = new PositionConstraint(ik->vars(), mTarget, node, offset, target, weight);
		ik->addObjective(p);

	}

}
void SupportState::addRightFootObjective(int frameNum, const Eigen::Vector3d& footConstraint, bool bUseLeftDir, IKProblem* ik)
{
	Eigen::Vector3d offset;
	Eigen::Vector3d target;
	double weight;
	dart::dynamics::BodyNode* node = NULL;
	PositionConstraint* p = NULL;

	weight = 50.0;
	node = mTarget->getBodyNode("r_foot");
	offset = mTarget->getMarker("r_foot")->getLocalPosition();
	p = new PositionConstraint(ik->vars(), mTarget, node, offset, footConstraint, weight);
	ik->addObjective(p);

	weight = 50;
	node = mTarget->getBodyNode("r_foot");
	offset = mTarget->getMarker("r_footUp")->getLocalPosition();
	target = footConstraint;
	target[1] = 0.1;
	p = new PositionConstraint(ik->vars(), mTarget, node, offset, target, weight);
	ik->addObjective(p);

	if (bUseLeftDir)
	{
		weight = 1e-3;
		node = mTarget->getBodyNode("r_foot");
		offset = mTarget->getMarker("r_footLeft")->getLocalPosition();
		target = footConstraint + mLeftGlobal;
		p = new PositionConstraint(ik->vars(), mTarget, node, offset, target, weight);
		ik->addObjective(p);
	}


}

void SupportState::addLeftFootConstraint(int frameNum, const Eigen::Vector3d& footConstraint, bool bUseLeftDir, IKProblem* ik)
{
	Eigen::Vector3d offset;
	Eigen::Vector3d target;
	double weight;
	dart::dynamics::BodyNode* node = NULL;
	PositionConstraint* p = NULL;

	weight = 0.1;
	node = mTarget->getBodyNode("l_foot");
	offset = mTarget->getMarker("l_foot")->getLocalPosition();
	p = new PositionConstraint(ik->vars(), mTarget, node, offset, footConstraint, weight);
	ik->addConstraint(p);

	//weight = 1e-2;
	//node = mTarget->getBodyNode("l_foot");
	//offset = mTarget->getMarker("l_footUp")->getLocalPosition();
	//target = footConstraint;
	//target[1] = 0.1;
	//p = new PositionConstraint(ik->vars(), mTarget, node, offset, target, weight);
	//ik->addConstraint(p);

	weight = 10;
	node = mTarget->getBodyNode("l_foot");
	Eigen::Vector3d offset1 = mTarget->getMarker("l_foot")->getLocalPosition();
	Eigen::Vector3d offset2 = mTarget->getMarker("l_footUp")->getLocalPosition();
	target = Eigen::Vector3d(0, 1, 0);
	DirectionConstraint* pd = new DirectionConstraint(ik->vars(), mTarget, node, offset1, offset2, target, weight);
	ik->addConstraint(pd);
	
	if (bUseLeftDir)
	{
		weight = 1e-2;
		node = mTarget->getBodyNode("l_foot");
		offset = mTarget->getMarker("l_footLeft")->getLocalPosition();
		target = footConstraint + mLeftGlobal;;
		p = new PositionConstraint(ik->vars(), mTarget, node, offset, target, weight);
		ik->addConstraint(p);
		//weight = 0.1;
		//node = mTarget->getBodyNode("l_foot");
		//Eigen::Vector3d offset1 = mTarget->getMarker("l_foot")->getLocalPosition();
		//Eigen::Vector3d offset2 = mTarget->getMarker("l_footLeft")->getLocalPosition();
		//target = mLeftGlobal;
		//DirectionConstraint* pd = new DirectionConstraint(ik->vars(), mTarget, node, offset1, offset2, target, weight);
		//ik->addConstraint(pd);
	}

}
void SupportState::addRightFootConstraint(int frameNum, const Eigen::Vector3d& footConstraint, bool bUseLeftDir, IKProblem* ik)
{
	Eigen::Vector3d offset;
	Eigen::Vector3d target;
	double weight;
	dart::dynamics::BodyNode* node = NULL;
	PositionConstraint* p = NULL;

	weight = 0.1;
	node = mTarget->getBodyNode("r_foot");
	offset = mTarget->getMarker("r_foot")->getLocalPosition();
	p = new PositionConstraint(ik->vars(), mTarget, node, offset, footConstraint, weight);
	ik->addConstraint(p);

	//weight = 1e-2;
	//node = mTarget->getBodyNode("r_foot");
	//offset = mTarget->getMarker("r_footUp")->getLocalPosition();
	//target = footConstraint;
	//target[1] = 0.1;
	//p = new PositionConstraint(ik->vars(), mTarget, node, offset, target, weight);
	//ik->addConstraint(p);
	weight = 10;
	node = mTarget->getBodyNode("r_foot");
	Eigen::Vector3d offset1 = mTarget->getMarker("r_foot")->getLocalPosition();
	Eigen::Vector3d offset2 = mTarget->getMarker("r_footUp")->getLocalPosition();
	target = Eigen::Vector3d(0, 1, 0);
	DirectionConstraint* pd = new DirectionConstraint(ik->vars(), mTarget, node, offset1, offset2, target, weight);
	ik->addConstraint(pd);


	if (bUseLeftDir)
	{
		weight = 1e-2;
		node = mTarget->getBodyNode("r_foot");
		offset = mTarget->getMarker("r_footLeft")->getLocalPosition();
		target = footConstraint + mLeftGlobal;
		p = new PositionConstraint(ik->vars(), mTarget, node, offset, target, weight);
		ik->addConstraint(p);

		//weight = 0.1;
		//node = mTarget->getBodyNode("r_foot");
		//Eigen::Vector3d offset1 = mTarget->getMarker("r_foot")->getLocalPosition();
		//Eigen::Vector3d offset2 = mTarget->getMarker("r_footLeft")->getLocalPosition();
		//target = mLeftGlobal;
		//DirectionConstraint* pd = new DirectionConstraint(ik->vars(), mTarget, node, offset1, offset2, target, weight);
		//ik->addConstraint(pd);
	}


}


void SupportState::addDoubleFootConstraint(int frameNum, const Eigen::Vector3d& leftFootConstraint, const Eigen::Vector3d& rightFootConstraint, IKProblem* ik)
{
	Eigen::Vector3d realLeftFootConstraint = leftFootConstraint;
	Eigen::Vector3d realRightFootConstraint = rightFootConstraint;
	realLeftFootConstraint[1] = realRightFootConstraint[1] = 0;
	addRightFootConstraint(frameNum, realRightFootConstraint, false, ik);

	addLeftFootConstraint(frameNum, realLeftFootConstraint, true, ik);
}

void SupportState::snapshotInitialFootLocations(int frameNum)
{
	if (frameNum == mStartFrame)
	{
		mInitialCOM = mTarget->getWorldCOM();
	}
	mInitialLeftFoot = mOrig->getMarker("lfoot")->getWorldPosition();
	mInitialRightFoot = mOrig->getMarker("rfoot")->getWorldPosition();

}