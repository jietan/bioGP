#include "CMUSkeletonReader.h"
#include "CMUSkeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/CylinderShape.h"
#include "myUtils/ConfigManager.h"
#include <fstream>
#include <glog/logging.h>


void convertFromCMUBones(Bone* bone, Bone* parentBone, const Eigen::Isometry3d& parentTransform, dart::dynamics::Skeleton* dartSkel, dart::dynamics::BodyNode* parentNode)
{
	//Skeleton* mSkeleton;
	//Joint* mParentJoint;
	//BodyNode* mParentBodyNode;
	//std::vector<BodyNode*> mChildBodyNodes;
	//std::vector<int> mDependentGenCoordIndices;
	//Eigen::Isometry3d mW;
	if (!dartSkel || !bone)
		return;
	dart::dynamics::BodyNode* body = new dart::dynamics::BodyNode;
	dart::dynamics::Shape* shape = NULL;
	if (parentBone)
	{
		shape = new dart::dynamics::CylinderShape(0.001, bone->length);
		//shape->setOffset(Eigen::Vector3d(0, 0, bone->length / 2));
		body->addVisualizationShape(shape);

	}
	body->setName(bone->name);
	if (!strcmp(bone->name, "lhumerus"))
		printf("hello");
	dart::dynamics::Joint* joint = NULL;
	if (bone->dof == 6)
	{
		joint = new dart::dynamics::FreeJoint;
	}
	else if (bone->dof == 3)
	{
		joint = new dart::dynamics::EulerJoint;
		dart::dynamics::EulerJoint* eulerJoint = static_cast<dart::dynamics::EulerJoint*>(joint);
		eulerJoint->setAxisOrder(dart::dynamics::EulerJoint::AO_ZYX);
	}
	else if (bone->dof == 2)
	{
		joint = new dart::dynamics::UniversalJoint(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
	}
	else if (bone->dof == 1)
	{
		joint = new dart::dynamics::RevoluteJoint;
	}
	else if (bone->dof == 0)
	{
		joint = new dart::dynamics::WeldJoint;
	}
	else
	{
		LOG(FATAL) << "Unsupported joint type.";
	}
	joint->setName(bone->name);

	Eigen::Isometry3d parentWorld = parentTransform;
	Eigen::Isometry3d parentToJoint = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d childWorld = Eigen::Isometry3d::Identity();
	if (parentBone)
	{
		childWorld.translation() = parentWorld.translation() + parentBone->length * Eigen::Vector3d(parentBone->dir[0], parentBone->dir[1], parentBone->dir[2]);
	}
	Eigen::Matrix3d childWorldLinear;
	childWorldLinear = Eigen::AngleAxisd(bone->axis_z / 180.0 * M_PI, Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(bone->axis_y / 180.0 * M_PI, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(bone->axis_x / 180.0 * M_PI, Eigen::Vector3d::UnitX());
	childWorld.linear() = childWorldLinear;
	
	parentToJoint = parentWorld.inverse()*childWorld*childToJoint;
	//childToJoint = (childWorld.inverse() * parentWorld * parentToJoint);
	Eigen::Vector3d dirGlobal(bone->dir[0], bone->dir[1], bone->dir[2]);
	if (parentBone && dirGlobal.norm() > 1e-6)
	{
		Eigen::Vector3d dirLocal = childWorldLinear.inverse() * dirGlobal;
		Eigen::Vector3d dirX(1, 1, 1);
		dirX.normalize();
		Eigen::Vector3d dirY = dirLocal.cross(dirX);
		dirY.normalize();
		dirX = dirY.cross(dirLocal);
		dirX.normalize();
		Eigen::Matrix3d RShape;
		RShape.col(0) = dirX;
		RShape.col(1) = dirY;
		RShape.col(2) = dirLocal;
		Eigen::Isometry3d tShape = Eigen::Isometry3d::Identity();
		tShape.linear() = RShape;
		tShape.translation() = bone->length / 2.0 * dirLocal;
		shape->setLocalTransform(tShape);

	}


	joint->setTransformFromChildBodyNode(childToJoint);
	joint->setTransformFromParentBodyNode(parentToJoint);
	body->setParentJoint(joint);
	if (parentNode)
		parentNode->addChildBodyNode(body);
	dartSkel->addBodyNode(body);
	convertFromCMUBones(bone->sibling, parentBone, parentWorld, dartSkel, parentNode);
	convertFromCMUBones(bone->child, bone, childWorld, dartSkel, body);
}

void convertFromCMUSkeleton(CMUSkeleton* cmuSkel, dart::dynamics::Skeleton* dartSkel)
{
	Bone* node = cmuSkel->getRoot();
	convertFromCMUBones(node, NULL, Eigen::Isometry3d::Identity(), dartSkel, NULL);

}


dart::dynamics::Skeleton* ReadCMUSkeleton(const std::string& filename)
{
	//CMUSkeleton cmuSkel(filename.c_str(), 0.012);
	double mocapScale = 0.06;
	DecoConfig::GetSingleton()->GetDouble("Mocap", "Scale", mocapScale);
	CMUSkeleton cmuSkel(filename.c_str(), mocapScale);
	dart::dynamics::Skeleton* ret = new dart::dynamics::Skeleton("mocap");
	convertFromCMUSkeleton(&cmuSkel, ret);
	return ret;
}