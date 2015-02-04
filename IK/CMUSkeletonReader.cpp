#include "CMUSkeletonReader.h"
#include "CMUSkeleton.h"
#include "dart/dynamics/BodyNode.h"
#include <fstream>


void convertFromCMUBone(Bone* bone, dart::dynamics::BodyNode* body)
{

}

void convertFromCMUSkeleton(CMUSkeleton* cmuSkel, dart::dynamics::Skeleton* dartSkel)
{
	int nBones = cmuSkel->numBonesInSkel(*(cmuSkel->getRoot()));
	for (int i = 0; i < nBones; ++i)
	{
		dart::dynamics::BodyNode* body = new dart::dynamics::BodyNode;
		Bone* bone = cmuSkel->getBone(i);
		convertFromCMUBone(bone, body);
		dartSkel->addBodyNode(body);
	}
}


dart::dynamics::Skeleton* ReadCMUSkeleton(const std::string& filename)
{
	CMUSkeleton cmuSkel(filename.c_str(), 0.06);
	dart::dynamics::Skeleton* ret = new dart::dynamics::Skeleton("mocap");
	convertFromCMUSkeleton(&cmuSkel, ret);
	return ret;
}