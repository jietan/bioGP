#ifndef _COLLISION_SPHERE_H
#define _COLLISION_SPHERE_H

#include <Eigen/Dense>

class CollisionSphere
{
public:
	CollisionSphere(Eigen::Vector3d offset, double radius) : mOffset(offset), mRadius(radius)
	{

	};
	Eigen::Vector3d mOffset;
	double mRadius;
};

#endif