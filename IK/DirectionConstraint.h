#ifndef _DIRECTION_CONSTRAINT_H
#define _DIRECTION_CONSTRAINT_H


#include "dart/optimizer/Constraint.h"
using namespace dart::optimizer;
#include <Eigen/Dense>

namespace dart
{
	namespace dynamics {
		class Skeleton;
		class BodyNode;
	} // namespace dynamics
}

class dart::optimizer::Var;

class DirectionConstraint : public Constraint {
public:
	DirectionConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel, dart::dynamics::BodyNode* node,
		const Eigen::Vector3d& offset1,
		const Eigen::Vector3d& offset2,
		const Eigen::Vector3d& val,
		double weight = 1);

	virtual Eigen::VectorXd evalCon();
	virtual void fillJac(VVD, int){}
	virtual void fillJac(VVD, VVB, int);

	void setTarget(const Eigen::Vector3d& target);
	Eigen::Vector3d getTarget() const;

protected:
	Eigen::Vector3d mTargetDir;
	Eigen::Vector3d mOffset1;
	Eigen::Vector3d mOffset2;

	dart::dynamics::Skeleton* mSkel;
	dart::dynamics::BodyNode* mNode;
};


#endif