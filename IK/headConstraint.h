#ifndef HEAD_CONSTRAINT_H
#define HEAD_CONSTRAINT_H

#include "dart/optimizer/Constraint.h"
using namespace dart::optimizer;

namespace dart
{
namespace dynamics {
	class Skeleton;
	class BodyNode;
} // namespace dynamics
}

class dart::optimizer::Var;

class HeadConstraint : public Constraint {
public:
	HeadConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel, dart::dynamics::BodyNode* node,
		const Eigen::Vector3d& offset,
		const Eigen::Vector3d& globalUp,
		const Eigen::Vector3d& globalFront);

	virtual Eigen::VectorXd evalCon();
	virtual void fillJac(VVD, int){}
	virtual void fillJac(VVD, VVB, int);
	virtual void fillObjGrad(std::vector<double>&);

	void setPositionTarget(const Eigen::Vector3d& target);
	void setFrontTarget(const Eigen::Vector3d& target);
	void SetUpTarget(const Eigen::Vector3d& target);
		

protected:
	Eigen::Vector3d mTargetPos;
	Eigen::Vector3d mTargetUp;
	Eigen::Vector3d mTargetFront;
	Eigen::Vector3d mOffset;
	Eigen::Vector3d mLocalUp;
	Eigen::Vector3d mLocalFront;

	dart::dynamics::Skeleton* mSkel;
	dart::dynamics::BodyNode* mNode;
};


#endif // #ifndef POSITION_CONSTRAINT_H

