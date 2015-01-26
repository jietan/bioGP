#ifndef POSE_CONSTRAINT_H
#define POSE_CONSTRAINT_H

#include "dart/optimizer/Constraint.h"
using namespace dart::optimizer;

namespace dart
{
	namespace dynamics {
		class Skeleton;
		class BodyNode;
	}
} // namespace dynamics

class Var;

class PoseConstraint : public Constraint 
{
public:
	PoseConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel);

	virtual Eigen::VectorXd evalCon();
	virtual void fillJac(VVD, int){}
	virtual void fillJac(VVD, VVB, int);
	virtual void fillObjGrad(std::vector<double>&);

	void setTarget(const Eigen::VectorXd& target);
	Eigen::VectorXd getTarget() const;

protected:
	
	Eigen::VectorXd mTarget;
	dart::dynamics::Skeleton* mSkel;
};


#endif // #ifndef POSITION_CONSTRAINT_H

