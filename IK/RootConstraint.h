#ifndef ROOT_CONSTRAINT_H
#define ROOT_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace dart::dynamics {
	class Skeleton;
	class BodyNode;
} // namespace kinematics

namespace optimizer {
	class dart::optimizer::Var;

	class RootConstraint : public Constraint {
	public:
		RootConstraint (std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel);

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
} // namespace optimizer

#endif // #ifndef POSITION_CONSTRAINT_H

