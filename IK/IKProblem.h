#ifndef PROBLEM_H
#define PROBLEM_H

#include <vector>
#include "dart/optimizer/Problem.h"
#include "dart/optimizer/Constraint.h"

using namespace std;

namespace dart {
	namespace dynamics {
		class Skeleton;
	} // namespace dynamics
}

namespace bioloidgp {
	namespace robot {
		class HumanoidController;
	}
}
class IKProblem : public dart::optimizer::Problem {
public:
	IKProblem(bioloidgp::robot::HumanoidController *skel, bool bCOMControl, bool bCollisionAvoidance);
	virtual ~IKProblem();

	void initProblem(bioloidgp::robot::HumanoidController *skel, bool bCOMControl, bool bCollisionAvoidance);

	virtual void update(double* coefs);
	void verifyConstraint() const;
	dart::dynamics::Skeleton* getSkel() const;

	dart::optimizer::Constraint* getConstraint(int index) const;
protected:
	bioloidgp::robot::HumanoidController* mController;
	std::vector<dart::optimizer::Constraint*> mConstraints;
	Eigen::VectorXd mInitialPose;

};


#endif
