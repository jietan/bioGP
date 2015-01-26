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


class PositionConstraint;

class IKProblem : public dart::optimizer::Problem {
public:
	IKProblem(dart::dynamics::Skeleton *skel, bool bCOMControl);
	virtual ~IKProblem();

	void initProblem(dart::dynamics::Skeleton *skel, bool bCOMControl);

	virtual void update(double* coefs);
	void verifyConstraint() const;
	dart::dynamics::Skeleton* getSkel() const;

	dart::optimizer::Constraint* getConstraint(int index) const;
protected:
	dart::dynamics::Skeleton* mSkel;
	std::vector<dart::optimizer::Constraint*> mConstraints;
	Eigen::VectorXd mInitialPose;

};


#endif
