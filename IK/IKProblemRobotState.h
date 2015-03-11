#ifndef IK_PROBLEM_ROBOT_STATE_H
#define IK_PROBLEM_ROBOT_STATE_H

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
class IKProblemRobotState : public dart::optimizer::Problem {
public:
	IKProblemRobotState(bioloidgp::robot::HumanoidController *skel, const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& markerPos, const vector<int>& isMarkerOccluded, const vector<int>& markersMapping);
	virtual ~IKProblemRobotState();

	void initProblem(bioloidgp::robot::HumanoidController *skel, const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& markerPos, const vector<int>& isMarkerOccluded, const vector<int>& markersMapping);
	virtual void update(double* coefs);
	void verifyConstraint() const;
	dart::dynamics::Skeleton* getSkel() const;
	dart::optimizer::Constraint* getConstraint(int index) const;
	void addConstraint(dart::optimizer::Constraint* c);
	void addObjective(dart::optimizer::Constraint* o);
protected:
	bioloidgp::robot::HumanoidController* mController;
	std::vector<dart::optimizer::Constraint*> mConstraints;
	

};


#endif
