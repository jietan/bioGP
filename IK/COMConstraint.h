#ifndef COM_CONSTRAINT_H
#define COM_CONSTRAINT_H

#include <vector>
using namespace std;

#include "dart/optimizer/Constraint.h"
using namespace dart::optimizer;

namespace dart {
namespace dynamics {
	class Skeleton;
	class BodyNode;
} // namespace kinematics
}


	class Var;

#define COM_CONSTRAINT_X 1
#define COM_CONSTRAINT_Y 2
#define COM_CONSTRAINT_Z 4

class COMConstraint : public Constraint {
public:
	COMConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel);
	virtual Eigen::VectorXd evalCon();
	virtual void fillJac(VVD, int){}
	virtual void fillJac(VVD, VVB, int);
	virtual void fillObjGrad(std::vector<double>&);

	void setTarget(const Eigen::Vector3d& target, int axisMask = 0);
	Eigen::Vector3d getTarget() const;

protected:
	int getNumControlAxis() const;
	vector<int> getControlAxis() const;

	Eigen::Vector3d mTarget;
	dart::dynamics::Skeleton* mSkel;
	int mAxisMask;
};


#endif // #ifndef POSITION_CONSTRAINT_H

