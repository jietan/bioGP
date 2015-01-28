#ifndef RELATIVE_POSITION_CONSTRAINT_H
#define RELATIVE_POSITION_CONSTRAINT_H

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

class RelativePositionConstraint : public Constraint {
public:
	RelativePositionConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel,
						dart::dynamics::BodyNode* node1, const Eigen::Vector3d& offset1,
						dart::dynamics::BodyNode* node2, const Eigen::Vector3d& offset2,
                        double distanceSqr);

    virtual Eigen::VectorXd evalCon();
    
    virtual void fillJac(VVD, VVB, int);

protected: 

    dart::dynamics::Skeleton* mSkel;
    dart::dynamics::BodyNode* mNode1;
	dart::dynamics::BodyNode* mNode2;
	Eigen::Vector3d mOffset1;
	Eigen::Vector3d mOffset2;
	double mDistanceSqr;
};

    
#endif // #ifndef POSITION_CONSTRAINT_H

