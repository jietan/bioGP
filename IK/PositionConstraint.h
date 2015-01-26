#ifndef POSITION_CONSTRAINT_H
#define POSITION_CONSTRAINT_H

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

class PositionConstraint : public Constraint {
public:
    PositionConstraint(std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel, dart::dynamics::BodyNode* node,
                        const Eigen::Vector3d& offset,
                        const Eigen::Vector3d& val);

    virtual Eigen::VectorXd evalCon();
    virtual void fillJac(VVD, int){}
    virtual void fillJac(VVD, VVB, int);
    virtual void fillObjGrad(std::vector<double>&);

    void setTarget(const Eigen::Vector3d& target);
    Eigen::Vector3d getTarget() const;

protected:
    Eigen::Vector3d mTarget;
    Eigen::Vector3d mOffset;
	bool mIsFirst6DofsIncluded;

    dart::dynamics::Skeleton* mSkel;
    dart::dynamics::BodyNode* mNode;
};

    
#endif // #ifndef POSITION_CONSTRAINT_H

