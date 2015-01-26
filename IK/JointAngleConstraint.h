#ifndef JOINT_ANGLE_CONSTRAINT_H
#define JOINT_ANGLE_CONSTRAINT_H

#include "optimizer/Constraint.h"

namespace dart::dynamics {
    class Skeleton;
    class BodyNode;
} // namespace kinematics

namespace optimizer {
    class dart::optimizer::Var;

    class JointAngleConstraint : public Constraint {
    public:
        JointAngleConstraint (std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel,
                           const Eigen::VectorXd& target,
                           const Eigen::VectorXi& index,
                           bool isFirst6DofIncluded);
        virtual ~JointAngleConstraint();
        virtual Eigen::VectorXd evalCon();
        virtual void fillJac(VVD, int);
        virtual void fillJac(VVD, VVB, int);
        virtual void fillObjGrad(std::vector<double>&);

        void setTarget(const Eigen::VectorXd& target, const Eigen::VectorXi& index);

    protected:
        Eigen::VectorXd mTarget;
        Eigen::VectorXi mIndex;
        bool mbFirst6DofIncluded;
        dart::dynamics::Skeleton* mSkel;
    };
} // namespace optimizer
    
#endif // #ifndef POSITION_CONSTRAINT_H

