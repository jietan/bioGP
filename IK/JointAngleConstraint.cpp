#include "stdafx.h"
#include "JointAngleConstraint.h"
#include "kinematics/Skeleton.h"
#include "optimizer/Var.h"
#include "utils/UtilsMath.h"

namespace optimizer {
    
    JointAngleConstraint::JointAngleConstraint (std::vector<dart::optimizer::Var *>& var, dart::dynamics::Skeleton* skel,
        const Eigen::VectorXd& target,
        const Eigen::VectorXi& index,
        bool isFirst6DofIncluded)
        : Constraint(var), mSkel(skel), mTarget(target), mIndex(index), mbFirst6DofIncluded(isFirst6DofIncluded)
    {
        mNumRows = mTarget.size();

        mWeight = VectorXd::Ones(mNumRows);
        mConstTerm = VectorXd::Zero(mNumRows);
        mCompletion = VectorXd::Zero(mNumRows);
    }
    JointAngleConstraint::~JointAngleConstraint()
    {

    }
    VectorXd JointAngleConstraint::evalCon() 
    {
        Eigen::VectorXd pose;
        mSkel->getPose(pose);
        VectorXd currentJointAngle = VectorXd::Zero(mIndex.size());
        for (int i = 0; i < mIndex.size(); ++i)
        {
            currentJointAngle[i] = pose[mIndex[i]];
        }
        
        VectorXd C = mWeight.cwiseProduct(currentJointAngle - mTarget);
        
        return C;
    }

    void JointAngleConstraint::fillJac(VVD, int)
    {
    }
    void JointAngleConstraint::fillJac(VVD jEntry, VVB jMap, int index) 
    {
        for (int i = 0; i < mIndex.size(); ++i)
        {
            int dofIndex = mIndex[i];
            if (!mbFirst6DofIncluded && dofIndex >= 6)
                dofIndex -= 6;
            jEntry->at(index + i)->at(dofIndex) = mWeight[i];
            jMap->at(index + i)->at(dofIndex) = true;
        }
    }

    void JointAngleConstraint::fillObjGrad(std::vector<double>& dG) 
    {
        CHECK(0) << "JointAngleConstraint::fillObjGrad() is not implemented yet.";
    }

    void JointAngleConstraint::setTarget(const Eigen::VectorXd& target, const Eigen::VectorXi& index) 
    {
        this->mTarget = target;
        this->mIndex = index;
    }

} // namespace optimizer
