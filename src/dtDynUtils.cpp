/*!
 \file      dtDynUtils.cpp
 \brief     Reusable native dtDynamics model and data utility definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynUtils.h"

#include "dtDynamics/dtDynMath.h"

namespace dt::Dynamics
{
namespace utils
{

dtDynIndex EffectiveRootCount(const dtDynModel &model)
{
    return (model.nroot > 0) ? model.nroot : 1;
}

dtDynIndex RootIndex(const dtDynModel &model, const dtDynIndex bodyId)
{
    if (bodyId <= 0)
    {
        return DTDYN_INVALID_INDEX;
    }
    return model.body_rootidx[static_cast<std::size_t>(bodyId)];
}

dt::Math::Vector3<dtDynReal> RootCom(const dtDynModel &model,
                                     const dtDynData &data,
                                     const dtDynIndex bodyId)
{
    const dtDynIndex rootBody = model.body_rootid[static_cast<std::size_t>(bodyId)];
    if (rootBody > 0)
    {
        return data.subtree_com[static_cast<std::size_t>(rootBody)];
    }
    return data.subtree_com[0];
}

dt::Math::Vector6<dtDynReal> MotionSubspaceAtPoint(const dtDynModel &model,
                                                   const dtDynData &data,
                                                   const dtDynIndex dofId,
                                                   const dt::Math::Vector3<dtDynReal> &point)
{
    const dtDynIndex jointId = model.dof_jntid[static_cast<std::size_t>(dofId)];
    const dtDynIndex dofOffset = dofId - model.jnt_dofadr[static_cast<std::size_t>(jointId)];
    const dtDynJointType jointType =
        static_cast<dtDynJointType>(model.jnt_type[static_cast<std::size_t>(jointId)]);

    dt::Math::Vector3<dtDynReal> angular;
    dt::Math::Vector3<dtDynReal> linear;
    angular.SetZero();
    linear.SetZero();

    if (jointType == dtDynJointType::Hinge)
    {
        angular = data.xaxis[static_cast<std::size_t>(jointId)];
        linear = angular & (point - data.xanchor[static_cast<std::size_t>(jointId)]);
    }
    else if (jointType == dtDynJointType::Slide)
    {
        linear = data.xaxis[static_cast<std::size_t>(jointId)];
    }
    else if (jointType == dtDynJointType::Free)
    {
        const dtDynIndex jointBody = model.jnt_bodyid[static_cast<std::size_t>(jointId)];
        if (dofOffset < 3)
        {
            linear = math::UnitAxis(dofOffset);
        }
        else
        {
            angular = math::MatrixColumn(data.xmat[static_cast<std::size_t>(jointBody)], dofOffset - 3);
            linear = angular & (point - data.xpos[static_cast<std::size_t>(jointBody)]);
        }
    }
    else if (jointType == dtDynJointType::Ball)
    {
        const dtDynIndex jointBody = model.jnt_bodyid[static_cast<std::size_t>(jointId)];
        angular = math::MatrixColumn(data.xmat[static_cast<std::size_t>(jointBody)], dofOffset);
        linear = angular & (point - data.xanchor[static_cast<std::size_t>(jointId)]);
    }
    else
    {
        angular.SetZero();
        linear.SetZero();
    }

    return math::SpatialMotionVector(angular, linear);
}

} // end of namespace utils
} // namespace dt::Dynamics
