/*!
 \file      dtDynKinematics.cpp
 \brief     Native dtDynamics kinematics and Jacobian definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynKinematics.h"

#include "dtDynamics/dtDynMath.h"
#include "dtDynamics/dtDynUtils.h"

#include <cassert>

namespace dt::Dynamics
{

void ForwardKinematics(const dtDynModel &model, dtDynData &data)
{
    // MuJoCo-style forward kinematics over the body tree. qpos drives joint
    // transforms, then inertial and site frames are derived from each body pose.
    data.xpos[0].SetZero();
    math::SetIdentityQuaternion(data.xquat[0]);
    data.xmat[0].SetIdentity();
    data.xipos[0].SetZero();
    data.ximat[0].SetIdentity();

    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        dt::Math::Vector3<dtDynReal> xpos;
        dt::Math::Vector4<dtDynReal> xquat;
        const dtDynIndex jntAdr = model.body_jntadr[static_cast<std::size_t>(bodyId)];
        const dtDynIndex jntNum = model.body_jntnum[static_cast<std::size_t>(bodyId)];

        if ((jntNum == 1) &&
            (model.jnt_type[static_cast<std::size_t>(jntAdr)] == static_cast<dtDynIndex>(dtDynJointType::Free)))
        {
            const dtDynIndex qadr = model.jnt_qposadr[static_cast<std::size_t>(jntAdr)];
            xpos.At(0) = data.qpos.At(qadr + 0);
            xpos.At(1) = data.qpos.At(qadr + 1);
            xpos.At(2) = data.qpos.At(qadr + 2);
            xquat.At(0) = data.qpos.At(qadr + 3);
            xquat.At(1) = data.qpos.At(qadr + 4);
            xquat.At(2) = data.qpos.At(qadr + 5);
            xquat.At(3) = data.qpos.At(qadr + 6);
            xquat = math::NormalizeOrIdentity(xquat);
            data.xanchor[static_cast<std::size_t>(jntAdr)] = xpos;
            data.xaxis[static_cast<std::size_t>(jntAdr)] = model.jnt_axis[static_cast<std::size_t>(jntAdr)];
        }
        else
        {
            const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
            const dt::Math::Vector4<dtDynReal> &bodyQuat =
                model.body_quat[static_cast<std::size_t>(bodyId)];

            if (parent > 0)
            {
                xpos = data.xpos[static_cast<std::size_t>(parent)] +
                       math::Rotate(data.xmat[static_cast<std::size_t>(parent)],
                                    model.body_pos[static_cast<std::size_t>(bodyId)]);
                xquat = math::MulQuat(data.xquat[static_cast<std::size_t>(parent)], bodyQuat);
            }
            else
            {
                xpos = model.body_pos[static_cast<std::size_t>(bodyId)];
                xquat = bodyQuat;
            }

            for (dtDynIndex localJointId = 0; localJointId < jntNum; ++localJointId)
            {
                const dtDynIndex jointId = jntAdr + localJointId;
                const dtDynIndex qadr = model.jnt_qposadr[static_cast<std::size_t>(jointId)];
                const dtDynJointType jointType =
                    static_cast<dtDynJointType>(model.jnt_type[static_cast<std::size_t>(jointId)]);

                const dt::Math::Vector3<dtDynReal> xaxis =
                    math::Rotate(xquat, model.jnt_axis[static_cast<std::size_t>(jointId)]);
                const dt::Math::Vector3<dtDynReal> xanchor =
                    xpos + math::Rotate(xquat, model.jnt_pos[static_cast<std::size_t>(jointId)]);

                if (jointType == dtDynJointType::Slide)
                {
                    xpos = xpos + xaxis * data.qpos.At(qadr);
                }
                else if ((jointType == dtDynJointType::Hinge) || (jointType == dtDynJointType::Ball))
                {
                    dt::Math::Vector4<dtDynReal> qloc;
                    if (jointType == dtDynJointType::Ball)
                    {
                        qloc.At(0) = data.qpos.At(qadr + 0);
                        qloc.At(1) = data.qpos.At(qadr + 1);
                        qloc.At(2) = data.qpos.At(qadr + 2);
                        qloc.At(3) = data.qpos.At(qadr + 3);
                        qloc = math::NormalizeOrIdentity(qloc);
                    }
                    else
                    {
                        qloc = math::AxisAngleQuat(model.jnt_axis[static_cast<std::size_t>(jointId)], data.qpos.At(qadr));
                    }

                    xquat = math::MulQuat(xquat, qloc);
                    xpos = xanchor -
                           math::Rotate(xquat, model.jnt_pos[static_cast<std::size_t>(jointId)]);
                }

                data.xanchor[static_cast<std::size_t>(jointId)] = xanchor;
                data.xaxis[static_cast<std::size_t>(jointId)] = xaxis;
            }
        }

        data.xpos[static_cast<std::size_t>(bodyId)] = xpos;
        data.xquat[static_cast<std::size_t>(bodyId)] = xquat;
        data.xmat[static_cast<std::size_t>(bodyId)] = math::QuatToMat(xquat);

        data.xipos[static_cast<std::size_t>(bodyId)] =
            xpos + math::Rotate(data.xmat[static_cast<std::size_t>(bodyId)],
                                model.body_ipos[static_cast<std::size_t>(bodyId)]);
        data.ximat[static_cast<std::size_t>(bodyId)] =
            data.xmat[static_cast<std::size_t>(bodyId)] *
            model.body_imat[static_cast<std::size_t>(bodyId)];
    }

    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        const dtDynIndex bodyId = model.site_bodyid[static_cast<std::size_t>(siteId)];
        data.site_xpos[static_cast<std::size_t>(siteId)] =
            data.xpos[static_cast<std::size_t>(bodyId)] +
            math::Rotate(data.xmat[static_cast<std::size_t>(bodyId)],
                         model.site_pos[static_cast<std::size_t>(siteId)]);
        data.site_xmat[static_cast<std::size_t>(siteId)] =
            data.xmat[static_cast<std::size_t>(bodyId)] *
            model.site_mat[static_cast<std::size_t>(siteId)];
    }
}

namespace
{

void PrepareJacobianMatrix(dtDynDenseMatrix *matrix,
                           const dtDynIndex nv,
                           const bool resetOutput)
{
    if (matrix == nullptr)
    {
        return;
    }

    if (resetOutput)
    {
        EnsureDenseMatrixSize(*matrix, 3, nv);
        matrix->SetZero();
    }
}

struct PointJacobianDotContext
{
    dt::Math::Vector3<dtDynReal> rootToPoint;
    dt::Math::Vector3<dtDynReal> pointVelocity;
};

PointJacobianDotContext MakePointJacobianDotContext(const dtDynModel &model,
                                                    const dtDynData &data,
                                                    const dtDynIndex bodyId,
                                                    const dt::Math::Vector3<dtDynReal> &point)
{
    // The point Jacobian derivative uses body velocity expressed at the root
    // CoM reference, then shifts the linear term to the requested world point.
    PointJacobianDotContext context;
    const dt::Math::Vector3<dtDynReal> rootCom = utils::RootCom(model, data, bodyId);
    context.rootToPoint = point - rootCom;

    dt::Math::Vector3<dtDynReal> bodyAngularVelocity;
    dt::Math::Vector3<dtDynReal> bodyLinearVelocityAtRoot;
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        bodyAngularVelocity.At(row) = data.cvel[static_cast<std::size_t>(bodyId)].At(row);
        bodyLinearVelocityAtRoot.At(row) = data.cvel[static_cast<std::size_t>(bodyId)].At(row + 3);
    }
    context.pointVelocity =
        bodyLinearVelocityAtRoot - (context.rootToPoint & bodyAngularVelocity);
    return context;
}

void ComputePointJacobianDotColumn(const dtDynModel &model,
                                   const dtDynData &data,
                                   const dtDynIndex dofId,
                                   const PointJacobianDotContext &context,
                                   dt::Math::Vector3<dtDynReal> &linearDot,
                                   dt::Math::Vector3<dtDynReal> &angularDot)
{
    // Column derivative for a point p:
    //   Jw_dot = cdof_dot.angular
    //   Jv_dot = cdof_dot.linear_at_root + Jw_dot x (p - rootCom)
    //            + Jw x pointVelocity
    const dtDynIndex dofBody = model.dof_bodyid[static_cast<std::size_t>(dofId)];
    const dtDynIndex jointId = model.dof_jntid[static_cast<std::size_t>(dofId)];
    const dtDynIndex dofOffset = dofId - model.jnt_dofadr[static_cast<std::size_t>(jointId)];
    const dtDynJointType jointType =
        static_cast<dtDynJointType>(model.jnt_type[static_cast<std::size_t>(jointId)]);
    dt::Math::Vector6<dtDynReal> cdofDot = data.cdof_dot[static_cast<std::size_t>(dofId)];
    if ((jointType == dtDynJointType::Ball) ||
        ((jointType == dtDynJointType::Free) && (dofOffset >= 3)))
    {
        cdofDot = math::CrossSpatialMotion(data.cvel[static_cast<std::size_t>(dofBody)],
                                           data.cdof[static_cast<std::size_t>(dofId)]);
    }

    dt::Math::Vector3<dtDynReal> angularColumn;
    dt::Math::Vector3<dtDynReal> linearDotAtRoot;
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        angularColumn.At(row) = data.cdof[static_cast<std::size_t>(dofId)].At(row);
        angularDot.At(row) = cdofDot.At(row);
        linearDotAtRoot.At(row) = cdofDot.At(row + 3);
    }

    linearDot =
        linearDotAtRoot +
        (angularDot & context.rootToPoint) +
        (angularColumn & context.pointVelocity);
}

void ComputePointJacobianTermsImpl(const dtDynModel &model,
                                   const dtDynData &data,
                                   const dtDynIndex bodyId,
                                   const dt::Math::Vector3<dtDynReal> &point,
                                   dtDynDenseMatrix *jacp,
                                   dtDynDenseMatrix *jacr,
                                   dtDynDenseMatrix *jacpDot,
                                   dtDynDenseMatrix *jacrDot,
                                   dt::Math::Vector3<dtDynReal> *linearJdotQvel,
                                   dt::Math::Vector3<dtDynReal> *angularJdotQvel,
                                   const bool resetMatrixOutputs)
{
    // Shared task-Jacobian traversal. Only ancestor DOFs affect the requested
    // body-fixed point, so output matrices are filled along that sparse chain.
    PrepareJacobianMatrix(jacp, model.nv, resetMatrixOutputs);
    PrepareJacobianMatrix(jacr, model.nv, resetMatrixOutputs);
    PrepareJacobianMatrix(jacpDot, model.nv, resetMatrixOutputs);
    PrepareJacobianMatrix(jacrDot, model.nv, resetMatrixOutputs);

    if (linearJdotQvel != nullptr)
    {
        linearJdotQvel->SetZero();
    }
    if (angularJdotQvel != nullptr)
    {
        angularJdotQvel->SetZero();
    }

    const bool needJacobian = (jacp != nullptr) || (jacr != nullptr);
    const bool needDerivative =
        (jacpDot != nullptr) || (jacrDot != nullptr) ||
        (linearJdotQvel != nullptr) || (angularJdotQvel != nullptr);

    PointJacobianDotContext dotContext;
    if (needDerivative)
    {
        dotContext = MakePointJacobianDotContext(model, data, bodyId, point);
    }

    dtDynIndex dofId = math::FindBodyLastDof(model, bodyId);
    while (dofId >= 0)
    {
        if (needJacobian)
        {
            const dt::Math::Vector6<dtDynReal> motion =
                utils::MotionSubspaceAtPoint(model, data, dofId, point);
            for (dtDynIndex row = 0; row < 3; ++row)
            {
                if (jacp != nullptr)
                {
                    (*jacp).At(row, dofId) = motion.At(row + 3);
                }
                if (jacr != nullptr)
                {
                    (*jacr).At(row, dofId) = motion.At(row);
                }
            }
        }

        if (needDerivative)
        {
            dt::Math::Vector3<dtDynReal> linearDot;
            dt::Math::Vector3<dtDynReal> angularDot;
            ComputePointJacobianDotColumn(model, data, dofId, dotContext, linearDot, angularDot);
            const dtDynReal velocity = data.qvel.At(dofId);
            for (dtDynIndex row = 0; row < 3; ++row)
            {
                if (jacpDot != nullptr)
                {
                    (*jacpDot).At(row, dofId) = linearDot.At(row);
                }
                if (jacrDot != nullptr)
                {
                    (*jacrDot).At(row, dofId) = angularDot.At(row);
                }
                if (linearJdotQvel != nullptr)
                {
                    (*linearJdotQvel).At(row) += linearDot.At(row) * velocity;
                }
                if (angularJdotQvel != nullptr)
                {
                    (*angularJdotQvel).At(row) += angularDot.At(row) * velocity;
                }
            }
        }

        dofId = model.dof_parentid[static_cast<std::size_t>(dofId)];
    }
}

} // namespace

void ComputeBodyJacobian(const dtDynModel &model,
                         const dtDynData &data,
                         const dtDynIndex bodyId,
                         dtDynDenseMatrix &jacp,
                         dtDynDenseMatrix &jacr)
{
    ComputePointJacobian(model, data, bodyId, data.xpos[static_cast<std::size_t>(bodyId)], jacp, jacr);
}

void ComputeSiteJacobian(const dtDynModel &model,
                         const dtDynData &data,
                         const dtDynIndex siteId,
                         dtDynDenseMatrix &jacp,
                         dtDynDenseMatrix &jacr)
{
    const dtDynIndex bodyId = model.site_bodyid[static_cast<std::size_t>(siteId)];
    ComputePointJacobian(model, data, bodyId, data.site_xpos[static_cast<std::size_t>(siteId)], jacp, jacr);
}

void ComputePointJacobian(const dtDynModel &model,
                          const dtDynData &data,
                          const dtDynIndex bodyId,
                          const dt::Math::Vector3<dtDynReal> &point,
                          dtDynDenseMatrix &jacp,
                          dtDynDenseMatrix &jacr)
{
    ComputePointJacobianTermsImpl(model, data, bodyId, point,
                                  &jacp, &jacr, nullptr, nullptr, nullptr, nullptr, true);
}

void ComputePointJacobianDot(const dtDynModel &model,
                             const dtDynData &data,
                             const dtDynIndex bodyId,
                             const dt::Math::Vector3<dtDynReal> &point,
                             dtDynDenseMatrix &jacpDot,
                             dtDynDenseMatrix &jacrDot)
{
    ComputePointJacobianTermsImpl(model, data, bodyId, point,
                                  nullptr, nullptr, &jacpDot, &jacrDot, nullptr, nullptr, true);
}

void ComputeBodyJacobianDot(const dtDynModel &model,
                            const dtDynData &data,
                            const dtDynIndex bodyId,
                            dtDynDenseMatrix &jacpDot,
                            dtDynDenseMatrix &jacrDot)
{
    ComputePointJacobianDot(model, data, bodyId,
                            data.xpos[static_cast<std::size_t>(bodyId)],
                            jacpDot, jacrDot);
}

void ComputeSiteJacobianDot(const dtDynModel &model,
                            const dtDynData &data,
                            const dtDynIndex siteId,
                            dtDynDenseMatrix &jacpDot,
                            dtDynDenseMatrix &jacrDot)
{
    const dtDynIndex bodyId = model.site_bodyid[static_cast<std::size_t>(siteId)];
    ComputePointJacobianDot(model, data, bodyId,
                            data.site_xpos[static_cast<std::size_t>(siteId)],
                            jacpDot, jacrDot);
}

void ComputePointJacobianDotTimesVelocity(const dtDynModel &model,
                                          const dtDynData &data,
                                          const dtDynIndex bodyId,
                                          const dt::Math::Vector3<dtDynReal> &point,
                                          dt::Math::Vector3<dtDynReal> &linear,
                                          dt::Math::Vector3<dtDynReal> &angular)
{
    ComputePointJacobianTermsImpl(model, data, bodyId, point,
                                  nullptr, nullptr, nullptr, nullptr, &linear, &angular, true);
}

void ComputePointJacobianTerms(const dtDynModel &model,
                               const dtDynData &data,
                               const dtDynIndex bodyId,
                               const dt::Math::Vector3<dtDynReal> &point,
                               dtDynDenseMatrix *jacp,
                               dtDynDenseMatrix *jacr,
                               dtDynDenseMatrix *jacpDot,
                               dtDynDenseMatrix *jacrDot,
                               dt::Math::Vector3<dtDynReal> *linearJdotQvel,
                               dt::Math::Vector3<dtDynReal> *angularJdotQvel)
{
    ComputePointJacobianTermsImpl(model, data, bodyId, point,
                                  jacp, jacr, jacpDot, jacrDot,
                                  linearJdotQvel, angularJdotQvel, true);
}

void ComputeBodyJacobianTerms(const dtDynModel &model,
                              const dtDynData &data,
                              const dtDynIndex bodyId,
                              dtDynDenseMatrix *jacp,
                              dtDynDenseMatrix *jacr,
                              dtDynDenseMatrix *jacpDot,
                              dtDynDenseMatrix *jacrDot,
                              dt::Math::Vector3<dtDynReal> *linearJdotQvel,
                              dt::Math::Vector3<dtDynReal> *angularJdotQvel)
{
    ComputePointJacobianTerms(model,
                              data,
                              bodyId,
                              data.xpos[static_cast<std::size_t>(bodyId)],
                              jacp,
                              jacr,
                              jacpDot,
                              jacrDot,
                              linearJdotQvel,
                              angularJdotQvel);
}

void ComputeSiteJacobianTerms(const dtDynModel &model,
                              const dtDynData &data,
                              const dtDynIndex siteId,
                              dtDynDenseMatrix *jacp,
                              dtDynDenseMatrix *jacr,
                              dtDynDenseMatrix *jacpDot,
                              dtDynDenseMatrix *jacrDot,
                              dt::Math::Vector3<dtDynReal> *linearJdotQvel,
                              dt::Math::Vector3<dtDynReal> *angularJdotQvel)
{
    const dtDynIndex bodyId = model.site_bodyid[static_cast<std::size_t>(siteId)];
    ComputePointJacobianTerms(model,
                              data,
                              bodyId,
                              data.site_xpos[static_cast<std::size_t>(siteId)],
                              jacp,
                              jacr,
                              jacpDot,
                              jacrDot,
                              linearJdotQvel,
                              angularJdotQvel);
}

void ComputeBodyJacobianDotTimesVelocity(const dtDynModel &model,
                                         const dtDynData &data,
                                         const dtDynIndex bodyId,
                                         dt::Math::Vector3<dtDynReal> &linear,
                                         dt::Math::Vector3<dtDynReal> &angular)
{
    ComputePointJacobianDotTimesVelocity(model, data, bodyId,
                                         data.xpos[static_cast<std::size_t>(bodyId)],
                                         linear, angular);
}

void ComputeSiteJacobianDotTimesVelocity(const dtDynModel &model,
                                         const dtDynData &data,
                                         const dtDynIndex siteId,
                                         dt::Math::Vector3<dtDynReal> &linear,
                                         dt::Math::Vector3<dtDynReal> &angular)
{
    const dtDynIndex bodyId = model.site_bodyid[static_cast<std::size_t>(siteId)];
    ComputePointJacobianDotTimesVelocity(model, data, bodyId,
                                         data.site_xpos[static_cast<std::size_t>(siteId)],
                                         linear, angular);
}

namespace
{

void ComputePointJacobiansIntoData(const dtDynModel &model,
                                   const dtDynData &data,
                                   const dtDynIndex bodyId,
                                   const dt::Math::Vector3<dtDynReal> &point,
                                   dtDynDenseMatrix &jacp,
                                   dtDynDenseMatrix &jacr,
                                   dtDynDenseMatrix &jacpDot,
                                   dtDynDenseMatrix &jacrDot)
{
    // Data-backed Jacobian writer used by all-body/all-site and indexed paths.
    // It reuses the same point equations as the public one-off APIs.
    const dt::Math::Vector3<dtDynReal> rootCom = utils::RootCom(model, data, bodyId);
    const dt::Math::Vector3<dtDynReal> rootToPoint = point - rootCom;

    dt::Math::Vector3<dtDynReal> bodyAngularVelocity;
    dt::Math::Vector3<dtDynReal> bodyLinearVelocityAtRoot;
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        bodyAngularVelocity.At(row) = data.cvel[static_cast<std::size_t>(bodyId)].At(row);
        bodyLinearVelocityAtRoot.At(row) = data.cvel[static_cast<std::size_t>(bodyId)].At(row + 3);
    }
    const dt::Math::Vector3<dtDynReal> pointVelocity =
        bodyLinearVelocityAtRoot - (rootToPoint & bodyAngularVelocity);

    dtDynIndex dofId = math::FindBodyLastDof(model, bodyId);
    while (dofId >= 0)
    {
        const dt::Math::Vector6<dtDynReal> motion =
            utils::MotionSubspaceAtPoint(model, data, dofId, point);

        const dtDynIndex dofBody = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const dtDynIndex jointId = model.dof_jntid[static_cast<std::size_t>(dofId)];
        const dtDynIndex dofOffset = dofId - model.jnt_dofadr[static_cast<std::size_t>(jointId)];
        const dtDynJointType jointType =
            static_cast<dtDynJointType>(model.jnt_type[static_cast<std::size_t>(jointId)]);
        dt::Math::Vector6<dtDynReal> cdofDot = data.cdof_dot[static_cast<std::size_t>(dofId)];
        if ((jointType == dtDynJointType::Ball) ||
            ((jointType == dtDynJointType::Free) && (dofOffset >= 3)))
        {
            cdofDot = math::CrossSpatialMotion(data.cvel[static_cast<std::size_t>(dofBody)],
                                               data.cdof[static_cast<std::size_t>(dofId)]);
        }

        dt::Math::Vector3<dtDynReal> angular;
        dt::Math::Vector3<dtDynReal> linearDot;
        dt::Math::Vector3<dtDynReal> angularDot;
        dt::Math::Vector3<dtDynReal> linearDotAtRoot;
        for (dtDynIndex row = 0; row < 3; ++row)
        {
            angular.At(row) = data.cdof[static_cast<std::size_t>(dofId)].At(row);
            angularDot.At(row) = cdofDot.At(row);
            linearDotAtRoot.At(row) = cdofDot.At(row + 3);
        }
        linearDot = linearDotAtRoot + (angularDot & rootToPoint) + (angular & pointVelocity);

        for (dtDynIndex row = 0; row < 3; ++row)
        {
            jacp.At(row, dofId) = motion.At(row + 3);
            jacr.At(row, dofId) = motion.At(row);
            jacpDot.At(row, dofId) = linearDot.At(row);
            jacrDot.At(row, dofId) = angularDot.At(row);
        }

        dofId = model.dof_parentid[static_cast<std::size_t>(dofId)];
    }
}

void ComputeBodyJacobiansIntoData(const dtDynModel &model,
                                  dtDynData &data,
                                  const dtDynIndex bodyId)
{
    ComputePointJacobiansIntoData(model,
                                  data,
                                  bodyId,
                                  data.xpos[static_cast<std::size_t>(bodyId)],
                                  data.body_jacp[static_cast<std::size_t>(bodyId)],
                                  data.body_jacr[static_cast<std::size_t>(bodyId)],
                                  data.body_jacp_dot[static_cast<std::size_t>(bodyId)],
                                  data.body_jacr_dot[static_cast<std::size_t>(bodyId)]);
}

void ComputeSiteJacobiansIntoData(const dtDynModel &model,
                                  dtDynData &data,
                                  const dtDynIndex siteId)
{
    const dtDynIndex bodyId = model.site_bodyid[static_cast<std::size_t>(siteId)];
    ComputePointJacobiansIntoData(model,
                                  data,
                                  bodyId,
                                  data.site_xpos[static_cast<std::size_t>(siteId)],
                                  data.site_jacp[static_cast<std::size_t>(siteId)],
                                  data.site_jacr[static_cast<std::size_t>(siteId)],
                                  data.site_jacp_dot[static_cast<std::size_t>(siteId)],
                                  data.site_jacr_dot[static_cast<std::size_t>(siteId)]);
}

void ComputeAllBodyJacobians(const dtDynModel &model, dtDynData &data)
{
    // Full-model diagnostic path: compute body-origin Jacobians for every body.
    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        ComputeBodyJacobiansIntoData(model, data, bodyId);
    }
}

void ComputeAllSiteJacobians(const dtDynModel &model, dtDynData &data)
{
    // Full-model diagnostic path: compute site Jacobians for every site.
    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        ComputeSiteJacobiansIntoData(model, data, siteId);
    }
}

void ComputeIndexedBodyJacobians(const dtDynModel &model,
                                 dtDynData &data,
                                 const dtDynArray<dtDynIndex> &bodyIds)
{
    // Controller path: compute only named task bodies cached by the caller.
    const std::size_t bodyCount = bodyIds.size();
    for (std::size_t idx = 0; idx < bodyCount; ++idx)
    {
        const dtDynIndex bodyId = bodyIds[idx];
        assert((bodyId >= 0) && (bodyId < model.nbody));
        ComputeBodyJacobiansIntoData(model, data, bodyId);
    }
}

void ComputeIndexedSiteJacobians(const dtDynModel &model,
                                 dtDynData &data,
                                 const dtDynArray<dtDynIndex> &siteIds)
{
    // Controller path: compute only named task sites cached by the caller.
    const std::size_t siteCount = siteIds.size();
    for (std::size_t idx = 0; idx < siteCount; ++idx)
    {
        const dtDynIndex siteId = siteIds[idx];
        assert((siteId >= 0) && (siteId < model.nsite));
        ComputeSiteJacobiansIntoData(model, data, siteId);
    }
}

} // namespace

void ComputeBodyJacobians(const dtDynModel &model, dtDynData &data)
{
    assert(data.IsCompatible(model));
    ComputeAllBodyJacobians(model, data);
}

void ComputeSiteJacobians(const dtDynModel &model, dtDynData &data)
{
    assert(data.IsCompatible(model));
    ComputeAllSiteJacobians(model, data);
}

void ComputeJacobians(const dtDynModel &model, dtDynData &data)
{
    assert(data.IsCompatible(model));
    ComputeAllBodyJacobians(model, data);
    ComputeAllSiteJacobians(model, data);
}

void ComputeBodyJacobians(const dtDynModel &model,
                          dtDynData &data,
                          const dtDynArray<dtDynIndex> &bodyIds)
{
    assert(data.IsCompatible(model));
    ComputeIndexedBodyJacobians(model, data, bodyIds);
}

void ComputeSiteJacobians(const dtDynModel &model,
                          dtDynData &data,
                          const dtDynArray<dtDynIndex> &siteIds)
{
    assert(data.IsCompatible(model));
    ComputeIndexedSiteJacobians(model, data, siteIds);
}

void ComputeJacobians(const dtDynModel &model,
                      dtDynData &data,
                      const dtDynArray<dtDynIndex> &bodyIds,
                      const dtDynArray<dtDynIndex> &siteIds)
{
    assert(data.IsCompatible(model));
    ComputeIndexedBodyJacobians(model, data, bodyIds);
    ComputeIndexedSiteJacobians(model, data, siteIds);
}

} // namespace dt::Dynamics
