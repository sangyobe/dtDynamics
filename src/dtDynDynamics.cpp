/*!
 \file      dtDynDynamics.cpp
 \brief     Native dtDynamics dynamics ingredient definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynDynamics.h"

#include "dtDynamics/dtDynKinematics.h"
#include "dtDynamics/dtDynMath.h"
#include "dtDynamics/dtDynUtils.h"

#include <cassert>
#include <limits>

namespace dt::Dynamics
{
namespace detail
{

bool IsZeroSpatial(const dt::Math::Vector6<dtDynReal> &value);

void AddScaledSpatialInPlace(dt::Math::Vector6<dtDynReal> &dst,
                             const dt::Math::Vector6<dtDynReal> &src,
                             const dtDynReal scale)
{
    // In-place form of dst += scale * src used in recursive spatial velocity
    // and acceleration accumulation.
    if (scale == 0.0)
    {
        return;
    }

    for (dtDynIndex row = 0; row < 6; ++row)
    {
        dst.At(row) += src.At(row) * scale;
    }
}

void CrossSpatialMotionInto(const dt::Math::Vector6<dtDynReal> &velocity,
                            const dt::Math::Vector6<dtDynReal> &motion,
                            dt::Math::Vector6<dtDynReal> &out)
{
    // Computes out = crm(velocity) * motion without allocating the 6x6 crm matrix.
    const dtDynReal wx = velocity.At(0);
    const dtDynReal wy = velocity.At(1);
    const dtDynReal wz = velocity.At(2);
    const dtDynReal vx = velocity.At(3);
    const dtDynReal vy = velocity.At(4);
    const dtDynReal vz = velocity.At(5);

    const dtDynReal mx = motion.At(0);
    const dtDynReal my = motion.At(1);
    const dtDynReal mz = motion.At(2);
    const dtDynReal ml0 = motion.At(3);
    const dtDynReal ml1 = motion.At(4);
    const dtDynReal ml2 = motion.At(5);

    out.At(0) = (wy * mz) - (wz * my);
    out.At(1) = (wz * mx) - (wx * mz);
    out.At(2) = (wx * my) - (wy * mx);
    out.At(3) = (((wy * ml2) - (wz * ml1)) + (vy * mz)) - (vz * my);
    out.At(4) = (((wz * ml0) - (wx * ml2)) + (vz * mx)) - (vx * mz);
    out.At(5) = (((wx * ml1) - (wy * ml0)) + (vx * my)) - (vy * mx);
}

void SetAngularPointMotionSubspace(const dtDynReal ax,
                                   const dtDynReal ay,
                                   const dtDynReal az,
                                   const dt::Math::Vector3<dtDynReal> &point,
                                   const dt::Math::Vector3<dtDynReal> &anchor,
                                   dt::Math::Vector6<dtDynReal> &out)
{
    // A rotational DOF contributes angular axis a and linear velocity a x (p - anchor).
    const dtDynReal dx = point.At(0) - anchor.At(0);
    const dtDynReal dy = point.At(1) - anchor.At(1);
    const dtDynReal dz = point.At(2) - anchor.At(2);

    out.At(0) = ax;
    out.At(1) = ay;
    out.At(2) = az;
    out.At(3) = ay * dz - az * dy;
    out.At(4) = az * dx - ax * dz;
    out.At(5) = ax * dy - ay * dx;
}

void SetMotionSubspaceAtPoint(const dtDynModel &model,
                              const dtDynData &data,
                              const dtDynIndex dofId,
                              const dt::Math::Vector3<dtDynReal> &point,
                              dt::Math::Vector6<dtDynReal> &out)
{
    // Writes the MuJoCo-order motion subspace column cdof for one scalar qvel.
    // Rotational columns are shifted to the supplied point; translational
    // free-joint columns remain pure linear motion.
    const dtDynIndex jointId = model.dof_jntid[static_cast<std::size_t>(dofId)];
    const dtDynIndex dofOffset = dofId - model.jnt_dofadr[static_cast<std::size_t>(jointId)];
    const dtDynJointType jointType =
        static_cast<dtDynJointType>(model.jnt_type[static_cast<std::size_t>(jointId)]);

    out.SetZero();
    if (jointType == dtDynJointType::Hinge)
    {
        const dt::Math::Vector3<dtDynReal> &axis = data.xaxis[static_cast<std::size_t>(jointId)];
        SetAngularPointMotionSubspace(axis.At(0),
                                      axis.At(1),
                                      axis.At(2),
                                      point,
                                      data.xanchor[static_cast<std::size_t>(jointId)],
                                      out);
    }
    else if (jointType == dtDynJointType::Slide)
    {
        const dt::Math::Vector3<dtDynReal> &axis = data.xaxis[static_cast<std::size_t>(jointId)];
        out.At(3) = axis.At(0);
        out.At(4) = axis.At(1);
        out.At(5) = axis.At(2);
    }
    else if (jointType == dtDynJointType::Free)
    {
        const dtDynIndex jointBody = model.jnt_bodyid[static_cast<std::size_t>(jointId)];
        if (dofOffset < 3)
        {
            out.At(3 + dofOffset) = 1.0;
        }
        else
        {
            const dtDynIndex col = dofOffset - 3;
            const dt::Math::Matrix3<dtDynReal> &bodyMat =
                data.xmat[static_cast<std::size_t>(jointBody)];
            SetAngularPointMotionSubspace(bodyMat.At(0, col),
                                          bodyMat.At(1, col),
                                          bodyMat.At(2, col),
                                          point,
                                          data.xpos[static_cast<std::size_t>(jointBody)],
                                          out);
        }
    }
    else if (jointType == dtDynJointType::Ball)
    {
        const dtDynIndex jointBody = model.jnt_bodyid[static_cast<std::size_t>(jointId)];
        const dt::Math::Matrix3<dtDynReal> &bodyMat =
            data.xmat[static_cast<std::size_t>(jointBody)];
        SetAngularPointMotionSubspace(bodyMat.At(0, dofOffset),
                                      bodyMat.At(1, dofOffset),
                                      bodyMat.At(2, dofOffset),
                                      point,
                                      data.xanchor[static_cast<std::size_t>(jointId)],
                                      out);
    }
}

void ComputeDofMotionSubspaces(const dtDynModel &model, dtDynData &data)
{
    // cdof columns are expressed at each root CoM, matching the CCRBA centroidal
    // reference point used by f1/f2/f3 and the centroidal outputs.
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        const std::size_t dofIdx = static_cast<std::size_t>(dofId);
        const dtDynIndex bodyId = model.dof_bodyid[dofIdx];
        const dtDynIndex rootBody = model.body_rootid[static_cast<std::size_t>(bodyId)];
        const dt::Math::Vector3<dtDynReal> &rootCom =
            data.subtree_com[static_cast<std::size_t>((rootBody > 0) ? rootBody : 0)];
        SetMotionSubspaceAtPoint(model, data, dofId, rootCom, data.cdof[dofIdx]);
    }
}

void ComputeSubtreeCom(const dtDynModel &model, dtDynData &data)
{
    // Two-pass subtree CoM:
    //   weightedCom_i = m_i * xipos_i + sum_child weightedCom_child
    //   subtree_com_i = weightedCom_i / subtree_mass_i
    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        data.scratch_weightedCom[static_cast<std::size_t>(bodyId)] =
            data.xipos[static_cast<std::size_t>(bodyId)] * model.body_mass.At(bodyId);
    }

    for (dtDynIndex bodyId = model.nbody - 1; bodyId > 0; --bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        if (parent >= 0)
        {
            data.scratch_weightedCom[static_cast<std::size_t>(parent)] =
                data.scratch_weightedCom[static_cast<std::size_t>(parent)] +
                data.scratch_weightedCom[static_cast<std::size_t>(bodyId)];
        }
    }

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        if (model.body_subtreemass.At(bodyId) <= std::numeric_limits<dtDynReal>::epsilon())
        {
            data.subtree_com[static_cast<std::size_t>(bodyId)].SetZero();
            continue;
        }

        data.subtree_com[static_cast<std::size_t>(bodyId)] =
            data.scratch_weightedCom[static_cast<std::size_t>(bodyId)] *
            (1.0 / model.body_subtreemass.At(bodyId));
    }
}

void ComputeDiffMotion(const dtDynModel &model, dtDynData &data)
{
    // Forward velocity recursion:
    //   cvel_body = cvel_parent + sum_j cdof_j * qvel_j
    //   cdof_dot_j = crm(cvel_before_j) * cdof_j
    // The free-joint translational columns are applied before angular columns
    // to preserve MuJoCo qvel ordering and the expected cdof_dot convention.
    data.cvel[0].SetZero();

    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        dt::Math::Vector6<dtDynReal> &bodyVelocity = data.cvel[static_cast<std::size_t>(bodyId)];
        if (parent > 0)
        {
            bodyVelocity = data.cvel[static_cast<std::size_t>(parent)];
        }
        else
        {
            bodyVelocity.SetZero();
        }

        const dtDynIndex jntAdr = model.body_jntadr[static_cast<std::size_t>(bodyId)];
        const dtDynIndex jntNum = model.body_jntnum[static_cast<std::size_t>(bodyId)];
        for (dtDynIndex localJointId = 0; localJointId < jntNum; ++localJointId)
        {
            const dtDynIndex jointId = jntAdr + localJointId;
            const dtDynIndex dofAdr = model.jnt_dofadr[static_cast<std::size_t>(jointId)];
            const dtDynJointType jointType =
                static_cast<dtDynJointType>(model.jnt_type[static_cast<std::size_t>(jointId)]);

            if (jointType == dtDynJointType::Free)
            {
                for (dtDynIndex offset = 0; offset < 3; ++offset)
                {
                    AddScaledSpatialInPlace(bodyVelocity,
                                            data.cdof[static_cast<std::size_t>(dofAdr + offset)],
                                            data.qvel.At(dofAdr + offset));
                }
                if (IsZeroSpatial(bodyVelocity))
                {
                    for (dtDynIndex offset = 3; offset < 6; ++offset)
                    {
                        data.cdof_dot[static_cast<std::size_t>(dofAdr + offset)].SetZero();
                    }
                }
                else
                {
                    for (dtDynIndex offset = 3; offset < 6; ++offset)
                    {
                        CrossSpatialMotionInto(bodyVelocity,
                                               data.cdof[static_cast<std::size_t>(dofAdr + offset)],
                                               data.cdof_dot[static_cast<std::size_t>(dofAdr + offset)]);
                    }
                }
                for (dtDynIndex offset = 0; offset < 3; ++offset)
                {
                    data.cdof_dot[static_cast<std::size_t>(dofAdr + offset)].SetZero();
                }
                for (dtDynIndex offset = 3; offset < 6; ++offset)
                {
                    AddScaledSpatialInPlace(bodyVelocity,
                                            data.cdof[static_cast<std::size_t>(dofAdr + offset)],
                                            data.qvel.At(dofAdr + offset));
                }
            }
            else if (jointType == dtDynJointType::Ball)
            {
                if (IsZeroSpatial(bodyVelocity))
                {
                    for (dtDynIndex offset = 0; offset < 3; ++offset)
                    {
                        data.cdof_dot[static_cast<std::size_t>(dofAdr + offset)].SetZero();
                    }
                }
                else
                {
                    for (dtDynIndex offset = 0; offset < 3; ++offset)
                    {
                        CrossSpatialMotionInto(bodyVelocity,
                                               data.cdof[static_cast<std::size_t>(dofAdr + offset)],
                                               data.cdof_dot[static_cast<std::size_t>(dofAdr + offset)]);
                    }
                }
                for (dtDynIndex offset = 0; offset < 3; ++offset)
                {
                    AddScaledSpatialInPlace(bodyVelocity,
                                            data.cdof[static_cast<std::size_t>(dofAdr + offset)],
                                            data.qvel.At(dofAdr + offset));
                }
            }
            else
            {
                if (IsZeroSpatial(bodyVelocity))
                {
                    data.cdof_dot[static_cast<std::size_t>(dofAdr)].SetZero();
                }
                else
                {
                    CrossSpatialMotionInto(bodyVelocity,
                                           data.cdof[static_cast<std::size_t>(dofAdr)],
                                           data.cdof_dot[static_cast<std::size_t>(dofAdr)]);
                }
                AddScaledSpatialInPlace(bodyVelocity,
                                        data.cdof[static_cast<std::size_t>(dofAdr)],
                                        data.qvel.At(dofAdr));
            }
        }
    }
}

void FillBodySpatialInertiaAtRootCom(const dtDynModel &model,
                                     const dtDynData &data,
                                     const dtDynIndex bodyId,
                                     dt::Math::Matrix<6, 6, dtDynReal> &out)
{
    // Spatial inertia about the root CoM reference:
    //   I = [ R Ic R^T + m(||c||^2 E - c c^T),  m cx
    //       -m cx,                               m E  ]
    // where c = body_com - root_com and cx is the skew matrix of c.
    const std::size_t bodyIdx = static_cast<std::size_t>(bodyId);
    const dtDynIndex rootBody = model.body_rootid[bodyIdx];
    const std::size_t refIdx = static_cast<std::size_t>(rootBody > 0 ? rootBody : 0);

    const dtDynReal mass = model.body_mass.At(bodyId);
    const dt::Math::Matrix3<dtDynReal> &rot = data.ximat[bodyIdx];
    const dt::Math::Vector3<dtDynReal> &diag = model.body_inertia[bodyIdx];
    const dt::Math::Vector3<dtDynReal> &bodyCom = data.xipos[bodyIdx];
    const dt::Math::Vector3<dtDynReal> &reference = data.subtree_com[refIdx];
    const dtDynReal c0 = bodyCom.At(0) - reference.At(0);
    const dtDynReal c1 = bodyCom.At(1) - reference.At(1);
    const dtDynReal c2 = bodyCom.At(2) - reference.At(2);
    const dtDynReal c[3] = {c0, c1, c2};
    const dtDynReal c2sum = c0 * c0 + c1 * c1 + c2 * c2;

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < 3; ++col)
        {
            const dtDynReal rotInertia = rot.At(row, 0) * diag.At(0) * rot.At(col, 0) +
                                         rot.At(row, 1) * diag.At(1) * rot.At(col, 1) +
                                         rot.At(row, 2) * diag.At(2) * rot.At(col, 2);

            dtDynReal skew = 0.0;
            if (row == 0 && col == 1)
            {
                skew = -c2;
            }
            else if (row == 0 && col == 2)
            {
                skew = c1;
            }
            else if (row == 1 && col == 0)
            {
                skew = c2;
            }
            else if (row == 1 && col == 2)
            {
                skew = -c0;
            }
            else if (row == 2 && col == 0)
            {
                skew = -c1;
            }
            else if (row == 2 && col == 1)
            {
                skew = c0;
            }

            out.At(row, col) =
                rotInertia +
                (row == col ? mass * c2sum : 0.0) -
                mass * c[row] * c[col];
            out.At(row, col + 3) = mass * skew;
            out.At(row + 3, col) = -mass * skew;
            out.At(row + 3, col + 3) = row == col ? mass : 0.0;
        }
    }
}

void ComputeCompositeRigidBodyInertias(const dtDynModel &model, dtDynData &data)
{
    // Native CRBA upward pass: initialize each body inertia at its root CoM,
    // then accumulate child composite inertias into the parent tree.
    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        if (model.body_mass.At(bodyId) <= std::numeric_limits<dtDynReal>::epsilon())
        {
            data.cinert[static_cast<std::size_t>(bodyId)].SetZero();
            data.crb[static_cast<std::size_t>(bodyId)].SetZero();
            continue;
        }

        FillBodySpatialInertiaAtRootCom(model,
                                        data,
                                        bodyId,
                                        data.cinert[static_cast<std::size_t>(bodyId)]);
        data.crb[static_cast<std::size_t>(bodyId)] =
            data.cinert[static_cast<std::size_t>(bodyId)];
    }

    for (dtDynIndex bodyId = model.nbody - 1; bodyId > 0; --bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        if (parent >= 0)
        {
            data.crb[static_cast<std::size_t>(parent)] += data.crb[static_cast<std::size_t>(bodyId)];
        }
    }
}

bool IsZeroSpatial(const dt::Math::Vector6<dtDynReal> &value)
{
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        if (value.At(row) != 0.0)
        {
            return false;
        }
    }
    return true;
}

// Fixed-size CCRBA spatial kernels. These helpers avoid dynamic local
// temporaries; end-to-end allocation behavior still depends on caller-owned
// storage and is reported only within the documented benchmark-counter scope.
// The comments show the equivalent dtMath-level expressions.

void SpatialMulPair(const dt::Math::Matrix<6, 6, dtDynReal> &matrix,
                    const dt::Math::Vector6<dtDynReal> &leftVector,
                    const dt::Math::Vector6<dtDynReal> &rightVector,
                    dt::Math::Vector6<dtDynReal> &leftOut,
                    dt::Math::Vector6<dtDynReal> &rightOut)
{
    // Computes leftOut = matrix * leftVector and rightOut = matrix * rightVector
    // in one matrix scan for the CCRBA DOF bias-force recurrence.
    const dtDynReal l0 = leftVector.At(0);
    const dtDynReal l1 = leftVector.At(1);
    const dtDynReal l2 = leftVector.At(2);
    const dtDynReal l3 = leftVector.At(3);
    const dtDynReal l4 = leftVector.At(4);
    const dtDynReal l5 = leftVector.At(5);
    const dtDynReal r0 = rightVector.At(0);
    const dtDynReal r1 = rightVector.At(1);
    const dtDynReal r2 = rightVector.At(2);
    const dtDynReal r3 = rightVector.At(3);
    const dtDynReal r4 = rightVector.At(4);
    const dtDynReal r5 = rightVector.At(5);

    for (dtDynIndex row = 0; row < 6; ++row)
    {
        const dtDynReal m0 = matrix.At(row, 0);
        const dtDynReal m1 = matrix.At(row, 1);
        const dtDynReal m2 = matrix.At(row, 2);
        const dtDynReal m3 = matrix.At(row, 3);
        const dtDynReal m4 = matrix.At(row, 4);
        const dtDynReal m5 = matrix.At(row, 5);
        leftOut.At(row) = m0 * l0 + m1 * l1 + m2 * l2 + m3 * l3 + m4 * l4 + m5 * l5;
        rightOut.At(row) = m0 * r0 + m1 * r1 + m2 * r2 + m3 * r3 + m4 * r4 + m5 * r5;
    }
}

void SpatialMulInto(const dt::Math::Matrix<6, 6, dtDynReal> &matrix,
                    const dt::Math::Vector6<dtDynReal> &vector,
                    dt::Math::Vector6<dtDynReal> &out)
{
    // Computes out = matrix * vector without a temporary. This is used when the
    // paired product above degenerates because cdof_dot is exactly zero.
    const dtDynReal v0 = vector.At(0);
    const dtDynReal v1 = vector.At(1);
    const dtDynReal v2 = vector.At(2);
    const dtDynReal v3 = vector.At(3);
    const dtDynReal v4 = vector.At(4);
    const dtDynReal v5 = vector.At(5);

    for (dtDynIndex row = 0; row < 6; ++row)
    {
        out.At(row) = matrix.At(row, 0) * v0 + matrix.At(row, 1) * v1 +
                      matrix.At(row, 2) * v2 + matrix.At(row, 3) * v3 +
                      matrix.At(row, 4) * v4 + matrix.At(row, 5) * v5;
    }
}

void AddSpatialMulAndTransposeMul(const dt::Math::Matrix<6, 6, dtDynReal> &matrix,
                                  const dt::Math::Vector6<dtDynReal> &vector,
                                  dt::Math::Vector6<dtDynReal> &forwardOut,
                                  dt::Math::Vector6<dtDynReal> &transposeOut)
{
    // Fused form of:
    //   forwardOut += matrix * vector
    //   transposeOut = matrix.Transpose() * vector
    // The fused scan avoids materializing the transpose in the realtime path.
    const dtDynReal v0 = vector.At(0);
    const dtDynReal v1 = vector.At(1);
    const dtDynReal v2 = vector.At(2);
    const dtDynReal v3 = vector.At(3);
    const dtDynReal v4 = vector.At(4);
    const dtDynReal v5 = vector.At(5);

    dtDynReal transpose0 = 0.0;
    dtDynReal transpose1 = 0.0;
    dtDynReal transpose2 = 0.0;
    dtDynReal transpose3 = 0.0;
    dtDynReal transpose4 = 0.0;
    dtDynReal transpose5 = 0.0;
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        const dtDynReal vectorRow = vector.At(row);
        const dtDynReal m0 = matrix.At(row, 0);
        const dtDynReal m1 = matrix.At(row, 1);
        const dtDynReal m2 = matrix.At(row, 2);
        const dtDynReal m3 = matrix.At(row, 3);
        const dtDynReal m4 = matrix.At(row, 4);
        const dtDynReal m5 = matrix.At(row, 5);
        const dtDynReal forwardValue = m0 * v0 + m1 * v1 + m2 * v2 + m3 * v3 + m4 * v4 + m5 * v5;
        transpose0 += m0 * vectorRow;
        transpose1 += m1 * vectorRow;
        transpose2 += m2 * vectorRow;
        transpose3 += m3 * vectorRow;
        transpose4 += m4 * vectorRow;
        transpose5 += m5 * vectorRow;
        forwardOut.At(row) += forwardValue;
    }

    transposeOut.At(0) = transpose0;
    transposeOut.At(1) = transpose1;
    transposeOut.At(2) = transpose2;
    transposeOut.At(3) = transpose3;
    transposeOut.At(4) = transpose4;
    transposeOut.At(5) = transpose5;
}

void AddHalfSymmetricMotionCrossColumn2(const dt::Math::Matrix<6, 6, dtDynReal> &inertia,
                                        const dtDynIndex motionCol,
                                        const dtDynIndex row0,
                                        const dtDynReal value0,
                                        const dtDynIndex row1,
                                        const dtDynReal value1,
                                        dt::Math::Matrix<6, 6, dtDynReal> &out)
{
    // Adds one sparse column of:
    //   -0.5 * (crm(v).Transpose() * inertia + inertia * crm(v))
    // rows/value pairs describe the nonzero entries in crm(v).col(motionCol).
    if (value0 == 0.0 && value1 == 0.0)
    {
        return;
    }

    for (dtDynIndex col = 0; col < 6; ++col)
    {
        const dtDynReal contribution =
            -0.5 * (value0 * inertia.At(row0, col) +
                    value1 * inertia.At(row1, col));
        out.At(motionCol, col) += contribution;
        out.At(col, motionCol) += contribution;
    }
}

void AddHalfSymmetricMotionCrossColumn4(const dt::Math::Matrix<6, 6, dtDynReal> &inertia,
                                        const dtDynIndex motionCol,
                                        const dtDynIndex row0,
                                        const dtDynReal value0,
                                        const dtDynIndex row1,
                                        const dtDynReal value1,
                                        const dtDynIndex row2,
                                        const dtDynReal value2,
                                        const dtDynIndex row3,
                                        const dtDynReal value3,
                                        dt::Math::Matrix<6, 6, dtDynReal> &out)
{
    // Adds one sparse column of:
    //   -0.5 * (crm(v).Transpose() * inertia + inertia * crm(v))
    // rows/value pairs describe the nonzero entries in crm(v).col(motionCol).
    if (value0 == 0.0 && value1 == 0.0 && value2 == 0.0 && value3 == 0.0)
    {
        return;
    }

    for (dtDynIndex col = 0; col < 6; ++col)
    {
        const dtDynReal contribution =
            -0.5 * (value0 * inertia.At(row0, col) +
                    value1 * inertia.At(row1, col) +
                    value2 * inertia.At(row2, col) +
                    value3 * inertia.At(row3, col));
        out.At(motionCol, col) += contribution;
        out.At(col, motionCol) += contribution;
    }
}

void AddHalfMomentumCrossMatrix(const dtDynReal momentum0,
                                const dtDynReal momentum1,
                                const dtDynReal momentum2,
                                const dtDynReal momentum3,
                                const dtDynReal momentum4,
                                const dtDynReal momentum5,
                                dt::Math::Matrix<6, 6, dtDynReal> &matrix)
{
    // Adds 0.5 * CrossSpatialMomentumMatrix(momentum) directly in place.
    const dtDynReal hwx = 0.5 * momentum0;
    const dtDynReal hwy = 0.5 * momentum1;
    const dtDynReal hwz = 0.5 * momentum2;
    const dtDynReal hvx = 0.5 * momentum3;
    const dtDynReal hvy = 0.5 * momentum4;
    const dtDynReal hvz = 0.5 * momentum5;

    matrix.At(0, 1) += hwz;
    matrix.At(0, 2) += -hwy;
    matrix.At(1, 0) += -hwz;
    matrix.At(1, 2) += hwx;
    matrix.At(2, 0) += hwy;
    matrix.At(2, 1) += -hwx;

    matrix.At(0, 4) += hvz;
    matrix.At(0, 5) += -hvy;
    matrix.At(1, 3) += -hvz;
    matrix.At(1, 5) += hvx;
    matrix.At(2, 3) += hvy;
    matrix.At(2, 4) += -hvx;

    matrix.At(3, 1) += hvz;
    matrix.At(3, 2) += -hvy;
    matrix.At(4, 0) += -hvz;
    matrix.At(4, 2) += hvx;
    matrix.At(5, 0) += hvy;
    matrix.At(5, 1) += -hvx;
}

void ComputeBiasMatrix(const dt::Math::Matrix<6, 6, dtDynReal> &inertia,
                       const dt::Math::Vector6<dtDynReal> &velocity,
                       dt::Math::Matrix<6, 6, dtDynReal> &out)
{
    // Builds the local CCRBA bias matrix from spatial inertia I and velocity v.
    // Here crm(v) denotes math::CrossSpatialMotionMatrix(v).
    // Dense equivalent in this spatial convention:
    //   B = -0.5 * (crm(v).Transpose() * I + I * crm(v))
    //       + 0.5 * CrossSpatialMomentumMatrix(I * v)
    // The implementation below accumulates sparse cross terms directly.
    out.SetZero();

    const dtDynReal wx = velocity.At(0);
    const dtDynReal wy = velocity.At(1);
    const dtDynReal wz = velocity.At(2);
    const dtDynReal vx = velocity.At(3);
    const dtDynReal vy = velocity.At(4);
    const dtDynReal vz = velocity.At(5);
    const dt::Math::Vector6<dtDynReal> momentum(inertia * velocity);

    AddHalfSymmetricMotionCrossColumn4(inertia, 0, 1, wz, 2, -wy, 4, vz, 5, -vy, out);
    AddHalfSymmetricMotionCrossColumn4(inertia, 1, 0, -wz, 2, wx, 3, -vz, 5, vx, out);
    AddHalfSymmetricMotionCrossColumn4(inertia, 2, 0, wy, 1, -wx, 3, vy, 4, -vx, out);
    AddHalfSymmetricMotionCrossColumn2(inertia, 3, 4, wz, 5, -wy, out);
    AddHalfSymmetricMotionCrossColumn2(inertia, 4, 3, -wz, 5, wx, out);
    AddHalfSymmetricMotionCrossColumn2(inertia, 5, 3, wy, 4, -wx, out);

    AddHalfMomentumCrossMatrix(momentum.At(0),
                               momentum.At(1),
                               momentum.At(2),
                               momentum.At(3),
                               momentum.At(4),
                               momentum.At(5),
                               out);
}

void ComputeLocalBiasMatrices(const dtDynModel &model, dtDynData &data)
{
    // Local body B matrices are zero for massless or zero-velocity bodies. The
    // nonzero flag lets the composite pass skip known-zero matrices without
    // clearing or scanning a 6x6 block every tick.
    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        if (model.body_mass.At(bodyId) <= std::numeric_limits<dtDynReal>::epsilon())
        {
            data.BmatCompNonzero[static_cast<std::size_t>(bodyId)] = 0;
            continue;
        }

        if (IsZeroSpatial(data.cvel[static_cast<std::size_t>(bodyId)]))
        {
            data.BmatCompNonzero[static_cast<std::size_t>(bodyId)] = 0;
            continue;
        }

        ComputeBiasMatrix(data.cinert[static_cast<std::size_t>(bodyId)],
                          data.cvel[static_cast<std::size_t>(bodyId)],
                          data.BmatComp[static_cast<std::size_t>(bodyId)]);
        data.BmatCompNonzero[static_cast<std::size_t>(bodyId)] = 1;
    }
}

void AccumulateCompositeBiasMatrices(const dtDynModel &model, dtDynData &data)
{
    // Upward composite-bias pass: Bcomp_parent += Bcomp_child. The first
    // nonzero child can be copied directly into a zero parent block.
    for (dtDynIndex bodyId = model.nbody - 1; bodyId > 0; --bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        if (parent >= 0 && data.BmatCompNonzero[static_cast<std::size_t>(bodyId)] != 0)
        {
            const std::size_t parentIdx = static_cast<std::size_t>(parent);
            if (data.BmatCompNonzero[parentIdx] != 0)
            {
                data.BmatComp[parentIdx] += data.BmatComp[static_cast<std::size_t>(bodyId)];
            }
            else
            {
                data.BmatComp[parentIdx] = data.BmatComp[static_cast<std::size_t>(bodyId)];
                data.BmatCompNonzero[parentIdx] = 1;
            }
        }
    }
}

void ComputeDofBiasForces(const dtDynModel &model, dtDynData &data)
{
    // Per-DOF CCRBA force columns:
    //   f1_j = crb_body * cdof_dot_j + Bcomp_body * cdof_j
    //   f2_j = crb_body * cdof_j
    //   f3_j = Bcomp_body^T * cdof_j
    // These columns feed Mdot, C, CoM Jacobian, and centroidal outputs.
    for (dtDynIndex dofId = model.nv - 1; dofId >= 0; --dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const bool zeroDofDot = IsZeroSpatial(data.cdof_dot[static_cast<std::size_t>(dofId)]);
        const bool zeroBiasMatrix =
            data.BmatCompNonzero[static_cast<std::size_t>(bodyId)] == 0;

        if (zeroDofDot)
        {
            data.f1[static_cast<std::size_t>(dofId)].SetZero();
            SpatialMulInto(data.crb[static_cast<std::size_t>(bodyId)],
                           data.cdof[static_cast<std::size_t>(dofId)],
                           data.f2[static_cast<std::size_t>(dofId)]);
        }
        else
        {
            SpatialMulPair(data.crb[static_cast<std::size_t>(bodyId)],
                           data.cdof_dot[static_cast<std::size_t>(dofId)],
                           data.cdof[static_cast<std::size_t>(dofId)],
                           data.f1[static_cast<std::size_t>(dofId)],
                           data.f2[static_cast<std::size_t>(dofId)]);
        }

        if (zeroBiasMatrix)
        {
            data.f3[static_cast<std::size_t>(dofId)].SetZero();
        }
        else
        {
            AddSpatialMulAndTransposeMul(data.BmatComp[static_cast<std::size_t>(bodyId)],
                                         data.cdof[static_cast<std::size_t>(dofId)],
                                         data.f1[static_cast<std::size_t>(dofId)],
                                         data.f3[static_cast<std::size_t>(dofId)]);
        }
    }
}

void ComputeBiasMatricesAndForcesFromCompositeInertias(const dtDynModel &model, dtDynData &data)
{
    ComputeLocalBiasMatrices(model, data);
    AccumulateCompositeBiasMatrices(model, data);
    ComputeDofBiasForces(model, data);
}

void ComputeCompositeInertiaAndBias(const dtDynModel &model, dtDynData &data)
{
    ComputeCompositeRigidBodyInertias(model, data);
    ComputeBiasMatricesAndForcesFromCompositeInertias(model, data);
}

void ComputeMassMatrixFromCompositeInertias(const dtDynModel &model, dtDynData &data)
{
    // Standard CRBA projection:
    //   M_ij = cdof_i^T * crb_body(j) * cdof_j
    // written only along ancestor chains; non-ancestor entries stay zero.
    data.Mmat.SetZero();

    for (dtDynIndex dofId = model.nv - 1; dofId >= 0; --dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const dt::Math::Vector6<dtDynReal> compositeForce =
            math::SpatialMul(data.crb[static_cast<std::size_t>(bodyId)],
                             data.cdof[static_cast<std::size_t>(dofId)]);

        dtDynIndex ancestorDofId = dofId;
        while (ancestorDofId >= 0)
        {
            const dtDynReal value =
                math::SpatialDot(data.cdof[static_cast<std::size_t>(ancestorDofId)], compositeForce);

            data.Mmat.At(dofId, ancestorDofId) = value;
            data.Mmat.At(ancestorDofId, dofId) = value;
            ancestorDofId = model.dof_parentid[static_cast<std::size_t>(ancestorDofId)];
        }
    }

    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        data.Mmat.At(dofId, dofId) += model.dof_armature.At(dofId);
    }
}

constexpr bool HasRNEATerm(const dtDynRneaTermFlags flags, const dtDynRneaTermFlags term)
{
    return (flags & term) != 0u;
}

constexpr bool HasCCRBATerm(const dtDynCcrbaTermFlags flags, const dtDynCcrbaTermFlags term)
{
    return (flags & term) != 0u;
}

void ClearOptionalDynamicsOutputs(dtDynData &data)
{
    data.MmatDot.SetZero();
    data.Cmat.SetZero();
    data.cmAmat.SetZero();
    data.cmAmatDot.SetZero();
    data.comJmat.SetZero();
    data.comJmatDot.SetZero();
    data.centMom.SetZero();
    data.comPos.SetZero();
    data.comVel.SetZero();
}

/*!
 \brief Compute CCRBA/RNEA shared subtree CoM, motion subspace, velocity, and composite inertia terms.
*/

void ComputeCentroidalPrerequisites(const dtDynModel &model, dtDynData &data)
{
    // CCRBA prerequisites produce cdof/cdof_dot, body velocities, composite
    // inertias, and the f1/f2/f3 force columns reused by all requested outputs.
    ComputeSubtreeCom(model, data);
    ComputeDofMotionSubspaces(model, data);
    ComputeDiffMotion(model, data);
    ComputeCompositeInertiaAndBias(model, data);
}

void ComputeRNEAPrerequisites(const dtDynModel &model, dtDynData &data)
{
    // RNEA shares kinematics and CRBA mass ingredients but does not need Bcomp
    // or the CCRBA f1/f2/f3 bias-force columns.
    ComputeSubtreeCom(model, data);
    ComputeDofMotionSubspaces(model, data);
    ComputeDiffMotion(model, data);
    ComputeCompositeRigidBodyInertias(model, data);
}

void ComputeMassMatrixCRBA(const dtDynModel &model, dtDynData &data)
{
    ComputeMassMatrixFromCompositeInertias(model, data);
}

void ClearDenseDynamicsOutputs(const dtDynModel &, dtDynData &data)
{
    data.Mmat.SetZero();
    ClearOptionalDynamicsOutputs(data);
}

void ComputeRootComOutputs(const dtDynModel &model, dtDynData &data)
{
    data.comPos.SetZero();

    for (dtDynIndex rootIdx = 0; rootIdx < model.nroot; ++rootIdx)
    {
        const dtDynIndex rootBody = model.root_bodyid[static_cast<std::size_t>(rootIdx)];
        const dtDynIndex comRow = utils::RootComRow(rootIdx);
        for (dtDynIndex row = 0; row < 3; ++row)
        {
            data.comPos.At(comRow + row) = data.subtree_com[static_cast<std::size_t>(rootBody)].At(row);
        }
    }
}

void ComputeCentroidalMomentumMatrix(const dtDynModel &model, dtDynData &data)
{
    // Centroidal momentum matrix columns are the composite spatial momentum
    // columns f2_j split into linear rows followed by angular rows per root.
    for (dtDynIndex dofId = model.nv - 1; dofId >= 0; --dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const dtDynIndex rootIdx = utils::RootIndex(model, bodyId);

        if (rootIdx >= 0)
        {
            const dtDynIndex linearRow = utils::RootCentroidalLinearRow(rootIdx);
            const dtDynIndex angularRow = utils::RootCentroidalAngularRow(rootIdx);
            data.cmAmat.At(linearRow + 0, dofId) = data.f2[static_cast<std::size_t>(dofId)].At(3);
            data.cmAmat.At(linearRow + 1, dofId) = data.f2[static_cast<std::size_t>(dofId)].At(4);
            data.cmAmat.At(linearRow + 2, dofId) = data.f2[static_cast<std::size_t>(dofId)].At(5);
            data.cmAmat.At(angularRow + 0, dofId) = data.f2[static_cast<std::size_t>(dofId)].At(0);
            data.cmAmat.At(angularRow + 1, dofId) = data.f2[static_cast<std::size_t>(dofId)].At(1);
            data.cmAmat.At(angularRow + 2, dofId) = data.f2[static_cast<std::size_t>(dofId)].At(2);
        }
    }
}

void ComputeCentroidalMomentumMatrixDot(const dtDynModel &model, dtDynData &data)
{
    // CMM time-variation columns are the matching derivative force columns f1_j.
    for (dtDynIndex dofId = model.nv - 1; dofId >= 0; --dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const dtDynIndex rootIdx = utils::RootIndex(model, bodyId);

        if (rootIdx >= 0)
        {
            const dtDynIndex linearRow = utils::RootCentroidalLinearRow(rootIdx);
            const dtDynIndex angularRow = utils::RootCentroidalAngularRow(rootIdx);
            data.cmAmatDot.At(linearRow + 0, dofId) = data.f1[static_cast<std::size_t>(dofId)].At(3);
            data.cmAmatDot.At(linearRow + 1, dofId) = data.f1[static_cast<std::size_t>(dofId)].At(4);
            data.cmAmatDot.At(linearRow + 2, dofId) = data.f1[static_cast<std::size_t>(dofId)].At(5);
            data.cmAmatDot.At(angularRow + 0, dofId) = data.f1[static_cast<std::size_t>(dofId)].At(0);
            data.cmAmatDot.At(angularRow + 1, dofId) = data.f1[static_cast<std::size_t>(dofId)].At(1);
            data.cmAmatDot.At(angularRow + 2, dofId) = data.f1[static_cast<std::size_t>(dofId)].At(2);
        }
    }
}

void ComputeMassMatrixCCRBA(const dtDynModel &model, dtDynData &data)
{
    // CCRBA mass matrix uses precomputed f2_j = crb_body(j) * cdof_j,
    // avoiding a separate composite-force projection per column.
    for (dtDynIndex dofId = model.nv - 1; dofId >= 0; --dofId)
    {
        data.Mmat.At(dofId, dofId) =
            math::SpatialDot(data.cdof[static_cast<std::size_t>(dofId)],
                             data.f2[static_cast<std::size_t>(dofId)]) +
            model.dof_armature.At(dofId);

        dtDynIndex ancestorDofId = model.dof_parentid[static_cast<std::size_t>(dofId)];
        while (ancestorDofId >= 0)
        {
            const dtDynReal massValue =
                math::SpatialDot(data.cdof[static_cast<std::size_t>(ancestorDofId)],
                                 data.f2[static_cast<std::size_t>(dofId)]);
            data.Mmat.At(dofId, ancestorDofId) = massValue;
            data.Mmat.At(ancestorDofId, dofId) = massValue;

            ancestorDofId = model.dof_parentid[static_cast<std::size_t>(ancestorDofId)];
        }
    }
}

void ComputeMassMatrixDotCCRBA(const dtDynModel &model, dtDynData &data)
{
    // Mdot entries are projected from cdof_dot, f1, f2, and f3 along the same
    // ancestor sparsity pattern as M.
    for (dtDynIndex dofId = model.nv - 1; dofId >= 0; --dofId)
    {
        data.MmatDot.At(dofId, dofId) =
            math::SpatialDot(data.cdof_dot[static_cast<std::size_t>(dofId)],
                             data.f2[static_cast<std::size_t>(dofId)]) +
            math::SpatialDot3(data.cdof[static_cast<std::size_t>(dofId)],
                              data.f1[static_cast<std::size_t>(dofId)],
                              data.f3[static_cast<std::size_t>(dofId)]);

        dtDynIndex ancestorDofId = model.dof_parentid[static_cast<std::size_t>(dofId)];
        while (ancestorDofId >= 0)
        {
            const dtDynReal massDotValue =
                math::SpatialDot(data.cdof_dot[static_cast<std::size_t>(ancestorDofId)],
                                 data.f2[static_cast<std::size_t>(dofId)]) +
                math::SpatialDot3(data.cdof[static_cast<std::size_t>(ancestorDofId)],
                                  data.f1[static_cast<std::size_t>(dofId)],
                                  data.f3[static_cast<std::size_t>(dofId)]);
            data.MmatDot.At(dofId, ancestorDofId) = massDotValue;
            data.MmatDot.At(ancestorDofId, dofId) = massDotValue;

            ancestorDofId = model.dof_parentid[static_cast<std::size_t>(ancestorDofId)];
        }
    }
}

void ComputeCoriolisMatrixCCRBA(const dtDynModel &model, dtDynData &data)
{
    // C_ij is assembled from the CCRBA force columns without finite differences.
    // Parent-chain traversal writes only dynamically coupled ancestor entries.
    for (dtDynIndex dofId = model.nv - 1; dofId >= 0; --dofId)
    {
        data.Cmat.At(dofId, dofId) =
            math::SpatialDot(data.cdof[static_cast<std::size_t>(dofId)],
                             data.f1[static_cast<std::size_t>(dofId)]);

        dtDynIndex ancestorDofId = model.dof_parentid[static_cast<std::size_t>(dofId)];
        while (ancestorDofId >= 0)
        {
            data.Cmat.At(ancestorDofId, dofId) =
                math::SpatialDot(data.cdof[static_cast<std::size_t>(ancestorDofId)],
                                 data.f1[static_cast<std::size_t>(dofId)]);
            data.Cmat.At(dofId, ancestorDofId) =
                math::SpatialDot(data.cdof_dot[static_cast<std::size_t>(ancestorDofId)],
                                 data.f2[static_cast<std::size_t>(dofId)]) +
                math::SpatialDot(data.cdof[static_cast<std::size_t>(ancestorDofId)],
                                 data.f3[static_cast<std::size_t>(dofId)]);

            ancestorDofId = model.dof_parentid[static_cast<std::size_t>(ancestorDofId)];
        }
    }
}

void ComputeCoriolisMatrixAndTorqueCCRBA(const dtDynModel &model, dtDynData &data)
{
    // Same C matrix assembly as ComputeCoriolisMatrixCCRBA, fused with
    // coriolisTorq += C_col * qvel_col to avoid a later dense C*qvel pass.
    data.coriolisTorq.SetZero();

    for (dtDynIndex dofId = model.nv - 1; dofId >= 0; --dofId)
    {
        const dtDynReal diagonalValue =
            math::SpatialDot(data.cdof[static_cast<std::size_t>(dofId)],
                             data.f1[static_cast<std::size_t>(dofId)]);
        data.Cmat.At(dofId, dofId) = diagonalValue;
        data.coriolisTorq.At(dofId) += diagonalValue * data.qvel.At(dofId);

        dtDynIndex ancestorDofId = model.dof_parentid[static_cast<std::size_t>(dofId)];
        while (ancestorDofId >= 0)
        {
            const dtDynReal ancestorRowValue =
                math::SpatialDot(data.cdof[static_cast<std::size_t>(ancestorDofId)],
                                 data.f1[static_cast<std::size_t>(dofId)]);
            const dtDynReal dofRowValue =
                math::SpatialDot(data.cdof_dot[static_cast<std::size_t>(ancestorDofId)],
                                 data.f2[static_cast<std::size_t>(dofId)]) +
                math::SpatialDot(data.cdof[static_cast<std::size_t>(ancestorDofId)],
                                 data.f3[static_cast<std::size_t>(dofId)]);

            data.Cmat.At(ancestorDofId, dofId) = ancestorRowValue;
            data.Cmat.At(dofId, ancestorDofId) = dofRowValue;
            data.coriolisTorq.At(ancestorDofId) += ancestorRowValue * data.qvel.At(dofId);
            data.coriolisTorq.At(dofId) += dofRowValue * data.qvel.At(ancestorDofId);

            ancestorDofId = model.dof_parentid[static_cast<std::size_t>(ancestorDofId)];
        }
    }
}

void ComputeComJacobianMatrixFromForces(const dtDynModel &model, dtDynData &data)
{
    // Linear rows of f2 are root linear momentum columns, so Jcom = linear(f2) / root_mass.
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const dtDynIndex rootIdx = utils::RootIndex(model, bodyId);
        if (rootIdx < 0)
        {
            continue;
        }
        if (model.root_mass.At(rootIdx) <= std::numeric_limits<dtDynReal>::epsilon())
        {
            continue;
        }

        const dtDynIndex comRow = utils::RootComRow(rootIdx);
        for (dtDynIndex row = 0; row < 3; ++row)
        {
            data.comJmat.At(comRow + row, dofId) =
                data.f2[static_cast<std::size_t>(dofId)].At(3 + row) / model.root_mass.At(rootIdx);
        }
    }
}

void ComputeComJacobianMatrixDotFromForces(const dtDynModel &model, dtDynData &data)
{
    // Linear rows of f1 give the matching time derivative of the CoM Jacobian.
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const dtDynIndex rootIdx = utils::RootIndex(model, bodyId);
        if (rootIdx < 0)
        {
            continue;
        }
        if (model.root_mass.At(rootIdx) <= std::numeric_limits<dtDynReal>::epsilon())
        {
            continue;
        }

        const dtDynIndex comRow = utils::RootComRow(rootIdx);
        for (dtDynIndex row = 0; row < 3; ++row)
        {
            data.comJmatDot.At(comRow + row, dofId) =
                data.f1[static_cast<std::size_t>(dofId)].At(3 + row) / model.root_mass.At(rootIdx);
        }
    }
}

void ComputeCentroidalMomentumProductFromForces(const dtDynModel &model, dtDynData &data)
{
    // centMom = cmAmat * qvel, accumulated directly from f2 columns to avoid
    // requiring cmAmat when only centroidal momentum is requested.
    data.centMom.SetZero();

    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const dtDynIndex rootIdx = utils::RootIndex(model, bodyId);
        if (rootIdx < 0)
        {
            continue;
        }

        const dtDynIndex linearRow = utils::RootCentroidalLinearRow(rootIdx);
        const dtDynIndex angularRow = utils::RootCentroidalAngularRow(rootIdx);
        const dtDynReal qvel = data.qvel.At(dofId);
        for (dtDynIndex row = 0; row < 3; ++row)
        {
            data.centMom.At(linearRow + row) += data.f2[static_cast<std::size_t>(dofId)].At(3 + row) * qvel;
            data.centMom.At(angularRow + row) += data.f2[static_cast<std::size_t>(dofId)].At(row) * qvel;
        }
    }
}

void ComputeComVelocityProductFromForces(const dtDynModel &model, dtDynData &data)
{
    // comVel = comJmat * qvel, accumulated directly from f2 linear rows.
    data.comVel.SetZero();

    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        const dtDynIndex rootIdx = utils::RootIndex(model, bodyId);
        if (rootIdx < 0)
        {
            continue;
        }
        if (model.root_mass.At(rootIdx) <= std::numeric_limits<dtDynReal>::epsilon())
        {
            continue;
        }

        const dtDynIndex comRow = utils::RootComRow(rootIdx);
        const dtDynReal qvel = data.qvel.At(dofId);
        for (dtDynIndex row = 0; row < 3; ++row)
        {
            data.comVel.At(comRow + row) +=
                data.f2[static_cast<std::size_t>(dofId)].At(3 + row) * qvel / model.root_mass.At(rootIdx);
        }
    }
}

void ComputeCoriolisTorqueProduct(const dtDynModel &model, dtDynData &data)
{
    data.coriolisTorq.SetZero();

    for (dtDynIndex row = 0; row < model.nv; ++row)
    {
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            data.coriolisTorq.At(row) += data.Cmat.At(row, col) * data.qvel.At(col);
        }
    }
}

void ComputeRNEATorque(const dtDynModel &model,
                       dtDynData &data,
                       const dt::Math::Vector3<dtDynReal> &gravity,
                       const bool includeVelocityTerms,
                       dtDynDenseVector &output)
{
    // Recursive Newton-Euler with qacc = 0:
    //   includeVelocityTerms=false -> generalized gravity
    //   includeVelocityTerms=true  -> gravity + Coriolis/centrifugal bias
    output.SetZero();

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        data.cacc[static_cast<std::size_t>(bodyId)].SetZero();
        data.cfrc[static_cast<std::size_t>(bodyId)].SetZero();
    }

    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        if (parent > 0)
        {
            data.cacc[static_cast<std::size_t>(bodyId)] =
                data.cacc[static_cast<std::size_t>(parent)];
        }
        else
        {
            data.cacc[static_cast<std::size_t>(bodyId)].At(3) = -gravity.At(0);
            data.cacc[static_cast<std::size_t>(bodyId)].At(4) = -gravity.At(1);
            data.cacc[static_cast<std::size_t>(bodyId)].At(5) = -gravity.At(2);
        }

        if (includeVelocityTerms)
        {
            const dtDynIndex dofAdr = model.body_dofadr[static_cast<std::size_t>(bodyId)];
            const dtDynIndex dofNum = model.body_dofnum[static_cast<std::size_t>(bodyId)];
            for (dtDynIndex localDofId = 0; localDofId < dofNum; ++localDofId)
            {
                const dtDynIndex dofId = dofAdr + localDofId;
                AddScaledSpatialInPlace(data.cacc[static_cast<std::size_t>(bodyId)],
                                        data.cdof_dot[static_cast<std::size_t>(dofId)],
                                        data.qvel.At(dofId));
            }
        }

        if (model.body_mass.At(bodyId) <= std::numeric_limits<dtDynReal>::epsilon())
        {
            continue;
        }

        data.cfrc[static_cast<std::size_t>(bodyId)] =
            math::SpatialMul(data.cinert[static_cast<std::size_t>(bodyId)],
                             data.cacc[static_cast<std::size_t>(bodyId)]);

        if (includeVelocityTerms)
        {
            const dt::Math::Vector6<dtDynReal> momentum =
                math::SpatialMul(data.cinert[static_cast<std::size_t>(bodyId)],
                                 data.cvel[static_cast<std::size_t>(bodyId)]);
            math::AddSpatialInPlace(data.cfrc[static_cast<std::size_t>(bodyId)],
                                    math::SpatialMul(math::CrossSpatialMotionMatrixStar(
                                                         data.cvel[static_cast<std::size_t>(bodyId)]),
                                                     momentum));
        }
    }

    for (dtDynIndex bodyId = model.nbody - 1; bodyId > 0; --bodyId)
    {
        const dtDynIndex dofAdr = model.body_dofadr[static_cast<std::size_t>(bodyId)];
        const dtDynIndex dofNum = model.body_dofnum[static_cast<std::size_t>(bodyId)];
        for (dtDynIndex localDofId = 0; localDofId < dofNum; ++localDofId)
        {
            const dtDynIndex dofId = dofAdr + localDofId;
            output.At(dofId) =
                math::SpatialDot(data.cdof[static_cast<std::size_t>(dofId)],
                                 data.cfrc[static_cast<std::size_t>(bodyId)]);
        }

        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        if (parent >= 0)
        {
            math::AddSpatialInPlace(data.cfrc[static_cast<std::size_t>(parent)],
                                    data.cfrc[static_cast<std::size_t>(bodyId)]);
        }
    }
}

void ComputeRNEABiasForces(const dtDynModel &model,
                           dtDynData &data,
                           const dt::Math::Vector3<dtDynReal> &gravity)
{
    ComputeRNEATorque(model, data, gravity, false, data.gravityTorq);
    ComputeRNEATorque(model, data, gravity, true, data.nonlinTorq);

    data.coriolisTorq.SetZero();
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        data.coriolisTorq.At(dofId) = data.nonlinTorq.At(dofId) - data.gravityTorq.At(dofId);
    }
}

void ComputeGravityFromCentroidalForces(const dtDynModel &model,
                                        dtDynData &data,
                                        const dt::Math::Vector3<dtDynReal> &gravity)
{
    // Gravity torque from CCRBA linear momentum columns:
    //   tau_g_j = -linear(f2_j)^T * gravity
    data.gravityTorq.SetZero();

    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        const dtDynIndex bodyId = model.dof_bodyid[static_cast<std::size_t>(dofId)];
        if (utils::RootIndex(model, bodyId) < 0)
        {
            continue;
        }

        dt::Math::Vector3<dtDynReal> linearMomentumColumn;
        linearMomentumColumn.At(0) = data.f2[static_cast<std::size_t>(dofId)].At(3);
        linearMomentumColumn.At(1) = data.f2[static_cast<std::size_t>(dofId)].At(4);
        linearMomentumColumn.At(2) = data.f2[static_cast<std::size_t>(dofId)].At(5);
        data.gravityTorq.At(dofId) -= linearMomentumColumn.Inner(gravity);
    }
}

void ComputeCCRBABiasForces(const dtDynModel &model,
                            dtDynData &data,
                            const dt::Math::Vector3<dtDynReal> &gravity)
{
    ComputeGravityFromCentroidalForces(model, data, gravity);
    ComputeCoriolisTorqueProduct(model, data);

    data.nonlinTorq.SetZero();
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        data.nonlinTorq.At(dofId) = data.gravityTorq.At(dofId) + data.coriolisTorq.At(dofId);
    }
}

void ComputeCCRBABiasForcesFromReadyCoriolisTorque(const dtDynModel &model,
                                                   dtDynData &data,
                                                   const dt::Math::Vector3<dtDynReal> &gravity)
{
    ComputeGravityFromCentroidalForces(model, data, gravity);

    data.nonlinTorq.SetZero();
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        data.nonlinTorq.At(dofId) = data.gravityTorq.At(dofId) + data.coriolisTorq.At(dofId);
    }
}

void ComputeRequestedRNEADynamicsTerms(const dtDynModel &model,
                                       dtDynData &data,
                                       const dtDynRneaTermFlags flags)
{
    if (HasRNEATerm(flags, DTDYN_RNEA_TERM_COM_POS))
    {
        ComputeRootComOutputs(model, data);
    }
}

void ComputeRequestedCCRBADynamicsTerms(const dtDynModel &model,
                                        dtDynData &data,
                                        const dtDynCcrbaTermFlags flags)
{
    if (HasCCRBATerm(flags, DTDYN_CCRBA_TERM_COM_POS))
    {
        ComputeRootComOutputs(model, data);
    }

    if (HasCCRBATerm(flags, DTDYN_CCRBA_TERM_CMAMAT))
    {
        ComputeCentroidalMomentumMatrix(model, data);
    }

    if (HasCCRBATerm(flags, DTDYN_CCRBA_TERM_CMAMAT_DOT))
    {
        ComputeCentroidalMomentumMatrixDot(model, data);
    }

    if (HasCCRBATerm(flags, DTDYN_CCRBA_TERM_COM_JMAT))
    {
        ComputeComJacobianMatrixFromForces(model, data);
    }

    if (HasCCRBATerm(flags, DTDYN_CCRBA_TERM_COM_JMAT_DOT))
    {
        ComputeComJacobianMatrixDotFromForces(model, data);
    }

    if (HasCCRBATerm(flags, DTDYN_CCRBA_TERM_CENT_MOM))
    {
        ComputeCentroidalMomentumProductFromForces(model, data);
    }

    if (HasCCRBATerm(flags, DTDYN_CCRBA_TERM_COM_VEL))
    {
        ComputeComVelocityProductFromForces(model, data);
    }

    if (HasCCRBATerm(flags, DTDYN_CCRBA_TERM_MMAT_DOT))
    {
        ComputeMassMatrixDotCCRBA(model, data);
    }
}

} // namespace detail

void ComputeDynamicsRNEA(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &gravity,
                         const dtDynRneaTermFlags flags)
{
    assert(data.IsCompatible(model));
    ForwardKinematics(model, data);
    detail::ClearOptionalDynamicsOutputs(data);

    // MuJoCo-style step1 path: CRBA mass ingredients plus RNEA bias torques.
    detail::ComputeRNEAPrerequisites(model, data);
    detail::ComputeMassMatrixCRBA(model, data);
    detail::ComputeRequestedRNEADynamicsTerms(model, data, flags);
    detail::ComputeRNEABiasForces(model, data, gravity);
}

void ComputeDynamicsCCRBA(const dtDynModel &model,
                          dtDynData &data,
                          const dt::Math::Vector3<dtDynReal> &gravity,
                          const dtDynCcrbaTermFlags flags)
{
    assert(data.IsCompatible(model));
    ForwardKinematics(model, data);

    // CCRBA path: shared prerequisites, required M/C/bias terms, then requested WBC extras.
    detail::ComputeCentroidalPrerequisites(model, data);
    detail::ClearDenseDynamicsOutputs(model, data);
    detail::ComputeMassMatrixCCRBA(model, data);
    detail::ComputeCoriolisMatrixAndTorqueCCRBA(model, data);
    detail::ComputeRequestedCCRBADynamicsTerms(model, data, flags);
    detail::ComputeCCRBABiasForcesFromReadyCoriolisTorque(model, data, gravity);
}

} // namespace dt::Dynamics
