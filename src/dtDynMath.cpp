/*!
 \file      dtDynMath.cpp
 \brief     Reusable dtDynamics math helper definitions built on dtMath.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynMath.h"

#include <cmath>
#include <limits>

namespace dt::Dynamics
{
namespace math
{

void SetIdentityQuaternion(dt::Math::Vector4<dtDynReal> &q)
{
    q.At(0) = 1.0;
    q.At(1) = 0.0;
    q.At(2) = 0.0;
    q.At(3) = 0.0;
}

dt::Math::Vector3<dtDynReal> NormalizeOrDefault(const dt::Math::Vector3<dtDynReal> &v)
{
    const dtDynReal norm = v.GetNorm();
    if (norm <= std::numeric_limits<dtDynReal>::epsilon())
    {
        dt::Math::Vector3<dtDynReal> out;
        out.At(0) = 1.0;
        out.At(1) = 0.0;
        out.At(2) = 0.0;
        return out;
    }

    return v * (1.0 / norm);
}

dt::Math::Vector4<dtDynReal> NormalizeOrIdentity(const dt::Math::Vector4<dtDynReal> &q)
{
    const dtDynReal norm = std::sqrt(q.At(0) * q.At(0) + q.At(1) * q.At(1) + q.At(2) * q.At(2) + q.At(3) * q.At(3));
    dt::Math::Vector4<dtDynReal> out;
    if (norm <= std::numeric_limits<dtDynReal>::epsilon())
    {
        SetIdentityQuaternion(out);
        return out;
    }

    out.At(0) = q.At(0) / norm;
    out.At(1) = q.At(1) / norm;
    out.At(2) = q.At(2) / norm;
    out.At(3) = q.At(3) / norm;
    return out;
}

dt::Math::Vector4<dtDynReal> MulQuat(const dt::Math::Vector4<dtDynReal> &a,
                                     const dt::Math::Vector4<dtDynReal> &b)
{
    dt::Math::Vector4<dtDynReal> out;
    out.At(0) = (((a.At(0) * b.At(0)) - (a.At(1) * b.At(1))) - (a.At(2) * b.At(2))) -
                (a.At(3) * b.At(3));
    out.At(1) = (((a.At(0) * b.At(1)) + (a.At(1) * b.At(0))) + (a.At(2) * b.At(3))) -
                (a.At(3) * b.At(2));
    out.At(2) = (((a.At(0) * b.At(2)) - (a.At(1) * b.At(3))) + (a.At(2) * b.At(0))) +
                (a.At(3) * b.At(1));
    out.At(3) = (((a.At(0) * b.At(3)) + (a.At(1) * b.At(2))) - (a.At(2) * b.At(1))) +
                (a.At(3) * b.At(0));
    return NormalizeOrIdentity(out);
}

dt::Math::Vector4<dtDynReal> AxisAngleQuat(const dt::Math::Vector3<dtDynReal> &axis,
                                           const dtDynReal angle)
{
    const dt::Math::Vector3<dtDynReal> unitAxis = NormalizeOrDefault(axis);
    const dtDynReal half = 0.5 * angle;
    const dtDynReal s = std::sin(half);

    dt::Math::Vector4<dtDynReal> out;
    out.At(0) = std::cos(half);
    out.At(1) = s * unitAxis.At(0);
    out.At(2) = s * unitAxis.At(1);
    out.At(3) = s * unitAxis.At(2);
    return NormalizeOrIdentity(out);
}

dt::Math::Vector3<dtDynReal> Rotate(const dt::Math::Vector4<dtDynReal> &q,
                                    const dt::Math::Vector3<dtDynReal> &v)
{
    dt::Math::Vector3<dtDynReal> qvec;
    qvec.At(0) = q.At(1);
    qvec.At(1) = q.At(2);
    qvec.At(2) = q.At(3);

    const dt::Math::Vector3<dtDynReal> uv = qvec & v;
    const dt::Math::Vector3<dtDynReal> uuv = qvec & uv;
    return v + uv * (2.0 * q.At(0)) + uuv * 2.0;
}

dt::Math::Vector3<dtDynReal> Rotate(const dt::Math::Matrix3<dtDynReal> &m,
                                    const dt::Math::Vector3<dtDynReal> &v)
{
    dt::Math::Vector3<dtDynReal> out;
    out.At(0) = ((m.At(0, 0) * v.At(0)) + (m.At(0, 1) * v.At(1))) + (m.At(0, 2) * v.At(2));
    out.At(1) = ((m.At(1, 0) * v.At(0)) + (m.At(1, 1) * v.At(1))) + (m.At(1, 2) * v.At(2));
    out.At(2) = ((m.At(2, 0) * v.At(0)) + (m.At(2, 1) * v.At(1))) + (m.At(2, 2) * v.At(2));
    return out;
}

dt::Math::Matrix3<dtDynReal> QuatToMat(const dt::Math::Vector4<dtDynReal> &quat)
{
    const dt::Math::Vector4<dtDynReal> q = NormalizeOrIdentity(quat);
    const dtDynReal w = q.At(0);
    const dtDynReal x = q.At(1);
    const dtDynReal y = q.At(2);
    const dtDynReal z = q.At(3);

    dt::Math::Matrix3<dtDynReal> out;
    out.At(0, 0) = (1.0 - (2.0 * ((y * y) + (z * z))));
    out.At(0, 1) = (2.0 * ((x * y) - (z * w)));
    out.At(0, 2) = (2.0 * ((x * z) + (y * w)));
    out.At(1, 0) = (2.0 * ((x * y) + (z * w)));
    out.At(1, 1) = (1.0 - (2.0 * ((x * x) + (z * z))));
    out.At(1, 2) = (2.0 * ((y * z) - (x * w)));
    out.At(2, 0) = (2.0 * ((x * z) - (y * w)));
    out.At(2, 1) = (2.0 * ((y * z) + (x * w)));
    out.At(2, 2) = (1.0 - (2.0 * ((x * x) + (y * y))));
    return out;
}

dt::Math::Vector3<dtDynReal> UnitAxis(const dtDynIndex axis)
{
    dt::Math::Vector3<dtDynReal> out;
    out.SetZero();
    out.At(axis) = 1.0;
    return out;
}

dt::Math::Vector3<dtDynReal> MatrixColumn(const dt::Math::Matrix3<dtDynReal> &m, const dtDynIndex col)
{
    dt::Math::Vector3<dtDynReal> out;
    out.At(0) = m.At(0, col);
    out.At(1) = m.At(1, col);
    out.At(2) = m.At(2, col);
    return out;
}

dt::Math::Vector3<dtDynReal> DenseMatrixColumn3(const dtDynDenseMatrix &matrix, const dtDynIndex col)
{
    dt::Math::Vector3<dtDynReal> out;
    out.At(0) = matrix.At(0, col);
    out.At(1) = matrix.At(1, col);
    out.At(2) = matrix.At(2, col);
    return out;
}

dt::Math::Matrix3<dtDynReal> DenseMatrixBlock3(const dtDynDenseMatrix &matrix,
                                               const dtDynIndex rowOffset,
                                               const dtDynIndex colOffset)
{
    dt::Math::Matrix3<dtDynReal> out;
    out.SetZero();
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < 3; ++col)
        {
            out.At(row, col) = matrix.At(rowOffset + row, colOffset + col);
        }
    }
    return out;
}

dt::Math::Vector6<dtDynReal> SpatialMotionVector(const dt::Math::Vector3<dtDynReal> &angular,
                                                 const dt::Math::Vector3<dtDynReal> &linear)
{
    dt::Math::Vector6<dtDynReal> out;
    out.At(0) = angular.At(0);
    out.At(1) = angular.At(1);
    out.At(2) = angular.At(2);
    out.At(3) = linear.At(0);
    out.At(4) = linear.At(1);
    out.At(5) = linear.At(2);
    return out;
}

dtDynReal SpatialDot(const dt::Math::Vector6<dtDynReal> &a, const dt::Math::Vector6<dtDynReal> &b)
{
    dtDynReal value = 0.0;
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        value += a.At(row) * b.At(row);
    }
    return value;
}

dtDynReal SpatialDot3(const dt::Math::Vector6<dtDynReal> &left,
                      const dt::Math::Vector6<dtDynReal> &a,
                      const dt::Math::Vector6<dtDynReal> &b)
{
    dtDynReal value = 0.0;
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        value += left.At(row) * (a.At(row) + b.At(row));
    }
    return value;
}

dt::Math::Vector6<dtDynReal> SpatialMul(const dt::Math::Matrix<6, 6, dtDynReal> &matrix,
                                        const dt::Math::Vector6<dtDynReal> &vector)
{
    dt::Math::Vector6<dtDynReal> out;
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        out.At(row) = 0.0;
        for (dtDynIndex col = 0; col < 6; ++col)
        {
            out.At(row) += matrix.At(row, col) * vector.At(col);
        }
    }
    return out;
}

dt::Math::Vector6<dtDynReal> SpatialMulTranspose(const dt::Math::Matrix<6, 6, dtDynReal> &matrix,
                                                 const dt::Math::Vector6<dtDynReal> &vector)
{
    dt::Math::Vector6<dtDynReal> out;
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        out.At(row) = 0.0;
        for (dtDynIndex col = 0; col < 6; ++col)
        {
            out.At(row) += matrix.At(col, row) * vector.At(col);
        }
    }
    return out;
}

dt::Math::Vector6<dtDynReal> ScaleSpatial(const dt::Math::Vector6<dtDynReal> &src,
                                          const dtDynReal scalar)
{
    dt::Math::Vector6<dtDynReal> out;
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        out.At(row) = scalar * src.At(row);
    }
    return out;
}

void AddSpatialInPlace(dt::Math::Vector6<dtDynReal> &dst,
                       const dt::Math::Vector6<dtDynReal> &src)
{
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        dst.At(row) += src.At(row);
    }
}

dt::Math::Vector6<dtDynReal> CrossSpatialMotion(const dt::Math::Vector6<dtDynReal> &velocity,
                                                const dt::Math::Vector6<dtDynReal> &motion)
{
    dt::Math::Vector3<dtDynReal> omega;
    dt::Math::Vector3<dtDynReal> linear;
    dt::Math::Vector3<dtDynReal> motionOmega;
    dt::Math::Vector3<dtDynReal> motionLinear;

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        omega.At(row) = velocity.At(row);
        linear.At(row) = velocity.At(row + 3);
        motionOmega.At(row) = motion.At(row);
        motionLinear.At(row) = motion.At(row + 3);
    }

    const dt::Math::Vector3<dtDynReal> angularPart = omega & motionOmega;
    const dt::Math::Vector3<dtDynReal> linearPart =
        (omega & motionLinear) + (linear & motionOmega);
    return SpatialMotionVector(angularPart, linearPart);
}

dt::Math::Matrix<6, 6, dtDynReal> CrossSpatialMotionMatrix(const dt::Math::Vector6<dtDynReal> &velocity)
{
    dt::Math::Matrix<6, 6, dtDynReal> out;
    out.SetZero();

    const dtDynReal wx = velocity.At(0);
    const dtDynReal wy = velocity.At(1);
    const dtDynReal wz = velocity.At(2);
    const dtDynReal vx = velocity.At(3);
    const dtDynReal vy = velocity.At(4);
    const dtDynReal vz = velocity.At(5);

    out.At(0, 1) = -wz;
    out.At(0, 2) = wy;
    out.At(1, 0) = wz;
    out.At(1, 2) = -wx;
    out.At(2, 0) = -wy;
    out.At(2, 1) = wx;

    out.At(3, 1) = -vz;
    out.At(3, 2) = vy;
    out.At(4, 0) = vz;
    out.At(4, 2) = -vx;
    out.At(5, 0) = -vy;
    out.At(5, 1) = vx;

    out.At(3, 4) = -wz;
    out.At(3, 5) = wy;
    out.At(4, 3) = wz;
    out.At(4, 5) = -wx;
    out.At(5, 3) = -wy;
    out.At(5, 4) = wx;
    return out;
}

dt::Math::Matrix<6, 6, dtDynReal> CrossSpatialMotionMatrixStar(const dt::Math::Vector6<dtDynReal> &velocity)
{
    return CrossSpatialMotionMatrix(velocity).Transpose() * -1.0;
}

dt::Math::Matrix<6, 6, dtDynReal> CrossSpatialMomentumMatrix(const dt::Math::Vector6<dtDynReal> &momentum)
{
    dt::Math::Matrix<6, 6, dtDynReal> out;
    out.SetZero();

    const dtDynReal hwx = momentum.At(0);
    const dtDynReal hwy = momentum.At(1);
    const dtDynReal hwz = momentum.At(2);
    const dtDynReal hvx = momentum.At(3);
    const dtDynReal hvy = momentum.At(4);
    const dtDynReal hvz = momentum.At(5);

    out.At(0, 1) = hwz;
    out.At(0, 2) = -hwy;
    out.At(1, 0) = -hwz;
    out.At(1, 2) = hwx;
    out.At(2, 0) = hwy;
    out.At(2, 1) = -hwx;

    out.At(0, 4) = hvz;
    out.At(0, 5) = -hvy;
    out.At(1, 3) = -hvz;
    out.At(1, 5) = hvx;
    out.At(2, 3) = hvy;
    out.At(2, 4) = -hvx;

    out.At(3, 1) = hvz;
    out.At(3, 2) = -hvy;
    out.At(4, 0) = -hvz;
    out.At(4, 2) = hvx;
    out.At(5, 0) = hvy;
    out.At(5, 1) = -hvx;
    return out;
}

dtDynIndex FindBodyLastDof(const dtDynModel &model, dtDynIndex body)
{
    while (body > 0)
    {
        if (model.body_dofnum[static_cast<std::size_t>(body)] > 0)
        {
            return model.body_dofadr[static_cast<std::size_t>(body)] +
                   model.body_dofnum[static_cast<std::size_t>(body)] - 1;
        }
        body = model.body_parentid[static_cast<std::size_t>(body)];
    }

    return DTDYN_INVALID_INDEX;
}

dt::Math::Matrix3<dtDynReal> BodyInertiaInWorld(const dtDynModel &model,
                                                const dtDynData &data,
                                                const dtDynIndex bodyId)
{
    const dt::Math::Matrix3<dtDynReal> &rot = data.ximat[static_cast<std::size_t>(bodyId)];
    const dt::Math::Vector3<dtDynReal> &diag = model.body_inertia[static_cast<std::size_t>(bodyId)];
    dt::Math::Matrix3<dtDynReal> out;

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < 3; ++col)
        {
            out.At(row, col) = rot.At(row, 0) * diag.At(0) * rot.At(col, 0) +
                               rot.At(row, 1) * diag.At(1) * rot.At(col, 1) +
                               rot.At(row, 2) * diag.At(2) * rot.At(col, 2);
        }
    }

    return out;
}

dt::Math::Matrix<6, 6, dtDynReal> SpatialInertiaAtWorldOrigin(const dtDynReal mass,
                                                              const dt::Math::Vector3<dtDynReal> &com,
                                                              const dt::Math::Matrix3<dtDynReal> &rotInertia)
{
    dt::Math::Matrix<6, 6, dtDynReal> out;
    out.SetZero();

    const dtDynReal c0 = com.At(0);
    const dtDynReal c1 = com.At(1);
    const dtDynReal c2 = com.At(2);
    const dtDynReal c2sum = (((c0 * c0) + (c1 * c1)) + (c2 * c2));

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < 3; ++col)
        {
            out.At(row, col) = rotInertia.At(row, col);
            if (row == col)
            {
                out.At(row, col) += mass * c2sum;
            }
            out.At(row, col) -= ((mass * com.At(row)) * com.At(col));
            out.At(row + 3, col + 3) = ((row == col) ? mass : 0.0);
        }
    }

    out.At(0, 4) = -mass * c2;
    out.At(0, 5) = mass * c1;
    out.At(1, 3) = mass * c2;
    out.At(1, 5) = -mass * c0;
    out.At(2, 3) = -mass * c1;
    out.At(2, 4) = mass * c0;

    out.At(3, 1) = mass * c2;
    out.At(3, 2) = -mass * c1;
    out.At(4, 0) = -mass * c2;
    out.At(4, 2) = mass * c0;
    out.At(5, 0) = mass * c1;
    out.At(5, 1) = -mass * c0;

    return out;
}

dt::Math::Matrix<6, 6, dtDynReal> SpatialInertiaAtReference(const dtDynReal mass,
                                                            const dt::Math::Vector3<dtDynReal> &bodyCom,
                                                            const dt::Math::Vector3<dtDynReal> &reference,
                                                            const dt::Math::Matrix3<dtDynReal> &rotInertia)
{
    return SpatialInertiaAtWorldOrigin(mass, bodyCom - reference, rotInertia);
}

dtDynReal QuadraticInertia(const dt::Math::Vector3<dtDynReal> &left,
                           const dt::Math::Matrix3<dtDynReal> &inertia,
                           const dt::Math::Vector3<dtDynReal> &right)
{
    dtDynReal value = 0.0;
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < 3; ++col)
        {
            value += left.At(row) * inertia.At(row, col) * right.At(col);
        }
    }
    return value;
}

} // namespace math
} // namespace dt::Dynamics
