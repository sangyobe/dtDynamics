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
    q(0) = 1.0;
    q(1) = 0.0;
    q(2) = 0.0;
    q(3) = 0.0;
}

dt::Math::Vector3<dtDynReal> NormalizeOrDefault(const dt::Math::Vector3<dtDynReal> &v)
{
    const dtDynReal norm = v.GetNorm();
    if (norm <= std::numeric_limits<dtDynReal>::epsilon())
    {
        dt::Math::Vector3<dtDynReal> out;
        out(0) = 1.0;
        out(1) = 0.0;
        out(2) = 0.0;
        return out;
    }

    return v * (1.0 / norm);
}

dt::Math::Vector4<dtDynReal> NormalizeOrIdentity(const dt::Math::Vector4<dtDynReal> &q)
{
    const dtDynReal norm = std::sqrt(q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3));
    dt::Math::Vector4<dtDynReal> out;
    if (norm <= std::numeric_limits<dtDynReal>::epsilon())
    {
        SetIdentityQuaternion(out);
        return out;
    }

    out(0) = q(0) / norm;
    out(1) = q(1) / norm;
    out(2) = q(2) / norm;
    out(3) = q(3) / norm;
    return out;
}

dt::Math::Vector4<dtDynReal> MulQuat(const dt::Math::Vector4<dtDynReal> &a,
                                     const dt::Math::Vector4<dtDynReal> &b)
{
    dt::Math::Vector4<dtDynReal> out;
    out(0) = a(0) * b(0) - a(1) * b(1) - a(2) * b(2) - a(3) * b(3);
    out(1) = a(0) * b(1) + a(1) * b(0) + a(2) * b(3) - a(3) * b(2);
    out(2) = a(0) * b(2) - a(1) * b(3) + a(2) * b(0) + a(3) * b(1);
    out(3) = a(0) * b(3) + a(1) * b(2) - a(2) * b(1) + a(3) * b(0);
    return NormalizeOrIdentity(out);
}

dt::Math::Vector4<dtDynReal> AxisAngleQuat(const dt::Math::Vector3<dtDynReal> &axis,
                                           const dtDynReal angle)
{
    const dt::Math::Vector3<dtDynReal> unitAxis = NormalizeOrDefault(axis);
    const dtDynReal half = 0.5 * angle;
    const dtDynReal s = std::sin(half);

    dt::Math::Vector4<dtDynReal> out;
    out(0) = std::cos(half);
    out(1) = s * unitAxis(0);
    out(2) = s * unitAxis(1);
    out(3) = s * unitAxis(2);
    return NormalizeOrIdentity(out);
}

dt::Math::Vector3<dtDynReal> Rotate(const dt::Math::Vector4<dtDynReal> &q,
                                    const dt::Math::Vector3<dtDynReal> &v)
{
    dt::Math::Vector3<dtDynReal> qvec;
    qvec(0) = q(1);
    qvec(1) = q(2);
    qvec(2) = q(3);

    const dt::Math::Vector3<dtDynReal> uv = qvec & v;
    const dt::Math::Vector3<dtDynReal> uuv = qvec & uv;
    return v + uv * (2.0 * q(0)) + uuv * 2.0;
}

dt::Math::Vector3<dtDynReal> Rotate(const dt::Math::Matrix3<dtDynReal> &m,
                                    const dt::Math::Vector3<dtDynReal> &v)
{
    dt::Math::Vector3<dtDynReal> out;
    out(0) = m(0, 0) * v(0) + m(0, 1) * v(1) + m(0, 2) * v(2);
    out(1) = m(1, 0) * v(0) + m(1, 1) * v(1) + m(1, 2) * v(2);
    out(2) = m(2, 0) * v(0) + m(2, 1) * v(1) + m(2, 2) * v(2);
    return out;
}

dt::Math::Matrix3<dtDynReal> QuatToMat(const dt::Math::Vector4<dtDynReal> &quat)
{
    const dt::Math::Vector4<dtDynReal> q = NormalizeOrIdentity(quat);
    const dtDynReal w = q(0);
    const dtDynReal x = q(1);
    const dtDynReal y = q(2);
    const dtDynReal z = q(3);

    dt::Math::Matrix3<dtDynReal> out;
    out(0, 0) = (1.0 - (2.0 * ((y * y) + (z * z))));
    out(0, 1) = (2.0 * ((x * y) - (z * w)));
    out(0, 2) = (2.0 * ((x * z) + (y * w)));
    out(1, 0) = (2.0 * ((x * y) + (z * w)));
    out(1, 1) = (1.0 - (2.0 * ((x * x) + (z * z))));
    out(1, 2) = (2.0 * ((y * z) - (x * w)));
    out(2, 0) = (2.0 * ((x * z) - (y * w)));
    out(2, 1) = (2.0 * ((y * z) + (x * w)));
    out(2, 2) = (1.0 - (2.0 * ((x * x) + (y * y))));
    return out;
}

dt::Math::Vector3<dtDynReal> UnitAxis(const dtDynIndex axis)
{
    dt::Math::Vector3<dtDynReal> out;
    out.SetZero();
    out(axis) = 1.0;
    return out;
}

dt::Math::Vector3<dtDynReal> MatrixColumn(const dt::Math::Matrix3<dtDynReal> &m, const dtDynIndex col)
{
    dt::Math::Vector3<dtDynReal> out;
    out(0) = m(0, col);
    out(1) = m(1, col);
    out(2) = m(2, col);
    return out;
}

dt::Math::Vector3<dtDynReal> DenseMatrixColumn3(const dtDynDenseMatrix &matrix, const dtDynIndex col)
{
    dt::Math::Vector3<dtDynReal> out;
    out(0) = matrix(0, col);
    out(1) = matrix(1, col);
    out(2) = matrix(2, col);
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
            out(row, col) = matrix(rowOffset + row, colOffset + col);
        }
    }
    return out;
}

dt::Math::Vector6<dtDynReal> SpatialMotionVector(const dt::Math::Vector3<dtDynReal> &angular,
                                                 const dt::Math::Vector3<dtDynReal> &linear)
{
    dt::Math::Vector6<dtDynReal> out;
    out(0) = angular(0);
    out(1) = angular(1);
    out(2) = angular(2);
    out(3) = linear(0);
    out(4) = linear(1);
    out(5) = linear(2);
    return out;
}

dtDynReal SpatialDot(const dt::Math::Vector6<dtDynReal> &a, const dt::Math::Vector6<dtDynReal> &b)
{
    dtDynReal value = 0.0;
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        value += a(row) * b(row);
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
        value += left(row) * (a(row) + b(row));
    }
    return value;
}

dt::Math::Vector6<dtDynReal> SpatialMul(const dt::Math::Matrix<6, 6, dtDynReal> &matrix,
                                        const dt::Math::Vector6<dtDynReal> &vector)
{
    dt::Math::Vector6<dtDynReal> out;
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        out(row) = 0.0;
        for (dtDynIndex col = 0; col < 6; ++col)
        {
            out(row) += matrix(row, col) * vector(col);
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
        out(row) = 0.0;
        for (dtDynIndex col = 0; col < 6; ++col)
        {
            out(row) += matrix(col, row) * vector(col);
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
        out(row) = scalar * src(row);
    }
    return out;
}

void AddSpatialInPlace(dt::Math::Vector6<dtDynReal> &dst,
                       const dt::Math::Vector6<dtDynReal> &src)
{
    for (dtDynIndex row = 0; row < 6; ++row)
    {
        dst(row) += src(row);
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
        omega(row) = velocity(row);
        linear(row) = velocity(row + 3);
        motionOmega(row) = motion(row);
        motionLinear(row) = motion(row + 3);
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

    const dtDynReal wx = velocity(0);
    const dtDynReal wy = velocity(1);
    const dtDynReal wz = velocity(2);
    const dtDynReal vx = velocity(3);
    const dtDynReal vy = velocity(4);
    const dtDynReal vz = velocity(5);

    out(0, 1) = -wz;
    out(0, 2) = wy;
    out(1, 0) = wz;
    out(1, 2) = -wx;
    out(2, 0) = -wy;
    out(2, 1) = wx;

    out(3, 1) = -vz;
    out(3, 2) = vy;
    out(4, 0) = vz;
    out(4, 2) = -vx;
    out(5, 0) = -vy;
    out(5, 1) = vx;

    out(3, 4) = -wz;
    out(3, 5) = wy;
    out(4, 3) = wz;
    out(4, 5) = -wx;
    out(5, 3) = -wy;
    out(5, 4) = wx;
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

    const dtDynReal hwx = momentum(0);
    const dtDynReal hwy = momentum(1);
    const dtDynReal hwz = momentum(2);
    const dtDynReal hvx = momentum(3);
    const dtDynReal hvy = momentum(4);
    const dtDynReal hvz = momentum(5);

    out(0, 1) = hwz;
    out(0, 2) = -hwy;
    out(1, 0) = -hwz;
    out(1, 2) = hwx;
    out(2, 0) = hwy;
    out(2, 1) = -hwx;

    out(0, 4) = hvz;
    out(0, 5) = -hvy;
    out(1, 3) = -hvz;
    out(1, 5) = hvx;
    out(2, 3) = hvy;
    out(2, 4) = -hvx;

    out(3, 1) = hvz;
    out(3, 2) = -hvy;
    out(4, 0) = -hvz;
    out(4, 2) = hvx;
    out(5, 0) = hvy;
    out(5, 1) = -hvx;
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
            out(row, col) = rot(row, 0) * diag(0) * rot(col, 0) +
                            rot(row, 1) * diag(1) * rot(col, 1) +
                            rot(row, 2) * diag(2) * rot(col, 2);
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

    const dtDynReal c0 = com(0);
    const dtDynReal c1 = com(1);
    const dtDynReal c2 = com(2);
    const dtDynReal c2sum = (((c0 * c0) + (c1 * c1)) + (c2 * c2));

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < 3; ++col)
        {
            out(row, col) = rotInertia(row, col);
            if (row == col)
            {
                out(row, col) += mass * c2sum;
            }
            out(row, col) -= ((mass * com(row)) * com(col));
            out(row + 3, col + 3) = ((row == col) ? mass : 0.0);
        }
    }

    out(0, 4) = -mass * c2;
    out(0, 5) = mass * c1;
    out(1, 3) = mass * c2;
    out(1, 5) = -mass * c0;
    out(2, 3) = -mass * c1;
    out(2, 4) = mass * c0;

    out(3, 1) = mass * c2;
    out(3, 2) = -mass * c1;
    out(4, 0) = -mass * c2;
    out(4, 2) = mass * c0;
    out(5, 0) = mass * c1;
    out(5, 1) = -mass * c0;

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
            value += left(row) * inertia(row, col) * right(col);
        }
    }
    return value;
}

} // namespace math
} // namespace dt::Dynamics
