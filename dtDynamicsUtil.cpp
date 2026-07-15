/*!
 \file      dtDynamicsUtil.cpp
 \brief     It describes the utilities for the rigid body algorithms including spatial vector algebra.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2023. 09. 20
 \version   1.0.5
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_UTIL_CPP_
#define ALGORITHM_DTDYNAMICS_UTIL_CPP_

#include "algorithm/dtDynamicsLegacy/dtDynamicsUtil.h"

#include <dtMath/dtMath.h>

#include "algorithm/robotData.h"
#include "algorithm/robotMacro.h"
#include "algorithm/robotModel.h"

enum JointType
{
    FreeJoint = 0,  //	= 0 : mjJNT_FREE (MUST BE !!)
    BallJoint,      //	= 1 : mjJNT_BALL (MUST BE !!)
    PrismaticJoint, //	= 2 : mjJNT_SLIDE (MUST BE !!)
    RevoluteJoint,  //	= 3 : mjJNT_HINGE (MUST BE !!)
    UndefinedJoint  //	= 4 : User defined !!
};

enum MotionComp
{
    RB_X_AXIS = 0,
    RB_Y_AXIS,
    RB_Z_AXIS
};

inline bool CompareVec3(const SYSREAL (&vec1)[DOF3], const SYSREAL (&vec2)[DOF3])
{
    for (SYSINDEX i = 0; i < DOF3; i++)
    {
        if (vec1[i] != vec2[i]) return false;
    }
    return true;
}

inline void MultiplyConstVec(SYSREAL constant, const SYSREAL (&vector)[DOF3], SYSREAL (&result)[DOF3])
{
    for (SYSINDEX i = 0; i < 3; i++)
    {
        result[i] = constant * vector[i];
    }
}

namespace dtDynamics
{

void Quat2Rot(const SYSREAL (&quat)[DOF4], SYSREAL (&result)[DOF9])
{
    result[0 * DOF3 + 0] = quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2] - quat[3] * quat[3];
    result[0 * DOF3 + 1] = 2 * (quat[1] * quat[2] - quat[0] * quat[3]);
    result[0 * DOF3 + 2] = 2 * (quat[1] * quat[3] + quat[0] * quat[2]);

    result[1 * DOF3 + 0] = 2 * (quat[1] * quat[2] + quat[0] * quat[3]);
    result[1 * DOF3 + 1] = quat[0] * quat[0] - quat[1] * quat[1] + quat[2] * quat[2] - quat[3] * quat[3];
    result[1 * DOF3 + 2] = 2 * (quat[2] * quat[3] - quat[0] * quat[1]);

    result[2 * DOF3 + 0] = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
    result[2 * DOF3 + 1] = 2 * (quat[2] * quat[3] + quat[0] * quat[1]);
    result[2 * DOF3 + 2] = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3];
}

int rbElemetaryTransform(const RobotModel &model, RobotData &data, const SYSINDEX &bodyId, dt::Math::Vector3<SYSREAL> &pos, dt::Math::Rotation<SYSREAL> &rot, dt::Math::Vector3<SYSREAL> &compos)
{
    SYSREAL s, c;

    switch (model.rigidBody[bodyId].jntType)
    {
    case PrismaticJoint:
        rot = model.rigidBody[bodyId].bodyRot0;
        pos = model.rigidBody[bodyId].bodyPos0 + model.rigidBody[bodyId].bodyRot0.GetColVec(model.rigidBody[bodyId].jntDir) * data.qPos(model.jntqPosAdr[bodyId]);
        break;
    case RevoluteJoint:
        switch (model.rigidBody[bodyId].jntDir)
        {
        case RB_X_AXIS:
            s = std::sin(model.rigidBody[bodyId].jntAxis(RB_X_AXIS) * data.qPos(model.jntqPosAdr[bodyId]));
            c = std::cos(model.rigidBody[bodyId].jntAxis(RB_X_AXIS) * data.qPos(model.jntqPosAdr[bodyId]));

            rot << model.rigidBody[bodyId].bodyRot0(0, 0), c * model.rigidBody[bodyId].bodyRot0(0, 1) + s * model.rigidBody[bodyId].bodyRot0(0, 2), -s * model.rigidBody[bodyId].bodyRot0(0, 1) + c * model.rigidBody[bodyId].bodyRot0(0, 2),
                model.rigidBody[bodyId].bodyRot0(1, 0), c * model.rigidBody[bodyId].bodyRot0(1, 1) + s * model.rigidBody[bodyId].bodyRot0(1, 2), -s * model.rigidBody[bodyId].bodyRot0(1, 1) + c * model.rigidBody[bodyId].bodyRot0(1, 2),
                model.rigidBody[bodyId].bodyRot0(2, 0), c * model.rigidBody[bodyId].bodyRot0(2, 1) + s * model.rigidBody[bodyId].bodyRot0(2, 2), -s * model.rigidBody[bodyId].bodyRot0(2, 1) + c * model.rigidBody[bodyId].bodyRot0(2, 2);

            pos = model.rigidBody[bodyId].bodyPos0;

            break;
        case RB_Y_AXIS:
            s = std::sin(model.rigidBody[bodyId].jntAxis(RB_Y_AXIS) * data.qPos(model.jntqPosAdr[bodyId]));
            c = std::cos(model.rigidBody[bodyId].jntAxis(RB_Y_AXIS) * data.qPos(model.jntqPosAdr[bodyId]));

            rot << c * model.rigidBody[bodyId].bodyRot0(0, 0) - s * model.rigidBody[bodyId].bodyRot0(0, 2), model.rigidBody[bodyId].bodyRot0(0, 1), s * model.rigidBody[bodyId].bodyRot0(0, 0) + c * model.rigidBody[bodyId].bodyRot0(0, 2),
                c * model.rigidBody[bodyId].bodyRot0(1, 0) - s * model.rigidBody[bodyId].bodyRot0(1, 2), model.rigidBody[bodyId].bodyRot0(1, 1), s * model.rigidBody[bodyId].bodyRot0(1, 0) + c * model.rigidBody[bodyId].bodyRot0(1, 2),
                c * model.rigidBody[bodyId].bodyRot0(2, 0) - s * model.rigidBody[bodyId].bodyRot0(2, 2), model.rigidBody[bodyId].bodyRot0(2, 1), s * model.rigidBody[bodyId].bodyRot0(2, 0) + c * model.rigidBody[bodyId].bodyRot0(2, 2);

            pos = model.rigidBody[bodyId].bodyPos0;

            break;
        case RB_Z_AXIS:
            s = std::sin(model.rigidBody[bodyId].jntAxis(RB_Z_AXIS) * data.qPos(model.jntqPosAdr[bodyId]));
            c = std::cos(model.rigidBody[bodyId].jntAxis(RB_Z_AXIS) * data.qPos(model.jntqPosAdr[bodyId]));

            rot << c * model.rigidBody[bodyId].bodyRot0(0, 0) + s * model.rigidBody[bodyId].bodyRot0(0, 1), -s * model.rigidBody[bodyId].bodyRot0(0, 0) + c * model.rigidBody[bodyId].bodyRot0(0, 1), model.rigidBody[bodyId].bodyRot0(0, 2),
                c * model.rigidBody[bodyId].bodyRot0(1, 0) + s * model.rigidBody[bodyId].bodyRot0(1, 1), -s * model.rigidBody[bodyId].bodyRot0(1, 0) + c * model.rigidBody[bodyId].bodyRot0(1, 1), model.rigidBody[bodyId].bodyRot0(1, 2),
                c * model.rigidBody[bodyId].bodyRot0(2, 0) + s * model.rigidBody[bodyId].bodyRot0(2, 1), -s * model.rigidBody[bodyId].bodyRot0(2, 0) + c * model.rigidBody[bodyId].bodyRot0(2, 1), model.rigidBody[bodyId].bodyRot0(2, 2);

            pos = model.rigidBody[bodyId].bodyPos0;

            break;
        }

        break;

    case UndefinedJoint:
        rot = model.rigidBody[bodyId].bodyRot0;
        pos = model.rigidBody[bodyId].bodyPos0;
        break;
    }

    compos = pos + rot * model.rigidBody[bodyId].bodyComPos0;

    return 0;
}

dt::Math::Matrix<DOF6, DOF6, SYSREAL> InertiaCalc(const dt::Math::Matrix3<SYSREAL> &Imat, const dt::Math::Vector3<SYSREAL> &vec, const SYSREAL &mass)
{
    dt::Math::Matrix3<SYSREAL> tempSkew = vec.GetSkew();
    dt::Math::Matrix<DOF6, DOF6, SYSREAL> res;

    // res.SetBlock(0, 0, Imat - mass * tempSkew * tempSkew);
    // res.SetBlock(0, 3, mass * tempSkew);
    // res.SetBlock(3, 0, -mass * tempSkew);
    // res.SetBlock(3, 3, mass * E3);

    res(0, 0) = Imat(0, 0) + mass * (vec(1) * vec(1) + vec(2) * vec(2));
    res(0, 1) = Imat(0, 1) - mass * vec(0) * vec(1);
    res(0, 2) = Imat(0, 2) - mass * vec(0) * vec(2);

    res(1, 0) = Imat(1, 0) - mass * vec(0) * vec(1);
    res(1, 1) = Imat(1, 1) + mass * (vec(0) * vec(0) + vec(2) * vec(2));
    res(1, 2) = Imat(1, 2) - mass * vec(1) * vec(2);

    res(2, 0) = Imat(2, 0) - mass * vec(0) * vec(2);
    res(2, 1) = Imat(2, 1) - mass * vec(1) * vec(2);
    res(2, 2) = Imat(2, 2) + mass * (vec(0) * vec(0) + vec(1) * vec(1));

    res(0, 4) = -mass * vec(2);
    res(0, 5) = mass * vec(1);

    res(1, 3) = mass * vec(2);
    res(1, 5) = -mass * vec(0);

    res(2, 3) = -mass * vec(1);
    res(2, 4) = mass * vec(0);

    res(3, 1) = res(1, 3);
    res(3, 2) = res(2, 3);

    res(4, 0) = res(0, 4);
    res(4, 2) = res(2, 4);

    res(5, 0) = res(0, 5);
    res(5, 1) = res(1, 5);

    res(3, 3) = mass;
    res(4, 4) = mass;
    res(5, 5) = mass;

    return res;
}

// motion cross motion
// v1x = [wx, 0  ] * vel2
//       [v0x, wx]
//
dt::Math::Vector6<SYSREAL> CrossSpatialMotion(const dt::Math::Vector6<SYSREAL> &vel1, const dt::Math::Vector6<SYSREAL> &vel2)
{
    dt::Math::Vector6<SYSREAL> res;
    res(0) = -vel1(2) * vel2(1) + vel1(1) * vel2(2);
    res(1) = vel1(2) * vel2(0) - vel1(0) * vel2(2);
    res(2) = -vel1(1) * vel2(0) + vel1(0) * vel2(1);
    res(3) = -vel1(2) * vel2(4) + vel1(1) * vel2(5);
    res(4) = vel1(2) * vel2(3) - vel1(0) * vel2(5);
    res(5) = -vel1(1) * vel2(3) + vel1(0) * vel2(4);

    res(3) += -vel1(5) * vel2(1) + vel1(4) * vel2(2);
    res(4) += vel1(5) * vel2(0) - vel1(3) * vel2(2);
    res(5) += -vel1(4) * vel2(0) + vel1(3) * vel2(1);

    return res;
}

// motion cross force
// vx^* = [wx, v0x] * f
//        [0,   wx]
//
dt::Math::Vector6<SYSREAL> CrossSpatialForce(const dt::Math::Vector6<SYSREAL> &vel, const dt::Math::Vector6<SYSREAL> &f)
{
    dt::Math::Vector6<SYSREAL> res;
    res(0) = -vel(2) * f(1) + vel(1) * f(2);
    res(1) = vel(2) * f(0) - vel(0) * f(2);
    res(2) = -vel(1) * f(0) + vel(0) * f(1);
    res(3) = -vel(2) * f(4) + vel(1) * f(5);
    res(4) = vel(2) * f(3) - vel(0) * f(5);
    res(5) = -vel(1) * f(3) + vel(0) * f(4);

    res(0) += -vel(5) * f(4) + vel(4) * f(5);
    res(1) += vel(5) * f(3) - vel(3) * f(5);
    res(2) += -vel(4) * f(3) + vel(3) * f(4);

    return res;
}

// get skew matrix for motion for cross(motion, motion)
//
dt::Math::Matrix<DOF6, DOF6, SYSREAL> CrossSpatialMotionMatrix(const dt::Math::Vector6<SYSREAL> &vel)
{
    dt::Math::Matrix<DOF6, DOF6, SYSREAL> res;

    // res.SetBlock(0, 0, vel.GetPos().GetSkew());
    // res.SetBlock(DOF3, 0, vel.GetOri().GetSkew());
    // res.SetBlock(DOF3, DOF3, vel.GetPos().GetSkew());

    res(0, 1) = -vel(2), res(0, 2) = vel(1);
    res(1, 0) = vel(2), res(1, 2) = -vel(0);
    res(2, 0) = -vel(1), res(2, 1) = vel(0);

    res(3, 1) = -vel(5), res(3, 2) = vel(4);
    res(4, 0) = vel(5), res(4, 2) = -vel(3);
    res(5, 0) = -vel(4), res(5, 1) = vel(3);

    res(3, 4) = -vel(2), res(3, 5) = vel(1);
    res(4, 3) = vel(2), res(4, 5) = -vel(0);
    res(5, 3) = -vel(1), res(5, 4) = vel(0);

    return res;
}

// get skew matrix for force for cross(motion, force)
//
dt::Math::Matrix<DOF6, DOF6, SYSREAL> CrossSpatialMotionMatrixStar(const dt::Math::Vector6<SYSREAL> &vel)
{
    dt::Math::Matrix<DOF6, DOF6, SYSREAL> res;

    // res.SetBlock(0, 0, vel.GetPos().GetSkew());
    // res.SetBlock(0, DOF3, vel.GetOri().GetSkew());
    // res.SetBlock(DOF3, DOF3, vel.GetPos().GetSkew());

    res(0, 1) = -vel(2), res(0, 2) = vel(1);
    res(1, 0) = vel(2), res(1, 2) = -vel(0);
    res(2, 0) = -vel(1), res(2, 1) = vel(0);

    res(0, 4) = -vel(5), res(0, 5) = vel(4);
    res(1, 3) = vel(5), res(1, 5) = -vel(3);
    res(2, 3) = -vel(4), res(2, 4) = vel(3);

    res(3, 4) = -vel(2), res(3, 5) = vel(1);
    res(4, 3) = vel(2), res(4, 5) = -vel(0);
    res(5, 3) = -vel(1), res(5, 4) = vel(0);

    return res;
}

// get skew matrix for force
//
dt::Math::Matrix<DOF6, DOF6, SYSREAL> CrossSpatialForceMatrix(const dt::Math::Vector6<SYSREAL> &f)
{
    dt::Math::Matrix<DOF6, DOF6, SYSREAL> res;

    // res.SetBlock(0, 0, -f.GetPos().GetSkew());
    // res.SetBlock(0, DOF3, -f.GetOri().GetSkew());
    // res.SetBlock(DOF3, 0, -f.GetOri().GetSkew());

    res(0, 1) = f(2), res(0, 2) = -f(1);
    res(1, 0) = -f(2), res(1, 2) = f(0);
    res(2, 0) = f(1), res(2, 1) = -f(0);

    res(0, 4) = f(5), res(0, 5) = -f(4);
    res(1, 3) = -f(5), res(1, 5) = f(3);
    res(2, 3) = f(4), res(2, 4) = -f(3);

    res(3, 1) = f(5), res(3, 2) = -f(4);
    res(4, 0) = -f(5), res(4, 2) = f(3);
    res(5, 0) = f(4), res(5, 1) = -f(3);

    return res;
}

} // namespace dtDynamics

#endif // ALGORITHM_DTDYNAMICS_UTIL_CPP_
