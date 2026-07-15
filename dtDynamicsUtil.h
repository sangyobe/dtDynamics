/*!
 \file      dtDynamicsUtil.h
 \brief     It describes the utilities for the rigid body algorithms including spatial vector algebra.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2023. 09. 20
 \version   1.0.5
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_UTIL_H_
#define ALGORITHM_DTDYNAMICS_UTIL_H_

#include <dtMath/dtMath.h>

#include "algorithm/robotData.h"
#include "algorithm/robotMacro.h"
#include "algorithm/robotModel.h"

namespace dtDynamics
{

void Quat2Rot(const SYSREAL (&quat)[DOF4], SYSREAL (&result)[DOF9]);

int rbElemetaryTransform(const RobotModel &model, RobotData &data, const SYSINDEX &bodyId, dt::Math::Vector3<SYSREAL> &pos, dt::Math::Rotation<SYSREAL> &rot, dt::Math::Vector3<SYSREAL> &compos);

dt::Math::Matrix<DOF6, DOF6, SYSREAL> InertiaCalc(const dt::Math::Matrix3<SYSREAL> &Imat, const dt::Math::Vector3<SYSREAL> &vec, const SYSREAL &mass);

dt::Math::Vector6<SYSREAL> CrossSpatialMotion(const dt::Math::Vector6<SYSREAL> &vel1, const dt::Math::Vector6<SYSREAL> &vel2);
dt::Math::Vector6<SYSREAL> CrossSpatialForce(const dt::Math::Vector6<SYSREAL> &vel, const dt::Math::Vector6<SYSREAL> &f);

dt::Math::Matrix<DOF6, DOF6, SYSREAL> CrossSpatialMotionMatrix(const dt::Math::Vector6<SYSREAL> &vel);
dt::Math::Matrix<DOF6, DOF6, SYSREAL> CrossSpatialMotionMatrixStar(const dt::Math::Vector6<SYSREAL> &vel);
dt::Math::Matrix<DOF6, DOF6, SYSREAL> CrossSpatialForceMatrix(const dt::Math::Vector6<SYSREAL> &f);

} // namespace dtDynamics

#endif // ALGORITHM_DTDYNAMICS_UTIL_H_