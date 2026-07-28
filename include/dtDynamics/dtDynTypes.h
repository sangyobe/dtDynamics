/*!
 \file      dtDynTypes.h
 \brief     Native dtDynamics model and data type definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.1
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNTYPES_H_
#define ALGORITHM_DTDYNAMICS_DTDYNTYPES_H_

#include "dtDynamics/dtArray.h"
#include "config.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <dtMath/dtMath.h>

namespace dt::Dynamics
{

using dtDynIndex = int;    //!< Signed index type used by public model/data arrays.
using dtDynReal = SYSREAL; //!< Scalar type selected by the package configuration.

static constexpr dtDynIndex DTDYN_INVALID_INDEX = -1; //!< Invalid model/data index sentinel.

/*! \brief Clamp a signed dtDynIndex to a non-negative std::size_t.
    \details Values less than or equal to zero map to 0. This helper is for sizes/counts,
             not for validating model indices where DTDYN_INVALID_INDEX must stay invalid. */
std::size_t ToSize(dtDynIndex value);

using dtDynDenseVector = dt::Math::VectorX<dtDynReal>; //!< Dynamic dense vector storage from dtMath.
using dtDynDenseMatrix = dt::Math::MatrixX<dtDynReal>; //!< Dynamic dense matrix storage from dtMath.

/*! \brief Convert a checked runtime dimension to the dtMath dynamic-dimension type.
    \pre value must be in the range [0, UINT16_MAX]; debug builds assert this contract. */
uint16_t ToDtMathDim(dtDynIndex value);
/*! \brief Resize a dtMath dynamic vector to dim rows.
    \pre dim must satisfy ToDtMathDim(). Existing values are subject to dtMath ReSize semantics. */
void ResizeDenseVector(dtDynDenseVector &vector, dtDynIndex dim);
/*! \brief Resize a dtMath dynamic matrix to rows by cols.
    \pre rows and cols must satisfy ToDtMathDim(). Existing values are subject to dtMath ReSize semantics. */
void ResizeDenseMatrix(dtDynDenseMatrix &matrix, dtDynIndex rows, dtDynIndex cols);
/*! \brief Resize a dtMath dynamic vector only when its dimension differs from dim. */
void EnsureDenseVectorSize(dtDynDenseVector &vector, dtDynIndex dim);
/*! \brief Resize a dtMath dynamic matrix only when its dimensions differ from rows by cols. */
void EnsureDenseMatrixSize(dtDynDenseMatrix &matrix, dtDynIndex rows, dtDynIndex cols);

/*!
 \brief Supported tree-joint coordinate types and their public index order.
 \details The qpos/qvel sizes intentionally match the MJCF-style convention used by the
          dtDynamics XML subset: free = 7/6, ball = 4/3, slide = 1/1, hinge = 1/1.
*/
enum class dtDynJointType : dtDynIndex
{
    Free = 0,  //!< 6-DoF floating joint with position plus quaternion qpos.
    Ball = 1,  //!< 3-DoF rotational joint with quaternion qpos.
    Slide = 2, //!< 1-DoF prismatic joint.
    Hinge = 3  //!< 1-DoF revolute joint.
};

/*! \brief Supported named object categories used by public lookup APIs. */
enum class dtDynObjectType : dtDynIndex
{
    Body = 1,     //!< Body object.
    Joint = 2,    //!< Joint object.
    Site = 3,     //!< Site/frame object.
    Actuator = 4, //!< Actuator object.
    Equality = 5  //!< Reserved category; equality/closed-chain constraints are not supported by native dynamics.
};

/*! \brief qpos/qvel dimensions for one joint type. */
struct dtDynJointDofSize
{
    dtDynIndex qposSize; //!< Number of generalized-position coordinates contributed by the joint.
    dtDynIndex qvelSize; //!< Number of generalized-velocity coordinates contributed by the joint.
};

/*! \brief Return the qpos/qvel dimensions for a supported joint type.
    \details Unknown enum values return {0, 0}. */
dtDynJointDofSize GetJointDofSize(dtDynJointType type);

/*!
 \brief Public generalized-velocity basis used by dtDynamics floating-base data.
 \details For models with free bases, qvel is ordered as
          \f$[v_{base}^{G}, \omega_{base}^{B}, \dot{q}_{joint}]\f$: base linear velocity
          expressed in the global frame, then base angular velocity expressed in the base
          frame, followed by joint velocities.
*/
struct dtDynVelocityConvention
{
    static constexpr bool baseLinearVelocityInGlobal = true; //!< Base linear velocity is expressed in global frame.
    static constexpr bool baseAngularVelocityInBase = true;  //!< Base angular velocity is expressed in base frame.
};

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNTYPES_H_
