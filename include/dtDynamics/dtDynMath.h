/*!
 \file      dtDynMath.h
 \brief     Reusable dtDynamics math helper declarations built on dtMath.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.2
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNMATH_H_
#define ALGORITHM_DTDYNAMICS_DTDYNMATH_H_

#include "dtDynamics/dtDynData.h"

namespace dt::Dynamics
{
namespace math
{

/*!
 \name Quaternion and rotation helpers
 \details Quaternions use scalar-first order \f$[w,x,y,z]\f$. Helpers normalize generated
          quaternions where documented and do not allocate heap memory themselves.
*/
///@{
/*! \brief Set q to the identity quaternion \f$[1,0,0,0]\f$. */
void SetIdentityQuaternion(dt::Math::Vector4<dtDynReal> &q);
/*! \brief Return v normalized, or \f$[1,0,0]\f$ when v is numerically zero. */
dt::Math::Vector3<dtDynReal> NormalizeOrDefault(const dt::Math::Vector3<dtDynReal> &v);
/*! \brief Return q normalized, or identity when q is numerically zero. */
dt::Math::Vector4<dtDynReal> NormalizeOrIdentity(const dt::Math::Vector4<dtDynReal> &q);
/*! \brief Return the normalized Hamilton product \f$a \otimes b\f$. */
dt::Math::Vector4<dtDynReal> MulQuat(const dt::Math::Vector4<dtDynReal> &a,
                                     const dt::Math::Vector4<dtDynReal> &b);
/*! \brief Return a normalized quaternion for a rotation of angle radians about axis. */
dt::Math::Vector4<dtDynReal> AxisAngleQuat(const dt::Math::Vector3<dtDynReal> &axis, dtDynReal angle);
/*! \brief Rotate vector v by quaternion q. q is assumed to be normalized by the caller. */
dt::Math::Vector3<dtDynReal> Rotate(const dt::Math::Vector4<dtDynReal> &q,
                                    const dt::Math::Vector3<dtDynReal> &v);
/*! \brief Multiply a 3x3 rotation matrix by vector v. */
dt::Math::Vector3<dtDynReal> Rotate(const dt::Math::Matrix3<dtDynReal> &m,
                                    const dt::Math::Vector3<dtDynReal> &v);
/*! \brief Convert a scalar-first quaternion to a 3x3 rotation matrix after normalization. */
dt::Math::Matrix3<dtDynReal> QuatToMat(const dt::Math::Vector4<dtDynReal> &quat);
/*! \brief Return Cartesian unit axis 0, 1, or 2.
    \pre axis must be in [0, 2]. */
dt::Math::Vector3<dtDynReal> UnitAxis(dtDynIndex axis);
/*! \brief Return column col from a 3x3 matrix.
    \pre col must be in [0, 2]. */
dt::Math::Vector3<dtDynReal> MatrixColumn(const dt::Math::Matrix3<dtDynReal> &m, dtDynIndex col);
///@}

/*!
 \name dtMath dynamic matrix access helpers
 \details These helpers assume the caller has already ensured the requested rows/columns
          exist. They are intended for fixed-size extraction from dtDynDenseMatrix storage.
*/
///@{
/*! \brief Return rows 0..2 from dynamic matrix column col.
    \pre \p matrix has at least three rows and \p col is valid. */
dt::Math::Vector3<dtDynReal> DenseMatrixColumn3(const dtDynDenseMatrix &matrix, dtDynIndex col);
/*! \brief Return a 3x3 block starting at rowOffset, colOffset.
    \pre The complete requested 3x3 block lies within \p matrix. */
dt::Math::Matrix3<dtDynReal> DenseMatrixBlock3(const dtDynDenseMatrix &matrix,
                                               dtDynIndex rowOffset,
                                               dtDynIndex colOffset);
///@}

/*!
 \name Spatial vector algebra helpers
 \details Spatial vectors and matrices in these helpers use row order \f$[\omega; v]\f$:
          angular rows 0..2 followed by linear rows 3..5. Matrix helpers return dense 6x6
          dtMath values by value and are intended for internal/reusable dynamics kernels, not
          for hidden ownership of public model data.
*/
///@{
/*! \brief Pack angular and linear 3-vectors into one spatial motion vector. */
dt::Math::Vector6<dtDynReal> SpatialMotionVector(const dt::Math::Vector3<dtDynReal> &angular,
                                                 const dt::Math::Vector3<dtDynReal> &linear);
/*! \brief Return the Euclidean dot product of two spatial vectors. */
dtDynReal SpatialDot(const dt::Math::Vector6<dtDynReal> &a, const dt::Math::Vector6<dtDynReal> &b);
/*! \brief Return \f$\sum_i left_i (a_i + b_i)\f$ for six spatial rows. */
dtDynReal SpatialDot3(const dt::Math::Vector6<dtDynReal> &left,
                      const dt::Math::Vector6<dtDynReal> &a,
                      const dt::Math::Vector6<dtDynReal> &b);
/*! \brief Return matrix * vector for a 6x6 spatial matrix. */
dt::Math::Vector6<dtDynReal> SpatialMul(const dt::Math::Matrix<6, 6, dtDynReal> &matrix,
                                        const dt::Math::Vector6<dtDynReal> &vector);
/*! \brief Return matrix^T * vector for a 6x6 spatial matrix. */
dt::Math::Vector6<dtDynReal> SpatialMulTranspose(const dt::Math::Matrix<6, 6, dtDynReal> &matrix,
                                                 const dt::Math::Vector6<dtDynReal> &vector);
/*! \brief Return scalar * src for all six spatial rows. */
dt::Math::Vector6<dtDynReal> ScaleSpatial(const dt::Math::Vector6<dtDynReal> &src,
                                          dtDynReal scalar);
/*! \brief Add src into dst in place for all six spatial rows. */
void AddSpatialInPlace(dt::Math::Vector6<dtDynReal> &dst,
                       const dt::Math::Vector6<dtDynReal> &src);
/*! \brief Return the spatial motion cross product velocity x motion. */
dt::Math::Vector6<dtDynReal> CrossSpatialMotion(const dt::Math::Vector6<dtDynReal> &velocity,
                                                const dt::Math::Vector6<dtDynReal> &motion);
/*! \brief Return the 6x6 matrix representation of the spatial motion cross product. */
dt::Math::Matrix<6, 6, dtDynReal> CrossSpatialMotionMatrix(const dt::Math::Vector6<dtDynReal> &velocity);
/*! \brief Return the dual spatial cross-product matrix \f$-crm(v)^T\f$ for force/momentum rows. */
dt::Math::Matrix<6, 6, dtDynReal> CrossSpatialMotionMatrixStar(const dt::Math::Vector6<dtDynReal> &velocity);
/*! \brief Return the matrix form used for momentum cross-product terms. */
dt::Math::Matrix<6, 6, dtDynReal> CrossSpatialMomentumMatrix(const dt::Math::Vector6<dtDynReal> &momentum);
///@}

/*!
 \name Model inertia helpers
 \details These helpers operate on already-compiled tree-model arrays and already-updated
          kinematic data. Index arguments must refer to valid entries unless a function
          explicitly documents DTDYN_INVALID_INDEX behavior.
*/
///@{
/*! \brief Return the last dof index owned by body or the nearest movable ancestor.
    \details Returns DTDYN_INVALID_INDEX if neither body nor any non-world ancestor owns dofs. */
dtDynIndex FindBodyLastDof(const dtDynModel &model, dtDynIndex body);
/*! \brief Return a body's diagonal inertial tensor rotated into the world frame. */
dt::Math::Matrix3<dtDynReal> BodyInertiaInWorld(const dtDynModel &model,
                                                const dtDynData &data,
                                                dtDynIndex bodyId);
/*! \brief Build a 6x6 spatial inertia about the world origin for a body CoM and rotational inertia.
    \details The returned matrix uses \f$[\omega; v]\f$ row order and applies the parallel-axis term. */
dt::Math::Matrix<6, 6, dtDynReal> SpatialInertiaAtWorldOrigin(dtDynReal mass,
                                                              const dt::Math::Vector3<dtDynReal> &com,
                                                              const dt::Math::Matrix3<dtDynReal> &rotInertia);
/*! \brief Build a 6x6 spatial inertia about reference from a body CoM and rotational inertia. */
dt::Math::Matrix<6, 6, dtDynReal> SpatialInertiaAtReference(dtDynReal mass,
                                                            const dt::Math::Vector3<dtDynReal> &bodyCom,
                                                            const dt::Math::Vector3<dtDynReal> &reference,
                                                            const dt::Math::Matrix3<dtDynReal> &rotInertia);
/*! \brief Return the quadratic form \f$left^T inertia\, right\f$. */
dtDynReal QuadraticInertia(const dt::Math::Vector3<dtDynReal> &left,
                           const dt::Math::Matrix3<dtDynReal> &inertia,
                           const dt::Math::Vector3<dtDynReal> &right);
///@}

} // namespace math
} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNMATH_H_
