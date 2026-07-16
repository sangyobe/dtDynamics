/*!
 \file      dtDynKinematics.h
 \brief     Native dtDynamics kinematics and Jacobian declarations.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.0
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNKINEMATICS_H_
#define ALGORITHM_DTDYNAMICS_DTDYNKINEMATICS_H_

#include "dtDynamics/dtArray.h"
#include "dtDynamics/dtDynData.h"

namespace dt::Dynamics
{

/*!
 \brief Compute body, joint, inertial, and site world transforms from qpos.
 \details Updates world-space body and inertial transforms, joint axes and
  anchors, and site transforms from qpos. It does not fill subtree_com, cvel,
  cdof, or cdof_dot; those work arrays are prepared by the dynamics passes.
 \pre data has been resized for \p model and data.qpos has model.nq entries.
 \param model Compiled immutable model.
 \param data Mutable runtime state and kinematic output buffers.
*/
void ForwardKinematics(const dtDynModel &model, dtDynData &data);

/*! \brief Compute translational and rotational Jacobian for one body origin.
 \details Outputs have shape \f$3 \times n_v\f$ and map generalized velocity to
  world-frame linear/angular body-origin velocity.
 \pre ForwardKinematics(model, data) has been run for the current state.
*/
void ComputeBodyJacobian(const dtDynModel &model,
                         const dtDynData &data,
                         dtDynIndex bodyId,
                         dtDynDenseMatrix &jacp,
                         dtDynDenseMatrix &jacr);
/*! \brief Compute translational and rotational Jacobian for one site origin.
 \details Outputs have shape \f$3 \times n_v\f$ and map generalized velocity to
  world-frame linear/angular site velocity.
 \pre ForwardKinematics(model, data) has been run for the current state.
*/
void ComputeSiteJacobian(const dtDynModel &model,
                         const dtDynData &data,
                         dtDynIndex siteId,
                         dtDynDenseMatrix &jacp,
                         dtDynDenseMatrix &jacr);
/*! \brief Compute translational and rotational Jacobian for an arbitrary point on a body.
 \details \p point is expressed in world coordinates. Outputs have shape
  \f$3 \times n_v\f$ and map generalized velocity to point linear/angular
  velocity.
 \pre ForwardKinematics(model, data) has been run for the current state.
*/
void ComputePointJacobian(const dtDynModel &model,
                          const dtDynData &data,
                          dtDynIndex bodyId,
                          const dt::Math::Vector3<dtDynReal> &point,
                          dtDynDenseMatrix &jacp,
                          dtDynDenseMatrix &jacr);
/*! \brief Compute the time derivative of a body-fixed point Jacobian.
 \details \p point is expressed in world coordinates. Outputs are world-frame
  \f$3 \times n_v\f$ linear and angular Jacobian derivatives.
 \pre \p data is compatible with \p model and has been refreshed for the current
  qpos/qvel state by ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
 \note ForwardKinematics() alone does not populate the subtree-CoM,
  motion-subspace, velocity, and derivative buffers used by this routine.
*/
void ComputePointJacobianDot(const dtDynModel &model,
                             const dtDynData &data,
                             dtDynIndex bodyId,
                             const dt::Math::Vector3<dtDynReal> &point,
                             dtDynDenseMatrix &jacpDot,
                             dtDynDenseMatrix &jacrDot);
/*! \brief Compute \f$\dot{J}(q,\dot{q})\dot{q}\f$ for a body-fixed point.
 \details \p point is expressed in world coordinates. \p linear and \p angular
  are world-frame 3-vectors with linear and angular acceleration units.
 \pre \p data is compatible with \p model and has been refreshed for the current
  qpos/qvel state by ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
void ComputePointJacobianDotTimesVelocity(const dtDynModel &model,
                                          const dtDynData &data,
                                          dtDynIndex bodyId,
                                          const dt::Math::Vector3<dtDynReal> &point,
                                          dt::Math::Vector3<dtDynReal> &linear,
                                          dt::Math::Vector3<dtDynReal> &angular);
/*!
 \brief Compute requested point Jacobian, derivative, and Jdot*qvel outputs in one traversal.
 \details Pass nullptr for outputs that are not needed. Each non-null matrix has
  shape \f$3 \times n_v\f$; Jdot*qvel outputs are world-frame 3-vectors.
  This helper avoids repeated traversal for the same body-fixed point.
 \pre ForwardKinematics(model, data) has been run for the current qpos state.
 \pre If any derivative or Jdot*qvel output is non-null, \p data has also been
  refreshed for the current qpos/qvel state by ComputeDynamicsRNEA() or
  ComputeDynamicsCCRBA().
*/
void ComputePointJacobianTerms(const dtDynModel &model,
                               const dtDynData &data,
                               dtDynIndex bodyId,
                               const dt::Math::Vector3<dtDynReal> &point,
                               dtDynDenseMatrix *jacp,
                               dtDynDenseMatrix *jacr,
                               dtDynDenseMatrix *jacpDot,
                               dtDynDenseMatrix *jacrDot,
                               dt::Math::Vector3<dtDynReal> *linearJdotQvel,
                               dt::Math::Vector3<dtDynReal> *angularJdotQvel);
/*! \brief Compute selected body-origin Jacobian terms in one traversal.
 \details Pass nullptr for outputs that are not needed. Each non-null matrix has
  shape \f$3 \times n_v\f$; Jdot*qvel outputs are world-frame 3-vectors.
 \pre ForwardKinematics(model, data) has been run for the current qpos state.
 \pre If any derivative or Jdot*qvel output is non-null, \p data has also been
  refreshed by ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
void ComputeBodyJacobianTerms(const dtDynModel &model,
                              const dtDynData &data,
                              dtDynIndex bodyId,
                              dtDynDenseMatrix *jacp,
                              dtDynDenseMatrix *jacr,
                              dtDynDenseMatrix *jacpDot,
                              dtDynDenseMatrix *jacrDot,
                              dt::Math::Vector3<dtDynReal> *linearJdotQvel,
                              dt::Math::Vector3<dtDynReal> *angularJdotQvel);
/*! \brief Compute selected site-origin Jacobian terms in one traversal.
 \details Pass nullptr for outputs that are not needed. Each non-null matrix has
  shape \f$3 \times n_v\f$; Jdot*qvel outputs are world-frame 3-vectors.
 \pre ForwardKinematics(model, data) has been run for the current qpos state.
 \pre If any derivative or Jdot*qvel output is non-null, \p data has also been
  refreshed by ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
void ComputeSiteJacobianTerms(const dtDynModel &model,
                              const dtDynData &data,
                              dtDynIndex siteId,
                              dtDynDenseMatrix *jacp,
                              dtDynDenseMatrix *jacr,
                              dtDynDenseMatrix *jacpDot,
                              dtDynDenseMatrix *jacrDot,
                              dt::Math::Vector3<dtDynReal> *linearJdotQvel,
                              dt::Math::Vector3<dtDynReal> *angularJdotQvel);
/*! \brief Compute world-frame derivatives of one body-origin Jacobian.
 \pre \p data has been refreshed for the current qpos/qvel state by
  ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
void ComputeBodyJacobianDot(const dtDynModel &model,
                            const dtDynData &data,
                            dtDynIndex bodyId,
                            dtDynDenseMatrix &jacpDot,
                            dtDynDenseMatrix &jacrDot);
/*! \brief Compute world-frame derivatives of one site-origin Jacobian.
 \pre \p data has been refreshed for the current qpos/qvel state by
  ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
void ComputeSiteJacobianDot(const dtDynModel &model,
                            const dtDynData &data,
                            dtDynIndex siteId,
                            dtDynDenseMatrix &jacpDot,
                            dtDynDenseMatrix &jacrDot);
/*! \brief Compute world-frame body-origin Jdot*qvel linear/angular terms.
 \pre \p data has been refreshed for the current qpos/qvel state by
  ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
void ComputeBodyJacobianDotTimesVelocity(const dtDynModel &model,
                                         const dtDynData &data,
                                         dtDynIndex bodyId,
                                         dt::Math::Vector3<dtDynReal> &linear,
                                         dt::Math::Vector3<dtDynReal> &angular);
/*! \brief Compute world-frame site-origin Jdot*qvel linear/angular terms.
 \pre \p data has been refreshed for the current qpos/qvel state by
  ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
void ComputeSiteJacobianDotTimesVelocity(const dtDynModel &model,
                                         const dtDynData &data,
                                         dtDynIndex siteId,
                                         dt::Math::Vector3<dtDynReal> &linear,
                                         dt::Math::Vector3<dtDynReal> &angular);

/*! \name Full-model Jacobian output paths
 \brief Fill dtDynData Jacobian arrays for every body and/or site.
 \details These functions fill both J and Jdot and are useful for diagnostics
  and full-model tooling. Realtime controllers should prefer indexed overloads.
 \pre \p data has been refreshed for the current qpos/qvel state by
  ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
///@{
void ComputeBodyJacobians(const dtDynModel &model, dtDynData &data);
void ComputeSiteJacobians(const dtDynModel &model, dtDynData &data);
void ComputeJacobians(const dtDynModel &model, dtDynData &data);
///@}

/*! \name Indexed Jacobian output paths
 \brief Fill dtDynData Jacobian arrays only for controller-selected task bodies and sites.
 \details These overloads fill both J and Jdot only for the supplied IDs.
  Unselected body/site entries retain their prior values.
 \pre \p data has been refreshed for the current qpos/qvel state by
  ComputeDynamicsRNEA() or ComputeDynamicsCCRBA().
*/
///@{
void ComputeBodyJacobians(const dtDynModel &model,
                          dtDynData &data,
                          const dtDynArray<dtDynIndex> &bodyIds);
void ComputeSiteJacobians(const dtDynModel &model,
                          dtDynData &data,
                          const dtDynArray<dtDynIndex> &siteIds);
void ComputeJacobians(const dtDynModel &model,
                      dtDynData &data,
                      const dtDynArray<dtDynIndex> &bodyIds,
                      const dtDynArray<dtDynIndex> &siteIds);
///@}

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNKINEMATICS_H_
