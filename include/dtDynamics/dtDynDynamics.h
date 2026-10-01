/*!
 \file      dtDynDynamics.h
 \brief     Native dtDynamics dynamics ingredient declarations.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNDYNAMICS_H_
#define ALGORITHM_DTDYNAMICS_DTDYNDYNAMICS_H_

#include "dtDynamics/dtDynData.h"

#include <cstdint>

namespace dt::Dynamics
{

using dtDynRneaTermFlags = std::uint32_t;  //!< Bitmask type for optional RNEA outputs.
using dtDynCcrbaTermFlags = std::uint32_t; //!< Bitmask type for optional CCRBA outputs.

static constexpr dtDynRneaTermFlags DTDYN_RNEA_TERM_NONE = 0u;         //!< Compute Mmat, gravityTorq, coriolisTorq, and nonlinTorq; clear Cmat.
static constexpr dtDynRneaTermFlags DTDYN_RNEA_TERM_COM_POS = 1u << 0; //!< Also fill stacked world-frame root CoM positions [m] (3*nr).
static constexpr dtDynRneaTermFlags DTDYN_RNEA_TERM_ALL =              //!< Compute all RNEA optional outputs.
    DTDYN_RNEA_TERM_COM_POS;

static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_NONE = 0u;              //!< Compute base CCRBA outputs: Mmat, Cmat, gravityTorq, coriolisTorq, and nonlinTorq.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_COM_POS = 1u << 0;      //!< Also compute root CoM position.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_CMAMAT = 1u << 1;       //!< Also compute centroidal momentum matrix.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_CMAMAT_DOT = 1u << 2;   //!< Also compute CMM time derivative.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_COM_JMAT = 1u << 3;     //!< Also compute CoM Jacobian from CCRBA forces.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_COM_JMAT_DOT = 1u << 4; //!< Also compute CoM Jacobian time derivative.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_CENT_MOM = 1u << 5;     //!< Also compute centroidal momentum.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_COM_VEL = 1u << 6;      //!< Also compute CoM velocity.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_MMAT_DOT = 1u << 7;     //!< Also compute mass matrix time derivative.
static constexpr dtDynCcrbaTermFlags DTDYN_CCRBA_TERM_ALL =                   //!< Compute all CCRBA optional outputs.
    DTDYN_CCRBA_TERM_COM_POS |
    DTDYN_CCRBA_TERM_CMAMAT |
    DTDYN_CCRBA_TERM_CMAMAT_DOT |
    DTDYN_CCRBA_TERM_COM_JMAT |
    DTDYN_CCRBA_TERM_COM_JMAT_DOT |
    DTDYN_CCRBA_TERM_CENT_MOM |
    DTDYN_CCRBA_TERM_COM_VEL |
    DTDYN_CCRBA_TERM_MMAT_DOT;

/*!
 \brief Compute base dynamics ingredients with recursive Newton-Euler bias forces.
 \details
  The RNEA path fills controller-facing bias terms for the manipulator equation
  \f$M(q)\dot{v} + h(q,v) = \tau + J(q)^T f\f$ while avoiding the heavier
  composite/centroidal quantities requested by CCRBA flags. It is appropriate
  when a controller needs gravity, Coriolis/bias, and nonlinear terms but does
  not need centroidal momentum, CoM Jacobian, or mass-matrix derivative outputs.
 \pre data has been resized for \p model and contains valid qpos/qvel state.
 \param model Compiled immutable dynamics model.
 \param data Mutable runtime state and output buffers.
 \param gravity World-frame gravitational acceleration [m/s^2].
 \param flags Optional RNEA outputs to compute.
 \note This function computes dynamics ingredients, not a controller torque
  command. Torque policy remains owned by the caller/controller.
*/
void ComputeDynamicsRNEA(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &gravity,
                         dtDynRneaTermFlags flags = DTDYN_RNEA_TERM_NONE);
/*!
 \brief Compute base dynamics ingredients and requested CCRBA/WBC quantities.
 \details
  The CCRBA path computes the mass/bias basis and optional control quantities
  selected by dtDynCcrbaTermFlags, including CoM, CoM Jacobian matrices,
  centroidal momentum matrix, centroidal momentum, selected time derivatives,
  and related products. Body/site/point Jacobians are not filled by this call;
  use the Compute*Jacobian* APIs for task Jacobians. Request only the terms
  consumed by the current controller tick to keep the realtime path bounded.
 \pre data has been resized for \p model and contains valid qpos/qvel state.
 \param model Compiled immutable dynamics model.
 \param data Mutable runtime state and output buffers.
 \param gravity World-frame gravitational acceleration [m/s^2].
 \param flags Bitmask of optional CCRBA/WBC outputs to compute.
 \note MuJoCo/Pinocchio are validation references only; this production API does
  not expose them as public dependencies.
*/
void ComputeDynamicsCCRBA(const dtDynModel &model,
                          dtDynData &data,
                          const dt::Math::Vector3<dtDynReal> &gravity,
                          dtDynCcrbaTermFlags flags = DTDYN_CCRBA_TERM_NONE);

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNDYNAMICS_H_
