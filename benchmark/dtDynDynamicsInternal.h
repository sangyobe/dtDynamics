/*!
 \file      dtDynDynamicsInternal.h
 \brief     Internal native dtDynamics dynamics stages used by benchmarks.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNDYNAMICSINTERNAL_H_
#define ALGORITHM_DTDYNAMICS_DTDYNDYNAMICSINTERNAL_H_

#include "dtDynamics/dtDynDynamics.h"

namespace dt::Dynamics
{
namespace detail
{

/*!
 \brief Benchmark-visible native dynamics stages.
 \details This header is intentionally kept under benchmark/ so production
          package headers expose only the stable dtDynamics public API.
*/
void ComputeCentroidalPrerequisites(const dtDynModel &model, dtDynData &data);
void ComputeRNEAPrerequisites(const dtDynModel &model, dtDynData &data);
void ComputeSubtreeCom(const dtDynModel &model, dtDynData &data);
void ComputeDofMotionSubspaces(const dtDynModel &model, dtDynData &data);
void ComputeDiffMotion(const dtDynModel &model, dtDynData &data);
void ComputeCompositeRigidBodyInertias(const dtDynModel &model, dtDynData &data);
void ComputeLocalBiasMatrices(const dtDynModel &model, dtDynData &data);
void AccumulateCompositeBiasMatrices(const dtDynModel &model, dtDynData &data);
void ComputeDofBiasForces(const dtDynModel &model, dtDynData &data);
void ComputeBiasMatricesAndForcesFromCompositeInertias(const dtDynModel &model, dtDynData &data);
void ClearDenseDynamicsOutputs(const dtDynModel &model, dtDynData &data);
void ComputeRootComOutputs(const dtDynModel &model, dtDynData &data);
void ComputeCentroidalMomentumMatrix(const dtDynModel &model, dtDynData &data);
void ComputeCentroidalMomentumMatrixDot(const dtDynModel &model, dtDynData &data);
void ComputeMassMatrixCRBA(const dtDynModel &model, dtDynData &data);
void ComputeMassMatrixCCRBA(const dtDynModel &model, dtDynData &data);
void ComputeMassMatrixDotCCRBA(const dtDynModel &model, dtDynData &data);
void ComputeCoriolisMatrixCCRBA(const dtDynModel &model, dtDynData &data);
void ComputeComJacobianMatrixFromForces(const dtDynModel &model, dtDynData &data);
void ComputeComJacobianMatrixDotFromForces(const dtDynModel &model, dtDynData &data);
void ComputeCoriolisTorqueProduct(const dtDynModel &model, dtDynData &data);
void ComputeGravityFromCentroidalForces(const dtDynModel &model,
                                        dtDynData &data,
                                        const dt::Math::Vector3<dtDynReal> &gravity);
void ComputeRNEABiasForces(const dtDynModel &model,
                           dtDynData &data,
                           const dt::Math::Vector3<dtDynReal> &gravity);

} // namespace detail
} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNDYNAMICSINTERNAL_H_
