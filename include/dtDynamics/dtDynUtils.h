/*!
 \file      dtDynUtils.h
 \brief     Reusable native dtDynamics model and data utility declarations.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.0
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNUTILS_H_
#define ALGORITHM_DTDYNAMICS_DTDYNUTILS_H_

#include "dtDynamics/dtDynData.h"

namespace dt::Dynamics
{
namespace utils
{

/*!
 \brief Return the number of root output blocks, with a minimum of one.
 \details Tree models without explicit dynamic roots reserve one sizing block in
          dtDynData. Dynamics producers iterate actual model roots, so this fallback block
          is not an implied populated aggregate result. No model validation is performed.
*/
dtDynIndex EffectiveRootCount(const dtDynModel &model);

/*! \brief Return the root index that owns bodyId.
    \details bodyId must name a non-world body. bodyId <= 0 returns DTDYN_INVALID_INDEX;
             other invalid positive indices are outside this helper's contract. */
dtDynIndex RootIndex(const dtDynModel &model, dtDynIndex bodyId);

/*! \brief Return the first linear centroidal row: 6 * rootIdx. */
dtDynIndex RootCentroidalLinearRow(dtDynIndex rootIdx);
/*! \brief Return the first angular centroidal row: 6 * rootIdx + 3. */
dtDynIndex RootCentroidalAngularRow(dtDynIndex rootIdx);
/*! \brief Return the first center-of-mass row: 3 * rootIdx. */
dtDynIndex RootComRow(dtDynIndex rootIdx);

/*! \brief Return the world-frame subtree center of mass [m] for bodyId's root.
    \details bodyId must be a valid body index. If the body has no positive root body,
             the aggregate/world subtree CoM entry is returned. */
dt::Math::Vector3<dtDynReal> RootCom(const dtDynModel &model,
                                     const dtDynData &data,
                                     dtDynIndex bodyId);

/*!
 \brief Return one qvel motion subspace column expressed at a requested world point.
 \details The returned spatial vector uses dtDynamics math row order \f$[\omega; v]\f$.
          Hinge and ball columns include the linear velocity induced at point by angular
          motion about their world anchor. Slide and translational free-joint columns are
          pure linear motion. dofId and point are assumed to correspond to a valid, current
          dtDynData kinematic state.
*/
dt::Math::Vector6<dtDynReal> MotionSubspaceAtPoint(const dtDynModel &model,
                                                   const dtDynData &data,
                                                   dtDynIndex dofId,
                                                   const dt::Math::Vector3<dtDynReal> &point);

} // namespace utils
} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNUTILS_H_
