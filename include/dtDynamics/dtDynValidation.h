/*!
 \file      dtDynValidation.h
 \brief     Setup-time validation for editable specifications and compiled models.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 27
 \version   0.1.2
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNVALIDATION_H_
#define ALGORITHM_DTDYNAMICS_DTDYNVALIDATION_H_

#include "dtDynamics/dtDynModel.h"
#include "dtDynamics/dtDynSpec.h"

#include <string>

namespace dt::Dynamics
{

enum class dtDynValidationCode : dtDynIndex
{
    None = 0,
    InvalidWorld,
    InvalidParent,
    InvalidOwner,
    InvalidReference,
    DetachedObject,
    TopologyCycle,
    DuplicateName,
    UnsupportedJoint,
    DimensionOverflow,
    NonFiniteValue
};

struct dtDynValidationError
{
    dtDynValidationCode code = dtDynValidationCode::None;
    dtDynObjectType object = dtDynObjectType::Body;
    dtDynIndex index = DTDYN_INVALID_INDEX;
    std::string message;
};

/*! \brief Validate an editable specification before checked compilation. */
bool ValidateSpec(const dtDynSpec &spec, dtDynValidationError *error = nullptr);
/*! \brief Validate setup-time structure and dimensions of a compiled model. */
bool ValidateModel(const dtDynModel &model, dtDynValidationError *error = nullptr);

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNVALIDATION_H_
