/*!
 \file      dtDynCompile.h
 \brief     Native dtDynamics spec-to-model compiler declaration.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.1
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNCOMPILE_H_
#define ALGORITHM_DTDYNAMICS_DTDYNCOMPILE_H_

#include "dtDynamics/dtDynModel.h"
#include "dtDynamics/dtDynSpec.h"

namespace dt::Dynamics
{

/*!
 \brief Compile editable dtDynSpec into fixed-index dtDynModel arrays.
 \details Assigns body, joint, site, root, qpos, and qvel addresses; expands
  inertial constants and fixed-body metadata; and rebuilds name maps. Compilation
  is setup-time work and should not be called from hard realtime control loops.
 \param spec Editable model specification to compile.
 \return A model ready to pair with dtDynData::Resize().
*/
dtDynModel CompileSpec(const dtDynSpec &spec);

/*!
 \brief Validate and compile an editable specification.
 \details This setup-time convenience rejects invalid references and topology before
  delegating to CompileSpec(). Trusted callers may continue to use CompileSpec().
 \throws std::runtime_error with object/index context when validation fails.
*/
dtDynModel CompileSpecChecked(const dtDynSpec &spec);

/*!
 \brief Compile editable dtDynSpec into an existing fixed-index dtDynModel.
 \details Replaces the contents of \p model with arrays compiled from \p spec.
  Existing dtDynData objects must be resized again before reuse with the updated
  model.
 \param spec Editable model specification to compile.
 \param model Destination model object.
*/
void CompileSpec(const dtDynSpec &spec, dtDynModel &model);

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNCOMPILE_H_
