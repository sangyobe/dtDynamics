/*!
 \file      config.h
 \brief     Standalone dtDynamics package configuration.
*/

#ifndef DTDYNAMICS_CONFIG_H_
#define DTDYNAMICS_CONFIG_H_

#include <cstdint>

#ifndef DTDYNAMICS_USE_FLOAT
using SYSREAL = double; //!< Default scalar type when DTDYNAMICS_USE_FLOAT is not defined.
#else
using SYSREAL = float; //!< Scalar type selected by DTDYNAMICS_USE_FLOAT.
#endif

#ifdef USE_UINT16_T
using SYSINDEX = uint16_t; //!< Index type selected by USE_UINT16_T.
#else
using SYSINDEX = int;  //!< Default index type when USE_UINT16_T is not defined.
#endif

#ifndef SYSMINVAL
#define SYSMINVAL 1e-15 //!< Generic numerical tolerance; no physical unit is implied.
#endif

#endif // DTDYNAMICS_CONFIG_H_
