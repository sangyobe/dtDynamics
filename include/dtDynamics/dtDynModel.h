/*!
 \file      dtDynModel.h
 \brief     Compiled native dtDynamics model.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.0
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNMODEL_H_
#define ALGORITHM_DTDYNAMICS_DTDYNMODEL_H_

#include "dtDynamics/dtDynTypes.h"

#include <string>
#include <unordered_map>

namespace dt::Dynamics
{

/*!
 \brief Compiled, deterministic model arrays consumed by dtDynData computations.
 \details
  dtDynModel is the compiled product of CompileSpec(). It owns topology,
  coordinate addresses, inertial constants, fixed-body metadata, site metadata,
  root metadata, and name lookup maps. Runtime algorithms treat these arrays as
  read-only, while advanced callers may edit public arrays during setup and then
  call RebuildNameMaps() when names change.

  The model follows MuJoCo-style address separation for the supported coordinate
  convention: configuration coordinates live in qpos addresses while tangent
  velocities live in qvel/dof addresses. Floating-base models therefore may have
  \f$n_q \neq n_v\f$.

  Body id 0 is reserved for the world body. Invalid lookup results are reported
  as DTDYN_INVALID_INDEX rather than throwing.
 \note Keep application-specific compatibility adapters outside this standalone
  model layer so compiled model semantics remain reusable.
*/
class dtDynModel
{
public:
    /*! \brief Construct an empty, uncompiled model with all counts set to zero. */
    dtDynModel() = default;
    /*! \brief Deep-copy all compiled arrays and body, joint, and site name maps. */
    dtDynModel(const dtDynModel &other);
    /*! \brief Replace this model with an independently owned deep copy of \p other. */
    dtDynModel &operator=(const dtDynModel &other);

    dtDynIndex nq = 0;    //!< Number of generalized-position coordinates.
    dtDynIndex nv = 0;    //!< Number of generalized velocities / scalar DOFs.
    dtDynIndex nbody = 0; //!< Number of bodies including world body 0.
    dtDynIndex njnt = 0;  //!< Number of compiled joints.
    dtDynIndex nsite = 0; //!< Number of sites.
    dtDynIndex nu = 0;    //!< Number of actuators.
    dtDynIndex nroot = 0; //!< Number of non-world kinematic roots.

    dtArray<std::string> body_names;     //!< Body names (nbody).
    dtArray<std::string> joint_names;    //!< Joint names (njnt).
    dtArray<std::string> site_names;     //!< Site names (nsite).
    dtArray<std::string> actuator_names; //!< Actuator names (nu).

    dtArray<dtDynIndex> body_parentid; //!< Parent body ids; world parent is 0 (nbody).
    dtArray<dtDynIndex> body_rootid;   //!< Top-level non-world root body id; world body 0 stores 0 (nbody).
    dtArray<dtDynIndex> body_rootidx;  //!< Compact root slot, or DTDYN_INVALID_INDEX when absent (nbody).
    dtArray<dtDynIndex> body_jntnum;   //!< Number of joints attached directly to each body (nbody).
    dtArray<dtDynIndex> body_jntadr;   //!< First attached joint id, or DTDYN_INVALID_INDEX (nbody).
    dtArray<dtDynIndex> body_dofnum;   //!< Number of scalar qvel DOFs contributed by each body (nbody).
    dtArray<dtDynIndex> body_dofadr;   //!< First scalar DOF address, or DTDYN_INVALID_INDEX (nbody).
    dtArray<dtDynIndex> body_weldid;   //!< Nearest jointed ancestor including self, or world body 0 when absent (nbody).

    dtArray<dtDynIndex> jnt_type;    //!< dtDynJointType values (njnt).
    dtArray<dtDynIndex> jnt_bodyid;  //!< Owning body id for each joint (njnt).
    dtArray<dtDynIndex> jnt_qposadr; //!< First qpos address for each joint (njnt).
    dtArray<dtDynIndex> jnt_dofadr;  //!< First qvel/DOF address for each joint (njnt).
    dtArray<dtDynIndex> jnt_limited; //!< Binary joint-limit flags (njnt).
    dtDynDenseVector jnt_range_min;  //!< Lower joint limits in joint units (njnt).
    dtDynDenseVector jnt_range_max;  //!< Upper joint limits in joint units (njnt).

    dtArray<dtDynIndex> dof_bodyid;   //!< Body id moved by each scalar DOF (nv).
    dtArray<dtDynIndex> dof_parentid; //!< Nearest ancestor scalar DOF, or DTDYN_INVALID_INDEX (nv).
    dtArray<dtDynIndex> dof_jntid;    //!< Joint id owning each scalar DOF (nv).
    dtDynDenseVector dof_armature;    //!< Armature inertia per scalar DOF (nv).

    dtArray<dtDynIndex> site_bodyid;       //!< Owning body id for each site (nsite).
    dtArray<dtDynIndex> actuator_jointid;  //!< Driven joint id, or DTDYN_INVALID_INDEX (nu).
    dtArray<dtDynIndex> root_bodyid;       //!< Body id for each compact root slot (nroot).
    dtDynDenseVector actuator_gear;        //!< Scalar transmission gear per actuator (nu).
    dtDynDenseVector root_mass;            //!< Total subtree mass [kg] per compact root slot (nroot).

    dtDynDenseVector body_mass;        //!< Body mass [kg] (nbody).
    dtDynDenseVector body_subtreemass; //!< Body-plus-descendant mass [kg] (nbody).
    dtArray<dt::Math::Vector3<dtDynReal>> body_pos;     //!< Body-frame origin position in parent frame [m] (nbody x 3).
    dtArray<dt::Math::Vector4<dtDynReal>> body_quat;    //!< Body-to-parent unit quaternion in wxyz order (nbody x 4).
    dtArray<dt::Math::Matrix3<dtDynReal>> body_mat;     //!< Body-to-parent rotation matrix (nbody x 3 x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> body_ipos;    //!< Inertial origin in body frame [m]; zero-mass bodies store body_pos fallback (nbody x 3).
    dtArray<dt::Math::Vector4<dtDynReal>> body_iquat;   //!< Inertial-to-body wxyz quaternion; zero-mass bodies store body_quat fallback (nbody x 4).
    dtArray<dt::Math::Matrix3<dtDynReal>> body_imat;    //!< Inertial-to-body rotation; zero-mass bodies store body_mat fallback (nbody x 3 x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> body_inertia; //!< Principal rotational inertia [kg m^2] in inertial frame (nbody x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> jnt_pos;      //!< Joint anchor position in the compiled local frame [m] (njnt x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> jnt_axis;     //!< Joint axis in the compiled local frame (njnt x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> site_pos;     //!< Site origin position in owning-body frame [m] (nsite x 3).
    dtArray<dt::Math::Vector4<dtDynReal>> site_quat;    //!< Site-to-body unit quaternion in wxyz order (nsite x 4).
    dtArray<dt::Math::Matrix3<dtDynReal>> site_mat;     //!< Site-to-body rotation matrix (nsite x 3 x 3).

    /*! \brief Return named body id, or DTDYN_INVALID_INDEX when the name is absent. */
    dtDynIndex BodyId(const std::string &name) const;
    /*! \brief Return named joint id, or DTDYN_INVALID_INDEX when the name is absent. */
    dtDynIndex JointId(const std::string &name) const;
    /*! \brief Return named site id, or DTDYN_INVALID_INDEX when the name is absent. */
    dtDynIndex SiteId(const std::string &name) const;

    /*! \brief True when \p body is the global world body id. */
    bool IsWorldBody(dtDynIndex body) const;
    /*! \brief True when \p body has no joint and contributes no generalized velocity columns. */
    bool IsFixedBody(dtDynIndex body) const;

    /*!
     \brief Rebuild body, joint, and site name-to-id maps after direct array edits.
     \details This is setup-time work; no actuator-name lookup API is exposed.
    */
    void RebuildNameMaps();

private:
    std::unordered_map<std::string, dtDynIndex> m_bodyNameToId;
    std::unordered_map<std::string, dtDynIndex> m_jointNameToId;
    std::unordered_map<std::string, dtDynIndex> m_siteNameToId;
};

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNMODEL_H_
