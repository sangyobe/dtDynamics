/*!
 \file      dtDynSpec.h
 \brief     Editable native dtDynamics model specification.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.0
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNSPEC_H_
#define ALGORITHM_DTDYNAMICS_DTDYNSPEC_H_

#include "dtDynamics/dtDynTypes.h"

#include <string>

namespace dt::Dynamics
{

/*!
 \brief Editable diagonal inertial properties for one body.
 \details The position and quaternion describe the inertial frame relative to the owning
          body frame. inertia stores the diagonal rotational inertia in that inertial frame.
          A zero mass leaves the body inertial-less during XML encoding.
*/
struct dtDynInertialSpec
{
    dtDynReal mass = 0.0;                  //!< Body mass [kg]; zero means no inertial element is emitted.
    dt::Math::Vector3<dtDynReal> pos;      //!< Inertial-frame origin in the body frame [m].
    dt::Math::Vector4<dtDynReal> quat;     //!< Inertial-frame orientation as \f$[w,x,y,z]\f$.
    dt::Math::Vector3<dtDynReal> inertia;  //!< Diagonal rotational inertia [kg m^2] in the inertial frame.
};

/*!
 \brief Editable joint description using dtDynamics/MJCF-compatible qpos/qvel sizes.
 \details A joint belongs to exactly one body in the tree model. Hinge and slide joints use
          axis; free and ball joints use quaternion qpos coordinates and angular qvel
          coordinates following dtDynVelocityConvention. rangeMin/rangeMax are radians for
          hinge/ball angular coordinates and linear units for slide coordinates.
*/
struct dtDynJointSpec
{
    std::string name;                         //!< Stable user-facing joint name used by XML and actuators.
    dtDynJointType type = dtDynJointType::Hinge; //!< Joint coordinate type.
    dtDynIndex body = DTDYN_INVALID_INDEX;    //!< Owning body index, or DTDYN_INVALID_INDEX while detached.
    dt::Math::Vector3<dtDynReal> pos;         //!< Joint anchor position in the owning body frame [m].
    dt::Math::Vector3<dtDynReal> axis;        //!< Unit hinge/slide axis in the owning body frame.
    bool limited = false;                     //!< Whether rangeMin/rangeMax should be interpreted as limits.
    dtDynReal rangeMin = 0.0;                 //!< Lower limit [rad] for hinge/ball or [m] for slide.
    dtDynReal rangeMax = 0.0;                 //!< Upper limit [rad] for hinge/ball or [m] for slide.
    dtDynReal armature = 0.0;                 //!< Added inertia [kg m^2] angular or [kg] linear.
};

/*!
 \brief Editable body-tree node.
 \details dtDynamics public model construction is tree-only: parent, firstChild, and
          nextSibling encode a rooted body hierarchy with body 0 reserved for world by
          dtDynSpec::Clear(). A body with no joints is fixed to its parent. joints and
          sites store indices into dtDynSpec::Joints() and dtDynSpec::Sites().
*/
struct dtDynBodySpec
{
    std::string name;                         //!< Stable body name used by XML and name lookup.
    dtDynIndex parent = DTDYN_INVALID_INDEX;  //!< Parent body index, or DTDYN_INVALID_INDEX for world/root.
    dtDynIndex firstChild = DTDYN_INVALID_INDEX; //!< First child body index in sibling-list order.
    dtDynIndex nextSibling = DTDYN_INVALID_INDEX; //!< Next sibling body index, or DTDYN_INVALID_INDEX.
    dtArray<dtDynIndex> joints;               //!< Joint indices attached directly to this body.
    dtArray<dtDynIndex> sites;                //!< Site indices attached directly to this body.
    dt::Math::Vector3<dtDynReal> pos;         //!< Body frame position relative to parent frame [m].
    dt::Math::Vector4<dtDynReal> quat;        //!< Body frame orientation relative to parent as \f$[w,x,y,z]\f$.
    dtDynInertialSpec inertial;               //!< Optional inertial properties for this body.
};

/*!
 \brief Editable named frame attached to a body.
 \details Sites are kinematic output frames. They do not add mass, constraints, or dynamics.
*/
struct dtDynSiteSpec
{
    std::string name;                         //!< Stable site name used by XML and name lookup.
    dtDynIndex body = DTDYN_INVALID_INDEX;    //!< Owning body index, or DTDYN_INVALID_INDEX while detached.
    dt::Math::Vector3<dtDynReal> pos;         //!< Site frame position in the owning body frame [m].
    dt::Math::Vector4<dtDynReal> quat;        //!< Site frame orientation as \f$[w,x,y,z]\f$.
};

/*!
 \brief Editable actuator metadata mapped to one joint.
 \details The native specification stores motor-style joint actuation metadata only. It does
          not model tendons, equality constraints, or closed-chain actuation.
*/
struct dtDynActuatorSpec
{
    std::string name;                         //!< Stable actuator name used by XML and name lookup.
    dtDynIndex joint = DTDYN_INVALID_INDEX;   //!< Actuated joint index, or DTDYN_INVALID_INDEX while detached.
    dtDynReal gear = 1.0;                     //!< Scalar actuator gear written to/read from XML.
};

/*!
 \brief Editable tree model specification with MJCF-like XML encode/decode support.
 \details dtDynSpec is an offline/model-construction container. It owns editable arrays of
          bodies, joints, sites, and actuators before compilation into dtDynModel/dtDynData.
          The public XML codec supports the subset required by native dtDynamics tree models:
          bodies, inertials, joints/freejoints, sites, motor actuators, and compiler angle
          units. Equality constraints and tendons are rejected by DecodeXml because the public
          dynamics API currently targets open tree models, not closed-chain models.

          Accessors expose the underlying arrays intentionally for advanced editing. Callers
          that mutate indices directly are responsible for preserving referential integrity
          before compiling or encoding the model.
*/
class dtDynSpec
{
public:
    /*! \brief Construct a specification containing only the world body. */
    dtDynSpec();

    /*! \brief Add a body under parent and return its index.
        \details If parent is not a valid existing body index, the new body remains detached
                 from the sibling tree but is still appended to Bodies(). */
    dtDynIndex AddBody(const std::string &name, dtDynIndex parent);
    /*! \brief Add a joint to body and return its index.
        \details Invalid body indices append a detached joint without adding it to any body's
                 joint list. The default axis is \f$[0,0,1]\f$. */
    dtDynIndex AddJoint(dtDynIndex body, const std::string &name, dtDynJointType type);
    /*! \brief Add a site to body and return its index.
        \details Invalid body indices append a detached site without adding it to any body's
                 site list. The default orientation is identity. */
    dtDynIndex AddSite(dtDynIndex body, const std::string &name);
    /*! \brief Add a motor actuator for joint and return its index.
        \details The joint index is stored as provided; XML encoding emits a joint reference
                 only when the index resolves to an existing joint. */
    dtDynIndex AddActuator(dtDynIndex joint, const std::string &name);

    /*! \brief Reset to a single world body and remove all joints, sites, and actuators. */
    void Clear();
    /*! \brief Encode the supported tree-model subset as MJCF-like XML using radians. */
    std::string EncodeXml() const;
    /*! \brief Decode a supported MJCF-like XML string into an editable specification.
        \throws std::runtime_error for malformed tags/comments, unsupported angle units,
                closed-chain/equality/tendon elements, or actuators referencing unknown joints. */
    static dtDynSpec DecodeXml(const std::string &xml);

    /*! \brief Return immutable body storage. */
    const dtArray<dtDynBodySpec> &Bodies() const;
    /*! \brief Return immutable joint storage. */
    const dtArray<dtDynJointSpec> &Joints() const;
    /*! \brief Return immutable site storage. */
    const dtArray<dtDynSiteSpec> &Sites() const;
    /*! \brief Return immutable actuator storage. */
    const dtArray<dtDynActuatorSpec> &Actuators() const;

    /*! \brief Return mutable body storage for advanced specification editing. */
    dtArray<dtDynBodySpec> &Bodies();
    /*! \brief Return mutable joint storage for advanced specification editing. */
    dtArray<dtDynJointSpec> &Joints();
    /*! \brief Return mutable site storage for advanced specification editing. */
    dtArray<dtDynSiteSpec> &Sites();
    /*! \brief Return mutable actuator storage for advanced specification editing. */
    dtArray<dtDynActuatorSpec> &Actuators();

private:
    dtArray<dtDynBodySpec> m_bodies;
    dtArray<dtDynJointSpec> m_joints;
    dtArray<dtDynSiteSpec> m_sites;
    dtArray<dtDynActuatorSpec> m_actuators;
};

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNSPEC_H_
