/*!
 \file      dtDynSpec.h
 \brief     Editable native dtDynamics model specification.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.2
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNSPEC_H_
#define ALGORITHM_DTDYNAMICS_DTDYNSPEC_H_
#include <dtDynamics/dtArray.h>
#include <dtDynamics/dtDynTypes.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace dt::Dynamics
{

/*! \brief Stable categories for supported XML decoding failures. */
enum class dtDynXmlErrorCode : dtDynIndex
{
    MalformedSyntax = 0,
    InvalidNumber = 1,
    InvalidVectorArity = 2,
    NonFiniteValue = 3,
    UnsupportedElement = 4,
    InvalidReference = 5,
    DuplicateName = 6
};

/*! \brief Path- and offset-aware XML decoding error. */
class dtDynXmlError : public std::runtime_error
{
public:
    /*! \brief Construct a typed XML error with source and byte-offset context. */
    dtDynXmlError(dtDynXmlErrorCode code,
                  std::string sourcePath,
                  std::size_t offset,
                  const std::string &message);

    /*! \brief Return the stable failure category. */
    dtDynXmlErrorCode Code() const noexcept;
    /*! \brief Return the source file path, or an empty string for DecodeXml(). */
    const std::string &SourcePath() const noexcept;
    /*! \brief Return the zero-based byte offset in the decoded XML text. */
    std::size_t Offset() const noexcept;

private:
    dtDynXmlErrorCode m_code;
    std::string m_sourcePath;
    std::size_t m_offset;
};

/*!
 \brief Editable diagonal inertial properties for one body.
 \details The position and quaternion describe the inertial frame relative to the owning
          body frame. inertia stores the diagonal rotational inertia in that inertial frame.
          A zero mass leaves the body inertial-less during XML encoding.
*/
struct dtDynInertialSpec
{
    dtDynReal mass = 0.0;                 //!< Body mass [kg]; zero means no inertial element is emitted.
    dt::Math::Vector3<dtDynReal> pos;     //!< Inertial-frame origin in the body frame [m].
    dt::Math::Vector4<dtDynReal> quat;    //!< Inertial-frame orientation as \f$[w,x,y,z]\f$.
    dt::Math::Vector3<dtDynReal> inertia; //!< Diagonal rotational inertia [kg m^2] in the inertial frame.
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
    std::string name;                            //!< Stable user-facing joint name used by XML and actuators.
    dtDynJointType type = dtDynJointType::Hinge; //!< Joint coordinate type.
    dtDynIndex body = DTDYN_INVALID_INDEX;       //!< Owning body index, or DTDYN_INVALID_INDEX while detached.
    dt::Math::Vector3<dtDynReal> pos;            //!< Joint anchor position in the owning body frame [m].
    dt::Math::Vector3<dtDynReal> axis;           //!< Unit hinge/slide axis in the owning body frame.
    bool limited = false;                        //!< Whether rangeMin/rangeMax should be interpreted as limits.
    dtDynReal rangeMin = 0.0;                    //!< Lower limit [rad] for hinge/ball or [m] for slide.
    dtDynReal rangeMax = 0.0;                    //!< Upper limit [rad] for hinge/ball or [m] for slide.
    dtDynReal armature = 0.0;                    //!< Added inertia [kg m^2] angular or [kg] linear.
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
    std::string name;                             //!< Stable body name used by XML and name lookup.
    dtDynIndex parent = DTDYN_INVALID_INDEX;      //!< Parent body index, or DTDYN_INVALID_INDEX for world/root.
    dtDynIndex firstChild = DTDYN_INVALID_INDEX;  //!< First child body index in sibling-list order.
    dtDynIndex nextSibling = DTDYN_INVALID_INDEX; //!< Next sibling body index, or DTDYN_INVALID_INDEX.
    dtArray<dtDynIndex> joints;                   //!< Joint indices attached directly to this body.
    dtArray<dtDynIndex> sites;                    //!< Site indices attached directly to this body.
    dt::Math::Vector3<dtDynReal> pos;             //!< Body frame position relative to parent frame [m].
    dt::Math::Vector4<dtDynReal> quat;            //!< Body frame orientation relative to parent as \f$[w,x,y,z]\f$.
    dtDynInertialSpec inertial;                   //!< Optional inertial properties for this body.
};

/*!
 \brief Editable named frame attached to a body.
 \details Sites are kinematic output frames. They do not add mass, constraints, or dynamics.
*/
struct dtDynSiteSpec
{
    std::string name;                      //!< Stable site name used by XML and name lookup.
    dtDynIndex body = DTDYN_INVALID_INDEX; //!< Owning body index, or DTDYN_INVALID_INDEX while detached.
    dt::Math::Vector3<dtDynReal> pos;      //!< Site frame position in the owning body frame [m].
    dt::Math::Vector4<dtDynReal> quat;     //!< Site frame orientation as \f$[w,x,y,z]\f$.
};

/*!
 \brief Editable actuator metadata mapped to one joint.
 \details The native specification stores motor-style joint actuation metadata only. It does
          not model tendons, equality constraints, or closed-chain actuation.
*/
struct dtDynActuatorSpec
{
    std::string name;                       //!< Stable actuator name used by XML and name lookup.
    dtDynIndex joint = DTDYN_INVALID_INDEX; //!< Actuated joint index, or DTDYN_INVALID_INDEX while detached.
    dtDynReal gear = 1.0;                   //!< Scalar actuator gear written to/read from XML.
};

/*!
 \brief Editable tree model specification with MJCF-like XML encode/decode support.
 \details dtDynSpec is an offline/model-construction container. It owns editable arrays of
          bodies, joints, sites, and actuators before compilation into dtDynModel/dtDynData.
          The public XML codec supports the subset required by native dtDynamics tree models:
          bodies, inertials, joints/freejoints, sites, motor actuators, and compiler angle
          units. Equality constraints and tendons are rejected by DecodeXml because the public
          dynamics API currently targets open tree models, not closed-chain models.

          Collection accessors are immutable. Setup-time property changes use callback-scoped
          EditBody/EditJoint/EditSite/EditActuator operations, which commit an item copy without
          exposing the owning container. CompileSpecChecked validates referential integrity before
          producing a model from externally assembled specifications.
*/
class dtDynSpec
{
public:
    /*! \brief Construct a specification containing only the world body. */
    dtDynSpec();
    /*! \brief Copy only specification storage; active edit state is never copied. */
    dtDynSpec(const dtDynSpec &other) = default;
    /*! \brief Move specification storage when the source is not being edited. */
    dtDynSpec(dtDynSpec &&other) = default;
    /*! \brief Assign inline storage when both specifications are outside scoped edits.
        \details The first-member edit transaction hook rejects an active source or destination
                 before any dtArray assignment starts, and poisons every active transaction.
        \note This minimal defaulted memberwise assignment does not provide a strong whole-object
              copy guarantee if a later dtArray assignment throws after an earlier one succeeds. */
    dtDynSpec &operator=(const dtDynSpec &other) = default;
    dtDynSpec &operator=(dtDynSpec &&other) = default;

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
    /*! \brief Encode and save the supported tree-model subset to an XML file.
        \param path Destination XML file path.
        \throws std::runtime_error if the file cannot be opened or written. */
    void SaveXmlFile(const std::string &path) const;
    /*! \brief Decode a supported MJCF-like XML string into an editable specification.
        \throws dtDynXmlError with an empty source path and stable code/offset for malformed
                syntax, invalid or non-finite numbers, wrong vector arity, duplicate names,
                unsupported elements/values, or invalid references. */
    static dtDynSpec DecodeXml(const std::string &xml);
    /*! \brief Load and decode a supported MJCF-like XML file into an editable specification.
        \param path XML file path.
        \throws std::runtime_error if the file cannot be opened or read.
        \throws dtDynXmlError with \p path and a byte offset when decoding fails. */
    static dtDynSpec LoadXmlFile(const std::string &path);

    /*! \brief Return body id by name, or DTDYN_INVALID_INDEX when absent.
        \details Performs a setup-time linear scan so committed scoped edits are visible. */
    dtDynIndex BodyId(const std::string &name) const;
    /*! \brief Return joint id by name, or DTDYN_INVALID_INDEX when absent.
        \details Performs a setup-time linear scan so committed scoped edits are visible. */
    dtDynIndex JointId(const std::string &name) const;
    /*! \brief Return site id by name, or DTDYN_INVALID_INDEX when absent.
        \details Performs a setup-time linear scan so committed scoped edits are visible. */
    dtDynIndex SiteId(const std::string &name) const;
    /*! \brief Return actuator id by name, or DTDYN_INVALID_INDEX when absent.
        \details Performs a setup-time linear scan so committed scoped edits are visible. */
    dtDynIndex ActuatorId(const std::string &name) const;

    /*! \brief Return immutable body storage. */
    const dtArray<dtDynBodySpec> &Bodies() const;
    /*! \brief Return immutable joint storage. */
    const dtArray<dtDynJointSpec> &Joints() const;
    /*! \brief Return immutable site storage. */
    const dtArray<dtDynSiteSpec> &Sites() const;
    /*! \brief Return immutable actuator storage. */
    const dtArray<dtDynActuatorSpec> &Actuators() const;

    /*!
     \brief Edit one body through a callback-scoped copy and commit it on return.
     \details The callback receives a temporary copy, not a reference into dtDynSpec storage.
              This setup-time API preserves editable model construction without exposing the
              owning container. Throwing from the callback leaves the stored body unchanged.
              Calling Edit*, Add*, Clear, assignment, or swap on this dtDynSpec from the callback
              throws std::logic_error; even if caught, the edit aborts and discards its draft.
    */
    template <typename Editor>
    void EditBody(const dtDynIndex bodyId, Editor &&editor)
    {
        EditItem(m_bodies, bodyId, std::forward<Editor>(editor), "body");
    }
    /*! \brief Edit one joint through a callback-scoped copy and commit it on return. */
    template <typename Editor>
    void EditJoint(const dtDynIndex jointId, Editor &&editor)
    {
        EditItem(m_joints, jointId, std::forward<Editor>(editor), "joint");
    }
    /*! \brief Edit one site through a callback-scoped copy and commit it on return. */
    template <typename Editor>
    void EditSite(const dtDynIndex siteId, Editor &&editor)
    {
        EditItem(m_sites, siteId, std::forward<Editor>(editor), "site");
    }
    /*! \brief Edit one actuator through a callback-scoped copy and commit it on return. */
    template <typename Editor>
    void EditActuator(const dtDynIndex actuatorId, Editor &&editor)
    {
        EditItem(m_actuators, actuatorId, std::forward<Editor>(editor), "actuator");
    }

private:
    friend void swap(dtDynSpec &left, dtDynSpec &right);

    class EditTransaction
    {
    public:
        EditTransaction() = default;
        EditTransaction(const EditTransaction &other) noexcept;
        EditTransaction(EditTransaction &&other);
        EditTransaction &operator=(const EditTransaction &other);
        EditTransaction &operator=(EditTransaction &&other);

        void Begin() noexcept;
        void Cancel() noexcept;
        bool Finish() noexcept;
        void EnsureInactive();
        void RejectSwap(EditTransaction &other);

    private:
        bool IsActive() const noexcept;
        void Abort() const noexcept;
        void RejectAssignment(const EditTransaction &other, const char *message);

        std::size_t m_depth = 0;
        mutable bool m_aborted = false;
    };

    void EnsureNotEditing();
    void SwapStorage(dtDynSpec &other);

    template <typename Item, typename Editor>
    void EditItem(dtArray<Item> &items,
                  const dtDynIndex itemId,
                  Editor &&editor,
                  const char *const itemType)
    {
        EnsureNotEditing();
        if (itemId < 0 || static_cast<std::size_t>(itemId) >= items.size())
        {
            throw std::out_of_range(std::string("dtDynSpec: invalid ") + itemType + " index");
        }

        Item draft = items[static_cast<std::size_t>(itemId)];
        m_editTransaction.Begin();
        try
        {
            std::forward<Editor>(editor)(draft);
        }
        catch (...)
        {
            m_editTransaction.Cancel();
            throw;
        }
        const bool aborted = m_editTransaction.Finish();
        if (aborted)
        {
            throw std::logic_error("dtDynSpec: reentrant mutation aborted Edit* callback");
        }
        items[static_cast<std::size_t>(itemId)] = std::move(draft);
    }

    // This must remain first so defaulted outer assignment checks it before inline storage.
    EditTransaction m_editTransaction;
    dtArray<dtDynBodySpec> m_bodies;
    dtArray<dtDynJointSpec> m_joints;
    dtArray<dtDynSiteSpec> m_sites;
    dtArray<dtDynActuatorSpec> m_actuators;
};

/*! \brief Swap specification storage when neither instance is inside an edit callback. */
void swap(dtDynSpec &left, dtDynSpec &right);

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNSPEC_H_
