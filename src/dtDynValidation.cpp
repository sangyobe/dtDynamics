/*!
 \file      dtDynValidation.cpp
 \brief     Setup-time validation for editable specifications and compiled models.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 27
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynValidation.h"

#include "dtDynamics/dtDynMath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace dt::Dynamics
{

namespace
{

void ClearError(dtDynValidationError *error)
{
    if (error != nullptr)
    {
        *error = dtDynValidationError{};
    }
}

bool Fail(dtDynValidationError *error,
          const dtDynValidationCode code,
          const dtDynObjectType object,
          const dtDynIndex index,
          const std::string &message)
{
    if (error != nullptr)
    {
        error->code = code;
        error->object = object;
        error->index = index;
        error->message = message;
    }
    return false;
}

bool RequireModelArraySize(const char *name,
                           const std::size_t actual,
                           const dtDynIndex expected,
                           dtDynValidationError *error)
{
    if (actual == static_cast<std::size_t>(expected))
    {
        return true;
    }
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                DTDYN_INVALID_INDEX, std::string("model array ") + name + " has size " + std::to_string(actual) + ", expected " + std::to_string(expected));
}

bool RequireModelDenseSize(const char *name,
                           const dtDynDenseVector &vector,
                           const dtDynIndex expected,
                           dtDynValidationError *error)
{
    return RequireModelArraySize(name, static_cast<std::size_t>(vector.GetDim()), expected, error);
}

bool FailNonFiniteBody(dtDynValidationError *error, const dtDynIndex bodyId)
{
    return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Body, bodyId,
                "compiled body[" + std::to_string(bodyId) + "] contains a non-finite value");
}

bool FailInvalidBodyPhysicalOrFrame(dtDynValidationError *error, const dtDynIndex bodyId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, bodyId,
                "compiled body has invalid physical or frame-pair values");
}

bool FailInvalidJointSpecProvenance(dtDynValidationError *error, const dtDynIndex jointId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint, jointId,
                "compiled joint specification provenance is invalid");
}

bool FailNonFiniteJoint(dtDynValidationError *error, const dtDynIndex jointId)
{
    return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Joint, jointId,
                "compiled joint contains a non-finite value");
}

bool FailInvalidSlideHingeAxis(dtDynValidationError *error, const dtDynIndex jointId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint, jointId,
                "compiled slide/hinge joint axis must be unit length");
}

bool FailNonFiniteSite(dtDynValidationError *error, const dtDynIndex siteId)
{
    return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Site, siteId,
                "compiled site contains a non-finite value");
}

bool FailInvalidSiteFramePair(dtDynValidationError *error, const dtDynIndex siteId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Site, siteId,
                "compiled site quaternion/matrix frame pair is invalid");
}

bool FailInvalidBodyJointSegment(dtDynValidationError *error, const dtDynIndex bodyId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, bodyId,
                "compiled body joint segments overlap or contain another body's joint");
}

bool FailInvalidBodyDofSegment(dtDynValidationError *error, const dtDynIndex bodyId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, bodyId,
                "compiled body DOF segments overlap or contain another body's DOF");
}

bool FailInvalidJointDofSpanOwnership(dtDynValidationError *error, const dtDynIndex jointId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint, jointId,
                "compiled joint DOF span ownership is inconsistent");
}

bool FailNonFiniteSpecBody(dtDynValidationError *error, const dtDynIndex bodyId)
{
    return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Body, bodyId,
                "body[" + std::to_string(bodyId) + "] contains a non-finite value");
}

bool FailInvalidSpecBodyPhysicalInertia(dtDynValidationError *error, const dtDynIndex bodyId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, bodyId,
                "body[" + std::to_string(bodyId) +
                    "] has a negative mass or physically invalid principal inertia");
}

bool FailInvalidSpecBodyQuaternion(dtDynValidationError *error, const dtDynIndex bodyId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, bodyId,
                "body[" + std::to_string(bodyId) + "] has a non-unit body or inertial quaternion");
}

bool FailNonFiniteSpecJoint(dtDynValidationError *error, const dtDynIndex jointId)
{
    return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Joint, jointId,
                "joint[" + std::to_string(jointId) + "] contains a non-finite value");
}

bool FailInvalidSpecSlideHingeAxis(dtDynValidationError *error, const dtDynIndex jointId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint, jointId,
                "joint axis must be unit length for slide/hinge joints");
}

bool FailNegativeSpecJointArmature(dtDynValidationError *error, const dtDynIndex jointId)
{
    return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint, jointId,
                "joint armature must be nonnegative");
}

bool FailNonFiniteSpecSite(dtDynValidationError *error, const dtDynIndex siteId)
{
    return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Site, siteId,
                "site[" + std::to_string(siteId) + "] contains a non-finite value");
}

template <typename Object>
bool ValidateUniqueNames(const dtArray<Object> &objects,
                         const dtDynObjectType objectType,
                         const char *label,
                         dtDynValidationError *error)
{
    std::unordered_map<std::string, dtDynIndex> seen;
    seen.reserve(objects.size());
    for (std::size_t index = 0; index < objects.size(); ++index)
    {
        const dtDynIndex objectIndex = static_cast<dtDynIndex>(index);
        const auto inserted = seen.emplace(objects[index].name, objectIndex);
        if (!inserted.second)
        {
            return Fail(error, dtDynValidationCode::DuplicateName, objectType, objectIndex,
                        std::string(label) + "[" + std::to_string(index) +
                            "] duplicates name '" + objects[index].name + "'");
        }
    }
    return true;
}

bool ValidateUniqueNames(const dtArray<std::string> &names,
                         const dtDynObjectType objectType,
                         const char *label,
                         dtDynValidationError *error)
{
    std::unordered_map<std::string, dtDynIndex> seen;
    seen.reserve(names.size());
    for (std::size_t index = 0; index < names.size(); ++index)
    {
        const dtDynIndex objectIndex = static_cast<dtDynIndex>(index);
        if (!seen.emplace(names[index], objectIndex).second)
        {
            return Fail(error, dtDynValidationCode::DuplicateName, objectType, objectIndex,
                        std::string(label) + "[" + std::to_string(index) +
                            "] duplicates name '" + names[index] + "'");
        }
    }
    return true;
}

bool IsFinite(const dt::Math::Vector3<dtDynReal> &value)
{
    return std::isfinite(value.At(0)) && std::isfinite(value.At(1)) && std::isfinite(value.At(2));
}

bool IsFinite(const dt::Math::Vector4<dtDynReal> &value)
{
    return std::isfinite(value.At(0)) && std::isfinite(value.At(1)) &&
           std::isfinite(value.At(2)) && std::isfinite(value.At(3));
}

bool IsFinite(const dt::Math::Matrix3<dtDynReal> &value)
{
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex column = 0; column < 3; ++column)
        {
            if (!std::isfinite(value.At(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

bool NearlyEqual(const dtDynReal lhs, const dtDynReal rhs)
{
    const dtDynReal scale = std::max(static_cast<dtDynReal>(1.0),
                                     std::max(std::fabs(lhs), std::fabs(rhs)));
    return std::fabs(lhs - rhs) <=
           ((static_cast<dtDynReal>(64.0) * std::numeric_limits<dtDynReal>::epsilon()) * scale);
}

bool NearlyEqualInput(const dtDynReal lhs, const dtDynReal rhs)
{
    const dtDynReal scale = std::max(static_cast<dtDynReal>(1.0),
                                     std::max(std::fabs(lhs), std::fabs(rhs)));
    return std::fabs(lhs - rhs) <=
           (std::sqrt(std::numeric_limits<dtDynReal>::epsilon()) * scale);
}

bool IsUnitVector(const dt::Math::Vector3<dtDynReal> &value)
{
    const dtDynReal squaredNorm = ((value.At(0) * value.At(0)) + (value.At(1) * value.At(1))) +
                                  (value.At(2) * value.At(2));
    return NearlyEqualInput(squaredNorm, static_cast<dtDynReal>(1.0));
}

bool IsUnitQuaternion(const dt::Math::Vector4<dtDynReal> &value)
{
    const dtDynReal squaredNorm = (((value.At(0) * value.At(0)) + (value.At(1) * value.At(1))) +
                                   (value.At(2) * value.At(2))) +
                                  (value.At(3) * value.At(3));
    return NearlyEqualInput(squaredNorm, static_cast<dtDynReal>(1.0));
}

bool IsPhysicalInertia(const dt::Math::Vector3<dtDynReal> &value)
{
    if ((value.At(0) < 0.0) || (value.At(1) < 0.0) || (value.At(2) < 0.0))
    {
        return false;
    }
    if (!(value.At(0) <= (value.At(1) + value.At(2))))
    {
        if (!NearlyEqual(value.At(0), value.At(1) + value.At(2)))
        {
            return false;
        }
    }
    if (!(value.At(1) <= (value.At(0) + value.At(2))))
    {
        if (!NearlyEqual(value.At(1), value.At(0) + value.At(2)))
        {
            return false;
        }
    }
    if (!(value.At(2) <= (value.At(0) + value.At(1))))
    {
        if (!NearlyEqual(value.At(2), value.At(0) + value.At(1)))
        {
            return false;
        }
    }
    return true;
}

bool IsProperRotation(const dt::Math::Matrix3<dtDynReal> &value)
{
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex column = 0; column < 3; ++column)
        {
            dtDynReal dot = 0.0;
            for (dtDynIndex index = 0; index < 3; ++index)
            {
                dot += value.At(index, row) * value.At(index, column);
            }
            const dtDynReal expected = (row == column) ? 1.0 : 0.0;
            if (!NearlyEqualInput(dot, expected))
            {
                return false;
            }
        }
    }
    const dtDynReal determinant =
        ((value.At(0, 0) * ((value.At(1, 1) * value.At(2, 2)) - (value.At(1, 2) * value.At(2, 1)))) -
         (value.At(0, 1) * ((value.At(1, 0) * value.At(2, 2)) - (value.At(1, 2) * value.At(2, 0))))) +
        (value.At(0, 2) * ((value.At(1, 0) * value.At(2, 1)) - (value.At(1, 1) * value.At(2, 0))));
    return NearlyEqualInput(determinant, static_cast<dtDynReal>(1.0));
}

bool MatrixMatchesQuaternion(const dt::Math::Matrix3<dtDynReal> &matrix,
                             const dt::Math::Vector4<dtDynReal> &quaternion)
{
    if (!IsUnitQuaternion(quaternion))
    {
        return false;
    }
    const dt::Math::Matrix3<dtDynReal> expected = math::QuatToMat(quaternion);
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex column = 0; column < 3; ++column)
        {
            if (!NearlyEqualInput(matrix.At(row, column), expected.At(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

} // namespace

bool ValidateSpec(const dtDynSpec &spec, dtDynValidationError *error)
{
    ClearError(error);
    const dtArray<dtDynBodySpec> &bodies = spec.Bodies();
    if (bodies.empty())
    {
        return Fail(error, dtDynValidationCode::InvalidWorld, dtDynObjectType::Body,
                    DTDYN_INVALID_INDEX, "spec has no world body[0]");
    }
    if (bodies[0].parent != DTDYN_INVALID_INDEX)
    {
        return Fail(error, dtDynValidationCode::InvalidWorld, dtDynObjectType::Body, 0,
                    "body[0] world must have an invalid parent");
    }
    for (std::size_t bodyIndex = 1; bodyIndex < bodies.size(); ++bodyIndex)
    {
        const dtDynIndex bodyId = static_cast<dtDynIndex>(bodyIndex);
        const dtDynIndex parent = bodies[bodyIndex].parent;
        if ((parent < 0) || (parent >= bodyId))
        {
            return Fail(error, dtDynValidationCode::InvalidParent, dtDynObjectType::Body, bodyId,
                        "body[" + std::to_string(bodyId) + "] '" + bodies[bodyIndex].name +
                            "' has invalid parent " + std::to_string(parent));
        }
    }
    for (std::size_t bodyIndex = 0; bodyIndex < bodies.size(); ++bodyIndex)
    {
        const dtDynBodySpec &body = bodies[bodyIndex];
        const dtDynIndex bodyId = static_cast<dtDynIndex>(bodyIndex);
        if (!IsFinite(body.pos))
        {
            return FailNonFiniteSpecBody(error, bodyId);
        }
        if (!IsFinite(body.quat))
        {
            return FailNonFiniteSpecBody(error, bodyId);
        }
        if (!std::isfinite(body.inertial.mass))
        {
            return FailNonFiniteSpecBody(error, bodyId);
        }
        if (!IsFinite(body.inertial.pos))
        {
            return FailNonFiniteSpecBody(error, bodyId);
        }
        if (!IsFinite(body.inertial.quat))
        {
            return FailNonFiniteSpecBody(error, bodyId);
        }
        if (!IsFinite(body.inertial.inertia))
        {
            return FailNonFiniteSpecBody(error, bodyId);
        }
        if (body.inertial.mass < 0.0)
        {
            return FailInvalidSpecBodyPhysicalInertia(error, bodyId);
        }
        if (!IsPhysicalInertia(body.inertial.inertia))
        {
            return FailInvalidSpecBodyPhysicalInertia(error, bodyId);
        }
        if (!IsUnitQuaternion(body.quat))
        {
            return FailInvalidSpecBodyQuaternion(error, bodyId);
        }
        if (!IsUnitQuaternion(body.inertial.quat))
        {
            return FailInvalidSpecBodyQuaternion(error, bodyId);
        }
    }

    const long long maximum = std::numeric_limits<uint16_t>::max();
    if (bodies.size() > static_cast<std::size_t>(maximum) ||
        spec.Joints().size() > static_cast<std::size_t>(maximum) ||
        spec.Sites().size() > static_cast<std::size_t>(maximum) ||
        spec.Actuators().size() > static_cast<std::size_t>(maximum))
    {
        return Fail(error, dtDynValidationCode::DimensionOverflow, dtDynObjectType::Body,
                    DTDYN_INVALID_INDEX, "spec object count exceeds UINT16_MAX");
    }
    long long nq = 0;
    long long nv = 0;
    for (std::size_t jointIndex = 0; jointIndex < spec.Joints().size(); ++jointIndex)
    {
        const dtDynJointDofSize size = GetJointDofSize(spec.Joints()[jointIndex].type);
        nq += size.qposSize;
        nv += size.qvelSize;
        if (nq > maximum || nv > maximum)
        {
            return Fail(error, dtDynValidationCode::DimensionOverflow, dtDynObjectType::Joint,
                        static_cast<dtDynIndex>(jointIndex), "joint[" + std::to_string(jointIndex) + "] makes nq/nv exceed UINT16_MAX");
        }
    }
    long long nroot = 0;
    for (std::size_t bodyIndex = 1; bodyIndex < bodies.size(); ++bodyIndex)
    {
        if (bodies[bodyIndex].parent == 0)
        {
            ++nroot;
        }
    }
    if ((6LL * nroot) > maximum)
    {
        return Fail(error, dtDynValidationCode::DimensionOverflow, dtDynObjectType::Body,
                    DTDYN_INVALID_INDEX, "root-stacked dimension 6*nroot exceeds UINT16_MAX");
    }

    dtArray<dtDynIndex> childSeen;
    childSeen.assign(bodies.size(), 0);
    for (std::size_t parentIndex = 0; parentIndex < bodies.size(); ++parentIndex)
    {
        const dtDynIndex parentId = static_cast<dtDynIndex>(parentIndex);
        dtDynIndex child = bodies[parentIndex].firstChild;
        std::size_t steps = 0;
        while (child != DTDYN_INVALID_INDEX)
        {
            if ((child <= 0) || (static_cast<std::size_t>(child) >= bodies.size()))
            {
                return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                            parentId, "body[" + std::to_string(parentId) + "] child list contains invalid body " + std::to_string(child));
            }
            if (++steps > bodies.size() || childSeen[static_cast<std::size_t>(child)] != 0)
            {
                return Fail(error, dtDynValidationCode::TopologyCycle, dtDynObjectType::Body, child,
                            "body[" + std::to_string(child) + "] appears repeatedly in child lists");
            }
            childSeen[static_cast<std::size_t>(child)] = 1;
            if (bodies[static_cast<std::size_t>(child)].parent != parentId)
            {
                return Fail(error, dtDynValidationCode::InvalidOwner, dtDynObjectType::Body, child,
                            "body[" + std::to_string(child) +
                                "] child-list owner disagrees with its parent");
            }
            child = bodies[static_cast<std::size_t>(child)].nextSibling;
        }
    }
    for (std::size_t bodyIndex = 1; bodyIndex < bodies.size(); ++bodyIndex)
    {
        if (childSeen[bodyIndex] != 1)
        {
            return Fail(error, dtDynValidationCode::InvalidOwner, dtDynObjectType::Body,
                        static_cast<dtDynIndex>(bodyIndex), "body[" + std::to_string(bodyIndex) + "] is absent from its parent's child list");
        }
    }

    const dtArray<dtDynJointSpec> &joints = spec.Joints();
    const dtArray<dtDynSiteSpec> &sites = spec.Sites();
    dtArray<dtDynIndex> jointBacklinks;
    dtArray<dtDynIndex> siteBacklinks;
    jointBacklinks.assign(joints.size(), 0);
    siteBacklinks.assign(sites.size(), 0);
    for (std::size_t bodyIndex = 0; bodyIndex < bodies.size(); ++bodyIndex)
    {
        const dtDynIndex bodyId = static_cast<dtDynIndex>(bodyIndex);
        const dtDynBodySpec &body = bodies[bodyIndex];
        for (std::size_t localIndex = 0; localIndex < body.joints.size(); ++localIndex)
        {
            const dtDynIndex jointId = body.joints[localIndex];
            if ((jointId < 0) || (static_cast<std::size_t>(jointId) >= joints.size()))
            {
                return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                            bodyId, "body[" + std::to_string(bodyId) + "] contains invalid joint " + std::to_string(jointId));
            }
            if (joints[static_cast<std::size_t>(jointId)].body != bodyId)
            {
                return Fail(error, dtDynValidationCode::InvalidOwner, dtDynObjectType::Joint,
                            jointId, "joint[" + std::to_string(jointId) + "] owner disagrees with body joint list");
            }
            ++jointBacklinks[static_cast<std::size_t>(jointId)];
        }
        for (std::size_t localIndex = 0; localIndex < body.sites.size(); ++localIndex)
        {
            const dtDynIndex siteId = body.sites[localIndex];
            if ((siteId < 0) || (static_cast<std::size_t>(siteId) >= sites.size()))
            {
                return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                            bodyId, "body[" + std::to_string(bodyId) + "] contains invalid site " + std::to_string(siteId));
            }
            if (sites[static_cast<std::size_t>(siteId)].body != bodyId)
            {
                return Fail(error, dtDynValidationCode::InvalidOwner, dtDynObjectType::Site,
                            siteId, "site[" + std::to_string(siteId) + "] owner disagrees with body site list");
            }
            ++siteBacklinks[static_cast<std::size_t>(siteId)];
        }
    }

    for (std::size_t jointIndex = 0; jointIndex < joints.size(); ++jointIndex)
    {
        const dtDynIndex jointId = static_cast<dtDynIndex>(jointIndex);
        const dtDynJointSpec &joint = joints[jointIndex];
        const dtDynIndex type = static_cast<dtDynIndex>(joint.type);
        if ((type < static_cast<dtDynIndex>(dtDynJointType::Free)) ||
            (type > static_cast<dtDynIndex>(dtDynJointType::Hinge)))
        {
            return Fail(error, dtDynValidationCode::UnsupportedJoint, dtDynObjectType::Joint, jointId,
                        "joint[" + std::to_string(jointId) + "] '" + joint.name +
                            "' has an unsupported joint type");
        }
        if (!IsFinite(joint.pos))
        {
            return FailNonFiniteSpecJoint(error, jointId);
        }
        if (!IsFinite(joint.axis))
        {
            return FailNonFiniteSpecJoint(error, jointId);
        }
        if (!std::isfinite(joint.rangeMin))
        {
            return FailNonFiniteSpecJoint(error, jointId);
        }
        if (!std::isfinite(joint.rangeMax))
        {
            return FailNonFiniteSpecJoint(error, jointId);
        }
        if (!std::isfinite(joint.armature))
        {
            return FailNonFiniteSpecJoint(error, jointId);
        }
        if ((joint.body <= 0) || (static_cast<std::size_t>(joint.body) >= bodies.size()))
        {
            return Fail(error, dtDynValidationCode::DetachedObject, dtDynObjectType::Joint, jointId,
                        "joint[" + std::to_string(jointId) + "] '" + joint.name +
                            "' is not attached to a non-world body");
        }
        if (jointBacklinks[jointIndex] != 1)
        {
            return Fail(error, dtDynValidationCode::InvalidOwner, dtDynObjectType::Joint, jointId,
                        "joint[" + std::to_string(jointId) + "] '" + joint.name +
                            "' does not have exactly one owning-body backlink");
        }
        if (joint.type == dtDynJointType::Slide)
        {
            if (!IsUnitVector(joint.axis))
            {
                return FailInvalidSpecSlideHingeAxis(error, jointId);
            }
            if (joint.armature < 0.0)
            {
                return FailNegativeSpecJointArmature(error, jointId);
            }
            continue;
        }
        if (joint.type == dtDynJointType::Hinge)
        {
            if (!IsUnitVector(joint.axis))
            {
                return FailInvalidSpecSlideHingeAxis(error, jointId);
            }
            if (joint.armature < 0.0)
            {
                return FailNegativeSpecJointArmature(error, jointId);
            }
            continue;
        }
        if (joint.armature < 0.0)
        {
            return FailNegativeSpecJointArmature(error, jointId);
        }
    }

    for (std::size_t siteIndex = 0; siteIndex < sites.size(); ++siteIndex)
    {
        const dtDynIndex siteId = static_cast<dtDynIndex>(siteIndex);
        const dtDynSiteSpec &site = sites[siteIndex];
        if (!IsFinite(site.pos))
        {
            return FailNonFiniteSpecSite(error, siteId);
        }
        if (!IsFinite(site.quat))
        {
            return FailNonFiniteSpecSite(error, siteId);
        }
        if (!IsUnitQuaternion(site.quat))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Site,
                        siteId, "site quaternion must be unit length");
        }
        if ((site.body <= 0) || (static_cast<std::size_t>(site.body) >= bodies.size()))
        {
            return Fail(error, dtDynValidationCode::DetachedObject, dtDynObjectType::Site, siteId,
                        "site[" + std::to_string(siteId) + "] '" + site.name +
                            "' is not attached to a non-world body");
        }
        if (siteBacklinks[siteIndex] != 1)
        {
            return Fail(error, dtDynValidationCode::InvalidOwner, dtDynObjectType::Site, siteId,
                        "site[" + std::to_string(siteId) + "] '" + site.name +
                            "' does not have exactly one owning-body backlink");
        }
    }

    const dtArray<dtDynActuatorSpec> &actuators = spec.Actuators();
    for (std::size_t actuatorIndex = 0; actuatorIndex < actuators.size(); ++actuatorIndex)
    {
        const dtDynIndex actuatorId = static_cast<dtDynIndex>(actuatorIndex);
        const dtDynActuatorSpec &actuator = actuators[actuatorIndex];
        if (!std::isfinite(actuator.gear))
        {
            return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Actuator,
                        actuatorId, "actuator[" + std::to_string(actuatorId) + "] contains a non-finite value");
        }
        if ((actuator.joint < 0) ||
            (static_cast<std::size_t>(actuator.joint) >= joints.size()))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Actuator,
                        actuatorId, "actuator[" + std::to_string(actuatorId) + "] '" + actuator.name + "' references invalid joint " + std::to_string(actuator.joint));
        }
    }
    if (!ValidateUniqueNames(bodies, dtDynObjectType::Body, "body", error))
    {
        return false;
    }
    if (!ValidateUniqueNames(joints, dtDynObjectType::Joint, "joint", error))
    {
        return false;
    }
    if (!ValidateUniqueNames(sites, dtDynObjectType::Site, "site", error))
    {
        return false;
    }
    return ValidateUniqueNames(actuators, dtDynObjectType::Actuator, "actuator", error);
}

bool ValidateModel(const dtDynModel &model, dtDynValidationError *error)
{
    ClearError(error);
    const long long maximum = std::numeric_limits<uint16_t>::max();
    struct NamedDimension
    {
        const char *name;
        long long value;
    };
    const NamedDimension dimensions[] = {
        {"nq", model.nq}, {"nv", model.nv}, {"nbody", model.nbody}, {"njnt", model.njnt}, {"nsite", model.nsite}, {"nu", model.nu}, {"nroot", model.nroot}, {"3*nroot", 3LL * model.nroot}, {"6*nroot", 6LL * model.nroot}};
    for (const NamedDimension &dimension : dimensions)
    {
        if ((dimension.value < 0) || (dimension.value > maximum))
        {
            return Fail(error, dtDynValidationCode::DimensionOverflow, dtDynObjectType::Body,
                        DTDYN_INVALID_INDEX, std::string("model dimension ") + dimension.name + "=" + std::to_string(dimension.value) + " is outside [0, UINT16_MAX]");
        }
    }
    if (model.nbody == 0)
    {
        return Fail(error, dtDynValidationCode::InvalidWorld, dtDynObjectType::Body,
                    DTDYN_INVALID_INDEX, "compiled model has no world body[0]");
    }

#define DTDYN_REQUIRE_ARRAY(member, count)                                \
    if (!RequireModelArraySize(#member, model.member.size(), model.count, \
                               error))                                    \
    return false
#define DTDYN_REQUIRE_DENSE(member, count)                                 \
    if (!RequireModelDenseSize(#member, model.member, model.count, error)) \
    return false

    DTDYN_REQUIRE_ARRAY(body_names, nbody);
    DTDYN_REQUIRE_ARRAY(body_parentid, nbody);
    DTDYN_REQUIRE_ARRAY(body_rootid, nbody);
    DTDYN_REQUIRE_ARRAY(body_rootidx, nbody);
    DTDYN_REQUIRE_ARRAY(body_jntnum, nbody);
    DTDYN_REQUIRE_ARRAY(body_jntadr, nbody);
    DTDYN_REQUIRE_ARRAY(body_dofnum, nbody);
    DTDYN_REQUIRE_ARRAY(body_dofadr, nbody);
    DTDYN_REQUIRE_ARRAY(body_weldid, nbody);
    DTDYN_REQUIRE_DENSE(body_mass, nbody);
    DTDYN_REQUIRE_DENSE(body_subtreemass, nbody);
    DTDYN_REQUIRE_ARRAY(body_pos, nbody);
    DTDYN_REQUIRE_ARRAY(body_quat, nbody);
    DTDYN_REQUIRE_ARRAY(body_mat, nbody);
    DTDYN_REQUIRE_ARRAY(body_ipos, nbody);
    DTDYN_REQUIRE_ARRAY(body_iquat, nbody);
    DTDYN_REQUIRE_ARRAY(body_imat, nbody);
    DTDYN_REQUIRE_ARRAY(body_inertia, nbody);

    DTDYN_REQUIRE_ARRAY(joint_names, njnt);
    DTDYN_REQUIRE_ARRAY(jnt_type, njnt);
    DTDYN_REQUIRE_ARRAY(jnt_bodyid, njnt);
    DTDYN_REQUIRE_ARRAY(jnt_specid, njnt);
    DTDYN_REQUIRE_ARRAY(jnt_qposadr, njnt);
    DTDYN_REQUIRE_ARRAY(jnt_dofadr, njnt);
    DTDYN_REQUIRE_ARRAY(jnt_limited, njnt);
    DTDYN_REQUIRE_DENSE(jnt_range_min, njnt);
    DTDYN_REQUIRE_DENSE(jnt_range_max, njnt);
    DTDYN_REQUIRE_ARRAY(jnt_pos, njnt);
    DTDYN_REQUIRE_ARRAY(jnt_axis, njnt);

    DTDYN_REQUIRE_ARRAY(dof_bodyid, nv);
    DTDYN_REQUIRE_ARRAY(dof_parentid, nv);
    DTDYN_REQUIRE_ARRAY(dof_jntid, nv);
    DTDYN_REQUIRE_DENSE(dof_armature, nv);

    DTDYN_REQUIRE_ARRAY(site_names, nsite);
    DTDYN_REQUIRE_ARRAY(site_bodyid, nsite);
    DTDYN_REQUIRE_ARRAY(site_pos, nsite);
    DTDYN_REQUIRE_ARRAY(site_quat, nsite);
    DTDYN_REQUIRE_ARRAY(site_mat, nsite);

    DTDYN_REQUIRE_ARRAY(actuator_names, nu);
    DTDYN_REQUIRE_ARRAY(actuator_jointid, nu);
    DTDYN_REQUIRE_DENSE(actuator_gear, nu);
    DTDYN_REQUIRE_ARRAY(root_bodyid, nroot);
    DTDYN_REQUIRE_DENSE(root_mass, nroot);

#undef DTDYN_REQUIRE_DENSE
#undef DTDYN_REQUIRE_ARRAY

    if (!ValidateUniqueNames(model.body_names, dtDynObjectType::Body, "body", error))
    {
        return false;
    }
    if (!ValidateUniqueNames(model.joint_names, dtDynObjectType::Joint, "joint", error))
    {
        return false;
    }
    if (!ValidateUniqueNames(model.site_names, dtDynObjectType::Site, "site", error))
    {
        return false;
    }
    if (!ValidateUniqueNames(model.actuator_names, dtDynObjectType::Actuator, "actuator", error))
    {
        return false;
    }

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t index = static_cast<std::size_t>(bodyId);
        if (!std::isfinite(model.body_mass.At(bodyId)))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (!std::isfinite(model.body_subtreemass.At(bodyId)))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (!IsFinite(model.body_pos[index]))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (!IsFinite(model.body_quat[index]))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (!IsFinite(model.body_mat[index]))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (!IsFinite(model.body_ipos[index]))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (!IsFinite(model.body_iquat[index]))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (!IsFinite(model.body_imat[index]))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (!IsFinite(model.body_inertia[index]))
        {
            return FailNonFiniteBody(error, bodyId);
        }
        if (model.body_mass.At(bodyId) < 0.0)
        {
            return FailInvalidBodyPhysicalOrFrame(error, bodyId);
        }
        if (!IsPhysicalInertia(model.body_inertia[index]))
        {
            return FailInvalidBodyPhysicalOrFrame(error, bodyId);
        }
        if (!IsProperRotation(model.body_mat[index]))
        {
            return FailInvalidBodyPhysicalOrFrame(error, bodyId);
        }
        if (!IsProperRotation(model.body_imat[index]))
        {
            return FailInvalidBodyPhysicalOrFrame(error, bodyId);
        }
        if (!MatrixMatchesQuaternion(model.body_mat[index], model.body_quat[index]))
        {
            return FailInvalidBodyPhysicalOrFrame(error, bodyId);
        }
        if (!MatrixMatchesQuaternion(model.body_imat[index], model.body_iquat[index]))
        {
            return FailInvalidBodyPhysicalOrFrame(error, bodyId);
        }
    }
    dtArray<dtDynIndex> jointSpecCoverage;
    jointSpecCoverage.assign(model.njnt, 0);
    for (dtDynIndex jointId = 0; jointId < model.njnt; ++jointId)
    {
        const std::size_t index = static_cast<std::size_t>(jointId);
        const dtDynIndex specJointId = model.jnt_specid[index];
        if (specJointId < 0)
        {
            return FailInvalidJointSpecProvenance(error, jointId);
        }
        if (specJointId >= model.njnt)
        {
            return FailInvalidJointSpecProvenance(error, jointId);
        }
        if (++jointSpecCoverage[static_cast<std::size_t>(specJointId)] != 1)
        {
            return FailInvalidJointSpecProvenance(error, jointId);
        }
        if (!std::isfinite(model.jnt_range_min.At(jointId)))
        {
            return FailNonFiniteJoint(error, jointId);
        }
        if (!std::isfinite(model.jnt_range_max.At(jointId)))
        {
            return FailNonFiniteJoint(error, jointId);
        }
        if (!IsFinite(model.jnt_pos[index]))
        {
            return FailNonFiniteJoint(error, jointId);
        }
        if (!IsFinite(model.jnt_axis[index]))
        {
            return FailNonFiniteJoint(error, jointId);
        }
        const dtDynJointType jointType = static_cast<dtDynJointType>(model.jnt_type[index]);
        if (jointType == dtDynJointType::Slide)
        {
            if (!IsUnitVector(model.jnt_axis[index]))
            {
                return FailInvalidSlideHingeAxis(error, jointId);
            }
            continue;
        }
        if (jointType == dtDynJointType::Hinge)
        {
            if (!IsUnitVector(model.jnt_axis[index]))
            {
                return FailInvalidSlideHingeAxis(error, jointId);
            }
        }
    }
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        if (!std::isfinite(model.dof_armature.At(dofId)))
        {
            return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Joint,
                        model.dof_jntid[static_cast<std::size_t>(dofId)],
                        "compiled DOF armature is non-finite");
        }
        if (model.dof_armature.At(dofId) < 0.0)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        model.dof_jntid[static_cast<std::size_t>(dofId)],
                        "compiled DOF armature must be nonnegative");
        }
    }
    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        const std::size_t index = static_cast<std::size_t>(siteId);
        if (!IsFinite(model.site_pos[index]))
        {
            return FailNonFiniteSite(error, siteId);
        }
        if (!IsFinite(model.site_quat[index]))
        {
            return FailNonFiniteSite(error, siteId);
        }
        if (!IsFinite(model.site_mat[index]))
        {
            return FailNonFiniteSite(error, siteId);
        }
        if (!IsProperRotation(model.site_mat[index]))
        {
            return FailInvalidSiteFramePair(error, siteId);
        }
        if (!MatrixMatchesQuaternion(model.site_mat[index], model.site_quat[index]))
        {
            return FailInvalidSiteFramePair(error, siteId);
        }
    }
    for (dtDynIndex actuatorId = 0; actuatorId < model.nu; ++actuatorId)
    {
        if (!std::isfinite(model.actuator_gear.At(actuatorId)))
        {
            return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Actuator,
                        actuatorId, "compiled actuator gear is non-finite");
        }
    }
    for (dtDynIndex rootIndex = 0; rootIndex < model.nroot; ++rootIndex)
    {
        if (!std::isfinite(model.root_mass.At(rootIndex)))
        {
            return Fail(error, dtDynValidationCode::NonFiniteValue, dtDynObjectType::Body,
                        model.root_bodyid[static_cast<std::size_t>(rootIndex)],
                        "compiled root mass is non-finite");
        }
    }

    if (model.nbody > 0 && model.body_parentid[0] != 0)
    {
        return Fail(error, dtDynValidationCode::InvalidWorld, dtDynObjectType::Body, 0,
                    "compiled body[0] world parent must be 0");
    }
    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        if (parent < 0 || parent >= bodyId)
        {
            return Fail(error, dtDynValidationCode::InvalidParent, dtDynObjectType::Body, bodyId,
                        "compiled body[" + std::to_string(bodyId) + "] has invalid parent " +
                            std::to_string(parent));
        }
    }
    dtArray<dtDynReal> expectedSubtreeMass;
    expectedSubtreeMass.resize(model.nbody);
    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        expectedSubtreeMass[static_cast<std::size_t>(bodyId)] = model.body_mass.At(bodyId);
    }
    for (dtDynIndex bodyId = model.nbody - 1; bodyId > 0; --bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        expectedSubtreeMass[static_cast<std::size_t>(parent)] +=
            expectedSubtreeMass[static_cast<std::size_t>(bodyId)];
    }
    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        if (!NearlyEqual(model.body_subtreemass.At(bodyId),
                         expectedSubtreeMass[static_cast<std::size_t>(bodyId)]))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                        bodyId, "compiled body subtree mass is inconsistent");
        }
    }
    if ((model.body_rootid[0] != 0) || (model.body_rootidx[0] != DTDYN_INVALID_INDEX))
    {
        return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, 0,
                    "compiled world root metadata is invalid");
    }
    dtDynIndex expectedRootCount = 0;
    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        if (model.body_parentid[static_cast<std::size_t>(bodyId)] == 0)
        {
            ++expectedRootCount;
        }
    }
    if (expectedRootCount != model.nroot)
    {
        return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                    DTDYN_INVALID_INDEX, "compiled nroot disagrees with top-level bodies");
    }
    dtArray<dtDynIndex> rootSeen;
    rootSeen.assign(model.nbody, 0);
    for (dtDynIndex rootIndex = 0; rootIndex < model.nroot; ++rootIndex)
    {
        const dtDynIndex rootBody = model.root_bodyid[static_cast<std::size_t>(rootIndex)];
        if ((rootBody <= 0) || (rootBody >= model.nbody) ||
            (model.body_parentid[static_cast<std::size_t>(rootBody)] != 0) ||
            (model.body_rootid[static_cast<std::size_t>(rootBody)] != rootBody) ||
            (model.body_rootidx[static_cast<std::size_t>(rootBody)] != rootIndex))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                        rootBody, "compiled root table contains inconsistent metadata");
        }
        if (rootSeen[static_cast<std::size_t>(rootBody)] != 0)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                        rootBody, "compiled root body appears more than once");
        }
        rootSeen[static_cast<std::size_t>(rootBody)] = 1;
        if (!NearlyEqual(model.root_mass.At(rootIndex),
                         expectedSubtreeMass[static_cast<std::size_t>(rootBody)]))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                        rootBody, "compiled root mass is inconsistent with subtree mass");
        }
    }
    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t index = static_cast<std::size_t>(bodyId);
        const dtDynIndex parent = model.body_parentid[index];
        const dtDynIndex rootBody = model.body_rootid[index];
        const dtDynIndex rootIndex = model.body_rootidx[index];
        const dtDynIndex expectedRoot =
            (parent == 0) ? bodyId : model.body_rootid[static_cast<std::size_t>(parent)];
        if ((rootBody != expectedRoot) || (rootIndex < 0) || (rootIndex >= model.nroot) ||
            (model.root_bodyid[static_cast<std::size_t>(rootIndex)] != rootBody))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                        bodyId, "compiled body root ID/slot is inconsistent");
        }
    }
    if (model.body_weldid[0] != 0)
    {
        return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, 0,
                    "compiled world weld ID must be 0");
    }
    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t index = static_cast<std::size_t>(bodyId);
        const dtDynIndex parent = model.body_parentid[index];
        const dtDynIndex expectedWeld = (model.body_jntnum[index] > 0)
                                            ? bodyId
                                            : ((parent > 0)
                                                   ? model.body_weldid[static_cast<std::size_t>(parent)]
                                                   : 0);
        if (model.body_weldid[index] != expectedWeld)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body,
                        bodyId, "compiled body weld ID is inconsistent");
        }
    }
    dtArray<dtDynIndex> jointBodyCoverage;
    dtArray<dtDynIndex> dofBodyCoverage;
    jointBodyCoverage.assign(model.njnt, 0);
    dofBodyCoverage.assign(model.nv, 0);
    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t index = static_cast<std::size_t>(bodyId);
        const dtDynIndex jointAddress = model.body_jntadr[index];
        const dtDynIndex jointCount = model.body_jntnum[index];
        const dtDynIndex dofAddress = model.body_dofadr[index];
        const dtDynIndex dofCount = model.body_dofnum[index];
        if ((jointCount < 0) ||
            ((jointCount == 0) && (jointAddress != DTDYN_INVALID_INDEX)) ||
            ((jointCount > 0) &&
             ((jointAddress < 0) ||
              ((static_cast<long long>(jointAddress) + jointCount) > model.njnt))))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, bodyId,
                        "compiled body joint segment is invalid");
        }
        if ((dofCount < 0) ||
            ((dofCount == 0) && (dofAddress != DTDYN_INVALID_INDEX)) ||
            ((dofCount > 0) &&
             ((dofAddress < 0) ||
              ((static_cast<long long>(dofAddress) + dofCount) > model.nv))))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Body, bodyId,
                        "compiled body DOF segment is invalid");
        }
        for (dtDynIndex offset = 0; offset < jointCount; ++offset)
        {
            const dtDynIndex jointId = jointAddress + offset;
            if (model.jnt_bodyid[static_cast<std::size_t>(jointId)] != bodyId)
            {
                return FailInvalidBodyJointSegment(error, bodyId);
            }
            if (++jointBodyCoverage[static_cast<std::size_t>(jointId)] != 1)
            {
                return FailInvalidBodyJointSegment(error, bodyId);
            }
        }
        for (dtDynIndex offset = 0; offset < dofCount; ++offset)
        {
            const dtDynIndex dofId = dofAddress + offset;
            if (model.dof_bodyid[static_cast<std::size_t>(dofId)] != bodyId)
            {
                return FailInvalidBodyDofSegment(error, bodyId);
            }
            if (++dofBodyCoverage[static_cast<std::size_t>(dofId)] != 1)
            {
                return FailInvalidBodyDofSegment(error, bodyId);
            }
        }
    }
    for (dtDynIndex jointId = 0; jointId < model.njnt; ++jointId)
    {
        if (jointBodyCoverage[static_cast<std::size_t>(jointId)] != 1)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        jointId, "compiled joint is absent from body ownership segments");
        }
    }
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        if (dofBodyCoverage[static_cast<std::size_t>(dofId)] != 1)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        model.dof_jntid[static_cast<std::size_t>(dofId)],
                        "compiled DOF is absent from body ownership segments");
        }
    }
    dtArray<dtDynIndex> qposCoverage;
    dtArray<dtDynIndex> jointDofCoverage;
    qposCoverage.assign(model.nq, 0);
    jointDofCoverage.assign(model.nv, 0);
    for (dtDynIndex jointId = 0; jointId < model.njnt; ++jointId)
    {
        const std::size_t index = static_cast<std::size_t>(jointId);
        const dtDynIndex typeValue = model.jnt_type[index];
        if ((typeValue < static_cast<dtDynIndex>(dtDynJointType::Free)) ||
            (typeValue > static_cast<dtDynIndex>(dtDynJointType::Hinge)))
        {
            return Fail(error, dtDynValidationCode::UnsupportedJoint, dtDynObjectType::Joint,
                        jointId, "compiled joint has an unsupported type");
        }
        const dtDynJointDofSize size = GetJointDofSize(static_cast<dtDynJointType>(typeValue));
        const dtDynIndex qposAddress = model.jnt_qposadr[index];
        const dtDynIndex dofAddress = model.jnt_dofadr[index];
        if ((qposAddress < 0) || (dofAddress < 0) ||
            ((static_cast<long long>(qposAddress) + size.qposSize) > model.nq) ||
            ((static_cast<long long>(dofAddress) + size.qvelSize) > model.nv))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        jointId, "compiled joint qpos/DOF span is invalid");
        }
        const dtDynIndex bodyId = model.jnt_bodyid[index];
        if (bodyId <= 0 || bodyId >= model.nbody)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        jointId, "compiled joint references an invalid body");
        }
        for (dtDynIndex offset = 0; offset < size.qposSize; ++offset)
        {
            const dtDynIndex qposId = qposAddress + offset;
            if (++qposCoverage[static_cast<std::size_t>(qposId)] != 1)
            {
                return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                            jointId, "compiled joint qpos spans overlap");
            }
        }
        for (dtDynIndex offset = 0; offset < size.qvelSize; ++offset)
        {
            const dtDynIndex dofId = dofAddress + offset;
            const std::size_t dofIndex = static_cast<std::size_t>(dofId);
            if (model.dof_jntid[dofIndex] != jointId)
            {
                return FailInvalidJointDofSpanOwnership(error, jointId);
            }
            if (model.dof_bodyid[dofIndex] != bodyId)
            {
                return FailInvalidJointDofSpanOwnership(error, jointId);
            }
            if (++jointDofCoverage[dofIndex] != 1)
            {
                return FailInvalidJointDofSpanOwnership(error, jointId);
            }
        }
    }
    for (dtDynIndex qposId = 0; qposId < model.nq; ++qposId)
    {
        if (qposCoverage[static_cast<std::size_t>(qposId)] != 1)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        DTDYN_INVALID_INDEX, "compiled qpos coordinate is not owned exactly once");
        }
    }
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        if (jointDofCoverage[static_cast<std::size_t>(dofId)] != 1)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        model.dof_jntid[static_cast<std::size_t>(dofId)],
                        "compiled DOF is not owned by exactly one joint span");
        }
    }
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        const std::size_t index = static_cast<std::size_t>(dofId);
        const dtDynIndex bodyId = model.dof_bodyid[index];
        const dtDynIndex jointId = model.dof_jntid[index];
        const dtDynIndex parent = model.dof_parentid[index];
        if (bodyId <= 0 || bodyId >= model.nbody || jointId < 0 || jointId >= model.njnt ||
            model.jnt_bodyid[static_cast<std::size_t>(jointId)] != bodyId)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        jointId, "compiled DOF[" + std::to_string(dofId) + "] has invalid body/joint ownership");
        }
        if (parent < DTDYN_INVALID_INDEX)
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Joint,
                        jointId, "compiled DOF has an invalid parent sentinel");
        }
        if (parent >= dofId)
        {
            return Fail(error, dtDynValidationCode::TopologyCycle, dtDynObjectType::Joint,
                        jointId, "compiled DOF[" + std::to_string(dofId) + "] parent is not an earlier DOF");
        }
    }
    dtArray<dtDynIndex> bodyLastDof;
    bodyLastDof.assign(model.nbody, DTDYN_INVALID_INDEX);
    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t bodyIndex = static_cast<std::size_t>(bodyId);
        const dtDynIndex parentBody = model.body_parentid[bodyIndex];
        dtDynIndex expectedParent = parentBody > 0
                                        ? bodyLastDof[static_cast<std::size_t>(parentBody)]
                                        : DTDYN_INVALID_INDEX;
        const dtDynIndex jointAddress = model.body_jntadr[bodyIndex];
        const dtDynIndex jointCount = model.body_jntnum[bodyIndex];
        for (dtDynIndex localJoint = 0; localJoint < jointCount; ++localJoint)
        {
            const dtDynIndex jointId = jointAddress + localJoint;
            const std::size_t jointIndex = static_cast<std::size_t>(jointId);
            const dtDynJointDofSize size = GetJointDofSize(
                static_cast<dtDynJointType>(model.jnt_type[jointIndex]));
            const dtDynIndex dofAddress = model.jnt_dofadr[jointIndex];
            for (dtDynIndex offset = 0; offset < size.qvelSize; ++offset)
            {
                const dtDynIndex dofId = dofAddress + offset;
                if (model.dof_parentid[static_cast<std::size_t>(dofId)] != expectedParent)
                {
                    return Fail(error, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Joint, jointId,
                                "compiled DOF parent does not match the exact ancestor chain");
                }
                expectedParent = dofId;
            }
        }
        bodyLastDof[bodyIndex] = expectedParent;
    }
    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        const dtDynIndex bodyId = model.site_bodyid[static_cast<std::size_t>(siteId)];
        if ((bodyId <= 0) || (bodyId >= model.nbody))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Site,
                        siteId, "compiled site references an invalid body");
        }
    }
    for (dtDynIndex actuatorId = 0; actuatorId < model.nu; ++actuatorId)
    {
        const dtDynIndex jointId = model.actuator_jointid[static_cast<std::size_t>(actuatorId)];
        if ((jointId < 0) || (jointId >= model.njnt))
        {
            return Fail(error, dtDynValidationCode::InvalidReference, dtDynObjectType::Actuator,
                        actuatorId, "compiled actuator references an invalid joint");
        }
    }
    return true;
}

} // namespace dt::Dynamics
