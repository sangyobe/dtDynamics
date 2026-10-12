/*!
 \file      dtDynCompile.cpp
 \brief     Native dtDynamics spec-to-model compiler definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynCompile.h"

#include "dtDynamics/dtDynMath.h"
#include "dtDynamics/dtDynValidation.h"

#include <stdexcept>

namespace dt::Dynamics
{
namespace
{

void ResizeModelArrays(dtDynModel &model)
{
    model.body_names.resize(model.nbody);
    model.joint_names.resize(model.njnt);
    model.site_names.resize(model.nsite);
    model.actuator_names.resize(model.nu);

    model.body_parentid.assign(model.nbody, 0);
    model.body_rootid.assign(model.nbody, 0);
    model.body_rootidx.assign(model.nbody, DTDYN_INVALID_INDEX);
    model.body_jntnum.assign(model.nbody, 0);
    model.body_jntadr.assign(model.nbody, DTDYN_INVALID_INDEX);
    model.body_dofnum.assign(model.nbody, 0);
    model.body_dofadr.assign(model.nbody, DTDYN_INVALID_INDEX);
    model.body_weldid.assign(model.nbody, 0);

    model.jnt_type.resize(model.njnt);
    model.jnt_bodyid.resize(model.njnt);
    model.jnt_specid.assign(model.njnt, DTDYN_INVALID_INDEX);
    model.jnt_qposadr.resize(model.njnt);
    model.jnt_dofadr.resize(model.njnt);
    model.jnt_limited.assign(model.njnt, 0);
    ResizeDenseVector(model.jnt_range_min, model.njnt);
    ResizeDenseVector(model.jnt_range_max, model.njnt);

    model.dof_bodyid.resize(model.nv);
    model.dof_parentid.assign(model.nv, DTDYN_INVALID_INDEX);
    model.dof_jntid.resize(model.nv);
    ResizeDenseVector(model.dof_armature, model.nv);

    model.site_bodyid.resize(model.nsite);
    model.actuator_jointid.assign(model.nu, DTDYN_INVALID_INDEX);
    model.root_bodyid.clear();
    ResizeDenseVector(model.actuator_gear, model.nu);
    ResizeDenseVector(model.root_mass, 0);

    ResizeDenseVector(model.body_mass, model.nbody);
    ResizeDenseVector(model.body_subtreemass, model.nbody);
    model.body_pos.resize(model.nbody);
    model.body_quat.resize(model.nbody);
    model.body_mat.resize(model.nbody);
    model.body_ipos.resize(model.nbody);
    model.body_iquat.resize(model.nbody);
    model.body_imat.resize(model.nbody);
    model.body_inertia.resize(model.nbody);
    model.jnt_pos.resize(model.njnt);
    model.jnt_axis.resize(model.njnt);
    model.site_pos.resize(model.nsite);
    model.site_quat.resize(model.nsite);
    model.site_mat.resize(model.nsite);
}

dtDynIndex GetJointSpecId(const dtDynBodySpec &body, const dtDynIndex jointIndex)
{
    return body.joints[static_cast<std::size_t>(jointIndex)];
}

} // namespace

bool UpdateSpec(const dtDynSpec &spec, dtDynModel &model) noexcept
{
    const auto hasArraySize = [](const std::size_t actual, const dtDynIndex expected) noexcept
    {
        return expected >= 0 && actual == static_cast<std::size_t>(expected);
    };
    const auto hasDenseSize = [](const dtDynDenseVector &value, const dtDynIndex expected) noexcept
    {
        return expected >= 0 && value.GetDim() == expected;
    };

    const dtArray<dtDynBodySpec> &bodies = spec.Bodies();
    const dtArray<dtDynJointSpec> &joints = spec.Joints();
    const dtArray<dtDynSiteSpec> &sites = spec.Sites();
    const dtArray<dtDynActuatorSpec> &actuators = spec.Actuators();

    if (model.nq < 0 || model.nv < 0 || model.nbody < 1 || model.njnt < 0 ||
        model.nsite < 0 || model.nu < 0 || model.nroot < 0 ||
        !hasArraySize(bodies.size(), model.nbody) ||
        !hasArraySize(joints.size(), model.njnt) ||
        !hasArraySize(sites.size(), model.nsite) ||
        !hasArraySize(actuators.size(), model.nu))
    {
        return false;
    }

    if (!hasArraySize(model.body_names.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_parentid.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_rootid.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_rootidx.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_jntnum.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_jntadr.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_dofnum.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_dofadr.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_weldid.size(), model.nbody))
    {
        return false;
    }
    if (!hasDenseSize(model.body_mass, model.nbody))
    {
        return false;
    }
    if (!hasDenseSize(model.body_subtreemass, model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_pos.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_quat.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_mat.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_ipos.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_iquat.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_imat.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.body_inertia.size(), model.nbody))
    {
        return false;
    }
    if (!hasArraySize(model.joint_names.size(), model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.jnt_type.size(), model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.jnt_bodyid.size(), model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.jnt_specid.size(), model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.jnt_qposadr.size(), model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.jnt_dofadr.size(), model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.jnt_limited.size(), model.njnt))
    {
        return false;
    }
    if (!hasDenseSize(model.jnt_range_min, model.njnt))
    {
        return false;
    }
    if (!hasDenseSize(model.jnt_range_max, model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.jnt_pos.size(), model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.jnt_axis.size(), model.njnt))
    {
        return false;
    }
    if (!hasArraySize(model.dof_bodyid.size(), model.nv))
    {
        return false;
    }
    if (!hasArraySize(model.dof_parentid.size(), model.nv))
    {
        return false;
    }
    if (!hasArraySize(model.dof_jntid.size(), model.nv))
    {
        return false;
    }
    if (!hasDenseSize(model.dof_armature, model.nv))
    {
        return false;
    }
    if (!hasArraySize(model.site_names.size(), model.nsite))
    {
        return false;
    }
    if (!hasArraySize(model.site_bodyid.size(), model.nsite))
    {
        return false;
    }
    if (!hasArraySize(model.site_pos.size(), model.nsite))
    {
        return false;
    }
    if (!hasArraySize(model.site_quat.size(), model.nsite))
    {
        return false;
    }
    if (!hasArraySize(model.site_mat.size(), model.nsite))
    {
        return false;
    }
    if (!hasArraySize(model.actuator_names.size(), model.nu))
    {
        return false;
    }
    if (!hasArraySize(model.actuator_jointid.size(), model.nu))
    {
        return false;
    }
    if (!hasDenseSize(model.actuator_gear, model.nu))
    {
        return false;
    }
    if (!hasArraySize(model.root_bodyid.size(), model.nroot))
    {
        return false;
    }
    if (!hasDenseSize(model.root_mass, model.nroot))
    {
        return false;
    }

    dtDynIndex nextJointId = 0;
    dtDynIndex nextQposAdr = 0;
    dtDynIndex nextDofAdr = 0;
    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t bodyIndex = static_cast<std::size_t>(bodyId);
        const dtDynBodySpec &body = bodies[bodyIndex];
        const dtDynIndex expectedParent = bodyId == 0 ? 0 : body.parent;
        if (model.body_names[bodyIndex] != body.name ||
            model.body_parentid[bodyIndex] != expectedParent ||
            (bodyId == 0 && body.parent != DTDYN_INVALID_INDEX) ||
            (bodyId > 0 && (body.parent < 0 || body.parent >= bodyId)) ||
            body.joints.size() > static_cast<std::size_t>(model.njnt))
        {
            return false;
        }

        const dtDynIndex bodyJointCount = static_cast<dtDynIndex>(body.joints.size());
        if (model.body_jntnum[bodyIndex] != bodyJointCount ||
            (bodyJointCount == 0 &&
             (model.body_jntadr[bodyIndex] != DTDYN_INVALID_INDEX ||
              model.body_dofadr[bodyIndex] != DTDYN_INVALID_INDEX ||
              model.body_dofnum[bodyIndex] != 0)) ||
            (bodyJointCount > 0 &&
             (model.body_jntadr[bodyIndex] != nextJointId ||
              model.body_dofadr[bodyIndex] != nextDofAdr)))
        {
            return false;
        }

        dtDynIndex parentDof = DTDYN_INVALID_INDEX;
        if (bodyId > 0)
        {
            for (dtDynIndex ancestor = body.parent; ancestor > 0;
                 ancestor = model.body_parentid[static_cast<std::size_t>(ancestor)])
            {
                const std::size_t ancestorIndex = static_cast<std::size_t>(ancestor);
                const dtDynIndex ancestorDofCount = model.body_dofnum[ancestorIndex];
                const dtDynIndex ancestorDofAdr = model.body_dofadr[ancestorIndex];
                if (ancestorDofCount > 0)
                {
                    if (ancestorDofAdr < 0 || ancestorDofAdr > model.nv - ancestorDofCount)
                    {
                        return false;
                    }
                    parentDof = ancestorDofAdr + ancestorDofCount - 1;
                    break;
                }
            }
        }

        dtDynIndex bodyDofCount = 0;
        for (dtDynIndex localJointId = 0; localJointId < bodyJointCount; ++localJointId)
        {
            const dtDynIndex specJointId = GetJointSpecId(body, localJointId);
            if (specJointId < 0 || specJointId >= model.njnt || nextJointId >= model.njnt)
            {
                return false;
            }

            const std::size_t specJointIndex = static_cast<std::size_t>(specJointId);
            const std::size_t modelJointIndex = static_cast<std::size_t>(nextJointId);
            const dtDynJointSpec &joint = joints[specJointIndex];
            const dtDynIndex type = static_cast<dtDynIndex>(joint.type);
            if (joint.body != bodyId ||
                type < static_cast<dtDynIndex>(dtDynJointType::Free) ||
                type > static_cast<dtDynIndex>(dtDynJointType::Hinge) ||
                model.joint_names[modelJointIndex] != joint.name ||
                model.jnt_specid[modelJointIndex] != specJointId ||
                model.jnt_type[modelJointIndex] != type ||
                model.jnt_bodyid[modelJointIndex] != bodyId ||
                model.jnt_qposadr[modelJointIndex] != nextQposAdr ||
                model.jnt_dofadr[modelJointIndex] != nextDofAdr)
            {
                return false;
            }

            const dtDynJointDofSize dofSize = GetJointDofSize(joint.type);
            if (dofSize.qposSize < 0 || dofSize.qvelSize < 0 ||
                nextQposAdr > model.nq - dofSize.qposSize ||
                nextDofAdr > model.nv - dofSize.qvelSize)
            {
                return false;
            }
            for (dtDynIndex dofOffset = 0; dofOffset < dofSize.qvelSize; ++dofOffset)
            {
                const dtDynIndex dofId = nextDofAdr + dofOffset;
                const std::size_t dofIndex = static_cast<std::size_t>(dofId);
                if (model.dof_bodyid[dofIndex] != bodyId ||
                    model.dof_parentid[dofIndex] != parentDof ||
                    model.dof_jntid[dofIndex] != nextJointId)
                {
                    return false;
                }
                parentDof = dofId;
            }

            nextQposAdr += dofSize.qposSize;
            nextDofAdr += dofSize.qvelSize;
            bodyDofCount += dofSize.qvelSize;
            ++nextJointId;
        }
        if (model.body_dofnum[bodyIndex] != bodyDofCount)
        {
            return false;
        }
    }

    dtDynIndex childLinkCount = 0;
    for (dtDynIndex parentId = 0; parentId < model.nbody; ++parentId)
    {
        const dtDynBodySpec &parent = bodies[static_cast<std::size_t>(parentId)];
        if (parentId == 0 && parent.nextSibling != DTDYN_INVALID_INDEX)
        {
            return false;
        }

        dtDynIndex child = parent.firstChild;
        dtDynIndex siblingSteps = 0;
        while (child != DTDYN_INVALID_INDEX)
        {
            if (child <= 0 || child >= model.nbody || ++siblingSteps > model.nbody ||
                childLinkCount >= model.nbody - 1)
            {
                return false;
            }
            if (bodies[static_cast<std::size_t>(child)].parent != parentId)
            {
                return false;
            }
            ++childLinkCount;
            child = bodies[static_cast<std::size_t>(child)].nextSibling;
        }
    }
    if (childLinkCount != model.nbody - 1)
    {
        return false;
    }

    if (nextJointId != model.njnt || nextQposAdr != model.nq || nextDofAdr != model.nv ||
        model.body_rootid[0] != 0 || model.body_rootidx[0] != DTDYN_INVALID_INDEX ||
        model.body_weldid[0] != 0)
    {
        return false;
    }

    dtDynIndex expectedRootIndex = 0;
    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t bodyIndex = static_cast<std::size_t>(bodyId);
        const dtDynIndex parent = model.body_parentid[bodyIndex];
        const dtDynIndex expectedRoot =
            parent == 0 ? bodyId : model.body_rootid[static_cast<std::size_t>(parent)];
        const dtDynIndex expectedRootSlot =
            parent == 0 ? expectedRootIndex : model.body_rootidx[static_cast<std::size_t>(parent)];
        const dtDynIndex expectedWeld = model.body_jntnum[bodyIndex] > 0
                                            ? bodyId
                                            : (parent > 0
                                                   ? model.body_weldid[static_cast<std::size_t>(parent)]
                                                   : 0);
        if (model.body_rootid[bodyIndex] != expectedRoot ||
            model.body_rootidx[bodyIndex] != expectedRootSlot ||
            model.body_weldid[bodyIndex] != expectedWeld)
        {
            return false;
        }
        if (parent == 0)
        {
            if (expectedRootIndex >= model.nroot ||
                model.root_bodyid[static_cast<std::size_t>(expectedRootIndex)] != bodyId)
            {
                return false;
            }
            ++expectedRootIndex;
        }
    }
    if (expectedRootIndex != model.nroot)
    {
        return false;
    }

    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        const std::size_t siteIndex = static_cast<std::size_t>(siteId);
        const dtDynSiteSpec &site = sites[siteIndex];
        if (site.body < 0 || site.body >= model.nbody ||
            model.site_names[siteIndex] != site.name || model.site_bodyid[siteIndex] != site.body)
        {
            return false;
        }
    }

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        const dtDynBodySpec &body = bodies[static_cast<std::size_t>(bodyId)];
        for (std::size_t localSiteId = 0; localSiteId < body.sites.size(); ++localSiteId)
        {
            const dtDynIndex siteId = body.sites[localSiteId];
            if (siteId < 0 || siteId >= model.nsite ||
                sites[static_cast<std::size_t>(siteId)].body != bodyId)
            {
                return false;
            }
        }
    }
    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        const dtDynSiteSpec &site = sites[static_cast<std::size_t>(siteId)];
        const dtDynBodySpec &owner = bodies[static_cast<std::size_t>(site.body)];
        dtDynIndex backlinkCount = 0;
        for (std::size_t localSiteId = 0; localSiteId < owner.sites.size(); ++localSiteId)
        {
            if (owner.sites[localSiteId] == siteId && ++backlinkCount > 1)
            {
                return false;
            }
        }
        if (backlinkCount != 1)
        {
            return false;
        }
    }

    for (dtDynIndex actuatorId = 0; actuatorId < model.nu; ++actuatorId)
    {
        const std::size_t actuatorIndex = static_cast<std::size_t>(actuatorId);
        const dtDynActuatorSpec &actuator = actuators[actuatorIndex];
        const dtDynIndex modelJointId = model.actuator_jointid[actuatorIndex];
        if (actuator.joint < 0 || actuator.joint >= model.njnt ||
            model.actuator_names[actuatorIndex] != actuator.name ||
            modelJointId < 0 || modelJointId >= model.njnt ||
            model.jnt_specid[static_cast<std::size_t>(modelJointId)] != actuator.joint)
        {
            return false;
        }
    }

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t bodyIndex = static_cast<std::size_t>(bodyId);
        const dtDynBodySpec &body = bodies[bodyIndex];
        model.body_pos[bodyIndex] = body.pos;
        model.body_quat[bodyIndex] = math::NormalizeOrIdentity(body.quat);
        model.body_mat[bodyIndex] = math::QuatToMat(model.body_quat[bodyIndex]);
        model.body_ipos[bodyIndex] = body.inertial.mass == 0.0 ? body.pos : body.inertial.pos;
        model.body_iquat[bodyIndex] = body.inertial.mass == 0.0
                                          ? model.body_quat[bodyIndex]
                                          : math::NormalizeOrIdentity(body.inertial.quat);
        model.body_imat[bodyIndex] = math::QuatToMat(model.body_iquat[bodyIndex]);
        model.body_inertia[bodyIndex] = body.inertial.inertia;
        model.body_mass(bodyId) = body.inertial.mass;
        model.body_subtreemass(bodyId) = body.inertial.mass;
    }

    for (dtDynIndex modelJointId = 0; modelJointId < model.njnt; ++modelJointId)
    {
        const std::size_t modelJointIndex = static_cast<std::size_t>(modelJointId);
        const dtDynIndex specJointId = model.jnt_specid[modelJointIndex];
        const dtDynJointSpec &joint = joints[static_cast<std::size_t>(specJointId)];
        const dtDynJointDofSize dofSize = GetJointDofSize(joint.type);
        model.jnt_limited[modelJointIndex] = joint.limited ? 1 : 0;
        model.jnt_range_min(modelJointId) = joint.rangeMin;
        model.jnt_range_max(modelJointId) = joint.rangeMax;
        model.jnt_pos[modelJointIndex] = joint.pos;
        model.jnt_axis[modelJointIndex] = joint.axis;
        for (dtDynIndex dofOffset = 0; dofOffset < dofSize.qvelSize; ++dofOffset)
        {
            model.dof_armature(model.jnt_dofadr[modelJointIndex] + dofOffset) = joint.armature;
        }
    }

    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        const std::size_t siteIndex = static_cast<std::size_t>(siteId);
        const dtDynSiteSpec &site = sites[siteIndex];
        model.site_pos[siteIndex] = site.pos;
        model.site_quat[siteIndex] = math::NormalizeOrIdentity(site.quat);
        model.site_mat[siteIndex] = math::QuatToMat(model.site_quat[siteIndex]);
    }

    for (dtDynIndex actuatorId = 0; actuatorId < model.nu; ++actuatorId)
    {
        model.actuator_gear(actuatorId) = actuators[static_cast<std::size_t>(actuatorId)].gear;
    }

    for (dtDynIndex bodyId = model.nbody - 1; bodyId > 0; --bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        model.body_subtreemass(parent) += model.body_subtreemass(bodyId);
    }
    for (dtDynIndex rootIndex = 0; rootIndex < model.nroot; ++rootIndex)
    {
        const dtDynIndex rootBody = model.root_bodyid[static_cast<std::size_t>(rootIndex)];
        model.root_mass(rootIndex) = model.body_subtreemass(rootBody);
    }

    return true;
}

void CompileSpec(const dtDynSpec &spec, dtDynModel &model)
{
    model.nq = 0;
    model.nv = 0;
    model.nbody = 0;
    model.njnt = 0;
    model.nsite = 0;
    model.nu = 0;
    model.nroot = 0;

    model.nbody = static_cast<dtDynIndex>(spec.Bodies().size());
    model.njnt = static_cast<dtDynIndex>(spec.Joints().size());
    model.nsite = static_cast<dtDynIndex>(spec.Sites().size());
    model.nu = static_cast<dtDynIndex>(spec.Actuators().size());

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        const dtDynBodySpec &body = spec.Bodies()[static_cast<std::size_t>(bodyId)];
        for (dtDynIndex localJointId = 0; localJointId < static_cast<dtDynIndex>(body.joints.size()); ++localJointId)
        {
            const dtDynJointSpec &joint = spec.Joints()[static_cast<std::size_t>(GetJointSpecId(body, localJointId))];
            const dtDynJointDofSize dofSize = GetJointDofSize(joint.type);
            model.nq += dofSize.qposSize;
            model.nv += dofSize.qvelSize;
        }
    }

    ResizeModelArrays(model);

    dtArray<dtDynIndex> jointIdMap;
    dtArray<dtDynIndex> bodyLastDof;
    jointIdMap.assign(model.njnt, DTDYN_INVALID_INDEX);
    bodyLastDof.assign(model.nbody, DTDYN_INVALID_INDEX);

    dtDynIndex nextJointId = 0;
    dtDynIndex nextQposAdr = 0;
    dtDynIndex nextDofAdr = 0;

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        const dtDynBodySpec &body = spec.Bodies()[static_cast<std::size_t>(bodyId)];

        model.body_names[static_cast<std::size_t>(bodyId)] = body.name;
        model.body_parentid[static_cast<std::size_t>(bodyId)] = bodyId == 0 ? 0 : body.parent;

        if (bodyId == 0)
        {
            continue;
        }

        const dtDynIndex parent = body.parent;
        model.body_rootid[static_cast<std::size_t>(bodyId)] =
            parent > 0 ? model.body_rootid[static_cast<std::size_t>(parent)] : bodyId;
        model.body_weldid[static_cast<std::size_t>(bodyId)] =
            parent > 0 ? model.body_weldid[static_cast<std::size_t>(parent)] : 0;

        if (!body.joints.empty())
        {
            model.body_jntadr[static_cast<std::size_t>(bodyId)] = nextJointId;
            model.body_jntnum[static_cast<std::size_t>(bodyId)] = static_cast<dtDynIndex>(body.joints.size());
            model.body_dofadr[static_cast<std::size_t>(bodyId)] = nextDofAdr;

            dtDynIndex bodyDofNum = 0;
            dtDynIndex parentDof = parent >= 0 ? bodyLastDof[static_cast<std::size_t>(parent)] : DTDYN_INVALID_INDEX;

            for (dtDynIndex localJointId = 0; localJointId < static_cast<dtDynIndex>(body.joints.size()); ++localJointId)
            {
                const dtDynIndex specJointId = GetJointSpecId(body, localJointId);
                const dtDynJointSpec &joint = spec.Joints()[static_cast<std::size_t>(specJointId)];
                const dtDynJointDofSize dofSize = GetJointDofSize(joint.type);
                const dtDynIndex modelJointId = nextJointId++;

                jointIdMap[static_cast<std::size_t>(specJointId)] = modelJointId;
                model.joint_names[static_cast<std::size_t>(modelJointId)] = joint.name;
                model.jnt_type[static_cast<std::size_t>(modelJointId)] = static_cast<dtDynIndex>(joint.type);
                model.jnt_bodyid[static_cast<std::size_t>(modelJointId)] = bodyId;
                model.jnt_specid[static_cast<std::size_t>(modelJointId)] = specJointId;
                model.jnt_qposadr[static_cast<std::size_t>(modelJointId)] = nextQposAdr;
                model.jnt_dofadr[static_cast<std::size_t>(modelJointId)] = nextDofAdr;

                for (dtDynIndex dofOffset = 0; dofOffset < dofSize.qvelSize; ++dofOffset)
                {
                    const dtDynIndex dofId = nextDofAdr + dofOffset;
                    model.dof_bodyid[static_cast<std::size_t>(dofId)] = bodyId;
                    model.dof_parentid[static_cast<std::size_t>(dofId)] = parentDof;
                    model.dof_jntid[static_cast<std::size_t>(dofId)] = modelJointId;
                    parentDof = dofId;
                }

                nextQposAdr += dofSize.qposSize;
                nextDofAdr += dofSize.qvelSize;
                bodyDofNum += dofSize.qvelSize;
            }

            model.body_dofnum[static_cast<std::size_t>(bodyId)] = bodyDofNum;
            bodyLastDof[static_cast<std::size_t>(bodyId)] = nextDofAdr - 1;
            model.body_weldid[static_cast<std::size_t>(bodyId)] = bodyId;
        }
        else if (parent >= 0)
        {
            bodyLastDof[static_cast<std::size_t>(bodyId)] = bodyLastDof[static_cast<std::size_t>(parent)];
        }
    }

    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        const dtDynSiteSpec &site = spec.Sites()[static_cast<std::size_t>(siteId)];
        model.site_names[static_cast<std::size_t>(siteId)] = site.name;
        model.site_bodyid[static_cast<std::size_t>(siteId)] = site.body;
    }

    for (dtDynIndex actuatorId = 0; actuatorId < model.nu; ++actuatorId)
    {
        const dtDynActuatorSpec &actuator = spec.Actuators()[static_cast<std::size_t>(actuatorId)];
        model.actuator_names[static_cast<std::size_t>(actuatorId)] = actuator.name;
        if (actuator.joint >= 0 && actuator.joint < static_cast<dtDynIndex>(jointIdMap.size()))
        {
            model.actuator_jointid[static_cast<std::size_t>(actuatorId)] =
                jointIdMap[static_cast<std::size_t>(actuator.joint)];
        }
    }

    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        if (model.body_rootid[static_cast<std::size_t>(bodyId)] == bodyId)
        {
            model.root_bodyid.push_back(bodyId);
        }
    }

    model.nroot = static_cast<dtDynIndex>(model.root_bodyid.size());
    ResizeDenseVector(model.root_mass, model.nroot);

    dtArray<dtDynIndex> rootSlotByBody;
    rootSlotByBody.assign(model.nbody, DTDYN_INVALID_INDEX);
    for (dtDynIndex rootIdx = 0; rootIdx < model.nroot; ++rootIdx)
    {
        const dtDynIndex rootBody = model.root_bodyid[static_cast<std::size_t>(rootIdx)];
        rootSlotByBody[static_cast<std::size_t>(rootBody)] = rootIdx;
    }

    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const dtDynIndex rootBody = model.body_rootid[static_cast<std::size_t>(bodyId)];
        model.body_rootidx[static_cast<std::size_t>(bodyId)] =
            rootSlotByBody[static_cast<std::size_t>(rootBody)];
    }

    model.RebuildNameMaps();
    if (!UpdateSpec(spec, model))
    {
        throw std::runtime_error("CompileSpec: failed to apply compiled specification");
    }
}

dtDynModel CompileSpec(const dtDynSpec &spec)
{
    dtDynModel model;
    CompileSpec(spec, model);
    return model;
}

dtDynModel CompileSpecChecked(const dtDynSpec &spec)
{
    dtDynValidationError error;
    if (!ValidateSpec(spec, &error))
    {
        throw std::runtime_error("CompileSpecChecked: " + error.message);
    }
    dtDynModel model = CompileSpec(spec);
    if (!ValidateModel(model, &error))
    {
        throw std::runtime_error("CompileSpecChecked: compiled model validation failed: " +
                                 error.message);
    }
    return model;
}

} // namespace dt::Dynamics
