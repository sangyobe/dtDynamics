/*!
 \file      dtDynCompile.cpp
 \brief     Native dtDynamics spec-to-model compiler definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.1
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
        model.body_pos[static_cast<std::size_t>(bodyId)] = body.pos;
        model.body_quat[static_cast<std::size_t>(bodyId)] =
            math::NormalizeOrIdentity(body.quat);
        model.body_mat[static_cast<std::size_t>(bodyId)] =
            math::QuatToMat(model.body_quat[static_cast<std::size_t>(bodyId)]);
        model.body_ipos[static_cast<std::size_t>(bodyId)] =
            body.inertial.mass == 0.0 ? body.pos : body.inertial.pos;
        model.body_iquat[static_cast<std::size_t>(bodyId)] =
            body.inertial.mass == 0.0
                ? model.body_quat[static_cast<std::size_t>(bodyId)]
                : math::NormalizeOrIdentity(body.inertial.quat);
        model.body_imat[static_cast<std::size_t>(bodyId)] =
            math::QuatToMat(model.body_iquat[static_cast<std::size_t>(bodyId)]);
        model.body_inertia[static_cast<std::size_t>(bodyId)] = body.inertial.inertia;
        model.body_mass(bodyId) = body.inertial.mass;
        model.body_subtreemass(bodyId) = body.inertial.mass;

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
                model.jnt_qposadr[static_cast<std::size_t>(modelJointId)] = nextQposAdr;
                model.jnt_dofadr[static_cast<std::size_t>(modelJointId)] = nextDofAdr;
                model.jnt_limited[static_cast<std::size_t>(modelJointId)] = joint.limited ? 1 : 0;
                model.jnt_range_min(modelJointId) = joint.rangeMin;
                model.jnt_range_max(modelJointId) = joint.rangeMax;
                model.jnt_pos[static_cast<std::size_t>(modelJointId)] = joint.pos;
                model.jnt_axis[static_cast<std::size_t>(modelJointId)] = joint.axis;

                for (dtDynIndex dofOffset = 0; dofOffset < dofSize.qvelSize; ++dofOffset)
                {
                    const dtDynIndex dofId = nextDofAdr + dofOffset;
                    model.dof_bodyid[static_cast<std::size_t>(dofId)] = bodyId;
                    model.dof_parentid[static_cast<std::size_t>(dofId)] = parentDof;
                    model.dof_jntid[static_cast<std::size_t>(dofId)] = modelJointId;
                    model.dof_armature(dofId) = joint.armature;
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
        model.site_pos[static_cast<std::size_t>(siteId)] = site.pos;
        model.site_quat[static_cast<std::size_t>(siteId)] =
            math::NormalizeOrIdentity(site.quat);
        model.site_mat[static_cast<std::size_t>(siteId)] =
            math::QuatToMat(model.site_quat[static_cast<std::size_t>(siteId)]);
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
        model.actuator_gear(actuatorId) = actuator.gear;
    }

    for (dtDynIndex bodyId = model.nbody - 1; bodyId > 0; --bodyId)
    {
        const dtDynIndex parent = model.body_parentid[static_cast<std::size_t>(bodyId)];
        model.body_subtreemass(parent) += model.body_subtreemass(bodyId);
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
        model.root_mass(rootIdx) = model.body_subtreemass(rootBody);
    }

    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        const dtDynIndex rootBody = model.body_rootid[static_cast<std::size_t>(bodyId)];
        model.body_rootidx[static_cast<std::size_t>(bodyId)] =
            rootSlotByBody[static_cast<std::size_t>(rootBody)];
    }

    model.RebuildNameMaps();
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
