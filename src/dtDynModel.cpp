/*!
 \file      dtDynModel.cpp
 \brief     Compiled native dtDynamics model definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynModel.h"

namespace dt::Dynamics
{
namespace
{

void AssignDenseVector(dtDynDenseVector &dst, const dtDynDenseVector &src)
{
    const dtDynIndex dim = static_cast<dtDynIndex>(src.GetDim());
    if (dim == 0)
    {
        dst.Release();
        return;
    }

    ResizeDenseVector(dst, dim);
    dst = src;
}

} // namespace

dtDynModel::dtDynModel(const dtDynModel &) = default;

dtDynModel &dtDynModel::operator=(const dtDynModel &other)
{
    if (this == &other)
    {
        return *this;
    }

    nq = other.nq;
    nv = other.nv;
    nbody = other.nbody;
    njnt = other.njnt;
    nsite = other.nsite;
    nu = other.nu;
    nroot = other.nroot;

    body_names = other.body_names;
    joint_names = other.joint_names;
    site_names = other.site_names;
    actuator_names = other.actuator_names;

    body_parentid = other.body_parentid;
    body_rootid = other.body_rootid;
    body_rootidx = other.body_rootidx;
    body_jntnum = other.body_jntnum;
    body_jntadr = other.body_jntadr;
    body_dofnum = other.body_dofnum;
    body_dofadr = other.body_dofadr;
    body_weldid = other.body_weldid;

    jnt_type = other.jnt_type;
    jnt_bodyid = other.jnt_bodyid;
    jnt_specid = other.jnt_specid;
    jnt_qposadr = other.jnt_qposadr;
    jnt_dofadr = other.jnt_dofadr;
    jnt_limited = other.jnt_limited;
    AssignDenseVector(jnt_range_min, other.jnt_range_min);
    AssignDenseVector(jnt_range_max, other.jnt_range_max);

    dof_bodyid = other.dof_bodyid;
    dof_parentid = other.dof_parentid;
    dof_jntid = other.dof_jntid;
    AssignDenseVector(dof_armature, other.dof_armature);

    site_bodyid = other.site_bodyid;
    actuator_jointid = other.actuator_jointid;
    root_bodyid = other.root_bodyid;
    AssignDenseVector(actuator_gear, other.actuator_gear);
    AssignDenseVector(root_mass, other.root_mass);

    AssignDenseVector(body_mass, other.body_mass);
    AssignDenseVector(body_subtreemass, other.body_subtreemass);
    body_pos = other.body_pos;
    body_quat = other.body_quat;
    body_mat = other.body_mat;
    body_ipos = other.body_ipos;
    body_iquat = other.body_iquat;
    body_imat = other.body_imat;
    body_inertia = other.body_inertia;
    jnt_pos = other.jnt_pos;
    jnt_axis = other.jnt_axis;
    site_pos = other.site_pos;
    site_quat = other.site_quat;
    site_mat = other.site_mat;

    m_bodyNameToId = other.m_bodyNameToId;
    m_jointNameToId = other.m_jointNameToId;
    m_siteNameToId = other.m_siteNameToId;
    m_actuatorNameToId = other.m_actuatorNameToId;

    return *this;
}

dtDynIndex dtDynModel::BodyId(const std::string &name) const
{
    const auto it = m_bodyNameToId.find(name);
    return (it == m_bodyNameToId.end()) ? DTDYN_INVALID_INDEX : it->second;
}

dtDynIndex dtDynModel::JointId(const std::string &name) const
{
    const auto it = m_jointNameToId.find(name);
    return (it == m_jointNameToId.end()) ? DTDYN_INVALID_INDEX : it->second;
}

dtDynIndex dtDynModel::SiteId(const std::string &name) const
{
    const auto it = m_siteNameToId.find(name);
    return (it == m_siteNameToId.end()) ? DTDYN_INVALID_INDEX : it->second;
}

dtDynIndex dtDynModel::ActuatorId(const std::string &name) const
{
    const auto it = m_actuatorNameToId.find(name);
    return (it == m_actuatorNameToId.end()) ? DTDYN_INVALID_INDEX : it->second;
}

bool dtDynModel::IsFixedBody(const dtDynIndex body) const
{
    if (body < 0 || body >= static_cast<dtDynIndex>(body_jntnum.size()))
    {
        return false;
    }

    return body_jntnum[body] == 0 &&
           body_jntadr[body] == DTDYN_INVALID_INDEX &&
           body_dofnum[body] == 0 &&
           body_dofadr[body] == DTDYN_INVALID_INDEX;
}

void dtDynModel::RebuildNameMaps()
{
    m_bodyNameToId.clear();
    m_jointNameToId.clear();
    m_siteNameToId.clear();
    m_actuatorNameToId.clear();

    for (dtDynIndex i = 0; i < static_cast<dtDynIndex>(body_names.size()); ++i)
    {
        m_bodyNameToId[body_names[static_cast<std::size_t>(i)]] = i;
    }

    for (dtDynIndex i = 0; i < static_cast<dtDynIndex>(joint_names.size()); ++i)
    {
        m_jointNameToId[joint_names[static_cast<std::size_t>(i)]] = i;
    }

    for (dtDynIndex i = 0; i < static_cast<dtDynIndex>(site_names.size()); ++i)
    {
        m_siteNameToId[site_names[static_cast<std::size_t>(i)]] = i;
    }

    for (dtDynIndex i = 0; i < static_cast<dtDynIndex>(actuator_names.size()); ++i)
    {
        m_actuatorNameToId[actuator_names[static_cast<std::size_t>(i)]] = i;
    }
}

} // namespace dt::Dynamics
