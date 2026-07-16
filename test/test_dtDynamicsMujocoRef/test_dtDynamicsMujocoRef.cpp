#include "dtDynamics/dtDynCompile.h"
#include "dtDynamics/dtDynData.h"
#include "dtDynamics/dtDynDynamics.h"
#include "dtDynamics/dtDynKinematics.h"
#include "dtDynamics/dtDynSpec.h"
#include "dtDynamics/dtDynUtils.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace dt::Dynamics;

namespace
{

struct Fixture
{
    const char *name;
    const char *xml;
};

struct MjModelDeleter
{
    void operator()(mjModel *model) const
    {
        if (model != nullptr)
        {
            mj_deleteModel(model);
        }
    }
};

struct MjDataDeleter
{
    void operator()(mjData *data) const
    {
        if (data != nullptr)
        {
            mj_deleteData(data);
        }
    }
};

using MjModelPtr = std::unique_ptr<mjModel, MjModelDeleter>;
using MjDataPtr = std::unique_ptr<mjData, MjDataDeleter>;

MjModelPtr LoadMjModel(const std::string &name, const std::string &xml)
{
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    const std::string filename = name + ".xml";
    const int addResult = mj_addBufferVFS(&vfs, filename.c_str(), xml.data(), static_cast<int>(xml.size()));
    if (addResult != 0)
    {
        mj_deleteVFS(&vfs);
        throw std::runtime_error("MuJoCo VFS load failed");
    }

    char error[1024] = {};
    mjModel *rawModel = mj_loadXML(filename.c_str(), &vfs, error, sizeof(error));
    mj_deleteVFS(&vfs);
    if (rawModel == nullptr)
    {
        throw std::runtime_error(std::string("MuJoCo XML load failed: ") + error);
    }
    return MjModelPtr(rawModel);
}

std::string MjName(const mjModel *model, const int objectType, const int id)
{
    const char *name = mj_id2name(model, objectType, id);
    return name == nullptr ? std::string() : std::string(name);
}

void Expect(const bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

template <typename Actual, typename Expected>
void ExpectEqual(const Actual actual, const Expected expected, const std::string &context)
{
    if (!(actual == expected))
    {
        std::ostringstream out;
        out << context << " mismatch: actual=" << actual << ", expected=" << expected;
        throw std::runtime_error(out.str());
    }
}

void ExpectEqual(const std::string &actual, const std::string &expected, const std::string &context)
{
    if (actual != expected)
    {
        throw std::runtime_error(context + " mismatch: actual=" + actual + ", expected=" + expected);
    }
}

void ExpectNear(const double actual, const double expected, const std::string &context)
{
    if (std::fabs(actual - expected) >= 1.0e-10)
    {
        std::ostringstream out;
        out << context << " mismatch: actual=" << actual << ", expected=" << expected;
        throw std::runtime_error(out.str());
    }
}

void ExpectNearTol(const double actual, const double expected, const double tolerance, const std::string &context)
{
    if (std::fabs(actual - expected) >= tolerance)
    {
        std::ostringstream out;
        out << context << " mismatch: actual=" << actual << ", expected=" << expected
            << ", tolerance=" << tolerance;
        throw std::runtime_error(out.str());
    }
}

void SetReferenceState(const mjModel *mjModel, mjData *mjData)
{
    for (int jointId = 0; jointId < mjModel->njnt; ++jointId)
    {
        const int qadr = mjModel->jnt_qposadr[jointId];
        const int dadr = mjModel->jnt_dofadr[jointId];
        const double scale = static_cast<double>(jointId + 1);

        if (mjModel->jnt_type[jointId] == mjJNT_FREE)
        {
            mjData->qpos[qadr + 0] += 0.03 * scale;
            mjData->qpos[qadr + 1] -= 0.02 * scale;
            mjData->qpos[qadr + 2] += 0.01 * scale;

            const double angle = 0.05 * scale;
            mjData->qpos[qadr + 3] = std::cos(0.5 * angle);
            mjData->qpos[qadr + 4] = std::sin(0.5 * angle);
            mjData->qpos[qadr + 5] = 0.0;
            mjData->qpos[qadr + 6] = 0.0;

            mjData->qvel[dadr + 0] = 0.10 * scale;
            mjData->qvel[dadr + 1] = -0.05 * scale;
            mjData->qvel[dadr + 2] = 0.03 * scale;
            mjData->qvel[dadr + 3] = 0.02 * scale;
            mjData->qvel[dadr + 4] = -0.04 * scale;
            mjData->qvel[dadr + 5] = 0.06 * scale;
        }
        else if (mjModel->jnt_type[jointId] == mjJNT_BALL)
        {
            const double angle = 0.04 * scale;
            mjData->qpos[qadr + 0] = std::cos(0.5 * angle);
            mjData->qpos[qadr + 1] = 0.0;
            mjData->qpos[qadr + 2] = std::sin(0.5 * angle);
            mjData->qpos[qadr + 3] = 0.0;

            mjData->qvel[dadr + 0] = 0.03 * scale;
            mjData->qvel[dadr + 1] = -0.02 * scale;
            mjData->qvel[dadr + 2] = 0.04 * scale;
        }
        else
        {
            mjData->qpos[qadr] = 0.12 * scale;
            mjData->qvel[dadr] = ((jointId % 2) == 0 ? 0.20 : -0.15) * scale;
        }
    }
}

void CopyQpos(const mjModel *mjModel, const mjData *mjData, dtDynData &dtData)
{
    for (int i = 0; i < mjModel->nq; ++i)
    {
        dtData.qpos(i) = static_cast<dtDynReal>(mjData->qpos[i]);
    }
}

void CopyQvel(const mjModel *mjModel, const mjData *mjData, dtDynData &dtData)
{
    for (int i = 0; i < mjModel->nv; ++i)
    {
        dtData.qvel(i) = static_cast<dtDynReal>(mjData->qvel[i]);
    }
}

void CompareIndexLayout(const Fixture &fixture, const mjModel *mjModel, const dtDynModel &dtModel)
{
    const std::string prefix = std::string(fixture.name) + ": ";
    ExpectEqual(dtModel.nq, mjModel->nq, prefix + "nq");
    ExpectEqual(dtModel.nv, mjModel->nv, prefix + "nv");
    ExpectEqual(dtModel.nbody, mjModel->nbody, prefix + "nbody");
    ExpectEqual(dtModel.njnt, mjModel->njnt, prefix + "njnt");
    ExpectEqual(dtModel.nsite, mjModel->nsite, prefix + "nsite");
    ExpectEqual(dtModel.nu, mjModel->nu, prefix + "nu");

    for (int bodyId = 1; bodyId < mjModel->nbody; ++bodyId)
    {
        const std::string context = prefix + "body[" + std::to_string(bodyId) + "]";
        ExpectEqual(dtModel.body_names[static_cast<std::size_t>(bodyId)], MjName(mjModel, mjOBJ_BODY, bodyId), context + ".name");
        ExpectEqual(dtModel.body_parentid[static_cast<std::size_t>(bodyId)], mjModel->body_parentid[bodyId], context + ".parentid");
        ExpectEqual(dtModel.body_jntnum[static_cast<std::size_t>(bodyId)], mjModel->body_jntnum[bodyId], context + ".jntnum");
        ExpectEqual(dtModel.body_jntadr[static_cast<std::size_t>(bodyId)], mjModel->body_jntadr[bodyId], context + ".jntadr");
        ExpectEqual(dtModel.body_dofnum[static_cast<std::size_t>(bodyId)], mjModel->body_dofnum[bodyId], context + ".dofnum");
        ExpectEqual(dtModel.body_dofadr[static_cast<std::size_t>(bodyId)], mjModel->body_dofadr[bodyId], context + ".dofadr");
    }

    dtArray<dtDynIndex> expectedRootBodies;
    for (int bodyId = 1; bodyId < mjModel->nbody; ++bodyId)
    {
        if (mjModel->body_parentid[bodyId] == 0)
        {
            expectedRootBodies.push_back(bodyId);
        }
    }
    ExpectEqual(dtModel.nroot, static_cast<dtDynIndex>(expectedRootBodies.size()), prefix + "nroot");
    for (dtDynIndex rootIdx = 0; rootIdx < dtModel.nroot; ++rootIdx)
    {
        const dtDynIndex rootBody = expectedRootBodies[static_cast<std::size_t>(rootIdx)];
        const std::string context = prefix + "root[" + std::to_string(rootIdx) + "]";
        ExpectEqual(dtModel.root_bodyid[static_cast<std::size_t>(rootIdx)], rootBody, context + ".bodyid");
        ExpectNearTol(dtModel.root_mass(rootIdx), mjModel->body_subtreemass[rootBody], 1.0e-12, context + ".mass");
    }
    for (int bodyId = 1; bodyId < mjModel->nbody; ++bodyId)
    {
        dtDynIndex expectedRootIdx = DTDYN_INVALID_INDEX;
        for (dtDynIndex rootIdx = 0; rootIdx < dtModel.nroot; ++rootIdx)
        {
            if (dtModel.root_bodyid[static_cast<std::size_t>(rootIdx)] == mjModel->body_rootid[bodyId])
            {
                expectedRootIdx = rootIdx;
                break;
            }
        }
        ExpectEqual(dtModel.body_rootidx[static_cast<std::size_t>(bodyId)],
                    expectedRootIdx,
                    prefix + "body[" + std::to_string(bodyId) + "].rootidx");
    }

    for (int jointId = 0; jointId < mjModel->njnt; ++jointId)
    {
        const std::string context = prefix + "joint[" + std::to_string(jointId) + "]";
        ExpectEqual(dtModel.joint_names[static_cast<std::size_t>(jointId)], MjName(mjModel, mjOBJ_JOINT, jointId), context + ".name");
        ExpectEqual(dtModel.jnt_type[static_cast<std::size_t>(jointId)], mjModel->jnt_type[jointId], context + ".type");
        ExpectEqual(dtModel.jnt_bodyid[static_cast<std::size_t>(jointId)], mjModel->jnt_bodyid[jointId], context + ".bodyid");
        ExpectEqual(dtModel.jnt_qposadr[static_cast<std::size_t>(jointId)], mjModel->jnt_qposadr[jointId], context + ".qposadr");
        ExpectEqual(dtModel.jnt_dofadr[static_cast<std::size_t>(jointId)], mjModel->jnt_dofadr[jointId], context + ".dofadr");
        ExpectEqual(dtModel.jnt_limited[static_cast<std::size_t>(jointId)],
                    static_cast<dtDynIndex>(mjModel->jnt_limited[jointId]),
                    context + ".limited");
        ExpectNearTol(dtModel.jnt_range_min(jointId), mjModel->jnt_range[2 * jointId], 1.0e-12, context + ".range_min");
        ExpectNearTol(dtModel.jnt_range_max(jointId), mjModel->jnt_range[2 * jointId + 1], 1.0e-12, context + ".range_max");
    }

    for (int dofId = 0; dofId < mjModel->nv; ++dofId)
    {
        const std::string context = prefix + "dof[" + std::to_string(dofId) + "]";
        ExpectEqual(dtModel.dof_bodyid[static_cast<std::size_t>(dofId)], mjModel->dof_bodyid[dofId], context + ".bodyid");
        ExpectEqual(dtModel.dof_parentid[static_cast<std::size_t>(dofId)], mjModel->dof_parentid[dofId], context + ".parentid");
        ExpectEqual(dtModel.dof_jntid[static_cast<std::size_t>(dofId)], mjModel->dof_jntid[dofId], context + ".jntid");
        ExpectNearTol(dtModel.dof_armature(dofId), mjModel->dof_armature[dofId], 1.0e-12, context + ".armature");
    }

    for (int siteId = 0; siteId < mjModel->nsite; ++siteId)
    {
        const std::string context = prefix + "site[" + std::to_string(siteId) + "]";
        ExpectEqual(dtModel.site_names[static_cast<std::size_t>(siteId)], MjName(mjModel, mjOBJ_SITE, siteId), context + ".name");
        ExpectEqual(dtModel.site_bodyid[static_cast<std::size_t>(siteId)], mjModel->site_bodyid[siteId], context + ".bodyid");
    }

    for (int actuatorId = 0; actuatorId < mjModel->nu; ++actuatorId)
    {
        const std::string context = prefix + "actuator[" + std::to_string(actuatorId) + "]";
        ExpectEqual(dtModel.actuator_names[static_cast<std::size_t>(actuatorId)],
                    MjName(mjModel, mjOBJ_ACTUATOR, actuatorId),
                    context + ".name");
        ExpectEqual(mjModel->actuator_trntype[actuatorId], static_cast<int>(mjTRN_JOINT), context + ".trntype");
        ExpectEqual(dtModel.actuator_jointid[static_cast<std::size_t>(actuatorId)],
                    mjModel->actuator_trnid[2 * actuatorId],
                    context + ".jointid");
        ExpectNearTol(dtModel.actuator_gear(actuatorId),
                      mjModel->actuator_gear[6 * actuatorId],
                      1.0e-12,
                      context + ".gear");
    }
}

void CompareVector3(const dt::Math::Vector3<dtDynReal> &actual,
                    const mjtNum *expected,
                    const double tolerance,
                    const std::string &context)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        ExpectNearTol(actual(axis),
                      static_cast<double>(expected[axis]),
                      tolerance,
                      context + "[" + std::to_string(axis) + "]");
    }
}

void CompareVector4(const dt::Math::Vector4<dtDynReal> &actual,
                    const mjtNum *expected,
                    const double tolerance,
                    const std::string &context)
{
    for (int axis = 0; axis < 4; ++axis)
    {
        ExpectNearTol(actual(axis),
                      static_cast<double>(expected[axis]),
                      tolerance,
                      context + "[" + std::to_string(axis) + "]");
    }
}

void CompareMatrix3(const dt::Math::Matrix3<dtDynReal> &actual,
                    const mjtNum *expected,
                    const double tolerance,
                    const std::string &context)
{
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            ExpectNearTol(actual(row, col),
                          static_cast<double>(expected[3 * row + col]),
                          tolerance,
                          context + "[" + std::to_string(row) + "," + std::to_string(col) + "]");
        }
    }
}

dtArray<mjtNum> MakeMjtBuffer(const std::size_t size)
{
    dtArray<mjtNum> buffer;
    buffer.assign(size, 0.0);
    return buffer;
}

void FillZero(dtArray<mjtNum> &buffer)
{
    for (mjtNum &value : buffer)
    {
        value = 0.0;
    }
}

void CompareKinematicModelParameters(const Fixture &fixture, const mjModel *mjModel, const dtDynModel &dtModel)
{
    const std::string prefix = std::string(fixture.name) + ": ";

    for (int bodyId = 0; bodyId < mjModel->nbody; ++bodyId)
    {
        const std::string context = prefix + "body[" + std::to_string(bodyId) + "]";
        ExpectEqual(dtModel.body_rootid[static_cast<std::size_t>(bodyId)], mjModel->body_rootid[bodyId], context + ".rootid");
        ExpectEqual(dtModel.body_weldid[static_cast<std::size_t>(bodyId)], mjModel->body_weldid[bodyId], context + ".weldid");
        ExpectNearTol(dtModel.body_mass(bodyId), mjModel->body_mass[bodyId], 1.0e-12, context + ".mass");
        ExpectNearTol(dtModel.body_subtreemass(bodyId), mjModel->body_subtreemass[bodyId], 1.0e-12, context + ".subtreemass");
        CompareVector3(dtModel.body_pos[static_cast<std::size_t>(bodyId)], mjModel->body_pos + 3 * bodyId, 1.0e-12, context + ".pos");
        CompareVector4(dtModel.body_quat[static_cast<std::size_t>(bodyId)], mjModel->body_quat + 4 * bodyId, 1.0e-12, context + ".quat");
        CompareVector3(dtModel.body_ipos[static_cast<std::size_t>(bodyId)], mjModel->body_ipos + 3 * bodyId, 1.0e-12, context + ".ipos");
        CompareVector4(dtModel.body_iquat[static_cast<std::size_t>(bodyId)], mjModel->body_iquat + 4 * bodyId, 1.0e-12, context + ".iquat");
        CompareVector3(dtModel.body_inertia[static_cast<std::size_t>(bodyId)], mjModel->body_inertia + 3 * bodyId, 1.0e-12, context + ".inertia");
    }

    for (int jointId = 0; jointId < mjModel->njnt; ++jointId)
    {
        const std::string context = prefix + "joint[" + std::to_string(jointId) + "]";
        CompareVector3(dtModel.jnt_pos[static_cast<std::size_t>(jointId)], mjModel->jnt_pos + 3 * jointId, 1.0e-12, context + ".pos");
        CompareVector3(dtModel.jnt_axis[static_cast<std::size_t>(jointId)], mjModel->jnt_axis + 3 * jointId, 1.0e-12, context + ".axis");
    }

    for (int siteId = 0; siteId < mjModel->nsite; ++siteId)
    {
        const std::string context = prefix + "site[" + std::to_string(siteId) + "]";
        CompareVector3(dtModel.site_pos[static_cast<std::size_t>(siteId)], mjModel->site_pos + 3 * siteId, 1.0e-12, context + ".pos");
        CompareVector4(dtModel.site_quat[static_cast<std::size_t>(siteId)], mjModel->site_quat + 4 * siteId, 1.0e-12, context + ".quat");
    }
}

void CompareDenseMatrix(const dtDynDenseMatrix &actual,
                        const dtArray<mjtNum> &expected,
                        const int rows,
                        const int cols,
                        const double tolerance,
                        const std::string &context)
{
    ExpectEqual(actual.GetRowSize(), rows, context + ".rows");
    ExpectEqual(actual.GetColSize(), cols, context + ".cols");

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            ExpectNearTol(actual(row, col),
                          static_cast<double>(expected[static_cast<std::size_t>(row * cols + col)]),
                          tolerance,
                          context + "[" + std::to_string(row) + "," + std::to_string(col) + "]");
        }
    }
}

void CompareSpatialArray(const dtArray<dt::Math::Vector6<dtDynReal>> &actual,
                         const mjtNum *expected,
                         const int count,
                         const double tolerance,
                         const std::string &context)
{
    for (int id = 0; id < count; ++id)
    {
        for (int row = 0; row < 6; ++row)
        {
            ExpectNearTol(actual[static_cast<std::size_t>(id)](row),
                          static_cast<double>(expected[6 * id + row]),
                          tolerance,
                          context + "[" + std::to_string(id) + "," + std::to_string(row) + "]");
        }
    }
}

void CompareKinematics(const mjModel *mjModel, mjData *mjData, const dtDynModel &dtModel)
{
    mj_forward(mjModel, mjData);

    dtDynData dtData;
    dtData.Resize(dtModel);
    CopyQpos(mjModel, mjData, dtData);
    ForwardKinematics(dtModel, dtData);

    for (int bodyId = 0; bodyId < mjModel->nbody; ++bodyId)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            const std::string context = "body[" + std::to_string(bodyId) + "].axis[" + std::to_string(axis) + "]";
            ExpectNear(dtData.xpos[static_cast<std::size_t>(bodyId)](axis),
                       mjData->xpos[3 * bodyId + axis],
                       context + ".xpos");
            ExpectNear(dtData.xipos[static_cast<std::size_t>(bodyId)](axis),
                       mjData->xipos[3 * bodyId + axis],
                       context + ".xipos");
        }
        CompareVector4(dtData.xquat[static_cast<std::size_t>(bodyId)],
                       mjData->xquat + 4 * bodyId,
                       1.0e-10,
                       "body[" + std::to_string(bodyId) + "].xquat");
        CompareMatrix3(dtData.xmat[static_cast<std::size_t>(bodyId)],
                       mjData->xmat + 9 * bodyId,
                       1.0e-10,
                       "body[" + std::to_string(bodyId) + "].xmat");
        CompareMatrix3(dtData.ximat[static_cast<std::size_t>(bodyId)],
                       mjData->ximat + 9 * bodyId,
                       1.0e-10,
                       "body[" + std::to_string(bodyId) + "].ximat");
    }

    for (int siteId = 0; siteId < mjModel->nsite; ++siteId)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            const std::string context = "site[" + std::to_string(siteId) + "].axis[" + std::to_string(axis) + "]";
            ExpectNear(dtData.site_xpos[static_cast<std::size_t>(siteId)](axis),
                       mjData->site_xpos[3 * siteId + axis],
                       context + ".xpos");
        }
        CompareMatrix3(dtData.site_xmat[static_cast<std::size_t>(siteId)],
                       mjData->site_xmat + 9 * siteId,
                       1.0e-10,
                       "site[" + std::to_string(siteId) + "].xmat");
    }

    for (int jointId = 0; jointId < mjModel->njnt; ++jointId)
    {
        CompareVector3(dtData.xanchor[static_cast<std::size_t>(jointId)],
                       mjData->xanchor + 3 * jointId,
                       1.0e-10,
                       "joint[" + std::to_string(jointId) + "].xanchor");
        CompareVector3(dtData.xaxis[static_cast<std::size_t>(jointId)],
                       mjData->xaxis + 3 * jointId,
                       1.0e-10,
                       "joint[" + std::to_string(jointId) + "].xaxis");
    }
}

void CompareMassMatrix(const mjModel *mjModel, const mjData *mjData, const dtDynData &dtData)
{
    dtArray<mjtNum> mjMass = MakeMjtBuffer(static_cast<std::size_t>(mjModel->nv * mjModel->nv));
    mj_fullM(mjModel, mjMass.data(), mjData->qM);
    CompareDenseMatrix(dtData.Mmat, mjMass, mjModel->nv, mjModel->nv, 1.0e-8, "Mmat");
}

void CompareSiteJacobians(const mjModel *mjModel,
                          const mjData *mjData,
                          const dtDynModel &dtModel,
                          const dtDynData &dtData)
{
    dtArray<mjtNum> mjJacp = MakeMjtBuffer(static_cast<std::size_t>(3 * mjModel->nv));
    dtArray<mjtNum> mjJacr = MakeMjtBuffer(static_cast<std::size_t>(3 * mjModel->nv));
    dtArray<mjtNum> mjJacpDot = MakeMjtBuffer(static_cast<std::size_t>(3 * mjModel->nv));
    dtArray<mjtNum> mjJacrDot = MakeMjtBuffer(static_cast<std::size_t>(3 * mjModel->nv));

    for (int siteId = 0; siteId < mjModel->nsite; ++siteId)
    {
        FillZero(mjJacp);
        FillZero(mjJacr);
        mj_jacSite(mjModel, mjData, mjJacp.data(), mjJacr.data(), siteId);

        dtDynDenseMatrix jacp;
        dtDynDenseMatrix jacr;
        ComputeSiteJacobian(dtModel, dtData, siteId, jacp, jacr);

        const std::string siteContext = "site[" + std::to_string(siteId) + "]";
        CompareDenseMatrix(jacp, mjJacp, 3, mjModel->nv, 1.0e-8, siteContext + ".jacp");
        CompareDenseMatrix(jacr, mjJacr, 3, mjModel->nv, 1.0e-8, siteContext + ".jacr");

        FillZero(mjJacpDot);
        FillZero(mjJacrDot);
        const int bodyId = mjModel->site_bodyid[siteId];
        mj_jacDot(mjModel,
                  mjData,
                  mjJacpDot.data(),
                  mjJacrDot.data(),
                  mjData->site_xpos + 3 * siteId,
                  bodyId);

        dtDynDenseMatrix jacpDot;
        dtDynDenseMatrix jacrDot;
        ComputeSiteJacobianDot(dtModel, dtData, siteId, jacpDot, jacrDot);
        CompareDenseMatrix(jacpDot, mjJacpDot, 3, mjModel->nv, 1.0e-7, siteContext + ".jacpDot");
        CompareDenseMatrix(jacrDot, mjJacrDot, 3, mjModel->nv, 1.0e-7, siteContext + ".jacrDot");

        dt::Math::Vector3<dtDynReal> linear;
        dt::Math::Vector3<dtDynReal> angular;
        ComputeSiteJacobianDotTimesVelocity(dtModel, dtData, siteId, linear, angular);
        for (int row = 0; row < 3; ++row)
        {
            double expectedLinear = 0.0;
            double expectedAngular = 0.0;
            for (int col = 0; col < mjModel->nv; ++col)
            {
                expectedLinear += mjJacpDot[static_cast<std::size_t>(row * mjModel->nv + col)] * mjData->qvel[col];
                expectedAngular += mjJacrDot[static_cast<std::size_t>(row * mjModel->nv + col)] * mjData->qvel[col];
            }
            ExpectNearTol(linear(row), expectedLinear, 1.0e-7, siteContext + ".jacpDotQvel[" + std::to_string(row) + "]");
            ExpectNearTol(angular(row), expectedAngular, 1.0e-7, siteContext + ".jacrDotQvel[" + std::to_string(row) + "]");
        }
    }
}

void CompareCentroidalQuantities(const mjModel *mjModel,
                                 mjData *mjData,
                                 const dtDynModel &dtModel,
                                 const dtDynData &dtData)
{
    for (int bodyId = 0; bodyId < mjModel->nbody; ++bodyId)
    {
        for (int row = 0; row < 3; ++row)
        {
            ExpectNearTol(dtData.subtree_com[static_cast<std::size_t>(bodyId)](row),
                          mjData->subtree_com[3 * bodyId + row],
                          1.0e-9,
                          "subtree_com[" + std::to_string(bodyId) + "," + std::to_string(row) + "]");
        }
    }

    dtArray<mjtNum> mjComJac = MakeMjtBuffer(static_cast<std::size_t>(3 * mjModel->nv));
    dtArray<mjtNum> mjAngMomMat = MakeMjtBuffer(static_cast<std::size_t>(3 * mjModel->nv));

    for (dtDynIndex rootIdx = 0; rootIdx < dtModel.nroot; ++rootIdx)
    {
        const dtDynIndex rootBody = dtModel.root_bodyid[static_cast<std::size_t>(rootIdx)];
        const dtDynIndex comRow = utils::RootComRow(rootIdx);
        const dtDynIndex linearRow = utils::RootCentroidalLinearRow(rootIdx);
        const dtDynIndex angularRow = utils::RootCentroidalAngularRow(rootIdx);
        const dtDynReal rootMass = dtModel.root_mass(rootIdx);

        FillZero(mjComJac);
        FillZero(mjAngMomMat);
        mj_jacSubtreeCom(mjModel, mjData, mjComJac.data(), rootBody);
        mj_angmomMat(mjModel, mjData, mjAngMomMat.data(), rootBody);

        for (int row = 0; row < 3; ++row)
        {
            ExpectNearTol(dtData.comVel(comRow + row),
                          mjData->subtree_linvel[3 * rootBody + row],
                          1.0e-8,
                          "root[" + std::to_string(rootIdx) + "].comVel[" + std::to_string(row) + "]");
            ExpectNearTol(dtData.centMom(linearRow + row),
                          rootMass * mjData->subtree_linvel[3 * rootBody + row],
                          1.0e-8,
                          "root[" + std::to_string(rootIdx) + "].linearMomentum[" + std::to_string(row) + "]");
            ExpectNearTol(dtData.centMom(angularRow + row),
                          mjData->subtree_angmom[3 * rootBody + row],
                          1.0e-8,
                          "root[" + std::to_string(rootIdx) + "].angularMomentum[" + std::to_string(row) + "]");

            for (int col = 0; col < mjModel->nv; ++col)
            {
                const std::size_t index = static_cast<std::size_t>(row * mjModel->nv + col);
                ExpectNearTol(dtData.comJmat(comRow + row, col),
                              mjComJac[index],
                              1.0e-8,
                              "root[" + std::to_string(rootIdx) + "].comJmat[" + std::to_string(row) + "," + std::to_string(col) + "]");
                ExpectNearTol(dtData.cmAmat(linearRow + row, col),
                              rootMass * mjComJac[index],
                              1.0e-8,
                              "root[" + std::to_string(rootIdx) + "].cmAmatLinear[" + std::to_string(row) + "," + std::to_string(col) + "]");
                ExpectNearTol(dtData.cmAmat(angularRow + row, col),
                              mjAngMomMat[index],
                              1.0e-8,
                              "root[" + std::to_string(rootIdx) + "].cmAmatAngular[" + std::to_string(row) + "," + std::to_string(col) + "]");
            }
        }
    }
}

void CompareBiasForces(const mjModel *mjModel, const mjData *mjData, const dtDynData &dtData)
{
    MjDataPtr zeroVelocityData(mj_makeData(mjModel));
    Expect(zeroVelocityData != nullptr, "MuJoCo zero-velocity data allocation failed");

    for (int qposId = 0; qposId < mjModel->nq; ++qposId)
    {
        zeroVelocityData->qpos[qposId] = mjData->qpos[qposId];
    }
    for (int dofId = 0; dofId < mjModel->nv; ++dofId)
    {
        zeroVelocityData->qvel[dofId] = 0.0;
    }
    mj_forward(mjModel, zeroVelocityData.get());

    for (int dofId = 0; dofId < mjModel->nv; ++dofId)
    {
        const double expectedGravity = zeroVelocityData->qfrc_bias[dofId];
        const double expectedCoriolis = mjData->qfrc_bias[dofId] - expectedGravity;
        const std::string context = "bias[" + std::to_string(dofId) + "]";

        ExpectNearTol(dtData.gravityTorq(dofId), expectedGravity, 1.0e-8, context + ".gravityTorq");
        ExpectNearTol(dtData.coriolisTorq(dofId), expectedCoriolis, 1.0e-8, context + ".coriolisTorq");
        ExpectNearTol(dtData.nonlinTorq(dofId), mjData->qfrc_bias[dofId], 1.0e-8, context + ".nonlinTorq");
        ExpectNearTol(dtData.nonlinTorq(dofId),
                      dtData.gravityTorq(dofId) + dtData.coriolisTorq(dofId),
                      1.0e-10,
                      context + ".nonlinConsistency");
    }
}

void CompareNativeDynamicsConsistency(const dtDynModel &dtModel, const dtDynData &dtData)
{
    for (int row = 0; row < dtModel.nv; ++row)
    {
        for (int col = 0; col < dtModel.nv; ++col)
        {
            const std::string context = "native[" + std::to_string(row) + "," + std::to_string(col) + "]";
            ExpectNearTol(dtData.Mmat(row, col),
                          dtData.Mmat(col, row),
                          1.0e-10,
                          context + ".MmatSymmetry");
            ExpectNearTol(dtData.MmatDot(row, col),
                          dtData.MmatDot(col, row),
                          1.0e-10,
                          context + ".MmatDotSymmetry");
        }

        double coriolis = 0.0;
        for (int col = 0; col < dtModel.nv; ++col)
        {
            coriolis += dtData.Cmat(row, col) * dtData.qvel(col);
        }

        const std::string context = "native[" + std::to_string(row) + "]";
        ExpectNearTol(dtData.coriolisTorq(row), coriolis, 1.0e-8, context + ".CmatQvel");
        ExpectNearTol(dtData.nonlinTorq(row),
                      dtData.gravityTorq(row) + dtData.coriolisTorq(row),
                      1.0e-10,
                      context + ".nonlinTorq");
    }

    for (dtDynIndex rootIdx = 0; rootIdx < dtModel.nroot; ++rootIdx)
    {
        const dtDynIndex comRow = utils::RootComRow(rootIdx);
        const dtDynIndex linearRow = utils::RootCentroidalLinearRow(rootIdx);
        const dtDynIndex angularRow = utils::RootCentroidalAngularRow(rootIdx);
        const dtDynReal rootMass = dtModel.root_mass(rootIdx);

        for (int axis = 0; axis < 3; ++axis)
        {
            double comVel = 0.0;
            double linearMomentum = 0.0;
            double angularMomentum = 0.0;

            for (int col = 0; col < dtModel.nv; ++col)
            {
                const double qvel = dtData.qvel(col);
                comVel += dtData.comJmat(comRow + axis, col) * qvel;
                linearMomentum += dtData.cmAmat(linearRow + axis, col) * qvel;
                angularMomentum += dtData.cmAmat(angularRow + axis, col) * qvel;
            }

            const std::string context = "root[" + std::to_string(rootIdx) + "," + std::to_string(axis) + "]";
            ExpectNearTol(dtData.comVel(comRow + axis), comVel, 1.0e-8, context + ".comJmatQvel");
            ExpectNearTol(dtData.centMom(linearRow + axis),
                          linearMomentum,
                          1.0e-8,
                          context + ".cmAmatLinearQvel");
            ExpectNearTol(dtData.centMom(angularRow + axis),
                          angularMomentum,
                          1.0e-8,
                          context + ".cmAmatAngularQvel");

            if (rootMass > 0.0)
            {
                for (int col = 0; col < dtModel.nv; ++col)
                {
                    ExpectNearTol(dtData.comJmat(comRow + axis, col),
                                  dtData.cmAmat(linearRow + axis, col) / rootMass,
                                  1.0e-10,
                                  context + ".comJmatLinearMomentumMap");
                    ExpectNearTol(dtData.comJmatDot(comRow + axis, col),
                                  dtData.cmAmatDot(linearRow + axis, col) / rootMass,
                                  1.0e-10,
                                  context + ".comJmatDotLinearMomentumMap");
                }
            }
        }
    }
}

void CompareDynamicsQuantities(const mjModel *mjModel, mjData *mjData, const dtDynModel &dtModel)
{
    mj_forward(mjModel, mjData);
    mj_subtreeVel(mjModel, mjData);

    dtDynData dtData;
    dtData.Resize(dtModel);
    CopyQpos(mjModel, mjData, dtData);
    CopyQvel(mjModel, mjData, dtData);

    dt::Math::Vector3<dtDynReal> gravity;
    for (int row = 0; row < 3; ++row)
    {
        gravity(row) = static_cast<dtDynReal>(mjModel->opt.gravity[row]);
    }
    ComputeDynamicsCCRBA(dtModel, dtData, gravity, DTDYN_CCRBA_TERM_ALL);
    ComputeJacobians(dtModel, dtData);

    CompareMassMatrix(mjModel, mjData, dtData);
    CompareBiasForces(mjModel, mjData, dtData);
    CompareNativeDynamicsConsistency(dtModel, dtData);
    CompareSpatialArray(dtData.cdof, mjData->cdof, mjModel->nv, 1.0e-8, "cdof");
    CompareSpatialArray(dtData.cdof_dot, mjData->cdof_dot, mjModel->nv, 1.0e-7, "cdof_dot");
    CompareSpatialArray(dtData.cvel, mjData->cvel, mjModel->nbody, 1.0e-8, "cvel");
    CompareSiteJacobians(mjModel, mjData, dtModel, dtData);
    CompareCentroidalQuantities(mjModel, mjData, dtModel, dtData);
}

void ValidateFixture(const Fixture &fixture)
{
    MjModelPtr mjModel = LoadMjModel(fixture.name, fixture.xml);
    MjDataPtr mjData(mj_makeData(mjModel.get()));
    Expect(mjData != nullptr, std::string(fixture.name) + ": MuJoCo data allocation failed");
    SetReferenceState(mjModel.get(), mjData.get());

    const dtDynSpec spec = dtDynSpec::DecodeXml(fixture.xml);
    const dtDynModel dtModel = CompileSpec(spec);

    CompareIndexLayout(fixture, mjModel.get(), dtModel);
    CompareKinematicModelParameters(fixture, mjModel.get(), dtModel);
    CompareKinematics(mjModel.get(), mjData.get(), dtModel);
    CompareDynamicsQuantities(mjModel.get(), mjData.get(), dtModel);
}

dtArray<Fixture> Fixtures()
{
    return {
        {"fixed_body",
         "<mujoco model=\"fixed_body\"><worldbody>"
         "<body name=\"base\" pos=\"0 0 0\">"
         "<body name=\"fixed_sensor_mount\" pos=\"0.1 0.2 0.3\">"
         "<site name=\"imu_site\" pos=\"0.01 0.02 0.03\"/>"
         "<body name=\"hinge_link\" pos=\"0 0 0.2\">"
         "<inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"0.01 0.02 0.03\"/>"
         "<joint name=\"hinge_z\" type=\"hinge\" axis=\"0 0 1\"/>"
         "<site name=\"tool\" pos=\"0 0 0.1\"/>"
         "</body></body></body>"
         "</worldbody></mujoco>"},
        {"rotated_massless_frames",
         "<mujoco model=\"rotated_massless_frames\"><worldbody>"
         "<body name=\"base_frame\" pos=\"0.1 -0.1 0.2\" quat=\"0.923879532511 0 0 0.382683432365\">"
         "<body name=\"sensor_frame\" pos=\"0.05 0.02 0.03\" quat=\"0.965925826289 0.258819045103 0 0\">"
         "<site name=\"sensor_site\" pos=\"0.01 0.02 0.03\" quat=\"0.707106781187 0 0.707106781187 0\"/>"
         "<body name=\"link\" pos=\"0.03 0 0.2\" quat=\"0.965925826289 0 0.258819045103 0\">"
         "<inertial pos=\"0 0 0.04\" mass=\"0.8\" diaginertia=\"0.01 0.012 0.02\"/>"
         "<joint name=\"hinge_y\" type=\"hinge\" axis=\"0 1 0\"/>"
         "<site name=\"tool\" pos=\"0 0 0.08\" quat=\"0.923879532511 0.382683432365 0 0\"/>"
         "</body></body></body>"
         "</worldbody></mujoco>"},
        {"single_arm",
         "<mujoco model=\"single_arm\"><worldbody>"
         "<body name=\"base\" pos=\"0 0 0\">"
         "<body name=\"link1\" pos=\"0 0 0.2\">"
         "<inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"0.01 0.01 0.01\"/>"
         "<joint name=\"shoulder\" type=\"hinge\" axis=\"0 0 1\" limited=\"true\" range=\"-1.5 1.5\" armature=\"0.1\"/>"
         "<body name=\"link2\" pos=\"0.3 0 0\">"
         "<inertial pos=\"0.15 0 0\" mass=\"0.5\" diaginertia=\"0.01 0.02 0.03\"/>"
         "<joint name=\"elbow\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-0.8 2.2\" armature=\"0.2\"/>"
         "<site name=\"tool\" pos=\"0.3 0 0\"/>"
         "</body></body></body>"
         "</worldbody>"
         "<actuator><motor name=\"elbow_motor\" joint=\"elbow\" gear=\"120\"/></actuator>"
         "</mujoco>"},
        {"dual_arm",
         "<mujoco model=\"dual_arm\"><worldbody>"
         "<body name=\"base\" pos=\"0 0 0\">"
         "<body name=\"left_link\" pos=\"0 0.2 0\">"
         "<inertial pos=\"0 0 0\" mass=\"0.5\" diaginertia=\"0.01 0.01 0.02\"/>"
         "<joint name=\"left_joint\" type=\"hinge\" axis=\"0 0 1\"/>"
         "<site name=\"left_tool\" pos=\"0.2 0 0\"/>"
         "</body>"
        "<body name=\"right_link\" pos=\"0 -0.2 0\">"
        "<inertial pos=\"0 0 0\" mass=\"0.5\" diaginertia=\"0.01 0.01 0.02\"/>"
        "<joint name=\"right_joint\" type=\"hinge\" axis=\"0 0 1\"/>"
        "<site name=\"right_tool\" pos=\"0.2 0 0\"/>"
        "</body></body>"
        "</worldbody></mujoco>"},
        {"multi_robot",
         "<mujoco model=\"multi_robot\"><compiler angle=\"radian\"/><worldbody>"
         "<body name=\"robot_a_base\" pos=\"0.4 0.25 0\">"
         "<inertial pos=\"0 0 0\" mass=\"2\" diaginertia=\"0.04 0.05 0.06\"/>"
         "<body name=\"robot_a_shoulder\" pos=\"0.1 0 0.2\">"
         "<inertial pos=\"0.05 0 0.02\" mass=\"0.8\" diaginertia=\"0.01 0.012 0.014\"/>"
         "<joint name=\"robot_a_shoulder_joint\" type=\"hinge\" axis=\"0 0 1\" limited=\"true\" range=\"-1.2 1.2\" armature=\"0.03\"/>"
         "<body name=\"robot_a_elbow\" pos=\"0.25 0 0\">"
         "<inertial pos=\"0.12 0 0\" mass=\"0.6\" diaginertia=\"0.008 0.01 0.012\"/>"
         "<joint name=\"robot_a_elbow_joint\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-0.7 1.8\" armature=\"0.04\"/>"
         "<site name=\"robot_a_tool\" pos=\"0.2 0 0\"/>"
         "</body></body></body>"
         "<body name=\"robot_b_base\" pos=\"-0.35 -0.2 0.05\">"
         "<inertial pos=\"0 0 0\" mass=\"1.5\" diaginertia=\"0.03 0.035 0.04\"/>"
         "<body name=\"robot_b_sensor_frame\" pos=\"0.02 0.03 0.04\">"
         "<site name=\"robot_b_sensor\" pos=\"0.01 0.02 0.03\"/>"
         "<body name=\"robot_b_hip\" pos=\"0 0 0.18\">"
         "<inertial pos=\"0 0 0.08\" mass=\"0.7\" diaginertia=\"0.009 0.011 0.013\"/>"
         "<joint name=\"robot_b_hip_joint\" type=\"hinge\" axis=\"1 0 0\" limited=\"true\" range=\"-0.9 0.9\" armature=\"0.05\"/>"
         "<body name=\"robot_b_knee\" pos=\"0.22 0 0\">"
         "<inertial pos=\"0.1 0 0\" mass=\"0.45\" diaginertia=\"0.006 0.008 0.01\"/>"
         "<joint name=\"robot_b_knee_joint\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-1.4 0.5\" armature=\"0.06\"/>"
         "<site name=\"robot_b_tool\" pos=\"0.18 0 0\"/>"
         "</body></body></body></body>"
         "</worldbody>"
         "<actuator>"
         "<motor name=\"robot_a_elbow_motor\" joint=\"robot_a_elbow_joint\" gear=\"90\"/>"
        "<motor name=\"robot_b_knee_motor\" joint=\"robot_b_knee_joint\" gear=\"70\"/>"
        "</actuator>"
        "</mujoco>"},
        {"multi_robot_floating",
         "<mujoco model=\"multi_robot_floating\"><compiler angle=\"radian\"/><worldbody>"
         "<body name=\"robot_a_floating_base\" pos=\"0.4 0.25 0.6\">"
         "<freejoint name=\"robot_a_free\"/>"
         "<inertial pos=\"0 0 0\" mass=\"3\" diaginertia=\"0.08 0.09 0.10\"/>"
         "<body name=\"robot_a_floating_shoulder\" pos=\"0.15 0 -0.10\">"
         "<inertial pos=\"0.075 0 -0.05\" mass=\"0.8\" diaginertia=\"0.009 0.0106 0.0122\"/>"
         "<joint name=\"robot_a_floating_shoulder_joint\" type=\"hinge\" axis=\"0 0 1\" limited=\"true\" range=\"-1.1 1.1\" armature=\"0.03\"/>"
         "<body name=\"robot_a_floating_elbow\" pos=\"0.25 0 0\">"
         "<inertial pos=\"0.125 0 0\" mass=\"0.6\" diaginertia=\"0.007 0.0082 0.0094\"/>"
         "<joint name=\"robot_a_floating_elbow_joint\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-0.8 1.7\" armature=\"0.04\"/>"
         "<site name=\"robot_a_floating_tool\" pos=\"0.2 0 0\"/>"
         "</body></body></body>"
         "<body name=\"robot_b_floating_base\" pos=\"-0.35 -0.2 0.7\">"
         "<freejoint name=\"robot_b_free\"/>"
         "<inertial pos=\"0 0 0\" mass=\"2.5\" diaginertia=\"0.07 0.08 0.09\"/>"
         "<body name=\"robot_b_floating_sensor_frame\" pos=\"0.02 0.03 0.04\">"
         "<site name=\"robot_b_floating_sensor\" pos=\"0.01 0.02 0.03\"/>"
         "<body name=\"robot_b_floating_hip\" pos=\"0 0 -0.18\">"
         "<inertial pos=\"0 0 -0.09\" mass=\"0.7\" diaginertia=\"0.008 0.0094 0.0108\"/>"
         "<joint name=\"robot_b_floating_hip_joint\" type=\"hinge\" axis=\"1 0 0\" limited=\"true\" range=\"-0.9 0.9\" armature=\"0.05\"/>"
         "<body name=\"robot_b_floating_knee\" pos=\"0.22 0 0\">"
         "<inertial pos=\"0.11 0 0\" mass=\"0.45\" diaginertia=\"0.0055 0.0064 0.0073\"/>"
         "<joint name=\"robot_b_floating_knee_joint\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-1.4 0.5\" armature=\"0.06\"/>"
         "<site name=\"robot_b_floating_tool\" pos=\"0.18 0 0\"/>"
         "</body></body></body></body>"
         "</worldbody>"
         "<actuator>"
         "<motor name=\"robot_a_floating_elbow_motor\" joint=\"robot_a_floating_elbow_joint\" gear=\"85\"/>"
         "<motor name=\"robot_b_floating_knee_motor\" joint=\"robot_b_floating_knee_joint\" gear=\"65\"/>"
         "</actuator>"
         "</mujoco>"},
        {"floating_quadruped",
         "<mujoco model=\"floating_quadruped\"><worldbody>"
         "<body name=\"trunk\" pos=\"0 0 0.45\">"
         "<freejoint name=\"floating_base\"/>"
         "<inertial pos=\"0 0 0\" mass=\"10\" diaginertia=\"0.2 0.3 0.4\"/>"
         "<body name=\"fl_thigh\" pos=\"0.2 0.15 -0.1\">"
         "<inertial pos=\"0 0 -0.1\" mass=\"1\" diaginertia=\"0.02 0.02 0.01\"/>"
         "<joint name=\"fl_hip\" type=\"hinge\" axis=\"1 0 0\"/>"
         "<body name=\"fl_shank\" pos=\"0 0 -0.25\">"
         "<inertial pos=\"0 0 -0.1\" mass=\"0.5\" diaginertia=\"0.01 0.01 0.005\"/>"
         "<joint name=\"fl_knee\" type=\"hinge\" axis=\"0 1 0\"/>"
         "<site name=\"fl_foot\" pos=\"0 0 -0.25\"/>"
         "</body></body>"
         "</body></worldbody></mujoco>"}};
}

} // namespace

int main()
{
    try
    {
        const char *version = mj_versionString();
        Expect(version != nullptr, "MuJoCo version string is null");
        Expect(std::string(version).find("3.8.0") != std::string::npos,
               std::string("test_dtDynamicsMujocoRef requires MuJoCo 3.8.0, found ") + version);

        for (const Fixture &fixture : Fixtures())
        {
            ValidateFixture(fixture);
        }
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
