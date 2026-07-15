/*!
 \file      dtDynamics.cpp
 \brief     Base class for the rigid body dynamics engine
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2023. 10. 25
 \version   1.2.1
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_CPP_
#define ALGORITHM_DTDYNAMICS_CPP_

#include "algorithm/dtDynamicsLegacy/dtDynamics.h"

#include "algorithm/dtDynamicsLegacy/dtDynamicsUtil.h"

#include <dtMath/dtMath.h>

#include <mujoco/mujoco.h>

namespace dtDynamics
{

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, typename m_type>
class RobotEngineVariables
{
public:
    // Temporal vectors & matrices
    dt::Math::Vector3<m_type> zeroVec3;
    dt::Math::Vector3<m_type> tempVec3, tempVec3_2;
    dt::Math::Vector4<m_type> tempVec4;
    dt::Math::Vector6<m_type> tempVec6;

    dt::Math::Matrix3<m_type> tempMat33;
    dt::Math::Matrix<DOF3, m_dofTotal, m_type> tempMat3Q;

    dt::Math::Matrix<DOF3, m_noj, m_type> tempMat3N;
    dt::Math::Matrix<m_noj, DOF3, m_type> tempMat3N_T;

    dt::Math::Matrix<3, m_dofTotal - 3, m_type> tempTmat3xQm3;
    dt::Math::Matrix<m_dofTotal - 3, 3, m_type> tempTmat3xQm3_T;

    dt::Math::Matrix<DOF3, DOF3, m_type> tempMatrix3x3;

    dt::Math::Matrix3<m_type> E3; // 3x3 identity matrix

    int bodyAddr;

    typedef struct _rigidBodyStates
    {
        dt::Math::Vector3<m_type> bodyPos;     // Position vector of i-th body/link frame rel. to p(i) frame
        dt::Math::Rotation<m_type> bodyRot;    // Rotation matrix i-th body/link frame rel. to p(i) frame
        dt::Math::Vector3<m_type> bodyComPos;  // CoM position vector i-th link/body frame rel. to p(i) frame
        dt::Math::Rotation<m_type> bodyComRot; // CoM Rotation(principal axes) i-th body/link frame rel. to p(i) frame
    } RigidBodyStates;

    RigidBodyStates rigidBodyStates[m_nob]; // w.r.t parent frame.

    dt::Math::Vector3<m_type> jntAxis[m_nob]; // joint axis in {B}

    dt::Math::Matrix3<m_type> lnkComImat[m_nob]; // CoM Inertia matrix of i-th link w.r.t. {B}

    dt::Math::Vector3<m_type> posLnk2LnkComSubtree[m_nob]; // from link to subtree CoM position of i-th link w.r.t {B} expressed in {B}

    dt::Math::Vector3<m_type> posBase2LnkComSubtree[m_nob]; // CoM position of i-th link w.r.t {B} expressed in {B}
    dt::Math::Vector3<m_type> posLnkCom2ComSubtree[m_nob];  // from link CoM to subtree CoM position of i-th link w.r.t {B} expressed in {B}

    dt::Math::Vector6<m_type> spVec[m_dofTotal];
    dt::Math::Vector6<m_type> spVecDot[m_dofTotal];
    dt::Math::Vector6<m_type> spVecDotMat[DOF6];
    dt::Math::Vector6<m_type> spVel[m_nob];

    dt::Math::Matrix<DOF6, DOF6, m_type> comImat[m_nob]; // CoM Inertia matrix of i-th link w.r.t. {G}
    dt::Math::Matrix<DOF6, DOF6, m_type> comImatComp[m_nob];

    dt::Math::Matrix<DOF6, DOF6, m_type> Bmat[m_nob];     // body factorization term
    dt::Math::Matrix<DOF6, DOF6, m_type> BmatComp[m_nob]; // body factorization term

    dt::Math::Vector6<m_type> f1[m_dofTotal];
    dt::Math::Vector6<m_type> f2[m_dofTotal];
    dt::Math::Vector6<m_type> f3[m_dofTotal];

    // mujoco model
    mjModel *m_modelMUJOCO = nullptr;

    // mujoco data
    mjData *m_dataMUJOCO = nullptr;

public:
    ~RobotEngineVariables()
    {
        if (m_dataMUJOCO)
        {
            mj_deleteData(m_dataMUJOCO);
        }
        if (m_modelMUJOCO)
        {
            mj_deleteModel(m_modelMUJOCO);
        }
    }
};

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::InitModel(RobotModel &model, RobotData &data, const char *modelFile)
{
    // Load the model
    char error[1000];
    reVar.m_modelMUJOCO = mj_loadXML(modelFile, nullptr, error, 1000);

    if (!reVar.m_modelMUJOCO)
    {
        std::cerr << "Error loading MUJOCO model: " << error << std::endl;
        throw std::runtime_error("Failed to load MUJOCO model");
    }

    // Create data structure for the MUJOCO model
    reVar.m_dataMUJOCO = mj_makeData(reVar.m_modelMUJOCO);

    //+ interface mujoco model and data to dtDynamics model and data +//

    SYSINDEX bodyId[m_nob];       //	Id of the body including fixed bodies
    SYSINDEX bodyParentId[m_nob]; //	Id of the parents body

    model.numPos = m_dofTotal + 1;
    model.numVel = m_dofTotal;
    model.numCtrl = m_nob;
    model.numBody = m_nob;
    model.numJnt = m_nob;

    SYSINDEX startBodyId; //	Start Body Id : floating base body id
    startBodyId = mj_name2id(reVar.m_modelMUJOCO, mjOBJ_BODY, robotBodyNames[0].c_str());

    for (SYSINDEX i = 0; i < m_nob; i++)
    {
        bodyId[i] = mj_name2id(reVar.m_modelMUJOCO, mjOBJ_BODY, robotBodyNames[i].c_str());
    }

    for (SYSINDEX i = 0; i < m_nob; i++)
    {
        bodyParentId[i] = reVar.m_modelMUJOCO->body_parentid[bodyId[i]];

        model.bodyId[i] = bodyId[i] - startBodyId;
        model.bodyRootId[i] = reVar.m_modelMUJOCO->body_rootid[bodyId[i]] - startBodyId;
        model.bodyParentId[i] = bodyParentId[i] - startBodyId;

        model.bodyJntNum[i] = reVar.m_modelMUJOCO->body_jntnum[bodyId[i]];
        model.bodyJntAdr[i] = reVar.m_modelMUJOCO->body_jntadr[bodyId[i]];
    }

    SYSINDEX bodyDofAdr = 0;
    for (SYSINDEX i = 0; i < model.numBody; i++)
    {
        model.bodyDofNum[i] = static_cast<SYSINDEX>(reVar.m_modelMUJOCO->body_dofnum[bodyId[i]]);
        model.bodyDofAdr[i] = static_cast<SYSINDEX>(reVar.m_modelMUJOCO->body_dofadr[bodyId[i]]);
    }

    for (SYSINDEX i = 0; i < model.numJnt; i++)
    {
        model.jntBodyId[i] = reVar.m_modelMUJOCO->jnt_bodyid[i] - 1;
        model.jntqPosAdr[i] = reVar.m_modelMUJOCO->jnt_qposadr[i];
        model.jntDofAdr[i] = reVar.m_modelMUJOCO->jnt_dofadr[i];

        //	Joint type : free joint = 0, prismatic joint(slide) = 2, revolute joint(hinge) = 3
        model.rigidBody[i].jntType = (reVar.m_modelMUJOCO->jnt_type[i]);
        if (model.rigidBody[i].jntType == FreeJoint)
        {
            model.rigidBody[i].jntAxis << 1.0, 1.0, 1.0; // floating base.
        }
        else
        {
            std::copy(&(reVar.m_modelMUJOCO->jnt_axis[i * DOF3]), &(reVar.m_modelMUJOCO->jnt_axis[i * DOF3]) + DOF3, &(model.rigidBody[i].jntAxis(0)));

            if (abs(abs(model.rigidBody[i].jntAxis(X_AXIS)) - 1.0) < 1e-10)
            {
                model.rigidBody[i].jntDir = static_cast<SYSINDEX>(X_AXIS);
            }
            else if (abs(abs(model.rigidBody[i].jntAxis(Y_AXIS)) - 1.0) < 1e-10)
            {
                model.rigidBody[i].jntDir = static_cast<SYSINDEX>(Y_AXIS);
            }
            else if (abs(abs(model.rigidBody[i].jntAxis(Z_AXIS)) - 1.0) < 1e-10)
            {
                model.rigidBody[i].jntDir = static_cast<SYSINDEX>(Z_AXIS);
            }
        }
    }

    for (SYSINDEX i = 0; i < m_noj; i++)
    {
        // Set Limit for Joint Range of Motion
        if (reVar.m_modelMUJOCO->jnt_limited[(i + 1)] == 1)
        {
            model.jntLimitFlag[i] = 1;
            model.jntLimitMin[i] = reVar.m_modelMUJOCO->jnt_range[(i + 1) * 2];
            model.jntLimitMax[i] = reVar.m_modelMUJOCO->jnt_range[(i + 1) * 2 + 1];

            model.jntGearRatio[i] = reVar.m_modelMUJOCO->actuator_gear[i * DOF6];
        }
        else
        {
            model.jntLimitFlag[i] = 0;
            model.jntLimitMin[i] = 0.0;
            model.jntLimitMax[i] = 0.0;

            model.jntGearRatio[i] = 1.0;
        }
    }

    for (SYSINDEX i = 0; i < model.numVel; i++)
    {
        model.dofBodyId[i] = reVar.m_modelMUJOCO->dof_bodyid[i] - 1;
        model.dofBodyParentId[i] = reVar.m_modelMUJOCO->dof_parentid[i];
        model.dofJointId[i] = reVar.m_modelMUJOCO->dof_jntid[i];
    }

    m_type tempMat33[DOF9];
    m_type tempArr4[DOF4];

    for (SYSINDEX i = 0; i < model.numBody; ++i)
    {
        if (model.rigidBody[i].jntType != FreeJoint)
        {
            std::copy(&(reVar.m_modelMUJOCO->body_pos[bodyId[i] * DOF3]), &(reVar.m_modelMUJOCO->body_pos[bodyId[i] * DOF3]) + DOF3, &(model.rigidBody[i].bodyPos0(0)));

            std::copy(&(reVar.m_modelMUJOCO->body_quat[bodyId[i] * DOF4]), &(reVar.m_modelMUJOCO->body_quat[bodyId[i] * DOF4]) + DOF4, &(tempArr4[0]));
            Quat2Rot(tempArr4, tempMat33);
            std::copy(&(tempMat33[0]), &(tempMat33[DOF9]), &(model.rigidBody[i].bodyRot0(0, 0)));
        }
        else
        {
            model.rigidBody[i].bodyPos0.SetZero();
            model.rigidBody[i].bodyRot0.SetIdentity();

            // std::copy(&(reVar.m_modelMUJOCO->body_pos[bodyId[i] * DOF3]), &(reVar.m_modelMUJOCO->body_pos[bodyId[i] * DOF3]) + DOF3, &(data.basePos(0)));
            std::copy(&(reVar.m_modelMUJOCO->body_pos[bodyId[i] * DOF3]), &(reVar.m_modelMUJOCO->body_pos[bodyId[i] * DOF3]) + DOF3, &(data.actControlState.basePosG_G(0)));
        }

        std::copy(&(reVar.m_modelMUJOCO->body_ipos[bodyId[i] * DOF3]), &(reVar.m_modelMUJOCO->body_ipos[bodyId[i] * DOF3]) + DOF3, &(model.rigidBody[i].bodyComPos0(0)));

        std::copy(&(reVar.m_modelMUJOCO->body_iquat[bodyId[i] * DOF4]), &(reVar.m_modelMUJOCO->body_iquat[bodyId[i] * DOF4]) + DOF4, &(tempArr4[0]));
        Quat2Rot(tempArr4, tempMat33);
        std::copy(&(tempMat33[0]), &(tempMat33[DOF9]), &(model.rigidBody[i].bodyComRot0(0, 0)));

        model.rigidBody[i].comImatDiag(0, 0) = (SYSREAL)reVar.m_modelMUJOCO->body_inertia[bodyId[i] * 3 + 0];
        model.rigidBody[i].comImatDiag(1, 1) = (SYSREAL)reVar.m_modelMUJOCO->body_inertia[bodyId[i] * 3 + 1];
        model.rigidBody[i].comImatDiag(2, 2) = (SYSREAL)reVar.m_modelMUJOCO->body_inertia[bodyId[i] * 3 + 2];
        model.rigidBody[i].comImat0 = model.rigidBody[i].bodyComRot0 * model.rigidBody[i].comImatDiag * model.rigidBody[i].bodyComRot0.Transpose(); // CoM Inertia! (CoM frame // Body frame)

        reVar.rigidBodyStates[i].bodyPos = model.rigidBody[i].bodyPos0;
        reVar.rigidBodyStates[i].bodyRot = model.rigidBody[i].bodyRot0;
        reVar.rigidBodyStates[i].bodyComRot = model.rigidBody[i].bodyRot0 * model.rigidBody[i].bodyComRot0;
        reVar.rigidBodyStates[i].bodyComPos = model.rigidBody[i].bodyPos0 + model.rigidBody[i].bodyRot0 * model.rigidBody[i].bodyComPos0;

        model.rigidBody[i].bodyMass = static_cast<SYSREAL>(reVar.m_modelMUJOCO->body_mass[bodyId[i]]);
        model.bodyMassSubtree[i] = model.rigidBody[i].bodyMass;
    }

    for (int i = model.numBody - 1; i >= 0; i--)
    {
        // add to parent, except for world
        if (i)
        {
            int j = model.bodyParentId[i];
            model.bodyMassSubtree[j] += model.bodyMassSubtree[i];
        }
    }

    //+ joint info +//
    // joint armature
    std::copy(&(reVar.m_modelMUJOCO->dof_armature[0]), &(reVar.m_modelMUJOCO->dof_armature[0]) + model.numVel, &(model.dofArmature[0]));

    if (abs(model.rigidBody[0].jntType - FreeJoint) < 1e-10)
    {
        model.isBaseFloating = true;
    }

    InitVariables(model);

    model.isInitialized = true;

    // variable init in data.
    data.baseRot.SetIdentity();
    data.baseRot_T.SetIdentity();
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::InitTaskVariables(RobotModel &model, RobotData &data, const SYSINDEX &taskIndex, const char *taskName)
{
    if (model.isInitialized)
    {
        SYSINDEX startBodyId; //	Start Body Id : floating base body id
        SYSINDEX siteId = 0;

        startBodyId = mj_name2id(reVar.m_modelMUJOCO, mjOBJ_BODY, robotBodyNames[0].c_str());
        siteId = mj_name2id(reVar.m_modelMUJOCO, mjOBJ_SITE, taskName);

        data.actTaskStates[taskIndex].id = reVar.m_modelMUJOCO->site_bodyid[mj_name2id(reVar.m_modelMUJOCO, mjOBJ_SITE, taskName)] - startBodyId;

        model.rigidBody[data.actTaskStates[taskIndex].id].taskOffset << reVar.m_modelMUJOCO->site_pos[siteId * DOF3], reVar.m_modelMUJOCO->site_pos[siteId * DOF3 + 1], reVar.m_modelMUJOCO->site_pos[siteId * DOF3 + 2];

        SYSREAL tempMat33[DOF9];
        SYSREAL tempArr4[DOF4];
        std::copy(&(reVar.m_modelMUJOCO->site_quat[siteId * DOF4]), &(reVar.m_modelMUJOCO->site_quat[siteId * DOF4]) + DOF4, &(tempArr4[0]));
        Quat2Rot(tempArr4, tempMat33);
        std::copy(&(tempMat33[0]), &(tempMat33[DOF9]), &(model.rigidBody[data.actTaskStates[taskIndex].id].taskRot(0, 0)));
    }

    for (SYSINDEX i = 0; i < model.numBody; i++)
    {
        data.rotBase2Lnk[i].SetIdentity();
        data.rotBase2Lnk_T[i].SetIdentity();
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::InitVariables(const RobotModel &model)
{
    i = 0, j = 0, k = 0, l = 0, u = 0;
    reVar.E3.SetIdentity();

    reVar.zeroVec3.SetZero();
    reVar.tempMat3Q.SetZero();

    for (SYSINDEX i = 0; i < model.numBody; i++)
    {
        reVar.comImat[i].SetIdentity();
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::ComputeMotionCore(const RobotModel &model, RobotData &data)
{
    for (i = 0; i < model.numBody; i++)
    {
        if (model.bodyJntNum[i] == 1 && model.rigidBody[model.bodyJntAdr[i]].jntType == FreeJoint)
        {
            if ((COORDINATE_TYPE == COORDINATE_TYPE_B))
            {
                data.rotBase2Lnk[i] = reVar.rigidBodyStates[i].bodyRot;
                data.posBase2Lnk[i] = reVar.rigidBodyStates[i].bodyPos;
                data.posBase2LnkCom[i] = reVar.rigidBodyStates[i].bodyComPos;
            }
            else
            {
                data.rotBase2Lnk[i] = data.baseRot * reVar.rigidBodyStates[i].bodyRot;
                data.posBase2Lnk[i] = data.baseRot * reVar.rigidBodyStates[i].bodyPos;
                data.posBase2LnkCom[i] = data.posBase2Lnk[i] + data.rotBase2Lnk[i] * reVar.rigidBodyStates[i].bodyComPos;
            }
        }
        else
        {
            rbElemetaryTransform(model, data, i, reVar.rigidBodyStates[i].bodyPos, reVar.rigidBodyStates[i].bodyRot, reVar.rigidBodyStates[i].bodyComPos);

            if (model.bodyId[i] != model.bodyRootId[i])
            {

                data.rotBase2Lnk[i] = data.rotBase2Lnk[model.bodyParentId[i]] * reVar.rigidBodyStates[i].bodyRot;
                data.posBase2Lnk[i] = data.posBase2Lnk[model.bodyParentId[i]] + data.rotBase2Lnk[model.bodyParentId[i]] * reVar.rigidBodyStates[i].bodyPos;
                data.posBase2LnkCom[i] = data.posBase2Lnk[model.bodyParentId[i]] + data.rotBase2Lnk[model.bodyParentId[i]] * reVar.rigidBodyStates[i].bodyComPos;
            }
            else
            {
                data.rotBase2Lnk[i] = reVar.rigidBodyStates[i].bodyRot;
                data.posBase2Lnk[i] = reVar.rigidBodyStates[i].bodyPos;
                data.posBase2LnkCom[i] = reVar.rigidBodyStates[i].bodyComPos;
            }
        }

        data.rotBase2Lnk_T[i] = data.rotBase2Lnk[i].Transpose();

        if ((model.rigidBody[i].jntType == RevoluteJoint) || (model.rigidBody[i].jntType == PrismaticJoint))
        {
            reVar.jntAxis[i] = data.rotBase2Lnk[i] * model.rigidBody[i].jntAxis;
        }
        else
        {
            reVar.jntAxis[i].SetZero();
        }
    }

    for (int i = model.numBody - 1; i >= 0; i--)
    {
        reVar.posBase2LnkComSubtree[i] = model.rigidBody[i].bodyMass * data.posBase2LnkCom[i];
    }

    for (int i = model.numBody - 1; i >= 0; i--)
    {
        // add to parent, except for world
        if (i)
        {
            int j = model.bodyParentId[i];
            reVar.posBase2LnkComSubtree[j] += reVar.posBase2LnkComSubtree[i];
        }

        // compute local com
        if (model.bodyMassSubtree[i] < 1e-15)
        {
            reVar.posBase2LnkComSubtree[i] = data.posBase2LnkCom[i];
        }
        else
        {
            reVar.posBase2LnkComSubtree[i] = reVar.posBase2LnkComSubtree[i] / model.bodyMassSubtree[i];
        }
    }

    for (i = 0; i < model.numBody; i++)
    {
        reVar.posLnk2LnkComSubtree[i] = reVar.posBase2LnkComSubtree[0] - data.posBase2Lnk[i];
        reVar.posLnkCom2ComSubtree[i] = data.posBase2LnkCom[i] - reVar.posBase2LnkComSubtree[0];

        reVar.lnkComImat[i] = data.rotBase2Lnk[i] * model.rigidBody[i].comImat0 * data.rotBase2Lnk_T[i];

        switch (model.rigidBody[i].jntType)
        {
        case FreeJoint:
            // translation components: x, y, z in global frame
            for (int j = 0; j < DOF3; j++)
            {
                reVar.spVec[model.jntDofAdr[i] + j].SetBlock(DOF3, reVar.E3.GetColVec(j));
            }

            if ((COORDINATE_TYPE == COORDINATE_TYPE_A))
            {
                for (int j = 0; j < DOF3; j++)
                {
                    reVar.tempVec3 = data.rotBase2Lnk[model.jntBodyId[i]].GetColVec(j);
                    reVar.spVec[model.jntDofAdr[i] + j + DOF3].SetBlock(0, reVar.tempVec3);
                    reVar.spVec[model.jntDofAdr[i] + j + DOF3].SetBlock(DOF3, (reVar.tempVec3 & reVar.posLnk2LnkComSubtree[i]));
                }
            }
            else
            {
                for (int j = 0; j < DOF3; j++)
                {
                    reVar.tempVec3 = reVar.E3.GetColVec(j);
                    reVar.spVec[model.jntDofAdr[i] + j + DOF3].SetBlock(0, reVar.tempVec3);
                    reVar.spVec[model.jntDofAdr[i] + j + DOF3].SetBlock(DOF3, (reVar.tempVec3 & reVar.posLnk2LnkComSubtree[i]));
                }
            }
            break;

        case BallJoint:
            for (int j = 0; j < DOF3; j++)
            {
                // I_3 rotation in child frame (assume no subsequent rotations)
                reVar.tempVec3 = data.rotBase2Lnk[model.jntBodyId[i]].GetColVec(j);
                reVar.spVec[model.jntDofAdr[i] + j + DOF3].SetBlock(0, reVar.tempVec3);
                reVar.spVec[model.jntDofAdr[i] + j + DOF3].SetBlock(DOF3, (reVar.tempVec3 & reVar.posLnk2LnkComSubtree[i]));
            }

            break;

        case PrismaticJoint:
            reVar.spVec[model.jntDofAdr[i]].SetBlock(DOF3, reVar.jntAxis[i]);

            break;

        case RevoluteJoint:
            reVar.spVec[model.jntDofAdr[i]].SetBlock(0, reVar.jntAxis[i]);
            reVar.spVec[model.jntDofAdr[i]].SetBlock(DOF3, (reVar.jntAxis[i] & reVar.posLnk2LnkComSubtree[i]));

            break;
        }
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::ComputeDiffMotionCore(const RobotModel &model, RobotData &data)
{
    for (i = 0; i < model.numBody; i++)
    {
        if (model.bodyId[i] != model.bodyRootId[i])
        {
            std::copy(&(reVar.spVel[model.bodyParentId[i]](0)), &(reVar.spVel[model.bodyParentId[i]](0)) + DOF6, &(reVar.spVel[i](0)));
        }
        else
        {
            reVar.spVel[i].SetZero();
        }

        // spVel = spVel_parent + spVec * qvel,  spVecDot = spVel x spVec
        for (j = 0; j < model.bodyDofNum[i]; j++)
        {
            // compute spVel and spVecDot
            switch (model.rigidBody[model.dofJointId[model.bodyDofAdr[i] + j]].jntType)
            {
            case FreeJoint:
                // spVecDot = 0
                for (k = 0; k < DOF3; k++)
                {
                    reVar.spVecDotMat[k].SetZero();
                }

                // update velocity
                reVar.tempVec6 = reVar.spVec[model.bodyDofAdr[i] + 0] * data.qVel(model.bodyDofAdr[i] + 0) + reVar.spVec[model.bodyDofAdr[i] + 1] * data.qVel(model.bodyDofAdr[i] + 1) + reVar.spVec[model.bodyDofAdr[i] + 2] * data.qVel(model.bodyDofAdr[i] + 2);
                reVar.spVel[i] += reVar.tempVec6;

                // continue with rotations
                j += 3;

                [[fallthrough]];

            case BallJoint:
                // compute all 3 cdofdots using parent velocity

                for (k = 0; k < DOF3; k++)
                {
                    reVar.spVecDotMat[j + k] = CrossSpatialMotion(reVar.spVel[i], reVar.spVec[model.bodyDofAdr[i] + j + k]);
                }

                // update velocity
                reVar.tempVec6 = reVar.spVec[model.bodyDofAdr[i] + j + 0] * data.qVel(model.bodyDofAdr[i] + j + 0) + reVar.spVec[model.bodyDofAdr[i] + j + 1] * data.qVel(model.bodyDofAdr[i] + j + 1) + reVar.spVec[model.bodyDofAdr[i] + j + 2] * data.qVel(model.bodyDofAdr[i] + j + 2);
                reVar.spVel[i] += reVar.tempVec6;

                // adjust for 3-dof joint
                j += 2;

                break;

            default:
                // in principle we should use the new velocity to compute spVecDot,
                // but it makes no difference becase crossMotion(spVec, spVec) = 0,
                // and using the old velocity may be more accurate numerically
                reVar.spVecDotMat[j] = CrossSpatialMotion(reVar.spVel[i], reVar.spVec[model.bodyDofAdr[i] + j]);

                // update velocity
                reVar.tempVec6 = reVar.spVec[model.bodyDofAdr[i] + j] * data.qVel(model.bodyDofAdr[i] + j);
                reVar.spVel[i] += reVar.tempVec6;

                break;
            }
        }

        // assign spVel, spVecDot
        for (j = 0; j < model.bodyDofNum[i]; j++)
        {
            reVar.spVecDot[model.bodyDofAdr[i] + j] = reVar.spVecDotMat[j];
        }
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::ComputeKinematics(const RobotModel &model, RobotData &data)
{
    ComputeMotionCore(model, data);
    ComputeDiffMotionCore(model, data);
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::ComputeCompositeInertia(const RobotModel &model, RobotData &data)
{
    int i = 0;
    int j = 0;

    for (i = 0; i < model.numBody; i++)
    {
        reVar.comImat[i] = InertiaCalc(reVar.lnkComImat[i], reVar.posLnkCom2ComSubtree[i], model.rigidBody[i].bodyMass); // inerita at com-based frame
        reVar.comImatComp[i] = reVar.comImat[i];

        reVar.Bmat[i] = static_cast<SYSREAL>(0.5) * (CrossSpatialMotionMatrixStar(reVar.spVel[i]) * reVar.comImat[i] + CrossSpatialForceMatrix(reVar.comImat[i] * reVar.spVel[i]) - reVar.comImat[i] * CrossSpatialMotionMatrix(reVar.spVel[i]));
        reVar.BmatComp[i] = reVar.Bmat[i];
    }

    for (i = model.numBody - 1; i > 0; i--)
    {
        j = model.bodyParentId[i];
        reVar.comImatComp[j] += reVar.comImatComp[i];
        reVar.BmatComp[j] += reVar.BmatComp[i];
    }

    for (i = model.numVel - 1; i >= 0; i--)
    {
        reVar.f1[i] = reVar.comImatComp[model.dofBodyId[i]] * reVar.spVecDot[i] + reVar.BmatComp[model.dofBodyId[i]] * reVar.spVec[i];
        reVar.f2[i] = reVar.comImatComp[model.dofBodyId[i]] * reVar.spVec[i];
        reVar.f3[i] = reVar.BmatComp[model.dofBodyId[i]].Transpose() * reVar.spVec[i];
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::ComputeDynamics(const RobotModel &model, RobotData &data)
{
    int i = 0;
    int j = 0;

    ComputeCompositeInertia(model, data);

    for (i = model.numVel - 1; i >= 0; i--)
    {
        for (k = 0; k < DOF3; k++)
        {
            data.cmAmat(k, i) = reVar.f2[i](DOF3 + k);
            data.cmAmat(DOF3 + k, i) = reVar.f2[i](k);
        }

        data.Mmat(i, i) = reVar.spVec[i].Inner(reVar.f2[i]);
        data.Cmat(i, i) = reVar.spVec[i].Inner(reVar.f1[i]);

        j = i;

        while (j != 0)
        {
            j = model.dofBodyParentId[j];

            data.Cmat(j, i) = reVar.spVec[j].Inner(reVar.f1[i]);
            data.Cmat(i, j) = (reVar.spVecDot[j].Inner(reVar.f2[i]) + reVar.spVec[j].Inner(reVar.f3[i]));
            data.Mmat(i, j) = reVar.spVec[j].Inner(reVar.f2[i]);
            data.Mmat(j, i) = data.Mmat(i, j);
        }

        data.Mmat(i, i) += model.dofArmature[i];
    }

    data.coriolisTorq = data.Cmat * data.qVel;
    data.cmAmat.GetBlock(0, 0, data.comJmat);
    data.comJmat = data.comJmat / model.bodyMassSubtree[0];

    dt::Math::Vector<18, SYSREAL> tmpGravityTorq; // adh
    data.cmAmat.GetRowVec(2, 0, tmpGravityTorq);
    data.gravityTorq = GRAVITY * tmpGravityTorq;
    data.nonlinTorq = data.coriolisTorq + data.gravityTorq;
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::ComputeCentMomentumDot(const RobotModel &model, RobotData &data)
{
    int i = 0;
    int j = 0;

    for (i = model.numVel - 1; i >= 0; i--)
    {
        for (k = 0; k < DOF3; k++)
        {
            data.cmAmatDot(k, i) = reVar.f1[i](DOF3 + k);
            data.cmAmatDot(DOF3 + k, i) = reVar.f1[i](k);
        }
    }
    data.comJmatDot.GetBlock(0, 0, data.comJmatDot);
    data.comJmatDot = data.comJmatDot / model.bodyMassSubtree[0];
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::ComputeInertiaDot(const RobotModel &model, RobotData &data)
{
    int i = 0;
    int j = 0;

    for (i = model.numVel - 1; i >= 0; i--)
    {
        data.MmatDot(i, i) = reVar.spVecDot[i].Inner(reVar.f2[i]) + reVar.spVec[i].Inner(reVar.f1[i] + reVar.f3[i]);

        j = i;

        while (j != 0)
        {
            j = model.dofBodyParentId[j];

            data.MmatDot(i, j) = reVar.spVecDot[j].Inner(reVar.f2[i]) + reVar.spVec[j].Inner(reVar.f1[i] + reVar.f3[i]);
            data.MmatDot(j, i) = data.MmatDot(i, j);
        }
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBase2BodyPose(RobotData &data, const int &ID_body, const dt::Math::Vector3<m_type> &offsetPos, const dt::Math::Rotation<m_type> &offsetRot, dt::Math::Vector3<m_type> &pos_body, dt::Math::Rotation<m_type> &bodyRot)
{
    pos_body = data.posBase2Lnk[ID_body] + data.rotBase2Lnk[ID_body] * offsetPos;
    bodyRot = data.rotBase2Lnk[ID_body] * offsetRot;
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBodyJacob(const RobotModel &model, RobotData &data, int bodyID, const dt::Math::Vector3<m_type> &pos, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJac, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &angJac)
{
    reVar.tempVec3 = pos - reVar.posBase2LnkComSubtree[0];

    // skip fixed bodies
    while (bodyID && !model.bodyDofNum[bodyID])
    {
        bodyID = model.bodyParentId[bodyID];
    }

    if (bodyID < 0 || bodyID >= 65535)
    {
        return;
    }

    // get last dof that affects this (as well as the original) body
    reVar.bodyAddr = model.bodyDofAdr[bodyID] + model.bodyDofNum[bodyID] - 1;

    // backward pass over dof ancestor chain
    while (reVar.bodyAddr >= 0 && reVar.bodyAddr < 65535)
    {
        // construct rotation jacobian
        angJac.SetColVec(0, reVar.bodyAddr, reVar.spVec[reVar.bodyAddr].GetPos());

        // construct translation jacobian (correct for rotation)
        reVar.tempVec3_2 = (reVar.spVec[reVar.bodyAddr].GetPos() & reVar.tempVec3);
        linJac.SetColVec(0, reVar.bodyAddr, (reVar.spVec[reVar.bodyAddr].GetOri() + reVar.tempVec3_2));

        // advance to parent dof
        reVar.bodyAddr = model.dofBodyParentId[reVar.bodyAddr];
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBodyLinJacob(const RobotModel &model, RobotData &data, int bodyID, const dt::Math::Vector3<m_type> &pos, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJac)
{
    reVar.tempVec3 = pos - reVar.posBase2LnkComSubtree[0];

    // skip fixed bodies
    while (bodyID && !model.bodyDofNum[bodyID])
    {
        bodyID = model.bodyParentId[bodyID];
    }

    if (bodyID < 0 || bodyID >= 65535)
    {
        return;
    }

    // get last dof that affects this (as well as the original) body
    reVar.bodyAddr = model.bodyDofAdr[bodyID] + model.bodyDofNum[bodyID] - 1;

    // backward pass over dof ancestor chain
    while (reVar.bodyAddr >= 0 && reVar.bodyAddr < 65535)
    {
        // construct translation jacobian (correct for rotation)
        reVar.tempVec3_2 = (reVar.spVec[reVar.bodyAddr].GetPos() & reVar.tempVec3);
        linJac.SetColVec(0, reVar.bodyAddr, (reVar.spVec[reVar.bodyAddr].GetOri() + reVar.tempVec3_2));

        // advance to parent dof
        reVar.bodyAddr = model.dofBodyParentId[reVar.bodyAddr];
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBodyJacobDeriv(const RobotModel &model, RobotData &data, int bodyID, const dt::Math::Vector3<m_type> &rpos_A, const dt::Math::Vector3<m_type> &rvel_A, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJacDot, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &angJacDot)
{
    reVar.tempVec3 = rpos_A - reVar.posBase2LnkComSubtree[0];

    // skip fixed bodies
    while (bodyID && !model.bodyDofNum[bodyID])
    {
        bodyID = model.bodyParentId[bodyID];
    }

    if (bodyID < 0 || bodyID >= 65535)
    {
        return;
    }

    // get last dof that affects this (as well as the original) body
    reVar.bodyAddr = model.bodyDofAdr[bodyID] + model.bodyDofNum[bodyID] - 1;

    // backward pass over dof ancestor chain
    while (reVar.bodyAddr >= 0 && reVar.bodyAddr < 65535)
    {
        // construct rotation jacobian
        angJacDot.SetColVec(0, reVar.bodyAddr, reVar.spVecDot[reVar.bodyAddr].GetPos());

        reVar.tempVec3_2 = reVar.spVecDot[reVar.bodyAddr].GetOri();
        reVar.tempVec3_2 += (reVar.spVecDot[reVar.bodyAddr].GetPos() & reVar.tempVec3);
        reVar.tempVec3_2 += (reVar.spVec[reVar.bodyAddr].GetPos() & rvel_A);
        linJacDot.SetColVec(0, reVar.bodyAddr, reVar.tempVec3_2);

        // advance to parent dof

        reVar.bodyAddr = model.dofBodyParentId[reVar.bodyAddr];
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBodyLinJacobDeriv(const RobotModel &model, RobotData &data, int bodyID, const dt::Math::Vector3<m_type> &rpos_A, const dt::Math::Vector3<m_type> &rvel_A, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJacDot)
{

    reVar.tempVec3 = rpos_A - reVar.posBase2LnkComSubtree[0];

    // skip fixed bodies
    while (bodyID && !model.bodyDofNum[bodyID])
    {
        bodyID = model.bodyParentId[bodyID];
    }

    if (bodyID < 0 || bodyID >= 65535)
    {
        return;
    }

    // get last dof that affects this (as well as the original) body
    reVar.bodyAddr = model.bodyDofAdr[bodyID] + model.bodyDofNum[bodyID] - 1;

    // backward pass over dof ancestor chain
    while (reVar.bodyAddr >= 0 && reVar.bodyAddr < 65535)
    {
        // construct translation jacobian (correct for rotation)
        reVar.tempVec3_2 = (reVar.spVecDot[reVar.bodyAddr].GetPos() & reVar.tempVec3);
        reVar.tempVec3_2 += (reVar.spVec[reVar.bodyAddr].GetPos() & rvel_A);
        linJacDot.SetColVec(reVar.bodyAddr, (reVar.spVecDot[reVar.bodyAddr].GetOri() + reVar.tempVec3_2));

        // advance to parent dof
        reVar.bodyAddr = model.dofBodyParentId[reVar.bodyAddr];
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBase2SubtreeCom(const RobotModel &model, RobotData &data, int bodyID, dt::Math::Vector3<m_type> &comPos)
{
    comPos = reVar.posBase2LnkComSubtree[bodyID];
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetComVel(const RobotModel &model, RobotData &data, dt::Math::Vector3<m_type> &comVel)
{
    data.comVel = data.comJmat * data.qVel;
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetInertia_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<m_dofTotal, m_dofTotal, m_type> &inertia)
{
    if (COORDINATE_TYPE == COORDINATE_TYPE_A) // transform qVel = (pDot, omega_B, qDot) -> (pDot, omega, qDot)
    {
        inertia.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        inertia.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        inertia.GetBlock(DOF3, 0, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3;
        inertia.SetBlock(DOF3, 0, reVar.tempMatrix3x3);

        inertia.GetBlock(DOF3, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        inertia.SetBlock(DOF3, DOF3, reVar.tempMatrix3x3);

        inertia.GetBlock(DOF3, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        inertia.SetBlock(DOF3, DOF6, reVar.tempMat3N);

        inertia.GetBlock(DOF6, DOF3, reVar.tempMat3N_T);
        reVar.tempMat3N_T = reVar.tempMat3N_T * data.baseRot_T;
        inertia.SetBlock(DOF6, DOF3, reVar.tempMat3N_T);
    }
    else if (COORDINATE_TYPE == COORDINATE_TYPE_B) // transform qVel = (pDot_B, omega_B, qDot) -> (pDot, omega, qDot)
    {
        inertia.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        inertia.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        inertia.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        inertia.SetBlock(0, DOF6, reVar.tempMat3N);

        inertia.GetBlock(DOF3, 0, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        inertia.SetBlock(DOF3, 0, reVar.tempMatrix3x3);

        inertia.GetBlock(DOF3, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        inertia.SetBlock(DOF3, DOF3, reVar.tempMatrix3x3);

        inertia.GetBlock(DOF3, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        inertia.SetBlock(DOF3, DOF6, reVar.tempMat3N);

        inertia.GetBlock(DOF6, 0, reVar.tempMat3N_T);
        reVar.tempMat3N_T = reVar.tempMat3N_T * data.baseRot_T;
        inertia.SetBlock(DOF6, 0, reVar.tempMat3N_T);

        inertia.GetBlock(DOF6, DOF3, reVar.tempMat3N_T);
        reVar.tempMat3N_T = reVar.tempMat3N_T * data.baseRot_T;
        inertia.SetBlock(DOF6, DOF3, reVar.tempMat3N_T);
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetCoriolis_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<m_dofTotal, m_dofTotal, m_type> &coriolis, dt::Math::Vector<m_dofTotal, m_type> &coriolisTorque)
{
    if (COORDINATE_TYPE == COORDINATE_TYPE_A) // transform qVel = (pDot, omega_B, qDot) -> (pDot, omega, qDot)
    {
        coriolis.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        coriolis.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        coriolis.GetBlock(DOF3, 0, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3;
        coriolis.SetBlock(DOF3, 0, reVar.tempMatrix3x3);

        coriolis.GetBlock(DOF3, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        coriolis.SetBlock(DOF3, DOF3, reVar.tempMatrix3x3);

        coriolis.GetBlock(DOF3, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        coriolis.SetBlock(DOF3, DOF6, reVar.tempMat3N);

        coriolis.GetBlock(DOF6, DOF3, reVar.tempMat3N_T);
        reVar.tempMat3N_T = reVar.tempMat3N_T * data.baseRot_T;
        coriolis.SetBlock(DOF6, DOF3, reVar.tempMat3N_T);

        coriolisTorque.GetBlockVec3(DOF3, reVar.tempVec3);
        reVar.tempVec3 = data.baseRot * reVar.tempVec3;
        coriolisTorque.SetBlock(DOF3, reVar.tempVec3);
    }
    else if (COORDINATE_TYPE == COORDINATE_TYPE_B) // transform qVel = (pDot_B, omega_B, qDot) -> (pDot, omega, qDot)
    {
        coriolis.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        coriolis.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        coriolis.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        coriolis.SetBlock(0, DOF6, reVar.tempMat3N);

        coriolis.GetBlock(DOF3, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        coriolis.SetBlock(DOF3, DOF3, reVar.tempMatrix3x3);

        coriolis.GetBlock(DOF3, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        coriolis.SetBlock(DOF3, DOF6, reVar.tempMat3N);

        coriolis.GetBlock(DOF6, DOF3, reVar.tempMat3N_T);
        reVar.tempMat3N_T = reVar.tempMat3N_T * data.baseRot_T;
        coriolis.SetBlock(DOF6, DOF3, reVar.tempMat3N_T);

        coriolisTorque.GetBlockVec3(0, reVar.tempVec3);
        reVar.tempVec3 = data.baseRot * reVar.tempVec3;
        coriolisTorque.SetBlock(0, reVar.tempVec3);

        coriolisTorque.GetBlockVec3(DOF3, reVar.tempVec3);
        reVar.tempVec3 = data.baseRot * reVar.tempVec3;
        coriolisTorque.SetBlock(DOF3, reVar.tempVec3);
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBodyJacob_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJac, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &angJac)
{
    if (COORDINATE_TYPE == COORDINATE_TYPE_A) // transform qVel = (pDot, omega_B, qDot) -> (pDot, omega, qDot)
    {
        linJac.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        linJac.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        angJac.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        angJac.SetBlock(0, DOF3, reVar.tempMatrix3x3);
    }
    else if (COORDINATE_TYPE == COORDINATE_TYPE_B) // transform qVel = (pDot_B, omega_B, qDot) -> (pDot, omega, qDot)
    {
        linJac.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        linJac.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        linJac.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        linJac.SetBlock(0, DOF6, reVar.tempMat3N);

        angJac.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        angJac.SetBlock(0, DOF6, reVar.tempMat3N);
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBodyLinJacob_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJac)
{
    if (COORDINATE_TYPE == COORDINATE_TYPE_A) // transform qVel = (pDot, omega_B, qDot) -> (pDot, omega, qDot)
    {
        linJac.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3;
        linJac.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        linJac.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        linJac.SetBlock(0, DOF6, reVar.tempMat3N);
    }
    else if (COORDINATE_TYPE == COORDINATE_TYPE_B) // transform qVel = (pDot_B, omega_B, qDot) -> (pDot, omega, qDot)
    {
        linJac.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        linJac.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        linJac.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        linJac.SetBlock(0, DOF6, reVar.tempMat3N);
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBodyJacobDeriv_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJacDot, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &angJacDot)
{
    if (COORDINATE_TYPE == COORDINATE_TYPE_A) // transform qVel = (pDot, omega_B, qDot) -> (pDot, omega, qDot)
    {
        linJacDot.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        linJacDot.SetBlock(0, DOF3, reVar.tempMatrix3x3);
    }
    if (COORDINATE_TYPE == COORDINATE_TYPE_B) // transform qVel = (pDot_B, omega_B, qDot) -> (pDot, omega, qDot)
    {
        linJacDot.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        linJacDot.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        linJacDot = data.baseRot * linJacDot;
        angJacDot = data.baseRot * angJacDot;
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetBodyLinJacobDeriv_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJacDot)
{
    if (COORDINATE_TYPE == COORDINATE_TYPE_B) // transform qVel = (pDot_B, omega_B, qDot) -> (pDot, omega, qDot)
    {
        linJacDot = data.baseRot * linJacDot;
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetCMMat_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF6, m_dofTotal, m_type> &cmAmat, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &comJmat, dt::Math::Vector<m_dofTotal, m_type> &gravityTorq)
{
    if (COORDINATE_TYPE == COORDINATE_TYPE_A) // transform qVel = (pDot, omega_B, qDot) -> (pDot, omega, qDot)
    {
        cmAmat.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        cmAmat.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        cmAmat.GetBlock(DOF3, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        cmAmat.SetBlock(DOF3, DOF3, reVar.tempMatrix3x3);

        comJmat.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        comJmat.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        gravityTorq = GRAVITY * cmAmat.GetRowVec(Z_AXIS);
    }
    else if (COORDINATE_TYPE == COORDINATE_TYPE_B) // transform qVel = (pDot_B, omega_B, qDot) -> (pDot, omega, qDot)
    {
        cmAmat.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        cmAmat.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        cmAmat.GetBlock(DOF3, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        cmAmat.SetBlock(DOF3, DOF3, reVar.tempMatrix3x3);

        cmAmat.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        cmAmat.SetBlock(0, DOF6, reVar.tempMat3N);

        cmAmat.GetBlock(DOF3, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        cmAmat.SetBlock(DOF3, DOF6, reVar.tempMat3N);

        comJmat.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        comJmat.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        comJmat.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        comJmat.SetBlock(0, DOF6, reVar.tempMat3N);

        gravityTorq = GRAVITY * cmAmat.GetRowVec(Z_AXIS);
    }
}

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE, typename m_type>
inline void RobotEngineBase<m_dofTotal, m_nob, m_noj, COORDINATE_TYPE, m_type>::GetCMMatDeriv_I(const RobotModel &model, RobotData &data, dt::Math::Matrix<DOF6, m_dofTotal, m_type> &cmAmatDot, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &comJmatDot)
{
    if (COORDINATE_TYPE == COORDINATE_TYPE_A) // transform qVel = (pDot, omega_B, qDot) -> (pDot, omega, qDot)
    {
        cmAmatDot.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        cmAmatDot.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        comJmatDot.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = reVar.tempMatrix3x3 * data.baseRot_T;
        comJmatDot.SetBlock(0, DOF3, reVar.tempMatrix3x3);
    }
    else if (COORDINATE_TYPE == COORDINATE_TYPE_B) // transform qVel = (pDot_B, omega_B, qDot) -> (pDot, omega, qDot)
    {
        cmAmatDot.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        cmAmatDot.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        comJmatDot.GetBlock(0, DOF3, reVar.tempMatrix3x3);
        reVar.tempMatrix3x3 = data.baseRot * reVar.tempMatrix3x3 * data.baseRot_T;
        comJmatDot.SetBlock(0, DOF3, reVar.tempMatrix3x3);

        cmAmatDot.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        cmAmatDot.SetBlock(0, DOF6, reVar.tempMat3N);

        cmAmatDot.GetBlock(DOF3, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        cmAmatDot.SetBlock(DOF3, DOF6, reVar.tempMat3N);

        comJmatDot.GetBlock(0, DOF6, reVar.tempMat3N);
        reVar.tempMat3N = data.baseRot * reVar.tempMat3N;
        comJmatDot.SetBlock(0, DOF6, reVar.tempMat3N);
    }
}

} // namespace dtDynamics

#endif // ALGORITHM_DTDYNAMICS_CPP_
