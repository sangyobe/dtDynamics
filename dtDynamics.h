/*!
 \file      dtDynamics.h
 \brief     Base class for the rigid body dynamics engine
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2023. 10. 25
 \version   1.2.1
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_H_
#define ALGORITHM_DTDYNAMICS_H_

#include "algorithm/robotData.h"
#include "algorithm/robotModel.h"

namespace dtDynamics
{

template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, typename m_type = SYSREAL>
class RobotEngineVariables;

/*! \brief Base class for the rigid body dynamics engine.
    \details
    This Class includes robotics engine functions.\n
    COORDINATE_TYPE == 0 : qVel = ( baseLinVel, baseAngVel_B, qdot ) \n
    COORDINATE_TYPE == 1 : qVel = ( baseLinVel_B, baseAngVel_B, qdot ) \n
    COORDINATE_TYPE == 2 : qVel = ( baseLinVel, baseAngVel, qdot )  \n
    \n
    For (COORDINATE_TYPE == 0) and (COORDINATE_TYPE == 2), return variables such as jacobian, centroidal matrix, and etc are expressed in inertial frame.
    For (COORDINATE_TYPE == 1), return variables such as jacobian, centroidal matrix, and etc are expressed in base frame.
*/
template <SYSINDEX m_dofTotal, SYSINDEX m_nob, SYSINDEX m_noj, SYSINDEX COORDINATE_TYPE = 0, typename m_type = SYSREAL>
class RobotEngineBase
{
public:
    enum CoordinateType
    {
        COORDINATE_TYPE_A = static_cast<SYSINDEX>(0),
        COORDINATE_TYPE_B = static_cast<SYSINDEX>(1),
        COORDINATE_TYPE_C = static_cast<SYSINDEX>(2)
    };

    enum JointType
    {
        FreeJoint = 0,  //	= 0 : mjJNT_FREE (MUST BE !!)
        BallJoint,      //	= 1 : mjJNT_BALL (MUST BE !!)
        PrismaticJoint, //	= 2 : mjJNT_SLIDE (MUST BE !!)
        RevoluteJoint,  //	= 3 : mjJNT_HINGE (MUST BE !!)
        UndefinedJoint  //	= 4 : User defined !!
    };

    enum DofType
    {
        DOF3 = static_cast<SYSINDEX>(3),
        DOF4 = static_cast<SYSINDEX>(4),
        DOF5 = static_cast<SYSINDEX>(5),
        DOF6 = static_cast<SYSINDEX>(6),
        DOF7 = static_cast<SYSINDEX>(7),
        DOF9 = static_cast<SYSINDEX>(9),
        DOF16 = static_cast<SYSINDEX>(16)
    };

private:
    SYSINDEX i, j, k, l, u;

    RobotEngineVariables<m_dofTotal, m_nob, m_noj, m_type> reVar;

public:
    ~RobotEngineBase() {}

    void InitModel(RobotModel &model, RobotData &data, const char *modelFile);
    void InitTaskVariables(RobotModel &model, RobotData &data, const SYSINDEX &taskIndex, const char *taskName);
    void InitVariables(const RobotModel &model);

    void ComputeMotionCore(const RobotModel &model, RobotData &data);
    void ComputeDiffMotionCore(const RobotModel &model, RobotData &data);
    void ComputeCompositeInertia(const RobotModel &model, RobotData &data);
    void ComputeCentMomentumDot(const RobotModel &model, RobotData &data);
    void ComputeInertiaDot(const RobotModel &model, RobotData &data);

    void ComputeKinematics(const RobotModel &model, RobotData &data);
    void ComputeDynamics(const RobotModel &model, RobotData &data);

    void GetBase2BodyPose(RobotData &data, const int &ID_body, const dt::Math::Vector3<m_type> &offsetPos, const dt::Math::Rotation<m_type> &offsetRot, dt::Math::Vector3<m_type> &pos_body, dt::Math::Rotation<m_type> &bodyRot);
    void GetBodyJacob(const RobotModel &model, RobotData &data, int bodyID, const dt::Math::Vector3<m_type> &pos, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJac, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &angJac);
    void GetBodyLinJacob(const RobotModel &model, RobotData &data, int bodyID, const dt::Math::Vector3<m_type> &pos, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJac);

    void GetBodyJacobDeriv(const RobotModel &model, RobotData &data, int bodyID, const dt::Math::Vector3<m_type> &rpos_A, const dt::Math::Vector3<m_type> &rvel_A, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJacDot, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &angJacDot);
    void GetBodyLinJacobDeriv(const RobotModel &model, RobotData &data, int bodyID, const dt::Math::Vector3<m_type> &rpos_A, const dt::Math::Vector3<m_type> &rvel_A, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJacDot);

    void GetBase2SubtreeCom(const RobotModel &model, RobotData &data, int bodyID, dt::Math::Vector3<m_type> &comPos);
    void GetComVel(const RobotModel &model, RobotData &data, dt::Math::Vector3<m_type> &comVel);

    // transformation functions for floating-based robot.
    void GetInertia_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<m_dofTotal, m_dofTotal, m_type> &inertia);
    void GetCoriolis_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<m_dofTotal, m_dofTotal, m_type> &coriolis, dt::Math::Vector<m_dofTotal, m_type> &coriolisTorque);

    void GetBodyJacob_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJac, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &angJac);
    void GetBodyLinJacob_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJac);

    void GetBodyJacobDeriv_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJacDot, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &angJacDot);
    void GetBodyLinJacobDeriv_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &linJacDot);

    void GetCMMat_I(const RobotModel &model, const RobotData &data, dt::Math::Matrix<DOF6, m_dofTotal, m_type> &cmAmat, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &comJmat, dt::Math::Vector<m_dofTotal, m_type> &gravityTorq);
    void GetCMMatDeriv_I(const RobotModel &model, RobotData &data, dt::Math::Matrix<DOF6, m_dofTotal, m_type> &cmAmatDot, dt::Math::Matrix<DOF3, m_dofTotal, m_type> &comJmatDot);
};

} // namespace dtDynamics

#include "dtDynamics.tpp"

#endif // ALGORITHM_DTDYNAMICS_H_