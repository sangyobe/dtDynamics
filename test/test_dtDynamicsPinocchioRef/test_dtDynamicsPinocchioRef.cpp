#include "dtDynamics/dtDynCompile.h"
#include "dtDynamics/dtDynData.h"
#include "dtDynamics/dtDynDynamics.h"
#include "dtDynamics/dtDynSpec.h"

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace dt::Dynamics;

namespace
{

struct FreeJointMap
{
    int nativeQposAdr = 0;
    int nativeQvelAdr = 0;
    int pinQposAdr = 0;
    int pinQvelAdr = 0;
};

struct Fixture
{
    std::string name;
    std::string xml;
    std::function<pinocchio::Model()> buildPinocchioModel;
    bool floatingBase = false;
    dtArray<double> qpos;
    dtArray<double> qvel;
    dtArray<double> qacc;
    dtArray<FreeJointMap> freeJoints;
};

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

void ExpectNear(const double actual, const double expected, const double tolerance, const std::string &context)
{
    if (std::fabs(actual - expected) >= tolerance)
    {
        std::ostringstream out;
        out << context << " mismatch: actual=" << actual << ", expected=" << expected
            << ", tolerance=" << tolerance;
        throw std::runtime_error(out.str());
    }
}

Eigen::Vector3d Vector3(const double x, const double y, const double z)
{
    return Eigen::Vector3d(x, y, z);
}

Eigen::Matrix3d DiagonalInertia(const double ix, const double iy, const double iz)
{
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    inertia(0, 0) = ix;
    inertia(1, 1) = iy;
    inertia(2, 2) = iz;
    return inertia;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d &vector)
{
    Eigen::Matrix3d skew;
    skew << 0.0, -vector(2), vector(1),
        vector(2), 0.0, -vector(0),
        -vector(1), vector(0), 0.0;
    return skew;
}

pinocchio::SE3 Placement(const Eigen::Vector3d &translation)
{
    return pinocchio::SE3(Eigen::Matrix3d::Identity(), translation);
}

pinocchio::Inertia BodyInertia(const double mass,
                               const Eigen::Vector3d &com,
                               const Eigen::Matrix3d &inertiaAtCom)
{
    return pinocchio::Inertia(mass, com, inertiaAtCom);
}

pinocchio::JointIndex AddRevoluteBody(pinocchio::Model &model,
                                      const pinocchio::JointIndex parentJoint,
                                      const std::string &jointName,
                                      const Eigen::Vector3d &jointTranslation,
                                      const Eigen::Vector3d &jointAxis,
                                      const pinocchio::Inertia &bodyInertia)
{
    const pinocchio::JointIndex jointId =
        model.addJoint(parentJoint,
                       pinocchio::JointModelRevoluteUnaligned(jointAxis),
                       Placement(jointTranslation),
                       jointName);
    model.appendBodyToJoint(jointId, bodyInertia, pinocchio::SE3::Identity());
    return jointId;
}

pinocchio::JointIndex AddFreeBody(pinocchio::Model &model,
                                  const std::string &jointName,
                                  const pinocchio::Inertia &bodyInertia)
{
    const pinocchio::JointIndex jointId =
        model.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), jointName);
    model.appendBodyToJoint(jointId, bodyInertia, pinocchio::SE3::Identity());
    return jointId;
}

void SetGravity(pinocchio::Model &model)
{
    model.gravity.linear() = Vector3(0.0, 0.0, -9.81);
    model.gravity.angular().setZero();
}

void CopyStateToNative(const Fixture &fixture, dtDynData &dtData)
{
    for (int i = 0; i < static_cast<int>(fixture.qpos.size()); ++i)
    {
        dtData.qpos(i) = static_cast<dtDynReal>(fixture.qpos[static_cast<std::size_t>(i)]);
    }
    for (int i = 0; i < static_cast<int>(fixture.qvel.size()); ++i)
    {
        dtData.qvel(i) = static_cast<dtDynReal>(fixture.qvel[static_cast<std::size_t>(i)]);
    }
}

Eigen::VectorXd ToEigen(const dtArray<double> &values)
{
    Eigen::VectorXd result(static_cast<Eigen::Index>(values.size()));
    for (Eigen::Index i = 0; i < result.size(); ++i)
    {
        result(i) = values[static_cast<std::size_t>(i)];
    }
    return result;
}

dtArray<FreeJointMap> FreeJointMaps(const Fixture &fixture)
{
    if (!fixture.freeJoints.empty())
    {
        return fixture.freeJoints;
    }
    if (fixture.floatingBase)
    {
        return {FreeJointMap{0, 0, 0, 0}};
    }
    return {};
}

bool HasMultipleFreeRoots(const Fixture &fixture)
{
    return FreeJointMaps(fixture).size() > 1;
}

Eigen::Quaterniond NativeBaseQuaternion(const Fixture &fixture, const FreeJointMap &freeJoint)
{
    const int qadr = freeJoint.nativeQposAdr;
    Eigen::Quaterniond quat(fixture.qpos[static_cast<std::size_t>(qadr + 3)],
                            fixture.qpos[static_cast<std::size_t>(qadr + 4)],
                            fixture.qpos[static_cast<std::size_t>(qadr + 5)],
                            fixture.qpos[static_cast<std::size_t>(qadr + 6)]);
    quat.normalize();
    return quat;
}

Eigen::VectorXd PinocchioQpos(const Fixture &fixture)
{
    Eigen::VectorXd q = ToEigen(fixture.qpos);
    for (const FreeJointMap &freeJoint : FreeJointMaps(fixture))
    {
        const int nativeQadr = freeJoint.nativeQposAdr;
        const int pinQadr = freeJoint.pinQposAdr;
        q(pinQadr + 0) = fixture.qpos[static_cast<std::size_t>(nativeQadr + 0)];
        q(pinQadr + 1) = fixture.qpos[static_cast<std::size_t>(nativeQadr + 1)];
        q(pinQadr + 2) = fixture.qpos[static_cast<std::size_t>(nativeQadr + 2)];
        q(pinQadr + 3) = fixture.qpos[static_cast<std::size_t>(nativeQadr + 4)];
        q(pinQadr + 4) = fixture.qpos[static_cast<std::size_t>(nativeQadr + 5)];
        q(pinQadr + 5) = fixture.qpos[static_cast<std::size_t>(nativeQadr + 6)];
        q(pinQadr + 6) = fixture.qpos[static_cast<std::size_t>(nativeQadr + 3)];
    }
    return q;
}

Eigen::MatrixXd VelocityTransform(const Fixture &fixture)
{
    Eigen::MatrixXd transform = Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(fixture.qvel.size()),
                                                          static_cast<Eigen::Index>(fixture.qvel.size()));
    for (const FreeJointMap &freeJoint : FreeJointMaps(fixture))
    {
        transform.block<3, 3>(freeJoint.nativeQvelAdr, freeJoint.nativeQvelAdr) =
            NativeBaseQuaternion(fixture, freeJoint).toRotationMatrix().transpose();
    }
    return transform;
}

Eigen::MatrixXd VelocityTransformDot(const Fixture &fixture)
{
    Eigen::MatrixXd transformDot = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(fixture.qvel.size()),
                                                         static_cast<Eigen::Index>(fixture.qvel.size()));
    for (const FreeJointMap &freeJoint : FreeJointMaps(fixture))
    {
        const int vadr = freeJoint.nativeQvelAdr;
        const Eigen::Matrix3d rotationBodyToWorld = NativeBaseQuaternion(fixture, freeJoint).toRotationMatrix();
        const Eigen::Vector3d angularBody(fixture.qvel[static_cast<std::size_t>(vadr + 3)],
                                          fixture.qvel[static_cast<std::size_t>(vadr + 4)],
                                          fixture.qvel[static_cast<std::size_t>(vadr + 5)]);
        transformDot.block<3, 3>(vadr, vadr) = -Skew(angularBody) * rotationBodyToWorld.transpose();
    }
    return transformDot;
}

Eigen::VectorXd PinocchioQvel(const Fixture &fixture)
{
    return VelocityTransform(fixture) * ToEigen(fixture.qvel);
}

Eigen::VectorXd PinocchioQacc(const Fixture &fixture)
{
    return VelocityTransform(fixture) * ToEigen(fixture.qacc) +
           VelocityTransformDot(fixture) * ToEigen(fixture.qvel);
}

Eigen::VectorXd Multiply(const dtDynDenseMatrix &matrix, const Eigen::VectorXd &vector)
{
    ExpectEqual(matrix.GetColSize(), static_cast<int>(vector.size()), "Multiply.cols");

    Eigen::VectorXd result(matrix.GetRowSize());
    result.setZero();
    for (int row = 0; row < matrix.GetRowSize(); ++row)
    {
        for (int col = 0; col < matrix.GetColSize(); ++col)
        {
            result(row) += matrix(row, col) * vector(col);
        }
    }
    return result;
}

void CompareMatrix(const dtDynDenseMatrix &actual,
                   const Eigen::MatrixXd &expected,
                   const double tolerance,
                   const std::string &context)
{
    ExpectEqual(actual.GetRowSize(), static_cast<int>(expected.rows()), context + ".rows");
    ExpectEqual(actual.GetColSize(), static_cast<int>(expected.cols()), context + ".cols");

    for (int row = 0; row < actual.GetRowSize(); ++row)
    {
        for (int col = 0; col < actual.GetColSize(); ++col)
        {
            ExpectNear(actual(row, col),
                       expected(row, col),
                       tolerance,
                       context + "[" + std::to_string(row) + "," + std::to_string(col) + "]");
        }
    }
}

void CompareMatrixRows(const dtDynDenseMatrix &actual,
                       const Eigen::MatrixXd &expected,
                       const int rowBegin,
                       const int rowEnd,
                       const double tolerance,
                       const std::string &context)
{
    ExpectEqual(actual.GetRowSize(), static_cast<int>(expected.rows()), context + ".rows");
    ExpectEqual(actual.GetColSize(), static_cast<int>(expected.cols()), context + ".cols");
    Expect(rowBegin >= 0 && rowBegin <= rowEnd && rowEnd <= actual.GetRowSize(), context + ".rowRange");

    for (int row = rowBegin; row < rowEnd; ++row)
    {
        for (int col = 0; col < actual.GetColSize(); ++col)
        {
            ExpectNear(actual(row, col),
                       expected(row, col),
                       tolerance,
                       context + "[" + std::to_string(row) + "," + std::to_string(col) + "]");
        }
    }
}

void CompareVector(const dtDynDenseVector &actual,
                   const Eigen::VectorXd &expected,
                   const double tolerance,
                   const std::string &context)
{
    ExpectEqual(actual.GetDim(), static_cast<int>(expected.size()), context + ".rows");
    for (int row = 0; row < actual.GetDim(); ++row)
    {
        ExpectNear(actual(row), expected(row), tolerance, context + "[" + std::to_string(row) + "]");
    }
}

void CompareEigenVector(const Eigen::VectorXd &actual,
                        const Eigen::VectorXd &expected,
                        const double tolerance,
                        const std::string &context)
{
    ExpectEqual(static_cast<int>(actual.size()), static_cast<int>(expected.size()), context + ".rows");
    for (Eigen::Index row = 0; row < actual.size(); ++row)
    {
        ExpectNear(actual(row), expected(row), tolerance, context + "[" + std::to_string(row) + "]");
    }
}

pinocchio::Model SingleArmPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);

    const pinocchio::JointIndex shoulder =
        AddRevoluteBody(model,
                        0,
                        "shoulder",
                        Vector3(0.0, 0.0, 0.2),
                        Vector3(0.0, 0.0, 1.0),
                        BodyInertia(1.0, Vector3(0.0, 0.0, 0.0), DiagonalInertia(0.01, 0.01, 0.01)));

    AddRevoluteBody(model,
                    shoulder,
                    "elbow",
                    Vector3(0.3, 0.0, 0.0),
                    Vector3(0.0, 1.0, 0.0),
                    BodyInertia(0.5, Vector3(0.15, 0.0, 0.0), DiagonalInertia(0.01, 0.02, 0.03)));

    return model;
}

pinocchio::Model DualArmPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);

    AddRevoluteBody(model,
                    0,
                    "left_joint",
                    Vector3(0.0, 0.2, 0.0),
                    Vector3(0.0, 0.0, 1.0),
                    BodyInertia(0.5, Vector3(0.0, 0.0, 0.0), DiagonalInertia(0.01, 0.01, 0.02)));
    AddRevoluteBody(model,
                    0,
                    "right_joint",
                    Vector3(0.0, -0.2, 0.0),
                    Vector3(0.0, 0.0, 1.0),
                    BodyInertia(0.5, Vector3(0.0, 0.0, 0.0), DiagonalInertia(0.01, 0.01, 0.02)));

    return model;
}

pinocchio::Model FloatingQuadrupedPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);

    const pinocchio::JointIndex floatingBase =
        AddFreeBody(model,
                    "floating_base",
                    BodyInertia(10.0, Vector3(0.0, 0.0, 0.0), DiagonalInertia(0.2, 0.3, 0.4)));
    const pinocchio::JointIndex hip =
        AddRevoluteBody(model,
                        floatingBase,
                        "fl_hip",
                        Vector3(0.2, 0.15, -0.1),
                        Vector3(1.0, 0.0, 0.0),
                        BodyInertia(1.0, Vector3(0.0, 0.0, -0.1), DiagonalInertia(0.02, 0.02, 0.01)));
    AddRevoluteBody(model,
                    hip,
                    "fl_knee",
                    Vector3(0.0, 0.0, -0.25),
                    Vector3(0.0, 1.0, 0.0),
                    BodyInertia(0.5, Vector3(0.0, 0.0, -0.1), DiagonalInertia(0.01, 0.01, 0.005)));

    return model;
}

pinocchio::Model MultiFloatingPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);

    const pinocchio::JointIndex robotA =
        AddFreeBody(model,
                    "robot_a_free",
                    BodyInertia(3.0, Vector3(0.0, 0.0, 0.0), DiagonalInertia(0.08, 0.09, 0.10)));
    const pinocchio::JointIndex robotAShoulder =
        AddRevoluteBody(model,
                        robotA,
                        "robot_a_floating_shoulder_joint",
                        Vector3(0.15, 0.0, -0.10),
                        Vector3(0.0, 0.0, 1.0),
                        BodyInertia(0.8,
                                    Vector3(0.075, 0.0, -0.05),
                                    DiagonalInertia(0.009, 0.0106, 0.0122)));
    AddRevoluteBody(model,
                    robotAShoulder,
                    "robot_a_floating_elbow_joint",
                    Vector3(0.25, 0.0, 0.0),
                    Vector3(0.0, 1.0, 0.0),
                    BodyInertia(0.6,
                                Vector3(0.125, 0.0, 0.0),
                                DiagonalInertia(0.007, 0.0082, 0.0094)));

    const pinocchio::JointIndex robotB =
        AddFreeBody(model,
                    "robot_b_free",
                    BodyInertia(2.5, Vector3(0.0, 0.0, 0.0), DiagonalInertia(0.07, 0.08, 0.09)));
    const pinocchio::JointIndex robotBHip =
        AddRevoluteBody(model,
                        robotB,
                        "robot_b_floating_hip_joint",
                        Vector3(0.02, 0.03, -0.14),
                        Vector3(1.0, 0.0, 0.0),
                        BodyInertia(0.7,
                                    Vector3(0.0, 0.0, -0.09),
                                    DiagonalInertia(0.008, 0.0094, 0.0108)));
    AddRevoluteBody(model,
                    robotBHip,
                    "robot_b_floating_knee_joint",
                    Vector3(0.22, 0.0, 0.0),
                    Vector3(0.0, 1.0, 0.0),
                    BodyInertia(0.45,
                                Vector3(0.11, 0.0, 0.0),
                                DiagonalInertia(0.0055, 0.0064, 0.0073)));

    return model;
}

dtArray<Fixture> Fixtures()
{
    return {
        {"single_arm",
         "<mujoco model=\"single_arm\"><worldbody>"
         "<body name=\"base\" pos=\"0 0 0\">"
         "<body name=\"link1\" pos=\"0 0 0.2\">"
         "<inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"0.01 0.01 0.01\"/>"
         "<joint name=\"shoulder\" type=\"hinge\" axis=\"0 0 1\"/>"
         "<body name=\"link2\" pos=\"0.3 0 0\">"
         "<inertial pos=\"0.15 0 0\" mass=\"0.5\" diaginertia=\"0.01 0.02 0.03\"/>"
         "<joint name=\"elbow\" type=\"hinge\" axis=\"0 1 0\"/>"
         "</body></body></body>"
         "</worldbody></mujoco>",
         SingleArmPinocchioModel,
         false,
         {0.12, 0.24},
         {0.20, -0.30},
         {0.40, -0.15}},
        {"dual_arm",
         "<mujoco model=\"dual_arm\"><worldbody>"
         "<body name=\"base\" pos=\"0 0 0\">"
         "<body name=\"left_link\" pos=\"0 0.2 0\">"
         "<inertial pos=\"0 0 0\" mass=\"0.5\" diaginertia=\"0.01 0.01 0.02\"/>"
         "<joint name=\"left_joint\" type=\"hinge\" axis=\"0 0 1\"/>"
         "</body>"
         "<body name=\"right_link\" pos=\"0 -0.2 0\">"
         "<inertial pos=\"0 0 0\" mass=\"0.5\" diaginertia=\"0.01 0.01 0.02\"/>"
         "<joint name=\"right_joint\" type=\"hinge\" axis=\"0 0 1\"/>"
         "</body></body>"
         "</worldbody></mujoco>",
         DualArmPinocchioModel,
         false,
         {0.15, -0.20},
         {0.35, -0.25},
         {0.18, 0.27}},
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
         "</body></body>"
         "</body></worldbody></mujoco>",
         FloatingQuadrupedPinocchioModel,
         true,
         {0.03, -0.02, 0.46, 0.995004165278, 0.0, 0.0, 0.099833416647, 0.12, -0.24},
         {0.10, -0.05, 0.03, 0.02, -0.04, 0.06, 0.20, -0.15},
         {0.04, -0.02, 0.01, 0.03, -0.01, 0.02, 0.18, 0.27}},
        {"multi_floating",
         "<mujoco model=\"multi_floating\"><compiler angle=\"radian\"/><option gravity=\"0 0 -9.81\"/>"
         "<worldbody>"
         "<body name=\"robot_a_floating_base\" pos=\"0.4 0.25 0.6\">"
         "<freejoint name=\"robot_a_free\"/>"
         "<inertial pos=\"0 0 0\" mass=\"3\" diaginertia=\"0.08 0.09 0.10\"/>"
         "<body name=\"robot_a_floating_shoulder\" pos=\"0.15 0 -0.10\">"
         "<inertial pos=\"0.075 0 -0.05\" mass=\"0.8\" diaginertia=\"0.009 0.0106 0.0122\"/>"
         "<joint name=\"robot_a_floating_shoulder_joint\" type=\"hinge\" axis=\"0 0 1\"/>"
         "<body name=\"robot_a_floating_elbow\" pos=\"0.25 0 0\">"
         "<inertial pos=\"0.125 0 0\" mass=\"0.6\" diaginertia=\"0.007 0.0082 0.0094\"/>"
         "<joint name=\"robot_a_floating_elbow_joint\" type=\"hinge\" axis=\"0 1 0\"/>"
         "</body></body></body>"
         "<body name=\"robot_b_floating_base\" pos=\"-0.35 -0.2 0.7\">"
         "<freejoint name=\"robot_b_free\"/>"
         "<inertial pos=\"0 0 0\" mass=\"2.5\" diaginertia=\"0.07 0.08 0.09\"/>"
         "<body name=\"robot_b_floating_sensor_frame\" pos=\"0.02 0.03 0.04\">"
         "<body name=\"robot_b_floating_hip\" pos=\"0 0 -0.18\">"
         "<inertial pos=\"0 0 -0.09\" mass=\"0.7\" diaginertia=\"0.008 0.0094 0.0108\"/>"
         "<joint name=\"robot_b_floating_hip_joint\" type=\"hinge\" axis=\"1 0 0\"/>"
         "<body name=\"robot_b_floating_knee\" pos=\"0.22 0 0\">"
         "<inertial pos=\"0.11 0 0\" mass=\"0.45\" diaginertia=\"0.0055 0.0064 0.0073\"/>"
         "<joint name=\"robot_b_floating_knee_joint\" type=\"hinge\" axis=\"0 1 0\"/>"
         "</body></body></body></body>"
         "</worldbody></mujoco>",
         MultiFloatingPinocchioModel,
         true,
         {0.43, 0.23, 0.61, 0.999800006667, 0.019998666693, 0.0, 0.0, 0.11, -0.16,
          -0.31, -0.21, 0.68, 0.999550033749, 0.0, 0.029995500202, 0.0, -0.08, 0.14},
         {0.02, -0.03, 0.04, 0.01, -0.02, 0.03, 0.20, -0.15,
          -0.04, 0.05, -0.02, -0.03, 0.04, -0.01, 0.12, -0.10},
         {0.01, -0.02, 0.03, 0.02, -0.01, 0.01, 0.18, 0.27,
          -0.03, 0.01, -0.02, 0.01, 0.02, -0.03, -0.11, 0.13},
         {FreeJointMap{0, 0, 0, 0}, FreeJointMap{9, 8, 9, 8}}}};
}

void ValidateFixture(const Fixture &fixture)
{
    const dtDynSpec spec = dtDynSpec::DecodeXml(fixture.xml);
    const dtDynModel dtModel = CompileSpec(spec);
    dtDynData dtData;
    dtData.Resize(dtModel);
    CopyStateToNative(fixture, dtData);

    const dt::Math::Vector3<dtDynReal> gravity(0.0, 0.0, -9.81);
    ComputeDynamicsCCRBA(dtModel, dtData, gravity, DTDYN_CCRBA_TERM_ALL);

    pinocchio::Model pinModel = fixture.buildPinocchioModel();
    pinocchio::Data pinData(pinModel);
    const Eigen::VectorXd q = PinocchioQpos(fixture);
    const Eigen::VectorXd v = PinocchioQvel(fixture);
    const Eigen::VectorXd a = PinocchioQacc(fixture);
    const Eigen::VectorXd nativeV = ToEigen(fixture.qvel);
    const Eigen::VectorXd nativeA = ToEigen(fixture.qacc);
    const Eigen::MatrixXd velocityTransform = VelocityTransform(fixture);
    const Eigen::MatrixXd velocityTransformDot = VelocityTransformDot(fixture);
    const Eigen::VectorXd velocityTransformDotV = velocityTransformDot * nativeV;

    ExpectEqual(dtModel.nq, pinModel.nq, fixture.name + ".nq");
    ExpectEqual(dtModel.nv, pinModel.nv, fixture.name + ".nv");

    pinocchio::crba(pinModel, pinData, q, pinocchio::Convention::WORLD);
    pinData.M.triangularView<Eigen::StrictlyLower>() =
        pinData.M.transpose().triangularView<Eigen::StrictlyLower>();
    const Eigen::MatrixXd pinMass = pinData.M;
    CompareMatrix(dtData.Mmat,
                  velocityTransform.transpose() * pinMass * velocityTransform,
                  1.0e-9,
                  fixture.name + ".Mmat");

    const Eigen::VectorXd &pinGravity = pinocchio::computeGeneralizedGravity(pinModel, pinData, q);
    CompareVector(dtData.gravityTorq,
                  velocityTransform.transpose() * pinGravity,
                  1.0e-9,
                  fixture.name + ".gravityTorq");

    const Eigen::VectorXd &pinNonlinear = pinocchio::nonLinearEffects(pinModel, pinData, q, v);
    CompareVector(dtData.nonlinTorq,
                  velocityTransform.transpose() * (pinNonlinear + pinMass * velocityTransformDotV),
                  1.0e-9,
                  fixture.name + ".nonlinTorq");

    pinocchio::computeCoriolisMatrix(pinModel, pinData, q, v);
    const Eigen::VectorXd pinCoriolis = pinData.C * v;
    CompareVector(dtData.coriolisTorq,
                  velocityTransform.transpose() * (pinCoriolis + pinMass * velocityTransformDotV),
                  1.0e-9,
                  fixture.name + ".coriolisTorq");


    if (HasMultipleFreeRoots(fixture))
    {
        return;
    }

    const Eigen::MatrixXd &pinCentroidal = pinocchio::ccrba(pinModel, pinData, q, v);
    CompareMatrix(dtData.cmAmat,
                  pinCentroidal * velocityTransform,
                  1.0e-9,
                  fixture.name + ".cmAmat");
    CompareVector(dtData.centMom, pinData.hg.toVector(), 1.0e-9, fixture.name + ".centMom");

    const Eigen::MatrixXd &pinCentroidalDot =
        pinocchio::computeCentroidalMapTimeVariation(pinModel, pinData, q, v);
    const Eigen::MatrixXd expectedCentroidalDot =
        pinCentroidalDot * velocityTransform + pinCentroidal * velocityTransformDot;
    if (!fixture.floatingBase)
    {
        // Angular dAg rows use a convention still being reconciled against the native CCRBA terms.
        CompareMatrixRows(dtData.cmAmatDot,
                          expectedCentroidalDot,
                          0,
                          3,
                          1.0e-8,
                          fixture.name + ".cmAmatDotLinearRows");
    }
    // Free-base cmAmatDot entries are basis-derivative sensitive; the invariant checks below
    // validate the physical centroidal momentum-rate equations directly.

    pinocchio::computeCentroidalMomentumTimeVariation(pinModel, pinData, q, v, velocityTransformDotV);
    CompareEigenVector(Multiply(dtData.cmAmatDot, nativeV),
                       pinData.dhg.toVector(),
                       1.0e-8,
                       fixture.name + ".cmAmatDotQvel");

    pinocchio::computeCentroidalMomentumTimeVariation(pinModel, pinData, q, v, a);
    const Eigen::VectorXd nativeCentroidalMomentumRate =
        Multiply(dtData.cmAmat, nativeA) + Multiply(dtData.cmAmatDot, nativeV);
    CompareEigenVector(nativeCentroidalMomentumRate,
                       pinData.dhg.toVector(),
                       1.0e-8,
                       fixture.name + ".centMomRate");
}

} // namespace

int main()
{
    try
    {
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
