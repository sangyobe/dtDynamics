#include "dtDynamics/dtArray.h"
#include "dtDynamics/dtDynData.h"
#include "dtDynamics/dtDynCompile.h"
#include "dtDynamics/dtDynDynamics.h"
#include "dtDynamics/dtDynKinematics.h"
#include "dtDynamics/dtDynMath.h"
#include "dtDynamics/dtDynSpec.h"
#include "dtDynamics/dtDynUtils.h"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace dt::Dynamics;

namespace
{

void TestJointSizes()
{
    assert(GetJointDofSize(dtDynJointType::Free).qposSize == 7);
    assert(GetJointDofSize(dtDynJointType::Free).qvelSize == 6);
    assert(GetJointDofSize(dtDynJointType::Ball).qposSize == 4);
    assert(GetJointDofSize(dtDynJointType::Ball).qvelSize == 3);
    assert(GetJointDofSize(dtDynJointType::Slide).qposSize == 1);
    assert(GetJointDofSize(dtDynJointType::Slide).qvelSize == 1);
    assert(GetJointDofSize(dtDynJointType::Hinge).qposSize == 1);
    assert(GetJointDofSize(dtDynJointType::Hinge).qvelSize == 1);
}

void TestDtArrayDirectHeader()
{
    dtArray<dtDynIndex> values;
    values.push_back(1);
    values.push_back(2);
    values.push_back(3);

    assert(values.size() == 3);
    assert(values[0] == 1);
    assert(values[2] == 3);

    dtArray<dtDynIndex> copied = values;
    copied[1] = 20;

    assert(values[1] == 2);
    assert(copied[1] == 20);
}

void TestSpecWorldAndFixedBody()
{
    dtDynSpec spec;
    const auto base = spec.AddBody("base", 0);
    const auto fixed = spec.AddBody("fixed_sensor_mount", base);
    const auto knee = spec.AddBody("knee", fixed);
    const auto hinge = spec.AddJoint(knee, "knee_joint", dtDynJointType::Hinge);

    assert(spec.Bodies().size() == 4);
    assert(spec.Bodies()[0].name == "world");
    assert(spec.Bodies()[0].parent == DTDYN_INVALID_INDEX);
    assert(spec.Bodies()[fixed].joints.empty());
    assert(spec.Bodies()[knee].joints.size() == 1);
    assert(spec.Joints()[hinge].type == dtDynJointType::Hinge);
}

void TestSpecXmlDecodeEncodeRoundTrip()
{
    const std::string xml =
        "<mujoco model=\"single_arm\">"
        "<worldbody>"
        "<body name=\"base\" pos=\"0 0 0\">"
        "<body name=\"link\" pos=\"1 0 0\">"
        "<inertial pos=\"0.5 0 0\" mass=\"2.5\" diaginertia=\"0.1 0.2 0.3\"/>"
        "<joint name=\"hinge_z\" type=\"hinge\" axis=\"0 0 1\" armature=\"0.05\"/>"
        "<site name=\"tool\" pos=\"1 0 0\"/>"
        "</body>"
        "</body>"
        "</worldbody>"
        "<actuator><motor name=\"hinge_motor\" joint=\"hinge_z\" gear=\"80\"/></actuator>"
        "</mujoco>";

    const dtDynSpec spec = dtDynSpec::DecodeXml(xml);
    assert(spec.Bodies().size() == 3);
    assert(spec.Bodies()[0].name == "world");
    assert(spec.Bodies()[1].name == "base");
    assert(spec.Bodies()[2].name == "link");
    assert(spec.Bodies()[2].parent == 1);
    assert(std::fabs(spec.Bodies()[2].pos(0) - 1.0) < 1.0e-12);
    assert(std::fabs(spec.Bodies()[2].inertial.mass - 2.5) < 1.0e-12);
    assert(std::fabs(spec.Bodies()[2].inertial.pos(0) - 0.5) < 1.0e-12);
    assert(std::fabs(spec.Bodies()[2].inertial.inertia(2) - 0.3) < 1.0e-12);
    assert(spec.Joints().size() == 1);
    assert(spec.Joints()[0].name == "hinge_z");
    assert(spec.Joints()[0].type == dtDynJointType::Hinge);
    assert(std::fabs(spec.Joints()[0].axis(2) - 1.0) < 1.0e-12);
    assert(std::fabs(spec.Joints()[0].armature - 0.05) < 1.0e-12);
    assert(spec.Sites().size() == 1);
    assert(spec.Sites()[0].name == "tool");
    assert(spec.Actuators().size() == 1);
    assert(spec.Actuators()[0].name == "hinge_motor");
    assert(std::fabs(spec.Actuators()[0].gear - 80.0) < 1.0e-12);

    const std::string encoded = spec.EncodeXml();
    const dtDynSpec reparsed = dtDynSpec::DecodeXml(encoded);
    assert(reparsed.Bodies().size() == spec.Bodies().size());
    assert(reparsed.Joints().size() == spec.Joints().size());
    assert(reparsed.Sites().size() == spec.Sites().size());
    assert(reparsed.Actuators().size() == spec.Actuators().size());
    assert(reparsed.Bodies()[2].name == "link");
    assert(reparsed.Joints()[0].name == "hinge_z");
    assert(reparsed.Sites()[0].name == "tool");
    assert(reparsed.Actuators()[0].name == "hinge_motor");
}

void TestSpecXmlDecodesFreeJointAndFixedBody()
{
    const std::string xml =
        "<mujoco>"
        "<worldbody>"
        "<body name=\"base\" pos=\"0 0 0\">"
        "<freejoint name=\"floating_base\"/>"
        "<body name=\"fixed_sensor_mount\" pos=\"0.1 0 0\">"
        "<site name=\"imu_site\" pos=\"0 0 0\"/>"
        "</body>"
        "</body>"
        "</worldbody>"
        "</mujoco>";

    const dtDynSpec spec = dtDynSpec::DecodeXml(xml);
    assert(spec.Bodies().size() == 3);
    assert(spec.Joints().size() == 1);
    assert(spec.Sites().size() == 1);
    assert(spec.Joints()[0].name == "floating_base");
    assert(spec.Joints()[0].type == dtDynJointType::Free);
    assert(spec.Bodies()[2].name == "fixed_sensor_mount");
    assert(spec.Bodies()[2].joints.empty());
    assert(spec.Sites()[0].body == 2);

    const dtDynModel model = CompileSpec(spec);
    assert(model.nq == 7);
    assert(model.nv == 6);
    assert(model.body_jntnum[1] == 1);
    assert(model.body_dofnum[1] == 6);
    assert(model.body_jntnum[2] == 0);
    assert(model.body_dofnum[2] == 0);
    assert(model.site_bodyid[0] == 2);
}

void TestSpecXmlSkipsCommentsAndInfersRangeLimits()
{
    const std::string xml =
        "<mujoco>"
        "<worldbody>"
        "<body name=\"base\">"
        "<joint name=\"limited_hinge\" axis=\"0 1 0\" range=\"-1.5 2.5\"/>"
        "</body>"
        "<!-- <body name=\"ignored\"><joint name=\"ignored_joint\"/></body> -->"
        "</worldbody>"
        "</mujoco>";

    const dtDynSpec spec = dtDynSpec::DecodeXml(xml);
    assert(spec.Bodies().size() == 2);
    assert(spec.Bodies()[1].name == "base");
    assert(spec.Joints().size() == 1);
    assert(spec.Joints()[0].limited);
    const dtDynReal degreeToRadian = static_cast<dtDynReal>(3.14159265358979323846 / 180.0);
    assert(std::fabs(spec.Joints()[0].rangeMin + 1.5 * degreeToRadian) < 1.0e-12);
    assert(std::fabs(spec.Joints()[0].rangeMax - 2.5 * degreeToRadian) < 1.0e-12);

    const dtDynModel model = CompileSpec(spec);
    assert(model.BodyId("ignored") == DTDYN_INVALID_INDEX);
    assert(model.jnt_limited[0] == 1);
    assert(std::fabs(model.jnt_range_min(0) + 1.5 * degreeToRadian) < 1.0e-12);
    assert(std::fabs(model.jnt_range_max(0) - 2.5 * degreeToRadian) < 1.0e-12);
}

void TestSpecXmlRejectsEqualityUntilConstraintLayerExists()
{
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(
            "<mujoco><worldbody><body name=\"a\"/></worldbody>"
            "<equality><connect body1=\"a\" body2=\"world\"/></equality></mujoco>");
    }
    catch (const std::runtime_error &)
    {
        rejected = true;
    }
    assert(rejected);
}

void TestModelFixedBodyConvention()
{
    dtDynModel model;
    model.body_jntnum = {0, 1, 0, 1};
    model.body_jntadr = {DTDYN_INVALID_INDEX, 0, DTDYN_INVALID_INDEX, 1};
    model.body_dofnum = {0, 6, 0, 1};
    model.body_dofadr = {DTDYN_INVALID_INDEX, 0, DTDYN_INVALID_INDEX, 6};

    assert(model.IsWorldBody(0));
    assert(!model.IsFixedBody(1));
    assert(model.IsFixedBody(2));
    assert(!model.IsFixedBody(3));
}

void TestCompiledModelCopiesIntoDefaultModel()
{
    dtDynSpec spec;
    const auto base = spec.AddBody("base", 0);
    spec.AddJoint(base, "floating_base", dtDynJointType::Free);
    const auto link = spec.AddBody("link", base);
    spec.Bodies()[static_cast<std::size_t>(link)].inertial.mass = 2.0;
    spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);
    spec.AddSite(link, "tool");
    const auto actuator = spec.AddActuator(1, "hinge_motor");
    spec.Actuators()[static_cast<std::size_t>(actuator)].gear = 120.0;

    const dtDynModel compiled = CompileSpec(spec);

    dtDynModel copied;
    copied = compiled;

    assert(copied.nq == compiled.nq);
    assert(copied.nv == compiled.nv);
    assert(copied.nbody == compiled.nbody);
    assert(copied.BodyId("link") == compiled.BodyId("link"));
    assert(copied.SiteId("tool") == compiled.SiteId("tool"));
    assert(copied.jnt_range_min.GetDim() == compiled.jnt_range_min.GetDim());
    assert(copied.dof_armature.GetDim() == compiled.dof_armature.GetDim());
    assert(copied.body_mass.GetDim() == compiled.body_mass.GetDim());
    assert(std::fabs(copied.body_mass(link) - 2.0) < 1.0e-12);
    assert(std::fabs(copied.actuator_gear(0) - 120.0) < 1.0e-12);

    dtDynModel copyConstructed(compiled);
    assert(copyConstructed.nv == compiled.nv);
    assert(copyConstructed.BodyId("base") == compiled.BodyId("base"));
}

void TestCompiledModelCopiesZeroSizedOptionalDenseFields()
{
    dtDynSpec spec;
    const auto link = spec.AddBody("link", 0);
    spec.Bodies()[static_cast<std::size_t>(link)].inertial.mass = 1.0;
    spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);

    const dtDynModel compiled = CompileSpec(spec);
    assert(compiled.nu == 0);
    assert(compiled.actuator_gear.GetDim() == 0);

    dtDynModel copied;
    copied = compiled;

    assert(copied.nu == 0);
    assert(copied.actuator_gear.GetDim() == 0);
    assert(copied.BodyId("link") == compiled.BodyId("link"));
}

void TestCompileSpecCanFillExistingModel()
{
    dtDynSpec spec;
    const auto link = spec.AddBody("link", 0);
    spec.Bodies()[static_cast<std::size_t>(link)].inertial.mass = 1.0;
    spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);

    dtDynModel model;
    CompileSpec(spec, model);

    assert(model.nbody == 2);
    assert(model.nv == 1);
    assert(model.BodyId("link") == link);
}

void TestDtMathDynamicResizeHelpers()
{
    dtDynDenseVector vector;
    ResizeDenseVector(vector, 3);
    assert(vector.GetDim() == 3);
    assert(vector(0) == 0.0);

    vector(0) = 1.25;
    ResizeDenseVector(vector, 5);
    assert(vector.GetDim() == 5);
    assert(vector(0) == 0.0);
    vector(4) = 7.25;

    dtDynDenseVector copiedVector = vector;
    assert(copiedVector.GetDim() == 5);
    assert(copiedVector(4) == 7.25);

    dtDynDenseMatrix matrix;
    ResizeDenseMatrix(matrix, 2, 3);
    assert(matrix.GetRowSize() == 2);
    assert(matrix.GetColSize() == 3);
    assert(matrix(1, 2) == 0.0);

    matrix(1, 2) = 4.5;
    ResizeDenseMatrix(matrix, 4, 2);
    assert(matrix.GetRowSize() == 4);
    assert(matrix.GetColSize() == 2);
    assert(matrix(1, 1) == 0.0);
    matrix(3, 1) = 8.5;

    dtDynDenseMatrix copiedMatrix = matrix;
    assert(copiedMatrix.GetRowSize() == 4);
    assert(copiedMatrix.GetColSize() == 2);
    assert(copiedMatrix(3, 1) == 8.5);
}

void TestDataResizeSeparatesBodyAndDofStorage()
{
    dtDynModel model;
    model.nq = 8;
    model.nv = 7;
    model.nbody = 4;
    model.nsite = 2;
    model.nroot = 2;

    dtDynData data;
    data.Resize(model);

    assert(data.IsCompatible(model));
    assert(data.qpos.GetDim() == 8);
    assert(data.qvel.GetDim() == 7);
    assert(data.xpos.size() == 4);
    assert(data.site_xpos.size() == 2);
    assert(data.cdof.size() == 7);
    assert(data.cvel.size() == 4);
    assert(data.body_jacp.size() == 4);
    assert(data.body_jacr.size() == 4);
    assert(data.body_jacp_dot.size() == 4);
    assert(data.body_jacr_dot.size() == 4);
    assert(data.site_jacp.size() == 2);
    assert(data.site_jacr.size() == 2);
    assert(data.site_jacp_dot.size() == 2);
    assert(data.site_jacr_dot.size() == 2);
    assert(data.body_jacp[0].GetRowSize() == 3);
    assert(data.body_jacp[0].GetColSize() == 7);
    assert(data.body_jacr_dot[3].GetRowSize() == 3);
    assert(data.body_jacr_dot[3].GetColSize() == 7);
    assert(data.site_jacp[1].GetRowSize() == 3);
    assert(data.site_jacp[1].GetColSize() == 7);
    assert(data.site_jacr_dot[1].GetRowSize() == 3);
    assert(data.site_jacr_dot[1].GetColSize() == 7);
    assert(data.Mmat.GetRowSize() == 7);
    assert(data.Mmat.GetColSize() == 7);
    assert(data.MmatDot.GetRowSize() == 7);
    assert(data.MmatDot.GetColSize() == 7);
    assert(data.comJmat.GetRowSize() == 6);
    assert(data.comJmat.GetColSize() == 7);
    assert(data.cmAmat.GetRowSize() == 12);
    assert(data.cmAmat.GetColSize() == 7);
    assert(data.comPos.GetDim() == 6);
    assert(data.comVel.GetDim() == 6);

    data.qpos(0) = 1.0;
    data.Mmat(0, 0) = 2.0;
    assert(data.qpos(0) == 1.0);
    assert(data.Mmat(0, 0) == 2.0);

    dtDynModel mismatchedModel = model;
    mismatchedModel.nv = 8;
    assert(!data.IsCompatible(mismatchedModel));
}

void TestDtDynMathHelpersAreReusable()
{
    dt::Math::Vector3<dtDynReal> x;
    dt::Math::Vector3<dtDynReal> y;
    x(0) = 1.0;
    x(1) = 0.0;
    x(2) = 0.0;
    y(0) = 0.0;
    y(1) = 1.0;
    y(2) = 0.0;

    const dt::Math::Vector3<dtDynReal> z = x & y;
    assert(z(0) == 0.0);
    assert(z(1) == 0.0);
    assert(z(2) == 1.0);

    const dt::Math::Vector4<dtDynReal> quat = math::AxisAngleQuat(z, 0.5 * 3.14159265358979323846);
    const dt::Math::Vector3<dtDynReal> rotated = math::Rotate(quat, x);
    assert(std::fabs(rotated(0)) < 1.0e-9);
    assert(std::fabs(rotated(1) - 1.0) < 1.0e-9);
    assert(std::fabs(rotated(2)) < 1.0e-9);

    dtDynDenseMatrix matrix;
    ResizeDenseMatrix(matrix, 3, 2);
    matrix(0, 1) = 4.0;
    matrix(1, 1) = 5.0;
    matrix(2, 1) = 6.0;
    const dt::Math::Vector3<dtDynReal> column = math::DenseMatrixColumn3(matrix, 1);
    assert(column(0) == 4.0);
    assert(column(1) == 5.0);
    assert(column(2) == 6.0);
    ResizeDenseMatrix(matrix, 4, 4);
    matrix.SetZero();
    matrix(1, 1) = 1.0;
    matrix(1, 2) = 2.0;
    matrix(1, 3) = 3.0;
    matrix(2, 1) = 4.0;
    matrix(2, 2) = 5.0;
    matrix(2, 3) = 6.0;
    matrix(3, 1) = 7.0;
    matrix(3, 2) = 8.0;
    matrix(3, 3) = 9.0;
    const dt::Math::Matrix3<dtDynReal> block = math::DenseMatrixBlock3(matrix, 1, 1);
    assert(block(0, 0) == 1.0);
    assert(block(1, 1) == 5.0);
    assert(block(2, 2) == 9.0);
    matrix.SetZero();
    assert(matrix(0, 1) == 0.0);
    assert(matrix(2, 1) == 0.0);

    dtDynModel utilityModel;
    utilityModel.nbody = 2;
    utilityModel.body_parentid = {0, 0};
    utilityModel.body_dofnum = {0, 1};
    utilityModel.body_dofadr = {DTDYN_INVALID_INDEX, 3};
    utilityModel.body_inertia.resize(2);
    utilityModel.body_inertia[1](0) = 1.0;
    utilityModel.body_inertia[1](1) = 2.0;
    utilityModel.body_inertia[1](2) = 3.0;

    dtDynData utilityData;
    utilityData.ximat.resize(2);
    utilityData.ximat[1].SetIdentity();

    assert(math::FindBodyLastDof(utilityModel, 1) == 3);
    const dt::Math::Matrix3<dtDynReal> worldInertia = math::BodyInertiaInWorld(utilityModel, utilityData, 1);
    assert(worldInertia(0, 0) == 1.0);
    assert(worldInertia(1, 1) == 2.0);
    assert(worldInertia(2, 2) == 3.0);

    dt::Math::Vector6<dtDynReal> spatialVelocity;
    dt::Math::Vector6<dtDynReal> spatialAxis;
    spatialVelocity.SetZero();
    spatialAxis.SetZero();
    spatialVelocity(2) = 2.0;
    spatialVelocity(3) = 3.0;
    spatialAxis(0) = 1.0;
    spatialAxis(4) = 5.0;

    const dt::Math::Vector6<dtDynReal> crossed = math::CrossSpatialMotion(spatialVelocity, spatialAxis);
    assert(crossed(1) == 2.0);
    assert(crossed(3) == -10.0);
    assert(crossed(4) == 0.0);

    dt::Math::Matrix<6, 6, dtDynReal> identity6;
    identity6.SetIdentity();
    const dt::Math::Matrix<6, 6, dtDynReal> transposed = identity6.Transpose();
    assert(transposed(0, 0) == 1.0);
    assert(transposed(5, 5) == 1.0);
}

void TestCompileSpecUsesNativeIndexLayout()
{
    dtDynSpec spec;
    const auto base = spec.AddBody("base", 0);
    const auto shoulder = spec.AddBody("shoulder", base);
    const auto elbow = spec.AddBody("elbow", shoulder);
    const auto tool = spec.AddBody("tool", elbow);
    const auto toolSite = spec.AddSite(tool, "tool_tip");

    spec.Bodies()[base].inertial.mass = 2.0;
    spec.Bodies()[shoulder].inertial.mass = 3.0;
    spec.Bodies()[elbow].inertial.mass = 4.0;
    spec.Bodies()[tool].inertial.mass = 1.0;

    const auto elbowJoint = spec.AddJoint(elbow, "elbow_hinge", dtDynJointType::Hinge);
    const auto floatingBase = spec.AddJoint(shoulder, "floating_base", dtDynJointType::Free);
    spec.Joints()[floatingBase].armature = 0.1;
    spec.Joints()[elbowJoint].armature = 0.2;
    const auto actuator = spec.AddActuator(elbowJoint, "elbow_motor");
    spec.Actuators()[actuator].gear = 120.0;

    const dtDynModel model = CompileSpec(spec);

    assert(model.nbody == 5);
    assert(model.njnt == 2);
    assert(model.nsite == 1);
    assert(model.nu == 1);
    assert(model.nq == 8);
    assert(model.nv == 7);
    assert(model.nroot == 1);

    assert(model.body_parentid[0] == 0);
    assert(model.body_parentid[base] == 0);
    assert(model.body_parentid[shoulder] == base);
    assert(model.body_parentid[elbow] == shoulder);
    assert(model.body_parentid[tool] == elbow);

    assert(model.IsWorldBody(0));
    assert(model.IsFixedBody(base));
    assert(!model.IsFixedBody(shoulder));
    assert(!model.IsFixedBody(elbow));
    assert(model.IsFixedBody(tool));

    assert(model.body_rootid[base] == base);
    assert(model.body_rootid[shoulder] == base);
    assert(model.body_rootid[elbow] == base);
    assert(model.body_rootid[tool] == base);
    assert(model.body_rootidx[base] == 0);
    assert(model.body_rootidx[shoulder] == 0);
    assert(model.body_rootidx[elbow] == 0);
    assert(model.body_rootidx[tool] == 0);
    assert(model.root_bodyid[0] == base);
    assert(model.root_mass(0) == 10.0);
    assert(model.body_weldid[base] == 0);
    assert(model.body_weldid[shoulder] == shoulder);
    assert(model.body_weldid[elbow] == elbow);
    assert(model.body_weldid[tool] == elbow);

    assert(model.body_jntadr[shoulder] == 0);
    assert(model.body_jntnum[shoulder] == 1);
    assert(model.body_dofadr[shoulder] == 0);
    assert(model.body_dofnum[shoulder] == 6);
    assert(model.body_jntadr[elbow] == 1);
    assert(model.body_jntnum[elbow] == 1);
    assert(model.body_dofadr[elbow] == 6);
    assert(model.body_dofnum[elbow] == 1);

    assert(model.joint_names[0] == "floating_base");
    assert(model.joint_names[1] == "elbow_hinge");
    assert(model.jnt_type[0] == static_cast<dtDynIndex>(dtDynJointType::Free));
    assert(model.jnt_type[1] == static_cast<dtDynIndex>(dtDynJointType::Hinge));
    assert(model.jnt_qposadr[0] == 0);
    assert(model.jnt_qposadr[1] == 7);
    assert(model.jnt_dofadr[0] == 0);
    assert(model.jnt_dofadr[1] == 6);

    for (dtDynIndex dof = 0; dof < 6; ++dof)
    {
        assert(model.dof_bodyid[dof] == shoulder);
        assert(model.dof_jntid[dof] == 0);
        assert(model.dof_parentid[dof] == dof - 1);
        assert(model.dof_armature(dof) == 0.1);
    }
    assert(model.dof_bodyid[6] == elbow);
    assert(model.dof_jntid[6] == 1);
    assert(model.dof_parentid[6] == 5);
    assert(model.dof_armature(6) == 0.2);

    assert(model.site_names[toolSite] == "tool_tip");
    assert(model.site_bodyid[toolSite] == tool);
    assert(model.body_subtreemass(0) == 10.0);
    assert(model.body_subtreemass(base) == 10.0);
    assert(model.body_subtreemass(shoulder) == 8.0);
    assert(model.body_subtreemass(elbow) == 5.0);
    assert(model.body_subtreemass(tool) == 1.0);
    assert(model.actuator_names[actuator] == "elbow_motor");
    assert(model.actuator_jointid[actuator] == 1);
    assert(model.actuator_gear(actuator) == 120.0);

    assert(model.BodyId("elbow") == elbow);
    assert(model.JointId("floating_base") == 0);
    assert(model.JointId("elbow_hinge") == 1);
    assert(model.SiteId("tool_tip") == toolSite);
}

void TestCompileSpecSupportsMultipleMovableRoots()
{
    dtDynSpec spec;
    const auto left = spec.AddBody("left_link", 0);
    const auto right = spec.AddBody("right_link", 0);

    spec.Bodies()[left].inertial.mass = 2.0;
    spec.Bodies()[left].inertial.pos(0) = 1.0;
    spec.Bodies()[right].pos(0) = 3.0;
    spec.Bodies()[right].inertial.mass = 3.0;
    spec.Bodies()[right].inertial.pos(0) = 1.0;

    const auto leftJoint = spec.AddJoint(left, "left_hinge", dtDynJointType::Hinge);
    const auto rightJoint = spec.AddJoint(right, "right_hinge", dtDynJointType::Hinge);
    spec.Joints()[leftJoint].axis(2) = 1.0;
    spec.Joints()[rightJoint].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);
    assert(model.nroot == 2);
    assert(model.root_bodyid[0] == left);
    assert(model.root_bodyid[1] == right);
    assert(model.body_rootidx[left] == 0);
    assert(model.body_rootidx[right] == 1);
    assert(model.root_mass(0) == 2.0);
    assert(model.root_mass(1) == 3.0);

    dtDynData data;
    data.Resize(model);
    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);
    ComputeJacobians(model, data);

    assert(data.cmAmat.GetRowSize() == 12);
    assert(data.cmAmat.GetColSize() == 2);
    assert(data.comJmat.GetRowSize() == 6);
    assert(data.comJmat.GetColSize() == 2);
    assert(std::fabs(data.Mmat(0, 1)) < 1.0e-12);
    assert(std::fabs(data.Mmat(1, 0)) < 1.0e-12);
    assert(std::fabs(data.cmAmat(1, 0) - 2.0) < 1.0e-12);
    assert(std::fabs(data.cmAmat(7, 1) - 3.0) < 1.0e-12);
    assert(std::fabs(data.comJmat(1, 0) - 1.0) < 1.0e-12);
    assert(std::fabs(data.comJmat(4, 1) - 1.0) < 1.0e-12);
}

void TestCompileSpecSupportsFloatingBaseQuadrupedTree()
{
    dtDynSpec spec;
    const auto trunk = spec.AddBody("trunk", 0);
    spec.Bodies()[trunk].inertial.mass = 5.0;
    spec.Bodies()[trunk].inertial.inertia(0) = 0.25;
    spec.Bodies()[trunk].inertial.inertia(1) = 0.35;
    spec.Bodies()[trunk].inertial.inertia(2) = 0.45;

    spec.AddJoint(trunk, "floating_base", dtDynJointType::Free);

    const char *legNames[4] = {"front_left", "front_right", "rear_left", "rear_right"};
    const dtDynReal xOffsets[4] = {0.4, 0.4, -0.4, -0.4};
    const dtDynReal yOffsets[4] = {0.25, -0.25, 0.25, -0.25};

    for (dtDynIndex leg = 0; leg < 4; ++leg)
    {
        const auto upper = spec.AddBody(std::string(legNames[leg]) + "_upper", trunk);
        spec.Bodies()[upper].pos(0) = xOffsets[leg];
        spec.Bodies()[upper].pos(1) = yOffsets[leg];
        spec.Bodies()[upper].inertial.mass = 1.0;
        spec.Bodies()[upper].inertial.pos(2) = -0.2;
        spec.Bodies()[upper].inertial.inertia(0) = 0.02;
        spec.Bodies()[upper].inertial.inertia(1) = 0.02;
        spec.Bodies()[upper].inertial.inertia(2) = 0.01;

        const auto hip = spec.AddJoint(upper, std::string(legNames[leg]) + "_hip", dtDynJointType::Hinge);
        spec.Joints()[hip].axis(1) = 1.0;
    }

    const dtDynModel model = CompileSpec(spec);

    assert(model.nroot == 1);
    assert(model.root_bodyid[0] == trunk);
    assert(std::fabs(model.root_mass(0) - 9.0) < 1.0e-12);
    assert(model.nq == 11);
    assert(model.nv == 10);

    for (dtDynIndex bodyId = 1; bodyId < model.nbody; ++bodyId)
    {
        assert(model.body_rootidx[bodyId] == 0);
    }

    dtDynData data;
    data.Resize(model);
    data.qpos(3) = 1.0;
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        data.qvel(dofId) = 0.1 * static_cast<dtDynReal>(dofId + 1);
    }

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();

    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    assert(data.Mmat.GetRowSize() == 10);
    assert(data.Mmat.GetColSize() == 10);
    assert(data.cmAmat.GetRowSize() == 6);
    assert(data.cmAmat.GetColSize() == 10);
    assert(data.comJmat.GetRowSize() == 3);
    assert(data.comJmat.GetColSize() == 10);
    assert(std::fabs(data.Mmat(0, 0) - model.root_mass(0)) < 1.0e-9);
    assert(data.Mmat(3, 3) > 0.0);
    assert(data.Mmat(6, 6) > 0.0);
    assert(std::fabs(data.Mmat(8, 9) - data.Mmat(9, 8)) < 1.0e-12);
    assert(std::fabs(data.nonlinTorq(0) - data.coriolisTorq(0)) < 1.0e-9);
}

void TestForwardKinematicsAndPointJacobianForFixedChain()
{
    dtDynSpec spec;
    const auto base = spec.AddBody("base", 0);
    const auto shoulder = spec.AddBody("shoulder", base);
    const auto forearm = spec.AddBody("forearm", shoulder);
    const auto tool = spec.AddSite(forearm, "tool");

    spec.Bodies()[shoulder].pos(0) = 1.0;
    spec.Bodies()[forearm].pos(0) = 1.0;
    spec.Sites()[tool].pos(0) = 1.0;

    const auto hinge = spec.AddJoint(shoulder, "shoulder_yaw", dtDynJointType::Hinge);
    spec.Joints()[hinge].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model);
    data.qpos(0) = 0.5 * 3.14159265358979323846;

    ForwardKinematics(model, data);

    assert(std::fabs(data.xpos[forearm](0) - 1.0) < 1.0e-9);
    assert(std::fabs(data.xpos[forearm](1) - 1.0) < 1.0e-9);
    assert(std::fabs(data.site_xpos[tool](0) - 1.0) < 1.0e-9);
    assert(std::fabs(data.site_xpos[tool](1) - 2.0) < 1.0e-9);

    dtDynDenseMatrix jacp;
    dtDynDenseMatrix jacr;
    ComputePointJacobian(model, data, forearm, data.site_xpos[tool], jacp, jacr);
    dtDynDenseMatrix siteJacp;
    dtDynDenseMatrix siteJacr;
    ComputeSiteJacobian(model, data, tool, siteJacp, siteJacr);
    dtDynDenseMatrix bodyJacp;
    dtDynDenseMatrix bodyJacr;
    ComputeBodyJacobian(model, data, forearm, bodyJacp, bodyJacr);

    assert(jacp.GetRowSize() == 3);
    assert(jacp.GetColSize() == model.nv);
    assert(jacr.GetRowSize() == 3);
    assert(jacr.GetColSize() == model.nv);
    assert(std::fabs(jacp(0, 0) + 2.0) < 1.0e-9);
    assert(std::fabs(jacp(1, 0)) < 1.0e-9);
    assert(std::fabs(jacr(2, 0) - 1.0) < 1.0e-9);
    assert(std::fabs(siteJacp(0, 0) - jacp(0, 0)) < 1.0e-12);
    assert(std::fabs(siteJacp(1, 0) - jacp(1, 0)) < 1.0e-12);
    assert(std::fabs(siteJacr(2, 0) - jacr(2, 0)) < 1.0e-12);
    assert(std::fabs(bodyJacp(0, 0) + 1.0) < 1.0e-9);
    assert(std::fabs(bodyJacp(1, 0)) < 1.0e-9);
    assert(std::fabs(bodyJacr(2, 0) - 1.0) < 1.0e-9);

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    assert(data.body_jacp.size() == static_cast<std::size_t>(model.nbody));
    assert(data.site_jacp.size() == static_cast<std::size_t>(model.nsite));
    assert(data.body_jacp[forearm].GetRowSize() == 3);
    assert(data.body_jacp[forearm].GetColSize() == model.nv);
    assert(data.site_jacp[tool].GetRowSize() == 3);
    assert(data.site_jacp[tool].GetColSize() == model.nv);

    dtDynDenseMatrix directSiteJacp;
    dtDynDenseMatrix directSiteJacr;
    dtDynDenseMatrix directSiteJacpDot;
    dtDynDenseMatrix directSiteJacrDot;
    ComputeSiteJacobian(model, data, tool, directSiteJacp, directSiteJacr);
    ComputeSiteJacobianDot(model, data, tool, directSiteJacpDot, directSiteJacrDot);
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            assert(std::fabs(data.site_jacp[tool](row, col) - directSiteJacp(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacr[tool](row, col) - directSiteJacr(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacp_dot[tool](row, col) - directSiteJacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacr_dot[tool](row, col) - directSiteJacrDot(row, col)) < 1.0e-12);
        }
    }

    const dt::Math::Vector6<dtDynReal> motion =
        utils::MotionSubspaceAtPoint(model, data, 0, data.site_xpos[tool]);
    assert(std::fabs(motion(3) - jacp(0, 0)) < 1.0e-12);
    assert(std::fabs(motion(4) - jacp(1, 0)) < 1.0e-12);
    assert(std::fabs(motion(2) - jacr(2, 0)) < 1.0e-12);
    assert(utils::RootCentroidalLinearRow(2) == 12);
    assert(utils::RootCentroidalAngularRow(2) == 15);
    assert(utils::RootComRow(2) == 6);
}

void TestIndexedJacobiansOnlyRefreshRequestedIds()
{
    dtDynSpec spec;
    const auto base = spec.AddBody("base", 0);
    const auto shoulder = spec.AddBody("shoulder", base);
    const auto forearm = spec.AddBody("forearm", shoulder);
    const auto tool = spec.AddSite(forearm, "tool");
    const auto unused = spec.AddSite(shoulder, "unused");

    spec.Bodies()[shoulder].pos(0) = 1.0;
    spec.Bodies()[forearm].pos(0) = 1.0;
    spec.Sites()[tool].pos(0) = 1.0;
    spec.Sites()[unused].pos(1) = 0.5;

    const auto hinge = spec.AddJoint(shoulder, "shoulder_yaw", dtDynJointType::Hinge);
    spec.Joints()[hinge].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model);
    data.qpos(0) = 0.25;
    data.qvel(0) = 1.2;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            assert(std::fabs(data.body_jacp[forearm](row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacp[tool](row, col)) < 1.0e-12);
        }
    }

    const dtDynArray<dtDynIndex> bodyIds = {forearm};
    const dtDynArray<dtDynIndex> siteIds = {tool};
    ComputeJacobians(model, data, bodyIds, siteIds);

    dtDynDenseMatrix directBodyJacp;
    dtDynDenseMatrix directBodyJacr;
    dtDynDenseMatrix directBodyJacpDot;
    dtDynDenseMatrix directBodyJacrDot;
    ComputeBodyJacobian(model, data, forearm, directBodyJacp, directBodyJacr);
    ComputeBodyJacobianDot(model, data, forearm, directBodyJacpDot, directBodyJacrDot);

    dtDynDenseMatrix directSiteJacp;
    dtDynDenseMatrix directSiteJacr;
    dtDynDenseMatrix directSiteJacpDot;
    dtDynDenseMatrix directSiteJacrDot;
    ComputeSiteJacobian(model, data, tool, directSiteJacp, directSiteJacr);
    ComputeSiteJacobianDot(model, data, tool, directSiteJacpDot, directSiteJacrDot);

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            assert(std::fabs(data.body_jacp[forearm](row, col) - directBodyJacp(row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacr[forearm](row, col) - directBodyJacr(row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacp_dot[forearm](row, col) - directBodyJacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacr_dot[forearm](row, col) - directBodyJacrDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacp[tool](row, col) - directSiteJacp(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacr[tool](row, col) - directSiteJacr(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacp_dot[tool](row, col) - directSiteJacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacr_dot[tool](row, col) - directSiteJacrDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacp[shoulder](row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacr[shoulder](row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacp_dot[shoulder](row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacr_dot[shoulder](row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacp[unused](row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacr[unused](row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacp_dot[unused](row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacr_dot[unused](row, col)) < 1.0e-12);
        }
    }
}

void TestIndexedJacobiansRefreshFloatingBaseBodyAndTaskSite()
{
    dtDynSpec spec;
    const auto trunk = spec.AddBody("trunk", 0);
    spec.Bodies()[trunk].inertial.mass = 5.0;
    spec.Bodies()[trunk].inertial.inertia(0) = 0.20;
    spec.Bodies()[trunk].inertial.inertia(1) = 0.30;
    spec.Bodies()[trunk].inertial.inertia(2) = 0.40;
    spec.AddJoint(trunk, "floating_base", dtDynJointType::Free);

    const auto thigh = spec.AddBody("thigh", trunk);
    spec.Bodies()[thigh].pos(0) = 0.35;
    spec.Bodies()[thigh].pos(2) = -0.10;
    spec.Bodies()[thigh].inertial.mass = 1.0;
    const auto hip = spec.AddJoint(thigh, "hip_pitch", dtDynJointType::Hinge);
    spec.Joints()[hip].axis(1) = 1.0;

    const auto foot = spec.AddSite(thigh, "foot");
    spec.Sites()[foot].pos(2) = -0.25;

    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model);
    data.qpos(2) = 0.45;
    data.qpos(3) = 1.0;
    data.qpos(7) = -0.20;
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        data.qvel(dofId) = 0.05 * static_cast<dtDynReal>(dofId + 1);
    }

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    const dtDynArray<dtDynIndex> bodyIds = {model.root_bodyid[0]};
    const dtDynArray<dtDynIndex> siteIds = {foot};
    ComputeJacobians(model, data, bodyIds, siteIds);

    dtDynDenseMatrix directBaseJacp;
    dtDynDenseMatrix directBaseJacr;
    dtDynDenseMatrix directBaseJacpDot;
    dtDynDenseMatrix directBaseJacrDot;
    ComputeBodyJacobian(model, data, trunk, directBaseJacp, directBaseJacr);
    ComputeBodyJacobianDot(model, data, trunk, directBaseJacpDot, directBaseJacrDot);

    dtDynDenseMatrix directFootJacp;
    dtDynDenseMatrix directFootJacr;
    dtDynDenseMatrix directFootJacpDot;
    dtDynDenseMatrix directFootJacrDot;
    ComputeSiteJacobian(model, data, foot, directFootJacp, directFootJacr);
    ComputeSiteJacobianDot(model, data, foot, directFootJacpDot, directFootJacrDot);

    assert(std::fabs(data.body_jacp[trunk](0, 0)) > 0.5);
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            assert(std::fabs(data.body_jacp[trunk](row, col) - directBaseJacp(row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacr[trunk](row, col) - directBaseJacr(row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacp_dot[trunk](row, col) - directBaseJacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacr_dot[trunk](row, col) - directBaseJacrDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacp[foot](row, col) - directFootJacp(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacr[foot](row, col) - directFootJacr(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacp_dot[foot](row, col) - directFootJacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.site_jacr_dot[foot](row, col) - directFootJacrDot(row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacp[thigh](row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacr[thigh](row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacp_dot[thigh](row, col)) < 1.0e-12);
            assert(std::fabs(data.body_jacr_dot[thigh](row, col)) < 1.0e-12);
        }
    }
}

void TestPointJacobianDotMatchesFiniteDifferenceForMaterialSite()
{
    dtDynSpec spec;
    const auto link1 = spec.AddBody("link1", 0);
    const auto link2 = spec.AddBody("link2", link1);
    const auto tool = spec.AddSite(link2, "tool");

    spec.Bodies()[link1].pos(0) = 0.5;
    spec.Bodies()[link1].inertial.mass = 1.0;
    spec.Bodies()[link1].inertial.pos(0) = 0.5;
    spec.Bodies()[link2].pos(0) = 1.0;
    spec.Bodies()[link2].inertial.mass = 0.75;
    spec.Bodies()[link2].inertial.pos(0) = 0.5;
    spec.Sites()[tool].pos(0) = 0.8;

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.Joints()[joint1].axis(2) = 1.0;
    spec.Joints()[joint2].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model);
    data.qpos(0) = 0.35;
    data.qpos(1) = -0.45;
    data.qvel(0) = 1.1;
    data.qvel(1) = -0.7;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    dtDynDenseMatrix jacpDot;
    dtDynDenseMatrix jacrDot;
    ComputePointJacobianDot(model, data, link2, data.site_xpos[tool], jacpDot, jacrDot);
    dtDynDenseMatrix siteJacpDot;
    dtDynDenseMatrix siteJacrDot;
    ComputeSiteJacobianDot(model, data, tool, siteJacpDot, siteJacrDot);

    const dtDynReal eps = 1.0e-6;
    dtDynDenseMatrix jacpPlus;
    dtDynDenseMatrix jacrPlus;
    dtDynDenseMatrix jacpMinus;
    dtDynDenseMatrix jacrMinus;

    dtDynData plus;
    plus.Resize(model);
    plus.qpos(0) = data.qpos(0) + eps * data.qvel(0);
    plus.qpos(1) = data.qpos(1) + eps * data.qvel(1);
    ForwardKinematics(model, plus);
    ComputePointJacobian(model, plus, link2, plus.site_xpos[tool], jacpPlus, jacrPlus);

    dtDynData minus;
    minus.Resize(model);
    minus.qpos(0) = data.qpos(0) - eps * data.qvel(0);
    minus.qpos(1) = data.qpos(1) - eps * data.qvel(1);
    ForwardKinematics(model, minus);
    ComputePointJacobian(model, minus, link2, minus.site_xpos[tool], jacpMinus, jacrMinus);

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            const dtDynReal fdLinear = (jacpPlus(row, col) - jacpMinus(row, col)) / (2.0 * eps);
            const dtDynReal fdAngular = (jacrPlus(row, col) - jacrMinus(row, col)) / (2.0 * eps);
            assert(std::fabs(jacpDot(row, col) - fdLinear) < 1.0e-5);
            assert(std::fabs(jacrDot(row, col) - fdAngular) < 1.0e-5);
            assert(std::fabs(siteJacpDot(row, col) - jacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(siteJacrDot(row, col) - jacrDot(row, col)) < 1.0e-12);
        }
    }

    dt::Math::Vector3<dtDynReal> linearJdotQvel;
    dt::Math::Vector3<dtDynReal> angularJdotQvel;
    ComputePointJacobianDotTimesVelocity(model, data, link2, data.site_xpos[tool],
                                         linearJdotQvel, angularJdotQvel);
    dt::Math::Vector3<dtDynReal> siteLinearJdotQvel;
    dt::Math::Vector3<dtDynReal> siteAngularJdotQvel;
    ComputeSiteJacobianDotTimesVelocity(model, data, tool,
                                        siteLinearJdotQvel, siteAngularJdotQvel);
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        dtDynReal expectedLinear = 0.0;
        dtDynReal expectedAngular = 0.0;
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            expectedLinear += jacpDot(row, col) * data.qvel(col);
            expectedAngular += jacrDot(row, col) * data.qvel(col);
        }
        assert(std::fabs(linearJdotQvel(row) - expectedLinear) < 1.0e-12);
        assert(std::fabs(angularJdotQvel(row) - expectedAngular) < 1.0e-12);
        assert(std::fabs(siteLinearJdotQvel(row) - linearJdotQvel(row)) < 1.0e-12);
        assert(std::fabs(siteAngularJdotQvel(row) - angularJdotQvel(row)) < 1.0e-12);
    }
}

void TestPointJacobianTermsMatchesSeparateAPIs()
{
    dtDynSpec spec;
    const auto link1 = spec.AddBody("link1", 0);
    const auto link2 = spec.AddBody("link2", link1);
    const auto tool = spec.AddSite(link2, "tool");

    spec.Bodies()[link1].pos(0) = 0.5;
    spec.Bodies()[link1].inertial.mass = 1.0;
    spec.Bodies()[link1].inertial.pos(0) = 0.5;
    spec.Bodies()[link2].pos(0) = 1.0;
    spec.Bodies()[link2].inertial.mass = 0.75;
    spec.Bodies()[link2].inertial.pos(0) = 0.5;
    spec.Sites()[tool].pos(0) = 0.8;

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.Joints()[joint1].axis(2) = 1.0;
    spec.Joints()[joint2].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model);
    data.qpos(0) = 0.35;
    data.qpos(1) = -0.45;
    data.qvel(0) = 1.1;
    data.qvel(1) = -0.7;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    dtDynDenseMatrix jacp;
    dtDynDenseMatrix jacr;
    dtDynDenseMatrix jacpDot;
    dtDynDenseMatrix jacrDot;
    ComputePointJacobian(model, data, link2, data.site_xpos[tool], jacp, jacr);
    ComputePointJacobianDot(model, data, link2, data.site_xpos[tool], jacpDot, jacrDot);

    dt::Math::Vector3<dtDynReal> linearJdotQvel;
    dt::Math::Vector3<dtDynReal> angularJdotQvel;
    ComputePointJacobianDotTimesVelocity(model, data, link2, data.site_xpos[tool],
                                         linearJdotQvel, angularJdotQvel);

    dtDynDenseMatrix combinedJacp;
    dtDynDenseMatrix combinedJacr;
    dtDynDenseMatrix combinedJacpDot;
    dtDynDenseMatrix combinedJacrDot;
    dt::Math::Vector3<dtDynReal> combinedLinearJdotQvel;
    dt::Math::Vector3<dtDynReal> combinedAngularJdotQvel;
    ComputePointJacobianTerms(model,
                              data,
                              link2,
                              data.site_xpos[tool],
                              &combinedJacp,
                              &combinedJacr,
                              &combinedJacpDot,
                              &combinedJacrDot,
                              &combinedLinearJdotQvel,
                              &combinedAngularJdotQvel);

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        assert(std::fabs(combinedLinearJdotQvel(row) - linearJdotQvel(row)) < 1.0e-12);
        assert(std::fabs(combinedAngularJdotQvel(row) - angularJdotQvel(row)) < 1.0e-12);
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            assert(std::fabs(combinedJacp(row, col) - jacp(row, col)) < 1.0e-12);
            assert(std::fabs(combinedJacr(row, col) - jacr(row, col)) < 1.0e-12);
            assert(std::fabs(combinedJacpDot(row, col) - jacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(combinedJacrDot(row, col) - jacrDot(row, col)) < 1.0e-12);
        }
    }

    ComputePointJacobianTerms(model,
                              data,
                              link2,
                              data.site_xpos[tool],
                              nullptr,
                              nullptr,
                              nullptr,
                              nullptr,
                              &combinedLinearJdotQvel,
                              &combinedAngularJdotQvel);
    for (dtDynIndex row = 0; row < 3; ++row)
    {
        assert(std::fabs(combinedLinearJdotQvel(row) - linearJdotQvel(row)) < 1.0e-12);
        assert(std::fabs(combinedAngularJdotQvel(row) - angularJdotQvel(row)) < 1.0e-12);
    }
}

void TestBodyAndSiteJacobianTermsMatchSeparateAPIs()
{
    dtDynSpec spec;
    const auto link1 = spec.AddBody("link1", 0);
    const auto link2 = spec.AddBody("link2", link1);
    const auto tool = spec.AddSite(link2, "tool");

    spec.Bodies()[link1].pos(0) = 0.5;
    spec.Bodies()[link1].inertial.mass = 1.0;
    spec.Bodies()[link1].inertial.pos(0) = 0.5;
    spec.Bodies()[link2].pos(0) = 1.0;
    spec.Bodies()[link2].inertial.mass = 0.75;
    spec.Bodies()[link2].inertial.pos(0) = 0.5;
    spec.Sites()[tool].pos(0) = 0.8;

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.Joints()[joint1].axis(2) = 1.0;
    spec.Joints()[joint2].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model);
    data.qpos(0) = 0.35;
    data.qpos(1) = -0.45;
    data.qvel(0) = 1.1;
    data.qvel(1) = -0.7;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    dtDynDenseMatrix bodyJacp;
    dtDynDenseMatrix bodyJacr;
    dtDynDenseMatrix bodyJacpDot;
    dtDynDenseMatrix bodyJacrDot;
    dt::Math::Vector3<dtDynReal> bodyLinearJdotQvel;
    dt::Math::Vector3<dtDynReal> bodyAngularJdotQvel;
    ComputeBodyJacobian(model, data, link2, bodyJacp, bodyJacr);
    ComputeBodyJacobianDot(model, data, link2, bodyJacpDot, bodyJacrDot);
    ComputeBodyJacobianDotTimesVelocity(model, data, link2,
                                        bodyLinearJdotQvel, bodyAngularJdotQvel);

    dtDynDenseMatrix combinedBodyJacp;
    dtDynDenseMatrix combinedBodyJacr;
    dtDynDenseMatrix combinedBodyJacpDot;
    dtDynDenseMatrix combinedBodyJacrDot;
    dt::Math::Vector3<dtDynReal> combinedBodyLinearJdotQvel;
    dt::Math::Vector3<dtDynReal> combinedBodyAngularJdotQvel;
    ComputeBodyJacobianTerms(model,
                             data,
                             link2,
                             &combinedBodyJacp,
                             &combinedBodyJacr,
                             &combinedBodyJacpDot,
                             &combinedBodyJacrDot,
                             &combinedBodyLinearJdotQvel,
                             &combinedBodyAngularJdotQvel);

    dtDynDenseMatrix siteJacp;
    dtDynDenseMatrix siteJacr;
    dtDynDenseMatrix siteJacpDot;
    dtDynDenseMatrix siteJacrDot;
    dt::Math::Vector3<dtDynReal> siteLinearJdotQvel;
    dt::Math::Vector3<dtDynReal> siteAngularJdotQvel;
    ComputeSiteJacobian(model, data, tool, siteJacp, siteJacr);
    ComputeSiteJacobianDot(model, data, tool, siteJacpDot, siteJacrDot);
    ComputeSiteJacobianDotTimesVelocity(model, data, tool,
                                        siteLinearJdotQvel, siteAngularJdotQvel);

    dtDynDenseMatrix combinedSiteJacp;
    dtDynDenseMatrix combinedSiteJacr;
    dtDynDenseMatrix combinedSiteJacpDot;
    dtDynDenseMatrix combinedSiteJacrDot;
    dt::Math::Vector3<dtDynReal> combinedSiteLinearJdotQvel;
    dt::Math::Vector3<dtDynReal> combinedSiteAngularJdotQvel;
    ComputeSiteJacobianTerms(model,
                             data,
                             tool,
                             &combinedSiteJacp,
                             &combinedSiteJacr,
                             &combinedSiteJacpDot,
                             &combinedSiteJacrDot,
                             &combinedSiteLinearJdotQvel,
                             &combinedSiteAngularJdotQvel);

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        assert(std::fabs(combinedBodyLinearJdotQvel(row) - bodyLinearJdotQvel(row)) < 1.0e-12);
        assert(std::fabs(combinedBodyAngularJdotQvel(row) - bodyAngularJdotQvel(row)) < 1.0e-12);
        assert(std::fabs(combinedSiteLinearJdotQvel(row) - siteLinearJdotQvel(row)) < 1.0e-12);
        assert(std::fabs(combinedSiteAngularJdotQvel(row) - siteAngularJdotQvel(row)) < 1.0e-12);
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            assert(std::fabs(combinedBodyJacp(row, col) - bodyJacp(row, col)) < 1.0e-12);
            assert(std::fabs(combinedBodyJacr(row, col) - bodyJacr(row, col)) < 1.0e-12);
            assert(std::fabs(combinedBodyJacpDot(row, col) - bodyJacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(combinedBodyJacrDot(row, col) - bodyJacrDot(row, col)) < 1.0e-12);
            assert(std::fabs(combinedSiteJacp(row, col) - siteJacp(row, col)) < 1.0e-12);
            assert(std::fabs(combinedSiteJacr(row, col) - siteJacr(row, col)) < 1.0e-12);
            assert(std::fabs(combinedSiteJacpDot(row, col) - siteJacpDot(row, col)) < 1.0e-12);
            assert(std::fabs(combinedSiteJacrDot(row, col) - siteJacrDot(row, col)) < 1.0e-12);
        }
    }
}

void TestMassMatrixGravityAndSubtreeComForSingleHinge()
{
    dtDynSpec spec;
    const auto link = spec.AddBody("link", 0);
    spec.Bodies()[link].inertial.mass = 2.0;
    spec.Bodies()[link].inertial.pos(0) = 1.0;
    spec.Bodies()[link].inertial.inertia(2) = 0.5;

    const auto hinge = spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);
    spec.Joints()[hinge].axis(2) = 1.0;
    spec.Joints()[hinge].armature = 0.25;

    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model);
    data.qpos(0) = 0.0;
    data.qvel(0) = 0.0;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = -9.81;
    gravity(2) = 0.0;

    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    assert(std::fabs(data.Mmat(0, 0) - 2.75) < 1.0e-9);
    assert(std::fabs(data.gravityTorq(0) - 19.62) < 1.0e-9);
    assert(std::fabs(data.coriolisTorq(0)) < 1.0e-12);
    assert(std::fabs(data.nonlinTorq(0) - data.gravityTorq(0)) < 1.0e-12);
    assert(std::fabs(data.subtree_com[link](0) - 1.0) < 1.0e-12);
    assert(std::fabs(data.subtree_com[0](0) - 1.0) < 1.0e-12);
}

void TestCrbaMassMatrixAndIntermediatesForTwoLinkChain()
{
    dtDynSpec spec;
    const auto link1 = spec.AddBody("link1", 0);
    const auto link2 = spec.AddBody("link2", link1);

    spec.Bodies()[link1].inertial.mass = 1.0;
    spec.Bodies()[link1].inertial.pos(0) = 1.0;
    spec.Bodies()[link2].pos(0) = 1.0;
    spec.Bodies()[link2].inertial.mass = 1.0;
    spec.Bodies()[link2].inertial.pos(0) = 1.0;

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.Joints()[joint1].axis(2) = 1.0;
    spec.Joints()[joint2].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model);

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();

    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    assert(std::fabs(data.Mmat(0, 0) - 5.0) < 1.0e-9);
    assert(std::fabs(data.Mmat(0, 1) - 2.0) < 1.0e-9);
    assert(std::fabs(data.Mmat(1, 0) - 2.0) < 1.0e-9);
    assert(std::fabs(data.Mmat(1, 1) - 1.0) < 1.0e-9);

    assert(std::fabs(data.cdof[0](2) - 1.0) < 1.0e-12);
    assert(std::fabs(data.cdof[0](3)) < 1.0e-12);
    assert(std::fabs(data.cdof[0](4) - 1.5) < 1.0e-12);
    assert(std::fabs(data.cdof[1](2) - 1.0) < 1.0e-12);
    assert(std::fabs(data.cdof[1](3)) < 1.0e-12);
    assert(std::fabs(data.cdof[1](4) - 0.5) < 1.0e-12);

    assert(std::fabs(data.crb[link2](2, 2) - 0.25) < 1.0e-9);
    assert(std::fabs(data.crb[link1](2, 2) - 0.5) < 1.0e-9);

    data.qvel(0) = 2.0;
    data.qvel(1) = 3.0;
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    assert(std::fabs(data.comJmat(0, 0)) < 1.0e-12);
    assert(std::fabs(data.comJmat(1, 0) - 1.5) < 1.0e-12);
    assert(std::fabs(data.comJmat(1, 1) - 0.5) < 1.0e-12);
    assert(std::fabs(data.cmAmat(1, 0) - 3.0) < 1.0e-12);
    assert(std::fabs(data.cmAmat(1, 1) - 1.0) < 1.0e-12);
    assert(std::fabs(data.cmAmat(5, 0) - 0.5) < 1.0e-12);
    assert(std::fabs(data.cmAmat(5, 1) - 0.5) < 1.0e-12);
    assert(std::fabs(data.centMom(1) - 9.0) < 1.0e-12);
    assert(std::fabs(data.centMom(5) - 2.5) < 1.0e-12);

    data.qpos(0) = 0.0;
    data.qpos(1) = 0.5 * 3.14159265358979323846;
    data.qvel(0) = 2.0;
    data.qvel(1) = 3.0;
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    assert(std::fabs(data.Mmat(0, 0) - 3.0) < 1.0e-6);
    assert(std::fabs(data.Mmat(0, 1) - 1.0) < 1.0e-6);
    assert(std::fabs(data.Mmat(1, 1) - 1.0) < 1.0e-6);
    assert(std::fabs(data.Cmat(0, 0) + 3.0) < 1.0e-4);
    assert(std::fabs(data.Cmat(0, 1) + 5.0) < 1.0e-4);
    assert(std::fabs(data.Cmat(1, 0) - 2.0) < 1.0e-4);
    assert(std::fabs(data.Cmat(1, 1)) < 1.0e-4);
    assert(std::fabs(data.MmatDot(0, 0) + 6.0) < 1.0e-4);
    assert(std::fabs(data.MmatDot(0, 1) + 3.0) < 1.0e-4);
    assert(std::fabs(data.MmatDot(1, 0) + 3.0) < 1.0e-4);
    assert(std::fabs(data.MmatDot(1, 1)) < 1.0e-4);
    assert(std::fabs(data.cmAmatDot(0, 0)) + std::fabs(data.cmAmatDot(1, 0)) +
               std::fabs(data.cmAmatDot(0, 1)) + std::fabs(data.cmAmatDot(1, 1)) >
           1.0e-6);
    assert(std::fabs(data.coriolisTorq(0) + 21.0) < 1.0e-4);
    assert(std::fabs(data.coriolisTorq(1) - 4.0) < 1.0e-4);
    assert(std::fabs(data.nonlinTorq(0) - data.coriolisTorq(0)) < 1.0e-8);
    assert(std::fabs(data.nonlinTorq(1) - data.coriolisTorq(1)) < 1.0e-8);
}

void TestDynamicsRNEAMatchesCCRBAForBaseTerms()
{
    dtDynSpec spec;
    const auto link1 = spec.AddBody("link1", 0);
    const auto link2 = spec.AddBody("link2", link1);

    spec.Bodies()[link1].inertial.mass = 1.4;
    spec.Bodies()[link1].inertial.pos(0) = 0.4;
    spec.Bodies()[link1].inertial.inertia(0) = 0.02;
    spec.Bodies()[link1].inertial.inertia(1) = 0.03;
    spec.Bodies()[link1].inertial.inertia(2) = 0.04;
    spec.Bodies()[link2].pos(0) = 0.8;
    spec.Bodies()[link2].pos(2) = -0.1;
    spec.Bodies()[link2].inertial.mass = 0.9;
    spec.Bodies()[link2].inertial.pos(0) = 0.35;
    spec.Bodies()[link2].inertial.pos(2) = -0.05;
    spec.Bodies()[link2].inertial.inertia(0) = 0.015;
    spec.Bodies()[link2].inertial.inertia(1) = 0.02;
    spec.Bodies()[link2].inertial.inertia(2) = 0.025;

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_y", dtDynJointType::Hinge);
    spec.Joints()[joint1].axis(2) = 1.0;
    spec.Joints()[joint2].axis(1) = 1.0;

    const dtDynModel model = CompileSpec(spec);

    dtDynData ccrba;
    dtDynData rnea;
    ccrba.Resize(model);
    rnea.Resize(model);
    ccrba.qpos(0) = 0.35;
    ccrba.qpos(1) = -0.42;
    ccrba.qvel(0) = 1.2;
    ccrba.qvel(1) = -0.8;
    rnea.qpos = ccrba.qpos;
    rnea.qvel = ccrba.qvel;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.2;
    gravity(1) = -9.81;
    gravity(2) = 0.4;

    ComputeDynamicsCCRBA(model, ccrba, gravity);
    ComputeDynamicsRNEA(model, rnea, gravity);

    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        assert(std::fabs(rnea.gravityTorq(dofId) - ccrba.gravityTorq(dofId)) < 1.0e-8);
        assert(std::fabs(rnea.coriolisTorq(dofId) - ccrba.coriolisTorq(dofId)) < 1.0e-8);
        assert(std::fabs(rnea.nonlinTorq(dofId) - ccrba.nonlinTorq(dofId)) < 1.0e-8);
        assert(std::fabs(rnea.nonlinTorq(dofId) -
                         rnea.gravityTorq(dofId) -
                         rnea.coriolisTorq(dofId)) < 1.0e-10);
        for (dtDynIndex col = 0; col < model.nv; ++col)
        {
            assert(std::fabs(rnea.Mmat(dofId, col) - ccrba.Mmat(dofId, col)) < 1.0e-8);
        }
    }

    assert(std::fabs(ccrba.Cmat(0, 0)) + std::fabs(ccrba.Cmat(0, 1)) +
               std::fabs(ccrba.Cmat(1, 0)) + std::fabs(ccrba.Cmat(1, 1)) >
           1.0e-8);
    assert(std::fabs(rnea.Cmat(0, 0)) + std::fabs(rnea.Cmat(0, 1)) +
               std::fabs(rnea.Cmat(1, 0)) + std::fabs(rnea.Cmat(1, 1)) <
           1.0e-12);
}

void TestDynamicsCCRBAComputesOnlyRequestedExtraTerms()
{
    dtDynSpec spec;
    const auto link1 = spec.AddBody("link1", 0);
    const auto link2 = spec.AddBody("link2", link1);

    spec.Bodies()[link1].inertial.mass = 1.0;
    spec.Bodies()[link1].inertial.pos(0) = 1.0;
    spec.Bodies()[link2].pos(0) = 1.0;
    spec.Bodies()[link2].inertial.mass = 1.0;
    spec.Bodies()[link2].inertial.pos(0) = 1.0;

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.Joints()[joint1].axis(2) = 1.0;
    spec.Joints()[joint2].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);
    dtDynData data;
    data.Resize(model);
    data.qpos(1) = 0.5 * 3.14159265358979323846;
    data.qvel(0) = 2.0;
    data.qvel(1) = 3.0;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();

    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_CMAMAT | DTDYN_CCRBA_TERM_MMAT_DOT);

    assert(std::fabs(data.Mmat(0, 0) - 3.0) < 1.0e-6);
    assert(std::fabs(data.Cmat(0, 0) + 3.0) < 1.0e-4);
    assert(std::fabs(data.coriolisTorq(0) + 21.0) < 1.0e-4);
    assert(std::fabs(data.MmatDot(0, 0) + 6.0) < 1.0e-4);
    assert(std::fabs(data.cmAmat(1, 0) - 2.0) < 1.0e-9);
    assert(std::fabs(data.cmAmatDot(0, 0)) + std::fabs(data.cmAmatDot(1, 0)) +
               std::fabs(data.cmAmatDot(0, 1)) + std::fabs(data.cmAmatDot(1, 1)) <
           1.0e-12);
    assert(std::fabs(data.centMom(0)) + std::fabs(data.centMom(1)) +
               std::fabs(data.centMom(2)) + std::fabs(data.centMom(3)) +
               std::fabs(data.centMom(4)) + std::fabs(data.centMom(5)) <
           1.0e-12);
    assert(std::fabs(data.comJmat(0, 0)) + std::fabs(data.comJmat(1, 0)) +
               std::fabs(data.comJmat(0, 1)) + std::fabs(data.comJmat(1, 1)) <
           1.0e-12);
}

void TestDynamicsRNEAComputesOnlyRneaRequestedTerms()
{
    dtDynSpec spec;
    const auto link = spec.AddBody("link", 0);
    spec.Bodies()[link].inertial.mass = 2.0;
    spec.Bodies()[link].inertial.pos(0) = 0.7;
    spec.Bodies()[link].inertial.inertia(2) = 0.2;

    const auto hinge = spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);
    spec.Joints()[hinge].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);
    dtDynData data;
    data.Resize(model);
    data.qpos(0) = 0.25;
    data.qvel(0) = 0.6;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = -9.81;
    gravity(2) = 0.0;

    ComputeDynamicsRNEA(model, data, gravity, DTDYN_RNEA_TERM_COM_POS);

    assert(std::fabs(data.Mmat(0, 0) - (2.0 * 0.7 * 0.7 + 0.2)) < 1.0e-12);
    assert(std::fabs(data.comPos(0)) + std::fabs(data.comPos(1)) + std::fabs(data.comPos(2)) >
           1.0e-12);
    assert(std::fabs(data.Cmat(0, 0)) < 1.0e-12);
    assert(std::fabs(data.MmatDot(0, 0)) < 1.0e-12);
    assert(std::fabs(data.cmAmat(0, 0)) + std::fabs(data.cmAmat(1, 0)) +
               std::fabs(data.cmAmat(2, 0)) + std::fabs(data.cmAmat(3, 0)) +
               std::fabs(data.cmAmat(4, 0)) + std::fabs(data.cmAmat(5, 0)) <
           1.0e-12);
    assert(std::fabs(data.centMom(0)) + std::fabs(data.centMom(1)) +
               std::fabs(data.centMom(2)) + std::fabs(data.centMom(3)) +
               std::fabs(data.centMom(4)) + std::fabs(data.centMom(5)) <
           1.0e-12);
    assert(std::fabs(data.nonlinTorq(0) -
                     data.gravityTorq(0) -
                     data.coriolisTorq(0)) < 1.0e-10);
}

void TestDynamicsCCRBAAllTermsIsRepeatable()
{
    dtDynSpec spec;
    const auto link = spec.AddBody("link", 0);
    spec.Bodies()[link].inertial.mass = 2.0;
    spec.Bodies()[link].inertial.pos(0) = 0.7;
    spec.Bodies()[link].inertial.inertia(2) = 0.2;
    const auto hinge = spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);
    spec.Joints()[hinge].axis(2) = 1.0;

    const dtDynModel model = CompileSpec(spec);
    dtDynData firstData;
    dtDynData ccrbaData;
    firstData.Resize(model);
    ccrbaData.Resize(model);
    firstData.qpos(0) = 0.25;
    firstData.qvel(0) = 0.6;
    ccrbaData.qpos = firstData.qpos;
    ccrbaData.qvel = firstData.qvel;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = -9.81;
    gravity(2) = 0.0;

    ComputeDynamicsCCRBA(model, firstData, gravity, DTDYN_CCRBA_TERM_ALL);
    ComputeDynamicsCCRBA(model, ccrbaData, gravity, DTDYN_CCRBA_TERM_ALL);

    assert(std::fabs(firstData.Mmat(0, 0) - ccrbaData.Mmat(0, 0)) < 1.0e-12);
    assert(std::fabs(firstData.Cmat(0, 0) - ccrbaData.Cmat(0, 0)) < 1.0e-12);
    assert(std::fabs(firstData.gravityTorq(0) - ccrbaData.gravityTorq(0)) < 1.0e-12);
    assert(std::fabs(firstData.coriolisTorq(0) - ccrbaData.coriolisTorq(0)) < 1.0e-12);
    assert(std::fabs(firstData.nonlinTorq(0) - ccrbaData.nonlinTorq(0)) < 1.0e-12);
    assert(std::fabs(firstData.cmAmat(1, 0) - ccrbaData.cmAmat(1, 0)) < 1.0e-12);
}

} // namespace

int main()
{
    TestJointSizes();
    TestDtArrayDirectHeader();
    TestSpecWorldAndFixedBody();
    TestSpecXmlDecodeEncodeRoundTrip();
    TestSpecXmlDecodesFreeJointAndFixedBody();
    TestSpecXmlSkipsCommentsAndInfersRangeLimits();
    TestSpecXmlRejectsEqualityUntilConstraintLayerExists();
    TestModelFixedBodyConvention();
    TestCompiledModelCopiesIntoDefaultModel();
    TestCompiledModelCopiesZeroSizedOptionalDenseFields();
    TestCompileSpecCanFillExistingModel();
    TestDtMathDynamicResizeHelpers();
    TestDataResizeSeparatesBodyAndDofStorage();
    TestDtDynMathHelpersAreReusable();
    TestCompileSpecUsesNativeIndexLayout();
    TestCompileSpecSupportsMultipleMovableRoots();
    TestCompileSpecSupportsFloatingBaseQuadrupedTree();
    TestForwardKinematicsAndPointJacobianForFixedChain();
    TestIndexedJacobiansOnlyRefreshRequestedIds();
    TestIndexedJacobiansRefreshFloatingBaseBodyAndTaskSite();
    TestPointJacobianDotMatchesFiniteDifferenceForMaterialSite();
    TestPointJacobianTermsMatchesSeparateAPIs();
    TestBodyAndSiteJacobianTermsMatchSeparateAPIs();
    TestMassMatrixGravityAndSubtreeComForSingleHinge();
    TestCrbaMassMatrixAndIntermediatesForTwoLinkChain();
    TestDynamicsRNEAMatchesCCRBAForBaseTerms();
    TestDynamicsCCRBAComputesOnlyRequestedExtraTerms();
    TestDynamicsRNEAComputesOnlyRneaRequestedTerms();
    TestDynamicsCCRBAAllTermsIsRepeatable();
    return 0;
}
