#include <dtDynamics/dtDynCompile.h>
#include <dtDynamics/dtDynData.h>
#include <dtDynamics/dtDynDynamics.h>
#include <dtDynamics/dtDynKinematics.h>
#include <dtDynamics/dtDynSpec.h>
#include <dtDynamics/dtDynUtils.h>

#include "dtDynDynamicsInternal.h"
#include "../dtDynBenchmarkMetadata.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace dt::Dynamics;

namespace
{

std::atomic<bool> g_countAllocations(false);
std::atomic<unsigned long long> g_allocationCount(0);

void CountAllocation()
{
    if (g_countAllocations.load(std::memory_order_relaxed))
    {
        g_allocationCount.fetch_add(1, std::memory_order_relaxed);
    }
}

void *Allocate(const std::size_t size)
{
    CountAllocation();
    if (void *ptr = std::malloc(size == 0 ? 1 : size))
    {
        return ptr;
    }
    throw std::bad_alloc();
}

void *AllocateAligned(const std::size_t size, const std::size_t alignment)
{
    CountAllocation();
    const std::size_t allocationSize = size == 0 ? alignment : ((size + alignment - 1) / alignment) * alignment;
    if (void *ptr = std::aligned_alloc(alignment, allocationSize))
    {
        return ptr;
    }
    throw std::bad_alloc();
}

} // namespace

void *operator new(std::size_t size)
{
    return Allocate(size);
}

void *operator new[](std::size_t size)
{
    return Allocate(size);
}

void *operator new(std::size_t size, std::align_val_t alignment)
{
    return AllocateAligned(size, static_cast<std::size_t>(alignment));
}

void *operator new[](std::size_t size, std::align_val_t alignment)
{
    return AllocateAligned(size, static_cast<std::size_t>(alignment));
}

void operator delete(void *ptr) noexcept
{
    std::free(ptr);
}

void operator delete[](void *ptr) noexcept
{
    std::free(ptr);
}

void operator delete(void *ptr, std::size_t) noexcept
{
    std::free(ptr);
}

void operator delete[](void *ptr, std::size_t) noexcept
{
    std::free(ptr);
}

void operator delete(void *ptr, std::align_val_t) noexcept
{
    std::free(ptr);
}

void operator delete[](void *ptr, std::align_val_t) noexcept
{
    std::free(ptr);
}

void operator delete(void *ptr, std::size_t, std::align_val_t) noexcept
{
    std::free(ptr);
}

void operator delete[](void *ptr, std::size_t, std::align_val_t) noexcept
{
    std::free(ptr);
}

namespace
{

struct Scenario
{
    std::string fixture;
    std::string state;
    dtDynSpec spec;
};

struct Stage
{
    const char *name = nullptr;
    void (*prepare)(const dtDynModel &, dtDynData &, const dt::Math::Vector3<dtDynReal> &) = nullptr;
    void (*compute)(const dtDynModel &,
                    dtDynData &,
                    const dt::Math::Vector3<dtDynReal> &,
                    const dtArray<dtDynIndex> &,
                    const dtArray<dtDynIndex> &) = nullptr;
};

struct Result
{
    std::string fixture;
    std::string state;
    std::string stage;
    dtDynIndex nq = 0;
    dtDynIndex nv = 0;
    dtDynIndex nbody = 0;
    dtDynIndex nsite = 0;
    int iterations = 0;
    double meanUs = 0.0;
    double medianUs = 0.0;
    double p99Us = 0.0;
    double allocationMean = 0.0;
    unsigned long long allocationMax = 0;
};

constexpr dtDynCcrbaTermFlags kLeoQuadWbcCcrbaTermFlags =
    DTDYN_CCRBA_TERM_COM_POS |
    DTDYN_CCRBA_TERM_CMAMAT |
    DTDYN_CCRBA_TERM_CENT_MOM;

std::string ReadTextFile(const std::string &path)
{
    std::ifstream input(path);
    if (!input)
    {
        throw std::runtime_error("benchmark_dtDynamicsStages: failed to open XML fixture: " + path);
    }

    std::ostringstream stream;
    stream << input.rdbuf();
    return stream.str();
}

bool EnvEnabled(const char *name)
{
    const char *value = std::getenv(name);
    if (value == nullptr)
    {
        return false;
    }
    return value[0] == '1' || value[0] == 't' || value[0] == 'T' ||
           value[0] == 'y' || value[0] == 'Y';
}

dtDynIndex AddHingeLink(dtDynSpec &spec,
                        const dtDynIndex parent,
                        const std::string &name,
                        const dtDynReal x,
                        const dtDynReal y,
                        const dtDynReal z,
                        const dtDynReal mass,
                        const dtDynReal axisX,
                        const dtDynReal axisY,
                        const dtDynReal axisZ)
{
    const dtDynIndex body = spec.AddBody(name, parent);
    spec.Bodies()[body].pos(0) = x;
    spec.Bodies()[body].pos(1) = y;
    spec.Bodies()[body].pos(2) = z;
    spec.Bodies()[body].inertial.mass = mass;
    spec.Bodies()[body].inertial.pos(0) = 0.5 * x;
    spec.Bodies()[body].inertial.pos(1) = 0.5 * y;
    spec.Bodies()[body].inertial.pos(2) = 0.5 * z;
    spec.Bodies()[body].inertial.inertia(0) = 0.01 * mass + 0.001;
    spec.Bodies()[body].inertial.inertia(1) = 0.012 * mass + 0.001;
    spec.Bodies()[body].inertial.inertia(2) = 0.014 * mass + 0.001;

    const dtDynIndex joint = spec.AddJoint(body, name + "_joint", dtDynJointType::Hinge);
    spec.Joints()[joint].axis(0) = axisX;
    spec.Joints()[joint].axis(1) = axisY;
    spec.Joints()[joint].axis(2) = axisZ;
    spec.Joints()[joint].armature = 0.001 * (static_cast<dtDynReal>(joint) + 1.0);
    return body;
}

dtDynSpec SingleManipulatorSpec()
{
    dtDynSpec spec;
    const dtDynIndex base = spec.AddBody("base", 0);
    spec.Bodies()[base].inertial.mass = 3.0;
    spec.Bodies()[base].inertial.inertia(0) = 0.05;
    spec.Bodies()[base].inertial.inertia(1) = 0.05;
    spec.Bodies()[base].inertial.inertia(2) = 0.05;

    dtDynIndex parent = base;
    parent = AddHingeLink(spec, parent, "link1", 0.30, 0.00, 0.05, 1.5, 0.0, 0.0, 1.0);
    parent = AddHingeLink(spec, parent, "link2", 0.25, 0.00, 0.00, 1.0, 0.0, 1.0, 0.0);
    parent = AddHingeLink(spec, parent, "link3", 0.20, 0.00, 0.00, 0.8, 1.0, 0.0, 0.0);
    spec.AddSite(parent, "tool");
    spec.Sites()[0].pos(0) = 0.2;
    return spec;
}

dtDynSpec DualManipulatorSpec()
{
    dtDynSpec spec;
    const dtDynIndex base = spec.AddBody("base", 0);
    spec.Bodies()[base].inertial.mass = 5.0;
    spec.Bodies()[base].inertial.inertia(0) = 0.08;
    spec.Bodies()[base].inertial.inertia(1) = 0.08;
    spec.Bodies()[base].inertial.inertia(2) = 0.10;

    const dtDynIndex leftShoulder = AddHingeLink(spec, base, "left_link1", 0.10, 0.25, 0.0, 1.0, 0.0, 0.0, 1.0);
    const dtDynIndex leftElbow = AddHingeLink(spec, leftShoulder, "left_link2", 0.25, 0.0, 0.0, 0.7, 0.0, 1.0, 0.0);
    const dtDynIndex rightShoulder = AddHingeLink(spec, base, "right_link1", 0.10, -0.25, 0.0, 1.0, 0.0, 0.0, 1.0);
    const dtDynIndex rightElbow = AddHingeLink(spec, rightShoulder, "right_link2", 0.25, 0.0, 0.0, 0.7, 0.0, 1.0, 0.0);

    const dtDynIndex leftTool = spec.AddSite(leftElbow, "left_tool");
    const dtDynIndex rightTool = spec.AddSite(rightElbow, "right_tool");
    spec.Sites()[leftTool].pos(0) = 0.2;
    spec.Sites()[rightTool].pos(0) = 0.2;
    return spec;
}

dtDynSpec MultiRobotSpec()
{
    dtDynSpec spec;

    const dtDynIndex robotABase = spec.AddBody("robot_a_base", 0);
    spec.Bodies()[robotABase].pos(0) = 0.4;
    spec.Bodies()[robotABase].pos(1) = 0.25;
    spec.Bodies()[robotABase].inertial.mass = 2.0;
    spec.Bodies()[robotABase].inertial.inertia(0) = 0.04;
    spec.Bodies()[robotABase].inertial.inertia(1) = 0.05;
    spec.Bodies()[robotABase].inertial.inertia(2) = 0.06;

    const dtDynIndex robotAShoulder =
        AddHingeLink(spec, robotABase, "robot_a_shoulder", 0.10, 0.0, 0.20, 0.8, 0.0, 0.0, 1.0);
    const dtDynIndex robotAElbow =
        AddHingeLink(spec, robotAShoulder, "robot_a_elbow", 0.25, 0.0, 0.0, 0.6, 0.0, 1.0, 0.0);
    const dtDynIndex robotATool = spec.AddSite(robotAElbow, "robot_a_tool");
    spec.Sites()[robotATool].pos(0) = 0.2;

    const dtDynIndex robotBBase = spec.AddBody("robot_b_base", 0);
    spec.Bodies()[robotBBase].pos(0) = -0.35;
    spec.Bodies()[robotBBase].pos(1) = -0.2;
    spec.Bodies()[robotBBase].pos(2) = 0.05;
    spec.Bodies()[robotBBase].inertial.mass = 1.5;
    spec.Bodies()[robotBBase].inertial.inertia(0) = 0.03;
    spec.Bodies()[robotBBase].inertial.inertia(1) = 0.035;
    spec.Bodies()[robotBBase].inertial.inertia(2) = 0.04;

    const dtDynIndex sensorFrame = spec.AddBody("robot_b_sensor_frame", robotBBase);
    spec.Bodies()[sensorFrame].pos(0) = 0.02;
    spec.Bodies()[sensorFrame].pos(1) = 0.03;
    spec.Bodies()[sensorFrame].pos(2) = 0.04;
    const dtDynIndex sensorSite = spec.AddSite(sensorFrame, "robot_b_sensor");
    spec.Sites()[sensorSite].pos(0) = 0.01;
    spec.Sites()[sensorSite].pos(1) = 0.02;
    spec.Sites()[sensorSite].pos(2) = 0.03;

    const dtDynIndex robotBHip =
        AddHingeLink(spec, sensorFrame, "robot_b_hip", 0.0, 0.0, 0.18, 0.7, 1.0, 0.0, 0.0);
    const dtDynIndex robotBKnee =
        AddHingeLink(spec, robotBHip, "robot_b_knee", 0.22, 0.0, 0.0, 0.45, 0.0, 1.0, 0.0);
    const dtDynIndex robotBTool = spec.AddSite(robotBKnee, "robot_b_tool");
    spec.Sites()[robotBTool].pos(0) = 0.18;

    const dtDynIndex robotAElbowMotor =
        spec.AddActuator(spec.Bodies()[robotAElbow].joints[0], "robot_a_elbow_motor");
    spec.Actuators()[robotAElbowMotor].gear = 90.0;
    const dtDynIndex robotBKneeMotor =
        spec.AddActuator(spec.Bodies()[robotBKnee].joints[0], "robot_b_knee_motor");
    spec.Actuators()[robotBKneeMotor].gear = 70.0;

    return spec;
}

dtDynSpec MultiRobotFloatingSpec()
{
    dtDynSpec spec;

    const dtDynIndex robotABase = spec.AddBody("robot_a_floating_base", 0);
    spec.Bodies()[robotABase].pos(0) = 0.4;
    spec.Bodies()[robotABase].pos(1) = 0.25;
    spec.Bodies()[robotABase].pos(2) = 0.6;
    spec.Bodies()[robotABase].inertial.mass = 3.0;
    spec.Bodies()[robotABase].inertial.inertia(0) = 0.08;
    spec.Bodies()[robotABase].inertial.inertia(1) = 0.09;
    spec.Bodies()[robotABase].inertial.inertia(2) = 0.10;
    spec.AddJoint(robotABase, "robot_a_free", dtDynJointType::Free);

    const dtDynIndex robotAShoulder =
        AddHingeLink(spec, robotABase, "robot_a_floating_shoulder", 0.15, 0.0, -0.10, 0.8, 0.0, 0.0, 1.0);
    const dtDynIndex robotAElbow =
        AddHingeLink(spec, robotAShoulder, "robot_a_floating_elbow", 0.25, 0.0, 0.0, 0.6, 0.0, 1.0, 0.0);
    const dtDynIndex robotATool = spec.AddSite(robotAElbow, "robot_a_floating_tool");
    spec.Sites()[robotATool].pos(0) = 0.2;

    const dtDynIndex robotBBase = spec.AddBody("robot_b_floating_base", 0);
    spec.Bodies()[robotBBase].pos(0) = -0.35;
    spec.Bodies()[robotBBase].pos(1) = -0.20;
    spec.Bodies()[robotBBase].pos(2) = 0.70;
    spec.Bodies()[robotBBase].inertial.mass = 2.5;
    spec.Bodies()[robotBBase].inertial.inertia(0) = 0.07;
    spec.Bodies()[robotBBase].inertial.inertia(1) = 0.08;
    spec.Bodies()[robotBBase].inertial.inertia(2) = 0.09;
    spec.AddJoint(robotBBase, "robot_b_free", dtDynJointType::Free);

    const dtDynIndex sensorFrame = spec.AddBody("robot_b_floating_sensor_frame", robotBBase);
    spec.Bodies()[sensorFrame].pos(0) = 0.02;
    spec.Bodies()[sensorFrame].pos(1) = 0.03;
    spec.Bodies()[sensorFrame].pos(2) = 0.04;
    const dtDynIndex sensorSite = spec.AddSite(sensorFrame, "robot_b_floating_sensor");
    spec.Sites()[sensorSite].pos(0) = 0.01;
    spec.Sites()[sensorSite].pos(1) = 0.02;
    spec.Sites()[sensorSite].pos(2) = 0.03;

    const dtDynIndex robotBHip =
        AddHingeLink(spec, sensorFrame, "robot_b_floating_hip", 0.0, 0.0, -0.18, 0.7, 1.0, 0.0, 0.0);
    const dtDynIndex robotBKnee =
        AddHingeLink(spec, robotBHip, "robot_b_floating_knee", 0.22, 0.0, 0.0, 0.45, 0.0, 1.0, 0.0);
    const dtDynIndex robotBTool = spec.AddSite(robotBKnee, "robot_b_floating_tool");
    spec.Sites()[robotBTool].pos(0) = 0.18;

    const dtDynIndex robotAElbowMotor =
        spec.AddActuator(spec.Bodies()[robotAElbow].joints[0], "robot_a_floating_elbow_motor");
    spec.Actuators()[robotAElbowMotor].gear = 85.0;
    const dtDynIndex robotBKneeMotor =
        spec.AddActuator(spec.Bodies()[robotBKnee].joints[0], "robot_b_floating_knee_motor");
    spec.Actuators()[robotBKneeMotor].gear = 65.0;

    return spec;
}

dtDynSpec FloatingQuadrupedSpec()
{
    dtDynSpec spec;
    const dtDynIndex trunk = spec.AddBody("trunk", 0);
    spec.Bodies()[trunk].inertial.mass = 10.0;
    spec.Bodies()[trunk].inertial.inertia(0) = 0.20;
    spec.Bodies()[trunk].inertial.inertia(1) = 0.30;
    spec.Bodies()[trunk].inertial.inertia(2) = 0.40;
    spec.AddJoint(trunk, "floating_base", dtDynJointType::Free);

    const char *legNames[4] = {"front_left", "front_right", "rear_left", "rear_right"};
    const dtDynReal xOffsets[4] = {0.35, 0.35, -0.35, -0.35};
    const dtDynReal yOffsets[4] = {0.22, -0.22, 0.22, -0.22};
    for (dtDynIndex leg = 0; leg < 4; ++leg)
    {
        const std::string prefix = legNames[leg];
        const dtDynIndex hip = AddHingeLink(spec, trunk, prefix + "_hip", xOffsets[leg], yOffsets[leg], -0.05, 1.0, 1.0, 0.0, 0.0);
        const dtDynIndex thigh = AddHingeLink(spec, hip, prefix + "_thigh", 0.0, 0.0, -0.25, 0.8, 0.0, 1.0, 0.0);
        const dtDynIndex shank = AddHingeLink(spec, thigh, prefix + "_shank", 0.0, 0.0, -0.25, 0.5, 0.0, 1.0, 0.0);
        const dtDynIndex foot = spec.AddSite(shank, prefix + "_foot");
        spec.Sites()[foot].pos(2) = -0.25;
    }
    return spec;
}

dtDynSpec FixedBodyHeavySpec()
{
    dtDynSpec spec;
    const dtDynIndex base = spec.AddBody("base", 0);
    spec.Bodies()[base].inertial.mass = 4.0;
    dtDynIndex parent = base;
    for (dtDynIndex i = 0; i < 8; ++i)
    {
        const dtDynIndex mount = spec.AddBody("fixed_mount_" + std::to_string(i), parent);
        spec.Bodies()[mount].pos(0) = 0.03 * static_cast<dtDynReal>(i + 1);
        spec.Bodies()[mount].pos(1) = 0.02 * static_cast<dtDynReal>((i % 2) == 0 ? 1 : -1);
        spec.Bodies()[mount].inertial.mass = 0.05;
        parent = mount;
    }
    const dtDynIndex jointed = AddHingeLink(spec, parent, "jointed_link", 0.25, 0.0, 0.0, 1.2, 0.0, 0.0, 1.0);
    spec.AddSite(jointed, "tool");
    spec.Sites()[0].pos(0) = 0.15;
    return spec;
}

void SetJointQuaternion(dtDynDenseVector &qpos,
                        const dtDynIndex qadr,
                        const dtDynReal angle,
                        const dtDynReal axisSelector)
{
    qpos(qadr + 0) = std::cos(static_cast<dtDynReal>(0.5) * angle);
    qpos(qadr + 1) = std::sin(static_cast<dtDynReal>(0.5) * angle) * (axisSelector < 0.0 ? 1.0 : 0.0);
    qpos(qadr + 2) = std::sin(static_cast<dtDynReal>(0.5) * angle) * (axisSelector == 0.0 ? 1.0 : 0.0);
    qpos(qadr + 3) = std::sin(static_cast<dtDynReal>(0.5) * angle) * (axisSelector > 0.0 ? 1.0 : 0.0);
}

void SetState(const dtDynModel &model, const std::string &state, dtDynData &data)
{
    for (dtDynIndex i = 0; i < data.qpos.GetDim(); ++i)
    {
        data.qpos(i) = 0.0;
    }
    for (dtDynIndex i = 0; i < data.qvel.GetDim(); ++i)
    {
        data.qvel(i) = 0.0;
    }

    for (dtDynIndex jointId = 0; jointId < model.njnt; ++jointId)
    {
        const dtDynJointType jointType =
            static_cast<dtDynJointType>(model.jnt_type[static_cast<std::size_t>(jointId)]);
        const dtDynIndex qadr = model.jnt_qposadr[static_cast<std::size_t>(jointId)];
        const dtDynIndex dadr = model.jnt_dofadr[static_cast<std::size_t>(jointId)];
        const dtDynIndex bodyId = model.jnt_bodyid[static_cast<std::size_t>(jointId)];
        const dtDynReal scale = static_cast<dtDynReal>(jointId + 1);

        if (jointType == dtDynJointType::Free)
        {
            for (dtDynIndex axis = 0; axis < 3; ++axis)
            {
                data.qpos(qadr + axis) = model.body_pos[static_cast<std::size_t>(bodyId)](axis);
            }
            dtDynReal angle = 0.0;
            if (state == "asymmetric")
            {
                angle = static_cast<dtDynReal>(0.04) * scale;
                data.qpos(qadr + 0) += static_cast<dtDynReal>(0.03) * scale;
                data.qpos(qadr + 1) -= static_cast<dtDynReal>(0.02) * scale;
                data.qpos(qadr + 2) += static_cast<dtDynReal>(0.01) * scale;
            }
            else if (state == "nonzero_velocity")
            {
                angle = static_cast<dtDynReal>(0.03) * scale;
                data.qpos(qadr + 0) += static_cast<dtDynReal>(0.02) * std::cos(static_cast<dtDynReal>(0.4) * scale);
                data.qpos(qadr + 1) += static_cast<dtDynReal>(0.02) * std::sin(static_cast<dtDynReal>(0.5) * scale);
                data.qpos(qadr + 2) -= static_cast<dtDynReal>(0.01) * scale;
                for (dtDynIndex offset = 0; offset < 6; ++offset)
                {
                    data.qvel(dadr + offset) =
                        static_cast<dtDynReal>(0.08) *
                        std::sin(static_cast<dtDynReal>(0.3) * static_cast<dtDynReal>(dadr + offset + 1));
                }
            }
            SetJointQuaternion(data.qpos, qadr + 3, angle, (jointId % 3) - 1);
        }
        else if (jointType == dtDynJointType::Ball)
        {
            dtDynReal angle = 0.0;
            if (state == "asymmetric" || state == "nonzero_velocity")
            {
                angle = static_cast<dtDynReal>(0.03) * scale;
            }
            SetJointQuaternion(data.qpos, qadr, angle, (jointId % 3) - 1);
            if (state == "nonzero_velocity")
            {
                for (dtDynIndex offset = 0; offset < 3; ++offset)
                {
                    data.qvel(dadr + offset) =
                        static_cast<dtDynReal>(0.08) *
                        std::sin(static_cast<dtDynReal>(0.3) * static_cast<dtDynReal>(dadr + offset + 1));
                }
            }
        }
        else
        {
            if (state == "asymmetric")
            {
                data.qpos(qadr) = static_cast<dtDynReal>(0.07) *
                                  std::sin(static_cast<dtDynReal>(0.7) * scale);
            }
            else if (state == "nonzero_velocity")
            {
                data.qpos(qadr) = static_cast<dtDynReal>(0.03) *
                                  std::cos(static_cast<dtDynReal>(0.5) * scale);
                data.qvel(dadr) = static_cast<dtDynReal>(0.2) *
                                  std::sin(static_cast<dtDynReal>(0.4) * scale);
            }
        }
    }
}

dtDynSpec SerialChainSpec(const int linkCount)
{
    dtDynSpec spec;
    const dtDynIndex base = spec.AddBody("base", 0);
    spec.Bodies()[base].inertial.mass = 2.0;
    spec.Bodies()[base].inertial.inertia(0) = 0.04;
    spec.Bodies()[base].inertial.inertia(1) = 0.04;
    spec.Bodies()[base].inertial.inertia(2) = 0.04;

    dtDynIndex parent = base;
    for (int i = 0; i < linkCount; ++i)
    {
        const dtDynReal length = static_cast<dtDynReal>(0.12 + 0.002 * (i % 5));
        const dtDynReal mass = static_cast<dtDynReal>(0.35 + 0.01 * (i % 7));
        const dtDynReal axisX = (i % 3) == 0 ? static_cast<dtDynReal>(1.0) : static_cast<dtDynReal>(0.0);
        const dtDynReal axisY = (i % 3) == 1 ? static_cast<dtDynReal>(1.0) : static_cast<dtDynReal>(0.0);
        const dtDynReal axisZ = (i % 3) == 2 ? static_cast<dtDynReal>(1.0) : static_cast<dtDynReal>(0.0);
        parent = AddHingeLink(spec,
                              parent,
                              "serial_link_" + std::to_string(i),
                              length,
                              static_cast<dtDynReal>(0.01 * ((i % 2) == 0 ? 1 : -1)),
                              static_cast<dtDynReal>(0.02 * ((i % 3) - 1)),
                              mass,
                              axisX,
                              axisY,
                              axisZ);
    }
    spec.AddSite(parent, "serial_tip");
    spec.Sites()[0].pos(0) = static_cast<dtDynReal>(0.10);
    return spec;
}

Scenario MakeScenario(const std::string &fixture, const std::string &state, const dtDynSpec &spec)
{
    Scenario scenario;
    scenario.fixture = fixture;
    scenario.state = state;
    scenario.spec = spec;
    return scenario;
}

dtArray<Scenario> MakeScenarios()
{
    dtArray<Scenario> scenarios;
    const dtArray<std::string> states = {"nominal", "asymmetric", "nonzero_velocity"};
    const bool onlyExternalFixture = EnvEnabled("DTDYNAMICS_BENCHMARK_ONLY_EXTERNAL");
    const auto addFixture = [&scenarios, &states](const std::string &fixture, const dtDynSpec &spec)
    {
        for (const std::string &state : states)
        {
            scenarios.push_back(MakeScenario(fixture, state, spec));
        }
    };

    if (!onlyExternalFixture)
    {
        addFixture("single_manipulator_fixed_base", SingleManipulatorSpec());
        addFixture("dual_manipulator_fixed_base", DualManipulatorSpec());
        addFixture("multi_robot_fixed_base", MultiRobotSpec());
        addFixture("multi_robot_floating_base", MultiRobotFloatingSpec());
        addFixture("quadruped_floating_base", FloatingQuadrupedSpec());
        addFixture("fixed_body_heavy", FixedBodyHeavySpec());
        addFixture("serial_chain_10", SerialChainSpec(10));
        addFixture("serial_chain_30", SerialChainSpec(30));
    }

    const char *externalXmlPath = std::getenv("DTDYNAMICS_BENCHMARK_XML");
    if (externalXmlPath != nullptr && externalXmlPath[0] != '\0')
    {
        const char *externalXmlName = std::getenv("DTDYNAMICS_BENCHMARK_XML_NAME");
        addFixture((externalXmlName != nullptr && externalXmlName[0] != '\0') ? externalXmlName : "external_xml",
                   dtDynSpec::DecodeXml(ReadTextFile(externalXmlPath)));
    }
    else if (onlyExternalFixture)
    {
        throw std::runtime_error("benchmark_dtDynamicsStages: DTDYNAMICS_BENCHMARK_ONLY_EXTERNAL requires DTDYNAMICS_BENCHMARK_XML");
    }

    return scenarios;
}

dt::Math::Vector3<dtDynReal> Gravity()
{
    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = 0.0;
    gravity(2) = -9.81;
    return gravity;
}

void ComputeBenchmarkCentroidalMatrices(const dtDynModel &model, dtDynData &data)
{
    detail::ComputeCentroidalMomentumMatrix(model, data);
    detail::ComputeCentroidalMomentumMatrixDot(model, data);
}

void ComputeBenchmarkComJacobians(const dtDynModel &model, dtDynData &data)
{
    detail::ComputeComJacobianMatrixFromForces(model, data);
    detail::ComputeComJacobianMatrixDotFromForces(model, data);
}

void ComputeBenchmarkCentroidalMomentumProduct(const dtDynModel &model, dtDynData &data)
{
    const dtDynIndex rootCount = utils::EffectiveRootCount(model);

    data.centMom.SetZero();
    for (dtDynIndex row = 0; row < 6 * rootCount; ++row)
    {
        for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
        {
            data.centMom(row) += data.cmAmat(row, dofId) * data.qvel(dofId);
        }
    }
}

void ComputeBenchmarkComVelocityProduct(const dtDynModel &model, dtDynData &data)
{
    const dtDynIndex rootCount = utils::EffectiveRootCount(model);

    data.comVel.SetZero();
    for (dtDynIndex row = 0; row < 3 * rootCount; ++row)
    {
        for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
        {
            data.comVel(row) += data.comJmat(row, dofId) * data.qvel(dofId);
        }
    }
}

void ComputeBenchmarkProducts(const dtDynModel &model, dtDynData &data)
{
    ComputeBenchmarkCentroidalMomentumProduct(model, data);
    ComputeBenchmarkComVelocityProduct(model, data);
    detail::ComputeCoriolisTorqueProduct(model, data);
}

void ComputeBenchmarkExtendedDynamics(const dtDynModel &model, dtDynData &data)
{
    detail::ClearDenseDynamicsOutputs(model, data);
    detail::ComputeRootComOutputs(model, data);
    ComputeBenchmarkCentroidalMatrices(model, data);
    detail::ComputeMassMatrixCCRBA(model, data);
    detail::ComputeMassMatrixDotCCRBA(model, data);
    detail::ComputeCoriolisMatrixCCRBA(model, data);
    ComputeBenchmarkComJacobians(model, data);
    ComputeBenchmarkProducts(model, data);
}

void PrepareNone(const dtDynModel &, dtDynData &, const dt::Math::Vector3<dtDynReal> &)
{
}

void PrepareForwardKinematics(const dtDynModel &model,
                              dtDynData &data,
                              const dt::Math::Vector3<dtDynReal> &)
{
    ForwardKinematics(model, data);
}

void PrepareCentroidal(const dtDynModel &model,
                       dtDynData &data,
                       const dt::Math::Vector3<dtDynReal> &)
{
    ForwardKinematics(model, data);
    detail::ComputeCentroidalPrerequisites(model, data);
    ComputeBenchmarkExtendedDynamics(model, data);
}

void PrepareCcrbaPrerequisites(const dtDynModel &model,
                                dtDynData &data,
                                const dt::Math::Vector3<dtDynReal> &)
{
    ForwardKinematics(model, data);
}

void PrepareCcrbaDofMotionSubspaces(const dtDynModel &model,
                                     dtDynData &data,
                                     const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaPrerequisites(model, data, gravity);
    detail::ComputeSubtreeCom(model, data);
}

void PrepareCcrbaDiffMotion(const dtDynModel &model,
                             dtDynData &data,
                             const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaDofMotionSubspaces(model, data, gravity);
    detail::ComputeDofMotionSubspaces(model, data);
}

void PrepareCcrbaCompositeInertias(const dtDynModel &model,
                                    dtDynData &data,
                                    const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaDiffMotion(model, data, gravity);
    detail::ComputeDiffMotion(model, data);
}

void PrepareCcrbaBiasForces(const dtDynModel &model,
                             dtDynData &data,
                             const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaCompositeInertias(model, data, gravity);
    detail::ComputeCompositeRigidBodyInertias(model, data);
}

void PrepareCcrbaBiasAccumulation(const dtDynModel &model,
                                   dtDynData &data,
                                   const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaBiasForces(model, data, gravity);
    detail::ComputeLocalBiasMatrices(model, data);
}

void PrepareCcrbaDofBiasForces(const dtDynModel &model,
                                dtDynData &data,
                                const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaBiasAccumulation(model, data, gravity);
    detail::AccumulateCompositeBiasMatrices(model, data);
}

void PrepareCcrbaExtended(const dtDynModel &model,
                           dtDynData &data,
                           const dt::Math::Vector3<dtDynReal> &)
{
    ForwardKinematics(model, data);
    detail::ComputeCentroidalPrerequisites(model, data);
}

void PrepareCcrbaRootCom(const dtDynModel &model,
                          dtDynData &data,
                          const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaExtended(model, data, gravity);
    detail::ClearDenseDynamicsOutputs(model, data);
}

void PrepareCcrbaMatrices(const dtDynModel &model,
                           dtDynData &data,
                           const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaExtended(model, data, gravity);
    detail::ClearDenseDynamicsOutputs(model, data);
}

void PrepareCcrbaComJacobians(const dtDynModel &model,
                               dtDynData &data,
                               const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaMatrices(model, data, gravity);
    ComputeBenchmarkCentroidalMatrices(model, data);
}

void PrepareCcrbaProducts(const dtDynModel &model,
                           dtDynData &data,
                           const dt::Math::Vector3<dtDynReal> &gravity)
{
    PrepareCcrbaComJacobians(model, data, gravity);
    detail::ComputeCoriolisMatrixCCRBA(model, data);
    ComputeBenchmarkComJacobians(model, data);
}

void RunForwardKinematics(const dtDynModel &model,
                          dtDynData &data,
                          const dt::Math::Vector3<dtDynReal> &,
                          const dtArray<dtDynIndex> &,
                          const dtArray<dtDynIndex> &)
{
    ForwardKinematics(model, data);
}

void RunMassMatrix(const dtDynModel &model,
                   dtDynData &data,
                   const dt::Math::Vector3<dtDynReal> &,
                   const dtArray<dtDynIndex> &,
                   const dtArray<dtDynIndex> &)
{
    detail::ComputeRNEAPrerequisites(model, data);
    detail::ComputeMassMatrixCRBA(model, data);
}

void RunCentroidalQuantities(const dtDynModel &model,
                             dtDynData &data,
                             const dt::Math::Vector3<dtDynReal> &,
                             const dtArray<dtDynIndex> &,
                             const dtArray<dtDynIndex> &)
{
    detail::ComputeCentroidalPrerequisites(model, data);
    ComputeBenchmarkExtendedDynamics(model, data);
}

void RunCcrbaPrerequisites(const dtDynModel &model,
                            dtDynData &data,
                            const dt::Math::Vector3<dtDynReal> &,
                            const dtArray<dtDynIndex> &,
                            const dtArray<dtDynIndex> &)
{
    detail::ComputeCentroidalPrerequisites(model, data);
}

void RunCcrbaSubtreeCom(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &,
                         const dtArray<dtDynIndex> &,
                         const dtArray<dtDynIndex> &)
{
    detail::ComputeSubtreeCom(model, data);
}

void RunCcrbaDofMotionSubspaces(const dtDynModel &model,
                                 dtDynData &data,
                                 const dt::Math::Vector3<dtDynReal> &,
                                 const dtArray<dtDynIndex> &,
                                 const dtArray<dtDynIndex> &)
{
    detail::ComputeDofMotionSubspaces(model, data);
}

void RunCcrbaDiffMotion(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &,
                         const dtArray<dtDynIndex> &,
                         const dtArray<dtDynIndex> &)
{
    detail::ComputeDiffMotion(model, data);
}

void RunCcrbaCompositeInertias(const dtDynModel &model,
                                dtDynData &data,
                                const dt::Math::Vector3<dtDynReal> &,
                                const dtArray<dtDynIndex> &,
                                const dtArray<dtDynIndex> &)
{
    detail::ComputeCompositeRigidBodyInertias(model, data);
}

void RunCcrbaLocalBiasMatrices(const dtDynModel &model,
                                dtDynData &data,
                                const dt::Math::Vector3<dtDynReal> &,
                                const dtArray<dtDynIndex> &,
                                const dtArray<dtDynIndex> &)
{
    detail::ComputeLocalBiasMatrices(model, data);
}

void RunCcrbaBiasAccumulation(const dtDynModel &model,
                               dtDynData &data,
                               const dt::Math::Vector3<dtDynReal> &,
                               const dtArray<dtDynIndex> &,
                               const dtArray<dtDynIndex> &)
{
    detail::AccumulateCompositeBiasMatrices(model, data);
}

void RunCcrbaDofBiasForces(const dtDynModel &model,
                            dtDynData &data,
                            const dt::Math::Vector3<dtDynReal> &,
                            const dtArray<dtDynIndex> &,
                            const dtArray<dtDynIndex> &)
{
    detail::ComputeDofBiasForces(model, data);
}

void RunCcrbaBiasForces(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &,
                         const dtArray<dtDynIndex> &,
                         const dtArray<dtDynIndex> &)
{
    detail::ComputeBiasMatricesAndForcesFromCompositeInertias(model, data);
}

void RunCcrbaExtended(const dtDynModel &model,
                       dtDynData &data,
                       const dt::Math::Vector3<dtDynReal> &,
                       const dtArray<dtDynIndex> &,
                       const dtArray<dtDynIndex> &)
{
    ComputeBenchmarkExtendedDynamics(model, data);
}

void RunCcrbaClearOutputs(const dtDynModel &model,
                           dtDynData &data,
                           const dt::Math::Vector3<dtDynReal> &,
                           const dtArray<dtDynIndex> &,
                           const dtArray<dtDynIndex> &)
{
    detail::ClearDenseDynamicsOutputs(model, data);
}

void RunCcrbaRootCom(const dtDynModel &model,
                      dtDynData &data,
                      const dt::Math::Vector3<dtDynReal> &,
                      const dtArray<dtDynIndex> &,
                      const dtArray<dtDynIndex> &)
{
    detail::ComputeRootComOutputs(model, data);
}

void RunCcrbaComJacobians(const dtDynModel &model,
                           dtDynData &data,
                           const dt::Math::Vector3<dtDynReal> &,
                           const dtArray<dtDynIndex> &,
                           const dtArray<dtDynIndex> &)
{
    ComputeBenchmarkComJacobians(model, data);
}

void RunCcrbaCentroidalMatrices(const dtDynModel &model,
                                 dtDynData &data,
                                 const dt::Math::Vector3<dtDynReal> &,
                                 const dtArray<dtDynIndex> &,
                                 const dtArray<dtDynIndex> &)
{
    ComputeBenchmarkCentroidalMatrices(model, data);
}

void RunCcrbaMassMatrix(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &,
                         const dtArray<dtDynIndex> &,
                         const dtArray<dtDynIndex> &)
{
    detail::ComputeMassMatrixCCRBA(model, data);
}

void RunCcrbaMassMatrixDot(const dtDynModel &model,
                            dtDynData &data,
                            const dt::Math::Vector3<dtDynReal> &,
                            const dtArray<dtDynIndex> &,
                            const dtArray<dtDynIndex> &)
{
    detail::ComputeMassMatrixDotCCRBA(model, data);
}

void RunCcrbaCoriolisMatrix(const dtDynModel &model,
                             dtDynData &data,
                             const dt::Math::Vector3<dtDynReal> &,
                             const dtArray<dtDynIndex> &,
                             const dtArray<dtDynIndex> &)
{
    detail::ComputeCoriolisMatrixCCRBA(model, data);
}

void RunCcrbaCentroidalMomentumProduct(const dtDynModel &model,
                                        dtDynData &data,
                                        const dt::Math::Vector3<dtDynReal> &,
                                        const dtArray<dtDynIndex> &,
                                        const dtArray<dtDynIndex> &)
{
    ComputeBenchmarkCentroidalMomentumProduct(model, data);
}

void RunCcrbaComVelocityProduct(const dtDynModel &model,
                                 dtDynData &data,
                                 const dt::Math::Vector3<dtDynReal> &,
                                 const dtArray<dtDynIndex> &,
                                 const dtArray<dtDynIndex> &)
{
    ComputeBenchmarkComVelocityProduct(model, data);
}

void RunCcrbaCoriolisTorqueProduct(const dtDynModel &model,
                                    dtDynData &data,
                                    const dt::Math::Vector3<dtDynReal> &,
                                    const dtArray<dtDynIndex> &,
                                    const dtArray<dtDynIndex> &)
{
    detail::ComputeCoriolisTorqueProduct(model, data);
}

void RunCcrbaProducts(const dtDynModel &model,
                       dtDynData &data,
                       const dt::Math::Vector3<dtDynReal> &,
                       const dtArray<dtDynIndex> &,
                       const dtArray<dtDynIndex> &)
{
    ComputeBenchmarkProducts(model, data);
}

void RunBodyJacobians(const dtDynModel &model,
                      dtDynData &data,
                      const dt::Math::Vector3<dtDynReal> &,
                      const dtArray<dtDynIndex> &,
                      const dtArray<dtDynIndex> &)
{
    ComputeBodyJacobians(model, data);
}

void RunSiteJacobians(const dtDynModel &model,
                      dtDynData &data,
                      const dt::Math::Vector3<dtDynReal> &,
                      const dtArray<dtDynIndex> &,
                      const dtArray<dtDynIndex> &)
{
    ComputeSiteJacobians(model, data);
}

void RunAllJacobians(const dtDynModel &model,
                     dtDynData &data,
                     const dt::Math::Vector3<dtDynReal> &,
                     const dtArray<dtDynIndex> &,
                     const dtArray<dtDynIndex> &)
{
    ComputeJacobians(model, data);
}

void RunIndexedJacobians(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &,
                         const dtArray<dtDynIndex> &bodyIds,
                         const dtArray<dtDynIndex> &siteIds)
{
    ComputeJacobians(model, data, bodyIds, siteIds);
}

void RunGravityForces(const dtDynModel &model,
                      dtDynData &data,
                      const dt::Math::Vector3<dtDynReal> &gravity,
                      const dtArray<dtDynIndex> &,
                      const dtArray<dtDynIndex> &)
{
    detail::ComputeGravityFromCentroidalForces(model, data, gravity);
}

void RunFullDynamicsCCRBAAllTerms(const dtDynModel &model,
                                  dtDynData &data,
                                  const dt::Math::Vector3<dtDynReal> &gravity,
                                  const dtArray<dtDynIndex> &,
                                  const dtArray<dtDynIndex> &)
{
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);
}

void RunFullDynamicsCCRBAWbcTerms(const dtDynModel &model,
                                  dtDynData &data,
                                  const dt::Math::Vector3<dtDynReal> &gravity,
                                  const dtArray<dtDynIndex> &,
                                  const dtArray<dtDynIndex> &)
{
    ComputeDynamicsCCRBA(model, data, gravity, kLeoQuadWbcCcrbaTermFlags);
}

void RunLeoQuadWbcUpdate(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &gravity,
                         const dtArray<dtDynIndex> &bodyIds,
                         const dtArray<dtDynIndex> &siteIds)
{
    ComputeDynamicsCCRBA(model, data, gravity, kLeoQuadWbcCcrbaTermFlags);
    ComputeJacobians(model, data, bodyIds, siteIds);
}

void RunFullDynamicsRNEA(const dtDynModel &model,
                         dtDynData &data,
                         const dt::Math::Vector3<dtDynReal> &gravity,
                         const dtArray<dtDynIndex> &,
                         const dtArray<dtDynIndex> &)
{
    ComputeDynamicsRNEA(model, data, gravity);
}

void RunFullDynamicsCCRBA(const dtDynModel &model,
                          dtDynData &data,
                          const dt::Math::Vector3<dtDynReal> &gravity,
                          const dtArray<dtDynIndex> &,
                          const dtArray<dtDynIndex> &)
{
    ComputeDynamicsCCRBA(model, data, gravity);
}

dtArray<Stage> MakeStages()
{
    return {{"forward_kinematics", PrepareNone, RunForwardKinematics},
            {"mass_matrix_after_fk", PrepareForwardKinematics, RunMassMatrix},
            {"ccrba_after_fk", PrepareForwardKinematics, RunCentroidalQuantities},
            {"ccrba_prerequisites_after_fk", PrepareCcrbaPrerequisites, RunCcrbaPrerequisites},
            {"ccrba_subtree_com_after_fk", PrepareCcrbaPrerequisites, RunCcrbaSubtreeCom},
            {"ccrba_dof_motion_subspaces_after_subtree_com", PrepareCcrbaDofMotionSubspaces, RunCcrbaDofMotionSubspaces},
            {"ccrba_diff_motion_after_cdof", PrepareCcrbaDiffMotion, RunCcrbaDiffMotion},
            {"ccrba_composite_inertias_after_diff_motion", PrepareCcrbaCompositeInertias, RunCcrbaCompositeInertias},
            {"ccrba_local_bias_matrices_after_composite_inertias", PrepareCcrbaBiasForces, RunCcrbaLocalBiasMatrices},
            {"ccrba_bias_accumulation_after_local_bias", PrepareCcrbaBiasAccumulation, RunCcrbaBiasAccumulation},
            {"ccrba_dof_bias_forces_after_bias_accumulation", PrepareCcrbaDofBiasForces, RunCcrbaDofBiasForces},
            {"ccrba_bias_forces_after_composite_inertias", PrepareCcrbaBiasForces, RunCcrbaBiasForces},
            {"ccrba_extended_after_prereq", PrepareCcrbaExtended, RunCcrbaExtended},
            {"ccrba_clear_dense_outputs_after_prereq", PrepareCcrbaExtended, RunCcrbaClearOutputs},
            {"ccrba_root_com_outputs_after_prereq", PrepareCcrbaRootCom, RunCcrbaRootCom},
            {"ccrba_centroidal_matrices_after_prereq", PrepareCcrbaMatrices, RunCcrbaCentroidalMatrices},
            {"ccrba_mass_matrix_after_prereq", PrepareCcrbaMatrices, RunCcrbaMassMatrix},
            {"ccrba_mass_matrix_dot_after_prereq", PrepareCcrbaMatrices, RunCcrbaMassMatrixDot},
            {"ccrba_coriolis_matrix_after_prereq", PrepareCcrbaMatrices, RunCcrbaCoriolisMatrix},
            {"ccrba_com_jacobians_after_centroidal_matrices", PrepareCcrbaComJacobians, RunCcrbaComJacobians},
            {"ccrba_centroidal_momentum_product_after_matrices", PrepareCcrbaProducts, RunCcrbaCentroidalMomentumProduct},
            {"ccrba_com_velocity_product_after_matrices", PrepareCcrbaProducts, RunCcrbaComVelocityProduct},
            {"ccrba_coriolis_torque_product_after_cmat", PrepareCcrbaProducts, RunCcrbaCoriolisTorqueProduct},
            {"ccrba_products_after_matrices", PrepareCcrbaProducts, RunCcrbaProducts},
            {"all_body_jacobians_after_ccrba", PrepareCentroidal, RunBodyJacobians},
            {"all_site_jacobians_after_ccrba", PrepareCentroidal, RunSiteJacobians},
            {"all_jacobians_after_ccrba", PrepareCentroidal, RunAllJacobians},
            {"indexed_jacobians_after_ccrba", PrepareCentroidal, RunIndexedJacobians},
            {"gravity_after_ccrba", PrepareCentroidal, RunGravityForces},
            {"compute_dynamics_rnea", PrepareNone, RunFullDynamicsRNEA},
            {"compute_dynamics_ccrba", PrepareNone, RunFullDynamicsCCRBA},
            {"compute_dynamics_ccrba_wbc_terms", PrepareNone, RunFullDynamicsCCRBAWbcTerms},
            {"leoquad_wbc_update", PrepareNone, RunLeoQuadWbcUpdate},
            {"compute_dynamics_ccrba_all_terms", PrepareNone, RunFullDynamicsCCRBAAllTerms}};
}

dtArray<dtDynIndex> MakeIndexedBodyIds(const dtDynModel &model)
{
    dtArray<dtDynIndex> bodyIds;
    if (model.nroot > 0)
    {
        bodyIds.push_back(model.root_bodyid[0]);
    }
    return bodyIds;
}

dtArray<dtDynIndex> MakeIndexedSiteIds(const dtDynModel &model)
{
    dtArray<dtDynIndex> siteIds;
    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        siteIds.push_back(siteId);
    }
    return siteIds;
}

Result RunStage(const Scenario &scenario, const Stage &stage, const int warmup, const int iterations)
{
    const dtDynModel model = CompileSpec(scenario.spec);
    dtDynData data;
    data.Resize(model);
    SetState(model, scenario.state, data);

    const dt::Math::Vector3<dtDynReal> gravity = Gravity();
    const dtArray<dtDynIndex> indexedBodyIds = MakeIndexedBodyIds(model);
    const dtArray<dtDynIndex> indexedSiteIds = MakeIndexedSiteIds(model);
    stage.prepare(model, data, gravity);

    for (int i = 0; i < warmup; ++i)
    {
        stage.compute(model, data, gravity, indexedBodyIds, indexedSiteIds);
    }

    dtArray<double> samples;
    samples.resize(iterations);
    dtArray<unsigned long long> allocationSamples;
    allocationSamples.resize(iterations);
    unsigned long long allocationSum = 0;
    unsigned long long allocationMax = 0;

    using Clock = std::chrono::steady_clock;
    for (int i = 0; i < iterations; ++i)
    {
        const unsigned long long allocationsBefore = g_allocationCount.load(std::memory_order_relaxed);
        g_countAllocations.store(true, std::memory_order_relaxed);
        const auto start = Clock::now();
        stage.compute(model, data, gravity, indexedBodyIds, indexedSiteIds);
        const auto finish = Clock::now();
        g_countAllocations.store(false, std::memory_order_relaxed);

        const unsigned long long allocations =
            g_allocationCount.load(std::memory_order_relaxed) - allocationsBefore;
        allocationSum += allocations;
        allocationMax = std::max(allocationMax, allocations);

        const std::chrono::duration<double, std::micro> elapsed = finish - start;
        samples[static_cast<std::size_t>(i)] = elapsed.count();
        allocationSamples[static_cast<std::size_t>(i)] = allocations;
    }

    if (dt::Dynamics::benchmark::BenchmarkEmitSamples())
    {
        for (int i = 0; i < iterations; ++i)
        {
            dt::Dynamics::benchmark::PrintSampleRow(std::cout,
                                                    "benchmark_dtDynamicsStages",
                                                    stage.name,
                                                    scenario.fixture,
                                                    scenario.state,
                                                    stage.name,
                                                    i,
                                                    samples[static_cast<std::size_t>(i)],
                                                    allocationSamples[static_cast<std::size_t>(i)]);
        }
    }

    std::sort(samples.begin(), samples.end());
    double sum = 0.0;
    for (const double value : samples)
    {
        sum += value;
    }

    Result result;
    result.fixture = scenario.fixture;
    result.state = scenario.state;
    result.stage = stage.name;
    result.nq = model.nq;
    result.nv = model.nv;
    result.nbody = model.nbody;
    result.nsite = model.nsite;
    result.iterations = iterations;
    result.meanUs = sum / static_cast<double>(samples.size());
    result.medianUs = samples[samples.size() / 2];
    const std::size_t p99Index = std::min(samples.size() - 1,
                                          static_cast<std::size_t>(0.99 * static_cast<double>(samples.size() - 1)));
    result.p99Us = samples[p99Index];
    result.allocationMean = static_cast<double>(allocationSum) / static_cast<double>(iterations);
    result.allocationMax = allocationMax;
    return result;
}

void PrintResult(const Result &result)
{
    std::cout << result.fixture << ','
              << result.state << ','
              << result.stage << ','
              << result.nq << ','
              << result.nv << ','
              << result.nbody << ','
              << result.nsite << ','
              << result.iterations << ','
              << result.meanUs << ','
              << result.medianUs << ','
              << result.p99Us << ','
              << result.allocationMean << ','
              << result.allocationMax << '\n';
}

} // namespace

int main()
{
    constexpr int warmup = 20;
    constexpr int iterations = 200;
    const int batchCount = dt::Dynamics::benchmark::BenchmarkBatchCount();

    dtArray<Scenario> scenarios = MakeScenarios();
    const dtArray<Stage> stages = MakeStages();
    dtArray<std::string> batchSummaryRows;

    std::cout << "fixture,state,stage,nq,nv,nbody,nsite,iterations,mean_us,median_us,p99_us,"
                 "allocation_mean,allocation_max\n";
    if (dt::Dynamics::benchmark::BenchmarkEmitSamples())
    {
        dt::Dynamics::benchmark::PrintSampleHeader(std::cout);
    }
    std::cout << std::fixed << std::setprecision(6);

    for (const Scenario &scenario : scenarios)
    {
        for (const Stage &stage : stages)
        {
            dtArray<double> meanUs;
            dtArray<double> medianUs;
            dtArray<double> p99Us;
            meanUs.resize(batchCount);
            medianUs.resize(batchCount);
            p99Us.resize(batchCount);
            unsigned long long allocationMax = 0;

            for (int batch = 0; batch < batchCount; ++batch)
            {
                const Result result = RunStage(scenario, stage, warmup, iterations);
                meanUs[batch] = result.meanUs;
                medianUs[batch] = result.medianUs;
                p99Us[batch] = result.p99Us;
                allocationMax = std::max(allocationMax, result.allocationMax);
                if (batch == 0)
                {
                    PrintResult(result);
                }
            }

            if (batchCount > 1)
            {
                batchSummaryRows.push_back(dt::Dynamics::benchmark::BuildBatchSummaryRow("benchmark_dtDynamicsStages",
                                                                                         scenario.fixture,
                                                                                         scenario.state,
                                                                                         stage.name,
                                                                                         batchCount,
                                                                                         meanUs,
                                                                                         medianUs,
                                                                                         p99Us,
                                                                                         allocationMax));
            }
        }
    }

    if (batchCount > 1)
    {
        dt::Dynamics::benchmark::PrintBatchSummaryHeader(std::cout);
        for (const std::string &row : batchSummaryRows)
        {
            std::cout << row << '\n';
        }
    }

    dt::Dynamics::benchmark::PrintBenchmarkMetadata(std::cout, "benchmark_dtDynamicsStages", warmup, iterations);

    return 0;
}
