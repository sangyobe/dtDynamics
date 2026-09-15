#include <dtDynamics/dtDynCompile.h>
#include <dtDynamics/dtDynData.h>
#include <dtDynamics/dtDynDynamics.h>
#include <dtDynamics/dtDynMath.h>
#include <dtDynamics/dtDynSpec.h>

#include "../dtDynBenchmarkMetadata.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <new>
#include <string>

using namespace dt::Dynamics;

namespace
{

std::atomic<bool> g_countAllocations(false);
std::atomic<unsigned long long> g_allocationCount(0);
std::atomic<unsigned long long> g_allocationBytes(0);
// These counters cover allocations that route through the benchmark executable's
// global C++ operator new/new[] overrides. They intentionally provide a
// lightweight steady-state signal, not a complete malloc/calloc/realloc or
// OS-level heap trace.

void CountAllocation(const std::size_t size)
{
    if (g_countAllocations.load(std::memory_order_relaxed))
    {
        g_allocationCount.fetch_add(1, std::memory_order_relaxed);
        g_allocationBytes.fetch_add(static_cast<unsigned long long>(size), std::memory_order_relaxed);
    }
}

void *Allocate(const std::size_t size)
{
    CountAllocation(size == 0 ? 1 : size);
    if (void *ptr = std::malloc(size == 0 ? 1 : size))
    {
        return ptr;
    }
    throw std::bad_alloc();
}

void *AllocateAligned(const std::size_t size, const std::size_t alignment)
{
    const std::size_t allocationSize = size == 0 ? alignment : ((size + alignment - 1) / alignment) * alignment;
    CountAllocation(allocationSize);
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

enum class NativeScope
{
    RneaComPos,
    CcrbaReduced,
    CcrbaAll,
};

const char *NativeScopeName(const NativeScope scope)
{
    switch (scope)
    {
    case NativeScope::RneaComPos:
        return "native_rnea_com_pos";
    case NativeScope::CcrbaReduced:
        return "native_ccrba_reduced";
    case NativeScope::CcrbaAll:
        return "native_ccrba_all";
    }
    std::abort();
}

void ComputeNativeScope(const NativeScope scope,
                        const dtDynModel &model,
                        dtDynData &data,
                        const dt::Math::Vector3<dtDynReal> &gravity)
{
    if (scope == NativeScope::RneaComPos)
    {
        ComputeDynamicsRNEA(model, data, gravity, DTDYN_RNEA_TERM_COM_POS);
        return;
    }
    if (scope == NativeScope::CcrbaReduced)
    {
        constexpr dtDynCcrbaTermFlags reducedMask =
            DTDYN_CCRBA_TERM_COM_POS |
            DTDYN_CCRBA_TERM_CMAMAT |
            DTDYN_CCRBA_TERM_CENT_MOM;
        ComputeDynamicsCCRBA(model, data, gravity, reducedMask);
        return;
    }
    if (scope == NativeScope::CcrbaAll)
    {
        ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);
        return;
    }
    std::abort();
}

struct Result
{
    std::string fixture;
    std::string state;
    std::string scope;
    dtDynIndex nq = 0;
    dtDynIndex nv = 0;
    dtDynIndex nbody = 0;
    int iterations = 0;
    double meanUs = 0.0;
    double medianUs = 0.0;
    double p95Us = 0.0;
    double p99Us = 0.0;
    double allocationMean = 0.0;
    unsigned long long allocationMax = 0;
    double allocationBytesMean = 0.0;
    unsigned long long allocationBytesMax = 0;
    double massSymmetryMax = 0.0;
    double massSymmetryRms = 0.0;
    double nonlinConsistencyMax = 0.0;
    double nonlinConsistencyRms = 0.0;
    double centroidalMomentumMax = 0.0;
    double centroidalMomentumRms = 0.0;
    double comVelocityMax = 0.0;
    double comVelocityRms = 0.0;
};

struct ErrorSummary
{
    double max = 0.0;
    double rms = 0.0;
};

struct ResourceResult
{
    std::string fixture;
    dtDynIndex nq = 0;
    dtDynIndex nv = 0;
    dtDynIndex nbody = 0;
    int iterations = 0;
    double meanUs = 0.0;
    double medianUs = 0.0;
    double p95Us = 0.0;
    double p99Us = 0.0;
    double allocationMean = 0.0;
    unsigned long long allocationMax = 0;
    double allocationBytesMean = 0.0;
    unsigned long long allocationBytesMax = 0;
};

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
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = x; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.pos(1) = y; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.pos(2) = z; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = mass; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.5 * x; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(1) = 0.5 * y; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(2) = 0.5 * z; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.01 * mass + 0.001; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.012 * mass + 0.001; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.014 * mass + 0.001; });

    const dtDynIndex joint = spec.AddJoint(body, name + "_joint", dtDynJointType::Hinge);
    spec.EditJoint(joint, [&](dtDynJointSpec &draft)
                   { draft.axis(0) = axisX; });
    spec.EditJoint(joint, [&](dtDynJointSpec &draft)
                   { draft.axis(1) = axisY; });
    spec.EditJoint(joint, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = axisZ; });
    spec.EditJoint(joint, [&](dtDynJointSpec &draft)
                   { draft.armature = 0.001 * (static_cast<dtDynReal>(joint) + 1.0); });
    return body;
}

dtDynSpec SingleManipulatorSpec()
{
    dtDynSpec spec;
    const dtDynIndex base = spec.AddBody("base", 0);
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 3.0; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.05; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.05; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.05; });

    dtDynIndex parent = base;
    parent = AddHingeLink(spec, parent, "link1", 0.30, 0.00, 0.05, 1.5, 0.0, 0.0, 1.0);
    parent = AddHingeLink(spec, parent, "link2", 0.25, 0.00, 0.00, 1.0, 0.0, 1.0, 0.0);
    parent = AddHingeLink(spec, parent, "link3", 0.20, 0.00, 0.00, 0.8, 1.0, 0.0, 0.0);
    spec.AddSite(parent, "tool");
    spec.EditSite(0, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.2; });
    return spec;
}

dtDynSpec DualManipulatorSpec()
{
    dtDynSpec spec;
    const dtDynIndex base = spec.AddBody("base", 0);
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 5.0; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.08; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.08; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.10; });

    const dtDynIndex leftShoulder = AddHingeLink(spec, base, "left_link1", 0.10, 0.25, 0.0, 1.0, 0.0, 0.0, 1.0);
    const dtDynIndex leftElbow = AddHingeLink(spec, leftShoulder, "left_link2", 0.25, 0.0, 0.0, 0.7, 0.0, 1.0, 0.0);
    const dtDynIndex rightShoulder = AddHingeLink(spec, base, "right_link1", 0.10, -0.25, 0.0, 1.0, 0.0, 0.0, 1.0);
    const dtDynIndex rightElbow = AddHingeLink(spec, rightShoulder, "right_link2", 0.25, 0.0, 0.0, 0.7, 0.0, 1.0, 0.0);

    const dtDynIndex leftTool = spec.AddSite(leftElbow, "left_tool");
    const dtDynIndex rightTool = spec.AddSite(rightElbow, "right_tool");
    spec.EditSite(leftTool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.2; });
    spec.EditSite(rightTool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.2; });
    return spec;
}

dtDynSpec MultiRobotSpec()
{
    dtDynSpec spec;

    const dtDynIndex robotABase = spec.AddBody("robot_a_base", 0);
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.4; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.pos(1) = 0.25; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.04; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.05; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.06; });

    const dtDynIndex robotAShoulder =
        AddHingeLink(spec, robotABase, "robot_a_shoulder", 0.10, 0.0, 0.20, 0.8, 0.0, 0.0, 1.0);
    const dtDynIndex robotAElbow =
        AddHingeLink(spec, robotAShoulder, "robot_a_elbow", 0.25, 0.0, 0.0, 0.6, 0.0, 1.0, 0.0);
    const dtDynIndex robotATool = spec.AddSite(robotAElbow, "robot_a_tool");
    spec.EditSite(robotATool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.2; });

    const dtDynIndex robotBBase = spec.AddBody("robot_b_base", 0);
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = -0.35; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.pos(1) = -0.2; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.pos(2) = 0.05; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.5; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.03; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.035; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.04; });

    const dtDynIndex sensorFrame = spec.AddBody("robot_b_sensor_frame", robotBBase);
    spec.EditBody(sensorFrame, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.02; });
    spec.EditBody(sensorFrame, [&](dtDynBodySpec &draft)
                  { draft.pos(1) = 0.03; });
    spec.EditBody(sensorFrame, [&](dtDynBodySpec &draft)
                  { draft.pos(2) = 0.04; });
    const dtDynIndex sensorSite = spec.AddSite(sensorFrame, "robot_b_sensor");
    spec.EditSite(sensorSite, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.01; });
    spec.EditSite(sensorSite, [&](dtDynSiteSpec &draft)
                  { draft.pos(1) = 0.02; });
    spec.EditSite(sensorSite, [&](dtDynSiteSpec &draft)
                  { draft.pos(2) = 0.03; });

    const dtDynIndex robotBHip =
        AddHingeLink(spec, sensorFrame, "robot_b_hip", 0.0, 0.0, 0.18, 0.7, 1.0, 0.0, 0.0);
    const dtDynIndex robotBKnee =
        AddHingeLink(spec, robotBHip, "robot_b_knee", 0.22, 0.0, 0.0, 0.45, 0.0, 1.0, 0.0);
    const dtDynIndex robotBTool = spec.AddSite(robotBKnee, "robot_b_tool");
    spec.EditSite(robotBTool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.18; });

    const dtDynIndex robotAElbowMotor =
        spec.AddActuator(spec.Bodies()[robotAElbow].joints[0], "robot_a_elbow_motor");
    spec.EditActuator(robotAElbowMotor, [&](dtDynActuatorSpec &draft)
                      { draft.gear = 90.0; });
    const dtDynIndex robotBKneeMotor =
        spec.AddActuator(spec.Bodies()[robotBKnee].joints[0], "robot_b_knee_motor");
    spec.EditActuator(robotBKneeMotor, [&](dtDynActuatorSpec &draft)
                      { draft.gear = 70.0; });

    return spec;
}

dtDynSpec MultiRobotFloatingSpec()
{
    dtDynSpec spec;

    const dtDynIndex robotABase = spec.AddBody("robot_a_floating_base", 0);
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.4; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.pos(1) = 0.25; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.pos(2) = 0.6; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 3.0; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.08; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.09; });
    spec.EditBody(robotABase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.10; });
    spec.AddJoint(robotABase, "robot_a_free", dtDynJointType::Free);

    const dtDynIndex robotAShoulder =
        AddHingeLink(spec, robotABase, "robot_a_floating_shoulder", 0.15, 0.0, -0.10, 0.8, 0.0, 0.0, 1.0);
    const dtDynIndex robotAElbow =
        AddHingeLink(spec, robotAShoulder, "robot_a_floating_elbow", 0.25, 0.0, 0.0, 0.6, 0.0, 1.0, 0.0);
    const dtDynIndex robotATool = spec.AddSite(robotAElbow, "robot_a_floating_tool");
    spec.EditSite(robotATool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.2; });

    const dtDynIndex robotBBase = spec.AddBody("robot_b_floating_base", 0);
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = -0.35; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.pos(1) = -0.20; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.pos(2) = 0.70; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.5; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.07; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.08; });
    spec.EditBody(robotBBase, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.09; });
    spec.AddJoint(robotBBase, "robot_b_free", dtDynJointType::Free);

    const dtDynIndex sensorFrame = spec.AddBody("robot_b_floating_sensor_frame", robotBBase);
    spec.EditBody(sensorFrame, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.02; });
    spec.EditBody(sensorFrame, [&](dtDynBodySpec &draft)
                  { draft.pos(1) = 0.03; });
    spec.EditBody(sensorFrame, [&](dtDynBodySpec &draft)
                  { draft.pos(2) = 0.04; });
    const dtDynIndex sensorSite = spec.AddSite(sensorFrame, "robot_b_floating_sensor");
    spec.EditSite(sensorSite, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.01; });
    spec.EditSite(sensorSite, [&](dtDynSiteSpec &draft)
                  { draft.pos(1) = 0.02; });
    spec.EditSite(sensorSite, [&](dtDynSiteSpec &draft)
                  { draft.pos(2) = 0.03; });

    const dtDynIndex robotBHip =
        AddHingeLink(spec, sensorFrame, "robot_b_floating_hip", 0.0, 0.0, -0.18, 0.7, 1.0, 0.0, 0.0);
    const dtDynIndex robotBKnee =
        AddHingeLink(spec, robotBHip, "robot_b_floating_knee", 0.22, 0.0, 0.0, 0.45, 0.0, 1.0, 0.0);
    const dtDynIndex robotBTool = spec.AddSite(robotBKnee, "robot_b_floating_tool");
    spec.EditSite(robotBTool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.18; });

    const dtDynIndex robotAElbowMotor =
        spec.AddActuator(spec.Bodies()[robotAElbow].joints[0], "robot_a_floating_elbow_motor");
    spec.EditActuator(robotAElbowMotor, [&](dtDynActuatorSpec &draft)
                      { draft.gear = 85.0; });
    const dtDynIndex robotBKneeMotor =
        spec.AddActuator(spec.Bodies()[robotBKnee].joints[0], "robot_b_floating_knee_motor");
    spec.EditActuator(robotBKneeMotor, [&](dtDynActuatorSpec &draft)
                      { draft.gear = 65.0; });

    return spec;
}

dtDynSpec FloatingQuadrupedSpec()
{
    dtDynSpec spec;
    const dtDynIndex trunk = spec.AddBody("trunk", 0);
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 10.0; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.20; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.30; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.40; });
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
        spec.EditSite(foot, [&](dtDynSiteSpec &draft)
                      { draft.pos(2) = -0.25; });
    }
    return spec;
}

dtDynSpec FixedBodyHeavySpec()
{
    dtDynSpec spec;
    const dtDynIndex base = spec.AddBody("base", 0);
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 4.0; });
    dtDynIndex parent = base;
    for (dtDynIndex i = 0; i < 8; ++i)
    {
        const dtDynIndex mount = spec.AddBody("fixed_mount_" + std::to_string(i), parent);
        spec.EditBody(mount, [&](dtDynBodySpec &draft)
                      { draft.pos(0) = 0.03 * static_cast<dtDynReal>(i + 1); });
        spec.EditBody(mount, [&](dtDynBodySpec &draft)
                      { draft.pos(1) = 0.02 * static_cast<dtDynReal>((i % 2) == 0 ? 1 : -1); });
        spec.EditBody(mount, [&](dtDynBodySpec &draft)
                      { draft.inertial.mass = 0.05; });
        parent = mount;
    }
    const dtDynIndex jointed = AddHingeLink(spec, parent, "jointed_link", 0.25, 0.0, 0.0, 1.2, 0.0, 0.0, 1.0);
    spec.AddSite(jointed, "tool");
    spec.EditSite(0, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.15; });
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
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.04; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.04; });
    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.04; });

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
    spec.EditSite(0, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = static_cast<dtDynReal>(0.10); });
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
    const auto addFixture = [&scenarios, &states](const std::string &fixture, const dtDynSpec &spec)
    {
        for (const std::string &state : states)
        {
            scenarios.push_back(MakeScenario(fixture, state, spec));
        }
    };

    addFixture("single_manipulator_fixed_base", SingleManipulatorSpec());
    addFixture("dual_manipulator_fixed_base", DualManipulatorSpec());
    addFixture("multi_robot_fixed_base", MultiRobotSpec());
    addFixture("multi_robot_floating_base", MultiRobotFloatingSpec());
    addFixture("quadruped_floating_base", FloatingQuadrupedSpec());
    addFixture("fixed_body_heavy", FixedBodyHeavySpec());
    addFixture("serial_chain_10", SerialChainSpec(10));
    addFixture("serial_chain_30", SerialChainSpec(30));
    return scenarios;
}

ErrorSummary MakeErrorSummary(const double maxError, const double squaredError, const int count)
{
    ErrorSummary summary;
    summary.max = maxError;
    summary.rms = count == 0 ? 0.0 : std::sqrt(squaredError / static_cast<double>(count));
    return summary;
}

void UpdateWorst(ErrorSummary &worst, const ErrorSummary &current)
{
    worst.max = std::max(worst.max, current.max);
    worst.rms = std::max(worst.rms, current.rms);
}

ErrorSummary MassSymmetryError(const dtDynData &data)
{
    double maxError = 0.0;
    double squaredError = 0.0;
    int count = 0;
    for (dtDynIndex row = 0; row < data.Mmat.GetRowSize(); ++row)
    {
        for (dtDynIndex col = row + 1; col < data.Mmat.GetColSize(); ++col)
        {
            const double error = std::fabs(data.Mmat(row, col) - data.Mmat(col, row));
            maxError = std::max(maxError, error);
            squaredError += error * error;
            ++count;
        }
    }
    return MakeErrorSummary(maxError, squaredError, count);
}

ErrorSummary NonlinearConsistencyError(const dtDynData &data)
{
    double maxError = 0.0;
    double squaredError = 0.0;
    int count = 0;
    for (dtDynIndex i = 0; i < data.nonlinTorq.GetDim(); ++i)
    {
        const double expected = data.coriolisTorq(i) + data.gravityTorq(i);
        const double error = std::fabs(data.nonlinTorq(i) - expected);
        maxError = std::max(maxError, error);
        squaredError += error * error;
        ++count;
    }
    return MakeErrorSummary(maxError, squaredError, count);
}

ErrorSummary CentroidalMomentumConsistencyError(const dtDynData &data)
{
    double maxError = 0.0;
    double squaredError = 0.0;
    int count = 0;
    for (dtDynIndex row = 0; row < data.centMom.GetDim(); ++row)
    {
        double expected = 0.0;
        for (dtDynIndex col = 0; col < data.qvel.GetDim(); ++col)
        {
            expected += data.cmAmat(row, col) * data.qvel(col);
        }
        const double error = std::fabs(data.centMom(row) - expected);
        maxError = std::max(maxError, error);
        squaredError += error * error;
        ++count;
    }
    return MakeErrorSummary(maxError, squaredError, count);
}

ErrorSummary ComVelocityConsistencyError(const dtDynData &data)
{
    double maxError = 0.0;
    double squaredError = 0.0;
    int count = 0;
    for (dtDynIndex row = 0; row < data.comVel.GetDim(); ++row)
    {
        double expected = 0.0;
        for (dtDynIndex col = 0; col < data.qvel.GetDim(); ++col)
        {
            expected += data.comJmat(row, col) * data.qvel(col);
        }
        const double error = std::fabs(data.comVel(row) - expected);
        maxError = std::max(maxError, error);
        squaredError += error * error;
        ++count;
    }
    return MakeErrorSummary(maxError, squaredError, count);
}

Result RunScenario(Scenario &scenario,
                   const NativeScope scope,
                   const int warmup,
                   const int iterations)
{
    const dtDynModel model = CompileSpec(scenario.spec);
    dtDynData data;
    data.Resize(model);
    SetState(model, scenario.state, data);

    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = 0.0;
    gravity(2) = -9.81;

    for (int i = 0; i < warmup; ++i)
    {
        ComputeNativeScope(scope, model, data, gravity);
    }

    dtArray<double> samples;
    samples.resize(iterations);
    unsigned long long allocationSum = 0;
    unsigned long long allocationMax = 0;
    unsigned long long allocationBytesSum = 0;
    unsigned long long allocationBytesMax = 0;
    ErrorSummary massSymmetry;
    ErrorSummary nonlinConsistency;
    ErrorSummary centroidalMomentum;
    ErrorSummary comVelocity;

    using Clock = std::chrono::steady_clock;
    for (int i = 0; i < iterations; ++i)
    {
        const unsigned long long allocationsBefore = g_allocationCount.load(std::memory_order_relaxed);
        const unsigned long long allocationBytesBefore = g_allocationBytes.load(std::memory_order_relaxed);
        g_countAllocations.store(true, std::memory_order_relaxed);
        const auto start = Clock::now();
        ComputeNativeScope(scope, model, data, gravity);
        const auto finish = Clock::now();
        g_countAllocations.store(false, std::memory_order_relaxed);
        const unsigned long long allocations =
            g_allocationCount.load(std::memory_order_relaxed) - allocationsBefore;
        const unsigned long long allocationBytes =
            g_allocationBytes.load(std::memory_order_relaxed) - allocationBytesBefore;
        const std::chrono::duration<double, std::micro> elapsed = finish - start;
        samples[static_cast<std::size_t>(i)] = elapsed.count();
        allocationSum += allocations;
        allocationMax = std::max(allocationMax, allocations);
        allocationBytesSum += allocationBytes;
        allocationBytesMax = std::max(allocationBytesMax, allocationBytes);
        UpdateWorst(massSymmetry, MassSymmetryError(data));
        UpdateWorst(nonlinConsistency, NonlinearConsistencyError(data));
        if (scope != NativeScope::RneaComPos)
        {
            UpdateWorst(centroidalMomentum, CentroidalMomentumConsistencyError(data));
        }
        if (scope == NativeScope::CcrbaAll)
        {
            UpdateWorst(comVelocity, ComVelocityConsistencyError(data));
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
    result.scope = NativeScopeName(scope);
    result.nq = model.nq;
    result.nv = model.nv;
    result.nbody = model.nbody;
    result.iterations = iterations;
    result.meanUs = sum / static_cast<double>(samples.size());
    result.medianUs = samples[samples.size() / 2];
    const std::size_t p95Index = std::min(samples.size() - 1,
                                          static_cast<std::size_t>(0.95 * static_cast<double>(samples.size() - 1)));
    result.p95Us = samples[p95Index];
    const std::size_t p99Index = std::min(samples.size() - 1,
                                          static_cast<std::size_t>(0.99 * static_cast<double>(samples.size() - 1)));
    result.p99Us = samples[p99Index];
    result.allocationMean = static_cast<double>(allocationSum) / static_cast<double>(iterations);
    result.allocationMax = allocationMax;
    result.allocationBytesMean = static_cast<double>(allocationBytesSum) / static_cast<double>(iterations);
    result.allocationBytesMax = allocationBytesMax;
    result.massSymmetryMax = massSymmetry.max;
    result.massSymmetryRms = massSymmetry.rms;
    result.nonlinConsistencyMax = nonlinConsistency.max;
    result.nonlinConsistencyRms = nonlinConsistency.rms;
    const double notApplicable = std::numeric_limits<double>::quiet_NaN();
    result.centroidalMomentumMax = scope != NativeScope::RneaComPos ? centroidalMomentum.max : notApplicable;
    result.centroidalMomentumRms = scope != NativeScope::RneaComPos ? centroidalMomentum.rms : notApplicable;
    result.comVelocityMax = scope == NativeScope::CcrbaAll ? comVelocity.max : notApplicable;
    result.comVelocityRms = scope == NativeScope::CcrbaAll ? comVelocity.rms : notApplicable;
    return result;
}

ResourceResult MeasureCompileSpec(const Scenario &scenario, const int iterations)
{
    const dtDynModel dimensionModel = CompileSpec(scenario.spec);

    dtArray<double> samples;
    samples.resize(iterations);
    unsigned long long allocationSum = 0;
    unsigned long long allocationMax = 0;
    unsigned long long allocationBytesSum = 0;
    unsigned long long allocationBytesMax = 0;

    using Clock = std::chrono::steady_clock;
    for (int i = 0; i < iterations; ++i)
    {
        const unsigned long long allocationsBefore = g_allocationCount.load(std::memory_order_relaxed);
        const unsigned long long allocationBytesBefore = g_allocationBytes.load(std::memory_order_relaxed);
        g_countAllocations.store(true, std::memory_order_relaxed);
        const auto start = Clock::now();
        const dtDynModel model = CompileSpec(scenario.spec);
        const auto finish = Clock::now();
        g_countAllocations.store(false, std::memory_order_relaxed);

        const unsigned long long allocations =
            g_allocationCount.load(std::memory_order_relaxed) - allocationsBefore;
        const unsigned long long allocationBytes =
            g_allocationBytes.load(std::memory_order_relaxed) - allocationBytesBefore;
        const std::chrono::duration<double, std::micro> elapsed = finish - start;
        samples[static_cast<std::size_t>(i)] = elapsed.count();
        allocationSum += allocations;
        allocationMax = std::max(allocationMax, allocations);
        allocationBytesSum += allocationBytes;
        allocationBytesMax = std::max(allocationBytesMax, allocationBytes);
        if (model.nbody == 0)
        {
            std::abort();
        }
    }

    std::sort(samples.begin(), samples.end());
    double sum = 0.0;
    for (const double value : samples)
    {
        sum += value;
    }

    ResourceResult result;
    result.fixture = scenario.fixture;
    result.nq = dimensionModel.nq;
    result.nv = dimensionModel.nv;
    result.nbody = dimensionModel.nbody;
    result.iterations = iterations;
    result.meanUs = sum / static_cast<double>(samples.size());
    result.medianUs = samples[samples.size() / 2];
    const std::size_t p95Index = std::min(samples.size() - 1,
                                          static_cast<std::size_t>(0.95 * static_cast<double>(samples.size() - 1)));
    result.p95Us = samples[p95Index];
    const std::size_t p99Index = std::min(samples.size() - 1,
                                          static_cast<std::size_t>(0.99 * static_cast<double>(samples.size() - 1)));
    result.p99Us = samples[p99Index];
    result.allocationMean = static_cast<double>(allocationSum) / static_cast<double>(iterations);
    result.allocationMax = allocationMax;
    result.allocationBytesMean = static_cast<double>(allocationBytesSum) / static_cast<double>(iterations);
    result.allocationBytesMax = allocationBytesMax;
    return result;
}

ResourceResult MeasureDtDynDataResize(const Scenario &scenario, const int iterations)
{
    const dtDynModel model = CompileSpec(scenario.spec);

    dtArray<double> samples;
    samples.resize(iterations);
    unsigned long long allocationSum = 0;
    unsigned long long allocationMax = 0;
    unsigned long long allocationBytesSum = 0;
    unsigned long long allocationBytesMax = 0;

    using Clock = std::chrono::steady_clock;
    for (int i = 0; i < iterations; ++i)
    {
        const unsigned long long allocationsBefore = g_allocationCount.load(std::memory_order_relaxed);
        const unsigned long long allocationBytesBefore = g_allocationBytes.load(std::memory_order_relaxed);
        g_countAllocations.store(true, std::memory_order_relaxed);
        const auto start = Clock::now();
        dtDynData data;
        data.Resize(model);
        const auto finish = Clock::now();
        g_countAllocations.store(false, std::memory_order_relaxed);

        const unsigned long long allocations =
            g_allocationCount.load(std::memory_order_relaxed) - allocationsBefore;
        const unsigned long long allocationBytes =
            g_allocationBytes.load(std::memory_order_relaxed) - allocationBytesBefore;
        const std::chrono::duration<double, std::micro> elapsed = finish - start;
        samples[static_cast<std::size_t>(i)] = elapsed.count();
        allocationSum += allocations;
        allocationMax = std::max(allocationMax, allocations);
        allocationBytesSum += allocationBytes;
        allocationBytesMax = std::max(allocationBytesMax, allocationBytes);
    }

    std::sort(samples.begin(), samples.end());
    double sum = 0.0;
    for (const double value : samples)
    {
        sum += value;
    }

    ResourceResult result;
    result.fixture = scenario.fixture;
    result.nq = model.nq;
    result.nv = model.nv;
    result.nbody = model.nbody;
    result.iterations = iterations;
    result.meanUs = sum / static_cast<double>(samples.size());
    result.medianUs = samples[samples.size() / 2];
    const std::size_t p95Index = std::min(samples.size() - 1,
                                          static_cast<std::size_t>(0.95 * static_cast<double>(samples.size() - 1)));
    result.p95Us = samples[p95Index];
    const std::size_t p99Index = std::min(samples.size() - 1,
                                          static_cast<std::size_t>(0.99 * static_cast<double>(samples.size() - 1)));
    result.p99Us = samples[p99Index];
    result.allocationMean = static_cast<double>(allocationSum) / static_cast<double>(iterations);
    result.allocationMax = allocationMax;
    result.allocationBytesMean = static_cast<double>(allocationBytesSum) / static_cast<double>(iterations);
    result.allocationBytesMax = allocationBytesMax;
    return result;
}

} // namespace

int main()
{
    constexpr int warmup = 20;
    constexpr int iterations = 200;
    const int batchCount = dt::Dynamics::benchmark::BenchmarkBatchCount();

    dtArray<Scenario> scenarios = MakeScenarios();
    dtArray<std::string> batchSummaryRows;

    std::cout << "fixture,state,scope,nq,nv,nbody,iterations,mean_us,median_us,p95_us,p99_us,"
                 "allocation_mean,allocation_max,allocation_bytes_mean,allocation_bytes_max,mass_symmetry_max,mass_symmetry_rms,"
                 "nonlin_consistency_max,nonlin_consistency_rms,centroidal_momentum_max,"
                 "centroidal_momentum_rms,com_velocity_max,com_velocity_rms\n";
    std::cout << std::fixed << std::setprecision(6);
    for (Scenario &scenario : scenarios)
    {
        const NativeScope scopes[] = {
            NativeScope::RneaComPos,
            NativeScope::CcrbaReduced,
            NativeScope::CcrbaAll,
        };
        for (const NativeScope scope : scopes)
        {
            dtArray<double> meanUs;
            dtArray<double> medianUs;
            dtArray<double> p95Us;
            dtArray<double> p99Us;
            meanUs.resize(batchCount);
            medianUs.resize(batchCount);
            p95Us.resize(batchCount);
            p99Us.resize(batchCount);
            unsigned long long allocationMax = 0;

            for (int batch = 0; batch < batchCount; ++batch)
            {
                const Result result = RunScenario(scenario, scope, warmup, iterations);
                meanUs[batch] = result.meanUs;
                medianUs[batch] = result.medianUs;
                p95Us[batch] = result.p95Us;
                p99Us[batch] = result.p99Us;
                allocationMax = std::max(allocationMax, result.allocationMax);

                if (batch == 0)
                {
                    std::cout << result.fixture << ','
                              << result.state << ','
                              << result.scope << ','
                              << result.nq << ','
                              << result.nv << ','
                              << result.nbody << ','
                              << result.iterations << ','
                              << result.meanUs << ','
                              << result.medianUs << ','
                              << result.p95Us << ','
                              << result.p99Us << ','
                              << result.allocationMean << ','
                              << result.allocationMax << ','
                              << result.allocationBytesMean << ','
                              << result.allocationBytesMax << ','
                              << result.massSymmetryMax << ','
                              << result.massSymmetryRms << ','
                              << result.nonlinConsistencyMax << ','
                              << result.nonlinConsistencyRms << ','
                              << result.centroidalMomentumMax << ','
                              << result.centroidalMomentumRms << ','
                              << result.comVelocityMax << ','
                              << result.comVelocityRms << '\n';
                }
            }

            if (batchCount > 1)
            {
                batchSummaryRows.push_back(dt::Dynamics::benchmark::BuildBatchSummaryRow("benchmark_dtDynamics",
                                                                                         scenario.fixture,
                                                                                         scenario.state,
                                                                                         NativeScopeName(scope),
                                                                                         batchCount,
                                                                                         meanUs,
                                                                                         medianUs,
                                                                                         p95Us,
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

    std::cout << "dtDynModel_resource,fixture,nq,nv,nbody,iterations,mean_us,median_us,p95_us,p99_us,"
                 "allocation_mean,allocation_max,allocation_bytes_mean,allocation_bytes_max\n";
    for (const Scenario &scenario : scenarios)
    {
        if (scenario.state != "nominal")
        {
            continue;
        }
        const ResourceResult result = MeasureCompileSpec(scenario, iterations);
        std::cout << "dtDynModel_resource,"
                  << result.fixture << ','
                  << result.nq << ','
                  << result.nv << ','
                  << result.nbody << ','
                  << result.iterations << ','
                  << result.meanUs << ','
                  << result.medianUs << ','
                  << result.p95Us << ','
                  << result.p99Us << ','
                  << result.allocationMean << ','
                  << result.allocationMax << ','
                  << result.allocationBytesMean << ','
                  << result.allocationBytesMax << '\n';
    }

    std::cout << "dtDynData_resource,fixture,nq,nv,nbody,iterations,mean_us,median_us,p95_us,p99_us,"
                 "allocation_mean,allocation_max,allocation_bytes_mean,allocation_bytes_max\n";
    for (const Scenario &scenario : scenarios)
    {
        if (scenario.state != "nominal")
        {
            continue;
        }
        const ResourceResult result = MeasureDtDynDataResize(scenario, iterations);
        std::cout << "dtDynData_resource,"
                  << result.fixture << ','
                  << result.nq << ','
                  << result.nv << ','
                  << result.nbody << ','
                  << result.iterations << ','
                  << result.meanUs << ','
                  << result.medianUs << ','
                  << result.p95Us << ','
                  << result.p99Us << ','
                  << result.allocationMean << ','
                  << result.allocationMax << ','
                  << result.allocationBytesMean << ','
                  << result.allocationBytesMax << '\n';
    }

    dt::Dynamics::benchmark::PrintBenchmarkMetadata(std::cout, "benchmark_dtDynamics", warmup, iterations);

    return 0;
}
