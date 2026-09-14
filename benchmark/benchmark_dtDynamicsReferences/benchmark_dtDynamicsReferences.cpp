#include <dtDynamics/dtArray.h>
#include <dtDynamics/dtDynCompile.h>
#include <dtDynamics/dtDynData.h>
#include <dtDynamics/dtDynDynamics.h>
#include <dtDynamics/dtDynKinematics.h>
#include <dtDynamics/dtDynSpec.h>
#include <dtDynamics/dtDynTypes.h>
#include <dtDynamics/dtDynUtils.h>

#include <mujoco/mujoco.h>

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "../dtDynBenchmarkMetadata.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

using namespace dt::Dynamics;

namespace
{

std::atomic<bool> g_countAllocations(false);
std::atomic<unsigned long long> g_allocationCount(0);
std::atomic<int> g_externalTimingEngine(0);
std::atomic<unsigned long long> g_mujocoNativeValidationRunsDuringTiming(0);
std::atomic<unsigned long long> g_pinocchioNativeValidationRunsDuringTiming(0);

class ExternalTimingGuard
{
public:
    explicit ExternalTimingGuard(const int engine)
    {
        const int previous = g_externalTimingEngine.exchange(engine, std::memory_order_relaxed);
        if (previous != 0)
        {
            g_externalTimingEngine.store(previous, std::memory_order_relaxed);
            throw std::logic_error("nested external-reference timing scope");
        }
    }

    ~ExternalTimingGuard()
    {
        g_externalTimingEngine.store(0, std::memory_order_relaxed);
    }

    ExternalTimingGuard(const ExternalTimingGuard &) = delete;
    ExternalTimingGuard &operator=(const ExternalTimingGuard &) = delete;
};

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

struct Result
{
    std::string engine;
    std::string fixture;
    std::string state;
    int nq = 0;
    int nv = 0;
    int nbody = 0;
    int iterations = 0;
    double meanUs = 0.0;
    double medianUs = 0.0;
    double p95Us = 0.0;
    double p99Us = 0.0;
    double allocationMean = 0.0;
    unsigned long long allocationMax = 0;
    std::string quantityScope;
};

struct ErrorSummary
{
    double max = 0.0;
    double maxRelative = 0.0;
    double squared = 0.0;
    int count = 0;
    bool finite = true;

    void Add(const double actual, const double expected)
    {
        ++count;
        if (!std::isfinite(actual) || !std::isfinite(expected))
        {
            finite = false;
            return;
        }
        const double error = std::fabs(actual - expected);
        const double scale = std::max(1.0, std::fabs(expected));
        const double relative = error / scale;
        const double squaredError = error * error;
        if (!std::isfinite(error) || !std::isfinite(scale) ||
            !std::isfinite(relative) || !std::isfinite(squaredError))
        {
            finite = false;
            return;
        }
        max = std::max(max, error);
        maxRelative = std::max(maxRelative, relative);
        squared += squaredError;
        if (!std::isfinite(squared))
        {
            finite = false;
        }
    }

    void Merge(const ErrorSummary &other)
    {
        finite = finite && other.finite;
        max = std::max(max, other.max);
        maxRelative = std::max(maxRelative, other.maxRelative);
        squared += other.squared;
        count += other.count;
        if (!std::isfinite(max) || !std::isfinite(maxRelative) || !std::isfinite(squared))
        {
            finite = false;
        }
    }

    double Rms() const
    {
        if (!finite)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return count == 0 ? 0.0 : std::sqrt(squared / static_cast<double>(count));
    }
};

struct ErrorRow
{
    std::string reference;
    std::string fixture;
    std::string state;
    std::string quantity;
    ErrorSummary summary;
    double absTolerance = 0.0;
    double relTolerance = 0.0;
    std::string status = "ok";
    std::string diagnostic;
};

struct TolerancePolicy
{
    double absTol = 1.0e-9;
    double relTol = 1.0e-9;
};

TolerancePolicy ToleranceFor(const std::string &reference,
                             const std::string &quantity,
                             const std::string &fixture)
{
    (void)reference;
    (void)fixture;
    if (quantity.find("site_jac") != std::string::npos ||
        quantity.find("cdof_dot") != std::string::npos ||
        quantity.find("cmAmatDot") != std::string::npos)
    {
        return {1.0e-8, 1.0e-8};
    }
    if (quantity.find("centMom") != std::string::npos ||
        quantity.find("subtree_com") != std::string::npos ||
        quantity.find("comJmat") != std::string::npos)
    {
        return {1.0e-9, 1.0e-9};
    }
    return {1.0e-10, 1.0e-10};
}

bool ErrorWithinTolerance(const ErrorSummary &summary, const TolerancePolicy &tolerance)
{
    const double rms = summary.Rms();
    return summary.count > 0 && summary.finite &&
           std::isfinite(summary.max) && std::isfinite(summary.maxRelative) && std::isfinite(rms) &&
           std::isfinite(tolerance.absTol) && std::isfinite(tolerance.relTol) &&
           tolerance.absTol >= 0.0 && tolerance.relTol >= 0.0 &&
           (summary.max <= tolerance.absTol || summary.maxRelative <= tolerance.relTol);
}

void ValidateNonFiniteErrorPolicy()
{
    ErrorSummary nonFiniteActual;
    nonFiniteActual.Add(std::numeric_limits<double>::quiet_NaN(), 0.0);
    ErrorSummary nonFiniteExpected;
    nonFiniteExpected.Add(0.0, std::numeric_limits<double>::infinity());
    ErrorSummary finite;
    finite.Add(0.0, 0.0);
    const TolerancePolicy nonFiniteTolerance{std::numeric_limits<double>::quiet_NaN(), 1.0e-9};
    if (ErrorWithinTolerance(nonFiniteActual, TolerancePolicy{}) ||
        ErrorWithinTolerance(nonFiniteExpected, TolerancePolicy{}) ||
        ErrorWithinTolerance(finite, nonFiniteTolerance))
    {
        throw std::logic_error("reference error policy accepted a non-finite value");
    }
}

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

struct MjFixture
{
    std::string name;
    std::string xml;
};

struct MjScratch
{
    dtArray<mjtNum> mass;
    dtArray<mjtNum> jacp;
    dtArray<mjtNum> jacr;
    dtArray<mjtNum> jacpDot;
    dtArray<mjtNum> jacrDot;
    dtArray<mjtNum> comJac;
    dtArray<mjtNum> angMomMat;
};

struct PinFreeJointMap
{
    int nativeQposAdr = 0;
    int nativeQvelAdr = 0;
    int pinQposAdr = 0;
    int pinQvelAdr = 0;
    double initialPosition[3] = {0.0, 0.0, 0.0};
};

struct PinFixture
{
    std::string name;
    pinocchio::Model model;
    dtArray<PinFreeJointMap> freeJoints;
    int nativeNq = 0;
    int nativeNv = 0;
    int nativeNbody = 0;
};

struct PinScratch
{
    pinocchio::Data data;
    Eigen::VectorXd q;
    Eigen::VectorXd nativeQpos;
    Eigen::VectorXd nativeQvel;
    Eigen::VectorXd v;
    Eigen::VectorXd zeroAcceleration;

    explicit PinScratch(const pinocchio::Model &model)
        : data(model)
    {
    }
};

dtArray<MjFixture> MujocoFixtures();
Eigen::MatrixXd VelocityTransform(const PinFixture &fixture,
                                  const Eigen::VectorXd &nativeQpos);

PinFreeJointMap MakeFreeJointMap(const int nativeQposAdr,
                                 const int nativeQvelAdr,
                                 const int pinQposAdr,
                                 const int pinQvelAdr,
                                 const double x,
                                 const double y,
                                 const double z)
{
    PinFreeJointMap map;
    map.nativeQposAdr = nativeQposAdr;
    map.nativeQvelAdr = nativeQvelAdr;
    map.pinQposAdr = pinQposAdr;
    map.pinQvelAdr = pinQvelAdr;
    map.initialPosition[0] = x;
    map.initialPosition[1] = y;
    map.initialPosition[2] = z;
    return map;
}

dtArray<PinFreeJointMap> NoFreeJointMap()
{
    return {};
}

dtArray<PinFreeJointMap> SingleFreeJointMap(const double x, const double y, const double z)
{
    dtArray<PinFreeJointMap> freeJoints;
    freeJoints.push_back(MakeFreeJointMap(0, 0, 0, 0, x, y, z));
    return freeJoints;
}

dtArray<PinFreeJointMap> MultiFloatingFreeJointMap()
{
    dtArray<PinFreeJointMap> freeJoints;
    freeJoints.push_back(MakeFreeJointMap(0, 0, 0, 0, 0.4, 0.25, 0.6));
    freeJoints.push_back(MakeFreeJointMap(9, 8, 9, 8, -0.35, -0.2, 0.7));
    return freeJoints;
}

bool HasFloatingBase(const PinFixture &fixture)
{
    return !fixture.freeJoints.empty();
}

bool HasSingleFloatingRoot(const PinFixture &fixture)
{
    return fixture.freeJoints.size() == 1;
}

bool IsFreeQposIndex(const PinFixture &fixture, const Eigen::Index index)
{
    for (const PinFreeJointMap &freeJoint : fixture.freeJoints)
    {
        if (index >= freeJoint.nativeQposAdr && index < freeJoint.nativeQposAdr + 7)
        {
            return true;
        }
    }
    return false;
}

bool IsFreeQvelIndex(const PinFixture &fixture, const Eigen::Index index)
{
    for (const PinFreeJointMap &freeJoint : fixture.freeJoints)
    {
        if (index >= freeJoint.nativeQvelAdr && index < freeJoint.nativeQvelAdr + 6)
        {
            return true;
        }
    }
    return false;
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

void SetGravity(pinocchio::Model &model)
{
    model.gravity.linear() = Vector3(0.0, 0.0, -9.81);
    model.gravity.angular().setZero();
}

pinocchio::JointIndex AddRevoluteBody(pinocchio::Model &model,
                                      const pinocchio::JointIndex parentJoint,
                                      const std::string &jointName,
                                      const Eigen::Vector3d &jointTranslation,
                                      const Eigen::Vector3d &jointAxis,
                                      const double mass,
                                      const Eigen::Vector3d &com,
                                      const Eigen::Vector3d &diagonalInertia,
                                      const double armature = 0.0)
{
    const pinocchio::JointIndex jointId =
        model.addJoint(parentJoint,
                       pinocchio::JointModelRevoluteUnaligned(jointAxis),
                       Placement(jointTranslation),
                       jointName);
    model.appendBodyToJoint(jointId,
                            BodyInertia(mass,
                                        com,
                                        DiagonalInertia(diagonalInertia(0),
                                                        diagonalInertia(1),
                                                        diagonalInertia(2))),
                            pinocchio::SE3::Identity());
    model.armature(model.idx_vs[jointId]) = armature;
    return jointId;
}

pinocchio::JointIndex AddFreeBody(pinocchio::Model &model,
                                  const std::string &jointName,
                                  const double mass,
                                  const Eigen::Vector3d &diagonalInertia)
{
    const pinocchio::JointIndex jointId =
        model.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), jointName);
    model.appendBodyToJoint(jointId,
                            BodyInertia(mass,
                                        Vector3(0.0, 0.0, 0.0),
                                        DiagonalInertia(diagonalInertia(0),
                                                        diagonalInertia(1),
                                                        diagonalInertia(2))),
                            pinocchio::SE3::Identity());
    return jointId;
}

Eigen::Vector3d LinkCom(const double x, const double y, const double z)
{
    return Vector3(0.5 * x, 0.5 * y, 0.5 * z);
}

Eigen::Vector3d LinkInertia(const double mass)
{
    return Vector3(0.01 * mass + 0.001,
                   0.012 * mass + 0.001,
                   0.014 * mass + 0.001);
}

std::string LinkXml(const std::string &name,
                    const double x,
                    const double y,
                    const double z,
                    const double mass,
                    const double axisX,
                    const double axisY,
                    const double axisZ)
{
    const Eigen::Vector3d com = LinkCom(x, y, z);
    const Eigen::Vector3d inertia = LinkInertia(mass);
    return "<body name=\"" + name + "\" pos=\"" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + "\">"
           "<inertial pos=\"" +
           std::to_string(com(0)) + " " + std::to_string(com(1)) + " " + std::to_string(com(2)) +
           "\" mass=\"" + std::to_string(mass) +
           "\" diaginertia=\"" + std::to_string(inertia(0)) + " " + std::to_string(inertia(1)) + " " +
           std::to_string(inertia(2)) + "\"/>"
           "<joint name=\"" +
           name + "_joint\" type=\"hinge\" axis=\"" + std::to_string(axisX) + " " +
           std::to_string(axisY) + " " + std::to_string(axisZ) + "\"/>";
}

std::string CloseBody()
{
    return "</body>";
}

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

Eigen::Vector3d FreeJointPosition(const PinFreeJointMap &freeJoint)
{
    return Eigen::Vector3d(freeJoint.initialPosition[0],
                           freeJoint.initialPosition[1],
                           freeJoint.initialPosition[2]);
}

void SetNativeFreeJointState(const std::string &state,
                             const PinFreeJointMap &freeJoint,
                             const int freeJointIndex,
                             Eigen::VectorXd &qpos,
                             Eigen::VectorXd &qvel)
{
    const int qadr = freeJoint.nativeQposAdr;
    const int dadr = freeJoint.nativeQvelAdr;
    const double scale = static_cast<double>(freeJointIndex + 1);
    Eigen::Vector3d position = FreeJointPosition(freeJoint);
    double angle = 0.0;

    if (state == "asymmetric")
    {
        angle = 0.04 * scale;
        position(0) += 0.03 * scale;
        position(1) -= 0.02 * scale;
        position(2) += 0.01 * scale;
    }
    else if (state == "nonzero_velocity")
    {
        angle = 0.03 * scale;
        position(0) += 0.02 * std::cos(0.4 * scale);
        position(1) += 0.02 * std::sin(0.5 * scale);
        position(2) -= 0.01 * scale;
        for (int offset = 0; offset < 6; ++offset)
        {
            qvel(dadr + offset) =
                0.08 * std::sin(0.3 * static_cast<double>(dadr + offset + 1));
        }
    }

    qpos.segment<3>(qadr) = position;
    const int axisSelector = (freeJointIndex % 3) - 1;
    qpos(qadr + 3) = std::cos(0.5 * angle);
    qpos(qadr + 4) = std::sin(0.5 * angle) * (axisSelector < 0 ? 1.0 : 0.0);
    qpos(qadr + 5) = std::sin(0.5 * angle) * (axisSelector == 0 ? 1.0 : 0.0);
    qpos(qadr + 6) = std::sin(0.5 * angle) * (axisSelector > 0 ? 1.0 : 0.0);
}

void SetNativeState(const std::string &state,
                    const PinFixture &fixture,
                    Eigen::VectorXd &qpos,
                    Eigen::VectorXd &qvel)
{
    qpos.setZero();
    qvel.setZero();

    for (std::size_t freeIdx = 0; freeIdx < fixture.freeJoints.size(); ++freeIdx)
    {
        SetNativeFreeJointState(state,
                                fixture.freeJoints[freeIdx],
                                static_cast<int>(freeIdx),
                                qpos,
                                qvel);
    }

    if (state == "asymmetric")
    {
        for (Eigen::Index i = 0; i < qpos.size(); ++i)
        {
            if (IsFreeQposIndex(fixture, i))
            {
                continue;
            }
            qpos(i) += 0.07 * std::sin(0.7 * static_cast<double>(i + 1));
        }
    }
    else if (state == "nonzero_velocity")
    {
        for (Eigen::Index i = 0; i < qpos.size(); ++i)
        {
            if (IsFreeQposIndex(fixture, i))
            {
                continue;
            }
            qpos(i) += 0.03 * std::cos(0.5 * static_cast<double>(i + 1));
        }
        for (Eigen::Index i = 0; i < qvel.size(); ++i)
        {
            if (IsFreeQvelIndex(fixture, i))
            {
                continue;
            }
            qvel(i) = 0.2 * std::sin(0.4 * static_cast<double>(i + 1));
        }
    }

    for (const PinFreeJointMap &freeJoint : fixture.freeJoints)
    {
        const int qadr = freeJoint.nativeQposAdr;
        Eigen::Quaterniond quat(qpos(qadr + 3),
                                qpos(qadr + 4),
                                qpos(qadr + 5),
                                qpos(qadr + 6));
        quat.normalize();
        qpos(qadr + 3) = quat.w();
        qpos(qadr + 4) = quat.x();
        qpos(qadr + 5) = quat.y();
        qpos(qadr + 6) = quat.z();
    }
}

void SetMujocoState(const std::string &state, const mjModel *model, mjData *data)
{
    for (int i = 0; i < model->nq; ++i)
    {
        data->qpos[i] = 0.0;
    }
    for (int i = 0; i < model->nv; ++i)
    {
        data->qvel[i] = 0.0;
    }

    const auto setQuaternion = [data](const int qadr, const double angle, const int axisSelector)
    {
        data->qpos[qadr + 0] = std::cos(0.5 * angle);
        data->qpos[qadr + 1] = std::sin(0.5 * angle) * (axisSelector < 0 ? 1.0 : 0.0);
        data->qpos[qadr + 2] = std::sin(0.5 * angle) * (axisSelector == 0 ? 1.0 : 0.0);
        data->qpos[qadr + 3] = std::sin(0.5 * angle) * (axisSelector > 0 ? 1.0 : 0.0);
    };

    for (int jointId = 0; jointId < model->njnt; ++jointId)
    {
        const int qadr = model->jnt_qposadr[jointId];
        const int dadr = model->jnt_dofadr[jointId];
        const int bodyId = model->jnt_bodyid[jointId];
        const double scale = static_cast<double>(jointId + 1);

        if (model->jnt_type[jointId] == mjJNT_FREE)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                data->qpos[qadr + axis] = model->body_pos[3 * bodyId + axis];
            }

            double angle = 0.0;
            if (state == "asymmetric")
            {
                angle = 0.04 * scale;
                data->qpos[qadr + 0] += 0.03 * scale;
                data->qpos[qadr + 1] -= 0.02 * scale;
                data->qpos[qadr + 2] += 0.01 * scale;
            }
            else if (state == "nonzero_velocity")
            {
                angle = 0.03 * scale;
                data->qpos[qadr + 0] += 0.02 * std::cos(0.4 * scale);
                data->qpos[qadr + 1] += 0.02 * std::sin(0.5 * scale);
                data->qpos[qadr + 2] -= 0.01 * scale;
                for (int offset = 0; offset < 6; ++offset)
                {
                    data->qvel[dadr + offset] = 0.08 * std::sin(0.3 * static_cast<double>(dadr + offset + 1));
                }
            }
            setQuaternion(qadr + 3, angle, (jointId % 3) - 1);
        }
        else if (model->jnt_type[jointId] == mjJNT_BALL)
        {
            const double angle = (state == "asymmetric" || state == "nonzero_velocity") ? 0.03 * scale : 0.0;
            setQuaternion(qadr, angle, (jointId % 3) - 1);
            if (state == "nonzero_velocity")
            {
                for (int offset = 0; offset < 3; ++offset)
                {
                    data->qvel[dadr + offset] = 0.08 * std::sin(0.3 * static_cast<double>(dadr + offset + 1));
                }
            }
        }
        else
        {
            if (state == "asymmetric")
            {
                data->qpos[qadr] = 0.07 * std::sin(0.7 * scale);
            }
            else if (state == "nonzero_velocity")
            {
                data->qpos[qadr] = 0.03 * std::cos(0.5 * scale);
                data->qvel[dadr] = 0.2 * std::sin(0.4 * scale);
            }
        }
    }

    mj_normalizeQuat(model, data->qpos);
}

void SetPinocchioState(const std::string &state, const PinFixture &fixture, PinScratch &scratch)
{
    scratch.nativeQpos.resize(fixture.nativeNq);
    scratch.nativeQvel.resize(fixture.nativeNv);
    SetNativeState(state, fixture, scratch.nativeQpos, scratch.nativeQvel);

    scratch.q.resize(fixture.model.nq);
    scratch.v.resize(fixture.model.nv);
    scratch.zeroAcceleration.resize(fixture.model.nv);
    scratch.zeroAcceleration.setZero();

    scratch.q = scratch.nativeQpos;
    for (const PinFreeJointMap &freeJoint : fixture.freeJoints)
    {
        const int nativeQadr = freeJoint.nativeQposAdr;
        const int pinQadr = freeJoint.pinQposAdr;
        scratch.q(pinQadr + 0) = scratch.nativeQpos(nativeQadr + 0);
        scratch.q(pinQadr + 1) = scratch.nativeQpos(nativeQadr + 1);
        scratch.q(pinQadr + 2) = scratch.nativeQpos(nativeQadr + 2);
        scratch.q(pinQadr + 3) = scratch.nativeQpos(nativeQadr + 4);
        scratch.q(pinQadr + 4) = scratch.nativeQpos(nativeQadr + 5);
        scratch.q(pinQadr + 5) = scratch.nativeQpos(nativeQadr + 6);
        scratch.q(pinQadr + 6) = scratch.nativeQpos(nativeQadr + 3);
    }
    scratch.v = VelocityTransform(fixture, scratch.nativeQpos) * scratch.nativeQvel;
}

void InitMujocoScratch(const mjModel *model, MjScratch &scratch)
{
    const std::size_t nv = static_cast<std::size_t>(model->nv);
    scratch.mass.assign(nv * nv, 0.0);
    scratch.jacp.assign(3 * nv, 0.0);
    scratch.jacr.assign(3 * nv, 0.0);
    scratch.jacpDot.assign(3 * nv, 0.0);
    scratch.jacrDot.assign(3 * nv, 0.0);
    scratch.comJac.assign(3 * nv, 0.0);
    scratch.angMomMat.assign(3 * nv, 0.0);
}

void ComputeMujocoIngredients(const mjModel *model, mjData *data, MjScratch &scratch)
{
    mj_forward(model, data);
    mj_subtreeVel(model, data);
    mj_fullM(model, scratch.mass.data(), data->qM);

    for (int siteId = 0; siteId < model->nsite; ++siteId)
    {
        mj_jacSite(model, data, scratch.jacp.data(), scratch.jacr.data(), siteId);
        const int bodyId = model->site_bodyid[siteId];
        mj_jacDot(model,
                  data,
                  scratch.jacpDot.data(),
                  scratch.jacrDot.data(),
                  data->site_xpos + 3 * siteId,
                  bodyId);
    }

    for (int bodyId = 1; bodyId < model->nbody; ++bodyId)
    {
        if (model->body_parentid[bodyId] == 0)
        {
            mj_jacSubtreeCom(model, data, scratch.comJac.data(), bodyId);
            mj_angmomMat(model, data, scratch.angMomMat.data(), bodyId);
        }
    }
}

void ComputePinocchioIngredients(const PinFixture &fixture, PinScratch &scratch)
{
    pinocchio::crba(fixture.model, scratch.data, scratch.q, pinocchio::Convention::WORLD);
    scratch.data.M.triangularView<Eigen::StrictlyLower>() =
        scratch.data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::computeGeneralizedGravity(fixture.model, scratch.data, scratch.q);
    pinocchio::nonLinearEffects(fixture.model, scratch.data, scratch.q, scratch.v);
    pinocchio::computeCoriolisMatrix(fixture.model, scratch.data, scratch.q, scratch.v);
    pinocchio::ccrba(fixture.model, scratch.data, scratch.q, scratch.v);
    pinocchio::computeCentroidalMapTimeVariation(fixture.model, scratch.data, scratch.q, scratch.v);
    pinocchio::computeCentroidalMomentumTimeVariation(fixture.model,
                                                      scratch.data,
                                                      scratch.q,
                                                      scratch.v,
                                                      scratch.zeroAcceleration);
}

void RecordError(dtArray<ErrorRow> &rows,
                 const std::string &reference,
                 const std::string &fixture,
                 const std::string &state,
                 const std::string &quantity,
                 const ErrorSummary &summary)
{
    ErrorRow row;
    row.reference = reference;
    row.fixture = fixture;
    row.state = state;
    row.quantity = quantity;
    row.summary = summary;
    const TolerancePolicy tolerance = ToleranceFor(reference, quantity, fixture);
    row.absTolerance = tolerance.absTol;
    row.relTolerance = tolerance.relTol;
    if (!ErrorWithinTolerance(summary, tolerance))
    {
        row.status = "fail";
        const double rms = summary.Rms();
        const bool finiteValues = summary.finite &&
                                  std::isfinite(summary.max) &&
                                  std::isfinite(summary.maxRelative) &&
                                  std::isfinite(rms) &&
                                  std::isfinite(tolerance.absTol) &&
                                  std::isfinite(tolerance.relTol);
        row.diagnostic = finiteValues ? "exceeds tolerance" : "non-finite comparison or tolerance";
    }
    rows.push_back(row);
}

void RecordSkipped(dtArray<ErrorRow> &rows,
                   const std::string &reference,
                   const std::string &fixture,
                   const std::string &state,
                   const std::string &quantity,
                   const std::string &diagnostic)
{
    ErrorRow row;
    row.reference = reference;
    row.fixture = fixture;
    row.state = state;
    row.quantity = quantity;
    const TolerancePolicy tolerance = ToleranceFor(reference, quantity, fixture);
    row.absTolerance = tolerance.absTol;
    row.relTolerance = tolerance.relTol;
    row.status = "skipped";
    row.diagnostic = diagnostic;
    rows.push_back(row);
}

struct NativeRun
{
    dtDynModel model;
    dtDynData data;
};

dt::Math::Vector3<dtDynReal> Gravity()
{
    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = 0.0;
    gravity(2) = -9.81;
    return gravity;
}

NativeRun ComputeNativeRun(const std::string &xml,
                           const Eigen::VectorXd &qpos,
                           const Eigen::VectorXd &qvel)
{
    const int externalEngine = g_externalTimingEngine.load(std::memory_order_relaxed);
    if (externalEngine == 1)
    {
        g_mujocoNativeValidationRunsDuringTiming.fetch_add(1, std::memory_order_relaxed);
    }
    else if (externalEngine == 2)
    {
        g_pinocchioNativeValidationRunsDuringTiming.fetch_add(1, std::memory_order_relaxed);
    }

    NativeRun run;
    const dtDynSpec spec = dtDynSpec::DecodeXml(xml);
    run.model = CompileSpec(spec);
    run.data.Resize(run.model);

    for (int i = 0; i < run.model.nq; ++i)
    {
        run.data.qpos(i) = qpos(i);
    }
    for (int i = 0; i < run.model.nv; ++i)
    {
        run.data.qvel(i) = qvel(i);
    }

    ComputeDynamicsCCRBA(run.model, run.data, Gravity(), DTDYN_CCRBA_TERM_ALL);
    ComputeJacobians(run.model, run.data);
    return run;
}

Eigen::VectorXd MjQpos(const mjModel *model, const mjData *data)
{
    Eigen::VectorXd qpos(model->nq);
    for (int i = 0; i < model->nq; ++i)
    {
        qpos(i) = data->qpos[i];
    }
    return qpos;
}

Eigen::VectorXd MjQvel(const mjModel *model, const mjData *data)
{
    Eigen::VectorXd qvel(model->nv);
    for (int i = 0; i < model->nv; ++i)
    {
        qvel(i) = data->qvel[i];
    }
    return qvel;
}

ErrorSummary DenseMatrixMjtError(const dtDynDenseMatrix &actual,
                                 const dtArray<mjtNum> &expected,
                                 const int rows,
                                 const int cols)
{
    ErrorSummary summary;
    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            summary.Add(actual(row, col), expected[static_cast<std::size_t>(row * cols + col)]);
        }
    }
    return summary;
}

ErrorSummary DenseVectorEigenError(const dtDynDenseVector &actual,
                                   const Eigen::VectorXd &expected)
{
    ErrorSummary summary;
    for (int row = 0; row < actual.GetDim(); ++row)
    {
        summary.Add(actual(row), expected(row));
    }
    return summary;
}

ErrorSummary DenseMatrixEigenError(const dtDynDenseMatrix &actual,
                                   const Eigen::MatrixXd &expected)
{
    ErrorSummary summary;
    for (int row = 0; row < actual.GetRowSize(); ++row)
    {
        for (int col = 0; col < actual.GetColSize(); ++col)
        {
            summary.Add(actual(row, col), expected(row, col));
        }
    }
    return summary;
}

ErrorSummary DenseMatrixEigenRowError(const dtDynDenseMatrix &actual,
                                      const Eigen::MatrixXd &expected,
                                      const int rowBegin,
                                      const int rowEnd)
{
    ErrorSummary summary;
    for (int row = rowBegin; row < rowEnd; ++row)
    {
        for (int col = 0; col < actual.GetColSize(); ++col)
        {
            summary.Add(actual(row, col), expected(row, col));
        }
    }
    return summary;
}

ErrorSummary SpatialArrayError(const dtArray<dt::Math::Vector6<dtDynReal>> &actual,
                               const mjtNum *expected,
                               const int count)
{
    ErrorSummary summary;
    for (int id = 0; id < count; ++id)
    {
        for (int row = 0; row < 6; ++row)
        {
            summary.Add(actual[static_cast<std::size_t>(id)](row), expected[6 * id + row]);
        }
    }
    return summary;
}

Eigen::VectorXd DenseMatrixTimesVector(const dtDynDenseMatrix &matrix,
                                       const Eigen::VectorXd &vector)
{
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

Eigen::MatrixXd VelocityTransform(const PinFixture &fixture,
                                  const Eigen::VectorXd &nativeQpos)
{
    Eigen::MatrixXd transform = Eigen::MatrixXd::Identity(fixture.nativeNv, fixture.nativeNv);
    for (const PinFreeJointMap &freeJoint : fixture.freeJoints)
    {
        const int qadr = freeJoint.nativeQposAdr;
        const int vadr = freeJoint.nativeQvelAdr;
        Eigen::Quaterniond quat(nativeQpos(qadr + 3),
                                nativeQpos(qadr + 4),
                                nativeQpos(qadr + 5),
                                nativeQpos(qadr + 6));
        quat.normalize();
        transform.block<3, 3>(vadr, vadr) = quat.toRotationMatrix().transpose();
    }
    return transform;
}

Eigen::MatrixXd VelocityTransformDot(const PinFixture &fixture,
                                     const Eigen::VectorXd &nativeQpos,
                                     const Eigen::VectorXd &nativeQvel)
{
    Eigen::MatrixXd transformDot = Eigen::MatrixXd::Zero(fixture.nativeNv, fixture.nativeNv);
    for (const PinFreeJointMap &freeJoint : fixture.freeJoints)
    {
        const int qadr = freeJoint.nativeQposAdr;
        const int vadr = freeJoint.nativeQvelAdr;
        Eigen::Quaterniond quat(nativeQpos(qadr + 3),
                                nativeQpos(qadr + 4),
                                nativeQpos(qadr + 5),
                                nativeQpos(qadr + 6));
        quat.normalize();
        const Eigen::Vector3d angularBody(nativeQvel(vadr + 3),
                                          nativeQvel(vadr + 4),
                                          nativeQvel(vadr + 5));
        transformDot.block<3, 3>(vadr, vadr) = -Skew(angularBody) * quat.toRotationMatrix().transpose();
    }
    return transformDot;
}

std::string FixtureXmlByName(const std::string &name)
{
    const dtArray<MjFixture> fixtures = MujocoFixtures();
    for (const MjFixture &fixture : fixtures)
    {
        if (fixture.name == name)
        {
            return fixture.xml;
        }
    }
    throw std::runtime_error("No benchmark XML fixture named " + name);
}

void AppendMujocoErrors(dtArray<ErrorRow> &rows,
                        const MjFixture &fixture,
                        const std::string &state,
                        const mjModel *model,
                        mjData *data)
{
    MjScratch scratch;
    InitMujocoScratch(model, scratch);
    ComputeMujocoIngredients(model, data, scratch);

    NativeRun native = ComputeNativeRun(fixture.xml, MjQpos(model, data), MjQvel(model, data));
    RecordError(rows, "mujoco", fixture.name, state, "Mmat",
                DenseMatrixMjtError(native.data.Mmat, scratch.mass, model->nv, model->nv));

    MjDataPtr zeroVelocityData(mj_makeData(model));
    if (zeroVelocityData == nullptr)
    {
        throw std::runtime_error(fixture.name + ": MuJoCo zero-velocity data allocation failed");
    }
    for (int i = 0; i < model->nq; ++i)
    {
        zeroVelocityData->qpos[i] = data->qpos[i];
    }
    for (int i = 0; i < model->nv; ++i)
    {
        zeroVelocityData->qvel[i] = 0.0;
    }
    mj_forward(model, zeroVelocityData.get());

    ErrorSummary gravityError;
    ErrorSummary coriolisError;
    ErrorSummary nonlinError;
    for (int dof = 0; dof < model->nv; ++dof)
    {
        const double gravity = zeroVelocityData->qfrc_bias[dof];
        gravityError.Add(native.data.gravityTorq(dof), gravity);
        coriolisError.Add(native.data.coriolisTorq(dof), data->qfrc_bias[dof] - gravity);
        nonlinError.Add(native.data.nonlinTorq(dof), data->qfrc_bias[dof]);
    }
    RecordError(rows, "mujoco", fixture.name, state, "gravityTorq", gravityError);
    RecordError(rows, "mujoco", fixture.name, state, "coriolisTorq", coriolisError);
    RecordError(rows, "mujoco", fixture.name, state, "nonlinTorq", nonlinError);

    RecordError(rows, "mujoco", fixture.name, state, "cdof",
                SpatialArrayError(native.data.cdof, data->cdof, model->nv));
    RecordError(rows, "mujoco", fixture.name, state, "cdof_dot",
                SpatialArrayError(native.data.cdof_dot, data->cdof_dot, model->nv));
    RecordError(rows, "mujoco", fixture.name, state, "cvel",
                SpatialArrayError(native.data.cvel, data->cvel, model->nbody));

    ErrorSummary siteJacpError;
    ErrorSummary siteJacrError;
    ErrorSummary siteJacpDotError;
    ErrorSummary siteJacrDotError;
    for (int siteId = 0; siteId < model->nsite; ++siteId)
    {
        mj_jacSite(model, data, scratch.jacp.data(), scratch.jacr.data(), siteId);
        mj_jacDot(model,
                  data,
                  scratch.jacpDot.data(),
                  scratch.jacrDot.data(),
                  data->site_xpos + 3 * siteId,
                  model->site_bodyid[siteId]);
        siteJacpError.Merge(DenseMatrixMjtError(native.data.site_jacp[static_cast<std::size_t>(siteId)],
                                                scratch.jacp,
                                                3,
                                                model->nv));
        siteJacrError.Merge(DenseMatrixMjtError(native.data.site_jacr[static_cast<std::size_t>(siteId)],
                                                scratch.jacr,
                                                3,
                                                model->nv));
        siteJacpDotError.Merge(DenseMatrixMjtError(native.data.site_jacp_dot[static_cast<std::size_t>(siteId)],
                                                   scratch.jacpDot,
                                                   3,
                                                   model->nv));
        siteJacrDotError.Merge(DenseMatrixMjtError(native.data.site_jacr_dot[static_cast<std::size_t>(siteId)],
                                                   scratch.jacrDot,
                                                   3,
                                                   model->nv));
    }
    RecordError(rows, "mujoco", fixture.name, state, "site_jacp", siteJacpError);
    RecordError(rows, "mujoco", fixture.name, state, "site_jacr", siteJacrError);
    RecordError(rows, "mujoco", fixture.name, state, "site_jacp_dot", siteJacpDotError);
    RecordError(rows, "mujoco", fixture.name, state, "site_jacr_dot", siteJacrDotError);

    ErrorSummary subtreeComError;
    for (int bodyId = 0; bodyId < model->nbody; ++bodyId)
    {
        for (int row = 0; row < 3; ++row)
        {
            subtreeComError.Add(native.data.subtree_com[static_cast<std::size_t>(bodyId)](row),
                                data->subtree_com[3 * bodyId + row]);
        }
    }
    RecordError(rows, "mujoco", fixture.name, state, "subtree_com", subtreeComError);

    ErrorSummary comJmatError;
    ErrorSummary cmAmatError;
    ErrorSummary centMomError;
    for (dtDynIndex rootIdx = 0; rootIdx < native.model.nroot; ++rootIdx)
    {
        const dtDynIndex rootBody = native.model.root_bodyid[static_cast<std::size_t>(rootIdx)];
        const dtDynIndex comRow = utils::RootComRow(rootIdx);
        const dtDynIndex linearRow = utils::RootCentroidalLinearRow(rootIdx);
        const dtDynIndex angularRow = utils::RootCentroidalAngularRow(rootIdx);
        const dtDynReal rootMass = native.model.root_mass(rootIdx);

        mj_jacSubtreeCom(model, data, scratch.comJac.data(), rootBody);
        mj_angmomMat(model, data, scratch.angMomMat.data(), rootBody);

        for (int row = 0; row < 3; ++row)
        {
            centMomError.Add(native.data.centMom(linearRow + row),
                             rootMass * data->subtree_linvel[3 * rootBody + row]);
            centMomError.Add(native.data.centMom(angularRow + row),
                             data->subtree_angmom[3 * rootBody + row]);

            for (int col = 0; col < model->nv; ++col)
            {
                const std::size_t index = static_cast<std::size_t>(row * model->nv + col);
                comJmatError.Add(native.data.comJmat(comRow + row, col), scratch.comJac[index]);
                cmAmatError.Add(native.data.cmAmat(linearRow + row, col), rootMass * scratch.comJac[index]);
                cmAmatError.Add(native.data.cmAmat(angularRow + row, col), scratch.angMomMat[index]);
            }
        }
    }
    RecordError(rows, "mujoco", fixture.name, state, "comJmat", comJmatError);
    RecordError(rows, "mujoco", fixture.name, state, "cmAmat", cmAmatError);
    RecordError(rows, "mujoco", fixture.name, state, "centMom", centMomError);
}

void AppendPinocchioErrors(dtArray<ErrorRow> &rows,
                           const PinFixture &fixture,
                           const std::string &state,
                           PinScratch &scratch)
{
    ComputePinocchioIngredients(fixture, scratch);
    NativeRun native = ComputeNativeRun(FixtureXmlByName(fixture.name),
                                        scratch.nativeQpos,
                                        scratch.nativeQvel);

    const Eigen::MatrixXd velocityTransform = VelocityTransform(fixture, scratch.nativeQpos);
    const Eigen::MatrixXd velocityTransformDot =
        VelocityTransformDot(fixture, scratch.nativeQpos, scratch.nativeQvel);
    const Eigen::VectorXd velocityTransformDotV = velocityTransformDot * scratch.nativeQvel;

    RecordError(rows, "pinocchio", fixture.name, state, "Mmat",
                DenseMatrixEigenError(native.data.Mmat,
                                      velocityTransform.transpose() * scratch.data.M * velocityTransform));
    RecordError(rows, "pinocchio", fixture.name, state, "gravityTorq",
                DenseVectorEigenError(native.data.gravityTorq,
                                      velocityTransform.transpose() * scratch.data.g));
    RecordError(rows, "pinocchio", fixture.name, state, "nonlinTorq",
                DenseVectorEigenError(native.data.nonlinTorq,
                                      velocityTransform.transpose() *
                                          (scratch.data.nle + scratch.data.M * velocityTransformDotV)));
    RecordError(rows, "pinocchio", fixture.name, state, "coriolisTorq",
                DenseVectorEigenError(native.data.coriolisTorq,
                                      velocityTransform.transpose() *
                                          (scratch.data.C * scratch.v + scratch.data.M * velocityTransformDotV)));
    if (HasSingleFloatingRoot(fixture))
    {
        RecordError(rows, "pinocchio", fixture.name, state, "cmAmat",
                    DenseMatrixEigenError(native.data.cmAmat, scratch.data.Ag * velocityTransform));
        RecordError(rows, "pinocchio", fixture.name, state, "centMom",
                    DenseVectorEigenError(native.data.centMom, scratch.data.hg.toVector()));

        pinocchio::computeCentroidalMomentumTimeVariation(fixture.model,
                                                          scratch.data,
                                                          scratch.q,
                                                          scratch.v,
                                                          velocityTransformDotV);
        const Eigen::VectorXd nativeCentroidalMomentumRate =
            DenseMatrixTimesVector(native.data.cmAmatDot, scratch.nativeQvel);
        ErrorSummary cmAmatDotQvelError;
        for (Eigen::Index row = 0; row < nativeCentroidalMomentumRate.size(); ++row)
        {
            cmAmatDotQvelError.Add(nativeCentroidalMomentumRate(row), scratch.data.dhg.toVector()(row));
        }
        RecordError(rows, "pinocchio", fixture.name, state, "cmAmatDot_qvel", cmAmatDotQvelError);
    }
    else if (HasFloatingBase(fixture))
    {
        constexpr const char *diagnostic =
            "multi-root native centroidal outputs are per-root stacked; Pinocchio reference is whole-system centroidal";
        RecordSkipped(rows, "pinocchio", fixture.name, state, "cmAmat", diagnostic);
        RecordSkipped(rows, "pinocchio", fixture.name, state, "centMom", diagnostic);
        RecordSkipped(rows, "pinocchio", fixture.name, state, "cmAmatDot_qvel", diagnostic);
    }
    else
    {
        constexpr const char *diagnostic =
            "fixed-base Pinocchio fixture omits world-fixed inertial bodies used by native/MuJoCo centroidal reference";
        RecordSkipped(rows, "pinocchio", fixture.name, state, "cmAmat", diagnostic);
        RecordSkipped(rows, "pinocchio", fixture.name, state, "centMom", diagnostic);
        RecordSkipped(rows, "pinocchio", fixture.name, state, "cmAmatDot_qvel", diagnostic);
    }
}

template <typename Callable>
Result RunTimed(const std::string &engine,
                const std::string &fixture,
                const std::string &state,
                const int nq,
                const int nv,
                const int nbody,
                const std::string &quantityScope,
                Callable &&compute)
{
    constexpr int warmup = 20;
    constexpr int iterations = 200;

    for (int i = 0; i < warmup; ++i)
    {
        compute();
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
        compute();
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
                                                    "benchmark_dtDynamicsReferences",
                                                    engine,
                                                    fixture,
                                                    state,
                                                    quantityScope,
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
    result.engine = engine;
    result.fixture = fixture;
    result.state = state;
    result.nq = nq;
    result.nv = nv;
    result.nbody = nbody;
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
    result.quantityScope = quantityScope;
    return result;
}

dtArray<std::string> States()
{
    return {"nominal", "asymmetric", "nonzero_velocity"};
}


std::string SerialChainXml(const int linkCount)
{
    std::string xml =
        "<mujoco model=\"serial_chain\"><option gravity=\"0 0 -9.81\"/>"
        "<worldbody><body name=\"base\" pos=\"0 0 0\">"
        "<inertial pos=\"0 0 0\" mass=\"2\" diaginertia=\"0.04 0.04 0.04\"/>";
    for (int i = 0; i < linkCount; ++i)
    {
        const double length = 0.12 + 0.002 * static_cast<double>(i % 5);
        const double y = 0.01 * static_cast<double>((i % 2) == 0 ? 1 : -1);
        const double z = 0.02 * static_cast<double>((i % 3) - 1);
        const double mass = 0.35 + 0.01 * static_cast<double>(i % 7);
        const double ax = (i % 3) == 0 ? 1.0 : 0.0;
        const double ay = (i % 3) == 1 ? 1.0 : 0.0;
        const double az = (i % 3) == 2 ? 1.0 : 0.0;
        xml += LinkXml("serial_link_" + std::to_string(i), length, y, z, mass, ax, ay, az);
    }
    xml += "<site name=\"serial_tip\" pos=\"0.10 0 0\"/>";
    for (int i = 0; i < linkCount + 1; ++i)
    {
        xml += CloseBody();
    }
    xml += "</worldbody></mujoco>";
    return xml;
}

dtArray<MjFixture> MujocoFixtures()
{
    const std::string single =
        "<mujoco model=\"single_manipulator_fixed_base\"><option gravity=\"0 0 -9.81\"/>"
        "<worldbody><body name=\"base\" pos=\"0 0 0\">"
        "<inertial pos=\"0 0 0\" mass=\"3\" diaginertia=\"0.05 0.05 0.05\"/>" +
        LinkXml("link1", 0.30, 0.00, 0.05, 1.5, 0.0, 0.0, 1.0) +
        LinkXml("link2", 0.25, 0.00, 0.00, 1.0, 0.0, 1.0, 0.0) +
        LinkXml("link3", 0.20, 0.00, 0.00, 0.8, 1.0, 0.0, 0.0) +
        "<site name=\"tool\" pos=\"0.2 0 0\"/>" + CloseBody() + CloseBody() + CloseBody() + CloseBody() +
        "</worldbody></mujoco>";

    const std::string dual =
        "<mujoco model=\"dual_manipulator_fixed_base\"><option gravity=\"0 0 -9.81\"/>"
        "<worldbody><body name=\"base\" pos=\"0 0 0\">"
        "<inertial pos=\"0 0 0\" mass=\"5\" diaginertia=\"0.08 0.08 0.10\"/>" +
        LinkXml("left_link1", 0.10, 0.25, 0.0, 1.0, 0.0, 0.0, 1.0) +
        LinkXml("left_link2", 0.25, 0.0, 0.0, 0.7, 0.0, 1.0, 0.0) +
        "<site name=\"left_tool\" pos=\"0.2 0 0\"/>" + CloseBody() + CloseBody() +
        LinkXml("right_link1", 0.10, -0.25, 0.0, 1.0, 0.0, 0.0, 1.0) +
        LinkXml("right_link2", 0.25, 0.0, 0.0, 0.7, 0.0, 1.0, 0.0) +
        "<site name=\"right_tool\" pos=\"0.2 0 0\"/>" + CloseBody() + CloseBody() + CloseBody() +
        "</worldbody></mujoco>";

    const std::string multiRobot =
        "<mujoco model=\"multi_robot_fixed_base\"><compiler angle=\"radian\"/><option gravity=\"0 0 -9.81\"/>"
        "<worldbody>"
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
        "</mujoco>";

    const std::string multiFloating =
        "<mujoco model=\"multi_robot_floating_base\"><compiler angle=\"radian\"/><option gravity=\"0 0 -9.81\"/>"
        "<worldbody>"
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
        "</mujoco>";

    std::string quadruped =
        "<mujoco model=\"quadruped_floating_base\"><option gravity=\"0 0 -9.81\"/>"
        "<worldbody><body name=\"trunk\" pos=\"0 0 0\">"
        "<freejoint name=\"floating_base\"/>"
        "<inertial pos=\"0 0 0\" mass=\"10\" diaginertia=\"0.20 0.30 0.40\"/>";
    const char *legNames[4] = {"front_left", "front_right", "rear_left", "rear_right"};
    const double xOffsets[4] = {0.35, 0.35, -0.35, -0.35};
    const double yOffsets[4] = {0.22, -0.22, 0.22, -0.22};
    for (int leg = 0; leg < 4; ++leg)
    {
        const std::string prefix = legNames[leg];
        quadruped += LinkXml(prefix + "_hip", xOffsets[leg], yOffsets[leg], -0.05, 1.0, 1.0, 0.0, 0.0);
        quadruped += LinkXml(prefix + "_thigh", 0.0, 0.0, -0.25, 0.8, 0.0, 1.0, 0.0);
        quadruped += LinkXml(prefix + "_shank", 0.0, 0.0, -0.25, 0.5, 0.0, 1.0, 0.0);
        quadruped += "<site name=\"" + prefix + "_foot\" pos=\"0 0 -0.25\"/>";
        quadruped += CloseBody() + CloseBody() + CloseBody();
    }
    quadruped += CloseBody() + "</worldbody></mujoco>";

    std::string fixed =
        "<mujoco model=\"fixed_body_heavy\"><option gravity=\"0 0 -9.81\"/>"
        "<worldbody><body name=\"base\" pos=\"0 0 0\">"
        "<inertial pos=\"0 0 0\" mass=\"4\" diaginertia=\"0 0 0\"/>";
    for (int i = 0; i < 8; ++i)
    {
        const double x = 0.03 * static_cast<double>(i + 1);
        const double y = 0.02 * static_cast<double>((i % 2) == 0 ? 1 : -1);
        fixed += "<body name=\"fixed_mount_" + std::to_string(i) + "\" pos=\"" +
                 std::to_string(x) + " " + std::to_string(y) + " 0\">"
                 "<inertial pos=\"0 0 0\" mass=\"0.05\" diaginertia=\"0 0 0\"/>";
    }
    fixed += LinkXml("jointed_link", 0.25, 0.0, 0.0, 1.2, 0.0, 0.0, 1.0);
    fixed += "<site name=\"tool\" pos=\"0.15 0 0\"/>" + CloseBody();
    for (int i = 0; i < 9; ++i)
    {
        fixed += CloseBody();
    }
    fixed += "</worldbody></mujoco>";

    const std::string serial10 = SerialChainXml(10);
    const std::string serial30 = SerialChainXml(30);

    return {{"single_manipulator_fixed_base", single},
            {"dual_manipulator_fixed_base", dual},
            {"multi_robot_fixed_base", multiRobot},
            {"multi_robot_floating_base", multiFloating},
            {"quadruped_floating_base", quadruped},
            {"fixed_body_heavy", fixed},
            {"serial_chain_10", serial10},
            {"serial_chain_30", serial30}};
}

pinocchio::Model SingleManipulatorPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);
    pinocchio::JointIndex parent =
        AddRevoluteBody(model, 0, "link1_joint", Vector3(0.30, 0.00, 0.05), Vector3(0.0, 0.0, 1.0),
                        1.5, LinkCom(0.30, 0.00, 0.05), LinkInertia(1.5));
    parent = AddRevoluteBody(model, parent, "link2_joint", Vector3(0.25, 0.00, 0.00), Vector3(0.0, 1.0, 0.0),
                             1.0, LinkCom(0.25, 0.00, 0.00), LinkInertia(1.0));
    AddRevoluteBody(model, parent, "link3_joint", Vector3(0.20, 0.00, 0.00), Vector3(1.0, 0.0, 0.0),
                    0.8, LinkCom(0.20, 0.00, 0.00), LinkInertia(0.8));
    return model;
}

pinocchio::Model DualManipulatorPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);
    pinocchio::JointIndex left =
        AddRevoluteBody(model, 0, "left_link1_joint", Vector3(0.10, 0.25, 0.0), Vector3(0.0, 0.0, 1.0),
                        1.0, LinkCom(0.10, 0.25, 0.0), LinkInertia(1.0));
    AddRevoluteBody(model, left, "left_link2_joint", Vector3(0.25, 0.0, 0.0), Vector3(0.0, 1.0, 0.0),
                    0.7, LinkCom(0.25, 0.0, 0.0), LinkInertia(0.7));
    pinocchio::JointIndex right =
        AddRevoluteBody(model, 0, "right_link1_joint", Vector3(0.10, -0.25, 0.0), Vector3(0.0, 0.0, 1.0),
                        1.0, LinkCom(0.10, -0.25, 0.0), LinkInertia(1.0));
    AddRevoluteBody(model, right, "right_link2_joint", Vector3(0.25, 0.0, 0.0), Vector3(0.0, 1.0, 0.0),
                    0.7, LinkCom(0.25, 0.0, 0.0), LinkInertia(0.7));
    return model;
}

pinocchio::Model MultiRobotPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);

    pinocchio::JointIndex robotA =
        AddRevoluteBody(model,
                        0,
                        "robot_a_shoulder_joint",
                        Vector3(0.50, 0.25, 0.20),
                        Vector3(0.0, 0.0, 1.0),
                        0.8,
                        Vector3(0.05, 0.0, 0.02),
                        Vector3(0.01, 0.012, 0.014),
                        0.03);
    AddRevoluteBody(model,
                    robotA,
                    "robot_a_elbow_joint",
                    Vector3(0.25, 0.0, 0.0),
                    Vector3(0.0, 1.0, 0.0),
                    0.6,
                    Vector3(0.12, 0.0, 0.0),
                    Vector3(0.008, 0.01, 0.012),
                    0.04);

    pinocchio::JointIndex robotB =
        AddRevoluteBody(model,
                        0,
                        "robot_b_hip_joint",
                        Vector3(-0.33, -0.17, 0.27),
                        Vector3(1.0, 0.0, 0.0),
                        0.7,
                        Vector3(0.0, 0.0, 0.08),
                        Vector3(0.009, 0.011, 0.013),
                        0.05);
    AddRevoluteBody(model,
                    robotB,
                    "robot_b_knee_joint",
                    Vector3(0.22, 0.0, 0.0),
                    Vector3(0.0, 1.0, 0.0),
                    0.45,
                    Vector3(0.1, 0.0, 0.0),
                    Vector3(0.006, 0.008, 0.01),
                    0.06);

    return model;
}

pinocchio::Model MultiFloatingPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);

    const pinocchio::JointIndex robotA =
        AddFreeBody(model, "robot_a_free", 3.0, Vector3(0.08, 0.09, 0.10));
    pinocchio::JointIndex robotAJoint =
        AddRevoluteBody(model,
                        robotA,
                        "robot_a_floating_shoulder_joint",
                        Vector3(0.15, 0.0, -0.10),
                        Vector3(0.0, 0.0, 1.0),
                        0.8,
                        Vector3(0.075, 0.0, -0.05),
                        Vector3(0.009, 0.0106, 0.0122),
                        0.03);
    AddRevoluteBody(model,
                    robotAJoint,
                    "robot_a_floating_elbow_joint",
                    Vector3(0.25, 0.0, 0.0),
                    Vector3(0.0, 1.0, 0.0),
                    0.6,
                    Vector3(0.125, 0.0, 0.0),
                    Vector3(0.007, 0.0082, 0.0094),
                    0.04);

    const pinocchio::JointIndex robotB =
        AddFreeBody(model, "robot_b_free", 2.5, Vector3(0.07, 0.08, 0.09));
    pinocchio::JointIndex robotBJoint =
        AddRevoluteBody(model,
                        robotB,
                        "robot_b_floating_hip_joint",
                        Vector3(0.02, 0.03, -0.14),
                        Vector3(1.0, 0.0, 0.0),
                        0.7,
                        Vector3(0.0, 0.0, -0.09),
                        Vector3(0.008, 0.0094, 0.0108),
                        0.05);
    AddRevoluteBody(model,
                    robotBJoint,
                    "robot_b_floating_knee_joint",
                    Vector3(0.22, 0.0, 0.0),
                    Vector3(0.0, 1.0, 0.0),
                    0.45,
                    Vector3(0.11, 0.0, 0.0),
                    Vector3(0.0055, 0.0064, 0.0073),
                    0.06);

    return model;
}

pinocchio::Model QuadrupedPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);
    const pinocchio::JointIndex trunk =
        AddFreeBody(model, "floating_base", 10.0, Vector3(0.20, 0.30, 0.40));
    const char *legNames[4] = {"front_left", "front_right", "rear_left", "rear_right"};
    const double xOffsets[4] = {0.35, 0.35, -0.35, -0.35};
    const double yOffsets[4] = {0.22, -0.22, 0.22, -0.22};
    for (int leg = 0; leg < 4; ++leg)
    {
        const std::string prefix = legNames[leg];
        pinocchio::JointIndex parent =
            AddRevoluteBody(model, trunk, prefix + "_hip_joint",
                            Vector3(xOffsets[leg], yOffsets[leg], -0.05),
                            Vector3(1.0, 0.0, 0.0),
                            1.0,
                            LinkCom(xOffsets[leg], yOffsets[leg], -0.05),
                            LinkInertia(1.0));
        parent = AddRevoluteBody(model, parent, prefix + "_thigh_joint",
                                 Vector3(0.0, 0.0, -0.25),
                                 Vector3(0.0, 1.0, 0.0),
                                 0.8,
                                 LinkCom(0.0, 0.0, -0.25),
                                 LinkInertia(0.8));
        AddRevoluteBody(model, parent, prefix + "_shank_joint",
                        Vector3(0.0, 0.0, -0.25),
                        Vector3(0.0, 1.0, 0.0),
                        0.5,
                        LinkCom(0.0, 0.0, -0.25),
                        LinkInertia(0.5));
    }
    return model;
}

pinocchio::Model FixedBodyHeavyPinocchioModel()
{
    pinocchio::Model model;
    SetGravity(model);
    Eigen::Vector3d fixedOffset = Vector3(0.0, 0.0, 0.0);
    for (int i = 0; i < 8; ++i)
    {
        fixedOffset(0) += 0.03 * static_cast<double>(i + 1);
        fixedOffset(1) += 0.02 * static_cast<double>((i % 2) == 0 ? 1 : -1);
    }
    AddRevoluteBody(model, 0, "jointed_link_joint",
                    fixedOffset + Vector3(0.25, 0.0, 0.0),
                    Vector3(0.0, 0.0, 1.0),
                    1.2,
                    LinkCom(0.25, 0.0, 0.0),
                    LinkInertia(1.2));
    return model;
}


pinocchio::Model SerialChainPinocchioModel(const int linkCount)
{
    pinocchio::Model model;
    SetGravity(model);
    pinocchio::JointIndex parent = 0;
    for (int i = 0; i < linkCount; ++i)
    {
        const double length = 0.12 + 0.002 * static_cast<double>(i % 5);
        const double y = 0.01 * static_cast<double>((i % 2) == 0 ? 1 : -1);
        const double z = 0.02 * static_cast<double>((i % 3) - 1);
        const double mass = 0.35 + 0.01 * static_cast<double>(i % 7);
        const Eigen::Vector3d axis = Vector3((i % 3) == 0 ? 1.0 : 0.0,
                                             (i % 3) == 1 ? 1.0 : 0.0,
                                             (i % 3) == 2 ? 1.0 : 0.0);
        parent = AddRevoluteBody(model,
                                  parent,
                                  "serial_link_" + std::to_string(i) + "_joint",
                                  Vector3(length, y, z),
                                  axis,
                                  mass,
                                  LinkCom(length, y, z),
                                  LinkInertia(mass));
    }
    return model;
}

dtArray<PinFixture> PinocchioFixtures()
{
    return {{"single_manipulator_fixed_base", SingleManipulatorPinocchioModel(), NoFreeJointMap(), 3, 3, 5},
            {"dual_manipulator_fixed_base", DualManipulatorPinocchioModel(), NoFreeJointMap(), 4, 4, 6},
            {"multi_robot_fixed_base", MultiRobotPinocchioModel(), NoFreeJointMap(), 4, 4, 8},
            {"multi_robot_floating_base", MultiFloatingPinocchioModel(), MultiFloatingFreeJointMap(), 18, 16, 8},
            {"quadruped_floating_base", QuadrupedPinocchioModel(), SingleFreeJointMap(0.0, 0.0, 0.0), 19, 18, 14},
            {"fixed_body_heavy", FixedBodyHeavyPinocchioModel(), NoFreeJointMap(), 1, 1, 11},
            {"serial_chain_10", SerialChainPinocchioModel(10), NoFreeJointMap(), 10, 10, 12},
            {"serial_chain_30", SerialChainPinocchioModel(30), NoFreeJointMap(), 30, 30, 32}};
}

void PrintHeader()
{
    std::cout << "engine,fixture,state,nq,nv,nbody,iterations,mean_us,median_us,p95_us,p99_us,"
                 "allocation_mean,allocation_max,quantity_scope\n";
}

void PrintResult(const Result &result)
{
    std::cout << result.engine << ','
              << result.fixture << ','
              << result.state << ','
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
              << result.quantityScope << '\n';
}

void PrintErrorHeader()
{
    std::cout << "comparison_reference,fixture,state,quantity,count,max_error,max_relative_error,rms_error,abs_tolerance,rel_tolerance,status,diagnostic\n";
}

void PrintErrorRows(const dtArray<ErrorRow> &rows)
{
    std::cout << std::scientific << std::setprecision(17);
    for (const ErrorRow &row : rows)
    {
        std::cout << row.reference << ','
                  << row.fixture << ','
                  << row.state << ','
                  << row.quantity << ','
                  << row.summary.count << ','
                  << row.summary.max << ','
                  << row.summary.maxRelative << ','
                  << row.summary.Rms() << ','
                  << row.absTolerance << ','
                  << row.relTolerance << ','
                  << row.status << ','
                  << row.diagnostic << '\n';
    }
    std::cout << std::fixed << std::setprecision(6);
}

void RunNativeReferenceContextBenchmarks(dtArray<std::string> &batchSummaryRows,
                                         const int batchCount)
{
    constexpr const char *quantityScope = "wbc_full_m_c_mdot_centroidal_com";
    for (const MjFixture &fixture : MujocoFixtures())
    {
        for (const std::string &state : States())
        {
            MjModelPtr mjModel = LoadMjModel(fixture.name, fixture.xml);
            MjDataPtr mjData(mj_makeData(mjModel.get()));
            if (mjData == nullptr)
            {
                throw std::runtime_error(fixture.name + ": MuJoCo data allocation failed for native timing state setup");
            }
            SetMujocoState(state, mjModel.get(), mjData.get());
            NativeRun run = ComputeNativeRun(fixture.xml, MjQpos(mjModel.get(), mjData.get()), MjQvel(mjModel.get(), mjData.get()));

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
                const Result result = RunTimed("native",
                                               fixture.name,
                                               state,
                                               run.model.nq,
                                               run.model.nv,
                                               run.model.nbody,
                                               quantityScope,
                                               [&]()
                                               {
                                                   ComputeDynamicsCCRBA(run.model, run.data, Gravity(), DTDYN_CCRBA_TERM_ALL);
                                               });
                meanUs[batch] = result.meanUs;
                medianUs[batch] = result.medianUs;
                p95Us[batch] = result.p95Us;
                p99Us[batch] = result.p99Us;
                allocationMax = std::max(allocationMax, result.allocationMax);
                if (batch == 0)
                {
                    PrintResult(result);
                }
            }
            if (batchCount > 1)
            {
                batchSummaryRows.push_back(
                    dt::Dynamics::benchmark::BuildBatchSummaryRow("benchmark_dtDynamicsReferences",
                                                                  fixture.name,
                                                                  state,
                                                                  std::string("native:") + quantityScope,
                                                                  batchCount,
                                                                  meanUs,
                                                                  medianUs,
                                                                  p95Us,
                                                                  p99Us,
                                                                  allocationMax));
            }
        }
    }
}

void RunMujocoBenchmarks(dtArray<ErrorRow> &errorRows,
                         dtArray<std::string> &batchSummaryRows,
                         const int batchCount)
{
    const char *version = mj_versionString();
    if (version == nullptr || std::string(version).find("3.8.0") == std::string::npos)
    {
        throw std::runtime_error(std::string("benchmark_dtDynamicsReferences requires MuJoCo 3.8.0, found ") +
                                 (version == nullptr ? "null" : version));
    }

    for (const MjFixture &fixture : MujocoFixtures())
    {
        MjModelPtr model = LoadMjModel(fixture.name, fixture.xml);
        MjDataPtr data(mj_makeData(model.get()));
        if (data == nullptr)
        {
            throw std::runtime_error(fixture.name + ": MuJoCo data allocation failed");
        }

        MjScratch scratch;
        InitMujocoScratch(model.get(), scratch);

        for (const std::string &state : States())
        {
            SetMujocoState(state, model.get(), data.get());
            constexpr const char *quantityScope = "forward_fullM_bias_site_jacobians_centroidal";
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
                const ExternalTimingGuard externalTiming(1);
                const Result result = RunTimed("mujoco",
                                               fixture.name,
                                               state,
                                               model->nq,
                                               model->nv,
                                               model->nbody,
                                               quantityScope,
                                               [&]()
                                               {
                                                   ComputeMujocoIngredients(model.get(), data.get(), scratch);
                                               });
                meanUs[batch] = result.meanUs;
                medianUs[batch] = result.medianUs;
                p95Us[batch] = result.p95Us;
                p99Us[batch] = result.p99Us;
                allocationMax = std::max(allocationMax, result.allocationMax);
                if (batch == 0)
                {
                    PrintResult(result);
                }
            }
            if (batchCount > 1)
            {
                batchSummaryRows.push_back(
                    dt::Dynamics::benchmark::BuildBatchSummaryRow("benchmark_dtDynamicsReferences",
                                                                  fixture.name,
                                                                  state,
                                                                  std::string("mujoco:") + quantityScope,
                                                                  batchCount,
                                                                  meanUs,
                                                                  medianUs,
                                                                  p95Us,
                                                                  p99Us,
                                                                  allocationMax));
            }
            AppendMujocoErrors(errorRows, fixture, state, model.get(), data.get());
        }
    }
}

void RunPinocchioBenchmarks(dtArray<ErrorRow> &errorRows,
                            dtArray<std::string> &batchSummaryRows,
                            const int batchCount)
{
    for (const PinFixture &fixture : PinocchioFixtures())
    {
        PinScratch scratch(fixture.model);
        for (const std::string &state : States())
        {
            SetPinocchioState(state, fixture, scratch);
            constexpr const char *quantityScope = "crba_gravity_nle_coriolis_ccrba_centroidal_dot";
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
                const ExternalTimingGuard externalTiming(2);
                const Result result = RunTimed("pinocchio",
                                               fixture.name,
                                               state,
                                               fixture.nativeNq,
                                               fixture.nativeNv,
                                               fixture.nativeNbody,
                                               quantityScope,
                                               [&]()
                                               {
                                                   ComputePinocchioIngredients(fixture, scratch);
                                               });
                meanUs[batch] = result.meanUs;
                medianUs[batch] = result.medianUs;
                p95Us[batch] = result.p95Us;
                p99Us[batch] = result.p99Us;
                allocationMax = std::max(allocationMax, result.allocationMax);
                if (batch == 0)
                {
                    PrintResult(result);
                }
            }
            if (batchCount > 1)
            {
                batchSummaryRows.push_back(
                    dt::Dynamics::benchmark::BuildBatchSummaryRow("benchmark_dtDynamicsReferences",
                                                                  fixture.name,
                                                                  state,
                                                                  std::string("pinocchio:") + quantityScope,
                                                                  batchCount,
                                                                  meanUs,
                                                                  medianUs,
                                                                  p95Us,
                                                                  p99Us,
                                                                  allocationMax));
            }
            AppendPinocchioErrors(errorRows, fixture, state, scratch);
        }
    }
}

} // namespace

int main()
{
    try
    {
        ValidateNonFiniteErrorPolicy();
        PrintHeader();
        if (dt::Dynamics::benchmark::BenchmarkEmitSamples())
        {
            dt::Dynamics::benchmark::PrintSampleHeader(std::cout);
        }
        std::cout << std::fixed << std::setprecision(6);
        dtArray<ErrorRow> errorRows;
        dtArray<std::string> batchSummaryRows;
        const int batchCount = dt::Dynamics::benchmark::BenchmarkBatchCount();
        RunNativeReferenceContextBenchmarks(batchSummaryRows, batchCount);
        RunMujocoBenchmarks(errorRows, batchSummaryRows, batchCount);
        RunPinocchioBenchmarks(errorRows, batchSummaryRows, batchCount);
        PrintErrorHeader();
        PrintErrorRows(errorRows);
        if (batchCount > 1)
        {
            dt::Dynamics::benchmark::PrintBatchSummaryHeader(std::cout);
            for (const std::string &row : batchSummaryRows)
            {
                std::cout << row << '\n';
            }
        }
        dt::Dynamics::benchmark::PrintBenchmarkMetadata(std::cout, "benchmark_dtDynamicsReferences", 20, 200);
        std::cout << "benchmark_metadata,mujoco_native_validation_runs_during_timing,"
                  << g_mujocoNativeValidationRunsDuringTiming.load(std::memory_order_relaxed) << '\n';
        std::cout << "benchmark_metadata,pinocchio_native_validation_runs_during_timing,"
                  << g_pinocchioNativeValidationRunsDuringTiming.load(std::memory_order_relaxed) << '\n';
        std::cout << "benchmark_metadata,nonfinite_error_policy_self_test,pass\n";
        for (const ErrorRow &row : errorRows)
        {
            if (row.status == "fail")
            {
                std::cerr << "reference validation failed; inspect comparison_reference rows\n";
                return 2;
            }
        }
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
