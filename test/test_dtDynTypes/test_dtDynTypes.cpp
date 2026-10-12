#include "dtDynamics/dtArray.h"
#include "dtDynamics/dtDynCompile.h"
#include "dtDynamics/dtDynData.h"
#include "dtDynamics/dtDynDynamics.h"
#include "dtDynamics/dtDynKinematics.h"
#include "dtDynamics/dtDynMath.h"
#include "dtDynamics/dtDynSpec.h"
#include "dtDynamics/dtDynUtils.h"
#include "dtDynamics/dtDynValidation.h"

#ifdef NDEBUG
#error "test_dtDynTypes requires active assertions"
#endif

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

using namespace dt::Dynamics;

static_assert(ToSize(-1) == 0U);
static_assert(ToSize(0) == 0U);
static_assert(ToSize(7) == 7U);
static_assert(utils::RootCentroidalLinearRow(2) == 12);
static_assert(utils::RootCentroidalAngularRow(2) == 15);
static_assert(utils::RootComRow(2) == 6);
static_assert(std::is_same_v<decltype(std::declval<dtDynSpec &>().Bodies()), const dtArray<dtDynBodySpec> &>);
static_assert(std::is_same_v<decltype(std::declval<dtDynSpec &>().Joints()), const dtArray<dtDynJointSpec> &>);
static_assert(std::is_same_v<decltype(std::declval<dtDynSpec &>().Sites()), const dtArray<dtDynSiteSpec> &>);
static_assert(std::is_same_v<decltype(std::declval<dtDynSpec &>().Actuators()), const dtArray<dtDynActuatorSpec> &>);

namespace
{

// Test-only allocation failpoint for transactional dtArray copy coverage.
constexpr std::size_t kNoAllocationFailure = std::numeric_limits<std::size_t>::max();
constexpr std::size_t kMaximumAllocationFailuresToTest = 64;
std::size_t g_allocationsBeforeFailure = kNoAllocationFailure;
std::size_t g_outstandingTestAllocations = 0;
std::size_t g_totalTestAllocations = 0;

bool IsDtMathNear(const dtDynReal actual,
                  const dtDynReal expected,
                  const dtDynReal tolerance = 1.0e-12)
{
    dt::Math::Vector<1, dtDynReal> difference;
    difference(0) = actual - expected;
    return difference.GetNorm() < tolerance;
}

void *AllocateForTest(const std::size_t size)
{
    if (g_allocationsBeforeFailure != kNoAllocationFailure)
    {
        if (g_allocationsBeforeFailure == 0)
        {
            throw std::bad_alloc();
        }
        --g_allocationsBeforeFailure;
    }

    void *const allocation = std::malloc(size == 0 ? 1 : size);
    if (allocation == nullptr)
    {
        throw std::bad_alloc();
    }
    ++g_outstandingTestAllocations;
    ++g_totalTestAllocations;
    return allocation;
}

class ScopedAllocationFailure
{
public:
    explicit ScopedAllocationFailure(const std::size_t allocationsBeforeFailure)
        : m_previous(g_allocationsBeforeFailure)
    {
        g_allocationsBeforeFailure = allocationsBeforeFailure;
    }

    ~ScopedAllocationFailure()
    {
        g_allocationsBeforeFailure = m_previous;
    }

private:
    std::size_t m_previous;
};

struct ThrowingTransfer
{
    inline static int liveCount = 0;
    inline static bool throwOnCopy = false;
    inline static bool throwOnMove = false;

    int value = 0;

    ThrowingTransfer()
    {
        ++liveCount;
    }

    explicit ThrowingTransfer(const int initialValue)
        : value(initialValue)
    {
        ++liveCount;
    }

    ThrowingTransfer(const ThrowingTransfer &other)
        : value(other.value)
    {
        ++liveCount;
    }

    ThrowingTransfer(ThrowingTransfer &&other) noexcept
        : value(other.value)
    {
        ++liveCount;
        other.value = 0;
    }

    ~ThrowingTransfer()
    {
        --liveCount;
    }

    ThrowingTransfer &operator=(const ThrowingTransfer &other)
    {
        if (throwOnCopy)
        {
            throw std::runtime_error("ThrowingTransfer copy assignment");
        }
        value = other.value;
        return *this;
    }

    ThrowingTransfer &operator=(ThrowingTransfer &&other)
    {
        if (throwOnMove)
        {
            throw std::runtime_error("ThrowingTransfer move assignment");
        }
        value = other.value;
        other.value = 0;
        return *this;
    }
};

struct CopyOnlyTrivialTransfer
{
    int value = 0;

    CopyOnlyTrivialTransfer() = default;
    CopyOnlyTrivialTransfer(const CopyOnlyTrivialTransfer &) = default;
    CopyOnlyTrivialTransfer(CopyOnlyTrivialTransfer &&) = delete;
    CopyOnlyTrivialTransfer &operator=(const CopyOnlyTrivialTransfer &) = default;
    CopyOnlyTrivialTransfer &operator=(CopyOnlyTrivialTransfer &&) = delete;
};

static_assert(std::is_trivially_copyable<CopyOnlyTrivialTransfer>::value);
static_assert(std::is_copy_constructible<CopyOnlyTrivialTransfer>::value);
static_assert(!std::is_move_constructible<CopyOnlyTrivialTransfer>::value);
static_assert(std::is_copy_assignable<CopyOnlyTrivialTransfer>::value);
static_assert(!std::is_move_assignable<CopyOnlyTrivialTransfer>::value);

struct MoveOnlyTrivialTransfer
{
    int value = 0;

    MoveOnlyTrivialTransfer() = default;
    MoveOnlyTrivialTransfer(const MoveOnlyTrivialTransfer &) = delete;
    MoveOnlyTrivialTransfer(MoveOnlyTrivialTransfer &&) = default;
    MoveOnlyTrivialTransfer &operator=(const MoveOnlyTrivialTransfer &) = delete;
    MoveOnlyTrivialTransfer &operator=(MoveOnlyTrivialTransfer &&) noexcept = default;
};

static_assert(std::is_trivially_copyable<MoveOnlyTrivialTransfer>::value);
static_assert(!std::is_copy_assignable<MoveOnlyTrivialTransfer>::value);
static_assert(std::is_nothrow_move_assignable<MoveOnlyTrivialTransfer>::value);

struct CopyConstructibleMoveAssignableOnly
{
    int value = 0;

    CopyConstructibleMoveAssignableOnly() = default;
    CopyConstructibleMoveAssignableOnly(const CopyConstructibleMoveAssignableOnly &) = default;
    CopyConstructibleMoveAssignableOnly(CopyConstructibleMoveAssignableOnly &&) = default;
    CopyConstructibleMoveAssignableOnly &operator=(const CopyConstructibleMoveAssignableOnly &) = delete;
    CopyConstructibleMoveAssignableOnly &operator=(CopyConstructibleMoveAssignableOnly &&other) noexcept
    {
        value = other.value;
        other.value = 0;
        return *this;
    }
};

static_assert(!std::is_copy_assignable<CopyConstructibleMoveAssignableOnly>::value);
static_assert(std::is_nothrow_move_assignable<CopyConstructibleMoveAssignableOnly>::value);

struct OverloadedCopyAssignment
{
    enum class Selection
    {
        None,
        Mutable,
        Const,
    };

    int value = 0;
    Selection selection = Selection::None;

    OverloadedCopyAssignment() = default;
    OverloadedCopyAssignment(const OverloadedCopyAssignment &other)
        : value(other.value)
    {
    }

    OverloadedCopyAssignment &operator=(OverloadedCopyAssignment &other)
    {
        value = other.value;
        selection = Selection::Mutable;
        return *this;
    }

    OverloadedCopyAssignment &operator=(const OverloadedCopyAssignment &other)
    {
        value = other.value;
        selection = Selection::Const;
        return *this;
    }
};

static_assert(std::is_copy_assignable<OverloadedCopyAssignment>::value);

} // namespace

void *operator new(const std::size_t size)
{
    return AllocateForTest(size);
}

void *operator new[](const std::size_t size)
{
    return AllocateForTest(size);
}

void operator delete(void *const allocation) noexcept
{
    if (allocation != nullptr)
    {
        --g_outstandingTestAllocations;
    }
    std::free(allocation);
}

void operator delete[](void *const allocation) noexcept
{
    if (allocation != nullptr)
    {
        --g_outstandingTestAllocations;
    }
    std::free(allocation);
}

void operator delete(void *const allocation, const std::size_t) noexcept
{
    if (allocation != nullptr)
    {
        --g_outstandingTestAllocations;
    }
    std::free(allocation);
}

void operator delete[](void *const allocation, const std::size_t) noexcept
{
    if (allocation != nullptr)
    {
        --g_outstandingTestAllocations;
    }
    std::free(allocation);
}

namespace
{

std::filesystem::path UniqueTempPath(const std::string &stem)
{
    static unsigned long long sequence = 0;
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           (stem + "_" + std::to_string(ticks) + "_" + std::to_string(sequence++));
}

void TestCheckedCompileAcceptsValidSpec()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);

    dtDynValidationError error;
    assert(ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::None);

    const dtDynModel trusted = CompileSpec(spec);
    const dtDynModel checked = CompileSpecChecked(spec);
    assert(checked.nbody == trusted.nbody);
    assert(checked.njnt == trusted.njnt);
    assert(checked.nq == trusted.nq);
    assert(checked.nv == trusted.nv);
}

void TestCheckedCompileRejectsInvalidParent()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.parent = 99; });

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::InvalidParent);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == body);
    assert(error.message.find("body[1]") != std::string::npos);

    bool rejected = false;
    try
    {
        (void)CompileSpecChecked(spec);
    }
    catch (const std::runtime_error &exception)
    {
        rejected = std::string(exception.what()).find("body[1]") != std::string::npos;
    }
    assert(rejected);
}

void TestSpecValidationRejectsDetachedJoint()
{
    dtDynSpec spec;
    const dtDynIndex joint = spec.AddJoint(99, "detached", dtDynJointType::Hinge);

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::DetachedObject);
    assert(error.object == dtDynObjectType::Joint);
    assert(error.index == joint);
    assert(error.message.find("joint[0]") != std::string::npos);
}

void TestSpecValidationRejectsSiblingCycle()
{
    dtDynSpec spec;
    const dtDynIndex first = spec.AddBody("first", 0);
    const dtDynIndex second = spec.AddBody("second", 0);
    spec.EditBody(second, [&](dtDynBodySpec &draft)
                  { draft.nextSibling = second; });

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::TopologyCycle);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == second);
    (void)first;
}

void TestSpecValidationRejectsDetachedSite()
{
    dtDynSpec spec;
    const dtDynIndex site = spec.AddSite(99, "detached_site");

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::DetachedObject);
    assert(error.object == dtDynObjectType::Site);
    assert(error.index == site);
}

void TestSpecValidationRejectsInvalidActuatorReference()
{
    dtDynSpec spec;
    const dtDynIndex actuator = spec.AddActuator(99, "invalid_motor");

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);
    assert(error.object == dtDynObjectType::Actuator);
    assert(error.index == actuator);
}

void TestSpecValidationRejectsInvalidBodyJointReference()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.joints[0] = 99; });

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == body);
}

void TestSpecValidationRejectsDuplicateNames()
{
    dtDynSpec spec;
    spec.AddBody("duplicate", 0);
    const dtDynIndex duplicate = spec.AddBody("duplicate", 0);

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::DuplicateName);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == duplicate);
}

void TestSpecValidationRejectsDuplicateEmptyNames()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "", dtDynJointType::Hinge);
    const dtDynIndex duplicate = spec.AddJoint(body, "", dtDynJointType::Hinge);

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::DuplicateName);
    assert(error.object == dtDynObjectType::Joint);
    assert(error.index == duplicate);
}

void TestSpecValidationRejectsNonFiniteValues()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = std::numeric_limits<dtDynReal>::infinity(); });

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::NonFiniteValue);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == body);
}

void TestModelValidationRejectsDimensionOverflow()
{
    dtDynModel model;
    model.nq = static_cast<dtDynIndex>(std::numeric_limits<uint16_t>::max()) + 1;

    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == dtDynValidationCode::DimensionOverflow);
    assert(error.message.find("nq") != std::string::npos);
}

void TestSpecValidationRejectsDimensionOverflowBeforeCompile()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    for (dtDynIndex joint = 0; joint < 9400; ++joint)
    {
        spec.AddJoint(body, "", dtDynJointType::Free);
    }

    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == dtDynValidationCode::DimensionOverflow);
    assert(error.object == dtDynObjectType::Joint);
}

void TestModelValidationRejectsArrayShapeMismatch()
{
    dtDynSpec spec;
    spec.AddBody("base", 0);
    dtDynModel model = CompileSpec(spec);
    model.body_parentid.clear();

    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);
    assert(error.message.find("body_parentid") != std::string::npos);
}

void TestModelValidationRejectsMissingWorld()
{
    dtDynModel model;
    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == dtDynValidationCode::InvalidWorld);
}

void TestModelValidationRejectsNonFiniteValues()
{
    dtDynSpec spec;
    spec.AddBody("base", 0);
    dtDynModel model = CompileSpec(spec);
    model.body_mass(1) = std::numeric_limits<dtDynReal>::quiet_NaN();

    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == dtDynValidationCode::NonFiniteValue);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == 1);
}

void TestModelValidationRejectsInvalidDofReferences()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    const dtDynModel valid = CompileSpec(spec);
    dtDynValidationError error;

    dtDynModel badBody = valid;
    badBody.dof_bodyid[0] = 99;
    assert(!ValidateModel(badBody, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);

    dtDynModel badJoint = valid;
    badJoint.dof_jntid[0] = 99;
    assert(!ValidateModel(badJoint, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);

    dtDynModel parentCycle = valid;
    parentCycle.dof_parentid[0] = 0;
    assert(!ValidateModel(parentCycle, &error));
    assert(error.code == dtDynValidationCode::TopologyCycle);
}

void TestModelValidationRejectsInvalidAddressSegments()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    const dtDynModel valid = CompileSpec(spec);
    dtDynValidationError error;

    dtDynModel badBodyJoint = valid;
    badBodyJoint.body_jntadr[body] = 99;
    assert(!ValidateModel(badBodyJoint, &error));

    dtDynModel badBodyDof = valid;
    badBodyDof.body_dofnum[body] = 2;
    assert(!ValidateModel(badBodyDof, &error));

    dtDynModel badQpos = valid;
    badQpos.jnt_qposadr[0] = 1;
    assert(!ValidateModel(badQpos, &error));

    dtDynModel badDof = valid;
    badDof.jnt_dofadr[0] = 1;
    assert(!ValidateModel(badDof, &error));

    dtDynModel badType = valid;
    badType.jnt_type[0] = 99;
    assert(!ValidateModel(badType, &error));
}

void TestModelValidationRejectsInvalidJointSpecProvenance()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "joint_a", dtDynJointType::Hinge);
    spec.AddJoint(body, "joint_b", dtDynJointType::Hinge);
    const dtDynModel valid = CompileSpecChecked(spec);

    dtDynModel duplicateSpecId = valid;
    duplicateSpecId.jnt_specid[1] = duplicateSpecId.jnt_specid[0];

    dtDynValidationError error;
    assert(!ValidateModel(duplicateSpecId, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);
    assert(error.object == dtDynObjectType::Joint);
}

void TestModelValidationRejectsInvalidRootMetadata()
{
    dtDynSpec spec;
    spec.AddBody("base", 0);
    const dtDynModel valid = CompileSpec(spec);
    dtDynValidationError error;

    dtDynModel badRootBody = valid;
    badRootBody.root_bodyid[0] = 99;
    assert(!ValidateModel(badRootBody, &error));

    dtDynModel badBodyRoot = valid;
    badBodyRoot.body_rootid[1] = 99;
    assert(!ValidateModel(badBodyRoot, &error));

    dtDynModel badRootSlot = valid;
    badRootSlot.body_rootidx[1] = 99;
    assert(!ValidateModel(badRootSlot, &error));
}

void TestModelValidationRejectsDuplicateNames()
{
    dtDynSpec spec;
    spec.AddBody("base", 0);
    dtDynModel model = CompileSpec(spec);
    model.body_names[1] = model.body_names[0];

    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == dtDynValidationCode::DuplicateName);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == 1);
}

void TestModelValidationRejectsOverlappingJointSpans()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "joint_a", dtDynJointType::Hinge);
    spec.AddJoint(body, "joint_b", dtDynJointType::Hinge);
    const dtDynModel valid = CompileSpec(spec);
    dtDynValidationError error;

    dtDynModel qposOverlap = valid;
    qposOverlap.jnt_qposadr[1] = 0;
    assert(!ValidateModel(qposOverlap, &error));

    dtDynModel dofOverlap = valid;
    dofOverlap.jnt_dofadr[1] = 0;
    assert(!ValidateModel(dofOverlap, &error));
}

void TestModelValidationRejectsOrphanBodySegments()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "joint_a", dtDynJointType::Hinge);
    spec.AddJoint(body, "joint_b", dtDynJointType::Hinge);
    const dtDynModel valid = CompileSpec(spec);
    dtDynValidationError error;

    dtDynModel orphanJoint = valid;
    orphanJoint.body_jntnum[body] = 1;
    assert(!ValidateModel(orphanJoint, &error));

    dtDynModel orphanDof = valid;
    orphanDof.body_dofnum[body] = 1;
    assert(!ValidateModel(orphanDof, &error));
}

void TestModelValidationRejectsDofOutsideDeclaredJointSpan()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.AddJoint(body, "free", dtDynJointType::Free);
    for (dtDynIndex index = 0; index < 10; ++index)
    {
        spec.AddJoint(body, "hinge_" + std::to_string(index), dtDynJointType::Hinge);
    }
    dtDynModel model = CompileSpec(spec);
    assert(model.nv == 16);
    model.jnt_dofadr[0] = 10;

    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);
}

void TestModelValidationRejectsInvalidWeldId()
{
    dtDynSpec spec;
    const dtDynIndex jointed = spec.AddBody("jointed", 0);
    spec.AddJoint(jointed, "hinge", dtDynJointType::Hinge);
    const dtDynIndex fixed = spec.AddBody("fixed", jointed);
    dtDynModel model = CompileSpec(spec);
    assert(model.body_weldid[fixed] == jointed);
    model.body_weldid[fixed] = 99;

    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == fixed);
}

void TestModelValidationRejectsIncorrectDofParentTopology()
{
    dtDynSpec multiRoot;
    const dtDynIndex rootA = multiRoot.AddBody("root_a", 0);
    multiRoot.AddJoint(rootA, "joint_a", dtDynJointType::Hinge);
    const dtDynIndex rootB = multiRoot.AddBody("root_b", 0);
    multiRoot.AddJoint(rootB, "joint_b", dtDynJointType::Hinge);
    dtDynModel crossRoot = CompileSpec(multiRoot);
    crossRoot.dof_parentid[1] = 0;
    dtDynValidationError error;
    assert(!ValidateModel(crossRoot, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);

    dtDynSpec chain;
    const dtDynIndex parent = chain.AddBody("parent", 0);
    chain.AddJoint(parent, "parent_joint", dtDynJointType::Hinge);
    const dtDynIndex child = chain.AddBody("child", parent);
    chain.AddJoint(child, "child_joint", dtDynJointType::Hinge);
    dtDynModel severed = CompileSpec(chain);
    assert(severed.dof_parentid[1] == 0);
    severed.dof_parentid[1] = DTDYN_INVALID_INDEX;
    assert(!ValidateModel(severed, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);
}

void TestModelValidationRejectsInconsistentDerivedMasses()
{
    dtDynSpec spec;
    const dtDynIndex root = spec.AddBody("root", 0);
    spec.EditBody(root, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    const dtDynIndex child = spec.AddBody("child", root);
    spec.EditBody(child, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 3.0; });
    const dtDynModel valid = CompileSpec(spec);
    dtDynValidationError error;

    dtDynModel badSubtree = valid;
    badSubtree.body_subtreemass(root) += 1.0;
    assert(!ValidateModel(badSubtree, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);

    dtDynModel badRoot = valid;
    badRoot.root_mass(0) += 1.0;
    assert(!ValidateModel(badRoot, &error));
    assert(error.code == dtDynValidationCode::InvalidReference);
}

void TestSpecValidationRejectsInvalidPhysicalValues()
{
    dtDynValidationError error;

    dtDynSpec negativeMass;
    const dtDynIndex massBody = negativeMass.AddBody("mass_body", 0);
    negativeMass.EditBody(massBody, [&](dtDynBodySpec &draft)
                          { draft.inertial.mass = -1.0; });
    assert(!ValidateSpec(negativeMass, &error));

    dtDynSpec invalidInertia;
    const dtDynIndex inertiaBody = invalidInertia.AddBody("inertia_body", 0);
    invalidInertia.EditBody(inertiaBody, [&](dtDynBodySpec &draft)
                            { draft.inertial.mass = 1.0; });
    invalidInertia.EditBody(inertiaBody, [&](dtDynBodySpec &draft)
                            { draft.inertial.inertia(0) = 3.0; });
    invalidInertia.EditBody(inertiaBody, [&](dtDynBodySpec &draft)
                            { draft.inertial.inertia(1) = 1.0; });
    invalidInertia.EditBody(inertiaBody, [&](dtDynBodySpec &draft)
                            { draft.inertial.inertia(2) = 1.0; });
    assert(!ValidateSpec(invalidInertia, &error));

    dtDynSpec invalidAxis;
    const dtDynIndex axisBody = invalidAxis.AddBody("axis_body", 0);
    const dtDynIndex axisJoint =
        invalidAxis.AddJoint(axisBody, "hinge", dtDynJointType::Hinge);
    invalidAxis.EditJoint(axisJoint, [&](dtDynJointSpec &draft)
                          { draft.axis(2) = 2.0; });
    assert(!ValidateSpec(invalidAxis, &error));

    dtDynSpec negativeArmature;
    const dtDynIndex armatureBody = negativeArmature.AddBody("armature_body", 0);
    const dtDynIndex armatureJoint =
        negativeArmature.AddJoint(armatureBody, "hinge", dtDynJointType::Hinge);
    negativeArmature.EditJoint(armatureJoint, [&](dtDynJointSpec &draft)
                               { draft.armature = -1.0; });
    assert(!ValidateSpec(negativeArmature, &error));

    dtDynSpec roundedAxis;
    const dtDynIndex roundedBody = roundedAxis.AddBody("rounded_axis_body", 0);
    const dtDynIndex roundedJoint =
        roundedAxis.AddJoint(roundedBody, "rounded_axis", dtDynJointType::Hinge);
    roundedAxis.EditJoint(roundedJoint, [&](dtDynJointSpec &draft)
                          { draft.axis(0) = 0.70710678; });
    roundedAxis.EditJoint(roundedJoint, [&](dtDynJointSpec &draft)
                          { draft.axis(1) = 0.70710678; });
    roundedAxis.EditJoint(roundedJoint, [&](dtDynJointSpec &draft)
                          { draft.axis(2) = 0.0; });
    assert(ValidateSpec(roundedAxis, &error));
}

void AssertSpecValidationFailure(const dtDynSpec &spec,
                                 const dtDynValidationCode code,
                                 const dtDynObjectType object,
                                 const dtDynIndex index,
                                 const std::string &message)
{
    dtDynValidationError error;
    assert(!ValidateSpec(spec, &error));
    assert(error.code == code);
    assert(error.object == object);
    assert(error.index == index);
    assert(error.message == message);
}

void TestSpecValidationPreservesOrderedPhysicalAndNameFailures()
{
    const dtDynReal nan = std::numeric_limits<dtDynReal>::quiet_NaN();

    dtDynSpec nonFiniteBody;
    const dtDynIndex nonFiniteBodyId = nonFiniteBody.AddBody("nonfinite_body", 0);
    nonFiniteBody.EditBody(nonFiniteBodyId, [&](dtDynBodySpec &draft)
                           { draft.pos(0) = nan; });
    AssertSpecValidationFailure(nonFiniteBody, dtDynValidationCode::NonFiniteValue,
                                dtDynObjectType::Body, nonFiniteBodyId,
                                "body[1] contains a non-finite value");
    bool checkedCompileRejected = false;
    try
    {
        (void)CompileSpecChecked(nonFiniteBody);
    }
    catch (const std::runtime_error &exception)
    {
        checkedCompileRejected =
            std::string(exception.what()) == "CompileSpecChecked: body[1] contains a non-finite value";
    }
    assert(checkedCompileRejected);

    dtDynSpec negativeMass;
    const dtDynIndex massBody = negativeMass.AddBody("negative_mass_body", 0);
    negativeMass.EditBody(massBody, [&](dtDynBodySpec &draft)
                          { draft.inertial.mass = -1.0; });
    AssertSpecValidationFailure(negativeMass, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Body, massBody,
                                "body[1] has a negative mass or physically invalid principal inertia");

    dtDynSpec invalidPrincipalInertia;
    const dtDynIndex inertiaBody = invalidPrincipalInertia.AddBody("invalid_inertia_body", 0);
    invalidPrincipalInertia.EditBody(inertiaBody, [&](dtDynBodySpec &draft)
                                     {
                                        draft.inertial.mass = 1.0;
                                        draft.inertial.inertia(0) = 3.0;
                                        draft.inertial.inertia(1) = 1.0;
                                        draft.inertial.inertia(2) = 1.0; });
    AssertSpecValidationFailure(invalidPrincipalInertia, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Body, inertiaBody,
                                "body[1] has a negative mass or physically invalid principal inertia");

    dtDynSpec boundaryPrincipalInertia;
    const dtDynIndex boundaryBody = boundaryPrincipalInertia.AddBody("boundary_inertia_body", 0);
    boundaryPrincipalInertia.EditBody(boundaryBody, [&](dtDynBodySpec &draft)
                                      {
                                         draft.inertial.mass = 1.0;
                                         draft.inertial.inertia(0) = 2.0;
                                         draft.inertial.inertia(1) = 1.0;
                                         draft.inertial.inertia(2) = 1.0; });
    dtDynValidationError boundaryError;
    assert(ValidateSpec(boundaryPrincipalInertia, &boundaryError));
    assert(boundaryError.code == dtDynValidationCode::None);

    dtDynSpec invalidQuaternion;
    const dtDynIndex quaternionBody = invalidQuaternion.AddBody("invalid_quaternion_body", 0);
    invalidQuaternion.EditBody(quaternionBody, [&](dtDynBodySpec &draft)
                               { draft.quat.SetZero(); });
    AssertSpecValidationFailure(invalidQuaternion, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Body, quaternionBody,
                                "body[1] has a non-unit body or inertial quaternion");

    dtDynSpec nonFiniteJoint;
    const dtDynIndex jointBody = nonFiniteJoint.AddBody("nonfinite_joint_body", 0);
    const dtDynIndex nonFiniteJointId =
        nonFiniteJoint.AddJoint(jointBody, "nonfinite_joint", dtDynJointType::Hinge);
    nonFiniteJoint.EditJoint(nonFiniteJointId, [&](dtDynJointSpec &draft)
                             { draft.rangeMin = nan; });
    AssertSpecValidationFailure(nonFiniteJoint, dtDynValidationCode::NonFiniteValue,
                                dtDynObjectType::Joint, nonFiniteJointId,
                                "joint[0] contains a non-finite value");

    dtDynSpec invalidSlideAxis;
    const dtDynIndex slideBody = invalidSlideAxis.AddBody("slide_body", 0);
    const dtDynIndex slideJoint =
        invalidSlideAxis.AddJoint(slideBody, "slide_joint", dtDynJointType::Slide);
    invalidSlideAxis.EditJoint(slideJoint, [&](dtDynJointSpec &draft)
                               {
                                   draft.axis.SetZero();
                                   draft.armature = -1.0; });
    AssertSpecValidationFailure(invalidSlideAxis, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Joint, slideJoint,
                                "joint axis must be unit length for slide/hinge joints");

    dtDynSpec negativeSlideArmature;
    const dtDynIndex negativeSlideBody = negativeSlideArmature.AddBody("negative_slide_body", 0);
    const dtDynIndex negativeSlideJoint =
        negativeSlideArmature.AddJoint(negativeSlideBody, "negative_slide", dtDynJointType::Slide);
    negativeSlideArmature.EditJoint(negativeSlideJoint, [&](dtDynJointSpec &draft)
                                    { draft.armature = -1.0; });
    AssertSpecValidationFailure(negativeSlideArmature, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Joint, negativeSlideJoint,
                                "joint armature must be nonnegative");

    dtDynSpec invalidHingeAxis;
    const dtDynIndex hingeBody = invalidHingeAxis.AddBody("hinge_body", 0);
    const dtDynIndex hingeJoint =
        invalidHingeAxis.AddJoint(hingeBody, "hinge_joint", dtDynJointType::Hinge);
    invalidHingeAxis.EditJoint(hingeJoint, [&](dtDynJointSpec &draft)
                               { draft.axis.SetZero(); });
    AssertSpecValidationFailure(invalidHingeAxis, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Joint, hingeJoint,
                                "joint axis must be unit length for slide/hinge joints");

    dtDynSpec negativeHingeArmature;
    const dtDynIndex negativeHingeBody = negativeHingeArmature.AddBody("negative_hinge_body", 0);
    const dtDynIndex negativeHingeJoint =
        negativeHingeArmature.AddJoint(negativeHingeBody, "negative_hinge", dtDynJointType::Hinge);
    negativeHingeArmature.EditJoint(negativeHingeJoint, [&](dtDynJointSpec &draft)
                                    { draft.armature = -1.0; });
    AssertSpecValidationFailure(negativeHingeArmature, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Joint, negativeHingeJoint,
                                "joint armature must be nonnegative");

    dtDynSpec validFreeAxis;
    const dtDynIndex freeBody = validFreeAxis.AddBody("free_body", 0);
    const dtDynIndex freeJoint = validFreeAxis.AddJoint(freeBody, "free_joint", dtDynJointType::Free);
    validFreeAxis.EditJoint(freeJoint, [&](dtDynJointSpec &draft)
                            { draft.axis.SetZero(); });
    const dtDynModel freeModel = CompileSpecChecked(validFreeAxis);
    assert(freeModel.njnt == 1);

    dtDynSpec negativeFreeArmature = validFreeAxis;
    negativeFreeArmature.EditJoint(freeJoint, [&](dtDynJointSpec &draft)
                                   { draft.armature = -1.0; });
    AssertSpecValidationFailure(negativeFreeArmature, dtDynValidationCode::InvalidReference,
                                dtDynObjectType::Joint, freeJoint,
                                "joint armature must be nonnegative");

    dtDynSpec nonFiniteSite;
    const dtDynIndex siteBody = nonFiniteSite.AddBody("site_body", 0);
    const dtDynIndex site = nonFiniteSite.AddSite(siteBody, "nonfinite_site");
    nonFiniteSite.EditSite(site, [&](dtDynSiteSpec &draft)
                           { draft.quat(0) = nan; });
    AssertSpecValidationFailure(nonFiniteSite, dtDynValidationCode::NonFiniteValue,
                                dtDynObjectType::Site, site,
                                "site[0] contains a non-finite value");

    dtDynSpec named;
    const dtDynIndex firstBody = named.AddBody("body_one", 0);
    const dtDynIndex secondBody = named.AddBody("body_two", 0);
    const dtDynIndex firstJoint = named.AddJoint(firstBody, "joint_one", dtDynJointType::Hinge);
    const dtDynIndex secondJoint = named.AddJoint(secondBody, "joint_two", dtDynJointType::Hinge);
    const dtDynIndex firstSite = named.AddSite(firstBody, "site_one");
    const dtDynIndex secondSite = named.AddSite(secondBody, "site_two");
    const dtDynIndex firstActuator = named.AddActuator(firstJoint, "actuator_one");
    const dtDynIndex secondActuator = named.AddActuator(secondJoint, "actuator_two");

    dtDynSpec duplicateBody = named;
    duplicateBody.EditBody(secondBody, [&](dtDynBodySpec &draft)
                           { draft.name = "body_one"; });
    AssertSpecValidationFailure(duplicateBody, dtDynValidationCode::DuplicateName,
                                dtDynObjectType::Body, secondBody,
                                "body[2] duplicates name 'body_one'");

    dtDynSpec duplicateJoint = named;
    duplicateJoint.EditJoint(secondJoint, [&](dtDynJointSpec &draft)
                             { draft.name = "joint_one"; });
    AssertSpecValidationFailure(duplicateJoint, dtDynValidationCode::DuplicateName,
                                dtDynObjectType::Joint, secondJoint,
                                "joint[1] duplicates name 'joint_one'");

    dtDynSpec duplicateSite = named;
    duplicateSite.EditSite(secondSite, [&](dtDynSiteSpec &draft)
                           { draft.name = "site_one"; });
    AssertSpecValidationFailure(duplicateSite, dtDynValidationCode::DuplicateName,
                                dtDynObjectType::Site, secondSite,
                                "site[1] duplicates name 'site_one'");

    dtDynSpec duplicateActuator = named;
    duplicateActuator.EditActuator(secondActuator, [&](dtDynActuatorSpec &draft)
                                   { draft.name = "actuator_one"; });
    AssertSpecValidationFailure(duplicateActuator, dtDynValidationCode::DuplicateName,
                                dtDynObjectType::Actuator, secondActuator,
                                "actuator[1] duplicates name 'actuator_one'");

    dtDynSpec duplicateBodyAndActuator = named;
    duplicateBodyAndActuator.EditBody(secondBody, [&](dtDynBodySpec &draft)
                                      { draft.name = "body_one"; });
    duplicateBodyAndActuator.EditActuator(secondActuator, [&](dtDynActuatorSpec &draft)
                                          { draft.name = "actuator_one"; });
    AssertSpecValidationFailure(duplicateBodyAndActuator, dtDynValidationCode::DuplicateName,
                                dtDynObjectType::Body, secondBody,
                                "body[2] duplicates name 'body_one'");
}

void TestModelValidationRejectsInvalidPhysicalValues()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 1.0; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 1.0; });
    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 1.0; });
    const dtDynIndex joint = spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    spec.EditJoint(joint, [&](dtDynJointSpec &draft)
                   { draft.armature = 1.0; });
    const dtDynIndex site = spec.AddSite(body, "site");
    const dtDynModel valid = CompileSpec(spec);
    dtDynValidationError error;

    dtDynModel zeroAxis = valid;
    zeroAxis.jnt_axis[0].SetZero();
    assert(!ValidateModel(zeroAxis, &error));

    dtDynModel negativeMass = valid;
    negativeMass.body_mass(body) = -1.0;
    negativeMass.body_subtreemass(body) = -1.0;
    negativeMass.body_subtreemass(0) = -1.0;
    negativeMass.root_mass(0) = -1.0;
    assert(!ValidateModel(negativeMass, &error));

    dtDynModel negativeInertia = valid;
    negativeInertia.body_inertia[body](0) = -1.0;
    assert(!ValidateModel(negativeInertia, &error));

    dtDynModel negativeArmature = valid;
    negativeArmature.dof_armature(0) = -1.0;
    assert(!ValidateModel(negativeArmature, &error));

    dtDynModel invalidBodyFrame = valid;
    invalidBodyFrame.body_mat[body].SetZero();
    assert(!ValidateModel(invalidBodyFrame, &error));

    dtDynModel invalidInertialFrame = valid;
    invalidInertialFrame.body_imat[body].SetZero();
    assert(!ValidateModel(invalidInertialFrame, &error));

    dtDynModel invalidSiteRotation = valid;
    invalidSiteRotation.site_mat[site].SetZero();
    assert(!ValidateModel(invalidSiteRotation, &error));
}

void TestModelValidationRejectsInvalidFramePairs()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("frame_body", 0);
    const dtDynIndex site = spec.AddSite(body, "frame_site");
    const dtDynModel valid = CompileSpec(spec);
    dtDynValidationError error;

    dtDynModel scaledBodyQuaternion = valid;
    scaledBodyQuaternion.body_quat[body](0) *= 2.0;
    assert(!ValidateModel(scaledBodyQuaternion, &error));

    dtDynModel mismatchedBodyFrame = valid;
    mismatchedBodyFrame.body_quat[body](0) = 0.0;
    mismatchedBodyFrame.body_quat[body](3) = 1.0;
    assert(!ValidateModel(mismatchedBodyFrame, &error));

    dtDynModel scaledInertialQuaternion = valid;
    scaledInertialQuaternion.body_iquat[body](0) *= 2.0;
    assert(!ValidateModel(scaledInertialQuaternion, &error));

    dtDynModel mismatchedInertialFrame = valid;
    mismatchedInertialFrame.body_iquat[body](0) = 0.0;
    mismatchedInertialFrame.body_iquat[body](3) = 1.0;
    assert(!ValidateModel(mismatchedInertialFrame, &error));

    dtDynModel scaledSiteQuaternion = valid;
    scaledSiteQuaternion.site_quat[site](0) *= 2.0;
    assert(!ValidateModel(scaledSiteQuaternion, &error));

    dtDynModel mismatchedSiteFrame = valid;
    mismatchedSiteFrame.site_quat[site](0) = 0.0;
    mismatchedSiteFrame.site_quat[site](3) = 1.0;
    assert(!ValidateModel(mismatchedSiteFrame, &error));
}

void AssertModelValidationFailure(const dtDynModel &model,
                                  const dtDynValidationCode code,
                                  const dtDynObjectType object,
                                  const dtDynIndex index,
                                  const std::string &message)
{
    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == code);
    assert(error.object == object);
    assert(error.index == index);
    assert(error.message == message);
}

void TestModelValidationPreservesUniqueNameValidationOrder()
{
    dtDynSpec spec;
    const dtDynIndex firstBody = spec.AddBody("body_one", 0);
    const dtDynIndex secondBody = spec.AddBody("body_two", 0);
    const dtDynIndex firstJoint = spec.AddJoint(firstBody, "joint_one", dtDynJointType::Hinge);
    const dtDynIndex secondJoint = spec.AddJoint(secondBody, "joint_two", dtDynJointType::Hinge);
    const dtDynIndex firstSite = spec.AddSite(firstBody, "site_one");
    const dtDynIndex secondSite = spec.AddSite(secondBody, "site_two");
    const dtDynIndex firstActuator = spec.AddActuator(firstJoint, "actuator_one");
    const dtDynIndex secondActuator = spec.AddActuator(secondJoint, "actuator_two");
    const dtDynModel valid = CompileSpecChecked(spec);

    dtDynModel duplicateBody = valid;
    duplicateBody.body_names[static_cast<std::size_t>(secondBody)] =
        duplicateBody.body_names[static_cast<std::size_t>(firstBody)];
    AssertModelValidationFailure(duplicateBody, dtDynValidationCode::DuplicateName,
                                 dtDynObjectType::Body, secondBody,
                                 "body[2] duplicates name 'body_one'");

    dtDynModel duplicateJoint = valid;
    duplicateJoint.joint_names[static_cast<std::size_t>(secondJoint)] =
        duplicateJoint.joint_names[static_cast<std::size_t>(firstJoint)];
    AssertModelValidationFailure(duplicateJoint, dtDynValidationCode::DuplicateName,
                                 dtDynObjectType::Joint, secondJoint,
                                 "joint[1] duplicates name 'joint_one'");

    dtDynModel duplicateSite = valid;
    duplicateSite.site_names[static_cast<std::size_t>(secondSite)] =
        duplicateSite.site_names[static_cast<std::size_t>(firstSite)];
    AssertModelValidationFailure(duplicateSite, dtDynValidationCode::DuplicateName,
                                 dtDynObjectType::Site, secondSite,
                                 "site[1] duplicates name 'site_one'");

    dtDynModel duplicateActuator = valid;
    duplicateActuator.actuator_names[static_cast<std::size_t>(secondActuator)] =
        duplicateActuator.actuator_names[static_cast<std::size_t>(firstActuator)];
    AssertModelValidationFailure(duplicateActuator, dtDynValidationCode::DuplicateName,
                                 dtDynObjectType::Actuator, secondActuator,
                                 "actuator[1] duplicates name 'actuator_one'");

    dtDynModel duplicateBodyAndActuator = duplicateBody;
    duplicateBodyAndActuator.actuator_names[static_cast<std::size_t>(secondActuator)] =
        duplicateBodyAndActuator.actuator_names[static_cast<std::size_t>(firstActuator)];
    AssertModelValidationFailure(duplicateBodyAndActuator, dtDynValidationCode::DuplicateName,
                                 dtDynObjectType::Body, secondBody,
                                 "body[2] duplicates name 'body_one'");
}

void TestModelValidationPreservesJointAndSiteValidationFailures()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("validated_body", 0);
    const dtDynIndex joint = spec.AddJoint(body, "validated_hinge", dtDynJointType::Hinge);
    const dtDynIndex site = spec.AddSite(body, "validated_site");
    const dtDynModel valid = CompileSpecChecked(spec);
    const dtDynReal nan = std::numeric_limits<dtDynReal>::quiet_NaN();

    dtDynModel negativeSpecId = valid;
    negativeSpecId.jnt_specid[static_cast<std::size_t>(joint)] = DTDYN_INVALID_INDEX;
    AssertModelValidationFailure(negativeSpecId, dtDynValidationCode::InvalidReference,
                                 dtDynObjectType::Joint, joint,
                                 "compiled joint specification provenance is invalid");

    dtDynModel oversizedSpecId = valid;
    oversizedSpecId.jnt_specid[static_cast<std::size_t>(joint)] = oversizedSpecId.njnt;
    AssertModelValidationFailure(oversizedSpecId, dtDynValidationCode::InvalidReference,
                                 dtDynObjectType::Joint, joint,
                                 "compiled joint specification provenance is invalid");

    dtDynModel badRangeMinimum = valid;
    badRangeMinimum.jnt_range_min(joint) = nan;
    AssertModelValidationFailure(badRangeMinimum, dtDynValidationCode::NonFiniteValue,
                                 dtDynObjectType::Joint, joint,
                                 "compiled joint contains a non-finite value");

    dtDynModel badRangeMaximum = valid;
    badRangeMaximum.jnt_range_max(joint) = nan;
    AssertModelValidationFailure(badRangeMaximum, dtDynValidationCode::NonFiniteValue,
                                 dtDynObjectType::Joint, joint,
                                 "compiled joint contains a non-finite value");

    dtDynModel badPosition = valid;
    badPosition.jnt_pos[static_cast<std::size_t>(joint)](0) = nan;
    AssertModelValidationFailure(badPosition, dtDynValidationCode::NonFiniteValue,
                                 dtDynObjectType::Joint, joint,
                                 "compiled joint contains a non-finite value");

    dtDynModel badAxis = valid;
    badAxis.jnt_axis[static_cast<std::size_t>(joint)](0) = nan;
    AssertModelValidationFailure(badAxis, dtDynValidationCode::NonFiniteValue,
                                 dtDynObjectType::Joint, joint,
                                 "compiled joint contains a non-finite value");

    dtDynModel nonUnitAxis = valid;
    nonUnitAxis.jnt_axis[static_cast<std::size_t>(joint)].SetZero();
    AssertModelValidationFailure(nonUnitAxis, dtDynValidationCode::InvalidReference,
                                 dtDynObjectType::Joint, joint,
                                 "compiled slide/hinge joint axis must be unit length");

    dtDynSpec slideSpec;
    const dtDynIndex slideBody = slideSpec.AddBody("slide_body", 0);
    slideSpec.AddJoint(slideBody, "slide_joint", dtDynJointType::Slide);
    const dtDynModel slideModel = CompileSpecChecked(slideSpec);
    dtDynValidationError slideError;
    assert(ValidateModel(slideModel, &slideError));
    assert(slideError.code == dtDynValidationCode::None);

    dtDynSpec freeSpec;
    const dtDynIndex freeBody = freeSpec.AddBody("free_body", 0);
    const dtDynIndex freeJoint = freeSpec.AddJoint(freeBody, "free_joint", dtDynJointType::Free);
    dtDynModel freeModel = CompileSpecChecked(freeSpec);
    freeModel.jnt_axis[static_cast<std::size_t>(freeJoint)].SetZero();
    dtDynValidationError freeError;
    assert(ValidateModel(freeModel, &freeError));
    assert(freeError.code == dtDynValidationCode::None);

    dtDynModel badSitePosition = valid;
    badSitePosition.site_pos[static_cast<std::size_t>(site)](0) = nan;
    AssertModelValidationFailure(badSitePosition, dtDynValidationCode::NonFiniteValue,
                                 dtDynObjectType::Site, site,
                                 "compiled site contains a non-finite value");

    dtDynModel badSiteQuaternion = valid;
    badSiteQuaternion.site_quat[static_cast<std::size_t>(site)](0) = nan;
    AssertModelValidationFailure(badSiteQuaternion, dtDynValidationCode::NonFiniteValue,
                                 dtDynObjectType::Site, site,
                                 "compiled site contains a non-finite value");

    dtDynModel badSiteMatrix = valid;
    badSiteMatrix.site_mat[static_cast<std::size_t>(site)](0, 0) = nan;
    AssertModelValidationFailure(badSiteMatrix, dtDynValidationCode::NonFiniteValue,
                                 dtDynObjectType::Site, site,
                                 "compiled site contains a non-finite value");

    dtDynModel improperSiteRotation = valid;
    improperSiteRotation.site_mat[static_cast<std::size_t>(site)].SetZero();
    AssertModelValidationFailure(improperSiteRotation, dtDynValidationCode::InvalidReference,
                                 dtDynObjectType::Site, site,
                                 "compiled site quaternion/matrix frame pair is invalid");

    dtDynModel mismatchedSiteFrame = valid;
    mismatchedSiteFrame.site_quat[static_cast<std::size_t>(site)](0) = 0.0;
    mismatchedSiteFrame.site_quat[static_cast<std::size_t>(site)](3) = 1.0;
    AssertModelValidationFailure(mismatchedSiteFrame, dtDynValidationCode::InvalidReference,
                                 dtDynObjectType::Site, site,
                                 "compiled site quaternion/matrix frame pair is invalid");
}

void AssertBodyValidationFailure(const dtDynModel &model, dtDynValidationCode code,
                                 const std::string &message)
{
    dtDynValidationError error;
    assert(!ValidateModel(model, &error));
    assert(error.code == code);
    assert(error.object == dtDynObjectType::Body);
    assert(error.index == 1);
    assert(error.message == message);
}

void TestModelValidationPreservesEveryBodyNonFiniteFailure()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("validated_body", 0);
    const dtDynModel valid = CompileSpecChecked(spec);
    assert(valid.nbody == 2);
    assert(valid.nroot == 1);

    const auto assertNonFinite = [&](const dtDynModel &candidate)
    {
        AssertBodyValidationFailure(candidate, dtDynValidationCode::NonFiniteValue,
                                    "compiled body[1] contains a non-finite value");
    };
    const dtDynReal nan = std::numeric_limits<dtDynReal>::quiet_NaN();

    dtDynModel badMass = valid;
    badMass.body_mass(body) = nan;
    assertNonFinite(badMass);

    dtDynModel badSubtreeMass = valid;
    badSubtreeMass.body_subtreemass(body) = nan;
    assertNonFinite(badSubtreeMass);

    dtDynModel badPosition = valid;
    badPosition.body_pos[static_cast<std::size_t>(body)](0) = nan;
    assertNonFinite(badPosition);

    dtDynModel badQuaternion = valid;
    badQuaternion.body_quat[static_cast<std::size_t>(body)](0) = nan;
    assertNonFinite(badQuaternion);

    dtDynModel badRotation = valid;
    badRotation.body_mat[static_cast<std::size_t>(body)](0, 0) = nan;
    assertNonFinite(badRotation);

    dtDynModel badInertialPosition = valid;
    badInertialPosition.body_ipos[static_cast<std::size_t>(body)](0) = nan;
    assertNonFinite(badInertialPosition);

    dtDynModel badInertialQuaternion = valid;
    badInertialQuaternion.body_iquat[static_cast<std::size_t>(body)](0) = nan;
    assertNonFinite(badInertialQuaternion);

    dtDynModel badInertialRotation = valid;
    badInertialRotation.body_imat[static_cast<std::size_t>(body)](0, 0) = nan;
    assertNonFinite(badInertialRotation);

    dtDynModel badInertia = valid;
    badInertia.body_inertia[static_cast<std::size_t>(body)](0) = nan;
    assertNonFinite(badInertia);
}

void TestModelValidationPreservesEveryBodyPhysicalOrFrameFailure()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("validated_body", 0);
    const dtDynModel valid = CompileSpecChecked(spec);
    assert(valid.nbody == 2);
    assert(valid.nroot == 1);

    const auto assertInvalidReference = [&](const dtDynModel &candidate)
    {
        AssertBodyValidationFailure(candidate, dtDynValidationCode::InvalidReference,
                                    "compiled body has invalid physical or frame-pair values");
    };

    dtDynModel negativeMass = valid;
    negativeMass.body_mass(body) = -1.0;
    assertInvalidReference(negativeMass);

    dtDynModel invalidInertia = valid;
    invalidInertia.body_inertia[static_cast<std::size_t>(body)](0) = 3.0;
    invalidInertia.body_inertia[static_cast<std::size_t>(body)](1) = 1.0;
    invalidInertia.body_inertia[static_cast<std::size_t>(body)](2) = 1.0;
    assertInvalidReference(invalidInertia);

    dtDynModel invalidBodyRotation = valid;
    invalidBodyRotation.body_mat[static_cast<std::size_t>(body)].SetZero();
    assertInvalidReference(invalidBodyRotation);

    dtDynModel invalidInertialRotation = valid;
    invalidInertialRotation.body_imat[static_cast<std::size_t>(body)].SetZero();
    assertInvalidReference(invalidInertialRotation);

    dtDynModel mismatchedBodyFrame = valid;
    mismatchedBodyFrame.body_quat[static_cast<std::size_t>(body)](0) = 0.0;
    mismatchedBodyFrame.body_quat[static_cast<std::size_t>(body)](3) = 1.0;
    assertInvalidReference(mismatchedBodyFrame);

    dtDynModel mismatchedInertialFrame = valid;
    mismatchedInertialFrame.body_iquat[static_cast<std::size_t>(body)](0) = 0.0;
    mismatchedInertialFrame.body_iquat[static_cast<std::size_t>(body)](3) = 1.0;
    assertInvalidReference(mismatchedInertialFrame);
}

void TestModelValidationPreservesBodyValidationPhasePrecedence()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("validated_body", 0);
    const dtDynModel valid = CompileSpecChecked(spec);
    assert(valid.nbody == 2);
    assert(valid.nroot == 1);

    dtDynModel nonFiniteBeforePhysical = valid;
    nonFiniteBeforePhysical.body_inertia[static_cast<std::size_t>(body)](0) =
        std::numeric_limits<dtDynReal>::quiet_NaN();
    nonFiniteBeforePhysical.body_mass(body) = -1.0;
    AssertBodyValidationFailure(nonFiniteBeforePhysical, dtDynValidationCode::NonFiniteValue,
                                "compiled body[1] contains a non-finite value");

    dtDynModel physicalBeforeTopology = valid;
    physicalBeforeTopology.body_mass(body) = -1.0;
    physicalBeforeTopology.body_parentid[static_cast<std::size_t>(body)] = body;
    AssertBodyValidationFailure(physicalBeforeTopology, dtDynValidationCode::InvalidReference,
                                "compiled body has invalid physical or frame-pair values");
}

void TestCompileSpecSupportsMaximumRootMappingWithoutNestedLookup()
{
    constexpr dtDynIndex rootCount = 10922;
    constexpr dtDynIndex bodyCount = 65535;
    dtDynSpec spec;
    dtDynIndex lastRoot = DTDYN_INVALID_INDEX;
    for (dtDynIndex root = 0; root < rootCount; ++root)
    {
        lastRoot = spec.AddBody("root_" + std::to_string(root), 0);
    }
    dtDynIndex parent = lastRoot;
    for (dtDynIndex body = 1 + rootCount; body < bodyCount; ++body)
    {
        parent = spec.AddBody("tail_" + std::to_string(body), parent);
    }

    const dtDynModel model = CompileSpecChecked(spec);
    assert(model.nbody == bodyCount);
    assert(model.nroot == rootCount);
    assert(model.body_rootidx[static_cast<std::size_t>(bodyCount - 1)] == rootCount - 1);
}

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

void TestDtArrayCopyConstructorReleasesFailedPartialCopy()
{
    dtArray<dtDynBodySpec> source;
    dtDynBodySpec body;
    body.name.assign(256, 'x');
    source.push_back(body);
    const std::size_t baselineAllocations = g_outstandingTestAllocations;
    bool observedFailure = false;

    for (std::size_t allocationsBeforeFailure = 0;
         allocationsBeforeFailure < kMaximumAllocationFailuresToTest;
         ++allocationsBeforeFailure)
    {
        bool rejected = false;
        {
            ScopedAllocationFailure allocationFailure(allocationsBeforeFailure);
            try
            {
                const dtArray<dtDynBodySpec> copy(source);
                assert(copy.size() == source.size());
            }
            catch (const std::bad_alloc &)
            {
                rejected = true;
            }
        }
        if (rejected)
        {
            observedFailure = true;
            assert(g_outstandingTestAllocations == baselineAllocations);
        }
    }

    assert(observedFailure);
}

void TestDtArrayReserveCopiesWithoutLeakingOrMutatingOnThrow()
{
    ThrowingTransfer::throwOnCopy = false;
    ThrowingTransfer::throwOnMove = false;
    ThrowingTransfer first(1);
    ThrowingTransfer second(2);

    dtArray<ThrowingTransfer> moveSensitive;
    moveSensitive.push_back(first);
    ThrowingTransfer::throwOnMove = true;
    moveSensitive.push_back(second);
    assert(moveSensitive.size() == 2);
    assert(moveSensitive[0].value == 1);
    assert(moveSensitive[1].value == 2);
    ThrowingTransfer::throwOnMove = false;

    dtArray<ThrowingTransfer> copySensitive;
    copySensitive.push_back(first);
    const int liveBeforeFailure = ThrowingTransfer::liveCount;
    ThrowingTransfer::throwOnCopy = true;
    bool rejected = false;
    try
    {
        copySensitive.push_back(second);
    }
    catch (const std::runtime_error &)
    {
        rejected = true;
    }
    ThrowingTransfer::throwOnCopy = false;

    assert(rejected);
    assert(copySensitive.size() == 1);
    assert(copySensitive[0].value == 1);
    assert(ThrowingTransfer::liveCount == liveBeforeFailure);
}

void TestDtArrayPushBackRetainsByValueSignature()
{
    void (dtArray<int>::*const append)(int) = &dtArray<int>::push_back;
    dtArray<int> values;
    (values.*append)(7);

    assert(values.size() == 1);
    assert(values[0] == 7);
}

void TestDtArrayPushBackMovesRvalue()
{
    ThrowingTransfer::throwOnCopy = false;
    ThrowingTransfer::throwOnMove = false;
    ThrowingTransfer source(9);
    dtArray<ThrowingTransfer> values;

    ThrowingTransfer::throwOnCopy = true;
    values.push_back(std::move(source));
    ThrowingTransfer::throwOnCopy = false;

    assert(values.size() == 1);
    assert(values[0].value == 9);
    assert(source.value == 0);
}

void TestDtArrayPushBackSupportsTriviallyCopyableRvalueWithoutMoveConstructionOrAssignment()
{
    dtArray<CopyOnlyTrivialTransfer> values;
    values.push_back(CopyOnlyTrivialTransfer{7});
    values.push_back(CopyOnlyTrivialTransfer{9});

    assert(values.size() == 2);
    assert(values[0].value == 7);
    assert(values[1].value == 9);
}

void TestDtArrayPushBackSupportsTriviallyCopyableMoveOnlyRvalue()
{
    MoveOnlyTrivialTransfer first;
    first.value = 7;
    MoveOnlyTrivialTransfer second;
    second.value = 9;

    dtArray<MoveOnlyTrivialTransfer> values;
    values.push_back(std::move(first));
    values.push_back(std::move(second));

    assert(values.size() == 2);
    assert(values[0].value == 7);
    assert(values[1].value == 9);
}

void TestDtArrayPushBackSupportsCopyConstructibleMoveAssignableLvalue()
{
    CopyConstructibleMoveAssignableOnly source;
    source.value = 11;
    dtArray<CopyConstructibleMoveAssignableOnly> values;
    values.push_back(source);
    values.push_back(source);

    assert(values.size() == 2);
    assert(values[0].value == 11);
    assert(values[1].value == 11);
    assert(source.value == 11);
}

void TestDtArrayPushBackPreservesMutableCopyAssignmentSelection()
{
    OverloadedCopyAssignment source;
    source.value = 13;
    dtArray<OverloadedCopyAssignment> values;
    values.push_back(source);

    assert(values.size() == 1);
    assert(values[0].value == 13);
    assert(values[0].selection == OverloadedCopyAssignment::Selection::Mutable);
}

void TestDtArrayPushBackSupportsMoveOnlyElements()
{
    dtArray<std::unique_ptr<int>> values;
    values.push_back(std::make_unique<int>(7));
    values.push_back(std::make_unique<int>(9));

    assert(values.size() == 2);
    assert(*values[0] == 7);
    assert(*values[1] == 9);
}

void TestDtArrayPushBackPreservesSelfRvalueTransferSemantics()
{
    ThrowingTransfer::throwOnCopy = false;
    ThrowingTransfer::throwOnMove = false;
    ThrowingTransfer value(5);
    dtArray<ThrowingTransfer> values;
    values.push_back(value);
    values.push_back(std::move(values[0]));

    assert(values.size() == 2);
    assert(values[0].value == 0);
    assert(values[1].value == 5);
}

void TestDtArrayPushBackPreservesAliasedLvalueOnGrowth()
{
    ThrowingTransfer::throwOnCopy = false;
    ThrowingTransfer::throwOnMove = false;
    ThrowingTransfer value(7);
    dtArray<ThrowingTransfer> values;
    values.push_back(value);
    values.push_back(values[0]);

    assert(values.size() == 2);
    assert(values[0].value == 7);
    assert(values[1].value == 7);
}

void TestDtArrayPushBackPreservesCopyOnlyAliasedLvalueOnGrowth()
{
    CopyOnlyTrivialTransfer value;
    value.value = 7;
    dtArray<CopyOnlyTrivialTransfer> values;
    values.push_back(value);
    values.push_back(values[0]);

    assert(values.size() == 2);
    assert(values[0].value == 7);
    assert(values[1].value == 7);
}

void TestDtArrayReserveSupportsMoveOnlyElements()
{
    dtArray<std::unique_ptr<int>> values;
    values.resize(1);
    values[0] = std::make_unique<int>(7);
    values.resize(2);

    assert(values.size() == 2);
    assert(values[0] != nullptr);
    assert(*values[0] == 7);
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

void TestSpecEditorsCommitScopedCopies()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    const dtDynIndex joint = spec.AddJoint(body, "joint", dtDynJointType::Hinge);
    const dtDynIndex site = spec.AddSite(body, "tool");
    const dtDynIndex actuator = spec.AddActuator(joint, "motor");

    spec.EditBody(body, [](dtDynBodySpec &draft)
                  {
        draft.pos(0) = 1.0;
        draft.inertial.mass = 2.0; });
    spec.EditJoint(joint, [](dtDynJointSpec &draft)
                   {
        draft.axis(1) = 1.0;
        draft.armature = 0.1; });
    spec.EditSite(site, [](dtDynSiteSpec &draft)
                  { draft.pos(2) = 0.5; });
    spec.EditActuator(actuator, [](dtDynActuatorSpec &draft)
                      { draft.gear = 80.0; });

    const dtDynSpec &view = spec;
    assert(view.Bodies()[body].pos(0) == 1.0);
    assert(view.Bodies()[body].inertial.mass == 2.0);
    assert(view.Joints()[joint].axis(1) == 1.0);
    assert(view.Joints()[joint].armature == 0.1);
    assert(view.Sites()[site].pos(2) == 0.5);
    assert(view.Actuators()[actuator].gear == 80.0);
}

void TestSpecEditorsDiscardFailedDrafts()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    const dtDynReal originalMass = spec.Bodies()[body].inertial.mass;

    bool rejected = false;
    try
    {
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
            draft.inertial.mass = 7.0;
            throw std::runtime_error("reject draft"); });
    }
    catch (const std::runtime_error &)
    {
        rejected = true;
    }
    assert(rejected);
    assert(spec.Bodies()[body].inertial.mass == originalMass);

    bool invalidIndexRejected = false;
    try
    {
        spec.EditBody(99, [](dtDynBodySpec &) {});
    }
    catch (const std::out_of_range &)
    {
        invalidIndexRejected = true;
    }
    assert(invalidIndexRejected);
}

void TestSpecEditorsRejectReentrantMutation()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);

    bool rejected = false;
    try
    {
        spec.EditBody(body, [&](dtDynBodySpec &)
                      {
            try
            {
                spec.EditBody(body, [](dtDynBodySpec &) {});
            }
            catch (const std::logic_error &)
            {
            } });
    }
    catch (const std::logic_error &)
    {
        rejected = true;
    }
    assert(rejected);
    assert(spec.Bodies()[body].name == "base");
}

void TestSpecEditorsDiscardCaughtReentrantAttempt()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    const dtDynReal originalMass = spec.Bodies()[body].inertial.mass;

    bool rejected = false;
    try
    {
        spec.EditBody(body, [&](dtDynBodySpec &draft)
                      {
            draft.inertial.mass = 7.0;
            try
            {
                spec.Clear();
            }
            catch (const std::logic_error &)
            {
            } });
    }
    catch (const std::logic_error &)
    {
        rejected = true;
    }
    assert(rejected);
    assert(spec.Bodies()[body].inertial.mass == originalMass);
}

void TestSpecEditorsRejectReentrantStructuralMutation()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    const dtDynIndex joint = spec.AddJoint(body, "joint", dtDynJointType::Hinge);
    const dtDynIndex site = spec.AddSite(body, "site");
    const dtDynIndex actuator = spec.AddActuator(joint, "actuator");
    dtDynSpec replacement;
    replacement.AddBody("replacement", 0);
    dtDynSpec copied;
    copied = spec;
    assert(copied.Bodies()[body].name == "base");
    assert(copied.Joints()[joint].name == "joint");

    spec = spec;
    spec = std::move(spec);
    assert(spec.Bodies()[body].name == "base");
    assert(spec.Joints()[joint].name == "joint");

    const auto expectLogicError = [](const auto &operation)
    {
        bool rejected = false;
        try
        {
            operation();
        }
        catch (const std::logic_error &)
        {
            rejected = true;
        }
        assert(rejected);
    };

    expectLogicError([&]
                     { spec.EditBody(body, [&](dtDynBodySpec &)
                                     {
            try
            {
                spec.Clear();
            }
            catch (const std::logic_error &)
            {
            } }); });
    assert(spec.Bodies().size() == 2);
    assert(spec.Bodies()[body].name == "base");

    expectLogicError([&]
                     { spec.EditJoint(joint, [&](dtDynJointSpec &)
                                      {
            try
            {
                spec.EditJoint(joint, [](dtDynJointSpec &) {});
            }
            catch (const std::logic_error &)
            {
            } }); });
    assert(spec.Joints()[joint].name == "joint");

    expectLogicError([&]
                     { spec.EditSite(site, [&](dtDynSiteSpec &)
                                     {
            try
            {
                spec.AddBody("forbidden", body);
            }
            catch (const std::logic_error &)
            {
            } }); });
    assert(spec.Bodies().size() == 2);

    expectLogicError([&]
                     { spec.EditActuator(actuator, [&](dtDynActuatorSpec &)
                                         {
            try
            {
                spec.EditActuator(actuator, [](dtDynActuatorSpec &) {});
            }
            catch (const std::logic_error &)
            {
            } }); });
    assert(spec.Actuators()[actuator].name == "actuator");

    expectLogicError([&]
                     { spec.EditBody(body, [&](dtDynBodySpec &)
                                     {
            try
            {
                swap(spec, replacement);
            }
            catch (const std::logic_error &)
            {
            } }); });
    assert(spec.Bodies()[body].name == "base");

    expectLogicError([&]
                     { spec.EditBody(body, [&](dtDynBodySpec &)
                                     {
            try
            {
                swap(replacement, spec);
            }
            catch (const std::logic_error &)
            {
            } }); });
    assert(spec.Bodies()[body].name == "base");
}

void TestSpecEditorsRejectReentrantAssignment()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    const dtDynReal originalMass = spec.Bodies()[body].inertial.mass;
    dtDynSpec replacement;
    replacement.AddBody("replacement", 0);

    const auto expectLogicError = [](const auto &operation)
    {
        bool rejected = false;
        try
        {
            operation();
        }
        catch (const std::logic_error &)
        {
            rejected = true;
        }
        assert(rejected);
    };

    expectLogicError([&]
                     { spec.EditBody(body, [&](dtDynBodySpec &draft)
                                     {
            draft.inertial.mass = 1.0;
            spec = replacement; }); });
    assert(spec.Bodies()[body].inertial.mass == originalMass);

    expectLogicError([&]
                     { spec.EditBody(body, [&](dtDynBodySpec &draft)
                                     {
            draft.inertial.mass = 2.0;
            try
            {
                spec = spec;
            }
            catch (const std::logic_error &)
            {
            } }); });
    assert(spec.Bodies()[body].inertial.mass == originalMass);

    expectLogicError([&]
                     { spec.EditBody(body, [&](dtDynBodySpec &draft)
                                     {
            draft.inertial.mass = 3.0;
            spec = std::move(spec); }); });
    assert(spec.Bodies()[body].inertial.mass == originalMass);

    expectLogicError([&]
                     { spec.EditBody(body, [&](dtDynBodySpec &draft)
                                     {
            draft.inertial.mass = 4.0;
            try
            {
                spec = std::move(spec);
            }
            catch (const std::logic_error &)
            {
            } }); });
    assert(spec.Bodies()[body].inertial.mass == originalMass);
}

void TestSpecEditorsAbortReentrantCopyAssignmentTransactions()
{
    const auto seed = [](dtDynSpec &first,
                         const dtDynIndex firstBody,
                         dtDynSpec &second,
                         const dtDynIndex secondBody)
    {
        first.EditBody(firstBody, [](dtDynBodySpec &draft)
                       { draft.inertial.mass = 1.25; });
        second.EditBody(secondBody, [](dtDynBodySpec &draft)
                        { draft.inertial.mass = 2.5; });
    };
    const auto assertUnchanged = [](const dtDynSpec &first,
                                    const dtDynIndex firstBody,
                                    const dtDynReal firstMass,
                                    const dtDynSpec &second,
                                    const dtDynIndex secondBody,
                                    const dtDynReal secondMass)
    {
        assert(first.Bodies().size() == 2);
        assert(second.Bodies().size() == 2);
        assert(first.Bodies()[firstBody].name == "first");
        assert(second.Bodies()[secondBody].name == "second");
        assert(first.Bodies()[firstBody].inertial.mass == firstMass);
        assert(second.Bodies()[secondBody].inertial.mass == secondMass);
    };

    {
        dtDynSpec source;
        dtDynSpec destination;
        const dtDynIndex sourceBody = source.AddBody("first", 0);
        const dtDynIndex destinationBody = destination.AddBody("second", 0);
        seed(source, sourceBody, destination, destinationBody);
        const dtDynReal sourceMass = source.Bodies()[sourceBody].inertial.mass;
        const dtDynReal destinationMass = destination.Bodies()[destinationBody].inertial.mass;
        bool assignmentCaught = false;
        bool sourceRejected = false;

        try
        {
            source.EditBody(sourceBody, [&](dtDynBodySpec &draft)
                            {
                draft.inertial.mass = 7.0;
                try
                {
                    destination = source;
                }
                catch (const std::logic_error &)
                {
                    assignmentCaught = true;
                } });
        }
        catch (const std::logic_error &)
        {
            sourceRejected = true;
        }

        assert(assignmentCaught);
        assert(sourceRejected);
        assertUnchanged(source, sourceBody, sourceMass,
                        destination, destinationBody, destinationMass);
    }

    {
        dtDynSpec source;
        dtDynSpec destination;
        const dtDynIndex sourceBody = source.AddBody("first", 0);
        const dtDynIndex destinationBody = destination.AddBody("second", 0);
        seed(source, sourceBody, destination, destinationBody);
        const dtDynReal sourceMass = source.Bodies()[sourceBody].inertial.mass;
        const dtDynReal destinationMass = destination.Bodies()[destinationBody].inertial.mass;
        bool assignmentCaught = false;
        bool destinationRejected = false;

        try
        {
            destination.EditBody(destinationBody, [&](dtDynBodySpec &draft)
                                 {
                draft.inertial.mass = 9.0;
                try
                {
                    destination = source;
                }
                catch (const std::logic_error &)
                {
                    assignmentCaught = true;
                } });
        }
        catch (const std::logic_error &)
        {
            destinationRejected = true;
        }

        assert(assignmentCaught);
        assert(destinationRejected);
        assertUnchanged(source, sourceBody, sourceMass,
                        destination, destinationBody, destinationMass);
    }

    for (const bool reverseOperands : {false, true})
    {
        dtDynSpec first;
        dtDynSpec second;
        const dtDynIndex firstBody = first.AddBody("first", 0);
        const dtDynIndex secondBody = second.AddBody("second", 0);
        seed(first, firstBody, second, secondBody);
        const dtDynReal firstMass = first.Bodies()[firstBody].inertial.mass;
        const dtDynReal secondMass = second.Bodies()[secondBody].inertial.mass;
        bool assignmentCaught = false;
        bool firstRejected = false;
        bool secondRejected = false;

        try
        {
            first.EditBody(firstBody, [&](dtDynBodySpec &firstDraft)
                           {
                firstDraft.inertial.mass = 11.0;
                try
                {
                    second.EditBody(secondBody, [&](dtDynBodySpec &secondDraft)
                                    {
                        secondDraft.inertial.mass = 13.0;
                        try
                        {
                            if (reverseOperands)
                            {
                                first = second;
                            }
                            else
                            {
                                second = first;
                            }
                        }
                        catch (const std::logic_error &)
                        {
                            assignmentCaught = true;
                        }
                    });
                }
                catch (const std::logic_error &)
                {
                    secondRejected = true;
                } });
        }
        catch (const std::logic_error &)
        {
            firstRejected = true;
        }

        assert(assignmentCaught);
        assert(firstRejected);
        assert(secondRejected);
        assertUnchanged(first, firstBody, firstMass, second, secondBody, secondMass);
    }
}

void TestSpecOutsideEditSelfAssignmentRemainsUsable()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("base", 0);
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  { draft.inertial.mass = 4.0; });

    spec = spec;
    assert(spec.Bodies().size() == 2);
    assert(spec.Bodies()[body].name == "base");
    assert(spec.Bodies()[body].inertial.mass == 4.0);

    spec = std::move(spec);
    assert(spec.Bodies().size() == 2);
    assert(spec.Bodies()[body].name == "base");
    assert(spec.Bodies()[body].inertial.mass == 4.0);
}

void TestSpecCopyAssignmentKeepsTargetUnchangedWhenStorageCopyThrows()
{
    dtDynSpec source;
    const dtDynIndex sourceBody = source.AddBody("sbody", 0);
    const dtDynIndex sourceJoint = source.AddJoint(sourceBody, "sjoint", dtDynJointType::Hinge);
    const dtDynIndex sourceSite = source.AddSite(sourceBody, "ssite");
    const dtDynIndex sourceActuator = source.AddActuator(sourceJoint, "sact");

    bool observedSuccessfulAssignment = false;
    for (std::size_t allocationsBeforeFailure = 0;
         allocationsBeforeFailure < kMaximumAllocationFailuresToTest;
         ++allocationsBeforeFailure)
    {
        dtDynSpec destination;
        const dtDynIndex destinationBody = destination.AddBody("tbody", 0);
        const dtDynIndex destinationJoint = destination.AddJoint(destinationBody, "tjoint", dtDynJointType::Hinge);
        const dtDynIndex destinationSite = destination.AddSite(destinationBody, "tsite");
        const dtDynIndex destinationActuator = destination.AddActuator(destinationJoint, "tact");
        const dtArray<dtDynBodySpec> &retainedBodies = destination.Bodies();
        const dtArray<dtDynJointSpec> &retainedJoints = destination.Joints();
        const dtArray<dtDynSiteSpec> &retainedSites = destination.Sites();
        const dtArray<dtDynActuatorSpec> &retainedActuators = destination.Actuators();

        bool rejected = false;
        {
            ScopedAllocationFailure allocationFailure(allocationsBeforeFailure);
            try
            {
                destination = source;
            }
            catch (const std::bad_alloc &)
            {
                rejected = true;
            }
        }

        if (!rejected)
        {
            assert(retainedBodies[sourceBody].name == "sbody");
            assert(retainedJoints[sourceJoint].name == "sjoint");
            assert(retainedSites[sourceSite].name == "ssite");
            assert(retainedActuators[sourceActuator].name == "sact");
            observedSuccessfulAssignment = true;
            break;
        }

        assert(&destination.Bodies() == &retainedBodies);
        assert(&destination.Joints() == &retainedJoints);
        assert(&destination.Sites() == &retainedSites);
        assert(&destination.Actuators() == &retainedActuators);
        assert(retainedBodies.size() == 2);
        assert(retainedBodies[destinationBody].name == "tbody");
        assert(retainedJoints.size() == 1);
        assert(retainedJoints[destinationJoint].name == "tjoint");
        assert(retainedSites.size() == 1);
        assert(retainedSites[destinationSite].name == "tsite");
        assert(retainedActuators.size() == 1);
        assert(retainedActuators[destinationActuator].name == "tact");
    }

    assert(observedSuccessfulAssignment);
}

void TestSpecCollectionViewsSurviveCopyAssignmentAndMutations()
{
    dtDynSpec source;
    const dtDynIndex sourceBody = source.AddBody("source_body", 0);
    const dtDynIndex sourceJoint = source.AddJoint(sourceBody, "source_joint", dtDynJointType::Hinge);
    const dtDynIndex sourceSite = source.AddSite(sourceBody, "source_site");
    const dtDynIndex sourceActuator = source.AddActuator(sourceJoint, "source_actuator");

    dtDynSpec destination;
    const dtDynIndex targetBody = destination.AddBody("target_body", 0);
    const dtDynIndex targetJoint = destination.AddJoint(targetBody, "target_joint", dtDynJointType::Hinge);
    destination.AddSite(targetBody, "target_site");
    destination.AddActuator(targetJoint, "target_actuator");
    const dtArray<dtDynBodySpec> &retainedBodies = destination.Bodies();
    const dtArray<dtDynJointSpec> &retainedJoints = destination.Joints();
    const dtArray<dtDynSiteSpec> &retainedSites = destination.Sites();
    const dtArray<dtDynActuatorSpec> &retainedActuators = destination.Actuators();

    destination = source;
    assert(&destination.Bodies() == &retainedBodies);
    assert(&destination.Joints() == &retainedJoints);
    assert(&destination.Sites() == &retainedSites);
    assert(&destination.Actuators() == &retainedActuators);
    assert(retainedBodies[sourceBody].name == "source_body");
    assert(retainedJoints[sourceJoint].name == "source_joint");
    assert(retainedSites[sourceSite].name == "source_site");
    assert(retainedActuators[sourceActuator].name == "source_actuator");

    destination.EditBody(sourceBody, [](dtDynBodySpec &draft)
                         { draft.name = "edited_body"; });
    destination.EditJoint(sourceJoint, [](dtDynJointSpec &draft)
                          { draft.name = "edited_joint"; });
    destination.EditSite(sourceSite, [](dtDynSiteSpec &draft)
                         { draft.name = "edited_site"; });
    destination.EditActuator(sourceActuator, [](dtDynActuatorSpec &draft)
                             { draft.name = "edited_actuator"; });
    assert(retainedBodies[sourceBody].name == "edited_body");
    assert(retainedJoints[sourceJoint].name == "edited_joint");
    assert(retainedSites[sourceSite].name == "edited_site");
    assert(retainedActuators[sourceActuator].name == "edited_actuator");

    const dtDynModel compiled = CompileSpec(destination);
    assert(compiled.nbody == 2);
    assert(compiled.njnt == 1);
    assert(compiled.nsite == 1);
    assert(compiled.nu == 1);
    const std::string encoded = destination.EncodeXml();
    assert(encoded.find("edited_body") != std::string::npos);
    assert(encoded.find("edited_joint") != std::string::npos);
    assert(encoded.find("edited_site") != std::string::npos);
    assert(encoded.find("edited_actuator") != std::string::npos);

    const dtDynIndex addedBody = destination.AddBody("added_body", 0);
    const dtDynIndex addedJoint = destination.AddJoint(addedBody, "added_joint", dtDynJointType::Hinge);
    const dtDynIndex addedSite = destination.AddSite(addedBody, "added_site");
    const dtDynIndex addedActuator = destination.AddActuator(addedJoint, "added_actuator");
    assert(retainedBodies[addedBody].name == "added_body");
    assert(retainedJoints[addedJoint].name == "added_joint");
    assert(retainedSites[addedSite].name == "added_site");
    assert(retainedActuators[addedActuator].name == "added_actuator");

    destination.Clear();
    assert(&destination.Bodies() == &retainedBodies);
    assert(&destination.Joints() == &retainedJoints);
    assert(&destination.Sites() == &retainedSites);
    assert(&destination.Actuators() == &retainedActuators);
    assert(retainedBodies.size() == 1);
    assert(retainedBodies[0].name == "world");
    assert(retainedJoints.empty());
    assert(retainedSites.empty());
    assert(retainedActuators.empty());
}

void TestSpecCollectionViewsSurviveMoveAssignment()
{
    dtDynSpec source;
    const dtDynIndex sourceBody = source.AddBody("source_body", 0);
    const dtDynIndex sourceJoint = source.AddJoint(sourceBody, "source_joint", dtDynJointType::Hinge);
    const dtDynIndex sourceSite = source.AddSite(sourceBody, "source_site");
    const dtDynIndex sourceActuator = source.AddActuator(sourceJoint, "source_actuator");

    dtDynSpec destination;
    const dtDynIndex targetBody = destination.AddBody("target_body", 0);
    const dtDynIndex targetJoint = destination.AddJoint(targetBody, "target_joint", dtDynJointType::Hinge);
    destination.AddSite(targetBody, "target_site");
    destination.AddActuator(targetJoint, "target_actuator");
    const dtArray<dtDynBodySpec> &retainedBodies = destination.Bodies();
    const dtArray<dtDynJointSpec> &retainedJoints = destination.Joints();
    const dtArray<dtDynSiteSpec> &retainedSites = destination.Sites();
    const dtArray<dtDynActuatorSpec> &retainedActuators = destination.Actuators();

    destination = std::move(source);
    assert(&destination.Bodies() == &retainedBodies);
    assert(&destination.Joints() == &retainedJoints);
    assert(&destination.Sites() == &retainedSites);
    assert(&destination.Actuators() == &retainedActuators);
    assert(retainedBodies[sourceBody].name == "source_body");
    assert(retainedJoints[sourceJoint].name == "source_joint");
    assert(retainedSites[sourceSite].name == "source_site");
    assert(retainedActuators[sourceActuator].name == "source_actuator");
    assert(source.Bodies().empty());
    assert(source.Joints().empty());
    assert(source.Sites().empty());
    assert(source.Actuators().empty());
}

void TestSpecEditorsAbortReentrantMoveAssignmentTransactions()
{
    const auto seed = [](dtDynSpec &first,
                         const dtDynIndex firstBody,
                         dtDynSpec &second,
                         const dtDynIndex secondBody)
    {
        first.EditBody(firstBody, [](dtDynBodySpec &draft)
                       { draft.inertial.mass = 1.25; });
        second.EditBody(secondBody, [](dtDynBodySpec &draft)
                        { draft.inertial.mass = 2.5; });
    };
    const auto assertUnchanged = [](const dtDynSpec &first,
                                    const dtDynIndex firstBody,
                                    const dtDynReal firstMass,
                                    const dtDynSpec &second,
                                    const dtDynIndex secondBody,
                                    const dtDynReal secondMass)
    {
        assert(first.Bodies().size() == 2);
        assert(second.Bodies().size() == 2);
        assert(first.Bodies()[firstBody].name == "first");
        assert(second.Bodies()[secondBody].name == "second");
        assert(first.Bodies()[firstBody].inertial.mass == firstMass);
        assert(second.Bodies()[secondBody].inertial.mass == secondMass);
    };
    const auto expectSourceAbort = [&](const bool reverseOperands)
    {
        dtDynSpec first;
        dtDynSpec second;
        const dtDynIndex firstBody = first.AddBody("first", 0);
        const dtDynIndex secondBody = second.AddBody("second", 0);
        seed(first, firstBody, second, secondBody);
        const dtDynReal firstMass = first.Bodies()[firstBody].inertial.mass;
        const dtDynReal secondMass = second.Bodies()[secondBody].inertial.mass;
        bool sourceRejected = false;

        dtDynSpec &source = reverseOperands ? second : first;
        dtDynSpec &destination = reverseOperands ? first : second;
        const dtDynIndex sourceBody = reverseOperands ? secondBody : firstBody;
        try
        {
            source.EditBody(sourceBody, [&](dtDynBodySpec &draft)
                            {
                draft.inertial.mass = 7.0;
                try
                {
                    destination = std::move(source);
                }
                catch (const std::logic_error &)
                {
                } });
        }
        catch (const std::logic_error &)
        {
            sourceRejected = true;
        }

        assert(sourceRejected);
        assertUnchanged(first, firstBody, firstMass, second, secondBody, secondMass);
    };
    const auto expectDestinationAbort = [&](const bool reverseOperands)
    {
        dtDynSpec first;
        dtDynSpec second;
        const dtDynIndex firstBody = first.AddBody("first", 0);
        const dtDynIndex secondBody = second.AddBody("second", 0);
        seed(first, firstBody, second, secondBody);
        const dtDynReal firstMass = first.Bodies()[firstBody].inertial.mass;
        const dtDynReal secondMass = second.Bodies()[secondBody].inertial.mass;
        bool destinationRejected = false;

        dtDynSpec &destination = reverseOperands ? first : second;
        dtDynSpec &source = reverseOperands ? second : first;
        const dtDynIndex destinationBody = reverseOperands ? firstBody : secondBody;
        try
        {
            destination.EditBody(destinationBody, [&](dtDynBodySpec &draft)
                                 {
                draft.inertial.mass = 9.0;
                try
                {
                    destination = std::move(source);
                }
                catch (const std::logic_error &)
                {
                } });
        }
        catch (const std::logic_error &)
        {
            destinationRejected = true;
        }

        assert(destinationRejected);
        assertUnchanged(first, firstBody, firstMass, second, secondBody, secondMass);
    };
    const auto expectBothAborted = [&](const bool reverseOperands)
    {
        dtDynSpec first;
        dtDynSpec second;
        const dtDynIndex firstBody = first.AddBody("first", 0);
        const dtDynIndex secondBody = second.AddBody("second", 0);
        seed(first, firstBody, second, secondBody);
        const dtDynReal firstMass = first.Bodies()[firstBody].inertial.mass;
        const dtDynReal secondMass = second.Bodies()[secondBody].inertial.mass;
        bool firstRejected = false;
        bool secondRejected = false;

        try
        {
            first.EditBody(firstBody, [&](dtDynBodySpec &firstDraft)
                           {
                firstDraft.inertial.mass = 11.0;
                try
                {
                    second.EditBody(secondBody, [&](dtDynBodySpec &secondDraft)
                                    {
                        secondDraft.inertial.mass = 13.0;
                        try
                        {
                            if (reverseOperands)
                            {
                                first = std::move(second);
                            }
                            else
                            {
                                second = std::move(first);
                            }
                        }
                        catch (const std::logic_error &)
                        {
                        }
                    });
                }
                catch (const std::logic_error &)
                {
                    secondRejected = true;
                } });
        }
        catch (const std::logic_error &)
        {
            firstRejected = true;
        }

        assert(firstRejected);
        assert(secondRejected);
        assertUnchanged(first, firstBody, firstMass, second, secondBody, secondMass);
    };

    expectSourceAbort(false);
    expectSourceAbort(true);
    expectDestinationAbort(false);
    expectDestinationAbort(true);
    expectBothAborted(false);
    expectBothAborted(true);
}

void TestSpecEditorsAbortBothReentrantSwapTransactions()
{
    const auto expectBothAborted = [](const bool reverseOperands)
    {
        dtDynSpec first;
        dtDynSpec second;
        const dtDynIndex firstBody = first.AddBody("first", 0);
        const dtDynIndex secondBody = second.AddBody("second", 0);
        const dtDynReal firstMass = first.Bodies()[firstBody].inertial.mass;
        const dtDynReal secondMass = second.Bodies()[secondBody].inertial.mass;
        bool firstRejected = false;
        bool secondRejected = false;

        try
        {
            first.EditBody(firstBody, [&](dtDynBodySpec &firstDraft)
                           {
                firstDraft.inertial.mass = 3.0;
                try
                {
                    second.EditBody(secondBody, [&](dtDynBodySpec &secondDraft)
                                    {
                        secondDraft.inertial.mass = 5.0;
                        try
                        {
                            if (reverseOperands)
                            {
                                swap(second, first);
                            }
                            else
                            {
                                swap(first, second);
                            }
                        }
                        catch (const std::logic_error &)
                        {
                        }
                    });
                }
                catch (const std::logic_error &)
                {
                    secondRejected = true;
                } });
        }
        catch (const std::logic_error &)
        {
            firstRejected = true;
        }

        assert(firstRejected);
        assert(secondRejected);
        assert(first.Bodies()[firstBody].inertial.mass == firstMass);
        assert(second.Bodies()[secondBody].inertial.mass == secondMass);
    };

    expectBothAborted(false);
    expectBothAborted(true);
}

void TestSpecNameLookupReflectsEditorMutations()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("body", 0);
    const dtDynIndex joint = spec.AddJoint(body, "joint", dtDynJointType::Hinge);
    const dtDynIndex site = spec.AddSite(body, "site");
    const dtDynIndex actuator = spec.AddActuator(joint, "actuator");

    assert(spec.BodyId("body") == body);
    assert(spec.JointId("joint") == joint);
    assert(spec.SiteId("site") == site);
    assert(spec.ActuatorId("actuator") == actuator);
    assert(spec.BodyId("missing") == DTDYN_INVALID_INDEX);
    assert(spec.JointId("missing") == DTDYN_INVALID_INDEX);
    assert(spec.SiteId("missing") == DTDYN_INVALID_INDEX);
    assert(spec.ActuatorId("missing") == DTDYN_INVALID_INDEX);

    spec.EditBody(body, [&](dtDynBodySpec &draft)
                  { draft.name = "renamed_body"; });
    spec.EditJoint(joint, [&](dtDynJointSpec &draft)
                   { draft.name = "renamed_joint"; });
    spec.EditSite(site, [&](dtDynSiteSpec &draft)
                  { draft.name = "renamed_site"; });
    spec.EditActuator(actuator, [&](dtDynActuatorSpec &draft)
                      { draft.name = "renamed_actuator"; });
    assert(spec.BodyId("body") == DTDYN_INVALID_INDEX);
    assert(spec.JointId("joint") == DTDYN_INVALID_INDEX);
    assert(spec.SiteId("site") == DTDYN_INVALID_INDEX);
    assert(spec.ActuatorId("actuator") == DTDYN_INVALID_INDEX);
    assert(spec.BodyId("renamed_body") == body);
    assert(spec.JointId("renamed_joint") == joint);
    assert(spec.SiteId("renamed_site") == site);
    assert(spec.ActuatorId("renamed_actuator") == actuator);
}

void TestModelActuatorNameLookupUsesRebuiltCache()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("body", 0);
    const dtDynIndex joint = spec.AddJoint(body, "joint", dtDynJointType::Hinge);
    const dtDynIndex actuator = spec.AddActuator(joint, "actuator");
    dtDynModel model = CompileSpecChecked(spec);

    assert(model.ActuatorId("actuator") == actuator);
    assert(model.ActuatorId("missing") == DTDYN_INVALID_INDEX);

    const dtDynModel copied = model;
    assert(copied.ActuatorId("actuator") == actuator);

    model.actuator_names[actuator] = "renamed_actuator";
    assert(model.ActuatorId("actuator") == actuator);
    assert(model.ActuatorId("renamed_actuator") == DTDYN_INVALID_INDEX);
    model.RebuildNameMaps();
    assert(model.ActuatorId("actuator") == DTDYN_INVALID_INDEX);
    assert(model.ActuatorId("renamed_actuator") == actuator);
}

void TestSpecXmlReportsTypedNumericErrors()
{
    const auto expectError = [](const std::string &value,
                                const dtDynXmlErrorCode expectedCode,
                                const std::string &anchor)
    {
        const std::string xml =
            "<mujoco><worldbody><body name=\"base\">"
            "<inertial mass=\"" +
            value + "\"/>"
                    "</body></worldbody></mujoco>";

        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == expectedCode &&
                       error.SourcePath().empty() &&
                       error.Offset() == xml.find(anchor);
        }
        assert(rejected);
    };

    expectError("not-a-number", dtDynXmlErrorCode::InvalidNumber, "not-a-number");
    expectError("1e99999", dtDynXmlErrorCode::InvalidNumber, "1e99999");
    expectError("1junk", dtDynXmlErrorCode::InvalidNumber, "junk");
    expectError("NaN", dtDynXmlErrorCode::NonFiniteValue, "NaN");
    expectError("Inf", dtDynXmlErrorCode::NonFiniteValue, "Inf");
}

void TestSpecXmlPreservesNumericWhitespaceAndFirstFailure()
{
    const std::string valid =
        "<mujoco><worldbody><body name=\"base\"><inertial mass=\" \t 2.5 \n \"/>"
        "</body></worldbody></mujoco>";
    const dtDynSpec decoded = dtDynSpec::DecodeXml(valid);
    assert(decoded.Bodies().size() == 2);
    assert(std::fabs(decoded.Bodies()[1].inertial.mass - 2.5) < 1.0e-12);

    const std::string trailingJunk =
        "<mujoco><worldbody><body name=\"base\"><inertial mass=\" \t 1junk \n \"/>"
        "</body></worldbody></mujoco>";
    bool junkRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(trailingJunk);
    }
    catch (const dtDynXmlError &error)
    {
        junkRejected = error.Code() == dtDynXmlErrorCode::InvalidNumber &&
                       error.Offset() == trailingJunk.find("junk");
    }
    assert(junkRejected);

    const std::string whitespaceOnly =
        "<mujoco><worldbody><body name=\"base\"><inertial mass=\" \t \n \"/>"
        "</body></worldbody></mujoco>";
    const std::size_t whitespaceOffset =
        whitespaceOnly.find("mass=\"") + std::string("mass=\"").size();
    bool whitespaceRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(whitespaceOnly);
    }
    catch (const dtDynXmlError &error)
    {
        whitespaceRejected = error.Code() == dtDynXmlErrorCode::InvalidNumber &&
                             error.Offset() == whitespaceOffset;
    }
    assert(whitespaceRejected);
}

void TestSpecXmlRejectsWrongVectorArity()
{
    const auto expectInvalidArity = [](const std::string &value, const std::string &anchor)
    {
        const std::string xml =
            "<mujoco><worldbody><body name=\"base\" pos=\"" + value +
            "\"/></worldbody></mujoco>";
        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == dtDynXmlErrorCode::InvalidVectorArity &&
                       error.Offset() == xml.find(anchor);
        }
        assert(rejected);
    };

    expectInvalidArity("1 2", "1 2");
    expectInvalidArity("1 2 3 4", "4");

    const auto expectInvalidArityXml = [](const std::string &xml, const std::string &anchor)
    {
        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == dtDynXmlErrorCode::InvalidVectorArity &&
                       error.Offset() == xml.find(anchor);
        }
        assert(rejected);
    };

    expectInvalidArityXml(
        "<mujoco><worldbody><body name=\"base\" quat=\"1 0 0\"/>"
        "</worldbody></mujoco>",
        "1 0 0");
    expectInvalidArityXml(
        "<mujoco><worldbody><body name=\"base\"><joint name=\"hinge\" range=\"0\"/>"
        "</body></worldbody></mujoco>",
        "0");
}

void TestSpecXmlPreservesVectorWhitespaceAndArityOffset()
{
    const std::string valid =
        "<mujoco><worldbody><body name=\"base\" pos=\" \t 1 \n 2  3 \n \"/>"
        "</worldbody></mujoco>";
    const dtDynSpec decoded = dtDynSpec::DecodeXml(valid);
    assert(decoded.Bodies().size() == 2);
    assert(std::fabs(decoded.Bodies()[1].pos(0) - 1.0) < 1.0e-12);
    assert(std::fabs(decoded.Bodies()[1].pos(1) - 2.0) < 1.0e-12);
    assert(std::fabs(decoded.Bodies()[1].pos(2) - 3.0) < 1.0e-12);

    const std::string tooMany =
        "<mujoco><worldbody><body name=\"base\" pos=\" \t 1 \n 2  3  4 \n \"/>"
        "</worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(tooMany);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::InvalidVectorArity &&
                   error.Offset() == tooMany.find("4");
    }
    assert(rejected);
}

void TestSpecXmlRejectsMalformedAttributes()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\">"
        "<inertial mass=1/>"
        "</body></worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.find("mass");
    }
    assert(rejected);
}

static void ExpectMalformedXmlAttributeError(const std::string &xml,
                                             const std::string &anchor,
                                             const std::string &requiredDiagnostic)
{
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        assert(error.Code() == dtDynXmlErrorCode::MalformedSyntax);
        assert(error.Offset() == xml.find(anchor));
        assert(std::string(error.what()).find(requiredDiagnostic) != std::string::npos);
        rejected = true;
    }
    assert(rejected);
}

void TestSpecXmlPreservesAttributeWhitespaceAndFirstFailure()
{
    const std::string valid =
        "<mujoco><worldbody><body name = \"base\" pos = \"1 0 0\" />"
        "</worldbody></mujoco>";
    const dtDynSpec decoded = dtDynSpec::DecodeXml(valid);
    assert(decoded.Bodies().size() == 2);
    assert(decoded.BodyId("base") >= 0);
    assert(std::fabs(decoded.Bodies()[1].pos(0) - 1.0) < 1.0e-12);

    ExpectMalformedXmlAttributeError(
        "<mujoco><worldbody><body name \"base\"/></worldbody></mujoco>",
        "name",
        "malformed XML attribute");
    ExpectMalformedXmlAttributeError(
        "<mujoco><worldbody><body name=base/></worldbody></mujoco>",
        "name",
        "XML attribute value must be quoted");
}

void TestSpecXmlRejectsMisnestedSupportedTags()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\"></worldbody></body></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.find("</worldbody>");
    }
    assert(rejected);
}

void TestSpecXmlPreservesClosingElementFailureOrder()
{
    const std::string selfClosing = "<mujoco><worldbody></worldbody/></mujoco>";
    bool selfClosingRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(selfClosing);
    }
    catch (const dtDynXmlError &error)
    {
        selfClosingRejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax;
        if (selfClosingRejected)
        {
            selfClosingRejected = error.Offset() == selfClosing.find("</worldbody/>");
        }
        if (selfClosingRejected)
        {
            selfClosingRejected = std::string(error.what()).find("misnested closing element </worldbody>") != std::string::npos;
        }
    }
    assert(selfClosingRejected);

    const std::string emptyStack = "</mujoco>";
    bool emptyStackRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(emptyStack);
    }
    catch (const dtDynXmlError &error)
    {
        emptyStackRejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax;
        if (emptyStackRejected)
        {
            emptyStackRejected = error.Offset() == emptyStack.find("</mujoco>");
        }
        if (emptyStackRejected)
        {
            emptyStackRejected = std::string(error.what()).find("misnested closing element </mujoco>") != std::string::npos;
        }
    }
    assert(emptyStackRejected);

    const std::string mismatched =
        "<mujoco><worldbody><body name=\"base\"></worldbody></body></mujoco>";
    bool mismatchedRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(mismatched);
    }
    catch (const dtDynXmlError &error)
    {
        mismatchedRejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax;
        if (mismatchedRejected)
        {
            mismatchedRejected = error.Offset() == mismatched.find("</worldbody>");
        }
        if (mismatchedRejected)
        {
            mismatchedRejected = std::string(error.what()).find("misnested closing element </worldbody>") != std::string::npos;
        }
    }
    assert(mismatchedRejected);
}

void TestSpecXmlRejectsDuplicateObjectNames()
{
    const auto expectDuplicate = [](const std::string &xml)
    {
        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == dtDynXmlErrorCode::DuplicateName &&
                       error.Offset() == xml.rfind("duplicate");
        }
        assert(rejected);
    };

    expectDuplicate(
        "<mujoco><worldbody><body name=\"duplicate\"/><body name=\"duplicate\"/>"
        "</worldbody></mujoco>");
    expectDuplicate(
        "<mujoco><worldbody><body name=\"base\"><joint name=\"duplicate\"/>"
        "<joint name=\"duplicate\"/></body></worldbody></mujoco>");
    expectDuplicate(
        "<mujoco><worldbody><body name=\"base\"><site name=\"duplicate\"/>"
        "<site name=\"duplicate\"/></body></worldbody></mujoco>");
    expectDuplicate(
        "<mujoco><worldbody><body name=\"base\"><joint name=\"j1\"/>"
        "<joint name=\"j2\"/></body></worldbody><actuator>"
        "<motor name=\"duplicate\" joint=\"j1\"/>"
        "<motor name=\"duplicate\" joint=\"j2\"/></actuator></mujoco>");
}

void TestSpecXmlRejectsUnknownReferencesWithTypedError()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\"/></worldbody>"
        "<actuator><motor name=\"motor\" joint=\"missing_joint\"/></actuator>"
        "</mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::InvalidReference &&
                   error.Offset() == xml.find("missing_joint");
    }
    assert(rejected);
}

void TestSpecXmlRejectsInvalidJointRange()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\">"
        "<joint name=\"hinge\" range=\"bad 1\"/>"
        "</body></worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::InvalidNumber &&
                   error.Offset() == xml.find("bad");
    }
    assert(rejected);
}

void TestSpecXmlRejectsUnsupportedSemanticValues()
{
    const auto expectUnsupported = [](const std::string &xml, const std::string &value)
    {
        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == dtDynXmlErrorCode::UnsupportedElement &&
                       error.Offset() == xml.find(value);
        }
        assert(rejected);
    };

    expectUnsupported(
        "<mujoco><worldbody><body name=\"base\"><joint name=\"joint\" type=\"planar\"/>"
        "</body></worldbody></mujoco>",
        "planar");
    expectUnsupported(
        "<mujoco><compiler angle=\"gradian\"/><worldbody/></mujoco>",
        "gradian");
}

void TestSpecXmlRejectsInvalidBooleanValue()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\">"
        "<joint name=\"joint\" limited=\"maybe\"/>"
        "</body></worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.find("maybe");
    }
    assert(rejected);
}

void TestSpecXmlPreservesLimitedBooleanGrammar()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\">"
        "<joint name=\"word_true\" limited=\"true\"/>"
        "<joint name=\"numeric_true\" limited=\"1\"/>"
        "<joint name=\"word_false\" limited=\"false\"/>"
        "<joint name=\"numeric_false\" limited=\"0\"/>"
        "</body></worldbody></mujoco>";
    const dtDynSpec decoded = dtDynSpec::DecodeXml(xml);
    assert(decoded.Joints().size() == 4);
    assert(decoded.Joints()[0].limited);
    assert(decoded.Joints()[1].limited);
    assert(!decoded.Joints()[2].limited);
    assert(!decoded.Joints()[3].limited);
}

void TestSpecXmlPreservesTagWhitespaceAndClosingSuffixOffset()
{
    const std::string whitespaceXml =
        "< \tmujoco>< \nworldbody>< \tbody name=\"base\"></ \tbody \n>"
        "</ \nworldbody ></ \tmujoco >";
    const dtDynSpec decoded = dtDynSpec::DecodeXml(whitespaceXml);
    assert(decoded.Bodies().size() == 2);
    assert(decoded.Bodies()[1].name == "base");

    const std::string invalidNameXml = "< \t9invalid/>";
    bool invalidNameRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(invalidNameXml);
    }
    catch (const dtDynXmlError &error)
    {
        invalidNameRejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax;
        if (invalidNameRejected)
        {
            invalidNameRejected = error.Offset() == invalidNameXml.find("9invalid");
        }
        if (invalidNameRejected)
        {
            invalidNameRejected = std::string(error.what()).find("invalid XML element name") !=
                                  std::string::npos;
        }
    }
    assert(invalidNameRejected);

    const std::string suffixXml =
        "<mujoco><worldbody><body name=\"base\"></body \ttrailing>"
        "</worldbody></mujoco>";
    bool suffixRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(suffixXml);
    }
    catch (const dtDynXmlError &error)
    {
        suffixRejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax;
        if (suffixRejected)
        {
            suffixRejected = error.Offset() == suffixXml.find("trailing");
        }
        if (suffixRejected)
        {
            suffixRejected = std::string(error.what()).find("closing XML tags cannot contain attributes or suffix text") !=
                             std::string::npos;
        }
    }
    assert(suffixRejected);
}

void TestSpecXmlRequiresSingleRootElement()
{
    const auto expectMalformed = [](const std::string &xml, const std::size_t expectedOffset)
    {
        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                       error.Offset() == expectedOffset;
        }
        assert(rejected);
    };

    expectMalformed("", 0);
    const std::string multipleRoots = "<mujoco/><mujoco/>";
    expectMalformed(multipleRoots, multipleRoots.rfind("<mujoco"));
    const std::string duplicateWorldbody = "<mujoco><worldbody/><worldbody/></mujoco>";
    expectMalformed(duplicateWorldbody, duplicateWorldbody.rfind("<worldbody"));
}

void TestSpecXmlRejectsSupportedElementInWrongContext()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\">"
        "<motor name=\"motor\" joint=\"missing\"/>"
        "</body></worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.find("<motor");
    }
    assert(rejected);
}

void TestSpecXmlAcceptsEverySupportedElementInItsDocumentedParent()
{
    const std::string xml =
        "<mujoco><compiler angle=\"radian\"/><option/><worldbody>"
        "<body name=\"base\"><body name=\"hinged\">"
        "<inertial mass=\"1\" diaginertia=\"1 1 1\"/>"
        "<joint name=\"hinge\"/><site name=\"tool\"/>"
        "</body><body name=\"floating\"><freejoint name=\"free\"/>"
        "</body></body></worldbody>"
        "<actuator><motor name=\"drive\" joint=\"hinge\"/></actuator>"
        "</mujoco>";

    const dtDynSpec spec = dtDynSpec::DecodeXml(xml);
    assert(spec.Bodies().size() == 4);
    assert(spec.Joints().size() == 2);
    assert(spec.Sites().size() == 1);
    assert(spec.Actuators().size() == 1);
    assert(spec.BodyId("base") >= 0);
    assert(spec.BodyId("hinged") >= 0);
    assert(spec.BodyId("floating") >= 0);
    assert(spec.JointId("hinge") >= 0);
    assert(spec.JointId("free") >= 0);
    assert(spec.SiteId("tool") >= 0);
    assert(spec.ActuatorId("drive") >= 0);
}

void TestSpecXmlPreservesWhitespaceAndTagNameGrammar()
{
    const std::string whitespace =
        "<  mujoco  ><  worldbody  ><  body name=\"base\"  /  >"
        "</  worldbody  ></  mujoco  >";
    const dtDynSpec decoded = dtDynSpec::DecodeXml(whitespace);
    assert(decoded.Bodies().size() == 2);
    assert(decoded.BodyId("base") >= 0);

    const auto expectError = [](const std::string &xml,
                                const dtDynXmlErrorCode code,
                                const std::string &anchor)
    {
        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == code && error.Offset() == xml.find(anchor);
        }
        assert(rejected);
    };

    expectError("<mujoco><1invalid/></mujoco>", dtDynXmlErrorCode::MalformedSyntax, "1invalid");
    expectError("<mujoco><invalid!/></mujoco>", dtDynXmlErrorCode::MalformedSyntax, "invalid!");
    expectError("<mujoco><_valid-1:name.part/></mujoco>",
                dtDynXmlErrorCode::UnsupportedElement,
                "<_valid-1:name.part");
}

void TestSpecXmlScannerIsQuoteAwareAndFailClosed()
{
    const std::string valid =
        "<?xml version=\"1.0\"?><mujoco><!-- <equality> is comment text -->"
        "<worldbody><body name=\"base>link\"/></worldbody></mujoco>";
    const dtDynSpec decoded = dtDynSpec::DecodeXml(valid);
    assert(decoded.Bodies().size() == 2);
    assert(decoded.Bodies()[1].name == "base>link");

    const auto expectError = [](const std::string &xml,
                                const dtDynXmlErrorCode code,
                                const std::size_t offset)
    {
        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == code && error.Offset() == offset;
        }
        assert(rejected);
    };

    const std::string unknown = "<mujoco><unknown/></mujoco>";
    expectError(unknown, dtDynXmlErrorCode::UnsupportedElement, unknown.find("<unknown"));

    const std::string text = "<mujoco>unexpected</mujoco>";
    expectError(text, dtDynXmlErrorCode::MalformedSyntax, text.find("unexpected"));

    const std::string unclosed = "<mujoco><worldbody><body name=\"base\">";
    expectError(unclosed, dtDynXmlErrorCode::MalformedSyntax, unclosed.find("<body"));
}

void TestSpecXmlRunsSetupValidationAfterParsing()
{
    const std::string invalid =
        "<mujoco><worldbody><body name=\"base\">"
        "<inertial mass=\"-1\"/>"
        "</body></worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(invalid);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::InvalidReference &&
                   error.Offset() == invalid.find("<body");
    }
    assert(rejected);

    const std::string roundedAxis =
        "<mujoco><worldbody><body name=\"base\">"
        "<joint name=\"joint\" axis=\"0.70710678 0.70710678 0\"/>"
        "</body></worldbody></mujoco>";
    const dtDynSpec valid = dtDynSpec::DecodeXml(roundedAxis);
    assert(valid.Joints().size() == 1);
}

void TestSpecXmlRejectsClosingTagAttributes()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\"></body bogus=\"x\">"
        "</worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.find("bogus");
    }
    assert(rejected);
}

void TestSpecXmlRejectsMalformedSelfClosingSuffix()
{
    const std::string xml =
        "<mujoco><worldbody><body / bogus=\"x\"></worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.find("/");
    }
    assert(rejected);
}

void TestSpecXmlPreservesSelfClosingTagDetection()
{
    const dtDynSpec rootSelfClosing = dtDynSpec::DecodeXml("<mujoco/>");
    assert(rootSelfClosing.Bodies().size() == 1);

    const dtDynSpec bodySelfClosing = dtDynSpec::DecodeXml(
        "<mujoco><worldbody><body name=\"base\"/></worldbody></mujoco>");
    assert(bodySelfClosing.Bodies().size() == 2);
    assert(bodySelfClosing.Bodies()[1].name == "base");

    const dtDynSpec explicitClosing = dtDynSpec::DecodeXml(
        "<mujoco><worldbody><body name=\"base\"></body></worldbody></mujoco>");
    assert(explicitClosing.Bodies().size() == 2);
    assert(explicitClosing.Bodies()[1].name == "base");
}

void TestSpecXmlRejectsDuplicateDeclaration()
{
    const std::string xml = "<?xml version=\"1.0\"?><?xml version=\"1.0\"?><mujoco/>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.rfind("<?xml");
    }
    assert(rejected);
}

void TestSpecXmlRejectsRawLessThanInAttributeValue()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base<link\"/></worldbody></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.find("<link");
    }
    assert(rejected);
}

void TestSpecXmlRejectsMalformedDeclarationGrammar()
{
    const auto expectMalformed = [](const std::string &declaration, const std::size_t offset)
    {
        const std::string xml = declaration + "<mujoco/>";
        bool rejected = false;
        try
        {
            (void)dtDynSpec::DecodeXml(xml);
        }
        catch (const dtDynXmlError &error)
        {
            rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                       error.Offset() == offset;
        }
        assert(rejected);
    };

    expectMalformed("<?foo?>", 0);
    expectMalformed("<?xml?>", 0);
    const std::string unquoted = "<?xml version=1.0?>";
    expectMalformed(unquoted, unquoted.find("version"));
}

void TestSpecXmlPreservesDeclarationVersionAndAttributeGrammar()
{
    const dtDynSpec version10 = dtDynSpec::DecodeXml(
        "<?xml version=\"1.0\" encoding=\"utf-8\" standalone=\"yes\"?><mujoco/>");
    assert(version10.Bodies().size() == 1);
    const dtDynSpec version11 = dtDynSpec::DecodeXml(
        "<?xml version=\"1.1\" standalone=\"no\" encoding=\"utf-8\"?><mujoco/>");
    assert(version11.Bodies().size() == 1);

    const std::string unsupportedAttribute = "<?xml version=\"1.0\" invalid=\"x\"?><mujoco/>";
    bool unsupportedRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(unsupportedAttribute);
    }
    catch (const dtDynXmlError &error)
    {
        unsupportedRejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax;
        if (unsupportedRejected)
        {
            unsupportedRejected = error.Offset() == unsupportedAttribute.rfind("x");
        }
        if (unsupportedRejected)
        {
            unsupportedRejected = std::string(error.what()).find("unsupported XML declaration attribute") != std::string::npos;
        }
    }
    assert(unsupportedRejected);

    const std::string invalidStandalone =
        "<?xml version=\"1.0\" standalone=\"maybe\"?><mujoco/>";
    bool standaloneRejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(invalidStandalone);
    }
    catch (const dtDynXmlError &error)
    {
        standaloneRejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax;
        if (standaloneRejected)
        {
            standaloneRejected = error.Offset() == invalidStandalone.find("maybe");
        }
        if (standaloneRejected)
        {
            standaloneRejected = std::string(error.what()).find("invalid XML standalone declaration value") != std::string::npos;
        }
    }
    assert(standaloneRejected);
}

void TestSpecXmlRejectsDeclarationAfterLeadingWhitespace()
{
    const std::string xml = "\n<?xml version=\"1.0\"?><mujoco/>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.Offset() == xml.find("<?xml");
    }
    assert(rejected);
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

void TestSpecXmlLoadsFromFile()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"loaded_base\">"
        "<joint name=\"loaded_joint\" type=\"hinge\"/>"
        "</body></worldbody></mujoco>";
    const std::filesystem::path path = UniqueTempPath("dtdynamics_test_load_spec.xml");
    {
        std::ofstream output(path, std::ios::binary);
        assert(output);
        output << xml;
        assert(output.good());
    }

    const dtDynSpec spec = dtDynSpec::LoadXmlFile(path.string());
    assert(spec.Bodies().size() == 2);
    assert(spec.Bodies()[1].name == "loaded_base");
    assert(spec.Joints().size() == 1);
    assert(spec.Joints()[0].name == "loaded_joint");

    std::filesystem::remove(path);
}

void TestSpecXmlLoadsAcrossBufferBoundaries()
{
    const auto xmlOfSize = [](const std::size_t size)
    {
        const std::string prefix = "<mujoco><!--";
        const std::string suffix = "--><worldbody/></mujoco>";
        assert(size >= prefix.size() + suffix.size());
        return prefix + std::string(size - prefix.size() - suffix.size(), 'x') + suffix;
    };

    for (const std::size_t size : {std::size_t(4095), std::size_t(4096), std::size_t(4097)})
    {
        const std::filesystem::path path = UniqueTempPath("dtdynamics_buffer_boundary.xml");
        const std::string xml = xmlOfSize(size);
        {
            std::ofstream output(path, std::ios::binary);
            assert(output);
            output.write(xml.data(), static_cast<std::streamsize>(xml.size()));
            assert(output.good());
        }
        const dtDynSpec loaded = dtDynSpec::LoadXmlFile(path.string());
        std::filesystem::remove(path);
        assert(loaded.Bodies().size() == 1);
        assert(loaded.Bodies()[0].name == "world");
    }
}

void TestSpecXmlLoadReportsTypedErrorPath()
{
    const std::string xml =
        "<mujoco><worldbody><body name=\"base\">"
        "<inertial mass=\"bad\"/>"
        "</body></worldbody></mujoco>";
    const std::filesystem::path path = UniqueTempPath("dtdynamics_typed_error.xml");
    {
        std::ofstream output(path, std::ios::binary);
        assert(output);
        output << xml;
        assert(output.good());
    }

    bool rejected = false;
    try
    {
        (void)dtDynSpec::LoadXmlFile(path.string());
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::InvalidNumber &&
                   error.SourcePath() == path.string() &&
                   error.Offset() == xml.find("bad");
    }
    std::filesystem::remove(path);
    assert(rejected);
}

void TestSpecXmlLoadReportsMissingFile()
{
    const std::filesystem::path path = UniqueTempPath("dtdynamics_missing_spec.xml");

    bool rejected = false;
    try
    {
        (void)dtDynSpec::LoadXmlFile(path.string());
    }
    catch (const std::runtime_error &error)
    {
        const std::string message = error.what();
        rejected = message.find("failed to open XML file") != std::string::npos &&
                   message.find(path.string()) != std::string::npos;
    }
    assert(rejected);
}

void TestSpecXmlLoadReportsMalformedFilePath()
{
    const std::string xml = "<mujoco><worldbody";
    const std::filesystem::path path = UniqueTempPath("dtdynamics_malformed_spec.xml");
    {
        std::ofstream output(path, std::ios::binary);
        assert(output);
        output << xml;
        assert(output.good());
    }

    bool rejected = false;
    try
    {
        (void)dtDynSpec::LoadXmlFile(path.string());
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::MalformedSyntax &&
                   error.SourcePath() == path.string() &&
                   error.Offset() == xml.find("<worldbody");
    }
    std::filesystem::remove(path);
    assert(rejected);
}

void TestSpecXmlLoadRejectsDirectoryPath()
{
    const std::filesystem::path path = UniqueTempPath("dtdynamics_xml_directory");
    assert(std::filesystem::create_directory(path));

    bool rejected = false;
    try
    {
        (void)dtDynSpec::LoadXmlFile(path.string());
    }
    catch (const std::runtime_error &error)
    {
        const std::string message = error.what();
        const bool ioFailure = message.find("failed to open XML file") != std::string::npos ||
                               message.find("failed to read XML file") != std::string::npos;
        rejected = ioFailure && message.find(path.string()) != std::string::npos;
    }
    assert(std::filesystem::remove(path));
    assert(rejected);
}

void TestSpecXmlSavesToFile()
{
    const dtDynSpec spec = dtDynSpec::DecodeXml(
        "<mujoco><worldbody><body name=\"saved_base\">"
        "<joint name=\"saved_joint\" type=\"hinge\"/>"
        "</body></worldbody></mujoco>");
    const std::filesystem::path path = UniqueTempPath("dtdynamics_test_save_spec.xml");

    spec.SaveXmlFile(path.string());

    std::ifstream input(path, std::ios::binary);
    assert(input);
    std::ostringstream stream;
    stream << input.rdbuf();
    assert(!stream.fail());
    assert(stream.str() == spec.EncodeXml());

    const dtDynSpec loaded = dtDynSpec::LoadXmlFile(path.string());
    assert(loaded.Bodies().size() == spec.Bodies().size());
    assert(loaded.Joints().size() == spec.Joints().size());
    assert(loaded.Bodies()[1].name == "saved_base");
    assert(loaded.Joints()[0].name == "saved_joint");

    std::filesystem::remove(path);
}

void TestSpecXmlSaveReportsOpenFailure()
{
    dtDynSpec spec;
    const std::filesystem::path parent = UniqueTempPath("dtdynamics_missing_output_directory");
    const std::filesystem::path path = parent / "saved_spec.xml";
    assert(!std::filesystem::exists(parent));

    bool rejected = false;
    try
    {
        spec.SaveXmlFile(path.string());
    }
    catch (const std::runtime_error &error)
    {
        const std::string message = error.what();
        rejected = message.find("failed to open XML file for writing") != std::string::npos &&
                   message.find(path.string()) != std::string::npos;
    }
    assert(rejected);
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
    const std::string xml =
        "<mujoco><worldbody><body name=\"a\"/></worldbody>"
        "<equality><connect body1=\"a\" body2=\"world\"/></equality></mujoco>";
    bool rejected = false;
    try
    {
        (void)dtDynSpec::DecodeXml(xml);
    }
    catch (const dtDynXmlError &error)
    {
        rejected = error.Code() == dtDynXmlErrorCode::UnsupportedElement &&
                   error.Offset() == xml.find("<equality");
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
    spec.EditBody(static_cast<std::size_t>(link), [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);
    spec.AddSite(link, "tool");
    const auto actuator = spec.AddActuator(1, "hinge_motor");
    spec.EditActuator(static_cast<std::size_t>(actuator), [&](dtDynActuatorSpec &draft)
                      { draft.gear = 120.0; });

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
    assert(copied.jnt_specid.size() == compiled.jnt_specid.size());
    for (std::size_t jointId = 0; jointId < compiled.jnt_specid.size(); ++jointId)
    {
        assert(copied.jnt_specid[jointId] == compiled.jnt_specid[jointId]);
    }
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
    spec.EditBody(static_cast<std::size_t>(link), [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
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
    spec.EditBody(static_cast<std::size_t>(link), [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);

    dtDynModel model;
    CompileSpec(spec, model);

    assert(model.nbody == 2);
    assert(model.nv == 1);
    assert(model.BodyId("link") == link);
}

void TestUpdateSpecRejectsSameDofJointTypeChangeWithoutWriting()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("link", 0);
    const dtDynIndex joint = spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });

    dtDynModel model = CompileSpecChecked(spec);
    const dtDynIndex nq = model.nq;
    const dtDynIndex nv = model.nv;
    const dtDynReal mass = model.body_mass(body);
    const dtDynIndex jointType = model.jnt_type[0];

    spec.EditJoint(joint, [](dtDynJointSpec &draft)
                   { draft.type = dtDynJointType::Slide; });
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  { draft.inertial.mass = 7.0; });

    assert(!UpdateSpec(spec, model));
    assert(model.nq == nq);
    assert(model.nv == nv);
    assert(model.body_mass(body) == mass);
    assert(model.jnt_type[0] == jointType);
}

void TestUpdateSpecRejectsWorldParentMutationWithoutWriting()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("link", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });

    dtDynModel model = CompileSpecChecked(spec);
    const dtDynReal mass = model.body_mass(body);

    spec.EditBody(0, [](dtDynBodySpec &draft)
                  { draft.parent = 0; });
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  { draft.inertial.mass = 7.0; });

    assert(!UpdateSpec(spec, model));
    assert(model.body_mass(body) == mass);
}

void TestUpdateSpecRejectsChildListMutationWithoutWriting()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("link", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });

    dtDynModel model = CompileSpecChecked(spec);
    const dtDynReal mass = model.body_mass(body);

    spec.EditBody(0, [](dtDynBodySpec &draft)
                  { draft.firstChild = DTDYN_INVALID_INDEX; });
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  { draft.inertial.mass = 7.0; });

    assert(!UpdateSpec(spec, model));
    assert(model.body_mass(body) == mass);
}

void TestUpdateSpecRejectsSiteBacklinkMutationWithoutWriting()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("link", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    spec.AddSite(body, "tool");
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });

    dtDynModel model = CompileSpecChecked(spec);
    const dtDynReal mass = model.body_mass(body);

    spec.EditBody(body, [](dtDynBodySpec &draft)
                  {
                      draft.sites.clear();
                      draft.inertial.mass = 7.0; });

    assert(!UpdateSpec(spec, model));
    assert(model.body_mass(body) == mass);
}

void TestUpdateSpecRejectsEarlyStorageMismatchWithoutWriting()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("link", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  {
                      draft.pos(0) = 1.0;
                      draft.inertial.mass = 1.0; });

    dtDynModel model = CompileSpecChecked(spec);
    const auto position = model.body_pos[static_cast<std::size_t>(body)];
    const dtDynReal mass = model.body_mass(body);
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  {
                      draft.pos(0) = 7.0;
                      draft.inertial.mass = 7.0; });
    model.body_names.clear();

    assert(!UpdateSpec(spec, model));
    assert(model.body_pos[static_cast<std::size_t>(body)](0) == position(0));
    assert(model.body_mass(body) == mass);
}

void TestUpdateSpecRejectsLateStorageMismatchWithoutWriting()
{
    dtDynSpec spec;
    const dtDynIndex body = spec.AddBody("link", 0);
    spec.AddJoint(body, "hinge", dtDynJointType::Hinge);
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  {
                      draft.pos(0) = 1.0;
                      draft.inertial.mass = 1.0; });

    dtDynModel model = CompileSpecChecked(spec);
    assert(model.nroot > 0);
    const auto position = model.body_pos[static_cast<std::size_t>(body)];
    const dtDynReal mass = model.body_mass(body);
    spec.EditBody(body, [](dtDynBodySpec &draft)
                  {
                      draft.pos(0) = 7.0;
                      draft.inertial.mass = 7.0; });
    ResizeDenseVector(model.root_mass, model.nroot - 1);

    assert(!UpdateSpec(spec, model));
    assert(model.body_pos[static_cast<std::size_t>(body)](0) == position(0));
    assert(model.body_mass(body) == mass);
}

void TestUpdateSpecRejectsLateCompatibilityMetadataMutationsWithoutWriting()
{
    {
        dtDynSpec spec;
        const dtDynIndex body = spec.AddBody("joint_body", 0);
        const dtDynIndex joint = spec.AddJoint(body, "joint", dtDynJointType::Hinge);
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
                          draft.pos(0) = 1.0;
                          draft.inertial.mass = 1.0; });

        dtDynModel model = CompileSpecChecked(spec);
        const dtDynReal mass = model.body_mass(body);
        const auto position = model.body_pos[static_cast<std::size_t>(body)];
        const std::string *const jointNameStorage = model.joint_names.data();
        spec.EditJoint(joint, [](dtDynJointSpec &draft)
                       { draft.name = "renamed_joint"; });
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
                          draft.pos(0) = 7.0;
                          draft.inertial.mass = 7.0; });
        const std::size_t allocationsBeforeUpdate = g_totalTestAllocations;

        assert(!UpdateSpec(spec, model));
        assert(g_totalTestAllocations == allocationsBeforeUpdate);
        assert(model.joint_names.data() == jointNameStorage);
        assert(model.joint_names[static_cast<std::size_t>(joint)] == "joint");
        assert(model.JointId("joint") == joint);
        assert(model.JointId("renamed_joint") == DTDYN_INVALID_INDEX);
        assert(model.body_pos[static_cast<std::size_t>(body)](0) == position(0));
        assert(model.body_mass(body) == mass);
    }

    {
        dtDynSpec spec;
        const dtDynIndex body = spec.AddBody("site_body", 0);
        spec.AddJoint(body, "site_joint", dtDynJointType::Hinge);
        const dtDynIndex site = spec.AddSite(body, "site");
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
                          draft.pos(0) = 1.0;
                          draft.inertial.mass = 1.0; });

        dtDynModel model = CompileSpecChecked(spec);
        const dtDynReal mass = model.body_mass(body);
        const auto position = model.body_pos[static_cast<std::size_t>(body)];
        const std::string *const siteNameStorage = model.site_names.data();
        spec.EditSite(site, [](dtDynSiteSpec &draft)
                      { draft.name = "renamed_site"; });
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
                          draft.pos(0) = 7.0;
                          draft.inertial.mass = 7.0; });
        const std::size_t allocationsBeforeUpdate = g_totalTestAllocations;

        assert(!UpdateSpec(spec, model));
        assert(g_totalTestAllocations == allocationsBeforeUpdate);
        assert(model.site_names.data() == siteNameStorage);
        assert(model.site_names[static_cast<std::size_t>(site)] == "site");
        assert(model.SiteId("site") == site);
        assert(model.SiteId("renamed_site") == DTDYN_INVALID_INDEX);
        assert(model.body_pos[static_cast<std::size_t>(body)](0) == position(0));
        assert(model.body_mass(body) == mass);
    }

    {
        dtDynSpec spec;
        const dtDynIndex body = spec.AddBody("backlink_body", 0);
        spec.AddJoint(body, "backlink_joint", dtDynJointType::Hinge);
        const dtDynIndex site = spec.AddSite(body, "backlink_site");
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
                          draft.pos(0) = 1.0;
                          draft.inertial.mass = 1.0; });

        dtDynModel model = CompileSpecChecked(spec);
        const dtDynReal mass = model.body_mass(body);
        const auto position = model.body_pos[static_cast<std::size_t>(body)];
        const dtDynIndex *const siteOwnerStorage = model.site_bodyid.data();
        spec.EditBody(body, [site](dtDynBodySpec &draft)
                      {
                          draft.sites.push_back(site);
                          draft.pos(0) = 7.0;
                          draft.inertial.mass = 7.0; });
        const std::size_t allocationsBeforeUpdate = g_totalTestAllocations;

        assert(!UpdateSpec(spec, model));
        assert(g_totalTestAllocations == allocationsBeforeUpdate);
        assert(model.site_bodyid.data() == siteOwnerStorage);
        assert(model.site_bodyid[static_cast<std::size_t>(site)] == body);
        assert(model.SiteId("backlink_site") == site);
        assert(model.body_pos[static_cast<std::size_t>(body)](0) == position(0));
        assert(model.body_mass(body) == mass);
    }

    {
        dtDynSpec spec;
        const dtDynIndex body = spec.AddBody("child_body", 0);
        spec.AddJoint(body, "child_joint", dtDynJointType::Hinge);
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
                          draft.pos(0) = 1.0;
                          draft.inertial.mass = 1.0; });

        dtDynModel model = CompileSpecChecked(spec);
        const dtDynReal mass = model.body_mass(body);
        const auto position = model.body_pos[static_cast<std::size_t>(body)];
        const dtDynIndex *const parentStorage = model.body_parentid.data();
        spec.EditBody(body, [body](dtDynBodySpec &draft)
                      {
                          draft.nextSibling = body;
                          draft.pos(0) = 7.0;
                          draft.inertial.mass = 7.0; });
        const std::size_t allocationsBeforeUpdate = g_totalTestAllocations;

        assert(!UpdateSpec(spec, model));
        assert(g_totalTestAllocations == allocationsBeforeUpdate);
        assert(model.body_parentid.data() == parentStorage);
        assert(model.body_parentid[static_cast<std::size_t>(body)] == 0);
        assert(model.BodyId("child_body") == body);
        assert(model.body_pos[static_cast<std::size_t>(body)](0) == position(0));
        assert(model.body_mass(body) == mass);
    }

    {
        dtDynSpec spec;
        const dtDynIndex body = spec.AddBody("actuator_body", 0);
        const dtDynIndex joint = spec.AddJoint(body, "actuator_joint", dtDynJointType::Hinge);
        const dtDynIndex actuator = spec.AddActuator(joint, "actuator");
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
                          draft.pos(0) = 1.0;
                          draft.inertial.mass = 1.0; });

        dtDynModel model = CompileSpecChecked(spec);
        const dtDynReal mass = model.body_mass(body);
        const auto position = model.body_pos[static_cast<std::size_t>(body)];
        const std::string *const actuatorNameStorage = model.actuator_names.data();
        spec.EditActuator(actuator, [](dtDynActuatorSpec &draft)
                          { draft.name = "renamed_actuator"; });
        spec.EditBody(body, [](dtDynBodySpec &draft)
                      {
                          draft.pos(0) = 7.0;
                          draft.inertial.mass = 7.0; });
        const std::size_t allocationsBeforeUpdate = g_totalTestAllocations;

        assert(!UpdateSpec(spec, model));
        assert(g_totalTestAllocations == allocationsBeforeUpdate);
        assert(model.actuator_names.data() == actuatorNameStorage);
        assert(model.actuator_names[static_cast<std::size_t>(actuator)] == "actuator");
        assert(model.ActuatorId("actuator") == actuator);
        assert(model.ActuatorId("renamed_actuator") == DTDYN_INVALID_INDEX);
        assert(model.body_pos[static_cast<std::size_t>(body)](0) == position(0));
        assert(model.body_mass(body) == mass);
    }
}

void TestUpdateSpecMatchesPhysicalRecompile()
{
    dtDynSpec original;
    const dtDynIndex root = original.AddBody("root", 0);
    const dtDynIndex child = original.AddBody("child", root);
    const dtDynIndex childJoint = original.AddJoint(child, "child_slide", dtDynJointType::Slide);
    const dtDynIndex rootJoint = original.AddJoint(root, "root_hinge", dtDynJointType::Hinge);
    const dtDynIndex site = original.AddSite(child, "tool");
    const dtDynIndex actuator = original.AddActuator(childJoint, "child_motor");

    original.EditBody(root, [](dtDynBodySpec &draft)
                      {
                          draft.inertial.mass = 2.0;
                          draft.inertial.inertia(0) = 0.2;
                          draft.inertial.inertia(1) = 0.3;
                          draft.inertial.inertia(2) = 0.4; });
    original.EditBody(child, [](dtDynBodySpec &draft)
                      { draft.inertial.mass = 0.0; });

    dtDynModel model = CompileSpecChecked(original);
    assert(model.jnt_type[0] == static_cast<dtDynIndex>(dtDynJointType::Hinge));
    assert(model.jnt_type[1] == static_cast<dtDynIndex>(dtDynJointType::Slide));
    assert(model.jnt_specid[0] == rootJoint);
    assert(model.jnt_specid[1] == childJoint);
    assert(model.actuator_jointid[actuator] == 1);

    const dtDynIndex *const bodyParentStorage = model.body_parentid.data();
    const dtDynIndex *const jointTypeStorage = model.jnt_type.data();
    const dtDynIndex *const jointSpecStorage = model.jnt_specid.data();
    const dt::Math::Vector3<dtDynReal> *const bodyPosStorage = model.body_pos.data();
    const dt::Math::Vector3<dtDynReal> *const jointPosStorage = model.jnt_pos.data();
    const dt::Math::Vector3<dtDynReal> *const sitePosStorage = model.site_pos.data();
    const dtDynIndex *const actuatorJointStorage = model.actuator_jointid.data();
    const dtDynIndex *const rootBodyStorage = model.root_bodyid.data();

    dtDynSpec updated = original;
    dt::Math::Vector3<dtDynReal> xAxis;
    dt::Math::Vector3<dtDynReal> yAxis;
    dt::Math::Vector3<dtDynReal> zAxis;
    xAxis.SetZero();
    yAxis.SetZero();
    zAxis.SetZero();
    xAxis(0) = 1.0;
    yAxis(1) = 1.0;
    zAxis(2) = 1.0;

    updated.EditBody(root, [&](dtDynBodySpec &draft)
                     {
                         draft.pos(0) = 0.2;
                         draft.pos(1) = -0.1;
                         draft.quat = math::AxisAngleQuat(zAxis, 0.4);
                         draft.inertial.mass = 2.5;
                         draft.inertial.pos(0) = 0.1;
                         draft.inertial.pos(1) = 0.2;
                         draft.inertial.quat = math::AxisAngleQuat(xAxis, 0.3);
                         draft.inertial.inertia(0) = 0.25;
                         draft.inertial.inertia(1) = 0.35;
                         draft.inertial.inertia(2) = 0.45; });
    updated.EditBody(child, [&](dtDynBodySpec &draft)
                     {
                         draft.pos(0) = 0.6;
                         draft.pos(2) = -0.2;
                         draft.quat = math::AxisAngleQuat(yAxis, -0.2);
                         draft.inertial.mass = 1.75;
                         draft.inertial.pos(0) = 0.3;
                         draft.inertial.quat = math::AxisAngleQuat(zAxis, -0.5);
                         draft.inertial.inertia(0) = 0.12;
                         draft.inertial.inertia(1) = 0.18;
                         draft.inertial.inertia(2) = 0.24; });
    updated.EditJoint(rootJoint, [&](dtDynJointSpec &draft)
                      {
                            draft.pos(0) = 0.1;
                            draft.axis = yAxis;
                            draft.limited = true;
                            draft.rangeMin = -0.7;
                            draft.rangeMax = 0.8;
                            draft.armature = 0.11; });
    updated.EditJoint(childJoint, [&](dtDynJointSpec &draft)
                      {
                            draft.pos(2) = -0.3;
                            draft.axis = xAxis;
                            draft.limited = true;
                            draft.rangeMin = -0.2;
                            draft.rangeMax = 0.4;
                            draft.armature = 0.23; });
    updated.EditSite(site, [&](dtDynSiteSpec &draft)
                     {
                         draft.pos(0) = 0.4;
                         draft.pos(2) = 0.1;
                         draft.quat = math::AxisAngleQuat(yAxis, 0.6); });
    updated.EditActuator(actuator, [](dtDynActuatorSpec &draft)
                         { draft.gear = 42.0; });

    const dtDynModel expected = CompileSpecChecked(updated);
    const std::size_t allocationsBeforeUpdate = g_totalTestAllocations;
    assert(UpdateSpec(updated, model));
    assert(g_totalTestAllocations == allocationsBeforeUpdate);

    assert(model.body_parentid.data() == bodyParentStorage);
    assert(model.jnt_type.data() == jointTypeStorage);
    assert(model.jnt_specid.data() == jointSpecStorage);
    assert(model.body_pos.data() == bodyPosStorage);
    assert(model.jnt_pos.data() == jointPosStorage);
    assert(model.site_pos.data() == sitePosStorage);
    assert(model.actuator_jointid.data() == actuatorJointStorage);
    assert(model.root_bodyid.data() == rootBodyStorage);

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        const std::size_t index = static_cast<std::size_t>(bodyId);
        assert(IsDtMathNear(model.body_mass(bodyId), expected.body_mass(bodyId)));
        assert(IsDtMathNear(model.body_subtreemass(bodyId), expected.body_subtreemass(bodyId)));
        for (dtDynIndex axis = 0; axis < 3; ++axis)
        {
            assert(IsDtMathNear(model.body_pos[index](axis), expected.body_pos[index](axis)));
            assert(IsDtMathNear(model.body_ipos[index](axis), expected.body_ipos[index](axis)));
            assert(IsDtMathNear(model.body_inertia[index](axis), expected.body_inertia[index](axis)));
            for (dtDynIndex column = 0; column < 3; ++column)
            {
                assert(IsDtMathNear(model.body_mat[index](axis, column),
                                    expected.body_mat[index](axis, column)));
                assert(IsDtMathNear(model.body_imat[index](axis, column),
                                    expected.body_imat[index](axis, column)));
            }
        }
        for (dtDynIndex component = 0; component < 4; ++component)
        {
            assert(IsDtMathNear(model.body_quat[index](component),
                                expected.body_quat[index](component)));
            assert(IsDtMathNear(model.body_iquat[index](component),
                                expected.body_iquat[index](component)));
        }
    }

    for (dtDynIndex jointId = 0; jointId < model.njnt; ++jointId)
    {
        const std::size_t index = static_cast<std::size_t>(jointId);
        assert(model.jnt_limited[index] == expected.jnt_limited[index]);
        assert(IsDtMathNear(model.jnt_range_min(jointId), expected.jnt_range_min(jointId)));
        assert(IsDtMathNear(model.jnt_range_max(jointId), expected.jnt_range_max(jointId)));
        for (dtDynIndex axis = 0; axis < 3; ++axis)
        {
            assert(IsDtMathNear(model.jnt_pos[index](axis), expected.jnt_pos[index](axis)));
            assert(IsDtMathNear(model.jnt_axis[index](axis), expected.jnt_axis[index](axis)));
        }
    }
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        assert(IsDtMathNear(model.dof_armature(dofId), expected.dof_armature(dofId)));
    }
    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        const std::size_t index = static_cast<std::size_t>(siteId);
        for (dtDynIndex axis = 0; axis < 3; ++axis)
        {
            assert(IsDtMathNear(model.site_pos[index](axis), expected.site_pos[index](axis)));
            for (dtDynIndex column = 0; column < 3; ++column)
            {
                assert(IsDtMathNear(model.site_mat[index](axis, column),
                                    expected.site_mat[index](axis, column)));
            }
        }
        for (dtDynIndex component = 0; component < 4; ++component)
        {
            assert(IsDtMathNear(model.site_quat[index](component),
                                expected.site_quat[index](component)));
        }
    }
    for (dtDynIndex rootId = 0; rootId < model.nroot; ++rootId)
    {
        assert(IsDtMathNear(model.root_mass(rootId), expected.root_mass(rootId)));
    }
    for (dtDynIndex actuatorId = 0; actuatorId < model.nu; ++actuatorId)
    {
        assert(IsDtMathNear(model.actuator_gear(actuatorId), expected.actuator_gear(actuatorId)));
    }

    dtDynValidationError error;
    assert(ValidateModel(model, &error));

    dtDynData modelData;
    dtDynData expectedData;
    modelData.Resize(model);
    expectedData.Resize(expected);
    modelData.qpos(0) = 0.25;
    modelData.qpos(1) = -0.1;
    modelData.qvel(0) = 0.2;
    modelData.qvel(1) = -0.3;
    expectedData.qpos = modelData.qpos;
    expectedData.qvel = modelData.qvel;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = -9.81;
    gravity(2) = 0.0;
    ComputeDynamicsCCRBA(model, modelData, gravity, DTDYN_CCRBA_TERM_ALL);
    ComputeDynamicsCCRBA(expected, expectedData, gravity, DTDYN_CCRBA_TERM_ALL);
    ForwardKinematics(model, modelData);
    ForwardKinematics(expected, expectedData);

    for (dtDynIndex row = 0; row < model.nv; ++row)
    {
        assert(IsDtMathNear(modelData.gravityTorq(row), expectedData.gravityTorq(row)));
        assert(IsDtMathNear(modelData.nonlinTorq(row), expectedData.nonlinTorq(row)));
        for (dtDynIndex column = 0; column < model.nv; ++column)
        {
            assert(IsDtMathNear(modelData.Mmat(row, column), expectedData.Mmat(row, column)));
        }
    }
    for (dtDynIndex axis = 0; axis < 3; ++axis)
    {
        assert(IsDtMathNear(modelData.site_xpos[static_cast<std::size_t>(site)](axis),
                            expectedData.site_xpos[static_cast<std::size_t>(site)](axis)));
    }

    const dtDynModel zeroMassExpected = CompileSpecChecked(original);
    const std::size_t allocationsBeforeZeroMassUpdate = g_totalTestAllocations;
    assert(UpdateSpec(original, model));
    assert(g_totalTestAllocations == allocationsBeforeZeroMassUpdate);
    const std::size_t childIndex = static_cast<std::size_t>(child);
    for (dtDynIndex axis = 0; axis < 3; ++axis)
    {
        assert(IsDtMathNear(model.body_ipos[childIndex](axis), zeroMassExpected.body_ipos[childIndex](axis)));
    }
    for (dtDynIndex component = 0; component < 4; ++component)
    {
        assert(IsDtMathNear(model.body_iquat[childIndex](component),
                            zeroMassExpected.body_iquat[childIndex](component)));
    }
}

void TestUpdateSpecRefreshesMultiRootMultiDofCaches()
{
    dtDynSpec original;
    const dtDynIndex freeBody = original.AddBody("free_root", 0);
    const dtDynIndex ballBody = original.AddBody("ball_root", 0);
    const dtDynIndex freeJoint = original.AddJoint(freeBody, "free_joint", dtDynJointType::Free);
    const dtDynIndex ballJoint = original.AddJoint(ballBody, "ball_joint", dtDynJointType::Ball);
    original.EditBody(freeBody, [](dtDynBodySpec &draft)
                      { draft.inertial.mass = 2.0; });
    original.EditBody(ballBody, [](dtDynBodySpec &draft)
                      { draft.inertial.mass = 3.0; });
    original.EditJoint(freeJoint, [](dtDynJointSpec &draft)
                       { draft.armature = 0.1; });
    original.EditJoint(ballJoint, [](dtDynJointSpec &draft)
                       { draft.armature = 0.2; });

    dtDynModel model = CompileSpecChecked(original);
    dtDynSpec updated = original;
    updated.EditBody(freeBody, [](dtDynBodySpec &draft)
                     { draft.inertial.mass = 5.0; });
    updated.EditBody(ballBody, [](dtDynBodySpec &draft)
                     { draft.inertial.mass = 7.0; });
    updated.EditJoint(freeJoint, [](dtDynJointSpec &draft)
                      { draft.armature = 0.4; });
    updated.EditJoint(ballJoint, [](dtDynJointSpec &draft)
                      { draft.armature = 0.6; });

    const dtDynModel expected = CompileSpecChecked(updated);
    const std::size_t allocationsBeforeUpdate = g_totalTestAllocations;
    assert(UpdateSpec(updated, model));
    assert(g_totalTestAllocations == allocationsBeforeUpdate);
    assert(model.nroot == 2);
    for (dtDynIndex dofId = 0; dofId < model.nv; ++dofId)
    {
        assert(IsDtMathNear(model.dof_armature(dofId), expected.dof_armature(dofId)));
    }
    for (dtDynIndex rootId = 0; rootId < model.nroot; ++rootId)
    {
        assert(IsDtMathNear(model.root_mass(rootId), expected.root_mass(rootId)));
    }

    dtDynValidationError error;
    assert(ValidateModel(model, &error));
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

void TestDataCompatibilityRejectsLateWorkspaceMismatch()
{
    const dtDynModel model = CompileSpecChecked(dtDynSpec{});
    assert(model.nroot == 0);

    dtDynData data;
    data.Resize(model);
    assert(data.IsCompatible(model));

    ResizeDenseVector(data.comVel, 2);
    assert(!data.IsCompatible(model));
}

void TestDataResizeSupportsFullyFixedZeroDofModel()
{
    for (const dtDynIndex siteCount : {dtDynIndex(0), dtDynIndex(1), dtDynIndex(3)})
    {
        dtDynSpec spec;
        const dtDynIndex fixedBody = spec.AddBody("fixed_body", 0);
        for (dtDynIndex site = 0; site < siteCount; ++site)
        {
            spec.AddSite(fixedBody, "fixed_site_" + std::to_string(site));
        }
        const dtDynModel model = CompileSpecChecked(spec);
        assert(model.nq == 0);
        assert(model.nv == 0);

        dtDynData data;
        data.Resize(model);

        assert(data.IsCompatible(model));
        assert(data.body_jacp[fixedBody].GetRowSize() == 3);
        assert(data.body_jacp[fixedBody].GetColSize() == 0);
        for (dtDynIndex site = 0; site < siteCount; ++site)
        {
            assert(data.site_jacp[site].GetRowSize() == 3);
            assert(data.site_jacp[site].GetColSize() == 0);
        }
        assert(data.Mmat.GetRowSize() == 0);
        assert(data.Mmat.GetColSize() == 0);
    }
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

void TestQuatToMatPreservesQuarterTurn()
{
    dt::Math::Vector3<dtDynReal> axis;
    axis.SetZero();
    axis(2) = 1.0;
    const dt::Math::Vector4<dtDynReal> quarterTurn =
        math::AxisAngleQuat(axis, 0.5 * 3.14159265358979323846);
    const dt::Math::Matrix3<dtDynReal> matrix = math::QuatToMat(quarterTurn);

    assert(std::fabs(matrix(0, 0)) < 1.0e-12);
    assert(std::fabs(matrix(0, 1) + 1.0) < 1.0e-12);
    assert(std::fabs(matrix(1, 0) - 1.0) < 1.0e-12);
    assert(std::fabs(matrix(1, 1)) < 1.0e-12);
    assert(std::fabs(matrix(2, 2) - 1.0) < 1.0e-12);
}

void TestSpatialInertiaAtWorldOriginUsesParallelAxisTheorem()
{
    dt::Math::Vector3<dtDynReal> com;
    com(0) = 1.0;
    com(1) = 2.0;
    com(2) = 3.0;
    dt::Math::Matrix3<dtDynReal> rotationalInertia;
    rotationalInertia.SetZero();
    rotationalInertia(0, 0) = 4.0;
    rotationalInertia(1, 1) = 5.0;
    rotationalInertia(2, 2) = 6.0;

    const dt::Math::Matrix<6, 6, dtDynReal> inertia =
        math::SpatialInertiaAtWorldOrigin(2.0, com, rotationalInertia);

    assert(inertia(0, 0) == 30.0);
    assert(inertia(1, 1) == 25.0);
    assert(inertia(2, 2) == 16.0);
    assert(inertia(0, 1) == -4.0);
    assert(inertia(1, 2) == -12.0);
    assert(inertia(3, 3) == 2.0);
    assert(inertia(0, 4) == -6.0);
    assert(inertia(3, 1) == 6.0);
}

void TestCompileSpecUsesNativeIndexLayout()
{
    dtDynSpec spec;
    const auto base = spec.AddBody("base", 0);
    const auto shoulder = spec.AddBody("shoulder", base);
    const auto elbow = spec.AddBody("elbow", shoulder);
    const auto tool = spec.AddBody("tool", elbow);
    const auto toolSite = spec.AddSite(tool, "tool_tip");

    spec.EditBody(base, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    spec.EditBody(shoulder, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 3.0; });
    spec.EditBody(elbow, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 4.0; });
    spec.EditBody(tool, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });

    const auto elbowJoint = spec.AddJoint(elbow, "elbow_hinge", dtDynJointType::Hinge);
    const auto floatingBase = spec.AddJoint(shoulder, "floating_base", dtDynJointType::Free);
    spec.EditJoint(floatingBase, [&](dtDynJointSpec &draft)
                   { draft.armature = 0.1; });
    spec.EditJoint(elbowJoint, [&](dtDynJointSpec &draft)
                   { draft.armature = 0.2; });
    const auto actuator = spec.AddActuator(elbowJoint, "elbow_motor");
    spec.EditActuator(actuator, [&](dtDynActuatorSpec &draft)
                      { draft.gear = 120.0; });

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

    spec.EditBody(left, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    spec.EditBody(left, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 1.0; });
    spec.EditBody(right, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 3.0; });
    spec.EditBody(right, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 3.0; });
    spec.EditBody(right, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 1.0; });

    const auto leftJoint = spec.AddJoint(left, "left_hinge", dtDynJointType::Hinge);
    const auto rightJoint = spec.AddJoint(right, "right_hinge", dtDynJointType::Hinge);
    spec.EditJoint(leftJoint, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });
    spec.EditJoint(rightJoint, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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

void SetRootMassForComProductRegression(dtDynBodySpec &draft)
{
    draft.inertial.mass = 2.0;
}

void SetRootHingeAxisForComProductRegression(dtDynJointSpec &draft)
{
    draft.axis(1) = 1.0;
}

void TestCcrbaPreservesRootMassGuardedComProducts()
{
    dtDynSpec spec;
    const dtDynIndex root = spec.AddBody("root", 0);
    spec.EditBody(root, SetRootMassForComProductRegression);
    const dtDynIndex joint = spec.AddJoint(root, "root_hinge", dtDynJointType::Hinge);
    spec.EditJoint(joint, SetRootHingeAxisForComProductRegression);

    const dtDynModel model = CompileSpec(spec);
    assert(model.nroot == 1);
    assert(model.nv == 1);
    assert(std::fabs(model.root_mass(0) - 2.0) < 1.0e-12);

    dtDynData data;
    data.Resize(model);
    data.qvel(0) = 0.75;
    dt::Math::Vector3<dtDynReal> gravity;
    gravity.SetZero();
    ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL);

    for (dtDynIndex row = 0; row < 3; ++row)
    {
        assert(std::isfinite(data.comJmat(row, 0)));
        assert(std::isfinite(data.comJmatDot(row, 0)));
        assert(std::isfinite(data.comVel(row)));
        assert(std::fabs(data.comVel(row) - data.comJmat(row, 0) * data.qvel(0)) < 1.0e-12);
    }
}

void TestCompileSpecSupportsFloatingBaseQuadrupedTree()
{
    dtDynSpec spec;
    const auto trunk = spec.AddBody("trunk", 0);
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 5.0; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.25; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.35; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.45; });

    spec.AddJoint(trunk, "floating_base", dtDynJointType::Free);

    const char *legNames[4] = {"front_left", "front_right", "rear_left", "rear_right"};
    const dtDynReal xOffsets[4] = {0.4, 0.4, -0.4, -0.4};
    const dtDynReal yOffsets[4] = {0.25, -0.25, 0.25, -0.25};

    for (dtDynIndex leg = 0; leg < 4; ++leg)
    {
        const auto upper = spec.AddBody(std::string(legNames[leg]) + "_upper", trunk);
        spec.EditBody(upper, [&](dtDynBodySpec &draft)
                      { draft.pos(0) = xOffsets[leg]; });
        spec.EditBody(upper, [&](dtDynBodySpec &draft)
                      { draft.pos(1) = yOffsets[leg]; });
        spec.EditBody(upper, [&](dtDynBodySpec &draft)
                      { draft.inertial.mass = 1.0; });
        spec.EditBody(upper, [&](dtDynBodySpec &draft)
                      { draft.inertial.pos(2) = -0.2; });
        spec.EditBody(upper, [&](dtDynBodySpec &draft)
                      { draft.inertial.inertia(0) = 0.02; });
        spec.EditBody(upper, [&](dtDynBodySpec &draft)
                      { draft.inertial.inertia(1) = 0.02; });
        spec.EditBody(upper, [&](dtDynBodySpec &draft)
                      { draft.inertial.inertia(2) = 0.01; });

        const auto hip = spec.AddJoint(upper, std::string(legNames[leg]) + "_hip", dtDynJointType::Hinge);
        spec.EditJoint(hip, [&](dtDynJointSpec &draft)
                       { draft.axis(1) = 1.0; });
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

    spec.EditBody(shoulder, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditBody(forearm, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditSite(tool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 1.0; });

    const auto hinge = spec.AddJoint(shoulder, "shoulder_yaw", dtDynJointType::Hinge);
    spec.EditJoint(hinge, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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
    ComputeSiteJacobians(model, data);

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

    spec.EditBody(shoulder, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditBody(forearm, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditSite(tool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditSite(unused, [&](dtDynSiteSpec &draft)
                  { draft.pos(1) = 0.5; });

    const auto hinge = spec.AddJoint(shoulder, "shoulder_yaw", dtDynJointType::Hinge);
    spec.EditJoint(hinge, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 5.0; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.20; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.30; });
    spec.EditBody(trunk, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.40; });
    spec.AddJoint(trunk, "floating_base", dtDynJointType::Free);

    const auto thigh = spec.AddBody("thigh", trunk);
    spec.EditBody(thigh, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.35; });
    spec.EditBody(thigh, [&](dtDynBodySpec &draft)
                  { draft.pos(2) = -0.10; });
    spec.EditBody(thigh, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    const auto hip = spec.AddJoint(thigh, "hip_pitch", dtDynJointType::Hinge);
    spec.EditJoint(hip, [&](dtDynJointSpec &draft)
                   { draft.axis(1) = 1.0; });

    const auto foot = spec.AddSite(thigh, "foot");
    spec.EditSite(foot, [&](dtDynSiteSpec &draft)
                  { draft.pos(2) = -0.25; });

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

    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.5; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.5; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 0.75; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.5; });
    spec.EditSite(tool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.8; });

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.EditJoint(joint1, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });
    spec.EditJoint(joint2, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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

    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.5; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.5; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 0.75; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.5; });
    spec.EditSite(tool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.8; });

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.EditJoint(joint1, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });
    spec.EditJoint(joint2, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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

    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.5; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.5; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 0.75; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.5; });
    spec.EditSite(tool, [&](dtDynSiteSpec &draft)
                  { draft.pos(0) = 0.8; });

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.EditJoint(joint1, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });
    spec.EditJoint(joint2, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 1.0; });
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.5; });

    const auto hinge = spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);
    spec.EditJoint(hinge, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });
    spec.EditJoint(hinge, [&](dtDynJointSpec &draft)
                   { draft.armature = 0.25; });

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

    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 1.0; });

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.EditJoint(joint1, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });
    spec.EditJoint(joint2, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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

    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.4; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.4; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.02; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.03; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.04; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 0.8; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.pos(2) = -0.1; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 0.9; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.35; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(2) = -0.05; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(0) = 0.015; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(1) = 0.02; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.025; });

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_y", dtDynJointType::Hinge);
    spec.EditJoint(joint1, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });
    spec.EditJoint(joint2, [&](dtDynJointSpec &draft)
                   { draft.axis(1) = 1.0; });

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

    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.EditBody(link1, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.pos(0) = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 1.0; });
    spec.EditBody(link2, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 1.0; });

    const auto joint1 = spec.AddJoint(link1, "joint1_z", dtDynJointType::Hinge);
    const auto joint2 = spec.AddJoint(link2, "joint2_z", dtDynJointType::Hinge);
    spec.EditJoint(joint1, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });
    spec.EditJoint(joint2, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.7; });
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.2; });

    const auto hinge = spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);
    spec.EditJoint(hinge, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.mass = 2.0; });
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.pos(0) = 0.7; });
    spec.EditBody(link, [&](dtDynBodySpec &draft)
                  { draft.inertial.inertia(2) = 0.2; });
    const auto hinge = spec.AddJoint(link, "hinge_z", dtDynJointType::Hinge);
    spec.EditJoint(hinge, [&](dtDynJointSpec &draft)
                   { draft.axis(2) = 1.0; });

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
    TestCheckedCompileAcceptsValidSpec();
    TestCheckedCompileRejectsInvalidParent();
    TestSpecValidationRejectsDetachedJoint();
    TestSpecValidationRejectsSiblingCycle();
    TestSpecValidationRejectsDetachedSite();
    TestSpecValidationRejectsInvalidActuatorReference();
    TestSpecValidationRejectsInvalidBodyJointReference();
    TestSpecValidationRejectsDuplicateNames();
    TestSpecValidationRejectsDuplicateEmptyNames();
    TestSpecValidationRejectsNonFiniteValues();
    TestModelValidationRejectsDimensionOverflow();
    TestSpecValidationRejectsDimensionOverflowBeforeCompile();
    TestModelValidationRejectsArrayShapeMismatch();
    TestModelValidationRejectsMissingWorld();
    TestModelValidationRejectsNonFiniteValues();
    TestModelValidationRejectsInvalidDofReferences();
    TestModelValidationRejectsInvalidAddressSegments();
    TestModelValidationRejectsInvalidJointSpecProvenance();
    TestModelValidationRejectsInvalidRootMetadata();
    TestModelValidationRejectsDuplicateNames();
    TestModelValidationRejectsOverlappingJointSpans();
    TestModelValidationRejectsOrphanBodySegments();
    TestModelValidationRejectsDofOutsideDeclaredJointSpan();
    TestModelValidationRejectsInvalidWeldId();
    TestModelValidationRejectsIncorrectDofParentTopology();
    TestModelValidationRejectsInconsistentDerivedMasses();
    TestSpecValidationRejectsInvalidPhysicalValues();
    TestSpecValidationPreservesOrderedPhysicalAndNameFailures();
    TestModelValidationRejectsInvalidPhysicalValues();
    TestModelValidationRejectsInvalidFramePairs();
    TestModelValidationPreservesUniqueNameValidationOrder();
    TestModelValidationPreservesJointAndSiteValidationFailures();
    TestModelValidationPreservesEveryBodyNonFiniteFailure();
    TestModelValidationPreservesEveryBodyPhysicalOrFrameFailure();
    TestModelValidationPreservesBodyValidationPhasePrecedence();
    TestCompileSpecSupportsMaximumRootMappingWithoutNestedLookup();
    TestJointSizes();
    TestDtArrayDirectHeader();
    TestDtArrayCopyConstructorReleasesFailedPartialCopy();
    TestDtArrayReserveCopiesWithoutLeakingOrMutatingOnThrow();
    TestDtArrayPushBackRetainsByValueSignature();
    TestDtArrayPushBackMovesRvalue();
    TestDtArrayPushBackSupportsTriviallyCopyableRvalueWithoutMoveConstructionOrAssignment();
    TestDtArrayPushBackSupportsTriviallyCopyableMoveOnlyRvalue();
    TestDtArrayPushBackSupportsCopyConstructibleMoveAssignableLvalue();
    TestDtArrayPushBackPreservesMutableCopyAssignmentSelection();
    TestDtArrayPushBackSupportsMoveOnlyElements();
    TestDtArrayPushBackPreservesSelfRvalueTransferSemantics();
    TestDtArrayPushBackPreservesAliasedLvalueOnGrowth();
    TestDtArrayPushBackPreservesCopyOnlyAliasedLvalueOnGrowth();
    TestDtArrayReserveSupportsMoveOnlyElements();
    TestSpecWorldAndFixedBody();
    TestSpecEditorsCommitScopedCopies();
    TestSpecEditorsDiscardFailedDrafts();
    TestSpecEditorsRejectReentrantMutation();
    TestSpecEditorsDiscardCaughtReentrantAttempt();
    TestSpecEditorsRejectReentrantStructuralMutation();
    TestSpecEditorsRejectReentrantAssignment();
    TestSpecEditorsAbortReentrantCopyAssignmentTransactions();
    TestSpecOutsideEditSelfAssignmentRemainsUsable();
    TestSpecCopyAssignmentKeepsTargetUnchangedWhenStorageCopyThrows();
    TestSpecCollectionViewsSurviveCopyAssignmentAndMutations();
    TestSpecCollectionViewsSurviveMoveAssignment();
    TestSpecEditorsAbortReentrantMoveAssignmentTransactions();
    TestSpecEditorsAbortBothReentrantSwapTransactions();
    TestSpecNameLookupReflectsEditorMutations();
    TestModelActuatorNameLookupUsesRebuiltCache();
    TestSpecXmlReportsTypedNumericErrors();
    TestSpecXmlPreservesNumericWhitespaceAndFirstFailure();
    TestSpecXmlRejectsWrongVectorArity();
    TestSpecXmlPreservesVectorWhitespaceAndArityOffset();
    TestSpecXmlRejectsMalformedAttributes();
    TestSpecXmlPreservesAttributeWhitespaceAndFirstFailure();
    TestSpecXmlRejectsMisnestedSupportedTags();
    TestSpecXmlPreservesClosingElementFailureOrder();
    TestSpecXmlRejectsDuplicateObjectNames();
    TestSpecXmlRejectsUnknownReferencesWithTypedError();
    TestSpecXmlRejectsInvalidJointRange();
    TestSpecXmlRejectsUnsupportedSemanticValues();
    TestSpecXmlRejectsInvalidBooleanValue();
    TestSpecXmlPreservesLimitedBooleanGrammar();
    TestSpecXmlPreservesTagWhitespaceAndClosingSuffixOffset();
    TestSpecXmlRequiresSingleRootElement();
    TestSpecXmlRejectsSupportedElementInWrongContext();
    TestSpecXmlAcceptsEverySupportedElementInItsDocumentedParent();
    TestSpecXmlPreservesWhitespaceAndTagNameGrammar();
    TestSpecXmlScannerIsQuoteAwareAndFailClosed();
    TestSpecXmlRunsSetupValidationAfterParsing();
    TestSpecXmlRejectsClosingTagAttributes();
    TestSpecXmlRejectsMalformedSelfClosingSuffix();
    TestSpecXmlPreservesSelfClosingTagDetection();
    TestSpecXmlRejectsDuplicateDeclaration();
    TestSpecXmlRejectsRawLessThanInAttributeValue();
    TestSpecXmlRejectsMalformedDeclarationGrammar();
    TestSpecXmlPreservesDeclarationVersionAndAttributeGrammar();
    TestSpecXmlRejectsDeclarationAfterLeadingWhitespace();
    TestSpecXmlDecodeEncodeRoundTrip();
    TestSpecXmlLoadsFromFile();
    TestSpecXmlLoadsAcrossBufferBoundaries();
    TestSpecXmlLoadReportsTypedErrorPath();
    TestSpecXmlLoadReportsMissingFile();
    TestSpecXmlLoadReportsMalformedFilePath();
    TestSpecXmlLoadRejectsDirectoryPath();
    TestSpecXmlSavesToFile();
    TestSpecXmlSaveReportsOpenFailure();
    TestSpecXmlDecodesFreeJointAndFixedBody();
    TestSpecXmlSkipsCommentsAndInfersRangeLimits();
    TestSpecXmlRejectsEqualityUntilConstraintLayerExists();
    TestModelFixedBodyConvention();
    TestCompiledModelCopiesIntoDefaultModel();
    TestCompiledModelCopiesZeroSizedOptionalDenseFields();
    TestCompileSpecCanFillExistingModel();
    TestUpdateSpecRejectsSameDofJointTypeChangeWithoutWriting();
    TestUpdateSpecRejectsWorldParentMutationWithoutWriting();
    TestUpdateSpecRejectsChildListMutationWithoutWriting();
    TestUpdateSpecRejectsSiteBacklinkMutationWithoutWriting();
    TestUpdateSpecRejectsEarlyStorageMismatchWithoutWriting();
    TestUpdateSpecRejectsLateStorageMismatchWithoutWriting();
    TestUpdateSpecRejectsLateCompatibilityMetadataMutationsWithoutWriting();
    TestUpdateSpecMatchesPhysicalRecompile();
    TestUpdateSpecRefreshesMultiRootMultiDofCaches();
    TestDtMathDynamicResizeHelpers();
    TestDataResizeSeparatesBodyAndDofStorage();
    TestDataCompatibilityRejectsLateWorkspaceMismatch();
    TestDataResizeSupportsFullyFixedZeroDofModel();
    TestDtDynMathHelpersAreReusable();
    TestQuatToMatPreservesQuarterTurn();
    TestSpatialInertiaAtWorldOriginUsesParallelAxisTheorem();
    TestCompileSpecUsesNativeIndexLayout();
    TestCompileSpecSupportsMultipleMovableRoots();
    TestCcrbaPreservesRootMassGuardedComProducts();
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
