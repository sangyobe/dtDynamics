# Controller 통합 {#dtdynamics_integration}

Author: Joonhee Jo

`dtDynamics`는 controller가 문제를 풀 때 필요한 재료를 제공한다. contact schedule, support polygon, estimator mapping, torque·friction limit, solver formulation은 integration layer가 소유한다.

## Setup에서 한 번 수행할 일

1. XML file이면 `dtDynSpec::LoadXmlFile()`, memory text이면 `DecodeXml()`을 호출한다.
2. 외부·편집 입력은 `ValidateSpec()` 또는 `CompileSpecChecked()`로 검증한다.
3. `dtDynData::Resize(model)`로 storage를 준비한다.
4. `BodyId`, `JointId`, `SiteId`, `ActuatorId`를 조회하고 `DTDYN_INVALID_INDEX`를 검사한다.
5. estimator의 state mapping과 active task ID를 controller 쪽에 cache한다.

```cpp
const dtDynSpec spec = dtDynSpec::LoadXmlFile(modelPath);
const dtDynModel model = CompileSpecChecked(spec);

dtDynData data;
data.Resize(model);

const dtDynIndex foot = model.SiteId("front_left_foot");
const dtDynIndex hipMotor = model.ActuatorId("front_left_hip_motor");
if (foot == DTDYN_INVALID_INDEX || hipMotor == DTDYN_INVALID_INDEX)
{
    throw std::runtime_error("required controller mapping is missing");
}
```

`IsCompatible(model)`은 data storage의 차원 compatibility만 확인한다. 같은 차원의 다른 model과 exact identity가 같다는 보장이 아니므로 model 교체 뒤에는 ID/address mapping도 다시 만든다.

## Control tick

다음은 controller-owned symbol을 포함한 **pseudocode**다. `Jp`, `Jr`, `JdvP`, `JdvR`은 setup에서 `3 x model.nv`로 미리 sizing한다.

```text
WriteEstimatorStateInto(data.qpos, data.qvel); // controller-owned mapping
ComputeDynamicsCCRBA(model, data, gravity, wbcTermMask);
ComputeSiteJacobianTerms(model, data, foot,
                         &Jp, &Jr, nullptr, nullptr, &JdvP, &JdvR);
SolveControllerProblem(data.Mmat, data.nonlinTorq,
                       Jp, Jr, JdvP, JdvR); // controller-owned solver
```

caller-owned Jacobian matrix는 strict zero-allocation path가 필요하면 loop 진입 전에 미리 sizing한다. name lookup, XML, validation, compilation, `Resize()`를 tick 안에서 실행하지 않는다.

## WBC handoff

일반적인 WBC는 다음 식을 controller policy에 맞게 구성한다.

\f[
M(q)\dot{v}+h(q,v)=S^\mathsf{T}\tau+J_c(q)^\mathsf{T}f.
\f]

`dtDynamics`가 제공하는 항은 다음과 같다.

- `Mmat`, `nonlinTorq` 또는 분리된 `gravityTorq`/`coriolisTorq`
- body/site/point Jacobian과 \f$\dot{J}v\f$
- 요청한 CoM·centroidal output
- compiled dimensions, half-open coordinate addresses, cached IDs

solver variable, contact constraint, actuator limit, safety policy는 package에 포함되지 않는다.

## 여러 instance와 model update

같은 immutable `dtDynModel`은 여러 worker가 공유할 수 있지만 mutable `dtDynData`를 동시에 공유하면 안 된다. estimator copy, MPC rollout, controller thread마다 별도 `dtDynData`를 둔다.

robot description이 바뀌면 realtime을 멈추고 setup으로 돌아간다. 새 spec을 checked compile하고, data를 resize하고, lookup/address cache를 재구성한 뒤 loop를 재개한다. package-level neutral/default state initializer는 NO-GO이므로 새 state는 controller/estimator contract에 따라 명시적으로 채운다.

## LeoQuad 경계

LeoQuad는 installed-package consumer다. LeoQuad의 required-ID 검사, state mapping, task 선택, compatibility export는 controller policy이므로 `dtDynamics`로 이동하지 않는다. filesystem XML은 controller의 local `ReadTextFile` workaround가 아니라 package-owned `LoadXmlFile(fileName)`을 사용해야 한다.
