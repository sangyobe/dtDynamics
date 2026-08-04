# Controller 통합 {#dtdynamics_integration}

Author: Joonhee Jo

- `dtDynamics` 제공 범위: controller가 문제를 풀 때 필요한 재료
- integration layer 소유 항목: contact schedule, support polygon, estimator mapping, torque·friction limit, solver formulation

## Setup에서 한 번 수행할 일

1. XML file이면 `dtDynSpec::LoadXmlFile()`, memory text이면 `DecodeXml()` 호출
2. 외부·편집 입력은 `ValidateSpec()` 또는 `CompileSpecChecked()`로 검증
3. `dtDynData::Resize(model)`로 storage 준비
4. `BodyId`, `JointId`, `SiteId`, `ActuatorId` 조회 후 `DTDYN_INVALID_INDEX` 검사
5. estimator의 state mapping과 active task ID를 controller 쪽에 cache

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

- `IsCompatible(model)` 확인 범위: data storage의 차원 compatibility만
- 미보장 항목: 같은 차원의 다른 model과 exact identity가 같다는 보장 → model 교체 뒤에는 ID/address mapping도 재생성

## Control tick

- 아래는 controller-owned symbol을 포함한 **pseudocode**
- `Jp`, `Jr`, `JdvP`, `JdvR`: setup에서 `3 x model.nv`로 미리 sizing

```text
WriteEstimatorStateInto(data.qpos, data.qvel); // controller-owned mapping
ComputeDynamicsCCRBA(model, data, gravity, wbcTermMask);
ComputeSiteJacobianTerms(model, data, foot,
                         &Jp, &Jr, nullptr, nullptr, &JdvP, &JdvR);
SolveControllerProblem(data.Mmat, data.nonlinTorq,
                       Jp, Jr, JdvP, JdvR); // controller-owned solver
```

- caller-owned Jacobian matrix 처리: strict zero-allocation path 필요 시 loop 진입 전에 미리 sizing
- tick 안에서 금지: name lookup, XML, validation, compilation, `Resize()`

## WBC handoff

- 일반적인 WBC 구성 대상 식:

\f[
M(q)\dot{v}+h(q,v)=S^\mathsf{T}\tau+J_c(q)^\mathsf{T}f.
\f]

- `dtDynamics`가 제공하는 항:
  - `Mmat`, `nonlinTorq` 또는 분리된 `gravityTorq`/`coriolisTorq`
  - body/site/point Jacobian과 \f$\dot{J}v\f$
  - 요청한 CoM·centroidal output
  - compiled dimensions, half-open coordinate addresses, cached IDs

- package 미포함 항목: solver variable, contact constraint, actuator limit, safety policy

## 여러 instance와 model update

- 공유 가능: 같은 immutable `dtDynModel`을 여러 worker가 공유 (mutable `dtDynData` 동시 공유는 금지)
- 개별 보유 원칙: estimator copy, MPC rollout, controller thread마다 별도 `dtDynData`

- robot description 변경 시 절차: realtime 정지 → setup 복귀 → 새 spec checked compile → data resize → lookup/address cache 재구성 후 loop 재개
- package-level neutral/default state initializer: NO-GO → 새 state는 controller/estimator contract에 따라 명시적으로 채움

## LeoQuad 경계

- LeoQuad 성격: installed-package consumer
- `dtDynamics`로 미이동 항목: LeoQuad의 required-ID 검사, state mapping, task 선택, compatibility export (controller policy)
- filesystem XML 처리 원칙: controller의 local `ReadTextFile` workaround 대신 package-owned `LoadXmlFile(fileName)` 사용
