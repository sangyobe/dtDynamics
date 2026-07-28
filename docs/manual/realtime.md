# Realtime resource model {#dtdynamics_realtime}

Author: Joonhee Jo

@dotfile diagrams/realtime_loop.dot

Realtime integration은 setup allocation과 control-loop compute를 분리한다.

## Setup phase

loop 밖에서 수행한다.

1. `LoadXmlFile()`/`DecodeXml()` 또는 programmatic API로 `dtDynSpec` 생성
2. `ValidateSpec()`/`CompileSpecChecked()` 실행
3. `dtDynData::Resize(model)` 실행
4. body/joint/site/actuator ID와 state address cache
5. caller-owned Jacobian/output matrix pre-size
6. controller/estimator convention에 따른 explicit state initialization

name lookup과 `IsCompatible()`는 setup/debug gate다. `IsCompatible()`은 dimension-only이므로 model identity gate로 사용하지 않는다.

benchmark의 `dtDynData_resource` row는 fresh construction + `Resize(model)` 비용을 측정한다. 이 setup 비용은 per-tick compute와 합산하지 않는다.

## Realtime phase

1. estimator state를 기존 `qpos`/`qvel` storage에 기록한다.
2. 필요한 최소 `ComputeDynamicsRNEA()` 또는 reduced-mask `ComputeDynamicsCCRBA()`를 호출한다.
3. active task에 필요한 Jacobian만 계산한다.
4. 결과를 controller task structure에 전달한다.

XML I/O, validation, compile, resize, name lookup, model hot-swap을 loop에 넣지 않는다.

## Allocation evidence

raw benchmark compute row는 executable의 C++ `operator new`/`operator new[]` counter로 다음을 기록한다.

- `allocation_mean`, `allocation_max`
- `allocation_bytes_mean`, `allocation_bytes_max`

preallocated `dtDynData`와 pre-sized caller output을 사용한 문서화된 compute scope에서 target은

\f[
\mathrm{alloc}_{count}^{tick}=0, \qquad
\mathrm{alloc}_{bytes}^{tick}=0.
\f]

이 값은 arbitrary API에 대한 universal guarantee가 아니다. dtMath container가 사용하는 `malloc`/`calloc`/`realloc`, external library, allocator 내부 동작, OS/runtime, logging, one-time initialization은 별도 profiler와 deployment test가 필요하다.

| Phase | 예 | C++ heap policy |
|---|---|---|
| setup | `ValidateSpec`, `CompileSpecChecked`, `Resize`, lookup | 허용 |
| loop | preallocated `ComputeDynamics*`, selected Jacobian | 계측 scope에서 0을 목표 |

## 여러 evaluation instance

immutable `dtDynModel`은 공유할 수 있다. mutable `dtDynData`는 estimator copy, MPC rollout, controller thread마다 별도로 둔다. 동시에 같은 data를 쓰지 않는다.

robot description이 바뀌면 setup으로 돌아가 checked compile, resize, ID/address rebuild를 마친 뒤 realtime을 재개한다. package-level neutral/default initializer는 NO-GO다.

## Timing 해석

median/p95/p99는 통계적 observation이다. CPU affinity를 고정해도 governor, scheduler, thermal state가 남는다. current evidence는 hard-realtime priority, deadline proof, frequency lock을 포함하지 않는다. setup `Resize` timing과 independently measured stage median은 additive decomposition이 아니다.
