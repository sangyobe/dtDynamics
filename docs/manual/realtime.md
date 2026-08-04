# Realtime resource model {#dtdynamics_realtime}

Author: Joonhee Jo

@dotfile diagrams/realtime_loop.dot

- Realtime 통합 원칙: setup allocation과 control-loop compute 분리

## Setup phase

- 수행 위치: loop 밖

1. `LoadXmlFile()`/`DecodeXml()` 또는 programmatic API로 `dtDynSpec` 생성
2. `ValidateSpec()`/`CompileSpecChecked()` 실행
3. `dtDynData::Resize(model)` 실행
4. body/joint/site/actuator ID와 state address cache
5. caller-owned Jacobian/output matrix pre-size
6. controller/estimator convention에 따른 explicit state initialization

- name lookup, `IsCompatible()`: setup/debug gate
- `IsCompatible()` 성격: dimension-only → model identity gate로 미사용

- benchmark의 `dtDynData_resource` row 측정 대상: fresh construction + `Resize(model)` 비용
- 처리 원칙: 이 setup 비용은 per-tick compute와 미합산

## Realtime phase

1. estimator state를 기존 `qpos`/`qvel` storage에 기록
2. 필요한 최소 `ComputeDynamicsRNEA()` 또는 reduced-mask `ComputeDynamicsCCRBA()` 호출
3. active task에 필요한 Jacobian만 계산
4. 결과를 controller task structure에 전달

- loop 배제 대상: XML I/O, validation, compile, resize, name lookup, model hot-swap

## Allocation evidence

- raw benchmark compute row 기록 방식: executable의 C++ `operator new`/`operator new[]` counter로 다음 기록
  - `allocation_mean`, `allocation_max`
  - `allocation_bytes_mean`, `allocation_bytes_max`

- preallocated `dtDynData`와 pre-sized caller output을 사용한 문서화된 compute scope의 목표값

\f[
\mathrm{alloc}_{count}^{tick}=0, \qquad
\mathrm{alloc}_{bytes}^{tick}=0.
\f]

- 성격: arbitrary API에 대한 universal guarantee 아님
- 별도 profiler·deployment test 필요 영역: dtMath container가 사용하는 `malloc`/`calloc`/`realloc`, external library, allocator 내부 동작, OS/runtime, logging, one-time initialization

| Phase | 예 | C++ heap policy |
|---|---|---|
| setup | `ValidateSpec`, `CompileSpecChecked`, `Resize`, lookup | 허용 |
| loop | preallocated `ComputeDynamics*`, selected Jacobian | 계측 scope에서 0을 목표 |

## 여러 evaluation instance

- 공유 가능: immutable `dtDynModel`
- 개별 보유 원칙: mutable `dtDynData`는 estimator copy, MPC rollout, controller thread마다 별도 (동시에 같은 data 미사용)

- robot description 변경 시 절차: setup으로 복귀 → checked compile, resize, ID/address rebuild 완료 후 realtime 재개
- package-level neutral/default initializer: NO-GO

## Timing 해석

- median/p95/p99 성격: 통계적 observation
- CPU affinity 고정 후에도 잔존 요인: governor, scheduler, thermal state
- current evidence 미포함 항목: hard-realtime priority, deadline proof, frequency lock
- 관계 아님: setup `Resize` timing과 independently measured stage median은 additive decomposition
