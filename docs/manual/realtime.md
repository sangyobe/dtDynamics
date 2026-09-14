# 5. Realtime resource model

Author: Joonhee Jo

![realtime loop diagram](diagrams/realtime_loop.svg)

- Realtime 통합 원칙: setup allocation과 control-loop compute 분리

## 5-1. Setup/Realtime 경계 원칙

- Realtime 통합 원칙: setup allocation과 control-loop compute 분리
- setup 수행 위치: loop 밖 (XML I/O, validation, compile, resize, name lookup)
- realtime 수행 항목: state staging, 필요한 최소 `ComputeDynamics*` 호출, active task Jacobian, controller로 결과 전달

> **주의 — setup-time vs realtime 경계**
>
> loop 배제 대상: XML I/O, validation, compile, resize, name lookup, model hot-swap.

- 단계별 checklist는 `docs/manual.md` 5장 "Realtime 적용 checklist" 참고

- benchmark의 `dtDynData_resource` row 측정 대상: fresh construction + `Resize(model)` 비용
- 처리 원칙: 이 setup 비용은 per-tick compute와 미합산

## 5-2. Allocation evidence

- raw benchmark compute row 기록 방식: executable의 C++ `operator new`/`operator new[]` counter로 다음 기록
  - `allocation_mean`, `allocation_max`
  - `allocation_bytes_mean`, `allocation_bytes_max`

- preallocated `dtDynData`와 pre-sized caller output을 사용한 문서화된 compute scope의 목표값

```math
\mathrm{alloc}_{count}^{tick}=0, \qquad
\mathrm{alloc}_{bytes}^{tick}=0.
```

- 성격: arbitrary API에 대한 universal guarantee 아님
- 별도 profiler·deployment test 필요 영역: dtMath container가 사용하는 `malloc`/`calloc`/`realloc`, external library, allocator 내부 동작, OS/runtime, logging, one-time initialization

| Phase | 예 | C++ heap policy |
|---|---|---|
| setup | `ValidateSpec`, `CompileSpecChecked`, `Resize`, lookup | 허용 |
| loop | preallocated `ComputeDynamics*`, selected Jacobian | 계측 scope에서 0을 목표 |

## 5-3. 여러 evaluation instance

- 공유 가능: immutable `dtDynModel`
- 개별 보유 원칙: mutable `dtDynData`는 estimator copy, MPC rollout, controller thread마다 별도 (동시에 같은 data 미사용)

- robot description 변경 시 절차: setup으로 복귀 → checked compile, resize, ID/address rebuild 완료 후 realtime 재개
- package-level neutral/default initializer: NO-GO

## 5-4. Timing 해석

- median/p95/p99 성격: 통계적 observation
- CPU affinity 고정 후에도 잔존 요인: governor, scheduler, thermal state
- current evidence 미포함 항목: hard-realtime priority, deadline proof, frequency lock
- 관계 아님: setup `Resize` timing과 independently measured stage median은 additive decomposition

## 관련 문서

- 이전 chapter: [최소 end-to-end 예제](example.md)
- 다음 chapter: [검증](validation.md)
- 전체 목차: [전체 목차](index.md)
- 실행 절차(setup/realtime checklist): `docs/manual.md` 5장
