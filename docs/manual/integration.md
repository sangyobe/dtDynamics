# 8. Controller 통합

Author: Joonhee Jo

- `dtDynamics` 제공 범위: controller가 문제를 풀 때 필요한 재료
- integration layer 소유 항목: contact schedule, support polygon, estimator mapping, torque·friction limit, solver formulation

## 8-1. Setup과 control tick 개요

- Setup에서 한 번 수행할 일: XML load → `ValidateSpec`/`CompileSpecChecked` → `dtDynData::Resize` → ID lookup(`DTDYN_INVALID_INDEX` 확인) → estimator state mapping/active task ID cache
- `IsCompatible(model)` 확인 범위: data storage의 차원 compatibility만
- 미보장 항목: 같은 차원의 다른 model과 exact identity가 같다는 보장 → model 교체 뒤에는 ID/address mapping도 재생성

- 실행 절차(setup 코드, control tick pseudocode)는 `docs/manual.md` 4장 "Controller/WBC 통합 절차" 참고

> **주의 — setup-time vs realtime 경계**
>
> tick 안에서 금지: name lookup, XML, validation, compilation, `Resize()`.
>
> caller-owned Jacobian matrix 처리: strict zero-allocation path 필요 시 loop 진입 전에 미리 sizing.

## 8-2. WBC handoff

- 일반적인 WBC 구성 대상 식:

```math
M(q)\dot{v}+h(q,v)=S^\mathsf{T}\tau+J_c(q)^\mathsf{T}f.
```

- `dtDynamics`가 제공하는 항:
  - `Mmat`, `nonlinTorq` 또는 분리된 `gravityTorq`/`coriolisTorq`
  - body/site/point Jacobian과 $`\dot{J}v`$
  - 요청한 CoM·centroidal output
  - compiled dimensions, half-open coordinate addresses, cached IDs

- package 미포함 항목: solver variable, contact constraint, actuator limit, safety policy

## 8-3. 여러 instance와 model update

- 공유 가능: 같은 immutable `dtDynModel`을 여러 worker가 공유 (mutable `dtDynData` 동시 공유는 금지)
- 개별 보유 원칙: estimator copy, MPC rollout, controller thread마다 별도 `dtDynData`

- robot description 변경 시 절차: realtime 정지 → setup 복귀 → 새 spec checked compile → data resize → lookup/address cache 재구성 후 loop 재개
- package-level neutral/default state initializer: NO-GO → 새 state는 controller/estimator contract에 따라 명시적으로 채움

## 8-4. LeoQuad 경계

- LeoQuad 성격: installed-package consumer
- `dtDynamics`로 미이동 항목: LeoQuad의 required-ID 검사, state mapping, task 선택, compatibility export (controller policy)
- filesystem XML 처리 원칙: controller의 local `ReadTextFile` workaround 대신 package-owned `LoadXmlFile(fileName)` 사용

## 관련 문서

- 이전 chapter: [Benchmark](benchmarks.md)
- 다음 chapter: [API 지도](api.md)
- 전체 목차: [전체 목차](index.md)
- 실행 절차(setup 코드, control tick pseudocode): `docs/manual.md` 4장
