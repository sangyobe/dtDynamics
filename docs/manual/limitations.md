# 제한과 scope 경계

Author: Joonhee Jo

- `dtDynamics 0.1.1` 범위: tree/forest rigid-body model의 setup 검증과 controller-facing dynamics ingredient

## 지원하는 범위

- fixed-base manipulator, multi-root fixed/floating model, floating-base quadruped
- no-joint fixed body와 `nq == 0`, `nv == 0`인 fully fixed model
- body, joint, site, actuator의 spec/model name lookup
- 문서화한 MJCF tree-model subset의 XML text/file codec
- RNEA/CCRBA, body/site/point Jacobian, CoM·centroidal 항

## 지원하지 않는 범위

- closed-chain/parallel mechanism의 solved dynamics와 equality-constraint layer
- contact/collision resolution, actuator dynamics, simulation stepping
- final torque, task selection, estimator mapping, safety/controller policy
- arbitrary MJCF defaults/classes, assets, sensor, tendon, equality syntax
- generic file utility, directory creation, resource resolver
- `CompileAndResize`처럼 setup allocation을 숨기는 wrapper

- unsupported XML syntax·semantics 처리: 추측값 대입 없이 `dtDynXmlError`로 실패

## Validation이 증명하는 것과 아닌 것

- MuJoCo 3.8.0, Pinocchio 3.9.0 성격: production dependency 아님, 겹치는 quantity의 independent reference
- `status=ok` 보고 대상: coordinate/frame transform 후 비교 가능한 항만
- equivalent contract 없는 Pinocchio centroidal row: `status=skipped`와 diagnostic으로 기록
- 해석 금지: contextual library-overhead timing을 same-output speed ratio로 해석

- `dtDynData::IsCompatible(model)` 확인 범위: storage dimension compatibility만
- 미증명 항목: exact model identity, topology, inertia, name equality

- benchmark의 C++ allocation 0 범위: 계측한 `operator new`/`operator new[]` scope에 한정
- 비보장 항목: direct `malloc` 계열, OS/runtime, external library, unsized caller output
- CPU affinity, seven-batch median/p95 효과: scheduler/DVFS 영향 완화 (hard-realtime deadline·byte-identical timing replay는 미증명)

## 상태 초기화

- neutral/default state initializer: NO-GO
- model/controller policy 소속: floating quaternion, rest pose, estimator convention
- 초기화 책임: caller가 compiled address와 자신의 state contract로 수행

## Compatibility 영향

- 0.1.1의 checked path·XML loader 영향: 이전에 silent default/partial parse로 통과하던 malformed numeric, wrong arity, duplicate name, invalid reference 거부 가능
- 성격: fail-closed setup contract → real robot XML을 release gate에서 확인 필요
- raw `CompileSpec()`: trusted backward-compatible path로 유지
