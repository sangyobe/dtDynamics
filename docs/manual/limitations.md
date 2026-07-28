# 제한과 scope 경계 {#dtdynamics_limitations}

Author: Joonhee Jo

`dtDynamics 0.1.1`은 tree/forest rigid-body model의 setup 검증과 controller-facing dynamics ingredient에 범위를 둔다.

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

unsupported XML syntax·semantics는 가능한 값으로 추측하지 않고 `dtDynXmlError`로 실패한다.

## Validation이 증명하는 것과 아닌 것

MuJoCo 3.8.0과 Pinocchio 3.9.0은 production dependency가 아니라 겹치는 quantity의 independent reference다. coordinate/frame transform 뒤 비교 가능한 항만 `status=ok`로 보고하며, equivalent contract가 없는 Pinocchio centroidal row는 `status=skipped`와 diagnostic으로 남긴다. contextual library-overhead timing을 same-output speed ratio로 해석하지 않는다.

`dtDynData::IsCompatible(model)`은 storage dimension compatibility만 확인한다. exact model identity, topology, inertia, name equality를 증명하지 않는다.

benchmark의 C++ allocation 0은 계측한 `operator new`/`operator new[]` scope에 한정된다. direct `malloc` 계열, OS/runtime, external library, unsized caller output까지 포함하는 보장이 아니다. CPU affinity와 seven-batch median/p95는 scheduler/DVFS 영향을 줄이지만 hard-realtime deadline이나 byte-identical timing replay를 증명하지 않는다.

## 상태 초기화

neutral/default state initializer는 NO-GO다. floating quaternion, rest pose, estimator convention은 model/controller policy에 속한다. caller가 compiled address와 자신의 state contract로 초기화한다.

## Compatibility 영향

0.1.1의 checked path와 XML loader는 이전에 silent default/partial parse로 통과하던 malformed numeric, wrong arity, duplicate name, invalid reference를 거부할 수 있다. 이것은 fail-closed setup contract이며 real robot XML을 release gate에서 확인해야 한다. raw `CompileSpec()`는 trusted backward-compatible path로 유지된다.
