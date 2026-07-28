# 개요 {#dtdynamics_overview}

Author: Joonhee Jo

`dtDynamics`는 realtime controller와 whole-body control(WBC)이 소비하는 강체 동역학 **재료**를 계산하는 standalone package다. model/data 분리, 명시적 indexing, setup validation, 필요한 항만 선택하는 compute path를 중심으로 설계했다.

## 제공하는 것

- `dtDynSpec`: XML subset 또는 programmatic API로 만드는 mutable specification
- `ValidateSpec`, `CompileSpecChecked`: 외부/editable setup input의 fail-closed boundary
- `dtDynModel`: body/joint/site/actuator ID와 coordinate address를 고정한 compiled topology
- `dtDynData`: state, kinematics, dynamics, Jacobian, scratch를 위한 preallocated storage
- RNEA/CCRBA와 body/site/point Jacobian family
- CoM, centroidal momentum, 선택적 derivative term

## 제공하지 않는 것

contact solver, actuator model, simulator stepping, torque controller, estimator mapping, closed-chain constraint solver는 제공하지 않는다. MuJoCo와 Pinocchio는 validation/benchmark reference일 뿐 production dependency가 아니다.

## 읽는 순서

처음 사용하는 경우 @ref dtdynamics_modeling 에서 XML → checked compile → resize 경계를 확인한 뒤 @ref dtdynamics_example 을 실행한다. 그 다음 @ref dtdynamics_computation 의 prerequisite/frame convention과 @ref dtdynamics_realtime 의 resource scope를 읽는다. generated signature만 보고 호출하면 coordinate order, lifetime, prerequisite를 놓치기 쉽다.

## Resource claim의 범위

문서의 zero-allocation 표현은 model compilation, `dtDynData::Resize()`, caller-owned output sizing이 끝난 뒤 benchmark가 측정한 C++ `operator new` 경로를 뜻한다. arbitrary API, logger, `malloc` family, OS/runtime까지 포함하는 보편적 hard-realtime 보장이 아니다.
