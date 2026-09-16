# 1. 개요

Author: Joonhee Jo

- `dtDynamics`: realtime controller와 whole-body control(WBC)이 소비하는 강체 동역학 **재료** 계산 standalone package
- 설계 중심: model/data 분리, 명시적 indexing, setup validation, 필요한 항만 선택하는 compute path

## 1-1. 제공하는 것

- `dtDynSpec`: XML subset 또는 programmatic API로 만드는 setup-time editable specification. collection view는 immutable이고 field update는 callback-scoped `Edit*` transaction으로 commit
- `ValidateSpec`, `CompileSpecChecked`: 외부/editable setup input의 fail-closed boundary
- `dtDynModel`: body/joint/site/actuator ID와 coordinate address를 고정한 compiled topology
- `dtDynData`: state, kinematics, dynamics, Jacobian, scratch를 위한 preallocated storage
- RNEA/CCRBA와 body/site/point Jacobian family
- CoM, centroidal momentum, 선택적 derivative term

## 1-2. 제공하지 않는 것

- 미제공 항목: contact solver, actuator model, simulator stepping, torque controller, estimator mapping, closed-chain constraint solver

> **주의 — production dependency vs validation dependency**
>
> `MuJoCo`, `Pinocchio`: validation/benchmark reference 전용이며 production dependency 아님 (production `dt::Dynamics` target에 link되지 않음).

## 1-3. 읽는 순서

1. 처음 사용 시 [모델과 데이터 수명주기](modeling.md) 에서 XML → checked compile → resize 경계 확인
2. [최소 end-to-end 예제](example.md) 실행
3. [계산 파이프라인](computation.md) 의 prerequisite/frame convention 확인
4. [Realtime resource model](realtime.md) 의 resource scope 확인

- 주의: generated signature만 보고 호출 시 coordinate order, lifetime, prerequisite 누락 위험

## 1-4. Resource claim의 범위

- 문서의 zero-allocation 표현 의미: model compilation, `dtDynData::Resize()`, caller-owned output sizing 완료 후 benchmark가 측정한 C++ `operator new` 경로
- 아닌 것: arbitrary API, logger, `malloc` family, OS/runtime까지 포함하는 보편적 hard-realtime 보장

## 관련 문서

- 전체 목차: [전체 목차](index.md)
- 다음 chapter: [모델과 데이터 수명주기](modeling.md)
