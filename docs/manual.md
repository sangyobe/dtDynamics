# dtDynamics Manual

Author: Joonhee Jo

> 유지되는 canonical manual source는 [`docs/manual/`](manual/index.md) 아래에 있다.
>
> ```bash
> tools/validate_docs.sh
> ```
>
> generated HTML entry: `build/docs/doxygen/html/index.html`

## Package boundary

`dtDynamics 0.1.1`은 `dtMath` 기반 dynamics ingredient engine이다. MuJoCo 3.8.0과 Pinocchio 3.9.0은 validation/benchmark reference이며 production target `dt::Dynamics`에 link되지 않는다.

```cmake
find_package(dtDynamics 0.1.1 EXACT CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE dt::Dynamics)
```

## Canonical lifecycle

```text
LoadXmlFile / DecodeXml
  -> ValidateSpec / CompileSpecChecked
  -> dtDynModel
  -> dtDynData::Resize
  -> BodyId / JointId / SiteId / ActuatorId cache
  -> qpos/qvel staging
  -> selected ComputeDynamics* / Jacobian calls
```

외부·editable input에는 checked path를 사용한다. raw `CompileSpec()`는 trusted setup path를 위한 호환 API다. lookup 실패는 `DTDYN_INVALID_INDEX`이며 확인하지 않은 ID로 indexing하지 않는다.

## Redesign figure

![To-Be dtDynamics architecture](figures/dtdynamics_to_be.svg)

## 읽을 문서

- [개요](manual/overview.md)
- [모델과 데이터 수명주기](manual/modeling.md)
- [계산 pipeline](manual/computation.md)
- [최소 예제](manual/example.md)
- [Realtime resource model](manual/realtime.md)
- [검증](manual/validation.md)
- [Benchmark](manual/benchmarks.md)
- [Controller 통합](manual/integration.md)
- [API 지도](manual/api.md)
- [제한](manual/limitations.md)
- [Release 경계](manual/release.md)

이 top-level 문서는 중복된 상세 contract를 유지하지 않는다. API, coordinate, benchmark 설명은 위 canonical chapter가 소유한다. 과거 release evidence는 `docs/release-notes/`에 별도로 보존한다.
