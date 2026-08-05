# dtDynamics Manual

Author: Joonhee Jo

> 유지되는 canonical manual source: [`docs/manual/`](manual/index.md) 아래
>
> ```bash
> tools/validate_docs.sh
> ```
>
> generated HTML entry: `build/docs/doxygen/html/index.html`

## Package boundary

- `dtDynamics 0.1.1`: `dtMath` 기반 dynamics ingredient engine
- MuJoCo 3.8.0, Pinocchio 3.9.0: validation/benchmark reference (production target `dt::Dynamics`에 link되지 않음)

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

- 외부·editable input: checked path 사용
- raw `CompileSpec()`: trusted setup path를 위한 호환 API
- lookup 실패: `DTDYN_INVALID_INDEX` (확인하지 않은 ID로 indexing 금지)

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

- 이 top-level 문서: 중복된 상세 contract 미유지
- API, coordinate, benchmark 설명: 위 canonical chapter가 소유
- 과거 release evidence: `docs/release-notes/`에 별도 보존
