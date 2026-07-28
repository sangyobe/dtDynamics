# API 지도 {#dtdynamics_api_map}

Author: Joonhee Jo

이 페이지는 generated signature를 읽기 전에 setup/runtime 경계를 빠르게 찾기 위한 지도다.

## Model lifecycle

| Layer | 주요 API | Contract |
|---|---|---|
| XML text | `DecodeXml`, `EncodeXml` | caller가 memory bytes를 소유 |
| XML file | `LoadXmlFile`, `SaveXmlFile` | package가 file I/O와 source path diagnostic 소유 |
| editable spec | `dtDynSpec`, `ValidateSpec` | public mutable arrays, setup-time linear lookup |
| checked compile | `CompileSpecChecked` | external/editable input을 validate한 뒤 compile |
| trusted compile | `CompileSpec` | 이미 검증된 setup input을 위한 호환 path |
| compiled model | `dtDynModel`, `ValidateModel` | dimensions/address/name map이 고정된 setup contract |
| runtime storage | `dtDynData::Resize`, `IsCompatible` | model-indexed storage 준비; compatibility는 dimension-only |

`dtDynSpec`와 `dtDynModel`은 `BodyId`, `JointId`, `SiteId`, `ActuatorId`를 모두 제공한다. missing name은 `DTDYN_INVALID_INDEX`다.

## Error contract

- XML: `dtDynXmlError::Code()`, `SourcePath()`, `Offset()`
- Spec/model: `dtDynValidationError::{code, object, index, message}`

human-readable text와 machine category를 분리한다. XML offset은 original input의 zero-based byte offset이다.

## Compute API

- `ForwardKinematics`: body/site kinematic quantity 갱신
- `ComputeDynamicsRNEA`: 기본 mass/bias/gravity/coriolis ingredient path
- `ComputeDynamicsCCRBA`: 선택한 CoM·centroidal·derivative term을 포함하는 path
- body/site/point Jacobian family: task kinematics와 derivative product

term mask와 prerequisite는 computation chapter를 따른다. output이 필요하지 않으면 무조건 `DTDYN_CCRBA_TERM_ALL`을 요청하지 않는다.

## Resource boundary

`Validate*`, XML I/O, `CompileSpec*`, `Resize`, name lookup은 setup API다. compute path의 allocation 주장은 benchmark에서 preallocated `dtDynData`와 pre-sized caller output을 사용한 C++ allocation counter 범위에 한정된다.
