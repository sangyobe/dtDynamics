# API 지도

Author: Joonhee Jo

- 이 페이지: generated signature를 읽기 전에 setup/runtime 경계를 빠르게 찾기 위한 지도

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

- `dtDynSpec`, `dtDynModel` 공통 제공: `BodyId`, `JointId`, `SiteId`, `ActuatorId`
- missing name: `DTDYN_INVALID_INDEX`

## Error contract

- XML: `dtDynXmlError::Code()`, `SourcePath()`, `Offset()`
- Spec/model: `dtDynValidationError::{code, object, index, message}`
- 분리 원칙: human-readable text와 machine category 분리
- XML offset: original input의 zero-based byte offset

## Compute API

- `ForwardKinematics`: body/site kinematic quantity 갱신
- `ComputeDynamicsRNEA`: 기본 mass/bias/gravity/coriolis ingredient path
- `ComputeDynamicsCCRBA`: 선택한 CoM·centroidal·derivative term을 포함하는 path
- body/site/point Jacobian family: task kinematics와 derivative product

- term mask, prerequisite: computation chapter 참조
- 주의: output이 필요하지 않으면 `DTDYN_CCRBA_TERM_ALL` 요청 금지

## Resource boundary

- setup API: `Validate*`, XML I/O, `CompileSpec*`, `Resize`, name lookup
- compute path의 allocation 주장 범위: benchmark에서 preallocated `dtDynData`와 pre-sized caller output을 사용한 C++ allocation counter 범위에 한정
