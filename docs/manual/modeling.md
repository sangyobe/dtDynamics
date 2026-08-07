# 2. 모델과 데이터 수명주기

Author: Joonhee Jo

- 외부 모델을 control loop에서 안전하게 사용하는 조건: 아래 경계를 한 번만 통과

@dotfile diagrams/model_layers.dot

```text
XML/file 또는 XML/text
  -> dtDynSpec (편집 가능)
  -> ValidateSpec / CompileSpecChecked
  -> dtDynModel (주소와 차원 고정)
  -> dtDynData::Resize (runtime storage 준비)
  -> ID와 state-address cache
  -> realtime compute
```

## 2-1. XML text와 filesystem XML

- XML bytes가 이미 memory에 있을 때: `DecodeXml()`/`EncodeXml()` 사용
- 파일 경로 처리 원칙: 별도 `ReadTextFile` helper 대신 package-owned API 사용

```cpp
const dtDynSpec spec = dtDynSpec::LoadXmlFile("model/robot.xml");
spec.SaveXmlFile("model/robot-native.xml");
```

- `SaveXmlFile()` 특성: `EncodeXml()`과 같은 bytes 기록, 부모 디렉터리는 생성하지 않음
- `LoadXmlFile()`, `SaveXmlFile()`: 모두 setup/offline API

- XML semantic failure 통일 방식: `dtDynXmlError`
- machine interface 원칙: `what()` 문자열이 아니라 다음 accessor 사용
  - `Code()`: stable `dtDynXmlErrorCode`
  - `SourcePath()`: `LoadXmlFile()`에 전달한 정확한 path (`DecodeXml()`이면 빈 문자열)
  - `Offset()`: original input의 **zero-based byte offset**

```cpp
try
{
    const dtDynSpec spec = dtDynSpec::LoadXmlFile("model/robot.xml");
    const dtDynModel model = CompileSpecChecked(spec);
}
catch (const dtDynXmlError &error)
{
    ReportXmlFailure(error.Code(), error.SourcePath(), error.Offset());
}
```

> **주의 — fail-closed validation 동작**
>
> fail-closed 거부 대상: invalid/overflow number, `NaN`/`Inf`, 잘못된 vector arity, malformed/misnested markup, duplicate name, unresolved reference, unsupported syntax·semantics (조용한 보정 없음).
>
> XML declaration 허용 조건: original input byte 0에서 한 번만.

## 2-2. Checked compilation과 trusted compilation

- 외부 XML과 public mutable array를 거친 spec의 처리 순서: setup validation을 먼저 적용

```cpp
dtDynValidationError error;
if (!ValidateSpec(spec, &error))
{
    ReportSpecFailure(error.code, error.object, error.index, error.message);
}
const dtDynModel model = CompileSpecChecked(spec);
```

- `dtDynValidationError`의 `code`, `object`, `index`: machine-readable 위치 제공
- `CompileSpecChecked()` 동작: validation 후 기존 compiler 호출
- `CompileSpec()` 유지 목적: backward compatibility, 이미 검증된 trusted setup data 전용
- validation scan 범위: dynamics hot loop 밖

## 2-3. dtDynSpec와 dtDynModel lookup

- 공통 조회 방식: body, joint, site, actuator 네 종류를 같은 방식으로 조회

```cpp
const dtDynIndex body = spec.BodyId("base");
const dtDynIndex joint = spec.JointId("hip");
const dtDynIndex site = spec.SiteId("foot");
const dtDynIndex actuator = spec.ActuatorId("hip_motor");

if (body == DTDYN_INVALID_INDEX ||
    joint == DTDYN_INVALID_INDEX ||
    site == DTDYN_INVALID_INDEX ||
    actuator == DTDYN_INVALID_INDEX)
{
    throw std::runtime_error("required body/joint/site/actuator is missing");
}
```

- `dtDynSpec`의 public arrays: 사용자가 직접 수정 가능 → lookup은 setup-time linear scan
- 영구 cache의 위험: direct mutation 뒤 stale map을 숨길 수 있음
- compiled `dtDynModel`의 lookup 방식: setup boundary가 통제되므로 rebuilt map 사용
- missing name 처리: 양쪽 모두 예외 대신 `DTDYN_INVALID_INDEX` 반환

## 2-4. 주소와 좌표

- body `0`: world body
- fixed body 특성: 물리적 inertia와 frame 보유 가능, generalized coordinate column은 미추가
- joint `j`의 주소 해석: half-open interval

\f[
[qadr_j, qadr_j+nq_j), \qquad [vadr_j, vadr_j+nv_j).
\f]

- floating base 특성: `nq != nv` 가능
- `qpos`: configuration coordinate, `qvel`: tangent-space velocity
- 금지 가정: body order, joint order, `qpos` order, `qvel` column order가 같다는 가정
- centroidal output의 root block 행 순서: `[linear; angular]`

## 2-5. dtDynData와 compatibility

- `dtDynData::Resize(model)` 동작: `nq`, `nv`, body/site/root count에 맞춰 state, output, scratch storage 준비
- `nq == 0`, `nv == 0`인 fully fixed model: 유효 (site Jacobian은 logical `3 x 0`, dynamics matrix는 `0 x 0`)

- `data.IsCompatible(model)` 확인 범위: **storage dimension과 model-indexed buffer size만**
- 미보장 항목: 같은 차원을 가진 서로 다른 model identity, name, topology, inertia가 같다는 뜻
- model 재compile 시 조치: 차원이 같더라도 cached ID/address assumption을 다시 만들고 `Resize()` 재검토

## 2-6. Neutral/default state initializer 결정

- 결정: package-level neutral-state initializer **NO-GO**
- 이유: free/ball joint quaternion, robot-specific rest pose, estimator convention을 하나의 보편적 default로 정하면 benchmark/controller policy가 package에 섞임
- 초기화 책임: caller가 compiled address와 자신의 model contract에 따라 `qpos`/`qvel`을 명시적으로 초기화

## 2-7. Setup 실행 절차

- 실행 절차(단계별 checklist, 코드 예제)는 `docs/manual.md` 2장 "모델 준비 절차" 참고

## 관련 문서

- 이전 chapter: @ref md_docs_manual_overview
- 다음 chapter: @ref md_docs_manual_computation
- 전체 목차: @ref md_docs_manual_index
- 실행 절차: `docs/manual.md` 2장
