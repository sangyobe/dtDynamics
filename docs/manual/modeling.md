# 모델과 데이터 수명주기 {#dtdynamics_modeling}

Author: Joonhee Jo

외부 모델을 control loop에서 안전하게 사용하려면 아래 경계를 한 번만 통과한다.

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

## XML text와 filesystem XML

XML bytes가 이미 memory에 있으면 `DecodeXml()`/`EncodeXml()`를 사용한다. 파일 경로를 다룰 때는 별도 `ReadTextFile` helper를 만들지 말고 package-owned API를 사용한다.

```cpp
const dtDynSpec spec = dtDynSpec::LoadXmlFile("model/robot.xml");
spec.SaveXmlFile("model/robot-native.xml");
```

`SaveXmlFile()`은 `EncodeXml()`과 같은 bytes를 기록하며 부모 디렉터리를 만들지 않는다. `LoadXmlFile()`과 `SaveXmlFile()`은 모두 setup/offline API다.

XML semantic failure는 `dtDynXmlError`로 통일된다. machine interface는 `what()` 문자열이 아니라 다음 accessor다.

- `Code()`: stable `dtDynXmlErrorCode`
- `SourcePath()`: `LoadXmlFile()`에 전달한 정확한 path; `DecodeXml()`이면 빈 문자열
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

invalid/overflow number, `NaN`/`Inf`, 잘못된 vector arity, malformed/misnested markup, duplicate name, unresolved reference, unsupported syntax·semantics는 조용히 보정하지 않고 실패한다. XML declaration은 original input byte 0에서 한 번만 허용된다.

## Checked compilation과 trusted compilation

외부 XML과 public mutable array를 거친 spec에는 먼저 setup validation을 적용한다.

```cpp
dtDynValidationError error;
if (!ValidateSpec(spec, &error))
{
    ReportSpecFailure(error.code, error.object, error.index, error.message);
}
const dtDynModel model = CompileSpecChecked(spec);
```

`dtDynValidationError`의 `code`, `object`, `index`가 machine-readable 위치를 제공한다. `CompileSpecChecked()`는 validation 후 기존 compiler를 호출한다. `CompileSpec()`는 backward compatibility를 위해 유지하며, 이미 검증된 trusted setup data에만 사용한다. validation scan은 dynamics hot loop에 들어가지 않는다.

## dtDynSpec와 dtDynModel lookup

두 layer는 body, joint, site, actuator 네 종류를 같은 방식으로 조회한다.

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

`dtDynSpec`의 public arrays는 사용자가 직접 수정할 수 있으므로 lookup은 setup-time linear scan이다. 영구 cache를 두면 direct mutation 뒤 stale map을 숨길 수 있다. compiled `dtDynModel`은 setup boundary가 통제되므로 rebuilt map을 사용한다. 어느 쪽도 missing name에 예외를 던지지 않고 `DTDYN_INVALID_INDEX`를 반환한다.

## 주소와 좌표

body `0`은 world body다. fixed body는 물리적 inertia와 frame을 가질 수 있지만 generalized coordinate column을 추가하지 않는다. joint `j`의 주소는 half-open interval로 해석한다.

\f[
[qadr_j, qadr_j+nq_j), \qquad [vadr_j, vadr_j+nv_j).
\f]

floating base에서는 `nq != nv`일 수 있다. `qpos`는 configuration coordinate, `qvel`은 tangent-space velocity다. body order, joint order, `qpos` order, `qvel` column order를 같은 것으로 가정하지 않는다. centroidal output의 각 root block은 `[linear; angular]` row 순서를 사용한다.

## dtDynData와 compatibility

`dtDynData::Resize(model)`은 `nq`, `nv`, body/site/root count에 맞춰 state, output, scratch storage를 준비한다. `nq == 0`, `nv == 0`인 fully fixed model도 유효하다. 이때 site Jacobian은 logical `3 x 0`, dynamics matrix는 `0 x 0`이다.

`data.IsCompatible(model)`은 **storage dimension과 model-indexed buffer size만** 확인한다. 같은 차원을 가진 서로 다른 model identity, name, topology, inertia가 같다는 뜻은 아니다. model을 다시 compile했다면 차원이 같더라도 cached ID/address assumption을 다시 만들고 `Resize()`를 재검토한다.

## Neutral/default state initializer 결정

package-level neutral-state initializer는 **NO-GO**다. free/ball joint quaternion, robot-specific rest pose, estimator convention을 하나의 보편적 default로 정하면 benchmark/controller policy가 package에 섞인다. caller는 compiled address와 자신의 model contract에 따라 `qpos`/`qvel`을 명시적으로 초기화해야 한다.

## Setup checklist

1. `LoadXmlFile()` 또는 `DecodeXml()`로 `dtDynSpec`을 만든다.
2. `ValidateSpec()`/`CompileSpecChecked()`로 외부 입력을 fail-closed 검증한다.
3. expected dimensions와 body/joint/site/actuator name을 확인한다.
4. `DTDYN_INVALID_INDEX`를 확인한 뒤 numeric ID와 state-address mapping을 cache한다.
5. 동시 평가 instance마다 하나의 `dtDynData`를 만들고 `Resize(model)`한다.
6. 그 뒤에만 realtime state update와 compute를 시작한다.
