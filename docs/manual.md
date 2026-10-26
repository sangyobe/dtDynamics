# dtDynamics Manual

Author: Joonhee Jo

> 이 문서: **실행 절차(how-to) 전용** 진입점 — 개념/API contract/equation은 `docs/manual/` 하위 chapter가 canonical source
>
> GitLab Markdown viewer에서 직접 열람. 수식은 GitLab math block/inline math 문법으로 직접 렌더링.

## 목차

1. [Package boundary와 최소 사용법](#1-package-boundary와-최소-사용법)
2. [모델 준비 절차](#2-모델-준비-절차)
3. [최소 end-to-end 예제](#3-최소-end-to-end-예제)
4. [Controller/WBC 통합 절차](#4-controllerwbc-통합-절차)
5. [Realtime 적용 checklist](#5-realtime-적용-checklist)
6. [검증 실행 방법](#6-검증-실행-방법)
7. [Benchmark 재현 방법](#7-benchmark-재현-방법)
8. [Markdown 문서·Release 절차](#8-markdown-문서release-절차)
9. [Troubleshooting](#9-troubleshooting)
10. [상세 설명 chapter와 관련 문서](#10-상세-설명-chapter와-관련-문서)

## 1. Package boundary와 최소 사용법

- `dtDynamics 0.1.3`: `dtMath` 기반 dynamics ingredient engine

> **주의 — production dependency vs validation dependency**
>
> `MuJoCo 3.8.0`, `Pinocchio 3.9.0`: validation/benchmark reference (production target `dt::Dynamics`에 link되지 않음).

```cmake
find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE dt::Dynamics)
```

- 설치 prefix(`${ARTF_INSTALL_DIR:-$HOME/.local/ART_Framework}`)는 CMake 기본 탐색 경로가 아니다. `CMAKE_PREFIX_PATH`에 prefix를 넣거나, 다른 package가 섞이지 않게 경로를 고정하려면 다음 형태를 쓴다.

```cmake
find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED
    PATHS "${ARTF_INSTALL_DIR}"
    NO_DEFAULT_PATH)
```

- Canonical lifecycle (아래 2~5장이 각 단계를 실행 절차로 확장):

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

![To-Be dtDynamics architecture](figures/dtdynamics_to_be.svg)

## 2. 모델 준비 절차

### 2-1. XML 로드

```cpp
const dtDynSpec spec = dtDynSpec::LoadXmlFile("model/robot.xml");
spec.SaveXmlFile("model/robot-native.xml");
```

### 2-2. 에러 처리 (`dtDynXmlError`)

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

### 2-3. Checked compile

```cpp
dtDynValidationError error;
if (!ValidateSpec(spec, &error))
{
    ReportSpecFailure(error.code, error.object, error.index, error.message);
}
const dtDynModel model = CompileSpecChecked(spec);
```

### 2-4. Programmatic spec edit

- `Bodies()`, `Joints()`, `Sites()`, `Actuators()`는 immutable collection view이다.
- setup-time field 변경은 `EditBody`, `EditJoint`, `EditSite`, `EditActuator` callback으로만 수행한다. callback이 정상 반환할 때만 item copy가 commit된다.
- callback 안에서 같은 spec의 nested `Edit*`, `Add*`, `Clear`, assignment 또는 `swap`을 시도하면 transaction은 abort되며, callback이 inner exception을 잡아도 outer edit는 commit되지 않는다.
- committed edit도 외부 input과 같으므로 `ValidateSpec()`/`CompileSpecChecked()` 이전에 realtime으로 전달하지 않는다.

### 2-5. ID lookup과 cache

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

### 2-6. Setup checklist

1. `LoadXmlFile()` 또는 `DecodeXml()`로 `dtDynSpec` 생성
2. programmatic field 변경은 callback-scoped `Edit*`로 완료하고, collection view reference를 보관하지 않음
3. `ValidateSpec()`/`CompileSpecChecked()`로 외부 입력과 committed edit를 fail-closed 검증
4. expected dimensions와 body/joint/site/actuator name 확인
5. `DTDYN_INVALID_INDEX` 확인 후 numeric ID와 state-address mapping cache
6. 동시 평가 instance마다 하나의 `dtDynData` 생성, `Resize(model)` 실행
7. 이후 realtime state update와 compute 시작

- 개념(lifecycle, address 규칙, `dtDynSpec`/`dtDynModel` 설계 근거): [modeling.md](manual/modeling.md)

## 3. 최소 end-to-end 예제

- 이 예제가 보여주는 순서: **load → validate → compile → resize → ID cache → state 입력 → compute**
- 성격: simulator나 torque controller 예제가 아닌 dynamics ingredient 사용 예제

### 3-1. 입력 XML

```xml
<mujoco>
  <worldbody>
    <body name="base">
      <body name="link1" pos="0.3 0 0">
        <inertial pos="0.15 0 0" mass="1" diaginertia="0.01 0.02 0.02"/>
        <joint name="shoulder" type="hinge" axis="0 0 1"/>
        <body name="link2" pos="0.25 0 0">
          <inertial pos="0.125 0 0" mass="0.8" diaginertia="0.008 0.012 0.012"/>
          <joint name="elbow" type="hinge" axis="0 0 1"/>
          <site name="tool" pos="0.2 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator><motor name="shoulder_motor" joint="shoulder"/></actuator>
</mujoco>
```

### 3-2. Program

```cpp
#include <dtDynamics/dtDynCompile.h>
#include <dtDynamics/dtDynData.h>
#include <dtDynamics/dtDynDynamics.h>
#include <dtDynamics/dtDynSpec.h>
#include <dtDynamics/dtDynValidation.h>

#include <stdexcept>
#include <string>

using namespace dt::Dynamics;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        throw std::runtime_error("usage: example <model.xml>");
    }

    // Filesystem XML. XML text가 memory에 있으면 DecodeXml(xml)을 사용한다.
    const dtDynSpec spec = dtDynSpec::LoadXmlFile(argv[1]);

    dtDynValidationError validation;
    if (!ValidateSpec(spec, &validation))
    {
        throw std::runtime_error(validation.message);
    }
    const dtDynModel model = CompileSpecChecked(spec);

    dtDynData data;
    data.Resize(model); // setup-time allocation
    if (!data.IsCompatible(model))
    {
        throw std::runtime_error("storage dimensions do not match model");
    }

    // Spec와 Model 모두 같은 네 lookup family를 제공한다.
    const dtDynIndex specBody = spec.BodyId("link2");
    const dtDynIndex specJoint = spec.JointId("elbow");
    const dtDynIndex tool = model.SiteId("tool");
    const dtDynIndex shoulderMotor = model.ActuatorId("shoulder_motor");
    if (specBody == DTDYN_INVALID_INDEX ||
        specJoint == DTDYN_INVALID_INDEX ||
        tool == DTDYN_INVALID_INDEX ||
        shoulderMotor == DTDYN_INVALID_INDEX)
    {
        throw std::runtime_error("required model name is missing");
    }

    // 이 fixture는 scalar hinge 두 개다. 실제 controller는 compiled address mapping을 사용한다.
    data.qpos(0) = 0.20;
    data.qpos(1) = -0.30;
    data.qvel(0) = 0.10;
    data.qvel(1) = 0.05;

    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = 0.0;
    gravity(2) = -9.81;

    const dtDynCcrbaTermFlags terms =
        DTDYN_CCRBA_TERM_COM_POS |
        DTDYN_CCRBA_TERM_COM_JMAT |
        DTDYN_CCRBA_TERM_CENT_MOM;
    ComputeDynamicsCCRBA(model, data, gravity, terms);

    // 결과는 data.Mmat, data.gravityTorq, data.coriolisTorq,
    // data.nonlinTorq와 요청한 CoM/centroidal field에 있다.
    (void)tool;
    (void)shoulderMotor;
    return 0;
}
```

### 3-3. 실패 진단

- 파일 XML semantic error 보존 정보: exact source path와 zero-based byte offset

```cpp
catch (const dtDynXmlError &error)
{
    Log(error.Code(), error.SourcePath(), error.Offset(), error.what());
}
```

- programmatic mutation으로 topology 오류 발생 시: `ValidateSpec(spec, &error)`가 `error.code`, `error.object`, `error.index`, `error.message` 채움
- 금지 사항: `what()`이나 process exit code만으로 machine category 추론
- 개념(예제 설계 의도, `CompileSpec`/neutral-state 주의): [example.md](manual/example.md)

## 4. Controller/WBC 통합 절차

### 4-1. Setup에서 한 번 수행할 일

1. XML file이면 `dtDynSpec::LoadXmlFile()`, memory text이면 `DecodeXml()` 호출
2. 외부·편집 입력은 `ValidateSpec()` 또는 `CompileSpecChecked()`로 검증
3. `dtDynData::Resize(model)`로 storage 준비
4. `BodyId`, `JointId`, `SiteId`, `ActuatorId` 조회 후 `DTDYN_INVALID_INDEX` 검사
5. estimator의 state mapping과 active task ID를 controller 쪽에 cache

```cpp
const dtDynSpec spec = dtDynSpec::LoadXmlFile(modelPath);
const dtDynModel model = CompileSpecChecked(spec);

dtDynData data;
data.Resize(model);

const dtDynIndex foot = model.SiteId("front_left_foot");
const dtDynIndex hipMotor = model.ActuatorId("front_left_hip_motor");
if (foot == DTDYN_INVALID_INDEX || hipMotor == DTDYN_INVALID_INDEX)
{
    throw std::runtime_error("required controller mapping is missing");
}
```

### 4-2. Control tick

- 아래는 controller-owned symbol을 포함한 **pseudocode**
- `Jp`, `Jr`: setup에서 `3 x model.nv`로 미리 sizing
- `JdvP`, `JdvR`: `dt::Math::Vector3<dtDynReal>`. `Jdot * qvel`의 linear/angular 3-vector

```text
WriteEstimatorStateInto(data.qpos, data.qvel); // controller-owned mapping
ComputeDynamicsCCRBA(model, data, gravity, wbcTermMask);
ComputeSiteJacobianTerms(model, data, foot,
                         &Jp, &Jr, nullptr, nullptr, &JdvP, &JdvR);
SolveControllerProblem(data.Mmat, data.nonlinTorq,
                       Jp, Jr, JdvP, JdvR); // controller-owned solver
```

> **주의 — setup-time vs realtime 경계**
>
> tick 안에서 금지: name lookup, XML, validation, compilation, `Resize()`.
>
> caller-owned Jacobian matrix 처리: strict zero-allocation path 필요 시 loop 진입 전에 미리 sizing.

- 개념(WBC handoff 식, 여러 instance/model update 원칙, LeoQuad 경계): [integration.md](manual/integration.md)

## 5. Realtime 적용 checklist

### 5-1. Setup phase (loop 밖에서 수행)

1. `LoadXmlFile()`/`DecodeXml()` 또는 programmatic API로 `dtDynSpec` 생성
2. `ValidateSpec()`/`CompileSpecChecked()` 실행
3. `dtDynData::Resize(model)` 실행
4. body/joint/site/actuator ID와 state address cache
5. caller-owned Jacobian/output matrix pre-size
6. controller/estimator convention에 따른 explicit state initialization

### 5-2. Realtime phase

1. estimator state를 기존 `qpos`/`qvel` storage에 기록
2. 필요한 최소 `ComputeDynamicsRNEA()` 또는 reduced-mask `ComputeDynamicsCCRBA()` 호출
3. active task에 필요한 Jacobian만 계산
4. 결과를 controller task structure에 전달

### 5-3. Fixed-layout physical parameter update

- `UpdateSpec(spec, model)`은 body inertial/pose, joint parameter, site pose, actuator gear처럼 compiled layout을 바꾸지 않는 physical parameter만 갱신한다.
- editable spec 생성·`Edit*`·외부 입력 validation은 loop 밖에서 끝낸다. update는 reader가 없는 control-safe boundary에서만 실행하며, concurrent kinematics/dynamics reader와 model을 공유하지 않는다.
- `false`는 topology 또는 address/mapping incompatibility를 뜻하며 model은 유지된다. 성공 후 기존 `dtDynData` output은 stale이므로 필요한 `ForwardKinematics`/`ComputeDynamics*()`를 다시 실행한다.
- allocation-free update path는 regression으로 확인했지만, model 전체 preflight와 cache refresh의 worst-case time은 target CPU에서 별도로 계측한다. hard-realtime deadline proof를 대신하지 않는다.

> **주의 — setup-time vs realtime 경계**
>
> loop 배제 대상: XML I/O, validation, compile, resize, name lookup, model hot-swap. `UpdateSpec()`은 위 fixed-layout·exclusive-access·worst-case-time 조건을 만족하는 경우만 예외다.

- 개념(allocation evidence, 여러 evaluation instance, timing 해석): [realtime.md](manual/realtime.md)

## 6. 검증 실행 방법

### 6-1. Reference/artifact 검증

```bash
python3 tools/validate_reference_benchmark.py \
  --references /path/to/accepted-reference.csv \
  --runtime-csv docs/figures/runtime_by_fixture.csv

python3 tools/validate_benchmark_results.py \
  --native /path/to/native.csv \
  --stages /path/to/stages.csv \
  --generated docs/figures
```

### 6-2. Package validation gate

```bash
cmake --preset validation
cmake --build --preset validation -- -j2
ctest --preset validation --output-on-failure
tools/validate_package.sh --skip-benchmarks
python3 tools/validate_docs.py
```

### 6-3. GitLab static-analysis gate

- authoritative execution: GitLab `polyspace-job` job과 job artifact `reports/polyspace/polyspace/result_list.txt`
- analysis scope: test/benchmark/reference target을 끈 production `dtDynamics` compile database
- 결과 범위: dtDynamics 자체 소스로 한정. 시스템 헤더, ART Framework prefix, 고정 SHA dtMath checkout(`.ci/dtmath`)은 `polyspace/polyspace.opts`에서 제외
- 설정 단계 heap deviation(잠정): 21.6.1/21.6.2/21.6.3은 `polyspace/setup_phase_deviation.txt`에 적은 설정 단계 파일에 한해 인정. 실시간 경로의 21.6.x 0건과 `TestRealtimeComputePathPerformsNoHeapAllocation` 무할당 테스트를 대체 증거로 사용. 목록 밖 파일에서 나온 같은 rule은 실패. 팀 정책 결정 전까지의 잠정 방침
- checker profile: production library compile database에 `polyspace/checkers.xml`로 고정한 Bug Finder 237개와 MISRA C++:2023 179개 rule 활성화. 활성화 항목 수이며, 실제로 실행된 체커는 Polyspace 로그의 경고(찾을 수 없거나 옵션이 없어 무시된 체커)를 함께 확인. exact profile은 `tools/test_polyspace_checker_profile.py`가 검증
- execution: `ci/polyspace.sh`가 cache 결과, 기존 configure options, 기존 report를 제거하고 현재 compile database에서 nonempty configure options를 새로 만든다. 이후 새 canonical `result_list.txt`만 `tools/polyspace_result_gate.py`에 전달. parser exit code가 GitLab job exit를 결정하며 Polyspace Access 업로드는 이 판정을 바꾸지 않음
- canonical schema: TSV header는 `ID`부터 `Col`까지 정해진 16개 column을 같은 순서로 모두 포함해야 함. 축약·순서 변경·추가 column은 즉시 실패
- TSV field boundary: 필드 경계는 literal TAB만 사용한다. 큰따옴표(`"`)는 일반 데이터로 취급하며 따옴표 처리나 행 결합에 사용하지 않는다. 따라서 큰따옴표로 감싼 TAB도 추가 field로 거부한다.
- raw-defect-zero policy: 모든 canonical export row는 `Color=Green`이고 status가 blank 또는 `Unreviewed`여야 한다. `Justified`, `No action planned`, `Not a defect`, non-Green row, malformed TSV, `File does not compile`은 모두 gate 실패다. 예외: 21.6.1/21.6.2/21.6.3 중 `polyspace/setup_phase_deviation.txt`에 적은 설정 단계 파일의 행은 잠정 deviated로 따로 집계하며, 그 밖의 행은 모두 unresolved다.

```bash
python3 tools/test_polyspace_checker_profile.py
python3 tools/test_polyspace_result_gate.py
python3 tools/test_polyspace_export_contract.py
python3 tools/test_polyspace_transport.py
```

- 개념(setup/native/external validation 의미, artifact 판정 기준): [validation.md](manual/validation.md)

## 7. Benchmark 재현 방법

### 7-1. CSV 생성

```bash
cd /path/to/dtDynamics
cmake --preset validation
cmake --build --preset validation -- -j2
export DTDYNAMICS_BENCHMARK_CPU=0
export DTDYNAMICS_BENCHMARK_BATCHES=7
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamics > /tmp/native.csv
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamicsStages > /tmp/stages.csv
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamicsReferences > /tmp/references.csv
```

### 7-2. Table/figure 생성 및 검증

```bash
python3 tools/plot_benchmark_results.py \
  --native /tmp/native.csv \
  --stages /tmp/stages.csv \
  --references /tmp/references.csv \
  --out /tmp/dtdyn_figures \
  --tables-out /tmp/dtdyn_tables

python3 tools/validate_benchmark_results.py \
  --native /tmp/native.csv \
  --stages /tmp/stages.csv \
  --generated /tmp/dtdyn_figures

python3 tools/validate_reference_benchmark.py \
  --references /tmp/references.csv \
  --runtime-csv /tmp/dtdyn_figures/runtime_by_fixture.csv
```

- 산출물: `runtime_by_fixture.{csv,png}`, `scaling_by_dof.{csv,png}`, `stage_breakdown.{csv,png}` (CSV는 mean/median/p95/p99 보존, figure는 median 사용)

### 7-3. GitLab Markdown 문서 확인

- 원칙: Markdown source를 GitLab 저장소에서 직접 열람, generated HTML/PDF artifact와 GitLab Pages 미사용
- CI `markdown-docs` job: `python3 tools/validate_documentation_content.py`, `python3 tools/validate_docs.py` 실행
- GitLab renderer는 fenced `math` block과 `$` backtick-delimited inline math를 직접 렌더링
- Markdown viewer 렌더링은 GitLab 웹 화면에서 확인; CI는 source/link/math-marker/SVG diagram 정합성만 fail-closed 검증
- 개념(측정 scope, 해석 경계, evidence snapshot): [benchmarks.md](manual/benchmarks.md)

## 8. Markdown 문서·Release 절차

### 8-1. Markdown 문서 검증

```bash
cd /path/to/dtDynamics
python3 tools/validate_documentation_content.py
python3 tools/validate_docs.py
```

- Markdown source, relative link, GitLab math marker, SVG diagram, current narrative invariant 검증
- HTML/PDF 생성·generated HTML·LaTeX·GitLab Pages 불필요

### 8-2. Release gate

```bash
cmake --preset validation
cmake --build --preset validation -- -j2
ctest --preset validation --output-on-failure

tools/validate_package.sh --skip-benchmarks
tools/validate_install_export.sh
python3 tools/validate_documentation_content.py
python3 tools/validate_docs.py
tools/audit_release_readiness.sh
```

> **주의 — Publication 경계**
>
> 검증 PASS ≠ 자동 publication consent. commit/push/tag/release 또는 외부 게시 전 필수 절차: 변경 범위와 evidence 제시, 명시적 승인 획득.
>
> source archive 생성 조건: clean committed tree와 exact `v0.1.3` tag 승인 이후만. archive version `0.1.3`은 `CMakeLists.txt` project version과 같고, exact `v0.1.3` tag가 현재 `HEAD`와 archive embedded commit ID를 모두 가리켜야 함.

- 개념(package surface 체크리스트, publication 경계 rationale): [release.md](manual/release.md)

## 9. Troubleshooting

| 증상                                            | 원인/확인                                                                             | 참고                                                                              |
| --------------------------------------------- | --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| `dtDynXmlError` 발생                            | malformed/duplicate/unresolved XML, invalid number, wrong arity, 잘못된 vector arity | `error.Code()`/`SourcePath()`/`Offset()` 확인 — [modeling.md](manual/modeling.md) |
| lookup이 `DTDYN_INVALID_INDEX` 반환              | 이름 오타, 아직 compile 안 된 spec에서 조회, 모델에 실제로 없는 body/joint/site/actuator              | 예외 대신 반환값이므로 반드시 확인 후 indexing — [modeling.md](manual/modeling.md)              |
| `IsCompatible(model)`이 `true`인데 결과가 이상함       | `IsCompatible()`은 storage dimension만 확인, model identity/topology/inertia 동일성 미보장  | model 교체 시 ID/address cache 재생성 — [modeling.md](manual/modeling.md)             |
| model 재compile 후 이상 동작                        | 차원이 같아도 cached ID/address가 stale할 수 있음                                            | `Resize()` 재검토 및 lookup 재수행 — [modeling.md](manual/modeling.md)                 |
| `find_package(dtDynamics 0.1.3 EXACT ...)` 실패 | installed package identity가 0.1.3이 아니거나 EXACT 불일치                                 | `tools/validate_install_export.sh` 재실행 — [release.md](manual/release.md)        |
| `Edit*` callback이 `logic_error`로 종료 | active edit 중 같은 spec의 nested edit/add/clear/assignment/swap을 시도함 | inner exception을 잡아도 outer transaction은 abort됨 — [modeling.md](manual/modeling.md) |
| 검증에서 external row가 `skipped`                  | 해당 quantity에 대응하는 Pinocchio/MuJoCo contract가 없음 (equivalent contract 없음)          | 정상 동작 (실패 아님) — [validation.md](manual/validation.md)                           |
| `qpos`를 0으로 초기화했더니 이상함                        | free/ball joint quaternion이 invalid할 수 있음 (package-level neutral initializer 없음)  | 명시적 초기화 필요 — [modeling.md](manual/modeling.md), [example.md](manual/example.md) |

## 10. 상세 설명 chapter와 관련 문서

- 1장 [개요](manual/overview.md)
- 2장 [모델과 데이터 수명주기](manual/modeling.md)
- 3장 [계산 pipeline](manual/computation.md)
- 4장 [최소 예제](manual/example.md)
- 5장 [Realtime resource model](manual/realtime.md)
- 6장 [검증](manual/validation.md)
- 7장 [Benchmark](manual/benchmarks.md)
- 8장 [Controller 통합](manual/integration.md)
- 9장 [API 지도](manual/api.md)
- 10장 [제한](manual/limitations.md)
- 11장 [Release 경계](manual/release.md)

| 독자                          | 우선 읽을 곳                                                                |
| --------------------------- | ---------------------------------------------------------------------- |
| library/controller consumer | 이 문서 2~4장 → chapter 8 Controller 통합 → chapter 9 API 지도                 |
| model author (XML 작성자)      | 이 문서 2장 → chapter 2 모델과 데이터 수명주기 → `docs/guides/model_guide/`          |
| package/release maintainer  | 이 문서 6~8장 → chapter 6 검증 → chapter 7 Benchmark → chapter 11 Release 경계 |

- 이 문서: how-to/실행 절차 전용 — 중복된 상세 contract 미유지
- API, coordinate, benchmark 설명: 위 canonical chapter가 소유
- 과거 release evidence: `docs/release-notes/`에 별도 보존

## 관련 문서

- 상세 설명 chapter 목차: `docs/manual/index.md`
- model guide: `docs/guides/model_guide/`
- usage/migration guide: `docs/guides/usage_guide/`
- benchmark evidence: [docs/benchmark-results.md](benchmark-results.md)
- release note: `docs/release-notes/v0.1.3.md`
- Markdown 문서 gate: Python 3. Diagram source 변경 시 `tools/render_diagrams.sh`와 Graphviz `dot` 사용
- `v0.1.0-rc1`~`v0.1.2` release note: 과거 기준 보존 (현재 `0.1.3`의 근거로 소급 해석 금지)
