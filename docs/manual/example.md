# 최소 end-to-end 예제 {#dtdynamics_example}

Author: Joonhee Jo

- 이 예제가 보여주는 순서: **load → validate → compile → resize → ID cache → state 입력 → compute**
- 성격: simulator나 torque controller 예제가 아닌 dynamics ingredient 사용 예제

## 입력 XML

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

## Program

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

## 실패 진단

- 파일 XML semantic error 보존 정보: exact source path와 zero-based byte offset

```cpp
catch (const dtDynXmlError &error)
{
    Log(error.Code(), error.SourcePath(), error.Offset(), error.what());
}
```

- programmatic mutation으로 topology 오류 발생 시: `ValidateSpec(spec, &error)`가 `error.code`, `error.object`, `error.index`, `error.message` 채움
- 금지 사항: `what()`이나 process exit code만으로 machine category 추론

## `CompileSpec`은 언제 쓰는가

- 위 예제처럼 외부 파일·editable spec을 다룰 때: `CompileSpecChecked()`가 기본
- raw `CompileSpec()` 유지 목적: package 내부 fixture처럼 validation이 별도로 끝난 trusted setup path의 호환성

## Neutral state 주의

- 미제공 항목: free/ball joint의 유효 quaternion과 robot rest pose를 자동으로 정하는 package-level initializer
- `qpos`를 모두 zero로 채울 때 위험: floating-base quaternion이 invalid할 수 있음
- 초기화 책임: estimator/controller가 compiled address와 model convention에 따라 state를 명시적으로 기록
