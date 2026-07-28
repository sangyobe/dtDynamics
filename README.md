# dtDynamics

`dtDynamics`는 제어기와 WBC가 사용하는 강체 동역학 **재료(ingredient)** 를 계산하는 독립 패키지다. 질량행렬, 중력·코리올리·비선형 토크, body/site Jacobian과 시간미분 항, CoM, centroidal momentum 항을 제공하지만 접촉 해석, 시뮬레이션 적분, 최종 토크 결정은 수행하지 않는다.

현재 패키지 버전은 **0.1.1**이다. production library는 `dtMath`에만 의존하며, MuJoCo 3.8.0과 Pinocchio 3.9.0은 `.validation_deps` 아래의 검증·benchmark 전용 참조 구현이다.

## 가장 짧은 사용 흐름

설정 단계와 realtime 단계를 분리한다.

```text
LoadXmlFile / DecodeXml
        -> ValidateSpec
        -> CompileSpecChecked
        -> dtDynData::Resize
        -> BodyId / JointId / SiteId / ActuatorId cache
        -> qpos/qvel 입력과 ComputeDynamics* 실행
```

외부 파일이나 사용자가 수정한 `dtDynSpec`에는 `CompileSpecChecked()`를 사용한다. 기존 `CompileSpec()`는 이미 신뢰 경계 안에서 검증된 setup data를 위한 호환 API다. 이름 조회 실패는 예외가 아니라 `DTDYN_INVALID_INDEX`를 반환하므로, ID를 배열 인덱스로 쓰기 전에 반드시 확인한다.

## 디렉터리

- `include/dtDynamics/`: `dt::Dynamics` public API
- `src/`: `dtMath` 기반 production 구현
- `test/`: native, MuJoCo, Pinocchio 참조 검증
- `benchmark/`: runtime/stage/reference benchmark
- `tools/`: package, install/export, 문서, benchmark 검증 도구
- `docs/`: manual, guide, 결과, release note

## 의존성과 설치 위치

기본 ART Framework prefix는 `${ARTF_INSTALL_DIR:-$HOME/.local/ART_Framework}`다. MuJoCo/Pinocchio를 이 production prefix에 설치하지 않는다.

```bash
export ARTF_INSTALL_DIR="${ARTF_INSTALL_DIR:-$HOME/.local/ART_Framework}"
cmake --preset release
cmake --build --preset release -- -j2
cmake --install build/release --prefix "${ARTF_INSTALL_DIR}"
```

consumer는 정확한 package identity와 canonical target을 요구한다.

```cmake
find_package(dtDynamics 0.1.1 EXACT CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE dt::Dynamics)
```

`dtDynamicsConfigVersion.cmake`, public header의 Doxygen version, installed library가 모두 같은 후보에서 생성되어야 한다. 새로운 library와 오래된 header를 섞으면 `dtDynModel`/`dtDynData` layout이 달라질 수 있다.

## 검증

전체 검증 의존성을 준비하려면 다음을 실행한다.

```bash
tools/install_validation_deps.sh --all
cmake --preset validation
cmake --build --preset validation -- -j2
ctest --preset validation --output-on-failure
```

package 경계와 installed consumer를 확인한다.

```bash
tools/validate_package.sh
tools/validate_install_export.sh
tools/audit_release_readiness.sh
```

`validate_install_export.sh`는 다음을 fail-closed로 확인한다.

- `find_package(dtDynamics 0.1.1 EXACT CONFIG REQUIRED)` 성공
- exported target이 `dt::Dynamics`이고 legacy target이 없음
- production CMake export에 MuJoCo/Pinocchio가 없음
- installed public-header version과 current SVG package claim이 `0.1.1`
- fresh downstream consumer의 configure, compile, link, run

benchmark 결과는 wall-clock byte replay를 보장하지 않는다. 채택된 raw run과 게시 artifact의 provenance는 byte identity로 확인하고, 새 실행은 schema, fixture/state/scope coverage, status, percentile ordering을 확인한다.

## Realtime 경계

`ValidateSpec`, XML I/O, compilation, `dtDynData::Resize`, name lookup은 setup 단계다. control loop에서는 미리 할당한 `dtDynData`와 caller-owned output을 사용한다. 문서의 allocation 0은 benchmark가 계측한 C++ `operator new` 경로에 한정되며 `malloc` 계열, OS, logger, 임의의 unsized output까지 포괄하는 보장은 아니다.

## 문서

- 시작점: `docs/manual.md`
- 현재 benchmark evidence: `docs/benchmark-results.md`
- 현재 release note: `docs/release-notes/v0.1.1.md`
- model guide: `docs/guides/model_guide/`
- usage/migration guide: `docs/guides/usage_guide/`

문서 gate `tools/validate_docs.sh`에는 Doxygen, Graphviz, Python 3와 guide PDF 내용·page·font를 검사하는 Poppler 도구(`pdftotext`, `pdfinfo`, `pdffonts`)가 필요하다.

`v0.1.0-rc1`~`rc3` release note는 과거 기준을 보존한다. 현재 `0.1.1`의 근거로 소급해 해석하지 않는다.

## 라이선스

Copyright (c) 2026 RoboticsLab ART. All rights reserved. See [`LICENSE`](LICENSE).
