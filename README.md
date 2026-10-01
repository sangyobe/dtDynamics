# dtDynamics

- `dtDynamics`: 제어기와 WBC가 사용하는 강체 동역학 **재료(ingredient)** 계산 독립 패키지
- 제공 항목: 질량행렬, 중력·코리올리·비선형 토크, body/site Jacobian과 시간미분 항, CoM, centroidal momentum 항
- 미제공 항목: 접촉 해석, 시뮬레이션 적분, 최종 토크 결정
- 현재 패키지 버전: **0.1.3**
- production library 의존성: `dtMath` 단일
- MuJoCo 3.8.0, Pinocchio 3.9.0: `.validation_deps` 아래의 검증·benchmark 전용 참조 구현

## 가장 짧은 사용 흐름

- 설정 단계와 realtime 단계 분리

```text
LoadXmlFile / DecodeXml
        -> ValidateSpec
        -> CompileSpecChecked
        -> dtDynData::Resize
        -> BodyId / JointId / SiteId / ActuatorId cache
        -> qpos/qvel 입력과 ComputeDynamics* 실행
```

- 외부 파일 또는 사용자가 수정한 `dtDynSpec`: `CompileSpecChecked()` 사용
- 기존 `CompileSpec()`: 이미 신뢰 경계 안에서 검증된 setup data용 호환 API
- 이름 조회 실패: 예외 대신 `DTDYN_INVALID_INDEX` 반환 → ID를 배열 인덱스로 쓰기 전 확인 필수

## 디렉터리

- `include/dtDynamics/`: `dt::Dynamics` public API
- `src/`: `dtMath` 기반 production 구현
- `test/`: native, MuJoCo, Pinocchio 참조 검증
- `benchmark/`: runtime/stage/reference benchmark
- `tools/`: package, install/export, 문서, benchmark 검증 도구
- `docs/`: manual, guide, 결과, release note

## 의존성과 설치 위치

- 기본 ART Framework prefix: `${ARTF_INSTALL_DIR:-$HOME/.local/ART_Framework}`
- MuJoCo/Pinocchio: 이 production prefix에 설치 금지

```bash
export ARTF_INSTALL_DIR="${ARTF_INSTALL_DIR:-$HOME/.local/ART_Framework}"
cmake --preset release
cmake --build --preset release -- -j2
cmake --install build/release --prefix "${ARTF_INSTALL_DIR}"
```

- consumer 요구 사항: 정확한 package identity와 canonical target

```cmake
find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE dt::Dynamics)
```

- `dtDynamicsConfigVersion.cmake`, public header version, installed library: 모두 같은 후보에서 생성 필수
- 새 library와 오래된 header 혼용 시: `dtDynModel`/`dtDynData` layout 불일치 위험

## 검증

- 전체 검증 의존성 준비

```bash
tools/install_validation_deps.sh --all
cmake --preset validation
cmake --build --preset validation -- -j2
ctest --preset validation --output-on-failure
```

- package 경계와 installed consumer 확인

```bash
tools/validate_package.sh
tools/validate_install_export.sh
tools/audit_release_readiness.sh
```

- `validate_install_export.sh`의 fail-closed 확인 항목:
  - `find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)` 성공
  - exported target이 `dt::Dynamics`이고 legacy target 없음
  - production CMake export에 MuJoCo/Pinocchio 없음
  - installed public-header version과 current SVG package claim이 `0.1.3`
  - fresh downstream consumer의 configure, compile, link, run

- benchmark 결과: wall-clock byte replay 비보장
- 채택된 raw run과 게시 artifact의 provenance: byte identity로 확인
- 새 실행 확인 항목: schema, fixture/state/scope coverage, status, percentile ordering
- GitLab Polyspace gate: production library compile database만 분석하고, Bug Finder와 MISRA C++:2023 subset `6.8.2`, `8.18.1`, `11.6.2`, `25.5.2`, `25.5.3`을 적용
- static-analysis 통과 기준: canonical `result_list.txt`의 모든 row가 `Color=Green` 및 blank/`Unreviewed` status. `Justified`를 포함한 reviewed row, non-Green, malformed export, compile failure는 모두 실패

## Realtime 경계

- setup 단계: `ValidateSpec`, XML I/O, compilation, `dtDynData::Resize`, name lookup
- control loop: 미리 할당한 `dtDynData`와 caller-owned output 사용
- 문서의 allocation 0 범위: benchmark가 계측한 C++ `operator new` 경로에 한정 (`malloc` 계열, OS, logger, 임의의 unsized output까지 포괄하는 보장 아님)

## 문서

- 시작점: `docs/manual.md`
- 현재 benchmark evidence: `docs/benchmark-results.md`
- 현재 release note: `docs/release-notes/v0.1.3.md`
- model guide: `docs/guides/model_guide/`
- usage/migration guide: `docs/guides/usage_guide/`
- 문서 gate: `python3 tools/validate_docs.py` (Markdown 링크·수식 marker·SVG diagram 검증), `python3 tools/validate_documentation_content.py` (현재 narrative invariant 검증)
- `v0.1.0-rc1`~`v0.1.2` release note: 과거 기준 보존 (현재 `0.1.3`의 근거로 소급 해석 금지)

## 라이선스

Copyright (c) 2026 RoboticsLab ART. All rights reserved. See [`LICENSE`](LICENSE).
