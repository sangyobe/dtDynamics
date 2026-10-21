# dtDynamics

- `dtDynamics`: 제어기와 WBC가 사용하는 강체 동역학 **재료(ingredient)** 계산 독립 패키지
- 제공 항목: 질량행렬, 중력·코리올리·비선형 토크, body/site Jacobian과 시간미분 항, CoM, centroidal momentum 항
- 미제공 항목: 접촉 해석, 시뮬레이션 적분, 최종 토크 결정
- 현재 패키지 버전: **0.1.3**
- production library 의존성: `dtMath` 단일
- CI 고정 revision: `1b495575cf99beaf04a61df3074a73f73b01f09b`
- 다른 위치의 dtMath 사용: `cmake --preset release -DDTDYNAMICS_DTMATH_INCLUDE_DIR=/path/to/dtMath/include` (`dtMath/dtMath.h`를 포함하는 include root)
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
# Pinocchio archive는 headers·library·CMake config와 그 config에 필요한 CMake dependency closure를 포함한 pinocchio-3.9.0/ prefix 및 SHA-256을 함께 지정한다. 기존 prefix도 같은 archive와 SHA-256으로 provenance를 확인한다.
DTDYNAMICS_PINOCCHIO_ARCHIVE=/path/to/pinocchio-3.9.0.tar.gz \
DTDYNAMICS_PINOCCHIO_SHA256=REPLACE_WITH_64_HEX_SHA256 \
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

- `validate_install_export.sh`의 선택 layout fail-closed 확인 항목 (`DTDYNAMICS_INSTALL_LIBDIR`, 기본 `lib`):
  - `find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)` 성공
  - exported target이 `dt::Dynamics`이고 legacy target 없음
  - production CMake export에 MuJoCo/Pinocchio 없음
  - installed public-header version과 current SVG package claim이 `0.1.3`
  - `PATHS ... NO_DEFAULT_PATH`로 선택한 config만 찾고, imported library가 선택한 staged `${prefix}/${install_libdir}` 아래에 있으며 package include가 staged prefix 아래에 있는지 확인
  - staged prefix를 이동한 뒤에도 fresh downstream consumer가 configure, compile, link, run
- `validate_package.sh` 기본 matrix는 위 validator를 `lib`와 `lib64` 각각에 독립 실행

- benchmark 결과: wall-clock byte replay 비보장
- 채택된 raw run과 게시 artifact의 provenance: byte identity로 확인
- 새 실행 확인 항목: schema, fixture/state/scope coverage, status, percentile ordering
- GitLab Polyspace gate: production library compile database만 분석하고, `polyspace/checkers.xml`에 고정한 Bug Finder 237개와 MISRA C++:2023 179개 rule을 적용. exact profile은 `tools/test_polyspace_checker_profile.py`가 검증
- static-analysis 실행 경로: `ci/polyspace.sh`가 cache 결과, 기존 configure options, 기존 report를 제거한 뒤 현재 compile database에서 nonempty configure options를 새로 만들고, 새 canonical `result_list.txt`만 `tools/polyspace_result_gate.py`에 전달한다. parser exit code를 GitLab job exit로 사용하며 Polyspace Access 업로드는 release pass/fail을 바꾸지 않음
- canonical schema: TSV header는 `ID`부터 `Col`까지 정해진 16개 column을 같은 순서로 모두 포함해야 함. 축약·순서 변경·추가 column은 raw-defect-zero 증거가 아님
- TSV field boundary: 필드 경계는 literal TAB만 사용한다. 큰따옴표(`"`)는 일반 데이터로 취급하며 따옴표 처리나 행 결합에 사용하지 않는다. 따라서 큰따옴표로 감싼 TAB도 추가 field로 거부한다.
- static-analysis 통과 기준: canonical `result_list.txt`의 모든 row가 `Color=Green` 및 blank/`Unreviewed` status. `Justified`를 포함한 reviewed row, non-Green, malformed export, compile failure는 모두 실패. 예외: 21.6.1/21.6.2/21.6.3 중 `polyspace/setup_phase_deviation.txt`에 적은 설정 단계 파일의 행은 잠정 deviated로 따로 집계하며, 그 밖의 행은 모두 unresolved다.

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
