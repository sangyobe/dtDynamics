# Release와 package boundary

Author: Joonhee Jo

- package identity 완결 조건: source version 변경만으로는 불충분
- release candidate 요구 사항: source, generated config, public headers, installed docs, library, fresh consumer가 동일 identity 보유

## Required gate

```bash
cmake --preset validation
cmake --build --preset validation -- -j2
ctest --preset validation --output-on-failure

tools/validate_package.sh --skip-benchmarks
tools/validate_install_export.sh
tools/validate_docs.sh
tools/audit_release_readiness.sh
```

- 0.1.1 consumer 사용 방식:

```cmake
find_package(dtDynamics 0.1.1 EXACT CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE dt::Dynamics)
```

## 확인할 package surface

- `dtDynamicsConfigVersion.cmake`가 exact `0.1.1`을 수용하고 wrong exact version을 거부
- installed public Doxygen version과 current SVG package claim이 `0.1.1`
- source/isolated/canonical library provenance와 installed headers가 같은 candidate에서 생성됨
- production export에 MuJoCo/Pinocchio reference library가 없음
- fresh out-of-tree consumer가 source include path 없이 compile/link/run
- canonical prefix의 unrelated `dtMath`/다른 ART package를 제거하지 않음

- Historical `v0.1.0` release note: 보존
- 일치 요구 대상: current package claim으로 검색되는 문구만 0.1.1과 일치

## Publication 경계

- 서로 다른 side effect: working tree validation, commit, push, tag, release, Jira/Confluence update
- 주의: 검증 PASS ≠ 자동 publication consent
- commit/push/tag/release 또는 외부 게시 전 필수 절차: 변경 범위와 evidence 제시, 명시적 승인 획득

- source archive 생성 조건: clean committed tree와 exact tag 승인 이후만
- 현재 0.1.1 candidate note 주장 범위: commit/tag 존재를 아직 주장하지 않음
