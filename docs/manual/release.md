# Release와 package boundary {#dtdynamics_release}

Author: Joonhee Jo

Source version 변경만으로 package identity가 끝나지 않는다. release candidate는 source, generated config, public headers, installed docs, library, fresh consumer가 같은 identity를 가져야 한다.

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

0.1.1 consumer는 다음을 사용한다.

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

Historical `v0.1.0` release note는 보존한다. current package claim으로 검색되는 문구만 0.1.1과 일치해야 한다.

## Publication 경계

working tree validation, commit, push, tag, release, Jira/Confluence update는 서로 다른 side effect다. 검증 PASS가 자동 publication consent를 뜻하지 않는다. commit/push/tag/release 또는 외부 게시 전에는 변경 범위와 evidence를 제시하고 명시적 승인을 받아야 한다.

source archive는 clean committed tree와 exact tag가 승인된 뒤에만 만든다. 현재 0.1.1 candidate note는 commit/tag가 아직 존재한다고 주장하지 않는다.
