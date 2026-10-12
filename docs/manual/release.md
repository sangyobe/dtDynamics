# 11. Release와 package boundary

Author: Joonhee Jo

- package identity 완결 조건: source version 변경만으로는 불충분
- release candidate 요구 사항: source, generated config, public headers, installed docs, library, fresh consumer가 동일 identity 보유

## 11-1. Required gate

- gate 실행 명령: `docs/manual.md` 8장 "문서 생성·Release 절차" 참고
- 0.1.3 consumer 사용 방식:

```cmake
find_package(dtDynamics 0.1.3 EXACT CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE dt::Dynamics)
```

## 11-2. 확인할 package surface

- `dtDynamicsConfigVersion.cmake`가 exact `0.1.3`을 수용하고 wrong exact version을 거부
- installed public header version과 current SVG package claim이 `0.1.3`
- source/isolated/canonical library provenance와 installed headers가 같은 candidate에서 생성됨
- production export에 MuJoCo/Pinocchio reference library가 없음
- fresh out-of-tree consumer가 source include path 없이 compile/link/run
- `validate_package.sh` matrix가 `lib`·`lib64` staged install 각각에서 `validate_install_export.sh`를 실행. 선택한 config만 `PATHS ... NO_DEFAULT_PATH`로 찾고, imported library는 선택한 `${prefix}/${install_libdir}` 아래에, package include는 staged prefix 아래에 있는지 확인
- GitLab `package-validation` job는 committed checkout에서 native CTest와 위 `lib`·`lib64` install/export/relocation consumer matrix를 실행. MuJoCo/Pinocchio reference suite는 runner image에 없는 validation-only dependency이므로 이 job에서 제외하며, release evidence는 별도 full validation으로 보강
- staged prefix를 이동한 뒤에도 fresh consumer가 configure/compile/link/run
- 게시 model/usage guide PDF는 각 guide의 tracked Markdown·theme·참조 SVG/PNG와 개별 `*.provenance.json` SHA-256 manifest가 모두 일치할 때만 release artifact로 인정
- canonical prefix의 unrelated `dtMath`/다른 ART package를 제거하지 않음

- static-analysis release condition: GitLab Polyspace artifact가 raw defect 0 (Green + blank/`Unreviewed` only), `Justified` 포함 reviewed row 0, compile failure 0
- canonical schema: TSV header는 `ID`부터 `Col`까지 정해진 16개 column을 같은 순서로 모두 포함해야 하며, 축약·순서 변경·추가 column은 release evidence가 아님
- TSV field boundary: 필드 경계는 literal TAB만 사용한다. 큰따옴표(`"`)는 일반 데이터로 취급하며 따옴표 처리나 행 결합에 사용하지 않는다. 따라서 큰따옴표로 감싼 TAB도 추가 field로 거부한다.
- execution proof: `ci/polyspace.sh`가 기존 configure options를 제거하고 현재 compile database에서 nonempty options를 새로 만든 뒤, 새 canonical `result_list.txt`를 `tools/polyspace_result_gate.py`에 전달하고 parser exit code를 job exit로 사용한 GitLab artifact여야 함
- Historical `v0.1.0`~`v0.1.2` release note: 보존
- 일치 요구 대상: current package claim으로 검색되는 문구만 0.1.3와 일치

## 11-3. Publication 경계

- 서로 다른 side effect: working tree validation, commit, push, tag, release, Jira/Confluence update
- 주의: 검증 PASS ≠ 자동 publication consent
- commit/push/tag/release 또는 외부 게시 전 필수 절차: 변경 범위와 evidence 제시, 명시적 승인 획득

- source archive 생성 조건: clean committed tree와 exact `v0.1.3` tag 승인 이후만
- archive version `0.1.3`은 `CMakeLists.txt` project version과 같아야 하며, exact `v0.1.3` tag가 현재 `HEAD`를 가리켜야 함
- 생성한 archive의 embedded commit ID도 exact `v0.1.3` tag commit과 일치해야 함
- `v0.1.3` tag는 package/install/docs/static-analysis gate를 완료한 release commit에만 생성

## 관련 문서

- 이전 chapter: [제한](limitations.md)
- 전체 목차: [전체 목차](index.md)
- top-level landing: `docs/manual.md`
- 실행 절차(release gate 명령): `docs/manual.md` 8장
