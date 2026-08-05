# 검증

Author: Joonhee Jo

@dotfile diagrams/validation_refs.dot

- Validation dependency 위치: production package 경계 밖
- production public header/export 미포함 항목: MuJoCo·Pinocchio API

## Setup validation

- `ValidateSpec()`와 `CompileSpecChecked()` 원칙: public mutable arrays 비신뢰
- setup에서 검사하는 항목: ownership, parent/root topology, duplicate name, joint type/axis, finite physical data, quaternion/rotation, inertia, index/dimension overflow
- 지원 범위 안에서 scan 성격: linear, dynamics hot loop 밖

- XML 오류 보고 방식: `dtDynXmlError`의 typed code, exact source path, zero-based byte offset으로 lexical·structural·semantic failure 보고
- `what()` 성격: human detail (machine interface 아님)

## Native validation

- Native test 확인 항목: dimensions, addresses, world/fixed body, multi-root, Jacobian shape, cross-path residual, zero-DoF resize, lookup copy/rebuild, malformed XML negative path
- 예: mass symmetry 확인 방식

\f[
\epsilon_M=\lVert M-M^\mathsf{T}\rVert_\infty
\f]

## External numerical reference

- Reference comparison 선행 작업: coordinate, frame, spatial row order 명시적 변환

\f[
\epsilon_y=\lVert y_{dtDynamics}-T(y_{ref})\rVert.
\f]

- MuJoCo 3.8.0 참조 대상: overlapping smooth-dynamics/indexing quantity
- Pinocchio 3.9.0 참조 대상: independent rigid-body quantity
- equivalent contract 없는 row 처리: 통과로 미집계, `skipped` diagnostic으로 기록
- 실패 조건: actual, expected, residual, relative error, RMS, tolerance 중 하나라도 non-finite

## Timing independence

- MuJoCo/Pinocchio measured interval 포함 범위: pinned external call set만
- interval 내 금지 사항: native validation, bridge comparison 실행
- 검사 방식: `benchmark_metadata` guard와 zero bridge-counter
- external timing 해석: same-output speed comparison이 아닌 contextual reference-library overhead

## Artifact validation

- 채택한 raw run 요구 사항: 72 timing key와 대응하는 seven-batch summary 72개
- promoted runtime CSV의 external row가 만족할 exact Cartesian product:
  - engine: MuJoCo, Pinocchio
  - fixture: 8개
  - state: 3개
  - 총 48개 unique key

- 거부 조건: row count만 맞고 key가 duplicate/missing인 경우
- 검사 항목: median/p95/p99 ordering, finite value, raw summary와 promoted CSV의 exact value agreement

```bash
python3 tools/validate_reference_benchmark.py \
  --references /path/to/accepted-reference.csv \
  --runtime-csv docs/figures/runtime_by_fixture.csv

python3 tools/validate_benchmark_results.py \
  --native /path/to/native.csv \
  --stages /path/to/stages.csv \
  --references /path/to/reference.csv \
  --generated docs/figures
```

## Current package gate

```bash
cmake --preset validation
cmake --build --preset validation -- -j2
ctest --preset validation --output-on-failure
tools/validate_package.sh --skip-benchmarks
tools/validate_docs.sh
```

- 문서에 적은 pass snapshot 성격: rerun을 미대체
- 재실행 조건: compiler, dependency, model, benchmark source 변경 시 같은 semantic gate 재실행
