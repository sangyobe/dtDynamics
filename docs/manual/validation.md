# 검증 {#dtdynamics_validation}

Author: Joonhee Jo

@dotfile diagrams/validation_refs.dot

Validation dependency는 production package 경계 밖에 둔다. production public header/export에는 MuJoCo·Pinocchio API가 들어가지 않는다.

## Setup validation

`ValidateSpec()`와 `CompileSpecChecked()`는 public mutable arrays를 신뢰하지 않는다. ownership, parent/root topology, duplicate name, joint type/axis, finite physical data, quaternion/rotation, inertia, index/dimension overflow를 setup에서 검사한다. 지원 범위 안에서 scan은 linear이며 dynamics hot loop에 들어가지 않는다.

XML은 `dtDynXmlError`의 typed code, exact source path, zero-based byte offset으로 lexical·structural·semantic failure를 보고한다. `what()`은 human detail이지 machine interface가 아니다.

## Native validation

Native test는 dimensions, addresses, world/fixed body, multi-root, Jacobian shape, cross-path residual, zero-DoF resize, lookup copy/rebuild, malformed XML negative path를 확인한다. 예를 들어 mass symmetry는

\f[
\epsilon_M=\lVert M-M^\mathsf{T}\rVert_\infty
\f]

로 확인한다.

## External numerical reference

Reference comparison은 coordinate, frame, spatial row order를 먼저 명시적으로 변환한다.

\f[
\epsilon_y=\lVert y_{dtDynamics}-T(y_{ref})\rVert.
\f]

MuJoCo 3.8.0은 overlapping smooth-dynamics/indexing quantity를, Pinocchio 3.9.0은 independent rigid-body quantity를 참조한다. equivalent contract가 없는 row를 통과로 세지 않고 `skipped` diagnostic으로 남긴다. actual, expected, residual, relative error, RMS, tolerance 중 하나라도 non-finite이면 실패한다.

## Timing independence

MuJoCo/Pinocchio measured interval에는 pinned external call set만 넣는다. native validation이나 bridge comparison을 interval 안에서 실행하지 않는다. `benchmark_metadata` guard와 zero bridge-counter로 이를 검사한다. external timing은 same-output speed comparison이 아니라 contextual reference-library overhead다.

## Artifact validation

채택한 raw run은 72 timing key와 대응하는 seven-batch summary 72개를 가져야 한다. promoted runtime CSV의 external row는 다음 exact Cartesian product를 만족한다.

- engine: MuJoCo, Pinocchio
- fixture: 8개
- state: 3개
- 총 48개 unique key

row count만 맞고 key가 duplicate/missing인 경우도 거부한다. median/p95/p99 ordering, finite value, raw summary와 promoted CSV의 exact value agreement를 검사한다.

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

문서에 적은 pass snapshot은 rerun을 대체하지 않는다. compiler, dependency, model, benchmark source가 바뀌면 같은 semantic gate를 다시 실행한다.
