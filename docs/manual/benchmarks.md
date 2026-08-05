# Benchmarks and Resource Reports {#dtdynamics_benchmarks}

Author: Joonhee Jo

- 이 chapter가 runtime 숫자보다 먼저 정의하는 것: **측정 scope와 해석 경계**
- native dynamics, independently prepared stage, MuJoCo/Pinocchio reference 성격: 서로 다른 호출 집합
- 금지 사항: median을 더해 다른 row를 만들거나 same-output speed ratio로 해석
- 각 batch 수행 절차: fresh `dtDynData`, deterministic state, prerequisite preparation 이후 warmup과 measurement

- 채택 artifact 검증 대상: raw CSV, seven-batch summary, normalized CSV, PNG
- reference output 통과 조건: exact engine × fixture × state key coverage, unique key, finite value, `median <= p95 <= p99`, raw summary와 promoted value agreement
- wall-clock rerun의 byte identity: 비요구 (단, candidate figure/CSV는 자신이 기록한 accepted raw run과 일치 필요)

- benchmark suite 보고 대상: runtime과 resource-consumption diagnostics 모두

## Main binaries

- `benchmark_dtDynamics`: dynamics-only native scope 3종 (`native_rnea_com_pos`, `native_ccrba_reduced`, `native_ccrba_all`) + setup/off-loop `dtDynModel_resource`와 `dtDynData_resource` row
- `benchmark_dtDynamicsStages`: FK, CCRBA substage, Jacobian, WBC-style reduced update path의 per-stage timing
- `benchmark_dtDynamicsReferences`: 비교 가능한 quantity에 대한 MuJoCo, Pinocchio reference timing/error row

## Benchmark scopes

### Part A: native dynamics and external references

- Part A 구성: 모든 fixture/state에 대한 package-native dynamics 호출 3종
  - `native_rnea_com_pos`: `ComputeDynamicsRNEA(model, data, gravity, DTDYN_RNEA_TERM_COM_POS)` 호출
  - `native_ccrba_reduced`: `ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_COM_POS | DTDYN_CCRBA_TERM_CMAMAT | DTDYN_CCRBA_TERM_CENT_MOM)` 호출
  - `native_ccrba_all`: `ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL)` 호출

- 동일 Part A timing figure 포함 대상: MuJoCo와 Pinocchio reference call sequence
- 해석 원칙: 이 라이브러리들은 동일한 CCRBA output contract를 노출하지 않으므로 figure는 contextual library-overhead timing이며 same-output speed ratio 아님
- 비교 가능 수치 정의: status-bearing reference-error row
- Release-active metadata counter 요구 사항: 각 external timing scope 동안 zero native validation run
- Native comparison run 실행 시점: external timing section 이후에만

- native CSV `scope` column 역할: native row 3종 구분
- `scope` 없는 historical CSV 해석: full CCRBA로 간주
- RNEA 보고 방식: centroidal-matrix와 CoM-velocity consistency를 `nan`으로 보고
- reduced CCRBA 보고 방식: centroidal momentum 검증, CoM-velocity consistency는 `nan`
- full CCRBA 보고 방식: 둘 다 검증

- deterministic state label 구분 기준: configuration과 velocity coverage 분리
  - `nominal`: model-default position, identity free/ball orientation, zero velocity
  - `asymmetric`: nonuniform position/orientation perturbation 적용, velocity는 zero 유지
  - `nonzero_velocity`: 다른 perturbed pose + nonzero generalized velocity로 velocity-dependent dynamics 활성화 (asymmetric pose에 velocity만 단순 추가한 것 아님)
- Reference adapter 적용 방식: engine-specific coordinate mapping을 통해 위 semantics 적용

- `fixed_body_heavy`: 1-DOF fixed-topology stress fixture
  - canonical 11-body morphology 구성: world-fixed base, nested fixed mount 8개, terminal revolute link 1개
  - Native dtDynamics와 MuJoCo: nested body topology 유지
  - hand-built Pinocchio reference: fixed offset을 terminal joint placement로 folding → body-recursion 작업량 감소
  - 결과 해석: timing은 representation-specific (same-morphology throughput 비교 아님)
  - 보고되는 `nbody=11`: canonical fixture metadata로 유지

- 확장된 morphology set 구성:
  - `multi_robot_fixed_base`: branching two-root fixed-base model, articulated branch, fixed sensor frame, 다중 site/actuator
  - `multi_robot_floating_base`: 대응하는 two-root free-joint topology
  - `serial_chain_10` / `serial_chain_30`: matched serial scaling point
- 목적: branching, floating-root, welded-body, DoF 증가 효과를 `nv` 단일 지표가 아니라 개별적으로 분리

- 반복되는 native scope/stage batch 공통 절차: fresh `dtDynData` construct/resize, deterministic state 복원, 해당 stage prerequisite 준비 이후 warmup과 measurement
- 원칙: stage row는 독립적으로 준비된 state에서 시작 → 다른 row 재구성을 위해 합산 금지

### Part B: LeoQuad controller scope

- controller-shaped stage 사용 mask:

```cpp
DTDYN_CCRBA_TERM_COM_POS |
DTDYN_CCRBA_TERM_CMAMAT |
DTDYN_CCRBA_TERM_CENT_MOM
```

- `leoquad_reduced_ccrba`: 이 dynamics 호출만 시간 측정
- `leoquad_wbc_update`: 선택한 root body와 foot site 4개에 대한 indexed Jacobian 추가
- External LeoQuad XML timing 성격: opt-in (LeoQuad를 installed-package dependency로 만들지 않음)

## Resource columns

- Latency table 보고 항목: mean, median, p95, p99 timing
- 두 tail statistic 성격: empirical order statistics
- runtime 기준 정렬된 sample을 \f$t^{\uparrow}_0, \ldots, t^{\uparrow}_{N-1}\f$라 할 때 보고값:

\f[
k_{95} = \mathrm{floor}(0.95(N - 1)), \qquad T_{95} = t^{\uparrow}_{k_{95}},
\qquad
k_{99} = \mathrm{floor}(0.99(N - 1)), \qquad T_{99} = t^{\uparrow}_{k_{99}}.
\f]

- Compute row 포함 항목:
  - `allocation_mean`: 측정된 compute iteration당 평균 counted C++ `operator new` / `operator new[]` allocation count
  - `allocation_max`: 측정된 compute iteration 중 최대 counted allocation count
  - `allocation_bytes_mean`: 측정된 compute iteration당 평균 counted allocated bytes
  - `allocation_bytes_max`: 측정된 compute iteration 중 최대 counted allocated bytes

- setup/off-loop resource section의 column 명명 방식: 유사
- `dtDynModel_resource` row 측정 대상: `CompileSpec(spec)`
- `dtDynData_resource` row 측정 대상: setup-time construction + `dtDynData data; data.Resize(model);` iteration (realtime compute iteration 아님)
- 단위: `mean_us`, `median_us`, `p95_us`, `p99_us`는 microsecond; `allocation_mean`, `allocation_max`는 counted allocation-call count; `allocation_bytes_mean`, `allocation_bytes_max`는 counted requested bytes
- 성격: benchmark C++ allocation-counter column (full malloc-family heap-profiler column 아님)

- Human-facing benchmark table 생략 조건: 표시 scope의 모든 row가 0일 때 allocation column 생략 가능
- 성격: presentation-only (raw CSV section은 allocation counter를 유지하여 allocation regression을 machine-checkable하게 보존)

## Recommended validation command

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

- 이후 fresh CSV로부터 table/figure 생성:

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

- plot 명령어가 나란히 작성하는 human figure 3종과 machine-readable source: `runtime_by_fixture.{csv,png}`, `scaling_by_dof.{csv,png}`, `stage_breakdown.{csv,png}`
- CSV 보존 값: mean, median, p95, p99
- figure 사용 값: median (tail 해석은 CSV와 table에 위임)

- multi-batch CSV 처리 방식: plotting tool이 per-batch mean/median/p95/p99 summary의 median을 human table/figure에 사용
- 미변경 대상: single-batch/historical CSV 동작
- raw CSV 보존 항목: 첫 timing row + batch-summary min/max spread
- 논리 CPU 1개 고정 효과: migration noise 감소 (단, CPU affinity는 frequency locking·realtime isolation 아님 → 게시 수치와 함께 governor/scheduling 조건 기록 필요)

## Publishing generated Doxygen in GitLab

- 결론: HTML commit 없이 GitLab CI job artifact로 즉시 게시 가능
- self-managed instance에 Pages가 활성화되어 있다면 private GitLab Pages site도 선호되는 안정적 URL
- 현재 상태: 이 project는 private Pages access를 요청했으나 Pages deployment는 미존재, API response는 instance-level Pages service 구성 여부를 증명하지 않음 → Pages URL에 의존하기 전 GitLab administrator에게 서비스 확인 필요

- 원칙: generated HTML은 Git에서 제외
- CI runner 필요 도구: Doxygen, Graphviz, 이 저장소의 local formula-image 구성이 요구하는 LaTeX/Ghostscript 도구
- 보장된 artifact job 형태:

```yaml
doxygen-artifact:
  stage: build
  script:
    - tools/validate_docs.sh
  artifacts:
    paths:
      - build/docs/doxygen/html/
```

- instance-level Pages 확인 후 추가 작업: pipeline `stages` 목록에 `deploy` 추가, generated directory 게시

```yaml
doxygen-pages:
  stage: deploy
  script:
    - tools/validate_docs.sh
    - mkdir -p public
    - cp -a build/docs/doxygen/html/. public/
  pages:
    publish: public
  rules:
    - if: '$CI_COMMIT_BRANCH == $CI_DEFAULT_BRANCH'
```

- 두 job 활성화 전 필수 작업: project-approved image 또는 runner toolchain 고정, 첫 pipeline 검증, artifact 다운로드, private Pages access 확인 (release artifact로 취급하기 전)

## Current benchmark evidence snapshot

- 더 긴 benchmark report와 generated figure 위치: @ref md_docs_benchmark_results
- 이 manual이 유지하는 범위: compact evidence snapshot만 (API contract가 stale performance number에 종속되지 않도록)

- 현재 canonical evidence 조건: CPU 0에 고정된 Release build, seven batch, batch당 warmup 20회·measured iteration 200회
- Human-facing 값 정의: seven batch-level mean/median/p95/p99 summary의 median (pooled-sample percentile 아님)

### Native floating-base quadruped snapshot

- 아래 compact Part A snapshot이 재현하는 대상: 상세 report의 floating-base quadruped row
- 각 cell 형식: `median / p95` (microsecond)

| State | RNEA + CoM | Reduced CCRBA | Full CCRBA | MuJoCo | Pinocchio |
|---|---:|---:|---:|---:|---:|
| nominal | 5.751 / 5.881 | 5.073 / 5.213 | 6.398 / 6.560 | 6.320 / 6.384 | 17.962 / 18.090 |
| asymmetric | 5.877 / 6.032 | 5.182 / 5.310 | 6.518 / 6.685 | 7.000 / 7.072 | 18.367 / 18.511 |
| nonzero_velocity | 6.049 / 6.187 | 6.907 / 7.045 | 8.166 / 8.293 | 7.039 / 7.119 | 18.364 / 18.507 |

- 같은 quadruped fixture의 setup-only resource row 보고값: compile median/p99 `5.263 / 11.240 us`, data-resize median/p99 `4.249 / 4.983 us`, fresh resize iteration당 counted C++ allocation call 31회
- setup 위치: realtime timing loop 밖
- 측정된 native dynamics/stage compute row 결과: counted C++ `operator new` allocation 0회 (direct malloc-family path는 미포함)

### LeoQuad real-XML snapshot

- 현재 CPU-affined seven-batch canonical real-XML scope의 reduced-CCRBA central median: nominal `5.162 us`, asymmetric `5.255 us`, nonzero-velocity `6.917 us`
- Reduced CCRBA + indexed Jacobian: `7.501 us`, `7.630 us`, `9.213 us`
- 별도 기록된 installed-package headless profile 성격: 유용한 end-to-end evidence (fresh dynamics-scope table에는 미포함)

- 이 값들의 성격: evidence snapshot (release-wide guarantee 아님)
- 영향 요인: compiler flag, CPU scheduling, model XML, 선택한 CCRBA term mask, indexed body/site task set, CPU affinity, frequency policy, realtime priority
- 새 비교 시 원칙: historic row와 current row 혼합 대신 benchmark binary로부터 CSV/table 재생성

- 현재 benchmark resource extension이 방출하는 setup row: `dtDynModel_resource`(측정 대상 `CompileSpec(spec)`), `dtDynData_resource`(측정 대상 fixture별 setup-time construction + `dtDynData::Resize(model)`, realtime compute row와 분리)
- 용도: initialization cost와 memory behavior 산정 (per-tick controller timing에 포함 금지)

- Setup inspection row timing column 단위: microsecond
- Human-facing table 생략 대상: all-zero compute allocation column (allocation counter는 regression/debug용으로 CSV에 유지)

- Allocation counter 구현 방식: benchmark executable에서 C++ `operator new` / `operator new[]`(aligned form 포함) override
- 용도: 측정 scope 내 C++ allocation 감지
- 한계: OS-level heap tracing의 완전한 대체 아님 — dtMath dynamic container는 `malloc`/`calloc`/`realloc`을 통해 allocate 가능, external library는 direct C allocation API나 allocator pool 사용 가능, allocator-internal behavior는 이 counter 범위 밖, 측정 scope 밖의 one-time runtime initialization은 이 column으로 특성화되지 않음
- final realtime heap certification 시 사용 권장: malloc-family interposer 또는 fail-on-allocation guard

- 현재 fresh-`dtDynData` resource row에서 allocation mean과 max가 같은 이유: 각 iteration이 같은 fixed model에 대해 fresh data object를 resize하고 같은 buffer set을 allocate
- 두 column 유지 이유: mean은 expected setup behavior 요약, max는 conditional/sporadic allocation regression 노출
- min allocation column 미추가 사유: 현재 deterministic setup path에서는 효용이 낮음 (향후 resource row가 reused-data나 conditional-allocation case를 포함하게 되면 추가 검토 가능)
