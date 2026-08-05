# dtDynamics Manual {#mainpage}

Author: Joonhee Jo

- 이 manual: generated API 목록보다 먼저 읽는 conceptual contract
- 다루는 순서: model/data lifecycle → coordinate/frame → setup/realtime 경계 → validation scope → 각 함수 signature 연결

현재 package identity: **0.1.1**

## Manual 생성

```bash
cd /path/to/dtDynamics
tools/validate_docs.sh
```

- `tools/validate_docs.sh` 실행 항목: current-document content oracle, Doxygen warning scan, Graphviz generation
- generated HTML 위치: `build/docs/doxygen/html/` 아래

## Manual map

- @ref dtdynamics_overview "개요": package scope와 non-goal
- @ref dtdynamics_modeling "모델과 데이터 수명주기": XML, validation, compilation, lookup, coordinates
- @ref dtdynamics_computation "계산 pipeline": FK, RNEA, CCRBA, Jacobian, CoM, centroidal term
- @ref dtdynamics_example "최소 end-to-end 예제": load부터 compute까지
- @ref dtdynamics_realtime "Realtime resource model": setup/loop 분리와 allocation evidence 범위
- @ref dtdynamics_validation "검증": native/external reference와 fail-closed artifact gate
- @ref dtdynamics_benchmarks "Benchmark": raw CSV, summary, figure, timing 해석
- @ref dtdynamics_integration "Controller 통합": WBC handoff와 model update
- @ref dtdynamics_api_map "API 지도": public setup/compute/error family
- @ref dtdynamics_limitations "제한": 지원 범위와 compatibility 영향
- @ref dtdynamics_release "Release 경계": exact package/install/publication gate

## 설계 원칙

- `dtDynamics` 계산 항목: mass/bias/gravity/Coriolis, Jacobian/derivative, CoM/centroidal quantity
- 아닌 것: simulator, contact solver, final torque controller, MuJoCo wrapper
- 외부 model 처리: setup에서 fail-closed 검증, realtime에는 compiled model·preallocated data·cached numeric mapping만 전달
