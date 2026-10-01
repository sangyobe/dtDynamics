# dtDynamics Manual

Author: Joonhee Jo

- 이 manual: generated API 목록보다 먼저 읽는 conceptual contract
- 다루는 순서: model/data lifecycle → coordinate/frame → setup/realtime 경계 → validation scope → 각 함수 signature 연결

현재 package identity: **0.1.3**

## Markdown 문서 검증

```bash
cd /path/to/dtDynamics
python3 tools/validate_documentation_content.py
python3 tools/validate_docs.py
```

- GitLab Markdown source, relative link, GitLab 수식 marker, SVG diagram, narrative invariant 검증
- HTML/PDF 생성·generated HTML·LaTeX 불필요

## 목차

1. [개요](overview.md) — package scope와 non-goal
2. [모델과 데이터 수명주기](modeling.md) — XML, validation, compilation, lookup, coordinates
3. [계산 파이프라인](computation.md) — FK, RNEA, CCRBA, Jacobian, CoM, centroidal term
4. [최소 end-to-end 예제](example.md) — load부터 compute까지
5. [Realtime resource model](realtime.md) — setup/loop 분리와 allocation evidence 범위
6. [검증](validation.md) — native/external reference와 fail-closed artifact gate
7. [Benchmark](benchmarks.md) — raw CSV, summary, figure, timing 해석
8. [Controller 통합](integration.md) — WBC handoff와 model update
9. [API 지도](api.md) — public setup/compute/error family
10. [제한](limitations.md) — 지원 범위와 compatibility 영향
11. [Release 경계](release.md) — exact package/install/publication gate

## 설계 원칙

- `dtDynamics` 계산 항목: mass/bias/gravity/Coriolis, Jacobian/derivative, CoM/centroidal quantity
- 아닌 것: simulator, contact solver, final torque controller, MuJoCo wrapper
- 외부 model 처리: setup에서 fail-closed 검증, realtime에는 compiled model·preallocated data·cached numeric mapping만 전달

## 관련 문서

- 시작점(top-level landing): `docs/manual.md`
- model guide: `docs/guides/model_guide/`
- usage/migration guide: `docs/guides/usage_guide/`
- benchmark evidence: [상세 benchmark report](../benchmark-results.md)
- release note: `docs/release-notes/v0.1.3.md`
