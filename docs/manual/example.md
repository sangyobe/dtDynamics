# 4. 최소 end-to-end 예제

Author: Joonhee Jo

- 이 예제가 보여주는 것: **load → validate → compile → resize → ID cache → state 입력 → compute** 순서를 지키는 최소 dynamics ingredient 사용 흐름
- 성격: simulator나 torque controller 예제가 아님 — 전체 입력 XML, C++ program, 실행 순서는 `docs/manual.md` 3장 "최소 end-to-end 예제"에 실행 절차로 이전됨

## 4-1. `CompileSpec`은 언제 쓰는가

- 외부 파일·editable spec을 다룰 때: `CompileSpecChecked()`가 기본
- raw `CompileSpec()` 유지 목적: package 내부 fixture처럼 validation이 별도로 끝난 trusted setup path의 호환성

## 4-2. Neutral state 주의

- 미제공 항목: free/ball joint의 유효 quaternion과 robot rest pose를 자동으로 정하는 package-level initializer
- `qpos`를 모두 zero로 채울 때 위험: floating-base quaternion이 invalid할 수 있음
- 초기화 책임: estimator/controller가 compiled address와 model convention에 따라 state를 명시적으로 기록

## 관련 문서

- 이전 chapter: [계산 파이프라인](computation.md)
- 다음 chapter: [Realtime resource model](realtime.md)
- 전체 목차: [전체 목차](index.md)
- 실행 절차(전체 예제 코드, 실패 진단): `docs/manual.md` 3장
