# 계산 파이프라인 {#dtdynamics_computation}

Author: Joonhee Jo

@dotfile diagrams/compute_pipeline.dot

## State staging

- 일반적인 control tick 동작: 이미 resize된 `dtDynData`에 `qpos`, `qvel`, gravity 기록
- 필수 조건: state staging이 compiled model dimensions와 address convention에 일치

## Forward kinematics

- Forward kinematics 계산 항목: dynamics와 Jacobian 함수가 사용하는 body/site pose와 motion quantity
- 명시적 호출 시점: kinematics만 필요할 때
- 대안: prerequisite kinematic update를 자체 수행하는 dynamics entry point에 의존

## RNEA path

- controller-facing API가 사용하는 manipulator equation:

\f[
M(q)\dot{v} + h(q, v) = \tau + J(q)^\mathsf{T} f,
\f]

- `M`: generalized mass matrix
- `h`: Coriolis, centrifugal, gravity effect 등 bias term 모음
- `J`: task/contact Jacobian
- `f`: 상위 software가 공급하는 external wrench 또는 constraint force vector

- `ComputeDynamicsRNEA` 성격: RNEA contract로 제공되는 mass/bias/gravity/Coriolis-style ingredient용 경량 recursive Newton-Euler path
- 사용 시점: controller가 CCRBA 전용 centroidal/composite output이 불필요할 때

## CCRBA path

- composite/centroidal path 제공 항목: WBC objective가 사용하는 quantity (centroidal momentum relation 포함)

\f[
h_G = A_G(q) v,
\f]

- 1차 변화량:

\f[
\dot{h}_G = A_G(q)\dot{v} + \dot{A}_G(q, v)v.
\f]

- `ComputeDynamicsCCRBA` 성격: control-oriented composite rigid-body path
- 계산 항목: mass/bias basis, term mask에 따른 선택적 WBC term (`Cmat`, `MmatDot`, `cmAmat`, `cmAmatDot`, `comJmat`, `comJmatDot`, `centMom`, CoM quantity)

- term mask 용도: realtime design tool
- 원칙: controller tick이 소비하는 output만 요청 (예: LeoQuad는 reduced CCRBA mask 사용, indexed body/site Jacobian은 별도 계산)

## Output families and equations

- 기본 dynamics output 구성 방식: 일반적인 분리

\f[
h(q,v) = C(q,v)v + g(q),
\f]

- 필드: mass matrix `Mmat`, matrix-style Coriolis representation `Cmat`, gravity torque `gravityTorq`, Coriolis torque `coriolisTorq`, combined nonlinear torque `nonlinTorq`
- 실제 controller 사용 방식 차이: 일부는 `nonlinTorq` 직접 소비, 일부는 feedforward/diagnostics를 위해 gravity와 velocity term을 분리 유지

- mass matrix 취급 방식: generalized-coordinate inertia matrix

\f[
T = \frac{1}{2} v^\mathsf{T} M(q) v,
\f]

- validation 검사 항목에 symmetry residual 포함
- controller의 별도 결정 사항: full dense matrix, selected rows, 또는 상위 layer가 공급하는 product만 필요한지 여부

- Centroidal quantity 노출 단위: root별
- root system의 centroidal momentum matrix 역할: generalized velocity를 centroidal momentum으로 매핑

\f[
h_G = A_G(q)v,
\f]

- 행 순서 문서화: `[linear; angular]`
- derivative term 요청 시 controller 구성 가능 식:

\f[
\dot{h}_G = A_G(q)\dot{v} + \dot{A}_G(q,v)v.
\f]

- CoM quantity 성격: 마찬가지로 선택적
- WBC objective가 CoM position과 Jacobian만 필요할 때: 해당 flag만 요청, centroidal derivative는 비활성 유지

## Dependency order

- 안전한 mental model:

1. state는 `qpos`/`qvel`에 staging
2. forward kinematics가 world transform과 motion subspace 확립
3. dynamics가 mass, bias, gravity/Coriolis, 선택적 composite quantity 채움
4. task Jacobian 호출이 현재 kinematic state와 선택한 task ID 사용

- Dynamics entry point 특성: 자체 prerequisite kinematic update 실행
- `ForwardKinematics()` 명시적 호출로 충분한 경우: tick이 transform 또는 단발성 body/site/point Jacobian만 필요할 때
- 추가로 현재 `ComputeDynamicsRNEA()` 또는 `ComputeDynamicsCCRBA()` pass가 필요한 경우: Jacobian-derivative, `Jdot*qvel`, combined-term derivative output, bulk/indexed `dtDynData` Jacobian writer (이들은 `subtree_com`, `cvel`, `cdof`, `cdof_dot` 소비)

## Jacobian APIs

- Task velocity가 따르는 표준 differential relation:

\f[
\dot{x} = J(q)v,
\f]

- acceleration-level controller가 일반적으로 소비하는 식:

\f[
\ddot{x} = J(q)\dot{v} + \dot{J}(q, v)v.
\f]

- indexed body/site Jacobian 호출 사용 시점: controller가 base body와 선택한 task site만 필요할 때
- all-body/all-site 호출 용도: diagnostics나 tooling (default realtime path 아님)

- realtime WBC 권장 방식: task가 \f$J\f$와 \f$\dot{J}v\f$ 모두 필요할 때 combined `Compute*JacobianTerms()` helper 사용
- 미사용 output에 `nullptr` 전달 효과: task가 필요로 하는 정확한 ingredient 문서화, 같은 body/site/point에 대한 중복 traversal 작업 방지
