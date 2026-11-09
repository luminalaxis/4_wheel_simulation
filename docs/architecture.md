# 구조와 과도(transient) 솔버 확장

## 모듈 구성

```
src/aerolap/
  config.py           YAML 로드·기본값·검증, 점 경로 get/set ("suspension.front.arb_rate_Nm_per_deg")
  environment.py      공기 밀도
  tire.py             Tire: 하중·캠버 민감 그립, 슬립 곡선
  suspension.py       Axle: 휠레이트, 히브/롤 강성, 승차 주파수
  powertrain.py       Powertrain: 속도별 최대 구동력 (ICE 기어 / EV 토크·출력 제한)
  aero/maps.py        ConstantAero / TableAero / ResponseSurfaceAero, AeroModel(셋업 노브)
  aero/strategy.py    StrategyConfig: fixed / rule / ideal
  features.py         ModelFeatures, 효과 단계 L0–L5
  vehicle.py          VehicleModel: evaluate() (상태 -> 힘·하중·이용률), solve_state() (준정적 평형)
  track.py            Track: 노드 s, 노드 곡률, 구간 곡률
  solvers/base.py     Solver 프로토콜, LapResult
  solvers/ggv.py      GGV 엔벨로프, SolverOptions
  solvers/qss.py      QSSSolver
  analysis/           cornering, ladder, events, objectives, sweep, optimize
  plotting.py, cli.py
```

데이터 흐름:

```
YAML ──config──▶ dict (공학 단위) ──VehicleModel──▶ SI 파라미터 + 부품 객체
                       ▲                                   │
     sweep / optimize: set_path 로 값 변경                   ▼
                                       evaluate(v, ax, ay, mode, rh_f, rh_r, roll, yaw)
                                                           │  (공력, 하중, 그립, 이용률, 새 자세)
                                     solve_state: 고정점 ◀──┘
                                                           │
                         GGV(ggv.py) ◀─────────────────────┘
                              │
                QSSSolver: 정점 속도 + 전진/후진 적분 ──▶ LapResult ──▶ plotting / analysis
```

## 핵심 설계: "상태 → 힘" 함수와 평형 풀이의 분리

`VehicleModel.evaluate(v, ax, ay, mode, rh_f, rh_r, roll, yaw)`는 **자세를 입력으로 받는 명시적 함수**입니다.
반환값은 그 자세에서의 공력, 바퀴 하중, 타이어 용량, 이용률, 그리고 "스프링이 정적으로 평형을 이룰
자세"(`new_attitude`)입니다.

- QSS: `solve_state()`가 `attitude = new_attitude(attitude)` 고정점을 풉니다.
- 과도: 같은 `evaluate()`를 매 시간 스텝 호출하고, 자세를 **적분되는 상태 변수**로 취급하면 됩니다.

## 과도 솔버를 추가하는 절차

1. **상태 벡터** — 경로 좌표 `s`, 속도 `v`, 히브 `z`·피치 `θ`·롤 `φ`와 각각의 속도, 차체 슬립각 `β`와
   요레이트 `r`, (액티브 에어로) 플랩 개도 `λ ∈ [0, 1]`.
2. **스프링·댐퍼 힘** — `suspension.py`의 `Axle`에 댐퍼 계수(예: `damper_bump_Ns_per_m`,
   `damper_rebound_Ns_per_m`)를 추가하고, 설정은 `config.py`의 `AXLE_DEFAULTS`에 기본값을 넣습니다.
   코너 스프링 힘 = 휠레이트 × (정적 차고 − 현재 차고) 에서 `new_attitude`가 바로 정적 평형점이므로,
   `F_spring ≈ K·(new_attitude − attitude)` 형태로 `evaluate()` 출력을 그대로 재사용할 수 있습니다.
3. **차체 운동 방정식** — 히브: `m_s z̈ = ΣF_spring + ΣF_damper − (DF − DF_static)`,
   피치: `I_yy θ̈ = …`, 롤: `I_xx φ̈ = m_s·ay·d + … − K_φ φ − C_φ φ̇` (관성 모멘트를 `mass`에 추가).
4. **횡·요 운동** — `tire.py`의 슬립 곡선을 정방향(α → Fy)으로 쓰고 `I_zz ṙ = a·Fy_f − b·Fy_r`.
5. **드라이버 모델** — `QSSSolver`가 낸 `LapResult.v(s)`를 목표 속도로 쓰는 PI 스로틀/브레이크와
   곡률 기반 조향(예: pure pursuit). 과도 솔버는 QSS 결과로 초기화하면 수렴이 빠릅니다.
6. **액티브 에어로** — `aero/strategy.py`의 개폐 지연을 1차 지연 `λ̇ = (λ_cmd − λ)/τ`로 바꾸고, 공력
   계수는 두 모드 맵을 `λ`로 보간합니다.
7. **결과** — `solvers/base.py`의 `LapResult`를 그대로 채우면(`s, v, ax, ay, t, mode, channels`)
   `compare`, `plotting`, `events`, `optimize`가 수정 없이 동작합니다. `Solver` 프로토콜의
   `solve(vehicle, track, start)`만 구현하면 됩니다.

## 확장 시 지켜야 할 것

- 물리 함수는 numpy 배열을 브로드캐스트해야 합니다(GGV 격자 전체를 한 번에 계산).
- 새 효과는 `ModelFeatures`에 스위치를 추가하고 `LEVELS`에 단계로 넣으면 `compare`에 자동 반영됩니다.
- 새 셋업 파라미터는 YAML 키 이름에 단위를 붙이고(`_mm`, `_N_per_mm`, `_deg`) `config.py`에 기본값을 둡니다.
  그러면 `--set`, `sweep`, `optimize`에서 바로 변수로 쓸 수 있습니다.
