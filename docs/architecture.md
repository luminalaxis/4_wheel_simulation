# 구조와 과도(transient) 솔버

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
  solvers/transient.py  TransientSolver, BodyDynamics, TransientOptions
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
자세"(`new_attitude`)입니다. QSS의 `solve_state()`는 `attitude = new_attitude(attitude)` 고정점을 풀고,
과도 솔버는 아래 블록을 직접 조합해 자세를 적분되는 상태 변수로 다룹니다.

## 과도(transient) 솔버 구조 (`solvers/transient.py`)

QSS와 같은 물리 블록을 씁니다. `VehicleModel`의 다음 메서드가 두 솔버의 공통 부분입니다.

| 메서드 | 역할 |
|---|---|
| `aero_coefficients(mode, rh_f, rh_r, roll, yaw)` | 자세 → ClA_f, ClA_r, CdA |
| `load_components(ax, ay, DF_f, DF_r, D, roll)` | 강체 하중이동: 스프링이 받을 하중 `el_f/el_r`, 롤 모멘트, 기하학적 성분 |
| `tyre_loads(..., spring_f, spring_r, roll_moment_f, roll_moment_r)` | 스프링·댐퍼가 실제 전달하는 힘 → 4개 바퀴 하중 |
| `attitude(heave_f, heave_r, axle_f, axle_r, roll)` | 차고, 코너 캠버 |
| `grip_usage(v, ax, ay, fz, camber, incl, D)` | 타이어 용량, 슬립각, 축·파워 이용률 |

- QSS(`suspension_response`): `spring = el`, `roll_moment = K·φ_ss`
- 과도(`BodyDynamics`): `spring = K·q + C·q̇`, `roll_moment = K·φ + C·φ̇`, `q, φ`를 적분

`TransientSolver.solve()` 흐름:

1. 트랙을 `turn_in_length`로 평활화한 라인에서 QSS 기준 랩을 계산
2. `_simulate()`: 시간 적분. 매 스텝 공력(동적 자세, 플랩 보간) → 드라이버 명령(jerk 제한, 전환 예측) →
   `_limit_ax()`가 후보 가속도들을 한 번에 벡터 평가해 허용 범위로 제한 → 차체·요·플랩 적분
3. 타이어 과사용이 있으면 `_learned_cap()`으로 그 지점 목표 속도를 낮추고 QSS(`v_cap`)로 다시 계획, 반복
4. 시간 이력을 트랙 노드에 보간해 `LapResult`로 반환 (`compare`, `plotting`과 같은 채널 이름)

더 확장할 때의 후보:

- **비현가 질량·노면 입력**: `BodyDynamics`에 코너별 휠 상태와 노면 프로파일을 추가하고 `tyre_loads`에 타이어 변형으로 넣기
- **횡 운동 자유도**: 지금은 경로(중심선)를 정확히 따라가며 횡가속도가 `v²κ`로 정해집니다. 요레이트·횡속도를 상태로 두고
  `tire.py`의 슬립 곡선을 정방향으로 쓰면 언더/오버스티어 응답과 라인 이탈을 볼 수 있습니다
- **드라이버**: 최적 트레일 브레이킹(브레이크 해제 타이밍 최적화)이나 MPC 드라이버

## 확장 시 지켜야 할 것

- 물리 함수는 numpy 배열을 브로드캐스트해야 합니다(GGV 격자 전체를 한 번에 계산).
- 새 효과는 `ModelFeatures`에 스위치를 추가하고 `LEVELS`에 단계로 넣으면 `compare`에 자동 반영됩니다.
- 새 셋업 파라미터는 YAML 키 이름에 단위를 붙이고(`_mm`, `_N_per_mm`, `_deg`) `config.py`에 기본값을 둡니다.
  그러면 `--set`, `sweep`, `optimize`에서 바로 변수로 쓸 수 있습니다.
