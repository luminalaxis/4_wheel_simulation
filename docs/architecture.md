# 구조와 과도(transient) 솔버

## 모듈 구성

```
src/aerolap/
  config.py           YAML 로드·기본값·검증, 점 경로 get/set ("suspension.front.arb_rate_Nm_per_deg")
  environment.py      공기 밀도
  tire.py             Tire: 하중·캠버 민감 그립, 슬립 곡선
  suspension.py       Axle: 휠레이트, 히브/롤 강성, 승차 주파수
  motors.py           Motor: 데이터시트 기반 모터 (EMRAX 228 HV/MV/LV) 토크–속도, 효율, 로터 관성
  powertrain.py       Powertrain: 속도별 최대 구동력, 배터리 출력, 회전 관성 등가 질량
  aero/maps.py        ConstantAero / TableAero / ResponseSurfaceAero, AeroModel(셋업 노브)
  aero/strategy.py    StrategyConfig: fixed / rule / ideal
  features.py         ModelFeatures, 효과 단계 L0–L5
  vehicle.py          VehicleModel: evaluate() (상태 -> 힘·하중·이용률), solve_state() (준정적 평형)
  track.py            Track: 노드 s, 노드 곡률, 구간 곡률
  solvers/base.py     Solver 프로토콜, LapResult
  solvers/ggv.py      GGV 엔벨로프, SolverOptions
  solvers/qss.py      QSSSolver
  solvers/transient.py  TransientSolver, RideModel (7자유도), TransientOptions
  road.py             RoadProfile: ISO 8608 랜덤 노면 / 측정 프로파일 (좌·우)
  analysis/           cornering, ladder, events, energy (배터리 에너지 한계 내구), objectives, sweep, optimize
  importers.py        Assetto Corsa AI 라인(fast_lane.ai), OptimumLap 트랙(.oltra)·차량(.olveh) -> Track / 차량 dict
  export.py           GGV / 랩 트레이스 -> CSV, MATLAB .mat
  plotting.py, cli.py
matlab/aerolap_ggv_demo.m   export된 GGV로 MATLAB/Octave에서 그래프·점질량 랩 (docs/python_vs_matlab.md)
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
| `tyre_loads(..., spring_f, spring_r, roll_moment_f, roll_moment_r)` | (QSS) 스프링이 전달하는 힘 → 4개 바퀴 하중 |
| `attitude(heave_f, heave_r, axle_f, axle_r, roll)` | (QSS) 차고, 코너 캠버 |
| `corner_camber(travel, roll)` | 코너 변위·롤 → 캠버, 회전 중심 쪽 기울기 |
| `grip_usage(v, ax, ay, fz, camber, incl, D)` | 타이어 용량, 슬립각, 축·파워 이용률 |

- QSS(`suspension_response`): `spring = el`, `roll_moment = K·φ_ss`
- 과도(`RideModel`): 차체 3자유도 + 바퀴 4자유도를 적분. `load_components`의 `el_f, el_r, m_roll`이 차체에,
  기하학적 성분이 바퀴에 들어가고, 바퀴 하중은 타이어 압축(노면 포함)에서 나옵니다

`TransientSolver.solve()` 흐름:

1. 트랙을 `turn_in_length`로 평활화한 라인에서 QSS 기준 랩을 계산
2. `_simulate()`: 시간 적분. 매 스텝 라이드 모델에서 바퀴 하중·차고(노면 포함) → 타이어 완화 → 공력(동적 자세, 플랩 보간)
   → 슬립각에서 축 횡력(횡 자유도) → 드라이버 종방향 명령(jerk 제한, 전환 예측) → `_limit_ax()`가 후보 가속도들을 한 번에
   벡터 평가해 남은 마찰원·파워트레인 안으로 제한 → 라이드 모델 하위 스텝 적분, 횡·요·Frenet 경로 상태와 조향 갱신, 플랩 갱신
3. 학습 신호(지속 과사용, 경로 이탈, 슬라이드, 스핀)가 있으면 `_learned_cap()`이 원인 코너 진입부의 목표 속도를 낮추고 QSS(`v_cap`)로 다시 계획, 반복
4. 시간 이력을 트랙 노드에 보간해 `LapResult`로 반환 (`compare`, `plotting`과 같은 채널 이름)

더 확장할 때의 후보:

- **비선형 댐퍼·범프 스톱·플랭크 접촉력**: `RideModel._forces`에 속도별 댐퍼 곡선, 변위 한계 스프링, 바닥 접촉 반력을 추가
- **드라이버**: 최적 트레일 브레이킹(브레이크 해제 타이밍 최적화), 라인 최적화, MPC 드라이버
- **좌우 종방향 힘의 요 모멘트**: LSD 잠김률, 토크 벡터링(2·4모터)
- **회생 제동**: 모터 역토크와 배터리 충전 한계, 내구 에너지 점수

## 확장 시 지켜야 할 것

- 물리 함수는 numpy 배열을 브로드캐스트해야 합니다(GGV 격자 전체를 한 번에 계산).
- 새 효과는 `ModelFeatures`에 스위치를 추가하고 `LEVELS`에 단계로 넣으면 `compare`에 자동 반영됩니다.
- 새 셋업 파라미터는 YAML 키 이름에 단위를 붙이고(`_mm`, `_N_per_mm`, `_deg`) `config.py`에 기본값을 둡니다.
  그러면 `--set`, `sweep`, `optimize`에서 바로 변수로 쓸 수 있습니다.
