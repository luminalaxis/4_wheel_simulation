# aerolap — FSAE 공력·서스펜션 연성 랩타임 시뮬레이터

OptimumLap은 차량을 **점질량**으로 보고 공력을 `Cl·A`, `Cd·A` 두 상수로만 다룹니다. 그래서 다음을 계산하지 못합니다.

| OptimumLap이 반영하지 못하는 것 | aerolap에서 |
|---|---|
| 다운포스의 전/후 배분(CoP)이 언더/오버스티어 한계에 주는 영향 | 축별 그립 한계, 언더스티어 지수 |
| 하중이동과 타이어 하중 민감도 (어느 바퀴에 다운포스가 실리는가) | 코너별 4개 타이어 하중, `μ(Fz)` |
| 항력 작용 높이에 의한 후방 하중이동 | `drag_height_mm` |
| 다운포스·제동·가속으로 차체가 내려가고 기울면서 공력이 바뀌는 연성 | 히브·피치 → 차고 → aero map → 다시 하중 (고정점 평형) |
| 코너링 롤과 차체 슬립각(yaw)에 따른 공력 손실, 캠버 변화 | 롤·요 → aero map, 캠버 → 그립 |
| DRS / 액티브 에어로 (작동 지연 포함) | `fixed / rule / ideal` 전략 |
| 바닥 접촉(bottoming) | 최소 바닥 간극 계산·제약 |

같은 차량을 **OptimumLap 방식(L0)부터 전체 모델(L5)까지 효과를 하나씩 켜며** 돌려 각 효과가 랩타임을 얼마나
바꾸는지 분해하고, 스프링·ARB·3rd 스프링·정적 차고(레이크)·공력 배분 같은 **셋업을 코너링/랩타임 기준으로
최적화**할 수 있습니다. 기본 해석은 준정적(QSS)이고, 같은 물리 모델 위에서 **과도(transient, 시간영역) 솔버**로 차체 히브·피치·롤의
스프링·댐퍼 동역학, 요 지연, DRS 플랩 작동 속도까지 넣어 다시 돌려 볼 수 있습니다([docs/architecture.md](docs/architecture.md)).

> `data/`의 차량·aero map·트랙은 경향만 그럴듯하게 만든 **가상 예제**입니다. 실제 판단에는 팀의 CFD/풍동,
> 타이어(TTC 등), 서스펜션 데이터를 넣으세요.

## 설치

Python 3.10 이상.

```bash
pip install -e ".[dev]"
```

## 빠른 시작

```bash
# 한 랩 (폐회로 트랙은 flying lap, 개회로는 standing start가 기본)
aerolap run data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_endurance_example.csv
```

```bash
# OptimumLap 동등(L0) -> 전체 모델(L5) 효과 분해
aerolap compare data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_autocross_example.csv
```

```bash
# 속도별 정상상태 코너링 한계와 그때의 차고·롤·요·ClA·CoP, 직선 제동 한계
aerolap cornering data/vehicles/fsae_ev_aero.yaml
```

```bash
# FSAE 다이내믹 종목
aerolap events data/vehicles/fsae_ev_aero.yaml --autocross data/tracks/fsae_autocross_example.csv --endurance data/tracks/fsae_endurance_example.csv
```

```bash
# 셋업 최적화 (예제: 오토크로스 랩타임, 7개 변수, 바닥 간극·롤·언더스티어·승차 주파수 제약)
aerolap optimize data/optimize/cornering_setup.yaml
```

결과는 `out/<명령>/`에 텍스트 요약, CSV 트레이스, PNG 그래프로 저장됩니다(`--out`으로 변경).

## 명령

| 명령 | 하는 일 | 주요 출력 |
|---|---|---|
| `run VEHICLE TRACK` | 한 랩 시뮬레이션 | `summary.txt`, `trace.csv`, `lap.png`, `track_map.png` |
| `compare VEHICLE TRACK` | 효과 단계 L0–L5 랩타임 분해 (`--levels L0,L3,L5`) | `ladder.txt`, `ladder.png`, `compare.png` |
| `transient VEHICLE TRACK` | 시간영역 랩: 7자유도 라이드·공력 동역학, 노면 요철, QSS와 비교 (`--road A..E/CSV`, `--road-seed`, `--dt`, `--iterations`, `--max-jerk`, `--turn-in`) | `transient.txt`, `dynamics.png`, `trace_transient.csv` |
| `cornering VEHICLE` | 속도별 코너링·제동 한계와 자세 (`--speeds 30:120:10`, `--mode drs`) | `cornering.txt`, `cornering.png` |
| `ggv VEHICLE` | g-g-V 엔벨로프 (`--all-modes`) | `ggv.png` |
| `sweep VEHICLE --param PATH=a:b:step` | 1D/2D 파라미터 스윕 (`--param` 두 번이면 등고선) | `sweep.txt`, `sweep.png` |
| `sensitivity VEHICLE --params p1,p2` | 목적함수의 파라미터별 민감도(중앙차분) | `sensitivity.txt` |
| `optimize SPEC.yaml` | 제약 조건부 셋업 최적화 | `optimization.txt`, `optimization.png`, `optimized_vehicle.yaml` |
| `events VEHICLE` | 가속 75 m, 스키드패드, 오토크로스, 내구(에너지 포함) | `events.txt` |
| `fit-map CFD.csv` | CFD 산발점에 2차 응답면 피팅, 격자 맵으로 내보내기(`--export-grid`) | R², rms |

공통 옵션:

- `--set PATH=VALUE` — 차량 파라미터 덮어쓰기(여러 번 가능). 예: `--set aero.cop_shift=0.02 --set suspension.rear.arb_rate_Nm_per_deg=200`
- `--level L0..L5` — 모델 단계 (기본 `full` = L5)
- `--coarse` — 거친 엔벨로프로 빠르게(랩타임 오차 약 0.1 %), `--ds` — 트랙 스텝 [m]

스윕·민감도·최적화의 목적함수: `--objective lap_time --track TRACK [--solver transient --road B]` / `skidpad_time` / `ay_max --speeds-kmh 40,60,80,100`,
제약 표시: `--min-clearance-mm 5 --max-roll-deg 1.5 --understeer 1.0,1.3`.

```bash
# 예: 공력 밸런스에 따른 스키드패드 시간과 언더스티어 지수
aerolap sweep data/vehicles/fsae_ev_aero.yaml --param aero.cop_shift=-0.08:0.08:0.02 --objective skidpad_time --understeer 1.0,1.3
```

```bash
# 예: 앞/뒤 ARB 2D 스윕 (오토크로스 랩타임)
aerolap sweep data/vehicles/fsae_ev_aero.yaml --param suspension.front.arb_rate_Nm_per_deg=0:500:100 --param suspension.rear.arb_rate_Nm_per_deg=0:500:100 --objective lap_time --track data/tracks/fsae_autocross_example.csv --coarse
```

## 모델 단계 (effect ladder)

| 레벨 | 추가되는 효과 |
|---|---|
| L0 | **OptimumLap 동등**: 점질량, 다운포스는 정적 하중비로 배분, 공력 계수는 정적 차고에서 고정 |
| L1 | 4륜 종·횡 하중이동 → 타이어 하중 민감도가 실제로 작동 |
| L2 | 공력 밸런스(aero map의 CoP) + 항력 작용 높이 |
| L3 | 히브·피치에 의한 차고 변화가 aero map에 반영 |
| L4 | 코너링 롤·요가 aero map에 반영, 캠버 변화 |
| L5 | 액티브 에어로 / DRS |

예제 EV 차량, 예제 오토크로스(standing start) 결과:

```
level   lap [s]  step [s]  vs L0 [s]
L0       34.682    +0.000     +0.000  OptimumLap 동등
L1       34.460    -0.222     -0.222  + 4륜 하중이동 / 하중 민감도   (RWD 탈출 가속 시 후륜 하중 증가)
L2       34.489    +0.028     -0.193  + CoP / 항력 작용 높이
L3       34.450    -0.039     -0.233  + 히브·피치 차고 변화           (차고가 낮아지며 다운포스 증가)
L4       34.755    +0.305     +0.073  + 롤·요 → aero map, 캠버      (코너에서 언더트레이·윙 손실)
L5       34.740    -0.015     +0.058  + DRS
```

같은 차량의 FSAE 종목 결과: 가속 3.97 s, 스키드패드 4.93 s, 내구 랩 44.29 s(774.6 m). 예제 최적화
(`data/optimize/cornering_setup.yaml`, 250회 평가, 약 2분)는 오토크로스 L4 랩타임을 34.755 → 34.668 s(−0.087 s)로
줄였고, 이때 바닥 간극 제약(5 mm)이 걸려 있습니다(앞 3rd 스프링 완화, 뒤 3rd 스프링 강화, 레이크 증가, CoP 약간 전방).

수치 자체보다 **어떤 효과가 어느 방향으로 얼마나 큰지**를 보는 용도입니다. 가상 aero map 기준이므로 팀
데이터로 바꾸면 크기와 부호가 달라질 수 있습니다.

## 과도(transient) 해석

```bash
aerolap transient data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_autocross_example.csv
```

```bash
# 거친 노면 (ISO 8608 등급 A~E, 또는 측정 프로파일 CSV)
aerolap transient data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_autocross_example.csv --road B
```

QSS는 매 순간 차체가 이미 평형에 도달했고 노면은 완전히 평평하다고 봅니다. `transient`는 같은 물리 모델로 시간 적분하면서
다음을 더합니다.

- **7자유도 라이드 모델**: 차체 히브·피치·롤 + 바퀴 4개(비현가 질량)의 수직 운동. 코너 스프링·댐퍼, 3rd 스프링, ARB,
  타이어 스프링이 각각 작동합니다. 짧은 제동 구간에서는 노즈가 정상상태만큼 내려가지 않고, 제동을 풀면 되튀며, 롤은 코너
  진입에서 늦게 생깁니다. aero map은 이 **동적 바닥 높이·롤**로 읽힙니다.
- **노면 요철**: 좌·우 바퀴 궤적에 각각 노면 높이가 들어갑니다. 타이어 하중이 흔들리고(하중 민감도 때문에 평균 그립 감소,
  바퀴 들림), 바닥 높이가 흔들립니다(공력 변화, 바닥 접촉).
- **타이어 완화 길이**(0.25 m): 그립은 하중 변화를 거리 지연을 두고 따라갑니다.
- **요(슬립각) 지연**, **DRS 플랩 작동 속도**(개폐 시간 동안 계수를 보간)
- **드라이버**: QSS 속도 프로파일을 따라가되 가감속 변화율(jerk)에 한계가 있고(거는 쪽 150, 푸는 쪽 400 m/s³), 코너 진입은
  `turn_in_length`(기본 3 m)에 걸쳐 곡률이 늘어납니다. 타이어를 지속적으로 과사용한 곳에서는 목표 속도를 낮춰 다시 도는 학습
  랩을 반복합니다.

예제 EV, 오토크로스 (같은 3 m 진입 라인의 QSS와 비교):

```
                          QSS     transient   transient   transient
                                  (flat)      (ISO A)     (ISO B)
lap_time_s              34.183    34.603      35.351      36.512
min_clearance_mm        10.53     15.01       14.03       10.13
max_roll_deg             1.04      1.14        1.27        1.42
tyre load std [N]           -        54         108         192
```

평탄한 노면에서는 짧은 제동 구간에서 다이브가 정상상태까지 가지 않아 최소 바닥 간극이 QSS보다 큽니다. 거친 노면에서는 타이어
하중 변동과 바닥 높이 변동이 랩타임과 간극을 깎습니다.

**셋업 비교에서 QSS와 결론이 달라지는 예** (오토크로스, L4, `--coarse`, 스프링·3rd 스프링만 변경):

```
                 QSS       transient flat   transient ISO A   transient ISO B (최소 간극)
soft           34.205       34.947           35.395            36.069 (1.4 mm, 바닥 접촉 3 m)
base           34.142       34.932           35.544            36.478 (10.0 mm)
stiff          34.117       34.991           35.765            36.909 (9.3 mm)
```

QSS는 항상 "단단할수록 빠름"이지만, 노면이 있으면 부드러운 셋업이 타이어 하중을 안정시켜 빨라지는 대신 바닥에 닿습니다.
이 절충을 최적화하려면 목적함수에 `solver: transient`와 `road`를 넣으세요(`data/optimize/rough_road_setup.yaml`).

과도 랩타임에는 단순한 드라이버 모델(jerk 제한, 최적 트레일 브레이킹 없음, 경로는 중심선 고정)의 영향이 섞여 있으므로
QSS와의 절대 차이보다는 **같은 설정에서 셋업끼리 비교**하는 용도로 쓰세요. 한 랩(학습 랩 포함)에 20–40초 걸립니다.

## 입력 파일

### 차량 YAML (`data/vehicles/fsae_ev_aero.yaml` 참고)

키 이름에 단위가 붙어 있습니다. 생략한 항목은 기본값(`src/aerolap/config.py`)을 씁니다.

| 섹션 | 주요 키 |
|---|---|
| `environment` | `temperature_C`, `pressure_kPa`(null이면 고도로 계산), `altitude_m`, `relative_humidity` |
| `mass` | `total_kg`, `weight_dist_front`, `cg_height_mm`, `unsprung_front_kg`, `unsprung_rear_kg`, 과도용 `pitch_inertia_kgm2`, `roll_inertia_kgm2` |
| `geometry` | `wheelbase_mm`, `track_front_mm`, `track_rear_mm` |
| `tires` | `radius_mm`, `mu_x`, `mu_y`, `ref_load_N`, `load_sensitivity`(+100 % 하중당 μ 변화율), `vertical_rate_N_per_mm`, `peak_slip_deg`, `camber_optimum_deg`, `camber_sensitivity_per_deg2`, `rolling_resistance` |
| `suspension.front/rear` | `spring_rate_N_per_mm`, `motion_ratio`, `arb_rate_Nm_per_deg`, `heave_spring_N_per_mm`, `roll_center_mm`, `static_ride_height_mm`, `static_camber_deg`, `camber_gain_deg_per_mm`, `anti_dive` / `anti_lift` / `anti_squat`, 과도용 `damping_ratio` 또는 `damper_rate_Ns_per_mm` |
| `brakes` | `bias_front` (null = 이상적 배분) |
| `aero` | `type`(constant/table/response_surface), `file`, `default_mode`, `cla_scale`, `cda_scale`, `cop_shift`, `drag_height_mm`, `floor_width_mm`, `bottoming_clearance_mm`, `cop_lateral_shift_mm_per_deg`, `strategy` |
| `aero.strategy` | `type`(fixed/rule/ideal), `open_mode`, `closed_mode`, `open_radius_m`, `min_length_m`, `min_speed_kmh`, `open_delay_s`, `close_delay_s`, `close_anticipation` |
| `powertrain` | `type`(ev/ice), `drive`(rwd/awd), `differential`(lsd/open), `efficiency`, `ev:` 토크 곡선·감속비·`power_limit_kW` / `ice:` 토크 곡선·기어비·`shift_time_s` |

aero map 없이 OptimumLap처럼 상수로 쓰려면:

```yaml
aero:
  type: constant
  cl: 2.6
  cd: 1.0
  frontal_area_m2: 1.2
  cop_front: 0.45
```

### aero map CSV

```
mode,front_rh_mm,rear_rh_mm,roll_deg,yaw_deg,ClA,CdA,cop_front
high_df,30,40,0,0,3.65,1.28,0.43
...
```

- 차고는 앞/뒤 **축 위치**의 바닥 높이(mm), `cop_front`는 다운포스의 앞축 분담률
- `roll_deg`, `yaw_deg` 열은 선택. 있으면 L4부터 반영됩니다
- `type: table`은 전 격자(full factorial)가 필요합니다. CFD 점이 흩어져 있으면 `type: response_surface`로 바로 쓰거나
  `aerolap fit-map points.csv --export-grid grid.csv`로 격자를 만드세요
- 모드를 여러 개(예: `high_df`, `drs`) 넣으면 DRS 전략에서 사용합니다

### 트랙 CSV

```
# name: My autocross
# closed: false
type,length_m,radius_m,direction
straight,51.0,,
corner,14.137,9,left
```

OptimumLap 트랙 편집기처럼 직선/코너(길이, 반경, 방향) 세그먼트로 적습니다. 그 밖에 중심선 좌표
`x_m,y_m`(스플라인 평활화) 또는 `s_m,curvature_1pm` 형식도 읽습니다. `# closed: true`면 주기 랩(flying lap)입니다.

### 노면 프로파일 CSV

```
s_m,z_left_mm,z_right_mm
0,0,0
0.5,0.8,0.3
```

`s_m`은 트랙 진행 거리(무게중심 기준), 높이는 위가 +입니다. 좌우가 같으면 `s_m,z_mm` 두 열만 써도 됩니다.

### 최적화 spec (`data/optimize/cornering_setup.yaml`)

```yaml
vehicle: ../vehicles/fsae_ev_aero.yaml
objective: {type: lap_time, track: ../tracks/fsae_autocross_example.csv, start: standing, level: L4}
variables:
  - {path: suspension.front.arb_rate_Nm_per_deg, bounds: [0, 600]}
  - {path: suspension.front.static_ride_height_mm, bounds: [20, 50]}
  - {path: aero.cop_shift, bounds: [-0.06, 0.06]}
constraints:
  min_clearance_mm: 5            # 제동·코너링 한계와 랩 전체에서 바닥 간극
  max_roll_deg: 1.5
  understeer_index: [1.0, 1.3]   # 코너링 한계에서 앞축이 먼저 포화(안정)하되 과도한 언더스티어는 금지
  ride_frequency_hz: [3.0, 5.5]
method: powell                   # 또는 differential_evolution (전역, 더 느림)
max_evals: 250
```

변수는 차량 YAML의 어떤 숫자 경로든 쓸 수 있습니다. 탐색 중에는 거친 엔벨로프를 쓰고, 상위 후보를 정밀
엔벨로프로 다시 평가해 제약을 만족하는 최선을 고릅니다. 최적 셋업은 `optimized_vehicle.yaml`로 저장됩니다.

## OptimumLap과 맞춰 보기 (권장 절차)

1. OptimumLap에 넣던 값(질량, μ, Cl·Cd·면적, 토크 곡선, 기어비 …)으로 차량 YAML을 만들고 `aero.type: constant`로 둡니다.
2. 같은 트랙에서 `aerolap run ... --level L0` 결과를 OptimumLap 랩타임과 비교합니다. 내부 구현(타이어 하중 민감도 정의,
   적분 방식 등)이 달라 정확히 일치한다고 보장하지는 않으므로, 차이가 크면 파라미터 대응부터 확인합니다.
3. 무게중심 높이, 롤센터, 스프링·ARB, 차고, aero map을 채우고 `aerolap compare`로 L1–L5에서 무엇이 바뀌는지 봅니다.

## 모델 요약

자세한 수식·가정·한계는 [docs/model.md](docs/model.md)를 보세요.

- 매 `(v, ax, ay)`에서 **공력 → 4륜 하중 → 서스펜션 변위(차고·피치·롤) → 슬립각(요) → 공력**을 고정점 반복으로 풉니다
- 속도별 g-g-V 엔벨로프를 만든 뒤 정점 속도 + 전진/후진 적분(Heun)으로 속도 프로파일을 구합니다
- 언더스티어 지수(앞축 횡 이용률/뒷축 횡 이용률)로 저속(기계적 밸런스) ↔ 고속(공력 밸런스) 전환을 봅니다
- 과도 해석에도 타이어 온도, 비선형 댐퍼, 범프 스톱, 바닥 접촉 반력, 횡 운동 자유도(라인 이탈)는 없습니다

## 개발

```bash
python -m pytest
```

```bash
# 예제 aero map·트랙 재생성
python scripts/generate_example_data.py
```

구조와 과도 솔버 설계: [docs/architecture.md](docs/architecture.md)
