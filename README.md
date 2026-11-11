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
| 모터 특성: 전압에 따른 기저 속도, 효율, 로터 관성, 연속 정격 | EMRAX 228 데이터시트 모델, 배터리 출력 한계 |
| 배터리 용량: 항력이 큰 윙이 내구 에너지를 얼마나 더 쓰고, 용량이 모자라면 얼마나 느려지는가 | 랩 에너지 분해(항력·제동·손실·회생), 에너지 한계 내구(출력 캡) |
| 과도 응답: 차체·바퀴 진동, 노면, 언더/오버스티어, 라인 이탈 | 과도 솔버(7자유도 라이드 + 횡·요 + 조향 드라이버) |

같은 차량을 **OptimumLap 방식(L0)부터 전체 모델(L5)까지 효과를 하나씩 켜며** 돌려 각 효과가 랩타임을 얼마나
바꾸는지 분해하고, 스프링·ARB·3rd 스프링·정적 차고(레이크)·공력 배분 같은 **셋업을 코너링/랩타임 기준으로
최적화**할 수 있습니다. 기본 해석은 준정적(QSS)이고, 같은 물리 모델 위에서 **과도(transient, 시간영역) 솔버**로 차체·바퀴의
스프링·댐퍼 동역학, 노면 요철, 횡·요 운동과 조향 드라이버, DRS 플랩 작동 속도까지 넣어 다시 돌려 볼 수 있습니다
([docs/architecture.md](docs/architecture.md)). 예제 EV는 **EMRAX 228** 모터 데이터시트 값으로 구동계를 계산합니다.

> `data/`의 차량·aero map·트랙은 경향만 그럴듯하게 만든 **가상 예제**입니다. 실제 판단에는 팀의 CFD/풍동,
> 타이어(TTC 등), 서스펜션 데이터를 넣으세요.

## 설치

Python 3.10 이상.

```bash
pip install -e ".[dev]"
```

## GUI (명령어 없이 쓰기)

```bat
py -m aerolap.gui
```

또는 저장소 폴더의 **`aerolap_gui.bat`을 더블클릭**합니다(추가 설치 없음, Python 기본 tkinter 사용).

![GUI](docs/img/gui.png)

1. 위에서 **차량 YAML**과 **트랙**을 고릅니다(기본: 26년도 차, FSK 내구 트랙 평활화). OptimumLap `.olveh`/`.oltra`나 Assetto Corsa
   `.ai`는 **"OptimumLap / AC 파일 가져오기..."**로 바로 변환해 불러옵니다
2. **파라미터** 탭: 질량, 무게중심, 스프링·ARB, 차고, 공력 장치 ClA·CdA·크기, 배터리 셀·팩 값을 고칩니다. 바꾼 값은 파란색으로
   표시되고, "YAML로 저장"은 원래 파일 맨 위 주석(ASSUMED 목록)을 남깁니다
3. **계산** 탭: 한 랩 / 효과 분해(L0→L5) / FSAE 종목 / 내구 에너지·배터리 / 코너링 한계·밸런스
4. **스윕 · 최적화** 탭: 값 하나를 범위로 바꿔 보는 **파라미터 스윕**(목적: 한 랩 / 내구 / 스키드패드), **공력 장치 최적화**
   (끝나면 "최적값을 파라미터에 적용")
5. 결과는 오른쪽 **결과·그래프** 탭에 나오고(그래프 더블클릭 = 원본 이미지), 실행할 때마다 **비교 기록** 탭에 한 줄씩
   쌓입니다(바꾼 파라미터와 결과, CSV 저장). 계산 중에는 진행 막대와 **취소** 버튼이 있습니다

![비교 기록](docs/img/gui_history.png)

"빠른 계산"을 켜면 거친 엔벨로프로 몇 배 빨라집니다(랩타임 오차 약 0.1 %).
차량 파일에 모르는 키(오타)가 있으면 로그 창(명령줄은 stderr)에 비슷한 이름과 함께 경고가 나옵니다.

**26년도 차·FSK 트랙 결과 모음: [docs/results_fsk2026.md](docs/results_fsk2026.md)**

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
# 내구 에너지 예산: 배터리 용량 안에 들어가는가, 항력 몫, 모자라면 출력 캡과 랩타임
aerolap energy data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_endurance_example.csv
```

```bash
# 셋업 최적화 (예제: 오토크로스 랩타임, 7개 변수, 바닥 간극·롤·언더스티어·승차 주파수 제약)
aerolap optimize data/optimize/cornering_setup.yaml
```

결과는 `out/<명령>/`에 텍스트 요약, CSV 트레이스, PNG 그래프로 저장됩니다(`--out`으로 변경).

## 명령

| 명령 | 하는 일 | 주요 출력 |
|---|---|---|
| `run VEHICLE TRACK` | 한 랩 시뮬레이션 (`--mat`: MATLAB용 `trace.mat`) | `summary.txt`, `trace.csv`, `lap.png`, `track_map.png` |
| `compare VEHICLE TRACK` | 효과 단계 L0–L5 랩타임 분해 (`--levels L0,L3,L5`) | `ladder.txt`, `ladder.png`, `compare.png` |
| `transient VEHICLE TRACK` | 시간영역 랩: 7자유도 라이드·공력 동역학, 노면 요철, QSS와 비교 (`--road A..E/CSV`, `--road-seed`, `--dt`, `--iterations`, `--max-jerk`, `--turn-in`) | `transient.txt`, `dynamics.png`, `trace_transient.csv` |
| `cornering VEHICLE` | 속도별 코너링·제동 한계와 자세 (`--speeds 30:120:10`, `--mode drs`) | `cornering.txt`, `cornering.png` |
| `ggv VEHICLE` | g-g-V 엔벨로프 (`--all-modes`, `--export`: `ggv.csv`·MATLAB `ggv.mat`) | `ggv.png` |
| `sweep VEHICLE --param PATH=a:b:step` | 1D/2D 파라미터 스윕 (`--param` 두 번이면 등고선) | `sweep.txt`, `sweep.png` |
| `sensitivity VEHICLE --params p1,p2` | 목적함수의 파라미터별 민감도(중앙차분) | `sensitivity.txt` |
| `optimize SPEC.yaml` | 제약 조건부 셋업 최적화 | `optimization.txt`, `optimization.png`, `optimized_vehicle.yaml` |
| `events VEHICLE` | 가속 75 m, 스키드패드, 오토크로스, 내구(배터리 에너지 한계·모터 열 점검 포함) | `events.txt` |
| `energy VEHICLE TRACK` | 내구 에너지 예산·분해, 용량 부족 시 출력 캡 (`--distance-km 22`, `--param aero.cla_scale,aero.cda_scale=0.6:1.4:0.2`, `--relative`: 현재 값 × 배율) | `energy.txt`, `energy_sweep.png` |
| `import-track FILE` | Assetto Corsa AI 라인(`fast_lane.ai`)·OptimumLap 트랙(`.oltra`)을 트랙 CSV로 (`--cutoff 10`, `--smooth 15`, `--reverse`) | 트랙 CSV |
| `aero-split VEHICLE TRACK` | 프론트·사이드·리어 공력 장치별 최적 크기 (개별 스윕 + 동시 최적화, `--objective endurance_time`) | `aero_split.txt`, `aero_split.png`, `optimized_vehicle.yaml` |
| `aero-exchange VEHICLE TRACK` | 공력 교환비: ClA·CdA·밸런스·질량·장치 1단위의 랩/내구 값어치 (`--objective endurance_time`) | `aero_exchange.txt`, `.png` |
| `aero-envelope VEHICLE TRACK` | 공력 작동 영역: 차고·롤·요 분포, 밸런스 이동, 맵 범위 밖·바닥 막힘 구간 | `aero_envelope.txt`, `.png` |
| `import-vehicle FILE.olveh` | OptimumLap 차량을 YAML로 (OptimumLap 동등 / `--full` 모터 모델, `--drive awd`) | 차량 YAML |
| `fit-map CFD.csv` | CFD 산발점에 2차 응답면 피팅, 격자 맵으로 내보내기(`--export-grid`) | R², rms |

공통 옵션:

- `--set PATH=VALUE` — 차량 파라미터 덮어쓰기(여러 번 가능). 예: `--set aero.cop_shift=0.02 --set suspension.rear.arb_rate_Nm_per_deg=200`
- `--level L0..L5` — 모델 단계 (기본 `full` = L5)
- `--coarse` — 거친 엔벨로프로 빠르게(랩타임 오차 약 0.1 %), `--ds` — 트랙 스텝 [m]

스윕·민감도·최적화의 목적함수: `--objective lap_time --track TRACK [--solver transient --road B]` / `skidpad_time` / `ay_max --speeds-kmh 40,60,80,100`
(최적화 spec에서는 `endurance_time`도 가능),
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
L0       34.695    +0.000     +0.000  OptimumLap 동등
L1       34.497    -0.198     -0.198  + 4륜 하중이동 / 하중 민감도   (RWD 탈출 가속 시 후륜 하중 증가)
L2       34.536    +0.039     -0.159  + CoP / 항력 작용 높이
L3       34.497    -0.039     -0.198  + 히브·피치 차고 변화           (차고가 낮아지며 다운포스 증가)
L4       34.799    +0.301     +0.104  + 롤·요 → aero map, 캠버      (코너에서 언더트레이·윙 손실)
L5       34.785    -0.014     +0.090  + DRS
```

같은 차량의 FSAE 종목 결과: 가속 3.99 s, 스키드패드 4.93 s, 내구는 전력 제한 없이 달리면 랩 44.35 s(774.6 m)에 22 km
10.6 kWh가 필요해 가정한 배터리(7.5 kWh × 0.95)를 넘으므로 출력 캡 35.8 kW로 랩 47.07 s, 합계 1318.0 s(아래 "배터리" 절).
예제 최적화
(`data/optimize/cornering_setup.yaml`, 250회 평가, 약 2분)는 오토크로스 L4 랩타임을 34.799 → 34.696 s(−0.103 s)로
줄였고, 이때 바닥 간극(5 mm)과 언더스티어 하한 제약이 걸려 있습니다(앞뒤 ARB 완화, 앞 3rd 스프링 완화·뒤 강화, 레이크 증가,
CoP를 허용 범위 끝까지 전방).

수치 자체보다 **어떤 효과가 어느 방향으로 얼마나 큰지**를 보는 용도입니다. 가상 aero map 기준이므로 팀
데이터로 바꾸면 크기와 부호가 달라질 수 있습니다.

## 모터 (EMRAX 228)

예제 EV(`data/vehicles/fsae_ev_aero.yaml`)의 구동계는 EMRAX 228 데이터시트(v1.7)로 계산합니다.

```yaml
powertrain:
  type: ev
  ev:
    motor: emrax_228
    motor_variant: MV          # HV | MV | LV 권선 (KV, KT, 최대 전류, 상저항)
    motor_cooling: LC          # AC | LC | CC (연속 정격, 로터 관성)
    dc_voltage_V: 550          # 부하 시 축전지 전압 -> 기저 속도 = KV x V
    controller_current_A: null # 인버터 상전류 한계 (null = 모터 최대 전류, MV 360 A)
    n_motors: 1
    reduction_ratio: 3.6
    drivetrain_efficiency: 0.97
    inverter_efficiency: 0.97
    power_limit_kW: 80         # 축전지 출력 한계 (배터리 측)
```

- 토크–속도: 220 Nm(또는 KT × 인버터 전류)를 기저 속도 `KV × 직류 전압`까지, 그 위는 정출력, 6500 rpm에서 끝. 데이터시트와
  대조하면 MV 630 V → 5500 rpm에서 124 kW, HV 830 V → 4500 rpm에서 104 kW로 일치합니다. 직류 전압을 낮추면 기저 속도가 내려가
  고속 토크가 줄어듭니다(MV 550 V → 4774 rpm)
- 효율: 데이터시트 상저항·KT로 동손을 계산하고, 속도 손실은 효율 맵(최대 96 %)에 맞춘 근사입니다. 랩 에너지는 배터리 출력의 적분
- 로터 관성 0.0398 kg·m²는 감속비 3.6에서 차량 질량 12.5 kg에 해당하며, 출력 한계 구간의 가속을 늦춥니다
- 예제 차는 약 72 km/h부터 축전지 80 kW 한계에 걸립니다
- **내구 열 점검**: `aerolap events ... --endurance`가 랩의 RMS 모터 토크·평균 출력을 연속(S1) 정격과 비교합니다. 예제 차는
  전력 제한 없이 달리면 RMS 119.9 Nm로 LC 정격 118 Nm를 약간 넘지만, 배터리 용량 때문에 출력 캡(35.8 kW)이 걸린 실제 내구
  랩에서는 77.2 Nm(평균 18.0 kW)로 여유가 있습니다. 냉각 방식(CC: 130 Nm)이나 감속비 선택에 참고할 값입니다
- 권선(HV/MV/LV)과 전압·인버터 전류 한계는 데이터시트만으로 정해지지 않으므로 팀 사양에 맞게 바꾸세요. 예제 값(MV, LC,
  550 V, 360 A, 감속비 3.6)은 가정입니다

## 배터리 · 내구 에너지

```yaml
battery:                      # 예제 값은 가정 - 팀 축전지 사양으로 바꾸세요
  capacity_kWh: 7.5           # 공칭 에너지 (null이면 에너지 점검 안 함)
  usable_fraction: 0.95       # 실제로 쓰는 SOC 범위 -> 7.12 kWh
  max_power_kW: null          # 팩 방전 한계. 실제 한계 = min(이 값, powertrain.ev.power_limit_kW)
  regen_power_kW: 0           # 회생 한계(배터리 측, 0 = 회생 없음)
  regen_axle_share: 0.35      # 구동축(뒤)이 맡는 제동력 비율 -> 회생 가능한 몫
```

- **출력 한계**: `power_limit_kW`(규정 80 kW)와 `max_power_kW`(팩) 중 작은 값이 배터리 측 출력 한계입니다. 모터·인버터
  효율을 거쳐 휠 구동력 한계로 바뀌어 GGV와 랩에 그대로 들어갑니다
- **랩 에너지 분해**(`energy_breakdown`): 배터리 에너지 = 휠 일 + 모터·인버터·구동계 손실 − 회생, 휠 일 = 공력 항력 +
  구름저항 + 제동(열) + 회생 + 운동에너지 변화
- **에너지 한계 내구**(`endurance_energy`): 22 km(트랙 길이로 반올림한 랩 수)를 한계 주행했을 때 필요한 에너지가 사용 가능
  에너지를 넘으면, 넘지 않을 때까지 **배터리 출력 캡을 낮춰**(이분법) 그때의 랩타임을 냅니다. 실제 팀이 내구에서 출력을
  제한하는 방식과 같습니다(리프트 앤 코스트는 모델링하지 않음)
- 회생은 브레이크 블렌딩이 이상적이라고 보고 제동력 중 `regen_axle_share`만큼을 모터 토크·`regen_power_kW` 한도 안에서
  회수합니다. 회생은 속도 프로파일을 바꾸지 않습니다

```bash
aerolap energy data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_endurance_example.csv
```

예제 EV, 예제 내구 트랙 28랩(21.7 km):

| 경우 | 한계 주행 필요 에너지 | 출력 캡 | 랩 | 내구 합계 | 랩당 항력 에너지 |
|---|---|---|---|---|---|
| 배터리 무제한 | 10.59 kWh | – | 44.353 s | 1241.9 s | 0.055 kWh |
| 7.5 kWh(사용 7.12), 회생 없음 | 10.59 kWh | 35.8 kW | 47.071 s | 1318.0 s (+76.1) | 0.048 kWh |
| 7.5 kWh, 회생 30 kW | 8.59 kWh | 52.5 kW | 45.376 s | 1270.5 s (+28.6) | 0.051 kWh |

한계 주행 랩에서 배터리 에너지의 약 69 %는 제동 열, 14 %는 공력 항력, 13 %는 구동계 손실입니다.

**윙 크기와 배터리**(`--param aero.cla_scale,aero.cda_scale=0.6:1.4:0.2`, 다운포스와 항력을 같은 비율로):

```
cla_scale+cda_scale  flat-out lap  flat-out kWh  drag kWh/lap  cap kW    lap s  endurance s
                0.6        45.023          9.84        0.0289    38.7   46.975       1315.3
                0.8        44.675         10.23        0.0384    37.2   47.009       1316.3
                1.0        44.353         10.59        0.0480    35.8   47.071       1318.0
                1.2        44.052         10.93        0.0579    34.4   47.158       1320.4
                1.4        43.771         11.25        0.0673    33.2   47.220       1322.2
```

전력 제한 없이 달리면 윙이 클수록 빠르지만(랩 −0.3 s / +20 %), **배터리가 한계면 윙이 클수록 내구가 느려집니다**
(+20 %마다 +1.0–2.4 s). 다운포스로 빨라진 코너에서 더 세게 제동하고 항력도 늘어 출력 캡이 내려가기 때문입니다. 최적화에서는
두 가지로 씁니다.

```yaml
objective: {type: endurance_time, track: ../tracks/fsae_endurance_example.csv, distance_km: 22}   # 에너지 한계 내구 시간
constraints:
  battery_energy: {track: ../tracks/fsae_endurance_example.csv, distance_km: 22, margin_kWh: 0.2}
  # -> 한계 주행 내구가 (사용 가능 에너지 - 여유) 안에 들어가야 함: "항력 때문에 배터리가 모자라면 안 된다"
```

### 셀 기반 팩 모델 (출력·전압 강하·SOC)

`battery.cell`을 적으면 배터리를 고정 출력 상한이 아니라 **셀로 만든 팩**으로 계산합니다(`data/vehicles/fsk2026.yaml` 참고).

```yaml
battery:
  series: 132                # 직렬 셀 수
  parallel: 4                # 병렬 셀 수
  soc: 1.0                   # 시작 SOC
  soc_min: 0.05              # 사용 하한
  soc_stages: 3              # 내구를 몇 단계 SOC로 나눠 계산할지
  interconnect_resistance_mOhm: 30
  max_current_A: null        # 팩 전류 한계(퓨즈 등)
  cell: {capacity_Ah: 4.0, resistance_mOhm: 13, max_current_A: 45, v_min: 2.8,
         ocv: [[0, 3.0], [0.1, 3.35], [0.2, 3.5], [0.5, 3.68], [0.8, 3.95], [1.0, 4.18]]}
```

- **출력**: 개방전압 `V_oc = 직렬 × OCV(SOC)`, 팩 저항 `R = 직렬 × R_셀 / 병렬 + 배선`. 전류는 셀 한계 × 병렬, 팩 한계, 셀 최저 전압에서
  정해지고, 최대 단자 출력 `V_oc·I − R·I²`가 규정 80 kW보다 작으면 그 값이 출력 상한이 됩니다
- **전압 강하 → 모터**: 전 출력 때 단자 전압 `V_t = V_oc − R·I`가 모터 직류 전압이 되어 기저 속도(`KV × V_t`)가 SOC와 전류에 따라
  내려갑니다(`dc_voltage_V`는 무시)
- **손실**: 셀에서 빠지는 에너지 = 단자 에너지 + `R·I²`. 사용 가능 에너지는 `soc`→`soc_min` 구간의 `∫OCV dSOC × 용량`
- **내구**: 22 km를 SOC 단계(기본 3)로 나눠 단계마다 그 SOC로 랩을 다시 풀고, 전하량으로 SOC를 내립니다. 모자라면 출력 캡을 찾는
  방식은 같습니다. 셀 온도와 SOC에 따른 내부저항 변화는 아직 없습니다

26년도 차(예시 셀 132S4P, 21700 4 Ah급 — **가정값**, 팀 셀 데이터로 교체 필요):

| SOC | 개방전압 | 최대 단자 출력 | 80 kW 때 버스 전압 | 모터 기저 속도 |
|---|---|---|---|---|
| 1.0 | 552 V | 84.4 kW | 474 V | 4117 rpm |
| 0.6 | 498 V | 74.7 kW | 415 V | 3602 rpm |
| 0.3 | 470 V | 69.7 kW | 387 V | 3362 rpm |
| 0.1 | 442 V | 58.5 kW | 370 V | 3208 rpm |

FSK 트랙에서의 SOC별 출력·전압, 내구 결과는 [docs/results_fsk2026.md](docs/results_fsk2026.md) 3절에 있습니다.

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
- **횡·요 동역학과 조향 드라이버**(기본, `--no-lateral`로 끔): 슬립각에서 축 횡력이 나오고, 횡속도·요레이트·라인 대비 횡오차·
  헤딩 오차를 적분합니다. 드라이버는 곡률 피드포워드 + 프리뷰 피드백 + 요레이트 카운터스티어로 조향하고, 라인 바깥으로 밀리면
  리프트합니다. 언더스티어(바깥으로 밀림), 오버스티어(뒤 미끄러짐, 스핀)가 나타나며 마찰원은 횡력 우선입니다
- **DRS 플랩 작동 속도**(개폐 시간 동안 계수를 보간)
- **드라이버 속도 제어**: QSS 속도 프로파일을 따라가되 가감속 변화율(jerk)에 한계가 있고(거는 쪽 150, 푸는 쪽 400 m/s³), 코너
  진입은 `turn_in_length`(기본 3 m)에 걸쳐 곡률이 늘어납니다. 타이어 지속 과사용·슬라이드·스핀·라인 이탈(0.75 m 초과)이 생기면
  그 코너 진입 전의 목표 속도를 낮춰 다시 도는 학습 랩을 반복합니다.

예제 EV, 오토크로스, 횡 자유도 포함 (같은 3 m 진입 라인의 QSS와 비교):

```
                          QSS     transient   transient   transient
                                  (flat)      (ISO A)     (ISO B)
lap_time_s              34.229    34.609      34.584      35.723
min_clearance_mm        10.53     15.12       13.78        9.28
max_roll_deg             1.04      1.18        1.41        1.71
max_path_error_m            -      0.28        0.27        0.40
tyre load std [N]           -        50         110         198
```

평탄한 노면에서는 짧은 제동 구간에서 다이브가 정상상태까지 가지 않아 최소 바닥 간극이 QSS보다 큽니다. 거친 노면에서는 타이어
하중 변동과 바닥 높이 변동이 커지고, B등급에서는 바퀴가 순간적으로 뜨며 랩타임과 간극이 줄어듭니다. 학습 랩이 반복마다 다른
코너에서 반응하므로 과도 랩타임에는 약 ±0.2 s의 산포가 있습니다(위 A등급이 평탄보다 0.03 s 빠른 것도 그 범위). 그보다 작은
차이는 결론에 쓰지 마세요.

**셋업 비교에서 QSS와 결론이 달라지는 예** (오토크로스, L4, `--coarse`, 스프링·3rd 스프링만 변경, 횡 자유도 포함):

```
                 QSS       transient flat   transient ISO A   transient ISO B (최소 간극)
soft           34.248       34.574           34.550            35.642 (3.4 mm, 바닥 접촉 3 m)
base           34.187       34.527           34.896            36.039 (10.1 mm)
stiff          34.162       34.506           35.152            35.957 (10.9 mm)
```

QSS와 평탄 노면 과도 해석은 "단단할수록 빠름"으로 같습니다. 노면이 있으면 부드러운 셋업이 타이어 하중을 안정시켜
A등급에서 0.35–0.6 s 빨라지지만, B등급에서는 바닥에 닿습니다(B등급의 base와 stiff 차이 0.08 s는 산포 범위). 이 절충을
최적화하려면 목적함수에 `solver: transient`와 `road`를 넣으세요(`data/optimize/rough_road_setup.yaml`).
예제를 `--max-evals 20`으로 돌리면(약 6.5분, 평가 한 번에 과도 랩 2–3회) 앞 스프링 40 → 50 N/mm, 뒤 45 → 27 N/mm로 바꿔
B등급 노면 랩타임을 35.55 → 34.68 s로 줄입니다. 같은 셋업의 QSS 랩은 오히려 0.16 s 느려지므로 QSS로는 찾을 수 없는 방향이고,
뒤를 부드럽게 해 거친 노면에서 뒤 접지를 안정시키는(언더스티어 쪽) 해입니다. 평가 수가 적고 과도 랩에 ±0.2 s 산포가 있으니
후보를 찾는 용도로 쓰고, 고른 셋업은 노면 시드(`road_seed`)를 바꿔 다시 확인하세요.

과도 랩타임에는 단순한 드라이버 모델(jerk 제한, 최적 트레일 브레이킹 없음, 경로는 중심선 고정)의 영향이 섞여 있으므로
QSS와의 절대 차이보다는 **같은 설정에서 셋업끼리 비교**하는 용도로 쓰세요. 한 랩(학습 랩 포함)에 20–40초 걸립니다.

## 입력 파일

### 차량 YAML (`data/vehicles/fsae_ev_aero.yaml` 참고)

키 이름에 단위가 붙어 있습니다. 생략한 항목은 기본값(`src/aerolap/config.py`)을 씁니다.

| 섹션 | 주요 키 |
|---|---|
| `environment` | `temperature_C`, `pressure_kPa`(null이면 고도로 계산), `altitude_m`, `relative_humidity`, `air_density_kg_m3`(있으면 위 값 대신 사용) |
| `mass` | `total_kg`, `weight_dist_front`, `cg_height_mm`, `unsprung_front_kg`, `unsprung_rear_kg`, 과도용 `pitch_inertia_kgm2`, `roll_inertia_kgm2`, `yaw_inertia_kgm2` |
| `geometry` | `wheelbase_mm`, `track_front_mm`, `track_rear_mm` |
| `tires` | `radius_mm`, `mu_x`, `mu_y`, `ref_load_N`, `load_sensitivity`(+100 % 하중당 μ 변화율), `vertical_rate_N_per_mm`, `peak_slip_deg`, `camber_optimum_deg`, `camber_sensitivity_per_deg2`, `rolling_resistance` |
| `suspension.front/rear` | `spring_rate_N_per_mm`, `motion_ratio`, `arb_rate_Nm_per_deg`, `heave_spring_N_per_mm`, `roll_center_mm`, `static_ride_height_mm`, `static_camber_deg`, `camber_gain_deg_per_mm`, `anti_dive` / `anti_lift` / `anti_squat`, 과도용 `damping_ratio` 또는 `damper_rate_Ns_per_mm` |
| `brakes` | `bias_front` (null = 이상적 배분) |
| `aero` | `type`(constant/table/response_surface), `file`, `default_mode`, `cla_scale`, `cda_scale`, `cop_shift`, `drag_height_mm`, `floor_width_mm`, `bottoming_clearance_mm`, `cop_lateral_shift_mm_per_deg`, `strategy` |
| `aero.strategy` | `type`(fixed/rule/ideal), `open_mode`, `closed_mode`, `open_radius_m`, `min_length_m`, `min_speed_kmh`, `open_delay_s`, `close_delay_s`, `close_anticipation` |
| `battery` | `capacity_kWh`, `usable_fraction`, `max_power_kW`, `regen_power_kW`, `regen_axle_share` / 팩 모델: `series`, `parallel`, `soc`, `soc_min`, `soc_stages`, `interconnect_resistance_mOhm`, `max_current_A`, `cell:` (`capacity_Ah`, `resistance_mOhm`, `max_current_A`, `v_min`, `ocv`) |
| `powertrain` | `type`(ev/ice), `drive`(rwd/awd), `differential`(lsd/open), `efficiency`, `ev:` `motor`(emrax_228)·`motor_variant`·`motor_cooling`·`dc_voltage_V`·`controller_current_A`·`n_motors`·`drivetrain_efficiency`·`inverter_efficiency`·`rotor_inertia_kgm2` 또는 `motor_torque_curve`·`max_rpm`, `reduction_ratio`, `power_limit_kW` / `ice:` 토크 곡선·기어비·`shift_time_s` |

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

OptimumLap 트랙 편집기처럼 직선/코너(길이, 반경, 방향) 세그먼트로 적습니다. 그 밖에 중심선·주행 라인 좌표
`x_m,y_m` 또는 `s_m,curvature_1pm` 형식도 읽습니다. `# closed: true`면 주기 랩(flying lap)입니다.

좌표 형식은 호 길이로 다시 샘플링한 뒤 Whittaker 평활화(곡률 변화율에 벌점)로 점 잡음을 지우고 곡률을 구합니다.
`# cutoff_m: 8`(기본)보다 짧은 파장의 흔들림은 잡음으로 봅니다. 곡률은 위치의 2차 미분이라 1 cm 잡음도 보간 스플라인에서는
곡률 스파이크(→ 그 지점 정점 속도 하락)가 되기 때문입니다. 클로소이드 진입부가 있는 시험 트랙에서 잡음이 없으면 랩타임
오차 −0.1 %, 1 cm 잡음 −0.05 %, 3 cm 잡음 +0.5 %입니다(차단 파장 12 m).

### Assetto Corsa 트랙

```bash
aerolap import-track "assettocorsa/content/tracks/<track>/ai/fast_lane.ai" --out data/tracks/my_track.csv
```

```bash
aerolap run data/vehicles/fsae_ev_aero.yaml data/tracks/my_track.csv
```

- `ai/fast_lane.ai`(AI 레이싱 라인 = 실제로 달리는 경로)를 읽어 평면 좌표(AC의 x, −z), 고도, 트랙 폭(있으면)을 CSV로
  씁니다. `ideal_line.ai`도 같은 형식입니다. `.ai` 파일을 `run`, `events` 등에 바로 넣어도 됩니다
- 폐회로 여부는 시작·끝 간격으로 판단합니다(`--closed yes/no`로 지정). `--reverse`는 반대 방향, `--cutoff`는 평활화 파장[m]
- 고도(경사)는 CSV에 남기지만 랩 솔버는 아직 쓰지 않습니다
- 바이너리 형식은 커뮤니티 문서(Content Manager의 `AiSpline`)를 따르며, FSK 2026 실제 파일(`data/import/fsk2026_fast_lane.ai`:
  v7, 872점, 1 m 간격, 트랙 폭 5.5 m)로 확인했습니다. 3D 모델(`.kn5`)에서 트랙 경계를 뽑지는 않습니다

### OptimumLap 파일 (`.oltra` 트랙, `.olveh` 차량)

```bash
aerolap import-track data/import/FSK2026_GiantCorners_ConeMap_V4_851m.oltra --out data/tracks/fsk2026_endurance.csv
```

```bash
aerolap import-vehicle data/import/fsk2026_4.0.olveh --out data/vehicles/fsk2026_optimumlap.yaml
```

```bash
aerolap import-vehicle data/import/fsk2026_4.0.olveh --full --out data/vehicles/fsk2026.yaml
```

- 트랙: 구간(길이, 반경, 좌/우)을 그대로 세그먼트 CSV로 씁니다. `.oltra`를 `run` 등에 바로 넣어도 됩니다.
  `--smooth 15`는 구간 선을 좌표로 바꿔 15 m 파장으로 평활화합니다(콘맵 트랙의 곡률 잡음 제거, 아래 결과)
- 차량(기본): **OptimumLap 동등** — 상수 공력, 파일의 토크 곡선(로터 관성 없음), 배터리 없음. `--level L0`로 OptimumLap과 비교합니다
- 차량(`--full`): 질량·타이어·공력은 파일 값, 구동계는 모터 모델(기본 EMRAX 228 MV LC 550 V, 배터리 측 80 kW)
- OptimumLap 파일에 없는 값(무게중심 높이, 휠베이스·트레드, 서스펜션, 브레이크, 공력 밸런스, 배터리)은 `--base` 차량
  (기본 예제 EV)에서 가져오고 YAML 맨 위에 **ASSUMED** 목록으로 적습니다. 실제 값으로 바꾸세요

| OptimumLap | aerolap |
|---|---|
| `mass`, `weightdrivenwheel` | `mass.total_kg`, `mass.weight_dist_front` = 1 − 값 (RWD) |
| `longfriction`, `latfriction` (× `gripfactor`) | `tires.mu_x`, `tires.mu_y` |
| `tireradius`, `tirerollingdrag` | `tires.radius_mm`, `tires.rolling_resistance` |
| `liftcoefficient`, `dragcoefficient` (× `aerofactor`), `frontarea` | `aero.cl`, `aero.cd`, `aero.frontal_area_m2` (`type: constant`) |
| `airdensity` | `environment.air_density_kg_m3` |
| `torquedata`(× `powerfactor`), `rpmdata`, `revlimit` | `powertrain.ev.motor_torque_curve`, `max_rpm` (기어 2개 이상이면 `ice`) |
| `finaldriveratio` × 기어비, `driveefficiency` | `reduction_ratio`, `powertrain.efficiency` |

형식은 실제 파일에서 읽어 낸 것입니다(.NET BinaryWriter, 구간 방향 +1 = 좌 — 파일의 `percentleft` 통계로 확인).
하중 민감도·`mass*friction` 값이 0이 아니면 매핑하지 않고 경고합니다.

### FSK 2026 결과

OptimumLap 동등(L0) 비교, 콘맵 트랙 곡률 잡음, 전체 모델·종목·배터리·공력 결과: [docs/results_fsk2026.md](docs/results_fsk2026.md)

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

## 공력 장치 (프론트 윙 / 바닥 / 리어 윙)

차량 YAML의 `aero.components`에 장치를 나눠 적습니다. 장치마다 **물리 기반 모델**(`type: wing` / `type: floor`)이나 단순 모델을
고를 수 있습니다. 물리 기반 모델은 CFD 없이도 받음각·형상·지상고에서 계수를 계산하고, **차고·피치·롤·요에 따라 변합니다**.
그래서 서스펜션 움직임이 공력을 바꾸는 효과(L3·L4)가 aero map 없이도 들어갑니다.

```yaml
aero:
  type: constant            # 맨 차체(섀시·바퀴·드라이버) 계수 + 아래 장치들의 합
  cla: 0.0
  cda: 0.299
  components:
    front: {type: wing, span_m: 1.0, chord_m: 0.25, elements: 1, aoa_deg: 2.0, range: [-4, 14],
            x_mm: -650, height_mm: 80, endplate_factor: 1.5}
    side:  {type: floor, area_m2: 1.0, cl_ref: 0.34, height_mm: 40, x_mm: 800, height_exponent: 0.6,
            stall_mm: 12, rake_gain: 0.05, cd_area: 0.04, scale: 1.0}
    rear:  {type: wing, span_m: 1.0, chord_m: 0.30, elements: 2, aoa_deg: 2.0, range: [-4, 18],
            x_mm: 1800, height_mm: 950, endplate_factor: 1.8}
```

| 장치 | 계산 | 설계 변수 |
|---|---|---|
| `wing` | 유한 날개 양력 기울기 `a = a0/(1 + a0/(π e AR_eff))`, `AR_eff = endplate_factor × 스팬/코드`; `CL = CL0 + a·α`, CL_max 근처 부드러운 실속; 항력 = 형상항력 + **유도항력 `CL²/(π e AR_eff)`** → 받음각을 키우면 L/D가 나빠짐. 지상고가 코드보다 낮으면 **지면효과**(낮을수록 증가, `h_stall_mm` 아래 지면 실속), 롤 때 양 끝 지상고 차이, 요 `cos²` 손실. 엘리먼트 수별 기본 CL0·CL_max·CD0(`cl0`, `cl_max`, `cd0`로 덮어쓰기) | `aoa_deg` |
| `floor` | `ClA = scale·cl_ref·면적·(h_ref/h)^n`, h는 그 위치의 지상고(앞뒤 차고 보간 = 히브·피치), `stall_mm` 아래 **막힘(choke)으로 급감**, 레이크 1°마다 `rake_gain`, 롤·요 손실 | `scale` |
| (type 없음) | `cla`·`cda` × `scale`, 또는 CFD 점 `points` 보간 | `scale` |

- `x_mm`(앞축 기준 위치, 앞이 −)에서 앞축 분담이 정해집니다: `1 − x/휠베이스`
- 26년도 차의 장치 형상은 **가정값**이고 정적 합계를 OptimumLap 값(ClA 0.997 / CdA 0.470)에 맞췄습니다

**공력 장치 최적화** — 윙은 받음각(°, 장치 `range`), 바닥은 크기를 찾습니다:

```bash
aerolap aero-split data/vehicles/fsk2026.yaml data/tracks/fsk2026_endurance_smoothed.csv --level L4 --objective endurance_time
```

출력: ① 장치 하나씩 바꾼 개별 스윕, ② 세 장치를 함께 바꾼 동시 최적값(언더스티어 지수 `--understeer 1.0,1.3` 범위 안),
③ 장치별 ClA·CdA·L/D·앞축 분담·차고 민감도·실속 여유 표와 `optimized_vehicle.yaml`. 장치들은 밸런스로 서로 얽혀 있으므로
**만들 값은 동시 최적값**입니다.

**공력 교환비** — 다운포스·항력·밸런스·질량·장치 1단위가 랩(또는 배터리 한계 내구)에서 몇 초인지:

```bash
aerolap aero-exchange data/vehicles/fsk2026.yaml data/tracks/fsk2026_endurance_smoothed.csv --objective endurance_time
```

"ClA +0.1 = CdA +x" 는 다운포스를 얻으려고 추가 항력을 그만큼까지 써도 된다는 기준이고, "ClA +0.1 = y kg"은 경량화와 비교한 값입니다.

**공력 작동 영역·맵 점검** — 랩 동안 차가 aero map(또는 장치 모델)의 어디를 쓰는지:

```bash
aerolap aero-envelope data/vehicles/fsk2026.yaml data/tracks/fsk2026_endurance_smoothed.csv
```

차고·레이크·롤·요·속도의 분포(5/50/95 %), 상황별(제동/코너/가속) 공력 밸런스와 그 이동(제동 때 앞으로 가면 진입 오버스티어 경고),
맵 범위 밖·바닥 닿음·바닥 막힘·앞 윙 지면 실속 구간의 비율과 위치를 내고, 차고 산점을 ClA 등고선 위에 그립니다.
세 기능 모두 GUI에 버튼이 있습니다(계산 탭: 작동 영역, 스윕·최적화 탭: 교환비·장치 최적화).

**장치별 CFD 값 넣기**: 크기·받음각을 몇 개 바꿔 돌린 결과가 있으면 `points`로 넣습니다. 그 사이는 보간하므로 "L/D 일정" 가정 대신
실제 다운포스·항력 관계가 최적 크기를 정합니다.

```yaml
    rear:
      cop_front: -0.16
      scale: 1.0
      points: [[0.5, 0.25, 0.06], [1.0, 0.40, 0.13], [1.5, 0.48, 0.24]]   # [scale, ClA, CdA]
```

- 개별 스윕은 빠른(거친) 계산 값, "best … alone"과 동시 최적값은 **정밀 계산으로 다시 평가한 값**이라 서로 비교할 수 있습니다
- 언더스티어 지수 하한은 저속(기계적) 밸런스보다 높게 잡을 수 없습니다. 그러면 어떤 크기로도 만족할 수 없으므로
  "유효한 최적값 아님" 경고가 나옵니다. 지수는 GUI "코너링 한계 · 밸런스"로 먼저 확인하세요

26년도 차 결과: [docs/results_fsk2026.md](docs/results_fsk2026.md) 4절. 장치 분할은 가정값이므로 장치별 CFD를 넣어야 실제 설계값이 됩니다.

## MATLAB 연동

계산은 Python 하나로 하고 MATLAB에는 결과를 넘기는 방식을 권합니다. 장단점 비교와 이유는
[docs/python_vs_matlab.md](docs/python_vs_matlab.md)에 있습니다.

```bash
aerolap ggv data/vehicles/fsae_ev_aero.yaml --all-modes --export --out out/ggv
```

```bash
aerolap run data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_autocross_example.csv --mat --out out/run
```

MATLAB/Octave에서 `S = load('out/ggv/ggv.mat')`, `L = load('out/run/trace.mat')`. `matlab/aerolap_ggv_demo.m`은 GGV를
그리고 그 GGV로 점질량 랩을 돌리는 예제입니다(Octave에서 실행 확인).

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
- 과도 해석에도 타이어 온도, 비선형 댐퍼, 범프 스톱, 바닥 접촉 반력, 회생 제동의 차량 거동 영향(에너지만 계산),
  좌우 구동력 차이에 의한 요 모멘트는 없습니다. 노면 경사(고도)도 아직 반영하지 않습니다.
  횡 동역학은 소각도 이륜(bicycle) 근사이고 드라이버는 라인을 최적화하지 않습니다

## 개발

```bash
python -m pytest
```

```bash
ruff check src tests
```

GitHub에 push하면 `.github/workflows/ci.yml`이 같은 두 가지를 자동으로 돌립니다.

```bash
# 예제 aero map·트랙 재생성
python scripts/generate_example_data.py
```

구조와 과도 솔버 설계: [docs/architecture.md](docs/architecture.md)
