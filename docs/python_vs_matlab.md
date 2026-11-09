# GGV 활용: Python 단독 vs MATLAB 연동

## 결론

**계산은 Python(aerolap) 하나로 하고, MATLAB은 결과를 받아 쓰는 쪽(후처리, 팀 기존 코드와 비교)으로만 쓰는 것을 추천합니다.**
GGV를 MATLAB에서 다시 만들거나 MATLAB에서 랩 솔버를 따로 유지하는 것은 권하지 않습니다.

이유는 이 프로그램의 GGV가 "한 번 만들어 쓰는 표"가 아니기 때문입니다.

1. **GGV는 셋업마다 새로 계산되는 결과물입니다.** 엔벨로프의 각 점 `(v, ay, ax)`에서 공력 → 4륜 하중 → 히브·피치·롤
   → 차고 → aero map → 다시 공력을 평형까지 풉니다. 스프링, ARB, 정적 차고, 윙(`cla_scale`, `cop_shift`)을 바꾸면
   GGV 자체가 바뀝니다. 스윕·최적화는 평가 한 번마다 GGV를 다시 만듭니다(예제 최적화 250회 = GGV 250장 이상).
   MATLAB이 export된 GGV를 **쓰기만** 하면 셋업 변경을 반영할 수 없고, MATLAB에서 **만들려면** 서스펜션·공력·타이어·모터
   모델 전체를 다시 구현해야 합니다.
2. **구현이 두 개면 결과가 어긋납니다.** 같은 GGV(예제 EV, high_df 모드)를 `matlab/aerolap_ggv_demo.m`(점질량 QSS,
   전진 오일러 적분)에 넣으면 오토크로스 35.009 s, 내구 랩 44.641 s가 나오고, Python 솔버(L4)는 34.799 s, 44.367 s입니다.
   **+0.6 % (약 0.2–0.3 s)** 차이로, 셋업 최적화로 얻은 이득(−0.10 s)보다 큽니다. 적분 방식, 노드 배치, 코너 입출구 곡률
   처리만 달라도 이 정도가 나옵니다. 두 숫자를 맞추는 데 시간을 쓰게 됩니다.
3. **GGV 밖에 있는 효과가 많습니다.** 랩을 따라 바뀌는 차고·DRS 전환과 작동 지연, 배터리 에너지 한계(출력 캡), 모터 열
   점검, 과도 해석(노면, 댐퍼, 횡·요 운동)은 GGV 한 장으로 전달되지 않습니다.

## 장단점 비교

| | **A. Python 단독 (현재 구조)** | **B. GGV를 MATLAB으로 넘겨 MATLAB에서 랩·분석** | **C. MATLAB에서 Python 호출** |
|---|---|---|---|
| 방식 | aerolap이 GGV 생성·랩·스윕·최적화·과도 해석을 모두 수행 | `aerolap ggv --export` → `ggv.mat` → MATLAB 랩 솔버/분석 | MATLAB에서 `system('aerolap ...')` 또는 `py.aerolap...` 호출, 결과 `.mat` 로드 |
| 물리 일관성 | 한 모델 → GGV, 랩, 최적화 결과가 서로 맞음 | GGV 생성(Python)과 랩 적분(MATLAB)이 다른 코드 → 0.6 % 차이 실측 | A와 같음 (계산은 Python) |
| 셋업 변경 반영 | 자동 (평가마다 GGV 재계산) | 셋업마다 Python에서 다시 export | 자동 (MATLAB이 Python을 다시 부름) |
| 최적화 | scipy (Powell / DE), 제약·페널티 구현됨 | MATLAB Optimization Toolbox 가능하지만 GGV를 매번 다시 받아야 함 | MATLAB 최적화기 + Python 평가 가능, 호출 오버헤드 |
| 속도 | GGV 1장 1.3 s(정밀) / 0.27 s(거친), 오토크로스 랩 2.8 s / 0.6 s | MATLAB 랩 적분은 빠름(GGV 생성 시간 제외) | A + 호출·데이터 변환 오버헤드 |
| 비용 | 무료, `pip install` | MATLAB 라이선스(+툴박스). 학생팀은 학교·스폰서 라이선스가 있는 경우가 많음 | 둘 다 필요 |
| 유지보수 | 코드 한 벌, git·자동 테스트(현재 76개) | 두 언어 두 벌의 솔버를 같이 고쳐야 함 | Python 환경 버전을 MATLAB 지원 버전에 맞춰야 함(`pyenv`) |
| 팀 숙련도 | 팀이 MATLAB 위주면 학습 비용 | 익숙한 환경 | MATLAB 사용자는 그대로, 계산 코드 수정은 Python |
| 확장 | 과도 솔버, 에너지, AC 트랙 임포터 이미 있음 | Simulink / Vehicle Dynamics Blockset과 연결 쉬움 | Simulink에서 GGV를 한계 표로 쓰는 정도는 가능 |
| 시각화 | matplotlib PNG 자동 생성 (대화형은 Jupyter) | 대화형 그래프·데이터 브라우징이 편함 | MATLAB 그래프 사용 |
| 실측 데이터 비교 | pandas로 가능 | 팀에 MATLAB 로거 분석 코드가 있으면 바로 비교 | 같음 |

## 상황별 권장

| 상황 | 권장 |
|---|---|
| 스프링·ARB·차고·윙·배터리 용량 같은 **셋업·설계 결정** | **A**. 스윕·최적화·`energy`를 Python에서 |
| 팀에 이미 MATLAB 보고서·로거 분석 도구가 있다 | **A로 계산 → `.mat`로 넘겨 MATLAB에서 그림·비교** (`aerolap ggv --export`, `aerolap run --mat`) |
| MATLAB 화면에서 셋업을 바꿔 가며 보고 싶다 | **C**. `system('aerolap run ... --set ... --mat')` 후 `load` — 솔버는 다시 짜지 않기 |
| Simulink 차량 모델의 드라이버 한계 표로 GGV가 필요 | **B** (한 셋업의 스냅샷이라는 점을 감안) |

## 사용법

```bash
# GGV를 CSV와 MATLAB .mat로 (모든 aero 모드)
aerolap ggv data/vehicles/fsae_ev_aero.yaml --all-modes --export --out out/ggv
```

```bash
# 랩 트레이스를 MATLAB .mat로 (모든 채널: 속도, 차고, 4륜 하중, 배터리 출력 ...)
aerolap run data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_autocross_example.csv --mat --out out/run
```

MATLAB / GNU Octave:

```matlab
S = load('out/ggv/ggv.mat');           % S.ggv.high_df.v, .frac, .ay_max, .ax_acc, .ax_brk (nv x nf, m/s^2)
L = load('out/run/trace.mat');         % L.lap.s, L.lap.v, L.lap.rh_front_mm, ..., L.meta.lap_time_s
addpath('matlab');
out = aerolap_ggv_demo('out/ggv/ggv.mat', 'data/tracks/fsae_autocross_example.csv');  % 그래프 + 점질량 랩
```

```matlab
% C 방식: MATLAB에서 셋업을 바꿔 Python으로 계산
system('aerolap run data/vehicles/fsae_ev_aero.yaml data/tracks/fsae_autocross_example.csv --set aero.cop_shift=0.02 --mat --out out/run_cop');
R = load('out/run_cop/trace.mat');
```

`ggv.mat` 구조: `ggv.<모드>.v` [m/s] (nv×1), `frac` = `ay/ay_max` (1×nf, 한계 근처가 촘촘한 sin 간격), `ay_max` (nv×1),
`ay`, `ax_acc`, `ax_brk`, `ax_coast` (nv×nf, m/s², `ax_brk` < 0), `meta`(차량, 모델 레벨, 단위). `ggv.csv`는 같은 내용을
긴 형식(모드, 속도, 횡 비율마다 한 행)으로 담습니다. `matlab/aerolap_ggv_demo.m`은 MATLAB 기본 함수만 쓰며 GNU Octave에서
실행을 확인했습니다.
