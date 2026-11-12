"""Desktop window for aerolap (tkinter, no extra install): ``py -m aerolap.gui``.

Pick a vehicle and a track, edit the usual parameters, press a button; text results and graphs
appear on the right and are also saved under ``out/gui/<task>/``. Calculations run in a worker
thread (progress bar, cancel); every run adds a line to the history tab so setups can be compared.
"""

from __future__ import annotations

import copy
import csv
import os
import queue
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .config import ConfigError, get_path, load_vehicle_config, set_path, unknown_keys, validate
from .features import LEVELS
from .gui_tasks import (ROOT, TASKS, TaskCancelled, changed_params, editable_params, error_params, error_text,
                        import_file, save_vehicle_yaml)

DEFAULT_VEHICLE = ROOT / "data" / "vehicles" / "fsk2026.yaml"
DEFAULT_TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"
OUT = Path.cwd() / "out" / "gui"
OBJECTIVES = {"한 랩 시간": "lap_time", "내구 시간 (배터리 한계)": "endurance_time", "스키드패드 시간": "skidpad_time"}
MAIN_TASKS = ("lap", "compare", "events", "energy", "cornering", "envelope")


def _open_path(path: Path) -> None:
    if sys.platform.startswith("win"):
        os.startfile(path)  # type: ignore[attr-defined]  # noqa: S606
    elif sys.platform == "darwin":
        subprocess.Popen(["open", str(path)])
    else:
        subprocess.Popen(["xdg-open", str(path)])


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("aerolap - FSAE 랩 시뮬레이터")
        root.geometry("1400x900")
        self.queue: queue.Queue = queue.Queue()
        self.busy = False
        self.cancel = threading.Event()
        self.cfg: dict | None = None
        self.param_vars: dict[str, tk.StringVar] = {}
        self.param_entries: dict[str, tk.Entry] = {}
        self.param_orig: dict[str, str] = {}
        self.param_labels: dict[str, str] = {}
        self.last_result = None
        self.images: list[tk.PhotoImage] = []
        self.image_spans: list[tuple[int, int, Path]] = []
        self.history: list[dict] = []
        self.t_start = 0.0

        self.vehicle_var = tk.StringVar(value=str(DEFAULT_VEHICLE))
        self.track_var = tk.StringVar(value=str(DEFAULT_TRACK))
        self.level_var = tk.StringVar(value="L5")
        self.coarse_var = tk.BooleanVar(value=False)
        self.distance_var = tk.StringVar(value="22")
        self.track_width_var = tk.StringVar(value="")
        self.sweep_wing_var = tk.BooleanVar(value=True)
        self.aero_detail_var = tk.BooleanVar(value=False)
        # aero-device optimisation
        self.obj_var = tk.StringVar(value="내구 시간 (배터리 한계)")
        self.lo_var, self.hi_var = tk.StringVar(value=""), tk.StringVar(value="")
        self.us_lo_var, self.us_hi_var = tk.StringVar(value="1.0"), tk.StringVar(value="1.3")
        self.evals_var = tk.StringVar(value="100")
        # parameter sweep
        self.sw_param_var = tk.StringVar()
        self.sw_lo_var, self.sw_hi_var, self.sw_n_var = tk.StringVar(), tk.StringVar(), tk.StringVar(value="7")
        self.sw_obj_var = tk.StringVar(value="한 랩 시간")
        self.section_csv_var = tk.StringVar(value=str(ROOT / "data" / "aero_sections" / "synthetic_front_summary.csv"))
        self.section_comp_var = tk.StringVar(value="front")
        # whole-package automatic optimisation
        self.fix_var = tk.StringVar(value="")
        self.auto_options_var = tk.BooleanVar(value=True)
        self.auto_evals_var = tk.StringVar(value="300")

        self._build()
        self.load_vehicle()
        root.after(100, self._poll)

    # ------------------------------------------------------------------ layout
    def _build(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        for r, (label, var, cmd) in enumerate((("차량 YAML", self.vehicle_var, self.pick_vehicle),
                                               ("트랙", self.track_var, self.pick_track))):
            ttk.Label(top, text=label, width=10).grid(row=r, column=0, sticky="w")
            ttk.Entry(top, textvariable=var, width=110).grid(row=r, column=1, sticky="we", padx=4)
            ttk.Button(top, text="찾기...", command=cmd).grid(row=r, column=2)
        ttk.Button(top, text="차량 다시 불러오기", command=self.load_vehicle).grid(row=0, column=3, padx=4)
        ttk.Button(top, text="OptimumLap / AC 파일 가져오기...", command=self.import_dialog).grid(row=1, column=3, padx=4)
        opts = ttk.Frame(top)
        opts.grid(row=2, column=0, columnspan=4, sticky="w", pady=4)
        ttk.Label(opts, text="모델 레벨").pack(side="left")
        ttk.Combobox(opts, textvariable=self.level_var, values=list(LEVELS), width=5, state="readonly").pack(
            side="left", padx=4)
        ttk.Label(opts, text="L0 = OptimumLap 방식(점질량) … L5 = 전체 모델", foreground="#666").pack(side="left", padx=6)
        ttk.Checkbutton(opts, text="빠른 계산(거친 엔벨로프, 오차 약 0.1 %)", variable=self.coarse_var).pack(
            side="left", padx=12)
        top.columnconfigure(1, weight=1)

        # status row (bottom): progress, cancel, message
        bottom = ttk.Frame(self.root, padding=(6, 2))
        bottom.pack(side="bottom", fill="x")
        self.progress = ttk.Progressbar(bottom, mode="indeterminate", length=160)
        self.progress.pack(side="left")
        self.cancel_btn = ttk.Button(bottom, text="취소", command=self.cancel_task, state="disabled")
        self.cancel_btn.pack(side="left", padx=6)
        self.status = ttk.Label(bottom, text="준비", anchor="w")
        self.status.pack(side="left", fill="x", expand=True)
        self.log = tk.Text(self.root, height=6, font=("Consolas", 9), background="#f4f4f4")
        self.log.pack(side="bottom", fill="x")

        pan = ttk.PanedWindow(self.root, orient="horizontal")
        pan.pack(fill="both", expand=True)
        left = ttk.Notebook(pan, width=440)
        right = ttk.Notebook(pan)
        pan.add(left, weight=0)
        pan.add(right, weight=1)
        self.task_buttons: list[ttk.Button] = []

        # --- tab 1: single calculations
        tasks = ttk.Frame(left, padding=8)
        left.add(tasks, text="계산")
        for key in MAIN_TASKS:
            self._task_button(tasks, key).pack(fill="x", pady=3)
        g = ttk.LabelFrame(tasks, text="내구 / 에너지 설정", padding=6)
        g.pack(fill="x", pady=8)
        self._row(g, 0, "내구 거리 [km]", self.distance_var)
        ttk.Checkbutton(g, text="공력 패키지 크기 스윕 포함 (현재 값 × 0.6 … 1.4)",
                        variable=self.sweep_wing_var).grid(row=1, column=0, columnspan=2, sticky="w")
        ln = ttk.LabelFrame(tasks, text="레이싱 라인 (트랙 폭 안에서)", padding=6)
        ln.pack(fill="x", pady=(0, 8))
        self._row(ln, 0, "트랙 폭 [m] (빈칸 = 파일의 폭)", self.track_width_var)
        self._task_button(ln, "line", "레이싱 라인 최적화").grid(row=1, column=0, columnspan=2, sticky="we", pady=2)
        self.use_line_btn = ttk.Button(ln, text="최적 라인을 트랙으로 사용", command=self.use_line, state="disabled")
        self.use_line_btn.grid(row=2, column=0, columnspan=2, sticky="we")
        ttk.Label(tasks, text="결과는 out/gui/<작업>/ 에도 저장됩니다.\n그래프를 더블클릭하면 원본 이미지가 열립니다.",
                  foreground="#666", justify="left").pack(anchor="w", pady=6)
        ttk.Button(tasks, text="결과 폴더 열기", command=self.open_out).pack(fill="x")

        # --- tab 2: sweep / optimisation
        opt = ttk.Frame(left, padding=8)
        left.add(opt, text="스윕 · 최적화")
        sw = ttk.LabelFrame(opt, text="파라미터 스윕 (값 하나를 바꿔 가며 비교)", padding=6)
        sw.pack(fill="x")
        ttk.Label(sw, text="파라미터").grid(row=0, column=0, sticky="w")
        self.sw_combo = ttk.Combobox(sw, textvariable=self.sw_param_var, width=34, state="readonly")
        self.sw_combo.grid(row=0, column=1, sticky="w")
        self.sw_combo.bind("<<ComboboxSelected>>", lambda e: self._sweep_defaults())
        self._row(sw, 1, "시작 값", self.sw_lo_var)
        self._row(sw, 2, "끝 값", self.sw_hi_var)
        self._row(sw, 3, "점 개수", self.sw_n_var)
        ttk.Label(sw, text="목적").grid(row=4, column=0, sticky="w")
        ttk.Combobox(sw, textvariable=self.sw_obj_var, values=list(OBJECTIVES), width=22, state="readonly").grid(
            row=4, column=1, sticky="w")
        self._task_button(sw, "sweep", "스윕 실행").grid(row=5, column=0, columnspan=2, sticky="we", pady=4)

        ex = ttk.LabelFrame(opt, text="공력 교환비 (목적은 아래 공력 장치 최적화의 목적)", padding=6)
        ex.pack(fill="x", pady=(10, 0))
        self._task_button(ex, "exchange", "교환비 계산: ClA·CdA·밸런스·질량·장치별 1단위의 값어치").pack(fill="x")
        a = ttk.LabelFrame(opt, text="공력 장치 최적화 (윙 받음각 / 바닥 크기)", padding=6)
        a.pack(fill="x", pady=10)
        ttk.Label(a, text="목적").grid(row=0, column=0, sticky="w")
        ttk.Combobox(a, textvariable=self.obj_var, values=list(OBJECTIVES)[:2], width=22, state="readonly").grid(
            row=0, column=1, sticky="w")
        self._row(a, 1, "scale 하한 (빈칸=장치 range)", self.lo_var)
        self._row(a, 2, "scale 상한 (윙은 각도 range)", self.hi_var)
        self._row(a, 3, "언더스티어 지수 하한", self.us_lo_var)
        self._row(a, 4, "언더스티어 지수 상한", self.us_hi_var)
        self._row(a, 5, "최적화 평가 횟수", self.evals_var)
        self._task_button(a, "aero_split", "최적화 실행").grid(row=6, column=0, columnspan=2, sticky="we", pady=4)
        self._task_button(a, "aero_all", "공력 전체 계산 (작동 영역 → 교환비 → 최적화 → 최적 차 점검)").grid(
            row=7, column=0, columnspan=2, sticky="we", pady=(0, 4))
        self.apply_btn = ttk.Button(a, text="최적값을 파라미터에 적용", command=self.apply_optimum, state="disabled")
        self.apply_btn.grid(row=8, column=0, columnspan=2, sticky="we")
        ttk.Label(opt, text="언더스티어 지수 > 1: 앞이 먼저 한계(안정), < 1: 오버스티어.\n"
                            "하한은 '코너링 한계 · 밸런스'에 나오는 저속 값보다 높게 잡을 수 없습니다\n"
                            "(저속 밸런스는 스프링·ARB가 정하고 공력으로는 못 바꿉니다).",
                  foreground="#666", justify="left").pack(anchor="w")

        au = ttk.LabelFrame(opt, text="공력 전체 자동 최적화 (보전 파라미터 외 전부, 목적·밸런스는 위 설정)", padding=6)
        au.pack(fill="x", pady=(0, 6))
        ttk.Label(au, text="보전(고정)").grid(row=0, column=0, sticky="w")
        ttk.Entry(au, textvariable=self.fix_var, width=30).grid(row=0, column=1, sticky="we")
        ttk.Label(au, text="예: rear, *.span_m, @position, front.elements.main.chord_mm",
                  foreground="#666").grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(au, text="장치·플랩 on/off 옵션도 탐색", variable=self.auto_options_var).grid(
            row=2, column=0, columnspan=2, sticky="w")
        self._row(au, 3, "평가 횟수", self.auto_evals_var)
        self._task_button(au, "auto_aero", "전체 자동 최적화 실행").grid(row=4, column=0, columnspan=2, sticky="we", pady=2)
        self.apply_auto_btn = ttk.Button(au, text="결과 패키지를 차량에 적용", command=self.apply_auto,
                                         state="disabled")
        self.apply_auto_btn.grid(row=5, column=0, columnspan=2, sticky="we")

        cfd = ttk.LabelFrame(opt, text="CFD 단면 순위 (AERO_AUTOMATION summary.csv)", padding=6)
        cfd.pack(fill="x", pady=6)
        ttk.Entry(cfd, textvariable=self.section_csv_var, width=34).grid(row=0, column=0, columnspan=2, sticky="we")
        ttk.Button(cfd, text="찾기...", command=self.pick_section_csv).grid(row=0, column=2)
        ttk.Label(cfd, text="장치").grid(row=1, column=0, sticky="w")
        ttk.Combobox(cfd, textvariable=self.section_comp_var, values=["front", "rear"], width=8,
                     state="readonly").grid(row=1, column=1, sticky="w")
        self._task_button(cfd, "rank_sections", "순위 계산 (목적·밸런스는 위 최적화 설정)").grid(
            row=2, column=0, columnspan=3, sticky="we", pady=2)
        self.apply_section_btn = ttk.Button(cfd, text="1위 단면을 차량에 적용", command=self.apply_section,
                                            state="disabled")
        self.apply_section_btn.grid(row=3, column=0, columnspan=3, sticky="we")

        # --- tab 3: parameters (scrollable)
        pframe = ttk.Frame(left)
        left.add(pframe, text="파라미터")
        bar = ttk.Frame(pframe, padding=4)
        bar.pack(fill="x")
        ttk.Button(bar, text="YAML로 저장...", command=self.save_vehicle).pack(side="left")
        ttk.Button(bar, text="되돌리기", command=self.load_vehicle).pack(side="left", padx=4)
        ttk.Label(bar, text="파란색 = 바꾼 값", foreground="#0050c8").pack(side="left", padx=8)
        ttk.Checkbutton(bar, text="공력 장치 상세 (위치·지상고·코드·갭·냉각 ...)", variable=self.aero_detail_var,
                        command=self._layout_params).pack(side="left", padx=4)
        tbar = ttk.Frame(pframe, padding=(4, 0))
        tbar.pack(fill="x")
        ttk.Label(tbar, text="타이어 모델").pack(side="left")
        self.tire_label = ttk.Label(tbar, text="단순", foreground="#666")
        self.tire_label.pack(side="left", padx=4)
        ttk.Button(tbar, text="Magic Formula .tir...", command=self.pick_tir).pack(side="left", padx=4)
        ttk.Button(tbar, text="단순 타이어로", command=lambda: self.set_tir(None)).pack(side="left")
        canvas = tk.Canvas(pframe, highlightthickness=0)
        sb = ttk.Scrollbar(pframe, orient="vertical", command=canvas.yview)
        self.param_frame = ttk.Frame(canvas)
        self.param_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.param_frame, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        canvas.bind("<Enter>", lambda e: canvas.bind_all("<MouseWheel>", lambda ev: canvas.yview_scroll(
            -1 if ev.delta > 0 else 1, "units")))
        canvas.bind("<Leave>", lambda e: canvas.unbind_all("<MouseWheel>"))

        # --- results: text / graphs / history
        tf = ttk.Frame(right)
        right.add(tf, text="결과")
        self.text = tk.Text(tf, wrap="none", font=("Consolas", 10))
        ys = ttk.Scrollbar(tf, orient="vertical", command=self.text.yview)
        xs = ttk.Scrollbar(tf, orient="horizontal", command=self.text.xview)
        self.text.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        ys.pack(side="right", fill="y")
        xs.pack(side="bottom", fill="x")
        self.text.pack(fill="both", expand=True)
        gf = ttk.Frame(right)
        right.add(gf, text="그래프")
        self.img_canvas = tk.Canvas(gf, background="white")
        gys = ttk.Scrollbar(gf, orient="vertical", command=self.img_canvas.yview)
        self.img_canvas.configure(yscrollcommand=gys.set)
        gys.pack(side="right", fill="y")
        self.img_canvas.pack(fill="both", expand=True)
        self.img_canvas.bind("<Double-Button-1>", self._open_image)
        hf = ttk.Frame(right)
        right.add(hf, text="비교 기록")
        hbar = ttk.Frame(hf, padding=4)
        hbar.pack(fill="x")
        ttk.Button(hbar, text="CSV로 저장...", command=self.save_history).pack(side="left")
        ttk.Button(hbar, text="기록 지우기", command=self.clear_history).pack(side="left", padx=4)
        ttk.Label(hbar, text="계산할 때마다 한 줄씩 쌓입니다 - 파라미터를 바꾸기 전후를 비교하세요.",
                  foreground="#666").pack(side="left", padx=8)
        cols = ("time", "task", "result", "changes", "setup")
        self.tree = ttk.Treeview(hf, columns=cols, show="headings")
        for c, (label, width) in zip(cols, (("시각", 70), ("계산", 130), ("결과", 430), ("바꾼 파라미터 (원래 값)", 300),
                                            ("차량 / 트랙 / 레벨", 260))):
            self.tree.heading(c, text=label)
            self.tree.column(c, width=width, anchor="w", stretch=c in ("result", "changes"))
        hys = ttk.Scrollbar(hf, orient="vertical", command=self.tree.yview)
        hxs = ttk.Scrollbar(hf, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=hys.set, xscrollcommand=hxs.set)
        hys.pack(side="right", fill="y")
        hxs.pack(side="bottom", fill="x")
        self.tree.pack(fill="both", expand=True)
        self.result_tabs = right

    def _task_button(self, parent, key: str, label: str | None = None) -> ttk.Button:
        b = ttk.Button(parent, text=label or TASKS[key][0], command=lambda: self.run_task(key))
        self.task_buttons.append(b)
        return b

    @staticmethod
    def _row(parent, r, label, var):
        ttk.Label(parent, text=label).grid(row=r, column=0, sticky="w")
        ttk.Entry(parent, textvariable=var, width=10).grid(row=r, column=1, sticky="w", pady=1)

    # ------------------------------------------------------------------ vehicle / params
    def pick_vehicle(self):
        p = filedialog.askopenfilename(initialdir=DEFAULT_VEHICLE.parent, filetypes=[("vehicle (YAML / OptimumLap)", "*.yaml *.olveh"),
                                                  ("all", "*.*")])
        if p:
            self.vehicle_var.set(p)
            self.load_vehicle()

    def pick_track(self):
        p = filedialog.askopenfilename(initialdir=DEFAULT_TRACK.parent,
                                       filetypes=[("track", "*.csv *.oltra *.ai"), ("all", "*.*")])
        if p:
            self.track_var.set(p)

    def load_vehicle(self):
        try:
            self.cfg = load_vehicle_config(self.vehicle_var.get(), warn=False)
        except Exception as exc:  # noqa: BLE001
            self.report_error(exc, "차량 파일 오류")
            return
        self._build_params()
        self._log(f"차량 불러옴: {self.cfg.get('name')}")
        self._show_tire()
        for msg in self.cfg.get("_notes", []):
            self._log(f"OptimumLap: {msg}")
        for msg in self.cfg.get("_warnings", []):
            self._log(f"경고: {msg}")

    def _show_tire(self) -> bool:
        """Show the tyre model of self.cfg; False (and an error message) when it cannot be built."""
        from .tire import tire_from_config

        try:
            tire = tire_from_config(self.cfg["tires"], self.cfg.get("_base_dir", "."))
        except Exception as exc:  # noqa: BLE001
            self.report_error(exc, "타이어 파일 오류")
            return False
        mf = getattr(tire, "model", "simple") == "mf"
        self.tire_label.configure(text=f"Magic Formula ({tire.source})" if mf else "단순 (mu, 하중 민감도, 캠버)")
        if mf:
            self._log("타이어: " + tire.describe())
        return True

    def pick_tir(self):
        p = filedialog.askopenfilename(initialdir=ROOT / "data" / "tires",
                                       filetypes=[("Magic Formula tyre", "*.tir"), ("all", "*.*")])
        if p:
            self.set_tir(str(Path(p).resolve()))

    def set_tir(self, path: str | None):
        """Switch the tyre model (Magic Formula file or the simple model), keeping the edited values."""
        if self.cfg is None:
            return
        edits = {k: v.get() for k, v in self.param_vars.items()}
        old_file, old_p = self.cfg["tires"].get("mf_file"), self.cfg["tires"].get("pressure_kPa")
        set_path(self.cfg, "tires.mf_file", path)
        if path and old_p is None:
            from .tire_mf import read_tir

            try:
                pres = read_tir(path).get("INFLPRES") or read_tir(path).get("NOMPRES")
            except OSError:
                pres = None
            if isinstance(pres, float) and pres > 0:
                set_path(self.cfg, "tires.pressure_kPa", round(pres / 1000.0, 1))
        if not self._show_tire():  # the file could not be read: keep the previous tyre
            set_path(self.cfg, "tires.mf_file", old_file)
            set_path(self.cfg, "tires.pressure_kPa", old_p)
            self._show_tire()
            return
        self._build_params()
        for k, v in edits.items():
            if k in self.param_vars:
                self.param_vars[k].set(v)

    def report_error(self, exc: Exception, title: str, tb: str = ""):
        """Full message in the dialog (parameter, condition, cause, fix); parameters involved marked red."""
        text = error_text(exc)
        self._log(f"[오류] {title}\n{text}" + (f"\n{tb}" if tb else ""))
        bad = [p for p in error_params(exc) if p in self.param_vars]
        if any(p not in self.param_entries for p in bad):
            self.aero_detail_var.set(True)
            self._layout_params()
        marked = [p for p in bad if p in self.param_entries]
        for p in marked:
            self.param_entries[p].configure(background="#ffd6d6")
        if marked:
            text += "\n\n파라미터 탭에서 빨간 칸: " + ", ".join(marked)
        messagebox.showerror(title, text)

    def clear_error_marks(self):
        for ent in self.param_entries.values():
            ent.configure(background="white")

    def _mark_changed(self, path: str):
        ent = self.param_entries.get(path)
        if ent is not None:
            changed = self.param_vars[path].get().strip() != self.param_orig[path]
            ent.configure(foreground="#0050c8" if changed else "black")

    def _sweep_defaults(self):
        """Start / end = current value -20 % .. +20 % when a sweep parameter is picked."""
        path = self.param_labels.get(self.sw_param_var.get())
        try:
            x = float(self.param_vars[path].get())
        except (KeyError, ValueError):
            return
        span = 0.2 * abs(x) if x else 0.2
        self.sw_lo_var.set(f"{x - span:g}")
        self.sw_hi_var.set(f"{x + span:g}")

    def current_cfg(self) -> dict:
        cfg = copy.deepcopy(self.cfg)
        for path, var in self.param_vars.items():
            txt = var.get().strip()
            if txt in ("", "None", "null"):
                set_path(cfg, path, None)
                continue
            try:
                set_path(cfg, path, float(txt))
            except ValueError:
                raise ConfigError(path=path, value=txt, condition="숫자여야 함",
                                  hint="파라미터 탭의 이 칸에 숫자를 넣으세요 (비우면 기본값)") from None
        validate(cfg)
        return cfg

    def save_vehicle(self):
        try:
            cfg = self.current_cfg()
        except Exception as exc:  # noqa: BLE001
            self.report_error(exc, "파라미터 오류")
            return
        p = filedialog.asksaveasfilename(initialdir=DEFAULT_VEHICLE.parent, defaultextension=".yaml",
                                         filetypes=[("vehicle YAML", "*.yaml")])
        if p:
            save_vehicle_yaml(cfg, p, self.vehicle_var.get())
            self._log(f"저장: {p}  (파일 맨 위 주석은 유지, 줄 끝 주석은 저장되지 않습니다)")

    def apply_optimum(self):
        res = getattr(self.last_result, "data", None)
        if res is None or getattr(res, "joint", None) is None:
            return
        for path, val in res.joint.best.items():
            if path in self.param_vars:
                self.param_vars[path].set(f"{val:.3f}")
        self._log("공력 장치 최적값을 파라미터 탭에 넣었습니다. 'YAML로 저장'으로 보관하세요.")

    def apply_auto(self):
        res = getattr(self.last_result, "data", None)
        if res is None or getattr(res, "best_cfg", None) is None or not hasattr(res, "option_changes"):
            return
        self.cfg["aero"]["components"] = copy.deepcopy(res.best_cfg["aero"]["components"])
        self.load_vehicle_keep()
        self._log(f"전체 자동 최적화 결과를 적용했습니다 (값 {len(res.changes)}개, 옵션 {len(res.option_changes)}개). "
                  "'YAML로 저장'으로 보관하세요.")

    def pick_section_csv(self):
        p = filedialog.askopenfilename(initialdir=ROOT / "data" / "aero_sections",
                                       filetypes=[("AERO_AUTOMATION summary", "*.csv"), ("all", "*.*")])
        if p:
            self.section_csv_var.set(p)

    def apply_section(self):
        rk = getattr(self.last_result, "data", None)
        if rk is None or getattr(rk, "best_cfg", None) is None:
            return
        comp = rk.component
        self.cfg["aero"]["components"][comp] = rk.best_cfg["aero"]["components"][comp]
        self.load_vehicle_keep()
        self._log(f"{comp} 윙을 {rk.ordered()[0].geometry_id} 단면으로 바꿨습니다. 'YAML로 저장'으로 보관하세요.")

    def load_vehicle_keep(self):
        """Rebuild the parameter list from self.cfg (after a structural change) keeping it as the base."""
        self._build_params()

    def _build_params(self):
        """One entry variable per editable value of self.cfg (all of them, shown or not), then lay out."""
        self.param_vars.clear()
        self.param_orig.clear()
        self.param_labels.clear()
        for label, path in editable_params(self.cfg, detail=True):
            text = f"{get_path(self.cfg, path, '')}"
            var = tk.StringVar(value=text)
            self.param_vars[path], self.param_orig[path] = var, text
            self.param_labels[f"{label}  ({path})"] = path
            var.trace_add("write", lambda *_, p=path: self._mark_changed(p))
        self.sw_combo.configure(values=list(self.param_labels))
        if self.sw_param_var.get() not in self.param_labels:
            self.sw_param_var.set("")
        self._layout_params()

    def _layout_params(self):
        """Show the short list, or every aero-device value when "공력 장치 상세" is ticked.

        Hidden values keep their variables, so edits, optimiser results and sweeps still apply to them.
        """
        for w in self.param_frame.winfo_children():
            w.destroy()
        self.param_entries.clear()
        if self.cfg is None:
            return
        shown = editable_params(self.cfg, detail=self.aero_detail_var.get())
        for r, (label, path) in enumerate(shown):
            ttk.Label(self.param_frame, text=label).grid(row=r, column=0, sticky="w", padx=4)
            ent = tk.Entry(self.param_frame, textvariable=self.param_vars[path], width=12)
            ent.grid(row=r, column=1, pady=1)
            ttk.Label(self.param_frame, text=path, foreground="#888").grid(row=r, column=2, sticky="w", padx=4)
            self.param_entries[path] = ent
            self._mark_changed(path)
        hidden = [p for p in self.param_vars if p not in self.param_entries]
        if hidden:
            changed = sum(self.param_vars[p].get().strip() != self.param_orig[p] for p in hidden)
            ttk.Label(self.param_frame, foreground="#666",
                      text=f"공력 장치 상세 {len(hidden)}개 숨김" + (f" (그중 {changed}개 바뀜)" if changed else "")
                      + " - 위의 '공력 장치 상세'를 켜면 보입니다").grid(
                row=len(shown), column=0, columnspan=3, sticky="w", padx=4, pady=6)

    def import_dialog(self):
        p = filedialog.askopenfilename(filetypes=[("OptimumLap / Assetto Corsa", "*.oltra *.olveh *.ai"),
                                                  ("all", "*.*")])
        if not p:
            return
        suf = Path(p).suffix.lower()
        opts = self.import_options(suf) if suf in (".olveh", ".oltra") else {}
        if opts is None:
            return
        dest = ROOT / "data" / ("vehicles" if suf == ".olveh" else "tracks")
        try:
            out = import_file(p, dest, **opts)
        except Exception as exc:  # noqa: BLE001
            self.report_error(exc, "가져오기 오류")
            return
        self._log(f"가져옴: {out}")
        if suf == ".olveh":
            self.vehicle_var.set(out)
            self.load_vehicle()
            power = ("구동계는 EMRAX 228 모터 모델 + 배터리입니다." if opts.get("full")
                     else "구동계는 OptimumLap 파일의 토크 곡선 그대로입니다(배터리 없음).")
            messagebox.showinfo("가져오기", f"차량을 불러왔습니다. {power}\nOptimumLap 파일에 없는 값(무게중심, "
                                "서스펜션, 공력 분할)은 예제 값입니다 - 파라미터 탭에서 고치세요.")
        else:
            self.track_var.set(out)

    def import_options(self, suf: str) -> dict | None:
        """Modal choice of the import variant; None = cancelled."""
        win = tk.Toplevel(self.root)
        win.title("가져오기 옵션")
        win.transient(self.root)
        frm = ttk.Frame(win, padding=10)
        frm.pack(fill="both", expand=True)
        result: dict = {}
        if suf == ".olveh":
            full = tk.BooleanVar(value=False)
            ttk.Label(frm, text="구동계").pack(anchor="w")
            ttk.Radiobutton(frm, text="OptimumLap 동등 - 파일의 토크 곡선 그대로 (OptimumLap과 L0 비교용)",
                            variable=full, value=False).pack(anchor="w", padx=8)
            ttk.Radiobutton(frm, text="전체 모델 - EMRAX 228 모터 + 배터리 출력·전압 한계 (내구 에너지 계산용)",
                            variable=full, value=True).pack(anchor="w", padx=8)
            collect = lambda: result.update(full=full.get())  # noqa: E731
        else:
            on, cut = tk.BooleanVar(value=False), tk.StringVar(value="15")
            row = ttk.Frame(frm)
            row.pack(anchor="w")
            ttk.Checkbutton(row, text="선 평활화, 차단 파장 [m]", variable=on).pack(side="left")
            ttk.Entry(row, textvariable=cut, width=6).pack(side="left", padx=4)
            ttk.Label(frm, foreground="#666", justify="left",
                      text="끄기: OptimumLap 구간(길이·반경) 그대로 - OptimumLap과 비교할 때.\n"
                           "켜기: x-y 선으로 다시 그려 콘맵의 곡률 잡음을 없앰 - 길이와 반경이 바뀌어 랩이 빨라집니다.\n"
                           "이미 OptimumLap에서 평활화한 트랙이면 끄세요.").pack(anchor="w", pady=4)

            def collect():
                if on.get():
                    result["smooth"] = float(cut.get())

        def ok():
            try:
                collect()
            except ValueError:
                messagebox.showerror("가져오기 옵션", "차단 파장은 숫자여야 합니다", parent=win)
                return
            result["_ok"] = True
            win.destroy()

        btns = ttk.Frame(frm)
        btns.pack(fill="x", pady=(8, 0))
        ttk.Button(btns, text="가져오기", command=ok).pack(side="right")
        ttk.Button(btns, text="취소", command=win.destroy).pack(side="right", padx=4)
        win.grab_set()
        self.root.wait_window(win)
        return {k: v for k, v in result.items() if k != "_ok"} if result.pop("_ok", False) else None

    # ------------------------------------------------------------------ tasks
    def task_kwargs(self) -> dict:
        us = (float(self.us_lo_var.get()), float(self.us_hi_var.get()))
        kw = {"distance_km": float(self.distance_var.get()), "wing_sweep": self.sweep_wing_var.get(),
              "objective": OBJECTIVES[self.obj_var.get()],
              "bounds": (float(self.lo_var.get()), float(self.hi_var.get())) if self.lo_var.get().strip() else None,
              "understeer": us, "max_evals": int(float(self.evals_var.get())),
              "sweep_path": self.param_labels.get(self.sw_param_var.get()),
              "sweep_objective": OBJECTIVES[self.sw_obj_var.get()], "sweep_n": int(float(self.sw_n_var.get() or 0)),
              "section_csv": self.section_csv_var.get().strip() or None,
              "section_component": self.section_comp_var.get(),
              "fixed": [f.strip() for f in self.fix_var.get().split(",") if f.strip()],
              "options": self.auto_options_var.get()}
        if self.auto_evals_var.get().strip():
            kw["auto_evals"] = int(float(self.auto_evals_var.get()))
        if self.track_width_var.get().strip():
            kw["track_width"] = float(self.track_width_var.get())
        if kw["sweep_path"]:
            kw["sweep_lo"], kw["sweep_hi"] = float(self.sw_lo_var.get()), float(self.sw_hi_var.get())
        return kw

    def run_task(self, key: str):
        if self.busy:
            return
        self.clear_error_marks()
        try:
            cfg = self.current_cfg()
            kw = self.task_kwargs()
            for msg in unknown_keys(cfg):
                self._log(f"경고: {msg}")
        except Exception as exc:  # noqa: BLE001
            self.report_error(exc, "입력 오류")
            return
        label, fn = TASKS[key]
        track, level, coarse = self.track_var.get(), self.level_var.get(), self.coarse_var.get()
        out_dir = OUT / key
        meta = {"task": label, "changes": changed_params(cfg, self.cfg),
                "setup": f"{Path(self.vehicle_var.get()).stem} / {Path(track).stem} / {level}"
                         + (" / 빠른 계산" if coarse else "")}
        self.cancel.clear()
        self.t_start = time.time()
        self._set_busy(True, f"{label} 계산 중 ...")

        def log(msg):
            if self.cancel.is_set():
                raise TaskCancelled
            self.queue.put(("log", msg))

        def work():
            try:
                res = fn(cfg, track, level, coarse, out_dir, log=log, **kw)
                self.queue.put(("done", (res, meta)))
            except TaskCancelled:
                self.queue.put(("cancelled", label))
            except Exception as exc:  # noqa: BLE001
                self.queue.put(("error", (exc, traceback.format_exc(), label)))

        threading.Thread(target=work, daemon=True).start()

    def cancel_task(self):
        if self.busy:
            self.cancel.set()
            self.status.configure(text="취소 요청됨 - 지금 계산 중인 단계가 끝나면 멈춥니다 ...")

    def _poll(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "log":
                    self._log(payload)
                elif kind == "done":
                    res, meta = payload
                    self._show(res)
                    self._add_history(res, meta)
                    self._set_busy(False, f"{res.title} 완료 ({time.time() - self.t_start:.0f} s) - {res.out_dir}")
                elif kind == "cancelled":
                    self._set_busy(False, f"{payload} 취소됨")
                    self._log(f"{payload}: 취소됨")
                elif kind == "error":
                    exc, tb, label = payload
                    self._set_busy(False, f"{label}: 오류")
                    self.report_error(exc, f"{label} 계산 오류", tb)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def _show(self, res):
        self.last_result = res
        self.text.delete("1.0", "end")
        self.text.insert("1.0", f"[{res.title}]\n\n{res.text}")
        self.img_canvas.delete("all")
        self.images.clear()
        self.image_spans.clear()
        self.result_tabs.select(1)
        self.img_canvas.update_idletasks()
        width = max(self.img_canvas.winfo_width() - 20, 600)
        y = 5
        for p in res.images:
            if not Path(p).exists():
                continue
            img = tk.PhotoImage(file=str(p))
            r = width / img.width()
            if r < 1.0:  # largest zoom/subsample fraction that still fits
                z, k = max(((z, k) for z in (1, 2, 3) for k in range(1, 10) if z / k <= r), key=lambda t: t[0] / t[1])
                img = (img.zoom(z, z) if z > 1 else img).subsample(k, k)
            self.images.append(img)
            self.img_canvas.create_image(5, y, image=img, anchor="nw")
            self.image_spans.append((y, y + img.height(), Path(p)))
            y += img.height() + 10
        self.img_canvas.configure(scrollregion=(0, 0, width, y))
        self.img_canvas.yview_moveto(0.0)
        self.apply_btn.configure(state="normal" if getattr(res.data, "joint", None) else "disabled")
        self.apply_section_btn.configure(state="normal" if getattr(res.data, "best_cfg", None) is not None
                                         and hasattr(res.data, "component") else "disabled")
        self.apply_auto_btn.configure(state="normal" if hasattr(res.data, "option_changes") else "disabled")
        self.use_line_btn.configure(state="normal" if getattr(res.data, "csv_path", None) else "disabled")
        if not res.images:
            self.result_tabs.select(0)

    def use_line(self):
        path = getattr(getattr(self.last_result, "data", None), "csv_path", None)
        if path:
            self.track_var.set(str(path))
            self._log(f"트랙을 최적 라인으로 바꿈: {path}")

    def _open_image(self, event):
        y = self.img_canvas.canvasy(event.y)
        for y0, y1, path in self.image_spans:
            if y0 <= y <= y1:
                _open_path(path)
                return

    # ------------------------------------------------------------------ history
    def _add_history(self, res, meta):
        row = {"time": time.strftime("%H:%M:%S"), "task": meta["task"], "result": res.summary,
               "changes": meta["changes"] or "-", "setup": meta["setup"]}
        self.history.append(row)
        self.tree.insert("", "end", values=[row[c] for c in ("time", "task", "result", "changes", "setup")])
        self.tree.yview_moveto(1.0)

    def save_history(self):
        if not self.history:
            return
        p = filedialog.asksaveasfilename(initialdir=OUT, defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if p:
            self.write_history(p)
            self._log(f"기록 저장: {p}")

    def write_history(self, path):
        with open(path, "w", newline="", encoding="utf-8-sig") as fh:  # BOM: Excel reads the Korean text
            w = csv.DictWriter(fh, fieldnames=["time", "task", "result", "changes", "setup"])
            w.writeheader()
            w.writerows(self.history)

    def clear_history(self):
        self.history.clear()
        self.tree.delete(*self.tree.get_children())

    # ------------------------------------------------------------------ helpers
    def _set_busy(self, busy: bool, msg: str):
        self.busy = busy
        for b in self.task_buttons:
            b.configure(state="disabled" if busy else "normal")
        self.cancel_btn.configure(state="normal" if busy else "disabled")
        if busy:
            self.progress.start(15)
        else:
            self.progress.stop()
        self.status.configure(text=msg)
        self.root.configure(cursor="watch" if busy else "")

    def _log(self, msg: str):
        self.log.insert("end", msg + "\n")
        self.log.see("end")

    def open_out(self):
        OUT.mkdir(parents=True, exist_ok=True)
        _open_path(OUT)


def main() -> int:
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
