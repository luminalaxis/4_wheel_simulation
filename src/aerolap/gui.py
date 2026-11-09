"""Desktop window for aerolap (tkinter, no extra install): ``py -m aerolap.gui``.

Pick a vehicle and a track, edit the usual parameters, press a button; text results and graphs
appear on the right and are also saved under ``out/gui/<task>/``. Calculations run in a worker
thread so the window stays responsive.
"""

from __future__ import annotations

import copy
import os
import queue
import subprocess
import sys
import threading
import traceback
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .config import dump_config, get_path, load_vehicle_config, set_path, validate
from .features import LEVELS
from .gui_tasks import ROOT, TASKS, editable_params, import_file

DEFAULT_VEHICLE = ROOT / "data" / "vehicles" / "fsk2026.yaml"
DEFAULT_TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"
OUT = Path.cwd() / "out" / "gui"


def _open_folder(path: Path) -> None:
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
        self.cfg: dict | None = None
        self.param_vars: dict[str, tk.StringVar] = {}
        self.last_result = None
        self.images: list[tk.PhotoImage] = []

        self.vehicle_var = tk.StringVar(value=str(DEFAULT_VEHICLE))
        self.track_var = tk.StringVar(value=str(DEFAULT_TRACK))
        self.level_var = tk.StringVar(value="L5")
        self.coarse_var = tk.BooleanVar(value=False)
        self.distance_var = tk.StringVar(value="22")
        self.obj_var = tk.StringVar(value="endurance_time")
        self.lo_var, self.hi_var = tk.StringVar(value="0.5"), tk.StringVar(value="1.6")
        self.us_lo_var, self.us_hi_var = tk.StringVar(value="1.05"), tk.StringVar(value="1.3")
        self.evals_var = tk.StringVar(value="100")
        self.sweep_var = tk.BooleanVar(value=True)

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
        ttk.Label(opts, text=" | ".join(f"{k}: {v[0]}" for k, v in LEVELS.items())[:160],
                  foreground="#666").pack(side="left", padx=6)
        ttk.Checkbutton(opts, text="빠른 계산(거친 엔벨로프)", variable=self.coarse_var).pack(side="left", padx=8)
        top.columnconfigure(1, weight=1)

        pan = ttk.PanedWindow(self.root, orient="horizontal")
        pan.pack(fill="both", expand=True)
        left = ttk.Notebook(pan, width=430)
        right = ttk.Notebook(pan)
        pan.add(left, weight=0)
        pan.add(right, weight=1)

        # --- tasks tab
        tasks = ttk.Frame(left, padding=8)
        left.add(tasks, text="계산")
        self.task_buttons = []
        for key, (label, _fn) in TASKS.items():
            b = ttk.Button(tasks, text=label, command=lambda k=key: self.run_task(k))
            b.pack(fill="x", pady=3)
            self.task_buttons.append(b)
        ttk.Separator(tasks).pack(fill="x", pady=8)
        g = ttk.LabelFrame(tasks, text="내구 / 에너지", padding=6)
        g.pack(fill="x")
        self._row(g, 0, "내구 거리 [km]", self.distance_var)
        ttk.Checkbutton(g, text="공력 패키지 크기 스윕 포함", variable=self.sweep_var).grid(row=1, column=0,
                                                                                     columnspan=2, sticky="w")
        a = ttk.LabelFrame(tasks, text="공력 장치 최적화", padding=6)
        a.pack(fill="x", pady=6)
        ttk.Label(a, text="목적").grid(row=0, column=0, sticky="w")
        ttk.Combobox(a, textvariable=self.obj_var, values=["endurance_time", "lap_time"], width=16,
                     state="readonly").grid(row=0, column=1, sticky="w")
        self._row(a, 1, "scale 하한", self.lo_var)
        self._row(a, 2, "scale 상한", self.hi_var)
        self._row(a, 3, "언더스티어 하한", self.us_lo_var)
        self._row(a, 4, "언더스티어 상한", self.us_hi_var)
        self._row(a, 5, "최적화 평가 횟수", self.evals_var)
        self.apply_btn = ttk.Button(a, text="최적값을 파라미터에 적용", command=self.apply_optimum, state="disabled")
        self.apply_btn.grid(row=6, column=0, columnspan=2, sticky="we", pady=4)
        ttk.Label(tasks, text="결과는 out/gui/<작업>/ 에도 저장됩니다.", foreground="#666").pack(anchor="w", pady=6)
        ttk.Button(tasks, text="결과 폴더 열기", command=self.open_out).pack(fill="x")

        # --- parameters tab (scrollable)
        pframe = ttk.Frame(left)
        left.add(pframe, text="파라미터")
        bar = ttk.Frame(pframe, padding=4)
        bar.pack(fill="x")
        ttk.Button(bar, text="YAML로 저장...", command=self.save_vehicle).pack(side="left")
        ttk.Button(bar, text="되돌리기", command=self.load_vehicle).pack(side="left", padx=4)
        canvas = tk.Canvas(pframe, highlightthickness=0)
        sb = ttk.Scrollbar(pframe, orient="vertical", command=canvas.yview)
        self.param_frame = ttk.Frame(canvas)
        self.param_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.param_frame, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        # --- results
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
        self.result_tabs = right

        self.log = tk.Text(self.root, height=7, font=("Consolas", 9), background="#f4f4f4")
        self.log.pack(fill="x")
        self.status = ttk.Label(self.root, text="준비", anchor="w")
        self.status.pack(fill="x")

    @staticmethod
    def _row(parent, r, label, var):
        ttk.Label(parent, text=label).grid(row=r, column=0, sticky="w")
        ttk.Entry(parent, textvariable=var, width=10).grid(row=r, column=1, sticky="w", pady=1)

    # ------------------------------------------------------------------ vehicle / params
    def pick_vehicle(self):
        p = filedialog.askopenfilename(initialdir=DEFAULT_VEHICLE.parent, filetypes=[("vehicle YAML", "*.yaml")])
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
            self.cfg = load_vehicle_config(self.vehicle_var.get())
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("차량 파일", str(exc))
            return
        for w in self.param_frame.winfo_children():
            w.destroy()
        self.param_vars.clear()
        for r, (label, path) in enumerate(editable_params(self.cfg)):
            ttk.Label(self.param_frame, text=label).grid(row=r, column=0, sticky="w", padx=4)
            var = tk.StringVar(value=f"{get_path(self.cfg, path, '')}")
            ttk.Entry(self.param_frame, textvariable=var, width=12).grid(row=r, column=1, pady=1)
            ttk.Label(self.param_frame, text=path, foreground="#888").grid(row=r, column=2, sticky="w", padx=4)
            self.param_vars[path] = var
        self._log(f"차량 불러옴: {self.cfg.get('name')}")

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
                raise ValueError(f"{path}: 숫자가 아닙니다 ({txt!r})") from None
        validate(cfg)
        return cfg

    def save_vehicle(self):
        try:
            cfg = self.current_cfg()
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("파라미터", str(exc))
            return
        p = filedialog.asksaveasfilename(initialdir=DEFAULT_VEHICLE.parent, defaultextension=".yaml",
                                         filetypes=[("vehicle YAML", "*.yaml")])
        if p:
            dump_config(cfg, p)
            self._log(f"저장: {p}  (주의: 원래 파일의 주석은 저장되지 않습니다)")

    def apply_optimum(self):
        res = getattr(self.last_result, "data", None)
        if res is None or res.joint is None:
            return
        for path, val in res.joint.best.items():
            if path in self.param_vars:
                self.param_vars[path].set(f"{val:.3f}")
        self._log("공력 장치 최적값을 파라미터 탭에 넣었습니다. 'YAML로 저장'으로 보관하세요.")

    def import_dialog(self):
        p = filedialog.askopenfilename(filetypes=[("OptimumLap / Assetto Corsa", "*.oltra *.olveh *.ai"),
                                                  ("all", "*.*")])
        if not p:
            return
        suf = Path(p).suffix.lower()
        dest = ROOT / "data" / ("vehicles" if suf == ".olveh" else "tracks")
        try:
            out = import_file(p, dest)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("가져오기", str(exc))
            return
        self._log(f"가져옴: {out}")
        if suf == ".olveh":
            self.vehicle_var.set(out)
            self.load_vehicle()
            messagebox.showinfo("가져오기", "차량을 불러왔습니다. OptimumLap 파일에 없는 값(무게중심, 서스펜션, "
                                "공력 분할, 배터리)은 예제 값입니다 - 파라미터 탭에서 고치세요.")
        else:
            self.track_var.set(out)

    # ------------------------------------------------------------------ tasks
    def run_task(self, key: str):
        if self.busy:
            return
        try:
            cfg = self.current_cfg()
            kw = {"distance_km": float(self.distance_var.get()), "wing_sweep": self.sweep_var.get(),
                  "objective": self.obj_var.get(), "bounds": (float(self.lo_var.get()), float(self.hi_var.get())),
                  "understeer": (float(self.us_lo_var.get()), float(self.us_hi_var.get())),
                  "max_evals": int(float(self.evals_var.get()))}
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("입력", str(exc))
            return
        label, fn = TASKS[key]
        track, level, coarse = self.track_var.get(), self.level_var.get(), self.coarse_var.get()
        out_dir = OUT / key
        self._set_busy(True, f"{label} 계산 중 ...")

        def work():
            try:
                res = fn(cfg, track, level, coarse, out_dir, log=lambda m: self.queue.put(("log", m)), **kw)
                self.queue.put(("done", res))
            except Exception as exc:  # noqa: BLE001
                self.queue.put(("error", f"{exc}\n\n{traceback.format_exc()}"))

        threading.Thread(target=work, daemon=True).start()

    def _poll(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "log":
                    self._log(payload)
                elif kind == "done":
                    self._show(payload)
                    self._set_busy(False, f"{payload.title} 완료 - {payload.out_dir}")
                elif kind == "error":
                    self._set_busy(False, "오류")
                    self._log(payload)
                    messagebox.showerror("계산 오류", payload.split("\n")[0])
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def _show(self, res):
        self.last_result = res
        self.text.delete("1.0", "end")
        self.text.insert("1.0", f"[{res.title}]\n\n{res.text}")
        self.img_canvas.delete("all")
        self.images.clear()
        y = 5
        self.result_tabs.select(1)
        self.img_canvas.update_idletasks()
        width = max(self.img_canvas.winfo_width() - 20, 600)
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
            y += img.height() + 10
        self.img_canvas.configure(scrollregion=(0, 0, width, y))
        self.apply_btn.configure(state="normal" if getattr(res.data, "joint", None) else "disabled")
        if not res.images:
            self.result_tabs.select(0)

    def _set_busy(self, busy: bool, msg: str):
        self.busy = busy
        for b in self.task_buttons:
            b.configure(state="disabled" if busy else "normal")
        self.status.configure(text=msg)
        self.root.configure(cursor="watch" if busy else "")

    def _log(self, msg: str):
        self.log.insert("end", msg + "\n")
        self.log.see("end")

    def open_out(self):
        OUT.mkdir(parents=True, exist_ok=True)
        _open_folder(OUT)


def main() -> int:
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
