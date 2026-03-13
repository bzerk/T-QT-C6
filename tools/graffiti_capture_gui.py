#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import asdict, dataclass, field
from pathlib import Path
from tkinter import ttk
from typing import Optional

import serial
import serial.tools.list_ports


TRACE_BEGIN_RE = re.compile(r"^\[trace\] begin id=(\d+) mode=([A-Za-z]+)$")
TRACE_POINT_RE = re.compile(r"^\[trace\] point id=(\d+) x=(-?\d+) y=(-?\d+) t=(\d+)$")
TRACE_END_RE = re.compile(
    r"^\[trace\] end id=(\d+) status=([^ ]+) pred=([^ ]+) accepted=(\d+) "
    r"score=([-0-9.]+) dist=([-0-9.]+) backend=([^ ]+) tok=(\d+) seq=([^ ]+) "
    r"points=(\d+) dur=(\d+) dx=(-?\d+) dy=(-?\d+)$"
)


@dataclass
class StrokeSample:
    stroke_id: int
    points: list[list[int]] = field(default_factory=list)
    trace_mode: str = "stop"
    status: str = "-"
    pred: str = "-"
    accepted: bool = False
    score: float = 0.0
    dist: float = 0.0
    backend: str = "NONE"
    tok: int = 0
    seq: str = "-"
    point_count: int = 0
    duration_ms: int = 0
    delta_x: int = 0
    delta_y: int = 0
    label: str = ""
    saved: bool = False
    captured_at_ms: int = 0


@dataclass
class ScriptedCaptureState:
    targets: list[str] = field(default_factory=list)
    reps_per_target: int = 5
    target_index: int = 0
    rep_index: int = 0
    session_id: str = ""
    active: bool = False
    total_saved: int = 0

    def current_target(self) -> str:
        if not self.active or self.target_index >= len(self.targets):
            return ""
        return self.targets[self.target_index]


class SerialReader(threading.Thread):
    def __init__(self, port: serial.Serial, out_queue: queue.Queue):
        super().__init__(daemon=True)
        self._port = port
        self._out_queue = out_queue
        self._alive = True

    def run(self) -> None:
        while self._alive:
            try:
                line = self._port.readline()
            except Exception as exc:  # pragma: no cover - hardware dependent
                self._out_queue.put(("error", str(exc)))
                return
            if not line:
                continue
            text = line.decode("utf-8", errors="replace").strip()
            self._out_queue.put(("line", text))

    def stop(self) -> None:
        self._alive = False


def autodetect_port() -> Optional[str]:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    usbmodem = [p for p in ports if "usbmodem" in p]
    if usbmodem:
        return sorted(usbmodem)[0]
    return sorted(ports)[0] if ports else None


def parse_script_targets(text: str) -> list[str]:
    stripped = text.strip()
    if not stripped:
        return []
    if re.search(r"[,\s]", stripped):
        parts = [item.strip() for item in re.split(r"[,\s]+", stripped) if item.strip()]
    else:
        parts = list(stripped)

    targets: list[str] = []
    for part in parts:
        if len(part) == 1 and part.isalpha():
            targets.append(part.lower())
        elif len(part) == 1:
            targets.append(part)
        else:
            targets.append(part.upper())
    return targets


def resample_stroke(points: list[list[int]], sample_count: int = 32) -> list[tuple[float, float]]:
    xy = [(float(x), float(y)) for x, y, _t in points]
    if not xy:
        return [(0.0, 0.0)] * sample_count

    deduped: list[tuple[float, float]] = [xy[0]]
    for point in xy[1:]:
        if point != deduped[-1]:
            deduped.append(point)

    if len(deduped) == 1:
        return [deduped[0]] * sample_count

    cumulative = [0.0]
    for idx in range(1, len(deduped)):
        prev_x, prev_y = deduped[idx - 1]
        cur_x, cur_y = deduped[idx]
        cumulative.append(cumulative[-1] + math.hypot(cur_x - prev_x, cur_y - prev_y))

    total_length = cumulative[-1]
    if total_length <= 1e-6:
        return [deduped[0]] * sample_count

    targets = [total_length * idx / max(1, sample_count - 1) for idx in range(sample_count)]
    resampled: list[tuple[float, float]] = []
    seg_idx = 1
    for target_dist in targets:
        while seg_idx < len(cumulative) - 1 and cumulative[seg_idx] < target_dist:
            seg_idx += 1
        prev_dist = cumulative[seg_idx - 1]
        next_dist = cumulative[seg_idx]
        prev_x, prev_y = deduped[seg_idx - 1]
        next_x, next_y = deduped[seg_idx]
        if next_dist <= prev_dist:
            resampled.append((prev_x, prev_y))
            continue
        mix = (target_dist - prev_dist) / (next_dist - prev_dist)
        resampled.append((prev_x + ((next_x - prev_x) * mix), prev_y + ((next_y - prev_y) * mix)))
    return resampled


def normalize_resampled_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not points:
        return [(0.0, 0.0)] * 32
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    scale = max(max_x - min_x, max_y - min_y, 1.0)
    return [(((x - center_x) * 2.0) / scale, ((y - center_y) * 2.0) / scale) for x, y in points]


def derive_feature_payload(sample: StrokeSample, sample_count: int = 32) -> dict:
    resampled = resample_stroke(sample.points, sample_count=sample_count)
    normalized = normalize_resampled_points(resampled)
    deltas: list[tuple[float, float]] = []
    prev_x = 0.0
    prev_y = 0.0
    for idx, (cur_x, cur_y) in enumerate(normalized):
        if idx == 0:
            deltas.append((0.0, 0.0))
        else:
            deltas.append((cur_x - prev_x, cur_y - prev_y))
        prev_x = cur_x
        prev_y = cur_y

    xs = [point[0] for point in sample.points]
    ys = [point[1] for point in sample.points]
    bbox_w = (max(xs) - min(xs)) if xs else 0
    bbox_h = (max(ys) - min(ys)) if ys else 0
    path_len = 0.0
    for idx in range(1, len(sample.points)):
        prev_x_i, prev_y_i, _prev_t = sample.points[idx - 1]
        cur_x_i, cur_y_i, _cur_t = sample.points[idx]
        path_len += math.hypot(float(cur_x_i - prev_x_i), float(cur_y_i - prev_y_i))

    return {
        "feature_format": "resampled_xy32_plus_dxy32_v1",
        "sample_count": sample_count,
        "resampled_xy": [[round(x, 6), round(y, 6)] for x, y in normalized],
        "resampled_dxy": [[round(dx, 6), round(dy, 6)] for dx, dy in deltas],
        "path_length_px": round(path_len, 3),
        "bbox_w_px": bbox_w,
        "bbox_h_px": bbox_h,
    }


class GraffitiCaptureGui:
    def __init__(self, root: tk.Tk, port_name: str, baud: int, output_path: Path):
        self.root = root
        self.port_name = port_name
        self.baud = baud
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.tinyml_jsonl_path = self.output_path.parent / "tinyml_dataset.jsonl"
        self.tinyml_csv_path = self.output_path.parent / "tinyml_dataset.csv"

        self.serial = serial.Serial(port_name, baudrate=baud, timeout=0.10)
        self.events: queue.Queue = queue.Queue()
        self.reader = SerialReader(self.serial, self.events)

        self.mode_var = tk.StringVar(value="stop")
        self.label_var = tk.StringVar(value="")
        self.autosave_var = tk.BooleanVar(value=False)
        self.port_var = tk.StringVar(value=port_name)
        self.status_var = tk.StringVar(value="Connecting...")
        self.output_var = tk.StringVar(value=str(output_path.resolve()))
        self.saved_var = tk.StringVar(value="Saved: 0")
        self.script_targets_var = tk.StringVar(value="abcdefghijklmnopqrstuvwxyz")
        self.script_reps_var = tk.IntVar(value=5)
        self.script_status_var = tk.StringVar(value="Script idle")
        self.script_target_var = tk.StringVar(value="-")
        self.script_progress_var = tk.StringVar(value="0 / 0")
        self.script_detail_var = tk.StringVar(value="Ready")

        self.saved_count = 0
        self.current_stroke: Optional[StrokeSample] = None
        self.last_stroke: Optional[StrokeSample] = None
        self.label_entry: Optional[ttk.Entry] = None
        self.script_state = ScriptedCaptureState()

        self._build_ui()
        self._refresh_script_status()
        self._bind_keys()
        self.reader.start()
        self.log(f"opened {self.port_name} @ {self.baud}")
        self.root.after(150, self._start_live_stream)
        self.root.after(30, self._poll_events)

    def _build_ui(self) -> None:
        self.root.title("Ringo Graffiti Capture")
        self.root.geometry("1180x760")
        self.root.minsize(960, 640)

        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(3, weight=1)
        main.rowconfigure(4, weight=1)

        top = ttk.Frame(main)
        top.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        top.columnconfigure(7, weight=1)

        ttk.Label(top, text="Port").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.port_var, state="readonly", width=22).grid(row=0, column=1, sticky="w", padx=(4, 12))
        ttk.Label(top, text="Label").grid(row=0, column=2, sticky="w")
        self.label_entry = ttk.Entry(top, textvariable=self.label_var, width=18)
        self.label_entry.grid(row=0, column=3, sticky="w", padx=(4, 12))
        ttk.Checkbutton(top, text="Autosave", variable=self.autosave_var).grid(row=0, column=4, sticky="w", padx=(0, 12))
        ttk.Label(top, textvariable=self.saved_var).grid(row=0, column=5, sticky="w", padx=(0, 12))
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=6, sticky="w")

        controls = ttk.Frame(main)
        controls.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        for idx in range(12):
            controls.columnconfigure(idx, weight=0)
        controls.columnconfigure(11, weight=1)

        ttk.Button(controls, text="Capture", command=lambda: self.set_mode("capture")).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(controls, text="Continuous", command=lambda: self.set_mode("continuous")).grid(row=0, column=1, padx=(0, 8))
        ttk.Button(controls, text="Stop", command=lambda: self.set_mode("stop")).grid(row=0, column=2, padx=(0, 20))
        ttk.Button(controls, text="Graffiti Mode", command=lambda: self.send_command("mode graffiti")).grid(row=0, column=3, padx=(0, 8))
        ttk.Button(controls, text="Mouse Mode", command=lambda: self.send_command("mode mouse")).grid(row=0, column=4, padx=(0, 20))
        ttk.Button(controls, text="Save Last", command=self.save_last).grid(row=0, column=5, padx=(0, 8))
        ttk.Button(controls, text="Capture Next", command=self.arm_capture_next).grid(row=0, column=6, padx=(0, 8))
        ttk.Button(controls, text="Cancel Capture", command=lambda: self.send_command("cap off")).grid(row=0, column=7, padx=(0, 8))
        ttk.Label(controls, text="Targets").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(controls, textvariable=self.script_targets_var, width=32).grid(row=1, column=1, columnspan=3, sticky="ew", pady=(10, 0), padx=(0, 8))
        ttk.Label(controls, text="Reps").grid(row=1, column=4, sticky="w", pady=(10, 0))
        ttk.Spinbox(controls, from_=1, to=50, textvariable=self.script_reps_var, width=5).grid(row=1, column=5, sticky="w", pady=(10, 0), padx=(0, 8))
        ttk.Button(controls, text="Start Script", command=self.start_scripted_capture).grid(row=1, column=6, pady=(10, 0), padx=(0, 8))
        ttk.Button(controls, text="Stop Script", command=self.stop_scripted_capture).grid(row=1, column=7, pady=(10, 0), padx=(0, 8))
        ttk.Label(controls, text="Shortcuts: c v x l s a g m q").grid(row=0, column=11, sticky="e")

        script_frame = ttk.LabelFrame(main, text="Scripted Capture")
        script_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        script_frame.columnconfigure(1, weight=1)
        script_frame.columnconfigure(3, weight=1)
        ttk.Label(script_frame, text="Target").grid(row=0, column=0, sticky="w", padx=(10, 12), pady=(8, 2))
        tk.Label(script_frame, textvariable=self.script_target_var, font=("Helvetica", 36, "bold"), fg="#0f766e").grid(row=0, column=1, sticky="w", pady=(4, 2))
        ttk.Label(script_frame, text="Progress").grid(row=0, column=2, sticky="w", padx=(20, 12), pady=(8, 2))
        ttk.Label(script_frame, textvariable=self.script_progress_var, font=("Helvetica", 18, "bold")).grid(row=0, column=3, sticky="w", pady=(4, 2))
        ttk.Label(script_frame, textvariable=self.script_status_var).grid(row=1, column=0, columnspan=2, sticky="w", padx=(10, 10), pady=(0, 8))
        ttk.Label(script_frame, textvariable=self.script_detail_var).grid(row=1, column=2, columnspan=2, sticky="w", padx=(20, 10), pady=(0, 8))

        live_frame = ttk.LabelFrame(main, text="Live Stroke")
        live_frame.grid(row=3, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        live_frame.rowconfigure(0, weight=1)
        live_frame.columnconfigure(0, weight=1)
        self.live_canvas = tk.Canvas(live_frame, bg="#0b0f16", highlightthickness=0)
        self.live_canvas.grid(row=0, column=0, sticky="nsew")

        last_frame = ttk.LabelFrame(main, text="Last Stroke")
        last_frame.grid(row=3, column=1, sticky="nsew", pady=(0, 8))
        last_frame.rowconfigure(0, weight=1)
        last_frame.columnconfigure(0, weight=1)
        self.last_canvas = tk.Canvas(last_frame, bg="#11161c", highlightthickness=0)
        self.last_canvas.grid(row=0, column=0, sticky="nsew")

        meta_frame = ttk.LabelFrame(main, text="Stroke Metadata")
        meta_frame.grid(row=4, column=0, sticky="nsew", padx=(0, 8))
        meta_frame.rowconfigure(0, weight=1)
        meta_frame.columnconfigure(0, weight=1)
        self.meta_text = tk.Text(meta_frame, height=10, wrap="word", state="disabled")
        self.meta_text.grid(row=0, column=0, sticky="nsew")

        log_frame = ttk.LabelFrame(main, text="Logs")
        log_frame.grid(row=4, column=1, sticky="nsew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=10, wrap="word", state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")

        ttk.Label(
            main,
            text=(
                f"Raw: {self.output_path.resolve()}   "
                f"TinyML JSONL: {self.tinyml_jsonl_path.resolve()}   "
                f"TinyML CSV: {self.tinyml_csv_path.resolve()}"
            ),
        ).grid(row=5, column=0, columnspan=2, sticky="w", pady=(10, 0))

        self.live_canvas.bind("<Configure>", lambda _e: self._render_live())
        self.last_canvas.bind("<Configure>", lambda _e: self._render_last())

        self.label_entry.focus_set()

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress-c>", lambda _e: self.set_mode("capture"))
        self.root.bind("<KeyPress-v>", lambda _e: self.set_mode("continuous"))
        self.root.bind("<KeyPress-x>", lambda _e: self.set_mode("stop"))
        self.root.bind("<KeyPress-g>", lambda _e: self.send_command("mode graffiti"))
        self.root.bind("<KeyPress-m>", lambda _e: self.send_command("mode mouse"))
        self.root.bind("<KeyPress-a>", lambda _e: self.toggle_autosave())
        self.root.bind("<KeyPress-s>", lambda _e: self.save_last())
        self.root.bind("<KeyPress-q>", lambda _e: self.close())
        self.root.bind("<KeyPress-l>", lambda _e: self.focus_label())
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def focus_label(self) -> None:
        if self.label_entry is not None:
            self.label_entry.focus_set()
            self.label_entry.icursor("end")

    def log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("1.0", f"{stamp} {message}\n")
        self.log_text.configure(state="disabled")
        self.log_text.see("1.0")
        self.status_var.set(message)

    def send_command(self, command: str) -> None:
        self.serial.write((command + "\n").encode("utf-8"))
        self.serial.flush()
        self.log(f"> {command}")

    def _start_live_stream(self) -> None:
        self.send_command("mode graffiti")
        self.set_mode("continuous")
        self.send_command("status")

    def set_mode(self, mode: str) -> None:
        if mode == "capture":
            self.send_command("trace once")
            self.mode_var.set("capture")
        elif mode == "continuous":
            self.send_command("trace cont")
            self.mode_var.set("continuous")
        else:
            self.send_command("trace off")
            self.mode_var.set("stop")
        self.status_var.set(f"mode={self.mode_var.get()}")

    def toggle_autosave(self) -> None:
        self.autosave_var.set(not self.autosave_var.get())
        self.log(f"autosave {'on' if self.autosave_var.get() else 'off'}")

    def arm_capture_next(self) -> None:
        label = self.label_var.get().strip()
        if not label:
            self.log("set a label first")
            return
        self.send_command(f"cap {label}")

    def _script_meta_for_current_target(self) -> dict:
        if not self.script_state.active:
            return {
                "scripted": False,
                "session_id": "",
                "target_index": -1,
                "rep_index": -1,
                "reps_per_target": 0,
                "prompt_label": "",
            }
        return {
            "scripted": True,
            "session_id": self.script_state.session_id,
            "target_index": self.script_state.target_index,
            "rep_index": self.script_state.rep_index,
            "reps_per_target": self.script_state.reps_per_target,
            "prompt_label": self.script_state.current_target(),
        }

    def _write_tinyml_csv_row(self, record: dict) -> None:
        sample_count = record["features"]["sample_count"]
        headers = [
            "label",
            "prompt_label",
            "session_id",
            "scripted",
            "target_index",
            "rep_index",
            "reps_per_target",
            "stroke_id",
            "status",
            "pred",
            "accepted",
            "score",
            "dist",
            "backend",
            "tok",
            "seq",
            "point_count",
            "duration_ms",
            "delta_x",
            "delta_y",
            "path_length_px",
            "bbox_w_px",
            "bbox_h_px",
        ]
        for idx in range(sample_count):
            headers.extend([f"x{idx:02d}", f"y{idx:02d}", f"dx{idx:02d}", f"dy{idx:02d}"])

        row = {
            "label": record["label"],
            "prompt_label": record["script"]["prompt_label"],
            "session_id": record["script"]["session_id"],
            "scripted": int(bool(record["script"]["scripted"])),
            "target_index": record["script"]["target_index"],
            "rep_index": record["script"]["rep_index"],
            "reps_per_target": record["script"]["reps_per_target"],
            "stroke_id": record["stroke_id"],
            "status": record["status"],
            "pred": record["pred"],
            "accepted": int(bool(record["accepted"])),
            "score": record["score"],
            "dist": record["dist"],
            "backend": record["backend"],
            "tok": record["tok"],
            "seq": record["seq"],
            "point_count": record["point_count"],
            "duration_ms": record["duration_ms"],
            "delta_x": record["delta_x"],
            "delta_y": record["delta_y"],
            "path_length_px": record["features"]["path_length_px"],
            "bbox_w_px": record["features"]["bbox_w_px"],
            "bbox_h_px": record["features"]["bbox_h_px"],
        }
        for idx, (xy, dxy) in enumerate(zip(record["features"]["resampled_xy"], record["features"]["resampled_dxy"])):
            row[f"x{idx:02d}"] = xy[0]
            row[f"y{idx:02d}"] = xy[1]
            row[f"dx{idx:02d}"] = dxy[0]
            row[f"dy{idx:02d}"] = dxy[1]

        write_header = not self.tinyml_csv_path.exists()
        with self.tinyml_csv_path.open("a", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=headers)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

    def save_sample(self, sample: StrokeSample, label: str, script_meta: Optional[dict] = None) -> bool:
        point_count = sample.point_count or len(sample.points)
        if point_count < 2:
            self.log(f"ignored stroke {sample.stroke_id}: not enough points")
            return False
        if sample.saved and sample.label == label:
            self.log(f"stroke {sample.stroke_id} already saved as {label}")
            return False

        features = derive_feature_payload(sample)
        if script_meta is None:
            script_meta = self._script_meta_for_current_target()

        record = asdict(sample)
        record["label"] = label
        record["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        record["script"] = script_meta
        record["features"] = features
        with self.output_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, separators=(",", ":")) + "\n")
        with self.tinyml_jsonl_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._write_tinyml_csv_row(record)
        sample.label = label
        sample.saved = True
        self.saved_count += 1
        self.saved_var.set(f"Saved: {self.saved_count}")
        self.log(f"saved stroke {sample.stroke_id} as {label}")
        self._render_meta()
        return True

    def _refresh_script_status(self) -> None:
        if not self.script_state.active:
            self.script_status_var.set("Script idle")
            self.script_target_var.set("-")
            self.script_progress_var.set("0 / 0")
            self.script_detail_var.set("Ready")
            return
        current = self.script_state.current_target()
        self.script_target_var.set(current)
        current_rep = self.script_state.rep_index + 1
        self.script_progress_var.set(f"{current_rep} / {self.script_state.reps_per_target}")
        self.script_status_var.set(
            f"Target {self.script_state.target_index + 1} of {len(self.script_state.targets)}"
        )
        self.script_detail_var.set(
            f"Session {self.script_state.session_id}   saved {self.script_state.total_saved}"
        )

    def start_scripted_capture(self) -> None:
        targets = parse_script_targets(self.script_targets_var.get())
        if not targets:
            self.log("no scripted targets configured")
            return
        reps = max(1, int(self.script_reps_var.get()))
        self.script_state = ScriptedCaptureState(
            targets=targets,
            reps_per_target=reps,
            session_id=time.strftime("%Y%m%dT%H%M%S"),
            active=True,
        )
        self.autosave_var.set(True)
        self.label_var.set(self.script_state.current_target())
        self._refresh_script_status()
        self.send_command("mode graffiti")
        self.set_mode("continuous")
        self.log(f"script started: {len(targets)} targets x {reps} reps")

    def stop_scripted_capture(self) -> None:
        if self.script_state.active:
            self.log(f"script stopped after {self.script_state.total_saved} saved samples")
        self.script_state = ScriptedCaptureState()
        self._refresh_script_status()

    def _advance_scripted_capture(self) -> None:
        if not self.script_state.active:
            return
        self.script_state.total_saved += 1
        self.script_state.rep_index += 1
        if self.script_state.rep_index >= self.script_state.reps_per_target:
            self.script_state.rep_index = 0
            self.script_state.target_index += 1
        if self.script_state.target_index >= len(self.script_state.targets):
            total = self.script_state.total_saved
            self.stop_scripted_capture()
            self.log(f"script complete: {total} samples")
            return
        self.label_var.set(self.script_state.current_target())
        self._refresh_script_status()
        self.log(
            f"next target {self.script_state.current_target()} "
            f"rep {self.script_state.rep_index + 1}/{self.script_state.reps_per_target}"
        )

    def save_last(self) -> None:
        if self.last_stroke is None:
            self.log("no stroke to save")
            return
        label = self.label_var.get().strip()
        if not label:
            self.log("set a label first")
            return
        self.save_sample(self.last_stroke, label)

    def process_line(self, line: str) -> None:
        if not line:
            return
        match = TRACE_BEGIN_RE.match(line)
        if match:
            stroke_id = int(match.group(1))
            trace_mode = match.group(2).lower()
            self.current_stroke = StrokeSample(
                stroke_id=stroke_id,
                trace_mode=trace_mode,
                captured_at_ms=int(time.time() * 1000),
            )
            self.log(f"trace begin #{stroke_id} mode={trace_mode}")
            self._render_live()
            return

        match = TRACE_POINT_RE.match(line)
        if match:
            stroke_id = int(match.group(1))
            if self.current_stroke is None or self.current_stroke.stroke_id != stroke_id:
                self.current_stroke = StrokeSample(stroke_id=stroke_id, captured_at_ms=int(time.time() * 1000))
            self.current_stroke.points.append(
                [int(match.group(2)), int(match.group(3)), int(match.group(4))]
            )
            self._render_live()
            return

        match = TRACE_END_RE.match(line)
        if match:
            stroke_id = int(match.group(1))
            sample = self.current_stroke if self.current_stroke and self.current_stroke.stroke_id == stroke_id else StrokeSample(stroke_id=stroke_id)
            sample.status = match.group(2)
            sample.pred = match.group(3)
            sample.accepted = bool(int(match.group(4)))
            sample.score = float(match.group(5))
            sample.dist = float(match.group(6))
            sample.backend = match.group(7)
            sample.tok = int(match.group(8))
            sample.seq = match.group(9)
            sample.point_count = int(match.group(10))
            sample.duration_ms = int(match.group(11))
            sample.delta_x = int(match.group(12))
            sample.delta_y = int(match.group(13))
            self.last_stroke = sample
            self.current_stroke = None
            if sample.trace_mode == "capture" or self.mode_var.get() == "capture":
                self.mode_var.set("stop")
            self.log(f"trace end #{stroke_id} status={sample.status} pred={sample.pred} score={sample.score:.2f}")
            saved_in_script = False
            if self.script_state.active:
                prompt_label = self.script_state.current_target()
                if prompt_label:
                    saved_in_script = self.save_sample(sample, prompt_label, script_meta=self._script_meta_for_current_target())
                    if saved_in_script:
                        self._advance_scripted_capture()
            if (not saved_in_script) and self.autosave_var.get():
                label = self.label_var.get().strip()
                if label:
                    self.save_sample(sample, label)
            self._render_live()
            self._render_last()
            self._render_meta()
            return

        self.log(line)

    def _poll_events(self) -> None:
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                self.process_line(payload)
            else:
                self.log(f"serial error: {payload}")
        self.root.after(30, self._poll_events)

    def _draw_sample(self, canvas: tk.Canvas, sample: Optional[StrokeSample], bg: str) -> None:
        canvas.configure(bg=bg)
        canvas.delete("all")
        w = max(1, canvas.winfo_width())
        h = max(1, canvas.winfo_height())
        if sample is None or not sample.points:
            canvas.create_text(w // 2, h // 2, text="No stroke", fill="#8892a0", font=("Helvetica", 20))
            return

        pts = sample.points
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(1, max_x - min_x)
        span_y = max(1, max_y - min_y)
        pad = 24
        plot_w = max(1, w - (2 * pad))
        plot_h = max(1, h - (2 * pad))

        coords: list[float] = []
        for x, y, _t in pts:
            px = pad + ((x - min_x) * plot_w / span_x)
            py = pad + ((y - min_y) * plot_h / span_y)
            coords.extend([px, py])

        if len(coords) >= 4:
            canvas.create_line(*coords, fill="#6ee7ff", width=3, smooth=False)
        start_x, start_y = coords[0], coords[1]
        end_x, end_y = coords[-2], coords[-1]
        canvas.create_oval(start_x - 5, start_y - 5, start_x + 5, start_y + 5, fill="#fbbf24", outline="")
        canvas.create_oval(end_x - 6, end_y - 6, end_x + 6, end_y + 6, fill="#34d399", outline="")
        canvas.create_text(12, 12, anchor="nw", fill="#cbd5e1",
                           text=f"id:{sample.stroke_id}  pts:{sample.point_count or len(sample.points)}  seq:{sample.seq}")

    def _render_live(self) -> None:
        self._draw_sample(self.live_canvas, self.current_stroke, "#0b0f16")

    def _render_last(self) -> None:
        self._draw_sample(self.last_canvas, self.last_stroke, "#11161c")

    def _render_meta(self) -> None:
        self.meta_text.configure(state="normal")
        self.meta_text.delete("1.0", "end")
        sample = self.last_stroke
        if sample is None:
            self.meta_text.insert("1.0", "No stroke yet.")
        else:
            lines = [
                f"id: {sample.stroke_id}",
                f"status: {sample.status}",
                f"prediction: {sample.pred}",
                f"accepted: {sample.accepted}",
                f"score: {sample.score:.3f}",
                f"distance: {sample.dist:.3f}",
                f"backend: {sample.backend}",
                f"tokens: {sample.tok}",
                f"sequence: {sample.seq}",
                f"points: {sample.point_count}",
                f"duration_ms: {sample.duration_ms}",
                f"delta_x: {sample.delta_x}",
                f"delta_y: {sample.delta_y}",
                f"saved: {sample.saved}",
                f"label: {sample.label or '-'}",
                f"script_target: {self.script_target_var.get()}",
                f"script_progress: {self.script_progress_var.get()}",
            ]
            self.meta_text.insert("1.0", "\n".join(lines))
        self.meta_text.configure(state="disabled")

    def close(self) -> None:
        try:
            self.send_command("trace off")
        except Exception:
            pass
        self.reader.stop()
        try:
            self.serial.close()
        except Exception:
            pass
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GUI graffiti stroke capture")
    parser.add_argument("--port", default=autodetect_port(), help="Serial port, defaults to first /dev/cu.usbmodem*")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("debug/graffiti_capture/samples.jsonl"),
        help="JSONL output path",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.port:
        print("No serial port found.")
        return 1
    root = tk.Tk()
    app = GraffitiCaptureGui(root, args.port, args.baud, args.output)
    app._render_live()
    app._render_last()
    app._render_meta()
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
