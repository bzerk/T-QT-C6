#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
import queue
import re
import shutil
import sys
import threading
import time
import traceback
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
CMD_MODE_RE = re.compile(r"^\[cmd\] mode=([A-Z]+)")
CMD_TRACE_RE = re.compile(r"^\[cmd\] trace=([^ ]+)")
MODE_SWITCH_RE = re.compile(r"^\[mode\] switched to ([A-Z]+)")
BOOT_BANNER_RE = re.compile(r"^\[boot\] Ringo BLE trackpad starting$")

LETTER_LABELS = list("abcdefghijklmnopqrstuvwxyz")
ALPHA_CONTROL_LABELS = ["SPACE", "BKSP", "RET", "SHIFT"]
PUNCT_LABELS = [
    "ESC",
    ".",
    ",",
    "(",
    ")",
    "{",
    "}",
    "[",
    "]",
    "<",
    ">",
    "-",
    "_",
    "#",
    "*",
    "?",
    "'",
]
NUMERIC_LABELS = list("0123456789")
ALPHA_CONTROL_SET = set(ALPHA_CONTROL_LABELS)
PUNCT_SET = set(PUNCT_LABELS)
NUMERIC_SET = set(NUMERIC_LABELS)
LETTER_SET = set(LETTER_LABELS)
TARGET_PRESETS = ("letters", "punct", "numeric", "all", "custom")
TOKEN_ALIASES = {
    "space": "SPACE",
    "bksp": "BKSP",
    "backspace": "BKSP",
    "ret": "RET",
    "return": "RET",
    "enter": "RET",
    "shift": "SHIFT",
    "esc": "ESC",
    "escape": "ESC",
    "apostrophe": "'",
    "quote": "'",
    "lparen": "(",
    "rbrace": "}",
    "lbrace": "{",
    "lbrack": "[",
    "rbrack": "]",
    "langle": "<",
    "rangle": ">",
    "rparen": ")",
}
CAPTURE_ROOT = Path("debug/graffiti_capture")


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
    condition: str = ""
    capture_uid: str = ""
    saved: bool = False
    captured_at_ms: int = 0


@dataclass(frozen=True)
class PromptSpec:
    label: str
    condition: str

    @property
    def display(self) -> str:
        return self.label


@dataclass
class ScriptedCaptureState:
    targets: list[PromptSpec] = field(default_factory=list)
    reps_per_target: int = 5
    target_index: int = 0
    rep_index: int = 0
    session_id: str = ""
    active: bool = False
    total_saved: int = 0

    def current_target(self) -> Optional[PromptSpec]:
        if not self.active or self.target_index >= len(self.targets):
            return None
        return self.targets[self.target_index]


@dataclass
class SavedEntry:
    capture_uid: str
    saved_count_before: int
    script_state_before: ScriptedCaptureState


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


class SingleInstanceLock:
    def __init__(self, path: Path):
        self.path = path
        self.handle = None

    def acquire(self) -> bool:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            return False
        self.handle.seek(0)
        self.handle.truncate()
        self.handle.write(f"{os.getpid()}\n")
        self.handle.flush()
        return True

    def release(self) -> None:
        if self.handle is None:
            return
        try:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            self.handle.close()
        except OSError:
            pass
        self.handle = None


def preferred_serial_ports() -> list[str]:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    preferred = [
        port
        for port in ports
        if any(token in port for token in ("usbmodem", "usbserial", "ttyACM", "wchusbserial"))
    ]
    return sorted(dict.fromkeys(preferred))


def autodetect_port() -> Optional[str]:
    preferred = preferred_serial_ports()
    return preferred[0] if preferred else None


def normalize_target_token(token: str) -> str:
    token = token.strip()
    if not token:
        return ""
    alias = TOKEN_ALIASES.get(token.lower())
    if alias:
        return alias
    if len(token) == 1 and token.isalpha():
        return token.lower()
    if len(token) == 1:
        return token
    return token.upper()


def infer_condition(label: str) -> Optional[str]:
    if label in LETTER_SET or label in ALPHA_CONTROL_SET:
        return "letters"
    if label in PUNCT_SET:
        return "punct"
    if label in NUMERIC_SET:
        return "numeric"
    return None


def parse_script_targets(text: str) -> list[PromptSpec]:
    stripped = text.strip()
    if not stripped:
        return []
    parts = [item.strip() for item in re.split(r"[,\s]+", stripped) if item.strip()]
    if not parts:
        parts = [stripped]

    prompts: list[PromptSpec] = []
    for part in parts:
        normalized = normalize_target_token(part)
        if len(part) > 1 and part.isalpha() and normalized not in PUNCT_SET:
            prompts.extend(PromptSpec(ch, "letters") for ch in part.lower())
            continue
        if len(part) > 1 and part.isdigit():
            prompts.extend(PromptSpec(ch, "numeric") for ch in part)
            continue
        condition = infer_condition(normalized)
        if condition is not None:
            prompts.append(PromptSpec(normalized, condition))
    return prompts


def prompt_specs_for_preset(preset: str) -> list[PromptSpec]:
    if preset == "letters":
        return [PromptSpec(label, "letters") for label in LETTER_LABELS]
    if preset == "punct":
        return [PromptSpec(label, "punct") for label in PUNCT_LABELS]
    if preset == "numeric":
        return [PromptSpec(label, "numeric") for label in NUMERIC_LABELS]
    if preset == "all":
        return (
            [PromptSpec(label, "letters") for label in LETTER_LABELS]
            + [PromptSpec(label, "letters") for label in ALPHA_CONTROL_LABELS]
            + [PromptSpec(label, "punct") for label in PUNCT_LABELS]
            + [PromptSpec(label, "numeric") for label in NUMERIC_LABELS]
        )
    return []


def target_text_for_preset(preset: str) -> str:
    if preset == "letters":
        return "".join(LETTER_LABELS)
    if preset == "punct":
        return " ".join(PUNCT_LABELS)
    if preset == "numeric":
        return "".join(NUMERIC_LABELS)
    if preset == "all":
        return " ".join(LETTER_LABELS + ALPHA_CONTROL_LABELS + PUNCT_LABELS + NUMERIC_LABELS)
    return ""


def clone_script_state(state: ScriptedCaptureState) -> ScriptedCaptureState:
    return ScriptedCaptureState(
        targets=list(state.targets),
        reps_per_target=state.reps_per_target,
        target_index=state.target_index,
        rep_index=state.rep_index,
        session_id=state.session_id,
        active=state.active,
        total_saved=state.total_saved,
    )


def timestamp_slug() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def next_available_attempt_dir(root: Path, prefix: str = "attempt") -> Path:
    base = root / f"{prefix}_{timestamp_slug()}"
    if not base.exists():
        return base
    suffix = 1
    while True:
        candidate = root / f"{prefix}_{timestamp_slug()}_{suffix:02d}"
        if not candidate.exists():
            return candidate
        suffix += 1


def resolve_capture_output(
    output_arg: Optional[Path],
    session_dir_arg: Optional[Path],
    fresh: bool,
    output_explicit: bool,
) -> tuple[Path, Path]:
    if output_explicit and output_arg is not None:
        output_path = output_arg
        session_dir = output_path.parent
        if fresh and output_path.exists():
            raise SystemExit(
                "--fresh cannot reuse an explicit --output path. Use --session-dir or omit --output."
            )
        return session_dir, output_path

    root = CAPTURE_ROOT
    if session_dir_arg is not None:
        session_dir = session_dir_arg
        if fresh and session_dir.exists() and any(session_dir.iterdir()):
            session_dir = next_available_attempt_dir(session_dir.parent, prefix=session_dir.name)
    else:
        session_dir = next_available_attempt_dir(root)
    return session_dir, session_dir / "samples.jsonl"


def write_session_metadata(session_dir: Path, port: str, baud: int, output_path: Path, fresh: bool) -> None:
    session_dir.mkdir(parents=True, exist_ok=True)
    metadata = {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "pid": os.getpid(),
        "port": port,
        "baud": baud,
        "session_dir": str(session_dir.resolve()),
        "output_path": str(output_path.resolve()),
        "tinyml_jsonl_path": str((session_dir / "tinyml_dataset.jsonl").resolve()),
        "tinyml_csv_path": str((session_dir / "tinyml_dataset.csv").resolve()),
        "gui_log_path": str((session_dir / "gui.log").resolve()),
        "fresh_requested": bool(fresh),
    }
    (session_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


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
    def __init__(
        self,
        root: tk.Tk,
        port_name: str,
        baud: int,
        output_path: Path,
        instance_lock: Optional[SingleInstanceLock] = None,
    ):
        self.root = root
        self.port_name = port_name
        self.baud = baud
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.tinyml_jsonl_path = self.output_path.parent / "tinyml_dataset.jsonl"
        self.tinyml_csv_path = self.output_path.parent / "tinyml_dataset.csv"
        self.log_path = self.output_path.parent / "gui.log"
        self.log_file = self.log_path.open("a", encoding="utf-8", buffering=1)
        self.instance_lock = instance_lock

        self.serial: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None
        self.events: queue.Queue = queue.Queue()

        self.mode_var = tk.StringVar(value="stop")
        self.preset_var = tk.StringVar(value="letters")
        self.label_var = tk.StringVar(value="")
        self.autosave_var = tk.BooleanVar(value=False)
        self.port_var = tk.StringVar(value=port_name)
        self.status_var = tk.StringVar(value="Waiting for device...")
        self.saved_var = tk.StringVar(value="Saved: 0")
        self.script_targets_var = tk.StringVar(value=target_text_for_preset("letters"))
        self.script_reps_var = tk.IntVar(value=5)
        self.script_status_var = tk.StringVar(value="Script idle")
        self.script_target_var = tk.StringVar(value="-")
        self.script_condition_var = tk.StringVar(value="-")
        self.script_progress_var = tk.StringVar(value="0 / 0")
        self.script_detail_var = tk.StringVar(value="Waiting for device")

        self.saved_count = 0
        self.current_stroke: Optional[StrokeSample] = None
        self.last_stroke: Optional[StrokeSample] = None
        self.label_entry: Optional[ttk.Entry] = None
        self.script_state = ScriptedCaptureState()
        self.undo_stack: list[SavedEntry] = []
        self.device_ready = False
        self._connect_retry_after_id: Optional[str] = None
        self._configure_generation = 0
        self._configure_reason = ""
        self._configure_pending = False
        self._configure_mode_ack = False
        self._configure_trace_ack = False
        self._configure_status_ack = False
        self._ready_callbacks: list = []
        self._last_mode_value = ""
        self._last_trace_value = ""
        self._last_error_message = ""
        self.device_action_widgets: list[tk.Widget] = []
        self._closing = False
        self._boot_recover_after_id: Optional[str] = None
        self._boot_recover_generation = 0
        self._deferred_config_after_id: Optional[str] = None
        self._boot_event_active = False

        self.root.report_callback_exception = self._report_callback_exception
        self._build_ui()
        self._refresh_script_status()
        self._bind_keys()
        self._load_existing_saved_entries()
        self._set_device_ready(False, "Waiting for device...")
        self._attempt_connect()
        self.root.after(30, self._poll_events)
        self.root.after(100, self._tick)
        self._debug_log(f"app start pid={os.getpid()} port={self.port_name} baud={self.baud}")

    def _build_ui(self) -> None:
        self.root.title("Ringo Graffiti Capture")
        self.root.geometry("1260x820")
        self.root.minsize(1040, 700)

        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(3, weight=1)
        main.rowconfigure(4, weight=1)

        top = ttk.Frame(main)
        top.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        top.columnconfigure(9, weight=1)

        ttk.Label(top, text="Port").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.port_var, state="readonly", width=22).grid(row=0, column=1, sticky="w", padx=(4, 12))
        ttk.Label(top, text="Preset").grid(row=0, column=2, sticky="w")
        preset_combo = ttk.Combobox(top, textvariable=self.preset_var, values=TARGET_PRESETS, state="readonly", width=10)
        preset_combo.grid(row=0, column=3, sticky="w", padx=(4, 12))
        preset_combo.bind("<<ComboboxSelected>>", lambda _e: self._apply_preset())
        ttk.Label(top, text="Label").grid(row=0, column=4, sticky="w")
        self.label_entry = ttk.Entry(top, textvariable=self.label_var, width=18)
        self.label_entry.grid(row=0, column=5, sticky="w", padx=(4, 12))
        ttk.Checkbutton(top, text="Autosave", variable=self.autosave_var).grid(row=0, column=6, sticky="w", padx=(0, 12))
        ttk.Label(top, textvariable=self.saved_var).grid(row=0, column=7, sticky="w", padx=(0, 12))
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=8, sticky="w")

        controls = ttk.Frame(main)
        controls.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        for idx in range(14):
            controls.columnconfigure(idx, weight=0)
        controls.columnconfigure(13, weight=1)

        self.capture_btn = ttk.Button(controls, text="Capture", command=lambda: self.set_mode("capture"))
        self.capture_btn.grid(row=0, column=0, padx=(0, 8))
        self.continuous_btn = ttk.Button(controls, text="Continuous", command=lambda: self.set_mode("continuous"))
        self.continuous_btn.grid(row=0, column=1, padx=(0, 8))
        self.stop_btn = ttk.Button(controls, text="Stop", command=lambda: self.set_mode("stop"))
        self.stop_btn.grid(row=0, column=2, padx=(0, 20))
        self.graffiti_btn = ttk.Button(controls, text="Graffiti Mode", command=lambda: self.send_command("mode graffiti"))
        self.graffiti_btn.grid(row=0, column=3, padx=(0, 8))
        self.mouse_btn = ttk.Button(controls, text="Mouse Mode", command=lambda: self.send_command("mode mouse"))
        self.mouse_btn.grid(row=0, column=4, padx=(0, 20))
        self.save_last_btn = ttk.Button(controls, text="Save Last", command=self.save_last)
        self.save_last_btn.grid(row=0, column=5, padx=(0, 8))
        self.capture_next_btn = ttk.Button(controls, text="Capture Next", command=self.arm_capture_next)
        self.capture_next_btn.grid(row=0, column=6, padx=(0, 8))
        self.cancel_capture_btn = ttk.Button(controls, text="Cancel Capture", command=lambda: self.send_command("cap off"))
        self.cancel_capture_btn.grid(row=0, column=7, padx=(0, 8))
        self.undo_btn = ttk.Button(controls, text="Undo Last", command=self.undo_last)
        self.undo_btn.grid(row=0, column=8, padx=(0, 8))
        ttk.Label(controls, text="Targets").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(controls, textvariable=self.script_targets_var, width=48).grid(row=1, column=1, columnspan=5, sticky="ew", pady=(10, 0), padx=(0, 8))
        ttk.Label(controls, text="Reps").grid(row=1, column=6, sticky="w", pady=(10, 0))
        ttk.Spinbox(controls, from_=1, to=50, textvariable=self.script_reps_var, width=5).grid(row=1, column=7, sticky="w", pady=(10, 0), padx=(0, 8))
        self.start_script_btn = ttk.Button(controls, text="Start Script", command=self.start_scripted_capture)
        self.start_script_btn.grid(row=1, column=8, pady=(10, 0), padx=(0, 8))
        self.stop_script_btn = ttk.Button(controls, text="Stop Script", command=self.stop_scripted_capture)
        self.stop_script_btn.grid(row=1, column=9, pady=(10, 0), padx=(0, 8))
        ttk.Label(controls, text="Shortcuts: c v x l s g m u q").grid(row=0, column=13, sticky="e")

        self.device_action_widgets = [
            self.capture_btn,
            self.continuous_btn,
            self.stop_btn,
            self.graffiti_btn,
            self.mouse_btn,
            self.save_last_btn,
            self.capture_next_btn,
            self.cancel_capture_btn,
            self.undo_btn,
            self.start_script_btn,
            self.stop_script_btn,
        ]

        script_frame = ttk.LabelFrame(main, text="Scripted Capture")
        script_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        script_frame.columnconfigure(1, weight=1)
        script_frame.columnconfigure(3, weight=1)
        script_frame.columnconfigure(5, weight=1)
        ttk.Label(script_frame, text="Target").grid(row=0, column=0, sticky="w", padx=(10, 12), pady=(8, 2))
        tk.Label(script_frame, textvariable=self.script_target_var, font=("Helvetica", 36, "bold"), fg="#0f766e").grid(row=0, column=1, sticky="w", pady=(4, 2))
        ttk.Label(script_frame, text="Condition").grid(row=0, column=2, sticky="w", padx=(20, 12), pady=(8, 2))
        ttk.Label(script_frame, textvariable=self.script_condition_var, font=("Helvetica", 18, "bold")).grid(row=0, column=3, sticky="w", pady=(4, 2))
        ttk.Label(script_frame, text="Progress").grid(row=0, column=4, sticky="w", padx=(20, 12), pady=(8, 2))
        ttk.Label(script_frame, textvariable=self.script_progress_var, font=("Helvetica", 18, "bold")).grid(row=0, column=5, sticky="w", pady=(4, 2))
        ttk.Label(script_frame, textvariable=self.script_status_var).grid(row=1, column=0, columnspan=3, sticky="w", padx=(10, 10), pady=(0, 8))
        ttk.Label(script_frame, textvariable=self.script_detail_var).grid(row=1, column=3, columnspan=3, sticky="w", padx=(20, 10), pady=(0, 8))

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
        self.root.bind("<KeyPress-s>", lambda _e: self.save_last())
        self.root.bind("<KeyPress-u>", lambda _e: self.undo_last())
        self.root.bind("<KeyPress-q>", lambda _e: self.close())
        self.root.bind("<KeyPress-l>", lambda _e: self.focus_label())
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def focus_label(self) -> None:
        if self.label_entry is not None:
            self.label_entry.focus_set()
            self.label_entry.icursor("end")

    def _debug_log(self, message: str) -> None:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        try:
            self.log_file.write(f"{stamp} {message}\n")
        except Exception:
            pass

    def _report_callback_exception(self, exc_type, exc_value, exc_traceback) -> None:
        trace = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        self._debug_log("Tk callback exception:\n" + trace)
        try:
            self.log(f"callback exception: {exc_value}")
        except Exception:
            pass

    def log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self._debug_log(message)
        self.log_text.configure(state="normal")
        self.log_text.insert("1.0", f"{stamp} {message}\n")
        self.log_text.configure(state="disabled")
        self.log_text.see("1.0")
        self.status_var.set(message)

    def _set_device_ready(self, ready: bool, status: Optional[str] = None) -> None:
        self.device_ready = ready
        for widget in self.device_action_widgets:
            widget.configure(state="normal" if ready else "disabled")
        if self.label_entry is not None:
            self.label_entry.configure(state="normal")
        if status:
            self.status_var.set(status)
            if not self.script_state.active:
                self.script_detail_var.set(status)

    def _apply_preset(self) -> None:
        preset = self.preset_var.get()
        if preset != "custom":
            self.script_targets_var.set(target_text_for_preset(preset))

    def _load_existing_saved_entries(self) -> None:
        if not self.output_path.exists():
            self.saved_var.set("Saved: 0")
            return

        saved_count = 0
        undo_stack: list[SavedEntry] = []
        with self.output_path.open("r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except Exception:
                    continue
                saved_count += 1
                capture_uid = record.get("capture_uid")
                if not capture_uid:
                    continue
                script = record.get("script", {})
                prompts = parse_script_targets(script.get("prompt_label", "")) if not script.get("scripted") else []
                prior_state = ScriptedCaptureState(
                    targets=prompts,
                    reps_per_target=int(script.get("reps_per_target", 0) or 0),
                    target_index=int(script.get("target_index", -1) or -1),
                    rep_index=int(script.get("rep_index", -1) or -1),
                    session_id=script.get("session_id", ""),
                    active=bool(script.get("scripted", False)) and bool(prompts),
                    total_saved=max(0, saved_count - 1),
                )
                undo_stack.append(
                    SavedEntry(
                        capture_uid=capture_uid,
                        saved_count_before=max(0, saved_count - 1),
                        script_state_before=prior_state,
                    )
                )

        self.saved_count = saved_count
        self.undo_stack = undo_stack
        self.saved_var.set(f"Saved: {self.saved_count}")

    def _attempt_connect(self) -> None:
        if self.serial is not None:
            return
        candidate_ports: list[str] = []
        if self.port_name:
            candidate_ports.append(self.port_name)
        detected = autodetect_port()
        if detected and detected not in candidate_ports:
            candidate_ports.append(detected)
        if not candidate_ports:
            self._set_device_ready(False, "Waiting for device...")
            self._schedule_reconnect()
            return

        last_exc: Optional[Exception] = None
        for candidate in candidate_ports:
            try:
                self.serial = serial.Serial(candidate, baudrate=self.baud, timeout=0.10)
                self.port_name = candidate
                break
            except Exception as exc:  # pragma: no cover - hardware dependent
                last_exc = exc
                self.serial = None

        if self.serial is None:
            msg = f"waiting for device: {last_exc}" if last_exc is not None else "waiting for device"
            if msg != self._last_error_message:
                self.log(msg)
                self._last_error_message = msg
            self._set_device_ready(False, "Waiting for device...")
            self._schedule_reconnect()
            return

        try:
            self.serial.dtr = False
            self.serial.rts = False
            self.port_var.set(self.port_name)
            self.events = queue.Queue()
            self.reader = SerialReader(self.serial, self.events)
            self.reader.start()
            self._last_error_message = ""
            self.log(f"opened {self.port_name} @ {self.baud}")
            self._set_device_ready(False, "Connected; waiting for boot...")
            self._schedule_delayed_configure(2500, reason="startup", force=True)
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.serial = None
            self.reader = None
            msg = f"waiting for device: {exc}"
            if msg != self._last_error_message:
                self.log(msg)
                self._last_error_message = msg
            self._set_device_ready(False, "Waiting for device...")
            self._schedule_reconnect()

    def _schedule_reconnect(self) -> None:
        if self._connect_retry_after_id is not None:
            return
        self._connect_retry_after_id = self.root.after(1000, self._retry_connect)

    def _retry_connect(self) -> None:
        self._connect_retry_after_id = None
        if self.serial is None:
            self._attempt_connect()

    def _disconnect_serial(self) -> None:
        if self._deferred_config_after_id is not None:
            self.root.after_cancel(self._deferred_config_after_id)
            self._deferred_config_after_id = None
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        self._configure_pending = False
        self._configure_mode_ack = False
        self._configure_trace_ack = False
        self._configure_status_ack = False
        self.current_stroke = None
        self._render_live()
        self._boot_event_active = False
        self._set_device_ready(False, "Disconnected; retrying...")

    def _schedule_delayed_configure(self, delay_ms: int, reason: str, force: bool = True, on_ready=None) -> None:
        if on_ready is not None:
            self._ready_callbacks.append(on_ready)
        if self._deferred_config_after_id is not None:
            self.root.after_cancel(self._deferred_config_after_id)
        self._deferred_config_after_id = self.root.after(
            delay_ms,
            lambda: self._run_deferred_configure(reason, force),
        )

    def _run_deferred_configure(self, reason: str, force: bool) -> None:
        self._deferred_config_after_id = None
        self._configure_device(reason=reason, force=force)

    def _configure_device(self, reason: str = "startup", force: bool = False, on_ready=None) -> None:
        if on_ready is not None:
            self._ready_callbacks.append(on_ready)
        if self.device_ready and not force:
            self._run_ready_callbacks()
            return
        if self.serial is None:
            self._schedule_reconnect()
            return
        self._configure_generation += 1
        self._configure_reason = reason
        self._configure_pending = True
        self._configure_mode_ack = False
        self._configure_trace_ack = False
        self._configure_status_ack = False
        self._set_device_ready(False, "Configuring device...")
        self.send_command("mode graffiti", user_visible=False)
        self.root.after(80, lambda: self.send_command("trace cont", user_visible=False))
        self.root.after(160, lambda: self.send_command("status", user_visible=False))
        current_generation = self._configure_generation
        self.root.after(3000, lambda: self._configuration_timeout(current_generation))

    def _configuration_timeout(self, generation: int) -> None:
        if generation != self._configure_generation or self.device_ready or not self._configure_pending:
            return
        self.log(f"device configuration timed out during {self._configure_reason}; retrying setup")
        if self.serial is not None:
            self._configure_device(reason=self._configure_reason, force=True)
            return
        self._disconnect_serial()
        self._schedule_reconnect()

    def _run_ready_callbacks(self) -> None:
        callbacks = list(self._ready_callbacks)
        self._ready_callbacks.clear()
        for callback in callbacks:
            try:
                callback()
            except Exception as exc:
                self.log(f"ready callback failed: {exc}")

    def _resume_after_recovery(self) -> None:
        if not self.script_state.active:
            self.log("device recovered")
            return
        current = self.script_state.current_target()
        if current is None:
            self.log("device recovered")
            return
        self.label_var.set(current.label)
        self.autosave_var.set(True)
        self._refresh_script_status()
        self.log(
            f"device recovered; resuming {current.label} ({current.condition}) "
            f"rep {self.script_state.rep_index + 1}/{self.script_state.reps_per_target}"
        )

    def _handle_device_boot(self, source: str) -> None:
        if self._closing:
            return
        if self._boot_event_active:
            self._debug_log(f"boot event already active; ignored duplicate signal from {source}")
            return
        self._boot_event_active = True
        self._boot_recover_generation += 1
        generation = self._boot_recover_generation
        if self._deferred_config_after_id is not None:
            self.root.after_cancel(self._deferred_config_after_id)
            self._deferred_config_after_id = None
        if self._boot_recover_after_id is not None:
            self.root.after_cancel(self._boot_recover_after_id)
            self._boot_recover_after_id = None
        self.current_stroke = None
        self._render_live()
        self._configure_pending = False
        self._configure_mode_ack = False
        self._configure_trace_ack = False
        self._configure_status_ack = False
        self._set_device_ready(False, f"Device reboot detected ({source}); recovering...")
        if self.script_state.active:
            current = self.script_state.current_target()
            if current is not None:
                self.script_detail_var.set(
                    f"Recovering session {self.script_state.session_id} at "
                    f"{current.label} {self.script_state.rep_index + 1}/{self.script_state.reps_per_target}"
                )
        self.log(f"device reboot detected via {source}; waiting for boot")
        self._boot_recover_after_id = self.root.after(2500, lambda: self._run_boot_recovery(generation))

    def _run_boot_recovery(self, generation: int) -> None:
        if generation != self._boot_recover_generation or self._closing:
            return
        self._boot_recover_after_id = None
        if self.serial is None:
            self._schedule_reconnect()
            return
        self._configure_device(reason="recovery", force=True, on_ready=self._resume_after_recovery)

    def _mark_device_ready(self, source: str = "device") -> None:
        if self._configure_pending:
            if not (self._configure_mode_ack and self._configure_trace_ack and self._configure_status_ack):
                return
            self._configure_pending = False
        self._boot_event_active = False
        if not self.device_ready:
            self._set_device_ready(True, f"Ready ({source})")
            self.log(f"device ready via {source}")
        self._run_ready_callbacks()

    def send_command(self, command: str, user_visible: bool = True) -> None:
        if self.serial is None:
            if user_visible:
                self.log("device not connected")
            return
        try:
            self.serial.write((command + "\n").encode("utf-8"))
            self.serial.flush()
            if user_visible:
                self.log(f"> {command}")
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.log(f"serial write failed: {exc}")
            self._disconnect_serial()
            self._schedule_reconnect()

    def _tick(self) -> None:
        if self._closing:
            return
        if self.serial is None and self._connect_retry_after_id is None:
            self._schedule_reconnect()
        self.root.after(250, self._tick)

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

    def arm_capture_next(self) -> None:
        label = normalize_target_token(self.label_var.get())
        if infer_condition(label) is None:
            self.log("set a valid label first")
            return
        self.send_command(f"cap {label}")

    def _script_meta_for_prompt(self, prompt: Optional[PromptSpec]) -> dict:
        if not self.script_state.active or prompt is None:
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
            "prompt_label": prompt.label,
        }

    def _ensure_tinyml_csv_schema(self, headers: list[str]) -> None:
        if not self.tinyml_csv_path.exists():
            return
        if self.tinyml_csv_path.stat().st_size == 0:
            return
        with self.tinyml_csv_path.open("r", newline="", encoding="utf-8") as fh:
            reader = csv.reader(fh)
            existing_headers = next(reader, [])
        if not existing_headers:
            return
        if existing_headers == headers:
            return
        stamp = time.strftime("%Y%m%dT%H%M%S")
        backup_path = self.tinyml_csv_path.with_name(f"{self.tinyml_csv_path.name}.bak-{stamp}")
        shutil.copy2(self.tinyml_csv_path, backup_path)
        self.tinyml_csv_path.write_text("", encoding="utf-8")
        self.log(f"backed up mismatched CSV schema to {backup_path.name}")

    def _write_tinyml_csv_row(self, record: dict) -> None:
        sample_count = record["features"]["sample_count"]
        headers = [
            "capture_uid",
            "label",
            "condition",
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
        self._ensure_tinyml_csv_schema(headers)

        row = {
            "capture_uid": record["capture_uid"],
            "label": record["label"],
            "condition": record["condition"],
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

    def _append_jsonl(self, path: Path, record: dict) -> None:
        with path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, separators=(",", ":")) + "\n")

    def save_sample(
        self,
        sample: StrokeSample,
        label: str,
        condition: str,
        script_meta: Optional[dict] = None,
        script_state_before: Optional[ScriptedCaptureState] = None,
    ) -> bool:
        point_count = sample.point_count or len(sample.points)
        if point_count < 2:
            self.log(f"ignored stroke {sample.stroke_id}: not enough points")
            return False
        if sample.saved and sample.label == label and sample.condition == condition:
            self.log(f"stroke {sample.stroke_id} already saved as {label}")
            return False

        features = derive_feature_payload(sample)
        if script_meta is None:
            script_meta = self._script_meta_for_prompt(None)
        if script_state_before is None:
            script_state_before = clone_script_state(self.script_state)

        capture_uid = f"{int(time.time() * 1000)}-{sample.stroke_id}"
        record = asdict(sample)
        record["capture_uid"] = capture_uid
        record["label"] = label
        record["condition"] = condition
        record["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        record["script"] = script_meta
        record["features"] = features
        self._append_jsonl(self.output_path, record)
        self._append_jsonl(self.tinyml_jsonl_path, record)
        self._write_tinyml_csv_row(record)

        sample.label = label
        sample.condition = condition
        sample.capture_uid = capture_uid
        sample.saved = True
        self.undo_stack.append(
            SavedEntry(
                capture_uid=capture_uid,
                saved_count_before=self.saved_count,
                script_state_before=script_state_before,
            )
        )
        self.saved_count += 1
        self.saved_var.set(f"Saved: {self.saved_count}")
        self.log(f"saved stroke {sample.stroke_id} as {label} ({condition})")
        self._render_meta()
        return True

    def _remove_last_jsonl_entry(self, path: Path, capture_uid: str) -> bool:
        if not path.exists():
            return False
        lines = path.read_text(encoding="utf-8").splitlines()
        removed = False
        for idx in range(len(lines) - 1, -1, -1):
            try:
                if json.loads(lines[idx]).get("capture_uid") == capture_uid:
                    del lines[idx]
                    removed = True
                    break
            except Exception:
                continue
        if removed:
            path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        return removed

    def _remove_last_csv_entry(self, path: Path, capture_uid: str) -> bool:
        if not path.exists():
            return False
        with path.open("r", newline="", encoding="utf-8") as fh:
            reader = list(csv.DictReader(fh))
            headers = reader[0].keys() if reader else None
        removed = False
        for idx in range(len(reader) - 1, -1, -1):
            if reader[idx].get("capture_uid") == capture_uid:
                del reader[idx]
                removed = True
                break
        if removed and headers is not None:
            with path.open("w", newline="", encoding="utf-8") as fh:
                writer = csv.DictWriter(fh, fieldnames=list(headers))
                writer.writeheader()
                writer.writerows(reader)
        elif removed and headers is None:
            path.write_text("", encoding="utf-8")
        return removed

    def undo_last(self) -> None:
        if not self.undo_stack:
            self.log("nothing to undo")
            return
        entry = self.undo_stack.pop()
        removed_raw = self._remove_last_jsonl_entry(self.output_path, entry.capture_uid)
        removed_jsonl = self._remove_last_jsonl_entry(self.tinyml_jsonl_path, entry.capture_uid)
        removed_csv = self._remove_last_csv_entry(self.tinyml_csv_path, entry.capture_uid)
        self.saved_count = entry.saved_count_before
        self.saved_var.set(f"Saved: {self.saved_count}")
        self.script_state = clone_script_state(entry.script_state_before)
        self._refresh_script_status()
        prompt = self.script_state.current_target()
        if prompt is not None:
            self.label_var.set(prompt.label)
        if self.last_stroke and self.last_stroke.capture_uid == entry.capture_uid:
            self.last_stroke.saved = False
            self.last_stroke.capture_uid = ""
        self.log(
            f"undo {entry.capture_uid}: raw={int(removed_raw)} tinyml_jsonl={int(removed_jsonl)} csv={int(removed_csv)}"
        )
        self._render_meta()

    def _refresh_script_status(self) -> None:
        current = self.script_state.current_target()
        if current is None:
            self.script_status_var.set("Script idle")
            self.script_target_var.set("-")
            self.script_condition_var.set("-")
            self.script_progress_var.set("0 / 0")
            self.script_detail_var.set("Ready" if self.device_ready else "Waiting for device")
            return
        self.script_target_var.set(current.display)
        self.script_condition_var.set(current.condition)
        self.script_progress_var.set(f"{self.script_state.rep_index + 1} / {self.script_state.reps_per_target}")
        self.script_status_var.set(f"Target {self.script_state.target_index + 1} of {len(self.script_state.targets)}")
        self.script_detail_var.set(
            f"Session {self.script_state.session_id}   saved {self.script_state.total_saved}"
        )

    def _begin_scripted_capture(self, targets: list[PromptSpec], reps: int) -> None:
        self.script_state = ScriptedCaptureState(
            targets=targets,
            reps_per_target=reps,
            session_id=time.strftime("%Y%m%dT%H%M%S"),
            active=True,
        )
        current = self.script_state.current_target()
        if current is not None:
            self.label_var.set(current.label)
        self.autosave_var.set(True)
        self._refresh_script_status()
        self.log(f"script started: {len(targets)} targets x {reps} reps")

    def start_scripted_capture(self) -> None:
        preset = self.preset_var.get()
        if preset == "custom":
            targets = parse_script_targets(self.script_targets_var.get())
        else:
            targets = prompt_specs_for_preset(preset)
            self.script_targets_var.set(target_text_for_preset(preset))
        if not targets:
            self.log("no scripted targets configured")
            return
        reps = max(1, int(self.script_reps_var.get()))
        self._configure_device(
            reason="script start",
            force=True,
            on_ready=lambda: self._begin_scripted_capture(targets, reps),
        )

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
        current = self.script_state.current_target()
        if current is not None:
            self.label_var.set(current.label)
        self._refresh_script_status()
        if current is not None:
            self.log(
                f"next target {current.label} ({current.condition}) rep {self.script_state.rep_index + 1}/{self.script_state.reps_per_target}"
            )

    def save_last(self) -> None:
        if self.last_stroke is None:
            self.log("no stroke to save")
            return
        label = normalize_target_token(self.label_var.get())
        condition = infer_condition(label)
        if condition is None:
            self.log("set a valid label first")
            return
        self.save_sample(self.last_stroke, label, condition)

    def process_line(self, line: str) -> None:
        if not line:
            return
        if BOOT_BANNER_RE.match(line):
            self.log(line)
            self._handle_device_boot("boot banner")
            return
        if line.startswith("ESP-ROM:") or line.startswith("rst:") or line.startswith("entry "):
            self.log(line)
            self._handle_device_boot("rom boot")
            return
        mode_match = CMD_MODE_RE.match(line)
        if mode_match:
            self._last_mode_value = mode_match.group(1).lower()
            self._configure_status_ack = True
            if self._last_mode_value == "graffiti":
                self._configure_mode_ack = True
            self.log(line)
            self._mark_device_ready("status")
            return
        trace_match = CMD_TRACE_RE.match(line)
        if trace_match:
            self._last_trace_value = trace_match.group(1).lower()
            if self._last_trace_value in {"cont", "continuous"}:
                self._configure_trace_ack = True
            self.log(line)
            self._mark_device_ready("trace")
            return
        mode_switch_match = MODE_SWITCH_RE.match(line)
        if mode_switch_match:
            self._last_mode_value = mode_switch_match.group(1).lower()
            if self._last_mode_value == "graffiti":
                self._configure_mode_ack = True
            self.log(line)
            self._mark_device_ready("mode")
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
            self.current_stroke.points.append([int(match.group(2)), int(match.group(3)), int(match.group(4))])
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
            prompt = self.script_state.current_target()
            if prompt is not None:
                state_before = clone_script_state(self.script_state)
                saved_in_script = self.save_sample(
                    sample,
                    prompt.label,
                    prompt.condition,
                    script_meta=self._script_meta_for_prompt(prompt),
                    script_state_before=state_before,
                )
                if saved_in_script:
                    self._advance_scripted_capture()
            elif self.autosave_var.get():
                label = normalize_target_token(self.label_var.get())
                condition = infer_condition(label)
                if condition is not None:
                    self.save_sample(sample, label, condition)
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
                self._disconnect_serial()
                self._schedule_reconnect()
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
        canvas.create_text(
            12,
            12,
            anchor="nw",
            fill="#cbd5e1",
            text=(
                f"id:{sample.stroke_id}  pts:{sample.point_count or len(sample.points)}  "
                f"seq:{sample.seq}  label:{sample.label or '-'}  cond:{sample.condition or '-'}"
            ),
        )

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
                f"condition: {sample.condition or '-'}",
                f"capture_uid: {sample.capture_uid or '-'}",
                f"script_target: {self.script_target_var.get()}",
                f"script_condition: {self.script_condition_var.get()}",
                f"script_progress: {self.script_progress_var.get()}",
            ]
            self.meta_text.insert("1.0", "\n".join(lines))
        self.meta_text.configure(state="disabled")

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self._debug_log("close requested")
        try:
            self.send_command("trace off", user_visible=False)
        except Exception:
            pass
        if self._boot_recover_after_id is not None:
            self.root.after_cancel(self._boot_recover_after_id)
            self._boot_recover_after_id = None
        if self._connect_retry_after_id is not None:
            self.root.after_cancel(self._connect_retry_after_id)
            self._connect_retry_after_id = None
        if self._deferred_config_after_id is not None:
            self.root.after_cancel(self._deferred_config_after_id)
            self._deferred_config_after_id = None
        self._disconnect_serial()
        if self.instance_lock is not None:
            self.instance_lock.release()
            self.instance_lock = None
        try:
            self.log_file.close()
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
        default=None,
        help="Explicit JSONL output path. If omitted, a per-attempt session directory is created automatically.",
    )
    parser.add_argument(
        "--session-dir",
        type=Path,
        default=None,
        help="Session directory to hold samples.jsonl, tinyml_dataset.*, gui.log, and metadata.json",
    )
    parser.add_argument(
        "--fresh",
        action="store_true",
        help="Start a fresh attempt directory instead of reusing an existing session directory",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.port:
        print("No serial port found.")
        return 1
    instance_lock = SingleInstanceLock(CAPTURE_ROOT / "gui.lock")
    if not instance_lock.acquire():
        print(f"Another graffiti_capture_gui.py instance is already running. Check {CAPTURE_ROOT / 'gui.lock'}.")
        return 2
    output_explicit = "--output" in sys.argv
    session_dir, output_path = resolve_capture_output(args.output, args.session_dir, args.fresh, output_explicit)
    session_dir.mkdir(parents=True, exist_ok=True)
    write_session_metadata(session_dir, args.port, args.baud, output_path, args.fresh)
    root = tk.Tk()
    app = GraffitiCaptureGui(root, args.port, args.baud, output_path, instance_lock=instance_lock)
    app._render_live()
    app._render_last()
    app._render_meta()
    try:
        root.mainloop()
        return 0
    except Exception:
        trace = traceback.format_exc()
        app._debug_log("fatal exception:\n" + trace)
        raise
    finally:
        if app.instance_lock is not None:
            app.instance_lock.release()
            app.instance_lock = None


if __name__ == "__main__":
    raise SystemExit(main())
