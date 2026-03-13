#!/usr/bin/env python3
from __future__ import annotations

import argparse
import curses
import json
import queue
import re
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
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
            try:
                text = line.decode("utf-8", errors="replace").strip()
            except Exception:
                text = repr(line)
            self._out_queue.put(("line", text))

    def stop(self) -> None:
        self._alive = False


class CaptureConsole:
    def __init__(self, port: str, baud: int, output_path: Path):
        self.port_name = port
        self.baud = baud
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.10)
        self.events: queue.Queue = queue.Queue()
        self.reader = SerialReader(self.serial, self.events)
        self.logs: deque[str] = deque(maxlen=10)
        self.mode = "stop"
        self.label = ""
        self.autosave = False
        self.saved_count = 0
        self.current_stroke: Optional[StrokeSample] = None
        self.last_stroke: Optional[StrokeSample] = None

    def log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.logs.appendleft(f"{stamp} {message}")

    def send_command(self, command: str) -> None:
        self.serial.write((command + "\n").encode("utf-8"))
        self.serial.flush()
        self.log(f"> {command}")

    def set_mode(self, mode: str) -> None:
        if mode == "capture":
            self.send_command("trace once")
        elif mode == "continuous":
            self.send_command("trace cont")
        else:
            self.send_command("trace off")
            mode = "stop"
        self.mode = mode

    def save_sample(self, sample: StrokeSample, label: str) -> None:
        record = asdict(sample)
        record["label"] = label
        record["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        with self.output_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, separators=(",", ":")) + "\n")
        sample.label = label
        sample.saved = True
        self.saved_count += 1
        self.log(f"saved stroke {sample.stroke_id} as {label}")

    def prompt(self, stdscr, title: str, initial: str = "") -> str:
        curses.curs_set(1)
        height, width = stdscr.getmaxyx()
        prompt = f"{title}: "
        value = initial
        while True:
            stdscr.move(height - 1, 0)
            stdscr.clrtoeol()
            text = (prompt + value)[: max(0, width - 1)]
            stdscr.addstr(height - 1, 0, text)
            stdscr.refresh()
            key = stdscr.getch()
            if key in (10, 13):
                break
            if key in (27,):
                value = initial
                break
            if key in (curses.KEY_BACKSPACE, 127, 8):
                value = value[:-1]
                continue
            if 32 <= key <= 126:
                value += chr(key)
        curses.curs_set(0)
        return value.strip()

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
            return

        match = TRACE_POINT_RE.match(line)
        if match:
            stroke_id = int(match.group(1))
            if self.current_stroke is None or self.current_stroke.stroke_id != stroke_id:
                self.current_stroke = StrokeSample(stroke_id=stroke_id, captured_at_ms=int(time.time() * 1000))
            self.current_stroke.points.append(
                [int(match.group(2)), int(match.group(3)), int(match.group(4))]
            )
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
            if sample.trace_mode == "capture" or self.mode == "capture":
                self.mode = "stop"
            self.log(
                f"trace end #{stroke_id} status={sample.status} pred={sample.pred} score={sample.score:.2f}"
            )
            if self.autosave and self.label:
                self.save_sample(sample, self.label)
            return

        self.log(line)

    def pump_events(self) -> None:
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                return
            if kind == "line":
                self.process_line(payload)
            else:
                self.log(f"serial error: {payload}")

    def draw_box(self, win, title: str) -> None:
        win.box()
        width = win.getmaxyx()[1]
        title_text = f" {title} "[: max(0, width - 2)]
        if len(title_text) < width - 1:
            win.addstr(0, 2, title_text)

    def draw_stroke(self, win, sample: Optional[StrokeSample], title: str) -> None:
        self.draw_box(win, title)
        max_y, max_x = win.getmaxyx()
        if sample is None or not sample.points:
            win.addstr(2, 2, "no points")
            return

        points = sample.points
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        min_x, max_x_src = min(xs), max(xs)
        min_y, max_y_src = min(ys), max(ys)
        span_x = max(1, max_x_src - min_x)
        span_y = max(1, max_y_src - min_y)
        plot_w = max(1, max_x - 4)
        plot_h = max(1, max_y - 4)

        for px, py, _ in points:
            x = 2 + int((px - min_x) * (plot_w - 1) / span_x)
            y = 2 + int((py - min_y) * (plot_h - 1) / span_y)
            y = min(max_y - 2, max(1, y))
            x = min(max_x - 2, max(1, x))
            win.addch(y, x, ord("."))

        end_x = 2 + int((points[-1][0] - min_x) * (plot_w - 1) / span_x)
        end_y = 2 + int((points[-1][1] - min_y) * (plot_h - 1) / span_y)
        end_y = min(max_y - 2, max(1, end_y))
        end_x = min(max_x - 2, max(1, end_x))
        win.addch(end_y, end_x, ord("*"))

    def draw_meta(self, win, sample: Optional[StrokeSample], title: str) -> None:
        self.draw_box(win, title)
        if sample is None:
            win.addstr(2, 2, "no stroke")
            return
        lines = [
            f"id: {sample.stroke_id}",
            f"status: {sample.status}",
            f"pred: {sample.pred}",
            f"score: {sample.score:.2f}",
            f"dist: {sample.dist:.2f}",
            f"seq: {sample.seq}",
            f"points: {sample.point_count}",
            f"dur: {sample.duration_ms} ms",
            f"dx/dy: {sample.delta_x}/{sample.delta_y}",
            f"saved: {'yes' if sample.saved else 'no'}",
            f"label: {sample.label or '-'}",
        ]
        for idx, line in enumerate(lines, start=2):
            if idx >= win.getmaxyx()[0] - 1:
                break
            win.addstr(idx, 2, line[: win.getmaxyx()[1] - 4])

    def run(self, stdscr) -> None:
        curses.curs_set(0)
        stdscr.nodelay(True)
        self.log(f"opened {self.port_name} @ {self.baud}")
        self.reader.start()
        self.send_command("trace off")

        while True:
            self.pump_events()
            stdscr.erase()
            height, width = stdscr.getmaxyx()
            header = (
                f"Port:{self.port_name}  Mode:{self.mode}  Label:{self.label or '-'}  "
                f"Autosave:{'on' if self.autosave else 'off'}  Saved:{self.saved_count}"
            )
            stdscr.addstr(0, 0, header[: max(0, width - 1)])
            stdscr.addstr(1, 0, f"Out:{str(self.output_path)}"[: max(0, width - 1)])

            canvas_h = max(10, height - 8)
            canvas_w = max(24, width // 2)
            live_win = stdscr.derwin(canvas_h, canvas_w, 2, 0)
            last_win = stdscr.derwin(canvas_h, width - canvas_w, 2, canvas_w)

            self.draw_stroke(live_win, self.current_stroke, "Live")
            if width - canvas_w >= 30:
                self.draw_meta(last_win, self.last_stroke, "Last Stroke")
            else:
                self.draw_stroke(last_win, self.last_stroke, "Last")

            help_line = "c:capture  v:continuous  x:stop  l:label  s:save-last  a:autosave  g:graffiti  m:mouse  q:quit"
            stdscr.addstr(height - 3, 0, help_line[: max(0, width - 1)])

            for idx, line in enumerate(list(self.logs)[:2], start=height - 2):
                if idx >= height:
                    break
                stdscr.addstr(idx, 0, line[: max(0, width - 1)])

            stdscr.refresh()

            key = stdscr.getch()
            if key == -1:
                time.sleep(0.03)
                continue
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("c"), ord("C")):
                self.set_mode("capture")
                continue
            if key in (ord("v"), ord("V")):
                self.set_mode("continuous")
                continue
            if key in (ord("x"), ord("X")):
                self.set_mode("stop")
                continue
            if key in (ord("g"), ord("G")):
                self.send_command("mode graffiti")
                continue
            if key in (ord("m"), ord("M")):
                self.send_command("mode mouse")
                continue
            if key in (ord("a"), ord("A")):
                self.autosave = not self.autosave
                self.log(f"autosave {'on' if self.autosave else 'off'}")
                continue
            if key in (ord("l"), ord("L")):
                value = self.prompt(stdscr, "label", self.label)
                self.label = value
                self.log(f"label={self.label or '-'}")
                continue
            if key in (ord("s"), ord("S")):
                if self.last_stroke is None:
                    self.log("no stroke to save")
                elif not self.label:
                    self.log("set a label first")
                else:
                    self.save_sample(self.last_stroke, self.label)
                continue

        try:
            self.send_command("trace off")
        except Exception:
            pass
        self.reader.stop()
        self.serial.close()


def autodetect_port() -> Optional[str]:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    usbmodem = [p for p in ports if "usbmodem" in p]
    if usbmodem:
        return sorted(usbmodem)[0]
    return sorted(ports)[0] if ports else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live graffiti stroke capture console")
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
    app = CaptureConsole(args.port, args.baud, args.output)
    curses.wrapper(app.run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
