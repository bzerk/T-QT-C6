#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if stripped:
                yield json.loads(stripped)


def dataset_paths_from_input(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    candidates = []
    for filename in ("tinyml_dataset.jsonl", "tinyml_dataset.csv"):
        candidate = path / filename
        if candidate.exists():
            candidates.append(candidate)
    return candidates


def main() -> int:
    parser = argparse.ArgumentParser(description="Combine one or more graffiti dataset attempts into a curated dataset.")
    parser.add_argument("inputs", nargs="+", type=Path, help="Attempt directories and/or dataset files")
    parser.add_argument("--output-dir", type=Path, required=True, help="Destination directory for combined dataset")
    args = parser.parse_args()

    dataset_files: list[Path] = []
    for item in args.inputs:
        dataset_files.extend(dataset_paths_from_input(item))
    dataset_files = [path for path in dataset_files if path.suffix.lower() == ".jsonl"]
    if not dataset_files:
        raise SystemExit("no tinyml_dataset.jsonl inputs found")

    merged: list[dict] = []
    counts = Counter()
    by_condition = Counter()
    for path in dataset_files:
        for record in iter_jsonl(path):
            merged.append(record)
            counts[str(record.get("label", ""))] += 1
            by_condition[str(record.get("condition", ""))] += 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = args.output_dir / "tinyml_dataset.jsonl"
    csv_path = args.output_dir / "tinyml_dataset.csv"
    manifest_path = args.output_dir / "manifest.json"

    with jsonl_path.open("w", encoding="utf-8") as handle:
        for record in merged:
            handle.write(json.dumps(record, sort_keys=True) + "\n")

    fieldnames = [
        "capture_uid",
        "label",
        "condition",
        "session_id",
        "prompt_label",
        "duration_ms",
        "path_length_px",
        "bbox_w_px",
        "bbox_h_px",
    ]
    for idx in range(32):
        fieldnames.extend([f"x{idx:02d}", f"y{idx:02d}", f"dx{idx:02d}", f"dy{idx:02d}"])

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for record in merged:
            features = record["features"]
            script = record.get("script") or {}
            row = {
                "capture_uid": record.get("capture_uid", ""),
                "label": record.get("label", ""),
                "condition": record.get("condition", ""),
                "session_id": script.get("session_id", record.get("session_id", "")),
                "prompt_label": script.get("prompt_label", record.get("prompt_label", "")),
                "duration_ms": record.get("duration_ms", ""),
                "path_length_px": features.get("path_length_px", ""),
                "bbox_w_px": features.get("bbox_w_px", ""),
                "bbox_h_px": features.get("bbox_h_px", ""),
            }
            for idx, (xy, dxy) in enumerate(zip(features["resampled_xy"], features["resampled_dxy"])):
                row[f"x{idx:02d}"] = xy[0]
                row[f"y{idx:02d}"] = xy[1]
                row[f"dx{idx:02d}"] = dxy[0]
                row[f"dy{idx:02d}"] = dxy[1]
            writer.writerow(row)

    manifest = {
        "sources": [str(path.resolve()) for path in dataset_files],
        "samples": len(merged),
        "label_counts": dict(sorted(counts.items())),
        "condition_counts": dict(sorted(by_condition.items())),
        "jsonl_path": str(jsonl_path.resolve()),
        "csv_path": str(csv_path.resolve()),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
