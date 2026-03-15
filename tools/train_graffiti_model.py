#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
import textwrap
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

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
EXPECTED_LABELS = LETTER_LABELS + ALPHA_CONTROL_LABELS + PUNCT_LABELS + NUMERIC_LABELS
CONDITIONS = ["letters", "punct", "numeric"]
LABEL_TO_CONDITION = {label: "letters" for label in LETTER_LABELS}
LABEL_TO_CONDITION.update({label: "letters" for label in ALPHA_CONTROL_LABELS})
LABEL_TO_CONDITION.update({label: "punct" for label in PUNCT_LABELS})
LABEL_TO_CONDITION.update({label: "numeric" for label in NUMERIC_LABELS})
LABELS_BY_CONDITION = {
    "letters": LETTER_LABELS + ALPHA_CONTROL_LABELS,
    "punct": PUNCT_LABELS,
    "numeric": NUMERIC_LABELS,
}
FEATURE_FORMAT = "resampled_xy32_plus_dxy32_v1"
STROKE_SAMPLE_COUNT = 32
STROKE_FEATURE_DIM = STROKE_SAMPLE_COUNT * 4


@dataclass
class SampleRecord:
    capture_uid: str
    label: str
    condition: str
    session_id: str
    prompt_label: str
    features: list[float]
    source_path: str


@dataclass
class SplitResult:
    train: list[SampleRecord]
    val: list[SampleRecord]
    method: str
    notes: list[str]


class TrainingError(RuntimeError):
    pass


def canonical_label_order(labels: Iterable[str]) -> list[str]:
    seen = set(labels)
    ordered = [label for label in EXPECTED_LABELS if label in seen]
    ordered.extend(sorted(seen - set(ordered)))
    return ordered


def encode_condition(condition: str, weight: float) -> list[float]:
    return [weight if condition == name else 0.0 for name in CONDITIONS]


def iter_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                yield json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise TrainingError(f"invalid JSONL at {path}:{line_number}: {exc}") from exc


def flatten_feature_record(record: dict) -> list[float]:
    features = record.get("features") or {}
    xy = features.get("resampled_xy") or []
    dxy = features.get("resampled_dxy") or []
    if len(xy) != STROKE_SAMPLE_COUNT or len(dxy) != STROKE_SAMPLE_COUNT:
        raise TrainingError(
            f"expected {STROKE_SAMPLE_COUNT} resampled points and deltas, got {len(xy)} / {len(dxy)}"
        )
    flattened: list[float] = []
    for idx in range(STROKE_SAMPLE_COUNT):
        point = xy[idx]
        delta = dxy[idx]
        if len(point) != 2 or len(delta) != 2:
            raise TrainingError(f"invalid feature payload at sample index {idx}")
        flattened.extend([float(point[0]), float(point[1]), float(delta[0]), float(delta[1])])
    return flattened


def sample_from_json_record(record: dict, source_path: Path, ordinal: int) -> SampleRecord:
    label = str(record.get("label", "")).strip()
    condition = str(record.get("condition", "")).strip()
    if not label:
        raise TrainingError(f"missing label in {source_path} record {ordinal}")
    if label not in LABEL_TO_CONDITION:
        raise TrainingError(f"unsupported label {label!r} in {source_path} record {ordinal}")
    expected_condition = LABEL_TO_CONDITION[label]
    if not condition:
        condition = expected_condition
    if condition != expected_condition:
        condition = expected_condition
    capture_uid = str(record.get("capture_uid") or f"{source_path.stem}-{ordinal}")
    script_meta = record.get("script") or {}
    session_id = str(script_meta.get("session_id") or record.get("session_id") or capture_uid)
    prompt_label = str(script_meta.get("prompt_label") or record.get("prompt_label") or label)
    return SampleRecord(
        capture_uid=capture_uid,
        label=label,
        condition=condition,
        session_id=session_id,
        prompt_label=prompt_label,
        features=flatten_feature_record(record),
        source_path=str(source_path),
    )


def sample_from_csv_row(row: dict[str, str], source_path: Path, ordinal: int) -> SampleRecord:
    label = (row.get("label") or "").strip()
    condition = (row.get("condition") or "").strip()
    if not label:
        raise TrainingError(f"missing label in {source_path} row {ordinal}")
    if label not in LABEL_TO_CONDITION:
        raise TrainingError(f"unsupported label {label!r} in {source_path} row {ordinal}")
    expected_condition = LABEL_TO_CONDITION[label]
    if not condition:
        condition = expected_condition
    if condition != expected_condition:
        condition = expected_condition
    flattened: list[float] = []
    for idx in range(STROKE_SAMPLE_COUNT):
        keys = (f"x{idx:02d}", f"y{idx:02d}", f"dx{idx:02d}", f"dy{idx:02d}")
        try:
            flattened.extend([float(row[key]) for key in keys])
        except KeyError as exc:
            raise TrainingError(f"missing feature column {exc.args[0]!r} in {source_path}") from exc
        except ValueError as exc:
            raise TrainingError(f"invalid numeric value in {source_path} row {ordinal}: {exc}") from exc
    capture_uid = (row.get("capture_uid") or f"{source_path.stem}-{ordinal}").strip()
    session_id = (row.get("session_id") or capture_uid).strip() or capture_uid
    prompt_label = (row.get("prompt_label") or label).strip() or label
    return SampleRecord(
        capture_uid=capture_uid,
        label=label,
        condition=condition,
        session_id=session_id,
        prompt_label=prompt_label,
        features=flattened,
        source_path=str(source_path),
    )


def load_dataset(path: Path) -> list[SampleRecord]:
    if not path.exists():
        raise TrainingError(f"dataset not found: {path}")
    suffix = path.suffix.lower()
    samples: list[SampleRecord] = []
    if suffix == ".jsonl":
        for ordinal, record in enumerate(iter_jsonl(path), start=1):
            samples.append(sample_from_json_record(record, path, ordinal))
    elif suffix == ".csv":
        with path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for ordinal, row in enumerate(reader, start=2):
                samples.append(sample_from_csv_row(row, path, ordinal))
    else:
        raise TrainingError(f"unsupported dataset format: {path.suffix}")

    if not samples:
        raise TrainingError(f"dataset is empty: {path}")
    if len(samples[0].features) != STROKE_FEATURE_DIM:
        raise TrainingError(
            f"unexpected feature dimension {len(samples[0].features)}; expected {STROKE_FEATURE_DIM}"
        )
    return samples


def auto_dataset_path(root: Path) -> Optional[Path]:
    capture_root = root / "debug/graffiti_capture"
    attempt_candidates: list[Path] = []
    if capture_root.exists():
        for directory in capture_root.iterdir():
            if not directory.is_dir():
                continue
            if not (directory.name.startswith("attempt_") or directory.name.startswith("run")):
                continue
            for filename in ("tinyml_dataset.jsonl", "tinyml_dataset.csv"):
                candidate = directory / filename
                if candidate.exists():
                    attempt_candidates.append(candidate)
    attempt_candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    if attempt_candidates:
        return attempt_candidates[0]

    candidates = [
        capture_root / "tinyml_dataset.jsonl",
        capture_root / "tinyml_dataset.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def summarize_dataset(samples: list[SampleRecord]) -> dict:
    label_counts = Counter(sample.label for sample in samples)
    condition_counts = Counter(sample.condition for sample in samples)
    sessions = Counter(sample.session_id for sample in samples)
    present = set(label_counts)
    missing = [label for label in EXPECTED_LABELS if label not in present]
    unexpected = sorted(present - set(EXPECTED_LABELS))
    return {
        "samples": len(samples),
        "labels": dict(sorted(label_counts.items(), key=lambda item: canonical_label_order([item[0]])[0] if item[0] in EXPECTED_LABELS else item[0])),
        "conditions": dict(sorted(condition_counts.items())),
        "session_count": len(sessions),
        "sessions": dict(sorted(sessions.items())),
        "missing_labels": missing,
        "unexpected_labels": unexpected,
    }


def count_by_label(samples: Iterable[SampleRecord]) -> Counter:
    return Counter(sample.label for sample in samples)


def make_split(samples: list[SampleRecord], val_fraction: float, seed: int) -> SplitResult:
    if len(samples) < 2:
        return SplitResult(train=list(samples), val=[], method="no_split", notes=["dataset too small for validation split"])

    group_to_samples: dict[str, list[SampleRecord]] = defaultdict(list)
    for sample in samples:
        group_to_samples[sample.session_id or sample.capture_uid].append(sample)

    total_by_label = count_by_label(samples)
    groups = list(group_to_samples.items())
    random.Random(seed).shuffle(groups)
    target_val_count = max(1, int(round(len(samples) * val_fraction)))
    remaining = Counter(total_by_label)
    val_groups: set[str] = set()
    val_count = 0
    notes: list[str] = []

    for group_id, group_samples in groups:
        if val_count >= target_val_count and val_groups:
            break
        group_counts = count_by_label(group_samples)
        violates_train_floor = False
        for label, group_count in group_counts.items():
            if total_by_label[label] <= 1:
                violates_train_floor = True
                break
            if remaining[label] - group_count < 1:
                violates_train_floor = True
                break
        if violates_train_floor:
            continue
        val_groups.add(group_id)
        val_count += len(group_samples)
        remaining.subtract(group_counts)

    train = [sample for sample in samples if sample.session_id not in val_groups]
    val = [sample for sample in samples if sample.session_id in val_groups]
    if val and train:
        notes.append(f"group split by session_id across {len(group_to_samples)} groups")
        return SplitResult(train=train, val=val, method="grouped_by_session", notes=notes)

    # Fallback: per-label sample split.
    by_label: dict[str, list[SampleRecord]] = defaultdict(list)
    for sample in samples:
        by_label[sample.label].append(sample)
    rng = random.Random(seed)
    train = []
    val = []
    for label, label_samples in by_label.items():
        shuffled = list(label_samples)
        rng.shuffle(shuffled)
        if len(shuffled) == 1:
            train.extend(shuffled)
            notes.append(f"label {label} has only one sample; kept in train")
            continue
        cut = max(1, int(round(len(shuffled) * val_fraction)))
        if cut >= len(shuffled):
            cut = len(shuffled) - 1
        val.extend(shuffled[:cut])
        train.extend(shuffled[cut:])
    if not val:
        val.append(train.pop())
        notes.append("forced one validation sample because stratified fallback produced none")
    return SplitResult(train=train, val=val, method="stratified_per_label", notes=notes)


def compute_scaler(samples: list[SampleRecord]) -> tuple[list[float], list[float]]:
    if not samples:
        raise TrainingError("cannot compute scaler without training samples")
    means = [0.0] * STROKE_FEATURE_DIM
    for sample in samples:
        for idx, value in enumerate(sample.features):
            means[idx] += value
    inv_n = 1.0 / float(len(samples))
    means = [value * inv_n for value in means]
    variances = [0.0] * STROKE_FEATURE_DIM
    for sample in samples:
        for idx, value in enumerate(sample.features):
            delta = value - means[idx]
            variances[idx] += delta * delta
    stds = []
    for variance in variances:
        std = math.sqrt(variance * inv_n)
        stds.append(std if std > 1e-6 else 1.0)
    return means, stds


def transform_features(features: list[float], condition: str, means: list[float], stds: list[float], condition_weight: float) -> list[float]:
    normalized = [(value - means[idx]) / stds[idx] for idx, value in enumerate(features)]
    normalized.extend(encode_condition(condition, condition_weight))
    return normalized


def squared_distance(a: list[float], b: list[float]) -> float:
    return sum((lhs - rhs) * (lhs - rhs) for lhs, rhs in zip(a, b))


def init_centroids(vectors: list[list[float]], k: int, seed: int) -> list[list[float]]:
    rng = random.Random(seed)
    if k >= len(vectors):
        return [list(vector) for vector in vectors]
    centroids = [list(rng.choice(vectors))]
    while len(centroids) < k:
        best_vector = None
        best_distance = -1.0
        for vector in vectors:
            distance = min(squared_distance(vector, centroid) for centroid in centroids)
            if distance > best_distance:
                best_distance = distance
                best_vector = vector
        centroids.append(list(best_vector))
    return centroids


def mean_vector(vectors: list[list[float]], dimension: int) -> list[float]:
    centroid = [0.0] * dimension
    if not vectors:
        return centroid
    inv_n = 1.0 / float(len(vectors))
    for vector in vectors:
        for idx, value in enumerate(vector):
            centroid[idx] += value
    return [value * inv_n for value in centroid]


def fit_label_prototypes(vectors: list[list[float]], k: int, iterations: int, seed: int) -> list[list[float]]:
    if not vectors:
        return []
    dimension = len(vectors[0])
    k = max(1, min(k, len(vectors)))
    centroids = init_centroids(vectors, k, seed)
    for _ in range(max(1, iterations)):
        assignments: list[list[list[float]]] = [[] for _ in range(k)]
        for vector in vectors:
            best_index = 0
            best_distance = squared_distance(vector, centroids[0])
            for idx in range(1, k):
                distance = squared_distance(vector, centroids[idx])
                if distance < best_distance:
                    best_distance = distance
                    best_index = idx
            assignments[best_index].append(vector)
        updated: list[list[float]] = []
        for idx, cluster in enumerate(assignments):
            if cluster:
                updated.append(mean_vector(cluster, dimension))
            else:
                updated.append(list(centroids[idx]))
        centroids = updated
    return centroids


def train_prototype_model(
    train_samples: list[SampleRecord],
    labels: list[str],
    means: list[float],
    stds: list[float],
    condition_weight: float,
    prototypes_per_label: int,
    iterations: int,
    seed: int,
) -> dict:
    by_label: dict[str, list[list[float]]] = defaultdict(list)
    for sample in train_samples:
        by_label[sample.label].append(transform_features(sample.features, sample.condition, means, stds, condition_weight))

    prototypes: dict[str, list[list[float]]] = {}
    for label in labels:
        vectors = by_label.get(label, [])
        if not vectors:
            continue
        prototypes[label] = fit_label_prototypes(
            vectors=vectors,
            k=prototypes_per_label,
            iterations=iterations,
            seed=seed + labels.index(label),
        )
    return {
        "backend": "prototype_v1",
        "feature_format": FEATURE_FORMAT,
        "stroke_feature_dim": STROKE_FEATURE_DIM,
        "stroke_sample_count": STROKE_SAMPLE_COUNT,
        "condition_weight": condition_weight,
        "means": means,
        "stds": stds,
        "labels": labels,
        "conditions": CONDITIONS,
        "labels_by_condition": LABELS_BY_CONDITION,
        "prototypes": prototypes,
    }


def predict_prototype(model: dict, sample: SampleRecord, allowed_labels: Optional[list[str]] = None) -> tuple[str, float, float]:
    vector = transform_features(
        sample.features,
        sample.condition,
        model["means"],
        model["stds"],
        float(model["condition_weight"]),
    )
    if allowed_labels is None:
        allowed_labels = model["labels"]
    best_label = ""
    best_distance = math.inf
    second_distance = math.inf
    for label in allowed_labels:
        for centroid in model["prototypes"].get(label, []):
            distance = squared_distance(vector, centroid)
            if distance < best_distance:
                second_distance = best_distance
                best_distance = distance
                best_label = label
            elif distance < second_distance:
                second_distance = distance
    if not best_label:
        raise TrainingError("prototype model has no centroids for prediction")
    margin = second_distance - best_distance if second_distance < math.inf else best_distance
    confidence = 1.0 / (1.0 + best_distance)
    return best_label, confidence, margin


def fit_tensorflow_model(
    train_samples: list[SampleRecord],
    val_samples: list[SampleRecord],
    labels: list[str],
    means: list[float],
    stds: list[float],
    condition_weight: float,
    output_dir: Path,
    epochs: int,
    seed: int,
) -> dict:
    try:
        import tensorflow as tf  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on environment
        raise TrainingError(f"TensorFlow unavailable: {exc}") from exc

    tf.random.set_seed(seed)
    label_to_index = {label: idx for idx, label in enumerate(labels)}

    def build_matrix(samples: list[SampleRecord]) -> tuple[list[list[float]], list[int]]:
        xs = [transform_features(sample.features, sample.condition, means, stds, condition_weight) for sample in samples]
        ys = [label_to_index[sample.label] for sample in samples]
        return xs, ys

    train_x, train_y = build_matrix(train_samples)
    val_x, val_y = build_matrix(val_samples)
    input_dim = len(train_x[0])

    model = tf.keras.Sequential(
        [
            tf.keras.layers.Input(shape=(input_dim,)),
            tf.keras.layers.Dense(96, activation="relu"),
            tf.keras.layers.Dense(48, activation="relu"),
            tf.keras.layers.Dense(len(labels), activation="softmax"),
        ]
    )
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )
    callbacks = [
        tf.keras.callbacks.EarlyStopping(monitor="val_accuracy", patience=12, restore_best_weights=True),
    ]
    model.fit(
        train_x,
        train_y,
        validation_data=(val_x, val_y) if val_x else None,
        epochs=epochs,
        batch_size=min(32, max(4, len(train_x))),
        verbose=0,
        callbacks=callbacks,
    )

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_bytes = converter.convert()
    (output_dir / "model.tflite").write_bytes(tflite_bytes)

    def predict(sample: SampleRecord) -> tuple[str, float, float]:
        logits = model.predict(
            [transform_features(sample.features, sample.condition, means, stds, condition_weight)],
            verbose=0,
        )[0]
        allowed_labels = LABELS_BY_CONDITION.get(sample.condition, labels)
        allowed_indices = [label_to_index[label] for label in allowed_labels if label in label_to_index]
        best_index = max(allowed_indices, key=lambda idx: float(logits[idx]))
        sorted_allowed = sorted((float(logits[idx]), idx) for idx in allowed_indices)
        best_prob = float(logits[best_index])
        second_prob = sorted_allowed[-2][0] if len(sorted_allowed) > 1 else 0.0
        return labels[best_index], best_prob, best_prob - second_prob

    return {
        "backend": "tensorflow_keras_tflite",
        "feature_format": FEATURE_FORMAT,
        "stroke_feature_dim": STROKE_FEATURE_DIM,
        "stroke_sample_count": STROKE_SAMPLE_COUNT,
        "condition_weight": condition_weight,
        "means": means,
        "stds": stds,
        "labels": labels,
        "conditions": CONDITIONS,
        "labels_by_condition": LABELS_BY_CONDITION,
        "predict": predict,
    }


def evaluate_model(model: dict, samples: list[SampleRecord], backend: str) -> dict:
    total = len(samples)
    correct = 0
    by_condition: dict[str, Counter] = defaultdict(Counter)
    by_label: dict[str, Counter] = defaultdict(Counter)
    confusion: dict[str, Counter] = defaultdict(Counter)
    detailed: list[dict] = []

    for sample in samples:
        allowed_labels = LABELS_BY_CONDITION.get(sample.condition, model["labels"])
        if backend == "prototype_v1":
            predicted, confidence, margin = predict_prototype(model, sample, allowed_labels=allowed_labels)
        else:
            predicted, confidence, margin = model["predict"](sample)
        is_correct = predicted == sample.label
        correct += int(is_correct)
        by_condition[sample.condition]["samples"] += 1
        by_condition[sample.condition]["correct"] += int(is_correct)
        by_label[sample.label]["samples"] += 1
        by_label[sample.label]["correct"] += int(is_correct)
        confusion[sample.label][predicted] += 1
        detailed.append(
            {
                "capture_uid": sample.capture_uid,
                "label": sample.label,
                "condition": sample.condition,
                "predicted": predicted,
                "correct": is_correct,
                "confidence": round(confidence, 6),
                "margin": round(margin, 6),
                "session_id": sample.session_id,
            }
        )

    accuracy = (correct / total) if total else 0.0
    return {
        "samples": total,
        "correct": correct,
        "accuracy": accuracy,
        "by_condition": {
            condition: {
                "samples": metrics["samples"],
                "correct": metrics["correct"],
                "accuracy": (metrics["correct"] / metrics["samples"]) if metrics["samples"] else 0.0,
            }
            for condition, metrics in sorted(by_condition.items())
        },
        "by_label": {
            label: {
                "samples": metrics["samples"],
                "correct": metrics["correct"],
                "accuracy": (metrics["correct"] / metrics["samples"]) if metrics["samples"] else 0.0,
            }
            for label, metrics in sorted(by_label.items(), key=lambda item: canonical_label_order([item[0]])[0] if item[0] in EXPECTED_LABELS else item[0])
        },
        "confusion": {
            label: dict(sorted(row.items(), key=lambda item: (-item[1], item[0])))
            for label, row in sorted(confusion.items(), key=lambda item: canonical_label_order([item[0]])[0] if item[0] in EXPECTED_LABELS else item[0])
        },
        "predictions": detailed,
    }


def top_confusions(metrics: dict, limit: int = 10) -> list[tuple[str, str, int]]:
    items: list[tuple[str, str, int]] = []
    for actual, row in metrics.get("confusion", {}).items():
        for predicted, count in row.items():
            if actual == predicted:
                continue
            items.append((actual, predicted, count))
    items.sort(key=lambda item: (-item[2], item[0], item[1]))
    return items[:limit]


def format_metrics_report(
    dataset_summary: dict,
    split: SplitResult,
    train_metrics: dict,
    val_metrics: dict,
    backend: str,
    notes: list[str],
) -> str:
    lines = []
    lines.append(f"Backend: {backend}")
    lines.append(f"Dataset samples: {dataset_summary['samples']}")
    lines.append(f"Session groups: {dataset_summary['session_count']}")
    lines.append(f"Split method: {split.method}")
    for note in split.notes:
        lines.append(f"Split note: {note}")
    for note in notes:
        lines.append(f"Training note: {note}")
    if dataset_summary["missing_labels"]:
        lines.append(f"Missing labels in dataset: {' '.join(dataset_summary['missing_labels'])}")
    if dataset_summary["unexpected_labels"]:
        lines.append(f"Unexpected labels in dataset: {' '.join(dataset_summary['unexpected_labels'])}")
    lines.append("")
    lines.append("Condition counts:")
    for condition, count in dataset_summary["conditions"].items():
        lines.append(f"  {condition:8s} {count}")
    lines.append("")
    lines.append("Train metrics:")
    lines.append(f"  samples={train_metrics['samples']} accuracy={train_metrics['accuracy']:.3f}")
    for condition, metrics in train_metrics["by_condition"].items():
        lines.append(
            f"  {condition:8s} samples={metrics['samples']:4d} accuracy={metrics['accuracy']:.3f}"
        )
    lines.append("")
    lines.append("Validation metrics:")
    lines.append(f"  samples={val_metrics['samples']} accuracy={val_metrics['accuracy']:.3f}")
    for condition, metrics in val_metrics["by_condition"].items():
        lines.append(
            f"  {condition:8s} samples={metrics['samples']:4d} accuracy={metrics['accuracy']:.3f}"
        )
    lines.append("")
    lines.append("Per-label validation accuracy:")
    for label in canonical_label_order(val_metrics["by_label"].keys()):
        metrics = val_metrics["by_label"][label]
        lines.append(
            f"  {label:>5s} samples={metrics['samples']:4d} accuracy={metrics['accuracy']:.3f}"
        )
    confusions = top_confusions(val_metrics)
    lines.append("")
    lines.append("Top validation confusions:")
    if confusions:
        for actual, predicted, count in confusions:
            lines.append(f"  {actual:>5s} -> {predicted:<5s} {count}")
    else:
        lines.append("  none")
    return "\n".join(lines) + "\n"


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def cpp_char_literal(symbol: str) -> str:
    if len(symbol) != 1:
        raise TrainingError(f"expected single-character symbol, got {symbol!r}")
    if symbol == "\\":
        return "'\\\\'"
    if symbol == "'":
        return "'\\''"
    if symbol == "\n":
        return "'\\n'"
    if symbol == "\r":
        return "'\\r'"
    if symbol == "\b":
        return "'\\b'"
    return f"'{symbol}'"


def export_prototype_cpp(model: dict, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    cpp_path = output_dir / "GraffitiPrototypeModel.generated.cpp"
    labels = model["labels"]
    prototypes = model["prototypes"]
    feature_dim = STROKE_FEATURE_DIM + len(CONDITIONS)

    lines: list[str] = []
    lines.append('#include "GraffitiPrototypeModel.h"')
    lines.append("")
    lines.append("namespace GraffitiPrototypeModelData {")
    lines.append("")
    lines.append(f"const float kConditionWeight = {float(model['condition_weight']):.8f}f;")
    lines.append("")

    def emit_float_array(name: str, values: list[float]) -> None:
        lines.append(f"const float {name}[{len(values)}] = {{")
        for start in range(0, len(values), 8):
            chunk = ", ".join(f"{float(value):.8f}f" for value in values[start:start + 8])
            suffix = "," if (start + 8) < len(values) else ""
            lines.append(f"  {chunk}{suffix}")
        lines.append("};")
        lines.append("")

    emit_float_array("kMeans", model["means"])
    emit_float_array("kStds", model["stds"])

    prototype_total = sum(len(prototypes.get(label, [])) for label in labels)
    lines.append(f"const uint16_t kPrototypeCount = {prototype_total};")
    lines.append("")

    vector_counter = 0
    for label in labels:
        for centroid in prototypes.get(label, []):
            emit_float_array(f"kPrototypeVector{vector_counter:03d}", centroid)
            vector_counter += 1

    lines.append("const PrototypeEntry kPrototypes[] = {")
    vector_counter = 0
    for label in labels:
        condition_index = CONDITIONS.index(LABEL_TO_CONDITION[label])
        symbol = (
            "\x0f"
            if label == "SHIFT"
            else (
            "\x1b"
            if label == "ESC"
            else ("\n" if label == "RET" else ("\b" if label == "BKSP" else (" " if label == "SPACE" else label)))
            )
        )
        symbol_literal = cpp_char_literal(symbol)
        for _centroid in prototypes.get(label, []):
            lines.append(
                f'  {{{symbol_literal}, {condition_index}, "{label}", kPrototypeVector{vector_counter:03d}}},'
            )
            vector_counter += 1
    lines.append("};")
    lines.append("")
    lines.append(f"static_assert(kInputFeatureDim == {feature_dim}, \"feature dimension mismatch\");")
    lines.append("")
    lines.append("}  // namespace GraffitiPrototypeModelData")
    lines.append("")
    cpp_path.write_text("\n".join(lines), encoding="utf-8")


def select_backend(name: str) -> str:
    if name != "auto":
        return name
    try:
        import tensorflow  # type: ignore  # noqa: F401
    except Exception:
        return "prototype"
    return "tensorflow"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train and export a conditioned graffiti classifier from tinyml_dataset.jsonl or .csv"
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="Dataset path. Defaults to debug/graffiti_capture/tinyml_dataset.jsonl or .csv",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("debug/graffiti_model/latest"),
        help="Directory for exported artifacts",
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "tensorflow", "prototype"),
        default="auto",
        help="Training backend. auto prefers TensorFlow if installed, otherwise prototype fallback.",
    )
    parser.add_argument("--val-fraction", type=float, default=0.2, help="Validation fraction, default 0.2")
    parser.add_argument("--seed", type=int, default=1337, help="Random seed")
    parser.add_argument(
        "--condition-weight",
        type=float,
        default=3.0,
        help="Weight applied to the one-hot mode condition before classification",
    )
    parser.add_argument(
        "--prototypes-per-label",
        type=int,
        default=2,
        help="Prototype fallback only: centroids per label",
    )
    parser.add_argument(
        "--prototype-iterations",
        type=int,
        default=12,
        help="Prototype fallback only: k-means refinement iterations",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=120,
        help="TensorFlow only: max training epochs",
    )
    parser.add_argument(
        "--firmware-export-dir",
        type=Path,
        default=Path("examples/Ringo_BLE_Trackpad"),
        help="Prototype backend only: export GraffitiPrototypeModel.generated.cpp into this directory",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    dataset_path = args.input or auto_dataset_path(repo_root)
    if dataset_path is None:
        print("No dataset found. Expected debug/graffiti_capture/tinyml_dataset.jsonl or .csv", file=sys.stderr)
        return 1

    try:
        samples = load_dataset(dataset_path)
    except TrainingError as exc:
        print(f"dataset error: {exc}", file=sys.stderr)
        return 1

    dataset_summary = summarize_dataset(samples)
    split = make_split(samples, val_fraction=args.val_fraction, seed=args.seed)
    labels = canonical_label_order({sample.label for sample in samples})
    means, stds = compute_scaler(split.train)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    requested_backend = select_backend(args.backend)
    notes: list[str] = []
    if requested_backend == "tensorflow":
        try:
            model = fit_tensorflow_model(
                train_samples=split.train,
                val_samples=split.val,
                labels=labels,
                means=means,
                stds=stds,
                condition_weight=args.condition_weight,
                output_dir=output_dir,
                epochs=args.epochs,
                seed=args.seed,
            )
            backend_name = "tensorflow_keras_tflite"
        except TrainingError as exc:
            if args.backend == "tensorflow":
                print(f"training error: {exc}", file=sys.stderr)
                return 1
            notes.append(f"TensorFlow path unavailable, fell back to prototype backend: {exc}")
            model = train_prototype_model(
                train_samples=split.train,
                labels=labels,
                means=means,
                stds=stds,
                condition_weight=args.condition_weight,
                prototypes_per_label=args.prototypes_per_label,
                iterations=args.prototype_iterations,
                seed=args.seed,
            )
            backend_name = "prototype_v1"
    else:
        notes.append(
            "TensorFlow/Numpy stack not available locally; exported prototype baseline instead of Keras/TFLite"
        )
        model = train_prototype_model(
            train_samples=split.train,
            labels=labels,
            means=means,
            stds=stds,
            condition_weight=args.condition_weight,
            prototypes_per_label=args.prototypes_per_label,
            iterations=args.prototype_iterations,
            seed=args.seed,
        )
        backend_name = "prototype_v1"

    train_metrics = evaluate_model(model, split.train, backend_name)
    val_metrics = evaluate_model(model, split.val, backend_name) if split.val else {
        "samples": 0,
        "correct": 0,
        "accuracy": 0.0,
        "by_condition": {},
        "by_label": {},
        "confusion": {},
        "predictions": [],
    }

    metadata = {
        "backend": backend_name,
        "dataset_path": str(dataset_path.resolve()),
        "feature_format": FEATURE_FORMAT,
        "stroke_sample_count": STROKE_SAMPLE_COUNT,
        "stroke_feature_dim": STROKE_FEATURE_DIM,
        "condition_weight": args.condition_weight,
        "labels": labels,
        "conditions": CONDITIONS,
        "labels_by_condition": LABELS_BY_CONDITION,
        "dataset_summary": dataset_summary,
        "split": {
            "method": split.method,
            "notes": split.notes,
            "train_samples": len(split.train),
            "val_samples": len(split.val),
        },
        "notes": notes,
    }
    if backend_name == "prototype_v1":
        write_json(output_dir / "model.json", model)
        export_prototype_cpp(model, args.firmware_export_dir)
    write_json(output_dir / "metadata.json", metadata)
    write_json(output_dir / "label_map.json", {"labels": labels})
    write_json(output_dir / "condition_map.json", {"conditions": CONDITIONS, "labels_by_condition": LABELS_BY_CONDITION})
    write_json(output_dir / "train_metrics.json", train_metrics)
    write_json(output_dir / "val_metrics.json", val_metrics)

    report = format_metrics_report(
        dataset_summary=dataset_summary,
        split=split,
        train_metrics=train_metrics,
        val_metrics=val_metrics,
        backend=backend_name,
        notes=notes,
    )
    (output_dir / "validation_report.txt").write_text(report, encoding="utf-8")

    print(report)
    print(f"Artifacts written to {output_dir.resolve()}")
    if backend_name == "prototype_v1":
        print(
            textwrap.dedent(
                """
                Note: TensorFlow/Keras is not installed in this environment, so the export is a conditioned
                nearest-prototype baseline (`model.json`) rather than a TFLite model. The dataset schema and
                metadata are preserved so this can be retrained as Keras/TFLite later without changing capture.
                """
            ).strip()
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
