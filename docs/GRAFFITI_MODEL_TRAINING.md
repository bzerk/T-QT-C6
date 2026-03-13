# Ringo Graffiti Model Training

Host-side trainer for the conditioned single-model graffiti plan.

## Inputs
The trainer consumes either of these capture exports:

- `debug/graffiti_capture/tinyml_dataset.jsonl`
- `debug/graffiti_capture/tinyml_dataset.csv`

Expected label space:

- lowercase letters: `a-z`
- punctuation/control: `SPACE BKSP . , ( ) - _ # * ? '`
- numerics: `0-9`

Expected condition field:

- `letters`
- `punct`
- `numeric`

Uppercase is not part of the classifier label space. It remains a runtime modifier.

## Run
Automatic dataset discovery:

```bash
python3 tools/train_graffiti_model.py
```

Explicit dataset and output directory:

```bash
python3 tools/train_graffiti_model.py \
  --input debug/graffiti_capture/tinyml_dataset.jsonl \
  --output-dir debug/graffiti_model/latest
```

Force the fallback backend explicitly:

```bash
python3 tools/train_graffiti_model.py --backend prototype
```

If TensorFlow is installed locally, `--backend auto` prefers a small Keras model and exports `model.tflite`.
If TensorFlow is not installed, the trainer exports a conditioned nearest-prototype baseline instead.

## Outputs
Artifacts are written to the output directory:

- `metadata.json`: dataset summary, split details, feature schema, training notes
- `label_map.json`: trained label order
- `condition_map.json`: condition order and allowed labels per condition
- `train_metrics.json`: train-set metrics and per-sample predictions
- `val_metrics.json`: validation metrics and per-sample predictions
- `validation_report.txt`: readable summary with per-condition and per-label accuracy
- `model.json`: fallback prototype model export
- `model.tflite`: only when TensorFlow/Keras is available

## Current environment note
On this machine, the default `python3` environment does not currently include TensorFlow, NumPy, pandas, or scikit-learn.
The trainer still works, but it will export the prototype fallback until that stack is installed.
