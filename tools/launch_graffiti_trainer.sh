#!/bin/zsh
set -euo pipefail

SCRIPT_DIR=${0:A:h}
PROJECT_ROOT=${SCRIPT_DIR:h}

typeset -a candidates
if [[ -n "${PYTHON_BIN:-}" ]]; then
  candidates+=("${PYTHON_BIN}")
fi
candidates+=(
  "/opt/homebrew/opt/python@3.14/bin/python3.14"
  "python3"
)

for py in "${candidates[@]}"; do
  if ! command -v "${py}" >/dev/null 2>&1; then
    continue
  fi
  exec "${py}" "${PROJECT_ROOT}/tools/train_graffiti_model.py" "$@"
done

echo "No usable Python runtime found for graffiti trainer." >&2
exit 1
