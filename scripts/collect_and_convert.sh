#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TRAIN_COUNT="${TRAIN_COUNT:-10}"
TEST_COUNT="${TEST_COUNT:-3}"
TRAIN_SEED="${TRAIN_SEED:-10000}"
TEST_SEED="${TEST_SEED:-20000}"

if [[ -f ../.venv/bin/activate ]]; then
  source ../.venv/bin/activate
else
  echo "warning: ../.venv not found; using the current Python environment" >&2
fi

if ! ffmpeg -hide_banner -encoders 2>/dev/null | grep -q libaom-av1; then
  echo "error: ffmpeg with libaom-av1 is required for USIM-compatible AV1 videos" >&2
  exit 1
fi

echo "== collecting train episodes =="
python scripts/collect.py \
  --split train --count "$TRAIN_COUNT" --start-index 0 \
  --raw-root episodes_raw/train --base-seed "$TRAIN_SEED" --attempts 3

echo "== collecting test episodes =="
python scripts/collect.py \
  --split test --count "$TEST_COUNT" --start-index 0 \
  --raw-root episodes_raw/test --base-seed "$TEST_SEED" --attempts 3

echo "== converting train split =="
python scripts/convert_to_lerobot.py \
  --raw episodes_raw/train --out datasets/usim/train --split train
python scripts/validate_lerobot.py --dataset datasets/usim/train

echo "== converting test split =="
python scripts/convert_to_lerobot.py \
  --raw episodes_raw/test --out datasets/usim/test --split test
python scripts/validate_lerobot.py --dataset datasets/usim/test

python scripts/make_dataset_card.py

echo
echo "Dataset ready:"
echo "  $ROOT/datasets/usim/train"
echo "  $ROOT/datasets/usim/test"
echo "Set DATA_BASE_DIR=$ROOT/datasets in the u0-openpi environment."
