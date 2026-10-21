#!/usr/bin/env bash
# Example π0.5 fine-tuning commands for the collected UUV charging dataset.
# Run this inside a configured u0-openpi environment (see openpi_ext/README.md).
set -euo pipefail

DATA_BASE_DIR="${DATA_BASE_DIR:-/media/ai/DATA1/dcw/stonefish/uuv_charge_nav/datasets}"
export DATA_BASE_DIR

echo "DATA_BASE_DIR=$DATA_BASE_DIR"
echo "train: $DATA_BASE_DIR/usim/train"
echo "test:  $DATA_BASE_DIR/usim/test"

# One-time normalization statistics.
python scripts/compute_norm_stats.py \
  --config_name pi05_u0bot \
  --repo_id "$DATA_BASE_DIR/usim/train"

# Fine-tune π0.5 (u0-openpi baseline config).
python scripts/train.py pi05_u0bot \
  --exp-name=uuv_charge_nav_pi05 \
  --overwrite
