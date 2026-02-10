#!/usr/bin/env bash
set -euo pipefail

# Runs three TUM RGB-D scenes with CorrInit (LoFTR) enabled.
# Required: build Photo-SLAM with -DWITH_CORRINIT_ZMQ=ON

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

TUM_ROOT="${TUM_ROOT:-/home/crl/Congthai/datasets/TUM}"
RESULTS_ROOT="${RESULTS_ROOT:-$ROOT/results/tum_rgbd_corrinit}"
ELOFTR_PYTHON="${ELOFTR_PYTHON:-python3}"
ELOFTR_WEIGHTS="${ELOFTR_WEIGHTS:-/home/crl/Congthai/photoslam_edgs/weights/eloftr_outdoor.ckpt}"
ELOFTR_DEVICE="${ELOFTR_DEVICE:-cuda}"
ZMQ_ENDPOINT="${ZMQ_ENDPOINT:-tcp://127.0.0.1:5555}"

usage() {
  echo "Usage: TUM_ROOT=/path/to/TUM $0" >&2
  echo "Optional envs:" >&2
  echo "  RESULTS_ROOT=/path/to/save/results" >&2
  echo "  ELOFTR_PYTHON=python3" >&2
  echo "  ELOFTR_WEIGHTS=/abs/path/eloftr_outdoor.ckpt" >&2
  echo "  ELOFTR_DEVICE=cuda|cpu" >&2
  echo "  ZMQ_ENDPOINT=tcp://127.0.0.1:5555" >&2
}

# TUM_ROOT is set to a default absolute path; keep usage for overrides.

if [[ ! -x "$ROOT/bin/tum_rgbd" ]]; then
  echo "Error: $ROOT/bin/tum_rgbd not found or not executable." >&2
  echo "Build with -DWITH_CORRINIT_ZMQ=ON first." >&2
  exit 1
fi

if [[ ! -f "$ELOFTR_WEIGHTS" ]]; then
  echo "Error: LoFTR weights not found: $ELOFTR_WEIGHTS" >&2
  exit 1
fi

mkdir -p "$RESULTS_ROOT"

echo "[CorrInit] Starting LoFTR sidecar..."
"$ELOFTR_PYTHON" "$ROOT/scripts/efficientloftr_service.py" \
  --endpoint "$ZMQ_ENDPOINT" \
  --weights "$ELOFTR_WEIGHTS" \
  --device "$ELOFTR_DEVICE" &

LOFTR_PID=$!
cleanup() { kill "$LOFTR_PID" 2>/dev/null || true; }
trap cleanup EXIT

sleep 1

run_scene() {
  local scene_name="$1"
  local orb_cfg="$2"
  local assoc_file="$3"

  echo "[CorrInit] Running scene: $scene_name"
  "$ROOT/bin/tum_rgbd" \
    "$ROOT/ORB-SLAM3/Vocabulary/ORBvoc.txt" \
    "$orb_cfg" \
    "$ROOT/cfg/gaussian_mapper/RGB-D/TUM/tum_rgbd_corrinit.yaml" \
    "$TUM_ROOT/$scene_name" \
    "$assoc_file" \
    "$RESULTS_ROOT/$scene_name" \
    no_viewer
}

run_scene "rgbd_dataset_freiburg1_desk" \
  "$ROOT/cfg/ORB_SLAM3/RGB-D/TUM/tum_freiburg1_desk.yaml" \
  "$ROOT/cfg/ORB_SLAM3/RGB-D/TUM/associations/tum_freiburg1_desk.txt"

run_scene "rgbd_dataset_freiburg2_xyz" \
  "$ROOT/cfg/ORB_SLAM3/RGB-D/TUM/tum_freiburg2_xyz.yaml" \
  "$ROOT/cfg/ORB_SLAM3/RGB-D/TUM/associations/tum_freiburg2_xyz.txt"

run_scene "rgbd_dataset_freiburg3_long_office_household" \
  "$ROOT/cfg/ORB_SLAM3/RGB-D/TUM/tum_freiburg3_long_office_household.yaml" \
  "$ROOT/cfg/ORB_SLAM3/RGB-D/TUM/associations/tum_freiburg3_long_office_household.txt"

echo "[CorrInit] Done."
