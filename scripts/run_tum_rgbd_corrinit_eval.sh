#!/usr/bin/env bash
set -euo pipefail

# End-to-end runner: TUM RGB-D CorrInit + Photo-SLAM-eval metrics

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Paths (override via env vars if needed)
TUM_ROOT="${TUM_ROOT:-/home/crl/Congthai/datasets/TUM}"
DATASET_CENTER="${DATASET_CENTER:-/home/crl/Congthai/datasets}"
RESULTS_ROOT="${RESULTS_ROOT:-$ROOT/results/tum_rgbd_corrinit_eval}"
# Use an isolated results_main folder for evaluation to avoid picking up unrelated runs
RESULTS_MAIN="${RESULTS_MAIN:-$RESULTS_ROOT/_eval_main}"
EVAL_ROOT="${EVAL_ROOT:-$ROOT/third_party/Photo-SLAM-eval}"

ELOFTR_PYTHON="${ELOFTR_PYTHON:-python3}"
ELOFTR_WEIGHTS="${ELOFTR_WEIGHTS:-/home/crl/Congthai/photoslam_edgs/weights/eloftr_outdoor.ckpt}"
ELOFTR_DEVICE="${ELOFTR_DEVICE:-cuda}"
ZMQ_ENDPOINT="${ZMQ_ENDPOINT:-tcp://127.0.0.1:5555}"

GAUSSIAN_CFG="${GAUSSIAN_CFG:-$ROOT/cfg/gaussian_mapper/RGB-D/TUM/tum_rgbd_corrinit_eval.yaml}"

usage() {
  echo "Usage: $0" >&2
  echo "Optional envs:" >&2
  echo "  TUM_ROOT=/path/to/TUM" >&2
  echo "  DATASET_CENTER=/path/to/datasets_root" >&2
  echo "  RESULTS_ROOT=/path/to/results/tum_rgbd_corrinit_eval" >&2
  echo "  RESULTS_MAIN=/path/to/results" >&2
  echo "  EVAL_ROOT=/path/to/Photo-SLAM-eval" >&2
  echo "  GAUSSIAN_CFG=/path/to/tum_rgbd_corrinit_eval.yaml" >&2
  echo "  ELOFTR_PYTHON=python3" >&2
  echo "  ELOFTR_WEIGHTS=/abs/path/eloftr_outdoor.ckpt" >&2
  echo "  ELOFTR_DEVICE=cuda|cpu" >&2
  echo "  ZMQ_ENDPOINT=tcp://127.0.0.1:5555" >&2
}

if [[ ! -x "$ROOT/bin/tum_rgbd" ]]; then
  echo "Error: $ROOT/bin/tum_rgbd not found or not executable." >&2
  echo "Build Photo-SLAM first." >&2
  exit 1
fi

if [[ ! -f "$ELOFTR_WEIGHTS" ]]; then
  echo "Error: LoFTR weights not found: $ELOFTR_WEIGHTS" >&2
  exit 1
fi

if [[ ! -d "$EVAL_ROOT" || ! -f "$EVAL_ROOT/onekey.py" ]]; then
  echo "Error: Photo-SLAM-eval not found at: $EVAL_ROOT" >&2
  echo "Set EVAL_ROOT to your cloned Photo-SLAM-eval path." >&2
  exit 1
fi

if [[ ! -f "$GAUSSIAN_CFG" ]]; then
  echo "Error: Gaussian config not found: $GAUSSIAN_CFG" >&2
  exit 1
fi

mkdir -p "$RESULTS_ROOT" "$RESULTS_MAIN"

# Copy camera.yaml for TUM (required by evaluation)
if [[ -f "$EVAL_ROOT/TUM/fr1/camera.yaml" ]]; then
  cp -f "$EVAL_ROOT/TUM/fr1/camera.yaml" "$TUM_ROOT/rgbd_dataset_freiburg1_desk" || true
fi
if [[ -f "$EVAL_ROOT/TUM/fr2/camera.yaml" ]]; then
  cp -f "$EVAL_ROOT/TUM/fr2/camera.yaml" "$TUM_ROOT/rgbd_dataset_freiburg2_xyz" || true
fi
if [[ -f "$EVAL_ROOT/TUM/fr3/camera.yaml" ]]; then
  cp -f "$EVAL_ROOT/TUM/fr3/camera.yaml" "$TUM_ROOT/rgbd_dataset_freiburg3_long_office_household" || true
fi

RUN_LOG="$RESULTS_ROOT/run_corrinit_eval.log"
LOFTR_LOG="$RESULTS_ROOT/loftr_service.log"
EVAL_LOG="$RESULTS_ROOT/eval_stdout.txt"

echo "[CorrInit] Starting LoFTR sidecar..."
"$ELOFTR_PYTHON" "$ROOT/scripts/efficientloftr_service.py" \
  --endpoint "$ZMQ_ENDPOINT" \
  --weights "$ELOFTR_WEIGHTS" \
  --device "$ELOFTR_DEVICE" >"$LOFTR_LOG" 2>&1 &

LOFTR_PID=$!
cleanup() { kill "$LOFTR_PID" 2>/dev/null || true; }
trap cleanup EXIT

sleep 1

run_scene() {
  local scene_name="$1"
  local orb_cfg="$2"
  local assoc_file="$3"

  echo "[CorrInit] Running scene: $scene_name" | tee -a "$RUN_LOG"
  "$ROOT/bin/tum_rgbd" \
    "$ROOT/ORB-SLAM3/Vocabulary/ORBvoc.txt" \
    "$orb_cfg" \
    "$GAUSSIAN_CFG" \
    "$TUM_ROOT/$scene_name" \
    "$assoc_file" \
    "$RESULTS_ROOT/$scene_name" \
    no_viewer 2>&1 | tee -a "$RUN_LOG"
}

run_scene "rgbd_dataset_freiburg1_desk" \
  "$ROOT/cfg/ORB_SLAM3/RGB-D/TUM/tum_freiburg1_desk.yaml" \
  "$ROOT/cfg/ORB_SLAM3/RGB-D/TUM/associations/tum_freiburg1_desk.txt"

# Create expected results alias for Photo-SLAM-eval
ln -sfn "$RESULTS_ROOT" "$RESULTS_MAIN/tum_rgbd_0"

echo "[Eval] Running Photo-SLAM-eval..."
(cd "$EVAL_ROOT" && "$ELOFTR_PYTHON" "onekey.py" \
  --dataset_center_path "$DATASET_CENTER" \
  --result_main_folder "$RESULTS_MAIN") 2>&1 | tee "$EVAL_LOG"

echo "[Eval] Done."
echo "Run log: $RUN_LOG"
echo "LoFTR log: $LOFTR_LOG"
echo "Eval log: $EVAL_LOG"
echo "Metrics: $RESULTS_MAIN/log.txt and $RESULTS_MAIN/log.csv"
