#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ELOFTR_PYTHON="${ELOFTR_PYTHON:-python3}"
ELOFTR_WEIGHTS="${ELOFTR_WEIGHTS:-$ROOT/weights/eloftr_outdoor.ckpt}"
ELOFTR_DEVICE="${ELOFTR_DEVICE:-cuda}"
ZMQ_ENDPOINT="${ZMQ_ENDPOINT:-tcp://127.0.0.1:5555}"

usage() {
  echo "Usage: $0 <bin> <vocab> <orb_cfg> <gauss_cfg> <dataset_path> <result_dir> [extra_args...]" >&2
  echo "Example:" >&2
  echo "  $0 ./bin/replica_rgbd ./ORB-SLAM3/Vocabulary/ORBvoc.txt \\" >&2
  echo "     ./cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \\" >&2
  echo "     ./cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd_corrinit.yaml \\" >&2
  echo "     /path/to/Replica/office0 /path/to/results" >&2
}

if [ $# -lt 6 ]; then
  usage
  exit 1
fi

BIN="$1"; shift

if [ ! -x "$BIN" ]; then
  echo "Error: binary not executable: $BIN" >&2
  exit 1
fi

if [ ! -f "$ELOFTR_WEIGHTS" ]; then
  echo "Error: LoFTR weights not found: $ELOFTR_WEIGHTS" >&2
  echo "Set ELOFTR_WEIGHTS=/absolute/path/to/eloftr_outdoor.ckpt" >&2
  exit 1
fi

"$ELOFTR_PYTHON" "$ROOT/scripts/efficientloftr_service.py" \
  --endpoint "$ZMQ_ENDPOINT" \
  --weights "$ELOFTR_WEIGHTS" \
  --device "$ELOFTR_DEVICE" &

LOFTR_PID=$!
cleanup() {
  kill "$LOFTR_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 1

"$BIN" "$@"
