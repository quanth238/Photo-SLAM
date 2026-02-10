# CorrInit (LoFTR) Implementation Notes

**Scope**
This document records exactly what was implemented for CorrInit (triangulation-based densification), which theory it follows, and the assumptions/guards enforced in the Photo‑SLAM pipeline.

**Design Goals**
1. Match the Photo‑SLAM geometry pipeline (pinhole, pixel‑space, OpenCV reprojection).
2. Use LoFTR pairwise correspondences without inventing EDGS‑specific warps.
3. Keep the system deterministic, debuggable, and safe for online SLAM.

**Source Theory References**
1. EDGS initialization logic in `reference/EDGS-main/source/corr_init.py`:
   - Triangulate correspondences.
   - Compute reprojection errors in reference and neighbor.
   - Select the best per keypoint with `min over neighbors of max reprojection error`.
2. EfficientLoFTR usage pattern (pairwise matching with `mkpts0_f`, `mkpts1_f`, `mconf`).
3. Photo‑SLAM/ORB‑SLAM3 geometry conventions:
   - Pose `Tcw` (world → camera).
   - Pinhole projection in pixel‑space.

**Key Implementation Decision**
1. EDGS uses dense warp fields and certainty maps to sample shared ref points across neighbors.
2. LoFTR returns sparse, pairwise correspondences with no shared ref‑pixel set across neighbors.
3. Therefore, the **correct LoFTR adaptation** is:
   - Process each neighbor independently.
   - Triangulate and gate.
   - Union valid candidates.
   - Select top‑N by reprojection error.
4. This avoids inventing a shared ref‑pixel set that LoFTR does not provide.

**Pipeline Overview**
1. Trigger CorrInit when a keyframe is added after Local BA.
2. Choose neighbors using covisibility and baseline ranking.
3. For each neighbor:
   - Run LoFTR (pairwise) and obtain `mkpts0_f`, `mkpts1_f`, `mconf`.
   - Keep top‑N' by `mconf` (deterministic).
   - Triangulate with `P = K [R|t]` in pixel‑space.
   - Gate by cheirality, reprojection error, and parallax.
   - Collect valid candidates.
4. Merge candidates from all neighbors.
5. Select top‑N by reprojection error and insert as Gaussian seeds.

**Step‑by‑Step Implementation Details**

**1) Snapshot Creation (GaussianMapper)**
1. Reference keyframe snapshot:
   - `Tcw`, intrinsics, undistorted RGB image.
2. Neighbor snapshots:
   - Chosen by covisibility, then highest baseline.
   - Use the same undistorted image and intrinsics.
3. Files:
   - `src/gaussian_mapper.cpp`
   - `include/gaussian_mapper.h`

**2) LoFTR Sidecar (Python)**
1. Input: undistorted grayscale images.
2. Resize to multiple of 32 for LoFTR.
3. Run LoFTR and return `mkpts0_f`, `mkpts1_f`, `mconf`.
4. Rescale keypoints back to original undistorted resolution.
5. Assumption: no crop is applied.
6. File:
   - `scripts/efficientloftr_service.py`

**3) IPC (ZeroMQ)**
1. C++ worker sends request: header + raw images.
2. Python replies: header + `mkpts0` + `mkpts1` + `mconf`.
3. ZMQ socket is reused (persistent), with timeouts and reconnect on error.
4. File:
   - `src/corr_init_worker.cpp`
   - `include/corr_init_worker.h`

**4) Triangulation and Gating**
1. Build `P0 = K0 [R|t]`, `P1 = K1 [R|t]`.
2. `cv::triangulatePoints` in pixel‑space.
3. Cheirality: `Z > 0` in both views.
4. Reprojection error: `e = max(e0, e1)` using pixel reprojection.
5. Parallax: compute angle between rays in world.
6. File:
   - `src/triangulator.cpp`
   - `include/triangulator.h`

**5) Candidate Selection (LoFTR‑correct)**
1. For each neighbor:
   - Take top‑N' by `mconf`.
   - Triangulate and gate.
   - Append valid candidates with their reproj error.
2. Merge all candidates from all neighbors.
3. Sort by reprojection error and keep top‑N.
4. This is the correct LoFTR adaptation.
5. File:
   - `src/corr_init_worker.cpp`

**6) Gaussian Seed Integration**
1. Colors sampled from ref image at `(u0, v0)` (nearest).
2. Points inserted via `gaussians_->increasePcd(...)`.
3. No changes to scale/opacity/rotation logic in Photo‑SLAM.
4. File:
   - `src/gaussian_mapper.cpp`

**Guards and Safety Checks**
1. Auto‑disable CorrInit if any camera is non‑pinhole.
2. Stereo requires rectified images:
   - `CorrInit.stereo_rectified = 1`
   - Otherwise assert and log.
3. All guards write to CorrInit CSV log.
4. File:
   - `src/gaussian_mapper.cpp`

**CSV Logging**
1. Single log file, no terminal spam.
2. Default path: `result_dir_/corrinit_log.csv`
3. Config override: `CorrInit.log_path`
4. Events:
   - `worker_start`, `worker_stop`, `zmq_connect`, `zmq_error`
   - `task_nb` for each neighbor
   - `task` for aggregated results
   - `integrate` for final insertion
   - `guard_disable` for safety aborts
5. File:
   - `src/corr_init_worker.cpp`

**Configuration Keys**
1. `CorrInit.enable`
2. `CorrInit.neighbor_k`
3. `CorrInit.neighbor_candidates`
4. `CorrInit.num_seeds`
5. `CorrInit.num_oversample`
6. `CorrInit.reproj_err_px`
7. `CorrInit.min_parallax_deg`
8. `CorrInit.queue_capacity`
9. `CorrInit.zmq_endpoint`
10. `CorrInit.log_path`
11. `CorrInit.stereo_rectified`

**Template YAMLs Provided**
CorrInit‑enabled template configs were generated for common datasets (files with `_corrinit.yaml` suffix):
- `cfg/gaussian_mapper/Monocular/Replica/replica_mono_corrinit.yaml`
- `cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd_corrinit.yaml`
- `cfg/gaussian_mapper/Monocular/TUM/tum_mono_corrinit.yaml`
- `cfg/gaussian_mapper/RGB-D/TUM/tum_rgbd_corrinit.yaml`
- `cfg/gaussian_mapper/Monocular/ETH3D/eth3d_mono_corrinit.yaml`
- `cfg/gaussian_mapper/RGB-D/ETH3D/eth3d_rgbd_corrinit.yaml`
- `cfg/gaussian_mapper/Stereo/EuRoC/EuRoC_corrinit.yaml`
- `cfg/gaussian_mapper/Stereo/KITTI/KITTI_corrinit.yaml`

**Auto‑Run Script**
An auto‑runner script starts LoFTR sidecar + Photo‑SLAM in one command:
- `scripts/run_corrinit.sh`

Environment variables:
- `ELOFTR_PYTHON` (default `python3`)
- `ELOFTR_WEIGHTS` (default `./weights/eloftr_outdoor.ckpt`)
- `ELOFTR_DEVICE` (default `cuda`)
- `ZMQ_ENDPOINT` (default `tcp://127.0.0.1:5555`)

Example:
```
./scripts/run_corrinit.sh ./bin/replica_rgbd \
  ./ORB-SLAM3/Vocabulary/ORBvoc.txt \
  ./cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \
  ./cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd_corrinit.yaml \
  /path/to/Replica/office0 \
  /path/to/results
```

**Files Added or Modified**
1. New:
   - `include/triangulator.h`
   - `src/triangulator.cpp`
   - `include/corr_init_worker.h`
   - `src/corr_init_worker.cpp`
   - `scripts/efficientloftr_service.py`
   - `scripts/run_corrinit.sh`
2. Modified:
   - `include/gaussian_mapper.h`
   - `src/gaussian_mapper.cpp`
   - `CMakeLists.txt`

**Known Differences from EDGS**
1. EDGS uses dense warps and certainty maps.
2. LoFTR does not provide shared ref‑pixel samples across neighbors.
3. Therefore:
   - EDGS “min‑over‑neighbors per ref‑pixel” is not strictly replicable.
   - Correct LoFTR adaptation is **pairwise candidate union** + top‑N selection.
4. When `k = 1`, the LoFTR pipeline is still correct and becomes a single pairwise triangulation with gating.

**Assumptions**
1. Images are undistorted (mono/RGBD).
2. No cropping is applied in LoFTR preprocessing.
3. Camera model is pinhole.
4. Stereo images are rectified if CorrInit is enabled.

**Validation Checklist**
1. Check CSV log:
   - `task` rows have reasonable reproj error and parallax.
2. Verify `guard_disable` does not trigger unexpectedly.
3. Confirm seeds increase `gaussians_->xyz_` count after integration.
4. If k = 1:
   - Pipeline should behave identically to single‑neighbor LoFTR triangulation.

---

# How To Run (CorrInit + LoFTR)

This section provides **exact step‑by‑step run instructions** for the new method.

## 0) Dependencies for CorrInit (ZMQ + LoFTR)

CorrInit uses ZeroMQ and a Python LoFTR sidecar.

Install ZeroMQ + cppzmq headers:
- Linux (apt):
  - `libzmq3-dev`, `cppzmq-dev`
- macOS (brew):
  - `zmq`, `cppzmq`

## 1) Build Photo‑SLAM with CorrInit Enabled

The default `build.sh` does not set CorrInit. You must enable it in CMake.

Example:
```
./build.sh

cd build
cmake .. -DWITH_CORRINIT_ZMQ=ON
make -j8
```

If you use custom Torch/OpenCV paths, append them to the `cmake ..` line.

## 2) Prepare EfficientLoFTR Environment

Inside this repo, EfficientLoFTR code is in:
```
reference/EfficientLoFTR
```

Create a python env and install requirements:
```
python3 -m venv .venv
source .venv/bin/activate
pip install -r reference/EfficientLoFTR/requirements.txt
```

Ensure LoFTR weights are available:
- Default expected path by the service: `weights/eloftr_outdoor.ckpt`
- You can override with `--weights` argument.

## 3) Start LoFTR Sidecar Service

Run from the repo root:
```
python3 scripts/efficientloftr_service.py \
  --endpoint tcp://127.0.0.1:5555 \
  --weights /absolute/path/to/eloftr_outdoor.ckpt \
  --device cuda
```

Notes:
- If you do not have GPU, use `--device cpu` (slower).
- Keep this process running while Photo‑SLAM executes.

## 4) Enable CorrInit in Gaussian Mapper YAML

Add the following section to your Gaussian mapper config (e.g. `cfg/gaussian_mapper/.../*.yaml`):
```
CorrInit.enable: 1
CorrInit.neighbor_k: 1
CorrInit.neighbor_candidates: 10
CorrInit.num_seeds: 512
CorrInit.num_oversample: 2048
CorrInit.reproj_err_px: 3.0
CorrInit.min_parallax_deg: 1.0
CorrInit.queue_capacity: 8
CorrInit.zmq_endpoint: "tcp://127.0.0.1:5555"
CorrInit.log_path: "corrinit_log.csv"
# For stereo only:
CorrInit.stereo_rectified: 1
```

## 5) Run Photo‑SLAM as Usual

Example (from README):
```
./bin/replica_rgbd \
  ./ORB-SLAM3/Vocabulary/ORBvoc.txt \
  ./cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \
  ./cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd.yaml \
  PATH_TO_Replica/office0 \
  PATH_TO_SAVE_RESULTS
```

## 6) Verify CorrInit Output

Check the CSV log:
```
PATH_TO_SAVE_RESULTS/corrinit_log.csv
```

Key checks:
- `event=task` rows should show reasonable reproj and parallax.
- If you see `guard_disable`, CorrInit was turned off (non‑pinhole or stereo not rectified).
