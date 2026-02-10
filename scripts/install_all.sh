#!/usr/bin/env bash
set -euo pipefail

# === Configurable ===
OPENCV_VER=${OPENCV_VER:-4.10.0}
OPENCV_PREFIX=${OPENCV_PREFIX:-$HOME/.local/opencv-${OPENCV_VER}-cuda}
THIRD_PARTY=${THIRD_PARTY:-$HOME/third_party}
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
NPROC=${NPROC:-$(nproc)}

# === Toolchain (CUDA 12.8 + GCC 12) ===
export CUDA_HOME=${CUDA_HOME:-/usr/local/cuda-12.8}
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
export CUDACXX=${CUDACXX:-$CUDA_HOME/bin/nvcc}

export CC=${CC:-/usr/bin/gcc-12}
export CXX=${CXX:-/usr/bin/g++-12}
export CUDAHOSTCXX=${CUDAHOSTCXX:-/usr/bin/g++-12}

# === Sanity ===
if ! command -v "$CUDACXX" >/dev/null 2>&1; then
  echo "ERROR: nvcc not found at $CUDACXX"
  exit 1
fi

nvcc --version || true

# === Torch_DIR ===
TORCH_DIR=$(python3 - <<'PY'
import torch, os
print(os.path.join(torch.__path__[0], "share", "cmake", "Torch"))
PY
)
if [[ ! -f "$TORCH_DIR/TorchConfig.cmake" ]]; then
  echo "ERROR: TorchConfig.cmake not found at $TORCH_DIR"
  echo "Activate correct conda env (photoslam) and retry."
  exit 1
fi

# === Clone OpenCV ===
mkdir -p "$THIRD_PARTY"
cd "$THIRD_PARTY"

if [[ ! -d opencv ]]; then
  git clone -b "$OPENCV_VER" https://github.com/opencv/opencv.git
else
  (cd opencv && git fetch --all && git checkout "$OPENCV_VER")
fi

if [[ ! -d opencv_contrib ]]; then
  git clone -b "$OPENCV_VER" https://github.com/opencv/opencv_contrib.git
else
  (cd opencv_contrib && git fetch --all && git checkout "$OPENCV_VER")
fi

# === Build OpenCV CUDA ===
cd "$THIRD_PARTY/opencv"
rm -rf build && mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$OPENCV_PREFIX" \
  -DOPENCV_EXTRA_MODULES_PATH="$THIRD_PARTY/opencv_contrib/modules" \
  -DWITH_CUDA=ON \
  -DCUDA_ARCH_NAME=Manual \
  -DCUDA_ARCH_BIN=12.0 \
  -DCUDA_ARCH_PTX= \
  -DCMAKE_CUDA_COMPILER="$CUDACXX" \
  -DCMAKE_CUDA_HOST_COMPILER="$CUDAHOSTCXX" \
  -DOPENCV_DNN_CUDA=OFF \
  -DWITH_CUDNN=OFF \
  -DWITH_NVCUVID=OFF \
  -DWITH_NVCUVENC=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DOPENCV_WARNINGS_ARE_ERRORS=OFF

make -j"$NPROC"
make install

# === Verify CUDA headers ===
if [[ ! -f "$OPENCV_PREFIX/include/opencv4/opencv2/cudawarping.hpp" ]]; then
  echo "ERROR: cudawarping.hpp not found under $OPENCV_PREFIX"
  exit 1
fi
if [[ ! -f "$OPENCV_PREFIX/include/opencv4/opencv2/cudaimgproc.hpp" ]]; then
  echo "ERROR: cudaimgproc.hpp not found under $OPENCV_PREFIX"
  exit 1
fi

# === Build Photo-SLAM ===
cd "$REPO_ROOT"
export OPENCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4"
export LD_LIBRARY_PATH="$OPENCV_PREFIX/lib:${LD_LIBRARY_PATH:-}"

# Clean stale build outputs to avoid loading old CUDA binaries
rm -f "$REPO_ROOT/lib/"*.so || true
rm -rf build && mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DWITH_CORRINIT_ZMQ=ON \
  -DOpenCV_DIR="$OPENCV_DIR" \
  -DTorch_DIR="$TORCH_DIR" \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_CUDA_COMPILER="$CUDACXX" \
  -DCMAKE_CUDA_HOST_COMPILER="$CUDAHOSTCXX"

make -j8

echo "DONE. OpenCV at: $OPENCV_PREFIX"
