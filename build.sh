#!/bin/bash
set -e  # dừng ngay nếu có lỗi

# ============================================================
# Environment Setup
# ============================================================
export CC=/usr/bin/gcc-12
export CXX=/usr/bin/g++-12
export CUDAHOSTCXX=/usr/bin/g++-12
export CUDA_HOME=/usr/local/cuda-12.8
export CUDA_ROOT=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH
# RTX 5090 (sm_120) support for custom CUDA kernels
export TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_LIST:-12.0}
CUDA_ARCH=${CUDA_ARCH:-120}

# Common cmake flags
CMAKE_FLAGS="\
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-12 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-12 \
  -DCMAKE_CXX_FLAGS=-Wno-error=array-bounds \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.8 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc \
  -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-12 \
  -DCUDA_HOST_COMPILER=/usr/bin/gcc-12 \
  -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} \
  -DOpenCV_DIR=/usr/local/lib/cmake/opencv4"

# ============================================================
# DBoW2
# ============================================================
echo "========================================="
echo "Building DBoW2 ..."
echo "========================================="
cd ./ORB-SLAM3/Thirdparty/DBoW2
rm -rf build && mkdir build
cd build
cmake .. $CMAKE_FLAGS
make -j$(nproc)

# ============================================================
# g2o
# ============================================================
echo "========================================="
echo "Building g2o ..."
echo "========================================="
cd ../../g2o
rm -rf build && mkdir build
cd build
cmake .. $CMAKE_FLAGS
make -j$(nproc)

# ============================================================
# Sophus
# ============================================================
echo "========================================="
echo "Building Sophus ..."
echo "========================================="
cd ../../Sophus
rm -rf build && mkdir build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-12 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-12 \
  -DCMAKE_CXX_FLAGS="-Wno-error=array-bounds -Wno-array-bounds" \
  -DBUILD_TESTS=OFF
make -j$(nproc)

# ============================================================
# ORB-SLAM3 Vocabulary
# ============================================================
echo "========================================="
echo "Uncompressing vocabulary ..."
echo "========================================="
cd ../../../Vocabulary
tar -xf ORBvoc.txt.tar.gz
echo "Vocabulary OK"

# ============================================================
# ORB-SLAM3
# ============================================================
echo "========================================="
echo "Building ORB-SLAM3 ..."
echo "========================================="
cd ..
rm -rf build && mkdir build
cd build
cmake .. $CMAKE_FLAGS
make -j8

# ============================================================
# Photo-SLAM
# ============================================================
echo "========================================="
echo "Building Photo-SLAM ..."
echo "========================================="
cd ../..
rm -rf build && mkdir build
cd build
cmake .. $CMAKE_FLAGS \
  -DTorch_DIR=/home/crl/miniconda3/envs/photoslam/lib/python3.10/site-packages/torch/share/cmake/Torch
make -j8

echo "========================================="
echo "Build hoàn tất! ✅"
echo "========================================="
