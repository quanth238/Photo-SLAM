# Photo-SLAM + CorrInit: Full Install (CUDA 12.8 + OpenCV CUDA)

Tài liệu này là checklist **từ đầu đến cuối** để build Photo-SLAM + CorrInit trên GPU **sm_120 (RTX 5090)**. Mục tiêu là **reproducible**, tránh lỗi do **nvcc sai version**, **OpenCV không có CUDA**, hoặc **CMake cache bẩn**.

## 0) Assumptions
- Bạn đã có CUDA 12.8 ở `/usr/local/cuda-12.8`.
- GPU: sm_120 (RTX 5090).
- Dùng GCC/G++ 12 (nvcc không hỗ trợ gcc-13).
- Conda env `photoslam` đã có `torch 2.8.0+cu128`.
- Bạn **không dùng /usr/local** để cài OpenCV (server chung dễ bị dọn), mà dùng `$HOME/.local/...`.

## 1) Environment bắt buộc
```bash
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export CUDACXX=$CUDA_HOME/bin/nvcc

export CC=/usr/bin/gcc-12
export CXX=/usr/bin/g++-12
export CUDAHOSTCXX=/usr/bin/g++-12
```

**Sanity check**:
```bash
which nvcc
nvcc --version
```
Phải thấy CUDA **12.8**. Nếu thấy `/usr/bin/nvcc` (CUDA 12.0) → sai.

## 2) Build OpenCV CUDA (khuyến nghị 4.10.0)
OpenCV 4.8.1 **bị lỗi cudev/cudaarithm** với CUDA 12.8. Vì vậy dùng **4.10.0**.

```bash
mkdir -p ~/third_party && cd ~/third_party

OPENCV_VER=4.10.0
rm -rf opencv opencv_contrib

git clone -b $OPENCV_VER https://github.com/opencv/opencv.git
git clone -b $OPENCV_VER https://github.com/opencv/opencv_contrib.git

export OPENCV_PREFIX=$HOME/.local/opencv-${OPENCV_VER}-cuda

cd ~/third_party/opencv
rm -rf build && mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$OPENCV_PREFIX \
  -DOPENCV_EXTRA_MODULES_PATH=$HOME/third_party/opencv_contrib/modules \
  -DWITH_CUDA=ON \
  -DCUDA_ARCH_NAME=Manual \
  -DCUDA_ARCH_BIN=12.0 \
  -DCUDA_ARCH_PTX= \
  -DCMAKE_CUDA_COMPILER=$CUDACXX \
  -DCMAKE_CUDA_HOST_COMPILER=$CUDAHOSTCXX \
  -DOPENCV_DNN_CUDA=OFF \
  -DWITH_CUDNN=OFF \
  -DWITH_NVCUVID=OFF \
  -DWITH_NVCUVENC=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DOPENCV_WARNINGS_ARE_ERRORS=OFF

make -j$(nproc)
make install
```

**Check bắt buộc**:
```bash
ls $OPENCV_PREFIX/include/opencv4/opencv2/cudawarping.hpp
ls $OPENCV_PREFIX/include/opencv4/opencv2/cudaimgproc.hpp
```
Nếu 2 file trên **không tồn tại**, OpenCV chưa build CUDA đúng.

## 3) Lấy Torch_DIR chính xác
```bash
TORCH_DIR=$(python3 - <<'PY'
import torch, os
print(os.path.join(torch.__path__[0], "share", "cmake", "Torch"))
PY
)

ls "$TORCH_DIR/TorchConfig.cmake"
```
Nếu không tìm thấy `TorchConfig.cmake` → bạn đang ở sai conda env.

## 4) Build Photo‑SLAM + CorrInit
```bash
cd /home/crl/Congthai/photoslam_edgs

export OPENCV_DIR=$OPENCV_PREFIX/lib/cmake/opencv4
export LD_LIBRARY_PATH=$OPENCV_PREFIX/lib:$LD_LIBRARY_PATH

rm -rf build && mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DWITH_CORRINIT_ZMQ=ON \
  -DOpenCV_DIR="$OPENCV_DIR" \
  -DTorch_DIR="$TORCH_DIR" \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_CUDA_COMPILER=$CUDACXX \
  -DCMAKE_CUDA_HOST_COMPILER=$CUDAHOSTCXX

make -j8
```

## 5) Lỗi thường gặp và nguyên nhân đúng

### ❌ `Unsupported gpu architecture 'compute_120'`
- nvcc bị trỏ nhầm (CUDA 12.0). Fix: export `CUDACXX=/usr/local/cuda-12.8/bin/nvcc`.

### ❌ `opencv2/cudaimgproc.hpp: No such file`
- OpenCV bạn link **không có CUDA**.
- Fix: build OpenCV CUDA đúng và set `OpenCV_DIR` đúng.

### ❌ `CUDA: 1.0 compute capability is not supported`
- Bạn chưa `CUDA_ARCH_NAME=Manual` hoặc cache chưa sạch.
- Fix: `rm -rf build` rồi cmake lại với `CUDA_ARCH_NAME=Manual`.

### ❌ `blockReduce` / cudaarithm compile fail
- OpenCV version cũ không tương thích CUDA 12.8.
- Fix: dùng OpenCV 4.10.0+.

## 6) Ghi chú quan trọng
- **Không dùng /usr/local** để tránh bị dọn trên server chung.
- Mỗi lần đổi CUDA/OpenCV/Torch → **xóa build** (`rm -rf build`).
- Luôn chạy trong **conda env đúng**.

## 7) Chạy nhanh (one-liner)
Ví dụ chạy evaluation 1 scene với kết quả lưu theo timestamp:
```bash
RESULTS_ROOT=results/tum_rgbd_corrinit_eval_$(date +%H%M%S) \
  bash scripts/run_tum_rgbd_corrinit_eval.sh
```

---

Nếu cần mình viết **script tự động** chạy toàn bộ từ đầu đến cuối, dùng `scripts/install_all.sh` bên dưới.
