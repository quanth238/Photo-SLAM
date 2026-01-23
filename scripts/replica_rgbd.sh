#!/bin/bash

# Set Photo-SLAM root directory
PHOTO_SLAM_DIR=/home/crl/lehieu/Photo-SLAM
REPLICA_DATA=/home/crl/lehieu/MyPhotoSLAM/data/Replica

# Set environment
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH

for i in 0
do
    $PHOTO_SLAM_DIR/bin/replica_rgbd \
        $PHOTO_SLAM_DIR/ORB-SLAM3/Vocabulary/ORBvoc.txt \
        $PHOTO_SLAM_DIR/cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \
        $PHOTO_SLAM_DIR/cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd.yaml \
        $REPLICA_DATA/office0 \
        $PHOTO_SLAM_DIR/results/replica_rgbd_$i/office0 \
        no_viewer
done