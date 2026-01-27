#!/bin/bash

# Set Photo-SLAM root directory
PHOTO_SLAM_DIR=/home/crl/lehieu/Photo-SLAM
REPLICA_DATA=/home/crl/lehieu/MyPhotoSLAM/data/Replica

# Set environment
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH

# Run for seed 0 (can change to "0 1 2 3 4" for 5 runs)
for i in 0
do
    # List of all Replica scenes
    scenes="office0 office1 office2 office3 office4 room0 room1 room2"
    
    for scene in $scenes
    do
        echo "Running Replica RGBD for scene: $scene (Run $i)"
        
        # Create result directory
        mkdir -p $PHOTO_SLAM_DIR/results/replica_rgbd_$i/$scene
        
        $PHOTO_SLAM_DIR/bin/replica_rgbd \
            $PHOTO_SLAM_DIR/ORB-SLAM3/Vocabulary/ORBvoc.txt \
            $PHOTO_SLAM_DIR/cfg/ORB_SLAM3/RGB-D/Replica/$scene.yaml \
            $PHOTO_SLAM_DIR/cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd.yaml \
            $REPLICA_DATA/$scene \
            $PHOTO_SLAM_DIR/results/replica_rgbd_$i/$scene \
            no_viewer
            
        echo "Finished $scene"
    done
done