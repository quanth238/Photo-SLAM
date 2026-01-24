#!/bin/bash

# Path handling
PHOTO_SLAM_DIR=$(pwd)
# Assuming script is run from project root, or set absolute path
DATASET_ROOT="/home/crl/lehieu/MyPhotoSLAM/data/Replica"
BIN="./bin/replica_rgbd"
VOC="ORB-SLAM3/Vocabulary/ORBvoc.txt"

# Set environment
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH

# Sequences
sequences=("office0" "office1" "office2" "office3" "office4" "room0" "room1" "room2")

echo "Starting batch processing for Replica RGB-D dataset (3 runs per sequence)..."

# Run for 3 seeds/trials
for i in 0 1 2
do
    echo "================================================================"
    echo "Starting RUN $i"
    echo "================================================================"

    for seq in "${sequences[@]}"; do
        echo "----------------------------------------------------------------"
        echo "Processing sequence: $seq (Run $i)"
        
        # Define paths
        CONFIG_SLAM="cfg/ORB_SLAM3/RGB-D/Replica/${seq}.yaml"
        # Using scene-specific config for Mapper as well
        CONFIG_MAPPER="cfg/gaussian_mapper/RGB-D/Replica/${seq}.yaml"
        DATA_PATH="${DATASET_ROOT}/${seq}"
        
        # Result path separation by run index
        RESULT_PATH="results/replica_rgbd_batch_${i}/${seq}"
        
        # Check if data exists
        if [ ! -d "$DATA_PATH" ]; then
            echo "Error: Data path does not exist: $DATA_PATH"
            continue
        fi
        
        # Create result dir
        mkdir -p "$RESULT_PATH"
        
        # Run command
        # Syntax: ./bin/replica_rgbd vocabulary slam_settings mapper_settings sequence_path output_path no_viewer
        $BIN $VOC $CONFIG_SLAM $CONFIG_MAPPER $DATA_PATH $RESULT_PATH no_viewer > "${RESULT_PATH}/run.log" 2>&1
        
        if [ $? -eq 0 ]; then
            echo "Finished sequence: $seq (Run $i)"
        else
            echo "Error running: $seq (Run $i). Log: ${RESULT_PATH}/run.log"
        fi
    done
done

echo "Batch processing complete."
