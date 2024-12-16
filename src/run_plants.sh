#!/bin/bash

# Directory containing the Python scripts
SCRIPT_DIR="/work/mech-ai/mozhgan/NURBSDiff/examples/Maize"

# Python scripts
EXTRACT_SCRIPT="extract_plants.py"
MAIN_SCRIPT="PSO_NURBSDIFF.py"

# Log and plant list paths
PLANT_LIST=$(python -c "import config; print(config.PLANT_LIST_FILE)")
LOG_DIR=$(python -c "import config; print(config.LOG_DIR)")

# Create the log directory if it does not exist
mkdir -p "$LOG_DIR"

# Step 1: Run the Python script to extract the plant names and leaf counts
python "$SCRIPT_DIR/$EXTRACT_SCRIPT"

# Function to get the CPU memory usage of the current process in MB
get_memory_usage() {
    local pid=$1
    local mem=$(ps -o rss= -p $pid)
    echo "scale=2; $mem/1024" | bc
}

# Function to check if `nvidia-smi` is available and to get the GPU memory usage in MB
get_gpu_memory_usage() {
    if command -v nvidia-smi &> /dev/null; then
        local gpu_id=$1
        local usage=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -i $gpu_id)
        echo "scale=2; $usage" | bc
    else
        echo "N/A"
    fi
}


# Step 2: Read the plant names and leaf counts from the file and run the main Python script for each plant
while IFS=',' read -r plant leaf_count; do
    echo "Processing plant: $plant with $leaf_count leaves"
    
    # Start the timer
    start=`date +%s.%N`
    
    # Start the main Python script and record memory usage
    python "$SCRIPT_DIR/$MAIN_SCRIPT" --plant "$plant" --leaf_count "$leaf_count" > "$LOG_DIR/${plant}_output.log" 2>&1 &
    maize_pid=$!

    # Write the header to the log files
    echo "Timestamp,CPU Memory (MB),GPU Memory (MB)" >> "$LOG_DIR/${plant}_memory_usage.log"

    # Record memory usage every second while the script is running
    while kill -0 $maize_pid 2>/dev/null; do
        current_time=$(date +%s.%N)
        cpu_memory_usage=$(get_memory_usage $maize_pid)
        gpu_memory_usage=$(get_gpu_memory_usage 0)  # Assuming a single GPU, use 0 as the GPU ID
        echo "$current_time, $cpu_memory_usage, $gpu_memory_usage" >> "$LOG_DIR/${plant}_memory_usage.log"
        sleep 1
    done

    # Calculate the total runtime
    end=`date +%s.%N`
    runtime=$( echo "$end - $start" | bc -l )
    echo "Total Time: $runtime seconds" >> "$LOG_DIR/${plant}_output.log"

    # Check if the script ran successfully
    if [ $? -eq 0 ]; then
        echo "Plant $plant processed successfully."
    else
        echo "Error processing plant $plant. Check log: $LOG_DIR/${plant}_output.log"
    fi

done < "$PLANT_LIST"

echo "All plants processed."
