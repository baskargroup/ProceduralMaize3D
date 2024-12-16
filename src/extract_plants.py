import os
from collections import defaultdict
import config 

if __name__ == "__main__":

    # Dictionary to store plant names and their corresponding leaf count
    pcd_dir = config.PLANT_DIR_PLY
    plant_leaf_count = defaultdict(int)

    # Iterate through all the files in the directory to find the plants
    distinct_plants = set()
    plants_and_leaves = []

    # Iterate through all the files in the directory
    for filename in sorted(os.listdir(pcd_dir)):
        if filename.endswith('.ply'):
            # Extract the plant name before the first underscore
            plant_name = filename.split('_L')[0]
            
            # Add the plant name to the set of distinct plants
            distinct_plants.add(plant_name)
            
            # Check if the file represents a leaf (L1, L2, ...)
            if '_L' in filename:
                plant_leaf_count[plant_name] += 1

    # Create a list of plant names and their leaf counts
    plants_and_leaves = [(plant, leaf_count) for plant, leaf_count in plant_leaf_count.items()]

    # Save plant names and leaf counts to a file
    with open(config.PLANT_LIST_FILE, 'w') as f:
        for plant, leaf_count in plants_and_leaves:
            f.write(f"{plant},{leaf_count}\n")
