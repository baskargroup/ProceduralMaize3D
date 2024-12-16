import os

# Directories
BASE_DIR = "./PSO_NURBS"

OUT_PSO_DIR = os.path.join(BASE_DIR, "Out_PSO")
OUT_NURBS_DIR = os.path.join(BASE_DIR, "Out_NURBS")
NPY_NURBS_DIFF_DIR = os.path.join(BASE_DIR, "npy_NURBS")
NPY_PSO_DIR = os.path.join(BASE_DIR, "npy_PSO")
CSV_DIR = os.path.join(BASE_DIR, "CSV")
PLANT_DIR_PLY = os.path.join(BASE_DIR, "plants_ply")
PLANT_DIR_DAT = os.path.join(BASE_DIR, "plants_dat")
PLANT_LIST_FILE = os.path.join(BASE_DIR, "plant_list.txt")
CHAMFER_CONVERGENCE_DIR = os.path.join(BASE_DIR, "Chamfer_convergence_plot")
COST_PLOT_DIR = os.path.join(BASE_DIR, "Cost_Plot")
LOG_DIR = os.path.join(BASE_DIR, "Plants_Log")
PSO_NURBSDIFF_LOG_DIR = os.path.join(BASE_DIR, "Execution_Times")
