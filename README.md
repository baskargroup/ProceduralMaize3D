# PSO and NURBS-Diff Optimization for Maize Plant Surfaces 🌽✨

This project implements a pipeline for optimizing NURBS surfaces using Particle Swarm Optimization (PSO) and NURBS-Diff. It is designed to process plant point clouds of maize leaves and generate optimized surfaces that are fitted to the point clouds of the leaves.

## Features 🚀

- Optimize 3D plant surfaces using NURBS. 🌱
- Visualize results with figures and videos. 📊🎥
- Modular scripts for extracting, processing, and optimizing data. 🛠️

## Prerequisites 🖥️

- Python 3.8 or higher 🐍
- CUDA-enabled GPU for optimal performance (optional) ⚡
- Libraries listed in `requirements.txt` 📜

## Installation

### 1. Clone NURBS-Diff Repository 🔗

Before starting, you need to clone and set up the **NURBS-Diff** repository:

```bash
git clone https://github.com/idealab-isu/NURBSDiff.git
cd NURBSDiff
pip install -r requirements.txt
```

### 2. Clone This Repository 🔗

```bash
Clone the PSO_NURBSDIFF repository and install the required libraries:

git clone https://github.com/yourusername/PSO_NURBSDIFF.git
cd PSO_NURBSDIFF
pip install -r requirements.txt
```

## Usage 🛠️

### 1. Directory Structure 📂

```bash
PSO_NURBSDIFF/
├── data/            # Input data (e.g., point clouds, configurations)
├── output/          # Results and logs
│   ├── figures/     # Convergence plots and visualizations
│   ├── logs/        # Log files
│   └── videos/      # Video results
├── src/             # Source code and scripts
│   ├── extract_plants.py
│   ├── PSO_NURBSDIFF.py
│   └── run_plants.sh
├── README.md        # Documentation
├── requirements.txt # Dependencies
└── .gitignore       # Ignored files
```

### 2. Extract Plant Data 🌿

Run the following command to extract plant names and leaf counts from the input data:

```bash
python scripts/extract_plants.py
```

### 3. Run the Optimization Pipeline 🚀

To process the plants and generate optimized surfaces, use:

```bash
bash scripts/run_plants.sh
```

### 3. View Results 📊

Optimized surfaces and plots will be saved in the `output/` directory. Videos and figures can be found in the `output/videos/` and `output/figures/` directories.

## Outputs 📦

- Optimized Surface Files: `.dat` and `.stl` formats. 📁

- Convergence Plots: Chamfer distance reduction for each plant. 📉

- Videos: Visualizations of 3D plant surfaces. 🎥