# Procedural Generation of 3D Maize Plant Architecture from LIDAR Data 🌽✨

This repository presents a robust framework for generating procedural 3D models of maize plants from LiDAR point cloud data. The method combines Particle Swarm Optimization (PSO) and NURBS-Diff to produce high-fidelity reconstructions of maize leaf surfaces, providing a scalable and automated approach for plant phenotyping.

## Features 🚀

- **Procedural Modeling:** Automates the reconstruction of maize leaf surfaces from point clouds.

- **Two-Step Optimization:** Combines PSO for initial surface fitting and NURBS-Diff for detailed refinement.

- **Genotype Versatility:** Demonstrates adaptability to diverse maize genotypes.

- **Open-Source:** Implements reproducible and accessible phenotyping methods.

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

Clone the **PSO_NURBSDIFF** repository and install the required libraries:

```bash
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

## Methodology 🔍

### Overview:

This two-step optimization approach includes:

1. **Particle Swarm Optimization (PSO):** Generates an approximate NURBS surface by optimizing control points to align with the point cloud data.

2. **NURBS-Diff Refinement:** Applies gradient-based optimization to refine the initial surface for high-fidelity reconstructions, capturing intricate leaf details like edges and tips.

### Key Advantages:

- Handles noisy and sparse point cloud data.

- Efficiently models diverse maize genotypes.

- Enables accurate trait extraction and simulation.

## Outputs 📦

- **Optimized Surface Files:** `.dat` and `.stl` formats. 📁

- **Convergence Plots:** Chamfer distance reduction for each plant. 📉

- **Videos:** Visualizations of 3D plant surfaces. 🎥


## Example Visualization for 78551S genotype 🎥

| segmented point cloud | segmented point cloud | NURBS-Diff output |
|-------|-------|-------|
| ![GIF 1](output/videos/PCD_color.gif) | ![GIF 2](output/videos/PSO.gif) | ![GIF 3](output/videos/NURBS-Diff.gif) |

## Citation
```bibtex
@article{hadadi2025procedural,
  title={Procedural Generation of 3D Maize Plant Architecture from LIDAR Data},
  author={Hadadi, Mozhgan and Saraeian, Mehdi and Godbersen, Jackson and Jubery, Talukder and Li, Yawei and Attigala, Lakshmi and Balu, Aditya and Sarkar, Soumik and Schnable, Patrick S and Krishnamurthy, Adarsh and others},
  journal={arXiv preprint arXiv:2501.13963},
  year={2025}
  doi={10.48550/arXiv.2501.13963}
}
```
