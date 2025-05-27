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

Clone the **ProceduralMaize3D** repository and install the required libraries:

```bash
git clone https://github.com/baskargroup/ProceduralMaize3D.git
cd PSO_NURBSDIFF
pip install -r requirements.txt
```

## Usage 🛠️

### 1. Directory Structure 📂

```bash
ProceduralMaize3D/
├── data/            # Input data (e.g., point clouds, configurations)
├── output/          # Results and logs
│   ├── videos/      # Video results
├── src/             # Source code and scripts
│   ├── PSO_NURBSDIFF.py
│   ├── config.py
│   ├── extract_plants.py
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
@article{HADADI2025110382,
  title = {Procedural generation of 3D maize plant architecture from LiDAR data},
  author = {Mozhgan Hadadi and Mehdi Saraeian and Jackson Godbersen and Talukder Z. Jubery and Yawei Li and Lakshmi Attigala and Aditya Balu and Soumik Sarkar and Patrick S. Schnable and Adarsh Krishnamurthy and Baskar Ganapathysubramanian},
  journal = {Computers and Electronics in Agriculture},
  volume = {236},
  pages = {110382},
  year = {2025},
  issn = {0168-1699},
  doi = {https://doi.org/10.1016/j.compag.2025.110382},
  url = {https://www.sciencedirect.com/science/article/pii/S0168169925004880},
  keywords = {Procedural modeling, Differentiable splines, 3D plant phenotyping, Field grown maize plants, Point cloud data}
}
```
