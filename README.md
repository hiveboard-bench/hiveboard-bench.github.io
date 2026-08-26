# HiveBoard Website

[![GitHub Pages](https://img.shields.io/badge/Hosted%20On-GitHub%20Pages-blue.svg)](https://hiveboard-bench.github.io)
[![Vite](https://img.shields.io/badge/Built%20with-Vite-646CFF.svg)](https://vitejs.dev/)
[![MuJoCo](https://img.shields.io/badge/Powered%20by-MuJoCo%20WASM-orange.svg)](https://mujoco.org/)

This repository hosts the official project website for **HiveBoard: An Open, Modular, 3D-Printed Benchmark of Industrial Mechanisms for Robotic and Prosthetic Manipulation**.

**Live Website:** [hiveboard-bench.github.io](https://hiveboard-bench.github.io)  
**Main Project Repository:** [github.com/EESC-LabRoM/HiveBoard](https://github.com/EESC-LabRoM/HiveBoard)


## Website Purpose

The purpose of this website is to serve as the interactive companion and presentation portal for the HiveBoard benchmark:

- **Interactive 3D & Physics Viewer:** Allows visitors to view and interactively simulate the mechanism modules directly in the browser using [Three.js](https://threejs.org/) and WebAssembly-powered [MuJoCo](https://mujoco.org/).
- **Module & Task Catalog:** Details the 13 functional 3D-printed mechanism attachments across three manipulation skill categories: *Torque*, *Precision*, and *Composed Assembly*.
- **Simulation Compatibility:** Provides information and direct access to digital assets for simulators including MuJoCo, Isaac Sim (USD), and standard URDFs.
- **Experimental Results & Media:** Showcases evaluation results and video demonstrations across diverse robotic embodiments (fixed-base arms, quadruped manipulators, VR-teleoperated robots, and wearable prosthetic hands).


## Tech Stack

- **Bundler / Dev Server:** [Vite](https://vitejs.dev/)
- **UI Framework / Styling:** [Bulma CSS](https://bulma.io/) & Custom CSS
- **3D & Physics Rendering:** [Three.js](https://threejs.org/) & [@mujoco/mujoco](https://www.npmjs.com/package/@mujoco/mujoco)
- **Deployment:** GitHub Pages via `gh-pages`


## Getting Started

### Prerequisites

Make sure you have [Node.js](https://nodejs.org/) (version 18+ recommended) installed.

### Installation

Clone the repository and install the dependencies:

```bash
git clone https://github.com/hiveboard-bench/hiveboard-bench.github.io.git
cd hiveboard-bench.github.io
npm install
```

### Local Development

Start the local Vite development server:

```bash
npm run dev
```

Open your browser at `http://localhost:5173` (or the URL printed in your terminal).

### Build for Production

To create an optimized production build in the `dist/` directory:

```bash
npm run build
```

You can preview the built site locally with:

```bash
npm run preview
```

### Deployment

Deploy to GitHub Pages:

```bash
npm run deploy
```

## Modifying & Authoring Trajectories

The simulation viewer replays deterministic task-space trajectories (approach, grasp, actuate, release, retreat) solved with inverse kinematics and verified against MuJoCo physics. You can modify these trajectories in two ways:

### 1. Interactive Browser-Based Editor

The repository includes a live visual editor to adjust waypoints directly in 3D:

1. **Start the editor backend server:**
   ```bash
   python3 tools/traj_edit.py
   ```
2. **Start the Vite dev server (in a separate terminal):**
   ```bash
   npm run dev
   ```
3. **Open the editor in your browser:**  
   Navigate to `http://localhost:5173/sim/hiveboard-sim.html?edit=1`
4. **Interact & Save:**
   - Click and drag the keyframe beads along the motion path (or select a bead and use arrow keys to nudge coordinates).
   - The backend re-solves IK and simulates acceptance replay in real-time.
   - Click **Save** to persist waypoint offsets to `tools/traj_edits.json` and update the robot's `.traj.json` file.

### 2. Programmatic Authoring via Python

- **Task Keyframes & Logic:** Authored in [`tools/sim_trajectories.py`](tools/sim_trajectories.py) (defines Cartesian waypoints, approach directions, gripper actions, and acceptance criteria).
- **Robot Specs & Layouts:** Configured in [`tools/build-sim-assets.py`](tools/build-sim-assets.py).
- **Rebuild Assets & Trajectories:**
  ```bash
  python3 tools/build-sim-assets.py
  ```
  *(This compiles MuJoCo scenes, solves IK, and runs acceptance tests across all modules).*

## Repository Structure


```text
├── assets-src/      # Source assets and 3D models
├── public/          # Static assets served directly (textures, models, videos)
├── src/             # Frontend source scripts and utilities
├── tools/           # Python helper scripts for model compression and asset optimization
├── index.html       # Main HTML page and presentation layout
├── viewer.js        # MuJoCo WebAssembly and Three.js 3D interactive viewer logic
├── style.css        # Website styling and responsive design rules
└── package.json     # Node.js dependencies and scripts
```

## Citation

If you find HiveBoard useful in your research, please cite:

```bibtex
@article{godoy2024hiveboard,
  title     = {HiveBoard: An Open, Modular, 3D-Printed Benchmark of Industrial Mechanisms for Robotic and Prosthetic Manipulation},
  author    = {Godoy, Ricardo V. and de Souza, Enzo F. and de Lange, Rudy De-Xin and Negri, Juliano and Marsicano, Jo{\~a}o A. and van Halst, Victor and Vijayan, Aravind Elanjimattathil and Capezzuto, Gianluca and Angarola, Matheus P. and Tommaselli, Felipe A. G. and Baptista, Rafael R. and van Berge, Meiko Adriana and Bezerra, Ranulfo and Lahr, Gustavo J. G. and Gerez, Lucas Ferrari and Becker, Marcelo},
  journal   = {arXiv preprint},
  year      = {2024}
}
```

