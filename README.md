<div align="left">
<h1 align="left">
<br>FG_SLAM</h1>
<h3> Developed with the software and tools below.</h3>
<p align="left">
</p>
<img src="https://img.shields.io/github/license/alstondu/GNSS-DR_Loco?style=flat-square&color=5D6D7E" alt="GitHub license" />
<img src="https://img.shields.io/github/last-commit/alstondu/GNSS-DR_Loco?style=flat-square&color=5D6D7E" alt="git-last-commit" />
<img src="https://img.shields.io/github/languages/top/alstondu/GNSS-DR_Loco?style=flat-square&color=5D6D7E" alt="GitHub top language" />
</div>

---

## 📖 Table of Contents
- [📖 Table of Contents](#-table-of-contents)
- [📍 Overview](#-overview)
- [📦 Workflow](#-workflow)
- [🔧 Installation](#-installation)
- [🤖 Running FG_SLAM](#-running-fg-slam)
- [📄 License](#-license)

---

## 📍 Overview
Practice of graphical-model based SLAM implemented with the MiniSLAM event-based framework and the MATLAB port of the g2o port.
For detailed description and instructions, please refer to [COMP0130_-_23-24_CW_02.pdf](https://github.com/alstondu/FG_SLAM/blob/main/COMP0130_-_23-24_CW_02.pdf).

---

## 📦 Workflow

1. Implement the prediction and update steps using the vehicle process model, together with
compass and GPS sensors.

2. Extend the implementation to undertake SLAM and explore some characteristics of
the algorithm behaviour.

3. Investigates ways to improve the performance of the SLAM system.

---

### 🔧 Installation

Clone the FG_SLAM repository:
```sh
git clone https://github.com/alstondu/FG_SLAM
```

---
### 🤖 Running FG_SLAM

1. Run [install_sparseinv.m](https://github.com/alstondu/FG_SLAM/blob/main/install_sparseinv.m) to install sparseinv

2. Run [setup.m](https://github.com/alstondu/FG_SLAM/blob/main/setup.m) add the Libraries path
   
3. Run qx_x.m file for corresponding tasks

---

## 📄 License


This project is protected under the [MIT](https://github.com/alstondu/GNSS-DR_Loco/blob/main/LICENSE) License.
