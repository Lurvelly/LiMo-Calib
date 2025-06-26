
# LiMo-Calib 🚀  
_On-Site, Fast LiDAR-Motor Calibration for Quadruped-Robot 360° Perception_  
**IROS 2025 (accepted)** · Nanyang Technological University & Dalian University of Technology

[![Paper](https://img.shields.io/badge/arXiv-2502.12655-b31b1b.svg)](https://arxiv.org/abs/2502.12655)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Build](https://github.com/kafeiyin00/LiMo-Calib/actions/workflows/ci.yml/badge.svg)](https://github.com/kafeiyin00/LiMo-Calib/actions)

<div align="center">
  <img src="docs/media/overview.png" width="75%" alt="LiMo-Calib system overview"/>
</div>

---

## ✨ Highlights
| ⚡ | What makes **LiMo-Calib** unique? |
|----|----------------------------------|
| **Target-free** | Uses raw LiDAR geometry – no checkerboards or AprilTags |
| **4-DOF optimization** | Calibrates roll, pitch, x & y with yaw/height constrained by mechanics |
| **Normal-aware sampling** | Homogenizes plane normals to cut compute by ×3 while keeping accuracy |
| **Robust re-weighting** | Quality-aware plane scoring resists outliers & vibration noise |
| **Real-time on site** | 0.5 M pts ⟶ < 10 s on an Intel i7-10875H (Ubuntu 20.04 + ROS Noetic) |

---

## 🎥 Quick Demo
| Calibration | Full-body mapping |
|-------------|------------------|
| <a href="https://www.youtube.com/watch?v=0rQSlfYYI2I"><img src="https://img.youtube.com/vi/0rQSlfYYI2I/0.jpg" width="360"/></a> | <a href="https://www.youtube.com/watch?v=pZFQQkEqGFM"><img src="https://img.youtube.com/vi/pZFQQkEqGFM/0.jpg" width="360"/></a> |

---

## Table of Contents
- [LiMo-Calib 🚀](#limo-calib-)
  - [✨ Highlights](#-highlights)
  - [🎥 Quick Demo](#-quick-demo)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
  - [Getting Started](#getting-started)
  - [Dataset](#dataset)
  - [Benchmarks](#benchmarks)
  - [Citation](#citation)
  - [Acknowledgements](#acknowledgements)
  - [License](#license)

---

## Installation
```bash
# 1️⃣ clone inside a catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/kafeiyin00/LiMo-Calib.git

# 2️⃣ build
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release

# 3️⃣ source
source devel/setup.bash
```
> **Dependencies**: Ubuntu 20.04 · ROS Noetic · C++17 · Eigen > 3.3 · Ceres-Solver ≥ 2.0

---

## Getting Started
```bash
# visualize raw LiDAR & encoder data and convert ROS bag to TXT
 roslaunch Motor_LiDAR_Calib visualization.launch 

#  convert ROS bag to TXT NQZ
 python scripts/convertTXT2NP.py 

# run calibration
 python scripts/rotorlidar.py 
```


## Dataset
A 445.5 mb ROS bag for quick testing is available on Google Drive  
➡️ **[Download](https://drive.google.com/file/d/1rbo-CWAkS5xx5Cm7_qk3gq5FqfrQtN8j/view?usp=sharing)**  

After calibration, LiMo-Calib reduces average plane-fit error from _5 mm ➜ 0.1 mm_ across three complex sites (campus, vegetation, industrial). See full numbers in the paper, Table I.

---

## Benchmarks
| Metric | Original PCD | Vanilla (planar LS) | **LiMo-Calib (ours)** |
|--------|--------------|---------------------|-----------------------|
| Plane MSE ↓ | 5.09 × 10⁻³ m | 9.02 × 10⁻⁴ m | **4.39 × 10⁻⁴ m** |
| APE @ Fast-LIO ↓ | 4.84 m | 0.31 m | **0.09 m** |

*(measured on Site 2; full table in paper)*

---

## Citation
If you use LiMo-Calib, please cite:

```bibtex
@inproceedings{Li2025LiMoCalib,
  author    = {Jianping Li and Zhongyuan Liu and Xinhang Xu and Xiong Qin
               and Jinxin Liu and Shenghai Yuan and Xu Fang and Lihua Xie},
  title     = {{LiMo-Calib}: On-Site Fast LiDAR-Motor Calibration
               for Quadruped Robot-Based Panoramic 3D Sensing System},
  booktitle = {Proc.\ IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)},
  year      = {2025},
  note      = {\url{https://github.com/kafeiyin00/LiMo-Calib}}
}
```

---

## Acknowledgements
This research was conducted under project WP5 within the Delta-NTU Corporate Lab with funding support from A*STAR under its IAFICP programme (Grant no: I2201E0013) and Delta Electronics Inc. This work was supported by China-Singapore International Joint Research Institute (CSIJRI) Development Plan for the Technology Application Center (TAC) and National Research Foundation, Singapore, under its Medium-Sized Center for Advanced Robotics Technology Innovation. This work was partially supported by open project funding of Key Laboratory of Intelligent Control and Optimization for Industrial Equipment of Ministry of Education under Grant LICO2023YB01.
Special thanks to the Fast-LIO team and the open-source robotics community.

---

## License
LiMo-Calib is released under the **MIT License**. See [LICENSE](LICENSE) for details.

---

> _Maintainers_: Jianping Li - jianping.li@ntu.edu.sg · Zhongyuan Liu - zliu051@e.ntu.edu.sg  
> Found a bug? Open an [issue](https://github.com/kafeiyin00/LiMo-Calib/issues) or pull request!




Good luck with your IROS 2025 camera-ready release! 🎉
