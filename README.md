# LiMo-Calib

> **LiMo-Calib: On-Site Fast LiDAR-Motor Calibration for Quadruped Robot-Based Panoramic 3D Sensing System**<br/>
> Jianping Li, Zhongyuan Liu, Xinhang Xu, Xiong Qin, Jinxin Liu, Shenghai Yuan, Xu Fang, and Lihua Xie<br/>
> Nanyang Technological University, Dalian University of Technology<br/>
> *arXiv 2024*<br/>
> [**Full Paper**](https://arxiv.org/pdf/2502.12655) 

## TL;DR
Conventional single LiDAR systems are inherently constrained by their limited field of view (FoV), leading to blind spots and incomplete environmental awareness, particularly on robotic platforms with strict payload limitations. Integrating a motorized LiDAR offers a practical solution by significantly expanding the sensor’s FoV and enabling adaptive panoramic 3D sensing. However, the high-frequency vibrations of the quadruped robot introduce calibration challenges, causing variations in the LiDAR-motor transformation that degrade sensing accuracy. Existing calibration methods that use artificial targets or dense feature extraction lack feasibility for on-site applications and real-time implementation. To overcome these limitations, we propose LiMo-Calib, an efficient on-site calibration method that eliminates the need for external targets by leveraging geometric features directly from raw LiDAR scans. LiMo-Calib optimizes feature selection based on normal distribution to accelerate convergence while maintaining accuracy and incorporates a reweighting mechanism that evaluates local plane fitting quality to enhance robustness. We integrate and validate the proposed method on a motorized LiDAR system mounted on a quadruped robot, demonstrating significant improvements in calibration efficiency and 3D sensing accuracy, making LiMo-Calib well-suited for real-world robotic applications. We further demonstrate the accuracy improvements of the LIO on the panoramic 3D sensing system using the calibrated parameters.

## Video Introduction

[![Sites](https://img.youtube.com/vi/0rQSlfYYI2I/maxresdefault.jpg)](https://www.youtube.com/watch?v=0rQSlfYYI2I)

## Calibration Sites

[![Sites](https://img.youtube.com/vi/FMINa-sap7g/maxresdefault.jpg)](https://www.youtube.com/watch?v=FMINa-sap7g)

## System Validation
[![Validation](https://img.youtube.com/vi/pZFQQkEqGFM/maxresdefault.jpg)](https://www.youtube.com/watch?v=pZFQQkEqGFM)
