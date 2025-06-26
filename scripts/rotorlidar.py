import os
import sys
import numpy as np
import json
import math
import time  

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from matplotlib import cm
import seaborn as sns

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
lib_direcotry = current_directory+"/../../../devel/lib"  # ugly but useful
sys.path.append(lib_direcotry)
print(lib_direcotry)

import pyCalib


def showpts(pts):
    # build a new matplotlib figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pts[:,0], pts[:,1], pts[:,2])
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('3D Scatter Plot')
    plt.show()
    
def projection(data):
    # Initial (x y z) Raw (x y z) intensity angle time
    xyz_l = data[:,3:6]
    rotor = data[:,7]
    
def cartesian_to_polar(normals):
    """Convert normalized cartesian coordinates to polar coordinates (theta, phi)"""
    theta = np.arccos(normals[:, 2])  # theta from z-axis [0, pi]
    phi = np.arctan2(normals[:, 1], normals[:, 0])  # phi in x-y plane [-pi, pi]
    return np.column_stack((theta, phi))

def visualize_normal_distribution(normals, title, output_path, before_after="before"):
    """Visualize normal vector distribution in both 3D and 2D polar projection"""
    fig = plt.figure(figsize=(15, 6))
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(normals[:, 0], normals[:, 1], normals[:, 2], alpha=0.6, s=1)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title(f'3D Normal Distribution ({before_after} sampling)')
    
    ax2 = fig.add_subplot(122)
    polar_coords = cartesian_to_polar(normals)
    theta_edges = np.linspace(0, np.pi, 37)
    phi_edges = np.linspace(-np.pi, np.pi, 73)
    H, xedges, yedges = np.histogram2d(polar_coords[:, 0], polar_coords[:, 1],
                                      bins=[theta_edges, phi_edges])
    im = ax2.imshow(H.T, extent=[0, np.pi, -np.pi, np.pi], 
                    aspect='auto', cmap='viridis', origin='lower')
    plt.colorbar(im, ax=ax2, label='Count')
    ax2.set_xlabel('θ (radians)')
    ax2.set_ylabel('φ (radians)')
    ax2.set_title(f'2D Polar Projection ({before_after} sampling)')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()

def main():


    rotlidarpath_npz = "/home/leo/workspace/data/motor_calibration_data/c.npz"
    rawdata = np.load(rotlidarpath_npz)
    rawdata = rawdata['arr_0']
    
    # 1. Save the original point cloud (a.pcd) using initial extrinsic parameters
    initial_params = [0,-math.radians(60), 0, 0, 0, 0, 0]
    pyCalib.ProjectionUsingCalibParam("/home/leo/workspace/data/motor_calibration_data/a.pcd",
                                      initial_params, rawdata[0:500000, :])
    print("Saved original point cloud: a.pcd")
    
    # 2. Optimization of hte calibration parameters and record the execution time
    start_time = time.perf_counter()
    accurateValues = pyCalib.Calib(initial_params, rawdata[0:500000, :], 1, 1.0)
    elapsed = time.perf_counter() - start_time
    print("Calib returned parameters:", accurateValues)
    print(f"Calib took {elapsed:.3f} seconds")
    pyCalib.ProjectionUsingCalibParam("/home/leo/workspace/data/motor_calibration_data/a_fine_v1_p0.3.pcd",
                                      accurateValues, rawdata[0:500000, :])
    print("Saved fine calibrated point cloud: a_fine.pcd")
    
    # 3. Re-weighted optimization: Record the execution time of the Calib_reweight function
    start_time = time.perf_counter()
    reweightedValues = pyCalib.Calib_reweight(initial_params, rawdata[0:500000, :], 1, 2.0)
    elapsed = time.perf_counter() - start_time
    print("Calib_reweight returned parameters:", reweightedValues)
    print(f"Calib_reweight took {elapsed:.3f} seconds")
    
    pyCalib.ProjectionUsingCalibParam("/home/leo/workspace/data/motor_calibration_data/c_fine_reweighted_v2.0_p0.7.pcd",
                                       reweightedValues, rawdata[0:500000, :])
    print("Saved reweighted calibrated point cloud: a_fine_reweighted.pcd")

if __name__ == '__main__':
    main()
