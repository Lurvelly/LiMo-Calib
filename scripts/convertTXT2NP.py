import os
import sys
import numpy as np
import json


rotlidarpath = "/home/leo/workspace/data/motor_calibration_data/a.txt"
rotlidarpath_npy = "/home/leo/workspace/data/motor_calibration_data/a.npz"

rawdata = np.loadtxt(rotlidarpath)
np.savez(rotlidarpath_npy,rawdata)