import json
import numpy as np

def calibration_read(fname):
    with open(fname, "r") as f:
        mtx = json.load(f)
    return np.array(mtx)

def calibration_write(fname, mtx):
    with open(fname, "w") as f:
        json.dump(mtx.tolist(), f, indent=4)