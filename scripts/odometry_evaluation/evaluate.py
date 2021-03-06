#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_evaluation')
import sys
import pylab
import math
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tf
import random
import numpy as np
import string
import utils

class Error(Exception):
    """ Base class for exceptions in this module. """
    pass

def angle_diff(ang1, ang2):
    diff = ang2 - ang1
    while diff < -np.pi:
        diff += 2*np.pi
    while diff > np.pi:
        diff -= 2*np.pi
    return diff

def calc_angle_diff(arr1, arr2):
    diffs = []
    for i in range(len(arr1)):
        diffs.append(angle_diff(arr1[i], arr2[i]))
    return np.array(diffs)

def evaluate(data, filter_string="True"):
    """
    Calculates and returns errors of input data.
    """

    # Re-build the data depending on the filter
    data = [row for row in data if eval(filter_string)]
    data = np.array(data)

    # Compute translation and rotation errors
    t = np.sum(np.abs(data[:,7:10])**2,axis=-1)**(1./2)
    t_est = np.sum(np.abs(data[:,19:22])**2,axis=-1)**(1./2)
    t_err = t_est - t
    r = data[:,10:13]
    r_est = data[:,22:25]
    yaw_err = calc_angle_diff(r[2], r_est[2])
    t_mae = np.average(np.abs(t_err), 0)
    yaw_mae = np.average(np.abs(yaw_err), 0)
    return ["{:10.6f}".format(t_mae), "{:10.6f}".format(yaw_mae)]

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Evaluate odometry comparing to ground truth')
    parser.add_argument('filename', nargs='+',
            help='file(s) with ground truth and estimated positions and velocities')
    args = parser.parse_args()

    # Evaluate each input file
    rows = []
    for filename in args.filename:
        data = pylab.loadtxt(filename)
        print "Loaded {} data points from {}".format(len(data), filename)
        this_errors = evaluate(data)
        this_errors_no_failures = evaluate(data, "row[25] == 0")
        rows.append([filename] + [len(data)] + this_errors + this_errors_no_failures)

    # Build the header for the output table
    header = [  "Filename", "Data Points", 
                "Trans. MAE (with failures)", "Yaw-Rot. MAE (with failures)", 
                "Trans. MAE (no failures)", "Yaw-Rot. MAE (no failures)"]

    utils.toRSTtable([header] + rows)

