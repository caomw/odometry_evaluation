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

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Evaluate odometry comparing to ground truth')
    parser.add_argument('filename',
            help='file with ground truth and estimated positions and velocities')
    args = parser.parse_args()

    data = pylab.loadtxt(args.filename)
    mean_dist_gt = 0;
    mean_dist_od = 0;
    for i in range(len(data) - 1):
        xdiff_gt = data[i,1] - data[i+1,1]
        ydiff_gt = data[i,2] - data[i+1,2]
        zdiff_gt = data[i,3] - data[i+1,3]
        xdiff_od = data[i,13] - data[i+1,13]
        ydiff_od = data[i,14] - data[i+1,14]
        zdiff_od = data[i,15] - data[i+1,15]
        mean_dist_gt += math.sqrt(xdiff_gt*xdiff_gt + ydiff_gt*ydiff_gt + zdiff_gt*zdiff_gt)
        mean_dist_od += math.sqrt(xdiff_od*xdiff_od + ydiff_od*ydiff_od + zdiff_od*zdiff_od)
    mean_dist_gt = mean_dist_gt / (len(data) - 1)
    mean_dist_od = mean_dist_od / (len(data) - 1)
    total_dist_gt = len(data) * mean_dist_gt
    total_dist_od = len(data) * mean_dist_od
    np.set_printoptions(precision=4)
    errors = data[:,19:25]-data[:,7:13]
    errors100 = errors/data[:,7:13]*100
    errors[:,3:6] = errors[:,3:6] / np.pi * 180.0
    t = np.sum(np.abs(data[:,7:10])**2,axis=-1)**(1./2)
    t_est = np.sum(np.abs(data[:,19:22])**2,axis=-1)**(1./2)
    t_err = t_est - t
    t_err100 = t_err/t*100
    r = data[:,10:13]
    r_est = data[:,22:25]
    yaw_err = calc_angle_diff(r[2], r_est[2])
    yaw_err100 = yaw_err/r[2]*100
    print chr(27)+"[0m"
    print "%i data samples." % len(data)
    print chr(27)+"[1;36m"+"Total distance in data (GT): "+chr(27)+"[0m", total_dist_gt
    print chr(27)+"[1;36m"+"Total distance in data (OD): "+chr(27)+"[0m", total_dist_od
    print chr(27)+"[1;36m"+"Average velocities in data (GT):\n"+chr(27)+"[0m", np.average(data[:,7:13], 0)
    print chr(27)+"[1;36m"+"Average velocities in data (OD):\n"+chr(27)+"[0m", np.average(data[:,19:25], 0)
    print chr(27)+"[1;36m"+"Average error (should be near zero)\n"+chr(27)+"[0m", np.average(errors, 0), np.std(errors, 0)
    print chr(27)+"[1;36m"+"Average absolute error\n"+chr(27)+"[0m", np.average(np.abs(errors), 0), np.std(np.abs(errors), 0)
    print chr(27)+"[1;36m"+"Error percentage (should be near zero)\n"+chr(27)+"[0m", np.average(errors100, 0), np.std(errors100, 0)
    print chr(27)+"[1;36m"+"Error percentage absolute error\n"+chr(27)+"[0m", np.average(np.abs(errors100), 0), np.std(np.abs(errors100), 0)
    print chr(27)+"[1;36m"+"Translation error:\n"+chr(27)+"[0m", np.average(t_err, 0), np.std(t_err, 0)
    print chr(27)+"[1;36m"+"Translation error %:\n"+chr(27)+"[0m", np.average(t_err100, 0), np.std(t_err100, 0)
    print chr(27)+"[1;36m"+"Rotational yaw error:\n"+chr(27)+"[0m", np.average(yaw_err, 0), np.std(yaw_err, 0)
    print chr(27)+"[1;36m"+"Rotational yaw error %:\n"+chr(27)+"[0m", np.average(yaw_err100, 0), np.std(yaw_err100, 0)

