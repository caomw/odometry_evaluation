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
    print "%i data samples." % len(data)
    print "Average velocities in data (GT):\n", np.average(data[:,7:13], 0)
    print "Average velocities in data (OD):\n", np.average(data[:,19:25], 0)
    print "Average error (should be near zero)\n", np.average(errors, 0), np.std(errors, 0)
    print "Average absolute error\n", np.average(np.abs(errors), 0), np.std(np.abs(errors), 0)
    print "Error percentage (should be near zero)\n", np.average(errors100, 0), np.std(errors100, 0)
    print "Error percentage absolute error\n", np.average(np.abs(errors100), 0), np.std(np.abs(errors100), 0)
    print "Translation error:\n", np.average(t_err, 0), np.std(t_err, 0)
    print "Translation error %:\n", np.average(t_err100, 0), np.std(t_err100, 0)
    print "Rotational yaw error:\n", np.average(yaw_err, 0), np.std(yaw_err, 0)
    print "Rotational yaw error %:\n", np.average(yaw_err100, 0), np.std(yaw_err100, 0)

