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

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Evaluate odometry comparing to ground truth')
    parser.add_argument('filename',
            help='file with ground truth and estimated positions and velocities')
    args = parser.parse_args()

    data = pylab.loadtxt(args.filename)
    np.set_printoptions(precision=6)
    errors = data[:,19:25]-data[:,7:13]
    errors100 = errors/data[:,7:13]*100
    errors[:,3:6] = errors[:,3:6] / np.pi * 180.0
    print "Average error (should be near zero)\n", np.average(errors, 0), np.std(errors, 0)
    print "Average absolute error\n", np.average(np.abs(errors), 0), np.std(np.abs(errors), 0)
    print "Error percentage (should be near zero)\n", np.average(errors100, 0), np.std(errors100, 0)
    print "Error percentage absolute error\n", np.average(np.abs(errors100), 0), np.std(np.abs(errors100), 0)
    

