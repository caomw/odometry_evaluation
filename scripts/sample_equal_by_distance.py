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
import utils

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Evaluate odometry comparing to ground truth',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('ground_truth_file',
            help='file with ground truth positions')
    parser.add_argument('odometry_file',
            help='file with logged odometry')
    parser.add_argument('outfile',
            help='output file for extracted stats')
    parser.add_argument('-s','--sample-step', type=float,
            help='step size of samples in meters',
            default=1.0)
    args = parser.parse_args()

    ground_truth, odometry = utils.load_data(args.ground_truth_file, args.odometry_file)
    ground_truth, odometry = utils.sample_equal(ground_truth, odometry)
    print "sample_equal: ", len(ground_truth), "GT /", len(odometry), "OD points:"
    ground_truth, odometry = utils.rebase(ground_truth), utils.rebase(odometry)

    ground_truth, odometry = utils.sample_equal_by_distance(ground_truth, odometry, args.sample_step)
    print "sample_equal_by_distance: ", len(ground_truth), "GT /", len(odometry), "OD points:"

    utils.write_joint_data(ground_truth, odometry, args.outfile)

