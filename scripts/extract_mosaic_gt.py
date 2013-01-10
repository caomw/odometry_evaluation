#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_evaluation')
import sys
import pylab
import math
import tf
import tf.transformations as tf
import random
import numpy as np
import scipy.io
import os
import subprocess

def convert(path, scale):
    new_poses = []
    for pose in path:
        # after long discussion with Ricard Campos we found out
        # that his rotation is coded as Yaw, Pitch, Roll in a rotating frame
        roll, pitch, yaw = pose[4], pose[5], pose[6]
        xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
        Rx = tf.rotation_matrix(np.pi, xaxis)
        R = tf.concatenate_matrices(Rx, tf.euler_matrix(yaw, pitch, roll, 'rzyx'))
        # q = tf.quaternion_from_euler(yaw, pitch, roll, 'rzyx')
        q = tf.quaternion_from_matrix(R)
        new_poses.append([pose[0], scale*pose[1], scale*pose[2], scale*pose[3], q[0], q[1], q[2], q[3]])
    return np.array(new_poses);

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Evaluate odometry comparing to ground truth')
    parser.add_argument('input',
            help='input file from matlab')
    parser.add_argument('output',
            help='output file with quaternions')
    parser.add_argument('--scale',
            help='scale applied to xyz', type=float, default=1.0)
    args = parser.parse_args()

    # convert from matlab as this does not work on 64-32bit change
    # mat = scipy.io.loadmat(args.input)
    cmd = ['octave', '-q', os.path.dirname(__file__) + "/extract_mosaic_data.m", args.input, '/tmp/pose_data.txt']
    subprocess.check_call(cmd)
    poses = pylab.loadtxt('/tmp/pose_data.txt')
    poses = convert(poses, args.scale)
    with open(args.output, 'w') as outfile:
        for i in range(len(poses)):
            outfile.write("%.9F " % poses[i][0])
            for x in poses[i][1:8]:
                outfile.write("%f " % x)
            outfile.write("\n")
        print "Written", len(poses), "poses to", args.output, "."

