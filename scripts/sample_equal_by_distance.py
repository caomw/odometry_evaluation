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

def get_pose_sample(data, timestamp):
    start_time = data[0,0]
    end_time = data[-1,0]
    if timestamp < start_time or timestamp > end_time:
        raise Error("sample time exeeds data coverage")
    index = 0
    while data[index,0] < timestamp:
        index = index + 1
    pose0 = data[index-1]
    pose1 = data[index]
    ts0 = pose0[0]
    ts1 = pose1[0]
    fraction = (timestamp-ts0)/(ts1-ts0)
    q0 = [pose0[4], pose0[5], pose0[6], pose0[7]]
    q1 = [pose1[4], pose1[5], pose1[6], pose1[7]]
    q = tf.quaternion_slerp(q0, q1, fraction)
    t = pose0[1:4] + fraction * (pose1[1:4]-pose0[1:4])
    return [timestamp, t[0], t[1], t[2], q[0], q[1], q[2], q[3]]

def calc_dist_xyz(data_point1, data_point2):
    xdiff = data_point1[1] - data_point2[1]
    ydiff = data_point1[2] - data_point2[2]
    zdiff = data_point1[3] - data_point2[3]
    return xdiff, ydiff, zdiff

def calc_dist(data_point1, data_point2):
    xdiff, ydiff, zdiff = calc_dist_xyz(data_point1, data_point2)
    return math.sqrt(xdiff*xdiff + ydiff*ydiff + zdiff*zdiff)

def sample_equal(ground_truth, odometry, sample_step):
    gt_index = 0
    while ground_truth[gt_index,0] < odometry[0,0]:
        gt_index = gt_index + 1
    gt_samples = []
    od_samples = []
    start_time = ground_truth[gt_index,0]
    gt_samples.append(get_pose_sample(ground_truth, start_time))
    od_samples.append(get_pose_sample(odometry, start_time))

    next_index = gt_index
    subpaths_left = True
    while subpaths_left:
        while calc_dist(ground_truth[gt_index], ground_truth[next_index]) < sample_step:
            next_index = next_index + 1
            if next_index >= len(ground_truth):
                subpaths_left = False
                break
        if subpaths_left:
            stamp = ground_truth[next_index,0]
            gt_samples.append(get_pose_sample(ground_truth, stamp))
            od_samples.append(get_pose_sample(odometry, stamp))
        gt_index = next_index
    return np.array(gt_samples), np.array(od_samples)

def calc_tf_vel(point1, point2):
    dt = point2[0] - point1[0]
    tf_start = to_transform(point1)
    tf_end = to_transform(point2)
    tf_delta = tf.concatenate_matrices(tf.inverse_matrix(tf_start), tf_end)
    roll, pitch, yaw = tf.euler_from_matrix(tf_start)
    droll, dpitch, dyaw = tf.euler_from_matrix(tf_delta)
    translation_vec = tf.translation_from_matrix(tf_delta)
#    translation = np.linalg.norm(translation_vec)
#    angle, direc, point = tf.rotation_from_matrix(tf_delta)
    return [
        point1[1], point1[2], point1[3], 
        roll, pitch, yaw,
        translation_vec[0]/dt, translation_vec[1]/dt, translation_vec[2]/dt, 
        droll/dt, dpitch/dt, dyaw/dt ]
#        translation_vec[0], translation_vec[1], translation_vec[2], 
#        roll, pitch, yaw,
#        translation, angle ]


def to_transform(data_point):
    t = [data_point[1], data_point[2], data_point[3]]
    q = [data_point[4], data_point[5], data_point[6], data_point[7]]
    rot_mat = tf.quaternion_matrix(q)
    trans_mat = tf.translation_matrix(t)
    return tf.concatenate_matrices(trans_mat, rot_mat)

def rebase(path):
    init_pose = to_transform(path[0])
    init_pose_inv = tf.inverse_matrix(init_pose)
    for data_point in path:
        pose = to_transform(data_point)
        new_pose = tf.concatenate_matrices(init_pose_inv, pose)
        q = tf.quaternion_from_matrix(new_pose)
        t = tf.translation_from_matrix(new_pose)
        data_point[1], data_point[2], data_point[3] = t[0], t[1], t[2]
        data_point[4], data_point[5], data_point[6], data_point[7] = q[0], q[1], q[2], q[3]
    return path

def load_data(ground_truth_file, odometry_file):
    ground_truth = pylab.loadtxt(ground_truth_file)
    odometry = pylab.loadtxt(odometry_file, delimiter=',', skiprows=1, 
            usecols=(2,5,6,7,8,9,10,11,48,49,50,51,52,53))
    # odometry log is in nanoseconds, so we have to convert to seconds
    odometry[:,0] = odometry[:,0]/1e9
    return ground_truth, odometry

def write_joint_data(ground_truth, odometry, filename):
    eps = 0.005
    assert(len(ground_truth) == len(odometry))
    with open(filename, 'w') as outfile:
        outfile.write("#")
        for x in range(1,26):
            outfile.write("%6i" % x)
        outfile.write("\n")
        outfile.write("#     t  "
                    "gt_x  gt_y  gt_z  gt_ro gt_pi gt_ya "
                    "gt_vx gt_vy gt_vz gt_vr gt_vp gt_vy "
                    "od_x  od_y  od_z  od_ro od_pi od_ya "
                    "od_vx od_vy od_vz od_vr od_vp od_vy   ts\n")
        sys.stdout.write("\n")
        start_time = ground_truth[0][0]
        for i in range(len(ground_truth)-1):
            time_diff = ground_truth[i][0] - odometry[i][0]
            if (abs(time_diff) > eps):
                raise Error("ground truth and odometry differ by %f secs."
                        % time_diff)
            sys.stdout.write("\r%.2f%%" % (100 * i / len(ground_truth)))
            outfile.write("%.9F " % (ground_truth[i][0] - start_time))
            for x in calc_tf_vel(ground_truth[i], ground_truth[i+1]):
                outfile.write("%f " % x) 
            for x in calc_tf_vel(odometry[i], odometry[i+1]):
                outfile.write("%f " % x) 
            outfile.write("%.9F " % ground_truth[i][0])
            outfile.write("\n")
        sys.stdout.write("\n")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Evaluate odometry comparing to ground truth')
    parser.add_argument('ground_truth_file',
            help='file with ground truth positions')
    parser.add_argument('odometry_file',
            help='file with logged odometry')
    parser.add_argument('outfile',
            help='output file for extracted stats')
    parser.add_argument('--sample-step', type=float,
            help='step size of samples in meters',
            default=1.0)
    args = parser.parse_args()

    ground_truth, odometry = load_data(args.ground_truth_file, args.odometry_file)
    print "Loaded", len(ground_truth), "GT data points."
    print "Loaded", len(odometry), "OD data points."

    print "GT time frame: %.9F %.9F " % (ground_truth[0][0], ground_truth[-1][0])
    print "OD time frame: %.9F %.9F " % (odometry[0][0], odometry[-1][0])

    ground_truth, odometry = sample_equal(ground_truth, odometry, args.sample_step)

    print "sampled", len(ground_truth), "points:"
    print "GT time frame: %.9F %.9F " % (ground_truth[0][0], ground_truth[-1][0])
    print "OD time frame: %.9F %.9F " % (odometry[0][0], odometry[-1][0])

    ground_truth, odometry = rebase(ground_truth), rebase(odometry)

    write_joint_data(ground_truth, odometry, args.outfile)

