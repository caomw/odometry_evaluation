#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_evaluation')
import pylab
import math
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tf
import random

def adapt(ground_truth, odometry):
    """
    finds first and last matching timestamps of given
    trajectories. Cuts both to match the same time
    frame and rebases both to start at zero.
    """
    eps = 0.005 # epsilon for equal time stamps
    gt_start = 0
    od_start = 0
    while ground_truth[gt_start][0] - odometry[od_start][0] < -eps and gt_start < len(ground_truth):
        gt_start = gt_start + 1
    while odometry[od_start][0] - ground_truth[gt_start][0] < -eps and od_start < len(odometry):
        od_start = 0
    gt_end = len(ground_truth) - 1
    od_end = len(odometry) - 1
    while ground_truth[gt_end][0] - odometry[od_end][0] > eps and gt_end > 0:
        gt_end = gt_end - 1
    while odometry[od_end][0] - ground_truth[gt_end][0] > eps and od_end > 0:
        od_end = od_end - 1

    if gt_start > gt_end or od_start > od_end:
        raise

    gt = ground_truth[gt_start:gt_end+1]
    od = odometry[od_start:od_end+1]
    return rebase(gt), rebase(od)


def calc_dist(data_point1, data_point2):
    xdiff = data_point1[1] - data_point2[1]
    ydiff = data_point1[2] - data_point2[2]
    zdiff = data_point1[3] - data_point2[3]
    return math.sqrt(xdiff*xdiff + ydiff*ydiff + zdiff*zdiff)

def calc_length(path):
    length = 0.0
    for i in range(len(path)-1):
        length = length + calc_dist(path[i], path[i+1])
    return length

def calc_stats(path):
    length = calc_length(path)
    dt = path[-1][0] - path[0][0]
    return length, length/dt

def split_into_paths(ground_truth, path_length):
    """
    Splits given ground truth into a set of paths with given length,
    starting at every point in the ground truth.
    """
    cumulative_distances = [0.0]
    total_length = 0.0
    for i in range(len(ground_truth)-1):
        distance = calc_dist(ground_truth[i], ground_truth[i+1])
        total_length = total_length + distance
        cumulative_distances.append(total_length)
    paths = []
    for i in range(len(ground_truth)):
        start_index = i
        end_index = i
        length = 0.0
        while length < path_length and end_index < len(ground_truth)-1:
            end_index = end_index + 1
            length = cumulative_distances[end_index] - cumulative_distances[start_index]
        if length >= path_length:
            paths.append(ground_truth[start_index:end_index,:])
    return paths

def find_equal_paths(ground_truth_paths, odometry):
    eps = 0.005 # epsilon for equal time stamps
    gt_paths = []
    odometry_paths = []
    prev_start = 0
    for path in ground_truth_paths:
        start_time = path[0][0]
        end_time = path[-1][0]
        assert(end_time > start_time)
        start_index = 0
        end_index = 0
        for i in range(prev_start, len(odometry)):
            if abs(odometry[i][0] - start_time) < eps:
                start_index = i
                break
        for i in range(start_index, len(odometry)):
            if abs(odometry[i][0] - end_time) < eps:
                end_index = i
                break
        if start_index > 0 and end_index > start_index:
            gt_paths.append(path)
            odometry_paths.append(odometry[start_index:end_index+1,:])
            prev_start = start_index
    return gt_paths, odometry_paths


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

def plot_paths(ground_truth, odometry):
    figure = plt.figure()
    axes = figure.gca(projection='3d')
    axes.plot(ground_truth[:,1], ground_truth[:,2], ground_truth[:,3], 
            label='Ground Truth')
    axes.plot(odometry[:,1], odometry[:,2], odometry[:,3], 
            label='Odometry')
    limits = axes.get_w_lims()
    data_ranges = [limits[1]-limits[0], limits[3]-limits[2], limits[5]-limits[4]];
    max_range = max(data_ranges)
    means = [(limits[i]+limits[i+1])/2 for i in range(0,6,2)]
    scales = [[mean-max_range/2, mean+max_range/2] for mean in means]
    axes.auto_scale_xyz(scales[0], scales[1], scales[2])
    axes.legend()
    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Evaluate odometry comparing to ground truth')
    parser.add_argument('ground_truth_file',
            help='file with ground truth positions')
    parser.add_argument('odometry_file',
            help='file with logged odometry')
    parser.add_argument('--path_length', type=float,
            help='length of the subpaths to examine',
            default=0.5)
    args = parser.parse_args()


    ground_truth = pylab.loadtxt(args.ground_truth_file)
    ground_truth = rebase(ground_truth)
    odometry = pylab.loadtxt(args.odometry_file, delimiter=',', skiprows=1, 
            usecols=(2,5,6,7,8,9,10,11))
    # odometry log is in nanoseconds, so we have to convert to seconds
    odometry[:,0] = odometry[:,0]/1e9

    print "Loaded", len(ground_truth), "GT data points."
    print "Loaded", len(odometry), "OD data points."

    print "GT time frame: %.9F %.9F " % (ground_truth[0][0], ground_truth[-1][0])
    print "OD time frame: %.9F %.9F " % (odometry[0][0], odometry[-1][0])
    ground_truth, odometry = adapt(ground_truth, odometry)
    print "adapted to:"
    print "GT time frame: %.9F %.9F " % (ground_truth[0][0], ground_truth[-1][0])
    print "OD time frame: %.9F %.9F " % (odometry[0][0], odometry[-1][0])

    print "Path length is", calc_length(ground_truth), "m."
    print "Odometry says", calc_length(odometry), "m."

    ground_truth_paths = split_into_paths(ground_truth, args.path_length)
    print "Split into",len(ground_truth_paths),"paths."

    ground_truth_paths, odometry_paths = find_equal_paths(ground_truth_paths, odometry)
    print "found",len(odometry_paths),"paths to compare"

    true_translations = []
    estimated_translations = []
    errors = []
    true_velocities = []
    estimated_velocities = []
    for i in range(len(ground_truth_paths)):
        true_length, true_vel = calc_stats(ground_truth_paths[i])
        estimated_length, est_vel = calc_stats(odometry_paths[i])
        true_translations.append(true_length)
        estimated_translations.append(estimated_length)
        errors.append(abs(true_length - estimated_length))
        true_velocities.append(true_vel)
        estimated_velocities.append(est_vel)
    plt.plot(true_translations)
    plt.plot(estimated_translations)
    plt.plot(errors)
    plt.plot(true_velocities)
    plt.plot(estimated_velocities)
    plt.show()

#    import pdb; pdb.set_trace()
#    plot_paths(ground_truth, odometry)


