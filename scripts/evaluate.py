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

def adapt(ground_truth, odometry):
    """
    finds first and last matching timestamps of given
    trajectories. Cuts both to match the same time
    frame and rebases both to start at zero.
    """
    eps = 0.005 # epsilon for equal time stamps
    gt_start = 0
    od_start = 0
    while ground_truth[gt_start][0] - odometry[od_start][0] < -eps and gt_start < len(ground_truth) - 1:
        gt_start = gt_start + 1
    while odometry[od_start][0] - ground_truth[gt_start][0] < -eps and od_start < len(odometry) - 1:
        od_start = 0
    gt_end = len(ground_truth) - 1
    od_end = len(odometry) - 1
    while ground_truth[gt_end][0] - odometry[od_end][0] > eps and gt_end > 0:
        gt_end = gt_end - 1
    while odometry[od_end][0] - ground_truth[gt_end][0] > eps and od_end > 0:
        od_end = od_end - 1

    if gt_start >= gt_end or od_start >= od_end:
        raise Error("no matching timestamps found in ground truth and odometry!")

    gt = ground_truth[gt_start:gt_end+1]
    od = odometry[od_start:od_end+1]
    return rebase(gt), rebase(od)

def calc_dist_xyz(data_point1, data_point2):
    xdiff = data_point1[1] - data_point2[1]
    ydiff = data_point1[2] - data_point2[2]
    zdiff = data_point1[3] - data_point2[3]
    return xdiff, ydiff, zdiff

def calc_dist(data_point1, data_point2):
    xdiff, ydiff, zdiff = calc_dist_xyz(data_point1, data_point2)
    return math.sqrt(xdiff*xdiff + ydiff*ydiff + zdiff*zdiff)

def calc_length(path):
    length = 0.0
    for i in range(len(path)-1):
        length = length + calc_dist(path[i], path[i+1])
    return length

def calc_rotation_amount(path):
    rotation = 0.0
    for i in range(len(path)-1):
        tf_start = to_transform(path[i])
        tf_end = to_transform(path[i+1])
        tf_delta = tf.concatenate_matrices(tf.inverse_matrix(tf_start), tf_end)
        angle, _, _ = tf.rotation_from_matrix(tf_delta)
        rotation = rotation + abs(angle)
    return rotation

def calc_stats(path):
#    path_length = calc_length(path)
#    rotation_amount = calc_rotation_amount(path)
    dt = path[-1][0] - path[0][0]
    tf_start = to_transform(path[0])
    tf_end = to_transform(path[-1])
    tf_delta = tf.concatenate_matrices(tf.inverse_matrix(tf_start), tf_end)
    roll, pitch, yaw = tf.euler_from_matrix(tf_delta)
    translation_vec = tf.translation_from_matrix(tf_delta)
    translation = np.linalg.norm(translation_vec)
    angle, direc, point = tf.rotation_from_matrix(tf_delta)
    return [
        translation_vec[0]/dt, translation_vec[1]/dt, translation_vec[2]/dt, 
        roll/dt, pitch/dt, yaw/dt,
        translation_vec[0], translation_vec[1], translation_vec[2], 
        roll, pitch, yaw,
        translation, angle ]

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

def find_subpaths_by_distance(path, path_length):
    """
    Searches sub-paths in given grond_truth with given length.
    Returns tuples to index the given ground_truth (start,end).
    end is the first index at which calc_dist(path[start], path[end])
    is bigger than path_length.
    """
    start_index = 0
    end_index = 0
    path_indices = []
    while start_index < len(path):
        while end_index < len(path) and calc_dist(
                path[start_index], path[end_index]) < path_length:
            end_index = end_index + 1
        if end_index == len(path):
            break;
        path_indices.append((start_index, end_index))
        start_index = start_index + 1
    return path_indices

def get_subpaths(path, path_indices):
    for (start, end) in path_indices:
        yield path[start:end]

def find_equal_paths(ground_truth, odometry, path_length):
    """
    Splits ground truth and finds equal odometry paths by matching
    timestamps. Matching subpaths are returned.
    """
    eps = 0.005 # epsilon for equal time stamps
    gt_paths = []
    od_paths = []
    prev_start = 0
    gt_path_indices = find_subpaths_by_distance(ground_truth, path_length)
    for path in get_subpaths(ground_truth, gt_path_indices):
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
            od_paths.append(odometry[start_index:end_index+1,:])
            prev_start = start_index
    assert(len(gt_paths) == len(od_paths))
    return gt_paths, od_paths

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
    parser.add_argument('outfile',
            help='output file for extracted stats')
    parser.add_argument('--path-length', type=float,
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

    ground_truth_paths, odometry_paths = \
        find_equal_paths(ground_truth, odometry, args.path_length)
    print "Found %i paths of %.2fm to compare" % (len(odometry_paths), args.path_length)

    with open(args.outfile, 'w') as outfile:
        outfile.write("#")
        for x in range(1,28):
            outfile.write("%6i" % x)
        outfile.write("\n")
        outfile.write("#     ts  "
                    "gt_x  gt_y  gt_z  gt_qx gt_qy gt_qz gt_qw "
                    "gt_vx gt_vy gt_vz gt_vr gt_vp gt_vy "
                    "od_x  od_y  od_z  od_qx od_qy od_qz od_qw "
                    "od_vx od_vy od_vz od_vr od_vp od_vy t\n")
        sys.stdout.write("\n")
        start_time = ground_truth_paths[0][0][0]
        print "Start time: ", start_time
        for i in range(len(ground_truth_paths)):
            sys.stdout.write("\r%.2f%%" % (100 * i / len(ground_truth_paths)))
            gt_path = ground_truth_paths[i]
            od_path = odometry_paths[i]
            gt_stats = calc_stats(gt_path)
            od_stats = calc_stats(od_path)
            outfile.write("%.9F " % gt_path[-1][0])
            for x in gt_path[-1][1:8]:
                outfile.write("%f " % x) 
            for v in gt_stats[0:6]:
                outfile.write("%f " % v)
            for x in od_path[-1][1:8]:
                outfile.write("%f " % x) 
            for v in od_stats[0:6]:
                outfile.write("%f " % v)
            outfile.write("%f" % (gt_path[-1][0] - start_time))
            outfile.write("\n")
        sys.stdout.write("\n")

#    true_translations = []
#    estimated_translations = []
#    errors = []
#    true_velocities = []
#    estimated_velocities = []
#    for i in range(len(ground_truth_paths)):
#        true_length, true_vel = calc_stats(ground_truth_paths[i])
#        estimated_length, est_vel = calc_stats(odometry_paths[i])
#        true_translations.append(true_length)
#        estimated_translations.append(estimated_length)
#        errors.append(abs(true_length - estimated_length))
#        true_velocities.append(true_vel)
#        estimated_velocities.append(est_vel)
#    plt.plot(true_translations)
#    plt.plot(estimated_translations)
#    plt.plot(errors)
#    plt.plot(true_velocities)
#    plt.plot(estimated_velocities)
#    plt.show()

#    import pdb; pdb.set_trace()
#    plot_paths(ground_truth, odometry)


