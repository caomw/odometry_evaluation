#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_evaluation')
import sys
import pylab
import math
import tf
import tf.transformations as tf
import random
import numpy as np
import string

class Error(Exception):
    """ Base class for exceptions in this module. """
    pass

def trajectory_distances(data):
    """
    Function to compute the trajectory distances from a dataset where
    each row contains the transformation matrix.
    """
    dist = []
    dist.append(0)
    for i in range(len(data) - 1):
        dist.append(dist[i] + calc_dist(data[i, :], data[i + 1, : ]))
    return dist

def calc_dist_xyz(data_point1, data_point2):
    xdiff = data_point1[1] - data_point2[1]
    ydiff = data_point1[2] - data_point2[2]
    zdiff = data_point1[3] - data_point2[3]
    return xdiff, ydiff, zdiff

def calc_dist(data_point1, data_point2):
    xdiff, ydiff, zdiff = calc_dist_xyz(data_point1, data_point2)
    return math.sqrt(xdiff*xdiff + ydiff*ydiff + zdiff*zdiff)

def angle_diff(ang1, ang2):
    diff = ang2 - ang1
    while diff < -np.pi:
        diff += 2*np.pi
    while diff > np.pi:
        diff -= 2*np.pi
    return diff

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
    print "looked for", timestamp, "found two samples at", ts0, ts1, "using fraction", fraction
    q0 = [pose0[4], pose0[5], pose0[6], pose0[7]]
    q1 = [pose1[4], pose1[5], pose1[6], pose1[7]]
    q = tf.quaternion_slerp(q0, q1, fraction)
    t = pose0[1:4] + fraction * (pose1[1:4]-pose0[1:4])
    return [timestamp, t[0], t[1], t[2], q[0], q[1], q[2], q[3]]

def sample_equal_by_time(ground_truth, odometry, sample_step):
    start_time = max(ground_truth[0,0], odometry[0,0])
    end_time = min(ground_truth[-1,0], odometry[-1,0])
    sample_time = start_time + sample_step
    gt_samples = []
    od_samples = []
    while sample_time < end_time:
        gt_samples.append(get_pose_sample(ground_truth, sample_time))
        od_samples.append(get_pose_sample(odometry, sample_time))
        sample_time = sample_time + sample_step
    return np.array(gt_samples), np.array(od_samples)

def sample_equal_by_distance(ground_truth, odometry, sample_step):
    
    # Init output vectors
    gt_samples = []
    od_samples = []
    gt_samples.append(ground_truth[0,:]);
    od_samples.append(odometry[0,:]);

    # Compute the trajectory distance increments
    dist = trajectory_distances(ground_truth)

    i = 0
    while i < len(ground_truth):
        for n in range(i, len(ground_truth) - 1):
            if (dist[n] - dist[i] > sample_step):
                # Save samples
                gt_samples.append(ground_truth[n,:]);
                od_samples.append(odometry[n,:]);
                i = n - 1
                break
        i = i + 1;

    return gt_samples, od_samples

def sample_equal(ground_truth, odometry, detect_failures = False):
    """ 
    Samples ground truth and odometry searching for
    matching timestamps without any limitations of
    time or distance. 
    In most cases that means that some ground truth
    samples are sorted out as the odometry is not fast
    enough.
    """
    eps = 0.005
    gt_index = 0
    od_index = 0
    gt_samples = []
    od_samples = []

    try:
        while True:
            failure = False
            while odometry[od_index,0] - ground_truth[gt_index,0] > eps:
                gt_index = gt_index + 1
            while ground_truth[gt_index,0] - odometry[od_index,0] > eps:
                od_index = od_index + 1
                if (odometry[od_index,14] == 1):
                    failure = True

            if abs(ground_truth[gt_index,0] - odometry[od_index,0]) < eps:
                # save previous failures in the current sample
                if (detect_failures and failure):
                    odometry[od_index,14] = 1

                # save
                gt_samples.append(ground_truth[gt_index])
                od_samples.append(odometry[od_index])
                od_index = od_index + 1
                gt_index = gt_index + 1

    except IndexError:
        pass

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
            usecols=(2,5,6,7,8,9,10,11,48,49,50,51,52,53,12))
    # odometry log is in nanoseconds, so we have to convert to seconds
    odometry[:,0] = odometry[:,0]/1e9
    odometry[:,14] = odometry[:,14] > 9990
    print "Loaded {} GT and {} OD poses. Odometry failed {} times.".format(len(ground_truth),
            len(odometry), int(np.sum(odometry[:,14])))
    return ground_truth, odometry

def write_joint_data(ground_truth, odometry, filename):
    eps = 0.005
    assert(len(ground_truth) == len(odometry))
    with open(filename, 'w') as outfile:
        outfile.write("#")
        for x in range(1,27):
            outfile.write("%6i" % x)
        outfile.write("\n")
        outfile.write("#     t  "
                    "gt_x  gt_y  gt_z  gt_ro gt_pi gt_ya "
                    "gt_vx gt_vy gt_vz gt_vr gt_vp gt_vy "
                    "od_x  od_y  od_z  od_ro od_pi od_ya "
                    "od_vx od_vy od_vz od_vr od_vp od_vy od_failure ts\n")
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
            outfile.write("%.9F " % odometry[i][14])
            outfile.write("%.9F " % ground_truth[i][0])
            outfile.write("\n")
        sys.stdout.write("\n")            

def write_pose_matrices(data, filename):
    with open(filename, 'w') as outfile:
        for d in data:
            pose = to_transform(d)
            for num in pose.flatten().tolist()[:12]:
               outfile.write("%f " % num)
            outfile.write("\n")

def toRSTtable(rows, header=True, vdelim="  ", padding=1, justify='right'):
    """
    Outputs a list of lists as a Restructured Text Table
    - rows - list of lists
    - header - if True the first row is treated as a table header
    - vdelim - vertical delimiter between columns
    - padding - nr. of spaces are left around the longest element in the column
    - justify - may be left, center, right
    """
    border="=" # character for drawing the border
    justify = {'left':string.ljust,'center':string.center,'right':string.rjust}[justify.lower()]

    # calculate column widhts (longest item in each col
    # plus "padding" nr of spaces on both sides)
    cols = zip(*rows)
    colWidths = [max([len(str(item))+2*padding for item in col]) for col in cols]

    # the horizontal border needed by rst
    borderline = vdelim.join([w*border for w in colWidths])

    # outputs table in rst format
    output = ""
    output += borderline + "\n"
    for row in rows:
        output += vdelim.join([justify(str(item),width) for (item,width) in zip(row,colWidths)])
        output += "\n"
        if header: output += borderline + "\n"; header=False
    output += borderline + "\n"
    print output
    return output