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


def trajectory_distances(data):
	"""
	Function to compute the trajectory distances from a dataset where
	each row contains the transformation matrix.
	"""
	dist = []
	dist.append(0)
	for i in range(len(data) - 1):
		p1 = data[i, :]
		p2 = data[i + 1, : ]
		dx = p1[3] - p2[3]
		dy = p1[7] - p2[7]
		dz = p1[11] - p2[11]
		dist.append(dist[i] + math.sqrt(dx*dx + dy*dy + dz*dz))
	return dist

def last_frame_from_segment_length(dist, first_frame, length):
	"""
	Function to compute the trajectory distances from a dataset where
	each row contains the transformation matrix.
	"""
	for i in range(first_frame, len(dist) - 1, 1):
		#print dist[i], " | ", dist[first_frame], " | ", length
		if (dist[i] > dist[first_frame] + length):
			return i
	return -1

def get_tf_matrix(data):
	"""
	Construct the transformation matrix from row of data
	"""
	M = np.array(((data[0],  data[1],  data[2],  data[3]),
	              (data[4],  data[5],  data[6],  data[7]),
	              (data[8],  data[9],  data[10], data[11]),
	              (data[12], data[13], data[14], data[15])), dtype=np.float64)
	return M

def rotationError(pose_error):
    a = pose_error[0,0]
    b = pose_error[1,1]
    c = pose_error[2,2]
    d = 0.5*(a+b+c-1.0)
    return math.acos(max(min(d,1),-1))

def translationError(pose_error):
    dx = pose_error[0,3]
    dy = pose_error[1,3]
    dz = pose_error[2,3]
    return math.sqrt(dx*dx+dy*dy+dz*dz)

def calc_seq_errors(data_gt, data_odom):
	"""
	Function to compute the sequence errors from ground truth and odometry results.
	"""
	# Initializations
	frames_err = []
	len_err = []
	speed_err = []
	r_err = []
	t_err = []

	# Parameters
	step_size = 10
	lengths = [1,2,5,10,20,30,40,50]

	# Compute the vector of trajectory distances
	dist = trajectory_distances(data_gt)

	for first_frame in range(0, len(data_gt) - 1, step_size):
		for i in range(len(lengths) - 1):
			length = lengths[i]
			last_frame = last_frame_from_segment_length(dist, first_frame, length)

			# Continue, if sequence not long enough
			if (last_frame == -1):
				continue

			M_gt_inv = tf.inverse_matrix(get_tf_matrix(data_gt[first_frame,:]))
			M_gt = get_tf_matrix(data_gt[last_frame,:])
			M_od_inv = tf.inverse_matrix(get_tf_matrix(data_odom[first_frame,:]))
			M_od = get_tf_matrix(data_odom[last_frame,:])
			pose_delta_gt = np.dot(M_gt_inv.reshape((4,4)), M_gt.reshape((4,4)))
			pose_delta_od = np.dot(M_od_inv.reshape((4,4)), M_od.reshape((4,4)))
			pose_error    = np.dot(tf.inverse_matrix(pose_delta_od), pose_delta_gt)

			num_frames = last_frame - first_frame + 1
      		speed = length / (0.1 * num_frames)

			# Translational and rotational errors
			frames_err.append(first_frame)
			r_err.append(rotationError(pose_error) / length)
			t_err.append(translationError(pose_error) / length)
			len_err.append(length)
			speed_err.append(speed)

	return frames_err, r_err, t_err, len_err, speed_err

def save_error_plots():
	"""
	Function to compute the sequence errors from ground truth and odometry results.
	"""
	# TODO


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='This script evaluates the viso2 algorithm following the code of development kit presented in: http://www.cvlibs.net/datasets/kitti/eval_odometry.php')
    parser.add_argument('ground_truth_file',
            help='file with ground truth positions')
    parser.add_argument('odometry_file',
            help='file with logged odometry')
    args = parser.parse_args()

    ground_truth, odometry = utils.load_data(args.ground_truth_file, args.odometry_file)
    ground_truth, odometry = utils.sample_equal(ground_truth, odometry)
    ground_truth, odometry = utils.rebase(ground_truth), utils.rebase(odometry)
    
    calc_seq_errors(ground_truth, odometry)


