#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_evaluation')
import utils

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Sample odometry and ground truth equally.')
    parser.add_argument('ground_truth_file',
            help='file with ground truth positions')
    parser.add_argument('odometry_file',
            help='file with logged odometry')
    parser.add_argument('gt_outfile',
            help='output file for resampled ground truth')
    parser.add_argument('od_outfile',
            help='output file for resampled odometry')
    parser.add_argument('-outfile','--outfile',
            help='output file with data for plotting',
            default='outfile.txt')
    args = parser.parse_args()

    ground_truth, odometry = utils.load_data(args.ground_truth_file, args.odometry_file)
    ground_truth, odometry = utils.sample_equal(ground_truth, odometry)
    print "sampled", len(ground_truth), "GT /", len(odometry), "OD points:"
    ground_truth, odometry = utils.rebase(ground_truth), utils.rebase(odometry)

    # To insert into development kit of libviso2
    utils.write_pose_matrices(ground_truth, args.gt_outfile)
    utils.write_pose_matrices(odometry, args.od_outfile)

    # To use for plotting
    utils.write_joint_data(ground_truth, odometry, args.outfile)

