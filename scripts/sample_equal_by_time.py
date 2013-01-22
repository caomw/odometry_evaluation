#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_evaluation')
import utils

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
            help='step size of samples in seconds',
            default=1.0)
    args = parser.parse_args()

    ground_truth, odometry = utils.load_data(args.ground_truth_file, args.odometry_file)
    print "Loaded", len(ground_truth), "GT data points."
    print "Loaded", len(odometry), "OD data points."

    print "GT time frame: %.9F %.9F " % (ground_truth[0][0], ground_truth[-1][0])
    print "OD time frame: %.9F %.9F " % (odometry[0][0], odometry[-1][0])

    ground_truth, odometry = utils.sample_equal_by_time(ground_truth, odometry, args.sample_step)

    print "sampled", len(ground_truth), "points:"
    print "GT time frame: %.9F %.9F " % (ground_truth[0][0], ground_truth[-1][0])
    print "OD time frame: %.9F %.9F " % (odometry[0][0], odometry[-1][0])

    ground_truth, odometry = utils.rebase(ground_truth), utils.rebase(odometry)

    utils.write_joint_data(ground_truth, odometry, args.outfile)

