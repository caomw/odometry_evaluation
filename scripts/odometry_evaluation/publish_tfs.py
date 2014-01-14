#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_evaluation')
import rospy
import tf
import pylab

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
            description='Publish poses as tfs')
    parser.add_argument('input',
            help='input file with containing timestamp,x,y,z,qx,qy,qz,qw')
    parser.add_argument('-l',
            help='limit to this number of poses', type=int, default=10)
    args = parser.parse_args()

    poses = pylab.loadtxt(args.input)

    rospy.init_node('tf_poses_pub')
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        i = 0
        for pose in poses:
            if i >= args.l:
                break
            br.sendTransform((pose[1],pose[2],pose[3]),(pose[4],pose[5],pose[6],pose[7]),
                    rospy.Time.now(), ("cam%i" % i), "mosaic")
            i=i+1
        print "published %i poses" % i
        rospy.sleep(1)

