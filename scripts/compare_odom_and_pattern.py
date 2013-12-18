#!/usr/bin/env python
import sys
import pylab
import math
import numpy as np
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D

colors = ['r','b','y','c','m','k']

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def vector_dist(v1, v2):
  error = np.sqrt( (v1[1]-v2[1])*(v1[1]-v2[1]) + (v1[2]-v2[2])*(v1[2]-v2[2]) + (v1[3]-v2[3])*(v1[3]-v2[3]))
  return error

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
          description='Plot 3D graphics of odometry data files in real time.',
          formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('odometry_file', 
          help='file with visual odometry')
  parser.add_argument('pattern_pose_file', 
          help='file with the patter pose')
  args = parser.parse_args()

  # Init figure
  fig = pylab.figure(1)
  ax = Axes3D(fig)
  ax.grid(True)
  ax.set_title("Odometry viewer")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")
    
  # Load data
  odom = pylab.loadtxt(args.odometry_file, delimiter=',', skiprows=1, usecols=(2,5,6,7))
  pattern = pylab.loadtxt(args.pattern_pose_file, delimiter=',', skiprows=1, usecols=(2,4,5,6))

  # Correct odometry time if needed
  if (odom[0,0] < 9999999999):
    odom[:,0] = odom[:,0] * 1000000000

  # Search the best time matching
  min_time_diff = 999999999
  min_time_idx = -1
  for i in range(len(odom)):
    time_diff = np.fabs(odom[i,0] - pattern[0,0])
    if (time_diff < min_time_diff):
      min_time_diff = time_diff
      min_time_idx = i

  if (min_time_idx == -1):
    print "Impossible to find time matchings between pattern poses and odometry"
    # sys.exit()

  # Cut the odometry file
  tmp = []
  for i in range(min_time_idx, len(odom)):
    tmp.append(odom[i,:])
  odom = np.array(tmp)

  # Correct odometry
  initial = odom[0,:]-pattern[0,:]
  initial = np.array(initial)
  odom_corrected = odom - initial

  # Plot odometry
  ax.plot(odom_corrected[:,1], odom_corrected[:,2], odom_corrected[:,3], 'g', label='Visual Odometry')

  # Search for correspondences between visual odometry and pattern poses
  idx_color = 0;
  min_time_diff = 999999999
  min_time_idx = -1
  for j in range(len(odom_corrected)):
    time_diff = np.fabs(odom[j,0] - pattern[len(pattern)-1,0])
    if (time_diff < min_time_diff):
      min_time_diff = time_diff
      min_time_idx = j

  if (min_time_idx == -1):
    print "Impossible to find odometry time matchings for last pattern pose."
  else:
    # Compute the error
    error = vector_dist(odom_corrected[min_time_idx,:], pattern[len(pattern)-1,:])
    print "Error: ", error

    # Print the matching
    vect = []
    vect.append([odom_corrected[min_time_idx,1], odom_corrected[min_time_idx,2], odom_corrected[min_time_idx,3]])
    vect.append([pattern[len(pattern)-1,1], pattern[len(pattern)-1,2], pattern[len(pattern)-1,3]])
    vect = np.array(vect)
    ax.plot(vect[:,0], vect[:,1], vect[:,2], colors[idx_color], marker='o', linestyle='--')

    idx_color = idx_color + 1
    if (idx_color >= len(colors)):
      idx_color = 0;

  pylab.show()