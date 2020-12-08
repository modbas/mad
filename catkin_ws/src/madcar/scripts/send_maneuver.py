#!/usr/bin/env python

#
# sends maneuver to madctrl
# Copyright (C) 2019, Frank Traenkle
# frank.traenkle@hs-heilbronn.de
#

import rospy
import sys
from madmsgs.srv import TrackGetWaypoints
from madmsgs.msg import DriveManeuver

def main():
  if len(sys.argv) > 1:
    vmax = float(sys.argv[1])
  else:
    vmax = 0.0
  print(vmax)
  # create ROS node
  rospy.init_node('send_maneuver')
  # wait for track_node
  rospy.wait_for_service('/mad/get_waypoints')
  # register service client
  getWayPoints = rospy.ServiceProxy('/mad/get_waypoints', TrackGetWaypoints)
  # register DriveManeuver publisher
  maneuverPub = rospy.Publisher("/mad/car0/navi/maneuver", DriveManeuver, queue_size=5, latch=True)
  # call /mad/get_waypoints service of track_node
  waypointsResp = getWayPoints(
    dx = 0.1, # sampling stepsize
    alpha = 0.5, # orthogonal coordinate of lane (0 = left, 1 = right, 0.5 = center)
    segmentSequence = [ 0 ] # get circular path containing segment 0
    #segmentSequence = [ 0, 1, 2, 3, 19, 20 ] # get path containing sequence of segments
    #segmentSequence = [ 11, 12, 13, 14, 15, 16, 17 ] # get path containing sequence of segments
    #segmentSequence = [ 16, 18, 19, 10, 11, 12, 13, 0, 1 ]
    )
  #rospy.loginfo('waypointsResp=%s', waypointsResp)
  # create DriveManeuver message
  maneuver = DriveManeuver(carid = 0,
    breaksLen = len(waypointsResp.breaks),
    breaks = waypointsResp.breaks,
    segments = waypointsResp.segments,
    s1 = waypointsResp.s1,
    s2 = waypointsResp.s2,
    splineCoefs1 = waypointsResp.splineCoefs1,
    splineCoefs2 = waypointsResp.splineCoefs2,
    periodic = True,
    vmax = vmax, # reference speed
    type = DriveManeuver.TYPE_PATHFOLLOW,
    #type = DriveManeuver.TYPE_PARK,
    #type = DriveManeuver.TYPE_HALT,
    xManeuverEnd = 1.0, # target parking position
    lapCount = 0,
    disableLaneMonitor = False
    )
  # publish maneuver message
  maneuverPub.publish(maneuver)
  rospy.sleep(2.0)

if __name__ == "__main__":
  main()
