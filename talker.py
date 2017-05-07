#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from math import *
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import dubins
from nav_msgs.msg import Odometry
import tf


def talker():

    #sequence = []

    #car_pose = Pose2D()
    #car_pose.x  = 0.0
    #car_pose.y = 0.0
    #car_pose.theta = 0.0

    #sequence.append(car_pose)
    #final_pose = Pose2D()
    #final_pose.x = 10.0001
    #final_pose.y = -10.0001
    #final_pose.theta = 0.0
    
    #turning_radius = 1.06
    #step_size = 0.499
    #pub = rospy.Publisher('car', Pose2D, queue_size=10)
    pub = rospy.Publisher('car', Odometry, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #qs, _ = dubins.path_sample((car_pose.x, car_pose.y, car_pose.theta), (final_pose.x, final_pose.y, final_pose.theta), turning_radius, step_size)
    

    #for j in range(len(qs)-1):
    #    middle_pose = Pose2D()
    #    middle_pose.x = qs[j+1][0]
    #    middle_pose.y = qs[j+1][1]
    #    middle_pose.theta = qs[j+1][2]
    #    sequence.append(middle_pose)
    
    ####

    #sequence = [(0.0, 0.0, -1.5707963267948966), (1.1492442353296506, -2.1036774620197414, 5.71238898038469), (3.2529216973493913, -3.454433226690091, 5.298175418011595), (5.3735894653275409, -4.5720151675368594, 0.014990110832008519), (7.4598039267142262, -3.3913668720020222, 1.0149901108320085), (7.5935091267814476, -0.99797093816173121, 2.0149901108320085), (5.6517771212005226, 0.40769545012210928, 3.0149901108320085), (3.4198273611466199, -0.46673090197598488, 4.0149901108320085), (2.5478381431518557, -2.7603165816818329, 4.683168748751307), (2.722994597606478, -5.2500434776276759, 4.882081116130966), (3.1451918844451212, -7.7141354911259725, 4.882081116130966), (4.3297178556990481, -9.8630418850565693, 5.5502597540502645), (6.6621117598728921, -10.416359020541234, 0.2670744468706783), (8.3879098793981761, -8.7526757492148537, 1.2670744468706783), (7.9204213821091543, -6.4015747982664033, 2.2670744468706783), (5.6894530364796898, -5.5246475393403358, 3.2670744468706783), (3.746146850843854, -6.9281368501360232, 4.267074446870678), (2.9000325454903937, -9.2762233864905639, 4.4659868142503365)]
    sequence = [(0,0), (0,-3), (0,-6), (0,-9), (3,-9)]

    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        for i in range(len(sequence)):
            odometry_variable = Odometry()
            odometry_variable.header.seq = i
            odometry_variable.header.stamp = rospy.get_rostime()
            odometry_variable.header.frame_id = "/world"
            odometry_variable.child_frame_id = "/world"
            #odometry_variable.pose.pose.position.x = sequence[i].x
            #odometry_variable.pose.pose.position.y = sequence[i].y
            odometry_variable.pose.pose.position.x = -sequence[i][1]
            odometry_variable.pose.pose.position.y = sequence[i][0]
            odometry_variable.pose.pose.position.z = 0.0
            ##quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, sequence[i].theta)
            #quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, (-3*pi/2) + sequence[i][2])
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            odometry_variable.pose.pose.orientation.x = quaternion[0]
            odometry_variable.pose.pose.orientation.y = quaternion[1]
            odometry_variable.pose.pose.orientation.z = quaternion[2]
            odometry_variable.pose.pose.orientation.w = quaternion[3]
            print odometry_variable
            #pub.publish(sequence[i])
            pub.publish(odometry_variable)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
