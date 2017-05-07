
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from nav_msgs.msg import Path
import tf


def path():

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
    pub = rospy.Publisher('astar_path', Path, queue_size=10)
    rospy.init_node('path', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #qs, _ = dubins.path_sample((car_pose.x, car_pose.y, car_pose.theta), (final_pose.x, final_pose.y, final_pose.theta), turning_radius, step_size)
    

    #for j in range(len(qs)-1):
    #    middle_pose = Pose2D()
    #    middle_pose.x = qs[j+1][0]
    #    middle_pose.y = qs[j+1][1]
    #    middle_pose.theta = qs[j+1][2]
    #    sequence.append(middle_pose)
    
    ####

    sequence = [(0,0,0), (3,0,0), (6,0,0), (6,-3,0), (6,-6,0)]
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        '''
        for i in range(len(sequence)):
            path_variable = Path()
            path_variable.header.seq = i
            path_variable.header.stamp = rospy.get_rostime()
            path_variable.header.frame_id = "/world"
            #odometry_variable.pose.pose.position.x = sequence[i].x
            #odometry_variable.pose.pose.position.y = sequence[i].y
            path_variable.poses.pose.position.x = -sequence[i][1]
            path_variable.poses.pose.position.y = sequence[i][0]
            path_variable.poses.pose.position.z = 0.0
            #quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, sequence[i].theta)
            #quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, (-3*pi/2) + sequence[i][2])
            #odometry_variable.pose.pose.orientation.x = quaternion[0]
            #odometry_variable.pose.pose.orientation.y = quaternion[1]
            #odometry_variable.pose.pose.orientation.z = quaternion[2]
            #odometry_variable.pose.pose.orientation.w = quaternion[3]
            print path_variable
            #pub.publish(sequence[i])
            '''
            path_variable = Path()
            path_variable.header.seq = i
            path_variable.header.stamp = rospy.get_rostime()
            path_variable.header.frame_id = "/world"
            path_variable.poses = sequence
            pub.publish(path_variable)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
