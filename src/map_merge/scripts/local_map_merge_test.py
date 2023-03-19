#!/usr/bin/python

import rospy
from nav_msgs.msg import OccupancyGrid

class copy_map():
    def __init__(self):
        rospy.init_node('map_copy')
        rospy.set_param('/new/map_merge/init_pose_x', 0)
        rospy.set_param('/new/map_merge/init_pose_y', 0)
        rospy.set_param('/new/map_merge/init_pose_z', 0)
        rospy.set_param('/new/map_merge/init_pose_yaw', 0)
        rospy.set_param('/another/map_merge/init_pose_x', 0)
        rospy.set_param('/another/map_merge/init_pose_y', 0)
        rospy.set_param('/another/map_merge/init_pose_z', 0)
        rospy.set_param('/another/map_merge/init_pose_yaw', 0)
        #rospy.set_param('/map_merge/init_pose_x', 5)
        #rospy.set_param('/map_merge/init_pose_y', 0)
        #rospy.set_param('/map_merge/init_pose_z', 0)
        #rospy.set_param('/map_merge/init_pose_yaw', 0.-0.78)

        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.callback)
        self.pub1 = rospy.Publisher('/new/submap', OccupancyGrid, queue_size=10)
        self.pub2 = rospy.Publisher('/another/submap', OccupancyGrid, queue_size=10)
        rospy.spin()
    def callback(self, data):
        print('123')
        self.pub1.publish(data)
        self.pub2.publish(data)

if __name__ == '__main__':
    copy_map()
