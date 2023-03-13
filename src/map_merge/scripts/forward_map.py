#! /usr/bin/python
import os
import rospy
import nav_msgs.msg import OccupancyGrid

class ForwardMap:
    def __init__(self, name : str):
        self.sub = rospy.Subscriber("map", OccupancyGrid, callback, queue_size=1000)
        self.pub = rospy.Publisher(f"/{name}/submap", OccupancyGrid, queue_size=1000)
        rospy.spin()
    def callback(self, map_msg):
        self.pub.publish(map_msg)

if "__main__" == __name__:
    rospy.init_node("forward_map_node")
    name = f"robot{os.environ.get('ROS_HOSTNAME')[5:]}"
    ForwardMap(name)

        

