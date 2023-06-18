import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import tf2_ros
from visualization_msgs.msg import MarkerArray, Marker
from jsk_rviz_plugins.msg import Pictogram, PictogramArray,marker_array,marker
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Duration
import yaml

class PictogramsVisualization(Node):
    def __init__(self):
        super().__init__('pictograms_visualization_server')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(PictogramArray,'/pictograms_array',10)
        # timer_period = 0.5
        #self.timer = self.create_timer(timer_period,self.timer_callback)
        self.actions = [Pictogram.JUMP, Pictogram.JUMP_ONCE, Pictogram.ADD,Pictogram.ROTATE_X, Pictogram.ROTATE_Y, Pictogram.ROTATE_Z]
        self.pictograms = ["fa-tag","fa-play"]
        object_frames = ["/world","/person_standing","haro_base_link"]  # it needs a function that creates a frame for each waypoints.

    # get position x and y of a room's waypoint
    def get_waypoint_position(self,rooms,point_number):
        x = rooms['points'][point_number]['position']['x']
        y = rooms['points'][point_number]['position']['y']
        return x,y

    # get orientation z and w of a room's waypoint
    def get_waypoint_orientation(self,rooms,point_number):
        z = rooms['points'][point_number]['orientation']['z']
        w = rooms['points'][point_number]['orientation']['w']
        return z,w

    # crete a single child frame respect the map frame
    def create_frame(self,rooms,point_number,frame_name):
        x,y = self.get_waypoint_position(rooms,point_number)
        z,w = self.get_waypoint_orientation(rooms,point_number)
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map'
        transform_stamped.child_frame_id = frame_name

        # position and orientation
        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.rotation.z = z
        transform_stamped.transform.rotation.w = w 

        # publish frame
        self.tf_broadcaster.sendTransform(transform_stamped)

    # create all the frames
    def object_frames(self,rooms):
        for id in range(len(rooms['points'])):
            frame_name = "waypoint_" +str(id)
            self.create_frame(rooms,id,frame_name)

    def publish_pictograms(self,rooms):
        self.object_frames(self,rooms)
        points = rooms['points']
        pictograms_marker_array = PictogramArray() # MarkerArray()
        for i, point in points.items():
            print(points)
            print(point)
            print(i)
            marker = Marker()
            pictogram = Pictogram()
            marker.header.frame_id = 'map'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = point["position"]["x"]
            marker.pose.position.y = point["position"]["y"]
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0 
            marker.lifetime = Duration(sec=0) # Duration(seconds=0)
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info("marker array displayed")
        

def main(args=None):
    rclpy.init(args=args)
    with open('/home/user/ros2_ws/src/rooms_pkg/config/office_points.yaml', 'r') as file:
            rooms = yaml.safe_load(file)['rooms']
    PictogramsVisualization().object_frames(rooms) 
    # rclpy.spin(markerviz_object)
    # markerviz_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
#!/usr/bin/env python

#
# Please run rviz by rosrun rviz rviz -d `rospack find jsk_rviz_plugins`/config/pictogram.rviz
#

import rospy
import math
from jsk_rviz_plugins.msg import Pictogram, PictogramArray
from random import random, choice
rospy.init_node("pictogram_object_demo_node")
p = rospy.Publisher("/pictogram_array", PictogramArray,  queue_size=1)

r = rospy.Rate(1)
actions = [Pictogram.JUMP, Pictogram.JUMP_ONCE, Pictogram.ADD, 
           Pictogram.ROTATE_X, Pictogram.ROTATE_Y, Pictogram.ROTATE_Z]
pictograms = ["fa-tag","fa-play","fa-align-justify"]
object_frames = ["/world","/person_standing","haro_base_link"]

while not rospy.is_shutdown():
    
    arr = PictogramArray()
    arr.header.frame_id = "/world"
    arr.header.stamp = rospy.Time.now()
    for index, character in enumerate(pictograms):
        msg = Pictogram()
        msg.header.frame_id = object_frames[index]
        msg.action = actions[3]
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        # It has to be like this to have them vertically orient the icons.
        msg.pose.orientation.w = -0.924 # 0.7
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0 # -0.7
        msg.pose.orientation.z = 0.383
        msg.size = 1
        msg.color.r = 25 / 255.0
        msg.color.g = 255 / 255.0
        msg.color.b = 240 / 255.0
        msg.color.a = 1.0
        msg.character = character
        arr.pictograms.append(msg)
        
    p.publish(arr)
    r.sleep()

"""