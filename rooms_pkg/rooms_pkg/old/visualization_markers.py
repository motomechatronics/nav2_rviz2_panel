import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import os
import rooms_pkg 

class MarkerVisualization(Node):
    def __init__(self):
        super().__init__('marker_visualization_server')
        self.marker_pub = self.create_publisher(Marker,'/marker',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.init_marker()
        self.init_marker2()
        
    def init_marker(self):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/map" # "/world"
        #self.marker_object.header.stamp    = rclpy.clock.Clock().now()
        self.marker_object.ns = "robot_disinfection"
        self.marker_object.id = 0
        self.marker_object.type = Marker.MESH_RESOURCE
        # self.marker_object.type = Marker.ARROW
        self.marker_object.action = Marker.ADD        
        self.marker_object.mesh_resource = "package://rooms_pkg/meshes/robot_low.dae"
        # print(os.getcwd)
        # os.chdir('/home/user/ros2_ws/src/rooms_pkg/meshes/')
        # # self.marker_object.mesh_resource = "amr_disinfection.dae"        
        self.marker_object.mesh_use_embedded_materials = True
        my_point = Point()
        my_point.x = -0.9665021896362305
        my_point.y = -3.013331651687622
        my_point.z = 0.0
        self.marker_object.pose.position = my_point
        
        self.marker_object.pose.orientation.x = 0.0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = 1.0
        self.marker_object.scale.y = 1.0
        self.marker_object.scale.z = 1.0
    
        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 1.0
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = Duration(sec=0) # Duration(seconds=0)


    def init_marker2(self):
        self.marker_object2 = Marker()
        self.marker_object2.header.frame_id = "/map" # "/world"
        #self.marker_object.header.stamp    = rclpy.clock.Clock().now()
        self.marker_object2.ns = "robot_disinfection"
        self.marker_object2.id = 1
        self.marker_object2.type = Marker.MESH_RESOURCE
        self.marker_object2.type = Marker.CYLINDER
        self.marker_object2.action = Marker.ADD        
        # self.marker_object.mesh_resource = "/home/user/ros2_ws/src/rooms_pkg/meshes/amr_disinfection.dae"
        self.marker_object2.mesh_use_embedded_materials = True
        my_point = Point()
        my_point.x = -0.9665021896362305
        my_point.y = -4.013331651687622
        my_point.z = 0.0
        self.marker_object2.pose.position = my_point
        
        self.marker_object2.pose.orientation.x = 0.0
        self.marker_object2.pose.orientation.z = 0.0
        self.marker_object2.pose.orientation.w = 1.0
        self.marker_object2.scale.x = 0.5
        self.marker_object2.scale.y = 0.5
        self.marker_object2.scale.z = 0.5
    
        self.marker_object2.color.r = 1.0
        self.marker_object2.color.g = 0.0
        self.marker_object2.color.b = 0.0
        # This has to be, otherwise it will be transparent
        self.marker_object2.color.a = 1.0
    
        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object2.lifetime = Duration(sec=0) # Duration(seconds=0)
    
   

    def timer_callback(self):
        self.start()
    
    def start(self):
        while rclpy.ok():
            self.marker_pub.publish(self.marker_object)
            # self.marker_pub.publish(self.marker_object2)

def main(args=None):
    rclpy.init(args=args)
    print(os.getcwd)
    markerviz_object = MarkerVisualization() 
    rclpy.spin(markerviz_object)
    markerviz_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



"""
int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11
int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3
std_msgs/msg/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/msg/Pose pose
geometry_msgs/msg/Vector3 scale
std_msgs/msg/ColorRGBA color
builtin_interfaces/msg/Duration lifetime
boolean frame_locked
geometry_msgs/msg/Point[] points
std_msgs/msg/ColorRGBA[] colors
string text
string mesh_resource
boolean mesh_use_embedded_materials
"""