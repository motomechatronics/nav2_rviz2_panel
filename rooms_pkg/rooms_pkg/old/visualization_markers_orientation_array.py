import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import yaml

class WaypointsVisualization(Node):
    def __init__(self):
        super().__init__('marker_visualization_server')
        self.marker_pub = self.create_publisher(MarkerArray,'/marker_array',10)
        timer_period = 0.5
        #self.timer = self.create_timer(timer_period,self.timer_callback)
    

    def get_waypoint_position(self,rooms,point_number):
        x = rooms['points'][point_number]['position']['x']
        y = rooms['points'][point_number]['position']['y']
        return x,y


    def get_waypoint_orientation(self,rooms,point_number):
        z = rooms['points'][point_number]['orientation']['z']
        w = rooms['points'][point_number]['orientation']['w']
        return z,w


    # Display a footprint in each waypoints
    def publish_footprint(self,rooms):
        points = rooms['points']
        marker_array = MarkerArray()
        for i, point in points.items():
            x,y = self.get_waypoint_position(rooms,i)
            z,w = self.get_waypoint_orientation(rooms,i)
            #print(point)
            #print(i)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1     
            marker.pose.orientation.z = z
            marker.pose.orientation.w = w       
            marker.scale.x = 0.5
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
    waypointviz_object = WaypointsVisualization()
    waypointviz_object.publish_footprint(rooms) 
    rclpy.spin(waypointviz_object)
    waypointviz_object.destroy_node()
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