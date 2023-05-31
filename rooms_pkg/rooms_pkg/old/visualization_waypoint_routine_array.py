import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import yaml
import math

class WaypointsVisualization(Node):
    def __init__(self):
        super().__init__('routine_visualization_server')
        self.marker_pub = self.create_publisher(MarkerArray,'/marker_array',10)
        self.marker_pub2 = self.create_publisher(MarkerArray,'/marker_routine_array',10)
        timer_period = 60
        self.timer = self.create_timer(timer_period,self.timer_callback)

    def timer_callback(self):
        rooms = self.get_yaml_file()
        self.publish_footprint(rooms)
        self.publish_phase(rooms,2)

    def get_yaml_file(self):
        with open('/home/user/ros2_ws/src/rooms_pkg/config/office_points.yaml', 'r') as file:
            rooms = yaml.safe_load(file)['rooms']
        return rooms

    # get name, number of waypoints and phase coordinates
    def get_routine_data(self,rooms,route_id):
        # rooms is the data rooms dictionary (waypoint, nav_routes, etc.)
        # route is the id that identifies the route in the nav_route dictionary

        # get route i.e. the dictionary of all couples: waypoint (key) and routine (value)
        route = rooms['nav_routes'][route_id]['points']

        # get the name of the route
        route_name = rooms['nav_routes'][route_id]['route_name']   

        # get the list of the waypoints reached in the route
        route_waypoints = list(route.keys())
        # get the number of waypoint present in the route
        route_waypoints_number = len(rooms['nav_routes'][route_id]['points'])
        
        # for every waypoint, it extracts from route the routine. 
        # phases_number is a vector that contains the number of phase inside a routine.
        routine =  {}
        phases_number = []
        for waypoint in route:
            values = route[waypoint]
            routine[waypoint] = values  
            phases_number.append(len(routine[waypoint]))
        return route_name,route_waypoints,routine,phases_number,route_waypoints_number

    def get_phase_position(self,x0,y0,phases_number):
    # list of 2D vectors: Given the waypoint_position (coordinates x0,y0 of waypoint) and the phases number,
    # this function finds all positions of the phases around own waypoint.
    # phases_number (integer) is the number of phases contained in a routine 
        # vector of phases coordinates
        phase_position = []
        # distance between phase and waypoint
        radius =  0.5
        # angle between two phases
        delta_angle = 45
        for i in range(phases_number):
            angle = delta_angle * i
            x = x0 + radius * math.cos(angle)
            y = y0 + radius * math.sin(angle)
            phase = [x,y]
            phase_position.append(phase)
        return phase_position

    def get_waypoint_position(self,rooms,point_number):
        # This function get the coordinates of a waypoint given its id (point_number)
        x = rooms['points'][point_number]['position']['x']
        y = rooms['points'][point_number]['position']['y']
        return x,y


    def get_waypoint_orientation(self,rooms,point_number):
        # This function get the orientation of a waypoint given its id (point_number)
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

    def publish_phase(self,rooms,route_id): # insert two for cycle
        route_name,route_waypoints,routine,phases_number,route_waypoints_number = self.get_routine_data(rooms,route_id)        
        # points = rooms['points']
        marker_array2 = MarkerArray()
        k = 0        
        for i in route_waypoints:         
            x,y = self.get_waypoint_position(rooms,i)
            j = 0
            for phase in rooms['nav_routes'][route_id]['points'][i]:                
                # phases_number/integer/ it is the number of phases contained in a routine
                phases_number = len(rooms['nav_routes'][route_id]['points'][i])
                # for j in range(phases_number[i] - 1): # ex. [3,2,2]
                phase_position = self.get_phase_position(x,y,phases_number)
                # z,w = self.get_waypoint_orientation(rooms,i)
                #print(point)
                #print(i)
                marker2 = Marker()
                marker2.header.frame_id = 'map'
                marker2.id = k  
                # marker2.ns = 'k'            
                marker2.type = Marker.CYLINDER
                marker2.action = Marker.ADD
                #print('**********')                
                #self.get_logger().info(f' "j" {j}  "k" {k} "phase_position" {phase_position}')
                #print('**********')
                marker2.pose.position.x = phase_position[j][0]
                marker2.pose.position.y = phase_position[j][1]
                marker2.pose.position.z = 0.1     
                marker2.pose.orientation.z = 0.0
                marker2.pose.orientation.w = 1.0     
                marker2.scale.x = 0.1
                marker2.scale.y = 0.1
                marker2.scale.z = 0.1
                marker2.color.r = 0.0
                marker2.color.g = 1.0
                marker2.color.b = 0.0
                marker2.color.a = 1.0 
                marker2.lifetime = Duration(sec=0) # Duration(seconds=0)   
                marker_array2.markers.append(marker2)
                j = j + 1
                k = k + 1

        self.marker_pub2.publish(marker_array2)
        self.get_logger().info("route array displayed")   

def main(args=None):
    rclpy.init(args=args)
    with open('/home/user/ros2_ws/src/rooms_pkg/config/office_points.yaml', 'r') as file:
            rooms = yaml.safe_load(file)['rooms']
    waypointviz_object = WaypointsVisualization()
    #waypointviz_object.publish_footprint(rooms) 
    #waypointviz_object.publish_phase(rooms,0) 
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