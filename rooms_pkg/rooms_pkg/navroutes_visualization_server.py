import rclpy
import os
import pkgutil
import yaml
import math
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.clock import Clock
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from tf_transformations import quaternion_from_euler
from custom_interfaces.srv import NavroutesServiceMessage
from geometry_msgs.msg import Point


class NavRoutesVisualizationServer(Node):
    def __init__(self):
        super().__init__('navroutes_visualization_node')
        self.get_logger().info("constructor calling...")
        self.srv_visualize_route = self.create_service(NavroutesServiceMessage, 'navroutes_visualization_server', self.visualize_route_callback)   
        self.waypoints_pub = self.create_publisher(MarkerArray,'/waypoints',10) # /marker_array
        self.waypoint_numbers_pub = self.create_publisher(MarkerArray,'/waypoint_numbers',10) #//marker_wp_text_array
        self.phase_arrows_pub = self.create_publisher(MarkerArray,'/phase_arrows',10) #/marker_routine_array
        self.phase_texts_pub = self.create_publisher(MarkerArray,'/phase_texts',10) # /marker_text_array      
        self.phase_texts_background_pub = self.create_publisher(MarkerArray,'/phase_texts_background',10)  
        self.routes_pub = self.create_publisher(MarkerArray,'/routes',10) #//marker_route_array
        self.route_directions_pub = self.create_publisher(MarkerArray,'/route_directions',10) # /marker_route_direction_array         
        # set the package that contains the hospitals data
        self.package_name = "nav2routes_datamanager"
        self.get_logger().info(self.package_name)
        self.package_path = get_package_share_directory(self.package_name) + "/config/hospitals"
        self.get_logger().info(self.package_path)

    def delete_old_markers(self):
        self.get_logger().info("old markers deleting...")
        # Delete previous markers by publishing an empty MarkerArray
        delete_markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_markers.markers.append(delete_marker)
        self.waypoints_pub.publish(delete_markers)
        self.waypoint_numbers_pub.publish(delete_markers)
        self.phase_arrows_pub.publish(delete_markers)
        self.phase_texts_pub.publish(delete_markers)
        self.phase_texts_background_pub.publish(delete_markers)
        self.routes_pub.publish(delete_markers)
        self.route_directions_pub.publish(delete_markers)
        

    def visualize_route_callback(self,request,response):
        self.delete_old_markers()
        self.get_logger().info("request received...")
        room_path = request.room  # it is the path of the room as: /home/user/ros2_ws/src/rooms_pkg/config/office_points.yaml
        rooms = self.get_yaml_file(room_path) # have to became rooms = self.get_yaml_file(room) 
        self.route_id = request.route  
        if self.route_id in rooms['nav_routes']:            
            self.publish_waypoints(rooms,self.route_id)
            self.publish_waypoint_numbers(rooms,self.route_id)  
            self.publish_phase_arrows(rooms,self.route_id)
            self.publish_phase_texts(rooms,self.route_id)        
            self.publish_phase_texts_background(rooms,self.route_id)      
            self.publish_routes(rooms,self.route_id)
            self.publish_route_directions(rooms,self.route_id)            
            
            response.output = "route successfuly displayed"
            response.success = True
        
        else:
            response.output = "route not found"
            response.success = False         

        self.get_logger().info("response sent...")
        return response
    
    # def get_yaml(self,hospital,room): 
    #     # hospital data path       
    #     package_path = get_package_share_directory(self.package_name) + "/config/hospitals" + hospital + room
    #     with open(package_path, 'r') as file:
    #         rooms = yaml.safe_load(file)['rooms']
    #     return rooms

    def get_yaml_file(self, path):
        with open(path, 'r') as file:
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

    def get_phase_pose(self,x0,y0,phases_number,radius):
    # list of 2D vectors: Given the waypoint_position (coordinates x0,y0 of waypoint) and the phases number,
    # this function finds all positions of the phases around own waypoint.
    # phases_number (integer) is the number of phases contained in a routine 
        # vector of phases coordinates
        phase_pose = []
        # distance between phase and waypoint
        # radius =  0.5
        # angle between two phases
        delta_angle = 45 * math.pi /  180
        for i in range(phases_number):
            angle = delta_angle * i
            x = x0 + radius * math.cos(angle)
            y = y0 + radius * math.sin(angle)
            phase_orientation = self.get_phase_orientation(angle)
            z = phase_orientation[2]
            w = phase_orientation[3]
            #self.get_logger().info(z)
            #self.get_logger().info(w)            
            phase = [x,y,z,w]
            phase_pose.append(phase)
        return phase_pose

    def get_phase_position(self,x0,y0,phases_number,radius):
    # list of 2D vectors: Given the waypoint_position (coordinates x0,y0 of waypoint) and the phases number,
    # this function finds all positions of the phases around own waypoint.
    # phases_number (integer) is the number of phases contained in a routine 
        # vector of phases coordinates
        phase_position = []
        # distance between phase and waypoint
        # radius =  0.5
        # angle between two phases
        delta_angle = 45 * math.pi /  180
        for i in range(phases_number):
            angle = delta_angle * i
            x = x0 + radius * math.cos(angle)
            y = y0 + radius * math.sin(angle)
            phase = [x,y]
            phase_position.append(phase)
        return phase_position
    
    def get_phase_orientation(self,yaw_angle):
    # phase orientation/quaternion/ get the orientation of the phase. Use 2 and 3 vector components to extract z and w.
    # angle in radians
            roll_angle = 0.0
            pitch_angle = 0.0
            phase_orientation = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
            return phase_orientation

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
    def publish_waypoints(self,rooms,route_id):
        route_name,route_waypoints,routine,phases_number,route_waypoints_number = self.get_routine_data(rooms,route_id) 
        # points = rooms['points']
        
        # # Delete previous markers by publishing an empty MarkerArray with DELETEALL action
        # delete_markers = MarkerArray()
        # delete_marker = Marker()
        # delete_marker.action = Marker.DELETEALL
        # delete_markers.markers.append(delete_marker)
        # self.waypoints_pub.publish(delete_markers)

        # # Wait for a short duration to allow RViz to remove the previous markers
        # rclpy.spin_once(self, timeout_sec=0.1)

        marker_array = MarkerArray()
        for i in route_waypoints: 
            x,y = self.get_waypoint_position(rooms,i)
            z,w = self.get_waypoint_orientation(rooms,i)
            #print(point)
            #print(i)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.id = i
            # marker.type = Marker.ARROW
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = "package://rooms_pkg/meshes/footprint.dae"
            marker.mesh_use_embedded_materials = True
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1     
            marker.pose.orientation.z = z
            marker.pose.orientation.w = w       
            marker.scale.x = 0.0018
            marker.scale.y = 0.0018
            marker.scale.z = 0.0018
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0 
            marker.lifetime = Duration(sec=0) # Duration(seconds=0)   
            marker_array.markers.append(marker)
        
        self.waypoints_pub.publish(marker_array)
        self.get_logger().info("waypoints displayed")


        # Display a footprint text in each waypoints
    def publish_waypoint_numbers(self,rooms,route_id):
        route_name,route_waypoints,routine,phases_number,route_waypoints_number = self.get_routine_data(rooms,route_id) 
        #points = rooms['points']
        marker_array = MarkerArray()
        for i in route_waypoints:
            x,y = self.get_waypoint_position(rooms,i)
            # z,w = self.get_waypoint_orientation(rooms,i)
            #print(point)
            #print(i)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y - 0.04
            marker.pose.position.z = 0.2    
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0       
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0 
            name = str(i)
            marker.text = name
            marker.lifetime = Duration(sec=0) # Duration(seconds=0)   
            marker_array.markers.append(marker)
        
        self.waypoint_numbers_pub.publish(marker_array)
        self.get_logger().info("waypoint numbers displayed")


    def publish_phase_arrows(self,rooms,route_id): # insert two for cycle
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
                phase_pose = self.get_phase_pose(x,y,phases_number,0.4)
                marker2 = Marker()
                marker2.header.frame_id = 'map'
                marker2.id = k                        
                marker2.type = Marker.MESH_RESOURCE
                marker2.action = Marker.ADD
                marker2.mesh_resource = "package://rooms_pkg/meshes/arrow3.dae"
                marker2.mesh_use_embedded_materials = True
                marker2.pose.position.x = phase_pose[j][0]
                marker2.pose.position.y = phase_pose[j][1]
                marker2.pose.position.z = 0.1   
                marker2.pose.orientation.z = phase_pose[j][2]
                marker2.pose.orientation.w = phase_pose[j][3]   
                marker2.scale.x = 0.0012
                marker2.scale.y = 0.0012
                marker2.scale.z = 0.0012
                marker2.color.r = 0.9
                marker2.color.g = 0.9
                marker2.color.b = 0.9
                marker2.color.a = 1.0 
                marker2.lifetime = Duration(sec=0) 
                marker_array2.markers.append(marker2)
                j = j + 1
                k = k + 1

        self.phase_arrows_pub.publish(marker_array2)
        self.get_logger().info("phase arrows displayed")   

    def publish_routes(self,rooms,route_id): # insert two for cycle
        route_name,route_waypoints,routine,phases_number,route_waypoints_number = self.get_routine_data(rooms,route_id)        
        # points = rooms['points']
        
        marker_array2 = MarkerArray()         
        line_strip = []       
        for i in route_waypoints:     
            point = Point()    
            x,y = self.get_waypoint_position(rooms,i)
            marker2 = Marker()
            marker2.header.frame_id = 'map'
            marker2.id = i                
            marker2.type = Marker.LINE_STRIP
            marker2.action = Marker.ADD
            marker2.scale.x = 0.05
            marker2.color.r = 0.5
            marker2.color.g = 0.5
            marker2.color.b = 0.5
            marker2.color.a = 1.0 
            #marker2.lifetime = Duration(sec=0) # Duration(seconds=0)  
            point.x = x
            point.y = y
            line_strip.append(point)
        
        marker2.points = line_strip
        marker_array2.markers.append(marker2)
        self.routes_pub.publish(marker_array2)
        self.get_logger().info("routes displayed")

    def publish_route_directions(self,rooms,route_id): # insert two for cycle
        route_name,route_waypoints,routine,phases_number,route_waypoints_number = self.get_routine_data(rooms,route_id)     
              
        marker_array2 = MarkerArray()         
        # line_strip = []       
        for i in range(len(route_waypoints) - 1):  
            point_0 = route_waypoints[i]   
            point_1 = route_waypoints[i+1]
            x0,y0 = self.get_waypoint_position(rooms,point_0)
            x1,y1 = self.get_waypoint_position(rooms,point_1)
            xm = 0.5 * (x0 + x1)
            ym = 0.5 * (y0 + y1)
            Dy = (y1 - y0)
            Dx = (x1 - x0)
            angle = math.atan2(Dy,Dx)
            alpha_orientation = self.get_phase_orientation(angle + 1.57)
            marker2 = Marker()
            marker2.header.frame_id = 'map'
            marker2.id = i                
            marker2.type = Marker.MESH_RESOURCE
            marker2.action = Marker.ADD
            marker2.mesh_resource = "package://rooms_pkg/meshes/arrow.dae"
            marker2.mesh_use_embedded_materials = True
            marker2.pose.position.x = xm
            marker2.pose.position.y = ym
            marker2.pose.position.z = 0.0     
            marker2.pose.orientation.z = alpha_orientation[2]
            marker2.pose.orientation.w = alpha_orientation[3]
            marker2.scale.x = 0.001
            marker2.scale.y = 0.003
            marker2.scale.z = 0.002
            marker2.color.r = 0.5
            marker2.color.g = 0.5
            marker2.color.b = 0.5
            marker2.color.a = 1.0 
            marker2.lifetime = Duration(sec=0) # Duration(seconds=0)   
            marker_array2.markers.append(marker2)     
                
        self.route_directions_pub.publish(marker_array2)
        self.get_logger().info("route directions displayed")
    
    def publish_phase_texts_background(self,rooms,route_id): # insert two for cycle
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
                phase_position = self.get_phase_position(x,y,phases_number,0.9)
                marker2 = Marker()
                marker2.header.frame_id = 'map'
                marker2.id = k            
                marker2.type = Marker.MESH_RESOURCE
                marker2.action = Marker.ADD
                marker2.mesh_resource = "package://rooms_pkg/meshes/background_text2.dae"
                marker2.mesh_use_embedded_materials = True
                correction_x = - 0.05
                correction_y = 0.01
                marker2.pose.position.x = phase_position[j][0] + correction_x
                marker2.pose.position.y = phase_position[j][1] + correction_y
                marker2.pose.position.z = 0.1     
                marker2.pose.orientation.z = 0.0
                marker2.pose.orientation.w = 1.0     
                marker2.scale.x = 0.006
                marker2.scale.y = 0.008
                marker2.scale.z = 0.008
                marker2.color.r = 0.9
                marker2.color.g = 0.9
                marker2.color.b = 0.9
                marker2.color.a = 0.9
                waypoint = i
                phase = j
                angle = rooms['nav_routes'][route_id]['points'][waypoint][phase][0]
                time = rooms['nav_routes'][route_id]['points'][waypoint][phase][1]
                marker2.text = "phase{}\n{}°\n{}s".format(phase,angle,time)   
                marker2.lifetime = Duration(sec=0) 
                marker_array2.markers.append(marker2)
                j = j + 1
                k = k + 1

        self.phase_texts_background_pub.publish(marker_array2)
        self.get_logger().info("phase texts background displayed")    


    def publish_phase_texts(self,rooms,route_id): 
    # it publishes phane no., angle and time of each phase
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
                phase_position = self.get_phase_position(x,y,phases_number,0.9)
                marker2 = Marker()
                marker2.header.frame_id = 'map'
                marker2.id = k            
                marker2.type = Marker.TEXT_VIEW_FACING
                marker2.action = Marker.ADD
                marker2.pose.position.x = phase_position[j][0]
                marker2.pose.position.y = phase_position[j][1]
                marker2.pose.position.z = 0.1     
                marker2.pose.orientation.z = 0.0
                marker2.pose.orientation.w = 1.0     
                marker2.scale.x = 0.1
                marker2.scale.y = 0.1
                marker2.scale.z = 0.1
                marker2.color.r = 0.3
                marker2.color.g = 0.3
                marker2.color.b = 0.3
                marker2.color.a = 1.0 
                waypoint = i
                phase = j
                angle = rooms['nav_routes'][route_id]['points'][waypoint][phase][0]
                time = rooms['nav_routes'][route_id]['points'][waypoint][phase][1]
                marker2.text = "phase{}\n{}°\n{}s".format(phase,angle,time)   
                marker2.lifetime = Duration(sec=0) 
                marker_array2.markers.append(marker2)
                j = j + 1
                k = k + 1

        self.phase_texts_pub.publish(marker_array2)
        self.get_logger().info("phase texts displayed")  

def main(args=None):
    rclpy.init(args=args)
    navroutesviz_object = NavRoutesVisualizationServer()
    rclpy.spin(navroutesviz_object)
    # navroutesviz_object.destroy_node()
    #rclpy.shutdown()

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