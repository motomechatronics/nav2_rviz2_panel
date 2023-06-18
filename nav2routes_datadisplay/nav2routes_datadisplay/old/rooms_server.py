import rclpy
import pathlib
from rclpy.node import Node
from std_srvs.srv import Empty
import yaml
import os
from ament_index_python.packages import get_package_share_directory
#from visualization_markers_array import MarkerVisualization
from custom_interfaces.srv import NavroutesServiceMessage
from navroutes_visualization import NavRoutesVisualization
import sys
print(sys.path)

class RoomsManagerServer(Node):
    def __init__(self):
        super().__init__('rooms_manager_server')
        self.srv_visualize_route = self.create_service(NavroutesServiceMessage, 'visualize_route', self.visualize_route_callback)
        self.navroutes_obj = NavRoutesVisualization()
        self.route_id = None

    def visualize_route_callback(self,request,response):
        # room = request.room             
        rooms = self.navroutes_obj.get_yaml_file() 
        self.route_id = request.route  
        if self.route_id in rooms['routes']:            
            self.publish_waypoints(rooms,self.route_id)
            self.publish_waypoint_numbers(rooms,self.route_id)  
            self.publish_phase_arrows(rooms,self.route_id)
            self.publish_phase_texts(rooms,self.route_id)        
            self.publish_phase_texts_background(rooms,self.route_id)      
            self.publish_routes(rooms,self.route_id)
            self.publish_route_directions(rooms,self.route_id)             
        
        response.output = "route successfully displayed"
        return response
 
def main(args=None):
    rclpy.init(args=args)

    rooms_service_node = RoomsManagerServer()
    rclpy.spin(rooms_service_node)
    rooms_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main


