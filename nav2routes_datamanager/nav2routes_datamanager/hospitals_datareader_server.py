# import the Empty module from std_servs Service interface
from std_srvs.srv import Empty
from custom_interfaces.srv import HospitalsServiceMessage
import rclpy
from rclpy.node import Node
import os
import yaml
import json


class HospitalsDataReaderServer(Node):

    # Here you have the class constructor
    # call the class constructor to initialize the node as service
    def __init__(self,hospitals_folder): 
        super().__init__('hospitals_datareader_node')
        self.get_logger().info("The Constructor has invoked...")
        self.hospitals_folder = hospitals_folder
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(HospitalsServiceMessage, 'hospitals_datareader_service', self.empty_callback)
        self.get_logger().info("The server is waiting...")

    def read_hospitals(self):
            hospitals = {}
            # get list of directories inside hospitals folders (contains all the hospitals)
            hospitals_list = os.listdir(self.hospitals_folder)

            for hospital_name in hospitals_list:
                # complete path of hospital name
                hospital_path = os.path.join(self.hospitals_folder, hospital_name)

                # verify the path is an existing directory
                if os.path.isdir(hospital_path):
                    hospitals[hospital_name] = self.read_rooms(hospital_path)
    
            return hospitals

    def read_rooms(self, hospital_path):
        rooms = {}
        # get list of rooms inside hospital subfolder (single hospital)
        rooms_list = os.listdir(hospital_path)
        for room_file in rooms_list:
            # complete path of room yaml file
            room_path = os.path.join(hospital_path, room_file)
            if os.path.isfile(room_path) and room_file.endswith('.yaml'):
                with open(room_path, 'r') as file:
                    room_data = yaml.safe_load(file)
                    room = room_file.replace('.yaml','') # room_data['rooms']['room_name']
                    room_name = room_data['rooms']['room_name']
                    room_id = room_data['rooms']['room_id']
                    yaml_file_path = room_path
                    room_routes = self.extract_routes(room_data)  
                    room_routes = {key: list(value) for key, value in room_routes.items()}                  
                    rooms[room] = {'room_id': room_id, 'room_name': room_name, 'yaml_file_path': yaml_file_path, 'nav_routes': room_routes}
        return rooms

    def extract_routes(self, room_data):
        routes = {}
        if 'nav_routes' in room_data['rooms']:
            nav_routes = room_data['rooms']['nav_routes']
            for i in range(len(nav_routes)):
                route_id = i
                route_name = nav_routes[i]['route_name']
                routes[route_name] = {route_id}
        return routes    

    def empty_callback(self, request, response):
        self.get_logger().info("The Server has invoked by the client...")
        # The callback function receives empty message and build the hospitals dict.      

        hospitals = self.read_hospitals()
        encoded_data_string = json.dumps(hospitals)
        # debug hospitals
        self.get_logger().info(str(hospitals))

        # make the nested dictionary ready (flatten) to the trasmission.
        # hospitals_flatten = self.flatten_dict(hospitals)           
        # Get keys and values of hospitaks_flatten dictionary
        # keys = list(hospitals_flatten.keys())
        # values = list(hospitals_flatten.values()) # it needs estract sub-dict also
        response.encoded_dict = encoded_data_string
        # response.value = str(values)            
        response.success = True
        self.get_logger().info("The Server has responded...")
        return response

    # def flatten_dict(self,dictionary):
    #     flattened = {}
    #     for key, value in dictionary.items():
    #         # find nested dict to flat
    #         if isinstance(value, dict):
    #             # ricorsive algorith
    #             nested = self.flatten_dict(value)
    #             for nested_key, nested_value in nested.items():
    #                 flattened[f"{key}.{nested_key}"] = nested_value
    #         else:
    #             flattened[key] = value
    #     return flattened
        
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    hospitals_folder = '/home/user/ros2_ws/src/nav2routes_datamanager/config/hospitals'
    hospitals_datareader_obj = HospitalsDataReaderServer(hospitals_folder)
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(hospitals_datareader_obj)
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
