# import the Empty module from std_servs Service interface
from std_srvs.srv import Empty
from custom_interfaces.srv import HospitalsServiceMessage
import rclpy
from rclpy.node import Node
import os


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
        for hospital_name in os.listdir(self.hospitals_folder):
            hospital_path = os.path.join(self.hospitals_folder, hospital_name)
            if os.path.isdir(hospital_path):
                hospitals[hospital_name] = self.read_rooms(hospital_path)
        return hospitals

    def read_rooms(self, hospital_path):
        rooms = []
        for room_file in os.listdir(hospital_path):
            room_path = os.path.join(hospital_path, room_file)
            if os.path.isfile(room_path) and room_file.endswith('.yaml'):
                rooms.append(room_file)
        return rooms     

    def empty_callback(self, request, response):
        self.get_logger().info("The Server has invoked by the client...")
        # The callback function receives empty message and build the hospitals dict.
        hospitals = self.read_hospitals()              
        # self.get_logger().info(str(hospitals))  
        # return the response parameter
        response.hospitals_dict = str(hospitals)        
        response.success = True
        self.get_logger().info("The Server has responded...")
        return response


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
