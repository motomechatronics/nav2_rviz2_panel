import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_prefix


class NavroutesManager(Node):
    def __init__(self,room_name):
        super().__init__('navroutes_manager_node')
        self.rooms = self.open_yaml_file(room_name)




    def open_yaml_file(self,room_name):
        package_name_dir = get_package_prefix('rooms_pkg')
        file_path = os.path.join(package_name_dir,'config')
        with open(file_path, 'r') as file:
            try:
                # assign the content of file to room
                room = yaml.safe_load(file)        

            except yaml.YAMLError as e:
                print("Error reafing YAML file:", e)
        return room

def main(args=None):
    rclpy.init(args=args)
    object = NavroutesManager('disinfection_rooms')
    rclpy.shutdown()

if __name__ == '__main__':
    main()




