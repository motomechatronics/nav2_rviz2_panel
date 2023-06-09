import os
import ament_index_python
import rclpy
import yaml

class HospitalsDataReader:
    def __init__(self, hospitals_folder):
        self.hospitals_folder = hospitals_folder

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
                # print('YYYYYYYYYYYYYYYYYYY')
                # print(str(hospitals))
                # print('YYYYYYYYYYYYYYYYYYY')
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
                    # print("****************")
                    # print(room_data)
                    # print("****************")
                    room = room_file.replace('.yaml','') # room_data['rooms']['room_name']
                    room_name = room_data['rooms']['room_name']
                    room_id = room_data['rooms']['room_id']
                    yaml_file_path = room_path
                    room_routes = self.extract_routes(room_data)                    
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


def main():
    package_name = 'nav2routes_datamanager'
    hospitals_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'hospitals')
    hospitals_folder = '/home/user/ros2_ws/src/nav2routes_datamanager/config/hospitals'
    hospitals_datareader_obj = HospitalsDataReader(hospitals_folder)
    hospitals = hospitals_datareader_obj.read_hospitals()
    print(hospitals)
    print(hospitals_path)

if __name__ == '__main__':
    main()