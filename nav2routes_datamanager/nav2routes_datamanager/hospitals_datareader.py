import os
import ament_index_python
import rclpy

class HospitalsDataReader:
    def __init__(self, hospitals_folder):
        self.hospitals_folder = hospitals_folder

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