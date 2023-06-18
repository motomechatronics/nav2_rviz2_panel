import rclpy
import pathlib
from rclpy.node import Node
from std_srvs.srv import Empty
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from visualization_markers_array import MarkerVisualization
    
class RoomsManagerServer(Node):
    def __init__(self):
        super().__init__('rooms_manager_server')
        self.rooms = self.load_rooms()
        self.marker_pub = self.create_publisher(Marker,'/marker',10)
        self.srv_add_room = self.create_service(Empty, 'add_room', self.add_room_callback)
        self.srv_remove_room = self.create_service(Empty, 'remove_room', self.remove_room_callback)
        self.srv_read_room = self.create_service(Empty, 'read_room', self.read_room_callback)
        #self.yaml_file = os.path.dirname(os.path.join(get_package_share_directory('rooms_pkg')))
        #self.yaml_file = os.path.abspath(os.getcwd())
        self.yaml_file = os.path.join(get_package_share_directory('rooms_pkg'), 'config', 'office_points.yaml')
        self.get_logger().info(self.yaml_file)

    def load_rooms(self):
        with open('/home/user/ros2_ws/src/rooms_pkg/config/office_points.yaml', 'r') as file:
            rooms = yaml.safe_load(file)['rooms']
        return rooms

    def save_rooms(self):
        with open('/home/user/ros2_ws/src/rooms_pkg/config/office_points.yaml', 'w') as file:
            yaml.safe_dump({'rooms': self.rooms}, file)

    def read_room_callback(self,request,response):
        room_id = self.rooms["room_id"]
        room_name = self.rooms["room_name"]

        x1 = self.rooms["points"][1]["position"]["x"]
        y1 = self.rooms["points"][1]["position"]["y"]

        x2 = self.rooms["points"][2]["position"]["x"]
        y2 = self.rooms["points"][2]["position"]["y"]

        x3 = self.rooms["points"][3]["position"]["x"]
        y3 = self.rooms["points"][3]["position"]["y"]
        
        #y1 = self.rooms["points"]["1"]["position"]["y"]

        #self.get_logger().info(self.rooms)
        self.get_logger().info(f"Room ID: {room_id}")
        self.get_logger().info(f"Room name: {room_name}")
        self.get_logger().info(f"x1: {x1}")   
        self.get_logger().info(f"y1: {y1}") 
        self.get_logger().info("------------------")
        self.get_logger().info(f"x2: {x2}")   
        self.get_logger().info(f"y2: {y2}") 
        self.get_logger().info("------------------")   
        self.get_logger().info(f"x3: {x3}")   
        self.get_logger().info(f"y3: {y3}") 
        self.get_logger().info("------------------")          
        #response.success = True
        return response

    def add_room_callback(self, request, response):
        # Assume request has the following fields:
        # request.id
        # request.name
        # request.x
        # request.y
        # request.z
        # request.w
        new_room = {

  "room1": {
        "id": 1,
        "name": "Sala 1",
        "initial_pose": {
            "position": {
                "x": 0,
                "y": 0
            },
            "orientation": {
                "x": 0,
                "w": 0
            }
        },
        "waypoints": {
            "waypoint1": {
                "id": 1,
                "position": {
                    "x": 1,
                    "y": 1
                },
                "orientation": {
                    "x": 0,
                    "w": 0
                },
                "note": "Primo waypoint"
            },
            "waypoint2": {
                "id": 2,
                "position": {
                    "x": 2,
                    "y": 2
                },
                "orientation": {
                    "x": 0,
                    "w": 0
                },
                "note": "Secondo waypoint"
            }
        },
        "routes": {
            "route1": {
                "name": "Primo percorso",
                "waypoints": {
                    "waypoint1": {
                        "angle": 0,
                        "time": 10
                    },
                    "waypoint2": {
                        "angle": 90,
                        "time": 20
                    }
                }
            }
        }
    },
   
}

        
        self.rooms.append(new_room)
        self.save_rooms()
        response.success = True
        return response

    def remove_room_callback(self, request, response):
        # Assume request has the following fields:
        # request.id
        room_idx = next((idx for idx, room in enumerate(self.rooms) if room['id'] == 1), None)
        if room_idx is not None:
            del self.rooms[room_idx]
            self.save_rooms()
            response.success = True
        else:
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)

    rooms_service_node = RoomsManagerServer()
    rclpy.spin(rooms_service_node)
    rooms_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main


