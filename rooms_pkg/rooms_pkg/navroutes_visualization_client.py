import rclpy
from rclpy.node import Node
from custom_interfaces.srv import NavroutesServiceMessage

class NavRoutesVisualizationClient(Node):
    def __init__(self):
        super().__init__('navroutes_visualization_client')
        self.cli = self.create_client(NavroutesServiceMessage, 'navroutes_visualization_server')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Server not available, waiting...')

    def send_request(self, room, route):
        request = NavroutesServiceMessage.Request()
        request.room = room
        request.route = route

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result().output}')
        else:
            self.get_logger().info('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    client = NavRoutesVisualizationClient()
    client.send_request('office', 1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




