import sys
import rclpy
from rclpy.node import Node
from sonia_common_ros2.srv import SerialService
from sonia_common_ros2.msg import SerialMessage

class SerialNodeClient(Node):

    def __init__(self):
        super().__init__('serial_node_client')
        self.__client = self.create_client(SerialService, "serial_server")
        while not self.__client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.__req = SerialService.Request()

    def send_request(self, _id, size, message=None):
        ser_msg = SerialMessage()
        ser_msg.id = int(_id)
        ser_msg.size = int(size)
        if message is None or size == 0:
            ser_msg.msg = []
        else:
            ser_msg.msg = [int(message).to_bytes(ser_msg.size, 'big')]
        self.__req.data = ser_msg
        self.future = self.__client.call_async(self.__req)

def main(args=None):
    rclpy.init(args=args)

    serial_node_client = SerialNodeClient()
    if len(sys.argv) == 4:
        serial_node_client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 3:
        serial_node_client.send_request(sys.argv[1], sys.argv[2])
    else:
        serial_node_client.destroy_node()
        rclpy.shutdown()
        return

    while rclpy.ok():
        rclpy.spin_once(serial_node_client)
        if serial_node_client.future.done():
            try:
                response = serial_node_client.future.result()
            except Exception as ex:
                serial_node_client.get_logger().info('Service call failed %r' % (ex,))
            else:
                serial_node_client.get_logger().info(f"Status: {response.status}")# CHANGE
                if response.status == SerialService.Response.SUCCESS:
                    serial_node_client.get_logger().info(f"Result => id: {response.data.id!r}, size: {response.data.size!r}, msg: {response.data.msg!r}")
            break

    serial_node_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
