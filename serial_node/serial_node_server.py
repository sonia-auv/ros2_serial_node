import rclpy
from rclpy.node import Node
import serial
from argparse import ArgumentParser
from queue import Empty
from time import sleep
from sonia_common_py.serial_interface import SerialInterface, SerialData
from sonia_common_ros2.msg import SerialMessage
from sonia_common_ros2.srv import SerialService

class SerialNodeServer(Node):
    """
    Serial Node Server for requests.
    """
    def __init__(self, port="/dev/ttyS5", baud=115200):
        super().__init__('serial_node_server')
        self.__port = port
        self.__baud = baud
        self.__serial_interface = None
        self.__try_connect_serial()

        self.__serial_server = self.create_service(SerialService, "serial_server", self.__serial_server_cb)

        self.get_logger().info("Serial Server Node Ready!")

    def __try_connect_serial(self, attempts=0):
        counter = 1
        while True and counter != attempts:
            self.get_logger().info("Trying to connect to port")
            try:
                if self.__serial_interface is not None:
                    self.__serial_interface.stop()
                self.__serial_interface = SerialInterface(self.__port, self.__baud)
                self.__serial_interface.start()
                break
            except OSError as ex:
                self.get_logger().error("Failed to connect to Serial Port")
                self.get_logger().error("".join(ex.args))
            except Exception as ex:
                self.get_logger().error("".join(ex.args))
            counter += 1
            sleep(1)

    def __serial_server_cb(self, request, response):
        data: SerialData = request.data
        try:
            if self.__serial_interface.transmit(data):
                msg: SerialData = self.__serial_interface.data_in_queue.get(timeout=5)
                pub_msg = SerialMessage()
                pub_msg.id = msg.id
                pub_msg.size = msg.size
                pub_msg.msg = msg.msg
                response.status = SerialService.Response.SUCCESS
                response.data = pub_msg
            else:
                response.status = SerialService.Response.CONNECTION_FAILED
                self.__try_connect_serial(10)
                if not self.__serial_interface.transmit(data):
                    raise serial.SerialException()
                msg: SerialData = self.__serial_interface.data_in_queue.get(timeout=5)
                pub_msg = SerialMessage()
                pub_msg.id = msg.id
                pub_msg.size = msg.size
                pub_msg.msg = msg.msg
                response.status = SerialService.Response.SUCCESS
                response.data = pub_msg
        except Empty as ex:
            self.get_logger().error("No message received")
            response.status = SerialService.Response.NO_RESPONSE
            # response.data = SerialMessage()
        except serial.SerialException as ex:
            self.get_logger().error("Failed to connect to Serial Port")
            self.get_logger().error("".join(ex.args))
            self.__try_connect_serial(10)
            self.__serial_interface.transmit(data)
            response.status = SerialService.Response.CONNECTION_FAILED
        except Exception as ex:
            self.get_logger().error("Unknown exception ocured")
            self.get_logger().error("".join(ex.args))
            response.status - SerialService.Response.FAILED
        return response


def main(args=None):
    arg_parse = ArgumentParser()
    arg_parse.add_argument('-p', '--port', default="/dev/ttyS5")
    arg_parse.add_argument('-b', '--baud', default=115200)
    parsed_args = arg_parse.parse_args()

    rclpy.init(args=args)

    serial_node = SerialNodeServer(parsed_args.port, parsed_args.baud)

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()