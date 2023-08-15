import rclpy
from rclpy.node import Node
from threading import Thread
import serial
from time import sleep
from sonia_common_py.serial_interface import SerialInterface, SerialData
from sonia_common_ros2.msg import SerialMessage

class SerialNodeStream(Node):
    """
    Serial Node to stream data to and from port.
    """

    def __init__(self):
        super().__init__('serial_node_stream')
        self.__serial_interface = None
        self.__try_connect_serial()
        self.__serial_publisher = self.create_publisher(SerialMessage, '/serial/out', 10)
        self.__listening_thread = Thread(target=self.__serial_out_worker, daemon=True)
        self.__listening_alive = True

        self.__serial_subscriber = self.create_subscription(SerialMessage, '/serial/in', self.__serial_in_callback, 10)

        self.__listening_thread.start()

        self.get_logger().info("Serial Node Ready!")

    def __serial_out_worker(self):
        self.get_logger().info("Serial Node Listener Ready!")
        while self.__listening_alive:
            msg: SerialData = self.__serial_interface.data_in_queue.get()
            pub_msg = SerialMessage()
            pub_msg.id = msg.id
            pub_msg.size = msg.size
            pub_msg.msg = msg.msg
            self.get_logger().info(f"Publishing msg: id={msg.id!r}, size={msg.size!r}, msg={msg.msg!r}")
            self.__serial_publisher.publish(pub_msg)

    def __serial_in_callback(self, msg: SerialMessage):
        try:
            serial_msg = SerialData(id=msg.id, size=msg.size, msg=msg.msg)
            self.get_logger().info(f"Transmiting msg: id={msg.id!r}, size={msg.size!r}, msg={msg.msg!r}")
            self.__serial_interface.transmit(serial_msg)
        except serial.serialutil.SerialException as ex:
            self.get_logger().error("Failed to write, trying to connect...")
            self.__try_connect_serial()

    def __try_connect_serial(self, attempts=0):
        counter = 1
        while True and counter != attempts:
            self.get_logger().info("Trying to connect to port")
            try:
                if self.__serial_interface is not None:
                    self.__serial_interface.stop()
                self.__serial_interface = SerialInterface('/dev/ttyS5', 115200)
                self.__serial_interface.start()
                break
            except OSError as ex:
                self.get_logger().error("Failed to connect to Serial Port")
                self.get_logger().error("".join(ex.args))
            except Exception as ex:
                self.get_logger().error("".join(ex.args))
            counter += 1
            sleep(1)


def main(args=None):
    rclpy.init(args=args)

    serial_node = SerialNodeStream()

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()