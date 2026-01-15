import rclpy
from rclpy.node import Node
import math
from dynamixel_sdk import PortHandler, PacketHandler

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.serial_port = '/dev/ttyUSB0'
        self.serial_baud = 1000000
        self.dxl_id = 2

        self.ADDR_PRESENT_POSITION = 132

        self.port_handler = PortHandler(self.serial_port)
        self.packet_handler = PacketHandler(2.0)

        if self.port_handler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().error("Failded to open the port")
            return

        if self.port_handler.setBaudRate(self.serial_baud):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().error("Failed to change the baudrate")

        self.timer = self.create_timer(0.5, self.read_motor_angle)
    
    def read_motor_angle(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, self.ADDR_PRESENT_POSITION)

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Comm Failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Dxl Error: {self.packet_handler.getTxRxResult(dxl_error)}")
        else:
            rad_angle = dxl_present_position * (2.0 * math.pi / 4095.0)
            self.get_logger().info(f"Angle: {rad_angle:.4f} rad")

    def stop(self):
        self.port_handler.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()