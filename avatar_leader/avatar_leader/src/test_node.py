import math

from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ADDR_PRESENT_POSITION=132
LEN_PRESENT_POSTION=4

def raw_angle_to_rad(raw_angle):
    return float(raw_angle - 1600) / 4096.0 * 2 * math.pi

class TestNode(Node):
    
    def __init__(self):
        super().__init__('test_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 1000000)
        self.declare_parameter('dxl_id', 2)
        

        port = self.get_parameter('serial_port').value
        baudrate = int(self.get_parameter('serial_baudrate').value)
        self.dxl_id = int(self.get_parameter('dxl_id').value)
        

        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.port_handler=PortHandler(port)
        self.packet_handler=PacketHandler(2.0)

        if self.port_handler.openPort():
            print("Succeeded to open the port!")
        else:
            raise RuntimeError(f'Failed to open port: {port}')
        
        if self.port_handler.setBaudRate(baudrate):
            print("Succeeded to change the baudrate!")
        else:
            raise RuntimeError(f'Failed to set baudrate: {baudrate}')
        

        self.timer=self.create_timer(0.5, self.tick)



    def tick(self):
        raw_angle, comm_result, error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION)

        if comm_result !=0 or error !=0:
            self.get_logger().warn(f"DXL read failed: comm = {comm_result}")
            return
        
        rad_angle = raw_angle_to_rad(int(raw_angle))

        self.get_logger().info(
            f"[DXL] raw={raw_angle}, rad={rad_angle:.4f}",
            throttle_duration_sec=1.0
        )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg
        msg.name = ["Joint1"]
        msg.position = [float(rad_angle)]
        self.pub.publish(msg)

  
    def main():
        rclpy.init()
        node=TestNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()