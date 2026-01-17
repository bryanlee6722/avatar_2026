import math

from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node

ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
PROTOCOL_VERSION = 2.0

def raw_angle_to_rad(raw_angle):
    return float(raw_angle - 1600) / 4096 * 2 * math.pi

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

    
        # 포트 정보
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        # 통신 속도, 초당 몇 비트로 주고받는지 숫자 보드rate
        self.declare_parameter('serial_baud', 1000000)
        # 다이나믹셀 id 설정
        self.declare_parameter('dxl_id', 2)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baud').value)
        self.dxl_id = int(self.get_parameter('dxl_id').value)     

        self.port_handler = PortHandler('/dev/ttyUSB0')
        self.packet_handler = PacketHandler(2.0)

        if self.port_handler.openPort():
            print("Succeeded to open the port!")
        else:
            raise RuntimeError(f'Falied to open port: {port}')
        
        if self.port_handler.setBaudRate(baud):
            print("Succeeded to change the baudrate!")
        else:
            raise RuntimeError(f'Falied to set baudrate: {baud}')
    
        self.create_timer(0.5, self.tick)

    def tick(self):
        raw_bytes = []
        for i in range(LEN_PRESENT_POSITION):
            data, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION + i)
            
            if result != 0:
                print(f"TICK ERROR")
                return
            
            raw_bytes.append(data)

        raw_angle = sum(b << (8 * i) for i, b in enumerate(raw_bytes))

        rad_angle = raw_angle_to_rad(raw_angle)

        print(f"Rad: {rad_angle:.4f}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TestNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__' :
    main()