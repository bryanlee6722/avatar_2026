import math
import rclpy
from rclpy.node import Node

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead
from sensor_msgs.msg import JointState

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
        # 다이나믹셀 id, 관절이름 리스트
        self.declare_parameter('joint_names', ["joint_L1", "joint_L2", "joint_L3", "joint_L4", "joint_L5", "joint_L6", "joint_L7",
                                       "joint_R1", "joint_R2", "joint_R3", "joint_R4", "joint_R5", "joint_R6", "joint_R7"])
        self.declare_parameter('dxl_ids', [1,2,3,4,5,6,7,8,9,10,11,12,13,14])

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baud').value)
        self.dxl_ids = self.get_parameter('dxl_ids').value
        self.joint_names = self.get_parameter('joint_names').value

        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        if self.port_handler.openPort():
            print("Succeeded to open the port!")
        else:
            raise RuntimeError(f'Falied to open port: {port}')
        
        if self.port_handler.setBaudRate(baud):
            print("Succeeded to change the baudrate!")
        else:
            raise RuntimeError(f'Falied to set baudrate: {baud}')

        for dxl_id in self.dxl_ids:
            if not self.group_sync_read.addParam(dxl_id):
                raise RuntimeError(f"ID:failliure add {dxl_id} in list")
        self.create_timer(0.5, self.tick)

    def tick(self):

        valid_positions = []
        valid_names = []

        comm_result = self.group_sync_read.txRxPacket()

        if comm_result != 0:
            self.get_logger().warn(f"GroupSyncRead Failure: {comm_result}")

        for dxl_id, j_names in zip(self.dxl_ids, self.joint_names):

            if self.group_sync_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raw_angle = self.group_sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                rad_angle = raw_angle_to_rad(raw_angle)

                #읽기 성공했을 때만 리스트에 추가
                valid_positions.append(rad_angle)
                valid_names.append(j_names)

            else:
                self.get_logger().warn(f"ID {dxl_id} ({j_names}) failure")

        if valid_names: 
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = valid_names
            msg.position = valid_positions

            self.pub.publish(msg)

            print(f"Published {len(valid_names)} joint GroupSyncRead")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TestNode()
        rclpy.spin(node) #spin은 이 노드를 실행하라는 뜻
    except RuntimeError as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__' :
    main()