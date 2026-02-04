import rclpy
from rclpy.node import Node
import math
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead
from sensor_msgs.msg import JointState

class MultiMotorNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.declare_parameter('joint_names', ['joint1'])
        self.declare_parameter('dxl_ids', [13])
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('port', '/dev/ttyUSB0')

        self.joint_names = self.get_parameter('joint_names').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        self.baudrate = self.get_parameter('baudrate').value
        self.port = self.get_parameter('port').value

        self.ADDR_PRESENT_POSITION = 132
        self.LEN_PRESENT_POSITION = 4

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(2.0)
        
        self.group_sync_read = GroupSyncRead(
            self.port_handler, 
            self.packet_handler, 
            self.ADDR_PRESENT_POSITION, 
            self.LEN_PRESENT_POSITION
        )

        if not self.port_handler.openPort() or not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error("포트 설정 실패")
            return

        for dxl_id in self.dxl_ids:
            add_param_result = self.group_sync_read.addParam(dxl_id)
            if not add_param_result:
                self.get_logger().error(f"ID:{dxl_id} GroupSyncRead 등록 실패")

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.5, self.tick)

    def raw_to_rad(self, raw_value):
        return float(raw_value) * (2.0 * math.pi / 4096.0)

    def tick(self):
        dxl_comm_result = self.group_sync_read.txRxPacket()
        if dxl_comm_result != 0:
            self.get_logger().error(f"통신 실패: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return

        positions = []
        
        for dxl_id in self.dxl_ids:
            if self.group_sync_read.isAvailable(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION):
                raw_pos = self.group_sync_read.getData(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
                positions.append(self.raw_to_rad(raw_pos))
            else:
                self.get_logger().warn(f"ID:{dxl_id} 데이터를 가져올 수 없습니다.")
                positions.append(0.0)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.joint_pub.publish(msg)
        
        self.get_logger().info(f"Published Positions: {['%.4f' % p for p in positions]}")

    def stop(self):
        self.port_handler.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = MultiMotorNode()
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