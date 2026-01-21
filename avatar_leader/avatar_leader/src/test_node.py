#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead


class XL430MultiEncoderNode(Node):
    ADDR_PRESENT_POSITION = 132 
    LEN_PRESENT_POSITION = 4

    def __init__(self):
        super().__init__('xl430_multi_encoder_node')

        self.declare_parameter('device_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('protocol_version', 2.0)

        self.declare_parameter('joint_names', ['joint_1'])
        self.declare_parameter('dxl_ids', [2])

        self.device_name = self.get_parameter('device_name').get_parameter_value().string_value
        self.baudrate = int(self.get_parameter('baudrate').get_parameter_value().integer_value)
        self.protocol_version = float(self.get_parameter('protocol_version').get_parameter_value().double_value)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.dxl_ids = list(self.get_parameter('dxl_ids').value)

        if len(self.joint_names) != len(self.dxl_ids):
            raise RuntimeError(
                f'joint_names(len={len(self.joint_names)}) and dxl_ids(len={len(self.dxl_ids)}) must match.'
            )

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        if not self.port_handler.openPort():
            raise RuntimeError(f'Failed to open port: {self.device_name}')

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f'Failed to set baudrate={self.baudrate} on {self.device_name}')

        self.get_logger().info(
            f'Connected: device={self.device_name}, baudrate={self.baudrate}, protocol={self.protocol_version}'
        )
        self.get_logger().info(
            f'Motors: {list(zip(self.joint_names, self.dxl_ids))}'
        )

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.ADDR_PRESENT_POSITION,
            self.LEN_PRESENT_POSITION
        )

        for dxl_id in self.dxl_ids:
            ok = self.group_sync_read.addParam(int(dxl_id))
            if not ok:
                raise RuntimeError(f'GroupSyncRead.addParam failed for dxl_id={dxl_id}')

        self.timer = self.create_timer(0.5, self._on_timer)

    @staticmethod
    def _ticks_to_rad(ticks: int) -> float:
        return float(ticks) * (2.0 * math.pi / 4096.0)

    @staticmethod
    def _to_int32_signed(raw_u32: int) -> int:
        if raw_u32 >= (1 << 31):
            return raw_u32 - (1 << 32)
        return raw_u32

    def _on_timer(self):
        try:
            dxl_comm_result = self.group_sync_read.txRxPacket()
            if dxl_comm_result != 0:
                msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                raise RuntimeError(f'GroupSyncRead txRxPacket failed: {msg}')
            
            for joint_name, dxl_id in zip(self.joint_names, self.dxl_ids):
                dxl_id = int(dxl_id)

                if not self.group_sync_read.isAvailable(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION):
                    self.get_logger().warn(f'[{joint_name} id={dxl_id}] data not available')
                    continue

                raw_u32 = self.group_sync_read.getData(
                    dxl_id,
                    self.ADDR_PRESENT_POSITION,
                    self.LEN_PRESENT_POSITION
                )

                ticks = self._to_int32_signed(int(raw_u32))
                rad = self._ticks_to_rad(ticks)

                print(f'[{joint_name} id={dxl_id}] ticks={ticks}  rad={rad:.6f}')

        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = XL430MultiEncoderNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.port_handler.closePort()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
