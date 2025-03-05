#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import sys
import tty
import termios
import threading
import csv

from sensor_msgs.msg import Imu

class KeyLoggerNode(Node):
    def __init__(self):
        super().__init__('key_logger_node')
        self.last_imu_stamp = None  # 用于存储最新的 IMU 时间戳
        self.setup_csv()
        self.setup_keyboard()

        # 订阅 IMU 话题
        self.imu_sub = self.create_subscription(
            Imu,
            '/diablo/sensor/Imu',
            self.imu_callback,
            10
        )

        self.get_logger().info('Key Logger Node Started. Press 1 for front; 2 for left; 3 for right, ` to exit.')

    def imu_callback(self, msg: Imu):
        # 更新最后一次收到的 IMU 时间戳
        self.last_imu_stamp = msg.header.stamp

    def setup_csv(self):
        directory = "/home/pc/BA_data_record/"  # 确保目录结构正确
        os.makedirs(directory, exist_ok=True)  # 创建目录（如果不存在）
        now = self.get_clock().now()  # 获取 ROS2 系统时间
        sec, nanosec = now.seconds_nanoseconds()  # 获取秒和纳秒
        timestamp = f"{sec}_{nanosec:09d}"  # 格式化时间戳为文件名
        self.csv_path = f"{directory}key_log_{timestamp}.csv"
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Key', 'Timestamp_sec', 'Timestamp_nanosec'])

    def setup_keyboard(self):
        self.key_thread = threading.Thread(target=self.key_monitor)
        self.key_thread.daemon = True
        self.key_thread.start()

    def read_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def key_monitor(self):
        while rclpy.ok():
            key = self.read_key()  # 读取键盘输入
            if key == '`':
                self.get_logger().info('Exiting...')
                rclpy.shutdown()
                break
            elif key in ['1', '2', '3']:  # 检查是否按下了 0, 1, 2 或 3
                # 如果已接收到 IMU 数据，则使用 IMU 时间戳，否则使用当前时间
                if self.last_imu_stamp is not None:
                    sec = self.last_imu_stamp.sec
                    nanosec = self.last_imu_stamp.nanosec
                else:
                    now = self.get_clock().now()
                    sec, nanosec = now.seconds_nanoseconds()

                # 将按键值和时间戳写入 CSV 文件
                with open(self.csv_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([key, sec, nanosec])  # 记录按键值和时间戳
                self.get_logger().info(f'Key "{key}" pressed. Timestamp logged: {sec}.{nanosec:09d}')

def main():
    rclpy.init()
    node = KeyLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


