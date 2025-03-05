import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import os

class ProcessedDataSaver(Node):
    def __init__(self):
        super().__init__('processed_data_saver')

        # 订阅 /processed_data 话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_data',
            self.data_callback,
            10
        )

        # CSV 文件路径
        self.csv_file = '/home/pc/BA_data_record/processed_diablo_data.csv'

        # 如果文件不存在，创建并写入表头
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["ROS_Time_Sec", "ROS_Time_Nsec"] + [f'Feature_{i+1}' for i in range(115)])  

        self.get_logger().info("ProcessedDataSaver Node Started")

    def data_callback(self, msg):
        """接收 /processed_data 话题的数据并存入 CSV 文件"""

        # 获取 ROS2 系统时间
        ros_time = self.get_clock().now().seconds_nanoseconds()
        ros_time_sec = ros_time[0]  # 秒
        ros_time_nsec = ros_time[1]  # 纳秒

        # 获取特征数据
        feature_data = list(msg.data)  # 确保数据是列表

        # 保存数据到 CSV
        self.save_to_csv(ros_time_sec, ros_time_nsec, feature_data)

    def save_to_csv(self, sec, nsec, data):
        """将数据存入 CSV 文件"""
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([sec, nsec] + data)  # 第一列 ROS2 秒，第二列纳秒，后面是特征数据
        self.get_logger().info(f"Saved data to CSV with ROS time: {sec}.{nsec}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessedDataSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
