import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from custom_msgs.msg import SyncedData  # 替换为自定义消息类型
from std_msgs.msg import Float32MultiArray  # 发布处理后的数组

class DataProcessingNode(Node):
    def __init__(self):
        super().__init__('data_processing_node')

        # 数据维度
        self.feature_dim = 33  # 只处理原始特征
        self.data_buffer = deque(maxlen=50)  # 滑动窗口（可选，当前不使用）

        # 订阅 /synced_data 话题
        self.subscription = self.create_subscription(
            SyncedData, '/synced_data', self.data_callback, 100
        )

        # 发布处理后的滑动窗口数据
        self.publisher = self.create_publisher(Float32MultiArray, '/processed_data', 10)

    def data_callback(self, msg):
        """处理 `/synced_data` 消息，直接提取并发布 `raw_features` (1, 33)"""

        # **提取原始特征**
        raw_features = self.extract_features(msg).reshape(1, -1)
        print(f"[INFO] raw_features.shape: {raw_features.shape}")  # **(1, 33)**

        # **发布数据**
        self.publish_single_sample(raw_features)

    def extract_features(self, msg):
        """从 `/synced_data` 提取 33 维原始特征"""
        imu_data = [
            msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w,
            msg.imu.angular_velocity.x, msg.imu.angular_velocity.y, msg.imu.angular_velocity.z,
            msg.imu.linear_acceleration.x, msg.imu.linear_acceleration.y, msg.imu.linear_acceleration.z
        ]

        motor_data = [
            msg.leg_motors.left_hip_vel, msg.leg_motors.left_hip_iq,
            msg.leg_motors.left_knee_vel, msg.leg_motors.left_knee_iq,
            msg.leg_motors.left_wheel_vel, msg.leg_motors.left_wheel_iq,
            msg.leg_motors.right_hip_vel, msg.leg_motors.right_hip_iq,
            msg.leg_motors.right_knee_vel, msg.leg_motors.right_knee_iq,
            msg.leg_motors.right_wheel_vel, msg.leg_motors.right_wheel_iq,
            msg.leg_motors.left_leg_length, msg.leg_motors.right_leg_length
        ]

        imu_euler_data = [msg.imu_euler.roll, msg.imu_euler.pitch, msg.imu_euler.yaw]
        motion_ctrl_data = [
            msg.motion_ctrl.value.forward, msg.motion_ctrl.value.left,
            msg.motion_ctrl.value.up, msg.motion_ctrl.value.roll,
            msg.motion_ctrl.value.pitch, msg.motion_ctrl.value.leg_split
        ]

        # 合并所有特征
        features = imu_data + motor_data + imu_euler_data + motion_ctrl_data
        assert len(features) == self.feature_dim, f"特征维度不匹配，期望 {self.feature_dim}，但得到 {len(features)}"
        
        print(f"[INFO] extract_features output shape: {np.array(features).shape}")  # **(33,)**
        return np.array(features, dtype=np.float32)

    def publish_single_sample(self, sample):
        """将单条数据 (1, 33) 发布为 Float32MultiArray"""
        msg = Float32MultiArray()
        msg.data = sample.flatten().tolist()
        self.publisher.publish(msg)
        #self.get_logger().info(f"Published single sample with shape: {sample.shape}")

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
