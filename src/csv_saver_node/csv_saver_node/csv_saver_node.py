###################   Description   ####################
# This node subscribes the synchronized topics 
# This node save the topics in a csv file for offline works.


from custom_msgs.msg import SyncedData  
import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime

class DataSubscriberNode(Node):

    def __init__(self):
        super().__init__('data_subscriber_node')
        self.subscription_data = self.create_subscription(
            SyncedData, 'synced_data', self.log_to_csv, 10)

        self.initialize_csv_file()

    def initialize_csv_file(self):
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            file_path = f"/home/pc/BA_data_record/diablo_data_{timestamp}.csv"
            
            self.data_file = open(file_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.data_file)
            
            # Write the CSV headers
            self.csv_writer.writerow([
                'ROS2 Timestamp (sec)', 'ROS2 Timestamp (nanosec)',
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
                'left_hip_enc_rev', 'left_hip_pos', 'left_hip_vel', 'left_hip_iq',
                'left_knee_enc_rev', 'left_knee_pos', 'left_knee_vel', 'left_knee_iq',
                'left_wheel_enc_rev', 'left_wheel_pos', 'left_wheel_vel', 'left_wheel_iq',
                'right_hip_enc_rev', 'right_hip_pos', 'right_hip_vel', 'right_hip_iq',
                'right_knee_enc_rev', 'right_knee_pos', 'right_knee_vel', 'right_knee_iq',
                'right_wheel_enc_rev', 'right_wheel_pos', 'right_wheel_vel', 'right_wheel_iq',
                'left_leg_length', 'right_leg_length',
                'roll', 'pitch', 'yaw',
                'cmd_forward', 'cmd_left', 'cmd_up', 'cmd_roll', 'cmd_pitch', 'cmd_leg_split'
            ])
            self.get_logger().info(f"CSV file created: {file_path}")
        
        except Exception as e:
            self.get_logger().error(f"Exception during CSV file initialization: {e}")


    def log_to_csv(self, msg):
        try:
            # Write to CSV file
            self.csv_writer.writerow([
                msg.imu.header.stamp.sec, msg.imu.header.stamp.nanosec,
                msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w,
                msg.imu.angular_velocity.x, msg.imu.angular_velocity.y, msg.imu.angular_velocity.z,
                msg.imu.linear_acceleration.x, msg.imu.linear_acceleration.y, msg.imu.linear_acceleration.z,
                msg.leg_motors.left_hip_enc_rev, msg.leg_motors.left_hip_pos, msg.leg_motors.left_hip_vel, msg.leg_motors.left_hip_iq,
                msg.leg_motors.left_knee_enc_rev, msg.leg_motors.left_knee_pos, msg.leg_motors.left_knee_vel, msg.leg_motors.left_knee_iq,
                msg.leg_motors.left_wheel_enc_rev, msg.leg_motors.left_wheel_pos, msg.leg_motors.left_wheel_vel, msg.leg_motors.left_wheel_iq,
                msg.leg_motors.right_hip_enc_rev, msg.leg_motors.right_hip_pos, msg.leg_motors.right_hip_vel, msg.leg_motors.right_hip_iq,
                msg.leg_motors.right_knee_enc_rev, msg.leg_motors.right_knee_pos, msg.leg_motors.right_knee_vel, msg.leg_motors.right_knee_iq,
                msg.leg_motors.right_wheel_enc_rev, msg.leg_motors.right_wheel_pos, msg.leg_motors.right_wheel_vel, msg.leg_motors.right_wheel_iq,
                msg.leg_motors.left_leg_length, msg.leg_motors.right_leg_length,
                msg.imu_euler.roll, msg.imu_euler.pitch, msg.imu_euler.yaw,
                msg.motion_ctrl.motion_ctrl.value.forward, msg.motion_ctrl.motion_ctrl.value.left, msg.motion_ctrl.motion_ctrl.value.up,
                msg.motion_ctrl.motion_ctrl.value.roll, msg.motion_ctrl.motion_ctrl.value.pitch, msg.motion_ctrl.motion_ctrl.value.leg_split
            ])
            self.get_logger().info("Synchronized data written to CSV")

        except Exception as e:
            self.get_logger().error(f"Error writing to CSV: {e}")


    def destroy_node(self):
        """Closes the CSV file when the node is destroyed."""
        try:
            if self.data_file:
                self.data_file.close()
                self.get_logger().info("CSV file closed")
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()