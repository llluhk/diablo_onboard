####################     Description         ###############
# This node subscribe the topics from timestamp_processing_node 
# This node synchronizes the topics with ROS2 package Approcimate Time Synchronizer

# ADD dependencies
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Add interfaces
from sensor_msgs.msg import Imu 
from motion_msgs.msg import LegMotors
from motion_msgs.msg import MotionCtrlstamp
from ception_msgs.msg import IMUEuler
from custom_msgs.msg import SyncedData  

class TimeSyncNode(Node):

    def __init__(self):
        super().__init__('sync_node')

        self.imu_sub = Subscriber(Imu, 'synced/imu', 10)
        # (messages type, 'name of the topic') make sure the QoS = 10 for publishers and subscribers
        self.leg_motors_sub = Subscriber(LegMotors, 'synced/leg_motors', 10)
        self.motion_ctrl_sub = Subscriber(MotionCtrlstamp, 'synced/motion_ctrl', 10)
        self.imu_euler_sub = Subscriber(IMUEuler, 'synced/imu_euler', 10)
        self.synced_data_publisher = self.create_publisher(SyncedData, 'synced_data', 10)
        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.imu_sub, self.leg_motors_sub,self.motion_ctrl_sub,self.imu_euler_sub],
                                                     queue_size, max_delay)
        
        
        self.time_sync.registerCallback(self.SyncCallback)
        # 
    
    def sync_callback(self, imu_msg, motors_msg, motion_ctrl_msg, imu_euler_msg):
        # Use ROS2 system time as the unified timestamp
        unified_timestamp = self.get_clock().now()
        # Create SyncedData message
        synced_data = SyncedData()
        synced_data.header.stamp = unified_timestamp
        synced_data.imu = imu_msg
        synced_data.leg_motors = motors_msg
        synced_data.imu_euler = imu_euler_msg
        synced_data.motion_ctrl = motion_ctrl_msg

        # Publish the synchronized data
        self.synced_data_publisher.publish(synced_data)

def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

