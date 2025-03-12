##########     Description    #########
# This node subscribes the topic which will be used for the ML model 
# If the topic does not have timestamp with the same clock source 
# ,then you can create a new msg with :from std_msgs.msg import Header
# ,finally the node publishes the topic with the timestamp from the same clock. 



#import dependencies
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Header

# import interfaces： py code from<package_name>.msg import <MessageType>
# use cmd to list all interfaces: ros2 interface list
from sensor_msgs.msg import Imu 
from motion_msgs.msg import LegMotors
from motion_msgs.msg import MotionCtrl, MotionCtrlstamp
from ception_msgs.msg import IMUEuler

# ADD Dependencies in package.xml file 

# subscribe the all topics, when a data recieved, add a ROS2 clock timestamp and publish them in real-time
class SensorDataSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_data_sync_node')
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'synced/imu', 10)
        # (messages type, 'name of the topic')
        self.leg_motors_pub = self.create_publisher(LegMotors, 'synced/leg_motors', 10)
        self.motion_ctrl_pub = self.create_publisher(MotionCtrlstamp, 'synced/motion_ctrl', 10)
        # because the MotionCtrl message does not have timestamp, therefore I create a msg. 
        self.imu_euler_pub = self.create_publisher(IMUEuler, 'synced/imu_euler', 10)
        
        # Create subscribers
        self.create_subscription(Imu, 'diablo/sensor/Imu', self.imu_callback, 10)
        self.create_subscription(LegMotors, 'diablo/sensor/LegMotors', self.leg_motors_callback, 10)
        self.create_subscription(MotionCtrl, 'diablo/MotionCmd', self.motion_ctrl_callback, 10)
        self.create_subscription(IMUEuler, 'diablo/sensor/ImuEuler', self.imu_euler_callback, 10)

    def imu_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        # it uses the ROS2 system colck so ROS2 should be installed correctly
        self.imu_pub.publish(msg)

    def leg_motors_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.leg_motors_pub.publish(msg)
    
    def motion_ctrl_callback(self, msg):
        stamped_msg = MotionCtrlstamp()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()  # Assign ROS time
        stamped_msg.motion_ctrl = msg  # Keep the original message data

        self.motion_ctrl_pub.publish(stamped_msg)
    def imu_euler_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_euler_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorDataSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
