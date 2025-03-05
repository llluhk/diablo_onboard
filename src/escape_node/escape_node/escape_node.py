import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl  # 机器人控制指令
from std_msgs.msg import Int32  # 用于接收预测的整数类型

class EscapeBehaviorNode(Node):
    def __init__(self):
        super().__init__('escape_node')

        # 发布脱困指令
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd_escape', 10)

        # 订阅最新的 MotionCmd 数据
        self.latest_motion_cmd = MotionCtrl()  # 保存最新的控制指令
        self.create_subscription(MotionCtrl, '/diablo/MotionCmd', self.motion_cmd_callback, 10)

        # 订阅预测结果
        self.subscription = self.create_subscription(Int32, '/collision_prediction', self.callback, 1)

        # 计时器
        self.escape_timer = None  # 负责执行运动后停止
        self.phase_timer = None  # 负责分阶段控制后退和旋转

    def motion_cmd_callback(self, msg):
        """处理 /diablo/MotionCmd 订阅的消息"""
        self.latest_motion_cmd = msg  # 保存最新的控制指令

    def callback(self, msg): 
        """处理 /collision_prediction 订阅的消息"""
        prediction = int(msg.data)
        self.get_logger().info(f"收到 collision_prediction: {prediction}")
        self.execute_escape_behavior(prediction)

    def execute_escape_behavior(self, prediction):
        """根据预测结果执行不同的脱困策略"""
        self.clear_timers()  # 每次执行之前先清除定时器，防止重复执行

        if prediction == 1:
            self.get_logger().info("脱困策略 1: 仅后退")
            self.publish_motion_command(-1.0, 0.0)  
            self.escape_timer = self.create_timer(0.5, self.stop_motion)

        elif prediction == 2:
            self.get_logger().info("脱困策略 2: 先后退 0.5s 再右转 0.5s")
            self.publish_motion_command(-1.0, 0.0)  
            self.phase_timer = self.create_timer(0.5, self.turn_right)

        elif prediction == 3:
            self.get_logger().info("脱困策略 3: 先后退 0.5s 再左转 0.5s")
            self.publish_motion_command(-1.0, 0.0)  
            self.phase_timer = self.create_timer(0.5, self.turn_left)

    def turn_right(self):
        """右转 0.5s"""
        self.clear_timers()  # 清除之前的定时器，防止重复执行
        self.get_logger().info("后退结束，执行右转 0.5s")
        self.publish_motion_command(0.0, 1.0)
        self.escape_timer = self.create_timer(0.5, self.stop_motion)

    def turn_left(self):
        """左转 0.5s"""
        self.clear_timers()
        self.get_logger().info("后退结束，执行左转 0.5s")
        self.publish_motion_command(0.0, -1.0)
        self.escape_timer = self.create_timer(0.5, self.stop_motion)

    def stop_motion(self):
        """停止机器人运动"""
        self.clear_timers()  # 确保停止后不会重复触发
        self.get_logger().info("停止运动")
        self.publish_motion_command(0.0, 0.0)

    def clear_timers(self):
        """清除所有定时器，防止重复执行"""
        if self.phase_timer is not None:
            self.phase_timer.cancel()
            self.phase_timer = None

        if self.escape_timer is not None:
            self.escape_timer.cancel()
            self.escape_timer = None

    def publish_motion_command(self, forward, left):
        """根据上一次的控制指令发布新的运动指令"""
        # 继承其他数据并仅更改 forward 和 left
        motion_cmd = MotionCtrl()
        motion_cmd.value = self.latest_motion_cmd.value  # 继承之前的数据
        motion_cmd.value.forward = forward
        motion_cmd.value.left = left

        self.publisher.publish(motion_cmd)
        self.get_logger().info(f"已发布运动指令: forward={forward}, left={left}")


def main(args=None):
    rclpy.init(args=args)
    node = EscapeBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
