import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class KeyboardPublisherSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_pub_sub')

        # 发布者
        self.publisher = self.create_publisher(String, 'keyboard_input', 10)
        self.get_logger().info("Publishing keyboard inputs. Type '0' to simulate the key press.")

        # 订阅者
        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.listener_callback,
            10
        )
        self.file_path = 'key_press_times.txt'  # 输出文件路径

        # 打开文件并添加标题
        with open(self.file_path, 'w') as f:
            f.write("Key Press Times for '0':\n")

    def run(self):
        try:
            while rclpy.ok():
                user_input = input("Enter a key: ")
                msg = String()
                msg.data = user_input
                self.publisher.publish(msg)
                self.get_logger().info(f"Published: {user_input}")
        except KeyboardInterrupt:
            pass

    def listener_callback(self, msg):
        if msg.data == '0':  # 检测是否为 "0" 键
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            self.get_logger().info(f"Key '0' pressed at {current_time}")
            self.record_time(current_time)

    def record_time(self, time_str):
        with open(self.file_path, 'a') as f:
            f.write(f"{time_str}\n")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisherSubscriber()
    try:
        node.run()  # 开启发布者的输入监听循环
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
