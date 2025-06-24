import rclpy
from rclpy.node import Node
import pygame
import os

from custom_msgs.msg import Collisionprediction  # Replace with your actual package name

class AlarmNode(Node):
    def __init__(self):
        super().__init__('alarm_node')
        self.get_logger().info('Alarm node has started.')

        # Initialize pygame mixer
        pygame.mixer.init()
        self.is_playing = False

        # Define sound map: direction class → mp3 filename
        self.sound_map = {
            1: 'front.mp3',
            2: 'left.mp3',
            3: 'right.mp3'
        }

        # Create subscriber to collision prediction topic
        self.subscriber = self.create_subscription(
            Collisionprediction,
            '/collision_prediction',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        direction = msg.prediction

        if self.is_playing:
            return  # Don't play overlapping sounds

        if direction in self.sound_map:
            mp3_file = self.sound_map[direction]
            mp3_path = os.path.join(os.path.dirname(__file__), mp3_file)

            try:
                pygame.mixer.music.load(mp3_path)
                pygame.mixer.music.play()
                self.get_logger().info(f'Playing: {mp3_file}')
                self.is_playing = True
                self.timer = self.create_timer(0.1, self.check_if_done)
            except Exception as e:
                self.get_logger().error(f'Error playing {mp3_file}: {e}')
        else:
            self.get_logger().warn(f"Unknown collision direction: {direction}")

    def check_if_done(self):
        if not pygame.mixer.music.get_busy():
            self.get_logger().info('Finished playing sound.')
            self.is_playing = False
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = AlarmNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
