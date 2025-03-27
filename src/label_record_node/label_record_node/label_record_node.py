#!/usr/bin/env python3
import rclpy
import os
import sys
import tty
import termios
import threading
import csv
import time
from datetime import datetime
from rclpy.node import Node
from sensor_msgs.msg import Imu

print("Teleop start now!")
print("Press '`' to exit!")
print("Press 1 for front; 2 for left; 3 for right")

keyQueue = []
old_setting = termios.tcgetattr(sys.stdin)

class TeleopNode(Node):
    def __init__(self):
        super().__init__("diablo_teleop_node")
        self.last_imu_stamp = None
        self.running = True  # Flag to control the keyboard thread
        self.setup_csv()
        self.setup_keyboard()

    def setup_csv(self):
        directory = "/home/pc/BA_data_record/"
        os.makedirs(directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"{directory}label_{timestamp}.csv"
        with open(self.csv_path, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['Key', 'Timestamp_sec', 'Timestamp_nanosec'])

    def setup_keyboard(self):
        self.key_thread = threading.Thread(target=self.getKeyBoard, daemon=False)  # Ensure the thread is stoppable
        self.key_thread.start()

    def readchar(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def getKeyBoard(self):
        global keyQueue
        while self.running:
            try:
                c = self.readchar()
                keyQueue.append(c)
            except Exception as e:
                break  # Exit thread on error

    def record_key_with_timestamp(self, key):
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        with open(self.csv_path, 'a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([key, sec, nanosec])
        self.get_logger().info(f'Key "{key}" pressed. Timestamp: {sec}.{nanosec:09d}')


def main(args=None):
    global keyQueue
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        while rclpy.ok():  # Use ROS2's loop check
            if keyQueue:
                key = keyQueue.pop(0)
                if key == '`':  # Exit condition
                    print("Exiting program...")
                    break
                if key in ['1', '2', '3']:
                    node.record_key_with_timestamp(key)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Shutting down...")

    finally:
        node.running = False  # Stop the keyboard thread
        node.key_thread.join()  # Wait for the thread to exit
        node.destroy_node()  # Clean up the node
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)  # Restore terminal settings
        rclpy.shutdown()
        print("Exit!")


if __name__ == '__main__':
    main()
