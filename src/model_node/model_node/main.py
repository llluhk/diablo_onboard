import joblib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from custom_msgs.msg import Collisionprediction  # ✅ your custom message
import numpy as np
import time


STAGE1_MODEL_PATH = '/home/carrierbot/BA_Zhu/diablo_collision_ws/src/model_node/checkpoints/clf_stage1.pkl'
STAGE2_MODEL_PATH = '/home/carrierbot/BA_Zhu/diablo_collision_ws/src/model_node/checkpoints/clf_stage2.pkl'
SCALER_PATH = '/home/carrierbot/BA_Zhu/diablo_collision_ws/src/model_node/checkpoints/scaler.pkl'

class CollisionDetectionNode(Node):
    def __init__(self):
        super().__init__('collision_detection_node')

        try:
            self.clf_stage1 = joblib.load(STAGE1_MODEL_PATH)
            self.clf_stage2 = joblib.load(STAGE2_MODEL_PATH)
            self.scaler = joblib.load(SCALER_PATH)
            self.get_logger().info("model successful loaded（Stage 1, Stage 2, Scaler）")
        except Exception as e:
            self.get_logger().error(f"loading models has some problem: {e}")
            self.clf_stage1 = None
            self.clf_stage2 = None
            self.scaler = None

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_data',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            Collisionprediction,
            '/collision_prediction',
            10
        )

        self.durations = []
        self.start_time = time.time()
        self.get_logger().info("Collision Detection Node, waiting for data...")

    def listener_callback(self, msg):
        if self.clf_stage1 is None or self.clf_stage2 is None or self.scaler is None:
            self.get_logger().error("models or scalar are not be loaded")
            return

        sensor_data = np.array(msg.data)
        if sensor_data.ndim == 1:
            sensor_data = sensor_data.reshape(1, -1)

        expected_features = self.clf_stage1.n_features_in_
        if sensor_data.shape[1] != expected_features:
            self.get_logger().error(f" incorrect input data shape, expected: {expected_features}, actual: {sensor_data.shape[1]}")
            return

        try:
            sensor_data_scaled = self.scaler.transform(sensor_data)
        except Exception as e:
            self.get_logger().error(f"scalaring false: {e}")
            return

        try:
            start = time.perf_counter()
            stage1_pred = self.clf_stage1.predict(sensor_data_scaled)[0]
            end = time.perf_counter()

            duration = (end - start) * 1000
            self.durations.append(duration)

            if stage1_pred == 1:
                # Stage 2: predict collision direction
                direction_pred = self.clf_stage2.predict(sensor_data_scaled)[0]

                now = self.get_clock().now().to_msg()
                prediction_msg = Collisionprediction()
                prediction_msg.prediction = int(direction_pred)
                prediction_msg.stamp = now

                self.get_logger().warn(
                    f" detect a collision: {direction_pred} time: {now.sec}.{now.nanosec:09d}"
                )
                self.publisher.publish(prediction_msg)

            if time.time() - self.start_time >= 10:
                avg_time = np.mean(self.durations)
                self.get_logger().info(f"last 10 seconds, average prediction time: {avg_time:.3f}ms")
                self.durations = []
                self.start_time = time.time()

        except Exception as e:
            self.get_logger().error(f"collision false: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
