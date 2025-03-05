import joblib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32  # 假设数据是数组类型, Int32 用于结果发布
import numpy as np

# 加载模型
MODEL_PATH = '/home/pc/diablo_ws/src/model_node/checkpoints/LinSVM_draft.pkl'
#SCALER_PATH = "/home/pc/diablo_ws/src/model_node/checkpoints/scaler.joblib"

try:
    model = joblib.load(MODEL_PATH)
    #scaler = joblib.load(SCALER_PATH)
    print(" SVM 模型和 MinMaxScaler 成功加载！")
except Exception as e:
    print(f"加载模型或 `scaler` 失败: {e}")
    model = None
    #scaler = None  # 避免程序崩溃

class CollisionDetectionNode(Node):
    def __init__(self):
        super().__init__('collision_detection_node')
        # 创建订阅器，订阅 /processed_data 话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_data',
            self.listener_callback,
            10
        )
        # 创建发布器，当发生碰撞后发布/collision_prediction 话题包含碰撞结果
        self.publisher = self.create_publisher(
            Int32, 
            '/collision_prediction', 
            10
        )
        
        self.subscription  # 防止变量被垃圾回收
        self.get_logger().info("Collision Detection Node 已启动，等待数据...")

    def listener_callback(self, msg):
        #if model is None or scaler is None:
            #self.get_logger().error ("模型未加载，无法进行预测！")

        # **转换 ROS2 消息数据为 NumPy 数组**
        sensor_data = np.array(msg.data)  # ⚠️ 这里不要 reshape ！

        # **确保数据是 2D (1, n)，不多加一个维度**
        if sensor_data.ndim == 1:  # 如果数据是一维 (n,)
            sensor_data = sensor_data.reshape(1, -1)  # 变成 (1, n)

        # **打印转换后数据维度**
        #self.get_logger().info(f"转换后数据维度: {sensor_data.shape}")

        # 确保数据维度和模型匹配
        expected_features = model.n_features_in_  
        if sensor_data.shape[1] != expected_features:
            self.get_logger().error(f"⚠️ 数据维度错误！期望 {expected_features} 维，实际 {sensor_data.shape[1]} 维")
            return

        #try:
            #sensor_data_scaled = scaler.transform(sensor_data)
        #except Exception as e:
            #self.get_logger().error(f"归一化失败: {e}")
            #return
        # 确保数据维度匹配
        
        try:


            # 进行碰撞预测
            prediction = model.predict(sensor_data)[0]  # 预测是否发生碰撞

            # **检查 prediction 是否是 1、2 或 3**
            if prediction in [1, 2, 3]:
                prediction_msg = Int32()
                prediction_msg.data = int(prediction)  # 强制转换为标准整数类型
                self.get_logger().warn(f'碰撞检测: 发生碰撞！(类别: {prediction_msg.data})')
                self.publisher.publish(prediction_msg)
            #else:
                #self.get_logger().info(f'碰撞检测: 无碰撞 (类别: {prediction})')

        except Exception as e:
            self.get_logger().error(f"预测错误: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
