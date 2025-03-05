import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motion_msgs.msg import MotionCtrl
from custom_msgs.msg import Panel

class Motionctrl_diablo(Node):
    def __init__(self):
        super().__init__('motionctrl_diablo')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.listener_callback_ppcontrl, 10)
        # 接收/cmd_vel, 模拟手柄的信号，并使用对应回调函数处理信号
        self.panel_sub = self.create_subscription(Panel, '/carrierbot/Panel', self.listener_callback_panel, 10)
        # 接收'/carrierbot/Panel'，真实手柄的信号，并使用对应回调函数处理信号
        self.mctrl_pub = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 10)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.cur_forw_vel_ppcontrl = None
        self.cur_angl_vel_ppcontrl = None
        self.cur_forw_vel_panel = 0
        self.cur_angl_vel_panel = 0

    def listener_callback_ppcontrl(self, msg):
        # get current forward vel
        self.cur_forw_vel_ppcontrl = msg.linear.x        
        # get current angular vel
        self.cur_angl_vel_ppcontrl = msg.angular.z

    def listener_callback_panel(self, msg):
        # get current forward vel
        self.cur_forw_vel_panel = msg.joystick.x / 150        
        # get current angular vel
        self.cur_angl_vel_panel = msg.joystick.y / 150

    def on_timer(self):
        #处理好的信号转换成MotionCtrl接口。
        #write msg for diablo motion ctrl
        mctrl_msg = MotionCtrl()
        mctrl_msg.mode.stand_mode = True
        mctrl_msg.mode.height_ctrl_mode = True
        mctrl_msg.value.up = 1.0
        mctrl_msg.value.pitch = 0.0
        if self.cur_forw_vel_ppcontrl is not None:
            mctrl_msg.value.forward = self.cur_forw_vel_ppcontrl
        else:
            mctrl_msg.value.forward = 0.0
        if self.cur_angl_vel_ppcontrl is not None:
            mctrl_msg.value.left = self.cur_angl_vel_ppcontrl
        else:
            mctrl_msg.value.left = 0.0
        # overwrite controlls with input from panel
        if self.cur_forw_vel_panel != 0 or self.cur_angl_vel_panel != 0:
            mctrl_msg.value.forward = self.cur_forw_vel_panel
            mctrl_msg.value.left = self.cur_angl_vel_panel
        
        self.mctrl_pub.publish(mctrl_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Motionctrl_diablo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()