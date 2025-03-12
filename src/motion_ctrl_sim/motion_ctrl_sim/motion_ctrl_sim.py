import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.msg import Panel

class Motionctrl_sim(Node):
    def __init__(self):
        super().__init__('motionctrl_sim')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.panel_sub = self.create_subscription(Panel, '/carrierbot/Panel', self.listener_callback_panel, 10)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.cur_forw_vel_panel = 0
        self.cur_angl_vel_panel = 0
        self.sema = True

    def listener_callback_panel(self, msg):
        # get current forward vel
        self.cur_forw_vel_panel = msg.joystick.x / 30        
        # get current angular vel
        self.cur_angl_vel_panel = msg.joystick.y / 30

    def on_timer(self):
        #write msg for diablo motion ctrl
        cmd_msg = Twist()
        # overwrite controlls with input from panel
        if self.cur_forw_vel_panel != 0 or self.cur_angl_vel_panel != 0:
            cmd_msg.linear.x = self.cur_forw_vel_panel
            cmd_msg.linear.y = 0.0
            cmd_msg.linear.z = 0.0

            cmd_msg.angular.x= 0.0
            cmd_msg.angular.y= 0.0
            cmd_msg.angular.z= self.cur_angl_vel_panel
            
            self.sema = True
            
            self.cmd_vel_pub.publish(cmd_msg)

        elif self.cur_forw_vel_panel == 0 and self.cur_angl_vel_panel == 0 and self.sema == True:
            cmd_msg.linear.x = 0.0
            cmd_msg.linear.y = 0.0
            cmd_msg.linear.z = 0.0

            cmd_msg.angular.x= 0.0
            cmd_msg.angular.y= 0.0
            cmd_msg.angular.z= 0.0

            self.sema = False
            
            self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Motionctrl_sim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()