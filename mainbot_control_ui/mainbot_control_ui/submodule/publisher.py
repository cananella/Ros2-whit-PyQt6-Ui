from rclpy.node import Node
from geometry_msgs.msg import Quaternion


class pub_target_vel(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.Vx_vel = 0.0
        self.Vy_vel = 0.0
        self.W_vel = 0.0
        self.pulisher_ = self.create_publisher(Quaternion, 'mainbot_target_velocity', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def update_msg(self, Vx,Vy,W):
        self.Vx_vel = Vx
        self.Vy_vel = Vy
        self.W_vel = W

    def timer_callback(self):
        msg = Quaternion()
        msg.x=self.Vx_vel
        msg.y=self.Vy_vel
        msg.w=self.W_vel
        self.pulisher_.publish(msg)
