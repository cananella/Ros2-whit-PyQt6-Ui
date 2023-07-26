from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
class sub_motor_anlvel(Node):
    def __init__(self):
        super().__init__('motor_angvel_subscriber')
        self.FR_Motor_Angvel = 0.0
        self.FL_Motor_Angvel = 0.0
        self.BR_Motor_Angvel = 0.0
        self.BL_Motor_Angvel = 0.0
        self.subscription = self.create_subscription(
            Quaternion,
            'mainbot_motor_angularVel',
            self.callback,
            10
        )

    def callback(self, msg):
        self.FR_Motor_Angvel = msg.x
        self.FL_Motor_Angvel = msg.y
        self.BR_Motor_Angvel = msg.z
        self.BL_Motor_Angvel = msg.w

class sub_tf_data(Node):
    def __init__(self):
        super().__init__("tf_data_subscriber")
        self.TF_data=TFMessage()
        self.subscription= self.create_subscription(
            TFMessage,
            'mainbot_tf',
            self.callback,
            10
        )
    def callback(self, msg):
        for transform_stamped in msg.transforms:
            self.TF_data=msg

class sub_imu_data(Node):
    def __init__(self):
        super().__init__('imu_data_subscriber')
        self.Imu_data=Imu()
        self.subscription = self.create_subscription(
            Imu,
            'mainbot_imu',
            self.callback,
            10
        )
    def callback(self,msg):
        self.Imu_data=msg

class sub_lidar_data(Node):
    def __init__(self):
        super().__init__('lidar_data_subscriber')
        self.lidar_data=LaserScan()
        self.subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.callback,
            10
        )
    def callback(self, msg):
        self.lidar_data = msg


