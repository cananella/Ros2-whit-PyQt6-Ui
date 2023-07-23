from PyQt6.QtWidgets import *
import sys
from PyQt6.QtCore import Qt, QThread
from PyQt6.QtGui import QPixmap, QBrush, QColor, QPen
import mainbot_control_ui.submodule.control_ui as control_ui
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from tf2_msgs.msg import _tf_message


class sub_Tread1(QThread):
    def __init__(self,parent):
        super().__init__()
        self.parent = parent
    def run(self):
        while True:
            rclpy.spin_once(sub_angvel_node)
            window.set_motor_ang_vel_label()
            window.pub_target_vel_msg()

class pub_Tread1(QThread):
    def __init__(self,parent):
        super().__init__()
        self.parent = parent
    def run(self):
        while True:
            #window.pub_target_vel_msg()
            #rclpy.spin(target_vel_publisher_node)



class UI(QWidget):
    def __init__(self, parent=None):
        super(UI,self).__init__(parent)
        self.ui=control_ui.Ui_Form()
        self.ui.setupUi(self)
        self.ui.retranslateUi(self)
        self.grayPen = QPen(QColor(211,211,211))
        self.redPen = QPen(QColor(200,0,0))
        self.redPen.setWidth(4)
        self.grayPen.setWidth(2)
        self.Vx_vel=0.0
        self.Vy_vel = 0.0
        self.W_vel = 0.0

        self.ui.Vx_doubleSpinBox.valueChanged.connect(self.spin_selected)
        self.ui.Vy_doubleSpinBox.valueChanged.connect(self.spin_selected)
        self.ui.W_doubleSpinBox.valueChanged.connect(self.spin_w_selected)
        self.ui.Vx_horizontalSlider.valueChanged.connect(self.scroll_selected)
        self.ui.Vy_horizontalSlider.valueChanged.connect(self.scroll_selected)
        self.ui.W_horizontalSlider.valueChanged.connect(self.scroll_w_selected)
        self.ui.vel_rst_btn.clicked.connect(self.vel_rst)
        scene1 = QGraphicsScene()
        scene1.addPixmap(QPixmap("images/mecanumbot.png"))
        self.ui.mecanum.setScene(scene1)
        self.ui.mecanum.show()
        self.scene2 = QGraphicsScene()
        self.set_vel_veiw_scene2()
        self.ui.vel_veiw.setScene(self.scene2)
        self.ui.vel_veiw.show()

        self.sub_T1=sub_Tread1(self)
        self.sub_T1.start()

        self.pub_T1=pub_Tread1(self)
        self.pub_T1.start()


    def set_vel_veiw_scene2(self):
        self.scene2.clear()
        self.scene2.addEllipse(2,2,150,150,self.grayPen)
        self.scene2.addEllipse(58, 58, 38, 38, self.grayPen)
        self.scene2.addLine(77,58,120,15,self.grayPen)
        self.scene2.addLine(15, 120, 58, 77, self.grayPen)
        self.scene2.addLine(35, 15, 77, 58, self.grayPen)
        self.scene2.addLine(96, 77, 138, 120, self.grayPen)
        self.scene2.addLine(96, 77, 138, 35, self.grayPen)
        self.scene2.addLine(35, 139, 77, 96, self.grayPen)
        self.scene2.addLine(15, 35, 58, 77, self.grayPen)
        self.scene2.addLine(77, 96, 120, 138, self.grayPen)
        self.scene2.addEllipse(int(75+(self.Vy_vel*1000)*150/230/2),int(75-(self.Vx_vel*1000)*150/230/2),4,4,self.redPen)
    def set_vel_velue_label(self):
        self.ui.Vx_value.setText('{:.2f}'.format(self.Vx_vel))
        self.ui.Vy_value.setText('{:.2f}'.format(self.Vy_vel))
        self.ui.W_value.setText('{:.2f}'.format(self.W_vel))


    def set_motor_ang_vel_label(self):
        self.ui.FL_angVel.setText('{:.2f}'.format(sub_angvel_node.FL_Motor_Angvel))
        self.ui.FR_angVel.setText('{:.2f}'.format(sub_angvel_node.FR_Motor_Angvel))
        self.ui.BL_angVel.setText('{:.2f}'.format(sub_angvel_node.BL_Motor_Angvel))
        self.ui.BR_angVel.setText('{:.2f}'.format(sub_angvel_node.BR_Motor_Angvel))

    def set_vel_velue_spinbox(self):
        self.ui.Vx_doubleSpinBox.setValue(self.Vx_vel)
        self.ui.Vy_doubleSpinBox.setValue(self.Vy_vel)
        self.ui.W_doubleSpinBox.setValue(self.W_vel)

    def set_vel_velue_slider(self):
        self.ui.Vx_horizontalSlider.setValue(int(self.Vx_vel * 100))
        self.ui.Vy_horizontalSlider.setValue(int(self.Vy_vel * 100))
        self.ui.W_horizontalSlider.setValue(int(self.W_vel * 100))

    def pub_target_vel_msg(self):
        target_vel_publisher_node.update_msg(self.Vx_vel, self.Vy_vel, self.W_vel)
        rclpy.spin_once(target_vel_publisher_node)


    def scroll_selected(self):
        self.Vx_vel = self.ui.Vx_horizontalSlider.value()/100.0
        self.Vy_vel = self.ui.Vy_horizontalSlider.value() / 100.0
        self.W_vel = 0.0
        self.set_vel_velue_label()
        self.set_vel_velue_slider()
        self.set_vel_velue_spinbox()
        self.set_vel_veiw_scene2()
        # self.pub_target_vel_msg()


    def scroll_w_selected(self):
        self.Vx_vel = 0.0
        self.Vy_vel = 0.0
        self.W_vel = self.ui.W_horizontalSlider.value()/100.0
        self.set_vel_velue_label()
        self.set_vel_velue_slider()
        self.set_vel_velue_spinbox()
        self.set_vel_veiw_scene2()
        # self.pub_target_vel_msg()


    def spin_w_selected(self):
        self.Vx_vel = 0.0
        self.Vy_vel = 0.0
        self.W_vel = self.ui.W_doubleSpinBox.value()
        self.set_vel_velue_label()
        self.set_vel_velue_slider()
        self.set_vel_velue_spinbox()
        self.set_vel_veiw_scene2()
        # self.pub_target_vel_msg()

    def spin_selected(self):
        self.Vx_vel= self.ui.Vx_doubleSpinBox.value()
        self.Vy_vel = self.ui.Vy_doubleSpinBox.value()
        self.W_vel = 0.0
        self.set_vel_velue_label()
        self.set_vel_velue_slider()
        self.set_vel_velue_spinbox()
        self.set_vel_veiw_scene2()
        # self.pub_target_vel_msg()
    def vel_rst(self):
        self.Vx_vel=0.0
        self.Vy_vel=0.0
        self.W_vel=0.0
        self.set_vel_velue_label()
        self.ui.Vx_horizontalSlider.setValue(0)
        self.ui.Vy_horizontalSlider.setValue(0)
        self.ui.W_horizontalSlider.setValue(0)
        self.ui.Vx_doubleSpinBox.setValue(0.0)
        self.ui.Vy_doubleSpinBox.setValue(0.0)
        self.ui.W_doubleSpinBox.setValue(0.0)
        self.set_vel_veiw_scene2()
        # self.pub_target_vel_msg()

class Vel_Pub(Node):
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


def main(args=None):
    rclpy.init(args=args)
    global target_vel_publisher_node
    global sub_angvel_node
    global window
    target_vel_publisher_node = Vel_Pub()
    sub_angvel_node = sub_motor_anlvel()
    app = QApplication(sys.argv)
    window = UI()
    window.show()

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        pass

    sub_angvel_node.destroy_node()
    target_vel_publisher_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


