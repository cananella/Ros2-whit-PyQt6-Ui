from PyQt6.QtWidgets import *
import sys
import math
from PyQt6.QtCore import *
from PyQt6.QtGui import QPixmap, QColor, QPen
import mainbot_control_ui.submodule.control_ui as control_ui
import mainbot_control_ui.submodule.publisher as pub
import mainbot_control_ui.submodule.subscriber as sub
import rclpy
from sensor_msgs.msg import Imu, LaserScan

MOVING_LOG_SIZE = 100
MAP_X_SIZE=500
MAP_Y_SIZE=500


class ros_Tread1(QThread):
    def __init__(self):
        super().__init__()

    def run(self):
        while rclpy.ok():
            try:
                rclpy.spin_once(sub_angvel_node, timeout_sec=0.01)
                rclpy.spin_once(sub_tf_node, timeout_sec=0.01)
                rclpy.spin_once(sub_imu_node, timeout_sec=0.01)
                rclpy.spin_once(sub_lidar_node, timeout_sec=0.01)
                window.set_motor_ang_vel_label()
                window.update_imu_data()
                window.update_laser_scan_data()
                window.pub_target_vel_msg()

            except KeyboardInterrupt:
                pass

        sub_imu_node.destroy_node()
        sub_tf_node.destroy_node()
        sub_angvel_node.destroy_node()
        sub_lidar_node.destroy_node()
        pub_target_vel_nod.destroy_node()
        rclpy.shutdown()


class update_map_Thread(QThread):
    show_mainbot_odom = pyqtSignal()
    show_laser_scan=pyqtSignal()

    def __init__(self):
        super().__init__()

    def run(self):
        while True:
            if not (window.ui.tab_1.isHidden()):
                window.scene3.clear()
                window.update_mainbot_orientation_log()
                if window.laser_scan_view_flag:
                    self.show_laser_scan.emit()
                self.show_mainbot_odom.emit()

            self.msleep(int(1000 / 20))


class UI(QWidget):
    def __init__(self, parent=None):
        super(UI, self).__init__(parent)
        self.ui = control_ui.Ui_Form()
        self.ui.setupUi(self)
        self.ui.retranslateUi(self)
        self.grayPen = QPen(QColor(211, 211, 211))
        self.grayPen.setWidth(2)
        self.redPen = QPen(QColor(200, 0, 0))
        self.redPen.setWidth(4)
        self.mainbot_imu = Imu()
        self.laser_scan = LaserScan()
        self.laser_scan_view_flag=False
        self.ui.laser_scan_view_btn.clicked.connect(self.set_laser_scan_view_onoff)

        self.Vx_vel = 0.0
        self.Vy_vel = 0.0
        self.W_vel = 0.0
        self.mainbot_x = 0
        self.mainbot_y = 0
        self.mainbot_w = 0
        self.mainbot_pos_x=0
        self.mainbot_pos_y=0

        self.move_update_flag = False
        self.mainbot_moving_log = [[0 for j in range(4)] for i in range(MOVING_LOG_SIZE)]
        self.save_moving_data_size = 0

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
        self.scene2.addEllipse(2, 2, 150, 150, self.grayPen)
        self.scene2.addEllipse(58, 58, 38, 38, self.grayPen)
        self.scene2.addLine(77, 58, 120, 15, self.grayPen)
        self.scene2.addLine(15, 120, 58, 77, self.grayPen)
        self.scene2.addLine(35, 15, 77, 58, self.grayPen)
        self.scene2.addLine(96, 77, 138, 120, self.grayPen)
        self.scene2.addLine(96, 77, 138, 35, self.grayPen)
        self.scene2.addLine(35, 139, 77, 96, self.grayPen)
        self.scene2.addLine(15, 35, 58, 77, self.grayPen)
        self.scene2.addLine(77, 96, 120, 138, self.grayPen)
        self.clc1 = QGraphicsEllipseItem(75, 75, 4, 4)
        self.clc1.setPen(self.redPen)
        self.scene2.addItem(self.clc1)
        self.set_vel_veiw_scene2()
        self.ui.vel_veiw.setScene(self.scene2)
        self.ui.vel_veiw.show()

        self.scene3 = QGraphicsScene()
        self.map_x_zeropoint = self.ui.map.geometry().width() / 2
        self.map_y_zeropoint = self.ui.map.geometry().height() / 2
        self.ui.map.setScene(self.scene3)
        self.ui.map.show()

        self.T1 = ros_Tread1()
        self.T1.start()

        self.T2 = update_map_Thread()
        self.T2.show_mainbot_odom.connect(self.show_mainbot_odom_on_map)
        self.T2.show_laser_scan.connect(self.show_laser_scan_on_map)
        self.T2.start()


    def set_laser_scan_view_onoff(self):
        if self.laser_scan_view_flag:
            self.laser_scan_view_flag=False
        else :
            self.laser_scan_view_flag=True
    def set_vel_veiw_scene2(self):
        x = (self.Vy_vel * 1000) * 150 / 230 / 2
        y = (self.Vx_vel * 1000) * 150 / 230 / 2
        meg = math.sqrt(x ** 2 + y ** 2)
        if meg > 77:
            x = x * 77 / meg
            y = y * 77 / meg
        self.clc1.setRect(int(75 + x), int(75 - y), 4, 4)

    def set_vel_velue_label(self):
        self.ui.Vx_value.setText('{:.2f}'.format(self.Vx_vel))
        self.ui.Vy_value.setText('{:.2f}'.format(self.Vy_vel))
        self.ui.W_value.setText('{:.2f}'.format(self.W_vel))

    def set_motor_ang_vel_label(self):
        self.ui.FL_angVel.setText('{:.2f}'.format(sub_angvel_node.FL_Motor_Angvel))
        self.ui.FR_angVel.setText('{:.2f}'.format(sub_angvel_node.FR_Motor_Angvel))
        self.ui.BL_angVel.setText('{:.2f}'.format(sub_angvel_node.BL_Motor_Angvel))
        self.ui.BR_angVel.setText('{:.2f}'.format(sub_angvel_node.BR_Motor_Angvel))

    def show_mainbot_odom_on_map(self):
        k = 0
        for i in self.mainbot_moving_log:
            pen = QPen(QColor(200, MOVING_LOG_SIZE - k, MOVING_LOG_SIZE - k))
            pen.setWidth(2)
            self.mainbot_pos_x = self.map_x_zeropoint + i[1]
            self.mainbot_pos_y = self.map_y_zeropoint + i[2]
            x= self.mainbot_pos_x -2
            y= self.mainbot_pos_y -2
            self.scene3.addEllipse(x, y, 5, 5, pen)
            pen.setWidth(1)
            self.scene3.addLine(x + 2, y + 2, x + 2 + math.sin(i[3]) * 8, y + 2 + math.cos(i[3]) * 8, pen)
            k += 1

    def show_laser_scan_on_map(self):
        angle_min=self.laser_scan.angle_min
        angle_increment=self.laser_scan.angle_increment
        k=0
        for i in self.laser_scan.ranges:
            x=self.mainbot_pos_x+math.cos(angle_min+angle_increment*k)*i
            y=self.mainbot_pos_y+math.sin(angle_min+angle_increment*k)*i
            self.scene3.addRect(x-1,y-1,3,3,QPen(QColor(255,255,255)))
            self.scene3.addLine(self.mainbot_pos_x,self.mainbot_pos_y,x,y,QPen(QColor(200,200,200)))
            k+=1


    def update_laser_scan_data(self):
        self.laser_scan = sub_lidar_node.lidar_data

    def update_imu_data(self):
        tempx = self.mainbot_x
        tempy = self.mainbot_y
        tempw = self.mainbot_w
        self.mainbot_imu = sub_imu_node.Imu_data
        self.mainbot_x = int(sub_imu_node.Imu_data.orientation.x * 100) / 5  # cm/ 5pix
        self.mainbot_y = int(sub_imu_node.Imu_data.orientation.y * 100) / 5
        self.mainbot_w = int(sub_imu_node.Imu_data.orientation.w * 100) / 5
        if not (tempx == self.mainbot_x and tempy == self.mainbot_y and tempw == self.mainbot_w):
            self.move_update_flag = True

    def update_mainbot_orientation_log(self):
        time = self.mainbot_imu.header.stamp.sec + self.mainbot_imu.header.stamp.nanosec / 10 ** 9
        x = self.mainbot_imu.orientation.x
        y = self.mainbot_imu.orientation.y
        w = self.mainbot_imu.orientation.w
        if self.move_update_flag:
            if self.save_moving_data_size < MOVING_LOG_SIZE:
                self.mainbot_moving_log[self.save_moving_data_size][0] = time
                self.mainbot_moving_log[self.save_moving_data_size][1] = x
                self.mainbot_moving_log[self.save_moving_data_size][2] = y
                self.mainbot_moving_log[self.save_moving_data_size][3] = w
                self.save_moving_data_size += 1
            else:
                self.mainbot_moving_log.pop(0)
                data = [time, x, y, w]
                self.mainbot_moving_log.append(data)
            self.move_update_flag = False

    def set_vel_velue_spinbox(self):
        self.ui.Vx_doubleSpinBox.setValue(self.Vx_vel)
        self.ui.Vy_doubleSpinBox.setValue(self.Vy_vel)
        self.ui.W_doubleSpinBox.setValue(self.W_vel)

    def set_vel_velue_slider(self):
        self.ui.Vx_horizontalSlider.setValue(int(self.Vx_vel * 100))
        self.ui.Vy_horizontalSlider.setValue(int(self.Vy_vel * 100))
        self.ui.W_horizontalSlider.setValue(int(self.W_vel * 100))

    def pub_target_vel_msg(self):
        pub_target_vel_nod.update_msg(self.Vx_vel, self.Vy_vel, self.W_vel)
        rclpy.spin_once(pub_target_vel_nod)

    def scroll_selected(self):
        self.Vx_vel = self.ui.Vx_horizontalSlider.value() / 100.0
        self.Vy_vel = self.ui.Vy_horizontalSlider.value() / 100.0
        self.W_vel = 0.0
        self.set_vel_velue_label()
        self.set_vel_velue_slider()
        self.set_vel_velue_spinbox()
        self.set_vel_veiw_scene2()

    def scroll_w_selected(self):
        self.Vx_vel = 0.0
        self.Vy_vel = 0.0
        self.W_vel = self.ui.W_horizontalSlider.value() / 100.0
        self.set_vel_velue_label()
        self.set_vel_velue_slider()
        self.set_vel_velue_spinbox()
        self.set_vel_veiw_scene2()

    def spin_w_selected(self):
        self.Vx_vel = 0.0
        self.Vy_vel = 0.0
        self.W_vel = self.ui.W_doubleSpinBox.value()
        self.set_vel_velue_label()
        self.set_vel_velue_slider()
        self.set_vel_velue_spinbox()
        self.set_vel_veiw_scene2()

    def spin_selected(self):
        self.Vx_vel = self.ui.Vx_doubleSpinBox.value()
        self.Vy_vel = self.ui.Vy_doubleSpinBox.value()
        self.W_vel = 0.0
        self.set_vel_velue_label()
        self.set_vel_velue_slider()
        self.set_vel_velue_spinbox()
        self.set_vel_veiw_scene2()

    def vel_rst(self):
        self.Vx_vel = 0.0
        self.Vy_vel = 0.0
        self.W_vel = 0.0
        self.set_vel_velue_label()
        self.ui.Vx_horizontalSlider.setValue(0)
        self.ui.Vy_horizontalSlider.setValue(0)
        self.ui.W_horizontalSlider.setValue(0)
        self.ui.Vx_doubleSpinBox.setValue(0.0)
        self.ui.Vy_doubleSpinBox.setValue(0.0)
        self.ui.W_doubleSpinBox.setValue(0.0)
        self.set_vel_veiw_scene2()


def main(args=None):
    rclpy.init(args=args)
    global pub_target_vel_nod
    global sub_angvel_node
    global window
    global sub_tf_node
    global sub_imu_node
    global sub_lidar_node
    pub_target_vel_nod = pub.pub_target_vel()
    sub_angvel_node = sub.sub_motor_anlvel()
    sub_imu_node = sub.sub_imu_data()
    sub_tf_node = sub.sub_tf_data()
    sub_lidar_node = sub.sub_lidar_data()
    app = QApplication(sys.argv)
    window = UI()
    window.show()

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
