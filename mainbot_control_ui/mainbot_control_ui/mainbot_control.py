import sys
import math
import numpy as np
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import QPixmap, QColor, QPen
import mainbot_control_ui.submodule.control_ui as control_ui
import mainbot_control_ui.submodule.publisher as pub
import mainbot_control_ui.submodule.subscriber as sub
import rclpy
from sensor_msgs.msg import Imu, LaserScan
import pyqtgraph as pg



MOVING_LOG_SIZE = 100
MAP_X_SIZE=500
MAP_Y_SIZE=500

class ros_Tread(QThread):
    def __init__(self,ui):
        super().__init__()
        self.ui=ui


    def run(self):
        while rclpy.ok() and self.ui.ros_update_flag :
            try:
                rclpy.spin_once(self.ui.sub_angvel_node, timeout_sec=0.01)
                rclpy.spin_once(self.ui.sub_tf_node, timeout_sec=0.01)
                rclpy.spin_once(self.ui.sub_imu_node, timeout_sec=0.01)
                rclpy.spin_once(self.ui.sub_lidar_node, timeout_sec=0.01)
                self.ui.set_motor_ang_vel_label()
                self.ui.update_imu_data()
                self.ui.update_laser_scan_data()
                self.ui.pub_target_vel_msg()

            except KeyboardInterrupt:
                pass

    def stop(self):
        self.ui.ros_update_flag = False
        self.quit()
        self.wait(100)


class UI(QDialog,QWidget):
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

        #ros2_node init
        print("mainbot ros node start")
        rclpy.init()
        self.pub_target_vel_nod = pub.pub_target_vel()
        self.sub_angvel_node = sub.sub_motor_anlvel()
        self.sub_imu_node = sub.sub_imu_data()
        self.sub_tf_node = sub.sub_tf_data()
        self.sub_lidar_node = sub.sub_lidar_data()

        self.Vx_vel = 0.0
        self.Vy_vel = 0.0
        self.W_vel = 0.0
        self.mainbot_x = 0
        self.mainbot_y = 0
        self.mainbot_w = 0
        self.mainbot_pos_x=0
        self.mainbot_pos_y=0
        self.ros_update_flag = True
        self.move_update_flag = False
        self.mainbot_moving_log = [[0 for j in range(4)] for i in range(MOVING_LOG_SIZE)]
        self.saved_moving_data_size = 0
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

        self.grap_init()


        self.T1 = ros_Tread(ui=self)
        self.T1.start()

        """
        # anoter way that ros_Tread didnt work (QThead => QTimer)
        
        self.ros_update_timer= QTimer()
        self.ros_update_timer.timeout.connect(self.update_ros_nodes)
        self.ros_update_timer.start(10)
        """

        self.map_update_timer = QTimer()
        self.map_update_timer.timeout.connect(self.update_graphics_window)
        self.map_update_timer.start(int(1000 / 20))  # 20FPS

        self.accel_update_timer = QTimer()
        self.accel_update_timer.timeout.connect(self.update_accel_plot_data)
        self.accel_update_timer.start(int(1000/5))  # 5 Hz


    def update_accel_plot_data(self):
        self.time_x = self.time_x[1:]
        self.time_x.append(self.time_x[-1] + 1)  # Add a new value 1 higher than the last.

        self.accel_ang_y[0]=self.accel_ang_y[0][1:]
        self.accel_ang_y[0].append(self.mainbot_imu.angular_velocity.x)
        self.accel_ang_y[1] = self.accel_ang_y[1][1:]
        self.accel_ang_y[1].append(self.mainbot_imu.angular_velocity.y)
        self.accel_ang_y[2] = self.accel_ang_y[2][1:]
        self.accel_ang_y[2].append(self.mainbot_imu.angular_velocity.z)
        self.ang_accel_line_x.setData(self.time_x, self.accel_ang_y[0])
        self.ang_accel_line_y.setData(self.time_x, self.accel_ang_y[1])
        self.ang_accel_line_z.setData(self.time_x, self.accel_ang_y[2])


        self.accel_linear_y[0] = self.accel_linear_y[0][1:]
        self.accel_linear_y[0].append(self.mainbot_imu.linear_acceleration.x)
        self.accel_linear_y[1] = self.accel_linear_y[1][1:]
        self.accel_linear_y[1].append(self.mainbot_imu.linear_acceleration.y)
        self.accel_linear_y[2] = self.accel_linear_y[2][1:]
        self.accel_linear_y[2].append(self.mainbot_imu.linear_acceleration.z)
        self.linear_accel_line_x.setData(self.time_x, self.accel_linear_y[0])
        self.linear_accel_line_y.setData(self.time_x, self.accel_linear_y[1])
        self.linear_accel_line_z.setData(self.time_x, self.accel_linear_y[2])

        self.odme_y[0] = self.odme_y[0][1:]
        self.odme_y[0].append(self.mainbot_imu.orientation.x)
        self.odme_y[1] = self.odme_y[1][1:]
        self.odme_y[1].append(self.mainbot_imu.orientation.y)
        self.odme_y[2] = self.odme_y[2][1:]
        self.odme_y[2].append(self.mainbot_imu.orientation.w)
        self.odme_line_x.setData(self.time_x, self.odme_y[0])
        self.odme_line_y.setData(self.time_x, self.odme_y[1])
        self.odme_line_w.setData(self.time_x, self.odme_y[2])



    def window_close(self):
        if self.map_update_timer.isActive():
            self.map_update_timer.stop()
        if self.accel_update_timer.isActive():
            self.map_update_timer.stop()
        """
        #if ros_node turn on wiht QTimer
        if self.ros_update_timer.isActive():
            self.ros_update_timer.stop()
        """
        self.T1.stop()
        self.sub_imu_node.destroy_node()
        self.sub_tf_node.destroy_node()
        self.sub_angvel_node.destroy_node()
        self.sub_lidar_node.destroy_node()
        self.pub_target_vel_nod.destroy_node()
        rclpy.shutdown()
        print("mainbot ros node shutdown")


    def update_ros_nodes(self):
        if rclpy.ok() and self.ros_update_flag:
            rclpy.spin_once(self.sub_angvel_node, timeout_sec=0.01)
            rclpy.spin_once(self.sub_tf_node, timeout_sec=0.01)
            rclpy.spin_once(self.sub_imu_node, timeout_sec=0.01)
            rclpy.spin_once(self.sub_lidar_node, timeout_sec=0.01)
            self.set_motor_ang_vel_label()
            self.update_imu_data()
            self.update_laser_scan_data()
            self.pub_target_vel_msg()
        elif not rclpy.ok() or not self.ros_update_flag:
            self.window_close()


    def update_graphics_window(self):
        if not (self.ui.tab_1.isHidden()):
            self.scene3.clear()
            if self.ui.laser_scan_view_state.isChecked():
                self.show_laser_scan_on_map()
            self.show_mainbot_odom_on_map()

        elif not(self.ui.tab_2.isHidden()):
            pass

        elif not(self.ui.tab_3.isHidden()):
            pass



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
        self.ui.FL_angVel.setText('{:.2f}'.format(self.sub_angvel_node.FL_Motor_Angvel))
        self.ui.FR_angVel.setText('{:.2f}'.format(self.sub_angvel_node.FR_Motor_Angvel))
        self.ui.BL_angVel.setText('{:.2f}'.format(self.sub_angvel_node.BL_Motor_Angvel))
        self.ui.BR_angVel.setText('{:.2f}'.format(self.sub_angvel_node.BR_Motor_Angvel))

    def show_mainbot_odom_on_map(self):
        line_len=10
        resol = 10  # cm/10pix
        for i in range(self.saved_moving_data_size):
            pen = QPen(QColor(200, MOVING_LOG_SIZE - i, MOVING_LOG_SIZE - i))
            pen.setWidth(3)
            self.mainbot_pos_x = self.map_x_zeropoint + self.mainbot_moving_log[i][1] * resol
            self.mainbot_pos_y = self.map_y_zeropoint + self.mainbot_moving_log[i][2] * resol
            x= self.mainbot_pos_x -2
            y= self.mainbot_pos_y -2
            self.scene3.addEllipse(x, y, 5, 5, pen)
            pen.setWidth(1)
            self.scene3.addLine(x + 2, y + 2, x + 2 + math.sin(self.mainbot_moving_log[i][3]) * line_len, y + 2 + math.cos(self.mainbot_moving_log[i][3]) * line_len, pen)

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
        self.laser_scan = self.sub_lidar_node.lidar_data

    def update_imu_data(self):
        tempx = self.mainbot_x
        tempy = self.mainbot_y
        tempw = self.mainbot_w
        resol=100  #m->cm
        self.mainbot_imu = self.sub_imu_node.Imu_data
        time = pow(self.mainbot_imu.header.stamp.sec + self.mainbot_imu.header.stamp.nanosec / 10 ** 9, 4) #sec
        self.mainbot_x = int(self.sub_imu_node.Imu_data.orientation.x * resol) # cm
        self.mainbot_y = int(self.sub_imu_node.Imu_data.orientation.y * resol)
        self.mainbot_w = int(self.sub_imu_node.Imu_data.orientation.w )

        #moving log update
        if not (tempx == self.mainbot_x and tempy == self.mainbot_y and tempw == self.mainbot_w):
            if self.saved_moving_data_size < MOVING_LOG_SIZE:
                self.mainbot_moving_log[self.saved_moving_data_size][0] = time
                self.mainbot_moving_log[self.saved_moving_data_size][1] = self.mainbot_x
                self.mainbot_moving_log[self.saved_moving_data_size][2] = self.mainbot_y
                self.mainbot_moving_log[self.saved_moving_data_size][3] = self.mainbot_w
                self.saved_moving_data_size += 1
            else:
                self.mainbot_moving_log.pop(0)
                data = [time, self.mainbot_x, self.mainbot_y, self.mainbot_w]
                self.mainbot_moving_log.append(data)


    def set_vel_velue_spinbox(self):
        self.ui.Vx_doubleSpinBox.setValue(self.Vx_vel)
        self.ui.Vy_doubleSpinBox.setValue(self.Vy_vel)
        self.ui.W_doubleSpinBox.setValue(self.W_vel)

    def set_vel_velue_slider(self):
        self.ui.Vx_horizontalSlider.setValue(int(self.Vx_vel * 100))
        self.ui.Vy_horizontalSlider.setValue(int(self.Vy_vel * 100))
        self.ui.W_horizontalSlider.setValue(int(self.W_vel * 100))

    def pub_target_vel_msg(self):
        self.pub_target_vel_nod.update_msg(self.Vx_vel, self.Vy_vel, self.W_vel)
        rclpy.spin_once(self.pub_target_vel_nod)

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

    def grap_init(self):
        imu_plot_x_max = 100
        imu_plot_y_max = 100

        self.time_x = list(range(imu_plot_x_max))

        self.accel_ang_y = [[0 for j in range(imu_plot_y_max)] for i in range(3)]
        self.grap_accel_ang = pg.PlotWidget()
        self.grap_accel_ang.setBackground('w')
        self.grap_accel_ang.setTitle("Imu angular velocity ")
        self.ui.Imu_layout.addWidget(self.grap_accel_ang,stretch=1)
        self.grap_accel_ang.addLegend(offset=(1,1),brush=pg.mkBrush(color=(255,255,255)))
        self.ang_accel_line_x = self.grap_accel_ang.plot(self.time_x, self.accel_ang_y[0][:],
                                                         name="asngular_velocity_x",
                                                         pen=pg.mkPen(color=(255, 0, 0)))
        self.ang_accel_line_y = self.grap_accel_ang.plot(self.time_x, self.accel_ang_y[1][:], name="angular_velocity_y",
                                                         pen=pg.mkPen(color=(0, 255, 0)))
        self.ang_accel_line_z = self.grap_accel_ang.plot(self.time_x, self.accel_ang_y[2][:], name="angular_velocity_z",
                                                         pen=pg.mkPen(color=(0, 0, 255)))



        self.accel_linear_y = [[0 for j in range(imu_plot_y_max)] for i in range(3)]
        self.grap_accel_linear = pg.PlotWidget()
        self.grap_accel_linear.setBackground('w')
        self.grap_accel_linear.setTitle("Imu linear velocity ")
        self.ui.Imu_layout.addWidget(self.grap_accel_linear,stretch=1)
        self.grap_accel_linear.addLegend(offset=(1,1),brush=pg.mkBrush(color=(255,255,255)))
        self.linear_accel_line_x = self.grap_accel_linear.plot(self.time_x, self.accel_linear_y[0][:],
                                                               name="linear_velocity_x",
                                                               pen=pg.mkPen(color=(255, 0, 0)))
        self.linear_accel_line_y = self.grap_accel_linear.plot(self.time_x, self.accel_linear_y[1][:],
                                                               name="linear_velocity_y",
                                                               pen=pg.mkPen(color=(0, 255, 0)))
        self.linear_accel_line_z = self.grap_accel_linear.plot(self.time_x, self.accel_linear_y[2][:],
                                                               name="linear_velocity_z",
                                                               pen=pg.mkPen(color=(0, 0, 255)))


        self.odme_y = [[0 for j in range(imu_plot_y_max)] for i in range(3)]
        self.grap_odme = pg.PlotWidget()
        self.grap_odme.setBackground('w')
        self.grap_odme.setTitle("Imu orientation ")
        self.ui.Imu_layout.addWidget(self.grap_odme,stretch=1)
        self.grap_odme.addLegend(offset=(1,1),brush=pg.mkBrush(color=(255,255,255)))
        self.odme_line_x = self.grap_odme.plot(self.time_x, self.odme_y[0][:],
                                               name="odme_x",
                                               pen=pg.mkPen(color=(255, 0, 0)))
        self.odme_line_y = self.grap_odme.plot(self.time_x, self.odme_y[1][:],
                                               name="odme_y",
                                               pen=pg.mkPen(color=(0, 255, 0)))
        self.odme_line_w = self.grap_odme.plot(self.time_x, self.odme_y[2][:],
                                               name="odme_w",
                                               pen=pg.mkPen(color=(0, 0, 255)))





def main(args=None):
    app = QApplication(sys.argv)
    mainbot_window = UI()
    mainbot_window.show()


    try:
        sys.exit(app.exec())

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
