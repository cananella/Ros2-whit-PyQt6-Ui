import mainbot_control_ui.mainbot_control as mainbot_control
import mainbot_control_ui.submodule.main_window as main_window
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
import sys

class main_w(QMainWindow,QWidget):
    def __init__(self):
        super(main_w,self).__init__()
        self.window=main_window.Ui_MainWindow()
        self.window.setupUi(self)
        self.window.retranslateUi(self)
        self.window.mainbot_contorl_open_btn.clicked.connect(self.mainbot_cotrol_wiget_open)

    def mainbot_cotrol_wiget_open(self):
        self.close()
        self.mainbot_cotrol_window = mainbot_control.UI()
        self.mainbot_cotrol_window.show()
        self.mainbot_cotrol_window.exec()
        self.mainbot_cotrol_window.window_close()

        self.show()




def main(args=None):

    app = QApplication(sys.argv)
    mainwindow = main_w()
    mainwindow.show()

    sys.exit(app.exec())



if __name__ =="__main__":
    main()