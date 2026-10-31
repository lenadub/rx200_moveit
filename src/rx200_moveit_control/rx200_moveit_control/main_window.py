import math
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from PyQt5.QtCore import QObject, QTimer, pyqtSignal, Qt
from PyQt5.QtWidgets import (QApplication, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QMainWindow, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget, QSizePolicy)

# GUI - Kian Curran Mahon
# kian.curranmahon.2023@mumail.ie
# 22367856


# movement limits (in meteres)
# REMINDER TO TUNE AND TEST BEFORE USING THIS WITH THE ROBOT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
AXIS_MIN, AXIS_MAX = -0.5, 0.5
Z_MIN, Z_MAX = 0.0, 0.4  # z is base
SHOULDER_Z = 0.10      # base to shoulder
MIN_REACH = 0.10       # not too close
MAX_REACH = 0.45       # or too far
MIN_SEPARATION = 0.03  # smallest movement between pick and place.

# node topics
# near your other constants
USER_INPUTS_TOPIC = 'user_inputs'
ACTION_TOPIC = 'user_inputs'

CONNECTIONS = {
    'user_inputs': USER_INPUTS_TOPIC,
    'action_node': ACTION_TOPIC,
}


class StatusBridge(QObject): # link between ros threads and gui
    status = pyqtSignal(str)
    
class GuiNode(Node):
    def __init__(self, on_status):
        super().__init__('rx200_gui')
        self.on_status = on_status
        self.pub = self.create_publisher(Float32MultiArray, 'USER_INPUTS_TOPIC', 10)
        self.create_subscription(String, 'gui_status', self.status_callback, 10)
        
    def status_callback(self, msg):
        self.on_status(msg.data)
        
    def send_point_data(self,pick,place):
        msg = Float32MultiArray()
        # send points in same order as input manager defines
        msg.data = [float(pick[0]), float(pick[1]), float(pick[2]), float(place[0]), float(place[1]), float(place[2])]
        self.pub.publish(msg)
        
    def subscriber_count(self, topic):
        return self.count_subscribers(topic)
        
class AxisRow(QWidget):  # label and numbered box for one axis, handle updates
    changed = pyqtSignal() # update

    def __init__(self, label_text, lo, hi): # lo, hi are axix limits
        super().__init__()
        label = QLabel(label_text)
        label.setMinimumWidth(30)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(lo, hi)
        self.spin.setDecimals(3)
        self.spin.setSingleStep(0.01)
        self.spin.setSuffix(' m')
        self.spin.setKeyboardTracking(False)  # only fire on Enter / focus loss

        self.spin.setFixedWidth(120)    # pick/place is a fixed width
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(label)
        row.addWidget(self.spin)
        row.addStretch() 

        self.spin.valueChanged.connect(self._on_spin)

    def _on_spin(self, metres): 
        self.changed.emit()

    def value(self):
        return self.spin.value()


class PointGroup(QGroupBox): # x,y,z point management
    changed = pyqtSignal() 

    def __init__(self, title):
        super().__init__(title)
        self.x_row = AxisRow('x', AXIS_MIN, AXIS_MAX)
        self.y_row = AxisRow('y', AXIS_MIN, AXIS_MAX)
        self.z_row = AxisRow('z', Z_MIN, Z_MAX)
        layout = QVBoxLayout(self)
        layout.addWidget(self.x_row)
        layout.addWidget(self.y_row)
        layout.addWidget(self.z_row)
        self.x_row.changed.connect(self.changed)
        self.y_row.changed.connect(self.changed)
        self.z_row.changed.connect(self.changed)

    def point(self):
        return (self.x_row.value(), self.y_row.value(), self.z_row.value())


def check_point(name, x, y, z): # check points are valid before moving, return error
    r = math.hypot(x, y) # horizontal distance from base
    reach = math.sqrt(r ** 2 + (z - SHOULDER_Z) ** 2) # straight line distance from shoulder
    if reach > MAX_REACH:
        return f'{name} is out of reach ({reach:.2f} m from shoulder, max {MAX_REACH:.2f} m)'
    if r < MIN_REACH:
        return f'{name} is too close to the base ({r:.2f} m, min {MIN_REACH:.2f} m)'
    return None

class RobotConnectionIndicator(QLabel): #robot status indicator and text for subscriber counts
    def __init__(self,name):
        super().__init__()
        self.name = name
        self.set_count(0)
 
    def set_count(self, n):
        colour = "green" if n > 0 else 'red'
        text = f'{n} subscriber(s)' if n > 0 else 'No subscribers'
        self.setText(f'<span style="color:{colour};">&#9679;</span> {self.name}: {text}')


class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('RX200 Pick and Place - Group 2')
        self.resize(900, 800)

        central = QWidget() # define main layout
        self.setCentralWidget(central)
        self.main_layout = QVBoxLayout(central)

        self.pick = PointGroup('Pick point')
        self.place = PointGroup('Place point')
        self.main_layout.addWidget(self.pick,0, Qt.AlignLeft)
        self.main_layout.addWidget(self.place, 0, Qt.AlignLeft)

        self.validation_label = QLabel()
        self.main_layout.addWidget(self.validation_label)

        # render status text
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.main_layout.addWidget(self.log, 1)

        self.send_button = QPushButton('Send to robot')
        self.send_button.clicked.connect(self.on_send)
        self.main_layout.addWidget(self.send_button)
        
        # status bar on bottom
        statusBar = self.statusBar()
        statusBar.setSizeGripEnabled(False)
        
        self.validation_label = QLabel()
        self.validation_label.setToolTip('')
 
        statusBar_row = QWidget()
        row = QHBoxLayout(statusBar_row)
        row.setContentsMargins(6, 0, 6, 0)
        row.addWidget(self.validation_label, 1)
        row.setSpacing(16)

        self.indicators = []   # (topic, indicator) pairs
        for label, topic in CONNECTIONS.items():
            indicator = RobotConnectionIndicator(label)
            row.addWidget(indicator)
            self.indicators.append((topic, indicator))

        statusBar.addPermanentWidget(statusBar_row, 1)         

        self.conn_timer = QTimer(self)
        self.conn_timer.timeout.connect(self.update_connection)
        self.conn_timer.start(500)
 
        self.pick.changed.connect(self.update_validation)
        self.place.changed.connect(self.update_validation)
 
        self.update_validation()
        self.update_connection()

        # robot connection status
        self.conn_label = QLabel()
        self.statusBar().addPermanentWidget(self.conn_label)
        self.conn_timer = QTimer(self)
        self.conn_timer.timeout.connect(self.update_connection)
        self.conn_timer.start(500)

        self.pick.changed.connect(self.update_validation)
        self.place.changed.connect(self.update_validation)

        self.update_validation()
        self.update_connection()

    def find_error(self): # check if there are errors on pick/place points
        pick, place = self.pick.point(), self.place.point()
        error = check_point('Pick', *pick) or check_point('Place', *place)
        if error:
            return error
        if math.dist(pick, place) < MIN_SEPARATION:
            return 'Pick and place points are the same'
        return None

    def update_validation(self): # text and colour for found errors
        error = self.find_error()
        if error:
            self.validation_label.setText(error)
            self.validation_label.setStyleSheet('color: #c0392b;')
        else:
            self.validation_label.setText('Ready to send')
            self.validation_label.setStyleSheet('color: #27ae60;')
        self.send_button.setEnabled(error is None)

    def update_connection(self): # display topic subscribers
        for topic, indicator in self.indicators:
          indicator.set_count(self.node.subscriber_count(topic))
          
    def on_gui_status(self, text): # render status msgs
        self.log.appendPlainText(text)
        self.statusBar().showMessage(text, 5000)

    def on_send(self): # action for send button to send point data
        if self.find_error():  # second sanity check, re-check points,
            return
        pick, place = self.pick.point(), self.place.point()
        self.node.send_point_data(pick, place) # name must match the GuiNode method
        msg = (f'Sent pick ({pick[0]:.3f}, {pick[1]:.3f}, {pick[2]:.3f}) -> '
               f'place ({place[0]:.3f}, {place[1]:.3f}, {place[2]:.3f})')
        print(msg)
        self.statusBar().showMessage(msg, 5000)


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    bridge = StatusBridge()
    node = GuiNode(bridge.status.emit)

    # spin ROS in a background thread so it doesn't block the Qt event loop
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    window = MainWindow(node)
    bridge.status.connect(window.on_gui_status)
    window.show()
    code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()