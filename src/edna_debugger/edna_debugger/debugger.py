
import argparse
import random
import signal
import sys
import threading

import rclpy

from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QFormLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget

from edna_debugger.joint_state_publisher import JointStatePublisher
from edna_debugger.flow_layout import FlowLayout

RANGE = 10000
LINE_EDIT_WIDTH = 45
SLIDER_WIDTH = 200
INIT_NUM_SLIDERS = 7  # Initial number of sliders to show in window

# Defined by style - currently using the default style
DEFAULT_WINDOW_MARGIN = 11
DEFAULT_CHILD_MARGIN = 9
DEFAULT_BTN_HEIGHT = 25
DEFAULT_SLIDER_HEIGHT = 64  # Is the combination of default heights in Slider

# Calculate default minimums for window sizing
MIN_WIDTH = SLIDER_WIDTH + DEFAULT_CHILD_MARGIN * 4 + DEFAULT_WINDOW_MARGIN * 2
MIN_HEIGHT = DEFAULT_BTN_HEIGHT * 2 + DEFAULT_WINDOW_MARGIN * 2 + DEFAULT_CHILD_MARGIN * 2

PNEUMATICS_JOINTS = [
    'arm_roller_bar_joint', 
    'top_slider_joint',
    'top_gripper_left_arm_joint',
    'bottom_intake_joint'
]

class Button(QWidget):
    def __init__(self, name):
        super().__init__()

        self.joint_layout = QVBoxLayout()
        self.row_layout = QHBoxLayout()

        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout.addWidget(self.label)


        self.joint_layout.addLayout(self.row_layout)

        self.button = QPushButton('toggle', self)
        self.button.setCheckable(True)
        self.joint_layout.addWidget(self.button)

        self.setLayout(self.joint_layout)
    

    def remove(self):
        self.joint_layout.remove_widget(self.button)
        self.button.setParent(None)

        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)

        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)

        self.row_layout.setParent(None)

class Slider(QWidget):
    def __init__(self, name):
        super().__init__()

        self.joint_layout = QVBoxLayout()

        # top row
        self.row_layout_top = QHBoxLayout()
        
        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout_top.addWidget(self.label)

        self.joint_layout.addLayout(self.row_layout_top)

        # subscribing row
        self.row_layout_sub = QHBoxLayout()
        
        self.display_sub = QLineEdit("0.00")
        self.display_sub.setAlignment(Qt.AlignRight)
        self.display_sub.setFont(font)
        self.display_sub.setReadOnly(True)
        self.display_sub.setFixedWidth(LINE_EDIT_WIDTH)
        self.row_layout_sub.addWidget(self.display_sub)

        self.slider_sub = QSlider(Qt.Horizontal)
        self.slider_sub.setFont(font)
        self.slider_sub.setRange(0, RANGE)
        self.slider_sub.setValue(int(RANGE / 2))
        self.slider_sub.setFixedWidth(SLIDER_WIDTH)
        self.row_layout_sub.addWidget(self.slider_sub)

        self.joint_layout.addLayout(self.row_layout_sub)

        # publishing row
        self.row_layout_pub = QHBoxLayout()

        self.display_pub = QLineEdit("0.00")
        self.display_pub.setAlignment(Qt.AlignRight)
        self.display_pub.setFont(font)
        self.display_pub.setReadOnly(False)
        self.display_pub.setFixedWidth(LINE_EDIT_WIDTH)
        self.row_layout_pub.addWidget(self.display_pub)

        self.slider_pub = QSlider(Qt.Horizontal)
        self.slider_pub.setFont(font)
        self.slider_pub.setRange(0, RANGE)
        self.slider_pub.setValue(int(RANGE / 2))
        self.slider_pub.setFixedWidth(SLIDER_WIDTH)
        self.row_layout_pub.addWidget(self.slider_pub)

        self.joint_layout.addLayout(self.row_layout_pub)

        self.setLayout(self.joint_layout)

    def remove(self):
        self.joint_layout.removeWidget(self.slider)
        self.slider.setParent(None)

        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)

        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)

        self.row_layout.setParent(None)


class JointStatePublisherGui(QMainWindow):
    sliderUpdateTrigger = Signal()
    initialize = Signal()

    def __init__(self, title, jsp : JointStatePublisher):
        super(JointStatePublisherGui, self).__init__()

        self.joint_map = {}

        self.setWindowTitle(title)

        # Button for randomizing the sliders
        self.rand_button = QPushButton('Randomize', self)
        self.rand_button.clicked.connect(self.randomizeEvent)

        # Button for centering the sliders
        self.ctr_button = QPushButton('Center', self)
        self.ctr_button.clicked.connect(self.centerEvent)

        # Button for resetting the joint butttons
        self.reset_button = QPushButton('Reset Pistons', self)
        self.reset_button.clicked.connect(self.resetButtons)

        # Scroll area widget contents - layout
        self.scroll_layout = FlowLayout()

        # Scroll area widget contents
        self.scroll_widget = QWidget()
        self.scroll_widget.setLayout(self.scroll_layout)

        # Scroll area for sliders
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.scroll_widget)

        # Main layout
        self.main_layout = QVBoxLayout()

        # Add buttons and scroll area to main layout
        self.main_layout.addWidget(self.rand_button)
        self.main_layout.addWidget(self.ctr_button)
        self.main_layout.addWidget(self.reset_button)
        self.main_layout.addWidget(self.scroll_area)

        # central widget
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)

        self.jsp = jsp
        self.jsp.set_source_update_cb(self.sliderUpdateCb)
        self.jsp.set_robot_description_update_cb(self.initializeCb)

        self.running = True
        self.sliders = {}
        self.buttons = {}

        # Setup signal for initializing the window
        self.initialize.connect(self.initializeSliders)
        # Set up a signal for updating the sliders based on external joint info
        self.sliderUpdateTrigger.connect(self.updateSliders)

        # Tell self to draw sliders in case the JointStatePublisher already has a robot_description
        self.initialize.emit()

    def initializeSliders(self):
        self.joint_map = {}

        for sl, _ in self.sliders.items():
            self.scroll_layout.removeWidget(sl)
            sl.remove()

        ### Generate sliders ###
        for name in self.jsp.joint_list:
            if name not in self.jsp.free_joints_sub:
                continue
            joint = self.jsp.free_joints_sub[name]

            if joint['min'] == joint['max']:
                continue
            
            if (name in PNEUMATICS_JOINTS):
                button = Button(name)

                self.joint_map[name] = {'button': button.button, 'joint': joint}

                self.scroll_layout.addWidget(button)
                button.button.toggled.connect(lambda event,name=name: self.onInputValueChanged(name))

                self.buttons[button] = button
            else:
                slider = Slider(name)

                self.joint_map[name] = {'display_sub': slider.display_sub, 'slider_sub': slider.slider_sub, 'display_pub': slider.display_pub, 'slider_pub': slider.slider_pub, 'joint': joint}

                self.scroll_layout.addWidget(slider)
                slider.display_pub.textEdited.connect(lambda event,name=name: self.makeSliderEqualToText(name))
                slider.slider_pub.valueChanged.connect(lambda event,name=name: self.makeTextEqualToSlider(name))

                self.sliders[slider] = slider

        # Set zero positions read from parameters
        self.centerEvent(None)

        # Set size of min size of window based on number of sliders.
        if len(self.sliders) >= INIT_NUM_SLIDERS:  # Limits min size to show INIT_NUM_SLIDERS
            num_sliders = INIT_NUM_SLIDERS
        else:
            num_sliders = len(self.sliders)
        scroll_layout_height = num_sliders * DEFAULT_SLIDER_HEIGHT
        scroll_layout_height += (num_sliders + 1) * DEFAULT_CHILD_MARGIN
        self.setMinimumSize(MIN_WIDTH, scroll_layout_height + MIN_HEIGHT)

        self.sliderUpdateTrigger.emit()

    def sliderUpdateCb(self):
        self.sliderUpdateTrigger.emit()

    def initializeCb(self):
        self.initialize.emit()

    # def onButtonClicked(self, name):
    #     # self.jsp.get_logger().info("button changed")
    #     joint_info = self.joint_map[name]
    #     buttonValue = 1 if joint_info['button'].isChecked() == True else 0
    #     joint_info['joint']['position'] = buttonValue
    #     joint_info['display'].setText(str(buttonValue))

    # def onSliderTextEdited(self, slider, joint):
        # self.jsp.get_logger().info(slider.display.text())
        # slider.slider1.setSliderPosition(self.valueToSlider(float(slider.display.displayText()), joint))
    
    
    def makeSliderEqualToText(self, name):
        joint_info = self.joint_map[name]
        textvalue = joint_info['display_pub'].text()
        try:
            joint_info['slider_pub'].setValue(self.valueToSlider(float(textvalue), joint_info['joint']))
            self.onInputValueChanged(name)
        except:
            pass

    def makeTextEqualToSlider(self, name):
        joint_info = self.joint_map[name]
        slidervalue = joint_info['slider_pub'].value()
        joint_info['display_pub'].setText(str(round(self.sliderToValue(slidervalue, joint_info['joint']), 2)))
        self.onInputValueChanged(name)

    def onInputValueChanged(self, name):
        # A slider value was changed, but we need to change the joint_info metadata.
        joint_info = self.joint_map[name]

        if ('slider_pub' in joint_info):
            slidervalue = joint_info['slider_pub'].value()
            joint = joint_info['joint']
            if 'wheel' in name:
                self.jsp.free_joints_pub[name]['velocity'] = self.sliderToValue(slidervalue, joint)
            else:
                self.jsp.free_joints_pub[name]['position'] = self.sliderToValue(slidervalue, joint)
        elif ('button' in joint_info):
            buttonValue = joint_info['joint']['max'] if joint_info['button'].isChecked() == True else joint_info['joint']['min']
            self.jsp.free_joints_pub[name]['position'] = buttonValue
            
    @pyqtSlot()
    def updateSliders(self):
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            if ('slider_sub' in joint_info):
                if 'wheel' in name:
                    slidervalue = self.valueToSlider(joint['velocity'], joint)
                else:
                    slidervalue = self.valueToSlider(joint['position'], joint)
                joint_info['slider_sub'].setValue(slidervalue)
                joint_info['display_sub'].setText(str(round(self.sliderToValue(slidervalue, joint), 2)))
            elif ('button' in joint_info):
                buttonvalue = True if joint['position'] == joint['max'] else False
                if (joint_info['button'].isChecked() == buttonvalue):
                    joint_info['button'].setStyleSheet('background-color: green')
                else:
                    joint_info['button'].setStyleSheet('background-color: yellow')

    
    def resetButtons(self, event):
        self.jsp.get_logger().info("Toggling off")
        for name, joint_info in self.joint_map.items():
            if('button' in joint_info):
                if(joint_info['button'].isChecked()):
                    joint_info['button'].setChecked(False)


    def centerEvent(self, event):
        self.jsp.get_logger().info("Centering")
        for name, joint_info in self.joint_map.items():
            if('slider' in joint_info):
                joint = joint_info['joint']
                joint_info['slider'].setValue(self.valueToSlider(joint['zero'], joint))

    def randomizeEvent(self, event):
        self.jsp.get_logger().info("Randomizing")
        for name, joint_info in self.joint_map.items():
            if('slider' in joint_info):
                joint = joint_info['joint']
                joint_info['slider'].setValue(
                    self.valueToSlider(random.uniform(joint['min'], joint['max']), joint))

    def valueToSlider(self, value, joint):
        # return int(value * 5000)
        return int((value - joint['min']) * float(RANGE) / (joint['max'] - joint['min']))

    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue

    def closeEvent(self, event):
        self.running = False

    def loop(self):
        while self.running:
            rclpy.spin_once(self.jsp, timeout_sec=0.1)


def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file', help='URDF file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    app = QApplication(sys.argv)
    jsp_gui = JointStatePublisherGui('Debugger',
                                     JointStatePublisher(parsed_args.urdf_file))

    jsp_gui.show()

    threading.Thread(target=jsp_gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()