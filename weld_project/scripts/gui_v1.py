#!/usr/bin/env python
import sys 
import rospy 
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from rviz import bindings as rviz 

class URWeldingGUI (QWidget):
    def __init__(self):
        # print(vars(rviz))
        # # print('above below')
        # # print(vars(rviz))
        # exit()
        super(URWeldingGUI, self).__init__()

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        #Read from the RViz config file 
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        # print(config)
        # print(dir(config))

        # print('config check')

        # print(vars(config))
        # /home/rylab/ur10e_wp_2/src/ur10e_wp_moveit_config/launch
        reader.readFile(config, "/home/rylab/ur10e_wp_2/src/ur10e_wp_moveit_config/launch/moveit.rviz")
        self.frame.load(config)

        self.setWindowTitle("UR_Welding_GUI")
        self.resize(2000,2000)

        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(True) 
        self.manager = self.frame.getManager()

        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

        #create main layout
        self.main_layout = QHBoxLayout()  # to arrange rviz and buttons side by side
        self.setLayout(self.main_layout)

        #create left layout and widget 
        self.left_widget = QWidget()
        self.left_layout = QVBoxLayout(self.left_widget)  # for buttons and sliders
        self.left_widget.setMaximumWidth(500) #limit width of the widget

        self.init_buttons()
        self.init_sliders()

        # Add left widget to main layout  
        self.main_layout.addWidget(self.left_widget)

        # Add RViz frame to main layout
        self.main_layout.addWidget(self.frame)
        

        # Replace this with your ROS publisher instantiation
        # self.dry_run_pub = rospy.Publisher("Your dry run topic", Bool, queue_size=10)
        # self.welding_start_pub = rospy.Publisher("Your start welding topic", Bool, queue_size=10)
        # self.welding_stop_pub = rospy.Publisher("Your stop welding topic", Bool, queue_size=10)
        # self.travel_speed_pub = rospy.Publisher("Your travel speed topic", Int32, queue_size=10)
        # self.voltage_pub = rospy.Publisher("Your voltage topic", Int32, queue_size=10)
        # self.wire_feed_pub = rospy.Publisher("Your wire feed topic", Int32, queue_size=10)

    def init_buttons(self):
        self.dryRunButton = self.create_button('Dry Run', self.dry_run, "background-color: yellow;")
        self.startButton = self.create_button('Start Welding', self.start_welding, "background-color: green;")
        self.stopButton = self.create_button('Stop Welding', self.stop_welding, "background-color: red;")
        top_button = self.create_button('Top View', self.on_top_button_click)
        side_button = self.create_button('Side View', self.on_side_button_click)

        self.left_layout.addWidget(self.dryRunButton)
        self.left_layout.addWidget(self.startButton)
        self.left_layout.addWidget(self.stopButton)
        self.left_layout.addWidget(top_button)
        self.left_layout.addWidget(side_button)

    def init_sliders(self):
        self.travelSpeedLabel = QLabel('Welding Gun Travel Speed: 10 ipm', self)
        self.travelSpeedSlider = self.create_slider(10, 25, self.update_travel_speed)

        self.weldingVoltageLabel = QLabel('Welding Voltage: 10 V', self)
        self.weldingVoltageSlider = self.create_slider(10, 38, self.update_voltage)

        self.wireFeedLabel = QLabel('Wire Feed Speed: 50 ipm', self)
        self.wireFeedSlider = self.create_slider(50, 780, self.update_wire_feed)

        self.left_layout.addWidget(self.travelSpeedLabel)
        self.left_layout.addWidget(self.travelSpeedSlider)
        self.left_layout.addWidget(self.weldingVoltageLabel)
        self.left_layout.addWidget(self.weldingVoltageSlider)
        self.left_layout.addWidget(self.wireFeedLabel)
        self.left_layout.addWidget(self.wireFeedSlider)

    def create_button(self, text, callback, style=None):
        button = QPushButton(text, self)
        button.clicked.connect(callback)
        if style:
            button.setStyleSheet(style)
        return button

    def create_slider(self, min_val, max_val, callback):
        slider = QSlider(Qt.Horizontal, self)
        slider.setRange(min_val, max_val)
        slider.valueChanged.connect(callback)
        slider.setStyleSheet("""
            QSlider::groove:horizontal {background: silver; height: 5px; border-radius: 3px;}
            QSlider::handle:horizontal {background: blue; width: 18px; margin: -6px 0; border-radius: 9px;}
        """)
        return slider

    def dry_run(self):
        self.dry_run_pub.publish(True)
        rospy.set_param('welding_status', 'Dry Run before actual welding')

    def start_welding(self):
        self.welding_start_pub.publish(True)
        rospy.set_param('welding_status', 'Started')

    def stop_welding(self):
        self.welding_stop_pub.publish(True)
        rospy.set_param('welding_status', 'Stopped')

    def update_travel_speed(self, value):
        self.travel_speed_pub.publish(value)
        self.weldingVoltageLabel.setText(f'Welding Gun Travel Speed: {value} ipm')
        rospy.set_param('travel_speed', value)

    def update_voltage(self, value):
        self.voltage_pub.publish(value)
        self.weldingVoltageLabel.setText(f'Welding Voltage: {value} V')
        rospy.set_param('welding_voltage', value)

    def update_wire_feed(self, value):
        self.wire_feed_pub.publish(value)
        self.wireFeedLabel.setText(f'Wire Feed Speed: {value} ipm')
        rospy.set_param('wire_feed_speed', value)

    def on_top_button_click(self):
        self.switch_to_view("Top View")

    def on_side_button_click(self):
        self.switch_to_view("Side View")

    def switch_to_view(self, view_name):
        view_man = self.manager.getViewManager()
        for view in view_man.getViews():
            if view.getName() == view_name:
                view_man.setCurrentForm(view)
                return
        print(f"Did not find view name {view_name}.")

def main():
    app = QApplication(sys.argv)
    myviz = URWeldingGUI()
    myviz.show()

    rospy.init_node('ur_welding_gui')

    timer = QTimer()
    timer.timeout.connect(lambda: None if rospy.is_shutdown() else None)
    timer.start(500)  # every 500 ms

    sys.exit(app.exec_()) 

if __name__ == '__main__':
    main() 
