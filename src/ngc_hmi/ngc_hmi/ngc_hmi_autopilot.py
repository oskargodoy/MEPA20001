import sys
import signal
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSpacerItem, QSizePolicy, QLCDNumber, QDial, QSlider, QLineEdit
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import rclpy
from rclpy.node import Node
from ngc_interfaces.msg import Eta, Nu, SystemMode
from ngc_utils.qos_profiles import default_qos_profile
import numpy as np
import ngc_utils.math_utils as mu

class AutopilotHMI(Node):
    def __init__(self):
        super().__init__('ship_autopilot_hmi')

        # Initialize setpoints
        self.heading_setpoint = 0.5  # Initial heading setpoint
        self.surge_setpoint   = 0.0  # Initial surge speed setpoint in knots

        # Subscribers to eta_sim and nu_sim
        self.create_subscription(Eta, 'eta_sim', self.update_eta_feedback, default_qos_profile)
        self.create_subscription(Nu, 'nu_sim', self.update_nu_feedback, default_qos_profile)

        # Publishers for eta_setpoint, nu_setpoint, and system_mode
        self.eta_publisher = self.create_publisher(Eta, 'eta_setpoint', default_qos_profile)
        self.nu_publisher = self.create_publisher(Nu, 'nu_setpoint', default_qos_profile)
        self.system_mode_publisher = self.create_publisher(SystemMode, 'system_mode', default_qos_profile)

        # Initialize the UI
        self.init_ui()

    def init_ui(self):
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Ship Autopilot HMI")
        self.layout = QVBoxLayout()

        # Heading setpoint with a continuous wheel (QDial)
        self.add_wheel_layout('Heading', 0, 360, '°', self.update_heading_setpoint)

        # Surge speed setpoint with finer granularity in knots
        self.add_slider_layout('Surge Speed', 0, 10, 'knots', self.update_surge_setpoint, resolution=0.1)

        # Add "Standby" and "Auto" buttons at the bottom
        button_layout = QHBoxLayout()
        
        self.standby_button = QPushButton("Standby")
        self.standby_button.clicked.connect(self.set_standby_mode)
        button_layout.addWidget(self.standby_button)

        self.auto_button = QPushButton("Auto")
        self.auto_button.clicked.connect(self.set_auto_mode)
        button_layout.addWidget(self.auto_button)
        
        self.layout.addLayout(button_layout)

        # Add input fields for fx_test and fz_test with a "Test" button
        test_layout = QHBoxLayout()
        
        self.fx_input = QLineEdit()
        self.fx_input.setPlaceholderText("Enter fx_test")
        test_layout.addWidget(self.fx_input)

        self.fz_input = QLineEdit()
        self.fz_input.setPlaceholderText("Enter fz_test")
        test_layout.addWidget(self.fz_input)

        self.test_button = QPushButton("Test")
        self.test_button.clicked.connect(self.set_test_force_mode)
        test_layout.addWidget(self.test_button)
        
        self.layout.addLayout(test_layout)

        test_layout2 = QHBoxLayout()
        
        self.rpm_strb_input = QLineEdit()
        self.rpm_strb_input.setPlaceholderText("Starboard rpm")
        test_layout2.addWidget(self.rpm_strb_input)

        self.rpm_port_input = QLineEdit()
        self.rpm_port_input.setPlaceholderText("Port rpm")
        test_layout2.addWidget(self.rpm_port_input)

        self.test_button = QPushButton("Test")
        self.test_button.clicked.connect(self.set_test_rpm_mode)
        test_layout2.addWidget(self.test_button)

        self.layout.addLayout(test_layout2)

        # Set layout and show window
        self.window.setLayout(self.layout)
        self.window.show()

        # Timer to process ROS 2 callbacks and publish setpoints regularly
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin_and_publish_setpoints)
        self.timer.start(100)  # 100 ms interval (10 Hz)

        self.standby_button.click()

    def set_standby_mode(self):

        msg = SystemMode()
        msg.standby_mode               = True
        msg.auto_mode                  = False
        msg.test_normalized_force_mode = False
        msg.test_rpm_output_mode       = False
        msg.fx_test                    = 0.0
        msg.fz_test                    = 0.0
        msg.rpm_strb_test              = 0.0
        msg.rpm_port_test              = 0.0

        self.system_mode_publisher.publish(msg)
        self.get_logger().info("Standby mode activated")
        self.standby_button.setStyleSheet("background-color: blue; color: white;")
        self.auto_button.setStyleSheet("")  # Reset to default

    def set_auto_mode(self):
        
        msg = SystemMode()
        msg.standby_mode               = False
        msg.auto_mode                  = True
        msg.test_normalized_force_mode = False
        msg.test_rpm_output_mode       = False
        msg.fx_test                    = 0.0
        msg.fz_test                    = 0.0
        msg.rpm_strb_test              = 0.0
        msg.rpm_port_test              = 0.0
        

        self.system_mode_publisher.publish(msg)
        self.get_logger().info("Auto mode activated")
        self.auto_button.setStyleSheet("background-color: blue; color: white;")
        self.standby_button.setStyleSheet("")  # Reset to default

    def set_test_force_mode(self):
        try:
            fx_test = float(self.fx_input.text())
            fz_test = float(self.fz_input.text())
        
        except ValueError:
            self.get_logger().error("Invalid input for fx_test or fz_test")
            return
        
        msg = SystemMode()
        msg.standby_mode               = False
        msg.auto_mode                  = False
        msg.test_normalized_force_mode = True
        msg.test_rpm_output_mode       = False
        msg.fx_test                    = fx_test
        msg.fz_test                    = fz_test
        msg.rpm_strb_test              = 0.0
        msg.rpm_port_test              = 0.0

        self.system_mode_publisher.publish(msg)
        self.get_logger().info(f"Test normalized force mode activated with fx_test={fx_test}, fz_test={fz_test}")

    def set_test_rpm_mode(self):
        try:
            rpm_port = float(self.rpm_port_input.text())
            rpm_stbd = float(self.rpm_strb_input.text())
        except ValueError:
            self.get_logger().error("Invalid input of rpm")
            return
        
        msg = SystemMode()
        msg.standby_mode               = False
        msg.auto_mode                  = False
        msg.test_normalized_force_mode = False
        msg.test_rpm_output_mode       = True
        msg.fx_test                    = 0.0
        msg.fz_test                    = 0.0
        msg.rpm_strb_test              = rpm_stbd
        msg.rpm_port_test              = rpm_port
        
        self.system_mode_publisher.publish(msg)
        self.get_logger().info(f"Test rpm setpoint mode activated with rpm_stbd={rpm_stbd}, rpm_port={rpm_port}")

    def ros_spin_and_publish_setpoints(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.publish_setpoints()

    def add_wheel_layout(self, label_text, min_val, max_val, unit, update_method):
        wheel_layout = QVBoxLayout()
        label = QLabel(f'{label_text} ({unit})')
        label.setAlignment(Qt.AlignCenter)
        wheel_layout.addWidget(label)

        dial = QDial()
        dial.setMinimum(min_val)
        dial.setMaximum(max_val)
        dial.setWrapping(True)
        dial.setValue(180)
        dial.setNotchesVisible(True)
        dial.valueChanged.connect(lambda value: self.handle_heading_wraparound(self.remap_dial_value(value), update_method))
        wheel_layout.addWidget(dial)

        lcd_display = QLCDNumber()
        lcd_display.setSegmentStyle(QLCDNumber.Flat)
        lcd_display.setDigitCount(6)
        dial.valueChanged.connect(lambda value: lcd_display.display(self.remap_dial_value(value)))
        wheel_layout.addWidget(lcd_display)

        self.layout.addLayout(wheel_layout)
        self.layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def remap_dial_value(self, value):
        remapped_value = (value - 180) % 360
        return remapped_value

    def add_slider_layout(self, label_text, min_val, max_val, unit, update_method, resolution=1.0):
        slider_layout = QVBoxLayout()
        label = QLabel(f'{label_text} ({unit})')
        label.setAlignment(Qt.AlignCenter)
        slider_layout.addWidget(label)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(min_val / resolution))
        slider.setMaximum(int(max_val / resolution))
        slider.setValue((min_val + max_val) // 2)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(int(1 / resolution))
        slider.valueChanged.connect(lambda value: update_method(value * resolution))
        slider_layout.addWidget(slider)

        lcd_display = QLCDNumber()
        lcd_display.setSegmentStyle(QLCDNumber.Flat)
        lcd_display.setDigitCount(6)
        lcd_display.display(slider.value() * resolution)
        slider.valueChanged.connect(lambda value: lcd_display.display(value * resolution))
        slider_layout.addWidget(lcd_display)

        self.layout.addLayout(slider_layout)
        self.layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def handle_heading_wraparound(self, current_value, update_method):
        update_method(current_value)

    def update_eta_feedback(self, msg):
        pass

    def update_nu_feedback(self, msg):
        pass

    def update_heading_setpoint(self, value):
        self.heading_setpoint = value

    def update_surge_setpoint(self, value):
        self.surge_setpoint = value

    def publish_setpoints(self):
        
        eta_msg = Eta()
        eta_msg.psi = float(mu.mapToPiPi(np.deg2rad(self.heading_setpoint)))
        self.eta_publisher.publish(eta_msg)

        nu_msg = Nu()
        nu_msg.u = float(self.surge_setpoint) * 0.514444
        self.nu_publisher.publish(nu_msg)

def main(args=None):
    rclpy.init(args=args)
    autopilot_hmi = AutopilotHMI()
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(autopilot_hmi, timeout_sec=0.01))
    timer.start(100)

    def signal_handler(sig, frame):
        print("SIGINT received, shutting down...")
        autopilot_hmi.destroy_node()
        rclpy.shutdown()
        QApplication.quit()

    signal.signal(signal.SIGINT, signal_handler)
    sys.exit(autopilot_hmi.app.exec_())

if __name__ == '__main__':
    main()
