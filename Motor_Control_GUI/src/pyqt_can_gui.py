import sys
import os
import subprocess
os.environ["QT_QPA_PLATFORM"] = "xcb"  # Force X11 platform
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton,
                             QLabel, QSlider, QLineEdit, QHBoxLayout, QMainWindow,
                             QGridLayout, QGroupBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QIntValidator, QRegularExpressionValidator
from PyQt6.QtCore import QRegularExpression
import zmq

def resource_path(relative_path):
    """Get absolute path to resource, works for dev and for PyInstaller """
    if hasattr(sys, '_MEIPASS'):
        # PyInstaller extracts to _MEIPASS
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)

# Start the C++ backend as a subprocess
backend_path = resource_path("can_server")  # PyInstaller will place can_server in the same directory
# Try to use xterm, fallback to gnome-terminal if not available
try:
    backend_proc = subprocess.Popen([
        "xterm", "-hold", "-e", backend_path
    
    ])
except FileNotFoundError:
    # xterm not found, try gnome-terminal
    backend_proc = subprocess.Popen([
        "gnome-terminal", "--", backend_path
    ])


class ZMQClient(QThread):
    """
    Handles ZMQ communication for control commands (PUSH) and monitoring data (PULL).
    """
    received = pyqtSignal(str)

    def __init__(self, control_addr="tcp://localhost:5555", monitor_addr="tcp://localhost:5556"):
        super().__init__()
        self.context = zmq.Context()
        self.control_socket = self.context.socket(zmq.PUSH)
        self.control_socket.connect(control_addr)
        self.monitor_socket = self.context.socket(zmq.PULL)
        self.monitor_socket.connect(monitor_addr)
        self._running = True

    def send_control_command(self, can_id, speed_val, forward, backward, motor, brake):
        """
        Send a control command to the C++ server via ZMQ.
        """
        message = f"{can_id} {speed_val} {int(forward)} {int(backward)} {int(motor)} {int(brake)}"
        self.control_socket.send_string(message)
        return message

    def run(self):
        """
        Continuously receive monitoring data from the C++ server via ZMQ and emit it to the GUI.
        """
        while self._running:
            try:
                msg = self.monitor_socket.recv_string(flags=zmq.NOBLOCK)
                self.received.emit(msg)
            except zmq.Again:
                self.msleep(10)

    def stop(self):
        self._running = False
        self.wait()
        self.control_socket.close()
        self.monitor_socket.close()
        self.context.term()

class ControlWindow(QWidget):
    def __init__(self, zmq_client):
        super().__init__()
        self.zmq_client = zmq_client
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle("Motor Control")
        self.setGeometry(100, 100, 300, 350)

        main_layout = QVBoxLayout()

        # CAN ID Input
        can_id_layout = QHBoxLayout()
        self.can_id_label = QLabel("CAN ID (hex):")
        self.can_id_field = QLineEdit()
        self.can_id_field.setPlaceholderText("202")
        hex_validator = QRegularExpressionValidator(QRegularExpression("[0-9A-Fa-f]{1,3}"))
        self.can_id_field.setValidator(hex_validator)
        self.can_id_field.setText("202")  # Default value
        can_id_layout.addWidget(self.can_id_label)
        can_id_layout.addWidget(self.can_id_field)
        main_layout.addLayout(can_id_layout)

        self.status_label = QLabel("Last Sent: None")
        main_layout.addWidget(self.status_label)

        # Speed Control
        self.slider_label = QLabel("Speed: 0")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 32768)
        self.slider.valueChanged.connect(self.update_slider_label)
        main_layout.addWidget(self.slider_label)
        main_layout.addWidget(self.slider)

        # Control Buttons
        self.brake_button = QPushButton("Brake OFF")
        self.brake_button.setCheckable(True)
        self.brake_button.clicked.connect(self.toggle_brake)
        main_layout.addWidget(self.brake_button)

        self.forward_button = QPushButton("Forward OFF")
        self.forward_button.setCheckable(True)
        self.forward_button.clicked.connect(self.toggle_forward)
        main_layout.addWidget(self.forward_button)

        self.backward_button = QPushButton("Backward OFF")
        self.backward_button.setCheckable(True)
        self.backward_button.clicked.connect(self.toggle_backward)
        main_layout.addWidget(self.backward_button)

        self.motor_button = QPushButton("Motor OFF")
        self.motor_button.setCheckable(True)
        self.motor_button.clicked.connect(self.toggle_motor)
        main_layout.addWidget(self.motor_button)

        # Manual Speed Input
        self.input_label = QLabel("Manual Speed:")
        self.input_field = QLineEdit()
        self.input_field.setValidator(QIntValidator(0, 32768))
        self.input_field.setText("0")  # Default value
        input_layout = QHBoxLayout()
        input_layout.addWidget(self.input_label)
        input_layout.addWidget(self.input_field)
        
        self.submit_button = QPushButton("Send")
        self.submit_button.clicked.connect(self.handle_submit)
        input_layout.addWidget(self.submit_button)
        
        main_layout.addLayout(input_layout)
        self.setLayout(main_layout)

    def update_slider_label(self):
        value = self.slider.value()
        self.slider_label.setText(f"Speed: {value}")
        self.input_field.setText(str(value))
        if self.motor_button.isChecked():
            self.send_control_command()

    def toggle_brake(self):
        state = "ON" if self.brake_button.isChecked() else "OFF"
        self.brake_button.setText(f"Brake {state}")

    def toggle_forward(self):
        state = "ON" if self.forward_button.isChecked() else "OFF"
        self.forward_button.setText(f"Forward {state}")

    def toggle_backward(self):
        state = "ON" if self.backward_button.isChecked() else "OFF"
        self.backward_button.setText(f"Backward {state}")

    def toggle_motor(self):
        state = "ON" if self.motor_button.isChecked() else "OFF"
        self.motor_button.setText(f"Motor {state}")

    def handle_submit(self):
        text = self.input_field.text()
        if text.strip() == "":
            self.input_field.setText("0")
        try:
            value = int(self.input_field.text())
            self.slider.setValue(value)
            self.send_control_command()
        except ValueError:
            self.input_field.setText("0")
            self.slider.setValue(0)
            self.send_control_command()

    def send_control_command(self):
        """
        Send the current control command to the C++ server via ZMQ.
        """
        text = self.input_field.text()
        can_id = self.can_id_field.text()
        if text.strip() == "":
            speed_val = 0
            self.input_field.setText("0")
        else:
            try:
                speed_val = int(text)
            except ValueError:
                speed_val = 0
                self.input_field.setText("0")
        forward = self.forward_button.isChecked()
        backward = self.backward_button.isChecked()
        motor = self.motor_button.isChecked()
        brake = self.brake_button.isChecked()
        
        self.zmq_client.send_control_command(can_id, speed_val, forward, backward, motor, brake)
        self.status_label.setText(f"Last Sent: 0x{can_id} 0x{forward | (backward << 1) | (brake << 2) | (motor << 3):02X} 0x{speed_val:02X}")

class MonitorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle("Motor Monitor")
        self.setGeometry(450, 100, 400, 500)
        
        main_layout = QVBoxLayout()
        
        # Motor State Group
        state_group = QGroupBox("Motor State")
        state_layout = QGridLayout()
        self.forward_label = QLabel("--")
        self.backward_label = QLabel("--")
        self.brake_label = QLabel("--")
        self.motor_label = QLabel("--")
        self.encoder_speed_label = QLabel("-- RPM")
        state_layout.addWidget(QLabel("Forward Status:"), 0, 0)
        state_layout.addWidget(self.forward_label, 0, 1)
        state_layout.addWidget(QLabel("Backward Status:"), 1, 0)
        state_layout.addWidget(self.backward_label, 1, 1)
        state_layout.addWidget(QLabel("Brake Status:"), 2, 0)
        state_layout.addWidget(self.brake_label, 2, 1)
        state_layout.addWidget(QLabel("Motor Status:"), 3, 0)
        state_layout.addWidget(self.motor_label, 3, 1)
        state_layout.addWidget(QLabel("Encoder Speed:"), 4, 0)
        state_layout.addWidget(self.encoder_speed_label, 4, 1)
        state_group.setLayout(state_layout)
        
        # Create group boxes for better organization
        temp_group = QGroupBox("Temperature Status")
        power_group = QGroupBox("Power Status")
        fault_group = QGroupBox("Fault Status")
        
        # Temperature Status Layout
        temp_layout = QGridLayout()
        self.motor_temp_label = QLabel("-- °C")
        self.controller_temp_label = QLabel("-- °C")
        temp_layout.addWidget(QLabel("Motor Temperature:"), 0, 0)
        temp_layout.addWidget(self.motor_temp_label, 0, 1)
        temp_layout.addWidget(QLabel("Controller Temperature:"), 1, 0)
        temp_layout.addWidget(self.controller_temp_label, 1, 1)
        temp_group.setLayout(temp_layout)
        
        # Power Status Layout
        power_layout = QGridLayout()
        self.battery_voltage_label = QLabel("-- V")
        self.busbar_current_label = QLabel("-- A")
        power_layout.addWidget(QLabel("Battery Voltage:"), 0, 0)
        power_layout.addWidget(self.battery_voltage_label, 0, 1)
        power_layout.addWidget(QLabel("Busbar Current:"), 1, 0)
        power_layout.addWidget(self.busbar_current_label, 1, 1)
        power_group.setLayout(power_layout)
        
        # Fault Status Layout
        fault_layout = QGridLayout()
        self.fault_code_label = QLabel("--")
        fault_layout.addWidget(QLabel("Fault Code:"), 0, 0)
        fault_layout.addWidget(self.fault_code_label, 0, 1)
        fault_group.setLayout(fault_layout)
        
        # Add all groups to main layout
        main_layout.addWidget(state_group)
        main_layout.addWidget(temp_group)
        main_layout.addWidget(power_group)
        main_layout.addWidget(fault_group)
        
        # Add stretch to push everything to the top
        main_layout.addStretch()
        
        self.setLayout(main_layout)
    
    def update_status(self, can_id, forward, backward, brake, encoder_speed, motor_temp, controller_temp, fault_code, battery_voltage, busbar_current):
        """
        Update the monitoring window with new monitoring data received from the C++ server.
        """
        print(can_id, forward, backward, brake, encoder_speed, motor_temp, controller_temp, fault_code, battery_voltage, busbar_current)
        self.forward_label.setText("ON" if forward else "OFF")
        self.backward_label.setText("ON" if backward else "OFF")
        self.brake_label.setText("ON" if brake else "OFF")
        self.motor_label.setText("ON" if forward or backward else "OFF")
        self.encoder_speed_label.setText(f"{encoder_speed} RPM")
        self.motor_temp_label.setText(f"{motor_temp} °C")
        self.controller_temp_label.setText(f"{controller_temp} °C")
        self.battery_voltage_label.setText(f"{battery_voltage:.2f} V")
        self.busbar_current_label.setText(f"{busbar_current:.2f} A")
        self.fault_code_label.setText(f"0x{fault_code:02X}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN Control System")
        
        # Initialize ZMQ client
        self.zmq_client = ZMQClient()
        self.zmq_client.received.connect(self.handle_monitoring_data)
        self.zmq_client.start()
        
        # Create and show both windows
        self.control_window = ControlWindow(self.zmq_client)
        self.monitor_window = MonitorWindow()
        
        self.control_window.show()
        self.monitor_window.show()

    def handle_monitoring_data(self, message):
        """
        Handle new monitoring data received from the C++ server and update the monitor window.
        """
        try:
            # Parse message: "CAN_ID forward backward brake encoder_speed motor_temp controller_temp fault_code battery_voltage busbar_current"
            parts = message.split()
            if len(parts) == 10:
                can_id = int(parts[0], 16)
                forward = bool(int(parts[1]))
                backward = bool(int(parts[2]))
                brake = bool(int(parts[3]))
                encoder_speed = int(parts[4])
                motor_temp = int(parts[5])
                controller_temp = int(parts[6])
                fault_code = int(parts[7])
                battery_voltage = float(parts[8])
                busbar_current = float(parts[9])
                self.monitor_window.update_status(
                    can_id, forward, backward, brake, encoder_speed, motor_temp, controller_temp, fault_code, battery_voltage, busbar_current
                )
        except (ValueError, IndexError) as e:
            print(f"Error parsing monitoring data: {e}")

    def closeEvent(self, event):
        self.backend_proc.terminate()
        self.backend_proc.wait()
        self.zmq_client.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    sys.exit(app.exec())
