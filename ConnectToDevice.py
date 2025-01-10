import sys
import numpy as np
import pyqtgraph as pg
import serial
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLineEdit, QPushButton, QLabel, QMessageBox,
                             QComboBox, QGridLayout)
from PyQt5.QtCore import Qt, QTimer

class DeviceControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Temperature Control")
        self.setGeometry(100, 100, 1200, 800)

        self.serial_port = 'COM6'  # Replace with your serial port
        self.baud_rate = 115200
        self.ser = None

        self.fieldnames = [
            'Set Kp',
            'Set Ki',
            'Set Kd',
            'Setpoint'
        ]

        self.command_prefixes = [
            '<#',  # Kp
            '<$',  # Ki
            '<^',  # Kd
            '<*'   # Setpoint
        ]

        main_widget = QWidget()
        main_layout = QVBoxLayout()

        command_section = QWidget()
        command_layout = QGridLayout()

        self.parameter_inputs = {}
        for i, field in enumerate(self.fieldnames):
            label = QLabel(field)
            input_field = QLineEdit()
            input_field.setPlaceholderText(f"Enter {field}")
            command_layout.addWidget(label, i, 0)
            command_layout.addWidget(input_field, i, 1)
            self.parameter_inputs[field] = input_field

        send_button = QPushButton("Send Commands")
        send_button.clicked.connect(self.send_commands)
        command_layout.addWidget(send_button, len(self.fieldnames), 1)

        command_section.setLayout(command_layout)

        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w')
        self.graph_widget.setTitle("Live Device Data")
        self.graph_widget.setLabel('left', 'Value')
        self.graph_widget.setLabel('bottom', 'Time')

        # Add a legend to the graph
        self.graph_widget.addLegend()

        # Define curves for plotting
        self.temperature_curve = self.graph_widget.plot(pen='r', name='Temperature (PID)')
        self.error_curve = self.graph_widget.plot(pen='g', name='Error')
        self.setpoint_curve = self.graph_widget.plot(pen='b', name='Setpoint')
        self.ambient_curve = self.graph_widget.plot(pen='m', name='Ambient Temp')

        self.time_data = []
        self.temperature_data = []
        self.error_data = []
        self.setpoint_data = []
        self.ambient_data = []

        self.connect_serial()

        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.update_data)
        self.data_timer.start(60)  # 60ms interval (keeps checking every 60ms)

        main_layout.addWidget(command_section)
        main_layout.addWidget(self.graph_widget)

        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        self.current_values_widget = QWidget()
        current_values_layout = QVBoxLayout()

        self.current_temperature_label = QLabel("Temperature: --")
        self.current_error_label = QLabel("Error: --")
        self.current_setpoint_label = QLabel("Setpoint: --")
        self.current_ambient_label = QLabel("Ambient Temp: --")

        current_values_layout.addWidget(self.current_temperature_label)
        current_values_layout.addWidget(self.current_error_label)
        current_values_layout.addWidget(self.current_setpoint_label)
        current_values_layout.addWidget(self.current_ambient_label)

        self.current_values_widget.setLayout(current_values_layout)
        main_layout.addWidget(self.current_values_widget)

    def connect_serial(self):
        """Establish serial connection"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1
            )
            print(f"Connected to serial port {self.serial_port}")
        except (OSError, FileNotFoundError) as e:
            self.show_error("Serial Connection Error", f"Error: {e}")
            self.ser = None
        except Exception as e:
            self.show_error("Serial Connection Error", f"Unexpected error: {e}")
            self.ser = None

    def send_commands(self):
        """Send commands to the device via serial port"""
        if not self.ser or not self.ser.is_open:
            self.show_error("Device Error", "No serial connection established.")
            return

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        commands = []
        for i, (field, input_field) in enumerate(self.parameter_inputs.items()):
            value = input_field.text().strip()

            if not value:
                continue

            try:
                float_value = float(value)
                command = f'{self.command_prefixes[i]}{float_value}>'
                commands.append(command)
            except ValueError:
                self.show_error("Invalid Input", f"{field} must be a number")
                return

        if not commands:
            self.show_error("Command Error", "No valid values to send.")
            return

        try:
            for command in commands:
                self.ser.write(command.encode('utf-8'))
                print(f"Sent command: {command.strip()}")
        except Exception as e:
            self.show_error("Command Send Error", str(e))

    def show_error(self, title, message):
        """Display error message to the user"""
        error_dialog = QMessageBox()
        error_dialog.setIcon(QMessageBox.Warning)
        error_dialog.setText(title)
        error_dialog.setInformativeText(message)
        error_dialog.setWindowTitle("Error")
        error_dialog.exec_()

    def update_data(self):
        """Receive and parse live data from the serial device"""
        if not self.ser or not self.ser.is_open:
            # Simulated data when no serial connection is established
            current_time = len(self.time_data) * 0.06  # Assuming 60ms intervals
            temperature = 45.64 + np.random.normal(0, 0.5)
            error = 1.43 + np.random.normal(0, 0.1)
            setpoint = 83.05 + np.random.normal(0, 0.3)
            ambient = 23.45 + np.random.normal(0, 0.2)
        else:
            try:
                if self.ser.in_waiting > 0:
                    data_response = self.ser.readline().decode('utf-8').strip()
                    if data_response.startswith('<') and data_response.endswith('>'):
                        data_str = data_response[1:-1]
                        try:
                            temperature, error, setpoint, ambient = map(float, data_str.split('/'))
                            current_time = len(self.time_data) * 0.06  # Assuming 60ms intervals
                        except ValueError:
                            print(f"Invalid data format: {data_response}")
                            return
                    else:
                        print(f"Unexpected data format: {data_response}")
                        return
                else:
                    return
            except Exception as e:
                print(f"Error reading serial data: {e}")
                return

        # Update data arrays
        self.time_data.append(current_time)
        self.temperature_data.append(temperature)
        self.error_data.append(error)
        self.setpoint_data.append(setpoint)
        self.ambient_data.append(ambient)

        # Limit the number of points to avoid memory overload
        max_points = 5000000
        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            self.temperature_data = self.temperature_data[-max_points:]
            self.error_data = self.error_data[-max_points:]
            self.setpoint_data = self.setpoint_data[-max_points:]
            self.ambient_data = self.ambient_data[-max_points:]

        # Update the graph
        self.temperature_curve.setData(np.array(self.time_data), np.array(self.temperature_data))
        self.error_curve.setData(np.array(self.time_data), np.array(self.error_data))
        self.setpoint_curve.setData(np.array(self.time_data), np.array(self.setpoint_data))
        self.ambient_curve.setData(np.array(self.time_data), np.array(self.ambient_data))

        # Update labels with current values
        self.current_temperature_label.setText(f"Temperature: {temperature:.2f}°C")
        self.current_error_label.setText(f"Error: {error:.2f}")
        self.current_setpoint_label.setText(f"Setpoint: {setpoint:.2f}°C")
        self.current_ambient_label.setText(f"Ambient Temp: {ambient:.2f}°C")

    def closeEvent(self, event):
        """Ensure serial port is closed when the application exits"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("Serial port closed.")
            except Exception as e:
                print(f"Error closing serial port: {e}")
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = DeviceControlApp()
    window.show()
    sys.exit(app.exec_())

main()
