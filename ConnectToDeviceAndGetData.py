import sys
import numpy as np
import pyqtgraph as pg
import serial
#import pyvisa
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QLineEdit, QPushButton, QLabel, QMessageBox, 
                             QComboBox, QGridLayout)
from PyQt5.QtGui import QPalette, QPixmap, QBrush
from PyQt5.QtCore import Qt, QTimer

class DeviceControlApp(QMainWindow):     
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Temperature Control")
        self.setGeometry(100, 100, 1200, 800)
        
        #self.rm = pyvisa.ResourceManager()
        #print(f'Connected VISA resources: {self.rm.list_resources()}')
        
        self.serial_port = '/dev/ttyUSB0'  # Replace with your serial port NIGGA
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
        self.data_timer.start(100)  # 100ms interval
   
        main_layout.addWidget(command_section)
        main_layout.addWidget(self.graph_widget)
        
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)
    
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
        
        commands = []
        for i, (field, input_field) in enumerate(self.parameter_inputs.items()):
            value = input_field.text().strip()
            
            if not value:
                self.show_error("Invalid Input", f"{field} cannot be empty")
                return
            
            try:
                float_value = float(value)
                
                command = f'{self.command_prefixes[i]}{float_value}>\n'
                commands.append(command)
            except ValueError:
                self.show_error("Invalid Input", f"{field} must be a number")
                return
        
        try:
            for command in commands:
                self.ser.write(command.encode('utf-8'))
                print(f"Sent command: {command.strip()}")
        except Exception as e:
            self.show_error("Command Send Error", str(e))
        finally:
            self.ser.close()
            self.ser.open()
    
    def show_error(self, title, message):
        """Display error message to the user"""
        error_dialog = QMessageBox()
        error_dialog.setIcon(QMessageBox.Warning)
        error_dialog.setText(title)
        error_dialog.setInformativeText(message)
        error_dialog.setWindowTitle("Error")
        error_dialog.exec_()
    
    def update_data(self):
        """Receive and parse live data from the serial device
        
        Expected data format: <temperature/error/setpoint/ambient>
        e.g., <45.64/1.43/83.05/23.45>
        """
        if not self.ser or not self.ser.is_open:
            # If no serial connection, use simulated data XD
            current_time = len(self.time_data)
            temperature = 45.64 + np.random.normal(0, 5)
            error = 1.43 + np.random.normal(0, 0.1)
            setpoint = 83.05 + np.random.normal(0, 23)
            ambient = 23.45 + np.random.normal(0, 0.9)
        else:
            try:
                if self.ser.in_waiting > 0:
                    data_response = self.ser.readline().decode('utf-8').strip()
                    
                    if data_response.startswith('<') and data_response.endswith('>'):
                        data_str = data_response[1:-1]
                        try:
                            temperature, error, setpoint, ambient = map(float, data_str.split('/'))
                            current_time = len(self.time_data)
                        except (ValueError, TypeError):
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

        self.time_data.append(current_time)
        self.temperature_data.append(temperature)
        self.error_data.append(error)
        self.setpoint_data.append(setpoint)
        self.ambient_data.append(ambient)
   
        max_points = 500 # number of data points
        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            self.temperature_data = self.temperature_data[-max_points:]
            self.error_data = self.error_data[-max_points:]
            self.setpoint_data = self.setpoint_data[-max_points:]
            self.ambient_data = self.ambient_data[-max_points:]

        self.temperature_curve.setData(self.time_data, self.temperature_data)
        self.error_curve.setData(self.time_data, self.error_data)
        self.setpoint_curve.setData(self.time_data, self.setpoint_data)
        self.ambient_curve.setData(self.time_data, self.ambient_data)
    
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

if __name__ == '__main__':
    main()