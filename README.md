# PIDDataApp
## Usage
This script is used to show app with graphs of data taken from arduino device. 
We get the data from PID regulator and we can send commands to it.
In __init__ function we define port we using and baud rate so we can connect properly with device.
We create graphs here and also define commands that we send to PID.
## connect_serial
This function connects us to the device using serial library
## send_commands
This function let us send commands to the PID regulator :
kp, ki, kd and setpoint
Arduino device was previously programmed so it gets commands we send them 
## update_data
Here we update our data into the graphs that were previously initialized in __init__ function every 60ms, 
if there is no serial connection data is randomly distributed into graphs.
## Environment
In bash you need to type :
```
./setup.sh
```
to setup environemt and install required dependencies