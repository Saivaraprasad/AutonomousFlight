# @Author: Saivaraprasad
# @Date:   2020-03-04T12:34:20+05:30
# @Filename: Connecton_Check.py
# @Last modified by:   Saivaraprasad
# @Last modified time: 2020-03-04T12:39:42+05:30

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

'''
This code is just to check the connection to the Vehicle
'''
def Establish_Connection_to_Drone() :

    ConnectionString = "/dev/ttyS0"
    #USB on Rasperry Pi = "/dev/ttyACM0"
    #Serial on Pi to Telemetry 2 on Pixhawk = "/dev/ttyS0"

    BaudRate = 38400
    #USB can go 115200 or even more(try)
    #Serial is not working above 38400  (Debug)

    # Connect to the Vehicle
    print("Connecting to vehicle on :  ", ConnectionString)
    print("Baudrate   :  " , BaudRate)

    vehicle = connect(ConnectionString, baud=BaudRate, wait_ready=True)
    print("Connected to the vehicle")
    return vehicle


######################################################################
###################     MAIN    CODE    #############################
#####################################################################
vehicle = Establish_Connection_to_Drone()
print(vehicle.is_armable)

# vehicle is an instance of the Vehicle class
print("Autopilot Firmware version: %s" % vehicle.version)
print( "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print( "Global Location: %s" % vehicle.location.global_frame)
print( "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print( "Local Location: %s" % vehicle.location.local_frame)    #NED
print( "Attitude: %s" % vehicle.attitude)
print( "Velocity: %s" % vehicle.velocity)
print( "GPS: %s" % vehicle.gps_0)
print( "Groundspeed: %s" % vehicle.groundspeed)
print( "Airspeed: %s" % vehicle.airspeed)
print( "Gimbal status: %s" % vehicle.gimbal)
print( "Battery: %s" % vehicle.battery)
print( "EKF OK?: %s" % vehicle.ekf_ok)
print( "Last Heartbeat: %s" % vehicle.last_heartbeat)
print( "Rangefinder: %s" % vehicle.rangefinder)
print( "Rangefinder distance: %s" % vehicle.rangefinder.distance)
print( "Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print( "Heading: %s" % vehicle.heading)
print( "Is Armable?: %s" % vehicle.is_armable)
print( "System status: %s" % vehicle.system_status.state)
print( "Mode: %s" % vehicle.mode.name)    # settable
print( "Armed: %s" % vehicle.armed)    # settable

# Close vehicle object
vehicle.close()
