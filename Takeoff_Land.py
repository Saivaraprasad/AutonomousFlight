# @Author: Saivaraprasad
# @Date:   2020-04-09T23:09:57+05:30
# @Filename: Takeoff_Land.py
# @Last modified by:   Saivaraprasad
# @Last modified time: 2020-04-09T23:10:01+05:30
'''
This is a simple Python script that connects to Pixhawk over Serial port of
Raspberry Pi and then Takes off dronr to given Altitude (Relative to Ground)
Hovers there for a while and lands
'''

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

ConnectionString = "/dev/ttyS0"
#USB on Rasperry Pi = "/dev/ttyACM0"
#Serial on Pi to Telemetry 2 on Pixhawk = "/dev/ttyS0"

BaudRate = 38400
#USB can go 115200 or even more(try)
#Serial is not working above 38400  (Debug)


# Connect to the Vehicle
print("Connecting to vehicle on :  "%s, ConnectionString)
print("Baudrate   :  "  %d, BaudRate)

vehicle = connect(ConnectionString, baud=BaudRate, wait_ready=True)
print("Connected to the vehicle")

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print( "Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print( " Waiting for vehicle to initialise...")
    time.sleep(1)

  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print( " Waiting for arming...")
    time.sleep(1)

  print( "Taking off!")
  time.sleep(5)
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
      print("Reached target altitude")
      break
    time.sleep(1)

#nitialize the takeoff sequence to 2m
arm_and_takeoff(2)

print("Take off complete")

#Hover for 10 seconds
time.sleep(10)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
