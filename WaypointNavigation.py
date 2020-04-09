# @Author: Saivaraprasad
# @Date:   2020-04-09T22:59:38+05:30
# @Filename: WaypointNavigation.py
# @Last modified by:   Saivaraprasad
# @Last modified time: 2020-04-09T23:00:52+05:30





""" This method of drone control is used when the coordinates of the
target location are known
The recommended method for position control is Vehicle.simple_goto().
This takes a LocationGlobal or LocationGlobalRelative argument.

LocationGlobal = Lat, Lon, Alt_from_MSL (Mean Sea Level)
LocationGlobalRelative = Lat, Lon, Alt_from_EKF_Origin
 """
#Simple Waypoint Navigation
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

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


'''

These are few coordinated taken within the Lords, NIT Durgapur
Latitude, ,Longitude, MSL_Alt       (Altitude from Mean Sea-level)
23.54644, 87.29155  , 91
23.54652, 87.9123   , 91
23.54668, 87.9122   , 92
23.54647, 87.29122  , 92

'''

#Set Waypoints  (Altitude relative to take-off point)
 #LocationGlobalRelative(Lat,Long,ALt)
wp1 = LocationGlobalRelative(23.54644, 87.29155  , 5)
wp2 = LocationGlobalRelative(23.54652, 87.9123   , 5)
wp3 = LocationGlobalRelative(23.54668, 87.9122   , 10)
wp4 = LocationGlobalRelative(23.54647, 87.29122  , 2)

#set the default groundspeed to be used in movement commands
vehicle.groundspeed = 3.2

###################################################################
#####################        MAIN    CODE       ##################
##################################################################


#Establish the connecttion between Companion Computer and the drone
vehicle = Establish_Connection_to_Drone()


#nitialize the takeoff sequence to 2m
arm_and_takeoff(2)
print("Take off complete")
#Hover for some time
time.sleep(10)
########### Take off Complete

'''
vehicle.simple_goto() command can be interrupted by a later command.
It is desirable to wait till the operation is done (Blindly)
Or we can check the proximity to the target by comparing the GPS coordinates
'''

#move to wp1
vehicle.simple_goto(wp1)
#Confirm Vehicle reched the target
#Perform the task at wp1
time.sleep(10)

#move to wp2
vehicle.simple_goto(wp2)
#Perform the task at wp2
time.sleep(10)

#move to wp3
vehicle.simple_goto(wp3)
#Perform the task at wp3
time.sleep(10)

#Come back to the Home
print('Returning to Launch site')
vehicle.mode = VehicleMode("RTL")

time.sleep(30)

# Close vehicle object
vehicle.close()
