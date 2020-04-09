# @Author: Saivaraprasad
# @Date:   2020-03-04T00:45:07+05:30
# @Filename: Velocity_Mode_Control.py
# @Last modified by:   Saivaraprasad
# @Last modified time: 2020-03-04T11:06:01+05:30
'''
This code is just to understand about how Velocity Mode Control works


Reference : https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html

The type_mask parameter is a bitmask that indicates which of the other
parameters in the message are used/ignored by the vehicle
 (0 means that the dimension is enabled, 1 means ignored).

In the bitmask - 0b0000111ihgfedcba
    Bit a : X
    Bit b : Y
    Bit c : Z
    Bit d : Vx
    Bit e : Vy
    Bit f : Vz
    Bit g : Ax
    Bit h : Ay
    Bit i : Az

In the example the value 0b0000111111000111 is used to enable the velocity components
   Vx : Parellel to North-South Axis
   Vy : Parellel to East-West   Axis
   Vz : Perpendicular to to the plane of earth

        @ Set up velocity mappings
           * velocity_x > 0 => fly North
           * velocity_x < 0 => fly South
           * velocity_y > 0 => fly East
           * velocity_y < 0 => fly West
           * velocity_z < 0 => ascend
           * velocity_z > 0 => descend

'''

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import time

######################################################################
##############  FUNCTION DEFINITIONS    #############################
#####################################################################

def velocity_control(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def Establish_Connection_to_Drone() :

    ConnectionString = "/dev/ttyACM0"
    #USB on Rasperry Pi = "/dev/ttyACM0"
    #Serial on Pi to Telemetry 2 on Pixhawk = "/dev/ttyS0"

    BaudRate = 115200
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


######################################################################
###################     MAIN    CODE    #############################
#####################################################################
vehicle = Establish_Connection_to_Drone()
#- Takeoff
arm_and_takeoff(0.4)

Vx = 5 #m/s
Vy = 0 #m/s
Vz = 0 #m/s

velocity_control(vehicle, Vx, Vy, Vz)
print("Velocity Command Sent")
time.sleep(10)

#Come back to the Home
print('Returning to Launch site')
vehicle.mode = VehicleMode("RTL")

time.sleep(30)

# Close vehicle object
vehicle.close()
