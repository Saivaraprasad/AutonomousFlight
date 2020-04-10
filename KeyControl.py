# @Author: Saivaraprasad
# @Date:   2020-03-04T11:03:26+05:30
# @Filename: Key_Control.py
# @Last modified by:   Saivaraprasad
# @Last modified time: 2020-03-04T12:39:19+05:30


from pynput import keyboard
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import time

Vx = 0
Vy = 0
Vz = 0
t_flag = True



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


def Perfom_Actions(vehicle,key):
    global Vx
    global Vy
    global Vz
    global t_flag

    #print(key)
    if key == 'r':
        print("RTL  : ", Vx, "    ", Vy, "    ", Vz)
        vehicle.mode = VehicleMode("RTL")
        time.sleep(20)
    elif key == 'w':
        Vz = -1
        print("Ascend   : ", Vx, "    ", Vy, "    ", Vz)
        velocity_control(vehicle, Vx, Vy, Vz)
    elif key == 's':
        Vz = 1
        print("Descend  : ", Vx, "    ", Vy, "    ", Vz)
        velocity_control(vehicle, Vx, Vy, Vz)
    elif key == 'a':
        print("Yaw_Left : ", Vx, "    ", Vy, "    ", Vz)
    elif key == 'd':
        print("Yaw Right : ", Vx, "    ", Vy, "    ", Vz)
    elif key == 'l':
        print("land : ", Vx, "    ", Vy, "    ", Vz)
    elif key == 't' :
        if t_flag :
            print("Takeoff : ", Vx, "    ", Vy, "    ", Vz)
            arm_and_takeoff(2)
            time.sleep(10)
            t_flag = False
        else :
            print("Already Airborne : " ,Vx, "    ", Vy, "    ", Vz)


def Perfom_Special_Actions(vehicle, key):
        global Vx
        global Vy
        global Vz
        global t_flag

        if key == key.up:
            Vx = 2
            print("Forward : ", Vx, "    ", Vy, "    ", Vz)
            velocity_control(vehicle, Vx, Vy, Vz)
        elif key == key.down:
            Vx = -2
            print("Backward : ", Vx, "    ", Vy, "    ", Vz)
            velocity_control(vehicle, Vx, Vy, Vz)
        elif key == key.left:
            Vy = 2
            print("Left : ", Vx, "    ", Vy, "    ", Vz)
            velocity_control(vehicle, Vx, Vy, Vz)
        elif key == key.right:
            Vy = -2
            print("Right : ", Vx, "    ", Vy, "    ", Vz)
            velocity_control(vehicle, Vx, Vy, Vz)


def on_press(vehicle,key):
    try:
        #print('alphanumeric key {0} pressed'.format(key.char))
        Perfom_Actions(vehicle, key.char)
    except AttributeError:
        #print('special key {0} pressed'.format(key))
        Perfom_Special_Actions(vehicle,key)

def on_release(vehicle,key):
    global Vx
    global Vy
    global Vz
    global t_flag
    #print('{0} released'.format(key))
    Vx = Vy = Vz = 0
    print("Velocities Reset")
    print(Vx, "    ", Vy, "    ", Vz)
    velocity_control(vehicle, Vx, Vy, Vz)
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    elif key == keyboard.Key.space :
        print("RTL Engaged")
        print('Returning to Launch site')
        vehicle.mode = VehicleMode("RTL")
        time.sleep(20)

vehicle = Establish_Connection_to_Drone()
# Collect events until released
with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:listener.join()
# ...or, in a non-blocking fashion:
listener = keyboard.Listener(on_press=on_press,on_release=on_release)
listener.start()
