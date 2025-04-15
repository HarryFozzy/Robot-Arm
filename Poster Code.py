from pynput import keyboard
import gpiozero, cv2, threading, pickle, math, queue, time, os
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import numpy as np
from picamera2 import Picamera2

factory = PiGPIOFactory()

#-----------------------------------------------------------
# CAMERA PARAMETERS
#-----------------------------------------------------------
# Camera intrinsic parameters
with open('calibration.pkl', 'rb') as f:
    CAMERA_MATRIX, DIST_COEFFS = pickle.load(f)

MARKER_SIZE = 0.02  # 2 cm 

REQUIRED_COORDS = [0, 10, 0] #mm
REQUIRED_CON_COORDS = [0, 10, 0] #mm
Error = 5 #mm

# Track How Long the Position is Stable
position_held_time = None
REQUIRED_HOLD_TIME = 5 #s
EXTENSION_TIME = 10 #s

# Font for the text
font = cv2.FONT_HERSHEY_PLAIN 

#-----------------------------------------------------------
# ARM PARAMETERS
#-----------------------------------------------------------
# Camera Frame Size
Frame_Height = int(3280/4)
Frame_Width = int(2464/4)

# Servo Start Angles
CurrentShoulderAngle = 0
CurrentElbowAngle = 0

# Max Servo Angle either way
maxAngle = 135

# Arm sizes
FOREARM_LENGTH = 185
UPPERARM_LENGTH = 105

ARM_LIMIT = [50,0,0] # SHOULDER_ANGLE, ELBOW_ANGLE, ACTUATOR_LENGTH

#-----------------------------------------------------------
# DEFINE PINS
#-----------------------------------------------------------

# Linear Actuator
LinearActuator_Enable_PIN = 18  # Enable Pin
LinearActuator_Input1_PIN = 17  # Input Pin 1
LinearActuator_Input2_PIN = 12  # Input Pin 2

# Servos
ShoulderServo_Pin = 27
ElbowServo_Pin = 22

# Electromagnets
EM_R_PIN = 23  # Energise to Release Pin
EM_H_PINS = [24, 25, 26]  # Energise to Hold Pins

#-----------------------------------------------------------
# PIN SETUP
#-----------------------------------------------------------
# Linear Actuator
LinearActuator_Enable = gpiozero.OutputDevice(LinearActuator_Enable_PIN, active_high=True, initial_value=False)
LinearActuator_Input1 = gpiozero.OutputDevice(LinearActuator_Input1_PIN, active_high=True, initial_value=False)
LinearActuator_Input2 = gpiozero.OutputDevice(LinearActuator_Input2_PIN, active_high=True, initial_value=False)

# Servos
ShoulderServo = Servo(ShoulderServo_Pin, min_pulse_width=0.6/1000, max_pulse_width=2.3/1000, pin_factory=factory)
ElbowServo = Servo(ElbowServo_Pin, min_pulse_width=0.6/1000, max_pulse_width=2.1/1000, pin_factory=factory)

# Electromagnets
EM_R = gpiozero.OutputDevice(EM_R_PIN, active_high=True, initial_value=True)
EM_H_1 = gpiozero.OutputDevice(EM_H_PINS[0], active_high=True, initial_value=False)
EM_H_2 = gpiozero.OutputDevice(EM_H_PINS[1], active_high=True, initial_value=False)
EM_H_3 = gpiozero.OutputDevice(EM_H_PINS[2], active_high=True, initial_value=False)

#-----------------------------------------------------------
# LINEAR ACTUATOR FUNCTIONS
#-----------------------------------------------------------
LinearActuator_Speed = 6  # mm/s - Movement speed with 12V supply

def extend_actuator():
    LinearActuator_Enable.on()
    LinearActuator_Input1.off()
    LinearActuator_Input2.on()

def retract_actuator():
    LinearActuator_Enable.on()
    LinearActuator_Input1.on()
    LinearActuator_Input2.off()

def stop_actuator():
    LinearActuator_Enable.on()
    LinearActuator_Input1.off()
    LinearActuator_Input2.off()
    
#-----------------------------------------------------------
# SERVO FUNCTIONS WITH THREADING
#-----------------------------------------------------------
maxAngle = 135  # Max Servo Angle either way

def shoulder_angle(Angle):
    PWM = (Angle+65.2485+90)/106.595
    ShoulderServo.value = PWM*2/1.7 - 1.71

def elbow_angle(Angle):
    PWM = (Angle+67.7555+90)/120.615
    ElbowServo.value = PWM*2/1.5 - 1.8
    
#-----------------------------------------------------------
# ArUco Setup
#-----------------------------------------------------------
# Marker Dictionary
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
}

# Set the type of ArUco dictionary being used
aruco_type = "DICT_4X4_100"
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters()

# ----------------------------------------------------------------
# START VIDEO STREAM
# ----------------------------------------------------------------
print("[INFO] starting video stream...")
cap = Picamera2()
cap.preview_configuration.main.size=(Frame_Height,Frame_Width)
cap.preview_configuration.main.format = 'RGB888'
cap.start()

# Camera Functions

def showPositions(img, rvec, tvec):
    # Draw the axis for each marker
    cv2.drawFrameAxes(img, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, MARKER_SIZE)

    #-- print tag position in camera frame
    str_position = "MARKER Position x=%4.1f y=%4.1f z=%4.1f"%(tvec[0]*10,-tvec[1]*10,tvec[2]*10)
    cv2.putText(img, str_position, (0,100), font,5, (0, 255,0), 2, cv2.LINE_AA)
    
def armRoutine():
    shoulder_angle(-25)
    elbow_angle(25)
    
    sleep(1)
    
    shoulder_angle(-50)
    elbow_angle(50)
    
    sleep(1)
    
    extend_actuator()
    sleep(5)
    stop_actuator
    sleep(1)
    retract_actuator()
    sleep(5)
    
    shoulder_angle(-25)
    elbow_angle(25)
    sleep(1)
    
    shoulder_angle(0)
    elbow_angle(0)
    sleep(1) 
    
arm_t = threading.Thread(target=armRoutine, daemon=True).start()

while True:
    img = cap.capture_array()
    
    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict)
    if ids is not None:
        
        for i, corner in enumerate(corners):

            # Estimate the pose of the marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
            
            # Change layout of tvec and rvec for ease of use
            tvec = tvec[0][0]
            rvec = rvec[0][0] # Only used for rotational matrix
            
            showPositions(img, rvec, tvec)
        
    # Display the output frame
    cv2.imshow('Camera', img)
    
    # Break the loop if 'q' is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
        
print("\n[INFO] Stopping system...")
    
