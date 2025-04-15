import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep
import sys, math
import pickle

factory = PiGPIOFactory() # Set timing factory for hardware timing

# Video Feed Size
Frame_Height = int(3280)
Frame_Width = int(2464)

# Markers
MARKER_SIZE = 2  # 2 cm marker size
main_marker = 0; # Main ArUco Marker id
arm_marker = 1; # Arm ArUco Marker id


#----------------------------------------------
# LIMITS OF ARM
#----------------------------------------------
arm_limit = [50,0,0] # SHOULDER_ANGLE, ELBOW_ANGLE, ACTUATOR_LENGTH

optimal_position = [0,0,0] # X, Y, Z

#---------------------------------------------------------------------------
# CAMERA FUNCTIONS
#---------------------------------------------------------------------------

def positioning(tvec):
    #-- Desired position 
    desr_pos = [0,4,0]

    # Calculate the differences
    dx = (tvec[0] - desr_pos[0]).item()
    dy = (tvec[1] - desr_pos[1]).item()
    dz = (tvec[2] - desr_pos[2]).item()

    print(f"Z Position: {dz} mm")
    return ([dx,dy,dz])
 
# Still to be finished
def orientating(rvec):
    desr_ori = [180,0,0]
     
def relativePositions():
    # Calculate the relative positions of the markers
    tvec = tvec_0 - tvec_1;
    rvec = rvec_0 - rvec_1;
    
    #-- Obtain the rotation matrix tag->camera
    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
    R_tc = R_ct.T

    #-- Get the attitude in terms of the euler 321 (needs to be flipped first)
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
    
    x,y,z = positioning(tvec)
    return ([x,y,z])
    
    
#-----------------------------------------------
# This code us used to detect one marker and tell the camera the direction to move
#-----------------------------------------------

ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
}

#-----------------------------------------------
#----------- ROTATIONS
#-----------------------------------------------
# Checks if a matrix is a valid rotation matrix
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The results is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x,y,z])

# Camera calibration parameters 
with open('calibration.pkl', 'rb') as f:
    camera_matrix, distortion_coeffs = pickle.load(f)

#--- 180 deg rotation matrix around he x axis
R_flip = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

# Set the type of ArUco dictionary being used
aruco_type = "DICT_4X4_100"
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters()

# ---------------------------
# Start Video Stream
# ---------------------------
print("[INFO] starting video stream...")
cap = Picamera2()
cap.preview_configuration.main.size=(Frame_Height,Frame_Width)
cap.preview_configuration.main.format = 'RGB888'
cap.start()

#--------------------------------------------------------------------------------
# ELECTRONIC SETUP
#--------------------------------------------------------------------------------

# Linear Actuator Setup
GPIO.setmode(GPIO.BCM)

# Linear Actuator
# Pin Definitions
enable_pin = 18  # PWM pin to control speed
input_pin1 = 17  # Direction pin 1
input_pin2 = 12  # Direction pin 2

# Setup
GPIO.setup(enable_pin, GPIO.OUT)
GPIO.setup(input_pin1, GPIO.OUT)
GPIO.setup(input_pin2, GPIO.OUT)

# Create PWM instance
pwm = GPIO.PWM(enable_pin, 1000)  # PWM on enable_pin at 1kHz frequency
pwm.start(0)  # Start PWM with 0% duty cycle

# Servo Setup
ShoulderServo = Servo(27, min_pulse_width=0.1/1000, max_pulse_width=2.5/1000, pin_factory=factory)
ElbowServo = Servo(22, min_pulse_width=0.1/1000, max_pulse_width=2.4/1000, pin_factory=factory)

# Servo Start Angles
CurrentShoulderAngle = 0
CurrentElbowAngle = 0

maxAngle = 135 # Max Servo Angle either way

# Arm sizes
FOREARM_LENGTH = 185
UPPERARM_LENGTH = 105

# -------------------------------------------
# Movement Functions
# -------------------------------------------

#Servo
def ChangeShoulderAngle(InputAngle):
    ElbowServo.value = InputAngle/maxAngle
    ShoulderServo.value = -InputAngle/maxAngle
    
    print(f"Final Elbow Angle: {CurrentElbowAngle}")
    print(f"Final Shoulder Angle: {CurrentShoulderAngle}")
     
#Linear Actuator
def extend_actuator():
    GPIO.output(input_pin1, GPIO.HIGH)
    GPIO.output(input_pin2, GPIO.LOW)
    pwm.ChangeDutyCycle(100)  # Full speed (100% duty cycle)

def retract_actuator():
    GPIO.output(input_pin1, GPIO.LOW)
    GPIO.output(input_pin2, GPIO.HIGH)
    pwm.ChangeDutyCycle(100)  # Full speed (100% duty cycle)

def stop_actuator():
    GPIO.output(input_pin1, GPIO.LOW)
    GPIO.output(input_pin2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)  # Stop the motor (0% duty cycle)
    
#---------------------------------------------
# INITIALISE POITIONS
#---------------------------------------------

ShoulderServo.mid()
ElbowServo.mid()

retract_actuator()
sleep(10)

stop_actuator()

#----------------------------------
# OPENING CAMERA VISUALISATION
#----------------------------------

while True:

    img = cap.capture_array()
    # Build in some sort of feedback if capture doesn't start
  
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Would it be good to have a recording of what happened?
  
    # Detect ArUco markers  
    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict)

    # Print the position (corner coordinates) and ID of each detected marker
    if ids is not None:
        if ids.size < 2:
            print("Missing Marker")
        else:
            tvec_0 = []
            tvec_1 = []
            rvec_0 = []
            rvec_1 = []

            for i, corner in enumerate(corners):
                if ids[i] == main_marker:
                    # Estimate the pose of the marker
                    rvec_0, tvec_0, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, camera_matrix, distortion_coeffs)
                    
                    # Change layout of tvec and rvec for ease of use
                    tvec_0 = tvec_0[0][0]
                    rvec_0 = rvec_0[0][0]
                    
                    # Draw detected markers on the frame
                    cv2.aruco.drawDetectedMarkers(img, (corner.astype(np.float32),), None , (0,255,255))
                    
                if ids[i] == arm_marker:
                    # Estimate the pose of the marker
                    rvec_1, tvec_1, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, camera_matrix, distortion_coeffs)
                    
                    # Change layout of tvec and rvec for ease of use
                    tvec_1 = tvec_1[0][0]
                    rvec_1 = rvec_1[0][0]
                
                if len(tvec_1) > 0 and len(tvec_0) > 0:
                    X,Y,Z = relativePositions()
                    try:
                        ANGLE_CHANGE = math.degrees(math.asin(Z/UPPERARM_LENGTH))
                    except ValueError:
                        print("Invalid angle calculation due to Z/UPPERARM_LENGTH exceeding bounds.")
                        continue
                    
                    # --------------------------------------
                    # CONTROL FROM CAMERA INPUT
                    # --------------------------------------
                    
                    MAX_ANGLE = arm_limit[0]
                    CurrentShoulderAngle = CurrentShoulderAngle+ANGLE_CHANGE
                    if abs(CurrentShoulderAngle) > MAX_ANGLE:
                        CurrentShoulderAngle = CurrentShoulderAngle-ANGLE_CHANGE
                        print(f"Angle {CurrentShoulderAngle:.2f} exceeds the limit of {MAX_ANGLE} degrees. Skipping movement.")
                        continue
                    
                    ChangeShoulderAngle(CurrentShoulderAngle)

                    # # InputDistance = int(input("Enter Distance: "))
                    # # Extend the actuator
                    # extend_actuator()
                    # sleep(5)  # Extend for 2 seconds
                    
                    # # Stop the actuator
                    # stop_actuator()
                    # sleep(1)  # Pause for 1 second

                    # # Retract the actuator
                    # retract_actuator()
                    # sleep(2)  # Retract for 2 seconds
                    
                    # # Stop the actuator
                    # stop_actuator()

    # Display the output frame
    cv2.imshow('Camera', img)

    # Break the loop if 'q' is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        ChangeShoulderAngle(0)
        break
  
# Release video and close all windows
cv2.destroyAllWindows()
print("Exiting program.")
pwm.stop()
GPIO.cleanup()
