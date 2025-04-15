from pynput import keyboard
import gpiozero, cv2, threading, pickle, math, queue, time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import numpy as np
from picamera2 import Picamera2
# from pymavlink import mavutil

factory = PiGPIOFactory()

num = 0

#-----------------------------------------------------------
# PIXHAWK CONNECTION
#-----------------------------------------------------------

# Connect to Pixhawk
# master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# # Wait for heartbeat
# master.wait_heartbeat()
# print("Connected to Pixhawk")

#-----------------------------------------------------------
# CAMERA PARAMETERS
#-----------------------------------------------------------
# Camera intrinsic parameters
with open('calibration.pkl', 'rb') as f:
    CAMERA_MATRIX, DIST_COEFFS = pickle.load(f)

MARKER_SIZE = 2  # 2 cm 

REQUIRED_COORDS = [2, 36, 54] #mm
REQUIRED_CON_COORDS = [2, 36, 54] #mm
Error = 5 #mm

# Track How Long the Position is Stable
position_held_time = None
REQUIRED_HOLD_TIME = 5 #s
EXTENSION_TIME = 10 #s

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
ShoulderServo = Servo(ShoulderServo_Pin, min_pulse_width=0.4/1000, max_pulse_width=2.4/1000, pin_factory=factory)
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
def shoulder_angle(Angle):
    ShoulderServo.value =  (Angle+65.2485+90)/106.595

def elbow_angle(Angle):
    ElbowServo.value = (Angle+67.7555+90)/120.615

#-----------------------------------------------------------
# ELECTROMAGNET FUNCTIONS
#-----------------------------------------------------------
def ETH_ON():
    EM_H_1.on()
    EM_H_2.on()
    EM_H_3.on()

def ETH_OFF():
    EM_H_1.off()
    EM_H_2.off()
    EM_H_3.off()

def ETR_ON():
    EM_R.on()

def ETR_OFF():
    EM_R.off()
    
#-----------------------------------------------------------
# FAILSAFE
#-----------------------------------------------------------  
# Return components to original positions
def failsafe():
    retract_actuator()
    shoulder_angle(0)
    elbow_angle(0)
    ETH_OFF()
    ETR_OFF()
#-----------------------------------------------------------
# DRONE CONTROL
#----------------------------------------------------------- 
  
# def send_velocity_command(vx, vy, vz):
    
#     # Sends velocity commands to the Pixhawk
#     # vx: forward/backward velocity (m/s) (+ = forward, - = backward)
#     # vy: left/right velocity (m/s) (+ = right, - = left)
#     # vz: up/down velocity (m/s) (+ = down, - = up)

#     master.mav.set_position_target_local_ned_send(
#         0,  # time_boot_ms (ignored)
#         0, 0,  # target system, target component
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Coordinate frame
#         0b0000111111000111,  # Type mask (only velocities enabled)
#         0, 0, 0,  # x, y, z positions (not used)
#         vx, vy, vz,  # x, y, z velocity (set these!)
#         0, 0, 0,  # x, y, z acceleration (not used)
#         0, 0  # yaw, yaw rate (not used)
#     )


#-----------------------------------------------------------
# CHECK ALIGNMENT
#-----------------------------------------------------------
def check_alignment(position):
    global position_held_time
    if all(abs(p - r) <= Error for p, r in zip(position, REQUIRED_CON_COORDS)):
        ETR_OFF()
    else:
        ETH_OFF()
        position_held_time = None
    
#-----------------------------------------------------------
# ARM MOVEMENTS
#-----------------------------------------------------------
def arm_control(position):
    global position_held_time
    print("[Check] Arm in Correct Position")
    if all(abs(p - r) <= Error for p, r in zip(position, REQUIRED_COORDS)):
        print("[Message] Arm in Correct Position")
        if position_held_time is None:
            position_held_time = time.time()
            
        elapsed_time = time.time() - position_held_time
        print(f"Position stable for: {elapsed_time:.2f} seconds")
        
        if elapsed_time >= REQUIRED_HOLD_TIME + EXTENSION_TIME:
            stop_actuator()
            ETH_ON()
            check_alignment(position)
        elif elapsed_time >= REQUIRED_HOLD_TIME:
            extend_actuator()
            
    else:
        print("[Message] Arm movement Required")
        
        # movement = [REQUIRED_COORDS[i] - position[i] for i in range(3)]

        # # Convert movement difference to velocity commands
        # vx = 0.5 if movement[0] > 0 else -0.5 if movement[0] < 0 else 0
        # vy = 0.5 if movement[1] > 0 else -0.5 if movement[1] < 0 else 0
        # vz = -0.5 if movement[2] > 0 else 0.5 if movement[2] < 0 else 0  # Negative for up

        # print(f"Moving drone: vx={vx}, vy={vy}, vz={vz}")
        # # send_velocity_command(vx, vy, vz)
        
        try:
            angle_change = math.degrees(math.asin((position[1]-REQUIRED_COORDS[1])/UPPERARM_LENGTH))
        except ValueError:
            print("Invalid angle calculation due to Z/UPPERARM_LENGTH exceeding bounds.")
            angle_change = 0
        
        # if position_held_time > REQUIRED_HOLD_TIME:
        #     extend_actuator()
        
        MAX_ANGLE = ARM_LIMIT[0]
        CurrentShoulderAngle = CurrentShoulderAngle+angle_change
        shoulder_angle(CurrentShoulderAngle)
        print("[Message] Moving Servos")
        if abs(CurrentShoulderAngle) > MAX_ANGLE:
            CurrentShoulderAngle = CurrentShoulderAngle-angle_change
            shoulder_angle(CurrentShoulderAngle)
            print(f"Angle {CurrentShoulderAngle:.2f} exceeds the limit of {MAX_ANGLE} degrees. Skipping movement.")
            
        position_held_time = None
        retract_actuator()
        
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

#-----------------------------------------------------------
# CAMERA FUNCTIONS
#-----------------------------------------------------------
# Thread-safe queue for sharing images between threads
image_queue = queue.Queue()

def CameraFrameDetection():
    global num
    while True:
        # Detect frame input from camera
        img = cap.capture_array()
        if img is None:
            print("[ERROR] Can't Start Video Feed")
            return
        else:
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) # Convert Picamera2 image (NumPy array) to OpenCV format
            image_queue.put(img)  # Put frame into the queue

            # cv2.imwrite('img' + str(num) + '.png', img)
            # print("image saved!")
            # num += 1
            # print("[Message] Frame Added") # --------------------------------------------------------------------DEBUG
        # sleep(0.1)  # Simulate camera frame rate (10 FPS)
        
def ProcessFrame(frame):
    if frame is None:
        print("[Error] No Frame detected")
        return None
    
    # Detect ArUco markers in single frame
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict)
    
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("Detection", frame)
    cv2.waitKey(1)
    
    if ids is not None and len(ids) >= 2:
        marker_positions = {}
        
        # Store detected marker positions
        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS
            )
            marker_positions[marker_id] = tvec[0][0] # Extract (X,Y,Z)
            
        # Ensure both marker 0 and marker 1 are detected
        if 0 in marker_positions and 1 in marker_positions:
            # Compute relative position of Marker 1 from Marker 0
            relative_position = marker_positions[1] - marker_positions[0]
            return relative_position  # Return calculated distance

    print("[Error] No Markers detected")
    return None  # If markers are not found

# def CameraProcessing():
#     global num
#     while True:
#         print(image_queue.qsize())
#         if image_queue.qsize() > 5:
#             print("[Message] Analysising Frame")
#             frames = [image_queue.get() for _ in range(5)]  # Store the last 5 frames
#             positions = []

#             for frame in frames:
#                 # cv2.imwrite('img' + str(num) + '.png', frame)
#                 # print("image saved!")
#                 # num += 1
#                 position = ProcessFrame(frame)
#                 if position is not None:
#                     positions.append(position)

#             if positions:
#                 avg_position = np.mean(positions, axis=0)
#                 print(f"Average Distance (Marker 1 from Marker 0): {avg_position:.2f}")
#                 arm_control(avg_position)
#         else:
#             print("[Warning] Queue is less than 5 Frames")
            
def CameraProcessing():
    global num
    while True:
        qsize = image_queue.qsize()
        print(f"[Queue Size] {qsize}")
        if qsize > 5:
            # Flush the queue and keep only the latest 5 frames
            frames = []
            while not image_queue.empty():
                frame = image_queue.get()
                frames.append(frame)

            # Take the last 5 frames only
            frames = frames[-5:]

            print("[Message] Analyzing Last 5 Frames")
            positions = []

            for frame in frames:
                position = ProcessFrame(frame)
                if position is not None:
                    positions.append(position)

            if positions:
                avg_position = np.mean(positions, axis=0)
                print(f"Average Distance (Marker 1 from Marker 0): {avg_position}")
                arm_control(avg_position)
        else:
            print("[Warning] Not enough frames in queue")

        
    
# Create and start threads
camera_t = threading.Thread(target=CameraFrameDetection, daemon=True)
processing_t = threading.Thread(target=CameraProcessing, daemon=True)

ShoulderServo.mid()
print("[Message] Initialising Shoulder Servo")
ElbowServo.mid()
print("[Message] Initialising Elbow Servo")

camera_t.start()
processing_t.start()

# Would it be good to have a recording of what happened?
try: 
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Stopping system...")
    failsafe()
