def showPositions(rvec, tvec, id):
    # Draw the axis for each marker
    cv2.drawFrameAxes(img, camera_matrix, distortion_coeffs, rvec, tvec, MARKER_SIZE)

    #-- print tag position in camera frame
    str_position = "MARKER "+str(id)+" Position x=%4.1f y=%4.1f z=%4.1f"%(tvec[0]*10,-tvec[1]*10,tvec[2]*10)
    cv2.putText(img, str_position, (0,100), font,5, (0, 255,0), 2, cv2.LINE_AA)

    #-- Obtain the rotation matrix tag->camera
    # R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
    # R_tc = R_ct.T

    #-- Get the attitude in terms of the euler 321 (needs to be flipped first)
    # roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

    #-- Print the marker's attitude respect to the camera frame
    # str_attitude = "MARKER "+str(id)+" Attitude r=%4.0f p=%4.0f y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),math.degrees(yaw_marker))
    # cv2.putText(img, str_attitude, (0, 200), font, 5, (255,0,0), 5, cv2.LINE_AA)

def compute_distance(tvec1, tvec2):
    dx = (tvec2[0] - tvec1[0]) * 10  # Convert to mm
    dy = -(tvec2[1] - tvec1[1]) * 10  # Convert to mm
    dz = (tvec2[2] - tvec1[2]) * 10  # Convert to mm
    return dx, dy, dz

#-----------------------------------------------
# This code us used to detect one marker and tell the camera the direction to move
#-----------------------------------------------
from picamera2 import Picamera2
import numpy as np
import sys, time, math
import cv2
import pickle

time_limit = 10

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16H5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25H9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36H10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36H11
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

# Taking photos 
num = 0

# Font for the text
font = cv2.FONT_HERSHEY_PLAIN

# ArUco marker size in meters (adjust this to your marker size)
MARKER_SIZE = 2  # 2 cm marker size

# Camera calibration parameters (replace these with your calibration data)

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

print("[INFO] starting video stream...")
cap = Picamera2()
cap.preview_configuration.main.size=(3280,2464)
cap.preview_configuration.main.format = 'RGB888'
cap.start()

data_points = []
start_time = time.time()
pos_str = "Id1\t Id2\t dX (mm) \t dY (mm) \t dZ (mm)"
data_points.append(pos_str)
marker_positions = {}

while True:
    elapsed_time = time.time() - start_time
    if elapsed_time > time_limit:
        break # stop after 10 seconds
    
    img = cap.capture_array()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  
    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict)

    # Print the relative position (corner coordinates) and ID of each detected marker

    if ids is not None and len(ids) >=2:
        for i, corner in enumerate(corners):

            # Estimate the pose of the marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, camera_matrix, distortion_coeffs)
            
            # Change layout of tvec and rvec for ease of use
            tvec = tvec[0][0]
            rvec = rvec[0][0] # Only used for rotational matrix
            
            marker_positions[ids[i][0]] = tvec
            
        if len(marker_positions) >=2:
            id_list = list(marker_positions.keys())[:2] # Take first two markers if more are detected
            dx,dy,dz = compute_distance(marker_positions[id_list[0]],marker_positions[id_list[1]])
            
            distance_str = f"{id_list[0]}\t {id_list[1]}\t {dx:.1f}\t {dy:.1f}\t {dz:.1f}"
            data_points.append(distance_str)
            
            print(f"Distance between marker {id_list[0]} and {id_list[1]}: dX={dx:.1f}mm, dY={dy:.1f}mm, dZ={dz:.1f}mm")
            
            # Draw detected markers on the frame
            # cv2.aruco.drawDetectedMarkers(img, (corner.astype(np.float32),), None , (0,255,255))
            
            # showPositions(rvec, tvec, ids[0])

    # Display the output frame
    # cv2.imshow('Camera', img)
    
    # Break the loop if 'q' is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
  
# Release video and close all windows
cv2.destroyAllWindows()

with open("marker_positions.txt", "w") as file:
    file.write("\n".join(data_points))
    
print("[INFO] Video stream stopped and data saved to marker_positions.txt")



