import numpy as np
import sys, time, math
import cv2
import pickle
import os
print(os.getcwd())

def positioning(tvec):
     #-- Desired position 
     desr_pos = ['0','4','0']
 
     # Define the possible directions
     directions = ["STABLE", "RIGHT", "LEFT"]
     directions2 = ["STABLE", "DOWN", "UP"]
     directions3 = ["STABLE", "BACKWARDS", "FORWARDS"]
 
     # Calculate the differences
     dx = (tvec[0] - desr_pos[0]).item()
     dy = (tvec[1] - desr_pos[1]).item()
     dz = (tvec[2] - desr_pos[2]).item()
 
     # Determine index using tolerance of 1 unit
     index = 0 if abs(dx) <= 1 else int(np.sign(dx))
     index2 = 0 if abs(dy) <= 1 else int(np.sign(dy))
     index3 = 0 if abs(dz) <= 1 else int(np.sign(dz))
 
     # Lookup the direction using the index
     text = directions[index]
     text2 = directions2[index2]
     text3 = directions3[index3]
 
     # Display the result
     cv2.putText(img, f"{text}, {text2}, {text3}", (50, 400), font, 2, (0, 0, 255), 2, cv2.LINE_AA)
     
def orientating(rvec):
     desr_ori = [180,0,0]
     
def showPositions(rvec, tvec, id):
        # Draw the axis for each marker
        cv2.drawFrameAxes(img, camera_matrix, distortion_coeffs, rvec, tvec, MARKER_SIZE)
 
         #-- print tag position in camera frame
        str_position = "MARKER "+str(id)+" Position x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
        cv2.putText(img, str_position, (0,100+id*25), font,1, (0, 255,0), 2, cv2.LINE_AA)

        #-- Obtain the rotation matrix tag->camera
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T

        #-- Get the attitude in terms of the euler 321 (needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #-- Print the marker's attitude respect to the camera frame
        str_attitude = "MARKER "+str(id)+" Attitude r=%4.0f p=%4.0f y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),math.degrees(yaw_marker))
        cv2.putText(img, str_attitude, (0, 150+id*25), font, 1, (255,0,0), 2, cv2.LINE_AA)

def relativePositions():
    # Calculate the relative positions of the markers
    tvec = tvec_0 - tvec_1;
    rvec = rvec_0 - rvec_1;
    
    #-- Obtain the rotation matrix tag->camera
    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
    R_tc = R_ct.T

    #-- Get the attitude in terms of the euler 321 (needs to be flipped first)
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

    #-- Print the marker's attitude respect to the camera frame
    str_attitude = "MARKER angle r=%4.0f p=%4.0f y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),math.degrees(yaw_marker))
    cv2.putText(img, str_attitude, (0, 75), font, 1, (255,255,0), 2, cv2.LINE_AA)
    
    #-- print tag position in camera frame
    str_position = "MARKER distances x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
    cv2.putText(img, str_position, (0, 50), font, 1, (255,255,0), 2, cv2.LINE_AA)
    
    positioning(tvec)
    
    
#-----------------------------------------------
# This code us used to detect one marker and tell the camera the direction to move
#-----------------------------------------------


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

# Main marker id
main_marker = 0;

# Arm Marker id
arm_marker = 1;

# Taking photos 
num = 0

# Font for the text
font = cv2.FONT_HERSHEY_PLAIN

# ArUco marker size in meters (adjust this to your marker size)
MARKER_SIZE = 2  # 2 cm marker size

# Camera calibration parameters (replace these with your calibration data)

with open('calibration.pkl', 'rb') as f:
    camera_matrix, distortion_coeffs = pickle.load(f)
    
# print(camera_matrix)
# print(distortion_coeffs)

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
cap = cv2.VideoCapture(0)

# Get the default frame width and height
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Might need to set the camera size

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))

while True:

  ret, img = cap.read()
  if not ret:
        print("[ERROR] Could not read frame.")
        break
  
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  # Write the frame to the output file
  out.write(img)
  
  # Detect ArUco markers
  detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
  corners, ids, rejected = detector.detectMarkers(img)
  

  # Print the position (corner coordinates) and ID of each detected marker
  if ids is not None:
      print("Marker Detected")
      print(len(ids))
      if len(ids) >= 2:
        
        print("Multiple markers detected.")
        marker_positions = {}
        # Store detected marker positions
        for i, marker_id in enumerate(ids.flatten()):
            print("[Message] Estimating Positions")
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], MARKER_SIZE, camera_matrix, distortion_coeffs
            )
            marker_positions[marker_id] = tvec[0][0] # Extract (X,Y,Z)
            
        # Ensure both marker 0 and marker 1 are detected
        if main_marker in marker_positions and arm_marker in marker_positions:
            print("[Check] Correct Markers Detected")
            # Compute relative position of Marker 1 from Marker 0
            relative_position = marker_positions[arm_marker] - marker_positions[main_marker]
            print(relative_position) # Return calculated distance
        else:
            print("[Error] Incorrect Markers Detected")
    # if len(ids) < 2:
    #     cv2.putText(img, "Marker Missing", (50, 50), font, 4, (0, 0, 255), 2, cv2.LINE_AA)
    # else:
    #   tvec_0 = []
    #   tvec_1 = []
    #   rvec_0 = []
    #   rvec_1 = []

    #   for i, corner in enumerate(corners):
    #     if ids[i] == main_marker:
    #         # Estimate the pose of the marker
    #         rvec_0, tvec_0, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, camera_matrix, distortion_coeffs)
            
    #         # Change layout of tvec and rvec for ease of use
    #         tvec_0 = tvec_0[0][0]
    #         rvec_0 = rvec_0[0][0]
            
    #         # Draw detected markers on the frame
    #         cv2.aruco.drawDetectedMarkers(img, (corner.astype(np.float32),), None , (0,255,255))
           
    #         showPositions(rvec_0, tvec_0, 0)
            
    #     if ids[i] == arm_marker:
    #         # Estimate the pose of the marker
    #         rvec_1, tvec_1, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, camera_matrix, distortion_coeffs)
            
    #         # Change layout of tvec and rvec for ease of use
    #         tvec_1 = tvec_1[0][0]
    #         rvec_1 = rvec_1[0][0]
            
    #         showPositions(rvec_1, tvec_1, 1)
        
    #     if len(tvec_1) > 0 and len(tvec_0) > 0:
    #         relativePositions()


  k = cv2.waitKey(5)
  if k == ord('s'): # wait for 's' key to save and exit
      cv2.imwrite('VideoImages/img' + str(num) + '.png', img)
      print("image saved!")
      num += 1

  # Display the output frame
  cv2.imshow('Camera', img)
  
  # Break the loop if 'q' is pressed
  key = cv2.waitKey(1) & 0xFF
  if key == ord("q"):
    break
  
# Release video and close all windows
cap.release()
out.release()
cv2.destroyAllWindows()



