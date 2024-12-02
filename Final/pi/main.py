import math
import cv2
import cv2.aruco as aruco
import numpy as np
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import threading
import queue
from smbus2 import SMBus
import RPi.GPIO as GPIO


#implement a search mode so the arduino is no continuously getting sent data

#Set GPIO to high
GPIO.setmode(GPIO.BCM)
GPIO.setup(7,GPIO.OUT)
GPIO.output(7,GPIO.LOW)
sleep(.2)
GPIO.output(7,GPIO.HIGH)

# Constants and initializations
markerLength = 0.0508  # in units of meters
camera = cv2.VideoCapture(0)  # Initialize the video capture
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)  # Load the QR dictionary
#i2c = board.I2C()  # Initialize I2C bus for LCD screen
ARD_ADDR = 8
ARD_START_CMD = 101
i2c_board = SMBus(1)  # Initialize SMBus library with I2C bus 1 for data transmission

# For color detector
lower_green = np.array([40, 170, 0])
upper_green = np.array([121, 255, 80])
lower_red = np.array([0, 210, 80])
upper_red = np.array([180, 255, 178])
parameters = cv2.aruco.DetectorParameters()

# Function to read camera calibration parameters
def read_camera_parameters(filename):
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    fs.release()
    return camera_matrix, dist_coeffs

def rotation_vector_to_euler_angles(tvec):
    tvec_array = tvec.tolist()
    z = tvec_array[0][2]
    x = -tvec_array[0][0]
    rot = np.arctan2(x, z)
    return np.degrees(rot)  # Convert radians to degrees

def get_y_axis_rotation(rvec):
    """
    Converts a rotation vector to the Y-axis rotation angle (pitch) in degrees.
    """
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # Calculate Euler angles from rotation matrix
    sy = math.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] +
                   rotation_matrix[1, 0] * rotation_matrix[1, 0])
    singular = sy < 1e-6

    if not singular:
        y = math.atan2(-rotation_matrix[2, 0], sy)
    else:
        y = math.atan2(-rotation_matrix[2, 0], sy)

    # Convert radians to degrees
    pitch = math.degrees(y)
    return pitch


# Function for arrow detect
def detect_arrow_direction(frame):
    # Convert frame to HSV color space
    imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for green and red arrows
    mask_green = cv2.inRange(imgHSV, lower_green, upper_green)
    mask_red = cv2.inRange(imgHSV, lower_red, upper_red)

    # Display the masks for debugging

    # Process red contours
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_red:
        area = cv2.contourArea(cnt)
        if area > 1000:  # Adjust the area threshold as needed
            # Approximate the contour to a polygon
            epsilon = 0.03 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            # Check if the approximated polygon has a reasonable number of vertices
            if 7 <= len(approx) <= 10:
                # Draw the contour on the frame
                
                # Since red arrows always point right
                return "right"

    # Process green contours
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_green:
        area = cv2.contourArea(cnt)
        if area > 1000:  # Adjust the area threshold as needed
            # Approximate the contour to a polygon
            epsilon = 0.03 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            # Check if the approximated polygon has a reasonable number of vertices
            if 7 <= len(approx) <= 10:
                # Draw the contour on the frame
                # Since green arrows always point left
                return "left"

    return "none"

# Function to write data to Arduino with exception handling and retries
def write_to_arduino(state, dist_whole, dist_dec, angle_whole, angle_dec, angle_sign, retries=3):
    global i2c_board
    data = [state, dist_whole, dist_dec, angle_whole, angle_dec, angle_sign]
    for attempt in range(retries):
        try:
            i2c_board.write_i2c_block_data(ARD_ADDR, 1, data)
            #sleep(0.01)
            #i2c_board.write_byte_data(ARD_ADDR, 1, state)
            #sleep(0.01)
            #i2c_board.write_byte_data(ARD_ADDR, 1, dist_whole)
            #sleep(0.01)
            #i2c_board.write_byte_data(ARD_ADDR, 1, dist_dec)
            #sleep(0.01)
            #i2c_board.write_byte_data(ARD_ADDR, 1, angle_whole)
            #sleep(0.01)
            #i2c_board.write_byte_data(ARD_ADDR, 1, angle_dec)
            sleep(0.03)
            return  # Success
        except OSError as e:
            print(f"Attempt {attempt+1}: Error writing to Arduino: {e}")
            GPIO.output(7,GPIO.LOW)
            sleep(.2)
            GPIO.output(7,GPIO.HIGH)
            sleep(2)
            # Re-initialize the SMBus
            try:
                i2c_board.close()
            except Exception as ex:
                print(f"Error closing i2c_board: {ex}")
            sleep(0.1)  # Wait before retrying
            try:
                i2c_board = SMBus(1)
            except Exception as ex:
                print(f"Error re-initializing i2c_board: {ex}")
        except Exception as e:
            print(f"Unexpected error: {e}")
    print("Failed to write to Arduino after retries")

def state0():  # Search by sending a 10-degree command
    print("search")
    angle = 25  # Angle for search
    write_to_arduino(1, 0, 0, angle, 0, 0)  # Command for angle adjustment

def state1(angle):  # Angle correction
    print("State1")
    angle_sign = 0

    if angle > 0:
        angle_sign = 0
    elif angle < 0:
        angle_sign = 1

    print("angle: ", angle)
    print("sign: ", angle_sign)
    angle_whole_num = abs(math.trunc(angle))
    angle_dec_num = abs(int(100 * (round((angle - int(angle)), 2))))

    write_to_arduino(1, 0, 0, angle_whole_num, angle_dec_num, angle_sign)
    
def state2(distance, angle):  # Angle correction
    
    print("State2")
    angle_sign = 0
    # Limits the rotation speed of the Arduino
    if angle > 0:
        angle_sign = 0
    elif angle < 0:
        angle_sign = 1
        
    angle = angle * 4 

    angle_whole_num = abs(math.trunc(angle))
    angle_dec_num = abs(int(100 * (round((angle - int(angle)), 2))))
    #print("angle",angle)
    
    distance = abs(distance - 0.8)
    if(distance > 1.5 ):
        distance = 1.5
    
    dist_whole_num = abs(math.trunc(distance))
    dist_dec_num = abs(int(100 * (round((distance - int(distance)), 2))))

    # Call function to write to Arduino
    write_to_arduino(1, dist_whole_num, dist_dec_num, angle_whole_num, angle_dec_num, angle_sign)


def state3(sleep_time): # Stop
    print("Stopping")
    write_to_arduino(2, 0, 0, 0, 0, 0)
    sleep(sleep_time)
    
def state4():
    ret, frame = camera.read()
    sleep(0.01) 
    direction = detect_arrow_direction(frame)

    if direction:
        print(f"Detected direction: {direction}")
        if direction == "left":
#             GPIO.output(7,GPIO.LOW)
#             sleep(.2)
#             GPIO.output(7,GPIO.HIGH)
#             sleep(2)
            write_to_arduino(1, 0, 0, 90, 0, 0)
            
        elif direction == "right":
#             GPIO.output(7,GPIO.LOW)
#             sleep(.2)
#             GPIO.output(7,GPIO.HIGH)
#             sleep(2)
            write_to_arduino(1, 0, 0, 90, 0, 1)
        else:
            return "stopped"

        sleep(2.5)
        return "turned"
     
    else:
        write_to_arduino(2, 0, 0, 0, 0, 0)
        return "stopped"
        

# Load camera parameters
camera_matrix, dist_coeffs = read_camera_parameters("test.yaml")

state = 0
marker = False

while True:

    ret, frame = camera.read()  # Capture image 
    sleep(0.001)
    if not ret:
        print("error")
        break
    
    corners, ids, _ = aruco.detectMarkers(frame, dictionary)
    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        
        # Initialize variables to track the closest marker
        min_distance = float('inf')
        closest_index = -1
    
        # Iterate over all detected markers to find the closest one
        for i in range(len(ids)):
            z = tvecs[i][0][2]
            distance = (z - 0.0117) / 0.3161
            if distance < min_distance:
                min_distance = distance
                closest_index = i
    
        # Use the closest marker's data
        angle = rotation_vector_to_euler_angles(tvecs[closest_index])
        y_angle = get_y_axis_rotation(rvecs[closest_index])
        z = tvecs[closest_index][0][2]
        distance = (z - 0.0117) / 0.3161
        marker = True
    else:
        marker = False

#__________________

    if (state == 0):
        if marker and distance <= 5.3 and distance > 1.5 and abs(y_angle) < 20:
            state = 1
        else:
            state0()

    if (state == 1):
        if not marker:
            state = 0

        elif (abs(angle) > 10):
            print("yangle",y_angle)
            state1(angle)

        else:
            state = 2
   
    if (state == 2):
        if not marker:
            state = 0

        elif ((abs(angle)<10) & (distance >= 1)):
            state2(distance,angle)

        else:
            state = 3
    
    if (state == 3):
        #state3(0)
        state4response = state4()

        if state4response == "turned":
            print("going to state 0")
            state = 0

        if state4response == "stopped":
            print("No arrow detected, stopping run")
            sleep(200)

    if cv2.waitKey(1) == ord('q'):  # Exit on 'q' key press
        break

# Release 
camera.release()
cv2.destroyAllWindows()

