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
'''
q = queue.Queue()
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [100, 0, 0]  # Set LCD color to red
'''
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
                print("Turn right")
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
                print("Turn left")
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
            #sleep(0.01)    
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
    print("dist",distance)

    # Call function to write to Arduino
    write_to_arduino(1, dist_whole_num, dist_dec_num, angle_whole_num, angle_dec_num, angle_sign)


def state3(sleep_time): # Stop
    print("Stopping")
    write_to_arduino(2, 0, 0, 0, 0, 0)
    sleep(sleep_time)
    
def state4():
    directions = []
    for _ in range(3):
        ret, frame = camera.read()
        if not ret:
            break
        direction = detect_arrow_direction(frame)
        if direction != "none":
            directions.append(direction)
        sleep(0.1)  # Small delay between frames

    # Determine the most frequent direction detected
    if directions:
        direction = max(set(directions), key=directions.count)
        print(f"Detected direction: {direction}")
        if direction == "left":
            print("Turning left")
            GPIO.output(7,GPIO.LOW)
            sleep(.2)
            GPIO.output(7,GPIO.HIGH)
            sleep(2)
            write_to_arduino(1, 0, 0, 90, 0, 0)
            
        elif direction == "right":
            print("Turning right")
            GPIO.output(7,GPIO.LOW)
            sleep(.2)
            GPIO.output(7,GPIO.HIGH)
            sleep(2)
            write_to_arduino(1, 0, 0, 90, 0, 1)
    else:
        print("No arrow detected, going random direction")
        write_to_arduino(1, 0, 0, 90, 0, 0)
        

# Define state dictionary for LCD display
state_dict = {
    state0: "search",
    state1: "angle_correct",
    state2: "dist_correct",
    state3: "stop"
}

# Load camera parameters
camera_matrix, dist_coeffs = read_camera_parameters("test.yaml")

angle_lock = False #flag when robot sees marker
marker_found = False

state2_flag = False

while True:
    ret, frame = camera.read()  # Capture image
    sleep(0.001)
    if not ret:
        break
    corners, ids, _ = aruco.detectMarkers(frame, dictionary)
    
    if ids is None:
        state0()
    else:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        angle = rotation_vector_to_euler_angles(tvecs[0])
        print("angle",angle)
        z = tvecs[0][0][2]
        distance = (z - 0.0117) / 0.3161
        if (abs(angle) >10 and state2_flag == 0):
            state1(angle)
        if (abs(angle)<10) & (distance >= 1):
            if(0 == state2_flag):
                sleep(3)
                state2_flag = True
                state3(0)
                
            state2(distance,angle)
        if (distance < 1):
            state3(1)
            state4()
            sleep(40)
            

    if cv2.waitKey(1) == ord('q'):  # Exit on 'q' key press
        break

# Release 
camera.release()
cv2.destroyAllWindows()
'''lcd.clear()'''