import cv2
import cv2.aruco as aruco
import numpy as np
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import threading
import queue


def read_camera_parameters(filename): 
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    fs.release()
    return camera_matrix, dist_coeffs

def rotation_vector_to_euler_angles(tvec):
    tvec_array = tvec.tolist()
    z = tvec_array[0][2] #z vector
    x = -tvec_array[0][0] #x vector
    
    rot = np.arctan2(x,z) #convert rectangular coordinates to angle
    
    return np.degrees(rot) # Convert from radians to degrees

markerLength = 0.0508 #in units of meters

camera = cv2.VideoCapture(0) # Initialize the video capture

camera_matrix, dist_coeffs = read_camera_parameters("test.yaml") # Read the calibration parameters

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) # Load the QR dict

i2c = board.I2C() #Initialize bus for LCD screens

q = queue.Queue()

#initialize lcd screen
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [100, 0, 0] #red  

def LCDHandler():
    while True:
        if not q.empty(): #if there is something in the queue, read it
            angle = q.get()
            
            lcd.message = f"{euler_angles:.2f}" #print QR code id on LCD screen
            sleep(1)

myThread = threading.Thread(target=LCDHandler, args=())
myThread.start() #start threading

while(True):
    ret,frame = camera.read() # Take an image
    sleep(.001) # wait for image to stabilize
    
    if not ret:
        break
    
    frame_copy = frame.copy()
    
    corners, ids, _ = aruco.detectMarkers(frame, dictionary) # Detect  markers

    if ids is not None:
        aruco.drawDetectedMarkers(frame_copy, corners, ids) # Draw detected markers

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # Estimate position of marker

        for i in range(len(ids)):
            cv2.drawFrameAxes(frame_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1) # Draw axis for each marker

            euler_angles = rotation_vector_to_euler_angles(tvecs[i])
            q.put(euler_angles)
            

    # Display the result
    cv2.imshow("out", frame_copy)

    if cv2.waitKey(1) == ord('q'): #exit when user hits q
        break

# Release the video capture and close windows
camera.release()
cv2.destroyAllWindows()
lcd.clear()

