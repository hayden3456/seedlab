import cv2
import cv2.aruco as aruco
import numpy as np
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board

#Do we need to adjust the marker length for camera calibration (same marker length)
#Estimate if z < 0.1, then z = 0?
#need to setup LCD screen

markerLength = 0.05

input_video = cv2.VideoCapture(0) # Initialize the video capture

camera_matrix, dist_coeffs = read_camera_parameters("test.yml") # Read the calibration parameters

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) # Load the QR dict

i2c = board.I2C() #Initialize bus for LCD screens

#initialize lcd screen
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [100, 0, 0] #red  

def read_camera_parameters(filename):
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    fs.release()
    return camera_matrix, dist_coeffs

def rotation_vector_to_euler_angles(tvec):
    tvec_array = tvec.tolist()
    print(tvec_array)
    z = tvec_array[0][2] #z vector
    x = tvec_array[0][0] #x vector
    
    rot = np.arctan2(x,z) #convert rectangular coordinates to angle
    
    print(np.degrees(rot))
    
    return np.degrees(rot) # Convert from radians to degrees


while(True):
    ret,frame = camera.read() # Take an image
    sleep(.01) # wait for image to stabilize
    
    if not ret:
        break
    
    frame_copy = frame.copy()
    
    corners, ids, _ = aruco.detectMarkers(image, dictionary) # Detect  markers

    if ids is not None:
        aruco.drawDetectedMarkers(frame_copy, corners, ids) # Draw detected markers

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # Estimate position of marker

        for i in range(len(ids)):
            cv2.drawFrameAxes(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1) # Draw axis for each marker

            euler_angles = rotation_vector_to_euler_angles(tvecs[i])
            
            print(f"Marker ID {ids[i][0]}: Rotation = {euler_angles:.2f} degrees")

    # Display the result
    cv2.imshow("out", image_copy)

    # Exit on pressing the 'Esc' key
    key = cv2.waitKey(1)
    if key == 27:
        break

# Release the video capture and close windows
input_video.release()
cv2.destroyAllWindows()


