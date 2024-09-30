import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import threading

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
camera = cv2.VideoCapture(0) # Initialize the camera

# Initialize SMBus library with I2C bus 1
i2c = board.I2C()

#initialize lcd screen
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [100, 0, 0] #red

y_start_point = (318,0)
y_end_point = (318,600)
x_start_point = (0, 240)
x_end_point = (650, 240)

output = [0,0]

color = (0,255, 0)
thickness = 5
center = (0,0)

while(True):
    ret,frame = camera.read() # Take an image
    sleep(.1) # wait for image to stabilize
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for
    #ArUco detection    
    
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)#check for marker

  #  overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2BGR) # Convert back to RGB for
    #imshow, as well as for the next step
    #overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
    
    if not ids is None: #if ids has data, in otherwords QR code is detected
        ids = ids.flatten()
        #for (outline, id) in zip(corners, ids):
            #markerCorners = outline.reshape((4,2)) #reshape array in 4x2 2D array (corner, [x coordinates,  y coordinates])
            #overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)
        #cv2.imshow("overlay2",overlay)#show QR code idborderColor

        #x_avg = (corners[0][0][0][0] + '''corners[0][0][1][0] + '''corners[0][0][2][0] '''+ corners[0][0][3][0]''') / 2 #avg x coordinates of corners
        #y_avg = (corners[0][0][0][1] + '''corners[0][0][1][1] + '''corners[0][0][2][1] '''+ corners[0][0][3][1]''') / 2 #avg y coordinates of corners
        x_avg = (corners[0][0][0][0] + corners[0][0][2][0]) / 2 #avg x coordinates of corners
        y_avg = (corners[0][0][0][1] + corners[0][0][2][1]) / 2 #avg y coordinates of corners

        center = (int(x_avg),int(y_avg))
        
        frame = cv2.circle(frame,center,10,color,thickness)

        if ((x_avg<318)&(y_avg<240)):
            output[0] = 0
            output[1] = 1
        if (x_avg>318)&(y_avg<240):
            output[0] = 0
            output[1] = 0
        if (x_avg>318)&(y_avg>240):
            output[0] = 1
            output[1] = 0
        if (x_avg<318)&(y_avg>240):
            output[0] = 1
            output[1] = 1
    
        lcd.message = "Desired Location:\n" + str(output) #print QR code id on LCD screen
        
    frame = cv2.line(frame, y_start_point, y_end_point, color, thickness)
    frame = cv2.line(frame, x_start_point, x_end_point, color, thickness)
    cv2.imshow("frame",frame) #show video camera
    
    if cv2.waitKey(1) == ord('q'): #exit when user hits q
        break

camera.release()
cv2.destroyAllWindows() #clear camera if user hits q
