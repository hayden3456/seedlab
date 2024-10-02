import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import threading
import queue
from smbus2 import SMBus

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
camera = cv2.VideoCapture(0) # Initialize tuino, set in Arduino sketch
ARD_ADDR = 8

i2c = board.I2C() #Initialize bus for LCD screens
i2c_board = SMBus(1)  #Initialize SMBus library with I2C bus 1 for data transmission

q = queue.Queue()

#initialize lcd screen
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [100, 0, 0] #red   

#initializing quadrants displayed
y_start_point = (318,0)
y_end_point = (318,600)
x_start_point = (0, 240)
x_end_point = (650, 240)

output = [2,2] #initiliaze coordinate array
coordinates = 0
temp = [0,0]
#color and thickness for coordinate lines
color = (0,255, 0)
thickness = 5

center = (0,0) #initialize array to store location of center of QR code

def LCDHandler():
    
    while True:
        if not q.empty(): #if there is something in the queue, read it
            gotSomething = q.get()
            print(gotSomething)
            
            lcd.message = "Desired Location:\n" + str(gotSomething) #print QR code id on LCD screen
            #Send data to the arduino
            #i2c_board.write_byte_data(ARD_ADDR,1, 100)#address,register, value,boolean
            i2c_board.write_byte_data(ARD_ADDR,1, gotSomething[0])#address,register, value,boolean
            sleep(0.1)
            i2c_board.write_byte_data(ARD_ADDR,1, gotSomething[1])
            sleep(.01)

myThread = threading.Thread(target=LCDHandler, args=())
myThread.start() #always run thread in the background

while(True):
    ret,frame = camera.read() # Take an image
    sleep(.01) # wait for image to stabilize
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for
    #ArUco detection    
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)#check for marker
    
    if not ids is None: #if ids has data, in otherwords QR code is detected
        x_avg = (corners[0][0][0][0] + corners[0][0][2][0]) / 2 #avg x coordinates of corners
        y_avg = (corners[0][0][0][1] + corners[0][0][2][1]) / 2 #avg y coordinates of corners
        
        temp[0] = output[0]
        temp[1] = output[1]
        
        if ((x_avg<318)&(y_avg<240)):
            output[0] = 0
            output[1] = 1
        if (x_avg>318)&(y_avg<240):
            coordinates = 1
            output[0] = 0
            output[1] = 0
        if (x_avg>318)&(y_avg>240):
            coordinates = 4
            output[0] = 1
            output[1] = 0
        if (x_avg<318)&(y_avg>240):
            coordinates = 3
            output[0] = 1
            output[1] = 1
        
        if temp != output: #if there has been a change to the location of the QR code put the coordinates on the queue
            q.put(output)
            
        center = (int(x_avg),int(y_avg)) #find the center of the QR code
        
        frame = cv2.circle(frame,center,10,(255,0,0),thickness) #add a small circle at the center of the QR code
        
    frame = cv2.line(frame, y_start_point, y_end_point, color, thickness)
    frame = cv2.line(frame, x_start_point, x_end_point, color, thickness)
    cv2.imshow("frame",frame) #show video camera
    
    if cv2.waitKey(1) == ord('q'): #exit when user hits q
        break

camera.release()
cv2.destroyAllWindows() #clear camera if user hits q
lcd.clear()
