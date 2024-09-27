import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board

# Initialize the ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# Initialize the camera
camera = cv2.VideoCapture(0)

# Initialize I2C bus
i2c = board.I2C()

# Initialize the LCD screen
lcd_columns = 3
lcd_rows = 1
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [100, 0, 0]  # Set the LCD color to red

# Main loop to capture frames
while True:
    ret, frame = camera.read()  # Capture an image
    sleep(0.5)  # Wait for the image to stabilize
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert image to grayscale

    # Display the camera feed in grayscale
    cv2.imshow("frame", grey)

    # ArUco detection
    corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict)

    # Convert the image back to RGB for overlay
    overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)

    # Draw detected markers
    overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=4)

    # Check if any markers were detected
    if ids is not None:
        ids = ids.flatten()

        # Loop through detected markers and display IDs
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4, 2))
            overlay = cv2.putText(
                overlay,
                str(id),
                (int(markerCorners[0, 0]), int(markerCorners[0, 1]) - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2
            )

        # Show overlay with QR code IDs
        cv2.imshow("overlay2", overlay)

        # Display the detected ID on the LCD screen
        lcd.message = str(ids)
        sleep(0.5)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the camera and close windows
camera.release()
cv2.destroyAllWindows()
