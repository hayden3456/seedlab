import numpy as np
import cv2 as cv
from cv2 import aruco
from time import sleep

# Load the image
img = cv.imread('Colors.jpg', 1)

# Check if the image was loaded successfully
if img is None:
    print("Error: Image could not be loaded. Please check the file path.")

# Load predefined aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# Open camera
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Capture frame
ret, frame = cap.read()
sleep(0.25)

# If frame is read correctly, ret is True
if not ret:
    print("Can't receive frame (stream end?). Exiting ...")

# Create initial mask isolating green
imgHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
upperred = np.array([90, 255, 255])
lowerred = np.array([30, 40, 10])
mask = cv.inRange(imgHSV, lowerred, upperred)
result = cv.bitwise_and(frame, frame, mask=mask)
cv.imshow("Result", frame)

# Perform erosion and dilation on the image
kernel = np.ones((5, 5), np.uint8)
erosion = cv.erode(mask, kernel, iterations=1)
cv.imshow("Erosion", erosion)
dilation = cv.dilate(mask, kernel, iterations=1)
cv.imshow("Dilation", dilation)

# Visualize contours of green areas
contour_green_visualize = frame.copy()
contours_green, _ = cv.findContours(erosion, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
cv.drawContours(contour_green_visualize, contours_green, -1, (255, 0, 0), 3)

Box = frame.copy()
Centers = np.empty((0, 2))
Areas = np.empty((0))

# Find concentrated pockets of green and apply a green box around them
for index, cnt in enumerate(contours_green):
    contour_area = cv.contourArea(cnt)
    if contour_area > 300:
        x, y, w, h = cv.boundingRect(cnt)
        center = int(x + w / 2), int(y + h / 2)
        Areas = np.append(Areas, contour_area)
        Centers = np.vstack((Centers, center))
        cv.rectangle(Box, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv.putText(Box, 'green', (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv.putText(Box, '+', center, cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

cv.imshow("Where's the Green?", Box)

# Exit code
while True:
    if cv.waitKey(1) == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv.destroyAllWindows()
