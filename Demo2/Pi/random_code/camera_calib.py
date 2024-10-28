import numpy as np
import cv2 as cv
import glob

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

# Load images
images = glob.glob('*.jpg')

for fname in images:
    img = cv.imread(fname)
    if img is None:
        print(f"Image {fname} not found or could not be opened.")
        continue

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

    # If found, add object points and image points
    if ret:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7, 6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

# Only proceed if we have enough points for calibration
if objpoints and imgpoints:
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Error in projection: \n", ret)
    print("\nCamera matrix : \n", mtx)
    print("\nDistortion coefficients: \n", dist)
    print("\nRotation vector: \n", rvecs)
    print("\nTranslation vector: \n", tvecs)
    print(len(images))
else:
    print("Not enough points for calibration.")
