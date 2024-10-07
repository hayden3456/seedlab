import cv2
import cv2.aruco as aruco
import numpy as np

def read_camera_parameters(filename):
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    fs.release()
    return camera_matrix, dist_coeffs

def rotation_vector_to_euler_angles(rvec):
    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    
    # Calculate Euler angles (in radians)
    sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
    x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    y = np.arctan2(-rotation_matrix[2, 0], sy)
    z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    
    # Convert from radians to degrees
    return np.degrees([x, y, z])

# Initialize the video capture
input_video = cv2.VideoCapture(0)

# Read the camera parameters from the file
camera_matrix, dist_coeffs = read_camera_parameters("generic_camera_params.yml")

# Load the predefined dictionary for marker detection
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

while input_video.isOpened():
    ret, image = input_video.read()
    if not ret:
        break
    
    image_copy = image.copy()

    # Detect the markers in the image
    corners, ids, _ = aruco.detectMarkers(image, dictionary)

    # If at least one marker is detected
    if ids is not None:
        # Draw the detected markers on the image
        aruco.drawDetectedMarkers(image_copy, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

        # Draw axis for each marker and print Y-axis rotation (yaw)
        for i in range(len(ids)):
            cv2.drawFrameAxes(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
            
            # Convert rvec to Euler angles and extract Y-axis rotation (yaw)
            euler_angles = rotation_vector_to_euler_angles(rvecs[i])
            yaw = euler_angles[1]  # Y-axis rotation (yaw)
            
            # Print the yaw (Y-axis rotation) in degrees
            print(f"Marker ID {ids[i][0]}: Yaw (Y-axis rotation) = {yaw:.2f} degrees")

    # Display the result
    cv2.imshow("out", image_copy)

    # Exit on pressing the 'Esc' key
    key = cv2.waitKey(1)
    if key == 27:
        break

# Release the video capture and close windows
input_video.release()
cv2.destroyAllWindows()
