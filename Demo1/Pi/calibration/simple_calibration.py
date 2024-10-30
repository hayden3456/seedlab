import cv2
import numpy as np

def main():
    # Initialize the video capture
    cap = cv2.VideoCapture(0)

    captured_images = []
    print("Press 's' to capture images for calibration. Press 'x' when done.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow('Calibration', frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            captured_images.append(frame.copy())
            print(f"Captured image {len(captured_images)}")
        elif key == ord('x'):
            print("Finished capturing images.")
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(captured_images) < 10:
        print("Not enough images were captured for calibration. Need at least 10 images.")
        return

    # Prepare object points
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    chessboard_size = (7, 6)  # Number of inner corners per a chessboard row and column
    objp = np.zeros((chessboard_size[1]*chessboard_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    objpoints = []  # 3D point in real-world space
    imgpoints = []  # 2D points in image plane.

    for image in captured_images:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            objpoints.append(objp)
            # Refine corner locations
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv2.drawChessboardCorners(image, chessboard_size, corners2, ret)
            cv2.imshow('Chessboard', image)
            cv2.waitKey(500)
        else:
            print("Chessboard corners not found in one of the images.")

    cv2.destroyAllWindows()

    # Perform camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    if ret:
        # Save the camera matrix and distortion coefficients to a YAML file
        save_calibration('camera_calibration.yaml', camera_matrix, dist_coeffs)
        print("Calibration complete. Camera matrix and distortion coefficients saved to 'camera_calibration.yml'.")
    else:
        print("Calibration failed. Please ensure that enough valid images were captured.")

def save_calibration(filename, camera_matrix, dist_coeffs):
    # Flatten the arrays and convert them to lists
    camera_matrix_flat = camera_matrix.flatten().tolist()
    dist_coeffs_flat = dist_coeffs.flatten().tolist()

    with open(filename, 'w') as f:
        f.write('%YAML:1.0\n')
        f.write('---\n')
        f.write('camera_matrix: !!opencv-matrix\n')
        f.write('   rows: 3\n')
        f.write('   cols: 3\n')
        f.write('   dt: f\n')
        f.write('   data: [ ' + ', '.join(f'{value:.6f}' for value in camera_matrix_flat) + ' ]\n')
        f.write('distortion_coefficients: !!opencv-matrix\n')
        f.write('   rows: 5\n')
        f.write('   cols: 1\n')
        f.write('   dt: f\n')
        f.write('   data: [ ' + ', '.join(f'{value:.6f}' for value in dist_coeffs_flat) + ' ]\n')

if __name__ == "__main__":
    main()
