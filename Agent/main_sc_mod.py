import numpy as np
import cv2
import cv2.aruco as aruco
import math
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# ------------- CAMERA SETTINGS --------------
marker_size = 100
with open('cam_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

cap = cv2.VideoCapture(0)
camera_width = 640
camera_height = 480
camera_frame_rate = 40
cap.set(2, camera_width)
cap.set(4, camera_height)
cap.set(5, camera_frame_rate)
# -------------------------------------------

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ion()  # Switch matplotlib to interactive mode

pose_history = []
x_data = []
y_data = []
z_data = []

window_size = 5  # The number of past points to consider
confidence_interval_multiplier = 2  # Adjust as needed


while True:
    ret, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, id, rejected = aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, camera_distortion)

    if id is not None and 2 in id:
        aruco.drawDetectedMarkers(frame, corners)

        rvec_all, tvec_all, _objPts = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix,
                                                                     camera_distortion)
        rvec = rvec_all[0][0]
        tvec = tvec_all[0][0]

        cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 100)

        rvec_flip = rvec * -1
        tvec_flip = tvec * -1
        rotation_mat, jacobian = cv2.Rodrigues(rvec_flip)
        real_tvec = np.dot(rotation_mat, tvec_flip)

        pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_mat)

        tvec_str = "x=%4.0f y=%4.0f z=%4.0f" % (real_tvec[0], real_tvec[0], real_tvec[2])
        cv2.putText(frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0xffff), 2, cv2.LINE_AA)
        cv2.circle(frame, (100, 100), 10, (255, 0, 0))

        # Add the new pose to the history
        pose = {
            'position': real_tvec.tolist(),
            'orientation': [math.degrees(angle) for angle in (pitch, roll, yaw)]
        }
        pose_history.append(pose)

        # Downsample the pose history
        if len(pose_history) > window_size:
            last_positions = np.array([pose['position'] for pose in pose_history[-window_size:]])
            mean_position = np.mean(last_positions, axis=0)

            pose['position'] = mean_position.tolist()

        # Convert the rotation matrix to a quaternion
        quaternion = R.from_matrix(rotation_mat).as_quat()
        pose['orientation'] = quaternion.tolist()

        # Add data to the x, y, and z lists
        x_data = [pose['position'][0] for pose in pose_history]
        y_data = [pose['position'][1] for pose in pose_history]
        z_data = [pose['position'][2] for pose in pose_history]

        # Update the plot data
        ax.clear()
        ax.plot(x_data, y_data, z_data)

        # Set plot labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Motion Path')

        # Draw and pause the figure to update in real-time
        fig.canvas.draw()
        plt.pause(0.001)

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Save the pose history to a JSON file after the loop
with open('pose_history.json', 'w') as f:
    json.dump(pose_history, f)

cap.release()
cv2.destroyAllWindows()
