import cv2
import numpy as np
from ultralytics import YOLO

# Đường dẫn tới mô hình đã huấn luyện
model_path = r'D:\muadoan\PBL6\PBL6\Code\Python\IOT\best.pt'
model = YOLO(model_path)

# Tham số hiệu chỉnh camera
camera_matrix = np.array([[1.50380710e+03, 0.00000000e+00, 6.86478160e+02],
                          [0.00000000e+00, 1.50474731e+03, 4.80534599e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coeffs = np.array([[-0.44584434, 0.30173071, 0.00053927, 0.00136141, -0.10749109]])

# Hàm hiệu chỉnh và crop ảnh
def undistort_and_crop(frame, camera_matrix, distortion_coeffs):
    h, w = frame.shape[:2]
    undistorted = cv2.undistort(frame, camera_matrix, distortion_coeffs, None)
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 1, (w, h))
    x, y, w, h = roi
    return undistorted[y:y+h, x:x+w]

# Mở webcam
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Không thể mở camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Hiệu chỉnh hình ảnh
    undistorted_frame = undistort_and_crop(frame, camera_matrix, distortion_coeffs)

    # Dự đoán
    results = model(undistorted_frame)
    annotated_frame = results[0].plot()

    # Hiển thị kết quả
    cv2.imshow("YOLOv8 Detection (Corrected Image)", annotated_frame)

    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
