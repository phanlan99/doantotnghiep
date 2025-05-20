import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO(r'D:\muadoan\PBL6\PBL6\Code\Python\IOT\best.pt')

camera_matrix = np.array([[1.50380710e+03, 0.00000000e+00, 6.86478160e+02],
                          [0.00000000e+00, 1.50474731e+03, 4.80534599e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coeffs = np.array([[-0.44584434, 0.30173071, 0.00053927, 0.00136141, -0.10749109]])

def undistort_and_crop(frame, camera_matrix, distortion_coeffs):
    h, w = frame.shape[:2]
    undistorted = cv2.undistort(frame, camera_matrix, distortion_coeffs, None)
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 1, (w, h))
    x, y, w, h = roi
    return undistorted[y:y+h, x:x+w]

def pixel_to_world(center_x, center_y, camera_matrix, scale=1.0):  # từ 0.1 lên 1.0
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    z0 = 0  # giả định
    x0 = (center_x - cx) / fx * scale
    y0 = (center_y - cy) / fy * scale

    return x0, y0, z0


cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Không thể mở camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    undistorted_frame = undistort_and_crop(frame, camera_matrix, distortion_coeffs)
    results = model(undistorted_frame)[0]
    overlay = undistorted_frame.copy()

    if results.masks is not None:
        masks = results.masks.data.cpu().numpy()
        for i, mask in enumerate(masks):
            color = np.random.randint(0, 255, (3,), dtype=np.uint8).tolist()
            mask_resized = cv2.resize(mask, (overlay.shape[1], overlay.shape[0]))
            mask_resized = (mask_resized * 255).astype(np.uint8)
            colored_mask = np.zeros_like(undistorted_frame)
            for c in range(3):
                colored_mask[:, :, c] = (mask_resized * (color[c] / 255.0)).astype(np.uint8)

            overlay = cv2.addWeighted(overlay, 1.0, colored_mask, 0.5, 0)

    boxes = results.boxes
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        class_id = int(box.cls[0])
        class_name = model.names[class_id]

        # Tính tọa độ x0, y0, z0 theo cm
        x0, y0, z0 = pixel_to_world(center_x, center_y, camera_matrix)

        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(overlay, (center_x, center_y), 5, (0, 0, 255), -1)
        
        label = f"{class_name} ({center_x},{center_y})"
        coords = f"x0={x0:.1f}cm, y0={y0:.1f}cm, z0={z0:.1f}cm"

        cv2.putText(overlay, label, (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        cv2.putText(overlay, coords, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

    cv2.imshow("YOLOv8 Segmentation + Center + World Coords", overlay)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
