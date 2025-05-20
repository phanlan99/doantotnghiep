import cv2
import numpy as np
from ultralytics import YOLO

# Đường dẫn tới model segmentation
model_path = r'D:/muadoan/PBL6/PBL6/Code/Python/IOT/best.pt'

# Load model
model = YOLO(model_path)

# Mở webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Không thể mở camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Dự đoán segment
    results = model(frame, task="segment")
    result = results[0]

    if result.masks is not None:
        masks = result.masks.data.cpu().numpy()   # (n, h, w)
        names = model.names
        classes = result.boxes.cls.cpu().numpy() if result.boxes is not None else []
        segments = result.masks.xy  # ✅ Lấy danh sách contour theo pixel

        for i, mask in enumerate(masks):
            # Tạo màu cho mỗi mask
            color = (0, 255, 0)
            mask = (mask * 255).astype(np.uint8)
            colored_mask = cv2.merge([np.zeros_like(mask), mask, mask])
            frame = cv2.addWeighted(frame, 1.0, colored_mask, 0.4, 0)

            # Hiển thị tên class tại vị trí đầu tiên trong polygon (nếu có)
            if len(segments) > i and len(segments[i]) > 0:
                x, y = segments[i][0]
                class_id = int(classes[i]) if i < len(classes) else 0
                label = names[class_id]
                cv2.putText(frame, label, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0), 2, cv2.LINE_AA)

    # Hiển thị ảnh
    cv2.imshow('YOLOv8 Segmentation (Mask + Label only)', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
