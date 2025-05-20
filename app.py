import cv2
from ultralytics import YOLO

# Đường dẫn tới mô hình đã huấn luyện
model_path = r'D:\muadoan\PBL6\PBL6\Code\Python\IOT\best.pt'

# Load mô hình YOLOv8
model = YOLO(model_path)

# Mở webcam (0 là mặc định)
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Không thể mở camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Dự đoán trên khung hình từ webcam
    results = model(frame)

    # Vẽ kết quả lên khung hình
    annotated_frame = results[0].plot()

    # Hiển thị kết quả
    cv2.imshow("YOLOv8 Detection", annotated_frame)

    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng tài nguyên
cap.release()
cv2.destroyAllWindows()
