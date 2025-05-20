import cv2
import numpy as np
from ultralytics import YOLO
import cvzone
import math
import serial
import time
ser = serial.Serial('COM9', 9600, timeout= 0.1)  # Mở kết nối với COM7, tốc độ baudrate 9600

a = 1
limits = [1180, 0, 1180, 720]
def transmit_data(data):
    global a
    if a == 1:
        ser.write(data.encode())
        a = 0

def undistort_and_crop(frame, camera_matrix, distortion_coeffs):
    h, w = frame.shape[:2]

    # Hiệu chỉnh ảnh bị biến dạng
    undistorted_frame = cv2.undistort(frame, camera_matrix, distortion_coeffs, None)

    # Tìm kích thước ảnh hiệu chỉnh
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 1, (w, h))

    # Cắt ảnh hiệu chỉnh
    x, y, w, h = roi
    undistorted_frame = undistorted_frame[y:y + h, x:x + w]

    return undistorted_frame

def inverse_kinematics(Px_inv, Py_inv, Pz_inv= 125): #cố đinh Pz lúc này bằng 125
    # Định nghĩa các tham số
    d1 = 206
    a1 = 0
    a2 = 320
    a3 = 14
    d4 = 327
    d6 = 158

    # Ma trận xoay R06
    R06 = np.array([[1, 0, 0],
                    [0, -1, 0],
                    [0, 0, -1]])

    # Tính Wx, Wy, Wz
    Wx = Px_inv - d6 * R06[2, 0]
    Wy = Py_inv - d6 * R06[2, 1]
    Wz = Pz_inv - d6 * R06[2, 2]

    # Tính theta1
    theta1 = np.degrees(np.arctan2(Wy, Wx))

    # Tính các góc phi1, phi2 cho theta2
    cos_phi = ((np.sqrt(Wx ** 2 + Wy ** 2) - a1) ** 2 + (Wz - d1) ** 2 + a2 ** 2 - (a3 ** 2 + d4 ** 2)) / (
            2 * a2 * np.sqrt((np.sqrt(Wx ** 2 + Wy ** 2) - a1) ** 2 + (Wz - d1) ** 2))
    cos_phi = np.clip(cos_phi, -1, 1)  # Giới hạn cos_phi trong phạm vi hợp lệ
    sin_phi = np.sqrt(1 - cos_phi ** 2)

    phi1 = np.degrees(np.arctan2(sin_phi, cos_phi))
    phi2 = np.degrees(np.arctan2(-sin_phi, cos_phi))

    sigma = np.degrees(np.arctan2(Wz - d1, np.sqrt(Wx ** 2 + Wy ** 2) - a1))
    t2_inv1 = sigma - phi1
    t2_inv2 = sigma - phi2

    # Tính các góc gamma1, gamma2 cho theta3
    cos_gamma = (-(a2 ** 2 + a3 ** 2 + d4 ** 2) + (np.sqrt(Wx ** 2 + Wy ** 2) - a1) ** 2 + (Wz - d1) ** 2) / (
            2 * a2 * np.sqrt(a3 ** 2 + d4 ** 2))
    cos_gamma = np.clip(cos_gamma, -1, 1)  # Giới hạn cos_gamma trong phạm vi hợp lệ
    sin_gamma = np.sqrt(1 - cos_gamma ** 2)

    gamma1 = np.degrees(np.arctan2(sin_gamma, cos_gamma))
    gamma2 = np.degrees(np.arctan2(-sin_gamma, cos_gamma))

    beta = np.degrees(np.arctan2(d4, a3))
    t3_inv1 = gamma1 + beta
    t3_inv2 = gamma2 + beta

    # Chọn giá trị nhỏ nhất (theo giá trị tuyệt đối) cho theta2 và theta3
    theta2 = t2_inv2
    theta3 = t3_inv2

    # Tính theta4 và theta6
    T03_inv = np.array([
        [np.cos(np.radians(theta2 + theta3)) * np.cos(np.radians(theta1)),
         np.cos(np.radians(theta2 + theta3)) * np.sin(np.radians(theta1)), np.sin(np.radians(theta2 + theta3))],
        [np.sin(np.radians(theta1)), -np.cos(np.radians(theta1)), 0],
        [np.sin(np.radians(theta2 + theta3)) * np.cos(np.radians(theta1)),
         np.sin(np.radians(theta2 + theta3)) * np.sin(np.radians(theta1)), -np.cos(np.radians(theta2 + theta3))]
    ])

    R36 = np.dot(T03_inv, R06)
    r13, r23, r33 = R36[0, 2], R36[1, 2], R36[2, 2]
    r31, r32 = R36[2, 0], R36[2, 1]

    # Tính theta4
    theta4_1 = np.degrees(np.arctan2(-r23, -r13))
    theta4_2 = np.degrees(np.arctan2(r23, r13))

    # Tính theta6
    theta6_1 = np.degrees(np.arctan2(r32, -r31))
    theta6_2 = np.degrees(np.arctan2(-r32, r31))

    # Tính theta5
    costheta5 = r33
    sintheta5 = np.sqrt(1 - costheta5**2)
    theta5_1 = np.degrees(np.arctan2(-sintheta5, costheta5))
    theta5_2 = np.degrees(np.arctan2(sintheta5, costheta5))

    # Chọn cặp theta4, theta5, theta6 dựa trên giá trị nhỏ nhất của theta4 (theo giá trị tuyệt đối)
    if abs(theta4_1) < abs(theta4_2):
        theta4 = round(theta4_1, 2)
        theta5 = round(theta5_1, 2)
        theta6 = round(theta6_1, 2)
    else:
        theta4 = round(theta4_2, 2)
        theta5 = round(theta5_2, 2)
        theta6 = round(theta6_2, 2)

    return theta1, theta2, theta3, theta4, theta5, theta6


# Thiết lập các thông số ma trận xoay và dịch tọa độ
cm_to_pixel = (291.5/ 1280) 
# Giải thích:
#291.5 là chiều dài thực tế (cm) của một đối tượng (hoặc một phần của nó) trên một vùng ảnh đã được đo.
#1280 là độ rộng của hình ảnh trong pixel (chiều rộng của ảnh trong hệ tọa độ ảnh).
Rot_180X = [[1, 0, 0], [0, np.cos(np.pi), -np.sin(np.pi)], [0, np.sin(np.pi), np.cos(np.pi)]]
rad = (-90 / 180) * np.pi
Rot_90Z = [[np.cos(rad), -np.sin(rad), 0], [np.sin(rad), np.cos(rad), 0], [0, 0, 1]]
RO_C = np.dot(Rot_180X, Rot_90Z)
dO_C = [[260+10], [-175-45], [0]]
HO_C = np.concatenate((RO_C, dO_C), axis=1)
HO_C = np.concatenate((HO_C, [[0, 0, 0, 1]]), axis=0)

# Thiết lập camera
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Tham số hiệu chỉnh camera mới

camera_matrix = np.array([[1.50380710e+03, 0.00000000e+00, 6.86478160e+02],
                          [0.00000000e+00, 1.50474731e+03, 4.80534599e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coeffs = np.array([[-0.44584434, 0.30173071, 0.00053927, 0.00136141, -0.10749109]])

# Thiết lập mô hình YOLO
model = YOLO(r'D:\muadoan\PBL6\PBL6\Code\Python\IOT\best.pt')



# Danh sách tên lớp và màu sắc tương ứng
# Danh sách tên lớp và màu sắc tương ứng
classNames = ['beroca', 'cachua', 'cam', 'egg', 'maleutyl', 'probio', 'sui', 'topralsin', 'vitatrum', 'zidocinDHG']

# Sinh màu ngẫu nhiên cho mỗi class
import random
random.seed(42)
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for _ in classNames]


while True:
    ret, frame = cap.read()
    if not ret:
        print("Không thể đọc từ camera. Vui lòng kiểm tra lại.")
        break

    # Hiệu chỉnh ảnh
    undistorted_frame = undistort_and_crop(frame, camera_matrix, distortion_coeffs)

    # Phân loại bằng YOLO
    results = model(undistorted_frame, stream=True, verbose=False)
    num_seeds = 0

    # Danh sách để lưu bounding box và khoảng cách
    bounding_boxes = []
    # Kẻ line
    cv2.line(undistorted_frame, (limits[0], limits[1]), (limits[2], limits[3]), (0, 0, 255), 5)

    for r in results:
        boxes = r.boxes
        num_seeds = len(boxes)  # Số lượng hạt
        for i, box in enumerate(boxes):
            # Bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            w, h = x2 - x1, y2 - y1

            # Centerpoint
            centerx = (x1 + x2) / 2
            centery = (y1 + y2) / 2
            centimetx = round(centerx * cm_to_pixel, 2)
            centimety = round(centery * cm_to_pixel, 2)

            # Tính khoảng cách từ gốc tọa độ (0, 0)
            distance = math.sqrt(centimetx**2 + centimety**2)
            bounding_boxes.append((distance, box, (centimetx, centimety)))

    # Sắp xếp bounding box theo khoảng cách
    bounding_boxes.sort(key=lambda x: x[0])

    # Vẽ bounding box và in thông tin
    for idx, (distance, box, (centimetx, centimety)) in enumerate(bounding_boxes):
        # Lấy tọa độ của bounding box
        x1, y1, x2, y2 = box.xyxy[0]
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # Confident
        conf = math.ceil((box.conf[0] * 100)) / 100

        # ClassName
        cls = int(box.cls[0])

        # Gán màu từ danh sách
        if 0 <= cls < len(classNames):
            currentClass = classNames[cls]
            myColor = colors[cls]
        else:
            # currentClass = "Unknown"
            # myColor = (0, 0, 0)
            print(f"Warning: Invalid class index {cls}")


        cv2.rectangle(undistorted_frame, (x1, y1), (x2, y2), myColor, 3)
        cv2.circle(undistorted_frame, (int(centerx), int(centery)), radius=3, color=(255, 255, 255), thickness=5)
        cvzone.putTextRect(undistorted_frame, f'{classNames[cls]} {conf}', (max(0, x1), max(35, y1)),
                           scale=1, thickness=1, colorB=myColor, colorT=(255, 255, 255), colorR=myColor)
        cv2.putText(undistorted_frame, f'{idx + 1}', (max(10, x1), max(0, y1 + 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 3)

        # In tọa độ lên màn hình với số thứ tự
        # print(f'Khung {idx + 1}: Tọa độ vật pixel: x={centimetx:.2f}, y={centimety:.2f}')
        PC = np.array([[centimetx], [centimety], [-400], [1]])
        P0 = np.dot(HO_C, PC)
        x0 = round(P0[0, 0], 2)
        y0 = round(P0[1, 0], 2)
        z0 = round(P0[2, 0], 2)
        print(f'Toạ độ thực tế (x, y, z): ({x0}, {y0}, {z0})')
        # print(f'x0: {x0}, y0: {y0}')
        cvzone.putTextRect(undistorted_frame, f'Toado: x={x0:.2f}, y={y0:.2f}', (x2, y2), scale=1, thickness=1, colorR=myColor)
        # Thêm điều kiện kiểm tra vị trí tâm trước khi gửi dữ liệu
        if idx + 1 == 1 and centerx <1180:
            # Tính động học nghịch
            t1_inv, t2_inv, t3_inv, t4_inv, t5_inv, t6_inv = inverse_kinematics(x0, y0)

            # Làm tròn các góc
            t1_inv_rounded = round(t1_inv, 2)
            t2_inv_rounded = round(t2_inv, 2)
            t3_inv_rounded = round(t3_inv, 2)
            t4_inv_rounded = round(t4_inv, 2)
            t5_inv_rounded = round(t5_inv, 2)
            t6_inv_rounded = round(t6_inv, 2)

            print(f'Theta1: {t1_inv_rounded}, Theta2: {t2_inv_rounded}, Theta3: {t3_inv_rounded}, '
                  f'Theta4: {t4_inv_rounded}, Theta5: {t5_inv_rounded}, Theta6: {t6_inv_rounded}')

            # Gửi dữ liệu đến Arduino qua serial
            data = f'Start,{currentClass},{t1_inv_rounded},{t2_inv_rounded},{t3_inv_rounded},{t4_inv_rounded},{t5_inv_rounded},{t6_inv_rounded},{x0},{y0},{z0},\n'
            transmit_data(data)
            print(data)

            response = ser.readline().strip()  # Đọc dữ liệu trả về từ Arduino
            if response == b'Done':
                a = 1
                print("Đã nhận 'Done' từ Arduino")
            else:
                print(f"Phản hồi không mong đợi")

    # Thêm nhãn số lượng hạt lên hình ảnh
    label = f'Seed Count: {num_seeds}'
    cvzone.putTextRect(undistorted_frame, label, (10, 710), scale=1, thickness=1, colorR=(0, 0, 255))

    cv2.imshow("God", undistorted_frame)

    # Thoát khỏi vòng lặp khi nhấn 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng camera và đóng các cửa sổ
cap.release()
cv2.destroyAllWindows()
