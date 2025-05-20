import os
from PIL import Image

# Đường dẫn đến thư mục ảnh gốc và thư mục test
folder_path = r'D:\abcdef\train'
test_folder_path = r'D:\abcdef\train\test'

# Nếu thư mục test không tồn tại, tạo mới
if not os.path.exists(test_folder_path):
    os.makedirs(test_folder_path)

# Duyệt qua tất cả các file trong thư mục
for filename in os.listdir(folder_path):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
        image_path = os.path.join(folder_path, filename)
        
        # Mở ảnh
        with Image.open(image_path) as img:
            width, height = img.size
            
            # Kiểm tra nếu ảnh là dọc (chiều cao > chiều rộng)
            if height > width:
                # Xoay ảnh 90 độ theo chiều kim đồng hồ
                rotated = img.rotate(90, expand=True)
                
                # Lưu ảnh đã xoay vào thư mục test với tên giống ảnh gốc
                rotated.save(os.path.join(test_folder_path, filename))
                print(f"Đã xoay và lưu: {filename}")
            else:
                # Nếu không cần xoay, sao chép ảnh gốc sang thư mục test
                img.save(os.path.join(test_folder_path, filename))
                print(f"Không cần xoay, sao chép: {filename}")
