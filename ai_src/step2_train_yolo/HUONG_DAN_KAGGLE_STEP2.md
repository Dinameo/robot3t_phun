# Hướng Dẫn Huấn Luyện Bước 2 (YOLO11-seg & YOLO11-pose) Trên Kaggle

Toàn bộ dữ liệu (300 mẫu: **240 mẫu Train + 60 mẫu Val**, đã loại bỏ trùng lặp và đồng bộ 100% nhãn bạn vừa chỉnh sửa) đã được đóng gói sẵn tại thư mục:
`/home/nhan/RB3t_PJ/ai_src/step2_train_yolo/`

---

## 1. Hai file duy nhất bạn cần đưa lên Kaggle

Trong thư mục `/home/nhan/RB3t_PJ/ai_src/step2_train_yolo/`, bạn chỉ cần dùng 2 file sau:

1. **Gói dữ liệu (Dataset Zip - 180 MB)**:  
   `/home/nhan/RB3t_PJ/ai_src/step2_train_yolo/kaggle_dataset_dobot_cr10a.zip`
   - Đã nén 300 ảnh sang chuẩn JPG Quality=95 (giảm từ 1.3 GB xuống còn 180 MB để bạn tải lên Kaggle chỉ mất vài giây).
   - Chứa sẵn cả 2 bộ nhãn `labels_seg/` (cho Segmentation) và `labels_pose/` (cho 12 mốc giải phẫu Pose).

2. **File kịch bản chạy tự động (Jupyter Notebook)**:  
   `/home/nhan/RB3t_PJ/ai_src/step2_train_yolo/dobot_cr10a_yolo11_kaggle_notebook.ipynb`

---

## 2. Thao tác 4 bước trên Kaggle (Chỉ mất ~2 phút thiết lập)

1. **Mở Notebook trên Kaggle**:
   - Truy cập [kaggle.com/code](https://www.kaggle.com/code) $\rightarrow$ Nhấn **`+ New Notebook`**.
   - Trên thanh menu của Kaggle, chọn **`File`** $\rightarrow$ **`Import Notebook`** $\rightarrow$ Chọn file **`dobot_cr10a_yolo11_kaggle_notebook.ipynb`** từ máy tính của bạn.

2. **Tải Dataset lên Notebook**:
   - Ở thanh công cụ bên phải màn hình Kaggle, nhấn **`+ Add Input`** $\rightarrow$ **`Upload`** $\rightarrow$ **`New Dataset`**.
   - Đặt tên bất kỳ (ví dụ: `dobot-cr10a-jeans`) và kéo thả file **`kaggle_dataset_dobot_cr10a.zip`** vào $\rightarrow$ Nhấn **`Create`**.
   - *(Notebook đã được lập trình tự động dò tìm đường dẫn `/kaggle/input/...` nên bạn đặt tên Dataset là gì code cũng tự nhận diện được!)*

3. **Bật GPU & Bấm Run All**:
   - Ở mục **Session Options -> Accelerator** (cột bên phải), chọn **`GPU T4 x2`** hoặc **`GPU P100`**.
   - Đảm bảo mục **Internet** đang bật (**`Internet on`**) để tải pretrained weights `yolo11s-seg.pt` và `yolo11s-pose.pt`.
   - Nhấn nút **`Run All`** trên cùng.

4. **Tải kết quả về máy**:
   - Sau khoảng **10 - 15 phút**, quá trình huấn luyện cả 2 mô hình hoàn tất.
   - Cell cuối cùng sẽ tự động gom cả 2 file trọng số tốt nhất (`best_yolo11_seg.pt` và `best_yolo11_pose.pt`) cùng toàn bộ biểu đồ đánh giá (`results.png`, `PR_curve.png`, `confusion_matrix.png`) thành **1 file duy nhất**:
     👉 **`/kaggle/working/dobot_cr10a_trained_models.zip`**
   - Bạn bấm tải file `.zip` này về máy và giải nén 2 file `.pt` vào thư mục:
     `/home/nhan/RB3t_PJ/ai_src/step2_train_yolo/weights/`
