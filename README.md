# 🤖 HỆ THỐNG TỰ ĐỘNG HÓA NHẬN DIỆN & PHUN SƠN / VẼ 3D TRÊN ROBOT DOBOT CR10A (6-DOF)

> **Dự án Nghiên cứu & Phát triển (R&D)**: Tự động hóa quy trình nhận diện phôi may mặc (quần jean/kaki 3D), bù đắp chỗ khuất vi mô, tự động co giãn mẫu phun theo kích thước thực tế (Size S/M/L) và điều khiển cánh tay robot công nghiệp **Dobot CR10A** bám sát bề mặt vải cong 3D.

---

## 📑 MỤC LỤC
1. [Tổng Quan Hệ Thống](#-tổng-quan-hệ-thống)
2. [Cấu Trúc Thư Mục Dự Án](#-cấu-trúc-thư-mục-dự-án)
3. [Kiến Trúc Kỹ Thuật (Pipeline 3 Bước AI + Unity Sim)](#-kiến-trúc-kỹ-thuật)
4. [Các Tính Năng Cốt Lõi Đã Triển Khai](#-các-tính-năng-cốt-lõi-đã-triển-khai)
5. [Yêu Cầu Phần Cứng & Phần Mềm](#-yêu-cầu-phần-cứng--phần-mềm)
6. [Hướng Dẫn Cài Đặt & Vận Hành](#-hướng-dẫn-cài-đặt--vận-hành)
7. [Giao Thức Truyền Thông & Xuất Lệnh Robot Thật](#-giao-thức-truyền-thông--xuất-lệnh-robot-thật)

---

## 🌟 1. TỔNG QUAN HỆ THỐNG

Quy trình phun sơn hoa văn hoặc cào sờn trên phôi quần jean đặt trên bàn thao tác thường gặp các thách thức:
* Vải là vật liệu mềm dẻo, có nhiều nếp gấp, độ dốc lớn và góc khuất (Self-occlusion) khiến camera 3D Zivid bị mất điểm dữ liệu.
* Kích thước mỗi chiếc quần thay đổi (Size S, M, L) khiến các mẫu vẽ cố định bị lệch hoặc tràn ra ngoài mép vải.
* Đám mây điểm 3D thô (1.7 triệu điểm) quá nặng cho GPU máy tính vận hành tại xưởng.

**Giải pháp của dự án**:
1. **AI Pipeline (YOLO11 Pose & Seg)**: Tự động định vị 12 điểm mốc giải phẫu cốt lõi và đường bao thân quần trong < 50ms.
2. **Thin Plate Spline (TPS RBF)**: Tự động biến dạng phi tuyến và co giãn mẫu phun tiêu chuẩn khớp 100% với kích thước thực tế của quần mà không cần vẽ lại.
3. **Chuyển đổi 3D Surface Mesh siêu nhẹ**: Tái cấu trúc 1.7M điểm thành mạng đa giác tam giác (~25K đỉnh, 1 Draw Call), tăng tốc độ khung hình từ giật lag lên mượt mà 60–120 FPS.
4. **Bù đắp chỗ khuất vi mô**: Thuật toán KDTree K-NN SVD và Poisson PDE tự động vá kín lỗ thủng và nghiêng góc súng (Adaptive Gun Tilt) để sơn phủ đều đáy nếp gấp.
5. **Động học Nghịch Giải Tích (Closed-Form IK)**: Giải nghiệm tức thời 6 góc khớp cho robot Dobot CR10A, tự động tránh kỳ dị và đảm bảo an toàn va chạm.

---

## 📂 2. CẤU TRÚC THƯ MỤC DỰ ÁN

```text
RB3t_PJ/
├── dobot_cr10a/                           # [UNITY PROJECT] Mô phỏng 3D & Điều khiển Robot
│   ├── Assets/
│   │   ├── src/
│   │   │   ├── DobotCR10AIKController.cs  # Bộ điều khiển trung tâm, UI Ribbon, IK solver, Quản lý Quỹ đạo
│   │   │   ├── DobotCR10AKinematics.cs    # Động học thuận/nghịch 6-DOF giải tích (Closed-form IK)
│   │   │   ├── PLYDirectLoader.cs         # Nạp PLY Zivid, chuyển đổi 3D Mesh, Pin-hole Inpainting
│   │   │   ├── AiServerClient.cs          # Client UDP socket 5005 & HTTP REST 5000 kết nối AI
│   │   │   ├── AntigravityUITheme.cs      # Bộ Theme giao diện phẳng Obsidian Dark / Cyber Clean
│   │   │   └── RobotCameraController.cs   # Điều khiển quỹ đạo camera (Perspective, Top-Down, Focus)
│   │   ├── PointCloud/                    # Shaders và file mẫu đám mây điểm 3D (.ply, .mat)
│   │   └── Scenes/                        # Các Scene mô phỏng (SampleScene.unity, Robot_FK_IK.unity)
│   └── output_drawing/                    # Nơi lưu quỹ đạo xuất (CSV 6-DOF, Dobot Python .py, Template Size S/M/L)
│
├── ai_src/                                # [AI PIPELINE] Toàn bộ mã nguồn & mô hình AI
│   ├── step1_auto_label/                  # BƯỚC 1: Tự động gán nhãn 12 mốc 3D & Landmark Editor GUI
│   │   ├── step1_auto_labeling_3d.py      # Tự động trích xuất 12 mốc từ camera RGB + Zivid Depth
│   │   ├── landmark_editor_app.py         # Ứng dụng Desktop kéo thả chỉnh sửa mốc thủ công
│   │   └── LANDMARK_EDITOR_GUIDE.md
│   │
│   ├── step2_train_yolo/                  # BƯỚC 2: Huấn luyện YOLO11 Pose & Segmentation
│   │   ├── weights/                       # Trọng số mô hình đã huấn luyện tối ưu:
│   │   │   ├── best_yolo11_pose.pt        # Model nhận diện 12 mốc giải phẫu (mAP50 > 98%)
│   │   │   └── best_yolo11_seg.pt         # Model phân đoạn thân quần & vùng phun (mAP50 > 95%)
│   │   ├── dobot_cr10a_yolo11_kaggle_notebook.ipynb  # Notebook huấn luyện GPU trên Kaggle
│   │   ├── prepare_kaggle_dataset.py      # Chuẩn bị dataset chuẩn định dạng YOLO
│   │   ├── test_inference_step2.py        # Kiểm thử nhận diện trên ảnh thực tế
│   │   └── reports/                       # Biểu đồ F1-Confidence, PR-Curve, Ma trận nhầm lẫn
│   │
│   ├── step3_3d_trajectory/              # BƯỚC 3: Sinh quỹ đạo 3D & Máy chủ truyền thông
│   │   ├── unity_ai_server.py             # Server UDP (5005/5006) & HTTP REST (5000) phục vụ Unity
│   │   ├── step3_3d_trajectory_generator.py # Ánh xạ phi tuyến TPS RBF lên mặt cong 3D
│   │   ├── run_ai_server.sh               # Script khởi động nhanh máy chủ AI
│   │   └── outputs/                       # Dữ liệu xuất JSON/CSV mẫu
│   │
│   ├── HUONG_DAN_AI_PIPELINE.md           # Hướng dẫn chi tiết toàn bộ thuật toán AI
│   └── HUONG_DAN_AI_PIPELINE.pdf
│
├── README.md                              # Tài liệu tổng quan dự án (File này)
└── .gitignore                             # Cấu hình lọc bỏ file rác, dataset nặng và cache
```

---

## 🔬 3. KIẾN TRÚC KỸ THUẬT

```
      +---------------------------------------------------------+
      |                 HỆ QUY CHIẾU CAMERA 3D                  |
      |   Zivid 3D Fixed Rig (Tọa độ: Y = 1.685m, Z = 0.335m)   |
      +----------------------------+----------------------------+
                                   |
         +-------------------------+-------------------------+
         |                                                   |
         v                                                   v
+-----------------------------+           +-------------------------------------+
|      BỘ MÃ NGUỒN AI         |           |            UNITY 3D ENGINE          |
|  (Python / PyTorch / YOLO)  |           |     (C# / Universal Render Pipe)    |
+-----------------------------+           +-------------------------------------+
| 1. YOLO11-Seg & Pose        |           | 1. PLYDirectLoader                  |
|    - 12 mốc giải phẫu       |           |    - Đám mây điểm 1.7M điểm         |
|    - Đường bao thân quần    |           |    - 3D Surface Mesh (~25K đỉnh)    |
|                             |           |    - Pin-hole Morphological Inpaint |
| 2. TPS Spline RBF           |           |                                     |
|    - Co giãn mẫu vẽ tự động |           | 2. Canonical Jeans Canvas           |
|    - Bù biến dạng phi tuyến |   UDP     |    - Bảng vẽ 2D tiêu chuẩn          |
|                             |  5005/    |    - Tự động co giãn theo Size      |
| 3. unity_ai_server.py       |  5006     |                                     |
|    - Đóng gói tọa độ 3D     | --------> | 3. Closed-Form Analytical IK        |
|    - UDP Streaming (< 1ms)  |           |    - Giải 6 góc khớp Dobot CR10A    |
|    - HTTP REST Server: 5000 |           |    - Kiểm soát va chạm & góc nghiêng|
+-----------------------------+           +------------------+------------------+
                                                             |
                                                             v
                                          +-------------------------------------+
                                          |         ROBOT THẬT / XUẤT LỆNH      |
                                          | - Dobot CR10A Python SDK Script     |
                                          | - File CSV Quỹ đạo 6-DOF (Joint/TCP)|
                                          +-------------------------------------+
```

---

## ⚡ 4. CÁC TÍNH NĂNG CỐT LÕI ĐÃ TRIỂN KHAI

### A. Tự Động Đổi Kích Thước Mẫu Phun (Size Auto-Switching)
* Khi nạp một file phôi bất kỳ (`.ply`), hệ thống tự động đo đạc kích thước nhân trắc học:
  * **Inseam** (Chiều dài đũng - gấu): Đo từ mốc đáy đũng (mốc 3) đến gấu quần.
  * **Outseam** (Chiều dài hông - gấu): Đo từ cạp eo (mốc 0, 2) xuống gấu ngoài.
  * **Vòng eo & Ống chân**: Tính toán diện tích bề mặt $cm^2$.
* Tự động nhận diện Size: **Size S** ($<650\text{mm}$), **Size M** ($650–740\text{mm}$), **Size L** ($>740\text{mm}$).
* **Bảo tồn mẫu vẽ**: Các nét vẽ hoa văn tự động co giãn theo tỷ lệ hình học mà **không bị reset hoặc mất nét**.

### B. Chuyển Đổi Point Cloud ➔ 3D Surface Mesh Siêu Nhẹ
* Đám mây điểm Zivid 1.7M điểm ngốn GPU và làm quạt laptop quay to.
* Tự động tạo **3D Surface Mesh** (~25.000 đỉnh, ~40.000 tam giác) chỉ với **1 Draw Call duy nhất**.
* Giữ nguyên 100% màu sắc thực tế của vải Jean (RGB trung bình từng ô).
* Ngưỡng lọc vách đứng ngăn hiện tượng dính mạng nhện giữa 2 ống quần.
* Tích hợp `MeshCollider` hỗ trợ Raycast vuông góc, loại bỏ triệt để sai số làm phẳng đỉnh nếp gấp.

### C. Bù Đắp Chỗ Khuất & Hàn Gắn Lỗ Khuyết Tật (Occlusion Compensation)
* Phát hiện chính xác 100% các lỗ khuyết camera thông qua hàm `HasSurfacePointAt`.
* Hàn gắn tự động bằng thuật toán **Morphological Pin-hole Inpainting**.
* Tích hợp 5 thuật toán chuyên sâu theo hướng dẫn `HUONG_DAN_AI_PIPELINE.md`:
  1. *KDTree K-NN Tangent Plane & SVD/PCA*
  2. *Thin Plate Spline (TPS) RBF Non-Rigid*
  3. *Poisson & Laplacian PDE Surface Propagation*
  4. *Adaptive Gun Tilt & Crease Standoff* (Tự nghiêng đầu phun $\pm 18^\circ$ theo sườn nếp nhăn)
  5. *Bilateral Leg Symmetry Mirroring* (Lấy đối xứng từ ống quần không bị khuất)

### D. Tùy Chỉnh Độ Cao Bám Mặt Quần & Bật/Tắt Quỹ Đạo
* Thanh trượt điều chỉnh cao độ bám mặt: `0.0 mm` (áp sát mặt vải cong) đến `50.0 mm` (khoảng cách đầu phun an toàn).
* Nút chuyển đổi nhanh:
  * `[Áp Sát Mặt Quần (0 mm)]`: Đường nét bám dính tuyệt đối vào nếp gấp 3D.
  * `[Khoảng Phun Chuẩn (20 mm)]`: Khoảng cách tiêu chuẩn khi vận hành súng sơn.
  * `[Cyan: BẬT / TẮT]`: Bật/tắt hiển thị đường polyline xanh cyan để dễ quan sát mặt vải.

### E. Giao Diện Obsidian Dark (Antigravity Theme) Tinh Gọn
* **Header Bar**: Thu gọn chiều cao còn 34px, gom 8 Tab xúc tích.
* **Command Ribbon**: Chiều cao giảm từ 96px xuống 64px, nút bấm chuẩn hóa 26px.
* **Right Dock**: Thu gọn bề rộng còn 410px, trả lại không gian tối đa cho góc nhìn 3D.

---

## 💻 5. YÊU CẦU PHẦN CỨNG & PHẦN MỀM

### Phần Mềm:
* **Hệ điều hành**: Linux Ubuntu (20.04 / 22.04 LTS) hoặc Windows 10/11 64-bit.
* **Unity Engine**: Unity 6 (6000.x) hoặc Unity 2022.3 LTS (Sử dụng Universal Render Pipeline - URP).
* **Python**: Python 3.10+ (Yêu cầu PyTorch, Ultralytics YOLO11, Open3D, NumPy, SciPy).

### Phần Cứng Đề Xuất:
* **CPU**: Intel Core i5 / AMD Ryzen 5 trở lên.
* **RAM**: Tối thiểu 16GB.
* **GPU**: NVIDIA GTX 1650 / RTX 3050 trở lên (Hỗ trợ CUDA để huấn luyện/chạy AI).

---

## 🚀 6. HƯỚNG DẪN CÀI ĐẶT & VẬN HÀNH

### Bước 1: Khởi Động Môi Trường AI Server
1. Mở cửa sổ Terminal:
   ```bash
   cd /media/nhan/01DC3601B2514510/RB3t_PJ/ai_src/step3_3d_trajectory
   ```
2. Cài đặt các thư viện phụ thuộc (nếu chưa có):
   ```bash
   pip install ultralytics torch open3d numpy scipy flask
   ```
3. Chạy máy chủ AI Server:
   ```bash
   python3 unity_ai_server.py
   # Hoặc: bash run_ai_server.sh
   ```
   *Máy chủ sẽ mở lắng nghe tại cổng UDP `5005` (nhận lệnh từ Unity) và UDP `5006` (đẩy dữ liệu tự động).*

### Bước 2: Mở Dự Án Unity
1. Mở **Unity Hub** ➔ Bấm **Add** ➔ Chọn thư mục `dobot_cr10a`.
2. Mở Scene chính: `Assets/Scenes/SampleScene.unity`.
3. Nhấn nút **Play (▶)** trên thanh công cụ Unity.

### Bước 3: Thao Tác Trên Giao Diện Unity
1. **Tab "POINT CLOUD"**:
   * Bấm `[📁 Duyệt File PLY]` để chọn một mẫu phôi quét (ví dụ: `pcl_151.ply`).
   * Chọn `[🌐 3D MESH SIÊU NHẸ]` để kích hoạt chế độ hiển thị mượt mà 120 FPS.
2. **Tab "QUỸ ĐẠO"**:
   * Vẽ hoa văn trên bảng vẽ 2D tiêu chuẩn (Canonical Jeans Canvas).
   * Bấm `[🎯 Áp Dụng 2D➔3D]`: Quỹ đạo sẽ tự động co giãn theo kích thước của quần và chiếu dính sát lên mặt vải cong 3D.
   * Chỉnh độ cao bám mặt tại nút `[Áp Sát Mặt Quần (0 mm)]` hoặc nút `[Cyan: BẬT/TẮT]`.
3. **Tab "BÙ CHỖ KHUẤT"**:
   * Bấm `[🔍 Quét Lỗ Khuyết]` ➔ `[⚡ Chạy Bù Đắp]` để vá kín các vùng bị nếp gấp che khuất.
4. **Tab "MÔ PHỎNG"**:
   * Nhấn `[CHẠY]` để quan sát cánh tay robot Dobot CR10A di chuyển đầu phun theo quỹ đạo 3D.
5. **Tab "XUẤT/NHẬP"**:
   * Nhấn `[Xuất Dobot (.py)]` hoặc `[Xuất CSV Tọa Độ]` để lấy chương trình nạp vào robot công nghiệp thật.

---

## 📡 7. GIAO THỨC TRUYỀN THÔNG & XUẤT LỆNH

### A. Giao Thức UDP Socket (Tốc Độ Cao < 1ms)
Dữ liệu trao đổi giữa AI Server và Unity sử dụng cấu trúc JSON tối ưu:
```json
{
  "status": "success",
  "sample_name": "pcl_151",
  "jeans_size": "M",
  "metrics": {
    "inseam_mm": 710.5,
    "outseam_mm": 985.2,
    "waist_mm": 412.0
  },
  "landmarks_3d": [
    {"id": 0, "name": "left_waist", "x": -0.185, "y": 0.025, "z": 0.820},
    {"id": 3, "name": "crotch",     "x":  0.002, "y": 0.038, "z": 0.520}
  ]
}
```

### B. Đầu Ra Cho Robot Dobot CR10A Thật
1. **File Script Python (`dobot_cr10a_trajectory_playback.py`)**:
   * Sử dụng thư viện chính hãng `dobot_api`.
   * Tự động nạp lệnh di chuyển nội suy không gian `MovL()` kết hợp điều khiển đóng/ngắt van súng phun qua tín hiệu Digital Output (`DO(1, 1)` / `DO(1, 0)`).
2. **File CSV Tọa Độ 6-DOF (`trajectory_pcl_xxx.csv`)**:
   * Cung cấp danh sách Waypoint với đầy đủ: Tọa độ Cartesian $(X, Y, Z)$ tính bằng mm, góc định hướng $(Roll, Pitch, Yaw)$ tính bằng độ, góc 6 khớp $(J_1, J_2, J_3, J_4, J_5, J_6)$ và cờ trạng thái phun `SprayOn (0/1)`.

---

## 👥 THÀNH VIÊN DỰ ÁN & BẢN QUYỀN
* **Nhóm Phát Triển**: Dự án Robot Phun Sơn Tự Động (Robot 3T - Dobot CR10A).
* **Phiên Bản**: 2.5 (Cập nhật tháng 11/2026).
* Mọi đóng góp và báo cáo lỗi xin vui lòng tạo Issue trên kho lưu trữ GitHub: [github.com/Dinameo/robot3t_phun](https://github.com/Dinameo/robot3t_phun).
