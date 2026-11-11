# HƯỚNG DẪN CHI TIẾT TOÀN DIỆN PIPELINE AI THỊ GIÁC & TỰ ĐỘNG SINH QUỸ ĐẠO PHUN (DOBOT CR10A)

> **Tài liệu tham chiếu chuẩn theo góp ý Báo cáo lần 1** (`Gop-y-Bao-cao-lan-1-Tu-dong-sinh-chuong-trinh-phun.pdf`)  
> **Dự án**: Trạm Robot Dobot CR10A tự động hóa phun sơn / hiệu ứng mài bạc trên sản phẩm may mặc (Denim).  
> **Phần cứng**: Robot 6 bậc tự do Dobot CR10A, Camera 3D Zivid 2 M70 (Eye-to-Hand / Fixed Rig), Súng phun / bút vẽ TCP.  
> **Dữ liệu đầu vào**: Ảnh màu 2D RGB (`dataset/_2D/image_*.png`) và Đám mây điểm 3D có tổ chức (`dataset/_3D/pcl_*.ply`, kích thước $1200 \times 1944$).

---

## MỤC LỤC
1. [Bản chất bài toán & Nguyên tắc cốt lõi](#1-bản-chất-bài-toán--nguyên-tắc-cốt-lõi)
2. [Hệ chỉ tiêu kỹ thuật đánh giá (Bộ 8 chỉ tiêu cốt lõi)](#2-hệ-chỉ-tiêu-kỹ-thuật-đánh-giá-bộ-8-chỉ-tiêu-cốt-lõi)
3. [Chuỗi hiệu chuẩn hệ thống (Calibration Chain)](#3-chuỗi-hiệu-chuẩn-hệ-thống-calibration-chain)
4. [Lộ trình 7 bước phát triển AI Pipeline](#4-lộ-trình-7-bước-phát-triển-ai-pipeline)
   - [Bước 1: Tiền xử lý & Tự sinh nhãn từ 3D (Auto-labeling)](#bước-1-tiền-xử-lý--tự-sinh-nhãn-từ-3d-auto-labeling)
   - [4.1. Chuẩn hóa Bộ 12 Mốc Giải Phẫu Quần Áo (Garment Landmarks)](#41-chuẩn-hóa-bộ-12-mốc-giải-phẫu-quần-áo-garment-landmarks)
   - [4.2. Thuật toán Đánh Mốc Bất Biến Góc Xoay (Invariant Geometry)](#42-thuật-toán-tự-động-đánh-mốc-bất-biến-góc-xoay-invariant-geometry)
   - [Bước 2: Huấn luyện mô hình thị giác AI (YOLO11-seg & YOLO11-pose)](#bước-2-huấn-luyện-mô-hình-thị-giác-ai-yolo11-seg--yolo11-pose)
   - [Bước 3: Chiếu điểm 2D sang 3D (2D-to-3D Projection)](#bước-3-chiếu-điểm-2d-sang-3d-2d-to-3d-projection)
   - [Bước 4: Ước lượng pháp tuyến vi mô bề mặt 3D (Surface Normal Estimation)](#bước-4-ước-lượng-pháp-tuyến-vi-mô-bề-mặt-3d-surface-normal-estimation)
   - [Bước 5: So sánh các phương pháp sinh quỹ đạo (P1, P3, P4)](#bước-5-so-sánh-các-phương-pháp-sinh-quỹ-đạo-p1-p3-p4)
   - [Bước 6: Khóa an toàn hình học & Động học (Safety Interlock)](#bước-6-khóa-an-toàn-hình-học--động-học-safety-interlock)
   - [Bước 7: Xuất lệnh & Kết nối thời gian thực qua ai_server.py](#bước-7-xuất-lệnh--kết-nối-thời-gian-thực-qua-ai_serverpy)
5. [Kế hoạch phân kỳ 3 đợt triển khai](#5-kế-hoạch-phân-kỳ-3-đợt-triển-khai)

---

## 1. BẢN CHẤT BÀI TOÁN & NGUYÊN TẮC CỐT LÕI

### 1.1. Tại sao bắt buộc phải dùng AI và 3D?
* **Đặc tính phôi vải mềm (Deformable Object)**: Quần jean / vải dệt không phải vật thể cứng (rigid CAD). Khi công nhân đặt phôi lên bàn gia công, vị trí tâm, góc xoay, nếp nhăn cục bộ, độ xòe ống và kích cỡ (S, M, L) đều thay đổi giữa mỗi chu kỳ.
* **Không được thu bài toán về 2D thuần túy**: Phun sơn / mài đồ trong công nghiệp yêu cầu khoảng cách súng tới mặt vải $d$ phải giữ nguyên và hướng đầu phun luôn phải vuông góc với bề mặt vải tại mọi nếp nhăn. Do đó, bài toán bắt buộc phải xử lý trong không gian 3 chiều $(X, Y, Z, \vec{n})$.
* **Nguyên tắc phân quyền an toàn**:
  > [!IMPORTANT]
  > **Mạng nơ-ron AI chỉ làm nhiệm vụ nhận dạng (Perception)**: phát hiện vùng vải (Segmentation) và định vị các mốc giải phẫu (Landmarks). **Không để mạng AI trực tiếp sinh chuỗi lệnh điều khiển động cơ robot.** Quỹ đạo hình học phải được sinh qua các thuật toán giải tích và được giám sát bởi **Khóa an toàn (Safety Interlock)**: kiểm tra tầm với, điểm kỳ dị, nguy cơ va chạm bàn trước khi gửi lệnh sang robot thật.

---

## 2. HỆ CHỈ TIÊU KỸ THUẬT ĐÁNH GIÁ (BỘ 8 CHỈ TIÊU CỐT LÕI)

Theo yêu cầu từ tài liệu góp ý báo cáo, mọi thuật toán và phương án triển khai phải được lượng hóa bằng số liệu đo lường cụ thể:

| STT | Chỉ tiêu kỹ thuật | Ký hiệu | Đơn vị | Phương pháp đo lường & Đánh giá | Mục tiêu |
| :---: | :--- | :---: | :---: | :--- | :---: |
| **1** | Sai số hình học quỹ đạo | $E_{geo}$ | $\text{mm}$ | So sánh vết vẽ/phun thực tế của robot với đường viền thiết kế danh nghĩa (Trung bình, P95, Hausdorff) | $\le 2.0\text{ mm}$ |
| **2** | Sai số khoảng cách súng | $E_d$ | $\text{mm}$ | Khoảng cách từ đầu phun TCP đến mặt vải dọc theo trục súng: $E_d = \|d_{thực} - d_{đặt}\|$ | $\le 3.0\text{ mm}$ |
| **3** | Sai số góc tiếp cận pháp tuyến | $E_\theta$ | $^\circ$ | Góc lệch giữa trục phun của súng và vector pháp tuyến cục bộ $\vec{n}$ của mặt vải | $\le 5.0^\circ$ |
| **4** | Tỷ lệ bao phủ & Tràn biên | $E_T, R_{drop}$ | $\%$ | Đo lường tỷ lệ diện tích sơn phủ đúng vùng yêu cầu và tỷ lệ sơn bị tràn ra ngoài biên vải | $E_T \ge 95\%$, $R_{drop} \le 3\%$ |
| **5** | Độ đều vận tốc TCP | $v_{TCP}$ | $\%$ | Độ ổn định tốc độ chuyển động của đầu phun (tránh hiện tượng đọng sơn khi đổi hướng khớp) | Dao động $\le 10\%$ |
| **6** | Sai số chuỗi hiệu chuẩn | $E_{calib}$ | $\text{mm}, ^\circ$ | Chạm điểm mốc bằng đầu nhọn TCP so với tọa độ 3D đo bởi camera Zivid | $\le 1.5\text{ mm}$ |
| **7** | Thời gian chu kỳ chuỗi (Cycle Time) | $t_{total}$ | Giây | Đo tổng thời gian từ lúc camera chụp $\rightarrow$ AI nhận dạng $\rightarrow$ sinh quỹ đạo $\rightarrow$ kiểm tra an toàn | $\le 10\text{ s}$ (Mục tiêu tổng thể $\le 60\text{s}$) |
| **8** | Thời gian đổi mã hàng so với dạy tay | $T_{setup}$ | Lần | So sánh thời gian lập trình thủ công bằng tay (Teach Pendant) so với tự động sinh quỹ đạo bằng AI | Nhanh hơn $\ge 15 - 20$ lần |

---

## 3. CHUỖI HIỆU CHUẨN HỆ THỐNG (CALIBRATION CHAIN)

Để ánh xạ chính xác một điểm từ ảnh chụp thành tọa độ điều khiển 6 khớp robot, hệ thống tuân thủ chuỗi biến đổi tọa độ khép kín:

$$\mathbf{P}_{TCP}^{Robot} = \mathbf{T}_{Base}^{World} \cdot \mathbf{T}_{Camera}^{Base} \cdot \mathbf{P}_{3D}^{Camera}$$

1. **Hiệu chuẩn Camera – Mặt bàn (Camera-to-Table Plane)**:
   * Sử dụng thuật toán RANSAC 3D trích xuất trực tiếp phương trình mặt bàn từ đám mây điểm Zivid ($Ax + By + Cz + D = 0$).
2. **Hiệu chuẩn Tay – Mắt (Eye-to-Hand Calibration)**:
   * Camera Zivid đặt cố định nhìn xuống bàn làm việc.
   * Áp dụng thuật toán Tsai-Lenz hoặc Park-Martin thông qua bàn cờ chuẩn (Chessboard/Charuco) để xác định ma trận biến đổi cố định $\mathbf{T}_{Camera}^{Base}$.
3. **Hiệu chuẩn Đầu công tác TCP (Tool Center Point Calibration)**:
   * Xác định offset không gian từ tâm mặt bích trục 6 (Axis 6 Flange) đến đầu súng phun / bút vẽ: $[X_{tool}, Y_{tool}, Z_{tool}]$.

---

## 4. LỘ TRÌNH 7 BƯỚC PHÁT TRIỂN AI PIPELINE

### Bước 1: Tiền xử lý & Tự sinh nhãn từ 3D (Auto-labeling)
* **Ý tưởng cốt lõi**: Thay vì tốn nhân công ngồi gán tay 300 ảnh 2D (dễ sai lệch ở các vùng vải tối màu, mép vải nhăn), hệ thống tận dụng dữ liệu 3D Zivid để **sinh nhãn tự động với độ chính xác tuyệt đối**.
* **Quy trình thực hiện**:
  1. Đọc đồng thời cặp dữ liệu `image_*.png` và `pcl_*.ply`.
  2. Dùng RANSAC 3D bóc tách mặt phẳng bàn gia công: $Ax + By + Cz + D = 0$.
  3. Lọc vật thể có độ cao nổi $H = z_{table} - z_{point} \ge 1.8\text{ mm}$.
  4. Ứng dụng kỹ thuật **3D-Guided GrabCut**: Dùng lõi độ cao 3D ($H \ge 4.5\text{ mm}$) làm hạt giống Foreground chắc chắn, mặt sàn phẳng ($H \le 1.2\text{ mm}$) làm Background, năng lượng biên cắt tối ưu hóa theo độ tương phản gradient RGB.
  5. **Bóc tách 12 Mốc Giải Phẫu Bất Biến Góc Xoay (Invariant Geometry)** theo thuật toán Convexity Defects & Centerline Ray-Casting.
  6. Xuất nhãn:
     * File `.txt` polygon chuẩn **YOLO11-seg**.
     * File `.txt` và `.json` chứa 12 điểm mốc **YOLO11-pose** (chuẩn hóa $[0.0, 1.0]$ kèm trạng thái nhìn thấy $v=2$).
  7. Lưu ảnh overlay kiểm tra trực quan 35 mẫu để kỹ sư thẩm định trước khi đưa vào huấn luyện.

---

### 4.1. CHUẨN HÓA BỘ 12 MỐC GIẢI PHẪU QUẦN ÁO (GARMENT LANDMARKS)

Theo tiêu chuẩn nhân trắc học may mặc công nghiệp và yêu cầu công nghệ của trạm phun sơn/hóa chất tự động Dobot CR10A, hệ thống định nghĩa **12 mốc giải phẫu cốt lõi** chia làm 4 cụm chức năng:

```
                [waist_left] ------- [waist_center] ------- [waist_right]   (Cạp / Lưng quần)
                       \                                   /
                        \            [crotch]             /                 (Đáy đũng)
                         \          /        \           /
                     [thigh_left]  /          \  [thigh_right]              (Vùng đùi - phun râu mèo)
                          |       /            \      |
                          |      /              \     |
                      [knee_left]                [knee_right]               (Đầu gối)
                          |                           |
                 [cuff_L_out] [cuff_L_in]    [cuff_R_in] [cuff_R_out]       (4 góc gấu 2 ống)
```

| Cụm mốc | Tên mốc | Số lượng | Ý nghĩa kỹ thuật cho Robot & Quỹ đạo phun |
| :--- | :--- | :---: | :--- |
| **Cụm Cạp/Lưng (Waist)** | `waist_left`, `waist_center`, `waist_right` | 3 | Xác định bề rộng lưng quần, hướng xoay đầu trên và mặt phẳng lưng. |
| **Cụm Đáy Đũng (Crotch)** | `crotch` | 1 | **Mốc then chốt nhất**: Điểm rẽ nhánh phân tách 2 ống quần, làm gốc tọa độ cục bộ cho toàn bộ thân dưới. |
| **Cụm Đùi (Thigh)** | `thigh_left`, `thigh_right` | 2 | Nằm tại **28% chiều dài ống**: Tâm vùng phun hóa chất PP / phun tạo hiệu ứng cào râu mèo (Whisker effect). |
| **Cụm Đầu Gối (Knee)** | `knee_left`, `knee_right` | 2 | Nằm tại **55% chiều dài ống**: Phân đoạn quỹ đạo ống trên và ống dưới, tính góc uốn cong của chân quần. |
| **Cụm Gấu Quần (Cuff)** | `cuff_left_outer`, `cuff_left_inner`, `cuff_right_inner`, `cuff_right_outer` | 4 | Định vị 2 miệng ống chân, xác định góc mở ống (Leg spread) và chiều dài thực tế từng ống. |

* **16 Cạnh Khung Xương Kết Nối (Skeleton Edges)**:
  `waist_left - waist_center`, `waist_center - waist_right`, `waist_left - crotch`, `waist_right - crotch`, `crotch - thigh_left`, `crotch - thigh_right`, `thigh_left - knee_left`, `thigh_right - knee_right`, `knee_left - cuff_left_outer`, `knee_left - cuff_left_inner`, `knee_right - cuff_right_outer`, `knee_right - cuff_right_inner`, `cuff_left_outer - cuff_left_inner`, `cuff_right_inner - cuff_right_outer`, `waist_left - thigh_left`, `waist_right - thigh_right`.

---

### 4.2. THUẬT TOÁN TỰ ĐỘNG ĐÁNH MỐC BẤT BIẾN GÓC XOAY (INVARIANT GEOMETRY)

#### Nguyên nhân sai lệch ở phương pháp cắt lát pixel cũ
Nếu dùng quy tắc cắt lát theo trục dọc Y của ảnh (`waist_y = y_min + 5%`, `crotch_y = y_min + 45%`...), khi công nhân đặt quần nằm ngang, chéo hoặc xoay bất kỳ góc nào trên bàn, trục Y của ảnh không còn trùng với trục giải phẫu của quần, dẫn đến các mốc bị dồn cục, lộn xộn và sai lệch hoàn toàn.

#### Quy trình 5 bước xác định mốc chuẩn xác bất chấp tư thế đặt
1. **Bước 1: Tìm Đáy Đũng (`crotch`) bằng Khuyết tật Lồi (Convexity Defects)**
   - Đường viền quần luôn có dạng chữ **Y** hoặc chữ **V** ngược.
   - Tính bao lồi (Convex Hull) bao quanh Contour viền ngoài, sau đó tính khoảng cách khuyết tật lồi:
     $$i_{crotch} = \arg\max_i \left( \text{defect}_i.\text{depth} \right)$$
   - Điểm khuyết sâu nhất luôn là **Đáy Đũng**, bất kể chiếc quần xoay theo góc nào trên mặt bàn.
2. **Bước 2: Xác định Trục Ống Chân ($\vec{v}_{leg}$) và Trục Cạp Quần ($\vec{v}_{waist}$)**
   - Hai điểm mút của bao lồi sinh ra khuyết tật này chính là tâm 2 gấu quần ($P_{cuff1}, P_{cuff2}$).
   - Vector từ Đáy Đũng đến trung điểm 2 gấu quần là **Trục Ống Quần**:
     $$\vec{v}_{leg} = \frac{\frac{P_{cuff1} + P_{cuff2}}{2} - P_{crotch}}{\left\| \frac{P_{cuff1} + P_{cuff2}}{2} - P_{crotch} \right\|}$$
   - Vector ngược chiều $180^\circ$ chính là **Trục Cạp Quần**: $\vec{v}_{waist} = -\vec{v}_{leg}$.
   - Vector trực giao: $\vec{v}_{perp} = (-v_{waist, y},\ v_{waist, x})$.
3. **Bước 3: Chiếu tìm Cụm Cạp Quần (Waist)**
   - Chiếu toàn bộ tập điểm viền contour $P$ theo phương $\vec{v}_{waist}$: $d_{proj} = (P - P_{crotch}) \cdot \vec{v}_{waist}$.
   - Lọc nhóm điểm đạt bách phân vị $\ge 97\%$.
   - Chiếu nhóm này lên phương trực giao $\vec{v}_{perp}$ để phân tách 2 góc cạp cực trị:
     * Góc cạp trái: $P_{waist\_left} = \arg\min ((P - P_{crotch}) \cdot \vec{v}_{perp})$.
     * Góc cạp phải: $P_{waist\_right} = \arg\max ((P - P_{crotch}) \cdot \vec{v}_{perp})$.
     * Tâm cạp: $P_{waist\_center} = \frac{P_{waist\_left} + P_{waist\_right}}{2}$.
4. **Bước 4: Bắn tia vuông góc tìm Tim Ống Vải (Centerline Ray-Casting) cho Đùi & Gối**
   - Từ Đáy Đũng đo dọc theo từng ống chân theo tỷ lệ nhân trắc may mặc:
     * $28\%$ chiều dài ống $\rightarrow$ Vị trí sơ bộ Đùi ($P_{thigh}^{seed}$).
     * $55\%$ chiều dài ống $\rightarrow$ Vị trí sơ bộ Đầu gối ($P_{knee}^{seed}$).
   - Tại mỗi vị trí seed, bắn một tia quét hai phía vuông góc với trục ống chân ($\pm \vec{v}_{perp\_leg}$).
   - Thuật toán loang trên mask nhị phân để tìm 2 mép vải thực tế ($P_{outer}, P_{inner}$) và tính trung điểm:
     $$P_{centerline} = \frac{P_{outer} + P_{inner}}{2}$$
   - **Ưu điểm vượt trội**: Điểm mốc luôn nằm **chính xác tại tim giữa của ống vải**, tự động điều chỉnh ngay cả khi chân quần bị cong, nhăn, lệch góc hoặc gấp mép.
5. **Bước 5: Trích xuất Độ cao 3D & Vector Pháp Tuyến Súng Phun $\vec{n}$**
   - Tra cứu tọa độ thực $(X, Y, Z)$ trong camera Zivid 2 M70 (có xử lý nội suy nếu rơi vào điểm khuyết depth).
   - Tính vector pháp tuyến vi mô bề mặt $\vec{n} = (n_x, n_y, n_z)$ bằng SVD/PCA trong lân cận bán kính $15\text{ mm}$ quanh mốc để cung cấp trực tiếp góc xoay định hướng cho đầu súng phun TCP của robot CR10A.

---

### Bước 2: Huấn luyện mô hình thị giác AI (YOLO11-seg & YOLO11-pose)
* **Kiến trúc mô hình**:
  * **YOLO11-seg**: Chuyên biệt phân vùng thân quần (Garment Mask) và các vùng hiệu ứng (mài đùi, túi sau).
  * **YOLO11-pose**: Chuyên biệt bắt 12 điểm mốc giải phẫu.
* **Tăng cường dữ liệu (Data Augmentation)**:
  * Xoay góc $\pm 15^\circ$, điều chỉnh độ sáng/tương phản để thích ứng tốt với cả denim sẫm màu và denim sáng màu (theo góp ý thử nghiệm Zivid HDR).
* **Đánh giá sau huấn luyện**:
  * Đạt $mAP_{50-95} \ge 90\%$ trên tập Validation tách biệt theo phiên chụp.
  * Thời gian suy luận GPU $\le 30\text{ ms}$.

### Bước 3: Chiếu điểm 2D sang 3D (2D-to-3D Projection)
* Nhờ cấu trúc **Organized Point Cloud ($1200 \times 1944$)** của Zivid, mỗi điểm pixel $(u, v)$ từ đường dự đoán 2D lập tức có tọa độ 3D $(X, Y, Z)$ tương ứng.
* Điểm khuyết tật quang học hoặc phản xạ (NaN) được làm đầy bằng thuật toán nội suy mặt phẳng lân cận KDTree ($r \le 5\text{ mm}$).

### Bước 4: Ước lượng pháp tuyến vi mô bề mặt 3D (Surface Normal Estimation)
* Tại mỗi waypoint 3D, thuật toán tìm $K$ điểm lân cận trong bán kính $r \approx 6\text{ mm}$.
* Ma trận hiệp phương sai của cụm điểm được phân tích suy biến SVD/PCA:
  $$\mathbf{C} = \frac{1}{K}\sum_{i=1}^K (\mathbf{p}_i - \bar{\mathbf{p}})(\mathbf{p}_i - \bar{\mathbf{p}})^T$$
* Vector riêng ứng với trị riêng nhỏ nhất chính là vector pháp tuyến $\vec{n} = (n_x, n_y, n_z)$.
* Hướng đầu súng phun luôn được định hướng ngược chiều pháp tuyến: $\vec{z}_{tool} = -\vec{n}$.

### Bước 5: So sánh các phương pháp sinh quỹ đạo (P1, P3, P4)
Theo góp ý của báo cáo, cần so sánh ít nhất 2 đến 3 phương pháp trên cùng bộ dữ liệu:

1. **Phương pháp P1 (Phủ viền / Zigzag Raster)**:
   * Sinh đường viền khép kín co mép $e = 3 - 5\text{ mm}$ hoặc đường điền đầy zíc-zắc bề mặt.
   * *Ưu điểm*: Đơn giản, độ phủ cao. *Nhược điểm*: Không thích ứng theo hoa văn mỹ thuật.
2. **Phương pháp P3 (Sinh quỹ đạo theo Mốc hình học - Landmarks)**:
   * Đường phun nối liền hoặc uốn cong tự nhiên theo các mốc: Đũng quần $\rightarrow$ Đầu gối $\rightarrow$ Gấu quần.
   * *Ưu điểm*: Bám đúng cấu trúc giải phẫu quần áo.
3. **Phương pháp P4 (Đăng ký đám mây điểm phi tuyến - Non-rigid Registration / CPD / TPS)**:
   * Dạy một mẫu hoa văn râu mèo / mài bạc chuẩn trên 1 cỡ mẫu (Template).
   * Khi gặp cỡ quần mới (S, M, L) hoặc vị trí đặt khác: Áp dụng thuật toán **Coherent Point Drift (CPD)** hoặc **Thin Plate Spline (TPS)** để uốn dẻo quỹ đạo mẫu khớp trọn vẹn vào đám mây điểm thực tế.

### Bước 6: Khóa an toàn hình học & Động học (Safety Interlock)
Trước khi xuất lệnh, bộ kiểm tra an toàn sẽ quét toàn bộ quỹ đạo:
* **Kiểm tra va chạm bàn**: Loại bỏ điểm có $Z < 10\text{ mm}$.
* **Kiểm tra tầm với**: Bán kính thao tác $R \le 1175\text{ mm}$.
* **Kiểm tra điểm kỳ dị**: Cận kỳ dị cổ tay ($|q_5| < 3^\circ$), cận kỳ dị khuỷu tay ($|q_3| < 3^\circ$).
* **Quyền chặn xuất lệnh**: Nếu vi phạm các điều kiện trên, hệ thống khóa toàn bộ chức năng gửi lệnh, ngăn ngừa nguy cơ va đập cơ khí.

### Bước 7: Xuất lệnh & Kết nối thời gian thực qua `ai_server.py`
* Server hỗ trợ **kênh kép (Dual-Channel)**:
  * **UDP Socket (Cổng 5005 / 5006)**: Tốc độ cao ($\sim 1\text{ ms}$), không trễ, tự động đẩy dữ liệu sang Unity.
  * **HTTP REST (Cổng 5000)**: Thuận tiện kiểm tra và gỡ lỗi qua API.
* Xuất file điều khiển:
  * Script Python Dobot CR10A (`dobot_cr10a_trajectory_playback.py`).
  * File Action Goal ROS 2 (`dobot_cr10a_ros2_trajectory.yaml`).
  * File CSV tọa độ và góc khớp đã cập nhật (`pants_outline_cr10a_updated.csv`).

---

## 5. KẾ HOẠCH PHÂN KỲ 3 ĐỢT TRIỂN KHAI

* **Đợt 1 — Khép kín Pipeline 2.5D (Mức A - Đang thực hiện)**:
  * Tự sinh nhãn từ dữ liệu 3D cho toàn bộ dataset; xuất kiểm tra trực quan ~35 ảnh.
  * Huấn luyện mô hình YOLO11-seg.
  * Kết nối mô hình vào `ai_server.py`, chạy mô phỏng trọn vẹn trên Unity Workbench 6 Tab và xuất lệnh ra robot thật.
  * Đo đạc và lập bảng số liệu 8 chỉ tiêu kỹ thuật.
* **Đợt 2 — Tự động đổi cỡ quần S/M/L (Mức D)**:
  * Cài đặt thuật toán đăng ký CPD/TPS để tự thích ứng đường râu mèo giữa các cỡ.
  * Bấm giờ so sánh thời gian dạy tay so với tự động sinh quỹ đạo.
* **Đợt 3 — Không gian 3D cong trên khuôn (Mức B/C)**:
  * Ứng dụng khi đưa phôi lên ma-nơ-canh / khuôn đứng xoay nhiều góc.
