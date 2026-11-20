# Hướng Dẫn Sử Dụng Giao Diện Hiệu Chỉnh Mốc Giải Phẫu 3D (Landmark Editor GUI)

Ứng dụng **Landmark Editor** được thiết kế chuyên biệt để kỹ sư trực quan hóa, kiểm tra, hiệu chỉnh thủ công chính xác từng mốc giải phẫu và làm sạch vùng mặt nạ (Mask) trên ảnh phôi vải khi cần thiết.

---

## 1. Cách khởi chạy ứng dụng
Chạy một trong 2 lệnh sau từ terminal:

```bash
# Cách 1: Chạy trực tiếp qua python (trên Ubuntu)
python3 /home/nhan/RB3t_PJ/ai_src/step1_auto_label/landmark_editor_app.py

# Hoặc cách 2: Chạy qua shell script
bash /home/nhan/RB3t_PJ/ai_src/step1_auto_label/run_editor.sh
```

---

## 2. Quy trình làm việc với 8 mốc chuẩn

Toàn bộ khung xương quần gồm 12 mốc giải phẫu. Bạn chỉ cần kiểm soát và định hình đúng **8 mốc chuẩn ngoại vi (Anchors)**:
1. **Cạp trái** (`waist_left`)
2. **Cạp giữa** (`waist_center`): *Điểm tim cạp, bạn có thể tự do kéo chỉnh theo độ trũng thực tế của cạp quần.*
3. **Cạp phải** (`waist_right`)
4. **Đáy đũng** (`crotch`): *Giao điểm giữa hai ống quần.*
5. **Gấu trái ngoài** (`cuff_left_outer`)
6. **Gấu trái trong** (`cuff_left_inner`)
7. **Gấu phải trong** (`cuff_right_inner`)
8. **Gấu phải ngoài** (`cuff_right_outer`)

Sau đó, nhấp vào nút:
👉 **`[⚡ Tự động tính 4 mốc Đùi & Gối từ 8 mốc chuẩn]`**
- Hệ thống sẽ **GIỮ NGUYÊN** toàn bộ 8 mốc chuẩn bạn đã đặt (bao gồm cả Cạp giữa `waist_center`).
- Tự động dùng thuật toán Centerline Ray-Casting chia đôi biên dạng ống vải để tính chuẩn xác 4 mốc tim chân còn lại: `thigh_left`, `knee_left`, `thigh_right`, `knee_right`.
- Đảm bảo điểm nằm chuẩn 100% chính giữa khối vải, triệt tiêu hoàn toàn lỗi lệch sang ống bên cạnh.

---

## 3. Các công cụ chỉnh sửa Mask vải (Mask Editing Tools)

Ở thanh công cụ trên cùng, hàng **🛠 Công cụ Mask** bao gồm:

### ✏️ Bút Vẽ Mask (Paint Brush) - Kéo tới đâu tô thêm tới đó:
- **Kích hoạt**: Bấm nút `[✏️ Bút Vẽ Mask (Tô thêm)]` (nút sáng màu xanh ngọc lục bảo Emerald, trên ảnh xuất hiện vòng tròn nét đứt màu xanh thể hiện chính xác bán kính cọ vẽ).
- **Thao tác**: Nhấp giữ chuột trái và kéo trên ảnh $\rightarrow$ **Vẽ bù thêm các vùng vải bị thiếu, mép cạp, gấu quần hoặc nếp gấp** mà thuật toán tự động bỏ sót.

### 🧹 Bút Tẩy Mask (Brush Eraser) - Kéo tới đâu xóa tới đó:
- **Kích hoạt**: Bấm nút `[🧹 Bút Tẩy Mask (Kéo xóa)]` (nút chuyển sang màu đỏ rực, trên ảnh xuất hiện vòng tròn nét đứt màu đỏ thể hiện chính xác bán kính cọ tẩy).
- **Chỉnh cỡ bút**: Dùng chung thanh trượt `Cỡ bút: [-----o---] 25 px` (tùy chỉnh linh hoạt từ **5 px đến 150 px** cho cả Bút Vẽ và Bút Tẩy).
- **Thao tác**: Nhấp giữ chuột trái và kéo di chuyển chuột trên ảnh $\rightarrow$ **Kéo tới đâu, vệt cọ sẽ tẩy sạch vùng mask tới đó** với tốc độ thời gian thực 60+ FPS siêu mượt!

### 🗑️ Click Xóa Cả Vùng Liên Thông:
- Bấm nút `[🗑 Click Xóa Cả Vùng]`, sau đó chỉ cần **CLICK 1 cái** vào bất kỳ đốm mask rác nào $\rightarrow$ Toàn bộ đốm rác đó sẽ biến mất ngay lập tức!

### 🧹 Tự Động Xóa Rác (Giữ 1 Thân Quần):
- Bấm nút `[🧹 Giữ 1 Thân Quần]` $\rightarrow$ Hệ thống tự động lọc bỏ mọi đốm rác xung quanh, chỉ giữ lại duy nhất 1 thân quần lớn nhất.

### ↺ Hoàn Tác (Undo):
- Bấm `[↺ Hoàn Tác (Undo)]` để khôi phục lại trạng thái mask nếu bạn lỡ tay vẽ thừa hoặc tẩy nhầm.

---

## 4. Khung làm việc Canvas & Lưu nhãn
- **Kéo thả chuột (Drag & Drop)**: Đưa chuột vào 12 vòng tròn mốc giải phẫu và kéo tới vị trí mong muốn.
- **Thu phóng & Kéo trượt (Zoom & Pan)**: Cuộn chuột để phóng to/thu nhỏ, giữ chuột giữa hoặc chuột phải để Pan kéo trượt khung nhìn.
- **💾 Lưu Nhãn**: Tự động ghi đè JSON 3D, nhãn YOLO11-seg (đã làm sạch mask), nhãn YOLO11-pose và ảnh verification overlay.
