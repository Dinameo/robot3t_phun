# Chụp Zivid trực tiếp trong project Downloads

Project đích: `C:\Users\Admin\Downloads\robot3t_phun-main\robot3t_phun-main`.

1. Trong Unity đang mở, lưu scene và thoát Play Mode. Không cần đổi project.
2. Trong Zivid Studio, Disconnect camera (hoặc đóng Studio) để Python có thể lấy quyền sử dụng camera; giữ nguyên nguồn và dây Ethernet.
3. Mở PowerShell, chạy:

```powershell
& "C:\Users\Admin\Documents\ChatGPT\Work\denim_ai\.venv\Scripts\python.exe" "C:\Users\Admin\Documents\ChatGPT\Work\CR10A_Downloads_Patch\apply_downloads_patch.py"
```

Lệnh này kiểm tra đúng project, sao lưu các file cũ, áp dụng bản vá và khởi động backend ẩn. Không thay scene hoặc model robot. Nếu file nguồn đã thay đổi ngoài dự kiến, lệnh dừng và không ghi đè.
Chờ dòng `READY`. Nếu cổng 5000 đang có server khác, dừng server cũ bằng Ctrl+C rồi chạy lại; bản vá không tự dừng tiến trình khác.

4. Trở lại chính Unity đang mở trong Downloads, chờ biên dịch rồi bật Play.
5. Vào tab **1 Nạp phôi**. Nếu bảng phải bị ẩn, bấm **Mở chi tiết**.
6. Bấm **CHỤP ZIVID → HIỂN THỊ PHÔI 3D**. Dữ liệu mới phải có tên `pcl_live_<timestamp>`. Nút này chỉ chụp/hiển thị, không dùng dữ liệu mẫu và không cần AI hay hiệu chuẩn robot.
7. Khi phôi thật đã hiện đúng, bấm **CHỤP + AI → TẠO QUỸ ĐẠO MÔ PHỎNG**. Nếu model nhận diện được và các điểm có nghiệm IK, Unity nạp kết quả mới. Sau đó vào Mô phỏng, chọn 0.5× và Chạy.

## Giới hạn cần phân biệt

- Dữ liệu đầu vào là camera thật; vị trí đặt dữ liệu trong scene hiện dùng camera rig mô phỏng nếu chưa có hiệu chuẩn.
- Mô phỏng chạy được không chứng minh tọa độ đã đúng cho robot thật.
- Bản này không phát lệnh chuyển động cho robot vật lý. Dùng camera dẫn đường robot thật cần hiệu chuẩn camera–robot, TCP và kiểm chứng sai số trên các điểm độc lập.
- Camera serial được đặt là `21523184`, đúng camera đã hiện trong Zivid Studio.
- Zivid Python 2.18.0 đã được cài vào Python 3.12 của project. Adapter bổ sung đường dẫn DLL trong tiến trình, không sửa cấu hình PATH toàn hệ thống.

File chụp nằm trong `ai_src/dataset/_3D` (`.zdf`, `.ply`, `.npz`, `.frame.json`). Log backend nằm ở `runtime/camera_server.log` trong project Downloads.

## Kiểm tra đã thực hiện

- Import được Zivid Python 2.18.0 với native SDK đã cài trên máy.
- Biên dịch hai file C# đã vá với tham chiếu Unity của project hiện tại.
- Kiểm tra preview trả frame mới mà không yêu cầu model/hiệu chuẩn và không trả quỹ đạo cũ.
- Kiểm tra bộ áp dụng bản vá có sao lưu, chạy lại được và từ chối ghi đè file đã sửa ngoài dự kiến.
- Chưa kiểm chứng camera chụp thật trong Unity sau khi áp dụng: bước áp dụng/khởi động trên Windows phải do người dùng chạy vì giới hạn quyền của phiên.
