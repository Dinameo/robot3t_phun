# Dashboard AI Server

Chạy `START_AI_SERVER.cmd` ở thư mục gốc, sau đó mở [dashboard](http://127.0.0.1:5000/dashboard) trên máy chạy server. Dashboard dùng chung tiến trình và cổng HTTP với API của Unity.

Đăng nhập web bằng tài khoản `admin`. Nếu đã đặt biến môi trường `DASHBOARD_PASSWORD`, dùng mật khẩu đó. Nếu để trống, server tạo mật khẩu mới khi khởi động và in ở dòng `Dashboard login | admin | …` trong terminal. Phiên web hết hạn sau 8 giờ hoặc khi bấm **Đăng xuất**. Các thiết lập kết nối Unity vẫn dùng như trước.

Trong **Model YOLO**, chọn trọng số Segmentation và Pose rồi bấm **Nạp model**. Danh sách lấy từ `ai_src/step2_train_yolo/weights`. Có thể thêm trọng số `.pt` từ nguồn tin cậy vào thư mục này, rồi chọn trên dashboard. Hai model được đổi cùng lúc sau khi nạp thành công. Nếu nạp lỗi, server tiếp tục dùng cặp đang chạy. Khi khởi động lại, server nạp cặp mặc định.

**Log máy chủ** hiển thị 500 dòng gần nhất của tiến trình, gồm log Python và nội dung in ra terminal. **Xóa hiển thị** chỉ dọn danh sách trong trình duyệt. Dashboard và API quản trị truy cập từ máy chạy server.
