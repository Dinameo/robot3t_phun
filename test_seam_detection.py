import os
import sys
import cv2
import numpy as np

# Thêm đường dẫn gốc để import PantsDetector3D
sys.path.insert(0, r"d:\RB3t_PJ")
from detect_pants_3d import PantsDetector3D

def inspect_seams(ply_path, output_dir=r"d:\RB3t_PJ\output_seams"):
    os.makedirs(output_dir, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(ply_path))[0]
    
    detector = PantsDetector3D()
    xyz, bgr, rgb = detector.load_ply(ply_path)
    h_img, w_img = bgr.shape[:2]
    
    # 1. Phát hiện phôi & bản đồ độ cao
    plane_params = detector.fit_dominant_plane(xyz)
    mask = detector.segment_pants(xyz, bgr, plane_params)
    
    a, b_coef, c_plane = plane_params['a'], plane_params['b'], plane_params['c']
    x, y, z = xyz[:, :, 0], xyz[:, :, 1], xyz[:, :, 2]
    z_table = a * x + b_coef * y + c_plane
    height = z_table - z
    height[z == 0] = 0
    
    # 2. Phân tích vùng ống chân (Leg panels)
    # Lấy gradient độ cao 3D (đường may nổi gờ 3D)
    sobel_hx = cv2.Sobel(height, cv2.CV_32F, 1, 0, ksize=3)
    sobel_hy = cv2.Sobel(height, cv2.CV_32F, 0, 1, ksize=3)
    height_grad = np.sqrt(sobel_hx**2 + sobel_hy**2)
    height_grad[mask == 0] = 0
    
    # 3. Phân tích độ tương phản 2D (Đường chỉ may / Rãnh sườn)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    enhanced_gray = clahe.apply(gray)
    
    # Cạnh màu (Canny trên vùng vải)
    edges_2d = cv2.Canny(enhanced_gray, 50, 150)
    edges_2d[mask == 0] = 0
    
    # 4. Tìm kiếm gờ nổi (Ridge / Seam) gần 2 mép
    # Chuẩn hóa gradient 3D
    norm_h_grad = np.clip((height_grad / 5.0) * 255, 0, 255).astype(np.uint8)
    
    # Kết hợp thông tin 2D và 3D
    # Đường may vừa có gờ 3D vừa có vân chỉ 2D
    combined_seam_score = cv2.addWeighted(edges_2d, 0.5, norm_h_grad, 0.5, 0)
    combined_seam_score[mask == 0] = 0
    
    # 5. Vẽ trực quan hóa
    vis = bgr.copy()
    # Tô màu vàng cho các gân / đường may phát hiện được
    seam_candidates = combined_seam_score > 80
    vis[seam_candidates] = [0, 255, 255]
    
    # Lưu kết quả
    cv2.imwrite(os.path.join(output_dir, f"{base_name}_seam_vis.png"), vis)
    cv2.imwrite(os.path.join(output_dir, f"{base_name}_height_grad.png"), norm_h_grad)
    cv2.imwrite(os.path.join(output_dir, f"{base_name}_edges_2d.png"), edges_2d)
    
    print(f"[DONE] Seam inspection saved to {output_dir}")

if __name__ == "__main__":
    test_ply = r"d:\RB3t_PJ\dataset\_3D\pcl_18.ply"
    inspect_seams(test_ply)
