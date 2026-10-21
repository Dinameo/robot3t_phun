"""
Package trích xuất Mốc hình học 3D (3D Geometric Landmarks) cho sản phẩm may mặc.
Phục vụ phương pháp P3: Quỹ đạo mẫu theo mốc & Biến đổi affine từng mảnh sang cỡ quần mới.
"""
from .extractor import PantsLandmarks3D

__all__ = ['PantsLandmarks3D']
