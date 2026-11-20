#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
CÔNG TY CỔ PHẦN CHẾ TẠO MÁY 3C / ROBOT 3T - DỰ ÁN DOBOT CR10A
GIAO DIỆN HIỆU CHỈNH MỐC GIẢI PHẪU 3D & XUẤT NHÃN YOLO (LANDMARK EDITOR GUI)
=============================================================================
Tính năng:
1. Duyệt trực quan danh sách mẫu 3D / 2D Zivid (pcl_1, pcl_10, ...).
2. Hiển thị ảnh phôi vải, mặt nạ phân vùng và 12 mốc giải phẫu + khung xương.
3. Cho phép KÉO THẢ (Drag & Drop) hoặc CLICK để chỉnh sửa vị trí từng mốc:
   - 7 mốc biên: 2 mép gấu ống trái, 2 mép gấu ống phải, đáy đũng, 2 góc cạp.
   - 5 mốc trung gian: tim cạp, đùi trái, gối trái, đùi phải, gối phải.
4. Tự động tính toán lại tim ống chân theo Centerline Ray-Casting khi cần.
5. Cập nhật tức thời tọa độ 3D (X, Y, Z), Vector pháp tuyến và độ cao nổi (mm).
6. Nút LƯU tự động ghi đè:
   - File JSON 3D landmarks
   - File nhãn YOLO11-pose (.txt)
   - Ảnh thẩm định trực quan Verification Overlay (.jpg)
=============================================================================
"""

import os
import sys
import json
import glob
import time
from pathlib import Path

# Đảm bảo đường dẫn tới module AutoLabeling3D
CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import numpy as np
import cv2

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsEllipseItem,
    QGraphicsLineItem, QGraphicsTextItem, QGraphicsPixmapItem,
    QLabel, QPushButton, QComboBox, QListWidget, QListWidgetItem,
    QGroupBox, QCheckBox, QMessageBox, QSplitter, QStatusBar,
    QFrame, QSizePolicy, QToolTip, QSlider, QSpinBox
)
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal, QObject
from PyQt5.QtGui import (
    QImage, QPixmap, QColor, QPen, QBrush, QFont, QCursor, QPainter
)

from step1_auto_labeling_3d import AutoLabeling3D


class LandmarkItem(QGraphicsEllipseItem):
    """Điểm mốc giải phẫu tương tác, hỗ trợ kéo thả và hiển thị nhãn tên."""

    # Màu sắc nhận diện theo từng nhóm giải phẫu
    GROUP_COLORS = {
        "waist": QColor(0, 220, 255),      # Xanh cyan cho cạp quần
        "crotch": QColor(255, 60, 60),     # Đỏ tươi cho đáy đũng
        "leg_left": QColor(50, 255, 100),  # Xanh lá cho ống trái (trên)
        "leg_right": QColor(255, 180, 0),  # Cam vàng cho ống phải (dưới)
    }

    def __init__(self, name, u, v, parent_gui, radius=9.0):
        super().__init__(-radius, -radius, 2 * radius, 2 * radius)
        self.name = name
        self.parent_gui = parent_gui
        self.radius = radius
        self.setPos(u, v)

        # Cờ tương tác
        self.setFlags(
            QGraphicsItem.ItemIsMovable |
            QGraphicsItem.ItemIsSelectable |
            QGraphicsItem.ItemSendsGeometryChanges
        )
        self.setAcceptHoverEvents(True)
        self.setCursor(QCursor(Qt.OpenHandCursor))
        self.setZValue(20.0)

        # Xác định nhóm màu
        if "waist" in name:
            self.base_color = self.GROUP_COLORS["waist"]
        elif name == "crotch":
            self.base_color = self.GROUP_COLORS["crotch"]
        elif "left" in name:
            self.base_color = self.GROUP_COLORS["leg_left"]
        else:
            self.base_color = self.GROUP_COLORS["leg_right"]

        self.update_style(selected=False)

        # Nhãn text tên mốc
        self.text_item = QGraphicsTextItem(name, self)
        self.text_item.setDefaultTextColor(Qt.white)
        font = QFont("Monospace", 9, QFont.Bold)
        self.text_item.setFont(font)
        self.text_item.setPos(12, -10)
        self.text_item.setZValue(21.0)

    def update_style(self, selected=False, hovered=False):
        color = QColor(255, 255, 255) if selected else (self.base_color.lighter(130) if hovered else self.base_color)
        pen_width = 3.0 if (selected or hovered) else 2.0
        self.setPen(QPen(Qt.black, pen_width))
        self.setBrush(QBrush(color))

    def hoverEnterEvent(self, event):
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self.update_style(selected=self.isSelected(), hovered=True)
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self.setCursor(QCursor(Qt.OpenHandCursor))
        self.update_style(selected=self.isSelected(), hovered=False)
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event):
        # Nếu đang ở chế độ cọ xóa, cọ vẽ hoặc click xóa mask, nhường sự kiện cho canvas xử lý mask
        if self.parent_gui and (getattr(self.parent_gui, 'is_delete_mask_mode', False) or 
                                 getattr(self.parent_gui, 'is_brush_mode', False) or 
                                 getattr(self.parent_gui, 'is_paint_mode', False)):
            event.ignore()
            return

        if event.button() == Qt.LeftButton:
            self.setCursor(QCursor(Qt.ClosedHandCursor))
            self.parent_gui.on_landmark_selected(self.name)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.setCursor(QCursor(Qt.OpenHandCursor))
        super().mouseReleaseEvent(event)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange and self.scene():
            new_pos = value
            # Giới hạn tọa độ trong phạm vi ảnh
            w = self.parent_gui.img_w
            h = self.parent_gui.img_h
            clamped_x = max(0.0, min(float(w - 1), new_pos.x()))
            clamped_y = max(0.0, min(float(h - 1), new_pos.y()))
            clamped_pos = QPointF(clamped_x, clamped_y)

            # Cập nhật đường xương và thanh thông tin
            self.parent_gui.on_landmark_moved(self.name, clamped_x, clamped_y)
            return clamped_pos

        return super().itemChange(change, value)


class SkeletonLine(QGraphicsLineItem):
    """Đoạn thẳng nối giữa 2 mốc giải phẫu tự co dãn theo vị trí điểm."""

    def __init__(self, item1, item2):
        super().__init__()
        self.item1 = item1
        self.item2 = item2
        self.setPen(QPen(QColor(0, 255, 255, 220), 2.5, Qt.SolidLine, Qt.RoundCap))
        self.setZValue(10.0)
        self.update_position()

    def update_position(self):
        p1 = self.item1.pos()
        p2 = self.item2.pos()
        self.setLine(p1.x(), p1.y(), p2.x(), p2.y())


class GarmentCanvasView(QGraphicsView):
    """Khung vẽ hiển thị ảnh phôi vải, hỗ trợ thu phóng (Zoom), kéo trượt (Pan) và Cọ Tẩy Mask."""

    def __init__(self, scene, parent_gui=None):
        super().__init__(scene, parent_gui)
        self.parent_gui = parent_gui
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QBrush(QColor(18, 20, 24)))
        self.setMouseTracking(True)
        self._is_panning = False
        self._pan_start = None
        self._is_brushing = False
        self._is_painting = False
        self._last_brush_pos = None

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        # 1. Chế độ Bút Vẽ/Tô Mask (Paint Brush - Thêm vùng)
        if self.parent_gui and getattr(self.parent_gui, 'is_paint_mode', False) and event.button() == Qt.LeftButton:
            self._is_painting = True
            scene_pos = self.mapToScene(event.pos())
            self._last_brush_pos = scene_pos
            self.parent_gui.push_mask_history()
            self.parent_gui.paint_mask_stroke(scene_pos, scene_pos, quick=True)
            event.accept()
            return

        # 2. Chế độ Bút Tẩy Mask (Brush Eraser - Xóa vùng)
        if self.parent_gui and getattr(self.parent_gui, 'is_brush_mode', False) and event.button() == Qt.LeftButton:
            self._is_brushing = True
            scene_pos = self.mapToScene(event.pos())
            self._last_brush_pos = scene_pos
            self.parent_gui.push_mask_history()
            self.parent_gui.erase_mask_stroke(scene_pos, scene_pos, quick=True)
            event.accept()
            return

        # 3. Chế độ Click Xóa Cả Vùng Liên Thông
        if self.parent_gui and getattr(self.parent_gui, 'is_delete_mask_mode', False) and event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            u = int(round(scene_pos.x()))
            v = int(round(scene_pos.y()))
            if self.parent_gui.delete_mask_region_at(u, v):
                event.accept()
                return

        # 4. Giữ chuột giữa hoặc chuột phải để Pan di chuyển khung nhìn
        if event.button() == Qt.MiddleButton or (event.button() == Qt.RightButton and not self.itemAt(event.pos())):
            self._is_panning = True
            self._pan_start = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.pos())

        # Cập nhật vòng tròn xem trước bán kính cọ (Brush Preview Circle)
        if self.parent_gui and (getattr(self.parent_gui, 'is_brush_mode', False) or getattr(self.parent_gui, 'is_paint_mode', False)):
            self.parent_gui.update_brush_cursor(scene_pos)

        # Xử lý kéo cọ vẽ thêm mask
        if self._is_painting:
            self.parent_gui.paint_mask_stroke(self._last_brush_pos, scene_pos, quick=True)
            self._last_brush_pos = scene_pos
            event.accept()
            return

        # Xử lý kéo cọ tẩy xóa mask
        if self._is_brushing:
            self.parent_gui.erase_mask_stroke(self._last_brush_pos, scene_pos, quick=True)
            self._last_brush_pos = scene_pos
            event.accept()
            return

        # Xử lý Pan khung nhìn
        if self._is_panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._is_painting and event.button() == Qt.LeftButton:
            self._is_painting = False
            self.parent_gui.update_canvas_image(quick=False)
            self.parent_gui.set_dirty(True)
            event.accept()
            return

        if self._is_brushing and event.button() == Qt.LeftButton:
            self._is_brushing = False
            self.parent_gui.update_canvas_image(quick=False)
            self.parent_gui.set_dirty(True)
            event.accept()
            return

        if self._is_panning and (event.button() == Qt.MiddleButton or event.button() == Qt.RightButton):
            self._is_panning = False
            if self.parent_gui and (getattr(self.parent_gui, 'is_brush_mode', False) or getattr(self.parent_gui, 'is_paint_mode', False)):
                self.setCursor(Qt.BlankCursor)
            elif self.parent_gui and getattr(self.parent_gui, 'is_delete_mask_mode', False):
                self.setCursor(Qt.CrossCursor)
            else:
                self.setCursor(Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def leaveEvent(self, event):
        if self.parent_gui and hasattr(self.parent_gui, 'brush_cursor_item'):
            self.parent_gui.brush_cursor_item.setVisible(False)
        super().leaveEvent(event)


class LandmarkEditorApp(QMainWindow):
    """Giao diện chính chỉnh sửa mốc giải phẫu 3D quần Zivid."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DOBOT CR10A - BƯỚC 1: GIAO DIỆN HIỆU CHỈNH MỐC GIẢI PHẪU 3D (LANDMARK EDITOR)")
        self.resize(1600, 960)

        # Khởi tạo lõi AutoLabeling3D
        self.core = AutoLabeling3D()
        self.dataset_dir = self.core.dataset_3d_dir
        self.ply_files = sorted(list(self.dataset_dir.glob("*.ply")))

        # Dữ liệu mẫu hiện tại
        self.current_idx = 0
        self.current_stem = None
        self.xyz = None
        self.bgr = None
        self.mask = None
        self.plane_params = None
        self.img_w = 1944
        self.img_h = 1200
        self.landmarks_data = {}
        self.landmark_items = {}
        self.skeleton_lines = []
        self.selected_landmark = None
        self.is_dirty = False
        self.is_delete_mask_mode = False
        self.is_brush_mode = False
        self.is_paint_mode = False
        self.brush_radius = 25
        self.mask_history = []

        self.setup_ui()
        self.apply_dark_theme()

        if self.ply_files:
            self.load_sample(0)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # -------------------------------------------------------------
        # KHUNG BÊN TRÁI: KHUNG VẼ CANVAS VÀ ĐIỀU KHIỂN VIEW
        # -------------------------------------------------------------
        left_box = QWidget()
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)

        # Thanh điều hướng mẫu trên cùng (2 hàng: Điều hướng & Công cụ Mask)
        top_bar = QFrame()
        top_bar.setObjectName("topBar")
        top_bar_layout = QVBoxLayout(top_bar)
        top_bar_layout.setContentsMargins(8, 6, 8, 6)
        top_bar_layout.setSpacing(6)

        # Hàng 1: Điều hướng mẫu và hiển thị
        row1_layout = QHBoxLayout()
        row1_layout.setSpacing(8)

        self.btn_prev = QPushButton("◀ Mẫu trước")
        self.btn_prev.clicked.connect(self.prev_sample)
        row1_layout.addWidget(self.btn_prev)

        lbl_select = QLabel("Chọn mẫu phôi:")
        row1_layout.addWidget(lbl_select)

        self.combo_samples = QComboBox()
        for p in self.ply_files:
            self.combo_samples.addItem(p.stem)
        self.combo_samples.currentIndexChanged.connect(self.on_combo_sample_changed)
        row1_layout.addWidget(self.combo_samples, stretch=1)

        self.btn_next = QPushButton("Mẫu sau ▶")
        self.btn_next.clicked.connect(self.next_sample)
        row1_layout.addWidget(self.btn_next)

        row1_layout.addSpacing(10)

        self.chk_show_mask = QCheckBox("Hiện Mask vải 3D")
        self.chk_show_mask.setChecked(True)
        self.chk_show_mask.toggled.connect(self.on_toggle_mask)
        row1_layout.addWidget(self.chk_show_mask)

        self.chk_show_skeleton = QCheckBox("Hiện Khung xương")
        self.chk_show_skeleton.setChecked(True)
        self.chk_show_skeleton.toggled.connect(self.on_toggle_skeleton)
        row1_layout.addWidget(self.chk_show_skeleton)

        self.btn_fit_view = QPushButton("Fit 100%")
        self.btn_fit_view.clicked.connect(self.fit_view)
        row1_layout.addWidget(self.btn_fit_view)

        top_bar_layout.addLayout(row1_layout)

        # Hàng 2: Công cụ chỉnh sửa Mask (Bút Vẽ thêm, Bút Tẩy xóa, Click xóa vùng, Xóa rác tự động)
        row2_layout = QHBoxLayout()
        row2_layout.setSpacing(8)

        lbl_mask_tools = QLabel("<font color='#38bdf8'><b>🛠 Công cụ Mask:</b></font>")
        row2_layout.addWidget(lbl_mask_tools)

        # 1. Bút Tô / Vẽ Mask (Paint Brush - Thêm vùng)
        self.btn_paint_mode = QPushButton("✏️ Bút Vẽ Mask (Tô thêm)")
        self.btn_paint_mode.setObjectName("btn_paint_mode")
        self.btn_paint_mode.setCheckable(True)
        self.btn_paint_mode.setToolTip("Bật bút vẽ: Kéo chuột trên ảnh để vẽ bù thêm các vùng mask bị thiếu. Có thể chỉnh cỡ bút.")
        self.btn_paint_mode.toggled.connect(self.toggle_paint_mode)
        row2_layout.addWidget(self.btn_paint_mode)

        # 2. Bút Tẩy Mask (Brush Eraser - Xóa vùng)
        self.btn_brush_mode = QPushButton("🧹 Bút Tẩy Mask (Kéo xóa)")
        self.btn_brush_mode.setObjectName("btn_brush_mode")
        self.btn_brush_mode.setCheckable(True)
        self.btn_brush_mode.setToolTip("Bật bút tẩy: Kéo chuột trên ảnh để tẩy sạch mask bị thừa/lỗi. Có thể chỉnh cỡ bút.")
        self.btn_brush_mode.toggled.connect(self.toggle_brush_mode)
        row2_layout.addWidget(self.btn_brush_mode)

        lbl_radius = QLabel("Cỡ bút:")
        row2_layout.addWidget(lbl_radius)

        self.slider_brush = QSlider(Qt.Horizontal)
        self.slider_brush.setRange(5, 150)
        self.slider_brush.setValue(25)
        self.slider_brush.setFixedWidth(100)
        self.slider_brush.setToolTip("Kéo thanh trượt để chỉnh bán kính cọ vẽ/tẩy (5 - 150 px)")
        row2_layout.addWidget(self.slider_brush)

        self.spin_brush = QSpinBox()
        self.spin_brush.setRange(5, 150)
        self.spin_brush.setValue(25)
        self.spin_brush.setSuffix(" px")
        self.spin_brush.setFixedWidth(70)
        row2_layout.addWidget(self.spin_brush)

        self.slider_brush.valueChanged.connect(self.spin_brush.setValue)
        self.spin_brush.valueChanged.connect(self.slider_brush.setValue)
        self.slider_brush.valueChanged.connect(self.on_brush_radius_changed)

        row2_layout.addSpacing(8)

        # 3. Click xóa cả vùng liên thông
        self.btn_delete_mask_mode = QPushButton("🗑 Click Xóa Cả Vùng")
        self.btn_delete_mask_mode.setObjectName("btn_delete_mask_mode")
        self.btn_delete_mask_mode.setCheckable(True)
        self.btn_delete_mask_mode.setToolTip("Click chuột trái trực tiếp vào vùng mask phụ/rác để xóa sạch toàn bộ vùng đó.")
        self.btn_delete_mask_mode.toggled.connect(self.toggle_delete_mask_mode)
        row2_layout.addWidget(self.btn_delete_mask_mode)

        # 4. Tự động giữ 1 thân quần lớn nhất
        self.btn_keep_largest_mask = QPushButton("🧹 Giữ 1 Thân Quần")
        self.btn_keep_largest_mask.setToolTip("Tự động xóa sạch mọi vùng rác nhỏ lẻ xung quanh, chỉ giữ lại thân quần duy nhất.")
        self.btn_keep_largest_mask.clicked.connect(self.keep_largest_mask)
        row2_layout.addWidget(self.btn_keep_largest_mask)

        # 5. Hoàn tác
        self.btn_undo_mask = QPushButton("↺ Hoàn Tác (Undo)")
        self.btn_undo_mask.setToolTip("Khôi phục lại vùng mask nếu lỡ tay thao tác nhầm.")
        self.btn_undo_mask.clicked.connect(self.undo_mask)
        row2_layout.addWidget(self.btn_undo_mask)

        row2_layout.addStretch(1)
        top_bar_layout.addLayout(row2_layout)

        left_layout.addWidget(top_bar)

        # Canvas Graphics Scene & View
        self.scene = QGraphicsScene(self)
        self.view = GarmentCanvasView(self.scene, self)
        self.pixmap_item = QGraphicsPixmapItem()
        self.scene.addItem(self.pixmap_item)

        # Vòng tròn xem trước cọ tẩy (Brush preview cursor)
        self.brush_cursor_item = QGraphicsEllipseItem()
        self.brush_cursor_item.setPen(QPen(QColor(255, 60, 60, 240), 2, Qt.DashLine))
        self.brush_cursor_item.setBrush(QBrush(QColor(255, 0, 0, 35)))
        self.brush_cursor_item.setZValue(100.0)
        self.brush_cursor_item.setVisible(False)
        self.scene.addItem(self.brush_cursor_item)

        left_layout.addWidget(self.view, stretch=1)

        # -------------------------------------------------------------
        # KHUNG BÊN PHẢI: BẢNG DANH SÁCH MỐC VÀ CÁC THAO TÁC LƯU/TÍNH TOÁN
        # -------------------------------------------------------------
        right_box = QWidget()
        right_box.setFixedWidth(440)
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        # Group 1: Thông tin mẫu
        grp_info = QGroupBox("THÔNG TIN MẪU 3D")
        grp_info_layout = QVBoxLayout(grp_info)
        self.lbl_sample_info = QLabel("Đang tải...")
        self.lbl_sample_info.setWordWrap(True)
        grp_info_layout.addWidget(self.lbl_sample_info)
        right_layout.addWidget(grp_info)

        # Group 2: Danh sách 12 mốc giải phẫu
        grp_lm = QGroupBox("DANH SÁCH 12 MỐC GIẢI PHẪU (CHỌN ĐỂ XEM)")
        grp_lm_layout = QVBoxLayout(grp_lm)
        self.list_landmarks = QListWidget()
        self.list_landmarks.itemClicked.connect(self.on_list_item_clicked)
        grp_lm_layout.addWidget(self.list_landmarks)

        # Chi tiết mốc đang chọn
        self.lbl_lm_details = QLabel("Click vào một mốc trên danh sách hoặc ảnh...")
        self.lbl_lm_details.setStyleSheet("background: #252830; padding: 8px; border-radius: 4px; font-family: Monospace;")
        self.lbl_lm_details.setWordWrap(True)
        grp_lm_layout.addWidget(self.lbl_lm_details)

        right_layout.addWidget(grp_lm, stretch=1)

        # Group 3: Các chức năng tự động hỗ trợ
        grp_tools = QGroupBox("CÔNG CỤ HÌNH HỌC TỰ ĐỘNG")
        grp_tools_layout = QVBoxLayout(grp_tools)

        self.btn_recalc_derived = QPushButton("⚡ Tự động tính 4 mốc Đùi & Gối từ 8 mốc chuẩn")
        self.btn_recalc_derived.setToolTip("Giữ nguyên 8 mốc chuẩn do bạn định hình (3 mốc Cạp: Trái/Giữa/Phải, 1 mốc Đáy đũng, 4 mốc Gấu: 2 mép gấu trái + 2 mép gấu phải) và tự động tính chuẩn xác 4 mốc Đùi trái, Gối trái, Đùi phải, Gối phải theo tim ống vải.")
        self.btn_recalc_derived.clicked.connect(self.recalc_derived_landmarks)
        grp_tools_layout.addWidget(self.btn_recalc_derived)

        self.btn_reset = QPushButton("↺ Đặt lại mốc ban đầu (Reset)")
        self.btn_reset.clicked.connect(self.reset_current_sample)
        grp_tools_layout.addWidget(self.btn_reset)

        right_layout.addWidget(grp_tools)

        # Group 4: Nút LƯU KẾT QUẢ
        grp_save = QGroupBox("LƯU KẾT QUẢ HIỆU CHỈNH")
        grp_save_layout = QVBoxLayout(grp_save)

        self.btn_save = QPushButton("💾 LƯU NHÃN (JSON, YOLO, OVERLAY)")
        self.btn_save.setObjectName("saveButton")
        self.btn_save.setMinimumHeight(44)
        self.btn_save.clicked.connect(self.save_current_sample)
        grp_save_layout.addWidget(self.btn_save)

        self.lbl_save_status = QLabel("Trạng thái: Đã đồng bộ")
        self.lbl_save_status.setAlignment(Qt.AlignCenter)
        grp_save_layout.addWidget(self.lbl_save_status)

        right_layout.addWidget(grp_save)

        # Ghép vào Layout chính
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_box)
        splitter.addWidget(right_box)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        main_layout.addWidget(splitter)

        # Status Bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Sẵn sàng. Kéo thả các vòng tròn đỏ/vàng trên ảnh để chỉnh sửa vị trí mốc.")

    def apply_dark_theme(self):
        """Áp dụng giao diện Industrial Dark Theme chuyên nghiệp."""
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1a1c23;
                color: #e2e8f0;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 13px;
            }
            QFrame#topBar {
                background-color: #242832;
                border: 1px solid #333a48;
                border-radius: 6px;
            }
            QGroupBox {
                border: 1px solid #333a48;
                border-radius: 6px;
                margin-top: 14px;
                padding-top: 10px;
                font-weight: bold;
                color: #38bdf8;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                background-color: #333a48;
                color: #f1f5f9;
                border: 1px solid #475569;
                border-radius: 4px;
                padding: 6px 14px;
                font-weight: 500;
            }
            QPushButton:hover {
                background-color: #475569;
                border-color: #38bdf8;
            }
            QPushButton:pressed {
                background-color: #1e293b;
            }
            QPushButton:checked {
                background-color: #2563eb;
                color: #ffffff;
                border: 2px solid #60a5fa;
                font-weight: bold;
            }
            QPushButton#btn_paint_mode:checked {
                background-color: #059669;
                color: #ffffff;
                border: 2px solid #34d399;
                font-weight: bold;
            }
            QPushButton#btn_brush_mode:checked {
                background-color: #dc2626;
                color: #ffffff;
                border: 2px solid #f87171;
                font-weight: bold;
            }
            QPushButton#btn_delete_mask_mode:checked {
                background-color: #b91c1c;
                color: #ffffff;
                border: 2px solid #fca5a5;
                font-weight: bold;
            }
            QPushButton#saveButton {
                background-color: #0284c7;
                color: white;
                font-weight: bold;
                font-size: 14px;
                border: 1px solid #38bdf8;
            }
            QPushButton#saveButton:hover {
                background-color: #0369a1;
            }
            QComboBox, QListWidget {
                background-color: #242832;
                border: 1px solid #333a48;
                border-radius: 4px;
                color: #f8fafc;
                padding: 4px;
            }
            QListWidget::item:selected {
                background-color: #0284c7;
                color: white;
                border-radius: 3px;
            }
            QCheckBox {
                color: #cbd5e1;
            }
            QCheckBox::indicator:checked {
                background-color: #0284c7;
                border: 1px solid #38bdf8;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #333a48;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #0284c7;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #38bdf8;
                width: 14px;
                margin-top: -4px;
                margin-bottom: -4px;
                border-radius: 7px;
            }
            QSpinBox {
                background-color: #242832;
                border: 1px solid #333a48;
                border-radius: 4px;
                color: #f8fafc;
                padding: 2px 4px;
            }
        """)

    def load_sample(self, idx):
        """Tải dữ liệu 3D, Mask và Landmarks của mẫu được chọn."""
        if idx < 0 or idx >= len(self.ply_files):
            return

        self.current_idx = idx
        ply_path = self.ply_files[idx]
        self.current_stem = ply_path.stem
        self.mask_history = []

        self.status_bar.showMessage(f"Đang tải dữ liệu đám mây điểm 3D mẫu {self.current_stem}...")
        QApplication.processEvents()

        # 1. Đọc đám mây điểm và mặt phẳng bàn
        self.xyz, self.bgr, _ = self.core.load_organized_ply(ply_path)
        self.img_h, self.img_w = self.bgr.shape[:2]
        self.plane_params = self.core.fit_table_plane(self.xyz)
        self.mask, height_map = self.core.segment_pants_3d(self.xyz, self.bgr, self.plane_params)

        # 2. Đọc file JSON đã lưu nếu có, hoặc trích xuất tự động
        json_path = self.core.verification_dir / f"{self.current_stem}_landmarks_3d.json"
        if json_path.exists():
            with open(json_path, 'r', encoding='utf-8') as f:
                saved = json.load(f)
                self.landmarks_data = saved.get("landmarks", {})
        else:
            self.landmarks_data = self.core.extract_landmarks_3d(self.xyz, self.mask, self.plane_params)

        # 3. Cập nhật giao diện
        self.update_canvas_image()
        self.build_interactive_landmarks()
        self.update_landmarks_list()

        # Thông tin mẫu
        a, b, c = self.plane_params['a'], self.plane_params['b'], self.plane_params['c']
        self.lbl_sample_info.setText(
            f"<b>Mẫu:</b> {self.current_stem} ({idx + 1}/{len(self.ply_files)})<br>"
            f"<b>Độ phân giải:</b> {self.img_w} x {self.img_h}<br>"
            f"<b>Mặt phẳng bàn:</b> Z = {a:.4f}X + {b:.4f}Y + {c:.1f}<br>"
            f"<b>Độ cao ngưỡng:</b> {self.core.height_threshold_mm} mm"
        )

        self.combo_samples.blockSignals(True)
        self.combo_samples.setCurrentIndex(idx)
        self.combo_samples.blockSignals(False)

        self.set_dirty(False)
        self.status_bar.showMessage(f"Đã nạp mẫu {self.current_stem}. Sẵn sàng hiệu chỉnh.", 5000)

    def toggle_paint_mode(self, checked):
        """Bật/tắt chế độ bút vẽ/tô bù mask (Paint Brush)."""
        self.is_paint_mode = checked
        if checked:
            if self.btn_brush_mode.isChecked():
                self.btn_brush_mode.setChecked(False)
            if self.btn_delete_mask_mode.isChecked():
                self.btn_delete_mask_mode.setChecked(False)
            self.brush_cursor_item.setPen(QPen(QColor(52, 211, 153, 240), 2, Qt.DashLine))
            self.brush_cursor_item.setBrush(QBrush(QColor(16, 185, 129, 45)))
            self.view.setCursor(Qt.BlankCursor)
            self.brush_cursor_item.setVisible(True)
            self.status_bar.showMessage(f"👉 BÚT VẼ MASK ĐANG BẬT (Cỡ bút: {self.brush_radius}px): Kéo chuột trái trên ảnh để tô/vẽ thêm vùng mask!", 10000)
        else:
            if not self.is_brush_mode:
                self.view.setCursor(Qt.ArrowCursor)
                self.brush_cursor_item.setVisible(False)
                self.status_bar.showMessage("Đã tắt bút vẽ. Trở lại chế độ bình thường.", 3000)

    def toggle_brush_mode(self, checked):
        """Bật/tắt chế độ bút tẩy mask (Brush Eraser)."""
        self.is_brush_mode = checked
        if checked:
            if self.btn_paint_mode.isChecked():
                self.btn_paint_mode.setChecked(False)
            if self.btn_delete_mask_mode.isChecked():
                self.btn_delete_mask_mode.setChecked(False)
            self.brush_cursor_item.setPen(QPen(QColor(248, 113, 113, 240), 2, Qt.DashLine))
            self.brush_cursor_item.setBrush(QBrush(QColor(239, 68, 68, 45)))
            self.view.setCursor(Qt.BlankCursor)
            self.brush_cursor_item.setVisible(True)
            self.status_bar.showMessage(f"👉 BÚT TẨY ĐANG BẬT (Cỡ bút: {self.brush_radius}px): Kéo chuột trái trên ảnh để tẩy xóa mask!", 10000)
        else:
            if not self.is_paint_mode:
                self.view.setCursor(Qt.ArrowCursor)
                self.brush_cursor_item.setVisible(False)
                self.status_bar.showMessage("Đã tắt bút tẩy. Trở lại chế độ bình thường.", 3000)

    def on_brush_radius_changed(self, val):
        """Thay đổi bán kính cọ vẽ/tẩy khi người dùng kéo thanh trượt."""
        self.brush_radius = int(val)
        r = self.brush_radius
        self.brush_cursor_item.setRect(-r, -r, 2 * r, 2 * r)

    def update_brush_cursor(self, scene_pos):
        """Cập nhật vị trí vòng tròn hiển thị bán kính cọ theo con trỏ chuột."""
        r = self.brush_radius
        self.brush_cursor_item.setRect(-r, -r, 2 * r, 2 * r)
        self.brush_cursor_item.setPos(scene_pos)
        self.brush_cursor_item.setVisible(True)

    def push_mask_history(self):
        """Lưu trạng thái mask vào lịch sử để hỗ trợ Undo."""
        if self.mask is not None:
            self.mask_history.append(self.mask.copy())
            if len(self.mask_history) > 15:
                self.mask_history.pop(0)

    def paint_mask_stroke(self, pt1, pt2, quick=True):
        """Tô vẽ thêm mask dọc theo đường di chuyển của chuột với bán kính đã chọn."""
        if self.mask is None:
            return
        x1, y1 = int(round(pt1.x())), int(round(pt1.y()))
        x2, y2 = int(round(pt2.x())), int(round(pt2.y()))
        r = self.brush_radius

        cv2.line(self.mask, (x1, y1), (x2, y2), 255, thickness=2 * r)
        cv2.circle(self.mask, (x2, y2), r, 255, -1)
        self.update_canvas_image(quick=quick)

    def erase_mask_stroke(self, pt1, pt2, quick=True):
        """Tẩy xóa mask dọc theo đường di chuyển của chuột với bán kính đã chọn."""
        if self.mask is None:
            return
        x1, y1 = int(round(pt1.x())), int(round(pt1.y()))
        x2, y2 = int(round(pt2.x())), int(round(pt2.y()))
        r = self.brush_radius

        cv2.line(self.mask, (x1, y1), (x2, y2), 0, thickness=2 * r)
        cv2.circle(self.mask, (x2, y2), r, 0, -1)
        self.update_canvas_image(quick=quick)

    def toggle_delete_mask_mode(self, checked):
        """Bật/tắt chế độ click để xóa vùng mask lỗi."""
        self.is_delete_mask_mode = checked
        if checked:
            if self.btn_brush_mode.isChecked():
                self.btn_brush_mode.setChecked(False)
            if self.btn_paint_mode.isChecked():
                self.btn_paint_mode.setChecked(False)
            self.view.setCursor(Qt.CrossCursor)
            self.brush_cursor_item.setVisible(False)
            self.status_bar.showMessage("👉 CHẾ ĐỘ CLICK XÓA VÙNG ĐANG BẬT: Click chuột trái trực tiếp vào vùng mask phụ/rác trên ảnh để xóa!", 10000)
        else:
            self.view.setCursor(Qt.ArrowCursor)
            self.status_bar.showMessage("Đã tắt chế độ xóa vùng. Trở lại chế độ bình thường.", 3000)

    def delete_mask_region_at(self, u, v):
        """Xóa toàn bộ vùng liên thông của Mask tại tọa độ click (u, v)."""
        if self.mask is None:
            return False
        h, w = self.mask.shape
        if u < 0 or u >= w or v < 0 or v >= h:
            return False

        # Phân tích các thành phần liên thông của mask
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            (self.mask > 0).astype(np.uint8), connectivity=8
        )
        if num_labels <= 1:
            self.status_bar.showMessage("Hiện không có vùng mask nào để xóa.", 3000)
            return False

        # Tìm nhãn tại vị trí click (mở rộng lân cận 6px nếu click gần mép)
        clicked_label = labels[v, u]
        if clicked_label == 0:
            u_min, u_max = max(0, u - 6), min(w, u + 7)
            v_min, v_max = max(0, v - 6), min(h, v + 7)
            sub = labels[v_min:v_max, u_min:u_max]
            valid = sub[sub > 0]
            if len(valid) > 0:
                clicked_label = int(np.bincount(valid).argmax())

        if clicked_label > 0:
            # Lưu lịch sử để hỗ trợ Undo
            self.mask_history.append(self.mask.copy())
            if len(self.mask_history) > 10:
                self.mask_history.pop(0)

            area = stats[clicked_label, cv2.CC_STAT_AREA]
            self.mask[labels == clicked_label] = 0
            self.update_canvas_image()
            self.set_dirty(True)
            self.status_bar.showMessage(f"✓ Đã xóa sạch vùng Mask lỗi ({area} pixels). Nhấn 'Hoàn tác Mask' nếu lỡ xóa nhầm.", 6000)
            return True
        else:
            self.status_bar.showMessage("Không có vùng mask màu xanh nào tại vị trí vừa click. Hãy click vào đúng vùng màu xanh cần xóa.", 3000)
            return False

    def keep_largest_mask(self):
        """Tự động giữ lại duy nhất 1 vùng thân quần lớn nhất, xóa sạch mọi vùng rác nhỏ."""
        if self.mask is None:
            return
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            (self.mask > 0).astype(np.uint8), connectivity=8
        )
        if num_labels <= 2:
            self.status_bar.showMessage("Hiện ảnh chỉ có đúng 1 vùng mask duy nhất, không có vùng rác phụ.", 3000)
            return

        self.mask_history.append(self.mask.copy())
        if len(self.mask_history) > 10:
            self.mask_history.pop(0)

        areas = stats[1:, cv2.CC_STAT_AREA]
        largest_label = int(np.argmax(areas)) + 1
        removed_count = num_labels - 2
        removed_area = int(np.sum(areas)) - int(areas[largest_label - 1])

        self.mask = np.where(labels == largest_label, 255, 0).astype(np.uint8)
        self.update_canvas_image()
        self.set_dirty(True)
        self.status_bar.showMessage(f"✓ Đã tự động loại bỏ {removed_count} vùng mask rác ({removed_area} pixels), chỉ giữ lại duy nhất thân quần!", 6000)

    def undo_mask(self):
        """Hoàn tác thao tác xóa mask vừa thực hiện."""
        if self.mask_history:
            self.mask = self.mask_history.pop()
            self.update_canvas_image()
            self.set_dirty(True)
            self.status_bar.showMessage("✓ Đã hoàn tác (Undo) khôi phục lại vùng mask trước đó.", 4000)
        else:
            self.status_bar.showMessage("Không có lịch sử xóa mask nào để hoàn tác.", 3000)

    def update_canvas_image(self, quick=False):
        """Hiển thị ảnh gốc kết hợp lớp phủ Mask bán trong suốt."""
        display_bgr = self.bgr.copy()
        if self.chk_show_mask.isChecked() and self.mask is not None:
            green = np.zeros_like(display_bgr)
            green[self.mask > 0] = [0, 230, 80]
            cv2.addWeighted(green, 0.30, display_bgr, 0.70, 0, display_bgr)
            # Chỉ tính toán vẽ viền khi không ở chế độ kéo nhanh (giúp kéo cọ tẩy đạt 60+ FPS cực mượt)
            if not quick:
                contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(display_bgr, contours, -1, (255, 255, 255), 1)

        rgb = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        q_img = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.pixmap_item.setPixmap(QPixmap.fromImage(q_img))
        self.scene.setSceneRect(0, 0, w, h)

    def build_interactive_landmarks(self):
        """Xóa và dựng lại 12 điểm mốc và các đoạn khung xương trên Canvas."""
        # Xóa các item cũ
        for item in self.landmark_items.values():
            self.scene.removeItem(item)
        for line in self.skeleton_lines:
            self.scene.removeItem(line)
        self.landmark_items.clear()
        self.skeleton_lines.clear()

        # Tạo 12 điểm mốc tương tác
        for name in self.core.LANDMARK_NAMES:
            if name in self.landmarks_data:
                lm = self.landmarks_data[name]
                u = lm.get("pixel_u", 0)
                v = lm.get("pixel_v", 0)
            else:
                u, v = self.img_w // 2, self.img_h // 2

            item = LandmarkItem(name, u, v, self)
            self.scene.addItem(item)
            self.landmark_items[name] = item

        # Tạo các đường nối khung xương
        self.rebuild_skeleton_lines()

    def rebuild_skeleton_lines(self):
        """Dựng lại các đoạn thẳng khung xương nối giữa các mốc."""
        for line in self.skeleton_lines:
            self.scene.removeItem(line)
        self.skeleton_lines.clear()

        if not self.chk_show_skeleton.isChecked():
            return

        for p1_name, p2_name in self.core.SKELETON_EDGES:
            if p1_name in self.landmark_items and p2_name in self.landmark_items:
                line_item = SkeletonLine(self.landmark_items[p1_name], self.landmark_items[p2_name])
                self.scene.addItem(line_item)
                self.skeleton_lines.append(line_item)

    def on_landmark_moved(self, name, u, v):
        """Xử lý sự kiện khi một mốc bị kéo sang vị trí mới."""
        u_int = int(round(u))
        v_int = int(round(v))

        # Cập nhật tọa độ 3D và vector pháp tuyến
        coord_3d = self.core.get_valid_3d_point(self.xyz, u_int, v_int)
        normal = self.core.estimate_surface_normal(self.xyz, u_int, v_int)
        a, b, c = self.plane_params['a'], self.plane_params['b'], self.plane_params['c']
        z_tbl = a * coord_3d[0] + b * coord_3d[1] + c
        h_above = float(z_tbl - coord_3d[2])

        self.landmarks_data[name] = {
            "pixel_u": u_int,
            "pixel_v": v_int,
            "coord_3d_mm": [float(coord_3d[0]), float(coord_3d[1]), float(coord_3d[2])],
            "height_above_table_mm": round(h_above, 2),
            "normal_vector": [float(normal[0]), float(normal[1]), float(normal[2])]
        }

        # Cập nhật đường xương nối với mốc này
        for line in self.skeleton_lines:
            if line.item1.name == name or line.item2.name == name:
                line.update_position()

        # Cập nhật text hiển thị chi tiết
        self.show_landmark_details(name)
        self.update_list_item_text(name)
        self.set_dirty(True)

    def on_landmark_selected(self, name):
        """Khi người dùng click chọn một mốc trên Canvas."""
        self.selected_landmark = name
        for item_name, item in self.landmark_items.items():
            item.update_style(selected=(item_name == name))

        # Đồng bộ chọn trên ListWidget
        for i in range(self.list_landmarks.count()):
            item_w = self.list_landmarks.item(i)
            if item_w.data(Qt.UserRole) == name:
                self.list_landmarks.setCurrentItem(item_w)
                break

        self.show_landmark_details(name)

    def on_list_item_clicked(self, list_item):
        name = list_item.data(Qt.UserRole)
        self.on_landmark_selected(name)

    def update_landmarks_list(self):
        """Cập nhật toàn bộ danh sách 12 mốc trên Sidebar."""
        self.list_landmarks.clear()
        for name in self.core.LANDMARK_NAMES:
            lm = self.landmarks_data.get(name, {})
            u = lm.get("pixel_u", 0)
            v = lm.get("pixel_v", 0)
            h_val = lm.get("height_above_table_mm", 0.0)

            item = QListWidgetItem(f"• {name:16s} ({u:4d}, {v:4d}) | H={h_val:4.1f}mm")
            item.setData(Qt.UserRole, name)
            self.list_landmarks.addItem(item)

    def update_list_item_text(self, name):
        lm = self.landmarks_data.get(name, {})
        u = lm.get("pixel_u", 0)
        v = lm.get("pixel_v", 0)
        h_val = lm.get("height_above_table_mm", 0.0)

        for i in range(self.list_landmarks.count()):
            item = self.list_landmarks.item(i)
            if item.data(Qt.UserRole) == name:
                item.setText(f"• {name:16s} ({u:4d}, {v:4d}) | H={h_val:4.1f}mm")
                break

    def show_landmark_details(self, name):
        lm = self.landmarks_data.get(name, {})
        u = lm.get("pixel_u", 0)
        v = lm.get("pixel_v", 0)
        c3d = lm.get("coord_3d_mm", [0, 0, 0])
        h_val = lm.get("height_above_table_mm", 0.0)
        norm = lm.get("normal_vector", [0, 0, 1])

        self.lbl_lm_details.setText(
            f"<b>MỐC ĐANG CHỌN: <font color='#38bdf8'>{name}</font></b><br>"
            f"• Tọa độ Pixel: (U={u}, V={v})<br>"
            f"• Tọa độ 3D: [{c3d[0]:.1f}, {c3d[1]:.1f}, {c3d[2]:.1f}] mm<br>"
            f"• Độ cao nổi: <b>{h_val:.2f} mm</b><br>"
            f"• Pháp tuyến n: [{norm[0]:.3f}, {norm[1]:.3f}, {norm[2]:.3f}]"
        )

    def recalc_derived_landmarks(self):
        """Tự động suy luận 4 mốc Đùi & Gối từ 8 mốc chuẩn: 3 mốc Cạp (trái, giữa, phải), 1 mốc Đáy đũng và 4 mốc Gấu quần (2 gấu trái, 2 gấu phải)."""
        try:
            # 8 mốc chuẩn do người dùng định hình (Giữ nguyên Cạp giữa waist_center)
            wl = np.array([self.landmarks_data["waist_left"]["pixel_u"], self.landmarks_data["waist_left"]["pixel_v"]], dtype=np.float32)
            wr = np.array([self.landmarks_data["waist_right"]["pixel_u"], self.landmarks_data["waist_right"]["pixel_v"]], dtype=np.float32)
            wc = np.array([self.landmarks_data["waist_center"]["pixel_u"], self.landmarks_data["waist_center"]["pixel_v"]], dtype=np.float32)
            crotch = np.array([self.landmarks_data["crotch"]["pixel_u"], self.landmarks_data["crotch"]["pixel_v"]], dtype=np.float32)

            c1_out = np.array([self.landmarks_data["cuff_left_outer"]["pixel_u"], self.landmarks_data["cuff_left_outer"]["pixel_v"]], dtype=np.float32)
            c1_in = np.array([self.landmarks_data["cuff_left_inner"]["pixel_u"], self.landmarks_data["cuff_left_inner"]["pixel_v"]], dtype=np.float32)
            c2_in = np.array([self.landmarks_data["cuff_right_inner"]["pixel_u"], self.landmarks_data["cuff_right_inner"]["pixel_v"]], dtype=np.float32)
            c2_out = np.array([self.landmarks_data["cuff_right_outer"]["pixel_u"], self.landmarks_data["cuff_right_outer"]["pixel_v"]], dtype=np.float32)

            # Tim 2 gấu
            c1_mid = (c1_out + c1_in) / 2.0
            c2_mid = (c2_out + c2_in) / 2.0

            # Vector trục và vuông góc của từng ống
            v_l1 = c1_mid - crotch
            dir_l1 = v_l1 / (np.linalg.norm(v_l1) + 1e-6)
            perp_l1 = np.array([-dir_l1[1], dir_l1[0]], dtype=np.float32)

            v_l2 = c2_mid - crotch
            dir_l2 = v_l2 / (np.linalg.norm(v_l2) + 1e-6)
            perp_l2 = np.array([-dir_l2[1], dir_l2[0]], dtype=np.float32)

            # Phân tách mặt nạ để dò tim
            v_mid = (c1_mid + c2_mid) / 2.0 - crotch
            v_mid_dir = v_mid / (np.linalg.norm(v_mid) + 1e-6)
            perp_dir = np.array([-v_mid_dir[1], v_mid_dir[0]], dtype=np.float32)
            if np.dot(c1_mid - crotch, perp_dir) > 0:
                perp_dir = -perp_dir

            h_m, w_m = self.mask.shape
            y_g, x_g = np.indices((h_m, w_m))
            sep_val = (x_g - crotch[0]) * perp_dir[0] + (y_g - crotch[1]) * perp_dir[1]

            mask_leg1 = self.mask.copy()
            mask_leg1[sep_val > 0] = 0
            mask_leg2 = self.mask.copy()
            mask_leg2[sep_val < 0] = 0

            # Điểm neo hình học: Đùi (28%), Gối (55%)
            seed_knee1 = 0.45 * crotch + 0.55 * c1_mid
            seed_knee2 = 0.45 * crotch + 0.55 * c2_mid
            seed_thigh1 = 0.72 * crotch + 0.28 * c1_mid
            seed_thigh2 = 0.72 * crotch + 0.28 * c2_mid

            knee_left = self.core.find_cross_section_center(mask_leg1, seed_knee1, perp_l1)
            knee_right = self.core.find_cross_section_center(mask_leg2, seed_knee2, perp_l2)
            thigh_left = self.core.find_cross_section_center(mask_leg1, seed_thigh1, perp_l1)
            thigh_right = self.core.find_cross_section_center(mask_leg2, seed_thigh2, perp_l2)

            # Chỉ cập nhật 4 mốc Đùi và Gối (GIỮ NGUYÊN 8 mốc do người dùng chỉ định)
            derived = {
                "thigh_left": thigh_left,
                "knee_left": knee_left,
                "thigh_right": thigh_right,
                "knee_right": knee_right
            }

            for name, pt in derived.items():
                self.landmark_items[name].setPos(pt[0], pt[1])
                self.on_landmark_moved(name, pt[0], pt[1])

            self.status_bar.showMessage("✓ Đã tự động căn chuẩn 4 mốc Đùi & Gối từ 8 mốc chuẩn (3 Cạp + 1 Đáy + 4 Gấu)!", 4000)

        except Exception as e:
            QMessageBox.warning(self, "Lỗi tính toán", f"Không thể suy luận tim ống: {str(e)}")

    def save_current_sample(self):
        """Lưu toàn bộ nhãn 3D, YOLO11-pose và ảnh Verification Overlay."""
        try:
            stem = self.current_stem
            sample_num = stem.replace("pcl_", "")

            # 1. Ghi đè file JSON 3D landmarks
            json_dest_path = self.core.verification_dir / f"{stem}_landmarks_3d.json"
            metrics = self.core.compute_3d_metrics(self.xyz, self.mask, self.xyz[:, :, 2])
            out_data = {
                "sample_name": stem,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "metrics_3d": metrics,
                "landmarks": self.landmarks_data
            }
            with open(json_dest_path, 'w', encoding='utf-8') as f:
                json.dump(out_data, f, indent=2, ensure_ascii=False)

            # 2. Cập nhật nhãn YOLO11-seg (.txt) và YOLO11-pose (.txt) trong train hoặc val
            for split in ["train", "val"]:
                seg_path = self.core.yolo_dir / "labels" / split / f"{stem}.txt"
                if seg_path.exists():
                    self.core.export_yolo_seg(self.mask, seg_path)
                pose_path = self.core.yolo_dir / "labels_pose" / split / f"{stem}.txt"
                if pose_path.exists():
                    self.core.export_yolo_pose(self.mask, self.landmarks_data, pose_path)

            # 3. Tạo lại ảnh Verification Overlay đẹp mắt
            overlay = self.core.generate_verification_overlay(
                self.bgr, self.mask, self.landmarks_data, stem, metrics
            )
            overlay_path = self.core.verification_dir / f"{stem}_verification.jpg"
            cv2.imwrite(str(overlay_path), overlay, [cv2.IMWRITE_JPEG_QUALITY, 85])

            self.set_dirty(False)
            self.lbl_save_status.setText("<font color='#4ade80'>✓ Đã lưu thành công!</font>")
            self.status_bar.showMessage(f"✓ Đã lưu thành công mẫu {stem} (JSON, YOLO-pose, Verification Overlay)", 6000)

        except Exception as e:
            QMessageBox.critical(self, "Lỗi Lưu", f"Không thể lưu kết quả: {str(e)}")

    def reset_current_sample(self):
        """Khôi phục lại mốc ban đầu từ thuật toán tự động."""
        res = QMessageBox.question(
            self, "Xác nhận đặt lại",
            f"Bạn có chắc muốn tính toán lại mốc tự động từ đầu cho mẫu {self.current_stem}?",
            QMessageBox.Yes | QMessageBox.No
        )
        if res == QMessageBox.Yes:
            self.landmarks_data = self.core.extract_landmarks_3d(self.xyz, self.mask, self.plane_params)
            self.build_interactive_landmarks()
            self.update_landmarks_list()
            self.set_dirty(True)

    def set_dirty(self, dirty=True):
        self.is_dirty = dirty
        if dirty:
            self.lbl_save_status.setText("<font color='#f59e0b'>● Có thay đổi chưa lưu</font>")
        else:
            self.lbl_save_status.setText("<font color='#94a3b8'>Trạng thái: Đã đồng bộ</font>")

    def on_combo_sample_changed(self, idx):
        if idx != self.current_idx:
            if self.is_dirty:
                res = QMessageBox.question(
                    self, "Chưa lưu thay đổi",
                    f"Mẫu {self.current_stem} có thay đổi chưa lưu. Bạn có muốn lưu trước khi chuyển mẫu?",
                    QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel
                )
                if res == QMessageBox.Save:
                    self.save_current_sample()
                elif res == QMessageBox.Cancel:
                    self.combo_samples.blockSignals(True)
                    self.combo_samples.setCurrentIndex(self.current_idx)
                    self.combo_samples.blockSignals(False)
                    return
            self.load_sample(idx)

    def prev_sample(self):
        if self.current_idx > 0:
            self.on_combo_sample_changed(self.current_idx - 1)

    def next_sample(self):
        if self.current_idx < len(self.ply_files) - 1:
            self.on_combo_sample_changed(self.current_idx + 1)

    def on_toggle_mask(self, checked):
        self.update_canvas_image()

    def on_toggle_skeleton(self, checked):
        self.rebuild_skeleton_lines()

    def fit_view(self):
        self.view.fitInView(self.pixmap_item, Qt.KeepAspectRatio)


def main():
    app = QApplication(sys.argv)
    window = LandmarkEditorApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
