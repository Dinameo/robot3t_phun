"""Capture one synchronized RGB/XYZ frame; never moves a robot."""
from datetime import datetime, timezone
from pathlib import Path
import importlib.util
import os
import threading
import numpy as np
from frame_io import save_frame

class ZividCapture:
    def __init__(self, dataset_dir, settings_file=None, serial=None, file_camera=None):
        self.dataset_dir=Path(dataset_dir)
        self.settings_file=Path(settings_file) if settings_file else None
        self.serial=serial
        self.file_camera=file_camera
        self.app=None
        self.camera=None
        self.lock=threading.Lock()
        self._dll_dir=None

    def status(self):
        return {'python_package_installed':importlib.util.find_spec('zivid') is not None,
                'sdk_directory_present':Path(r'C:\Program Files\Zivid\bin').is_dir(),
                'connected':self.camera is not None,
                'settings_file':str(self.settings_file or ''), 'serial':self.serial or '',
                'mode':'file_camera' if self.file_camera else 'physical_camera'}

    def capture(self):
        with self.lock:
            if importlib.util.find_spec('zivid') is None:
                raise RuntimeError('Chua cai Zivid SDK/Python tuong thich. Hay cai SDK va xac nhan chup duoc trong Zivid Studio truoc.')
            sdk_bin=Path(r'C:\Program Files\Zivid\bin')
            if os.name=='nt' and sdk_bin.is_dir():
                if self._dll_dir is None: self._dll_dir=os.add_dll_directory(str(sdk_bin))
                if str(sdk_bin) not in os.environ.get('PATH','').split(os.pathsep):
                    os.environ['PATH']=str(sdk_bin)+os.pathsep+os.environ.get('PATH','')
            import zivid
            if self.app is None: self.app=zivid.Application()
            if self.camera is None:
                if self.file_camera: self.camera=self.app.create_file_camera(str(self.file_camera))
                elif self.serial: self.camera=self.app.connect_camera(serial_number=self.serial)
                else: self.camera=self.app.connect_camera()
            if self.settings_file:
                if not self.settings_file.is_file(): raise FileNotFoundError(str(self.settings_file))
                settings=zivid.Settings.load(str(self.settings_file))
            else:
                params=zivid.capture_assistant.SuggestSettingsParameters()
                settings=zivid.capture_assistant.suggest_settings(self.camera,params)
            # New API captures aligned color and depth together; legacy SDKs use capture().
            if hasattr(self.camera,'capture_2d_3d'): frame=self.camera.capture_2d_3d(settings)
            else: frame=self.camera.capture(settings)
            sample='pcl_live_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
            target=self.dataset_dir/'_3D'/sample
            target.parent.mkdir(parents=True,exist_ok=True)
            frame.save(str(target.with_suffix('.zdf')))
            cloud=frame.point_cloud()
            xyz=cloud.copy_data('xyz')
            rgba=cloud.copy_data('rgba')
            ply=save_frame(target,xyz,rgba[:,:,:3])
            import cv2
            image_dir=self.dataset_dir/'_2D'; image_dir.mkdir(parents=True,exist_ok=True)
            rgb=np.asarray(rgba[:,:,:3],dtype=np.uint8)
            if not cv2.imwrite(str(image_dir/(sample+'.png')),rgb[:,:,::-1]):
                raise OSError('Cannot save captured RGB image.')
            return sample,ply

    def close(self):
        with self.lock:
            if self.camera is not None and hasattr(self.camera,'disconnect'): self.camera.disconnect()
            self.camera=None
            self.app=None
