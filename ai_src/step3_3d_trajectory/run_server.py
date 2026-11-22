"""Windows-ready Unity bridge: responsive UDP/HTTP, honest readiness, real inference."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import argparse
import json
import logging
import os
import re
import shutil
import socket
import sys
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parents[2]
STEP2_DIR=ROOT/'ai_src/step2_train_yolo'
if str(STEP2_DIR) not in sys.path:
    sys.path.insert(0,str(STEP2_DIR))
import retrain_pose_feedback as pose_fb
os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'runtime/ultralytics'))
(ROOT/'runtime/ultralytics').mkdir(parents=True,exist_ok=True)
LOG=logging.getLogger('cr10a')

class Service:
    def __init__(self,root=ROOT,settings_file=None,serial=None,file_camera=None,calibration_file=None):
        self.root=Path(root)
        self.dataset=self.root/'ai_src/dataset'
        for folder in ('_2D','_3D'): (self.dataset/folder).mkdir(parents=True,exist_ok=True)
        self.backend=None; self.loading=True; self.model_error=''
        self.lock=threading.Lock()
        self.capture_progress=None
        self.calibration=None
        if calibration_file:
            from calibration import load_calibration
            self.calibration=load_calibration(calibration_file)
        from zivid_capture import ZividCapture
        self.capture=ZividCapture(self.dataset,settings_file,serial,file_camera)
        self.output=self.root/'dobot_cr10a/output_drawing'; self.output.mkdir(parents=True,exist_ok=True)

    def load_models(self):
        try:
            from unity_ai_server import DobotUnityAIServer, UnityDobotKinematicsBridge
            if self.calibration is not None:
                from calibration import camera_to_robot
                def calibrated_transform(vx,vy,vz,z_contact_offset_mm=22.0):
                    p=camera_to_robot([vx,vy,vz],self.calibration)
                    return float(p[0]),float(p[1]),float(p[2]+z_contact_offset_mm)
                UnityDobotKinematicsBridge.zivid_mm_to_robot_global_mm=staticmethod(calibrated_transform)
            self.backend=DobotUnityAIServer(unity_project_dir=self.root/'dobot_cr10a')
            LOG.info('Models ready. Real YOLO segmentation and pose enabled.')
        except Exception as ex:
            self.model_error=str(ex); LOG.exception('Model initialization failed')
        finally: self.loading=False

    def status(self):
        return {'status':'ONLINE','service':'CR10A_Windows_AI','version':'3.0',
                'model_ready':self.backend is not None,'loading':self.loading,'model_error':self.model_error,
                'project_root':str(self.root),'camera_preview_supported':True,'simulation_only':True,'calibrated':self.calibration is not None,'camera':self.capture.status(),
                'feedback':pose_fb.get_feedback_status()}

    def start_pose_retrain_async(self,force=False,min_samples=10,epochs=25):
        st=pose_fb.get_feedback_status(threshold=min_samples)
        if st.get('is_retraining'):
            return {'status':'BUSY','message':'Đang train lại YOLO11-pose ở nền.','is_retraining':True,
                    'pending_count':st.get('pending_count',0),'threshold':st.get('threshold',min_samples),
                    'history_count':st.get('history_count',0),'total_retrains':st.get('total_retrains',0)}
        if not force and st.get('pending_count',0)<min_samples:
            return {'status':'SKIPPED','message':f"Chưa đủ {min_samples} mẫu sai ({st.get('pending_count',0)}/{min_samples}).",
                    'is_retraining':False,'pending_count':st.get('pending_count',0),'threshold':min_samples,
                    'history_count':st.get('history_count',0),'total_retrains':st.get('total_retrains',0)}
        if st.get('pending_count',0)==0 and st.get('history_count',0)==0:
            return {'status':'ERROR','message':'Chưa có mẫu sửa mốc nào để train lại.','is_retraining':False,
                    'pending_count':0,'threshold':min_samples,'history_count':0,'total_retrains':st.get('total_retrains',0)}

        def _on_retrain_done(ok,msg):
            if ok and self.backend is not None:
                try:
                    with self.lock:
                        reloaded=self.backend.pipeline.reload_pose_model()
                    LOG.info('Hot-reloaded YOLO11-pose weights: %s',reloaded)
                except Exception as ex:
                    LOG.exception('Failed to hot-reload YOLO11-pose: %s',ex)
            LOG.info('Pose retraining finished (ok=%s): %s',ok,msg)

        def _worker():
            pose_fb.run_pose_retraining(min_samples=min_samples,force=force,epochs=epochs,on_complete=_on_retrain_done)

        threading.Thread(target=_worker,daemon=True,name='RetrainYOLO11Pose').start()
        return {'status':'STARTED','message':f"Đã kích hoạt train lại YOLO11-pose ({st.get('pending_count',0)} mẫu mới, {epochs} epochs)...",
                'is_retraining':True,'pending_count':st.get('pending_count',0),'threshold':min_samples,
                'history_count':st.get('history_count',0),'total_retrains':st.get('total_retrains',0)}

    def register_sample(self,req):
        sample=str(req.get('sample','pcl_101'))
        raw=req.get('sample_path')
        if raw:
            src=Path(raw).resolve()
            if src.suffix.lower()!='.ply' or not src.is_file(): raise ValueError('sample_path must be an existing .ply file.')
            sample=src.stem
            if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}',sample):
                sample='pcl_import_'+uuid.uuid4().hex[:10]
            target=self.dataset/'_3D'/(sample+'.ply')
            if src!=target:
                shutil.copy2(src,target)
                # Never reuse a previous NPZ after a new PLY has been registered.
                for suffix in ('.npz','.frame.json'):
                    side=src.with_suffix(suffix); dest=target.with_suffix(suffix)
                    if side.is_file(): shutil.copy2(side,dest)
                    elif dest.exists(): dest.unlink()
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}',sample): raise ValueError('Invalid sample name.')
        return sample

    def handle(self,req):
        command=req.get('command','TRIGGER_DETECT')
        if command in ('PING','STATUS'): return self.status()
        if command=='CAMERA_STATUS': return {'status':'SUCCESS',**self.capture.status()}
        if command=='FEEDBACK_STATUS': return pose_fb.get_feedback_status(threshold=int(req.get('threshold',10) or 10))
        if command=='SAVE_LANDMARK_FEEDBACK':
            try:
                sample=str(req.get('sample','sample'))
                lm_uv=req.get('landmarks_uv',[])
                threshold=int(req.get('threshold',10) or 10)
                saved=pose_fb.save_corrected_landmark_sample(
                    sample_name=sample,
                    landmarks_uv=lm_uv,
                    image_path=req.get('image_path'),
                    image_b64=req.get('image_b64'),
                    bbox_uv=req.get('bbox_uv'),
                    threshold=threshold,
                )
                auto_retrain=bool(req.get('auto_retrain',True))
                retrain_triggered=False
                if auto_retrain and saved.get('ready_to_retrain') and not saved.get('is_retraining'):
                    rt=self.start_pose_retrain_async(force=False,min_samples=threshold,epochs=int(req.get('epochs',25) or 25))
                    if rt.get('status')=='STARTED':
                        retrain_triggered=True
                        saved['is_retraining']=True
                        saved['status_message']=rt.get('message',saved.get('status_message',''))
                saved['retrain_triggered']=retrain_triggered
                return saved
            except Exception as ex:
                LOG.warning('SAVE_LANDMARK_FEEDBACK failed: %s',ex)
                return {'status':'ERROR','message':str(ex),'error_code':422}
        if command=='RETRAIN_POSE':
            return self.start_pose_retrain_async(
                force=bool(req.get('force',True)),
                min_samples=int(req.get('threshold',10) or 10),
                epochs=int(req.get('epochs',25) or 25),
            )
        try:
            if command not in ('TRIGGER_DETECT','CUSTOM_DRAW_3D','CAPTURE_DETECT','CAPTURE_ONLY','CAPTURE_PREVIEW'):
                raise ValueError('Unknown command: '+str(command))
            if command not in ('CAPTURE_ONLY','CAPTURE_PREVIEW') and self.backend is None:
                raise RuntimeError('AI dang khoi tao; thu lai sau.' if self.loading else 'Khong nap duoc model: '+self.model_error)
            # One capture/inference transaction at a time. PING/status remain responsive.
            if not self.lock.acquire(blocking=False):
                return {'status':'ERROR','message':'Server dang xu ly yeu cau truoc. Cho hoan tat roi thu lai.','error_code':409}
            try:
                if command in ('CAPTURE_DETECT','CAPTURE_ONLY','CAPTURE_PREVIEW'):
                    progress_id=req.get('progress_id')
                    if isinstance(progress_id,str) and re.fullmatch(r'[0-9a-f]{32}',progress_id):
                        self.capture_progress=(progress_id,'capture')
                    sample,ply=self.capture.capture()
                    cam_img=self.dataset/'_2D'/(sample+'.png')
                    cam_img_str=str(cam_img) if cam_img.is_file() else ''
                    if command in ('CAPTURE_ONLY','CAPTURE_PREVIEW'):
                        return {'status':'SUCCESS','sample':sample,'pointcloud_ply':str(ply),'camera_image':cam_img_str,'preview_image':cam_img_str,'trajectory_csv':'','frame_only':True,'message':'Da chup camera that; hinh hoc dang hien trong he quy chieu mo phong.','simulation_only':True}
                    if self.calibration is None and not req.get('preview_nominal_rig',False):
                        raise ValueError(f'Da luu anh {sample}. Can hieu chuan camera-robot truoc khi sinh quy dao tu camera that. File: {ply}')
                    if self.capture_progress is not None:
                        self.capture_progress=(self.capture_progress[0],'detect')
                else: sample=self.register_sample(req)
                if command=='CUSTOM_DRAW_3D':
                    strokes=req.get('strokes',[])
                    if not strokes or sum(len(s) for s in strokes)<2: raise ValueError('Drawing needs at least two points.')
                    result=self.backend.run_custom_draw_3d_for_unity(sample,strokes,float(req.get('z_offset_mm',22)))
                else: result=self.backend.run_detection_for_unity(sample,req.get('size','M'))
                if result.get('status') not in ('SUCCESS','OK'): return result
                if int(result.get('waypoints_count',0))<2: raise ValueError('AI khong tao duoc quy dao. Khong nap ket qua rong.')
                revision=uuid.uuid4().hex[:12]
                path=Path(result['trajectory_csv'])
                if not path.is_absolute(): path=self.root/'dobot_cr10a'/path
                # Publish immutable result files so Unity never skips an overwritten path.
                dest=self.output/(sample+'_'+revision+'.csv')
                shutil.copy2(path,dest)
                import csv
                with dest.open(encoding='utf-8-sig',newline='') as f: rows=list(csv.DictReader(f))
                failed=sum(r.get('status') not in ('SUCCESS','OK') for r in rows)
                if failed: raise ValueError(f'{failed}/{len(rows)} diem khong giai duoc IK. Khong nap lenh khop 0 thay the.')
                seg=self.output/(sample+'_landmarks_seg.json')
                seg_dest=self.output/(sample+'_'+revision+'_landmarks_seg.json')
                if not seg.is_file(): raise FileNotFoundError('Missing landmarks/segmentation output.')
                shutil.copy2(seg,seg_dest)
                cam_candidates=[self.dataset/'_2D'/(sample+'.png'),self.dataset/'_2D'/('image_'+sample.replace('pcl_','')+'.png')]
                cam_img_path=next((str(p) for p in cam_candidates if p.is_file()),'')
                result.update(trajectory_csv=str(dest),landmarks_json=str(seg_dest),camera_image=cam_img_path,revision=revision,simulation_only=True)
                if self.calibration is not None:
                    from frame_io import load_frame,save_frame
                    from calibration import camera_to_robot,robot_to_legacy_view_camera
                    xyz,_,rgb=load_frame(result['pointcloud_ply'])
                    view_xyz=robot_to_legacy_view_camera(camera_to_robot(xyz,self.calibration))
                    view=self.output/'frames'/revision/(sample+'.ply')
                    save_frame(view,view_xyz,rgb)
                    result['pointcloud_ply']=str(view)
                LOG.info('Published %s: %s waypoints revision=%s',sample,len(rows),revision)
                return result
            finally:
                self.capture_progress=None
                self.lock.release()
        except Exception as ex:
            LOG.warning('Request rejected (%s): %s',command,ex)
            return {'status':'ERROR','message':str(ex),'error_code':422,'simulation_only':True}

def serve(service,host='127.0.0.1',udp_port=5005,http_port=5000):
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    # Bind synchronously; a port conflict must fail startup clearly.
    udp.bind((host,udp_port))
    executor=ThreadPoolExecutor(max_workers=4,thread_name_prefix='cr10a-request')
    def respond(req,addr):
        result=service.handle(req)
        body=json.dumps(result,ensure_ascii=False,allow_nan=False).encode('utf-8')
        if len(body)>60000: body=b'{"status":"ERROR","message":"Response too large; use HTTP.","error_code":413}'
        udp.sendto(body,addr)
    def udp_worker():
        while True:
            try:
                raw,addr=udp.recvfrom(65535)
                try:
                    text=raw.decode('utf-8').strip()
                    req={'command':'PING'} if text.upper()=='PING' else json.loads(text)
                    if not isinstance(req,dict): raise ValueError('Object required')
                    if req.get('command') in ('PING','STATUS','CAMERA_STATUS','FEEDBACK_STATUS'): respond(req,addr)
                    else: executor.submit(respond,req,addr)
                except (ValueError,UnicodeError):
                    udp.sendto(b'{"status":"ERROR","message":"Invalid JSON request","error_code":400}',addr)
            except OSError: break
    class Handler(BaseHTTPRequestHandler):
        def reply(self,obj,code=200):
            body=json.dumps(obj,ensure_ascii=False,allow_nan=False).encode('utf-8')
            self.send_response(code); self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Content-Length',str(len(body))); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            if self.path=='/api/status': self.reply(service.status())
            elif self.path=='/api/camera/status': self.reply(service.handle({'command':'CAMERA_STATUS'}))
            elif self.path.startswith('/api/capture/progress/'):
                request_id=self.path.rsplit('/',1)[-1]
                progress=service.capture_progress
                self.reply({'phase':progress[1] if progress and progress[0]==request_id else ''})
            elif self.path.startswith('/api/feedback/status'): self.reply(service.handle({'command':'FEEDBACK_STATUS'}))
            else: self.reply({'status':'ERROR','message':'Unknown endpoint'},404)
        def do_POST(self):
            commands={
                '/api/detect':'TRIGGER_DETECT',
                '/api/custom_draw_3d':'CUSTOM_DRAW_3D',
                '/api/capture':'CAPTURE_DETECT',
                '/api/feedback/landmarks':'SAVE_LANDMARK_FEEDBACK',
                '/api/feedback/retrain':'RETRAIN_POSE',
            }
            if self.path not in commands: self.reply({'status':'ERROR','message':'Unknown endpoint'},404); return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if length<=0 or length>15000000: raise ValueError('Invalid request size')
                req=json.loads(self.rfile.read(length))
                if not isinstance(req,dict): raise ValueError('JSON object required')
                req.setdefault('command',commands[self.path]); self.reply(service.handle(req))
            except (ValueError,TypeError): self.reply({'status':'ERROR','message':'Invalid JSON request'},400)
        def log_message(self,*args): pass
    try: http=ThreadingHTTPServer((host,http_port),Handler)
    except Exception:
        udp.close(); executor.shutdown(wait=False); raise
    threading.Thread(target=udp_worker,daemon=True,name='UDP5005').start()
    threading.Thread(target=service.load_models,daemon=True,name='LoadYOLO').start()
    LOG.info('UDP %s:%s | HTTP http://%s:%s | simulation only',host,udp_port,host,http_port)
    try: http.serve_forever()
    finally:
        http.server_close(); udp.close(); executor.shutdown(wait=False); service.capture.close()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--udp-port',type=int,default=5005)
    parser.add_argument('--http-port',type=int,default=5000)
    parser.add_argument('--settings')
    parser.add_argument('--serial')
    parser.add_argument('--file-camera')
    parser.add_argument('--calibration')
    args=parser.parse_args()
    os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'runtime/ultralytics'))
    logging.basicConfig(level=logging.INFO,format='%(asctime)s %(levelname)s %(message)s')
    serve(Service(settings_file=args.settings,serial=args.serial,file_camera=args.file_camera,calibration_file=args.calibration),args.host,args.udp_port,args.http_port)

if __name__=='__main__': main()
