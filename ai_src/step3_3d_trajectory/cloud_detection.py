"""Stream an uploaded RGB PLY and return recognition metadata for Unity."""

from collections import OrderedDict
import logging
from pathlib import Path
import re
from tempfile import NamedTemporaryFile
from threading import Lock, Thread
import time
from uuid import uuid4


LOG = logging.getLogger("cr10a")
POST_PATH = "/api/detect/ply"
JOB_PREFIX = POST_PATH + "/"
MAX_BYTES = 80 * 1024 * 1024


class CloudDetection:
    def __init__(self, service):
        self.service = service
        self.jobs = OrderedDict()
        self.jobs_lock = Lock()

    def submit(self, request):
        try:
            size = int(request.headers.get("Content-Length", "0"))
        except ValueError:
            size = 0
        if not 0 < size <= MAX_BYTES:
            request.reply({"message": "Cần nội dung PLY có kích thước tối đa 80 MiB."}, 413)
            return
        if request.headers.get_content_type() != "application/octet-stream":
            request.reply({"message": "Cần nội dung PLY dạng application/octet-stream."}, 415)
            return
        if not self.service.lock.acquire(blocking=False):
            request.reply({"message": "AI đang xử lý yêu cầu khác. Hãy thử lại sau."}, 409)
            return

        upload = None
        started = False
        job_id = uuid4().hex
        try:
            backend = self.service.backend
            if backend is None:
                request.reply({"message": "Model AI chưa sẵn sàng. Kiểm tra Ping và log."}, 503)
                return
            request.connection.settimeout(25)
            with NamedTemporaryFile(prefix="robot3t-cloud-", suffix=".ply", delete=False) as file:
                upload = Path(file.name)
                first = request.rfile.readline(min(size, 8))
                if first.rstrip(b"\r\n") != b"ply":
                    request.reply({"message": "Nội dung gửi lên không phải file PLY."}, 400)
                    return
                file.write(first)
                remaining = size - len(first)
                while remaining:
                    chunk = request.rfile.read(min(remaining, 256 * 1024))
                    if not chunk:
                        raise ValueError("File PLY tải lên chưa đủ dữ liệu.")
                    file.write(chunk)
                    remaining -= len(chunk)

            self.record(job_id, {"state": "processing"})
            Thread(target=self.detect, args=(job_id, upload, backend), name="CloudPlyDetection", daemon=True).start()
            started = True
            request.reply({"state": "processing", "job_id": job_id}, 202)
        except (ValueError, OSError) as exc:
            if started:
                LOG.info("Cloud detection accepted, but the client disconnected: %s", job_id)
            else:
                LOG.warning("PLY upload failed: %s", exc)
                try:
                    request.reply({"message": "Tải PLY lên thất bại. Hãy gửi lại file."}, 400)
                except OSError:
                    pass
        finally:
            if not started:
                if upload is not None:
                    upload.unlink(missing_ok=True)
                self.service.lock.release()

    def detect(self, job_id, upload, backend):
        start = time.monotonic()
        try:
            LOG.info("Cloud recognition started: %s (%d bytes)", job_id, upload.stat().st_size)
            landmarks, *_ = backend._export_landmarks_and_segmentation_3d(
                job_id, ply_path=upload, save_outputs=False)
            if len(landmarks.get("landmarks", [])) != 12 or len(landmarks.get("garment_contour_3d", [])) < 3:
                raise ValueError("Không nhận diện được đủ 12 mốc và đường bao quần.")
            import json
            self.record(job_id, {"state": "done", "garment_size": landmarks["detected_size"],
                                 "landmarks_json": json.dumps(landmarks, ensure_ascii=False, allow_nan=False),
                                 "time_generation_s": time.monotonic() - start})
            LOG.info("Cloud recognition completed: %s", job_id)
        except Exception as exc:
            self.record(job_id, {"state": "error", "message": str(exc)})
            LOG.exception("Cloud recognition failed: %s", job_id)
        finally:
            try:
                upload.unlink(missing_ok=True)
            finally:
                self.service.lock.release()

    def record(self, job_id, state):
        with self.jobs_lock:
            self.jobs[job_id] = state
            self.jobs.move_to_end(job_id)
            while len(self.jobs) > 8:
                self.jobs.popitem(last=False)

    def status(self, request, job_id):
        with self.jobs_lock:
            state = self.jobs.get(job_id) if re.fullmatch(r"[0-9a-f]{32}", job_id) else None
        if state is None:
            request.reply({"message": "Phiên nhận diện đã hết. Hãy gửi lại mẫu."}, 404)
        else:
            request.reply(state)
