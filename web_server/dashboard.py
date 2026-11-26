"""Local model and log dashboard for run_server.py."""

from collections import deque
from contextlib import contextmanager
from datetime import datetime
import logging
from pathlib import Path
import sys
import threading
from urllib.parse import parse_qs, urlsplit

from web_server.session import DashboardSession


ASSETS = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/dashboard": ("index.html", "text/html; charset=utf-8"),
    "/dashboard/": ("index.html", "text/html; charset=utf-8"),
    "/dashboard/styles.css": ("styles.css", "text/css; charset=utf-8"),
    "/dashboard/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/dashboard/login": ("login.html", "text/html; charset=utf-8"),
    "/dashboard/login.js": ("login.js", "text/javascript; charset=utf-8"),
}

def list_model_files(directory, suffix=".pt"):
    directory = directory.resolve()
    return {
        path.name: path
        for path in sorted(directory.glob(f"*{suffix}"), key=lambda path: path.name.casefold())
        if path.is_file() and path.resolve().parent == directory
    }


class RecentLogs(logging.Handler):
    def __init__(self):
        super().__init__(logging.INFO)
        self.entries = deque(maxlen=500)
        self.cursor = 0

    def add(self, level, message):
        with self.lock:
            self.cursor += 1
            self.entries.append({
                "id": self.cursor,
                "time": datetime.now().astimezone().isoformat(timespec="seconds"),
                "level": level,
                "message": message[:4000],
            })

    def emit(self, record):
        self.add(record.levelname, self.format(record))

    def read(self, after):
        with self.lock:
            # A browser may still hold a cursor from before a server restart.
            reset = after > self.cursor or (self.entries and after < self.entries[0]["id"] - 1)
            return {
                "entries": [entry for entry in self.entries if reset or entry["id"] > after],
                "cursor": self.cursor,
                "reset": bool(reset),
            }


class ConsoleMirror:
    def __init__(self, stream, logs):
        self.stream = stream
        self.logs = logs
        self.local = threading.local()

    def write(self, text):
        written = self.stream.write(text)
        lines = (getattr(self.local, "partial", "") + text).split("\n")
        self.local.partial = lines.pop()[-4000:]
        for line in lines:
            line = line.strip()
            if line:
                self.logs.add("CONSOLE", line)
        return written

    def __getattr__(self, name):
        return getattr(self.stream, name)


class Dashboard:
    def __init__(self, service, password=None, public_origin=None):
        self.service = service
        self.logs = RecentLogs()
        self.public_origin = public_origin
        self.session = DashboardSession(password, secure=public_origin is not None)

    def show_local_credentials(self):
        if self.public_origin is None and self.session.generated_password:
            print(f"Dashboard login | admin | {self.session.generated_password}", flush=True)

    @contextmanager
    def capture_logs(self):
        root = logging.getLogger()
        root.addHandler(self.logs)
        original = sys.stdout
        sys.stdout = ConsoleMirror(original, self.logs)
        try:
            yield
        finally:
            sys.stdout = original
            root.removeHandler(self.logs)
            self.logs.close()

    @staticmethod
    def is_dashboard_path(path):
        return path in ASSETS or path.startswith("/dashboard/")

    def is_local(self, request):
        if self.public_origin is not None:
            return True
        if request.client_address[0] not in {"127.0.0.1", "::1"}:
            return False
        try:
            host = urlsplit("//" + request.headers.get("Host", "")).hostname
        except ValueError:
            return False
        return host in {"localhost", "127.0.0.1", "::1"}

    @staticmethod
    def redirect(request, location):
        request.send_response(303)
        request.send_header("Location", location)
        request.send_header("Cache-Control", "no-store")
        request.send_header("Content-Length", "0")
        request.end_headers()

    def get(self, request):
        target = urlsplit(request.path)
        if not self.is_local(request):
            request.reply({"message": "Mở dashboard trên máy đang chạy server."}, 403)
            return
        path = target.path
        signed_in = self.session.is_signed_in(request)
        if path == "/dashboard/login" and signed_in:
            self.redirect(request, "/dashboard")
        elif path in ("/", "/dashboard", "/dashboard/") and not signed_in:
            self.redirect(request, "/dashboard/login")
        elif path in ASSETS:
            filename, content_type = ASSETS[path]
            body = Path(__file__).with_name(filename).read_bytes()
            request.send_response(200)
            request.send_header("Content-Type", content_type)
            request.send_header("Content-Length", str(len(body)))
            request.send_header("Cache-Control", "no-store")
            request.send_header("X-Content-Type-Options", "nosniff")
            request.send_header("Content-Security-Policy", "default-src 'self'; frame-ancestors 'none'; form-action 'self'")
            request.end_headers()
            request.wfile.write(body)
        elif path == "/dashboard/api/state":
            if not signed_in:
                request.reply({"message": "Phiên đăng nhập đã hết."}, 401)
                return
            try:
                after = max(0, int(parse_qs(target.query).get("after", ["0"])[0]))
            except ValueError:
                request.reply({"message": "Cursor log không hợp lệ."}, 400)
                return
            request.reply({
                **self.service.model_status(),
                "available": list(list_model_files(self.service.weights_dir, self.service.weight_suffix)),
                **self.logs.read(after),
            })
        else:
            request.reply({"message": "Không tìm thấy mục này."}, 404)

    def post(self, request, read_json):
        path = urlsplit(request.path).path
        origin = self.public_origin or ("http://" + request.headers.get("Host", ""))
        if not self.is_local(request) or request.headers.get("Origin") != origin:
            request.reply({"message": "Mở dashboard từ đúng địa chỉ AI Server."}, 403)
            return

        if path == "/dashboard/api/login":
            self.login(request, read_json)
            return
        if path == "/dashboard/api/logout":
            self.session.sign_out(request)
            request.reply({"message": "Đã đăng xuất."},
                          headers={"Set-Cookie": self.session.cookie_header()})
            return
        if path != "/dashboard/api/models":
            request.reply({"message": "Không tìm thấy mục này."}, 404)
            return
        if not self.session.is_signed_in(request):
            request.reply({"message": "Phiên đăng nhập đã hết."}, 401)
            return
        if request.headers.get_content_type() != "application/json":
            request.reply({"message": "Yêu cầu cần có định dạng JSON."}, 415)
            return
        try:
            names = read_json(4096)
            available = list_model_files(self.service.weights_dir, self.service.weight_suffix)
            seg = available[names["segmentation"]]
            pose = available[names["pose"]]
            if seg == pose:
                raise ValueError("Cần hai tệp khác nhau cho Segmentation và Pose.")
        except (ValueError, TypeError, KeyError, UnicodeError):
            request.reply({"message": f"Chọn hai tệp {self.service.weight_suffix} hợp lệ trong thư mục weights."}, 400)
            return
        if not self.service.reload_models_async(seg, pose):
            request.reply({"message": "Server đang nạp model. Hãy đợi lần nạp này hoàn tất."}, 409)
            return
        request.reply({"message": "Đang nạp model…"}, 202)

    def login(self, request, read_json):
        if request.headers.get_content_type() != "application/json":
            request.reply({"message": "Yêu cầu cần có định dạng JSON."}, 415)
            return
        try:
            credentials = read_json(4096)
        except (ValueError, TypeError, UnicodeError):
            request.reply({"message": "Thông tin đăng nhập không hợp lệ."}, 400)
            return

        code, token = self.session.sign_in(
            credentials.get("username"), credentials.get("password"), self.session.read_token(request))
        if code == 429:
            request.reply({"message": "Đã thử quá nhiều lần. Chờ một phút rồi thử lại."}, 429,
                          headers={"Retry-After": "60"})
        elif code == 401:
            request.reply({"message": "Tài khoản hoặc mật khẩu không đúng."}, 401)
        else:
            request.reply({"message": "Đã đăng nhập."},
                          headers={"Set-Cookie": self.session.cookie_header(token)})
