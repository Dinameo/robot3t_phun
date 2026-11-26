"""Password verification and short-lived browser sessions for the AI dashboard."""

from hashlib import sha256
from hmac import compare_digest
from http.cookies import CookieError, SimpleCookie
from secrets import token_urlsafe
from threading import Lock
from time import monotonic


class DashboardSession:
    cookie_name = "robot3t_ai_session"
    lifetime = 8 * 60 * 60

    def __init__(self, password=None, secure=False):
        self.generated_password = token_urlsafe(18) if password is None else None
        secret = self.generated_password if password is None else password
        self.password_hash = self.digest(secret)
        self.secure = secure
        self.sessions = {}
        self.failures = []
        self.lock = Lock()

    @staticmethod
    def digest(text):
        return sha256(text.encode("utf-8", errors="surrogatepass")).digest()

    def read_token(self, request):
        try:
            cookie = SimpleCookie(request.headers.get("Cookie", ""))
        except CookieError:
            return None
        value = cookie.get(self.cookie_name)
        return value.value if value is not None else None

    def is_signed_in(self, request):
        token = self.read_token(request)
        if token is None:
            return False
        with self.lock:
            return self.sessions.get(self.digest(token), 0) > monotonic()

    def sign_in(self, username, password, previous_token=None):
        now = monotonic()
        with self.lock:
            self.failures = [stamp for stamp in self.failures if stamp > now - 60]
            if len(self.failures) >= 6:
                return 429, None
            if (not isinstance(username, str) or not isinstance(password, str) or
                    not compare_digest(self.digest("admin"), self.digest(username)) or
                    not compare_digest(self.password_hash, self.digest(password))):
                self.failures.append(now)
                return 401, None

            self.sessions = {key: expiry for key, expiry in self.sessions.items() if expiry > now}
            if previous_token:
                self.sessions.pop(self.digest(previous_token), None)
            if len(self.sessions) >= 24:
                self.sessions.pop(min(self.sessions, key=self.sessions.get))
            token = token_urlsafe(32)
            self.sessions[self.digest(token)] = now + self.lifetime
            self.failures.clear()
            return 200, token

    def sign_out(self, request):
        token = self.read_token(request)
        if token is not None:
            with self.lock:
                self.sessions.pop(self.digest(token), None)

    def cookie_header(self, token=""):
        max_age = self.lifetime if token else 0
        secure = "; Secure" if self.secure else ""
        return f"{self.cookie_name}={token}; Path=/dashboard; Max-Age={max_age}; HttpOnly; SameSite=Strict{secure}"
