const form = document.getElementById("login-form");
const password = document.getElementById("password");
const toggle = document.getElementById("show-password");
const button = document.getElementById("login-submit");
const feedback = document.getElementById("login-feedback");

toggle.addEventListener("click", () => {
  const visible = password.type === "password";
  password.type = visible ? "text" : "password";
  toggle.textContent = visible ? "Ẩn" : "Hiện";
  toggle.setAttribute("aria-pressed", String(visible));
});

form.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (button.disabled) return;
  button.disabled = true;
  button.textContent = "Đang đăng nhập…";
  feedback.textContent = "";
  try {
    const response = await fetch("/dashboard/api/login", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ username: form.elements.username.value, password: password.value }),
    });
    if (response.ok) {
      password.value = "";
      window.location.replace("/dashboard");
      return;
    }
    const result = await response.json();
    throw new Error(result.message);
  } catch (error) {
    feedback.dataset.error = "true";
    feedback.textContent = error.message || "Không kết nối được AI Server.";
    password.focus();
    password.select();
  } finally {
    button.disabled = false;
    button.textContent = "Đăng nhập";
  }
});
