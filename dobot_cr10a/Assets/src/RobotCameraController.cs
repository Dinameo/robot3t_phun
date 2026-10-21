using UnityEngine;

/// <summary>
/// Camera Controller chuyên dụng cho mô phỏng cánh tay robot Dobot CR10A.
/// Hỗ trợ:
/// 1. Orbit (Xoay 360 độ quanh Robot): Giữ chuột phải (Right Click) hoặc Alt + Chuột trái.
/// 2. Zoom (Phóng to / Thu nhỏ): Lăn chuột (Scroll Wheel).
/// 3. Pan (Trượt camera): Giữ chuột giữa (Middle Click) hoặc Shift + Chuột phải.
/// 4. Follow Target: Tự động bám theo khớp hoặc đầu công cụ (Link6 / TCP).
/// 5. Phím tắt góc nhìn nhanh (Presets):
///    - Phím 1: Góc nhìn 3/4 Perspective (Toàn cảnh)
///    - Phím 2: Góc nhìn Top-Down (Từ trên nhìn xuống)
///    - Phím 3: Góc nhìn Side (Ngang)
///    - Phím 4: Góc nhìn Front (Chính diện)
///    - Phím F: Focus vào tâm Robot
///    - Phím Space: Bật/Tắt bám theo đầu công cụ (Follow Tool)
/// </summary>
public class RobotCameraController : MonoBehaviour
{
    [Header("Mục tiêu quan sát (Target)")]
    [Tooltip("Tâm quan sát (mặc định sẽ tự tìm Robot Base hoặc Link6)")]
    public Transform target;

    [Tooltip("Đầu công cụ robot (Link6 hoặc Tool TCP)")]
    public Transform toolTipTransform;

    [Tooltip("Tự động bám theo đầu công cụ khi robot chuyển động")]
    public bool followTool = false;

    [Header("Khoảng cách & Góc đặt (Orbit Settings)")]
    [Tooltip("Khoảng cách từ camera tới tâm")]
    public float distance = 2.2f;
    public float minDistance = 0.2f;
    public float maxDistance = 6.0f;

    [Tooltip("Tốc độ xoay camera")]
    public float rotateSpeed = 120.0f;

    [Tooltip("Tốc độ phóng to thu nhỏ")]
    public float zoomSpeed = 2.5f;

    [Tooltip("Tốc độ trượt (Pan)")]
    public float panSpeed = 0.35f;

    [Tooltip("Độ mượt mà chuyển động (Damping)")]
    public float smoothTime = 0.08f;

    [Header("Giới hạn góc nghiêng")]
    public float minVerticalAngle = -10.0f;
    public float maxVerticalAngle = 88.0f;

    private float currentX = 45.0f;
    private float currentY = 25.0f;

    private float targetX = 45.0f;
    private float targetY = 25.0f;
    private float targetDistance;

    private Vector3 targetPivotOffset = new Vector3(0f, 0.45f, 0f);
    private Vector3 currentPivotPosition;

    private Vector3 dragOrigin;
    private Camera cam;

    private void Start()
    {
        cam = GetComponent<Camera>();
        if (cam == null) cam = Camera.main;

        // Tối ưu các thông số camera cho robot
        if (cam != null)
        {
            cam.nearClipPlane = 0.03f; // Giúp zoom sát ngòi bút không bị cắt hình
            cam.farClipPlane = 100.0f;
            cam.fieldOfView = 55.0f;
        }

        // Tự động tìm target nếu chưa gán
        if (target == null)
        {
            GameObject robot = GameObject.Find("Base") ?? GameObject.Find("axis0") ?? GameObject.Find("manual");
            if (robot != null)
            {
                target = robot.transform;
            }
        }

        // Tự động tìm đầu công cụ (Link6 hoặc axis5)
        if (toolTipTransform == null)
        {
            GameObject tool = GameObject.Find("Link6") ?? GameObject.Find("axis5");
            if (tool != null)
            {
                toolTipTransform = tool.transform;
            }
        }

        targetDistance = distance;
        currentPivotPosition = GetPivotPosition();

        // Khởi tạo góc nhìn mặc định 3/4
        SetPresetPerspective();
    }

    private void LateUpdate()
    {
        HandleInput();
        UpdateCameraTransform();
    }

    private Vector3 GetPivotPosition()
    {
        if (followTool && toolTipTransform != null)
        {
            return toolTipTransform.position;
        }
        else if (target != null)
        {
            return target.position + targetPivotOffset;
        }
        return targetPivotOffset;
    }

    private void HandleInput()
    {
        // 1. Phím tắt góc nhìn nhanh (1, 2, 3, 4, F, Space)
        if (Input.GetKeyDown(KeyCode.Alpha1)) SetPresetPerspective();
        if (Input.GetKeyDown(KeyCode.Alpha2)) SetPresetTopDown();
        if (Input.GetKeyDown(KeyCode.Alpha3)) SetPresetSide();
        if (Input.GetKeyDown(KeyCode.Alpha4)) SetPresetFront();
        if (Input.GetKeyDown(KeyCode.F)) ResetFocus();
        if (Input.GetKeyDown(KeyCode.Space)) followTool = !followTool;

        // 2. Xoay (Orbit) bằng chuột phải hoặc Alt + Chuột trái
        bool isOrbiting = Input.GetMouseButton(1) || (Input.GetKey(KeyCode.LeftAlt) && Input.GetMouseButton(0));
        if (isOrbiting)
        {
            targetX += Input.GetAxis("Mouse X") * rotateSpeed * Time.deltaTime * 10f;
            targetY -= Input.GetAxis("Mouse Y") * rotateSpeed * Time.deltaTime * 10f;
            targetY = Mathf.Clamp(targetY, minVerticalAngle, maxVerticalAngle);
        }

        // 3. Phóng to / Thu nhỏ (Zoom) bằng con lăn chuột
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.001f)
        {
            targetDistance -= scroll * zoomSpeed * targetDistance;
            targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
        }

        // 4. Trượt (Pan) bằng chuột giữa hoặc Shift + Chuột phải
        bool isPanning = Input.GetMouseButton(2) || (Input.GetKey(KeyCode.LeftShift) && Input.GetMouseButton(1));
        if (isPanning)
        {
            float panX = -Input.GetAxis("Mouse X") * panSpeed * (targetDistance * 0.4f);
            float panY = -Input.GetAxis("Mouse Y") * panSpeed * (targetDistance * 0.4f);

            Vector3 right = transform.right * panX;
            Vector3 up = transform.up * panY;

            targetPivotOffset += right + up;
        }
    }

    private void UpdateCameraTransform()
    {
        // Làm mượt góc quay và khoảng cách
        currentX = Mathf.Lerp(currentX, targetX, Time.deltaTime / smoothTime);
        currentY = Mathf.Lerp(currentY, targetY, Time.deltaTime / smoothTime);
        distance = Mathf.Lerp(distance, targetDistance, Time.deltaTime / smoothTime);

        Vector3 desiredPivot = GetPivotPosition();
        currentPivotPosition = Vector3.Lerp(currentPivotPosition, desiredPivot, Time.deltaTime / smoothTime);

        // Tính vị trí camera
        Quaternion rotation = Quaternion.Euler(currentY, currentX, 0);
        Vector3 negDistance = new Vector3(0.0f, 0.0f, -distance);
        Vector3 position = rotation * negDistance + currentPivotPosition;

        transform.rotation = rotation;
        transform.position = position;
    }

    // --- CÁC GÓC NHÌN ĐẶT TRƯỚC (PRESETS) ---

    /// <summary>
    /// Góc nhìn toàn cảnh 3/4 (Perspective Overview - Đẹp nhất để xem toàn bộ thao tác)
    /// </summary>
    public void SetPresetPerspective()
    {
        targetX = 45.0f;
        targetY = 25.0f;
        targetDistance = 2.0f;
        targetPivotOffset = new Vector3(0f, 0.45f, 0f);
    }

    /// <summary>
    /// Góc nhìn từ trên xuống (Top-down - Thuận tiện quan sát quỹ đạo vẽ 2D/3D trên bàn)
    /// </summary>
    public void SetPresetTopDown()
    {
        targetX = 0.0f;
        targetY = 85.0f;
        targetDistance = 1.8f;
        targetPivotOffset = new Vector3(0f, 0.45f, 0f);
    }

    /// <summary>
    /// Góc nhìn ngang sườn (Side View - Quan sát độ cao Z và góc cắm bút)
    /// </summary>
    public void SetPresetSide()
    {
        targetX = 90.0f;
        targetY = 5.0f;
        targetDistance = 1.9f;
        targetPivotOffset = new Vector3(0f, 0.45f, 0f);
    }

    /// <summary>
    /// Góc nhìn chính diện (Front View)
    /// </summary>
    public void SetPresetFront()
    {
        targetX = 0.0f;
        targetY = 15.0f;
        targetDistance = 2.0f;
        targetPivotOffset = new Vector3(0f, 0.45f, 0f);
    }

    /// <summary>
    /// Đưa tâm quan sát về lại tâm robot
    /// </summary>
    public void ResetFocus()
    {
        targetPivotOffset = new Vector3(0f, 0.45f, 0f);
    }

    private void OnGUI()
    {
        // Hiển thị thanh hướng dẫn phím tắt ở góc màn hình Game
        GUIStyle style = new GUIStyle(GUI.skin.box);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = 12;
        style.normal.textColor = Color.white;

        string info = "<b>DOBOT CR10A CAMERA CONTROLS:</b>\n" +
                      "• Chuột phải / Alt+Trái: Xoay (Orbit 360°)\n" +
                      "• Lăn chuột: Phóng to/Thu nhỏ (Zoom)\n" +
                      "• Chuột giữa: Trượt camera (Pan)\n" +
                      "• [1] Toàn cảnh 3/4  |  [2] Nhìn trên xuống\n" +
                      "• [3] Nhìn ngang     |  [4] Nhìn chính diện\n" +
                      "• [Space] Theo đầu bút: " + (followTool ? "<color=green>BẬT</color>" : "<color=yellow>TẮT</color>");

        GUI.Box(new Rect(10, 10, 270, 120), info, style);
    }
}
