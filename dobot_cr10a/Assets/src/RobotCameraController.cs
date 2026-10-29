using UnityEngine;

/// <summary>
/// Camera Controller chuyên dụng cho mô phỏng cánh tay robot Dobot CR10A.
/// Hỗ trợ tương tác trực tiếp tự do trong khu vực Unity 3D Viewport:
/// 1. Chuột trái kéo: Xoay (Orbit 360 độ quanh tâm)
/// 2. Chuột phải kéo: Xoay (Orbit 360 độ)
/// 3. Chuột giữa kéo HOẶC Shift + Kéo chuột: Trượt camera (Pan)
/// 4. Lăn chuột (Scroll Wheel) hoặc phím +/-: Phóng to / Thu nhỏ (Zoom)
/// 5. Phím A/D/Left/Right: Xoay trái/phải; W/S/Up/Down: Nâng hạ góc nhìn
/// 6. Thanh công cụ HUD nổi tiện ích ở góc màn hình 3D
/// </summary>
[ExecuteAlways]
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

    [Header("Tùy chọn cuộn chuột")]
    public bool invertScroll = false;

    private float currentX = 45.0f;
    private float currentY = 25.0f;

    private float targetX = 45.0f;
    private float targetY = 25.0f;
    private float targetDistance = 2.0f;

    private Vector3 targetPivotOffset = new Vector3(0f, 0.45f, 0f);
    private Vector3 currentPivotPosition;

    private Vector3 dragOrigin;
    private Camera cam;
    private bool isInitialized = false;

    private void Awake()
    {
        InitializeIfNeeded();
    }

    private void Start()
    {
        InitializeIfNeeded();
    }

    public void InitializeIfNeeded()
    {
        if (isInitialized) return;
        isInitialized = true;

        cam = GetComponent<Camera>();
        if (cam == null) cam = Camera.main;

        // Tối ưu các thông số camera cho robot
        if (cam != null)
        {
            cam.nearClipPlane = 0.03f; // Giúp zoom sát ngòi bút không bị cắt hình
            cam.farClipPlane = 100.0f;
            cam.fieldOfView = 55.0f;
        }

        FindTargets();

        invertScroll = PlayerPrefs.GetInt("CR10A_Camera_InvertScroll", 0) == 1;

        targetDistance = distance > 0.1f ? distance : 2.0f;
        currentPivotPosition = GetPivotPosition();
    }

    public void FindTargets()
    {
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
            GameObject tool = GameObject.Find("Link6") ?? GameObject.Find("axis5") ?? GameObject.Find("tcp");
            if (tool != null)
            {
                toolTipTransform = tool.transform;
            }
        }
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

    private bool isDraggingViewport = false;
    private bool isDraggingGui = false;

    public bool IsMouseOverUI(Vector2 mouseScreenPos)
    {
        // mouseScreenPos theo hệ tọa độ OnGUI (gốc 0,0 ở góc trên bên trái)
        float totalRibbonH = 114f;
        float dockW = 470f;
        bool anyDockOpen = true;

        var ikCtrl = FindAnyObjectByType<DobotCR10AIKController>(FindObjectsInactive.Exclude);
        if (ikCtrl != null && ikCtrl.enabled)
        {
            totalRibbonH = ikCtrl.ribbonExpanded ? 114f : 38f;
            anyDockOpen = ikCtrl.showJointsPanel || ikCtrl.showFkMonitorPanel || ikCtrl.showIkTargetPanel ||
                          ikCtrl.showTrajectoryPanel || ikCtrl.showPipelinePanel || ikCtrl.showEvaluationPanel;
        }
        else
        {
            var man = FindAnyObjectByType<manual>(FindObjectsInactive.Exclude);
            if (man != null && man.enabled)
            {
                totalRibbonH = man.ribbonExpanded ? 114f : 38f;
                anyDockOpen = man.showJointsPanel || man.showFkMonitorPanel || man.showIkTargetPanel;
                dockW = 460f;
            }
        }

        // 1. Khu vực Dải Ruy Băng trên cùng
        if (mouseScreenPos.y < totalRibbonH) return true;

        // 2. Khu vực Dock Inspector bên phải
        if (anyDockOpen && mouseScreenPos.x >= (Screen.width - dockW - 16f)) return true;

        // 3. Thanh công cụ HUD nổi ở góc dưới bên trái
        if (mouseScreenPos.x >= 10f && mouseScreenPos.x <= 540f && mouseScreenPos.y >= (Screen.height - 42f)) return true;

        return false;
    }

    public void ZoomIn(float step = 0.20f)
    {
        targetDistance = Mathf.Clamp(targetDistance - step * targetDistance, minDistance, maxDistance);
        UpdateCameraTransform();
    }

    public void ZoomOut(float step = 0.20f)
    {
        targetDistance = Mathf.Clamp(targetDistance + step * targetDistance, minDistance, maxDistance);
        UpdateCameraTransform();
    }

    private void HandleInput()
    {
        // Phím tắt góc nhìn nhanh (1, 2, 3, 4, F, Space)
        if (Input.GetKeyDown(KeyCode.Alpha1)) SetPresetPerspective();
        if (Input.GetKeyDown(KeyCode.Alpha2)) SetPresetTopDown();
        if (Input.GetKeyDown(KeyCode.Alpha3)) SetPresetSide();
        if (Input.GetKeyDown(KeyCode.Alpha4)) SetPresetFront();
        if (Input.GetKeyDown(KeyCode.F)) ResetFocus();
        if (Input.GetKeyDown(KeyCode.Space)) followTool = !followTool;

        Vector2 mouseGuiPos = new Vector2(Input.mousePosition.x, Screen.height - Input.mousePosition.y);
        bool mouseOverUI = IsMouseOverUI(mouseGuiPos);

        if ((Input.GetMouseButtonDown(0) || Input.GetMouseButtonDown(1) || Input.GetMouseButtonDown(2)) && !mouseOverUI)
        {
            isDraggingViewport = true;
        }
        if (!Input.GetMouseButton(0) && !Input.GetMouseButton(1) && !Input.GetMouseButton(2))
        {
            isDraggingViewport = false;
        }

        float dt = Application.isPlaying ? Time.deltaTime : 0.02f;

        // 1. XOAY (ORBIT 360°): Chuột trái HOẶC Chuột phải kéo trong khu vực 3D
        bool isOrbiting = isDraggingViewport && (Input.GetMouseButton(0) || Input.GetMouseButton(1)) && !Input.GetKey(KeyCode.LeftShift);
        if (isOrbiting)
        {
            float deltaX = Input.GetAxis("Mouse X");
            float deltaY = Input.GetAxis("Mouse Y");
            targetX += deltaX * rotateSpeed * dt * 10f;
            targetY -= deltaY * rotateSpeed * dt * 10f;
            targetY = Mathf.Clamp(targetY, minVerticalAngle, maxVerticalAngle);
        }

        // 2. TRƯỢT (PAN): Chuột giữa kéo HOẶC Shift + Chuột trái / Shift + Chuột phải
        bool isPanning = isDraggingViewport && (Input.GetMouseButton(2) || (Input.GetKey(KeyCode.LeftShift) && (Input.GetMouseButton(0) || Input.GetMouseButton(1))));
        if (isPanning)
        {
            float panX = -Input.GetAxis("Mouse X") * panSpeed * (targetDistance * 0.4f);
            float panY = -Input.GetAxis("Mouse Y") * panSpeed * (targetDistance * 0.4f);

            Vector3 right = transform.right * panX;
            Vector3 up = transform.up * panY;

            targetPivotOffset += right + up;
        }

        // 3. THU / PHÓNG (ZOOM): Con lăn chuột khi con trỏ ở trong khu vực 3D
        if (!mouseOverUI)
        {
            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (invertScroll) scroll = -scroll;
            if (Mathf.Abs(scroll) > 0.0001f)
            {
                targetDistance -= scroll * zoomSpeed * targetDistance;
                targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
            }
        }

        // 4. Phím tắt tương tác trực tiếp
        if (Input.GetKey(KeyCode.Equals) || Input.GetKey(KeyCode.KeypadPlus)) ZoomIn(0.04f);
        if (Input.GetKey(KeyCode.Minus) || Input.GetKey(KeyCode.KeypadMinus)) ZoomOut(0.04f);
        if (Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.A)) targetX -= rotateSpeed * dt * 0.8f;
        if (Input.GetKey(KeyCode.RightArrow) || Input.GetKey(KeyCode.D)) targetX += rotateSpeed * dt * 0.8f;
        if (Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.W)) targetY = Mathf.Clamp(targetY + rotateSpeed * dt * 0.5f, minVerticalAngle, maxVerticalAngle);
        if (Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.S)) targetY = Mathf.Clamp(targetY - rotateSpeed * dt * 0.5f, minVerticalAngle, maxVerticalAngle);
    }

    public void UpdateCameraTransform()
    {
        float factor = smoothTime > 0.001f ? Mathf.Clamp01(Time.deltaTime / smoothTime) : 1.0f;

        // Làm mượt góc quay và khoảng cách
        currentX = Mathf.Lerp(currentX, targetX, factor);
        currentY = Mathf.Lerp(currentY, targetY, factor);
        distance = Mathf.Lerp(distance, targetDistance, factor);

        Vector3 desiredPivot = GetPivotPosition();
        currentPivotPosition = Vector3.Lerp(currentPivotPosition, desiredPivot, factor);

        // Tính vị trí camera
        Quaternion rotation = Quaternion.Euler(currentY, currentX, 0);
        Vector3 negDistance = new Vector3(0.0f, 0.0f, -distance);
        Vector3 position = rotation * negDistance + currentPivotPosition;

        transform.rotation = rotation;
        transform.position = position;
    }

    [Tooltip("Phôi đồ / Mẫu quần áo (Garment / Workpiece)")]
    public Transform garmentTransform;

    public bool showDebugOnGui = false;

    public Vector3 GetWorkpieceCenter()
    {
        if (garmentTransform == null)
        {
            GameObject g = GameObject.Find("Garment_PLY_Model") ?? GameObject.Find("CloudChunk_0") ?? GameObject.Find("tcp");
            if (g != null) garmentTransform = g.transform;
        }

        if (garmentTransform != null)
        {
            Renderer rend = garmentTransform.GetComponentInChildren<Renderer>();
            if (rend != null && rend.bounds.size.sqrMagnitude > 0.01f)
                return rend.bounds.center;
            if (garmentTransform.position.sqrMagnitude > 0.01f)
                return garmentTransform.position;
        }
        return new Vector3(-0.10f, 0.10f, 0.65f);
    }

    public void ApplyInstantPreset()
    {
        currentX = targetX;
        currentY = targetY;
        distance = targetDistance;
        currentPivotPosition = GetPivotPosition();

        Quaternion rotation = Quaternion.Euler(currentY, currentX, 0);
        Vector3 negDistance = new Vector3(0.0f, 0.0f, -distance);
        transform.rotation = rotation;
        transform.position = rotation * negDistance + currentPivotPosition;

        if (cam != null && cam.transform != transform)
        {
            cam.transform.rotation = rotation;
            cam.transform.position = transform.position;
        }
    }

    // --- CÁC GÓC NHÌN ĐẶT TRƯỚC (PRESETS) ---

    /// <summary>
    /// Góc nhìn toàn cảnh 3/4 (Perspective Overview - Bao quát cả robot và cái đồ)
    /// </summary>
    public void SetPresetPerspective(bool instant = true)
    {
        InitializeIfNeeded();
        Vector3 wpCenter = GetWorkpieceCenter();
        Vector3 basePos = target != null ? target.position : Vector3.zero;
        Vector3 center = (basePos + wpCenter) * 0.5f;
        center.y += 0.20f;
        targetPivotOffset = (target != null) ? center - target.position : center;
        targetX = 35.0f;
        targetY = 25.0f;
        targetDistance = 1.9f;
        if (instant) ApplyInstantPreset();
    }

    /// <summary>
    /// Góc nhìn từ trên xuống (Top-down - Nhìn thẳng từ trên trần xuống cái đồ / bàn thao tác)
    /// </summary>
    public void SetPresetTopDown(bool instant = true)
    {
        InitializeIfNeeded();
        Vector3 wpCenter = GetWorkpieceCenter();
        targetPivotOffset = (target != null) ? wpCenter - target.position : wpCenter;
        targetX = 0.0f;
        targetY = 88.0f; // Nhìn thẳng từ trên xuống vuông góc bàn
        targetDistance = 1.4f;
        if (instant) ApplyInstantPreset();
    }

    /// <summary>
    /// Góc nhìn ngang sườn (Side View - Quan sát độ cao súng phun và độ dốc của đồ)
    /// </summary>
    public void SetPresetSide(bool instant = true)
    {
        InitializeIfNeeded();
        Vector3 wpCenter = GetWorkpieceCenter();
        targetPivotOffset = (target != null) ? wpCenter - target.position : wpCenter;
        targetX = 75.0f;
        targetY = 18.0f;
        targetDistance = 1.5f;
        if (instant) ApplyInstantPreset();
    }

    /// <summary>
    /// Góc nhìn chính diện từ trước cái đồ (Front View - Nhìn trực diện vào mặt trước cái đồ hướng về robot)
    /// </summary>
    public void SetPresetFront(bool instant = true)
    {
        InitializeIfNeeded();
        Vector3 wpCenter = GetWorkpieceCenter();
        targetPivotOffset = (target != null) ? wpCenter - target.position : wpCenter;
        targetX = 0.0f;  // Nhìn thẳng trực diện từ trước đồ (+Z) hướng vào đồ
        targetY = 28.0f; // Góc nghiêng nhìn rõ bề mặt trên của đồ
        targetDistance = 1.4f;
        if (instant) ApplyInstantPreset();
    }

    /// <summary>
    /// Đưa tâm quan sát về lại tâm robot
    /// </summary>
    public void ResetFocus(bool instant = true)
    {
        InitializeIfNeeded();
        targetPivotOffset = new Vector3(0f, 0.45f, 0f);
        targetX = 35.0f;
        targetY = 25.0f;
        targetDistance = 1.8f;
        if (instant) ApplyInstantPreset();
    }

    /// <summary>
    /// Focus trực tiếp vào tâm cái đồ (Garment Workpiece)
    /// </summary>
    public void FocusGarment(bool instant = true)
    {
        InitializeIfNeeded();
        Vector3 wpCenter = GetWorkpieceCenter();
        targetPivotOffset = (target != null) ? wpCenter - target.position : wpCenter;
        targetX = 0.0f;
        targetY = 32.0f;
        targetDistance = 1.0f;
        if (instant) ApplyInstantPreset();
    }

    private void OnGUI()
    {
        AntigravityUITheme.EnsureInitialized();
        Event e = Event.current;

        // Xử lý sự kiện chuột trực tiếp trong OnGUI để phản hồi tức thì
        if (e != null)
        {
            if (e.type == EventType.MouseDown && !IsMouseOverUI(e.mousePosition))
            {
                isDraggingGui = true;
                e.Use();
            }
            else if (e.type == EventType.MouseUp)
            {
                isDraggingGui = false;
            }
            else if (e.type == EventType.MouseDrag && isDraggingGui)
            {
                if (e.button == 0 || e.button == 1)
                {
                    if (e.shift)
                    {
                        // Pan
                        float panFactor = targetDistance * 0.0022f;
                        targetPivotOffset += (-transform.right * e.delta.x + transform.up * e.delta.y) * panFactor;
                    }
                    else
                    {
                        // Orbit xoay 360 độ
                        targetX += e.delta.x * 0.50f;
                        targetY -= e.delta.y * 0.50f;
                        targetY = Mathf.Clamp(targetY, minVerticalAngle, maxVerticalAngle);
                    }
                    UpdateCameraTransform();
                    e.Use();
                }
                else if (e.button == 2)
                {
                    // Pan bằng chuột giữa
                    float panFactor = targetDistance * 0.0022f;
                    targetPivotOffset += (-transform.right * e.delta.x + transform.up * e.delta.y) * panFactor;
                    UpdateCameraTransform();
                    e.Use();
                }
            }
            else if (e.type == EventType.ScrollWheel && !IsMouseOverUI(e.mousePosition))
            {
                float scroll = e.delta.y * (invertScroll ? -1f : 1f);
                targetDistance += scroll * 0.08f * targetDistance;
                targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
                UpdateCameraTransform();
                e.Use();
            }
        }

        // Vẽ Floating HUD ở góc dưới màn hình 3D
        DrawFloatingViewportToolbar();
    }

    private void DrawFloatingViewportToolbar()
    {
        float hudW = 540f;
        float hudH = 34f;
        float hudX = 14f;
        float hudY = Screen.height - hudH - 8f;

        // Nền mờ kính tối phong cách hiện đại
        GUI.DrawTexture(new Rect(hudX, hudY, hudW, hudH), AntigravityUITheme.TexBgHeader);
        GUI.DrawTexture(new Rect(hudX, hudY, hudW, 1), AntigravityUITheme.TexBorder);
        GUI.DrawTexture(new Rect(hudX, hudY + hudH - 1, hudW, 1), AntigravityUITheme.TexBorder);

        GUILayout.BeginArea(new Rect(hudX + 8f, hudY + 4f, hudW - 16f, hudH - 8f));
        GUILayout.BeginHorizontal();

        GUILayout.Label("CAMERA 3D:", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(76));

        if (GUILayout.Button("+ Phóng", AntigravityUITheme.StyleButton, GUILayout.Width(58), GUILayout.Height(24))) ZoomIn();
        if (GUILayout.Button("- Thu", AntigravityUITheme.StyleButton, GUILayout.Width(48), GUILayout.Height(24))) ZoomOut();
        if (GUILayout.Button("Góc 3/4", AntigravityUITheme.StyleButton, GUILayout.Width(56), GUILayout.Height(24))) SetPresetPerspective();
        if (GUILayout.Button("Trên Đồ", AntigravityUITheme.StyleButton, GUILayout.Width(58), GUILayout.Height(24))) SetPresetTopDown();
        if (GUILayout.Button("Trước Đồ", AntigravityUITheme.StyleButton, GUILayout.Width(62), GUILayout.Height(24))) SetPresetFront();
        if (GUILayout.Button("Reset Focus", AntigravityUITheme.StyleButton, GUILayout.Width(76), GUILayout.Height(24))) ResetFocus();

        // Tùy chọn bật/tắt cuộn ngược chuột và lưu setting
        GUIStyle invertBtnStyle = invertScroll ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button(invertScroll ? "Cuộn: Ngược" : "Cuộn: Chuẩn", invertBtnStyle, GUILayout.Width(86), GUILayout.Height(24)))
        {
            invertScroll = !invertScroll;
            PlayerPrefs.SetInt("CR10A_Camera_InvertScroll", invertScroll ? 1 : 0);
            PlayerPrefs.Save();
        }

        GUILayout.EndHorizontal();
        GUILayout.EndArea();
    }
}
