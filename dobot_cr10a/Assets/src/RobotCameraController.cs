using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

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
    public float distance = 2.0f;
    public float minDistance = 0.35f;
    public float maxDistance = 5.0f;

    [Tooltip("Độ nhạy xoay camera (Độ/pixel)")]
    public float rotateSpeed = 0.12f;

    [Tooltip("Tốc độ phóng to thu nhỏ (% mỗi nấc cuộn)")]
    public float zoomSpeed = 0.05f;

    [Tooltip("Độ nhạy trượt (Pan)")]
    public float panSpeed = 0.0009f;

    [Tooltip("Hệ số làm mượt chống trôi quán tính (Càng cao càng dừng tức thì)")]
    public float dampingSharpness = 28.0f;

    [Header("Giới hạn góc nghiêng")]
    public float minVerticalAngle = -10.0f;
    public float maxVerticalAngle = 88.0f;

    [Header("Tùy chọn di chuyển & cuộn chuột")]
    public bool invertScroll = false;
    public bool invertPan = false;

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

    private void OnValidate()
    {
        SanitizeLegacyValues();
    }

    public void SanitizeLegacyValues()
    {
        // Khắc phục giá trị cũ bị Unity lưu cứng trong Inspector/Scene
        if (panSpeed > 0.005f) panSpeed = 0.0009f;
        if (rotateSpeed > 1.0f) rotateSpeed = 0.12f;
        if (zoomSpeed > 0.2f) zoomSpeed = 0.05f;
        if (minDistance < 0.1f) minDistance = 0.35f;
        if (maxDistance > 8.0f) maxDistance = 5.0f;
        if (dampingSharpness < 5.0f || dampingSharpness > 100f) dampingSharpness = 28.0f;
    }

    private void Awake()
    {
        SanitizeLegacyValues();
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
        invertPan = PlayerPrefs.GetInt("CR10A_Camera_InvertPan", 0) == 1;

        targetDistance = Mathf.Clamp(distance, minDistance, maxDistance);
        if (targetDistance < minDistance || targetDistance > maxDistance || float.IsNaN(targetDistance)) targetDistance = 2.0f;
        distance = targetDistance;
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

    private void Update()
    {
        HandleKeyboardShortcuts();
    }

    private void LateUpdate()
    {
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
            totalRibbonH = ikCtrl.ribbonExpanded ? 134f : 38f;
            anyDockOpen = ikCtrl.showDockPanel;
            dockW = 480f;
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
        if (mouseScreenPos.x >= 10f && mouseScreenPos.x <= 630f && mouseScreenPos.y >= (Screen.height - 42f)) return true;

        return false;
    }

    public void ZoomIn(float step = 0.12f)
    {
        targetDistance = Mathf.Clamp(targetDistance - step * targetDistance, minDistance, maxDistance);
        UpdateCameraTransform();
    }

    public void ZoomOut(float step = 0.12f)
    {
        targetDistance = Mathf.Clamp(targetDistance + step * targetDistance, minDistance, maxDistance);
        UpdateCameraTransform();
    }

    private void HandleKeyboardShortcuts()
    {
#if ENABLE_INPUT_SYSTEM
        Keyboard kb = Keyboard.current;
        if (kb != null)
        {
            if (kb.digit1Key.wasPressedThisFrame || kb.numpad1Key.wasPressedThisFrame) SetPresetPerspective();
            if (kb.digit2Key.wasPressedThisFrame || kb.numpad2Key.wasPressedThisFrame) SetPresetTopDown();
            if (kb.digit3Key.wasPressedThisFrame || kb.numpad3Key.wasPressedThisFrame) SetPresetSide();
            if (kb.digit4Key.wasPressedThisFrame || kb.numpad4Key.wasPressedThisFrame) SetPresetFront();
            if (kb.fKey.wasPressedThisFrame) ResetFocus();
            if (kb.spaceKey.wasPressedThisFrame) followTool = !followTool;

            float dt = Application.isPlaying ? Time.deltaTime : 0.02f;
            if (kb.equalsKey.isPressed || kb.numpadPlusKey.isPressed) ZoomIn(0.02f);
            if (kb.minusKey.isPressed || kb.numpadMinusKey.isPressed) ZoomOut(0.02f);
            if (kb.leftArrowKey.isPressed || kb.aKey.isPressed) targetX -= 60f * dt;
            if (kb.rightArrowKey.isPressed || kb.dKey.isPressed) targetX += 60f * dt;
            if (kb.upArrowKey.isPressed || kb.wKey.isPressed) targetY = Mathf.Clamp(targetY + 45f * dt, minVerticalAngle, maxVerticalAngle);
            if (kb.downArrowKey.isPressed || kb.sKey.isPressed) targetY = Mathf.Clamp(targetY - 45f * dt, minVerticalAngle, maxVerticalAngle);
        }
#else
        if (Input.GetKeyDown(KeyCode.Alpha1)) SetPresetPerspective();
        if (Input.GetKeyDown(KeyCode.Alpha2)) SetPresetTopDown();
        if (Input.GetKeyDown(KeyCode.Alpha3)) SetPresetSide();
        if (Input.GetKeyDown(KeyCode.Alpha4)) SetPresetFront();
        if (Input.GetKeyDown(KeyCode.F)) ResetFocus();
        if (Input.GetKeyDown(KeyCode.Space)) followTool = !followTool;

        float dt = Application.isPlaying ? Time.deltaTime : 0.02f;
        if (Input.GetKey(KeyCode.Equals) || Input.GetKey(KeyCode.KeypadPlus)) ZoomIn(0.02f);
        if (Input.GetKey(KeyCode.Minus) || Input.GetKey(KeyCode.KeypadMinus)) ZoomOut(0.02f);
        if (Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.A)) targetX -= 60f * dt;
        if (Input.GetKey(KeyCode.RightArrow) || Input.GetKey(KeyCode.D)) targetX += 60f * dt;
        if (Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.W)) targetY = Mathf.Clamp(targetY + 45f * dt, minVerticalAngle, maxVerticalAngle);
        if (Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.S)) targetY = Mathf.Clamp(targetY - 45f * dt, minVerticalAngle, maxVerticalAngle);
#endif
    }

    public void UpdateCameraTransform()
    {
        float dt = Application.isPlaying ? Time.deltaTime : 0.02f;
        // Damping hàm mũ sắc nét: phản hồi cực nhạy theo tay, dừng tức thì không trôi trượt quán tính
        float factor = 1f - Mathf.Exp(-dampingSharpness * dt);

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
    [ContextMenu("Reset Focus Camera (F)")]
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

        // Xử lý toàn bộ tương tác chuột 3D (Xoay Orbit, Trượt Pan, Thu phóng Zoom) trực tiếp qua Event OnGUI
        if (e != null)
        {
            if (e.type == EventType.MouseDown)
            {
                if (!IsMouseOverUI(e.mousePosition) && (e.button == 0 || e.button == 1 || e.button == 2))
                {
                    isDraggingGui = true;
                    e.Use();
                }
            }
            else if (e.type == EventType.MouseUp || e.rawType == EventType.MouseUp)
            {
                isDraggingGui = false;
            }
            else if (e.type == EventType.MouseDrag && isDraggingGui)
            {
                float currentPanSpeed = Mathf.Clamp(panSpeed, 0.0002f, 0.0012f);
                float currentRotateSpeed = Mathf.Clamp(rotateSpeed, 0.04f, 0.20f);

                if (e.button == 0 || e.button == 1) // Chuột trái hoặc phải: Xoay Orbit (hoặc Shift để Pan)
                {
                    if (e.shift)
                    {
                        // Pan trượt
                        float panSign = invertPan ? -1f : 1f;
                        float panScale = targetDistance * currentPanSpeed * panSign;
                        targetPivotOffset += (-transform.right * e.delta.x + transform.up * e.delta.y) * panScale;
                    }
                    else
                    {
                        // Orbit xoay 360 độ: đầm tay, mượt mà, không giật
                        targetX += e.delta.x * currentRotateSpeed;
                        targetY -= e.delta.y * currentRotateSpeed;
                        targetY = Mathf.Clamp(targetY, minVerticalAngle, maxVerticalAngle);
                    }
                    e.Use();
                }
                else if (e.button == 2) // Chuột giữa: Pan trượt
                {
                    float panSign = invertPan ? -1f : 1f;
                    float panScale = targetDistance * currentPanSpeed * panSign;
                    targetPivotOffset += (-transform.right * e.delta.x + transform.up * e.delta.y) * panScale;
                    e.Use();
                }
            }
            else if (e.type == EventType.ScrollWheel && !IsMouseOverUI(e.mousePosition))
            {
                // Thu phóng (Zoom): 1 nấc cuộn thay đổi nhẹ nhàng 5% khoảng cách (tối thiểu 0.04m)
                float currentZoomSpeed = Mathf.Clamp(zoomSpeed, 0.02f, 0.08f);
                float notch = Mathf.Clamp(e.delta.y, -3f, 3f);
                float scrollDir = Mathf.Sign(notch) * (invertScroll ? -1f : 1f);
                float step = Mathf.Max(0.04f, targetDistance * currentZoomSpeed);

                targetDistance += scrollDir * step;
                targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
                e.Use();
            }
        }

        // Vẽ Floating HUD ở góc dưới màn hình 3D
        DrawFloatingViewportToolbar();
    }

    private void DrawFloatingViewportToolbar()
    {
        float hudW = 635f;
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

        // Tùy chọn bật/tắt trượt Pan ngược chuột
        GUIStyle invertPanBtnStyle = invertPan ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button(invertPan ? "Pan: Ngược" : "Pan: Chuẩn", invertPanBtnStyle, GUILayout.Width(76), GUILayout.Height(24)))
        {
            invertPan = !invertPan;
            PlayerPrefs.SetInt("CR10A_Camera_InvertPan", invertPan ? 1 : 0);
            PlayerPrefs.Save();
        }

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
