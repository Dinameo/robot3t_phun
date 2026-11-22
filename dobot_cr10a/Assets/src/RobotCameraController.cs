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

    [Header("Góc nhìn Camera Zivid (Fixed Rig POV & PiP)")]
    [Tooltip("Tham chiếu Camera Zivid gắn cố định trên giá chữ L")]
    public FixedRigCamera fixedRigCamera;
    [Tooltip("Đang xem trực tiếp từ góc nhìn Camera Zivid")]
    public bool isZividCameraView = false;
    [Tooltip("Hiện khung xem nhỏ trực tiếp (Picture-in-Picture) của Camera Zivid")]
    public bool showZividPipWindow = false;
    [Tooltip("Góc mở FOV mặc định khi xoay 3D tự do")]
    public float defaultOrbitFov = 55.0f;

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
    private RenderTexture zividPipRT;
    private const int ZividPipTexWidth = 384;
    private const int ZividPipTexHeight = 256;

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
        showZividPipWindow = PlayerPrefs.GetInt("CR10A_Camera_ShowZividPip", showZividPipWindow ? 1 : 0) == 1;
        followTool = PlayerPrefs.GetInt("CR10A_Camera_FollowTool", followTool ? 1 : 0) == 1;

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
        UpdateCameraViewportRect();
        UpdateCameraTransform();
        UpdateFixedRigCameraRenderingState();
    }

    private void OnDisable()
    {
        if (Application.isPlaying && isInitialized)
        {
            PlayerPrefs.SetInt("CR10A_Camera_InvertScroll", invertScroll ? 1 : 0);
            PlayerPrefs.SetInt("CR10A_Camera_InvertPan", invertPan ? 1 : 0);
            PlayerPrefs.SetInt("CR10A_Camera_ShowZividPip", showZividPipWindow ? 1 : 0);
            PlayerPrefs.SetInt("CR10A_Camera_FollowTool", followTool ? 1 : 0);
            PlayerPrefs.Save();
        }
        if (cam != null)
        {
            cam.rect = new Rect(0f, 0f, 1f, 1f);
            cam.fieldOfView = defaultOrbitFov;
        }
        ReleaseZividPipResources();
    }

    private void OnDestroy()
    {
        ReleaseZividPipResources();
    }

    public FixedRigCamera EnsureFixedRigCamera(bool createIfMissing = true)
    {
        if (fixedRigCamera == null)
        {
            fixedRigCamera = FindAnyObjectByType<FixedRigCamera>();
        }
        if (fixedRigCamera == null && createIfMissing)
        {
            GameObject rigGo = new GameObject("Zivid_FixedRigCamera");
            Camera rigUnityCam = rigGo.AddComponent<Camera>();
            rigUnityCam.enabled = false;
            rigUnityCam.nearClipPlane = 0.03f;
            rigUnityCam.farClipPlane = 50.0f;
            rigUnityCam.fieldOfView = 45.0f;
            fixedRigCamera = rigGo.AddComponent<FixedRigCamera>();
            fixedRigCamera.ApplyTransform();

            CalibrationManager calib = FindAnyObjectByType<CalibrationManager>();
            if (calib != null)
            {
                calib.SyncToSceneCameraAndLoader(reloadPointCloud: false);
            }
        }
        return fixedRigCamera;
    }

    private void UpdateFixedRigCameraRenderingState()
    {
        FixedRigCamera rig = fixedRigCamera != null ? fixedRigCamera : FindAnyObjectByType<FixedRigCamera>();
        if (rig == null) return;
        fixedRigCamera = rig;

        // Khi đang nhìn trực tiếp từ chính ống kính Camera Zivid thì ẩn khối vỏ 3D để tránh che hình
        rig.SetVisualSuppressed(isZividCameraView);

        Camera rigUnityCam = rig.GetRigCamera();
        if (rigUnityCam == null || rigUnityCam == cam) return;

        bool needPip = showZividPipWindow && !isZividCameraView;
        if (needPip)
        {
            if (zividPipRT == null)
            {
                zividPipRT = new RenderTexture(ZividPipTexWidth, ZividPipTexHeight, 16, RenderTextureFormat.ARGB32)
                {
                    name = "ZividCameraPiP_RT",
                    hideFlags = HideFlags.DontSave
                };
                zividPipRT.Create();
            }
            if (rigUnityCam.targetTexture != zividPipRT)
                rigUnityCam.targetTexture = zividPipRT;
            rigUnityCam.rect = new Rect(0f, 0f, 1f, 1f);
            rigUnityCam.depth = -2f;
            rigUnityCam.fieldOfView = Mathf.Clamp(rig.cameraFovDeg, 20f, 90f);
            if (!rigUnityCam.enabled) rigUnityCam.enabled = true;
        }
        else
        {
            if (rigUnityCam.targetTexture != null)
                rigUnityCam.targetTexture = null;
            if (rigUnityCam.enabled)
                rigUnityCam.enabled = false;
        }
    }

    private void ReleaseZividPipResources()
    {
        if (fixedRigCamera != null)
        {
            fixedRigCamera.SetVisualSuppressed(false);
            Camera rigUnityCam = fixedRigCamera.GetRigCamera();
            if (rigUnityCam != null && rigUnityCam != cam)
            {
                if (rigUnityCam.targetTexture == zividPipRT)
                    rigUnityCam.targetTexture = null;
                rigUnityCam.enabled = false;
            }
        }
        if (zividPipRT != null)
        {
            zividPipRT.Release();
            if (Application.isPlaying) Destroy(zividPipRT);
            else DestroyImmediate(zividPipRT);
            zividPipRT = null;
        }
    }

    /// <summary>
    /// Đẩy toàn bộ khung nhìn 3D xem phôi sang bên trái khớp chính xác với chiều rộng bảng Tab gắn phải.
    /// Khi đóng bảng Tab, khung nhìn 3D tự động mở rộng lại 100% chiều ngang.
    /// </summary>
    public void UpdateCameraViewportRect()
    {
        if (cam == null) cam = GetComponent<Camera>() ?? Camera.main;
        if (cam == null) return;

        float sw = Mathf.Max(1f, (float)Screen.width);
        float sh = Mathf.Max(1f, (float)Screen.height);

        float dockW = DobotCR10AIKController.WorkbenchDockVisible ? DobotCR10AIKController.WorkbenchDockWidth : 0f;
        float topH = DobotCR10AIKController.WorkbenchHeaderHeight;

        float wNorm = Mathf.Clamp((sw - dockW) / sw, 0.25f, 1f);
        float hNorm = Mathf.Clamp((sh - topH) / sh, 0.25f, 1f);

        Rect targetRect = new Rect(0f, 0f, wNorm, hNorm);
        if (Mathf.Abs(cam.rect.width - targetRect.width) > 1e-4f ||
            Mathf.Abs(cam.rect.height - targetRect.height) > 1e-4f ||
            Mathf.Abs(cam.rect.x - targetRect.x) > 1e-4f ||
            Mathf.Abs(cam.rect.y - targetRect.y) > 1e-4f)
        {
            cam.rect = targetRect;
        }
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
        // Ưu tiên kiểm tra chính xác mọi cửa sổ nổi kéo thả & thanh công cụ đã đăng ký
        if (DraggableUIManager.IsPointOverAnyUI(mouseScreenPos))
            return true;

        // Fallback cho trường hợp standalone manual.cs
        float totalRibbonH = 94f;
        var ikCtrl = FindAnyObjectByType<DobotCR10AIKController>(FindObjectsInactive.Exclude);
        if (ikCtrl != null && ikCtrl.enabled)
        {
            totalRibbonH = DobotCR10AIKController.WorkbenchHeaderHeight;
        }
        else
        {
            var man = FindAnyObjectByType<manual>(FindObjectsInactive.Exclude);
            if (man != null && man.enabled)
            {
                totalRibbonH = man.ribbonExpanded ? 114f : 38f;
                bool anyDockOpen = man.showJointsPanel || man.showFkMonitorPanel || man.showIkTargetPanel;
                if (anyDockOpen && mouseScreenPos.x >= (Screen.width - 476f)) return true;
            }
        }

        // 1. Khu vực Dải Ruy Băng trên cùng
        if (mouseScreenPos.y < totalRibbonH) return true;

        // 2. Thanh công cụ HUD nổi ở góc dưới bên trái
        if (mouseScreenPos.x >= 10f && mouseScreenPos.x <= 750f && mouseScreenPos.y >= (Screen.height - 44f)) return true;

        return false;
    }

    public void ZoomIn(float step = 0.12f)
    {
        if (isZividCameraView) UnlockFromZividToOrbit();
        targetDistance = Mathf.Clamp(targetDistance - step * targetDistance, minDistance, maxDistance);
        UpdateCameraTransform();
    }

    public void ZoomOut(float step = 0.12f)
    {
        if (isZividCameraView) UnlockFromZividToOrbit();
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
            if (kb.digit5Key.wasPressedThisFrame || kb.numpad5Key.wasPressedThisFrame || kb.cKey.wasPressedThisFrame) ToggleZividCameraView();
            if (kb.pKey.wasPressedThisFrame) ToggleZividPip();
            if (kb.fKey.wasPressedThisFrame) ResetFocus();
            if (kb.spaceKey.wasPressedThisFrame)
            {
                if (isZividCameraView) ExitZividCameraView();
                followTool = !followTool;
            }

            float dt = Application.isPlaying ? Time.deltaTime : 0.02f;
            if (kb.equalsKey.isPressed || kb.numpadPlusKey.isPressed) ZoomIn(0.02f);
            if (kb.minusKey.isPressed || kb.numpadMinusKey.isPressed) ZoomOut(0.02f);
            bool anyMoveKey = kb.leftArrowKey.isPressed || kb.aKey.isPressed ||
                              kb.rightArrowKey.isPressed || kb.dKey.isPressed ||
                              kb.upArrowKey.isPressed || kb.wKey.isPressed ||
                              kb.downArrowKey.isPressed || kb.sKey.isPressed;
            if (anyMoveKey && isZividCameraView) UnlockFromZividToOrbit();
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
        if (Input.GetKeyDown(KeyCode.Alpha5) || Input.GetKeyDown(KeyCode.C)) ToggleZividCameraView();
        if (Input.GetKeyDown(KeyCode.P)) ToggleZividPip();
        if (Input.GetKeyDown(KeyCode.F)) ResetFocus();
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (isZividCameraView) ExitZividCameraView();
            followTool = !followTool;
        }

        float dt = Application.isPlaying ? Time.deltaTime : 0.02f;
        if (Input.GetKey(KeyCode.Equals) || Input.GetKey(KeyCode.KeypadPlus)) ZoomIn(0.02f);
        if (Input.GetKey(KeyCode.Minus) || Input.GetKey(KeyCode.KeypadMinus)) ZoomOut(0.02f);
        bool anyMoveKey = Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.A) ||
                          Input.GetKey(KeyCode.RightArrow) || Input.GetKey(KeyCode.D) ||
                          Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.W) ||
                          Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.S);
        if (anyMoveKey && isZividCameraView) UnlockFromZividToOrbit();
        if (Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.A)) targetX -= 60f * dt;
        if (Input.GetKey(KeyCode.RightArrow) || Input.GetKey(KeyCode.D)) targetX += 60f * dt;
        if (Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.W)) targetY = Mathf.Clamp(targetY + 45f * dt, minVerticalAngle, maxVerticalAngle);
        if (Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.S)) targetY = Mathf.Clamp(targetY - 45f * dt, minVerticalAngle, maxVerticalAngle);
#endif
    }

    public void UpdateCameraTransform()
    {
        if (isZividCameraView)
        {
            FixedRigCamera rig = EnsureFixedRigCamera();
            if (rig != null)
            {
                transform.position = rig.transform.position;
                transform.rotation = rig.transform.rotation;
                if (cam == null) cam = GetComponent<Camera>() ?? Camera.main;
                if (cam != null)
                {
                    cam.fieldOfView = Mathf.Clamp(rig.cameraFovDeg, 20f, 90f);
                }
                return;
            }
        }

        if (cam == null) cam = GetComponent<Camera>() ?? Camera.main;
        if (cam != null && Mathf.Abs(cam.fieldOfView - defaultOrbitFov) > 0.05f)
        {
            cam.fieldOfView = defaultOrbitFov;
        }

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
    /// Chuyển sang góc nhìn trực tiếp từ Camera Zivid thực tế (FixedRigCamera: đúng vị trí, góc nghiêng & FOV)
    /// </summary>
    public void SetPresetZividCamera(bool instant = true)
    {
        InitializeIfNeeded();
        FixedRigCamera rig = EnsureFixedRigCamera(createIfMissing: true);
        CalibrationManager calib = FindAnyObjectByType<CalibrationManager>();
        if (calib != null)
        {
            calib.SyncToSceneCameraAndLoader(reloadPointCloud: false);
        }
        else if (rig != null)
        {
            rig.ApplyTransform();
        }

        followTool = false;
        isZividCameraView = true;
        UpdateCameraTransform();
        UpdateFixedRigCameraRenderingState();
    }

    public void ExitZividCameraView()
    {
        if (!isZividCameraView) return;
        isZividCameraView = false;
        if (cam == null) cam = GetComponent<Camera>() ?? Camera.main;
        if (cam != null) cam.fieldOfView = defaultOrbitFov;
        if (fixedRigCamera != null) fixedRigCamera.SetVisualSuppressed(false);
        UpdateFixedRigCameraRenderingState();
    }

    public void ToggleZividCameraView()
    {
        if (isZividCameraView) SetPresetPerspective();
        else SetPresetZividCamera();
    }

    public void ToggleZividPip()
    {
        InitializeIfNeeded();
        if (isZividCameraView) ExitZividCameraView();
        showZividPipWindow = !showZividPipWindow;
        PlayerPrefs.SetInt("CR10A_Camera_ShowZividPip", showZividPipWindow ? 1 : 0);
        PlayerPrefs.Save();
        if (showZividPipWindow) EnsureFixedRigCamera(createIfMissing: true);
        UpdateFixedRigCameraRenderingState();
    }

    private void UnlockFromZividToOrbit()
    {
        if (!isZividCameraView) return;
        Vector3 wpCenter = GetWorkpieceCenter();
        targetPivotOffset = (target != null) ? wpCenter - target.position : wpCenter;
        currentPivotPosition = GetPivotPosition();
        Vector3 offset = transform.position - currentPivotPosition;
        targetDistance = Mathf.Clamp(offset.magnitude, minDistance, maxDistance);
        distance = targetDistance;
        Vector3 eul = transform.rotation.eulerAngles;
        float pitch = eul.x > 180f ? eul.x - 360f : eul.x;
        targetY = currentY = Mathf.Clamp(pitch, minVerticalAngle, maxVerticalAngle);
        targetX = currentX = eul.y;
        ExitZividCameraView();
    }

    /// <summary>
    /// Góc nhìn toàn cảnh 3/4 (Perspective Overview - Bao quát cả robot và cái đồ)
    /// </summary>
    public void SetPresetPerspective(bool instant = true)
    {
        InitializeIfNeeded();
        ExitZividCameraView();
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
        ExitZividCameraView();
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
        ExitZividCameraView();
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
        ExitZividCameraView();
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
        ExitZividCameraView();
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
        ExitZividCameraView();
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

        // Vẽ khung ngắm Camera Zivid (khi đang ở Góc nhìn Camera) hoặc khung PiP trực tiếp ở góc viewport
        if (isZividCameraView)
        {
            DrawZividViewfinderOverlay();
        }
        else if (showZividPipWindow)
        {
            DrawZividPipOverlay();
        }

        // Vẽ Floating HUD ở góc dưới màn hình 3D (trước khi xử lý drag để đăng ký vùng chặn chuột)
        DrawFloatingViewportToolbar();

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
                if (isZividCameraView) UnlockFromZividToOrbit();

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
                if (isZividCameraView) UnlockFromZividToOrbit();

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
    }

    private void DrawZividViewfinderOverlay()
    {
        float dockW = DobotCR10AIKController.WorkbenchDockVisible ? DobotCR10AIKController.WorkbenchDockWidth : 0f;
        float topH = DobotCR10AIKController.WorkbenchHeaderHeight;
        float viewW = Mathf.Max(220f, Screen.width - dockW);
        float viewH = Mathf.Max(180f, Screen.height - topH);

        // Vẽ dấu chữ thập tâm quang học + 4 góc khung ngắm Camera Zivid
        Color prevColor = GUI.color;
        GUI.color = new Color(0.15f, 0.90f, 1.0f, 0.55f);
        float cx = viewW * 0.5f;
        float cy = topH + viewH * 0.5f;
        GUI.DrawTexture(new Rect(cx - 14f, cy - 1f, 28f, 2f), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(cx - 1f, cy - 14f, 2f, 28f), Texture2D.whiteTexture);

        float m = 18f;
        float bLen = 22f;
        float bThick = 2f;
        float yTop = topH + m;
        float yBot = Screen.height - 48f - m;
        float xLeft = m;
        float xRight = viewW - m;
        if (yBot > yTop + 60f && xRight > xLeft + 80f)
        {
            GUI.DrawTexture(new Rect(xLeft, yTop, bLen, bThick), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(xLeft, yTop, bThick, bLen), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(xRight - bLen, yTop, bLen, bThick), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(xRight - bThick, yTop, bThick, bLen), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(xLeft, yBot - bThick, bLen, bThick), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(xLeft, yBot - bLen, bThick, bLen), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(xRight - bLen, yBot - bThick, bLen, bThick), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(xRight - bThick, yBot - bLen, bThick, bLen), Texture2D.whiteTexture);
        }
        GUI.color = prevColor;

        // Thanh nhãn trạng thái Góc nhìn Camera Zivid ở góc trên bên trái vùng 3D (tự co giãn theo chiều rộng, không tràn chữ)
        FixedRigCamera rig = EnsureFixedRigCamera(createIfMissing: false);
        float hCm = rig != null ? rig.heightFromGround_cm : 168.5f;
        float dCm = rig != null ? rig.distanceFromAxis_cm : 33.5f;
        float fov = rig != null ? rig.cameraFovDeg : 45.0f;

        string fullLabel = $"CAMERA ZIVID  |  Cao {hCm:F1}cm  |  Trục {dCm:F1}cm  |  FOV {fov:F0}°";
        string shortLabel = $"CAMERA ZIVID | H:{hCm:F0}cm | D:{dCm:F0}cm | FOV {fov:F0}°";
        float fullTextW = AntigravityUITheme.StyleLabelCyan.CalcSize(new GUIContent(fullLabel)).x;
        float maxBadgeW = Mathf.Max(220f, viewW - 24f);
        bool useShort = (fullTextW + 86f > maxBadgeW);
        string activeLabel = useShort ? shortLabel : fullLabel;
        float activeTextW = AntigravityUITheme.StyleLabelCyan.CalcSize(new GUIContent(activeLabel)).x;

        float badgeW = Mathf.Min(maxBadgeW, Mathf.Max(250f, activeTextW + 88f));
        float badgeH = 30f;
        Rect badgeRect = new Rect(12f, topH + 10f, badgeW, badgeH);
        DraggableUIManager.RegisterBlockingRect(badgeRect);

        GUI.DrawTexture(badgeRect, AntigravityUITheme.TexBgHeader);
        GUI.DrawTexture(new Rect(badgeRect.x, badgeRect.y, badgeRect.width, 1f), AntigravityUITheme.TexSliderThumb);
        GUI.DrawTexture(new Rect(badgeRect.x, badgeRect.yMax - 1f, badgeRect.width, 1f), AntigravityUITheme.TexBorder);

        GUIStyle singleLineCyan = new GUIStyle(AntigravityUITheme.StyleLabelCyan)
        {
            wordWrap = false,
            clipping = TextClipping.Clip,
            alignment = TextAnchor.MiddleLeft
        };
        float btnW = 58f;
        GUI.Label(new Rect(badgeRect.x + 10f, badgeRect.y + 2f, Mathf.Max(60f, badgeRect.width - btnW - 22f), badgeRect.height - 4f), activeLabel, singleLineCyan);
        if (GUI.Button(new Rect(badgeRect.xMax - btnW - 6f, badgeRect.y + 4f, btnW, 22f), "Về 3D", AntigravityUITheme.StyleButton))
        {
            SetPresetPerspective();
        }
    }

    private void DrawZividPipOverlay()
    {
        if (zividPipRT == null) return;

        float dockW = DobotCR10AIKController.WorkbenchDockVisible ? DobotCR10AIKController.WorkbenchDockWidth : 0f;
        float topH = DobotCR10AIKController.WorkbenchHeaderHeight;
        float viewW = Mathf.Max(220f, Screen.width - dockW);

        float boxW = Mathf.Min(292f, Mathf.Max(200f, viewW * 0.34f));
        float imgH = boxW * (2f / 3f);
        float headerH = 24f;
        float footerH = 18f;
        float boxH = headerH + imgH + footerH + 6f;

        float boxX = Mathf.Max(10f, viewW - boxW - 12f);
        float boxY = topH + 10f;

        Rect boxRect = new Rect(boxX, boxY, boxW, boxH);
        DraggableUIManager.RegisterBlockingRect(boxRect);

        GUI.DrawTexture(boxRect, AntigravityUITheme.TexBgHeader);
        GUI.DrawTexture(new Rect(boxX, boxY, boxW, 1f), AntigravityUITheme.TexSliderThumb);
        GUI.DrawTexture(new Rect(boxX, boxY + boxH - 1f, boxW, 1f), AntigravityUITheme.TexBorder);
        GUI.DrawTexture(new Rect(boxX, boxY, 1f, boxH), AntigravityUITheme.TexBorder);
        GUI.DrawTexture(new Rect(boxX + boxW - 1f, boxY, 1f, boxH), AntigravityUITheme.TexBorder);

        // Thanh tiêu đề khung PiP
        GUILayout.BeginArea(new Rect(boxX + 6f, boxY + 2f, boxW - 12f, headerH));
        GUILayout.BeginHorizontal();
        GUILayout.Label("CAMERA ZIVID (TRỰC TIẾP)", AntigravityUITheme.StyleLabelCyan);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Toàn màn", AntigravityUITheme.StyleButton, GUILayout.Width(68), GUILayout.Height(19)))
        {
            SetPresetZividCamera();
        }
        if (GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(20), GUILayout.Height(19)))
        {
            showZividPipWindow = false;
            UpdateFixedRigCameraRenderingState();
        }
        GUILayout.EndHorizontal();
        GUILayout.EndArea();

        // Ảnh trực tiếp từ Camera Zivid
        Rect imgRect = new Rect(boxX + 4f, boxY + headerH + 2f, boxW - 8f, imgH);
        GUI.DrawTexture(imgRect, zividPipRT, ScaleMode.StretchToFill, false);

        // Chữ thập tâm nhỏ trên khung PiP
        Color prev = GUI.color;
        GUI.color = new Color(0.15f, 0.92f, 1f, 0.65f);
        float cx = imgRect.center.x;
        float cy = imgRect.center.y;
        GUI.DrawTexture(new Rect(cx - 8f, cy - 0.5f, 16f, 1f), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(cx - 0.5f, cy - 8f, 1f, 16f), Texture2D.whiteTexture);
        GUI.color = prev;

        // Click vào ảnh PiP để phóng to sang góc nhìn Camera Zivid toàn màn hình
        if (Event.current != null && Event.current.type == EventType.MouseDown && Event.current.button == 0 && imgRect.Contains(Event.current.mousePosition))
        {
            SetPresetZividCamera();
            Event.current.Use();
        }

        // Dòng thông số phía dưới (gọn, không tràn khung PiP)
        FixedRigCamera rig = fixedRigCamera;
        float hCm = rig != null ? rig.heightFromGround_cm : 168.5f;
        float dCm = rig != null ? rig.distanceFromAxis_cm : 33.5f;
        float fov = rig != null ? rig.cameraFovDeg : 45.0f;
        GUIStyle pipFooterStyle = new GUIStyle(AntigravityUITheme.StyleLabelMuted)
        {
            wordWrap = false,
            clipping = TextClipping.Clip
        };
        GUI.Label(new Rect(boxX + 6f, imgRect.yMax + 2f, boxW - 12f, footerH),
            $"Cao {hCm:F0}cm | Trục {dCm:F0}cm | FOV {fov:F0}°",
            pipFooterStyle);
    }

    private void DrawFloatingViewportToolbar()
    {
        float dockW = DobotCR10AIKController.WorkbenchDockVisible ? DobotCR10AIKController.WorkbenchDockWidth : 0f;
        float availW = Mathf.Max(320f, Screen.width - dockW - 20f);
        float hudW = Mathf.Min(730f, availW);
        float hudH = 32f;
        float hudX = 10f;
        float hudY = Screen.height - hudH - 8f;

        Rect hudRect = new Rect(hudX, hudY, hudW, hudH);
        DraggableUIManager.RegisterBlockingRect(hudRect);

        // Nền tối gọn nằm gọn bên trong vùng xem 3D bên trái
        GUI.DrawTexture(hudRect, AntigravityUITheme.TexBgHeader);
        GUI.DrawTexture(new Rect(hudX, hudY, hudW, 1), AntigravityUITheme.TexBorder);
        GUI.DrawTexture(new Rect(hudX, hudY + hudH - 1, hudW, 1), AntigravityUITheme.TexBorder);

        GUILayout.BeginArea(new Rect(hudX + 6f, hudY + 4f, hudW - 12f, hudH - 8f));
        GUILayout.BeginHorizontal();

        if (GUILayout.Button("+", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(24))) ZoomIn();
        if (GUILayout.Button("-", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(24))) ZoomOut();
        if (GUILayout.Button("3/4", AntigravityUITheme.StyleButton, GUILayout.Height(24))) SetPresetPerspective();
        if (GUILayout.Button("Trên", AntigravityUITheme.StyleButton, GUILayout.Height(24))) SetPresetTopDown();
        if (GUILayout.Button("Trước", AntigravityUITheme.StyleButton, GUILayout.Height(24))) SetPresetFront();

        GUIStyle zividCamBtnStyle = isZividCameraView ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("Góc Cam", zividCamBtnStyle, GUILayout.Height(24))) ToggleZividCameraView();

        GUIStyle pipBtnStyle = (showZividPipWindow && !isZividCameraView) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("PiP Cam", pipBtnStyle, GUILayout.Height(24))) ToggleZividPip();

        if (GUILayout.Button("Reset", AntigravityUITheme.StyleButton, GUILayout.Height(24))) ResetFocus();

        GUIStyle invertPanBtnStyle = invertPan ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button(invertPan ? "Pan: Ngược" : "Pan: Chuẩn", invertPanBtnStyle, GUILayout.Height(24)))
        {
            invertPan = !invertPan;
            PlayerPrefs.SetInt("CR10A_Camera_InvertPan", invertPan ? 1 : 0);
            PlayerPrefs.Save();
        }

        GUIStyle invertBtnStyle = invertScroll ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button(invertScroll ? "Zoom: Ngược" : "Zoom: Chuẩn", invertBtnStyle, GUILayout.Height(24)))
        {
            invertScroll = !invertScroll;
            PlayerPrefs.SetInt("CR10A_Camera_InvertScroll", invertScroll ? 1 : 0);
            PlayerPrefs.Save();
        }

        GUILayout.EndHorizontal();
        GUILayout.EndArea();
    }
}
