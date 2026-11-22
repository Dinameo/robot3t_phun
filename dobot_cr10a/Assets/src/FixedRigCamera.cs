using UnityEngine;

/// <summary>
/// Fixed Camera gắn trên giá chữ L ngược ngoài đời thực.
/// Thông số vật lý:
/// - Chiều cao cách mặt đất: 168.5 cm = 1.685 m
/// - Khoảng cách tới trục robot: 33.5 cm = 0.335 m
/// - Hướng nhìn: Vuông góc với mặt đất (Top-down, pitch = 90°)
/// </summary>
[ExecuteAlways]
public class FixedRigCamera : MonoBehaviour
{
    public enum MountDirection
    {
        Forward_Z_Positive,   // Phía trước (+Z)
        Back_Z_Negative,      // Phía sau (-Z)
        Right_X_Positive,     // Bên phải (+X)
        Left_X_Negative,      // Bên trái (-X)
        CustomAngle           // Tùy chỉnh góc xoay quanh trục
    }

    [Header("1. Thông số giá chữ L ngược (Physical Dimensions)")]
    [Tooltip("Độ cao camera cách mặt đất (cm) - Ngoài đời: 168.5 cm")]
    public float heightFromGround_cm = 168.5f;

    [Tooltip("Khoảng cách từ camera tới trục robot (cm) - Ngoài đời: 33.5 cm")]
    public float distanceFromAxis_cm = 33.5f;

    [Header("2. Hướng đặt giá treo chữ L quanh trục Robot")]
    [Tooltip("Chọn hướng đặt giá chữ L quanh robot")]
    public MountDirection mountDirection = MountDirection.Forward_Z_Positive;

    [Range(0f, 360f)]
    [Tooltip("Góc đặt giá chữ L quanh trục Y của robot (độ)")]
    public float customAngleDeg = 0f;

    [Header("3. Hướng nhìn của Camera")]
    [Tooltip("Nhìn vuông góc mặt đất (Pitch = 90 độ)")]
    public bool lookPerpendicularToGround = true;

    [Range(0f, 360f)]
    [Tooltip("Góc xoay hướng ngang của camera (Roll/Yaw trên mặt phẳng)")]
    public float cameraRollDeg = 0f;

    [Range(-45f, 45f)]
    [Tooltip("Góc tinh chỉnh nghiêng Pitch từ Hiệu chuẩn (độ)")]
    public float pitchOffsetDeg = 0f;

    [Range(-45f, 45f)]
    [Tooltip("Góc tinh chỉnh nghiêng Roll từ Hiệu chuẩn (độ)")]
    public float rollOffsetDeg = 0f;

    [Range(20f, 90f)]
    [Tooltip("Góc mở ống kính dọc FOV của Camera Zivid (độ)")]
    public float cameraFovDeg = 45.0f;

    [Header("4. Tham chiếu trục Robot")]
    [Tooltip("Transform gốc của Robot (mặc định lấy Base hoặc tọa độ 0,0,0)")]
    public Transform robotBase;

    [Header("5. Hiển thị mô phỏng giá treo chữ L & FOV (Scene + Game View)")]
    public bool drawLBracketGizmo = true;
    public bool showRuntimeRigVisual = true;
    public bool showRuntimeFovFrustum = true;
    public Color bracketColor = new Color(0.2f, 0.8f, 1f, 0.9f);
    public Color fovColor = new Color(1f, 0.92f, 0.016f, 0.35f);

    private Camera cam;
    private bool visualSuppressedByPov = false;

    // Đối tượng hiển thị 3D thời gian thực trong Game View (không lưu vào Scene)
    private GameObject runtimeVisualRoot;
    private Transform visualMast;
    private Transform visualArm;
    private Transform visualCameraHead;
    private LineRenderer fovLinesRenderer;
    private LineRenderer opticalAxisRenderer;
    private PLYDirectLoader cachedPlyLoader;
    private readonly System.Collections.Generic.List<Material> createdMaterials = new System.Collections.Generic.List<Material>();

    private bool hasLoadedRigPrefs = false;

    public void SaveRigVisualSettings()
    {
        PlayerPrefs.SetFloat("CR10A_Rig_FOV", cameraFovDeg);
        PlayerPrefs.SetInt("CR10A_Rig_ShowVisual", showRuntimeRigVisual ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_Rig_ShowFov", showRuntimeFovFrustum ? 1 : 0);
        PlayerPrefs.Save();
    }

    public void LoadRigVisualSettings()
    {
        if (PlayerPrefs.HasKey("CR10A_Rig_FOV"))
        {
            cameraFovDeg = Mathf.Clamp(PlayerPrefs.GetFloat("CR10A_Rig_FOV", cameraFovDeg), 20f, 90f);
            showRuntimeRigVisual = PlayerPrefs.GetInt("CR10A_Rig_ShowVisual", showRuntimeRigVisual ? 1 : 0) == 1;
            showRuntimeFovFrustum = PlayerPrefs.GetInt("CR10A_Rig_ShowFov", showRuntimeFovFrustum ? 1 : 0) == 1;
        }
        hasLoadedRigPrefs = true;
    }

    private void Awake()
    {
        cam = GetComponent<Camera>();
        LoadRigVisualSettings();
        CalibrationManager calibMgr = FindAnyObjectByType<CalibrationManager>();
        if (calibMgr != null)
        {
            calibMgr.EnsureLoadedFromStorage();
            calibMgr.SyncToSceneCameraAndLoader(reloadPointCloud: false);
        }
        else
        {
            ApplyTransform();
        }
    }

    private void OnEnable()
    {
        cam = GetComponent<Camera>();
        if (!hasLoadedRigPrefs) LoadRigVisualSettings();
        CalibrationManager calibMgr = FindAnyObjectByType<CalibrationManager>();
        if (calibMgr != null)
        {
            calibMgr.EnsureLoadedFromStorage();
            calibMgr.SyncToSceneCameraAndLoader(reloadPointCloud: false);
        }
        else
        {
            ApplyTransform();
        }
        EnsureRuntimeVisual();
        UpdateRuntimeVisual();
    }

    private void Start()
    {
        cam = GetComponent<Camera>();
        LoadRigVisualSettings();
        if (cam != null)
        {
            cam.nearClipPlane = 0.03f;
            cam.farClipPlane = 50.0f;
            cam.fieldOfView = Mathf.Clamp(cameraFovDeg, 20f, 90f);
        }
        CalibrationManager calibMgr = FindAnyObjectByType<CalibrationManager>();
        if (calibMgr != null)
        {
            calibMgr.EnsureLoadedFromStorage();
            calibMgr.SyncToSceneCameraAndLoader(reloadPointCloud: false);
        }
        else
        {
            ApplyTransform();
        }
        EnsureRuntimeVisual();
        UpdateRuntimeVisual();
    }

    private void OnDisable()
    {
        if (Application.isPlaying && hasLoadedRigPrefs)
        {
            SaveRigVisualSettings();
        }
        CleanupRuntimeVisual();
    }

    private void OnDestroy()
    {
        if (Application.isPlaying && hasLoadedRigPrefs)
        {
            SaveRigVisualSettings();
        }
        CleanupRuntimeVisual();
    }

    private void OnApplicationQuit()
    {
        if (hasLoadedRigPrefs)
        {
            SaveRigVisualSettings();
        }
    }

    private void OnValidate()
    {
        ApplyTransform();
    }

    private void Update()
    {
        if (!Application.isPlaying && FindAnyObjectByType<CalibrationManager>() == null)
        {
            ApplyTransform();
        }
        UpdateRuntimeVisual();
    }

    public Camera GetRigCamera()
    {
        if (cam == null) cam = GetComponent<Camera>();
        return cam;
    }

    /// <summary>
    /// Ẩn/hiện khối mô phỏng thân Camera & tia FOV khi người dùng đang nhìn trực tiếp từ chính ống kính Camera Zivid
    /// </summary>
    public void SetVisualSuppressed(bool suppressed)
    {
        visualSuppressedByPov = suppressed;
        UpdateRuntimeVisual();
    }

    /// <summary>
    /// Đồng bộ trực tiếp tọa độ thế giới & góc hiệu chuẩn từ CalibrationManager
    /// </summary>
    public void ApplyCalibratedWorldPose(Vector3 camWorldPos, float pitchDeg, float yawDeg, float rollDeg)
    {
        Vector3 basePos = robotBase != null ? robotBase.position : Vector3.zero;
        Vector3 rel = camWorldPos - basePos;

        heightFromGround_cm = camWorldPos.y * 100.0f;
        distanceFromAxis_cm = Mathf.Sqrt(rel.x * rel.x + rel.z * rel.z) * 100.0f;
        cameraRollDeg = yawDeg;
        pitchOffsetDeg = pitchDeg;
        rollOffsetDeg = rollDeg;

        if (Mathf.Abs(rel.x) > 0.005f)
        {
            mountDirection = MountDirection.CustomAngle;
            customAngleDeg = Mathf.Repeat(Mathf.Atan2(rel.x, rel.z) * Mathf.Rad2Deg, 360f);
        }
        else
        {
            mountDirection = rel.z >= 0f ? MountDirection.Forward_Z_Positive : MountDirection.Back_Z_Negative;
        }

        transform.position = camWorldPos;
        if (lookPerpendicularToGround)
        {
            transform.rotation = Quaternion.Euler(90.0f + pitchOffsetDeg, cameraRollDeg, rollOffsetDeg);
        }

        if (cam == null) cam = GetComponent<Camera>();
        if (cam != null)
        {
            cam.fieldOfView = Mathf.Clamp(cameraFovDeg, 20f, 90f);
        }

        UpdateRuntimeVisual();
    }

    /// <summary>
    /// Cập nhật vị trí và góc quay chuẩn xác cho Camera theo thông số thực tế
    /// </summary>
    [ContextMenu("Áp dụng thông số vật lý (Apply Rig Transform)")]
    public void ApplyTransform()
    {
        // Chuyển đổi cm sang mét trong Unity (1m = 100cm)
        float height_m = heightFromGround_cm / 100.0f;       // 168.5cm -> 1.685m
        float dist_m = distanceFromAxis_cm / 100.0f;          // 33.5cm -> 0.335m

        // Xác định góc đặt giá chữ L quanh trục robot
        float angle = 0f;
        switch (mountDirection)
        {
            case MountDirection.Forward_Z_Positive:
                angle = 0f;
                break;
            case MountDirection.Back_Z_Negative:
                angle = 180f;
                break;
            case MountDirection.Right_X_Positive:
                angle = 90f;
                break;
            case MountDirection.Left_X_Negative:
                angle = 270f;
                break;
            case MountDirection.CustomAngle:
                angle = customAngleDeg;
                break;
        }

        float rad = angle * Mathf.Deg2Rad;
        float offsetX = dist_m * Mathf.Sin(rad);
        float offsetZ = dist_m * Mathf.Cos(rad);

        Vector3 basePos = Vector3.zero;
        if (robotBase != null)
        {
            basePos = robotBase.position;
        }

        // Vị trí camera cố định
        Vector3 camPos = new Vector3(basePos.x + offsetX, height_m, basePos.z + offsetZ);
        transform.position = camPos;

        // Góc nhìn vuông góc mặt đất: Pitch = 90 độ nhìn cắm thẳng xuống đất (+ tinh chỉnh hiệu chuẩn)
        if (lookPerpendicularToGround)
        {
            transform.rotation = Quaternion.Euler(90.0f + pitchOffsetDeg, cameraRollDeg, rollOffsetDeg);
        }

        if (cam == null) cam = GetComponent<Camera>();
        if (cam != null)
        {
            cam.fieldOfView = Mathf.Clamp(cameraFovDeg, 20f, 90f);
        }

        UpdateRuntimeVisual();
    }

    private Material CreateUrpSafeMaterial(Color color, bool unlit)
    {
        Shader shader = null;
        if (!unlit) shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null) shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null) shader = Shader.Find("Sprites/Default");
        if (shader == null) shader = Shader.Find("Unlit/Color");

        Material mat = new Material(shader);
        mat.hideFlags = HideFlags.DontSave;
        if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", color);
        if (mat.HasProperty("_Color")) mat.SetColor("_Color", color);
        createdMaterials.Add(mat);
        return mat;
    }

    private GameObject CreateVisualPrimitive(PrimitiveType type, string objName, Transform parent, Color color, bool unlit = false)
    {
        GameObject go = GameObject.CreatePrimitive(type);
        go.name = objName;
        go.hideFlags = HideFlags.DontSave;
        go.transform.SetParent(parent, false);
        Collider col = go.GetComponent<Collider>();
        if (col != null)
        {
            if (Application.isPlaying) Destroy(col);
            else DestroyImmediate(col);
        }
        MeshRenderer mr = go.GetComponent<MeshRenderer>();
        if (mr != null)
        {
            mr.sharedMaterial = CreateUrpSafeMaterial(color, unlit);
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            mr.receiveShadows = false;
        }
        return go;
    }

    private void EnsureRuntimeVisual()
    {
        if (runtimeVisualRoot != null) return;

        runtimeVisualRoot = new GameObject("ZividCameraRig_RuntimeVisual");
        runtimeVisualRoot.hideFlags = HideFlags.HideAndDontSave;

        // 1. Thân giá chữ L ngược (Cột đứng + Tay đòn ngang)
        Color poleCol = new Color(0.22f, 0.27f, 0.34f, 1f);
        visualMast = CreateVisualPrimitive(PrimitiveType.Cylinder, "BracketMast", runtimeVisualRoot.transform, poleCol).transform;
        visualArm = CreateVisualPrimitive(PrimitiveType.Cylinder, "BracketArm", runtimeVisualRoot.transform, poleCol).transform;

        // 2. Đầu Camera Zivid 3D (đặt lệch về phía sau ống kính local -Z để không chắn góc nhìn của chính nó)
        GameObject headGo = new GameObject("ZividCameraHead");
        headGo.hideFlags = HideFlags.HideAndDontSave;
        headGo.transform.SetParent(runtimeVisualRoot.transform, false);
        visualCameraHead = headGo.transform;

        GameObject body = CreateVisualPrimitive(PrimitiveType.Cube, "ZividBody", visualCameraHead, new Color(0.14f, 0.17f, 0.22f, 1f));
        body.transform.localPosition = new Vector3(0f, 0f, -0.065f);
        body.transform.localScale = new Vector3(0.20f, 0.08f, 0.09f);

        GameObject stripe = CreateVisualPrimitive(PrimitiveType.Cube, "ZividAccent", visualCameraHead, new Color(0.08f, 0.78f, 0.95f, 1f), true);
        stripe.transform.localPosition = new Vector3(0f, 0f, -0.022f);
        stripe.transform.localScale = new Vector3(0.204f, 0.018f, 0.012f);

        GameObject lensLeft = CreateVisualPrimitive(PrimitiveType.Cylinder, "LensRGB", visualCameraHead, new Color(0.05f, 0.07f, 0.10f, 1f));
        lensLeft.transform.localPosition = new Vector3(-0.05f, 0f, -0.018f);
        lensLeft.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
        lensLeft.transform.localScale = new Vector3(0.036f, 0.012f, 0.036f);

        GameObject lensRight = CreateVisualPrimitive(PrimitiveType.Cylinder, "LensProjector", visualCameraHead, new Color(0.08f, 0.55f, 0.95f, 1f), true);
        lensRight.transform.localPosition = new Vector3(0.05f, 0f, -0.018f);
        lensRight.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
        lensRight.transform.localScale = new Vector3(0.040f, 0.012f, 0.040f);

        // 3. Khung viền phẳng vùng thu nhận thực tế của Camera trên mặt bàn (khớp ảnh từ PLY — không vẽ tia chiếu chóp)
        GameObject fovGo = new GameObject("ZividCaptureFootprint");
        fovGo.hideFlags = HideFlags.HideAndDontSave;
        fovGo.transform.SetParent(runtimeVisualRoot.transform, false);
        fovLinesRenderer = fovGo.AddComponent<LineRenderer>();
        fovLinesRenderer.useWorldSpace = true;
        fovLinesRenderer.loop = true;
        fovLinesRenderer.startWidth = 0.007f;
        fovLinesRenderer.endWidth = 0.007f;
        fovLinesRenderer.material = CreateUrpSafeMaterial(new Color(0.15f, 0.90f, 1.0f, 0.92f), true);
        fovLinesRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        fovLinesRenderer.receiveShadows = false;

        // 4. Trục quang học (ẩn mặc định theo yêu cầu chỉ hiện vùng thu nhận thực tế)
        GameObject axisGo = new GameObject("ZividOpticalAxis");
        axisGo.hideFlags = HideFlags.HideAndDontSave;
        axisGo.transform.SetParent(runtimeVisualRoot.transform, false);
        opticalAxisRenderer = axisGo.AddComponent<LineRenderer>();
        opticalAxisRenderer.useWorldSpace = true;
        opticalAxisRenderer.positionCount = 2;
        opticalAxisRenderer.enabled = false;
        opticalAxisRenderer.startWidth = 0.005f;
        opticalAxisRenderer.endWidth = 0.005f;
        opticalAxisRenderer.material = CreateUrpSafeMaterial(new Color(0.15f, 1.0f, 0.45f, 0.85f), true);
        opticalAxisRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        opticalAxisRenderer.receiveShadows = false;
    }

    private bool TryComputeGeometricCameraFootprintWorld(out Vector3 cTL, out Vector3 cTR, out Vector3 cBR, out Vector3 cBL)
    {
        cTL = cTR = cBR = cBL = Vector3.zero;
        Vector3 origin = transform.position;
        float tableY = (cachedPlyLoader != null && cachedPlyLoader.totalLoadedPoints > 0)
            ? cachedPlyLoader.estimatedTableY + 0.004f
            : 0.005f;

        float vFovRad = Mathf.Clamp(cameraFovDeg, 15f, 120f) * Mathf.Deg2Rad;
        float halfH = Mathf.Tan(vFovRad * 0.5f);
        float aspect = (cam != null && cam.aspect > 0.2f) ? cam.aspect : (1944f / 1200f);
        float halfW = halfH * aspect;

        Vector3[] localDirs =
        {
            new Vector3(-halfW,  halfH, 1f).normalized, // TL
            new Vector3( halfW,  halfH, 1f).normalized, // TR
            new Vector3( halfW, -halfH, 1f).normalized, // BR
            new Vector3(-halfW, -halfH, 1f).normalized  // BL
        };

        Vector3[] hits = new Vector3[4];
        for (int i = 0; i < 4; i++)
        {
            Vector3 dirW = transform.TransformDirection(localDirs[i]);
            if (Mathf.Abs(dirW.y) < 1e-4f) return false;
            float t = (tableY - origin.y) / dirW.y;
            if (t <= 0.05f || t > 10f) return false;
            hits[i] = origin + dirW * t;
        }

        cTL = hits[0];
        cTR = hits[1];
        cBR = hits[2];
        cBL = hits[3];
        return true;
    }

    private void UpdateRuntimeVisual()
    {
        bool showRigModel = showRuntimeRigVisual && !visualSuppressedByPov;
        bool showFootprint = showRuntimeFovFrustum && !visualSuppressedByPov;
        bool showAny = showRigModel || showFootprint;
        if (!showAny)
        {
            if (runtimeVisualRoot != null && runtimeVisualRoot.activeSelf)
                runtimeVisualRoot.SetActive(false);
            return;
        }

        EnsureRuntimeVisual();
        if (runtimeVisualRoot == null) return;
        if (!runtimeVisualRoot.activeSelf) runtimeVisualRoot.SetActive(true);

        Vector3 camPos = transform.position;
        Vector3 robotOrigin = (robotBase != null) ? robotBase.position : Vector3.zero;
        // Đặt đỉnh cột đứng hơi cao hơn đầu camera 0.09m để không che ống kính
        Vector3 bracketTop = new Vector3(robotOrigin.x, camPos.y + 0.09f, robotOrigin.z);
        Vector3 armEnd = camPos + Vector3.up * 0.09f;

        if (visualMast != null)
        {
            visualMast.gameObject.SetActive(showRigModel);
            if (showRigModel)
            {
                Vector3 mastDir = bracketTop - robotOrigin;
                float mastLen = Mathf.Max(0.01f, mastDir.magnitude);
                visualMast.position = (robotOrigin + bracketTop) * 0.5f;
                visualMast.up = mastDir.normalized;
                visualMast.localScale = new Vector3(0.032f, mastLen * 0.5f, 0.032f);
            }
        }

        if (visualArm != null)
        {
            Vector3 armDir = armEnd - bracketTop;
            float armLen = armDir.magnitude;
            if (showRigModel && armLen > 0.005f)
            {
                visualArm.gameObject.SetActive(true);
                visualArm.position = (bracketTop + armEnd) * 0.5f;
                visualArm.up = armDir.normalized;
                visualArm.localScale = new Vector3(0.026f, armLen * 0.5f, 0.026f);
            }
            else
            {
                visualArm.gameObject.SetActive(false);
            }
        }

        if (visualCameraHead != null)
        {
            visualCameraHead.gameObject.SetActive(showRigModel);
            if (showRigModel)
            {
                visualCameraHead.position = camPos;
                visualCameraHead.rotation = transform.rotation;
            }
        }

        // Ẩn tia chiếu quang học, chỉ hiện khung vùng camera trên mặt bàn
        if (opticalAxisRenderer != null) opticalAxisRenderer.enabled = false;

        if (cachedPlyLoader == null) cachedPlyLoader = FindAnyObjectByType<PLYDirectLoader>();

        bool hasFootprint = false;
        Vector3 cTL = Vector3.zero, cTR = Vector3.zero, cBR = Vector3.zero, cBL = Vector3.zero;
        if (showFootprint && fovLinesRenderer != null)
        {
            if (cachedPlyLoader != null && cachedPlyLoader.TryGetActualCameraCaptureFootprintWorld(out cTL, out cTR, out cBR, out cBL))
            {
                hasFootprint = true;
            }
            else if (TryComputeGeometricCameraFootprintWorld(out cTL, out cTR, out cBR, out cBL))
            {
                hasFootprint = true;
            }
        }

        if (hasFootprint && fovLinesRenderer != null)
        {
            fovLinesRenderer.enabled = true;
            fovLinesRenderer.loop = true;
            fovLinesRenderer.positionCount = 4;
            fovLinesRenderer.SetPosition(0, cTL);
            fovLinesRenderer.SetPosition(1, cTR);
            fovLinesRenderer.SetPosition(2, cBR);
            fovLinesRenderer.SetPosition(3, cBL);
        }
        else if (fovLinesRenderer != null)
        {
            fovLinesRenderer.enabled = false;
        }
    }

    private void CleanupRuntimeVisual()
    {
        if (runtimeVisualRoot != null)
        {
            if (Application.isPlaying) Destroy(runtimeVisualRoot);
            else DestroyImmediate(runtimeVisualRoot);
            runtimeVisualRoot = null;
        }
        for (int i = 0; i < createdMaterials.Count; i++)
        {
            if (createdMaterials[i] != null)
            {
                if (Application.isPlaying) Destroy(createdMaterials[i]);
                else DestroyImmediate(createdMaterials[i]);
            }
        }
        createdMaterials.Clear();
    }

    private void OnDrawGizmos()
    {
        if (!drawLBracketGizmo) return;

        float height_m = heightFromGround_cm / 100.0f;
        Vector3 camPos = transform.position;
        Vector3 groundUnderCam = new Vector3(camPos.x, 0f, camPos.z);
        Vector3 robotOrigin = (robotBase != null) ? robotBase.position : Vector3.zero;

        // Vẽ giá chữ L ngược:
        // Cột đứng từ gốc robot lên cao 168.5cm
        Gizmos.color = bracketColor;
        Vector3 bracketTop = new Vector3(robotOrigin.x, height_m, robotOrigin.z);
        Gizmos.DrawLine(robotOrigin, bracketTop); // Thanh đứng chữ L
        Gizmos.DrawLine(bracketTop, camPos);      // Thanh ngang chữ L vươn ra 33.5cm tới camera

        // Đánh dấu vị trí Camera
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(camPos, 0.04f);

        // Chỉ vẽ khung vùng thu nhận thực tế trên mặt bàn khớp với ảnh từ PLY (ẩn tia chiếu cam)
        if (cachedPlyLoader == null) cachedPlyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (showRuntimeFovFrustum && cachedPlyLoader != null &&
            cachedPlyLoader.TryGetActualCameraCaptureFootprintWorld(out Vector3 gTL, out Vector3 gTR, out Vector3 gBR, out Vector3 gBL))
        {
            Gizmos.color = new Color(0.15f, 0.90f, 1.0f, 0.92f);
            Gizmos.DrawLine(gTL, gTR);
            Gizmos.DrawLine(gTR, gBR);
            Gizmos.DrawLine(gBR, gBL);
            Gizmos.DrawLine(gBL, gTL);
        }
    }
}
