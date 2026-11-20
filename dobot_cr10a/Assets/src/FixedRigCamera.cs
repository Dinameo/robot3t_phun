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

    [Header("4. Tham chiếu trục Robot")]
    [Tooltip("Transform gốc của Robot (mặc định lấy Base hoặc tọa độ 0,0,0)")]
    public Transform robotBase;

    [Header("5. Hiển thị mô phỏng giá treo chữ L (Gizmos)")]
    public bool drawLBracketGizmo = true;
    public Color bracketColor = new Color(0.2f, 0.8f, 1f, 0.9f);
    public Color fovColor = new Color(1f, 0.92f, 0.016f, 0.35f);

    private Camera cam;

    private void Awake()
    {
        cam = GetComponent<Camera>();
        ApplyTransform();
    }

    private void Start()
    {
        cam = GetComponent<Camera>();
        if (cam != null)
        {
            cam.nearClipPlane = 0.03f;
            cam.farClipPlane = 50.0f;
            cam.fieldOfView = 45.0f; // FOV tương đương Zivid camera
        }
        ApplyTransform();
    }

    private void OnValidate()
    {
        ApplyTransform();
    }

    private void Update()
    {
        if (!Application.isPlaying)
        {
            ApplyTransform();
        }
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

        // Góc nhìn vuông góc mặt đất: Pitch = 90 độ nhìn cắm thẳng xuống đất
        if (lookPerpendicularToGround)
        {
            transform.rotation = Quaternion.Euler(90.0f, cameraRollDeg, 0.0f);
        }
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

        // Vẽ tia camera nhìn vuông góc xuống mặt đất
        Gizmos.color = Color.green;
        Gizmos.DrawLine(camPos, groundUnderCam);
        Gizmos.DrawWireSphere(groundUnderCam, 0.05f);

        // Vẽ nón vùng nhìn FOV của camera xuống sàn
        if (cam == null) cam = GetComponent<Camera>();
        if (cam != null && lookPerpendicularToGround)
        {
            Gizmos.color = fovColor;
            float fovRad = cam.fieldOfView * Mathf.Deg2Rad;
            float frustumHeight = 2.0f * height_m * Mathf.Tan(fovRad * 0.5f);
            float frustumWidth = frustumHeight * cam.aspect;
            Gizmos.DrawWireCube(groundUnderCam, new Vector3(frustumWidth, 0.01f, frustumHeight));
        }
    }
}
