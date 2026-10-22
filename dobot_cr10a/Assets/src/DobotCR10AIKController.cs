using System;
using System.IO;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// Bộ điều khiển Động học Nghịch (Inverse Kinematics) & Giao diện kiểm thử cho Dobot CR10A.
/// Yêu cầu đáp ứng:
/// 1. Tọa độ TCP (GameObject tcp, gồm d6 và offset local 0.1 m): X, Y, Z đi theo waypoint.
/// 2. Hướng TCP (Roll, Pitch, Yaw) theo mặt phẳng vi mô cục bộ.
/// 3. Giao diện trực quan kiểm thử: Play/Pause, Step Slider, hiển thị 6 góc khớp, sai số và đường vẽ.
/// </summary>
[ExecuteAlways]
public class DobotCR10AIKController : MonoBehaviour
{
    [System.Serializable]
    public class WaypointData
    {
        public int index;
        public float distance_mm;
        [FormerlySerializedAs("targetPos_mm")]
        public Vector3 targetPosGlobal_mm; // Tọa độ TCP trong Global của robot (mm), nguồn dữ liệu chính
        [FormerlySerializedAs("targetPos_m")]
        public Vector3 targetPosWorld_m;   // Chỉ là bản đổi sang Unity world để vẽ gizmo (m)
        public Vector3 microNormal;       // Pháp tuyến vi mô trong hệ Robot
        public float roll_deg;
        public float pitch_deg;
        public float yaw_deg;
        public float[] jointAngles_deg = new float[6];
        public float[] jointAngles_rad = new float[6];
        public float posError_mm;
        public bool isSolved;
    }

    [Header("1. Nguồn dữ liệu quỹ đạo")]
    [Tooltip("Đường dẫn file CSV quỹ đạo góc khớp hoặc tọa độ")]
    public string trajectoryCsvPath = "output_drawing/cr10a_joint_angles.csv";

    [Header("2. Kích thước hình học Dobot CR10A (mm)")]
    public float d1 = 176.5f;   // Đế -> Khớp 2
    public float a2 = 607.0f;   // Khớp 2 -> Khớp 3
    public float a3 = 568.0f;   // Khớp 3 -> Khớp 4
    public float d4 = 125.0f;   // Offset Khớp 4 -> Khớp 5
    public float d5 = 125.0f;   // Offset Khớp 5 -> Khớp 6
    public float d6 = 111.3f;   // Khớp 6 -> axis6/flange
    public float toolLength = 0.0f; // Không cộng offset ngoài; Transform tcp đã chứa offset 0.1 m

    [Header("3. Tham chiếu mô hình Robot")]
    [Tooltip("Script manual.cs điều khiển 6 khớp robot")]
    public manual robotManual;

    [Tooltip("Frame Global gắn với gốc robot; waypoint lưu theo frame này, Unity world chỉ dùng để hiển thị")]
    public Transform globalReference;

    [Tooltip("TCP thực: GameObject tcp (con của axis6), dùng trực tiếp để vẽ và hiển thị")]
    public Transform tcpTransform;

    [Header("4. Trạng thái điều khiển & Kiểm thử")]
    public bool isPlaying = false;
    [Range(0, 100)]
    public int currentWaypointIndex = 0;
    [Range(0.1f, 1.0f)]
    public float playbackSpeed = 0.5f; // Hệ số nhân tốc độ khớp
    [Range(1f, 10f)]
    public float maxJointSpeedDegPerSecond = 10f;

    [Header("5. Hiển thị đường vẽ mực (Trail)")]
    public bool drawInkTrail = true;
    public Color inkColor = Color.red;

    // Danh sách waypoint đã nạp
    public List<WaypointData> waypoints = new List<WaypointData>();

    private LineRenderer lineRenderer;
    private readonly List<Vector3> actualTcpTrail = new List<Vector3>();

    private void Awake()
    {
        EnsureComponents();
    }

    private void Start()
    {
        EnsureComponents();
        LoadTrajectoryFile();
    }

    private void EnsureComponents()
    {
        if (robotManual == null)
        {
            robotManual = GetComponent<manual>() ?? FindAnyObjectByType<manual>();
        }

        if (globalReference == null && robotManual != null && robotManual.axisTransforms != null &&
            robotManual.axisTransforms.Length > 0 && robotManual.axisTransforms[0] != null)
            globalReference = robotManual.axisTransforms[0].parent;

        if (globalReference == null)
        {
            GameObject globalObject = GameObject.Find("Global");
            if (globalObject != null)
                globalReference = globalObject.transform;
        }

        if (tcpTransform == null && robotManual != null && robotManual.axisTransforms.Length >= 6)
        {
            Transform finalJoint = robotManual.axisTransforms[5];
            if (finalJoint != null)
            {
                tcpTransform = FindDescendant(finalJoint, "tcp");
                if (tcpTransform == null)
                    tcpTransform = FindDescendant(finalJoint, "axis6") ?? finalJoint;
            }
        }

        lineRenderer = GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
            lineRenderer.startWidth = 0.003f; // 3mm
            lineRenderer.endWidth = 0.003f;
            lineRenderer.material = new Material(Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Sprites/Default"));
            lineRenderer.material.color = inkColor;
            lineRenderer.useWorldSpace = true;
        }
    }

    /// <summary>
    /// Nạp file CSV chứa danh sách waypoint và nghiệm động học nghịch
    /// </summary>
    [ContextMenu("Nạp file quỹ đạo CSV")]
    public void LoadTrajectoryFile()
    {
        string fullPath = trajectoryCsvPath;
        if (!Path.IsPathRooted(fullPath))
        {
            fullPath = Path.Combine(Application.dataPath, "..", trajectoryCsvPath);
        }

        if (!File.Exists(fullPath))
        {
            // Thử tìm trong thư mục output_drawing
            string altPath = Path.Combine(Application.dataPath, "..", "..", trajectoryCsvPath);
            if (File.Exists(altPath)) fullPath = altPath;
        }

        if (!File.Exists(fullPath))
        {
            Debug.LogWarning($"[CR10A IK] Chưa tìm thấy file {fullPath}. Đang tính toán động học trực tiếp...");
            return;
        }

        waypoints.Clear();
        actualTcpTrail.Clear();

        try
        {
            string[] lines = File.ReadAllLines(fullPath);
            if (lines.Length <= 1) return;

            string[] headers = lines[0].Split(',');
            int idx_j1 = Array.IndexOf(headers, "j1_deg");
            int idx_x = Array.IndexOf(headers, "x_mm");
            int idx_y = Array.IndexOf(headers, "y_mm");
            int idx_z = Array.IndexOf(headers, "z_mm");
            int idx_roll = Array.IndexOf(headers, "roll_deg");
            int idx_pitch = Array.IndexOf(headers, "pitch_deg");
            int idx_yaw = Array.IndexOf(headers, "yaw_deg");
            int idx_dist = Array.IndexOf(headers, "dist_mm");

            for (int i = 1; i < lines.Length; i++)
            {
                if (string.IsNullOrWhiteSpace(lines[i])) continue;
                string[] cols = lines[i].Split(',');

                WaypointData wp = new WaypointData();
                wp.index = i - 1;
                wp.distance_mm = idx_dist >= 0 ? float.Parse(cols[idx_dist], CultureInfo.InvariantCulture) : (i - 1) * 1.0f;

                float x_mm = float.Parse(cols[idx_x], CultureInfo.InvariantCulture);
                float y_mm = float.Parse(cols[idx_y], CultureInfo.InvariantCulture);
                float z_mm = float.Parse(cols[idx_z], CultureInfo.InvariantCulture);
                wp.targetPosGlobal_mm = new Vector3(x_mm, y_mm, z_mm);
                Vector3 pointInRobotGlobal = wp.targetPosGlobal_mm * 0.001f;
                // Coordinates stay in the robot Global frame. Convert only this
                // derived copy to Unity world for visualization in the scene.
                wp.targetPosWorld_m = globalReference != null
                    ? globalReference.TransformPoint(pointInRobotGlobal)
                    : pointInRobotGlobal;

                wp.roll_deg = idx_roll >= 0 ? float.Parse(cols[idx_roll], CultureInfo.InvariantCulture) : 0f;
                wp.pitch_deg = idx_pitch >= 0 ? float.Parse(cols[idx_pitch], CultureInfo.InvariantCulture) : 0f;
                wp.yaw_deg = idx_yaw >= 0 ? float.Parse(cols[idx_yaw], CultureInfo.InvariantCulture) : 0f;

                if (idx_j1 >= 0)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        wp.jointAngles_deg[j] = float.Parse(cols[idx_j1 + j], CultureInfo.InvariantCulture);
                        wp.jointAngles_rad[j] = wp.jointAngles_deg[j] * Mathf.Deg2Rad;
                    }
                    wp.isSolved = true;
                }

                waypoints.Add(wp);
            }

            Debug.Log($"[CR10A IK] Đã nạp thành công {waypoints.Count} waypoint cho quỹ đạo viền quần.");
            // Loading trajectory data must not move the robot. Apply a waypoint
            // only after an explicit selection/reset or when playback advances.
            currentWaypointIndex = Mathf.Clamp(currentWaypointIndex, 0, waypoints.Count - 1);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[CR10A IK] Lỗi khi đọc file CSV: {ex.Message}");
        }
    }

    private void Update()
    {
        if (waypoints.Count == 0) return;

        if (isPlaying && Application.isPlaying)
        {
            if (MoveTowardWaypoint(currentWaypointIndex, Time.deltaTime))
            {
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Count;
            }
        }

        UpdateInkTrail();
    }

    /// <summary>
    /// Áp dụng nghiệm động học của waypoint được chọn vào các khớp robot
    /// </summary>
    public void ApplyWaypoint(int index)
    {
        if (index < 0 || index >= waypoints.Count) return;
        currentWaypointIndex = index;

        WaypointData wp = waypoints[index];

        if (robotManual != null && wp.isSolved)
        {
            // Gán góc quay (radian) vào robot manual
            robotManual.axis0 = wp.jointAngles_rad[0];
            robotManual.axis1 = wp.jointAngles_rad[1];
            robotManual.axis2 = wp.jointAngles_rad[2];
            robotManual.axis3 = wp.jointAngles_rad[3];
            robotManual.axis4 = wp.jointAngles_rad[4];
            robotManual.axis5 = wp.jointAngles_rad[5];

            robotManual.ApplyRotations();
            if (tcpTransform != null)
                actualTcpTrail.Add(tcpTransform.position);
        }
    }

    /// <summary>
    /// Move each joint toward the selected waypoint with a hard angular-speed cap.
    /// Returns true when all six joints have reached the waypoint.
    /// </summary>
    private bool MoveTowardWaypoint(int index, float deltaTime)
    {
        if (robotManual == null || index < 0 || index >= waypoints.Count)
            return false;

        WaypointData waypoint = waypoints[index];
        if (!waypoint.isSolved || waypoint.jointAngles_rad == null || waypoint.jointAngles_rad.Length < 6)
            return true;

        float speedScale = Mathf.Clamp(playbackSpeed, 0.1f, 1f);
        float speedLimit = Mathf.Clamp(maxJointSpeedDegPerSecond, 1f, 10f);
        float maxStepRad = speedLimit * speedScale * Mathf.Deg2Rad * Mathf.Max(0f, deltaTime);
        float[] current =
        {
            robotManual.axis0, robotManual.axis1, robotManual.axis2,
            robotManual.axis3, robotManual.axis4, robotManual.axis5
        };
        bool arrived = true;

        for (int joint = 0; joint < 6; joint++)
        {
            float deltaDeg = Mathf.DeltaAngle(current[joint] * Mathf.Rad2Deg,
                waypoint.jointAngles_rad[joint] * Mathf.Rad2Deg);
            float deltaRad = deltaDeg * Mathf.Deg2Rad;
            if (Mathf.Abs(deltaDeg) > 0.05f)
                arrived = false;
            current[joint] += Mathf.Clamp(deltaRad, -maxStepRad, maxStepRad);
        }

        robotManual.axis0 = current[0];
        robotManual.axis1 = current[1];
        robotManual.axis2 = current[2];
        robotManual.axis3 = current[3];
        robotManual.axis4 = current[4];
        robotManual.axis5 = current[5];
        robotManual.ApplyRotations();

        if (tcpTransform != null && (actualTcpTrail.Count == 0 ||
            Vector3.Distance(actualTcpTrail[actualTcpTrail.Count - 1], tcpTransform.position) >= 0.001f))
        {
            actualTcpTrail.Add(tcpTransform.position);
            // Keep enough 1 mm samples to display the complete ~4.5 m garment outline.
            if (actualTcpTrail.Count > 7000)
                actualTcpTrail.RemoveRange(0, actualTcpTrail.Count - 7000);
        }

        return arrived;
    }

    private void UpdateInkTrail()
    {
        if (!drawInkTrail || lineRenderer == null || waypoints.Count == 0) return;

        int count = actualTcpTrail.Count;
        lineRenderer.positionCount = count;

        for (int i = 0; i < count; i++)
        {
            // Vẽ theo vị trí TCP thật sau khi áp dụng góc khớp, không lấy điểm đích lý thuyết.
            lineRenderer.SetPosition(i, actualTcpTrail[i]);
        }
    }

    // =========================================================================
    // GIAO DIỆN KIỂM THỬ TƯƠNG TÁC (INTERACTIVE TESTING GUI)
    // =========================================================================
    private void OnGUI()
    {
        int panelWidth = 360;
        int panelHeight = 440;
        Rect panelRect = new Rect(Screen.width - panelWidth - 20, 20, panelWidth, panelHeight);

        GUI.Box(panelRect, "");
        GUILayout.BeginArea(new Rect(panelRect.x + 10, panelRect.y + 10, panelWidth - 20, panelHeight - 20));

        // Tiêu đề
        GUIStyle titleStyle = new GUIStyle(GUI.skin.label);
        titleStyle.fontSize = 14;
        titleStyle.fontStyle = FontStyle.Bold;
        titleStyle.normal.textColor = Color.yellow;
        GUILayout.Label("DOBOT CR10A - KIỂM THỬ ĐỘNG HỌC NGHỊCH", titleStyle);

        GUILayout.Space(5);

        // Nút điều khiển Playback
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(isPlaying ? "❚❚ TẠM DỪNG" : "▶ CHẠY TỰ ĐỘNG", GUILayout.Height(30)))
        {
            if (isPlaying)
            {
                isPlaying = false;
            }
            else
            {
                // Start from the currently selected waypoint. Otherwise playback
                // eases toward it instead of jumping directly to its joint angles.
                isPlaying = true;
            }
        }

        if (GUILayout.Button("<< ĐIỂM ĐẦU", GUILayout.Height(30)))
        {
            ApplyWaypoint(0);
        }

        if (GUILayout.Button("RESET", GUILayout.Height(30)))
        {
            isPlaying = false;
            actualTcpTrail.Clear();
            currentWaypointIndex = 0;
            if (robotManual != null)
                robotManual.ResetAllSlidersToZero();
            if (lineRenderer != null) lineRenderer.positionCount = 0;
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(5);

        // Thanh trượt chọn Waypoint
        GUILayout.Label($"Tiến trình vẽ: Điểm {currentWaypointIndex} / {Mathf.Max(0, waypoints.Count - 1)} ({(waypoints.Count > 0 ? waypoints[currentWaypointIndex].distance_mm : 0):F1} mm)");
        int newIdx = Mathf.RoundToInt(GUILayout.HorizontalSlider(currentWaypointIndex, 0, Mathf.Max(0, waypoints.Count - 1)));
        if (newIdx != currentWaypointIndex)
        {
            isPlaying = false;
            ApplyWaypoint(newIdx);
        }

        // Tốc độ vẽ
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Tốc độ: {playbackSpeed:F1}x", GUILayout.Width(90));
        playbackSpeed = GUILayout.HorizontalSlider(playbackSpeed, 0.1f, 1.0f);
        GUILayout.EndHorizontal();
        GUILayout.Label($"Giới hạn mỗi khớp: {Mathf.Clamp(maxJointSpeedDegPerSecond, 1f, 10f):F0}°/s");

        GUILayout.Space(8);

        // BẢNG THÔNG SỐ ĐỘNG HỌC (KINEMATICS DASHBOARD)
        GUIStyle boxStyle = new GUIStyle(GUI.skin.box);
        boxStyle.alignment = TextAnchor.UpperLeft;
        GUILayout.BeginVertical(boxStyle);

        if (waypoints.Count > 0 && currentWaypointIndex < waypoints.Count)
        {
            WaypointData wp = waypoints[currentWaypointIndex];

            // Trạng thái IK
            GUIStyle statusStyle = new GUIStyle(GUI.skin.label);
            statusStyle.fontStyle = FontStyle.Bold;
            statusStyle.normal.textColor = wp.isSolved ? Color.green : Color.red;
            GUILayout.Label(wp.isSolved ? "● TRẠNG THÁI: IK SOLVED (HỘI TỤ CHÍNH XÁC)" : "● TRẠNG THÁI: NGOÀI TẦM VỚI", statusStyle);

            GUILayout.Space(3);
            GUILayout.Label($"<b>Tọa độ TCP mục tiêu:</b>");
            GUILayout.Label($" Global robot: X = {wp.targetPosGlobal_mm.x:F2} | Y = {wp.targetPosGlobal_mm.y:F2} | Z = {wp.targetPosGlobal_mm.z:F2} mm");

            GUILayout.Space(3);
            GUILayout.Label($"<b>Tư thế TCP (Roll, Pitch, Yaw):</b>");
            GUILayout.Label($" Roll = {wp.roll_deg:F1}° | Pitch = {wp.pitch_deg:F1}° | Yaw = {wp.yaw_deg:F1}°");

            GUILayout.Space(5);
            GUILayout.Label($"<b>Góc quay 6 khớp (Joint Angles):</b>");
            GUILayout.Label($" J1: {wp.jointAngles_deg[0],6:F1}°   J2: {wp.jointAngles_deg[1],6:F1}°   J3: {wp.jointAngles_deg[2],6:F1}°");
            GUILayout.Label($" J4: {wp.jointAngles_deg[3],6:F1}°   J5: {wp.jointAngles_deg[4],6:F1}°   J6: {wp.jointAngles_deg[5],6:F1}°");
        }
        else
        {
            GUILayout.Label("Chưa nạp dữ liệu quỹ đạo.");
            if (GUILayout.Button("Nạp file quỹ đạo"))
            {
                LoadTrajectoryFile();
            }
        }

        GUILayout.EndVertical();

        GUILayout.Space(5);
        drawInkTrail = GUILayout.Toggle(drawInkTrail, " Hiển thị đường vẽ mực đỏ trên quần");

        GUILayout.EndArea();
    }

    private void OnDrawGizmos()
    {
        if (waypoints.Count == 0 || currentWaypointIndex >= waypoints.Count) return;

        WaypointData wp = waypoints[currentWaypointIndex];

        // Điểm mục tiêu TCP và vị trí TCP thực của GameObject tcp.
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(wp.targetPosWorld_m, 0.008f);
        if (tcpTransform != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(tcpTransform.position, 0.006f);
        }
    }

    private static Transform FindDescendant(Transform parent, string childName)
    {
        if (parent == null) return null;
        for (int i = 0; i < parent.childCount; i++)
        {
            Transform child = parent.GetChild(i);
            if (child.name == childName) return child;
            Transform nested = FindDescendant(child, childName);
            if (nested != null) return nested;
        }
        return null;
    }
}
