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
    [Range(1f, 223f)]
    public float maxJointSpeedDegPerSecond = 223f;
    [Range(1f, 1000f)]
    public float maxJointAccelerationDegPerSecond2 = 12f;
    [Tooltip("Giới hạn tốc độ riêng cho J1-J6, đơn vị độ/giây")]
    public float[] jointSpeedDegPerSecond = { 10f, 10f, 10f, 10f, 10f, 10f };

    [Header("5. Phun sơn (vệt màu đỏ)")]
    [FormerlySerializedAs("drawInkTrail")]
    public bool sprayPaintEnabled = true;
    public Color inkColor = Color.red;

    // Danh sách waypoint đã nạp
    public List<WaypointData> waypoints = new List<WaypointData>();

    private LineRenderer lineRenderer;
    private readonly List<Vector3> actualTcpTrail = new List<Vector3>();
    private bool paintingStarted;
    private bool isMovingToManualTarget;
    private bool showJointSpeedControls;
    private int interfaceTab;
    private Vector2 interfaceScrollPosition;
    private string accelerationInputText = "12";
    private readonly string[] jointSpeedInputTexts = { "10", "10", "10", "10", "10", "10" };
    private readonly float[] manualTargetAnglesRad = new float[6];
    private readonly float[] jointVelocitiesRadPerSecond = new float[6];

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
        EnsureJointSpeedSettings();

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

    private void EnsureJointSpeedSettings()
    {
        if (jointSpeedDegPerSecond == null || jointSpeedDegPerSecond.Length != 6)
        {
            float[] speeds = new float[6];
            for (int joint = 0; joint < speeds.Length; joint++)
                speeds[joint] = jointSpeedDegPerSecond != null && joint < jointSpeedDegPerSecond.Length
                    ? jointSpeedDegPerSecond[joint]
                    : maxJointSpeedDegPerSecond;
            jointSpeedDegPerSecond = speeds;
        }

        float[] hardwareLimits = { 150f, 150f, 180f, 223f, 223f, 223f };
        for (int joint = 0; joint < jointSpeedDegPerSecond.Length; joint++)
            jointSpeedDegPerSecond[joint] = Mathf.Clamp(jointSpeedDegPerSecond[joint], 1f, hardwareLimits[joint]);
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
        paintingStarted = false;
        isMovingToManualTarget = false;

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
                else
                {
                    // Tự động giải nghịch bằng thuật toán Thuần Giải Tích (Closed-Form Analytical IK)
                    Quaternion rot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                    float[] prevAngles = waypoints.Count > 0 ? waypoints[waypoints.Count - 1].jointAngles_rad : null;
                    var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, prevAngles, toolLength, useAdaptiveTilt: true);
                    if (sols != null && sols.Count > 0)
                    {
                        var best = sols[0];
                        for (int j = 0; j < 6; j++)
                        {
                            wp.jointAngles_rad[j] = (float)best.jointsRad[j];
                            wp.jointAngles_deg[j] = (float)best.jointsDeg[j];
                        }
                        wp.isSolved = true;
                        wp.posError_mm = (float)best.positionErrorMm;
                    }
                }

                waypoints.Add(wp);
            }

            Debug.Log($"[CR10A IK] Đã nạp thành công {waypoints.Count} waypoint từ {trajectoryCsvPath}.");
            // Loading trajectory data must not move the robot. Apply a waypoint
            // only after an explicit selection/reset or when playback advances.
            currentWaypointIndex = Mathf.Clamp(currentWaypointIndex, 0, waypoints.Count - 1);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[CR10A IK] Lỗi khi đọc file CSV: {ex.Message}");
        }
    }

    private void LoadNamedTrajectory(string relativePath)
    {
        isPlaying = false;
        isMovingToManualTarget = false;
        trajectoryCsvPath = relativePath;
        LoadTrajectoryFile();
    }

    public void LoadPantsOutlineTrajectory()
    {
        LoadNamedTrajectory("output_drawing/pants_outline_cr10a.csv");
    }

    public void LoadTriangleTestTrajectory()
    {
        LoadNamedTrajectory("output_drawing/triangle_test_cr10a.csv");
    }

    /// <summary>
    /// Giải lại toàn bộ IK cho tất cả các điểm waypoint bằng Thuật toán Thuần Giải Tích (Closed-Form Analytical IK)
    /// </summary>
    [ContextMenu("Tính lại toàn bộ waypoint bằng Thuần Giải Tích")]
    public void RecomputeAllWaypointsAnalyticalIK()
    {
        if (waypoints == null || waypoints.Count == 0) return;
        int solvedCount = 0;
        float[] lastAngles = null;

        for (int i = 0; i < waypoints.Count; i++)
        {
            WaypointData wp = waypoints[i];
            Quaternion rot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
            var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, lastAngles, toolLength, useAdaptiveTilt: true);
            if (sols != null && sols.Count > 0)
            {
                var best = sols[0];
                for (int j = 0; j < 6; j++)
                {
                    wp.jointAngles_rad[j] = (float)best.jointsRad[j];
                    wp.jointAngles_deg[j] = (float)best.jointsDeg[j];
                }
                wp.isSolved = true;
                wp.posError_mm = (float)best.positionErrorMm;
                lastAngles = wp.jointAngles_rad;
                solvedCount++;
            }
        }
        Debug.Log($"[CR10A IK] Đã giải tích thành công {solvedCount}/{waypoints.Count} waypoint bằng Closed-Form Analytical IK.");
    }

    private void Update()
    {
        if (waypoints.Count == 0 && !isMovingToManualTarget) return;

        if (isMovingToManualTarget && Application.isPlaying)
        {
            if (MoveTowardJointAngles(manualTargetAnglesRad, Time.deltaTime, false))
                isMovingToManualTarget = false;
        }
        else if (isPlaying && Application.isPlaying && waypoints.Count > 0)
        {
            if (MoveTowardWaypoint(currentWaypointIndex, Time.deltaTime))
            {
                // Start spraying only after reaching the first selected waypoint;
                // the travel from the robot's current pose stays unpainted.
                paintingStarted = true;
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Count;
            }
        }

        UpdateInkTrail();
    }

    /// <summary>
    /// Áp dụng nghiệm động học của waypoint được chọn vào các khớp robot
    /// </summary>
    private void MoveToWaypointGradually(int index)
    {
        if (index < 0 || index >= waypoints.Count) return;

        WaypointData waypoint = waypoints[index];
        if (!waypoint.isSolved || waypoint.jointAngles_rad == null || waypoint.jointAngles_rad.Length < 6)
            return;

        currentWaypointIndex = index;
        paintingStarted = false;
        for (int joint = 0; joint < manualTargetAnglesRad.Length; joint++)
            manualTargetAnglesRad[joint] = waypoint.jointAngles_rad[joint];
        isMovingToManualTarget = true;
    }

    public void ApplyWaypoint(int index)
    {
        if (index < 0 || index >= waypoints.Count) return;
        currentWaypointIndex = index;
        paintingStarted = false;

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

        return MoveTowardJointAngles(waypoint.jointAngles_rad, deltaTime, true);
    }

    private bool MoveTowardJointAngles(float[] targetAnglesRad, float deltaTime, bool allowSpraying)
    {
        if (robotManual == null || targetAnglesRad == null || targetAnglesRad.Length < 6)
            return false;

        float speedScale = Mathf.Clamp(playbackSpeed, 0.1f, 1f);
        float globalSpeedLimit = Mathf.Clamp(maxJointSpeedDegPerSecond, 1f, 223f);
        float maxAccelerationRad = Mathf.Clamp(maxJointAccelerationDegPerSecond2, 1f, 1000f) *
            speedScale * Mathf.Deg2Rad;
        float frameTime = Mathf.Max(0f, deltaTime);
        float[] current =
        {
            robotManual.axis0, robotManual.axis1, robotManual.axis2,
            robotManual.axis3, robotManual.axis4, robotManual.axis5
        };
        bool arrived = true;

        for (int joint = 0; joint < 6; joint++)
        {
            float deltaDeg = Mathf.DeltaAngle(current[joint] * Mathf.Rad2Deg,
                targetAnglesRad[joint] * Mathf.Rad2Deg);
            float deltaRad = deltaDeg * Mathf.Deg2Rad;
            if (Mathf.Abs(deltaDeg) <= 0.05f)
            {
                current[joint] += deltaRad;
                jointVelocitiesRadPerSecond[joint] = 0f;
                continue;
            }

            arrived = false;
            // Limit speed by the remaining stopping distance so each joint
            // accelerates smoothly and brakes before reaching its target.
            float jointSpeedLimit = Mathf.Clamp(jointSpeedDegPerSecond[joint], 1f,
                Mathf.Min(globalSpeedLimit, GetHardwareJointSpeedLimit(joint)));
            float maxVelocityRad = jointSpeedLimit * speedScale * Mathf.Deg2Rad;
            float stoppingSpeed = Mathf.Sqrt(2f * maxAccelerationRad * Mathf.Abs(deltaRad));
            float desiredSpeed = Mathf.Sign(deltaRad) * Mathf.Min(maxVelocityRad, stoppingSpeed);
            jointVelocitiesRadPerSecond[joint] = Mathf.MoveTowards(
                jointVelocitiesRadPerSecond[joint], desiredSpeed, maxAccelerationRad * frameTime);
            float stepRad = jointVelocitiesRadPerSecond[joint] * frameTime;

            if (Mathf.Sign(stepRad) == Mathf.Sign(deltaRad) &&
                Mathf.Abs(stepRad) >= Mathf.Abs(deltaRad))
            {
                current[joint] += deltaRad;
                jointVelocitiesRadPerSecond[joint] = 0f;
            }
            else
            {
                current[joint] += stepRad;
            }
        }

        robotManual.axis0 = current[0];
        robotManual.axis1 = current[1];
        robotManual.axis2 = current[2];
        robotManual.axis3 = current[3];
        robotManual.axis4 = current[4];
        robotManual.axis5 = current[5];
        robotManual.ApplyRotations();

        if (allowSpraying && sprayPaintEnabled && paintingStarted && tcpTransform != null && (actualTcpTrail.Count == 0 ||
            Vector3.Distance(actualTcpTrail[actualTcpTrail.Count - 1], tcpTransform.position) >= 0.001f))
        {
            actualTcpTrail.Add(tcpTransform.position);
            // Keep enough 1 mm samples to display the complete ~4.5 m garment outline.
            if (actualTcpTrail.Count > 7000)
                actualTcpTrail.RemoveRange(0, actualTcpTrail.Count - 7000);
        }

        return arrived;
    }

    private static float GetHardwareJointSpeedLimit(int joint)
    {
        switch (joint)
        {
            case 0:
            case 1: return 150f;
            case 2: return 180f;
            default: return 223f;
        }
    }

    private void UpdateInkTrail()
    {
        if (lineRenderer == null) return;

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
        int panelWidth = 460;
        int panelHeight = Mathf.Min(820, Mathf.Max(360, Screen.height - 40));
        Rect panelRect = new Rect(Screen.width - panelWidth - 20, 20, panelWidth, panelHeight);

        GUI.Box(panelRect, "");
        GUILayout.BeginArea(new Rect(panelRect.x + 12, panelRect.y + 10, panelWidth - 24, panelHeight - 20));

        GUIStyle titleStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 16,
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.white }
        };
        GUILayout.Label("TRẠM PHUN TỰ ĐỘNG  |  DOBOT CR10A", titleStyle);
        GUILayout.Label("Mô phỏng quỹ đạo và trạng thái quy trình", GUI.skin.label);
        GUILayout.Space(8);

        GUILayout.BeginHorizontal();
        DrawInterfaceTabButton(0, "ĐIỀU KHIỂN");
        DrawInterfaceTabButton(1, "PIPELINE 3D / AI");
        DrawInterfaceTabButton(2, "CHỈ TIÊU");
        GUILayout.EndHorizontal();
        GUILayout.Space(6);

        interfaceScrollPosition = GUILayout.BeginScrollView(interfaceScrollPosition,
            GUILayout.Width(panelWidth - 28), GUILayout.Height(panelHeight - 88));
        if (interfaceTab == 0)
            DrawRobotControlPanel();
        else if (interfaceTab == 1)
            DrawPipelinePanel();
        else
            DrawEvaluationPanel();
        GUILayout.EndScrollView();

        GUILayout.EndArea();
    }

    private void DrawInterfaceTabButton(int tab, string label)
    {
        GUIStyle style = new GUIStyle(GUI.skin.button)
        {
            fontStyle = interfaceTab == tab ? FontStyle.Bold : FontStyle.Normal
        };
        if (interfaceTab == tab)
            style.normal.textColor = Color.white;
        if (GUILayout.Button(label, style, GUILayout.Height(32)))
            interfaceTab = tab;
    }

    private void DrawRobotControlPanel()
    {
        GUILayout.Label("CHẠY QUỸ ĐẠO MÔ PHỎNG", GUI.skin.box);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(isPlaying ? "❚❚ TẠM DỪNG" : "▶ CHẠY TỰ ĐỘNG", GUILayout.Height(32)))
            isPlaying = !isPlaying;
        if (GUILayout.Button("VỀ ĐIỂM ĐẦU", GUILayout.Height(32)))
        {
            isPlaying = false;
            MoveToWaypointGradually(0);
        }
        if (GUILayout.Button("RESET", GUILayout.Height(32)))
        {
            isPlaying = false;
            actualTcpTrail.Clear();
            paintingStarted = false;
            currentWaypointIndex = 0;
            for (int joint = 0; joint < manualTargetAnglesRad.Length; joint++)
                manualTargetAnglesRad[joint] = 0f;
            isMovingToManualTarget = true;
            if (lineRenderer != null) lineRenderer.positionCount = 0;
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("NẠP QUỸ ĐẠO QUẦN", GUILayout.Height(28)))
            LoadPantsOutlineTrajectory();
        if (GUILayout.Button("NẠP TAM GIÁC THỬ", GUILayout.Height(28)))
            LoadTriangleTestTrajectory();
        GUILayout.EndHorizontal();

        int waypoint = waypoints.Count > 0 ? Mathf.Clamp(currentWaypointIndex, 0, waypoints.Count - 1) : 0;
        float distance = waypoints.Count > 0 ? waypoints[waypoint].distance_mm : 0f;
        GUILayout.Label($"Waypoint: {waypoint} / {Mathf.Max(0, waypoints.Count - 1)}    Quãng đường: {distance:F1} mm");
        int newIdx = Mathf.RoundToInt(GUILayout.HorizontalSlider(waypoint, 0, Mathf.Max(0, waypoints.Count - 1)));
        if (newIdx != currentWaypointIndex)
        {
            isPlaying = false;
            ApplyWaypoint(newIdx);
        }

        GUILayout.Space(4);
        GUILayout.Label("TỐC ĐỘ VÀ CHUYỂN ĐỘNG", GUI.skin.box);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Tốc độ tổng: {playbackSpeed:F1}×", GUILayout.Width(130));
        playbackSpeed = GUILayout.HorizontalSlider(playbackSpeed, 0.1f, 1.0f);
        GUILayout.EndHorizontal();
        GUILayout.Label($"Trần chung: {Mathf.Clamp(maxJointSpeedDegPerSecond, 1f, 223f):F0}°/s");
        DrawAccelerationInput();

        showJointSpeedControls = GUILayout.Toggle(showJointSpeedControls, " Chỉnh tốc độ riêng từng khớp");
        if (showJointSpeedControls)
            DrawJointSpeedInputs();

        GUILayout.Space(4);
        DrawCurrentWaypointStatus();

        GUILayout.Space(4);
        sprayPaintEnabled = GUILayout.Toggle(sprayPaintEnabled,
            sprayPaintEnabled ? " PHUN SƠN ĐỎ: BẬT" : " PHUN SƠN ĐỎ: TẮT");
    }

    private void DrawAccelerationInput()
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label("Gia tốc giới hạn:", GUILayout.Width(130));
        const string controlName = "JointAccelerationInput";
        GUI.SetNextControlName(controlName);
        if (GUI.GetNameOfFocusedControl() != controlName)
            accelerationInputText = maxJointAccelerationDegPerSecond2.ToString("F1", CultureInfo.InvariantCulture);
        accelerationInputText = GUILayout.TextField(accelerationInputText, GUILayout.Width(70));
        if (float.TryParse(accelerationInputText, NumberStyles.Float, CultureInfo.InvariantCulture,
            out float enteredAcceleration))
            maxJointAccelerationDegPerSecond2 = Mathf.Clamp(enteredAcceleration, 1f, 1000f);
        GUILayout.Label("°/s²");
        GUILayout.EndHorizontal();
    }

    private void DrawJointSpeedInputs()
    {
        EnsureJointSpeedSettings();
        for (int joint = 0; joint < jointSpeedDegPerSecond.Length; joint++)
        {
            float jointLimit = Mathf.Min(Mathf.Clamp(maxJointSpeedDegPerSecond, 1f, 223f),
                GetHardwareJointSpeedLimit(joint));
            GUILayout.BeginHorizontal();
            GUILayout.Label($"J{joint + 1}", GUILayout.Width(34));
            float previousSpeed = jointSpeedDegPerSecond[joint];
            float sliderSpeed = GUILayout.HorizontalSlider(previousSpeed, 1f, jointLimit);
            if (!Mathf.Approximately(sliderSpeed, previousSpeed))
            {
                jointSpeedDegPerSecond[joint] = sliderSpeed;
                jointSpeedInputTexts[joint] = sliderSpeed.ToString("F1", CultureInfo.InvariantCulture);
                GUI.FocusControl(null);
            }

            string controlName = $"Joint{joint + 1}SpeedInput";
            GUI.SetNextControlName(controlName);
            if (GUI.GetNameOfFocusedControl() != controlName)
                jointSpeedInputTexts[joint] = jointSpeedDegPerSecond[joint].ToString("F1", CultureInfo.InvariantCulture);
            jointSpeedInputTexts[joint] = GUILayout.TextField(jointSpeedInputTexts[joint], GUILayout.Width(66));
            if (float.TryParse(jointSpeedInputTexts[joint], NumberStyles.Float, CultureInfo.InvariantCulture,
                out float enteredSpeed))
                jointSpeedDegPerSecond[joint] = Mathf.Clamp(enteredSpeed, 1f, jointLimit);
            GUILayout.Label("°/s", GUILayout.Width(32));
            GUILayout.EndHorizontal();
        }
    }

    private void DrawCurrentWaypointStatus()
    {
        GUILayout.Label("TRẠNG THÁI ROBOT", GUI.skin.box);
        if (waypoints.Count == 0)
        {
            GUILayout.Label("Chưa nạp quỹ đạo mô phỏng.");
            return;
        }

        WaypointData wp = waypoints[Mathf.Clamp(currentWaypointIndex, 0, waypoints.Count - 1)];
        GUIStyle statusStyle = new GUIStyle(GUI.skin.label)
        {
            fontStyle = FontStyle.Bold,
            normal = { textColor = wp.isSolved ? Color.green : Color.red }
        };
        GUILayout.Label(wp.isSolved ? "IK: CÓ NGHIỆM TRONG DỮ LIỆU QUỸ ĐẠO" : "IK: KHÔNG CÓ NGHIỆM", statusStyle);
        GUILayout.Label($"TCP mục tiêu (mm): X {wp.targetPosGlobal_mm.x:F1}   Y {wp.targetPosGlobal_mm.y:F1}   Z {wp.targetPosGlobal_mm.z:F1}");
        GUILayout.Label($"Góc TCP RPY (°): {wp.roll_deg:F1}, {wp.pitch_deg:F1}, {wp.yaw_deg:F1}");
        GUILayout.Label($"Góc khớp (°): {wp.jointAngles_deg[0]:F1}, {wp.jointAngles_deg[1]:F1}, {wp.jointAngles_deg[2]:F1}");
        GUILayout.Label($"                 {wp.jointAngles_deg[3]:F1}, {wp.jointAngles_deg[4]:F1}, {wp.jointAngles_deg[5]:F1}");
    }

    private void DrawPipelinePanel()
    {
        GUILayout.Label("PIPELINE ĐỀ XUẤT TRONG BÁO CÁO", GUI.skin.box);
        DrawBlankPipelineSection("01  THU NHẬN DỮ LIỆU", "Ảnh RGB  |  Point cloud XYZ  |  Zivid .ZDF / .PLY", 72);
        DrawBlankPipelineSection("02  PHÂN VÙNG VÀ MỐC AI", "Mask quần  |  vùng phun  |  mốc hình học", 72);
        DrawBlankPipelineSection("03  BỀ MẶT 2.5D / 3D", "Lọc điểm  |  mặt bàn/khuôn  |  pháp tuyến", 72);
        DrawBlankPipelineSection("04  THAM SỐ SÚNG", "Khoảng cách d  |  vệt w, σ  |  lưu lượng Q  |  tốc độ v  |  θmax  |  biên e", 64);
        DrawBlankPipelineSection("05  SINH QUỸ ĐẠO", "P1 Zigzag  |  P3 theo mốc  |  P4 đăng ký ICP / CPD / TPS", 82);
        DrawBlankPipelineSection("06  KIỂM TRA VÀ XUẤT LỆNH", "Trong vùng  |  tầm với / IK  |  kỳ dị  |  va chạm  |  tốc độ TCP  |  ROS 2 / Gazebo", 72);
    }

    private void DrawBlankPipelineSection(string title, string fieldLabels, float height)
    {
        GUILayout.BeginVertical(GUI.skin.box);
        GUILayout.Label(title);
        GUILayout.Label(fieldLabels, GUI.skin.label);
        GUILayout.Box(GUIContent.none, GUILayout.ExpandWidth(true), GUILayout.Height(height));
        GUILayout.EndVertical();
    }

    private void DrawEvaluationPanel()
    {
        GUILayout.Label("KẾT QUẢ ĐÁNH GIÁ", GUI.skin.box);
        DrawBlankMetricRow("E_geo", "Sai số hình học", "mm · TB / P95 / Hausdorff");
        DrawBlankMetricRow("E_d", "Sai số khoảng cách súng", "mm");
        DrawBlankMetricRow("E_θ", "Sai số góc so với pháp tuyến", "°");
        DrawBlankMetricRow("E_T", "Sai lệch mật độ phun", "% RMS");
        DrawBlankMetricRow("R_drop", "Độ phủ / lượng sơn tràn", "%");
        DrawBlankMetricRow("v_TCP", "Độ đều tốc độ TCP", "% dao động");
        DrawBlankMetricRow("t", "Thời gian xử lý / chu kỳ", "s");

        GUILayout.Space(8);
        GUILayout.Label("SO SÁNH PHƯƠNG PHÁP TRÊN CÙNG DỮ LIỆU", GUI.skin.box);
        GUILayout.BeginHorizontal();
        GUILayout.Label("Chỉ tiêu", GUILayout.Width(120));
        GUILayout.Label("P1 Zigzag", GUILayout.Width(82));
        GUILayout.Label("P3 Mốc", GUILayout.Width(72));
        GUILayout.Label("P4 ICP/CPD", GUILayout.Width(88));
        GUILayout.EndHorizontal();
        DrawBlankComparisonRow("E_geo");
        DrawBlankComparisonRow("E_d");
        DrawBlankComparisonRow("E_θ");
        DrawBlankComparisonRow("E_T");
        DrawBlankComparisonRow("R_drop");
        DrawBlankComparisonRow("v_TCP");
        DrawBlankComparisonRow("Tạo (s)");
        DrawBlankComparisonRow("Khả thi (%)");
    }

    private void DrawBlankMetricRow(string code, string title, string unit)
    {
        GUILayout.BeginVertical(GUI.skin.box);
        GUILayout.BeginHorizontal();
        GUILayout.Label(code, GUILayout.Width(58));
        GUILayout.Label(title, GUILayout.ExpandWidth(true));
        GUILayout.Label(unit, GUILayout.Width(130));
        GUILayout.EndHorizontal();
        GUILayout.Box(GUIContent.none, GUILayout.ExpandWidth(true), GUILayout.Height(24));
        GUILayout.EndVertical();
    }

    private void DrawBlankComparisonRow(string metric)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(metric, GUILayout.Width(120));
        GUILayout.Box(GUIContent.none, GUILayout.Width(82), GUILayout.Height(22));
        GUILayout.Box(GUIContent.none, GUILayout.Width(72), GUILayout.Height(22));
        GUILayout.Box(GUIContent.none, GUILayout.Width(88), GUILayout.Height(22));
        GUILayout.EndHorizontal();
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
