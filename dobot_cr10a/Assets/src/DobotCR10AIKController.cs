using System;
using System.IO;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// Bộ điều khiển Động học Nghịch (Inverse Kinematics) & Giao diện kiểm thử cho Dobot CR10A.
/// Giao diện phong cách Windows Ribbon UI (Dải Ruy Băng) hiện đại, thống nhất.
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
        public Vector3 targetPosWorld_m;   // Bản đổi sang Unity world để vẽ gizmo (m)
        public Vector3 microNormal;       // Pháp tuyến vi mô trong hệ Robot
        public float roll_deg;
        public float pitch_deg;
        public float yaw_deg;
        public float[] jointAngles_deg = new float[6];
        public float[] jointAngles_rad = new float[6];
        public float posError_mm;
        public bool isSolved;
        public int selectedSolutionIndex = -1;
        public string selectedBranchName = "";
    }

    [Header("1. Nguồn dữ liệu quỹ đạo")]
    [Tooltip("Đường dẫn file CSV quỹ đạo góc khớp hoặc tọa độ")]
    public string trajectoryCsvPath = "output_drawing/pants_outline_cr10a.csv";

    [Header("2. Kích thước hình học Dobot CR10A (mm)")]
    public float d1 = 176.5f;   // Đế -> Khớp 2
    public float a2 = 607.0f;   // Khớp 2 -> Khớp 3
    public float a3 = 568.0f;   // Khớp 3 -> Khớp 4
    public float d4 = 125.0f;   // Offset Khớp 4 -> Khớp 5
    public float d5 = 125.0f;   // Offset Khớp 5 -> Khớp 6
    public float d6 = 111.0f;   // Khớp 6 -> axis6/flange
    public float toolLength = 100.0f; // Chiều dài đầu phun TCP từ mặt bích (mm)

    [Header("3. Tham chiếu mô hình Robot")]
    [Tooltip("Script manual.cs điều khiển 6 khớp robot")]
    public manual robotManual;

    [Tooltip("Frame Global gắn với gốc robot")]
    public Transform globalReference;

    [Tooltip("TCP thực: GameObject tcp (con của axis6)")]
    public Transform tcpTransform;

    [Header("4. Trạng thái điều khiển & Kiểm thử")]
    public bool isPlaying = false;
    [Range(0, 5000)]
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
    
    // Quản lý trạng thái Windows Ribbon UI
    public int interfaceTab = 0; // 0: Trang chủ, 1: Động học, 2: Quỹ đạo & Sơn, 3: Pipeline 3D/AI, 4: Chỉ tiêu, 5: Xem (View)
    public bool ribbonExpanded = true;
    public bool showGizmos = true;
    private Vector2 dockScrollPosition;

    [Header("Quản lý hiển thị Panels (Tab View)")]
    public bool showJointsPanel = true;
    public bool showFkMonitorPanel = true;
    public bool showIkTargetPanel = true;
    public bool showTrajectoryPanel = true;
    public bool showPipelinePanel = false;
    public bool showEvaluationPanel = false;

    private string accelerationInputText = "12";
    private readonly string[] jointSpeedInputTexts = { "10", "10", "10", "10", "10", "10" };
    private readonly float[] manualTargetAnglesRad = new float[6];
    private readonly float[] jointVelocitiesRadPerSecond = new float[6];

    private readonly Dictionary<int, int> customWaypointConfigurations = new Dictionary<int, int>();
    private Vector2 waypointSolsScroll = Vector2.zero;

    private void Awake()
    {
        EnsureComponents();
        LoadAllSettings();
    }

    private void Start()
    {
        EnsureComponents();
        LoadAllSettings();
        LoadTrajectoryFile();
        EnsureCameraController();
    }

    private void OnDisable()
    {
        SaveAllSettings();
    }

    private void OnDestroy()
    {
        SaveAllSettings();
    }

    public void SaveAllSettings()
    {
        PlayerPrefs.SetInt("CR10A_ShowJoints", showJointsPanel ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_ShowFk", showFkMonitorPanel ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_ShowIk", showIkTargetPanel ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_ShowTrajectory", showTrajectoryPanel ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_ShowPipeline", showPipelinePanel ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_ShowEvaluation", showEvaluationPanel ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_RibbonExpanded", ribbonExpanded ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_InterfaceTab", interfaceTab);
        PlayerPrefs.SetInt("CR10A_ShowGizmos", showGizmos ? 1 : 0);

        PlayerPrefs.SetFloat("CR10A_PlaybackSpeed", playbackSpeed);
        PlayerPrefs.SetFloat("CR10A_MaxJointSpeed", maxJointSpeedDegPerSecond);
        PlayerPrefs.SetFloat("CR10A_MaxJointAcc", maxJointAccelerationDegPerSecond2);
        PlayerPrefs.SetInt("CR10A_SprayPaint", sprayPaintEnabled ? 1 : 0);
        PlayerPrefs.SetString("CR10A_TrajectoryCsv", trajectoryCsvPath);

        // Lưu danh sách cấu hình waypoint tùy chỉnh
        List<string> entries = new List<string>();
        foreach (var kvp in customWaypointConfigurations)
        {
            entries.Add($"{kvp.Key}:{kvp.Value}");
        }
        string key = "CR10A_CustomWpConfigs_" + (string.IsNullOrEmpty(trajectoryCsvPath) ? "default" : trajectoryCsvPath.Replace("/", "_").Replace("\\", "_"));
        PlayerPrefs.SetString(key, string.Join(";", entries));

        PlayerPrefs.Save();
    }

    public void LoadAllSettings()
    {
        if (PlayerPrefs.HasKey("CR10A_ShowJoints"))
        {
            showJointsPanel = PlayerPrefs.GetInt("CR10A_ShowJoints", 1) == 1;
            showFkMonitorPanel = PlayerPrefs.GetInt("CR10A_ShowFk", 1) == 1;
            showIkTargetPanel = PlayerPrefs.GetInt("CR10A_ShowIk", 1) == 1;
            showTrajectoryPanel = PlayerPrefs.GetInt("CR10A_ShowTrajectory", 1) == 1;
            showPipelinePanel = PlayerPrefs.GetInt("CR10A_ShowPipeline", 0) == 1;
            showEvaluationPanel = PlayerPrefs.GetInt("CR10A_ShowEvaluation", 0) == 1;
            ribbonExpanded = PlayerPrefs.GetInt("CR10A_RibbonExpanded", 1) == 1;
            interfaceTab = PlayerPrefs.GetInt("CR10A_InterfaceTab", 0);
            showGizmos = PlayerPrefs.GetInt("CR10A_ShowGizmos", 1) == 1;

            playbackSpeed = PlayerPrefs.GetFloat("CR10A_PlaybackSpeed", playbackSpeed);
            maxJointSpeedDegPerSecond = PlayerPrefs.GetFloat("CR10A_MaxJointSpeed", maxJointSpeedDegPerSecond);
            maxJointAccelerationDegPerSecond2 = PlayerPrefs.GetFloat("CR10A_MaxJointAcc", maxJointAccelerationDegPerSecond2);
            sprayPaintEnabled = PlayerPrefs.GetInt("CR10A_SprayPaint", sprayPaintEnabled ? 1 : 0) == 1;
            trajectoryCsvPath = PlayerPrefs.GetString("CR10A_TrajectoryCsv", trajectoryCsvPath);
        }

        customWaypointConfigurations.Clear();
        string key = "CR10A_CustomWpConfigs_" + (string.IsNullOrEmpty(trajectoryCsvPath) ? "default" : trajectoryCsvPath.Replace("/", "_").Replace("\\", "_"));
        if (PlayerPrefs.HasKey(key))
        {
            string raw = PlayerPrefs.GetString(key, "");
            if (!string.IsNullOrEmpty(raw))
            {
                string[] entries = raw.Split(';');
                foreach (string entry in entries)
                {
                    string[] parts = entry.Split(':');
                    if (parts.Length == 2 && int.TryParse(parts[0], out int wpIdx) && int.TryParse(parts[1], out int solIdx))
                    {
                        customWaypointConfigurations[wpIdx] = solIdx;
                    }
                }
            }
        }
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

        if (tcpTransform == null && robotManual != null && robotManual.axisTransforms != null && robotManual.axisTransforms.Length >= 6)
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
            string altPath = Path.Combine(Application.dataPath, "..", "..", trajectoryCsvPath);
            if (File.Exists(altPath)) fullPath = altPath;
        }

        if (!File.Exists(fullPath))
        {
            Debug.LogWarning($"[CR10A IK] Chưa tìm thấy file {fullPath}.");
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
            currentWaypointIndex = Mathf.Clamp(currentWaypointIndex, 0, waypoints.Count - 1);
            ApplySavedWaypointConfigurations();
        }
        catch (Exception ex)
        {
            Debug.LogError($"[CR10A IK] Lỗi khi đọc file CSV: {ex.Message}");
        }
    }

    private void ApplySavedWaypointConfigurations()
    {
        if (waypoints == null || waypoints.Count == 0 || customWaypointConfigurations.Count == 0) return;
        foreach (var kvp in customWaypointConfigurations)
        {
            int idx = kvp.Key;
            int solIdx = kvp.Value;
            if (idx >= 0 && idx < waypoints.Count)
            {
                WaypointData wp = waypoints[idx];
                Quaternion rot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                float[] seed = idx > 0 ? waypoints[idx - 1].jointAngles_rad : null;
                var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, seed, toolLength, useAdaptiveTilt: true);
                if (sols != null && solIdx >= 0 && solIdx < sols.Count)
                {
                    var chosen = sols[solIdx];
                    for (int j = 0; j < 6; j++)
                    {
                        wp.jointAngles_rad[j] = (float)chosen.jointsRad[j];
                        wp.jointAngles_deg[j] = (float)chosen.jointsDeg[j];
                    }
                    wp.isSolved = true;
                    wp.posError_mm = (float)chosen.positionErrorMm;
                    wp.selectedSolutionIndex = solIdx;
                    wp.selectedBranchName = chosen.branchName;
                }
            }
        }
    }

    private void LoadNamedTrajectory(string relativePath)
    {
        isPlaying = false;
        isMovingToManualTarget = false;
        trajectoryCsvPath = relativePath;
        LoadTrajectoryFile();
        SaveAllSettings();
    }

    public void LoadPantsOutlineTrajectory()
    {
        LoadNamedTrajectory("output_drawing/pants_outline_cr10a.csv");
    }

    public void LoadTriangleTestTrajectory()
    {
        LoadNamedTrajectory("output_drawing/triangle_test_cr10a.csv");
    }

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
                paintingStarted = true;
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Count;
            }
        }

        UpdateInkTrail();
    }

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
            robotManual.axis0 = wp.jointAngles_rad[0];
            robotManual.axis1 = wp.jointAngles_rad[1];
            robotManual.axis2 = wp.jointAngles_rad[2];
            robotManual.axis3 = wp.jointAngles_rad[3];
            robotManual.axis4 = wp.jointAngles_rad[4];
            robotManual.axis5 = wp.jointAngles_rad[5];

            robotManual.ApplyRotations();
        }
    }

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
            lineRenderer.SetPosition(i, actualTcpTrail[i]);
        }
    }

    // =========================================================================
    // GIAO DIỆN KIỂU WINDOWS RIBBON (DẢI RUY BĂNG) THỐNG NHẤT
    // =========================================================================

    private void OnGUI()
    {
        AntigravityUITheme.EnsureInitialized();
        if (robotManual == null) EnsureComponents();

        float screenW = Screen.width;
        float headerHeight = 38f;
        float stripHeight = ribbonExpanded ? 96f : 0f;
        float totalRibbonH = headerHeight + stripHeight;

        // 1. THANH RUY BĂNG TRÊN CÙNG (TOP RIBBON BAR - NỀN TỐI PHẲNG ANTIGRAVITY)
        GUI.DrawTexture(new Rect(0, 0, screenW, totalRibbonH), AntigravityUITheme.TexBgHeader);
        GUI.DrawTexture(new Rect(0, totalRibbonH - 1, screenW, 1), AntigravityUITheme.TexBorder);
        if (ribbonExpanded)
        {
            GUI.DrawTexture(new Rect(0, headerHeight, screenW, stripHeight), AntigravityUITheme.TexBgStrip);
            GUI.DrawTexture(new Rect(0, headerHeight - 1, screenW, 1), AntigravityUITheme.TexBorder);
        }

        // Header Line: Logo + Tabs + Status Badges
        GUILayout.BeginArea(new Rect(12, 3, screenW - 24, headerHeight));
        GUILayout.BeginHorizontal();

        // Brand Title
        GUILayout.Label("DOBOT CR10A WORKBENCH", AntigravityUITheme.StyleHeaderTitle, GUILayout.Width(240));

        // Ribbon Tabs phong cách Antigravity
        string[] tabNames = {
            "TRANG CHỦ",
            "ĐỘNG HỌC",
            "QUỸ ĐẠO & SƠN",
            "PIPELINE 3D / AI",
            "CHỈ TIÊU ĐÁNH GIÁ",
            "XEM (VIEW)"
        };

        for (int t = 0; t < tabNames.Length; t++)
        {
            bool isCurrent = (interfaceTab == t);
            if (AntigravityUITheme.RibbonTab(tabNames[t], isCurrent, t == 4 ? 150f : 135f, 30f))
            {
                interfaceTab = t;
            }
        }

        GUILayout.FlexibleSpace();

        // Ribbon Collapse/Expand Toggle
        if (GUILayout.Button(ribbonExpanded ? "Thu gọn Dải lệnh" : "Mở rộng Dải lệnh", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            ribbonExpanded = !ribbonExpanded;
        }

        GUILayout.EndHorizontal();
        GUILayout.EndArea();

        // 2. DẢI CÔNG CỤ RUY BĂNG NGANG (COMMAND RIBBON STRIP)
        if (ribbonExpanded)
        {
            GUILayout.BeginArea(new Rect(12, headerHeight + 2, screenW - 24, stripHeight - 4));
            GUILayout.BeginHorizontal();

            RobotCameraController rcc = EnsureCameraController();

            if (interfaceTab == 0)
            {
                // TAB 0: TRANG CHỦ (TỔNG HỢP CÁC LỆNH CƠ BẢN)
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(300));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Home (0°)", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { robotManual?.ResetAllSlidersToZero(); }
                if (GUILayout.Button("Bích Cắm Xuống", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { robotManual?.SetFlangeDownTargetAndApplyIk(); }
                if (GUILayout.Button("Lấy Pose FK", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { robotManual?.CopyCurrentPoseToTarget(); }
                if (GUILayout.Button("Giải IK", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(36))) { robotManual?.SolveAndApplyIk(); }
                GUILayout.EndHorizontal();
                GUILayout.Label("THAO TÁC CƠ BẢN", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(240));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(isPlaying ? "TẠM DỪNG" : "CHẠY", isPlaying ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton, GUILayout.Height(36))) { isPlaying = !isPlaying; }
                if (GUILayout.Button("VỀ ĐẦU", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { isPlaying = false; MoveToWaypointGradually(0); }
                if (GUILayout.Button("RESET VỆT", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { actualTcpTrail.Clear(); if (lineRenderer) lineRenderer.positionCount = 0; }
                GUILayout.EndHorizontal();
                GUILayout.Label("VẬN HÀNH QUỸ ĐẠO", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(230));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Quần", AntigravityUITheme.StyleButton, GUILayout.Height(36))) LoadPantsOutlineTrajectory();
                if (GUILayout.Button("Tam giác", AntigravityUITheme.StyleButton, GUILayout.Height(36))) LoadTriangleTestTrajectory();
                sprayPaintEnabled = GUILayout.Toggle(sprayPaintEnabled, " Phun sơn", AntigravityUITheme.StyleToggle, GUILayout.Height(36));
                GUILayout.EndHorizontal();
                GUILayout.Label("DỮ LIỆU & PHUN SƠN", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(300));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Góc 3/4", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetPerspective(); }
                if (GUILayout.Button("Trên Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetTopDown(); }
                if (GUILayout.Button("Trước Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetFront(); }
                if (GUILayout.Button("Ngang", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetSide(); }
                if (GUILayout.Button("Focus Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.FocusGarment(); }
                GUILayout.EndHorizontal();
                GUILayout.Label("GÓC NHÌN CAMERA TRÊN ĐỒ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 1)
            {
                // TAB 1: ĐỘNG HỌC (KINEMATICS)
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(310));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Home (0°)", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { robotManual?.ResetAllSlidersToZero(); }
                if (GUILayout.Button("Bích Cắm Xuống", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { robotManual?.SetFlangeDownTargetAndApplyIk(); }
                if (GUILayout.Button("Lấy Pose FK", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { robotManual?.CopyCurrentPoseToTarget(); }
                if (GUILayout.Button("Giải IK", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(36))) { robotManual?.SolveAndApplyIk(); }
                GUILayout.EndHorizontal();
                GUILayout.Label("THAO TÁC CƠ BẢN", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(280));
                if (robotManual != null)
                {
                    robotManual.useAdaptiveTilt = GUILayout.Toggle(robotManual.useAdaptiveTilt, " Adaptive Tilt (Tiếp cận thích ứng)", AntigravityUITheme.StyleToggle, GUILayout.Height(20));
                }
                GUILayout.Label("Thuần giải tích Pieper 8 nhánh (0 lần lặp số)", AntigravityUITheme.StyleLabelMuted);
                GUILayout.Label("THIẾT LẬP BỘ GIẢI", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(270));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(showJointsPanel ? "Bảng Khớp [Mở]" : "Bảng Khớp", showJointsPanel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(36))) showJointsPanel = !showJointsPanel;
                if (GUILayout.Button(showFkMonitorPanel ? "Giám Sát [Mở]" : "Giám Sát", showFkMonitorPanel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(36))) showFkMonitorPanel = !showFkMonitorPanel;
                if (GUILayout.Button(showIkTargetPanel ? "Mục Tiêu [Mở]" : "Mục Tiêu", showIkTargetPanel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(36))) showIkTargetPanel = !showIkTargetPanel;
                GUILayout.EndHorizontal();
                GUILayout.Label("BẬT / TẮT BẢNG ĐỘNG HỌC", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 2)
            {
                // TAB 2: QUỸ ĐẠO & PHUN SƠN
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(250));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(isPlaying ? "TẠM DỪNG" : "CHẠY", isPlaying ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton, GUILayout.Height(36))) { isPlaying = !isPlaying; }
                if (GUILayout.Button("VỀ ĐẦU", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { isPlaying = false; MoveToWaypointGradually(0); }
                if (GUILayout.Button("RESET VỆT", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { actualTcpTrail.Clear(); if (lineRenderer) lineRenderer.positionCount = 0; }
                GUILayout.EndHorizontal();
                GUILayout.Label("VẬN HÀNH QUỸ ĐẠO", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(240));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Quần Jean CR10A", AntigravityUITheme.StyleButton, GUILayout.Height(36))) LoadPantsOutlineTrajectory();
                if (GUILayout.Button("Test Tam Giác", AntigravityUITheme.StyleButton, GUILayout.Height(36))) LoadTriangleTestTrajectory();
                GUILayout.EndHorizontal();
                GUILayout.Label("TẬP DỮ LIỆU ĐƯỜNG VẼ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(300));
                GUILayout.BeginHorizontal();
                sprayPaintEnabled = GUILayout.Toggle(sprayPaintEnabled, " Phun sơn", AntigravityUITheme.StyleToggle, GUILayout.Width(90), GUILayout.Height(36));
                GUILayout.BeginVertical();
                GUILayout.Label($"Tốc độ: {playbackSpeed:F1}×", AntigravityUITheme.StyleLabelCyan);
                playbackSpeed = GUILayout.HorizontalSlider(playbackSpeed, 0.1f, 1.0f);
                GUILayout.EndVertical();
                GUILayout.EndHorizontal();
                GUILayout.Label("HIỆU ỨNG & TỐC ĐỘ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(170));
                if (GUILayout.Button(showTrajectoryPanel ? "Bảng Quỹ Đạo [Mở]" : "Mở Bảng Quỹ Đạo", showTrajectoryPanel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(36))) showTrajectoryPanel = !showTrajectoryPanel;
                GUILayout.Label("BẢNG QUỸ ĐẠO", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 3)
            {
                // TAB 3: PIPELINE 3D / AI
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(620));
                GUILayout.BeginHorizontal();
                GUILayout.Box("01 Dữ liệu PLY/XYZ", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(95));
                GUILayout.Box("02 Mask AI", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(95));
                GUILayout.Box("03 Bề mặt 3D", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(95));
                GUILayout.Box("04 Súng phun", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(95));
                GUILayout.Box("05 Quỹ đạo P1/P4", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(105));
                GUILayout.Box("06 Kiểm tra CR10A", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(110));
                GUILayout.EndHorizontal();
                GUILayout.Label("LUỒNG PIPELINE TOÀN DIỆN TỪ BÁO CÁO", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(170));
                if (GUILayout.Button(showPipelinePanel ? "Bảng Pipeline [Mở]" : "Mở Bảng Pipeline", showPipelinePanel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(36))) showPipelinePanel = !showPipelinePanel;
                GUILayout.Label("BẢNG CHI TIẾT", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 4)
            {
                // TAB 4: CHỈ TIÊU & ĐÁNH GIÁ
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(600));
                GUILayout.BeginHorizontal();
                GUILayout.Box("E_geo\nHình học", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(90));
                GUILayout.Box("E_d\nKhoảng cách", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(90));
                GUILayout.Box("E_θ\nPháp tuyến", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(90));
                GUILayout.Box("E_T\nĐộ phủ", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(90));
                GUILayout.Box("v_TCP\nĐều tốc", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(90));
                GUILayout.Box("t\nThời gian", AntigravityUITheme.StyleButton, GUILayout.Height(36), GUILayout.Width(90));
                GUILayout.EndHorizontal();
                GUILayout.Label("CÁC CHỈ TIÊU KỸ THUẬT QUAN TRỌNG", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(170));
                if (GUILayout.Button(showEvaluationPanel ? "Bảng Chỉ Tiêu [Mở]" : "Mở Bảng Chỉ Tiêu", showEvaluationPanel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(36))) showEvaluationPanel = !showEvaluationPanel;
                GUILayout.Label("BẢNG ĐÁNH GIÁ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else
            {
                // TAB 5: XEM / VIEW (TRỌNG TÂM: BẬT / TẮT MỌI BẢNG PANEL)
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(520));
                GUILayout.BeginHorizontal();
                showJointsPanel = GUILayout.Toggle(showJointsPanel, " Bảng Khớp q1-q6", AntigravityUITheme.StyleToggle, GUILayout.Height(18));
                showFkMonitorPanel = GUILayout.Toggle(showFkMonitorPanel, " Giám Sát FK TCP", AntigravityUITheme.StyleToggle, GUILayout.Height(18));
                showIkTargetPanel = GUILayout.Toggle(showIkTargetPanel, " Mục Tiêu IK", AntigravityUITheme.StyleToggle, GUILayout.Height(18));
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                showTrajectoryPanel = GUILayout.Toggle(showTrajectoryPanel, " Bảng Quỹ Đạo & Sơn", AntigravityUITheme.StyleToggle, GUILayout.Height(18));
                showPipelinePanel = GUILayout.Toggle(showPipelinePanel, " Bảng Pipeline AI", AntigravityUITheme.StyleToggle, GUILayout.Height(18));
                showEvaluationPanel = GUILayout.Toggle(showEvaluationPanel, " Bảng Chỉ Tiêu", AntigravityUITheme.StyleToggle, GUILayout.Height(18));
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Hiện Tất Cả Bảng", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(22)))
                {
                    showJointsPanel = showFkMonitorPanel = showIkTargetPanel = showTrajectoryPanel = showPipelinePanel = showEvaluationPanel = true;
                }
                if (GUILayout.Button("Tắt Hết (Toàn Màn Hình 3D)", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
                {
                    showJointsPanel = showFkMonitorPanel = showIkTargetPanel = showTrajectoryPanel = showPipelinePanel = showEvaluationPanel = false;
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("BẬT / TẮT CÁC BẢNG ĐIỀU KHIỂN (PANELS)", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(280));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Góc 3/4", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.SetPresetPerspective(); }
                if (GUILayout.Button("Trên Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.SetPresetTopDown(); }
                if (GUILayout.Button("Ngang", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.SetPresetSide(); }
                if (GUILayout.Button("Trước Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.SetPresetFront(); }
                GUILayout.EndHorizontal();
                if (GUILayout.Button("Reset Focus vào Robot", AntigravityUITheme.StyleButton, GUILayout.Height(22))) { if (rcc) rcc.ResetFocus(); }
                GUILayout.Label("GÓC NHÌN CAMERA", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(200));
                sprayPaintEnabled = GUILayout.Toggle(sprayPaintEnabled, " Vệt Phun Sơn Đỏ", AntigravityUITheme.StyleToggle, GUILayout.Height(22));
                showGizmos = GUILayout.Toggle(showGizmos, " Gizmo Mục Tiêu", AntigravityUITheme.StyleToggle, GUILayout.Height(22));
                GUILayout.Label("TÙY CHỌN HIỂN THỊ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }

            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
            GUILayout.EndArea();
        }

        // 3. DOCK CHỨA CÁC BẢNG PANEL ĐƯỢC BẬT (INSPECTOR DOCK BÊN PHẢI)
        bool anyPanelOpen = showJointsPanel || showFkMonitorPanel || showIkTargetPanel ||
                            showTrajectoryPanel || showPipelinePanel || showEvaluationPanel;

        if (anyPanelOpen)
        {
            float dockW = 470f;
            float dockH = Screen.height - totalRibbonH - 16f;
            Rect dockRect = new Rect(screenW - dockW - 12f, totalRibbonH + 8f, dockW, dockH);

            GUI.DrawTexture(dockRect, AntigravityUITheme.TexBgDock);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y, 1, dockRect.height), AntigravityUITheme.TexBorder);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y, dockRect.width, 1), AntigravityUITheme.TexBorder);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y + dockRect.height - 1, dockRect.width, 1), AntigravityUITheme.TexBorder);

            GUILayout.BeginArea(new Rect(dockRect.x + 8f, dockRect.y + 6f, dockW - 16f, dockH - 12f));

            dockScrollPosition = GUILayout.BeginScrollView(dockScrollPosition, GUILayout.Width(dockW - 16f), GUILayout.Height(dockH - 12f));
            float cardWidth = dockW - 36f;

            // Card 1: Bảng Khớp q1-q6
            if (showJointsPanel && robotManual != null)
            {
                robotManual.DrawJointsControlCard(cardWidth, () => showJointsPanel = false);
                GUILayout.Space(6f);
            }

            // Card 2: Bảng Giám Sát FK TCP
            if (showFkMonitorPanel && robotManual != null)
            {
                robotManual.DrawFkMonitorCard(cardWidth, () => showFkMonitorPanel = false);
                GUILayout.Space(6f);
            }

            // Card 3: Bảng Mục Tiêu IK & 8 Nghiệm
            if (showIkTargetPanel && robotManual != null)
            {
                robotManual.DrawIkTargetCard(cardWidth, () => showIkTargetPanel = false);
                GUILayout.Space(6f);
            }

            // Card 4: Bảng Quỹ Đạo & Phun Sơn
            if (showTrajectoryPanel)
            {
                DrawTrajectoryCard(cardWidth, () => showTrajectoryPanel = false);
                GUILayout.Space(6f);
            }

            // Card 5: Bảng Pipeline 3D / AI
            if (showPipelinePanel)
            {
                DrawPipelineCard(cardWidth, () => showPipelinePanel = false);
                GUILayout.Space(6f);
            }

            // Card 6: Bảng Chỉ Tiêu Đánh Giá
            if (showEvaluationPanel)
            {
                DrawEvaluationCard(cardWidth, () => showEvaluationPanel = false);
                GUILayout.Space(6f);
            }

            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }
    }

    private void DrawTrajectoryCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("ĐIỀU KHIỂN QUỸ ĐẠO & PHUN SƠN", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18)))
        {
            onClose();
        }
        GUILayout.EndHorizontal();

        int waypoint = waypoints.Count > 0 ? Mathf.Clamp(currentWaypointIndex, 0, waypoints.Count - 1) : 0;
        float distance = waypoints.Count > 0 ? waypoints[waypoint].distance_mm : 0f;
        GUILayout.Label($"Waypoint: {waypoint} / {Mathf.Max(0, waypoints.Count - 1)}    Quãng đường: {distance:F1} mm", AntigravityUITheme.StyleLabelCyan);
        int newIdx = Mathf.RoundToInt(GUILayout.HorizontalSlider(waypoint, 0, Mathf.Max(0, waypoints.Count - 1)));
        if (newIdx != currentWaypointIndex)
        {
            isPlaying = false;
            ApplyWaypoint(newIdx);
        }

        GUILayout.Space(6);
        GUILayout.Label("TỐC ĐỘ VÀ CHUYỂN ĐỘNG", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Tốc độ tổng: {playbackSpeed:F1}×", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(130));
        playbackSpeed = GUILayout.HorizontalSlider(playbackSpeed, 0.1f, 1.0f);
        GUILayout.EndHorizontal();
        GUILayout.Label($"Trần chung: {Mathf.Clamp(maxJointSpeedDegPerSecond, 1f, 223f):F0}°/s", AntigravityUITheme.StyleLabelMuted);
        DrawAccelerationInput();

        showJointSpeedControls = GUILayout.Toggle(showJointSpeedControls, " Chỉnh tốc độ riêng từng khớp", AntigravityUITheme.StyleToggle);
        if (showJointSpeedControls)
            DrawJointSpeedInputs();

        GUILayout.Space(6);
        DrawCurrentWaypointStatus();

        GUILayout.Space(6);
        sprayPaintEnabled = GUILayout.Toggle(sprayPaintEnabled,
            sprayPaintEnabled ? " PHUN SƠN ĐỎ: BẬT" : " PHUN SƠN ĐỎ: TẮT", AntigravityUITheme.StyleToggle);

        GUILayout.EndVertical();
    }

    private void DrawPipelineCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("PIPELINE 3D / AI (THEO BÁO CÁO)", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18)))
        {
            onClose();
        }
        GUILayout.EndHorizontal();

        DrawBlankPipelineSection("01  THU NHẬN DỮ LIỆU", "Ảnh RGB  |  Point cloud XYZ  |  Zivid .ZDF / .PLY", 52);
        DrawBlankPipelineSection("02  PHÂN VÙNG VÀ MỐC AI", "Mask quần  |  vùng phun  |  mốc hình học", 52);
        DrawBlankPipelineSection("03  BỀ MẶT 2.5D / 3D", "Lọc điểm  |  mặt bàn/khuôn  |  pháp tuyến", 52);
        DrawBlankPipelineSection("04  THAM SỐ SÚNG", "Khoảng cách d  |  vệt w, σ  |  lưu lượng Q  |  tốc độ v  |  θmax", 48);
        DrawBlankPipelineSection("05  SINH QUỸ ĐẠO", "P1 Zigzag  |  P3 theo mốc  |  P4 đăng ký ICP / CPD / TPS", 56);
        DrawBlankPipelineSection("06  KIỂM TRA VÀ XUẤT LỆNH", "Trong vùng  |  tầm với / IK  |  kỳ dị  |  va chạm  |  ROS 2", 52);

        GUILayout.EndVertical();
    }

    private void DrawEvaluationCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("KẾT QUẢ ĐÁNH GIÁ & CHỈ TIÊU", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18)))
        {
            onClose();
        }
        GUILayout.EndHorizontal();

        DrawBlankMetricRow("E_geo", "Sai số hình học", "mm · TB / P95 / Hausdorff");
        DrawBlankMetricRow("E_d", "Sai số khoảng cách súng", "mm");
        DrawBlankMetricRow("E_θ", "Sai số góc so với pháp tuyến", "°");
        DrawBlankMetricRow("E_T", "Sai lệch mật độ phun", "% RMS");
        DrawBlankMetricRow("R_drop", "Độ phủ / lượng sơn tràn", "%");
        DrawBlankMetricRow("v_TCP", "Độ đều tốc độ TCP", "% dao động");
        DrawBlankMetricRow("t", "Thời gian xử lý / chu kỳ", "s");

        GUILayout.Space(6);
        GUILayout.Label("SO SÁNH PHƯƠNG PHÁP TRÊN CÙNG DỮ LIỆU", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        GUILayout.Label("Chỉ tiêu", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(110));
        GUILayout.Label("P1 Zigzag", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(80));
        GUILayout.Label("P3 Mốc", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(70));
        GUILayout.Label("P4 ICP/CPD", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(84));
        GUILayout.EndHorizontal();
        DrawBlankComparisonRow("E_geo");
        DrawBlankComparisonRow("E_d");
        DrawBlankComparisonRow("E_θ");
        DrawBlankComparisonRow("E_T");
        DrawBlankComparisonRow("R_drop");
        DrawBlankComparisonRow("v_TCP");
        DrawBlankComparisonRow("Tạo (s)");
        DrawBlankComparisonRow("Khả thi (%)");

        GUILayout.EndVertical();
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
        GUILayout.Label("TRẠNG THÁI WAYPOINT & CẤU HÌNH NGHIỆM", GUI.skin.box);
        if (waypoints.Count == 0)
        {
            GUILayout.Label("Chưa nạp quỹ đạo mô phỏng.");
            return;
        }

        int wpIdx = Mathf.Clamp(currentWaypointIndex, 0, waypoints.Count - 1);
        WaypointData wp = waypoints[wpIdx];
        GUIStyle statusStyle = new GUIStyle(GUI.skin.label)
        {
            fontStyle = FontStyle.Bold,
            normal = { textColor = wp.isSolved ? Color.green : Color.red }
        };
        GUILayout.Label(wp.isSolved ? $"IK: CÓ NGHIỆM (Waypoint #{wpIdx})" : "IK: KHÔNG CÓ NGHIỆM", statusStyle);
        GUILayout.Label($"TCP mục tiêu (mm): X {wp.targetPosGlobal_mm.x:F1}   Y {wp.targetPosGlobal_mm.y:F1}   Z {wp.targetPosGlobal_mm.z:F1}", AntigravityUITheme.StyleLabelMuted);
        GUILayout.Label($"Góc TCP RPY (°): Roll {wp.roll_deg:F1}°   Pitch {wp.pitch_deg:F1}°   Yaw {wp.yaw_deg:F1}°", AntigravityUITheme.StyleLabelMuted);
        GUILayout.Label($"Góc khớp hiện tại (°): {wp.jointAngles_deg[0]:F1}°, {wp.jointAngles_deg[1]:F1}°, {wp.jointAngles_deg[2]:F1}°, {wp.jointAngles_deg[3]:F1}°, {wp.jointAngles_deg[4]:F1}°, {wp.jointAngles_deg[5]:F1}°", AntigravityUITheme.StyleLabelCyan);

        // Giải tích toàn bộ 8 cấu hình nghiệm cho waypoint hiện tại
        Quaternion rot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
        float[] seedAngles = wpIdx > 0 ? waypoints[wpIdx - 1].jointAngles_rad : null;
        List<DobotCR10AKinematics.IKSolution> sols = DobotCR10AKinematics.SolveTcpPose(
            wp.targetPosGlobal_mm,
            rot,
            seedAngles,
            toolLength,
            useAdaptiveTilt: true
        );

        if (sols != null && sols.Count > 0)
        {
            GUILayout.Space(4);
            GUILayout.Label($"Chọn lại cấu hình tay máy ({sols.Count} cấu hình giải tích):", AntigravityUITheme.StyleSectionLabel);

            waypointSolsScroll = GUILayout.BeginScrollView(waypointSolsScroll, GUILayout.Height(Mathf.Min(140f, sols.Count * 26f + 4f)));
            for (int i = 0; i < sols.Count; i++)
            {
                DobotCR10AKinematics.IKSolution s = sols[i];
                bool isCurrentChoice = IsMatchingConfiguration(wp.jointAngles_rad, s.jointsRad);
                string pref = isCurrentChoice ? "★ [ĐANG CHỌN] " : $"#{i + 1} ";
                string label = $"{pref}{s.branchName} (Sai số: {s.positionErrorMm:F4}mm)";

                GUIStyle btnStyle = isCurrentChoice ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
                if (GUILayout.Button(label, btnStyle, GUILayout.Height(22)))
                {
                    ApplyConfigurationToWaypoint(wpIdx, s, i);
                }
            }
            GUILayout.EndScrollView();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Lan truyền cấu hình cho các điểm sau", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                PropagateCurrentConfigurationForward(wpIdx);
            }
            if (GUILayout.Button("Khôi phục mặc định Q*", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                ResetWaypointToDefault(wpIdx);
            }
            GUILayout.EndHorizontal();
        }
        else
        {
            GUILayout.Label("Không có nghiệm khả thi cho điểm này.", AntigravityUITheme.StyleLabelRed);
        }
    }

    private bool IsMatchingConfiguration(float[] a, double[] b)
    {
        if (a == null || b == null || a.Length < 6 || b.Length < 6) return false;
        for (int i = 0; i < 6; i++)
        {
            float diff = Mathf.DeltaAngle(a[i] * Mathf.Rad2Deg, (float)b[i] * Mathf.Rad2Deg);
            if (Mathf.Abs(diff) > 2.0f) return false;
        }
        return true;
    }

    public void ApplyConfigurationToWaypoint(int wpIndex, DobotCR10AKinematics.IKSolution sol, int solIndex)
    {
        if (wpIndex < 0 || wpIndex >= waypoints.Count || sol == null) return;
        WaypointData wp = waypoints[wpIndex];
        for (int j = 0; j < 6; j++)
        {
            wp.jointAngles_rad[j] = (float)sol.jointsRad[j];
            wp.jointAngles_deg[j] = (float)sol.jointsDeg[j];
        }
        wp.posError_mm = (float)sol.positionErrorMm;
        wp.isSolved = true;
        wp.selectedSolutionIndex = solIndex;
        wp.selectedBranchName = sol.branchName;

        customWaypointConfigurations[wpIndex] = solIndex;
        SaveAllSettings();

        ApplyWaypoint(wpIndex);
    }

    public void PropagateCurrentConfigurationForward(int startIdx)
    {
        if (startIdx < 0 || startIdx >= waypoints.Count) return;
        float[] seed = waypoints[startIdx].jointAngles_rad;

        for (int i = startIdx + 1; i < waypoints.Count; i++)
        {
            WaypointData wp = waypoints[i];
            Quaternion rot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
            var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, seed, toolLength, useAdaptiveTilt: true);
            if (sols != null && sols.Count > 0)
            {
                var best = sols[0]; // Nearest to seed
                for (int j = 0; j < 6; j++)
                {
                    wp.jointAngles_rad[j] = (float)best.jointsRad[j];
                    wp.jointAngles_deg[j] = (float)best.jointsDeg[j];
                }
                wp.isSolved = true;
                wp.posError_mm = (float)best.positionErrorMm;
                seed = wp.jointAngles_rad;

                customWaypointConfigurations[i] = 0;
            }
        }
        SaveAllSettings();
    }

    public void ResetWaypointToDefault(int wpIndex)
    {
        if (wpIndex < 0 || wpIndex >= waypoints.Count) return;
        customWaypointConfigurations.Remove(wpIndex);
        SaveAllSettings();

        WaypointData wp = waypoints[wpIndex];
        Quaternion rot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
        float[] seed = wpIndex > 0 ? waypoints[wpIndex - 1].jointAngles_rad : null;
        var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, seed, toolLength, useAdaptiveTilt: true);
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
            ApplyWaypoint(wpIndex);
        }
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
        if (!showGizmos) return;
        if (waypoints.Count == 0 || currentWaypointIndex >= waypoints.Count) return;

        WaypointData wp = waypoints[currentWaypointIndex];

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

    private GUIStyle CardTitleStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            fontStyle = FontStyle.Bold,
            normal = { textColor = new Color(0.2f, 0.85f, 1f) }
        };
    }

    private GUIStyle PanelHeaderStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            fontStyle = FontStyle.Bold,
            normal = { textColor = new Color(0.3f, 0.85f, 1f) }
        };
    }

    private GUIStyle CenteredMiniLabel()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 9,
            alignment = TextAnchor.MiddleCenter,
            normal = { textColor = new Color(0.65f, 0.65f, 0.65f) }
        };
    }

    private RobotCameraController EnsureCameraController()
    {
        RobotCameraController[] allRccs = FindObjectsByType<RobotCameraController>(FindObjectsInactive.Exclude);
        foreach (var r in allRccs)
        {
            if (r != null && r.GetComponent<FixedRigCamera>() == null) return r;
        }

        Camera targetCam = null;
        Camera[] allCams = FindObjectsByType<Camera>(FindObjectsInactive.Exclude);
        foreach (Camera c in allCams)
        {
            if (c.GetComponent<FixedRigCamera>() != null)
            {
                c.depth = -1;
                c.tag = "Untagged";
            }
            else if (targetCam == null && c.name == "Camera")
            {
                targetCam = c;
            }
        }

        if (targetCam == null)
        {
            foreach (Camera c in allCams)
            {
                if (c.GetComponent<FixedRigCamera>() == null) { targetCam = c; break; }
            }
        }
        if (targetCam == null) targetCam = Camera.main;

        if (targetCam != null)
        {
            targetCam.depth = 1;
            targetCam.tag = "MainCamera";
            var rcc = targetCam.GetComponent<RobotCameraController>();
            if (rcc == null) rcc = targetCam.gameObject.AddComponent<RobotCameraController>();
            rcc.InitializeIfNeeded();
            return rcc;
        }
        return null;
    }
}
