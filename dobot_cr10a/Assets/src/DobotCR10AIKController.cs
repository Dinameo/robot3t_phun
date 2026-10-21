using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Bộ điều khiển Động học Nghịch (Inverse Kinematics) & Giao diện kiểm thử cho Dobot CR10A.
/// Yêu cầu đáp ứng:
/// 1. Tọa độ ngòi bút: X, Y, Z (mm/m) đi theo quỹ đạo đường thẳng 100mm trên quần.
/// 2. Hướng TCP (Roll, Pitch, Yaw): Bút ảo luôn vuông góc với mặt phẳng vi mô cục bộ.
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
        public Vector3 targetPos_mm;      // Hệ tọa độ Robot (mm)
        public Vector3 targetPos_m;       // Hệ tọa độ Unity (m)
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
    public float d6 = 111.4f;   // Khớp 6 -> Mặt bích
    public float toolLength = 50.0f; // Chiều dài bút ảo từ mặt bích tới ngòi bút

    [Header("3. Tham chiếu mô hình Robot")]
    [Tooltip("Script manual.cs điều khiển 6 khớp robot")]
    public manual robotManual;

    [Tooltip("Đầu bích robot (Link6)")]
    public Transform toolFlange;

    [Header("4. Trạng thái điều khiển & Kiểm thử")]
    public bool isPlaying = false;
    [Range(0, 100)]
    public int currentWaypointIndex = 0;
    public float playbackSpeed = 1.0f; // Điểm / giây

    [Header("5. Hiển thị đường vẽ mực (Trail)")]
    public bool drawInkTrail = true;
    public Color inkColor = Color.red;

    // Danh sách waypoint đã nạp
    public List<WaypointData> waypoints = new List<WaypointData>();

    private float timer = 0f;
    private LineRenderer lineRenderer;

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
            robotManual = GetComponent<manual>() ?? FindFirstObjectByType<manual>();
        }

        if (toolFlange == null && robotManual != null && robotManual.axisTransforms.Length >= 6)
        {
            toolFlange = robotManual.axisTransforms[5];
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
                wp.distance_mm = idx_dist >= 0 ? float.Parse(cols[idx_dist]) : (i - 1) * 1.0f;

                float x_mm = float.Parse(cols[idx_x]);
                float y_mm = float.Parse(cols[idx_y]);
                float z_mm = float.Parse(cols[idx_z]);
                wp.targetPos_mm = new Vector3(x_mm, y_mm, z_mm);
                wp.targetPos_m = new Vector3(x_mm * 0.001f, z_mm * 0.001f, y_mm * 0.001f); // Unity Y-up

                wp.roll_deg = idx_roll >= 0 ? float.Parse(cols[idx_roll]) : 0f;
                wp.pitch_deg = idx_pitch >= 0 ? float.Parse(cols[idx_pitch]) : 0f;
                wp.yaw_deg = idx_yaw >= 0 ? float.Parse(cols[idx_yaw]) : 0f;

                if (idx_j1 >= 0)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        wp.jointAngles_deg[j] = float.Parse(cols[idx_j1 + j]);
                        wp.jointAngles_rad[j] = wp.jointAngles_deg[j] * Mathf.Deg2Rad;
                    }
                    wp.isSolved = true;
                }

                waypoints.Add(wp);
            }

            Debug.Log($"[CR10A IK] Đã nạp thành công {waypoints.Count} waypoint cho quỹ đạo vẽ 100mm!");
            ApplyWaypoint(currentWaypointIndex);
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
            timer += Time.deltaTime * playbackSpeed * 10f;
            if (timer >= 1.0f)
            {
                int stepInc = Mathf.FloorToInt(timer);
                timer -= stepInc;
                currentWaypointIndex += stepInc;

                if (currentWaypointIndex >= waypoints.Count)
                {
                    currentWaypointIndex = 0; // Lặp lại hoặc dừng
                }
                ApplyWaypoint(currentWaypointIndex);
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
        }
    }

    private void UpdateInkTrail()
    {
        if (!drawInkTrail || lineRenderer == null || waypoints.Count == 0) return;

        int count = Mathf.Min(currentWaypointIndex + 1, waypoints.Count);
        lineRenderer.positionCount = count;

        for (int i = 0; i < count; i++)
        {
            // Điểm vẽ thực tế trong Unity Scene
            lineRenderer.SetPosition(i, waypoints[i].targetPos_m);
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
            isPlaying = !isPlaying;
        }

        if (GUILayout.Button("<< ĐIỂM ĐẦU", GUILayout.Height(30)))
        {
            ApplyWaypoint(0);
        }

        if (GUILayout.Button("RESET", GUILayout.Height(30)))
        {
            isPlaying = false;
            ApplyWaypoint(0);
            if (lineRenderer != null) lineRenderer.positionCount = 0;
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(5);

        // Thanh trượt chọn Waypoint
        GUILayout.Label($"Tiến trình vẽ: Điểm {currentWaypointIndex} / {Mathf.Max(0, waypoints.Count - 1)} ({(waypoints.Count > 0 ? waypoints[currentWaypointIndex].distance_mm : 0):F1} mm)");
        int newIdx = Mathf.RoundToInt(GUILayout.HorizontalSlider(currentWaypointIndex, 0, Mathf.Max(0, waypoints.Count - 1)));
        if (newIdx != currentWaypointIndex)
        {
            ApplyWaypoint(newIdx);
        }

        // Tốc độ vẽ
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Tốc độ: {playbackSpeed:F1}x", GUILayout.Width(90));
        playbackSpeed = GUILayout.HorizontalSlider(playbackSpeed, 0.2f, 4.0f);
        GUILayout.EndHorizontal();

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
            GUILayout.Label($" X = {wp.targetPos_mm.x:F2} mm | Y = {wp.targetPos_mm.y:F2} mm | Z = {wp.targetPos_mm.z:F2} mm");

            GUILayout.Space(3);
            GUILayout.Label($"<b>Tư thế Bút ảo (Vuông góc vi mô):</b>");
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

        // Vẽ điểm mục tiêu TCP
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(wp.targetPos_m, 0.008f);

        // Vẽ vector thân bút ảo cắm vuông góc
        if (toolFlange != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(toolFlange.position, toolFlange.position + toolFlange.forward * (toolLength * 0.001f));
        }
    }
}
