using UnityEngine;
using System;
using System.Globalization;
using System.Collections.Generic;

/// <summary>
/// BẢNG ĐIỀU KHIỂN ĐỘNG HỌC ROBOT DOBOT CR10A (MANUAL FK / IK CONTROLLER)
/// Tích hợp trực tiếp thuật toán Thuần Giải Tích (Closed-Form Analytical IK) từ Báo Cáo Kỹ Thuật Dobot CR5.
/// </summary>
[ExecuteAlways]
public class manual : MonoBehaviour
{
    [Header("1. Joint Sliders: Axis 0 -> 5 (Khớp q1 -> q6, Radian & Độ)")]
    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay khớp 1 / Axis 0 (radian)")]
    public float axis0 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay khớp 2 / Axis 1 (radian)")]
    public float axis1 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay khớp 3 / Axis 2 (radian)")]
    public float axis2 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay khớp 4 / Axis 3 (radian)")]
    public float axis3 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay khớp 5 / Axis 4 (radian)")]
    public float axis4 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay khớp 6 / Axis 5 (radian)")]
    public float axis5 = 0f;

    [Header("2. Liên kết Transforms 6 khớp (Axis0 -> Axis5)")]
    [Tooltip("Mảng chứa 6 Transform của các khớp")]
    public Transform[] axisTransforms = new Transform[6];

    [Header("3. Offset góc ban đầu để bảo toàn cấu hình prefab khi slider = 0")]
    [SerializeField]
    private Quaternion[] initialRotations = new Quaternion[6];

    [SerializeField]
    private bool isInitialized = false;

    [Header("4. Cấu hình quy tắc chiều xoay")]
    [Tooltip("Đảo ngược chiều xoay để chuẩn theo quy tắc bàn tay phải (Robotics chuẩn)")]
    public bool invertDirection = true;

    [Header("5. Cấu hình công cụ TCP")]
    [Tooltip("Chiều dài công cụ TCP kẹp dọc theo trục Z mặt bích (mm)")]
    public float toolLengthMm = 100.0f;

    [Tooltip("Bật tính năng góc tiếp cận thích ứng (Adaptive Approach Angle) khi với xa")]
    public bool useAdaptiveTilt = false;

    // Các biến nội bộ cho Giao diện OnGUI
    private readonly float[] targetPose = new float[6]; // X, Y, Z (mm), Roll, Pitch, Yaw (deg)
    private readonly string[] targetText = new string[6];
    private bool targetInitialized = false;
    private string ikStatus = "Nhập tọa độ mục tiêu hoặc bấm 'Lấy Pose FK Hiện Tại'.";
    private readonly List<DobotCR10AKinematics.IKSolution> ikSolutions = new List<DobotCR10AKinematics.IKSolution>();
    private Vector2 solutionScroll = Vector2.zero;
    private Transform globalReference;
    private Transform tcpTransform;

    private void Awake()
    {
        EnsureOffsetsInitialized();
    }

    private void Start()
    {
        EnsureOffsetsInitialized();
        InitializeTargetFromCurrentPose();
    }

    private void Update()
    {
        ApplyRotations();
    }

    private void OnValidate()
    {
        if (!isInitialized)
        {
            EnsureOffsetsInitialized();
        }
        ApplyRotations();
    }

    /// <summary>
    /// Đảm bảo offset ban đầu được lưu đúng để khi slider = 0 thì robot ở tư thế gốc chuẩn
    /// </summary>
    private void EnsureOffsetsInitialized()
    {
        AutoAssignTransforms();
        if (isInitialized && initialRotations != null && initialRotations.Length == 6)
            return;

        InitializeOffsets();
    }

    [ContextMenu("Ghi nhớ góc ban đầu (Capture Zero Offset)")]
    public void InitializeOffsets()
    {
        if (axisTransforms == null || axisTransforms.Length != 6)
        {
            axisTransforms = new Transform[6];
        }

        AutoAssignTransforms();

        if (initialRotations == null || initialRotations.Length != 6)
        {
            initialRotations = new Quaternion[6];
        }

        for (int i = 0; i < 6; i++)
        {
            if (axisTransforms[i] != null)
            {
                initialRotations[i] = axisTransforms[i].localRotation;
            }
        }

        isInitialized = true;
    }

    /// <summary>
    /// Tự động tìm kiếm các Transform theo tên axis0..5 hoặc Link1..6 nếu chưa được gán
    /// </summary>
    private void AutoAssignTransforms()
    {
        for (int i = 0; i < 6; i++)
        {
            if (axisTransforms[i] == null)
            {
                string[] possibleNames = new string[] {
                    $"axis{i}", $"Axis{i}", $"axis_{i}",
                    $"Link{i + 1}", $"link{i + 1}", $"L{i + 1}",
                    $"Joint{i + 1}", $"joint{i + 1}"
                };

                foreach (string n in possibleNames)
                {
                    Transform t = FindDeepChild(transform, n);
                    if (t == null)
                    {
                        GameObject go = GameObject.Find(n);
                        if (go != null) t = go.transform;
                    }

                    if (t != null)
                    {
                        axisTransforms[i] = t;
                        break;
                    }
                }
            }
        }
    }

    private Transform FindDeepChild(Transform parent, string childName)
    {
        if (parent == null) return null;
        if (parent.name.Equals(childName, StringComparison.OrdinalIgnoreCase)) return parent;

        foreach (Transform child in parent)
        {
            Transform result = FindDeepChild(child, childName);
            if (result != null) return result;
        }
        return null;
    }

    private void ResolveKinematicsReferences()
    {
        if (globalReference == null && axisTransforms != null && axisTransforms.Length > 0 && axisTransforms[0] != null)
            globalReference = axisTransforms[0].parent;

        if (globalReference == null)
        {
            GameObject globalObject = GameObject.Find("Global");
            if (globalObject != null)
                globalReference = globalObject.transform;
        }

        if (tcpTransform == null && axisTransforms != null && axisTransforms.Length >= 6 && axisTransforms[5] != null)
            tcpTransform = FindDeepChild(axisTransforms[5], "tcp");
    }

    /// <summary>
    /// Áp dụng xoay quanh trục Z cục bộ kết hợp với offset góc ban đầu
    /// </summary>
    public void ApplyRotations()
    {
        if (axisTransforms == null || axisTransforms.Length != 6) return;
        if (initialRotations == null || initialRotations.Length != 6) return;

        float[] sliders = new float[6] { axis0, axis1, axis2, axis3, axis4, axis5 };
        float dirMultiplier = invertDirection ? -1f : 1f;

        for (int i = 0; i < 6; i++)
        {
            if (axisTransforms[i] != null)
            {
                float angleDeg = dirMultiplier * sliders[i] * Mathf.Rad2Deg;
                axisTransforms[i].localRotation = initialRotations[i] * Quaternion.Euler(0f, 0f, angleDeg);
            }
        }
    }

    /// <summary>
    /// Reset tất cả các slider về 0
    /// </summary>
    [ContextMenu("Reset All Sliders To 0")]
    public void ResetAllSlidersToZero()
    {
        axis0 = 0f;
        axis1 = 0f;
        axis2 = 0f;
        axis3 = 0f;
        axis4 = 0f;
        axis5 = 0f;
        ApplyRotations();
        ikStatus = "Đã đưa toàn bộ 6 khớp về cấu hình Home (0°).";
    }

    // =========================================================================
    // GIAO DIỆN TRỰC QUAN KIỂM THỬ TRÊN MÀN HÌNH GAME (Runtime OnGUI Panel)
    // =========================================================================
    private void OnGUI()
    {
        if (axisTransforms == null || axisTransforms.Length != 6) return;
        if (!targetInitialized) InitializeTargetFromCurrentPose();
        ResolveKinematicsReferences();

        const float panelWidth = 430f;
        float panelHeight = Mathf.Min(Screen.height - 24f, 740f);
        Rect panelRect = new Rect(12f, 12f, panelWidth, panelHeight);

        GUI.Box(panelRect, GUIContent.none);
        GUILayout.BeginArea(new Rect(panelRect.x + 10f, panelRect.y + 8f, panelWidth - 20f, panelHeight - 16f));
        GUILayout.BeginVertical();

        // 1. Tiêu đề
        GUILayout.Label("DOBOT CR10A — KINEMATICS ENGINE", PanelTitleStyle());
        GUILayout.Label("Thuần Giải Tích (Closed-Form Analytical IK) • 8 Nghiệm", SubtitleStyle());
        GUILayout.Space(4f);

        // 2. 6 Sliders khớp góc
        GUILayout.Label("Điều khiển 6 khớp q1…q6 (Độ & Radian):", PanelHeaderStyle());
        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"q{i + 1}: {GetJoint(i) * Mathf.Rad2Deg,6:F1}°", GUILayout.Width(80f));
            float val = GUILayout.HorizontalSlider(GetJoint(i), -2f * Mathf.PI, 2f * Mathf.PI);
            if (Mathf.Abs(val - GetJoint(i)) > 1e-5f)
            {
                SetJoint(i, val);
                ApplyRotations();
            }
            if (GUILayout.Button("0", GUILayout.Width(22f), GUILayout.Height(18f)))
            {
                SetJoint(i, 0f);
                ApplyRotations();
            }
            GUILayout.EndHorizontal();
        }

        GUILayout.Space(6f);

        // 3. Động học thuận FK hiện tại
        Vector3 currentTcpPosMm;
        Vector3 currentRpyDeg;
        GetForwardPoseValues(out currentTcpPosMm, out currentRpyDeg);

        GUILayout.Label("Động học thuận FK hiện tại (TCP):", PanelHeaderStyle());
        GUILayout.Label($"Vị trí (mm): X {currentTcpPosMm.x,7:F1}   Y {currentTcpPosMm.y,7:F1}   Z {currentTcpPosMm.z,7:F1}");
        GUILayout.Label($"Góc RPY (°): Roll {currentRpyDeg.x,6:F1}°  Pitch {currentRpyDeg.y,6:F1}°  Yaw {currentRpyDeg.z,6:F1}°");

        // Kiểm tra sai số thực tế giữa FK tính toán và GameObject tcp trong Scene
        Vector3 actualSceneTcpPosMm;
        if (TryGetActualSceneTcpPos(out actualSceneTcpPosMm))
        {
            float errorMm = Vector3.Distance(currentTcpPosMm, actualSceneTcpPosMm);
            GUILayout.Label($"Sai số toán học FK ↔ Mesh Scene: {errorMm:F4} mm", ErrorStyle(errorMm));
        }

        GUILayout.Space(6f);

        // 4. Nhập Pose mục tiêu cho IK
        GUILayout.Label("Pose mục tiêu (XYZ: mm • RPY: độ ZYX):", PanelHeaderStyle());
        DrawPoseRow(0, 1, 2);
        DrawPoseRow(3, 4, 5);

        GUILayout.BeginHorizontal();
        useAdaptiveTilt = GUILayout.Toggle(useAdaptiveTilt, " Adaptive Tilt (Gắp xa)");
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Lấy Pose FK Hiện Tại", GUILayout.Height(26f)))
        {
            CopyCurrentPoseToTarget();
        }
        if (GUILayout.Button("Giải IK (8 Cấu Hình)", GUILayout.Height(26f)))
        {
            SolveAndApplyIk();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Mặt bích hướng xuống (Flange Down)", GUILayout.Height(24f)))
        {
            SetFlangeDownTargetAndApplyIk();
        }
        if (GUILayout.Button("Reset về Home (0°)", GUILayout.Height(24f)))
        {
            ResetAllSlidersToZero();
        }
        GUILayout.EndHorizontal();

        GUILayout.Label(ikStatus, PanelStatusStyle());

        // 5. Hiển thị danh sách 8 nghiệm giải tích
        if (ikSolutions.Count > 0)
        {
            GUILayout.Label($"Danh sách nghiệm giải tích tìm được ({ikSolutions.Count} cấu hình):", PanelHeaderStyle());
            solutionScroll = GUILayout.BeginScrollView(solutionScroll, GUILayout.Height(Mathf.Min(140f, 28f * ikSolutions.Count)));
            for (int i = 0; i < ikSolutions.Count; i++)
            {
                DobotCR10AKinematics.IKSolution sol = ikSolutions[i];
                string prefTag = i == 0 ? "★ [TỐI ƯU Q*] " : $"#{i + 1} ";
                string label = $"{prefTag}{sol.branchName} (Sai số: {sol.positionErrorMm:F4}mm)";
                
                GUI.color = i == 0 ? Color.green : Color.white;
                if (GUILayout.Button(label, GUILayout.Height(22f)))
                {
                    ApplySolution(i);
                }
                GUI.color = Color.white;
            }
            GUILayout.EndScrollView();
        }

        GUILayout.EndVertical();
        GUILayout.EndArea();
    }

    private void DrawPoseRow(int a, int b, int c)
    {
        string[] labels = { "X", "Y", "Z", "Roll", "Pitch", "Yaw" };
        int[] indices = { a, b, c };
        GUILayout.BeginHorizontal();
        for (int k = 0; k < 3; k++)
        {
            int i = indices[k];
            GUILayout.Label(labels[i], GUILayout.Width(36f));
            targetText[i] = GUILayout.TextField(targetText[i] ?? "0", GUILayout.Width(85f));
            float parsed;
            if (float.TryParse(targetText[i], NumberStyles.Float, CultureInfo.InvariantCulture, out parsed) ||
                float.TryParse(targetText[i], NumberStyles.Float, CultureInfo.CurrentCulture, out parsed))
            {
                targetPose[i] = parsed;
            }
        }
        GUILayout.EndHorizontal();
    }

    private void InitializeTargetFromCurrentPose()
    {
        if (targetInitialized) return;
        CopyCurrentPoseToTarget();
        targetInitialized = true;
    }

    private void CopyCurrentPoseToTarget()
    {
        Vector3 posMm;
        Vector3 rpyDeg;
        GetForwardPoseValues(out posMm, out rpyDeg);

        targetPose[0] = posMm.x;
        targetPose[1] = posMm.y;
        targetPose[2] = posMm.z;
        targetPose[3] = rpyDeg.x;
        targetPose[4] = rpyDeg.y;
        targetPose[5] = rpyDeg.z;

        for (int i = 0; i < 6; i++)
        {
            targetText[i] = targetPose[i].ToString("F1", CultureInfo.InvariantCulture);
        }
        ikStatus = "Đã sao chép Pose FK hiện tại vào mục tiêu.";
    }

    private void GetForwardPoseValues(out Vector3 posMm, out Vector3 rpyDeg)
    {
        float[] q = GetJointVector();
        Matrix4x4 T = DobotCR10AKinematics.ForwardTcp(q, tcpTransform, toolLengthMm);
        posMm = new Vector3(T[0, 3] * 1000f, T[1, 3] * 1000f, T[2, 3] * 1000f);
        rpyDeg = DobotCR10AKinematics.UnityRotationToRpy(T.rotation);
    }

    private bool TryGetActualSceneTcpPos(out Vector3 posMm)
    {
        ResolveKinematicsReferences();
        if (globalReference == null || tcpTransform == null)
        {
            posMm = Vector3.zero;
            return false;
        }

        // Tọa độ của tcpTransform quy về hệ quy chiếu robot base (Global)
        Vector3 localPosM = globalReference.InverseTransformPoint(tcpTransform.position);
        posMm = localPosM * 1000f;
        return true;
    }

    private void SolveAndApplyIk()
    {
        Vector3 targetPos = new Vector3(targetPose[0], targetPose[1], targetPose[2]);
        Quaternion targetRot = DobotCR10AKinematics.RpyToUnityRotation(targetPose[3], targetPose[4], targetPose[5]);

        float[] currentQ = GetJointVector();

        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
        ikSolutions.Clear();
        List<DobotCR10AKinematics.IKSolution> sols = DobotCR10AKinematics.SolveTcpPose(
            targetPos,
            targetRot,
            currentQ,
            toolLengthMm,
            useAdaptiveTilt
        );
        sw.Stop();

        if (sols == null || sols.Count == 0)
        {
            ikStatus = "IK: Không tìm thấy nghiệm thực (Mục tiêu nằm ngoài vùng với hoặc trong vùng chết vai).";
            return;
        }

        ikSolutions.AddRange(sols);

        // Tự động áp dụng nghiệm tối ưu Q* (ở vị trí đầu tiên sols[0])
        ApplySolution(0);

        ikStatus = string.Format(CultureInfo.InvariantCulture,
            "IK THÀNH CÔNG: Tìm thấy {0} cấu hình giải tích trong {1:F3} ms. Đã áp dụng nghiệm tối ưu Q* ({2}). Sai số vị trí: {3:E2} mm.",
            sols.Count, sw.Elapsed.TotalMilliseconds, sols[0].branchName, sols[0].positionErrorMm);
    }

    private void SetFlangeDownTargetAndApplyIk()
    {
        // Giữ nguyên vị trí X, Y, Z hiện tại và Yaw hiện tại, ép Roll = 180°, Pitch = 0° để mặt bích kẹp cắm thẳng đứng xuống sàn
        targetPose[3] = 180f; // Roll
        targetPose[4] = 0f;   // Pitch
        targetText[3] = "180.0";
        targetText[4] = "0.0";

        SolveAndApplyIk();
    }

    private void ApplySolution(int index)
    {
        if (index < 0 || index >= ikSolutions.Count) return;
        DobotCR10AKinematics.IKSolution sol = ikSolutions[index];

        for (int i = 0; i < 6; i++)
        {
            SetJoint(i, (float)sol.jointsRad[i]);
        }

        ApplyRotations();

        ikStatus = string.Format(CultureInfo.InvariantCulture,
            "Đã áp dụng nghiệm #{0} [{1}]. Sai số FK: {2:F4} mm.",
            index + 1, sol.branchName, sol.positionErrorMm);
    }

    private float GetJoint(int i)
    {
        switch (i)
        {
            case 0: return axis0;
            case 1: return axis1;
            case 2: return axis2;
            case 3: return axis3;
            case 4: return axis4;
            default: return axis5;
        }
    }

    private void SetJoint(int i, float value)
    {
        value = Mathf.Clamp(value, -2f * Mathf.PI, 2f * Mathf.PI);
        switch (i)
        {
            case 0: axis0 = value; break;
            case 1: axis1 = value; break;
            case 2: axis2 = value; break;
            case 3: axis3 = value; break;
            case 4: axis4 = value; break;
            default: axis5 = value; break;
        }
    }

    private float[] GetJointVector()
    {
        return new float[] { axis0, axis1, axis2, axis3, axis4, axis5 };
    }

    // Styles cho OnGUI
    private GUIStyle PanelTitleStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 15,
            fontStyle = FontStyle.Bold,
            normal = { textColor = new Color(0.2f, 0.9f, 1f) }
        };
    }

    private GUIStyle SubtitleStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            normal = { textColor = new Color(0.8f, 0.8f, 0.8f) }
        };
    }

    private GUIStyle PanelHeaderStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.yellow }
        };
    }

    private GUIStyle PanelStatusStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            wordWrap = true,
            fontSize = 11,
            normal = { textColor = ikStatus.Contains("THÀNH CÔNG") ? Color.green : (ikStatus.Contains("Không tìm thấy") ? Color.red : Color.white) }
        };
    }

    private GUIStyle ErrorStyle(float errorMm)
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            normal = { textColor = errorMm < 0.01f ? Color.green : Color.yellow }
        };
    }
}
