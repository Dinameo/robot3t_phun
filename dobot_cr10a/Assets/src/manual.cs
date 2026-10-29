using UnityEngine;
using System;
using System.Globalization;
using System.Collections.Generic;

/// <summary>
/// BẢNG ĐIỀU KHIỂN ĐỘNG HỌC ROBOT DOBOT CR10A (MANUAL FK / IK CONTROLLER)
/// Giao diện phong cách Windows Desktop App với dải ruy băng (Ribbon Tabs).
/// Các bảng điều khiển (Panels) có thể được bật/tắt linh hoạt trong Tab VIEW.
/// </summary>
[ExecuteAlways]
public class manual : MonoBehaviour
{
    [Header("1. Joint Sliders: Axis 0 -> 5 (Khớp q1 -> q6, Radian & Độ)")]
    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    public float axis0 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    public float axis1 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    public float axis2 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    public float axis3 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    public float axis4 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    public float axis5 = 0f;

    [Header("2. Liên kết Transforms 6 khớp (Axis0 -> Axis5)")]
    public Transform[] axisTransforms = new Transform[6];

    [Header("3. Offset góc ban đầu để bảo toàn cấu hình prefab khi slider = 0")]
    [SerializeField]
    public Quaternion[] initialRotations = new Quaternion[6];

    [SerializeField]
    public bool isInitialized = false;

    [Header("4. Cấu hình quy tắc chiều xoay")]
    public bool invertDirection = true;

    [Header("5. Cấu hình công cụ TCP")]
    public float toolLengthMm = 100.0f;
    public bool useAdaptiveTilt = false;

    // Các biến nội bộ cho Giao diện OnGUI
    public readonly float[] targetPose = new float[6]; // X, Y, Z (mm), Roll, Pitch, Yaw (deg)
    public readonly string[] targetText = new string[6];
    public bool targetInitialized = false;
    public string ikStatus = "Nhập tọa độ mục tiêu hoặc bấm 'Lấy Pose FK Hiện Tại'.";
    public readonly List<DobotCR10AKinematics.IKSolution> ikSolutions = new List<DobotCR10AKinematics.IKSolution>();
    public Vector2 solutionScroll = Vector2.zero;
    public Vector2 inspectorScroll = Vector2.zero;
    public Transform globalReference;
    public Transform tcpTransform;

    // Các cờ BẬT / TẮT Panel trong Tab View
    [Header("6. Quản lý hiển thị Panels (Tab View)")]
    public bool showJointsPanel = true;
    public bool showFkMonitorPanel = true;
    public bool showIkTargetPanel = true;

    // Quản lý Ribbon khi chạy standalone
    public int ribbonTab = 0; // 0: Trang chủ, 1: Động học, 2: Xem (View)
    public bool ribbonExpanded = true;
    private DobotCR10AIKController ikController;

    private void Awake()
    {
        EnsureOffsetsInitialized();
        ikController = GetComponent<DobotCR10AIKController>();
        LoadSettings();
    }

    private void OnDisable()
    {
        SaveSettings();
    }

    private void OnDestroy()
    {
        SaveSettings();
    }

    public void SaveSettings()
    {
        PlayerPrefs.SetInt("Manual_ShowJoints", showJointsPanel ? 1 : 0);
        PlayerPrefs.SetInt("Manual_ShowFk", showFkMonitorPanel ? 1 : 0);
        PlayerPrefs.SetInt("Manual_ShowIk", showIkTargetPanel ? 1 : 0);
        PlayerPrefs.SetInt("Manual_RibbonExpanded", ribbonExpanded ? 1 : 0);
        PlayerPrefs.SetInt("Manual_RibbonTab", ribbonTab);
        PlayerPrefs.SetInt("Manual_AdaptiveTilt", useAdaptiveTilt ? 1 : 0);
        PlayerPrefs.Save();
    }

    public void LoadSettings()
    {
        if (PlayerPrefs.HasKey("Manual_ShowJoints"))
        {
            showJointsPanel = PlayerPrefs.GetInt("Manual_ShowJoints", 1) == 1;
            showFkMonitorPanel = PlayerPrefs.GetInt("Manual_ShowFk", 1) == 1;
            showIkTargetPanel = PlayerPrefs.GetInt("Manual_ShowIk", 1) == 1;
            ribbonExpanded = PlayerPrefs.GetInt("Manual_RibbonExpanded", 1) == 1;
            ribbonTab = PlayerPrefs.GetInt("Manual_RibbonTab", 0);
            useAdaptiveTilt = PlayerPrefs.GetInt("Manual_AdaptiveTilt", 0) == 1;
        }
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

    public void EnsureOffsetsInitialized()
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

    public void ResolveKinematicsReferences()
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
    // CÁC HÀM VẼ TỪNG BẢNG CHI TIẾT (PANEL CARDS) ĐỘC LẬP
    // =========================================================================

    /// <summary>
    /// Bảng 1: Điều khiển 6 khớp góc (Sliders + Vi chỉnh)
    /// </summary>
    public void DrawJointsControlCard(float width, Action onClose = null)
    {
        AntigravityUITheme.EnsureInitialized();
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("ĐIỀU KHIỂN 6 KHỚP (SLIDERS)", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18)))
        {
            onClose();
        }
        GUILayout.EndHorizontal();

        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"q{i + 1}", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(24));

            float degVal = GetJoint(i) * Mathf.Rad2Deg;
            GUILayout.Label($"{degVal,6:F1}°", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(58));

            if (GUILayout.Button("-5°", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20)))
            {
                SetJoint(i, (degVal - 5f) * Mathf.Deg2Rad);
                ApplyRotations();
            }
            if (GUILayout.Button("-1°", AntigravityUITheme.StyleButton, GUILayout.Width(28), GUILayout.Height(20)))
            {
                SetJoint(i, (degVal - 1f) * Mathf.Deg2Rad);
                ApplyRotations();
            }

            float sliderVal = GUILayout.HorizontalSlider(GetJoint(i), -2f * Mathf.PI, 2f * Mathf.PI);
            if (Mathf.Abs(sliderVal - GetJoint(i)) > 1e-5f)
            {
                SetJoint(i, sliderVal);
                ApplyRotations();
            }

            if (GUILayout.Button("+1°", AntigravityUITheme.StyleButton, GUILayout.Width(28), GUILayout.Height(20)))
            {
                SetJoint(i, (degVal + 1f) * Mathf.Deg2Rad);
                ApplyRotations();
            }
            if (GUILayout.Button("+5°", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20)))
            {
                SetJoint(i, (degVal + 5f) * Mathf.Deg2Rad);
                ApplyRotations();
            }
            if (GUILayout.Button("0", AntigravityUITheme.StyleButton, GUILayout.Width(22), GUILayout.Height(20)))
            {
                SetJoint(i, 0f);
                ApplyRotations();
            }
            GUILayout.EndHorizontal();
        }

        GUILayout.EndVertical();
    }

    /// <summary>
    /// Bảng 2: Giám sát FK và sai số thực tế thời gian thực
    /// </summary>
    public void DrawFkMonitorCard(float width, Action onClose = null)
    {
        AntigravityUITheme.EnsureInitialized();
        Vector3 currentTcpPosMm;
        Vector3 currentRpyDeg;
        GetForwardPoseValues(out currentTcpPosMm, out currentRpyDeg);

        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("GIÁM SÁT FK TCP THỜI GIAN THỰC", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18)))
        {
            onClose();
        }
        GUILayout.EndHorizontal();

        GUILayout.Label($"Vị trí (mm):   X: {currentTcpPosMm.x,7:F1}   Y: {currentTcpPosMm.y,7:F1}   Z: {currentTcpPosMm.z,7:F1}", AntigravityUITheme.StyleLabelMuted);
        GUILayout.Label($"Góc RPY (°):   Roll: {currentRpyDeg.x,6:F1}°  Pitch: {currentRpyDeg.y,6:F1}°  Yaw: {currentRpyDeg.z,6:F1}°", AntigravityUITheme.StyleLabelMuted);

        Vector3 actualSceneTcpPosMm;
        if (TryGetActualSceneTcpPos(out actualSceneTcpPosMm))
        {
            float errorMm = Vector3.Distance(currentTcpPosMm, actualSceneTcpPosMm);
            GUIStyle errStyle = errorMm < 0.1f ? AntigravityUITheme.StyleLabelGreen : (errorMm < 1.0f ? AntigravityUITheme.StyleLabelYellow : AntigravityUITheme.StyleLabelRed);
            GUILayout.Label($"Sai số toán học FK ↔ Mesh: {errorMm:F4} mm", errStyle);
        }

        GUILayout.EndVertical();
    }

    /// <summary>
    /// Bảng 3: Nhập Pose mục tiêu và chọn 8 nhánh nghiệm giải tích
    /// </summary>
    public void DrawIkTargetCard(float width, Action onClose = null)
    {
        AntigravityUITheme.EnsureInitialized();
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("MỤC TIÊU IK & 8 NGHIỆM GIẢI TÍCH", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18)))
        {
            onClose();
        }
        GUILayout.EndHorizontal();

        DrawPoseRow(0, 1, 2);
        DrawPoseRow(3, 4, 5);

        GUILayout.BeginHorizontal();
        useAdaptiveTilt = GUILayout.Toggle(useAdaptiveTilt, " Adaptive Tilt (Tiếp cận thích ứng)", AntigravityUITheme.StyleToggle);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Lấy Pose FK", AntigravityUITheme.StyleButton, GUILayout.Height(24))) CopyCurrentPoseToTarget();
        if (GUILayout.Button("Giải IK", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(24))) SolveAndApplyIk();
        if (GUILayout.Button("Bích Cắm Xuống", AntigravityUITheme.StyleButton, GUILayout.Height(24))) SetFlangeDownTargetAndApplyIk();
        if (GUILayout.Button("Home", AntigravityUITheme.StyleButton, GUILayout.Height(24))) ResetAllSlidersToZero();
        GUILayout.EndHorizontal();

        GUILayout.Label(ikStatus, AntigravityUITheme.StyleLabelCyan);

        // Danh sách 8 nghiệm giải tích
        if (ikSolutions.Count > 0)
        {
            GUILayout.Space(2f);
            GUILayout.Label($"Danh sách nghiệm giải tích ({ikSolutions.Count} cấu hình):", AntigravityUITheme.StyleSectionLabel);
            solutionScroll = GUILayout.BeginScrollView(solutionScroll, GUILayout.Height(Mathf.Min(140f, 28f * ikSolutions.Count)));
            for (int i = 0; i < ikSolutions.Count; i++)
            {
                DobotCR10AKinematics.IKSolution sol = ikSolutions[i];
                string prefTag = i == 0 ? "[TỐI ƯU Q*] " : $"#{i + 1} ";
                string label = $"{prefTag}{sol.branchName} (Sai số: {sol.positionErrorMm:F4}mm)";

                GUIStyle btnSt = i == 0 ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
                if (GUILayout.Button(label, btnSt, GUILayout.Height(22)))
                {
                    ApplySolution(i);
                }
            }
            GUILayout.EndScrollView();
        }

        GUILayout.EndVertical();
    }

    private void DrawPoseRow(int a, int b, int c)
    {
        string[] labels = { "X", "Y", "Z", "Roll", "Pitch", "Yaw" };
        int[] indices = { a, b, c };
        GUILayout.BeginHorizontal();
        for (int k = 0; k < 3; k++)
        {
            int i = indices[k];
            GUILayout.Label(labels[i], AntigravityUITheme.StyleLabelCyan, GUILayout.Width(30f));
            targetText[i] = GUILayout.TextField(targetText[i] ?? "0", AntigravityUITheme.StyleTextField, GUILayout.Width(92f));
            float parsed;
            if (float.TryParse(targetText[i], NumberStyles.Float, CultureInfo.InvariantCulture, out parsed) ||
                float.TryParse(targetText[i], NumberStyles.Float, CultureInfo.CurrentCulture, out parsed))
            {
                targetPose[i] = parsed;
            }
        }
        GUILayout.EndHorizontal();
    }

    // =========================================================================
    // GIAO DIỆN ONGUI (STANDALONE RIBBON NẾU CHẠY MỘT MÌNH)
    // =========================================================================

    private void OnGUI()
    {
        if (axisTransforms == null || axisTransforms.Length != 6) return;
        if (!targetInitialized) InitializeTargetFromCurrentPose();
        ResolveKinematicsReferences();

        // Nhường quyền cho DobotCR10AIKController nếu có trên cùng GameObject
        if (ikController == null) ikController = GetComponent<DobotCR10AIKController>();
        if (ikController != null && ikController.enabled)
            return;

        AntigravityUITheme.EnsureInitialized();
        float screenW = Screen.width;
        float headerHeight = 38f;
        float stripHeight = ribbonExpanded ? 76f : 0f;
        float totalRibbonH = headerHeight + stripHeight;

        // 1. THANH RUY BĂNG TRÊN CÙNG
        GUI.DrawTexture(new Rect(0, 0, screenW, totalRibbonH), AntigravityUITheme.TexBgHeader);
        GUI.DrawTexture(new Rect(0, totalRibbonH - 1, screenW, 1), AntigravityUITheme.TexBorder);
        if (ribbonExpanded)
        {
            GUI.DrawTexture(new Rect(0, headerHeight, screenW, stripHeight), AntigravityUITheme.TexBgStrip);
            GUI.DrawTexture(new Rect(0, headerHeight - 1, screenW, 1), AntigravityUITheme.TexBorder);
        }

        GUILayout.BeginArea(new Rect(12, 3, screenW - 24, headerHeight));
        GUILayout.BeginHorizontal();

        GUILayout.Label("DOBOT CR10A WORKBENCH", AntigravityUITheme.StyleHeaderTitle, GUILayout.Width(240));

        // Các Tab Ruy Băng
        string[] tabs = { "TRANG CHỦ", "ĐỘNG HỌC", "XEM (VIEW)" };
        for (int t = 0; t < tabs.Length; t++)
        {
            bool isCurrent = (ribbonTab == t);
            if (AntigravityUITheme.RibbonTab(tabs[t], isCurrent, 130f, 30f))
            {
                ribbonTab = t;
            }
        }

        GUILayout.FlexibleSpace();

        if (GUILayout.Button(ribbonExpanded ? "Thu gọn Dải lệnh" : "Mở rộng Dải lệnh", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            ribbonExpanded = !ribbonExpanded;
        }

        GUILayout.EndHorizontal();
        GUILayout.EndArea();

        // 2. DẢI CÔNG CỤ DƯỚI TAB (COMMAND STRIP)
        if (ribbonExpanded)
        {
            GUILayout.BeginArea(new Rect(12, headerHeight + 2, screenW - 24, stripHeight - 4));
            GUILayout.BeginHorizontal();

            if (ribbonTab == 0 || ribbonTab == 1)
            {
                // Nhóm: THAO TÁC CƠ BẢN
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(340));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Home (0°)", AntigravityUITheme.StyleButton, GUILayout.Height(36))) ResetAllSlidersToZero();
                if (GUILayout.Button("Bích Cắm Xuống", AntigravityUITheme.StyleButton, GUILayout.Height(36))) SetFlangeDownTargetAndApplyIk();
                if (GUILayout.Button("Lấy Pose FK", AntigravityUITheme.StyleButton, GUILayout.Height(36))) CopyCurrentPoseToTarget();
                if (GUILayout.Button("Giải IK", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(36))) SolveAndApplyIk();
                GUILayout.EndHorizontal();
                GUILayout.Label("THAO TÁC CƠ BẢN", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                // Nhóm: CAMERA
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(330));
                GUILayout.BeginHorizontal();
                RobotCameraController rcc = EnsureCameraController();
                if (GUILayout.Button("Góc 3/4", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetPerspective(); }
                if (GUILayout.Button("Trên Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetTopDown(); }
                if (GUILayout.Button("Trước Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetFront(); }
                if (GUILayout.Button("Ngang", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetSide(); }
                if (GUILayout.Button("Focus Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.FocusGarment(); }
                GUILayout.EndHorizontal();
                GUILayout.Label("GÓC NHÌN CAMERA TRÊN ĐỒ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else
            {
                // TAB 2: XEM (VIEW) - BẬT TẮT CÁC BẢNG PANEL
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(460));
                GUILayout.BeginHorizontal();
                showJointsPanel = GUILayout.Toggle(showJointsPanel, " Bảng Khớp q1-q6", AntigravityUITheme.StyleToggle);
                showFkMonitorPanel = GUILayout.Toggle(showFkMonitorPanel, " Giám Sát FK TCP", AntigravityUITheme.StyleToggle);
                showIkTargetPanel = GUILayout.Toggle(showIkTargetPanel, " Mục Tiêu IK", AntigravityUITheme.StyleToggle);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Hiện Tất Cả Bảng", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(20)))
                {
                    showJointsPanel = showFkMonitorPanel = showIkTargetPanel = true;
                }
                if (GUILayout.Button("Tắt Hết (Toàn Màn Hình 3D)", AntigravityUITheme.StyleButton, GUILayout.Height(20)))
                {
                    showJointsPanel = showFkMonitorPanel = showIkTargetPanel = false;
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("BẬT / TẮT CÁC BẢNG ĐIỀU KHIỂN (PANELS)", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                // Nhóm: CAMERA
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(330));
                GUILayout.BeginHorizontal();
                RobotCameraController rcc = EnsureCameraController();
                if (GUILayout.Button("Góc 3/4", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetPerspective(); }
                if (GUILayout.Button("Trên Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetTopDown(); }
                if (GUILayout.Button("Trước Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetFront(); }
                if (GUILayout.Button("Ngang", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.SetPresetSide(); }
                if (GUILayout.Button("Focus Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(36))) { if (rcc) rcc.FocusGarment(); }
                GUILayout.EndHorizontal();
                GUILayout.Label("GÓC NHÌN CAMERA TRÊN ĐỒ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }

            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
            GUILayout.EndArea();
        }

        // 3. DOCK CHỨA CÁC BẢNG ĐƯỢC BẬT (INSPECTOR DOCK)
        bool anyPanelOpen = showJointsPanel || showFkMonitorPanel || showIkTargetPanel;
        if (anyPanelOpen)
        {
            float dockW = 460f;
            float dockH = Screen.height - totalRibbonH - 16f;
            Rect dockRect = new Rect(screenW - dockW - 12f, totalRibbonH + 8f, dockW, dockH);

            GUI.DrawTexture(dockRect, AntigravityUITheme.TexBgDock);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y, 1, dockRect.height), AntigravityUITheme.TexBorder);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y, dockRect.width, 1), AntigravityUITheme.TexBorder);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y + dockRect.height - 1, dockRect.width, 1), AntigravityUITheme.TexBorder);

            GUILayout.BeginArea(new Rect(dockRect.x + 10f, dockRect.y + 8f, dockW - 20f, dockH - 16f));

            inspectorScroll = GUILayout.BeginScrollView(inspectorScroll, GUILayout.Width(dockW - 20f), GUILayout.Height(dockH - 16f));
            GUILayout.BeginVertical();

            if (showJointsPanel)
            {
                DrawJointsControlCard(dockW - 24f, () => showJointsPanel = false);
                GUILayout.Space(6f);
            }

            if (showFkMonitorPanel)
            {
                DrawFkMonitorCard(dockW - 24f, () => showFkMonitorPanel = false);
                GUILayout.Space(6f);
            }

            if (showIkTargetPanel)
            {
                DrawIkTargetCard(dockW - 24f, () => showIkTargetPanel = false);
                GUILayout.Space(6f);
            }

            GUILayout.EndVertical();
            GUILayout.EndScrollView();

            GUILayout.EndArea();
        }
    }

    public void InitializeTargetFromCurrentPose()
    {
        if (targetInitialized) return;
        CopyCurrentPoseToTarget();
        targetInitialized = true;
    }

    public void CopyCurrentPoseToTarget()
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

    public void GetForwardPoseValues(out Vector3 posMm, out Vector3 rpyDeg)
    {
        float[] q = GetJointVector();
        Matrix4x4 T = DobotCR10AKinematics.ForwardTcp(q, tcpTransform, toolLengthMm);
        posMm = new Vector3(T[0, 3] * 1000f, T[1, 3] * 1000f, T[2, 3] * 1000f);
        rpyDeg = DobotCR10AKinematics.UnityRotationToRpy(T.rotation);
    }

    public bool TryGetActualSceneTcpPos(out Vector3 posMm)
    {
        ResolveKinematicsReferences();
        if (globalReference == null || tcpTransform == null)
        {
            posMm = Vector3.zero;
            return false;
        }

        Vector3 localPosM = globalReference.InverseTransformPoint(tcpTransform.position);
        posMm = localPosM * 1000f;
        return true;
    }

    public void SolveAndApplyIk()
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
        ApplySolution(0);

        ikStatus = string.Format(CultureInfo.InvariantCulture,
            "IK THÀNH CÔNG: {0} nghiệm trong {1:F3} ms. Áp dụng Q* [{2}]. Sai số: {3:E2} mm.",
            sols.Count, sw.Elapsed.TotalMilliseconds, sols[0].branchName, sols[0].positionErrorMm);
    }

    public void SetFlangeDownTargetAndApplyIk()
    {
        targetPose[3] = 180f; // Roll
        targetPose[4] = 0f;   // Pitch
        targetText[3] = "180.0";
        targetText[4] = "0.0";

        SolveAndApplyIk();
    }

    public void ApplySolution(int index)
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

    public float GetJoint(int i)
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

    public void SetJoint(int i, float value)
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

    public float[] GetJointVector()
    {
        return new float[] { axis0, axis1, axis2, axis3, axis4, axis5 };
    }

    // Styles
    private GUIStyle CardTitleStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            fontStyle = FontStyle.Bold,
            normal = { textColor = new Color(0.3f, 0.85f, 1f) }
        };
    }

    private GUIStyle SubHeaderStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 10,
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.yellow }
        };
    }

    private GUIStyle PanelStatusStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            wordWrap = true,
            fontSize = 10,
            normal = { textColor = ikStatus.Contains("THÀNH CÔNG") ? Color.green : (ikStatus.Contains("Không tìm thấy") ? Color.red : Color.white) }
        };
    }

    private GUIStyle ErrorStyle(float errorMm)
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 10,
            normal = { textColor = errorMm < 0.01f ? Color.green : Color.yellow }
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
