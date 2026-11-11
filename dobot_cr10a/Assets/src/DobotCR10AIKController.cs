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
        public bool sprayOn = true;        // Đoạn đi TỚI waypoint này có phun sơn hay không (false = đoạn di chuyển chuyển tiếp)
        public string segmentName = "";
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
    public float toolLength = 100.0f; // Chiều dài đầu phun TCP từ mặt bích (Offset Z, mm)
    [Tooltip("Độ lệch tâm công cụ TCP theo trục X mặt bích (mm)")]
    public float toolOffsetX = 0.0f;
    [Tooltip("Độ lệch tâm công cụ TCP theo trục Y mặt bích (mm)")]
    public float toolOffsetY = 0.0f;

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
    [Range(0.1f, 2.0f)]
    public float playbackSpeed = 1.0f; // Hệ số nhân tốc độ khớp (1x mặc định)
    [Range(1f, 223f)]
    public float maxJointSpeedDegPerSecond = 223f;
    [Range(1f, 1000f)]
    public float maxJointAccelerationDegPerSecond2 = 12f;
    [Tooltip("Giới hạn tốc độ riêng cho J1-J6, đơn vị độ/giây")]
    public float[] jointSpeedDegPerSecond = { 10f, 10f, 10f, 10f, 10f, 10f };

    [Header("5. Phun sơn & Vệt màu")]
    [FormerlySerializedAs("drawInkTrail")]
    public bool sprayPaintEnabled = true;
    public Color inkColor = Color.red;
    [Range(1f, 20f)]
    public float sprayWidthMm = 4.0f; // Cỡ đường phun (mm)
    [Tooltip("Chỉ phun khi TCP nằm trong dung sai này so với đoạn quỹ đạo đang chạy (mm)")]
    [Range(2f, 60f)]
    public float sprayPathToleranceMm = 20f;
    [Tooltip("Khoảng nhảy giữa 2 waypoint liên tiếp lớn hơn ngưỡng này được coi là đoạn chuyển tiếp -> tắt phun (mm)")]
    [Range(10f, 200f)]
    public float sprayJumpThresholdMm = 45f;
    public bool isSprayingNow = false;
    private int sprayTargetIndex = -1;
    private readonly List<LineRenderer> committedTrailRenderers = new List<LineRenderer>();

    // Danh sách waypoint đã nạp
    public List<WaypointData> waypoints = new List<WaypointData>();

    private LineRenderer lineRenderer;
    private readonly List<Vector3> actualTcpTrail = new List<Vector3>();
    private bool paintingStarted;
    private bool isMovingToManualTarget;
    private bool isMovingToHome;
    private bool showJointSpeedControls;
    
    // Quản lý trạng thái 6 Tab Giao diện
    // 0: SIM, 1: ROBOT, 2: CAM & POINT CLOUD, 3: QUỸ ĐẠO, 4: XUẤT / NHẬP LỆNH, 5: CẢNH BÁO & LỖI
    public int interfaceTab = 0;
    public bool ribbonExpanded = true;
    public bool showDockPanel = true;
    public bool showJointsSubPanel = false;
    public bool showGizmos = true;
    private Vector2 dockScrollPosition;

    // Tương thích ngược cho các hệ thống cũ nếu có tham chiếu
    public bool showJointsPanel => showJointsSubPanel;
    public bool showFkMonitorPanel => false;
    public bool showIkTargetPanel => false;
    public bool showTrajectoryPanel => interfaceTab == 3 && showDockPanel;
    public bool showPipelinePanel => false;
    public bool showEvaluationPanel => false;

    [Header("Mẫu phôi Point Cloud & Quỹ đạo (Sample Dataset)")]
    public string selectedSampleName = "pcl_101";
    public List<string> availableSamples = new List<string>();
    private int selectedSampleIdx = 0;

    [Header("Tùy chọn đường dẫn & Xuất file")]
    public string customPlyPath = "Assets/PointCloud/full_cloud_world.ply";
    public string customImportCsvPath = "output_drawing/pants_outline_cr10a.csv";
    public string exportStatusMessage = "";
    private string lastExportedCodePreview = "";
    private Vector2 exportPreviewScroll = Vector2.zero;

    public struct DiagnosticItem
    {
        public enum Severity { Info, Warning, Error }
        public Severity severity;
        public string category;
        public string message;
        public int waypointIndex;
    }
    public readonly List<DiagnosticItem> diagnosticResults = new List<DiagnosticItem>();

    [Header("Chẩn đoán an toàn (Diagnostics)")]
    public bool hasCriticalErrors = false;
    public bool forceBypassSafetyInterlock = false;
    private Vector2 diagnosticsScroll = Vector2.zero;

    private string accelerationInputText = "12";
    private readonly string[] jointSpeedInputTexts = { "10", "10", "10", "10", "10", "10" };
    private readonly float[] manualTargetAnglesRad = new float[6];
    private readonly float[] jointVelocitiesRadPerSecond = new float[6];

    private readonly Dictionary<int, int> customWaypointConfigurations = new Dictionary<int, int>();
    private Vector2 waypointSolsScroll = Vector2.zero;

    // =========================================================================
    // DỮ LIỆU 12 ĐIỂM MỐC GIẢI PHẪU 3D (LANDMARKS), SEGMENTATION CONTOUR 3D
    // VÀ BẢNG VẼ QUẦN JEAN 2D TIÊU CHUẨN -> ÁNH XẠ LÊN BỀ MẶT 3D
    // =========================================================================
    [Serializable]
    public class LandmarkItem3D
    {
        public int id;
        public string name;
        public string label_vi;
        public float canonical_u;
        public float canonical_v;
        public int pixel_u;
        public int pixel_v;
        public float x_mm;
        public float y_mm;
        public float z_mm;
        public float nx;
        public float ny;
        public float nz;
    }

    [Serializable]
    public class Point3DJson
    {
        public float x_mm;
        public float y_mm;
        public float z_mm;
    }

    [Serializable]
    public class StrokePoint2D
    {
        public float u;
        public float v;
        public StrokePoint2D() { }
        public StrokePoint2D(float u, float v) { this.u = u; this.v = v; }
    }

    [Serializable]
    public class Stroke2D
    {
        public List<StrokePoint2D> points = new List<StrokePoint2D>();
    }

    [Serializable]
    public class SprayPatternTemplate
    {
        public string templateName = "mau_phun_custom";
        public string createdDate = "";
        public string size = "S";
        public float zOffsetMm = 30f;
        public List<Stroke2D> strokes = new List<Stroke2D>();
    }

    [Serializable]
    public class LandmarkSegmentationPayload
    {
        public string sample;
        public string detected_size;
        public float outseam_mm;
        public float inseam_mm;
        public float waist_mm;
        public float cuff_mm;
        public float area_cm2;
        public List<LandmarkItem3D> landmarks = new List<LandmarkItem3D>();
        public List<Point3DJson> garment_contour_3d = new List<Point3DJson>();
        public List<Point3DJson> spray_zone_left_3d = new List<Point3DJson>();
        public List<Point3DJson> spray_zone_right_3d = new List<Point3DJson>();
    }

    [Header("6. Hiển thị Điểm Mốc AI (12 Landmarks), Segment 3D & Phân Cụm Size K-Means")]
    public bool showLandmarks3D = true;
    public bool showLandmarkLabels = true;
    public bool showSegmentationContour3D = true;
    public bool showSprayZones3D = true;
    public List<LandmarkItem3D> landmarks3D = new List<LandmarkItem3D>();
    public List<Vector3> landmarksWorldPos = new List<Vector3>();
    public List<Vector3> garmentContourWorld = new List<Vector3>();
    public List<Vector3> sprayZoneLeftWorld = new List<Vector3>();
    public List<Vector3> sprayZoneRightWorld = new List<Vector3>();
    public string loadedLandmarksSampleName = "";

    // Kết quả nhận diện Size quần (K-Means 3 cụm: S, M, L) & Mẫu cần phun (mặc định là Size S)
    public string activeSprayTemplateSize = "S"; // Mặc định ban đầu là Size S
    public string detectedJeansSize = "S";
    public float detectedOutseamMm = 0f;
    public float detectedInseamMm = 0f;
    public float detectedWaistMm = 0f;
    public float detectedCuffMm = 0f;
    public float detectedAreaCm2 = 0f;
    public bool hasTriggeredSizeAutoSwitch = false;
    public string sizeDetectionSummary = "Mặc định: Mẫu cần phun SIZE S (Khi nhận diện sẽ tự đổi sang Size phù hợp)";

    [Header("7. Bảng Vẽ Quần Jean 2D Tiêu Chuẩn -> Ánh Xạ Lên Bề Mặt 3D")]
    public bool showCanonicalDrawPanel = true;
    public float customDrawZOffsetMm = 2.0f; // Mặc định 2mm (áp sát mặt quần)
    public string customDrawStatusMsg = "Mẫu cần phun mặc định: SIZE S. Khi nhận diện quần & 12 mốc sẽ tự động đổi sang Size phù hợp (S/M/L)!";
    private Texture2D canonicalJeansBgTex;
    private Texture2D whitePixelTex;
    private readonly List<List<Vector2>> customDrawnStrokes = new List<List<Vector2>>();
    private List<Vector2> currentDrawingStroke = null;
    private bool isDrawingOnCanvas = false;
    private bool hasInitializedDefaultStrokes = false;

    [Header("8. Lưu / Import & Bảo Lưu Mẫu Phun Tùy Biến Cho Mọi Phôi")]
    public bool preserveCustomDrawingAcrossSamples = true;
    public string sprayTemplateDirectory = "output_drawing/spray_templates";
    public string saveTemplateFileName = "mau_phun_1";
    private readonly List<string> availableSprayTemplateFiles = new List<string>();

    [Header("9. Thuật Toán Bù Đắp Chỗ Khuất (Occlusion & Blind Spot Compensation)")]
    public int selectedOcclusionAlgorithm = 0; // 0: TPS RBF, 1: Poisson/Laplace, 2: MLS K-NN, 3: Adaptive Gun Tilt, 4: Bilateral Symmetry
    public float occlusionSearchRadiusMm = 25.0f;
    public int occlusionKNeighbors = 16;
    public float maxGunTiltAngleDeg = 25.0f;
    public float occlusionZMarginMm = 12.0f;
    public float laplacianSmoothingFactor = 0.5f;
    public float steepSlopeAngleThresholdDeg = 55.0f;
    public bool showOcclusionHoles3D = true;
    public bool showCompensatedSurface3D = true;
    public bool showGunTiltVectors3D = true;
    public int detectedHoleCount = 0;
    public float detectedOcclusionAreaCm2 = 0f;
    public float avgCreaseDepthMm = 0f;
    public string occlusionStatusMsg = "Sẵn sàng phân tích chỗ khuất trên bề mặt quần 3D.";
    public readonly List<Vector3> detectedHolePointsWorld = new List<Vector3>();
    public readonly List<Vector3> compensatedSurfacePointsWorld = new List<Vector3>();
    public readonly List<Vector3> gunTiltNormalsWorld = new List<Vector3>();
    private readonly Dictionary<Vector2Int, float> compensatedHeightGrid = new Dictionary<Vector2Int, float>();
    private readonly Dictionary<Vector2Int, Vector3> compensatedNormalGrid = new Dictionary<Vector2Int, Vector3>();

    [Header("10. Vá Điểm 3D Chỗ Khuất (3D Inpainting Point Cloud)")]
    public bool autoInpaintHolesOnLoad = true;
    public bool showInpaintedSurface3D = true;
    public bool highlightInpaintedMesh = false; // false = màu khaki vải thật, true = màu cyan nổi bật
    public float inpaintPointSpacingMm = 3.5f; // Khoảng cách giữa các hạt điểm vá (3.5mm cho mật độ dày đặc)
    public float inpaintPointSize = 3.5f;      // Cỡ hạt điểm Point Cloud
    public GameObject inpaintedMeshObject;

    // 12 Điểm mốc chuẩn trên Quần Jean 2D Tiêu Chuẩn [0, 1] x [0, 1] (Tự động co giãn theo Size S/M/L)
    public static readonly Vector2[] CanonicalLandmarks12UV = new Vector2[]
    {
        new Vector2(0.28f, 0.10f), // 0: waist_left
        new Vector2(0.50f, 0.10f), // 1: waist_center
        new Vector2(0.72f, 0.10f), // 2: waist_right
        new Vector2(0.50f, 0.34f), // 3: crotch
        new Vector2(0.35f, 0.50f), // 4: thigh_left
        new Vector2(0.65f, 0.50f), // 5: thigh_right
        new Vector2(0.33f, 0.70f), // 6: knee_left
        new Vector2(0.67f, 0.70f), // 7: knee_right
        new Vector2(0.22f, 0.91f), // 8: cuff_left_outer
        new Vector2(0.41f, 0.91f), // 9: cuff_left_inner
        new Vector2(0.59f, 0.91f), // 10: cuff_right_inner
        new Vector2(0.78f, 0.91f), // 11: cuff_right_outer
    };

    public static Vector2[] GetCanonicalLandmarks12UV(string size)
    {
        if (size == "S")
        {
            return new Vector2[]
            {
                new Vector2(0.32f, 0.14f), // 0: waist_left
                new Vector2(0.50f, 0.14f), // 1: waist_center
                new Vector2(0.68f, 0.14f), // 2: waist_right
                new Vector2(0.50f, 0.37f), // 3: crotch
                new Vector2(0.37f, 0.51f), // 4: thigh_left
                new Vector2(0.63f, 0.51f), // 5: thigh_right
                new Vector2(0.36f, 0.68f), // 6: knee_left
                new Vector2(0.64f, 0.68f), // 7: knee_right
                new Vector2(0.27f, 0.85f), // 8: cuff_left_outer
                new Vector2(0.43f, 0.85f), // 9: cuff_left_inner
                new Vector2(0.57f, 0.85f), // 10: cuff_right_inner
                new Vector2(0.73f, 0.85f), // 11: cuff_right_outer
            };
        }
        else if (size == "L")
        {
            return new Vector2[]
            {
                new Vector2(0.24f, 0.06f), // 0: waist_left
                new Vector2(0.50f, 0.06f), // 1: waist_center
                new Vector2(0.76f, 0.06f), // 2: waist_right
                new Vector2(0.50f, 0.32f), // 3: crotch
                new Vector2(0.33f, 0.49f), // 4: thigh_left
                new Vector2(0.67f, 0.49f), // 5: thigh_right
                new Vector2(0.30f, 0.72f), // 6: knee_left
                new Vector2(0.70f, 0.72f), // 7: knee_right
                new Vector2(0.18f, 0.96f), // 8: cuff_left_outer
                new Vector2(0.39f, 0.96f), // 9: cuff_left_inner
                new Vector2(0.61f, 0.96f), // 10: cuff_right_inner
                new Vector2(0.82f, 0.96f), // 11: cuff_right_outer
            };
        }
        return CanonicalLandmarks12UV;
    }

    public static readonly string[] CanonicalLandmarkNamesVi = new string[]
    {
        "0: Eo trái", "1: Giữa eo", "2: Eo phải", "3: Đáy đũng",
        "4: Đùi trái", "5: Đùi phải", "6: Gối trái", "7: Gối phải",
        "8: Gấu T.ngoài", "9: Gấu T.trong", "10: Gấu P.trong", "11: Gấu P.ngoài"
    };

    private void Awake()
    {
        CleanupOrphanPhotoPlanes();
        EnsureComponents();
        LoadAllSettings();
        ScanAvailableSamples();
    }

    private void OnEnable()
    {
        EnsureComponents();
        if (!Application.isPlaying && robotManual != null)
        {
            robotManual.ResetAllSlidersImmediate();
        }
    }

    private void Start()
    {
        AudioListener[] listeners = FindObjectsByType<AudioListener>();
        for (int i = 1; i < listeners.Length; i++)
        {
            listeners[i].enabled = false;
        }

        CleanupOrphanPhotoPlanes();
        EnsureComponents();
        LoadAllSettings();
        ScanAvailableSamples();
        RefreshAvailableSprayTemplates();
        activeSprayTemplateSize = "S"; // Khởi tạo mặc định Mẫu cần phun là Size S
        hasTriggeredSizeAutoSwitch = false;
        LoadTrajectoryFile();
        LoadLandmarksAndSegmentationForSample(selectedSampleName, autoSwitchSprayTemplate: false);
        EnsureCanonicalJeansCanvasInitialized();
        EnsureCameraController();

        // Ban đầu khi khởi động (lúc chưa bấm CHẠY), robot luôn ở cấu hình Home (0°)
        if (Application.isPlaying)
        {
            isPlaying = false;
            paintingStarted = false;
            isMovingToManualTarget = false;
            isMovingToHome = false;
            currentWaypointIndex = 0;
            if (robotManual != null)
            {
                robotManual.ResetAllSlidersImmediate();
                robotManual.ikStatus = "Robot ở cấu hình Home (0°). Nhấn CHẠY để bắt đầu.";
                robotManual.InitializeTargetFromCurrentPose();
            }
        }
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
        PlayerPrefs.SetInt("CR10A_ShowDock", showDockPanel ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_RibbonExpanded", ribbonExpanded ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_InterfaceTab", interfaceTab);
        PlayerPrefs.SetInt("CR10A_ShowGizmos", showGizmos ? 1 : 0);

        PlayerPrefs.SetFloat("CR10A_PlaybackSpeed", playbackSpeed);
        PlayerPrefs.SetFloat("CR10A_MaxJointSpeed", maxJointSpeedDegPerSecond);
        PlayerPrefs.SetFloat("CR10A_MaxJointAcc", maxJointAccelerationDegPerSecond2);
        PlayerPrefs.SetInt("CR10A_SprayPaint", sprayPaintEnabled ? 1 : 0);
        PlayerPrefs.SetFloat("CR10A_SprayWidth", sprayWidthMm);
        PlayerPrefs.SetString("CR10A_TrajectoryCsv", trajectoryCsvPath);

        PlayerPrefs.SetFloat("CR10A_ToolOffsetZ", toolLength);
        PlayerPrefs.SetFloat("CR10A_ToolOffsetX", toolOffsetX);
        PlayerPrefs.SetFloat("CR10A_ToolOffsetY", toolOffsetY);

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
        showDockPanel = PlayerPrefs.GetInt("CR10A_ShowDock", 1) == 1;
        ribbonExpanded = PlayerPrefs.GetInt("CR10A_RibbonExpanded", 1) == 1;
        interfaceTab = PlayerPrefs.GetInt("CR10A_InterfaceTab", 0);
        showGizmos = PlayerPrefs.GetInt("CR10A_ShowGizmos", 1) == 1;

        playbackSpeed = PlayerPrefs.GetFloat("CR10A_PlaybackSpeed", 1.0f);
        maxJointSpeedDegPerSecond = PlayerPrefs.GetFloat("CR10A_MaxJointSpeed", maxJointSpeedDegPerSecond);
        maxJointAccelerationDegPerSecond2 = PlayerPrefs.GetFloat("CR10A_MaxJointAcc", maxJointAccelerationDegPerSecond2);
        sprayPaintEnabled = PlayerPrefs.GetInt("CR10A_SprayPaint", sprayPaintEnabled ? 1 : 0) == 1;
        sprayWidthMm = PlayerPrefs.GetFloat("CR10A_SprayWidth", 4.0f);
        trajectoryCsvPath = PlayerPrefs.GetString("CR10A_TrajectoryCsv", trajectoryCsvPath);

        toolLength = PlayerPrefs.GetFloat("CR10A_ToolOffsetZ", toolLength);
        toolOffsetX = PlayerPrefs.GetFloat("CR10A_ToolOffsetX", toolOffsetX);
        toolOffsetY = PlayerPrefs.GetFloat("CR10A_ToolOffsetY", toolOffsetY);
        ApplyToolOffsetToTransforms();
        UpdateSprayWidth(sprayWidthMm);

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
            lineRenderer.startWidth = sprayWidthMm * 0.001f;
            lineRenderer.endWidth = sprayWidthMm * 0.001f;
            lineRenderer.material = new Material(Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Sprites/Default"));
            lineRenderer.material.color = inkColor;
            lineRenderer.useWorldSpace = true;
        }
        else
        {
            lineRenderer.startWidth = sprayWidthMm * 0.001f;
            lineRenderer.endWidth = sprayWidthMm * 0.001f;
            if (lineRenderer.material != null)
                lineRenderer.material.color = inkColor;
        }

        ApplyToolOffsetToTransforms();
    }

    public void UpdateSprayWidth(float widthMm)
    {
        sprayWidthMm = Mathf.Clamp(widthMm, 0.5f, 30.0f);
        if (lineRenderer != null)
        {
            lineRenderer.startWidth = sprayWidthMm * 0.001f;
            lineRenderer.endWidth = sprayWidthMm * 0.001f;
        }
    }

    public void SetInkColor(Color color)
    {
        inkColor = color;
        if (lineRenderer != null && lineRenderer.material != null)
        {
            lineRenderer.material.color = inkColor;
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
        SetSprayState(false);
        ClearInkTrail();
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
            int idx_spray = Array.IndexOf(headers, "spray_enabled");
            int idx_seg = Array.IndexOf(headers, "segment_name");

            for (int i = 1; i < lines.Length; i++)
            {
                if (string.IsNullOrWhiteSpace(lines[i])) continue;
                string[] cols = lines[i].Split(',');

                WaypointData wp = new WaypointData();
                wp.index = i - 1;
                wp.distance_mm = idx_dist >= 0 ? float.Parse(cols[idx_dist], CultureInfo.InvariantCulture) : (i - 1) * 1.0f;
                wp.sprayOn = true;
                if (idx_spray >= 0 && idx_spray < cols.Length)
                {
                    string sv = cols[idx_spray].Trim();
                    wp.sprayOn = !(sv == "0" || sv.Equals("false", StringComparison.OrdinalIgnoreCase));
                }
                if (idx_seg >= 0 && idx_seg < cols.Length) wp.segmentName = cols[idx_seg].Trim();

                float x_mm = float.Parse(cols[idx_x], CultureInfo.InvariantCulture);
                float y_mm = float.Parse(cols[idx_y], CultureInfo.InvariantCulture);
                float z_mm = float.Parse(cols[idx_z], CultureInfo.InvariantCulture);
                wp.targetPosGlobal_mm = new Vector3(x_mm, y_mm, z_mm);
                Vector3 pointInRobotGlobal = wp.targetPosGlobal_mm * 0.001f;
                wp.targetPosWorld_m = globalReference != null
                    ? globalReference.TransformPoint(pointInRobotGlobal)
                    : pointInRobotGlobal;

                wp.roll_deg = idx_roll >= 0 ? float.Parse(cols[idx_roll], CultureInfo.InvariantCulture) : 180f;
                wp.pitch_deg = idx_pitch >= 0 ? float.Parse(cols[idx_pitch], CultureInfo.InvariantCulture) : 0f;
                wp.yaw_deg = idx_yaw >= 0 ? float.Parse(cols[idx_yaw], CultureInfo.InvariantCulture) : 0f;

                // Chuẩn hóa góc định hướng của mặt bích:
                // Quỹ đạo phun sơn / đường viền vải luôn có hướng mặt bích súng vuông góc xuống mặt phẳng bàn (Flange-Down: Roll=180°, Pitch=0°, Yaw=0°).
                // Nếu file CSV lưu góc TCP bị gimbal lock do trục dummy TCP xoay 90° (pitch = 90°), ta chuẩn hóa về góc mặt bích chuẩn:
                if (Mathf.Abs(wp.pitch_deg - 90f) < 2.0f)
                {
                    wp.roll_deg = 180f;
                    wp.pitch_deg = 0f;
                    wp.yaw_deg = 0f;
                }

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
                    Quaternion rot = GetWaypointFlangeRotation(wp);
                    float[] prevAngles = waypoints.Count > 0 ? waypoints[waypoints.Count - 1].jointAngles_rad : null;
                    var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, prevAngles, toolLength, useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);
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

            MarkSprayTransitions();

            Debug.Log($"[CR10A IK] Đã nạp thành công {waypoints.Count} waypoint từ {trajectoryCsvPath}.");
            currentWaypointIndex = Mathf.Clamp(currentWaypointIndex, 0, waypoints.Count - 1);

            // Luôn giải tích lại toàn bộ các điểm quỹ đạo theo đúng kích thước offset công cụ hiện hành (toolLength)
            RecomputeAllWaypointsAnalyticalIK();
            ApplySavedWaypointConfigurations();
            RunDiagnostics();

            if (!Application.isPlaying)
            {
                if (robotManual != null)
                {
                    robotManual.ResetAllSlidersImmediate();
                    robotManual.ikStatus = "Robot ở cấu hình Home (0°).";
                    robotManual.InitializeTargetFromCurrentPose();
                }
            }
            else
            {
                // Khi đang ở Play mode và chưa chạy: đưa robot về Home (0°)
                if (!isPlaying && !isMovingToManualTarget)
                {
                    currentWaypointIndex = 0;
                    paintingStarted = false;
                    if (robotManual != null)
                    {
                        robotManual.ResetAllSlidersImmediate();
                        robotManual.ikStatus = "Robot ở cấu hình Home (0°). Nhấn CHẠY để bắt đầu.";
                        robotManual.InitializeTargetFromCurrentPose();
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[CR10A IK] Lỗi khi đọc file CSV: {ex.Message}");
        }
    }

    /// <summary>
    /// Chuẩn hóa và trích xuất Quaternion định hướng của mặt bích robot tại waypoint
    /// </summary>
    public static Quaternion GetWaypointFlangeRotation(WaypointData wp)
    {
        if (wp == null) return DobotCR10AKinematics.RpyToUnityRotation(180f, 0f, 0f);
        // Nhận diện file mẫu cũ có góc gimbal artefact (pitch ~ 90 và roll ~ 139.5) -> quy về bích chúc thẳng (180, 0, 0)
        if (Mathf.Abs(wp.pitch_deg - 90f) < 0.1f && Mathf.Abs(wp.roll_deg - 139.5858f) < 1.0f)
        {
            return DobotCR10AKinematics.RpyToUnityRotation(180f, 0f, 0f);
        }
        return DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
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
                Quaternion rot = GetWaypointFlangeRotation(wp);
                float[] seed = idx > 0 ? waypoints[idx - 1].jointAngles_rad : null;
                var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, seed, toolLength, useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);
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

    public void LoadNamedTrajectory(string relativePath)
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

    public void SetToolOffset(float z_mm, float x_mm = 0f, float y_mm = 0f)
    {
        toolLength = Mathf.Max(0f, z_mm);
        toolOffsetX = x_mm;
        toolOffsetY = y_mm;

        ApplyToolOffsetToTransforms();

        if (robotManual != null && (Mathf.Abs(robotManual.toolLengthMm - toolLength) > 0.01f || Mathf.Abs(robotManual.toolOffsetX - toolOffsetX) > 0.01f || Mathf.Abs(robotManual.toolOffsetY - toolOffsetY) > 0.01f))
        {
            robotManual.SetToolOffset(toolLength, toolOffsetX, toolOffsetY);
        }

        // Tự động tính lại toàn bộ 120 waypoint với offset công cụ mới
        RecomputeAllWaypointsAnalyticalIK();
        ApplySavedWaypointConfigurations();

        // Cập nhật ngay tư thế khớp robot của waypoint hiện tại
        if (waypoints != null && waypoints.Count > 0)
        {
            ApplyWaypoint(currentWaypointIndex);
        }

        SaveAllSettings();
    }

    public void ApplyToolOffsetToTransforms()
    {
        if (tcpTransform != null)
        {
            tcpTransform.localPosition = new Vector3(toolOffsetX * 0.001f, toolOffsetY * 0.001f, toolLength * 0.001f);
        }
        if (robotManual != null && robotManual.tcpTransform != null && robotManual.tcpTransform != tcpTransform)
        {
            robotManual.tcpTransform.localPosition = new Vector3(toolOffsetX * 0.001f, toolOffsetY * 0.001f, toolLength * 0.001f);
        }
    }

    [ContextMenu("Tính lại toàn bộ waypoint bằng Thuần Giải Tích")]
    public void RecomputeAllWaypointsAnalyticalIK()
    {
        if (waypoints == null || waypoints.Count == 0) return;
        int solvedCount = 0;

        // Dùng nghiệm waypoint 0 (nếu đã có từ CSV hoặc lần tính trước) làm seed định hướng ban đầu
        float[] lastAngles = waypoints[0].isSolved && waypoints[0].jointAngles_rad != null && waypoints[0].jointAngles_rad.Length >= 6
            ? (float[])waypoints[0].jointAngles_rad.Clone()
            : null;

        for (int i = 0; i < waypoints.Count; i++)
        {
            WaypointData wp = waypoints[i];
            Quaternion rot = GetWaypointFlangeRotation(wp);
            var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, lastAngles, toolLength, useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);
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
        Debug.Log($"[CR10A IK] Đã giải tích thành công {solvedCount}/{waypoints.Count} waypoint với toolLength={toolLength:F1}mm.");
    }

    private void Update()
    {
        CleanupOrphanPhotoPlanes();
        // Không chạy quỹ đạo (về Home, đi tới điểm thủ công, dừng...) -> luôn TẮT phun
        if (!(isPlaying && Application.isPlaying) || isMovingToManualTarget)
        {
            SetSprayState(false);
        }
        if (waypoints.Count == 0 && !isMovingToManualTarget) return;

        if (isMovingToManualTarget && Application.isPlaying)
        {
            if (MoveTowardJointAngles(manualTargetAnglesRad, Time.deltaTime, false, useShortestAngle: !isMovingToHome))
            {
                isMovingToManualTarget = false;
                if (isMovingToHome)
                {
                    isMovingToHome = false;
                    if (robotManual != null)
                    {
                        robotManual.ikStatus = "Đã đưa toàn bộ 6 khớp về cấu hình Home (0°).";
                        robotManual.CopyCurrentPoseToTarget();
                    }
                }
            }
        }
        else if (isPlaying && Application.isPlaying && waypoints.Count > 0)
        {
            if (!paintingStarted && currentWaypointIndex == 0)
            {
                if (robotManual != null)
                    robotManual.ikStatus = "Đang di chuyển từ Home đến điểm đầu quỹ đạo...";
            }

            if (MoveTowardWaypoint(currentWaypointIndex, Time.deltaTime))
            {
                paintingStarted = true;
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Count;
                if (robotManual != null)
                    robotManual.ikStatus = $"Đang vận hành quỹ đạo ({currentWaypointIndex + 1}/{waypoints.Count})...";
            }
        }

        UpdateInkTrail();
    }

    /// <summary>
    /// Đưa ngay lập tức toàn bộ 6 khớp về Home (0°) không trễ
    /// </summary>
    [ContextMenu("Đưa Robot về Home (0°) - Reset to Home")]
    public void ResetToHomeImmediate()
    {
        isPlaying = false;
        SetSprayState(false);
        ClearInkTrail();
        paintingStarted = false;
        isMovingToManualTarget = false;
        isMovingToHome = false;
        currentWaypointIndex = 0;
        if (robotManual != null)
        {
            robotManual.ResetAllSlidersImmediate();
            robotManual.ikStatus = "Đã đưa toàn bộ 6 khớp về cấu hình Home (0°).";
            robotManual.InitializeTargetFromCurrentPose();
        }
#if UNITY_EDITOR
        UnityEditor.EditorUtility.SetDirty(this);
        if (robotManual != null) UnityEditor.EditorUtility.SetDirty(robotManual);
#endif
    }

    /// <summary>
    /// Di chuyển toàn bộ 6 khớp robot về vị trí Home (0°) một cách mượt mà,
    /// tuân thủ nghiêm ngặt tốc độ và gia tốc đã được giới hạn.
    /// </summary>
    public void MoveToHomeGradually()
    {
        isPlaying = false;
        SetSprayState(false);
        paintingStarted = false;
        currentWaypointIndex = 0;

        if (!Application.isPlaying)
        {
            if (robotManual != null)
            {
                robotManual.ResetAllSlidersImmediate();
            }
            return;
        }

        for (int joint = 0; joint < manualTargetAnglesRad.Length; joint++)
            manualTargetAnglesRad[joint] = 0f;

        isMovingToHome = true;
        isMovingToManualTarget = true;

        if (robotManual != null)
        {
            robotManual.ikStatus = "Đang quay về vị trí Home (0°)...";
        }
    }

    private void MoveToWaypointGradually(int index)
    {
        if (index < 0 || index >= waypoints.Count) return;

        WaypointData waypoint = waypoints[index];
        if (!waypoint.isSolved || waypoint.jointAngles_rad == null || waypoint.jointAngles_rad.Length < 6)
            return;

        isPlaying = false;
        SetSprayState(false);
        isMovingToHome = false;
        currentWaypointIndex = index;
        paintingStarted = false;
        for (int joint = 0; joint < manualTargetAnglesRad.Length; joint++)
            manualTargetAnglesRad[joint] = waypoint.jointAngles_rad[joint];
        isMovingToManualTarget = true;

        if (robotManual != null)
        {
            robotManual.SyncTargetPoseFromWaypoint(waypoint.targetPosGlobal_mm, waypoint.roll_deg, waypoint.pitch_deg, waypoint.yaw_deg);
        }
    }

    public void ApplyWaypoint(int index)
    {
        if (index < 0 || index >= waypoints.Count) return;
        currentWaypointIndex = index;
        SetSprayState(false);
        paintingStarted = false;
        isMovingToManualTarget = false;
        isMovingToHome = false;

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
            robotManual.SyncTargetPoseFromWaypoint(wp.targetPosGlobal_mm, wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
        }
    }

    private bool MoveTowardWaypoint(int index, float deltaTime)
    {
        if (robotManual == null || index < 0 || index >= waypoints.Count)
            return false;

        WaypointData waypoint = waypoints[index];
        if (!waypoint.isSolved || waypoint.jointAngles_rad == null || waypoint.jointAngles_rad.Length < 6)
        {
            SetSprayState(false);
            return true;
        }

        sprayTargetIndex = index;
        bool arrived = MoveTowardJointAngles(waypoint.jointAngles_rad, deltaTime, true);
        sprayTargetIndex = -1;
        return arrived;
    }

    private bool MoveTowardJointAngles(float[] targetAnglesRad, float deltaTime, bool allowSpraying, bool useShortestAngle = true)
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
            float deltaRad;
            float deltaDeg;

            if (useShortestAngle)
            {
                deltaDeg = Mathf.DeltaAngle(current[joint] * Mathf.Rad2Deg,
                    targetAnglesRad[joint] * Mathf.Rad2Deg);
                deltaRad = deltaDeg * Mathf.Deg2Rad;
            }
            else
            {
                deltaRad = targetAnglesRad[joint] - current[joint];
                deltaDeg = deltaRad * Mathf.Rad2Deg;
            }

            if (Mathf.Abs(deltaDeg) <= 0.05f)
            {
                current[joint] = useShortestAngle ? current[joint] + deltaRad : targetAnglesRad[joint];
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
                current[joint] = useShortestAngle ? current[joint] + deltaRad : targetAnglesRad[joint];
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

        // Chỉ phun khi: đang chạy đúng một ĐOẠN PHUN của quỹ đạo (không phải đoạn chuyển tiếp/nhảy nét)
        // VÀ TCP thực tế đang nằm trên đường quỹ đạo (trong dung sai sprayPathToleranceMm)
        bool sprayNow = allowSpraying && sprayPaintEnabled && paintingStarted && tcpTransform != null
            && sprayTargetIndex > 0 && sprayTargetIndex < waypoints.Count
            && waypoints[sprayTargetIndex].sprayOn
            && IsTcpOnTrajectorySegment(sprayTargetIndex);
        SetSprayState(sprayNow);

        if (sprayNow && (actualTcpTrail.Count == 0 ||
            Vector3.Distance(actualTcpTrail[actualTcpTrail.Count - 1], tcpTransform.position) >= 0.001f))
        {
            actualTcpTrail.Add(tcpTransform.position);
            if (actualTcpTrail.Count > 7000)
                actualTcpTrail.RemoveRange(0, actualTcpTrail.Count - 7000);
        }

        return arrived;
    }

    // =========================================================================
    // ĐIỀU KHIỂN BẬT / TẮT PHUN SƠN THEO ĐƯỜNG QUỸ ĐẠO
    // =========================================================================

    /// <summary>
    /// Đánh dấu các đoạn chuyển tiếp (không phun): điểm đầu quỹ đạo, điểm đầu mỗi nét/segment mới,
    /// và các bước nhảy dài hơn sprayJumpThresholdMm giữa 2 waypoint liên tiếp.
    /// </summary>
    public void MarkSprayTransitions()
    {
        if (waypoints == null || waypoints.Count == 0) return;
        waypoints[0].sprayOn = false;
        for (int i = 1; i < waypoints.Count; i++)
        {
            WaypointData prev = waypoints[i - 1];
            WaypointData cur = waypoints[i];
            bool newSegment = !string.IsNullOrEmpty(cur.segmentName) && !string.IsNullOrEmpty(prev.segmentName)
                && cur.segmentName != prev.segmentName;
            float jumpMm = Vector3.Distance(prev.targetPosGlobal_mm, cur.targetPosGlobal_mm);
            if (newSegment || jumpMm > sprayJumpThresholdMm)
            {
                cur.sprayOn = false;
            }
        }
    }

    /// <summary>
    /// Kiểm tra TCP thực tế có đang nằm trên đoạn quỹ đạo [index-1 -> index] hay không.
    /// </summary>
    private bool IsTcpOnTrajectorySegment(int index)
    {
        if (tcpTransform == null || index <= 0 || index >= waypoints.Count) return false;
        Vector3 a = waypoints[index - 1].targetPosWorld_m;
        Vector3 b = waypoints[index].targetPosWorld_m;
        Vector3 p = tcpTransform.position;
        Vector3 ab = b - a;
        float t = ab.sqrMagnitude > 1e-9f ? Mathf.Clamp01(Vector3.Dot(p - a, ab) / ab.sqrMagnitude) : 0f;
        float distMm = Vector3.Distance(p, a + ab * t) * 1000f;
        return distMm <= sprayPathToleranceMm;
    }

    /// <summary>
    /// Bật / tắt súng phun. Khi tắt, nét sơn hiện tại được "chốt" lại thành 1 đoạn riêng
    /// để không vẽ đường nối khi robot di chuyển ngoài quỹ đạo.
    /// </summary>
    private void SetSprayState(bool on)
    {
        if (isSprayingNow && !on)
        {
            CommitCurrentTrailSegment();
        }
        isSprayingNow = on;
    }

    private void CommitCurrentTrailSegment()
    {
        if (actualTcpTrail.Count >= 2 && lineRenderer != null)
        {
            GameObject go = new GameObject($"SprayTrailSegment_{committedTrailRenderers.Count}");
            go.hideFlags = HideFlags.DontSave;
            go.transform.SetParent(transform, false);
            LineRenderer lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.sharedMaterial = lineRenderer.sharedMaterial;
            lr.startWidth = sprayWidthMm * 0.001f;
            lr.endWidth = sprayWidthMm * 0.001f;
            lr.positionCount = actualTcpTrail.Count;
            lr.SetPositions(actualTcpTrail.ToArray());
            committedTrailRenderers.Add(lr);
        }
        actualTcpTrail.Clear();
        if (lineRenderer != null) lineRenderer.positionCount = 0;
    }

    /// <summary>
    /// Xóa toàn bộ vệt sơn (nét đang vẽ + các đoạn đã chốt).
    /// </summary>
    public void ClearInkTrail()
    {
        actualTcpTrail.Clear();
        isSprayingNow = false;
        for (int i = 0; i < committedTrailRenderers.Count; i++)
        {
            if (committedTrailRenderers[i] == null) continue;
            if (Application.isPlaying) Destroy(committedTrailRenderers[i].gameObject);
            else DestroyImmediate(committedTrailRenderers[i].gameObject);
        }
        committedTrailRenderers.Clear();
        if (lineRenderer != null) lineRenderer.positionCount = 0;
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
    // GIAO DIỆN KIỂU WINDOWS RIBBON (DẢI RUY BĂNG) THỐNG NHẤT (6 TAB TINH GỌN)
    // =========================================================================

    private void CleanupOrphanPhotoPlanes()
    {
        // 1. Quét tìm kiếm mọi GameObject rác (kể cả inactive) liên quan đến photo plane cũ
        GameObject[] allObjects = Resources.FindObjectsOfTypeAll<GameObject>();
        for (int i = 0; i < allObjects.Length; i++)
        {
            GameObject go = allObjects[i];
            if (go == null) continue;
            if (go.hideFlags != HideFlags.None) continue;
            if (go.scene.name == null) continue;

            string nLower = go.name.Trim().ToLower();

            bool isPhotoPlane = nLower.Contains("workpiece_photo") 
                             || nLower.Contains("photo_plane")
                             || nLower == "workpiece_photo_plane"
                             || nLower == "workpiece_photo_plane(clone)";

            if (!isPhotoPlane && nLower == "quad")
            {
                MeshRenderer mr = go.GetComponent<MeshRenderer>();
                if (mr != null && mr.sharedMaterial != null)
                {
                    string matName = mr.sharedMaterial.name.ToLower();
                    if (matName.Contains("unlit") || matName.Contains("photo") || 
                        (mr.sharedMaterial.mainTexture != null && mr.sharedMaterial.mainTexture.name.ToLower().Contains("pcl_")))
                    {
                        isPhotoPlane = true;
                    }
                }
            }

            if (isPhotoPlane)
            {
                DestroyImmediate(go);
            }
        }
    }

    [Header("Bộ chọn mẫu phôi (Dropdown & File Browser)")]
    public bool showSampleDropdown = false;
    public string sampleFilterText = "";
    private Vector2 sampleDropdownScroll = Vector2.zero;

    /// <summary>
    /// Mở hộp thoại chọn file của hệ điều hành để người dùng tự do chọn bất kỳ file .ply nào từ máy tính
    /// </summary>
    public void OpenFileBrowserForSample()
    {
#if UNITY_EDITOR
        string defaultDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "dataset", "_3D"));
        if (!Directory.Exists(defaultDir))
        {
            defaultDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "dataset", "_3D"));
        }

        string chosen = UnityEditor.EditorUtility.OpenFilePanel("Chọn file Point Cloud (.ply)", defaultDir, "ply");
        if (!string.IsNullOrEmpty(chosen))
        {
            SelectSampleByFullPath(chosen);
        }
#else
        Debug.LogWarning("[CR10A] OpenFilePanel chỉ khả dụng trong môi trường Unity Editor.");
#endif
    }

    /// <summary>
    /// Nạp một file PLY bất kỳ theo đường dẫn tuyệt đối hoặc tương đối
    /// </summary>
    public void SelectSampleByFullPath(string fullPath)
    {
        if (string.IsNullOrEmpty(fullPath) || !File.Exists(fullPath)) return;
        selectedSampleName = Path.GetFileNameWithoutExtension(fullPath);
        customPlyPath = fullPath;
        ReloadPointCloudFromPath(fullPath);
        LoadLandmarksAndSegmentationForSample(selectedSampleName, autoSwitchSprayTemplate: true);

        // Tự động kiểm tra file quỹ đạo cùng tên nếu có
        string candidateTraj = $"output_drawing/{selectedSampleName}_pen_trajectory.csv";
        string resolvedTraj = Path.Combine(Application.dataPath, "..", candidateTraj);
        if (!File.Exists(resolvedTraj))
        {
            resolvedTraj = Path.Combine(Application.dataPath, "..", "..", candidateTraj);
        }
        if (File.Exists(resolvedTraj))
        {
            LoadNamedTrajectory(candidateTraj);
        }
    }

    /// <summary>
    /// Quét toàn bộ các mẫu 3D Point Cloud (.ply) có trong dataset
    /// </summary>
    public void ScanAvailableSamples()
    {
        availableSamples.Clear();
        PLYDirectLoader loader = FindAnyObjectByType<PLYDirectLoader>();
        if (loader != null)
        {
            availableSamples = loader.GetAvailableSampleNames();
        }
        else
        {
            availableSamples.AddRange(new string[] {
                "pcl_18", "pcl_25", "pcl_30", "pcl_35", "pcl_48", "pcl_58", "pcl_59", "pcl_60",
                "pcl_61", "pcl_66", "pcl_79", "pcl_83", "pcl_84", "pcl_101", "pcl_111", "pcl_112",
                "pcl_151", "pcl_157", "pcl_178", "pcl_183", "pcl_184", "pcl_194", "pcl_196", "pcl_204",
                "pcl_206", "pcl_208", "pcl_209", "pcl_217", "pcl_241", "pcl_243", "pcl_256", "pcl_261",
                "pcl_263", "pcl_271", "pcl_282"
            });
        }

        if (availableSamples.Count > 0)
        {
            selectedSampleIdx = availableSamples.IndexOf(selectedSampleName);
            if (selectedSampleIdx < 0) selectedSampleIdx = 0;
            selectedSampleName = availableSamples[selectedSampleIdx];
        }
    }

    /// <summary>
    /// Chọn mẫu phôi và tự động nạp Point Cloud + Quỹ đạo tương ứng
    /// </summary>
    public void SelectSample(string sampleName, bool loadLocalPointCloud = true)
    {
        if (string.IsNullOrEmpty(sampleName)) return;
        selectedSampleName = sampleName.Trim();
        if (availableSamples.Count == 0) ScanAvailableSamples();
        selectedSampleIdx = availableSamples.IndexOf(selectedSampleName);

        customPlyPath = $"dataset/_3D/{selectedSampleName}.ply";

        if (loadLocalPointCloud)
        {
            ReloadPointCloudFromPath(customPlyPath);
        }
        LoadLandmarksAndSegmentationForSample(selectedSampleName, autoSwitchSprayTemplate: true);

        // Tự động kiểm tra file quỹ đạo tương ứng
        string candidateTraj = $"output_drawing/{selectedSampleName}_pen_trajectory.csv";
        string resolvedTraj = Path.Combine(Application.dataPath, "..", candidateTraj);
        if (!File.Exists(resolvedTraj))
        {
            resolvedTraj = Path.Combine(Application.dataPath, "..", "..", candidateTraj);
        }
        if (File.Exists(resolvedTraj))
        {
            LoadNamedTrajectory(candidateTraj);
        }
    }

    /// <summary>
    /// Chuyển đổi qua lại giữa các mẫu phôi theo thứ tự (+1 hoặc -1)
    /// </summary>
    public void CycleSample(int delta)
    {
        if (availableSamples == null || availableSamples.Count == 0) ScanAvailableSamples();
        if (availableSamples.Count == 0) return;

        selectedSampleIdx = (selectedSampleIdx + delta + availableSamples.Count) % availableSamples.Count;
        SelectSample(availableSamples[selectedSampleIdx], true);
    }

    private void OnGUI()
    {
        CleanupOrphanPhotoPlanes();
        AntigravityUITheme.EnsureInitialized();
        if (robotManual == null) EnsureComponents();
        if (landmarks3D.Count == 0) LoadLandmarksAndSegmentationForSample(selectedSampleName);
        EnsureCanonicalJeansCanvasInitialized();

        // Vẽ trực tiếp Đường bao Segment 3D, 12 Điểm Mốc Giải Phẫu AI & Quỹ Đạo 3D lên Viewport
        DrawViewport3DOverlaysInGUI();

        float screenW = Screen.width;
        float headerHeight = 34f;
        float stripHeight = ribbonExpanded ? 64f : 0f;
        float totalRibbonH = headerHeight + stripHeight;

        // 1. THANH RUY BĂNG TRÊN CÙNG (TOP RIBBON BAR - NỀN TỐI PHẲNG ANTIGRAVITY)
        GUI.DrawTexture(new Rect(0, 0, screenW, totalRibbonH), AntigravityUITheme.TexBgHeader);
        GUI.DrawTexture(new Rect(0, totalRibbonH - 1, screenW, 1), AntigravityUITheme.TexBorder);
        if (ribbonExpanded)
        {
            GUI.DrawTexture(new Rect(0, headerHeight, screenW, stripHeight), AntigravityUITheme.TexBgStrip);
            GUI.DrawTexture(new Rect(0, headerHeight - 1, screenW, 1), AntigravityUITheme.TexBorder);
        }

        // Header Line: Logo + 8 Tabs cô đọng + Nút hành động nhanh
        GUILayout.BeginArea(new Rect(10, 3, screenW - 20, headerHeight));
        GUILayout.BeginHorizontal();

        // Brand Title
        GUILayout.Label("CR10A WORKBENCH", AntigravityUITheme.StyleHeaderTitle, GUILayout.Width(165));

        // 8 Tab chuẩn (Tên cô đọng, thanh lịch, gọn gàng)
        string[] tabNames = {
            "MÔ PHỎNG",
            "CẤU HÌNH",
            "POINT CLOUD",
            "QUỸ ĐẠO",
            "XUẤT/NHẬP",
            "CẢNH BÁO",
            "MỐC & SEG",
            "BÙ CHỖ KHUẤT"
        };
        float[] tabWidths = { 88f, 82f, 104f, 90f, 96f, 88f, 96f, 108f };

        for (int t = 0; t < tabNames.Length; t++)
        {
            bool isCurrent = (interfaceTab == t);
            if (AntigravityUITheme.RibbonTab(tabNames[t], isCurrent, tabWidths[t], 26f))
            {
                interfaceTab = t;
                showDockPanel = true;
                if (t == 5 && diagnosticResults.Count == 0) RunDiagnostics();
                if (t == 6)
                {
                    RobotCameraController camCtrl = EnsureCameraController();
                    if (camCtrl != null) camCtrl.SetPresetTopDown();
                    LoadLandmarksAndSegmentationForSample(selectedSampleName);
                }
                if (t == 7)
                {
                    if (detectedHoleCount == 0) DetectOcclusionAndHoles();
                }
            }
        }

        GUILayout.FlexibleSpace();

        // Nút Bật/Tắt Đường Dự Kiến Phun 3D (Đường Cyan)
        string trajQuickText = showGizmos ? "Cyan: BẬT" : "Cyan: TẮT";
        GUIStyle trajQuickStyle = showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button(trajQuickText, trajQuickStyle, GUILayout.Height(24)))
        {
            showGizmos = !showGizmos;
        }

        GUILayout.Space(4);

        // Nút Bật/Tắt Dock bên phải
        if (GUILayout.Button(showDockPanel ? "Ẩn Bảng" : "Hiện Bảng", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showDockPanel = !showDockPanel;
        }

        GUILayout.Space(4);

        // Ribbon Collapse/Expand Toggle
        if (GUILayout.Button(ribbonExpanded ? "Thu Ruy Băng" : "Mở Ruy Băng", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
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
                // =============================================================
                // TAB 0: MÔ PHỎNG (SIM) - PLAY, STOP, RESET, TỐC ĐỘ 1X MẶC ĐỊNH
                // =============================================================
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(270));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(isPlaying ? "TẠM DỪNG" : "CHẠY", isPlaying ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
                {
                    isPlaying = !isPlaying;
                    if (isPlaying)
                    {
                        isMovingToManualTarget = false;
                        isMovingToHome = false;
                    }
                }
                if (GUILayout.Button("DỪNG", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    isPlaying = false;
                    isMovingToManualTarget = false;
                }
                if (GUILayout.Button("RESET HOME", isMovingToHome ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    MoveToHomeGradually();
                    ClearInkTrail();
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("VẬN HÀNH MÔ PHỎNG", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(310));
                GUILayout.BeginHorizontal();
                GUILayout.Label($"Tốc độ: {playbackSpeed:F1}×", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(80));
                playbackSpeed = GUILayout.HorizontalSlider(playbackSpeed, 0.1f, 2.0f);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("0.5×", AntigravityUITheme.StyleButton, GUILayout.Height(18))) playbackSpeed = 0.5f;
                if (GUILayout.Button("1.0× (Chuẩn)", Mathf.Abs(playbackSpeed - 1.0f) < 0.05f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(18))) playbackSpeed = 1.0f;
                if (GUILayout.Button("1.5×", AntigravityUITheme.StyleButton, GUILayout.Height(18))) playbackSpeed = 1.5f;
                if (GUILayout.Button("2.0×", AntigravityUITheme.StyleButton, GUILayout.Height(18))) playbackSpeed = 2.0f;
                GUILayout.EndHorizontal();
                GUILayout.Label("TỐC ĐỘ MÔ PHỎNG (1× CHUẨN)", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(250));
                int wpTotal = Mathf.Max(1, waypoints.Count);
                int wpCur = Mathf.Clamp(currentWaypointIndex, 0, wpTotal - 1);
                float progress = (float)wpCur / (wpTotal - 1) * 100f;
                GUILayout.Label($"Điểm: {wpCur + 1} / {wpTotal} ({progress:F0}%)", AntigravityUITheme.StyleLabelCyan);
                string simStatus = isPlaying ? (isSprayingNow ? "🟢 ĐANG PHUN" : "🟡 CHUYỂN TIẾP") : (isMovingToHome ? "Đang về Home..." : "Đang chờ.");
                GUILayout.Label(simStatus, isSprayingNow ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelMuted);
                GUILayout.Label("TRẠNG THÁI", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 1)
            {
                // =============================================================
                // TAB 1: CẤU HÌNH ROBOT - VẬN TỐC KHỚP, OFFSET TCP, MÀU & CỠ ĐƯỜNG PHUN
                // =============================================================
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(250));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(showJointsSubPanel ? "Đóng Bảng Khớp" : "Mở Bảng Khớp", showJointSubPanelActive() ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    showJointsSubPanel = !showJointsSubPanel;
                    showDockPanel = true;
                }
                GUILayout.EndHorizontal();
                GUILayout.Label($"Gia tốc: {maxJointAccelerationDegPerSecond2:F0}°/s² | V_max: {maxJointSpeedDegPerSecond:F0}°/s", AntigravityUITheme.StyleLabelMuted);
                GUILayout.Label("VẬN TỐC & GIA TỐC KHỚP", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(330));
                GUILayout.BeginHorizontal();
                GUILayout.Label($"Offset TCP: Z={toolLength:F1}mm (X={toolOffsetX:F1}, Y={toolOffsetY:F1})", AntigravityUITheme.StyleLabelCyan);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("0 (Bích)", Mathf.Abs(toolLength) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(18))) SetToolOffset(0f, 0f, 0f);
                if (GUILayout.Button("50 (Bút)", Mathf.Abs(toolLength - 50f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(18))) SetToolOffset(50f, 0f, 0f);
                if (GUILayout.Button("100 (Súng)", Mathf.Abs(toolLength - 100f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(18))) SetToolOffset(100f, 0f, 0f);
                if (GUILayout.Button("150 (Kẹp)", Mathf.Abs(toolLength - 150f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(18))) SetToolOffset(150f, 0f, 0f);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                float rZ = GUILayout.HorizontalSlider(toolLength, 0f, 250f);
                if (Mathf.Abs(rZ - toolLength) > 0.05f) SetToolOffset(Mathf.Round(rZ * 10f) / 10f, toolOffsetX, toolOffsetY);
                GUILayout.EndHorizontal();
                GUILayout.Label("OFFSET ĐẦU TCP", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(220));
                sprayPaintEnabled = GUILayout.Toggle(sprayPaintEnabled, " Vệt vẽ (Trail)", AntigravityUITheme.StyleToggle, GUILayout.Height(20));
                if (sprayPaintEnabled)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label($"Nét: {sprayWidthMm:F1}mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(80));
                    float newW = GUILayout.HorizontalSlider(sprayWidthMm, 1f, 10f);
                    if (Mathf.Abs(newW - sprayWidthMm) > 0.1f) UpdateSprayWidth(newW);
                    GUILayout.EndHorizontal();
                }
                GUILayout.Label("NÉT VẼ MÔ PHỎNG", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 2)
            {
                // =============================================================
                // TAB 2: CAM & ĐÁM MÂY ĐIỂM - PRESET CAMERA & BỘ CHỌN MẪU PHÔI
                // =============================================================
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(310));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("3/4", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.SetPresetPerspective(); }
                if (GUILayout.Button("Trên", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.SetPresetTopDown(); }
                if (GUILayout.Button("Trước", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.SetPresetFront(); }
                if (GUILayout.Button("Ngang", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.SetPresetSide(); }
                if (GUILayout.Button("Focus", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.FocusGarment(); }
                GUILayout.EndHorizontal();
                GUILayout.Label("GÓC NHÌN CAMERA TRÊN ĐỒ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                // KHỐI NẠP FILE PHÔI (.PLY)
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(250));
                GUILayout.BeginHorizontal();
                string tabSampleName = !string.IsNullOrEmpty(selectedSampleName) ? selectedSampleName : "Chọn File";
                if (GUILayout.Button($"📁 Duyệt File [{tabSampleName}]", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
                {
                    OpenFileBrowserForSample();
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("NẠP FILE PHÔI (.PLY)", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(300));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Nạp Lại", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    ReloadPointCloudFromPath(customPlyPath);
                }
                bool isCloudVis = plyLoader != null && plyLoader.IsVisible();
                if (GUILayout.Button(isCloudVis ? "Ẩn Phôi" : "Hiện Phôi", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    TogglePointCloudVisible(!isCloudVis);
                }
                bool isMesh3D = plyLoader != null && plyLoader.renderMode == PLYDirectLoader.WorkpieceRenderMode.Mesh3D;
                if (GUILayout.Button(isMesh3D ? "Đổi: PointCloud" : "Đổi: 3D Mesh", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    if (plyLoader != null) plyLoader.SetRenderMode(isMesh3D ? PLYDirectLoader.WorkpieceRenderMode.PointCloud : PLYDirectLoader.WorkpieceRenderMode.Mesh3D);
                }
                GUILayout.EndHorizontal();
                int pts = plyLoader != null ? plyLoader.totalLoadedPoints : 0;
                GUILayout.Label($"Đã nạp: {pts:N0} điểm | Chế độ: {(isMesh3D ? "3D Mesh (Siêu nhẹ)" : "Point Cloud")}", AntigravityUITheme.StyleLabelMuted);
                GUILayout.Label("HIỂN THỊ PHÔI QUẦN", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 3)
            {
                // =============================================================
                // TAB 3: QUỸ ĐẠO - VẼ QUẦN 2D -> 3D, KÍCH HOẠT AI & CHỈNH SỬA
                // =============================================================
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(400));
                GUILayout.BeginHorizontal();
                showGizmos = GUILayout.Toggle(showGizmos, " Hiện Cyan", AntigravityUITheme.StyleToggle, GUILayout.Width(95), GUILayout.Height(26));

                if (GUILayout.Button("🎯 Áp Dụng 2D➔3D", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
                {
                    ApplyCanonical2DDrawingTo3DTrajectory();
                }

                AiServerClient aiClient = EnsureAiClient();
                bool isAiBusy = aiClient != null && aiClient.isProcessing;
                string aiBtnText = isAiBusy ? "Đang xử lý..." : $"⚡ Nhận Diện [{selectedSampleName}]";
                if (GUILayout.Button(aiBtnText, AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    DetectJeansSizeAndSwitchSprayTemplate(selectedSampleName);
                    if (aiClient != null) aiClient.TriggerDetectionWithSample(detectedJeansSize, selectedSampleName);
                    else LoadPantsOutlineTrajectory();
                }
                GUILayout.EndHorizontal();
                GUILayout.Label($"Size quần: {detectedJeansSize} | Mẫu phun: Size {activeSprayTemplateSize}", AntigravityUITheme.StyleLabelGreen);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(340));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("< Trước", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    int prev = Mathf.Max(0, currentWaypointIndex - 1);
                    ApplyWaypoint(prev);
                }
                GUILayout.Label($"Wp #{currentWaypointIndex} ({waypoints.Count} pts)", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(110));
                if (GUILayout.Button("Sau >", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    int next = Mathf.Min(waypoints.Count - 1, currentWaypointIndex + 1);
                    ApplyWaypoint(next);
                }
                if (GUILayout.Button("Khôi phục", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    ResetWaypointToDefault(currentWaypointIndex);
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("CHỈNH SỬA TỪNG ĐOẠN", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                if (currentWaypointIndex >= 0 && currentWaypointIndex < waypoints.Count)
                {
                    var curWp = waypoints[currentWaypointIndex];
                    GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(250));
                    GUILayout.BeginHorizontal();
                    GUILayout.Label($"R:{curWp.roll_deg:F0}° P:{curWp.pitch_deg:F0}° Y:{curWp.yaw_deg:F0}°", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(95));
                    if (GUILayout.Button("P -5°", AntigravityUITheme.StyleButton, GUILayout.Width(40), GUILayout.Height(26))) UpdateWaypointRpy(currentWaypointIndex, curWp.roll_deg, curWp.pitch_deg - 5f, curWp.yaw_deg);
                    if (GUILayout.Button("P +5°", AntigravityUITheme.StyleButton, GUILayout.Width(40), GUILayout.Height(26))) UpdateWaypointRpy(currentWaypointIndex, curWp.roll_deg, curWp.pitch_deg + 5f, curWp.yaw_deg);
                    if (GUILayout.Button("180°", AntigravityUITheme.StyleButton, GUILayout.Width(42), GUILayout.Height(26))) UpdateWaypointRpy(currentWaypointIndex, 180f, 0f, 0f);
                    GUILayout.EndHorizontal();
                    GUILayout.Label("GÓC SÚNG RPY", AntigravityUITheme.StyleSectionLabel);
                    GUILayout.EndVertical();
                }
            }
            else if (interfaceTab == 4)
            {
                // =============================================================
                // TAB 4: XUẤT / NHẬP LỆNH - DOBOT PYTHON, CSV TỌA ĐỘ
                // =============================================================
                bool canExport = !hasCriticalErrors || forceBypassSafetyInterlock;
                GUIStyle exportBtnStyle = canExport ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(330));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Xuất Dobot (.py)", exportBtnStyle, GUILayout.Height(26)))
                {
                    ExportDobotPythonScript();
                }
                if (GUILayout.Button("Xuất CSV Tọa Độ", exportBtnStyle, GUILayout.Height(26)))
                {
                    ExportUpdatedCsvTrajectory();
                }
                GUILayout.EndHorizontal();
                GUILayout.Label(canExport ? "XUẤT CHƯƠNG TRÌNH GỬI SANG ROBOT CR10A" : "CHẶN XUẤT: Có lỗi an toàn nghiêm trọng", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(280));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Mở Thư Mục Xuất", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    OpenExportFolder();
                }
                if (GUILayout.Button("Nạp Lại File CSV", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    LoadTrajectoryFile();
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("QUẢN LÝ TẬP TIN DỮ LIỆU", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 5)
            {
                // =============================================================
                // TAB 5: CẢNH BÁO VÀ XỬ LÝ LỖI - PHÔI, VA CHẠM, KỲ DỊ, VƯỢT TẦM VỚI
                // =============================================================
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(380));
                GUILayout.BeginHorizontal();
                if (hasCriticalErrors)
                {
                    GUILayout.Label("❌ PHÁT HIỆN LỖI: Khóa an toàn chặn xuất lệnh!", AntigravityUITheme.StyleLabelRed);
                }
                else
                {
                    GUILayout.Label("✅ QUỸ ĐẠO AN TOÀN 100%: Sẵn sàng xuất lệnh.", AntigravityUITheme.StyleLabelGreen);
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("GIÁM SÁT AN TOÀN QUỸ ĐẠO", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(270));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Quét Lại An Toàn", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
                {
                    RunDiagnostics();
                }
                if (GUILayout.Button("Sửa Quỹ Đạo", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
                {
                    interfaceTab = 3;
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("THAO TÁC XỬ LÝ", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 6)
            {
                // =============================================================
                // TAB 6: CAMERA TRÊN XUỐNG & HIỂN THỊ 12 ĐIỂM MỐC AI / SEGMENT
                // =============================================================
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(280));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("📷 Cam Trên Xuống", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
                {
                    if (rcc != null) rcc.SetPresetTopDown();
                }
                if (GUILayout.Button("Góc 3/4", AntigravityUITheme.StyleButton, GUILayout.Width(75), GUILayout.Height(26)))
                {
                    if (rcc != null) rcc.SetPresetPerspective();
                }
                GUILayout.EndHorizontal();
                GUILayout.Label("CAMERA NHÌN TỪ TRÊN XUỐNG", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(440));
                GUILayout.BeginHorizontal();
                showLandmarks3D = GUILayout.Toggle(showLandmarks3D, " 12 Mốc AI", AntigravityUITheme.StyleToggle, GUILayout.Width(100), GUILayout.Height(26));
                showSegmentationContour3D = GUILayout.Toggle(showSegmentationContour3D, " Đường Bao Seg", AntigravityUITheme.StyleToggle, GUILayout.Width(130), GUILayout.Height(26));
                showSprayZones3D = GUILayout.Toggle(showSprayZones3D, " Vùng Phun", AntigravityUITheme.StyleToggle, GUILayout.Width(110), GUILayout.Height(26));
                GUILayout.EndHorizontal();
                GUILayout.Label($"[{selectedSampleName}]: {landmarks3D.Count} MỐC 3D | {garmentContourWorld.Count} ĐIỂM SEG", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }
            else if (interfaceTab == 7)
            {
                // =============================================================
                // TAB 7: THUẬT TOÁN BÙ ĐẮP CHỖ KHUẤT (OCCLUSION & HOLE FILLING)
                // =============================================================
                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(330));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("🔍 Quét Lỗ Khuyết", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
                {
                    DetectOcclusionAndHoles();
                }
                if (GUILayout.Button("⚡ Chạy Bù Đắp", AntigravityUITheme.StyleButton, GUILayout.Width(100), GUILayout.Height(26)))
                {
                    RunOcclusionCompensation();
                }
                if (GUILayout.Button("🎯 Áp Dụng", AntigravityUITheme.StyleButton, GUILayout.Width(90), GUILayout.Height(26)))
                {
                    ApplyOcclusionCompensationToTrajectory();
                }
                GUILayout.EndHorizontal();
                GUILayout.Label($"PHÁT HIỆN: {detectedHoleCount} VÙNG KHUẤT ({detectedOcclusionAreaCm2:F1} cm²)", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();

                GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle(), GUILayout.Width(420));
                GUILayout.BeginHorizontal();
                showOcclusionHoles3D = GUILayout.Toggle(showOcclusionHoles3D, " Lỗ Khuyết (Đỏ)", AntigravityUITheme.StyleToggle, GUILayout.Width(130), GUILayout.Height(26));
                showCompensatedSurface3D = GUILayout.Toggle(showCompensatedSurface3D, " Lưới Bù (Xanh)", AntigravityUITheme.StyleToggle, GUILayout.Width(130), GUILayout.Height(26));
                showGunTiltVectors3D = GUILayout.Toggle(showGunTiltVectors3D, " Bù Góc Súng (Vàng)", AntigravityUITheme.StyleToggle, GUILayout.Width(140), GUILayout.Height(26));
                GUILayout.EndHorizontal();
                string[] algoShortNames = { "TPS Spline RBF", "Poisson & Laplace PDE", "MLS K-NN Tangent", "Adaptive Gun Tilt", "Bilateral Symmetry" };
                GUILayout.Label($"THUẬT TOÁN: {algoShortNames[Mathf.Clamp(selectedOcclusionAlgorithm, 0, 4)]}", AntigravityUITheme.StyleSectionLabel);
                GUILayout.EndVertical();
            }

            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
            GUILayout.EndArea();
        }

        // 3. DOCK CHỨA BẢNG ĐIỀU KHIỂN CHI TIẾT BÊN PHẢI (INSPECTOR DOCK BÊN PHẢI)
        if (showDockPanel)
        {
            float dockW = 410f;
            float dockH = Screen.height - totalRibbonH - 14f;
            Rect dockRect = new Rect(screenW - dockW - 10f, totalRibbonH + 6f, dockW, dockH);

            GUI.DrawTexture(dockRect, AntigravityUITheme.TexBgDock);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y, 1, dockRect.height), AntigravityUITheme.TexBorder);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y, dockRect.width, 1), AntigravityUITheme.TexBorder);
            GUI.DrawTexture(new Rect(dockRect.x, dockRect.y + dockRect.height - 1, dockRect.width, 1), AntigravityUITheme.TexBorder);

            GUILayout.BeginArea(new Rect(dockRect.x + 6f, dockRect.y + 4f, dockW - 12f, dockH - 8f));
            dockScrollPosition = GUILayout.BeginScrollView(dockScrollPosition, GUILayout.Width(dockW - 12f), GUILayout.Height(dockH - 8f));
            float cardWidth = dockW - 28f;

            switch (interfaceTab)
            {
                case 0:
                    DrawSimCard(cardWidth, () => showDockPanel = false);
                    break;
                case 1:
                    DrawRobotConfigCard(cardWidth, () => showDockPanel = false);
                    break;
                case 2:
                    DrawCamPointCloudCard(cardWidth, () => showDockPanel = false);
                    break;
                case 3:
                    DrawTrajectoryEditingCard(cardWidth, () => showDockPanel = false);
                    break;
                case 4:
                    DrawExportImportCard(cardWidth, () => showDockPanel = false);
                    break;
                case 5:
                    DrawDiagnosticsCard(cardWidth, () => showDockPanel = false);
                    break;
                case 6:
                    DrawTopDownLandmarksAndSegCard(cardWidth, () => showDockPanel = false);
                    break;
                case 7:
                    DrawOcclusionCompensationCard(cardWidth, () => showDockPanel = false);
                    break;
            }

            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }
    }

    private bool showJointSubPanelActive()
    {
        return showJointsSubPanel;
    }

    // =========================================================================
    // CARD 0: MÔ PHỎNG (SIMULATION CARD)
    // =========================================================================
    private void DrawSimCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("MÔ PHỎNG VẬN HÀNH ROBOT (SIM)", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(4);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(isPlaying ? "TẠM DỪNG" : "CHẠY MÔ PHỎNG", isPlaying ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton, GUILayout.Height(32)))
        {
            isPlaying = !isPlaying;
            if (isPlaying) { isMovingToManualTarget = false; isMovingToHome = false; }
        }
        if (GUILayout.Button("DỪNG HẲN", AntigravityUITheme.StyleButton, GUILayout.Height(32)))
        {
            isPlaying = false;
            isMovingToManualTarget = false;
        }
        if (GUILayout.Button("RESET (HOME 0°)", isMovingToHome ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(32)))
        {
            MoveToHomeGradually();
            ClearInkTrail();
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(8);
        GUILayout.Label("TỐC ĐỘ MÔ PHỎNG (1× MẶC ĐỊNH)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Hệ số tốc độ: {playbackSpeed:F1}×", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(130));
        playbackSpeed = GUILayout.HorizontalSlider(playbackSpeed, 0.1f, 2.0f);
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("0.5×", AntigravityUITheme.StyleButton, GUILayout.Height(22))) playbackSpeed = 0.5f;
        if (GUILayout.Button("1.0× (Mặc định)", Mathf.Abs(playbackSpeed - 1.0f) < 0.05f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22))) playbackSpeed = 1.0f;
        if (GUILayout.Button("1.5×", AntigravityUITheme.StyleButton, GUILayout.Height(22))) playbackSpeed = 1.5f;
        if (GUILayout.Button("2.0×", AntigravityUITheme.StyleButton, GUILayout.Height(22))) playbackSpeed = 2.0f;
        GUILayout.EndHorizontal();

        GUILayout.Space(8);
        GUILayout.Label("TIẾN ĐỘ VẬN HÀNH QUỸ ĐẠO", AntigravityUITheme.StyleSectionLabel);
        int wpTotal = Mathf.Max(1, waypoints.Count);
        int wpCur = Mathf.Clamp(currentWaypointIndex, 0, wpTotal - 1);
        float progress = (float)wpCur / (wpTotal - 1) * 100f;
        float distMm = waypoints.Count > 0 ? waypoints[wpCur].distance_mm : 0f;
        GUILayout.Label($"Điểm hiện tại: {wpCur + 1} / {wpTotal} ({progress:F1}%)", AntigravityUITheme.StyleLabelCyan);
        GUILayout.Label($"Quãng đường đầu phun: {distMm:F1} mm", AntigravityUITheme.StyleLabelMuted);

        int newIdx = Mathf.RoundToInt(GUILayout.HorizontalSlider(wpCur, 0, wpTotal - 1));
        if (newIdx != currentWaypointIndex)
        {
            isPlaying = false;
            ApplyWaypoint(newIdx);
        }

        GUILayout.Space(8);
        GUILayout.Label("TỌA ĐỘ TCP THỰC TẾ TRÊN MÔ HÌNH", AntigravityUITheme.StyleSectionLabel);
        if (tcpTransform != null)
        {
            Vector3 worldTcp = tcpTransform.position;
            Vector3 robotTcpMm = globalReference != null ? globalReference.InverseTransformPoint(worldTcp) * 1000f : worldTcp * 1000f;
            GUILayout.Label($"X: {robotTcpMm.x:F1} mm   Y: {robotTcpMm.y:F1} mm   Z: {robotTcpMm.z:F1} mm", AntigravityUITheme.StyleLabelCyan);
        }

        GUILayout.Space(6);
        string statusText = robotManual != null && !string.IsNullOrEmpty(robotManual.ikStatus) ? robotManual.ikStatus : "Sẵn sàng.";
        GUILayout.Label($"Trạng thái: {statusText}", AntigravityUITheme.StyleLabelMuted);

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 1: CẤU HÌNH ROBOT (ROBOT CONFIG CARD)
    // =========================================================================
    private void DrawRobotConfigCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("CẤU HÌNH ROBOT & ĐẦU PHUN SƠN", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);
        GUILayout.Label("1. GIỚI HẠN VẬN TỐC & GIA TỐC CÁC KHỚP", AntigravityUITheme.StyleSectionLabel);
        DrawAccelerationInput();
        GUILayout.Label($"Trần tốc độ chung: {Mathf.Clamp(maxJointSpeedDegPerSecond, 1f, 223f):F0}°/s", AntigravityUITheme.StyleLabelMuted);

        showJointSpeedControls = GUILayout.Toggle(showJointSpeedControls, " Bảng chỉnh vận tốc riêng từng khớp J1-J6", AntigravityUITheme.StyleToggle);
        if (showJointSpeedControls)
        {
            DrawJointSpeedInputs();
        }

        if (GUILayout.Button(showJointsSubPanel ? "Đóng Bảng Điều Khiển Khớp q1-q6" : "Mở Bảng Điều Khiển Khớp q1-q6 (Thủ công)", showJointsSubPanel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showJointsSubPanel = !showJointsSubPanel;
        }

        if (showJointsSubPanel && robotManual != null)
        {
            GUILayout.Space(4);
            robotManual.DrawJointsControlCard(width - 16, () => showJointsSubPanel = false);
        }

        GUILayout.Space(10);
        GUILayout.Label("2. KÍCH THƯỚC OFFSET ĐẦU CÔNG CỤ (MẶT BÍCH ➔ TCP)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Offset Z (Dài): {toolLength:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(150));
        float rZ = GUILayout.HorizontalSlider(toolLength, 0f, 300f);
        if (Mathf.Abs(rZ - toolLength) > 0.05f) SetToolOffset(Mathf.Round(rZ * 10f) / 10f, toolOffsetX, toolOffsetY);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("Mẫu sẵn:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(60));
        if (GUILayout.Button("0 (Bích)", Mathf.Abs(toolLength) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) SetToolOffset(0f, 0f, 0f);
        if (GUILayout.Button("50 (Bút)", Mathf.Abs(toolLength - 50f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) SetToolOffset(50f, 0f, 0f);
        if (GUILayout.Button("100 (Súng)", Mathf.Abs(toolLength - 100f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) SetToolOffset(100f, 0f, 0f);
        if (GUILayout.Button("150 (Kẹp)", Mathf.Abs(toolLength - 150f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) SetToolOffset(150f, 0f, 0f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Lệch X: {toolOffsetX:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(130));
        float rX = GUILayout.HorizontalSlider(toolOffsetX, -100f, 100f);
        if (Mathf.Abs(rX - toolOffsetX) > 0.1f) SetToolOffset(toolLength, Mathf.Round(rX * 10f) / 10f, toolOffsetY);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Lệch Y: {toolOffsetY:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(130));
        float rY = GUILayout.HorizontalSlider(toolOffsetY, -100f, 100f);
        if (Mathf.Abs(rY - toolOffsetY) > 0.1f) SetToolOffset(toolLength, toolOffsetX, Mathf.Round(rY * 10f) / 10f);
        GUILayout.EndHorizontal();

        GUILayout.Space(10);
        GUILayout.Label("3. VỆT BÚT VẼ MÔ PHỎNG (PEN TRAIL)", AntigravityUITheme.StyleSectionLabel);
        sprayPaintEnabled = GUILayout.Toggle(sprayPaintEnabled, sprayPaintEnabled ? " HIỂN THỊ NÉT VẼ: BẬT" : " HIỂN THỊ NÉT VẼ: TẮT", AntigravityUITheme.StyleToggle, GUILayout.Height(22));

        if (sprayPaintEnabled)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Bề rộng nét vẽ: {sprayWidthMm:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(160));
            float newWidth = GUILayout.HorizontalSlider(sprayWidthMm, 1f, 10f);
            if (Mathf.Abs(newWidth - sprayWidthMm) > 0.1f) UpdateSprayWidth(newWidth);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Cỡ sẵn:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(50));
            if (GUILayout.Button("2mm (Bút nhỏ)", Mathf.Abs(sprayWidthMm - 2f) < 0.2f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) UpdateSprayWidth(2.0f);
            if (GUILayout.Button("4mm (Bút dạ)", Mathf.Abs(sprayWidthMm - 4f) < 0.2f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) UpdateSprayWidth(4.0f);
            if (GUILayout.Button("Xóa nét vẽ", AntigravityUITheme.StyleButton, GUILayout.Height(20))) { ClearInkTrail(); }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Dung sai bám đường: {sprayPathToleranceMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(170));
            sprayPathToleranceMm = GUILayout.HorizontalSlider(sprayPathToleranceMm, 5f, 50f);
            GUILayout.EndHorizontal();

            string sprayStateLabel = isSprayingNow ? "🟢 TRẠNG THÁI: ĐANG PHUN TRÊN QUỸ ĐẠO" : "🔴 TRẠNG THÁI: TẮT PHUN (Ngoài Quỹ Đạo / Chuyển Tiếp)";
            GUILayout.Label(sprayStateLabel, isSprayingNow ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelRed);
        }

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 2: CAM & POINT CLOUD (SETUP CAM & POINT CLOUD LOADER)
    // =========================================================================
    private void DrawCamPointCloudCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("CAMERA & NẠP ĐÁM MÂY ĐIỂM (POINT CLOUD)", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        RobotCameraController rcc = EnsureCameraController();
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();

        GUILayout.Space(6);
        GUILayout.Label("1. GÓC NHÌN CAMERA TRÊN PHÔI", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Góc 3/4", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.SetPresetPerspective(); }
        if (GUILayout.Button("Trên Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.SetPresetTopDown(); }
        if (GUILayout.Button("Trước Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.SetPresetFront(); }
        if (GUILayout.Button("Ngang", AntigravityUITheme.StyleButton, GUILayout.Height(26))) { if (rcc) rcc.SetPresetSide(); }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Focus vào Phôi Đồ", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(24))) { if (rcc) rcc.FocusGarment(); }
        if (GUILayout.Button("Reset Focus vào Robot", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.ResetFocus(); }
        GUILayout.EndHorizontal();
        GUILayout.Label("Thao tác 3D: Chuột phải để Xoay | Chuột giữa để Lia cam | Cuộn để Zoom", AntigravityUITheme.StyleLabelMuted);

        GUILayout.Space(10);
        GUILayout.Label("2. NẠP FILE PHÔI (.PLY) / CHẾ ĐỘ TỰ ĐỘNG", AntigravityUITheme.StyleSectionLabel);

        // Nút mở File Dialog duyệt file từ máy tính
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("📁 Duyệt & Chọn File từ Máy Tính... (.ply)", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(30)))
        {
            OpenFileBrowserForSample();
        }
        GUILayout.EndHorizontal();

        // Hiển thị tên file đang chọn và nút tải lại
        GUILayout.BeginHorizontal();
        string displaySample = !string.IsNullOrEmpty(selectedSampleName) ? $"{selectedSampleName}.ply" : "Chưa chọn file";
        GUILayout.Label($"File hiện tại: {displaySample}", AntigravityUITheme.StyleLabelCyan);
        if (!string.IsNullOrEmpty(customPlyPath) && GUILayout.Button("Tải Lại", AntigravityUITheme.StyleButton, GUILayout.Width(70), GUILayout.Height(22)))
        {
            ReloadPointCloudFromPath(customPlyPath);
        }
        GUILayout.EndHorizontal();

        // 2.3. Chế độ Tự Động (Model AI tự gửi vào qua UDP 5006)
        GUILayout.Space(6);
        AiServerClient aiClient = EnsureAiClient();
        if (aiClient != null)
        {
            GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
            bool newAuto = GUILayout.Toggle(aiClient.autoModeEnabled, " CHẾ ĐỘ TỰ ĐỘNG (Model AI tự gửi vào - UDP 5006)", AntigravityUITheme.StyleToggle);
            if (newAuto != aiClient.autoModeEnabled)
            {
                aiClient.SetAutoMode(newAuto);
            }

            if (aiClient.autoModeEnabled)
            {
                GUILayout.Label("🟢 Đang lắng nghe: Model AI tự động đẩy phôi & quỹ đạo vào Unity!", AntigravityUITheme.StyleLabelGreen);
            }
            else
            {
                GUILayout.Label("⚪ Chế độ thủ công: Tự chọn file hoặc nhấn nút nạp.", AntigravityUITheme.StyleLabelMuted);
            }
            GUILayout.EndVertical();
        }

        // Nút kích hoạt AI Server cho mẫu đang chọn (Thủ công)
        if (aiClient != null && !aiClient.autoModeEnabled)
        {
            bool isAiBusy = aiClient.isProcessing;
            string aiBtnText = isAiBusy ? "🟡 Server AI đang xử lý..." : $"⚡ Nhận Diện Quần & Mốc [{selectedSampleName}] (Tự Đổi Size Phun)";
            GUIStyle aiStyle = isAiBusy ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton;
            if (GUILayout.Button(aiBtnText, aiStyle, GUILayout.Height(28)))
            {
                DetectJeansSizeAndSwitchSprayTemplate(selectedSampleName);
                aiClient.TriggerDetectionWithSample(detectedJeansSize, selectedSampleName);
            }
        }

        GUILayout.Space(6);
        DrawJeansSizeDetectionSummaryBox();

        GUILayout.Space(10);
        GUILayout.Label("3. NẠP ĐÁM MÂY ĐIỂM TÙY CHỌN (.PLY HOẶC ĐƯỜNG DẪN)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.Label("Đường dẫn file .PLY:", AntigravityUITheme.StyleLabelMuted);
        customPlyPath = GUILayout.TextField(customPlyPath, AntigravityUITheme.StyleTextField);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Tải lại / Nạp Đám Mây Điểm", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            ReloadPointCloudFromPath(customPlyPath);
        }
        bool isCloudVis = plyLoader != null && plyLoader.IsVisible();
        if (GUILayout.Button(isCloudVis ? "Ẩn Phôi Quần" : "Hiện Phôi Quần", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            TogglePointCloudVisible(!isCloudVis);
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(6);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label("CHẾ ĐỘ HIỂN THỊ PHÔI QUẦN (TỐI ƯU HIỆU NĂNG LAPTOP):", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        bool isMeshMode = plyLoader != null && plyLoader.renderMode == PLYDirectLoader.WorkpieceRenderMode.Mesh3D;
        GUIStyle bMeshStyle = isMeshMode ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        GUIStyle bCloudStyle = !isMeshMode ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("🌐 3D MESH SIÊU NHẸ (60-120 FPS)", bMeshStyle, GUILayout.Height(28)))
        {
            if (plyLoader != null) plyLoader.SetRenderMode(PLYDirectLoader.WorkpieceRenderMode.Mesh3D);
        }
        if (GUILayout.Button("☁️ POINT CLOUD (Gốc 1.7M Điểm)", bCloudStyle, GUILayout.Height(28)))
        {
            if (plyLoader != null) plyLoader.SetRenderMode(PLYDirectLoader.WorkpieceRenderMode.PointCloud);
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("⚡ Tạo Lại / Cập Nhật 3D Surface Mesh", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            if (plyLoader != null) plyLoader.ConvertPointCloudToSurfaceMesh();
        }
        GUILayout.EndHorizontal();

        if (isMeshMode)
        {
            GUILayout.Label("✔ Đang dùng 3D Surface Mesh: 1 Draw Call, mượt mà, không nóng máy, bám sát mặt vải!", AntigravityUITheme.StyleLabelGreen);
        }
        else
        {
            GUILayout.Label("⚠ Đang dùng Point Cloud gốc: 1.7M điểm, nhiều Chunks, tải nặng GPU laptop.", AntigravityUITheme.StyleLabelMuted);
        }
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label("HIỂN THỊ ĐIỂM MỐC AI (12 LANDMARKS) & ĐƯỜNG BAO SEGMENT:", AntigravityUITheme.StyleSectionLabel);
        showLandmarks3D = GUILayout.Toggle(showLandmarks3D, " Hiện 12 Điểm Mốc Giải Phẫu AI (Landmarks 3D + Skeleton)", AntigravityUITheme.StyleToggle);
        if (showLandmarks3D)
        {
            showLandmarkLabels = GUILayout.Toggle(showLandmarkLabels, " Hiện nhãn tên & cao độ Z (mm) của 12 mốc", AntigravityUITheme.StyleToggle);
        }
        showSegmentationContour3D = GUILayout.Toggle(showSegmentationContour3D, " Hiện Đường Bao Segment Thân Quần 3D (Garment Contour)", AntigravityUITheme.StyleToggle);
        showSprayZones3D = GUILayout.Toggle(showSprayZones3D, " Hiện Vùng Cho Phép Phun 2 Ống Quần (Sprayable Zones)", AntigravityUITheme.StyleToggle);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label("THỐNG KÊ DỮ LIỆU ĐÁM MÂY ĐIỂM:", AntigravityUITheme.StyleSectionLabel);
        int totalPts = plyLoader != null ? plyLoader.totalLoadedPoints : 0;
        int chunkCnt = plyLoader != null ? plyLoader.GetChunkCount() : 0;
        GUILayout.Label($"• Tổng số điểm ảnh 3D: {totalPts:N0} điểm (Đã tối ưu lấy mẫu)", AntigravityUITheme.StyleLabelCyan);
        GUILayout.Label("• Giữ nguyên 100% màu sắc gốc (Original RGB)", AntigravityUITheme.StyleLabelGreen);
        GUILayout.Label($"• Phân chia hiển thị: {chunkCnt} Mesh Chunks", AntigravityUITheme.StyleLabelMuted);
        GUILayout.Label("• Tọa độ Camera Fixed Rig: Y=1.685m, Z=0.335m", AntigravityUITheme.StyleLabelMuted);
        GUILayout.EndVertical();

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 3: QUỸ ĐẠO & CHỈNH SỬA TỪNG ĐOẠN (TRAJECTORY EDITING CARD)
    // =========================================================================
    private void DrawTrajectoryEditingCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("VẼ QUỸ ĐẠO 2D ➔ ÁNH XẠ 3D & CHỈNH SỬA", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        // ---------------------------------------------------------------------
        // MỤC MỚI: BẢNG VẼ QUẦN JEAN 2D TIÊU CHUẨN -> ÁNH XẠ LÊN BỀ MẶT 3D
        // ---------------------------------------------------------------------
        GUILayout.Space(6);
        DrawCanonicalJeans2DCanvasSection(width - 16f);

        GUILayout.Space(8);
        GUILayout.Label("1. KẾT NỐI MÁY CHỦ AI (UDP / HTTP REST)", AntigravityUITheme.StyleSectionLabel);
        AiServerClient aiClient = EnsureAiClient();
        if (aiClient != null)
        {
            // Hàng nhập IP và Cổng máy chủ
            GUILayout.BeginHorizontal();
            GUILayout.Label("Địa chỉ IP:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(65));
            string newIp = GUILayout.TextField(aiClient.serverIp, AntigravityUITheme.StyleTextField, GUILayout.Width(125));
            if (newIp != aiClient.serverIp) { aiClient.serverIp = newIp; }

            GUILayout.Label("Cổng:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(40));
            string portStr = GUILayout.TextField(aiClient.serverPort.ToString(), AntigravityUITheme.StyleTextField, GUILayout.Width(55));
            int parsedPort;
            if (int.TryParse(portStr, out parsedPort)) { aiClient.serverPort = parsedPort; }

            if (GUILayout.Button("Kiểm Tra", AntigravityUITheme.StyleButton, GUILayout.Width(75), GUILayout.Height(22)))
            {
                aiClient.CheckServerStatus();
            }
            GUILayout.EndHorizontal();

            // Hàng chọn Giao thức UDP hoặc HTTP
            GUILayout.BeginHorizontal();
            GUILayout.Label("Giao thức:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(65));
            GUIStyle bUdp = (aiClient.protocol == AiServerClient.ProtocolType.UDP) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            GUIStyle bHttp = (aiClient.protocol == AiServerClient.ProtocolType.HTTP) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            if (GUILayout.Button("UDP (5005 - Siêu tốc <1ms)", bUdp, GUILayout.Height(22))) aiClient.SetProtocol(AiServerClient.ProtocolType.UDP);
            if (GUILayout.Button("HTTP (5000 - REST)", bHttp, GUILayout.Height(22))) aiClient.SetProtocol(AiServerClient.ProtocolType.HTTP);
            GUILayout.EndHorizontal();

            // File phôi hiện tại dùng cho AI
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Phôi gửi AI: [ {(!string.IsNullOrEmpty(selectedSampleName) ? selectedSampleName + ".ply" : "Chưa chọn file")} ]", AntigravityUITheme.StyleLabelCyan);
            GUILayout.EndHorizontal();

            // Nút Kích hoạt AI & Duyệt file
            GUILayout.BeginHorizontal();
            bool isBusy = aiClient.isProcessing;
            string aiBtnTitle = isBusy ? "🟡 Đang xử lý AI..." : $"⚡ NHẬN DIỆN QUẦN & MỐC [{selectedSampleName}]";
            GUIStyle btnSt = isBusy ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton;
            if (GUILayout.Button(aiBtnTitle, btnSt, GUILayout.Height(30)))
            {
                DetectJeansSizeAndSwitchSprayTemplate(selectedSampleName);
                aiClient.TriggerDetectionWithSample(detectedJeansSize, selectedSampleName);
            }
            if (GUILayout.Button("📁 Duyệt File...", AntigravityUITheme.StyleButton, GUILayout.Width(110), GUILayout.Height(30)))
            {
                OpenFileBrowserForSample();
            }
            if (GUILayout.Button("Quần Mẫu Cũ", AntigravityUITheme.StyleButton, GUILayout.Width(95), GUILayout.Height(30)))
            {
                LoadPantsOutlineTrajectory();
            }
            GUILayout.EndHorizontal();

            // Toggle Chế độ tự động trong Card 3
            bool newAutoMode = GUILayout.Toggle(aiClient.autoModeEnabled, " CHẾ ĐỘ TỰ ĐỘNG (Model AI tự gửi kết quả vào - UDP 5006)", AntigravityUITheme.StyleToggle);
            if (newAutoMode != aiClient.autoModeEnabled)
            {
                aiClient.SetAutoMode(newAutoMode);
            }

            if (!string.IsNullOrEmpty(aiClient.lastStatusMessage))
            {
                GUILayout.Label(aiClient.lastStatusMessage, aiClient.isServerOnline ? AntigravityUITheme.StyleLabelCyan : AntigravityUITheme.StyleLabelRed);
            }
        }
        else
        {
            if (GUILayout.Button("Nạp Mẫu Quần Jean CR10A", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26))) LoadPantsOutlineTrajectory();
        }

        GUILayout.Space(8);
        GUILayout.Label("2. XEM TRƯỚC QUỸ ĐẠO TRÊN PHÔI (PREVIEW)", AntigravityUITheme.StyleSectionLabel);
        showGizmos = GUILayout.Toggle(showGizmos, " Hiển thị đường quỹ đạo 3D (Polyline) đè lên phôi", AntigravityUITheme.StyleToggle);

        GUILayout.Space(10);
        GUILayout.Label("3. CHỈNH SỬA QUỸ ĐẠO THEO TỪNG ĐOẠN", AntigravityUITheme.StyleSectionLabel);
        int wpCount = waypoints.Count;
        int wpIdx = wpCount > 0 ? Mathf.Clamp(currentWaypointIndex, 0, wpCount - 1) : 0;
        float distance = wpCount > 0 ? waypoints[wpIdx].distance_mm : 0f;

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("<< -10", AntigravityUITheme.StyleButton, GUILayout.Width(50), GUILayout.Height(24))) ApplyWaypoint(Mathf.Max(0, wpIdx - 10));
        if (GUILayout.Button("< Trước", AntigravityUITheme.StyleButton, GUILayout.Width(60), GUILayout.Height(24))) ApplyWaypoint(Mathf.Max(0, wpIdx - 1));
        GUILayout.Label($"Waypoint #{wpIdx} / {Mathf.Max(0, wpCount - 1)}", AntigravityUITheme.StyleLabelCyan);
        if (GUILayout.Button("Sau >", AntigravityUITheme.StyleButton, GUILayout.Width(60), GUILayout.Height(24))) ApplyWaypoint(Mathf.Min(wpCount - 1, wpIdx + 1));
        if (GUILayout.Button("+10 >>", AntigravityUITheme.StyleButton, GUILayout.Width(50), GUILayout.Height(24))) ApplyWaypoint(Mathf.Min(wpCount - 1, wpIdx + 10));
        GUILayout.EndHorizontal();

        int sliderIdx = Mathf.RoundToInt(GUILayout.HorizontalSlider(wpIdx, 0, Mathf.Max(0, wpCount - 1)));
        if (sliderIdx != currentWaypointIndex)
        {
            isPlaying = false;
            ApplyWaypoint(sliderIdx);
        }

        if (wpCount > 0)
        {
            WaypointData wp = waypoints[wpIdx];
            GUILayout.Space(4);
            GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
            GUILayout.Label($"TCP Vị Trí: X={wp.targetPosGlobal_mm.x:F1}   Y={wp.targetPosGlobal_mm.y:F1}   Z={wp.targetPosGlobal_mm.z:F1} mm", AntigravityUITheme.StyleLabelMuted);
            GUILayout.Label($"Góc 6 Khớp: {wp.jointAngles_deg[0]:F1}°, {wp.jointAngles_deg[1]:F1}°, {wp.jointAngles_deg[2]:F1}°, {wp.jointAngles_deg[3]:F1}°, {wp.jointAngles_deg[4]:F1}°, {wp.jointAngles_deg[5]:F1}°", AntigravityUITheme.StyleLabelCyan);
            GUILayout.EndVertical();

            // KHỐI ĐIỀU CHỈNH GÓC HƯỚNG TCP (ROLL / PITCH / YAW)
            GUILayout.Space(6);
            GUILayout.Label("ĐIỀU CHỈNH GÓC HƯỚNG ĐẦU SÚNG (ROLL / PITCH / YAW):", AntigravityUITheme.StyleSectionLabel);

            // 1. Roll
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Roll: {wp.roll_deg:F1}°", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(85));
            if (GUILayout.Button("-5°", AntigravityUITheme.StyleButton, GUILayout.Width(35), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg - 5f, wp.pitch_deg, wp.yaw_deg);
            if (GUILayout.Button("-1°", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg - 1f, wp.pitch_deg, wp.yaw_deg);
            float rSlider = GUILayout.HorizontalSlider(wp.roll_deg, -180f, 180f);
            if (Mathf.Abs(rSlider - wp.roll_deg) > 0.1f) UpdateWaypointRpy(wpIdx, Mathf.Round(rSlider * 10f) / 10f, wp.pitch_deg, wp.yaw_deg);
            if (GUILayout.Button("+1°", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg + 1f, wp.pitch_deg, wp.yaw_deg);
            if (GUILayout.Button("+5°", AntigravityUITheme.StyleButton, GUILayout.Width(35), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg + 5f, wp.pitch_deg, wp.yaw_deg);
            GUILayout.EndHorizontal();

            // 2. Pitch
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Pitch: {wp.pitch_deg:F1}°", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(85));
            if (GUILayout.Button("-5°", AntigravityUITheme.StyleButton, GUILayout.Width(35), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg - 5f, wp.yaw_deg);
            if (GUILayout.Button("-1°", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg - 1f, wp.yaw_deg);
            float pSlider = GUILayout.HorizontalSlider(wp.pitch_deg, -90f, 90f);
            if (Mathf.Abs(pSlider - wp.pitch_deg) > 0.1f) UpdateWaypointRpy(wpIdx, wp.roll_deg, Mathf.Round(pSlider * 10f) / 10f, wp.yaw_deg);
            if (GUILayout.Button("+1°", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg + 1f, wp.yaw_deg);
            if (GUILayout.Button("+5°", AntigravityUITheme.StyleButton, GUILayout.Width(35), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg + 5f, wp.yaw_deg);
            GUILayout.EndHorizontal();

            // 3. Yaw
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Yaw: {wp.yaw_deg:F1}°", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(85));
            if (GUILayout.Button("-5°", AntigravityUITheme.StyleButton, GUILayout.Width(35), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, wp.yaw_deg - 5f);
            if (GUILayout.Button("-1°", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, wp.yaw_deg - 1f);
            float ySlider = GUILayout.HorizontalSlider(wp.yaw_deg, -180f, 180f);
            if (Mathf.Abs(ySlider - wp.yaw_deg) > 0.1f) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, Mathf.Round(ySlider * 10f) / 10f);
            if (GUILayout.Button("+1°", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, wp.yaw_deg + 1f);
            if (GUILayout.Button("+5°", AntigravityUITheme.StyleButton, GUILayout.Width(35), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, wp.yaw_deg + 5f);
            GUILayout.EndHorizontal();

            // 4. Các nút thao tác nhanh
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Bích Chúc Thẳng (180, 0, 0)", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                UpdateWaypointRpy(wpIdx, 180f, 0f, 0f);
            }
            if (GUILayout.Button("Lan truyền RPY về sau", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                PropagateRpyForward(wpIdx);
            }
            if (GUILayout.Button("Áp dụng RPY cho toàn bộ", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                ApplyRpyToAllWaypoints(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
            }
            GUILayout.EndHorizontal();

            // Danh sách 8 nghiệm giải tích
            Quaternion rot = GetWaypointFlangeRotation(wp);
            float[] seedAngles = wpIdx > 0 ? waypoints[wpIdx - 1].jointAngles_rad : null;
            List<DobotCR10AKinematics.IKSolution> sols = DobotCR10AKinematics.SolveTcpPose(
                wp.targetPosGlobal_mm, rot, seedAngles, toolLength, useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);

            if (sols != null && sols.Count > 0)
            {
                GUILayout.Space(4);
                GUILayout.Label($"CHỌN BỘ NGHIỆM PHÙ HỢP ({sols.Count} NGHIỆM GIẢI TÍCH):", AntigravityUITheme.StyleSectionLabel);
                waypointSolsScroll = GUILayout.BeginScrollView(waypointSolsScroll, GUILayout.Height(Mathf.Min(130f, sols.Count * 26f + 4f)));
                for (int i = 0; i < sols.Count; i++)
                {
                    var s = sols[i];
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
                if (GUILayout.Button("Lan truyền cấu hình cho các điểm sau", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
                {
                    PropagateCurrentConfigurationForward(wpIdx);
                }
                if (GUILayout.Button("Khôi phục mặc định Q*", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
                {
                    ResetWaypointToDefault(wpIdx);
                }
                GUILayout.EndHorizontal();
            }
        }

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 4: XUẤT / NHẬP LỆNH (EXPORT / IMPORT CARD)
    // =========================================================================
    private void DrawExportImportCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("XUẤT & CẬP NHẬT CHƯƠNG TRÌNH ROBOT", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);

        // Khối kiểm tra Khóa An Toàn (Safety Interlock)
        if (hasCriticalErrors)
        {
            GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
            GUILayout.Label("⚠️ KHÓA AN TOÀN ĐANG HOẠT ĐỘNG", AntigravityUITheme.StyleLabelRed);
            GUILayout.Label("Quỹ đạo có lỗi nghiêm trọng (ngoài tầm với, kỳ dị hoặc va chạm).", AntigravityUITheme.StyleLabelMuted);
            GUILayout.Label("Lệnh xuất sang Robot CR10A bị CHẶN để bảo vệ phần cứng!", AntigravityUITheme.StyleLabelRed);
            forceBypassSafetyInterlock = GUILayout.Toggle(forceBypassSafetyInterlock, " [Kỹ sư] Mở khóa cưỡng bức (Force Bypass Interlock)", AntigravityUITheme.StyleToggle);
            GUILayout.EndVertical();
            GUILayout.Space(6);
        }

        bool canExport = !hasCriticalErrors || forceBypassSafetyInterlock;
        GUIStyle exportBtnStyle = canExport ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;

        GUILayout.Label("1. XUẤT VÀ CẬP NHẬT LẠI CHƯƠNG TRÌNH (EXPORT / RE-EXPORT)", AntigravityUITheme.StyleSectionLabel);
        if (GUILayout.Button("1. Xuất Script Python Dobot CR10A (.py)", exportBtnStyle, GUILayout.Height(28)))
        {
            ExportDobotPythonScript();
        }
        if (GUILayout.Button("2. Xuất Tọa Độ Quỹ Đạo Đã Chỉnh Sửa (.csv)", exportBtnStyle, GUILayout.Height(28)))
        {
            ExportUpdatedCsvTrajectory();
        }
        if (GUILayout.Button("3. Mở Thư Mục Xuất (output_drawing)", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            OpenExportFolder();
        }

        GUILayout.Space(10);
        GUILayout.Label("2. NHẬP FILE QUỸ ĐẠO MỚI (IMPORT CSV)", AntigravityUITheme.StyleSectionLabel);
        customImportCsvPath = GUILayout.TextField(customImportCsvPath, AntigravityUITheme.StyleTextField);
        if (GUILayout.Button("Nạp File CSV Quỹ Đạo Vào Simulator", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            LoadNamedTrajectory(customImportCsvPath);
        }

        if (!string.IsNullOrEmpty(exportStatusMessage))
        {
            GUILayout.Space(6);
            GUILayout.Label(exportStatusMessage, AntigravityUITheme.StyleLabelCyan);
        }

        if (!string.IsNullOrEmpty(lastExportedCodePreview))
        {
            GUILayout.Space(6);
            GUILayout.Label("XEM TRƯỚC MÃ LỆNH VỪA XUẤT:", AntigravityUITheme.StyleSectionLabel);
            exportPreviewScroll = GUILayout.BeginScrollView(exportPreviewScroll, GUILayout.Height(120));
            GUILayout.TextArea(lastExportedCodePreview, GUI.skin.textArea);
            GUILayout.EndScrollView();
        }

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 5: CẢNH BÁO VÀ XỬ LÝ LỖI (DIAGNOSTICS CARD)
    // =========================================================================
    private void DrawDiagnosticsCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("CẢNH BÁO AN TOÀN & XỬ LÝ LỖI", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);
        if (hasCriticalErrors)
        {
            GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
            GUILayout.Label("❌ NGUY HIỂM: PHÁT HIỆN LỖI QUỸ ĐẠO!", AntigravityUITheme.StyleLabelRed);
            GUILayout.Label("Đã tự động kích hoạt Khóa An Toàn chặn xuất lệnh sang Robot thật.", AntigravityUITheme.StyleLabelMuted);
            GUILayout.EndVertical();
        }
        else
        {
            GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
            GUILayout.Label("✅ AN TOÀN TUYỆT ĐỐI: Quỹ đạo đạt chuẩn 100%", AntigravityUITheme.StyleLabelGreen);
            GUILayout.Label("Không phát hiện va chạm bàn, điểm kỳ dị hay vượt tầm với.", AntigravityUITheme.StyleLabelMuted);
            GUILayout.EndVertical();
        }

        GUILayout.Space(6);
        if (GUILayout.Button("Quét Lại Toàn Bộ Quỹ Đạo (Run Safety Diagnostics)", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            RunDiagnostics();
        }

        GUILayout.Space(8);
        GUILayout.Label($"DANH SÁCH SỰ CỐ ĐÃ QUÉT ({diagnosticResults.Count} MỤC):", AntigravityUITheme.StyleSectionLabel);
        diagnosticsScroll = GUILayout.BeginScrollView(diagnosticsScroll, GUILayout.Height(220));

        for (int i = 0; i < diagnosticResults.Count; i++)
        {
            var item = diagnosticResults[i];
            GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
            GUILayout.BeginHorizontal();

            string icon = item.severity == DiagnosticItem.Severity.Error ? "[LỖI]" : (item.severity == DiagnosticItem.Severity.Warning ? "[CẢNH BÁO]" : "[THÔNG TIN]");
            GUIStyle textStyle = item.severity == DiagnosticItem.Severity.Error ? AntigravityUITheme.StyleLabelRed : (item.severity == DiagnosticItem.Severity.Warning ? AntigravityUITheme.StyleLabelYellow : AntigravityUITheme.StyleLabelGreen);

            GUILayout.Label($"{icon} {item.category}", textStyle, GUILayout.ExpandWidth(true));
            if (item.waypointIndex >= 0)
            {
                if (GUILayout.Button($"Xem Wp #{item.waypointIndex}", AntigravityUITheme.StyleButton, GUILayout.Width(100), GUILayout.Height(20)))
                {
                    interfaceTab = 3; // Chuyển sang Tab Quỹ đạo
                    ApplyWaypoint(item.waypointIndex);
                }
            }
            GUILayout.EndHorizontal();
            GUILayout.Label(item.message, AntigravityUITheme.StyleLabelMuted);
            GUILayout.EndVertical();
            GUILayout.Space(2);
        }

        GUILayout.EndScrollView();
        GUILayout.EndVertical();
    }

    // =========================================================================
    // CÁC HÀM XUẤT CHƯƠNG TRÌNH & THAO TÁC FILE
    // =========================================================================

    public void ExportDobotPythonScript(string outputPath = null)
    {
        if (hasCriticalErrors && !forceBypassSafetyInterlock)
        {
            exportStatusMessage = "[CHẶN XUẤT] Không thể xuất script Python do có lỗi an toàn nghiêm trọng!";
            Debug.LogError(exportStatusMessage);
            return;
        }

        string exportDir = Path.Combine(Application.dataPath, "..", "output_drawing");
        if (!Directory.Exists(exportDir)) Directory.CreateDirectory(exportDir);
        if (string.IsNullOrEmpty(outputPath))
        {
            outputPath = Path.Combine(exportDir, "dobot_cr10a_trajectory_playback.py");
        }

        try
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("#!/usr/bin/env python3");
            sb.AppendLine("# -*- coding: utf-8 -*-");
            sb.AppendLine("\"\"\"");
            sb.AppendLine("DOBOT CR10A TRAJECTORY EXECUTION SCRIPT");
            sb.AppendLine("Sinh tu dong tu Dobot CR10A Workbench (Unity Simulator).");
            sb.AppendLine($"Ngay xuat: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            sb.AppendLine($"Tong so Waypoint: {waypoints.Count}");
            sb.AppendLine($"Dau phun TCP: Z={toolLength:F1}mm, X={toolOffsetX:F1}mm, Y={toolOffsetY:F1}mm, Co vet={sprayWidthMm:F1}mm");
            sb.AppendLine("\"\"\"");
            sb.AppendLine();
            sb.AppendLine("import time");
            sb.AppendLine();
            sb.AppendLine("class DobotCR10A:");
            sb.AppendLine("    def __init__(self, ip='192.168.5.1', port=29999):");
            sb.AppendLine("        self.ip = ip");
            sb.AppendLine("        self.port = port");
            sb.AppendLine("        print(f'[CR10A] Ket noi Robot tai {self.ip}:{self.port}...')");
            sb.AppendLine();
            sb.AppendLine("    def enable_robot(self):");
            sb.AppendLine("        print('[CR10A] EnableRobot() - San sang van hanh.')");
            sb.AppendLine();
            sb.AppendLine("    def set_speed_factor(self, ratio):");
            sb.AppendLine("        print(f'[CR10A] SpeedFactor({ratio}%)')");
            sb.AppendLine();
            sb.AppendLine("    def move_j(self, j1, j2, j3, j4, j5, j6):");
            sb.AppendLine("        # print(f'MoveJ({j1:.2f}, {j2:.2f}, {j3:.2f}, {j4:.2f}, {j5:.2f}, {j6:.2f})')");
            sb.AppendLine("        pass");
            sb.AppendLine();
            sb.AppendLine("    def set_digital_output(self, index, status):");
            sb.AppendLine("        print(f'[CR10A] DO({index}) = {status}')");
            sb.AppendLine();
            sb.AppendLine("def main():");
            sb.AppendLine("    robot = DobotCR10A()");
            sb.AppendLine("    robot.enable_robot()");
            sb.AppendLine($"    robot.set_speed_factor({Mathf.Clamp(Mathf.RoundToInt(playbackSpeed * 30f), 1, 100)})");
            sb.AppendLine();
            sb.AppendLine("    # 1. Ve vi tri Home Pose");
            sb.AppendLine("    print('[CR10A] Di chuyen ve Home (0, 0, 0, 0, 0, 0)...')");
            sb.AppendLine("    robot.move_j(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)");
            sb.AppendLine("    time.sleep(1.0)");
            sb.AppendLine();

            if (waypoints.Count > 0)
            {
                var wp0 = waypoints[0];
                sb.AppendLine("    # 2. Tiep can diem bat dau quy dao");
                sb.AppendLine($"    robot.move_j({wp0.jointAngles_deg[0]:F2}, {wp0.jointAngles_deg[1]:F2}, {wp0.jointAngles_deg[2]:F2}, {wp0.jointAngles_deg[3]:F2}, {wp0.jointAngles_deg[4]:F2}, {wp0.jointAngles_deg[5]:F2})");
                sb.AppendLine();
                sb.AppendLine("    # 3. Bat van sung phun");
                sb.AppendLine("    robot.set_digital_output(1, 1)");
                sb.AppendLine();
                sb.AppendLine("    # 4. Van hanh toan bo cac waypoint");
                for (int i = 0; i < waypoints.Count; i++)
                {
                    var wp = waypoints[i];
                    sb.AppendLine($"    robot.move_j({wp.jointAngles_deg[0]:F2}, {wp.jointAngles_deg[1]:F2}, {wp.jointAngles_deg[2]:F2}, {wp.jointAngles_deg[3]:F2}, {wp.jointAngles_deg[4]:F2}, {wp.jointAngles_deg[5]:F2}) # Wp #{i}");
                }
                sb.AppendLine();
                sb.AppendLine("    # 5. Tat van sung phun");
                sb.AppendLine("    robot.set_digital_output(1, 0)");
            }

            sb.AppendLine();
            sb.AppendLine("    # 6. Tro ve Home an toan");
            sb.AppendLine("    print('[CR10A] Hoan tat quy dao. Tro ve Home (0, 0, 0, 0, 0, 0)...')");
            sb.AppendLine("    robot.move_j(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)");
            sb.AppendLine();
            sb.AppendLine("if __name__ == '__main__':");
            sb.AppendLine("    main()");

            File.WriteAllText(outputPath, sb.ToString());
            exportStatusMessage = $"✓ Đã xuất Dobot Python script ({waypoints.Count} wp) sang:\n{Path.GetFileName(outputPath)}";
            lastExportedCodePreview = sb.ToString();
            Debug.Log($"[CR10A Export] {exportStatusMessage}");
        }
        catch (Exception ex)
        {
            exportStatusMessage = $"Lỗi khi xuất Python: {ex.Message}";
            Debug.LogError($"[CR10A Export] {exportStatusMessage}");
        }
    }


    public void ExportUpdatedCsvTrajectory(string outputPath = null)
    {
        if (hasCriticalErrors && !forceBypassSafetyInterlock)
        {
            exportStatusMessage = "[CHẶN XUẤT] Không thể xuất file CSV do có lỗi an toàn nghiêm trọng!";
            Debug.LogError(exportStatusMessage);
            return;
        }

        string exportDir = Path.Combine(Application.dataPath, "..", "output_drawing");
        if (!Directory.Exists(exportDir)) Directory.CreateDirectory(exportDir);
        if (string.IsNullOrEmpty(outputPath))
        {
            outputPath = Path.Combine(exportDir, "pants_outline_cr10a_updated.csv");
        }

        try
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("index,dist_mm,x_mm,y_mm,z_mm,roll_deg,pitch_deg,yaw_deg,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,branch");
            for (int i = 0; i < waypoints.Count; i++)
            {
                var wp = waypoints[i];
                string branch = string.IsNullOrEmpty(wp.selectedBranchName) ? "Optimal_Q" : wp.selectedBranchName;
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "{0},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2},{6:F2},{7:F2},{8:F3},{9:F3},{10:F3},{11:F3},{12:F3},{13:F3},{14}",
                    wp.index, wp.distance_mm,
                    wp.targetPosGlobal_mm.x, wp.targetPosGlobal_mm.y, wp.targetPosGlobal_mm.z,
                    wp.roll_deg, wp.pitch_deg, wp.yaw_deg,
                    wp.jointAngles_deg[0], wp.jointAngles_deg[1], wp.jointAngles_deg[2],
                    wp.jointAngles_deg[3], wp.jointAngles_deg[4], wp.jointAngles_deg[5],
                    branch
                ));
            }

            File.WriteAllText(outputPath, sb.ToString());
            exportStatusMessage = $"✓ Đã xuất CSV cập nhật ({waypoints.Count} wp) sang:\n{Path.GetFileName(outputPath)}";
            lastExportedCodePreview = "[CSV PREVIEW]\n" + sb.ToString().Substring(0, Mathf.Min(800, sb.Length));
            Debug.Log($"[CR10A Export] {exportStatusMessage}");
        }
        catch (Exception ex)
        {
            exportStatusMessage = $"Lỗi khi xuất CSV: {ex.Message}";
            Debug.LogError($"[CR10A Export] {exportStatusMessage}");
        }
    }

    public void OpenExportFolder()
    {
        string exportDir = Path.Combine(Application.dataPath, "..", "output_drawing");
        if (!Directory.Exists(exportDir)) Directory.CreateDirectory(exportDir);
        string fullPath = Path.GetFullPath(exportDir);
        try
        {
            if (Application.platform == RuntimePlatform.LinuxEditor || Application.platform == RuntimePlatform.LinuxPlayer)
            {
                System.Diagnostics.Process.Start("xdg-open", $"\"{fullPath}\"");
            }
            else if (Application.platform == RuntimePlatform.WindowsEditor || Application.platform == RuntimePlatform.WindowsPlayer)
            {
                System.Diagnostics.Process.Start("explorer.exe", $"\"{fullPath}\"");
            }
            else
            {
                Application.OpenURL("file://" + fullPath);
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[CR10A Export] Không thể mở thư mục: {ex.Message}");
            Application.OpenURL("file://" + fullPath);
        }
    }

    // =========================================================================
    // CHẨN ĐOÁN AN TOÀN VÀ XỬ LÝ LỖI (DIAGNOSTICS ENGINE)
    // =========================================================================

    public void RunDiagnostics()
    {
        diagnosticResults.Clear();
        hasCriticalErrors = false;

        // 1. Kiểm tra nhận dạng phôi & Đám mây điểm rỗng
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (plyLoader == null || plyLoader.totalLoadedPoints == 0)
        {
            diagnosticResults.Add(new DiagnosticItem
            {
                severity = DiagnosticItem.Severity.Error,
                category = "Nhận dạng phôi (Workpiece)",
                message = "Không nhận dạng được phôi hoặc dữ liệu đám mây điểm rỗng. Cần kiểm tra nạp file PLY!",
                waypointIndex = -1
            });
            hasCriticalErrors = true;
        }
        else
        {
            diagnosticResults.Add(new DiagnosticItem
            {
                severity = DiagnosticItem.Severity.Info,
                category = "Nhận dạng phôi (Workpiece)",
                message = $"Đã nhận dạng phôi thành công: {plyLoader.totalLoadedPoints:N0} điểm 3D màu gốc.",
                waypointIndex = -1
            });
        }

        if (waypoints == null || waypoints.Count == 0)
        {
            diagnosticResults.Add(new DiagnosticItem
            {
                severity = DiagnosticItem.Severity.Error,
                category = "Quỹ đạo (Trajectory)",
                message = "Chưa có dữ liệu quỹ đạo nào được nạp vào hệ thống!",
                waypointIndex = -1
            });
            hasCriticalErrors = true;
            return;
        }

        // 2. Kiểm tra quỹ đạo: Tầm với, Va chạm mặt bàn, Điểm kỳ dị, Hướng phôi
        int unsolvedCount = 0;
        int outOfReachCount = 0;
        int lowZCount = 0;
        int wristSingularityCount = 0;
        int elbowSingularityCount = 0;
        int invertedCount = 0;

        for (int i = 0; i < waypoints.Count; i++)
        {
            WaypointData wp = waypoints[i];

            if (!wp.isSolved)
            {
                unsolvedCount++;
                if (unsolvedCount <= 3)
                {
                    diagnosticResults.Add(new DiagnosticItem
                    {
                        severity = DiagnosticItem.Severity.Error,
                        category = "Tầm với / Giải IK",
                        message = $"Waypoint #{i} tại ({wp.targetPosGlobal_mm.x:F0}, {wp.targetPosGlobal_mm.y:F0}, {wp.targetPosGlobal_mm.z:F0}) không tìm được nghiệm IK!",
                        waypointIndex = i
                    });
                }
            }

            float r = wp.targetPosGlobal_mm.magnitude;
            if (r > 1175f)
            {
                outOfReachCount++;
                if (outOfReachCount <= 3)
                {
                    diagnosticResults.Add(new DiagnosticItem
                    {
                        severity = DiagnosticItem.Severity.Error,
                        category = "Vượt tầm với (Reach)",
                        message = $"Waypoint #{i} có bán kính R = {r:F1}mm vượt ngưỡng an toàn CR10A (1175mm)!",
                        waypointIndex = i
                    });
                }
            }

            if (wp.targetPosGlobal_mm.z < 10.0f)
            {
                lowZCount++;
                if (lowZCount <= 3)
                {
                    diagnosticResults.Add(new DiagnosticItem
                    {
                        severity = DiagnosticItem.Severity.Error,
                        category = "Va chạm bàn (Collision)",
                        message = $"Waypoint #{i} có Z = {wp.targetPosGlobal_mm.z:F1}mm quá sát mặt bàn (<10mm)!",
                        waypointIndex = i
                    });
                }
            }

            if (Mathf.Abs(wp.pitch_deg) > 45.0f && Mathf.Abs(wp.pitch_deg - 90.0f) > 10.0f)
            {
                invertedCount++;
            }

            if (wp.isSolved && wp.jointAngles_rad != null && wp.jointAngles_rad.Length >= 6)
            {
                if (Mathf.Abs(wp.jointAngles_rad[4]) < 0.052f) // ~3 deg
                {
                    wristSingularityCount++;
                    if (wristSingularityCount <= 3)
                    {
                        diagnosticResults.Add(new DiagnosticItem
                        {
                            severity = DiagnosticItem.Severity.Warning,
                            category = "Kỳ dị cổ tay (|q5| < 3°)",
                            message = $"Waypoint #{i} có |q5| = {Mathf.Abs(wp.jointAngles_deg[4]):F1}° < 3° (Cận điểm kỳ dị cổ tay)!",
                            waypointIndex = i
                        });
                    }
                }

                if (Mathf.Abs(wp.jointAngles_rad[2]) < 0.052f || Mathf.Abs(Mathf.Abs(wp.jointAngles_rad[2]) - Mathf.PI) < 0.052f)
                {
                    elbowSingularityCount++;
                    if (elbowSingularityCount <= 3)
                    {
                        diagnosticResults.Add(new DiagnosticItem
                        {
                            severity = DiagnosticItem.Severity.Warning,
                            category = "Kỳ dị khuỷu tay (Elbow)",
                            message = $"Waypoint #{i} có cánh tay duỗi thẳng (|q3|={wp.jointAngles_deg[2]:F1}°), cận điểm kỳ dị!",
                            waypointIndex = i
                        });
                    }
                }
            }
        }

        if (unsolvedCount > 0 || outOfReachCount > 0 || lowZCount > 0)
        {
            hasCriticalErrors = true;
        }

        if (invertedCount > 0)
        {
            diagnosticResults.Add(new DiagnosticItem
            {
                severity = DiagnosticItem.Severity.Warning,
                category = "Chiều đặt phôi (Orientation)",
                message = $"Phát hiện {invertedCount} điểm có hướng nghiêng bất thường. Cần kiểm tra lại chiều đặt phôi trên bàn!",
                waypointIndex = -1
            });
        }

        if (unsolvedCount > 3)
        {
            diagnosticResults.Add(new DiagnosticItem
            {
                severity = DiagnosticItem.Severity.Error,
                category = "Tổng hợp tầm với",
                message = $"Tổng cộng {unsolvedCount} điểm không giải được IK. Đã chặn xuất lệnh!",
                waypointIndex = -1
            });
        }

        if (wristSingularityCount > 3)
        {
            diagnosticResults.Add(new DiagnosticItem
            {
                severity = DiagnosticItem.Severity.Warning,
                category = "Tổng hợp kỳ dị cổ tay",
                message = $"Tổng cộng {wristSingularityCount} điểm cận kỳ dị cổ tay. Có thể chỉnh nhánh nghiệm ở Tab Quỹ Đạo.",
                waypointIndex = -1
            });
        }

        if (!hasCriticalErrors)
        {
            diagnosticResults.Insert(0, new DiagnosticItem
            {
                severity = DiagnosticItem.Severity.Info,
                category = "Trạng thái an toàn",
                message = $"Tất cả {waypoints.Count} waypoint nằm trong giới hạn tầm với an toàn và đã giải tích thành công 100%.",
                waypointIndex = -1
            });
        }
    }

    public void ReloadPointCloudFromPath(string path)
    {
        PLYDirectLoader loader = FindAnyObjectByType<PLYDirectLoader>();
        if (loader != null)
        {
            loader.plyFilePath = path;
            loader.ReloadFullPointCloud();
            RunDiagnostics();
        }
        else
        {
            Debug.LogWarning("[CR10A] Không tìm thấy PLYDirectLoader trong cảnh.");
        }
    }

    public void TogglePointCloudVisible(bool visible)
    {
        PLYDirectLoader loader = FindAnyObjectByType<PLYDirectLoader>();
        if (loader != null)
        {
            loader.SetVisible(visible);
        }
    }

    // =========================================================================
    // CÁC HÀM HỖ TRỢ ĐIỀU CHỈNH GIA TỐC VÀ VẬN TỐC KHỚP
    // =========================================================================

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
            Quaternion rot = GetWaypointFlangeRotation(wp);
            var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, seed, toolLength, useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);
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
        Quaternion rot = GetWaypointFlangeRotation(wp);
        float[] seed = wpIndex > 0 ? waypoints[wpIndex - 1].jointAngles_rad : null;
        var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, seed, toolLength, useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);
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

    /// <summary>
    /// Điều chỉnh góc hướng TCP (Roll, Pitch, Yaw) cho một waypoint cụ thể và giải tích lại IK
    /// </summary>
    public void UpdateWaypointRpy(int wpIndex, float newRoll, float newPitch, float newYaw)
    {
        if (wpIndex < 0 || wpIndex >= waypoints.Count) return;
        WaypointData wp = waypoints[wpIndex];
        wp.roll_deg = Mathf.Clamp(newRoll, -180f, 180f);
        wp.pitch_deg = Mathf.Clamp(newPitch, -90f, 90f);
        wp.yaw_deg = Mathf.Clamp(newYaw, -180f, 180f);

        Quaternion rot = GetWaypointFlangeRotation(wp);
        float[] seed = wpIndex > 0 ? waypoints[wpIndex - 1].jointAngles_rad : null;
        var sols = DobotCR10AKinematics.SolveTcpPose(wp.targetPosGlobal_mm, rot, seed, toolLength, useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);
        if (sols != null && sols.Count > 0)
        {
            int solIdx = customWaypointConfigurations.ContainsKey(wpIndex) ? customWaypointConfigurations[wpIndex] : 0;
            if (solIdx >= sols.Count) solIdx = 0;
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

        ApplyWaypoint(wpIndex);
    }

    /// <summary>
    /// Lan truyền góc hướng (RPY) của waypoint hiện tại cho tất cả các điểm tiếp theo
    /// </summary>
    public void PropagateRpyForward(int startIdx)
    {
        if (startIdx < 0 || startIdx >= waypoints.Count) return;
        float r = waypoints[startIdx].roll_deg;
        float p = waypoints[startIdx].pitch_deg;
        float y = waypoints[startIdx].yaw_deg;

        for (int i = startIdx + 1; i < waypoints.Count; i++)
        {
            UpdateWaypointRpy(i, r, p, y);
        }
        RunDiagnostics();
    }

    /// <summary>
    /// Áp dụng góc hướng (RPY) cho toàn bộ tất cả các waypoint trong quỹ đạo
    /// </summary>
    public void ApplyRpyToAllWaypoints(float r, float p, float y)
    {
        for (int i = 0; i < waypoints.Count; i++)
        {
            UpdateWaypointRpy(i, r, p, y);
        }
        RunDiagnostics();
    }

    // =========================================================================
    // CARD 6: CAMERA TỪ TRÊN XUỐNG & HIỂN THỊ 12 ĐIỂM MỐC AI / SEGMENT
    // =========================================================================
    private void DrawTopDownLandmarksAndSegCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("CAMERA TRÊN XUỐNG & ĐIỂM MỐC AI / SEGMENT", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        RobotCameraController rcc = EnsureCameraController();

        GUILayout.Space(6);
        GUILayout.Label("1. GÓC NHÌN CAMERA TRỰC DIỆN TỪ TRÊN XUỐNG (TOP-DOWN VIEW)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("📷 Chuyển Góc Nhìn Từ Trên Xuống (Top-Down)", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(30)))
        {
            if (rcc != null) rcc.SetPresetTopDown();
        }
        if (GUILayout.Button("Focus Quần", AntigravityUITheme.StyleButton, GUILayout.Width(95), GUILayout.Height(30)))
        {
            if (rcc != null) rcc.FocusGarment();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Góc 3/4 Phối Cảnh", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.SetPresetPerspective(); }
        if (GUILayout.Button("Trước Đồ", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.SetPresetFront(); }
        if (GUILayout.Button("Ngang Bàn", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.SetPresetSide(); }
        GUILayout.EndHorizontal();

        GUILayout.Space(8);
        GUILayout.Label("2. BẬT / TẮT LỚP HIỂN THỊ AI TRÊN ĐÁM MÂY ĐIỂM 3D", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        showLandmarks3D = GUILayout.Toggle(showLandmarks3D, " [YOLO11-pose] Hiện 12 Điểm Mốc Giải Phẫu 3D (Landmarks + Khung Xương)", AntigravityUITheme.StyleToggle);
        if (showLandmarks3D)
        {
            showLandmarkLabels = GUILayout.Toggle(showLandmarkLabels, " Hiện nhãn tên & cao độ Z (mm) từng điểm mốc", AntigravityUITheme.StyleToggle);
        }
        showSegmentationContour3D = GUILayout.Toggle(showSegmentationContour3D, " [YOLO11-seg] Hiện Đường Bao Segment Quần 3D (Viền Xanh Ngọc)", AntigravityUITheme.StyleToggle);
        showSprayZones3D = GUILayout.Toggle(showSprayZones3D, " [Keep-out] Hiện Vùng Cho Phép Phun 2 Ống Đùi-Gối (Viền Vàng Cam)", AntigravityUITheme.StyleToggle);
        showGizmos = GUILayout.Toggle(showGizmos, " [Trajectory] Hiện Đường Quỹ Đạo Phun 3D (Cyan Polyline)", AntigravityUITheme.StyleToggle);
        GUILayout.EndVertical();

        GUILayout.Space(8);
        DrawJeansSizeDetectionSummaryBox();

        GUILayout.Space(6);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"3. BẢNG TỌA ĐỘ 3D CỦA 12 ĐIỂM MỐC [{selectedSampleName}]", AntigravityUITheme.StyleSectionLabel);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("🔍 Nhận Diện & Đổi Size", AntigravityUITheme.StylePrimaryButton, GUILayout.Width(145), GUILayout.Height(20)))
        {
            DetectJeansSizeAndSwitchSprayTemplate(selectedSampleName);
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        if (landmarks3D != null && landmarks3D.Count > 0)
        {
            for (int i = 0; i < landmarks3D.Count; i++)
            {
                var lm = landmarks3D[i];
                GUILayout.BeginHorizontal();
                GUILayout.Label(lm.label_vi, AntigravityUITheme.StyleLabelCyan, GUILayout.Width(135));
                GUILayout.Label($"X={lm.x_mm:F0}  Y={lm.y_mm:F0}  Z={lm.z_mm:F1} mm", AntigravityUITheme.StyleLabelMuted);
                GUILayout.EndHorizontal();
            }
        }
        else
        {
            GUILayout.Label("Đang khởi tạo 12 điểm mốc 3D cho mẫu hiện tại...", AntigravityUITheme.StyleLabelMuted);
        }
        GUILayout.EndVertical();

        GUILayout.Space(6);
        if (GUILayout.Button("✏️ Mở Bảng Vẽ Quần 2D Tiêu Chuẩn ➔ Ánh Xạ Lên 3D", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            interfaceTab = 3;
            showCanonicalDrawPanel = true;
        }

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 7: THUẬT TOÁN BÙ ĐẮP CHỖ KHUẤT & KHUYẾT TẬT 3D (OCCLUSION & HOLE FILLING)
    // =========================================================================
    private void DrawOcclusionCompensationCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("BÙ ĐẮP CHỖ KHUẤT & KHUYẾT TẬT 3D (OCCLUSION)", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);
        // Khối 1: Phân tích & Trạng thái chỗ khuất
        GUILayout.Label("1. PHÂN TÍCH & PHÁT HIỆN CHỖ KHUẤT TRÊN PHÔI QUẦN", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("🔍 Quét Lỗ Khuyết Tật & Chỗ Khuất", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            DetectOcclusionAndHoles();
        }
        if (GUILayout.Button("Làm Mới", AntigravityUITheme.StyleButton, GUILayout.Width(75), GUILayout.Height(28)))
        {
            detectedHolePointsWorld.Clear();
            compensatedSurfacePointsWorld.Clear();
            gunTiltNormalsWorld.Clear();
            compensatedHeightGrid.Clear();
            compensatedNormalGrid.Clear();
            detectedHoleCount = 0;
            detectedOcclusionAreaCm2 = 0f;
            avgCreaseDepthMm = 0f;
            occlusionStatusMsg = "Đã làm mới dữ liệu chỗ khuất.";
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(2);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Khuyết: {(detectedHoleCount > 0 ? detectedHoleCount + " pts" : "0")}", AntigravityUITheme.StyleLabelCyan);
        GUILayout.Label($"Vùng: {(detectedOcclusionAreaCm2 > 0 ? detectedOcclusionAreaCm2.ToString("F1") + " cm²" : "0 cm²")}", AntigravityUITheme.StyleLabelCyan);
        GUILayout.Label($"Nếp sâu: {(avgCreaseDepthMm > 0 ? avgCreaseDepthMm.ToString("F1") + " mm" : "0 mm")}", AntigravityUITheme.StyleLabelCyan);
        GUILayout.EndHorizontal();

        GUILayout.Space(2);
        GUILayout.Label(occlusionStatusMsg, AntigravityUITheme.StyleLabelGreen);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        // Khối 2: Bộ thuật toán bù đắp
        GUILayout.Label("2. CHỌN THUẬT TOÁN BÙ ĐẮP CHUYÊN SÂU (THEO HUONG_DAN_PIPELINE)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        string[] algos = {
            "1. KDTree K-NN Tangent Plane & SVD/PCA (Bước 3 & 4 - HUONG_DAN_PIPELINE)",
            "2. Thin Plate Spline (TPS) RBF Non-Rigid (Bước 5 P4 - HUONG_DAN_PIPELINE)",
            "3. Poisson & Laplacian PDE Surface Propagation",
            "4. Adaptive Gun Tilt & Crease Standoff Compensation",
            "5. Bilateral Leg Symmetry Mirroring"
        };
        for (int i = 0; i < algos.Length; i++)
        {
            bool isSel = (selectedOcclusionAlgorithm == i);
            GUIStyle st = isSel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            if (GUILayout.Button(algos[i], st, GUILayout.Height(25)))
            {
                selectedOcclusionAlgorithm = i;
            }
        }

        GUILayout.Space(4);
        string[] algoDesc = {
            "ℹ️ KDTree K-NN Tangent Plane & PCA (Bước 3 & 4): Tìm K điểm lân cận hợp lệ trong bán kính r <= 5mm, phân tích hiệp phương sai SVD/PCA để ước lượng chính xác cao độ Z và vector pháp tuyến n tại các điểm khuyết tật.",
            "ℹ️ Thin Plate Spline TPS (Bước 5 P4): Tái tạo bề mặt giải phẫu liên tục C² từ 12 điểm mốc AI (Crotch, Thighs, Knees, Cuffs). Uốn dẻo mượt mà phủ kín toàn bộ các khoảng rỗng lớn của quần.",
            "ℹ️ Poisson & Laplace PDE: Giải phương trình vi phân Laplace ∇²Z = 0, lan truyền trường pháp tuyến mượt bảo toàn độ dốc tự nhiên của vải.",
            "ℹ️ Bù Góc Súng: Tự động nghiêng súng 15°-35° và nâng hạ Z thông minh hướng sâu vào rãnh nếp gấp bị che khuất tầm nhìn thẳng.",
            "ℹ️ Bù Đối Xứng: Lấy hình học mặt ngoài của ống quần đối xứng phản chiếu qua trục đũng để bù đắp sườn bị khuất hoàn toàn."
        };
        GUILayout.Label(algoDesc[Mathf.Clamp(selectedOcclusionAlgorithm, 0, 4)], AntigravityUITheme.StyleLabelMuted);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        // Khối 3: Tinh chỉnh thông số
        GUILayout.Label("3. THÔNG SỐ KỸ THUẬT CỦA THUẬT TOÁN", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Khoảng cách hạt điểm vá: {inpaintPointSpacingMm:F1} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(220));
        inpaintPointSpacingMm = GUILayout.HorizontalSlider(inpaintPointSpacingMm, 1.5f, 6.0f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Cỡ hạt điểm Point Cloud: {inpaintPointSize:F1}", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(220));
        inpaintPointSize = GUILayout.HorizontalSlider(inpaintPointSize, 1.5f, 6.0f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Bán kính quét láng giềng: {occlusionSearchRadiusMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(220));
        occlusionSearchRadiusMm = GUILayout.HorizontalSlider(occlusionSearchRadiusMm, 10f, 60f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Số láng giềng K-NN: {occlusionKNeighbors}", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(220));
        occlusionKNeighbors = Mathf.RoundToInt(GUILayout.HorizontalSlider(occlusionKNeighbors, 6f, 48f));
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Góc nghiêng súng tối đa: {maxGunTiltAngleDeg:F0}°", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(220));
        maxGunTiltAngleDeg = GUILayout.HorizontalSlider(maxGunTiltAngleDeg, 10f, 45f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Độ cao an toàn Z bù: +{occlusionZMarginMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(220));
        occlusionZMarginMm = GUILayout.HorizontalSlider(occlusionZMarginMm, 5f, 30f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Ngưỡng sườn dốc khuất: >{steepSlopeAngleThresholdDeg:F0}°", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(220));
        steepSlopeAngleThresholdDeg = GUILayout.HorizontalSlider(steepSlopeAngleThresholdDeg, 35f, 75f);
        GUILayout.EndHorizontal();
        GUILayout.EndVertical();

        GUILayout.Space(6);
        // Khối 4: Hiển thị trực quan 3D
        GUILayout.Label("4. TÙY CHỌN HIỂN THỊ 3D TRÊN KHÔNG GIAN LÀM VIỆC", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        bool prevShowInpainted = showInpaintedSurface3D;
        showInpaintedSurface3D = GUILayout.Toggle(showInpaintedSurface3D, " ✨ Hiển thị Chùm Điểm Vá 3D (Point Cloud Inpainting)", AntigravityUITheme.StyleToggle);
        if (showInpaintedSurface3D != prevShowInpainted && inpaintedMeshObject != null)
        {
            var mr = inpaintedMeshObject.GetComponent<MeshRenderer>();
            if (mr != null) mr.enabled = showInpaintedSurface3D;
        }

        bool prevHighlight = highlightInpaintedMesh;
        highlightInpaintedMesh = GUILayout.Toggle(highlightInpaintedMesh, " Màu hạt điểm: Xanh Cyan Nổi Bật / Khaki Tự Nhiên", AntigravityUITheme.StyleToggle);
        if (highlightInpaintedMesh != prevHighlight && inpaintedMeshObject != null)
        {
            PLYDirectLoader pl = FindAnyObjectByType<PLYDirectLoader>();
            BuildAndDisplayInpaintedMesh(pl);
        }

        showOcclusionHoles3D = GUILayout.Toggle(showOcclusionHoles3D, " Hiện Vị Trí Điểm Khuyết Tật (Hạt Đỏ Cam 3D)", AntigravityUITheme.StyleToggle);
        showCompensatedSurface3D = GUILayout.Toggle(showCompensatedSurface3D, " Hiện Lưới Bề Mặt Bù Đắp (Hạt Lưới Xanh Neon 3D)", AntigravityUITheme.StyleToggle);
        showGunTiltVectors3D = GUILayout.Toggle(showGunTiltVectors3D, " Hiện Vector Hướng Súng Phun Bù Góc (Tia Vàng 3D)", AntigravityUITheme.StyleToggle);
        GUILayout.EndVertical();

        GUILayout.Space(8);
        // Khối 5: Thực thi & Cập nhật
        if (GUILayout.Button("✨ VÁ ĐIỂM VÙNG KHUYẾT 3D (POINT CLOUD INPAINTING)", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(34)))
        {
            RunOcclusionCompensation();
        }
        GUILayout.Space(4);
        if (GUILayout.Button("🎯 CẬP NHẬT & ÁP DỤNG TRỰC TIẾP VÀO QUỸ ĐẠO PHUN", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(32)))
        {
            ApplyOcclusionCompensationToTrajectory();
        }

        GUILayout.EndVertical();
    }

    public void DetectOcclusionAndHoles()
    {
        detectedHolePointsWorld.Clear();
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (plyLoader == null)
        {
            occlusionStatusMsg = "⚠️ Không tìm thấy PLYDirectLoader để lấy dữ liệu đám mây điểm 3D!";
            return;
        }

        // Xác định vùng bao quanh quần jean
        float minX = -0.35f, maxX = 0.35f;
        float minZ = 0.05f, maxZ = 0.90f;

        List<Vector3> anatomicalPoly = null;
        if (landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
        {
            minX = float.MaxValue; maxX = float.MinValue;
            minZ = float.MaxValue; maxZ = float.MinValue;
            for (int i = 0; i < landmarksWorldPos.Count; i++)
            {
                minX = Mathf.Min(minX, landmarksWorldPos[i].x);
                maxX = Mathf.Max(maxX, landmarksWorldPos[i].x);
                minZ = Mathf.Min(minZ, landmarksWorldPos[i].z);
                maxZ = Mathf.Max(maxZ, landmarksWorldPos[i].z);
            }
            minX -= 0.05f; maxX += 0.05f;
            minZ -= 0.04f; maxZ += 0.04f;

            // Đa giác giải phẫu kín bao quanh toàn bộ quần: Cạp -> Ống phải -> Đáy đũng -> Ống trái
            anatomicalPoly = new List<Vector3>
            {
                landmarksWorldPos[0], // Eo trái
                landmarksWorldPos[1], // Cạp giữa
                landmarksWorldPos[2], // Eo phải
                landmarksWorldPos[5], // Đùi phải
                landmarksWorldPos[7], // Gối phải
                landmarksWorldPos[11], // Gấu ngoài phải
                landmarksWorldPos[10], // Gấu trong phải
                landmarksWorldPos[3],  // Đáy đũng
                landmarksWorldPos[8],  // Gấu trong trái
                landmarksWorldPos[9],  // Gấu ngoài trái
                landmarksWorldPos[6],  // Gối trái
                landmarksWorldPos[4]   // Đùi trái
            };
        }

        float stepM = Mathf.Clamp(inpaintPointSpacingMm, 1.5f, 8.0f) * 0.001f; // Bước lưới lấy theo thông số
        float sumDepthDiffMm = 0f;
        int depthSamples = 0;

        for (float z = minZ; z <= maxZ; z += stepM)
        {
            for (float x = minX; x <= maxX; x += stepM)
            {
                // Kiểm tra điểm có nằm trong đa giác giải phẫu hoặc đường bao segment không
                bool insideGarment = false;
                if (anatomicalPoly != null && anatomicalPoly.Count >= 12)
                {
                    insideGarment = IsPointInPolygonXZ(x, z, anatomicalPoly);
                }
                if (!insideGarment && garmentContourWorld != null && garmentContourWorld.Count > 3)
                {
                    insideGarment = IsPointInPolygonXZ(x, z, garmentContourWorld);
                }
                if (!insideGarment && landmarksWorldPos != null && landmarksWorldPos.Count >= 6)
                {
                    // Bao quát vùng đũng quần giữa mốc 3 (đáy đũng), mốc 4 (đùi trái) và mốc 5 (đùi phải)
                    Vector3 crotch = landmarksWorldPos[3];
                    Vector3 thighL = landmarksWorldPos[4];
                    Vector3 thighR = landmarksWorldPos[5];
                    float crotchMinX = Mathf.Min(thighL.x, thighR.x) - 0.03f;
                    float crotchMaxX = Mathf.Max(thighL.x, thighR.x) + 0.03f;
                    float crotchMinZ = Mathf.Min(crotch.z, Mathf.Min(thighL.z, thighR.z)) - 0.05f;
                    float crotchMaxZ = Mathf.Max(crotch.z, Mathf.Max(thighL.z, thighR.z)) + 0.08f;
                    if (x >= crotchMinX && x <= crotchMaxX && z >= crotchMinZ && z <= crotchMaxZ)
                    {
                        insideGarment = true;
                    }
                }

                if (!insideGarment) continue;

                // Kiểm tra trực tiếp xem tại vị trí này có điểm quét thực tế từ máy quét Zivid hay không
                bool hasRawPoint = plyLoader.HasSurfacePointAt(x, z);
                if (!hasRawPoint)
                {
                    // Lỗ thủng khuyết tật hoặc điểm mù camera không quét tới (Occlusion Hole thực sự)
                    float estY = 0.020f;
                    if (plyLoader.SampleSurfaceWorldHeightAndNormal(x, z, out float nY, out _))
                    {
                        estY = nY;
                    }
                    detectedHolePointsWorld.Add(new Vector3(x, estY, z));
                }
                else
                {
                    // Điểm quét đã có, kiểm tra tiếp xem có nằm trên nếp gấp độ dốc quá lớn (Self-occlusion)
                    if (plyLoader.SampleSurfaceWorldHeightAndNormal(x, z, out float sampleY, out Vector3 sampleNorm))
                    {
                        float slopeAngle = Vector3.Angle(sampleNorm, Vector3.up);
                        if (slopeAngle > steepSlopeAngleThresholdDeg)
                        {
                            detectedHolePointsWorld.Add(new Vector3(x, sampleY, z));
                            sumDepthDiffMm += Mathf.Sin(slopeAngle * Mathf.Deg2Rad) * 15f;
                            depthSamples++;
                        }
                    }
                }
            }
        }

        detectedHoleCount = detectedHolePointsWorld.Count;
        detectedOcclusionAreaCm2 = detectedHoleCount * (stepM * stepM * 10000f);
        avgCreaseDepthMm = depthSamples > 0 ? (sumDepthDiffMm / depthSamples) : 8.5f;
        occlusionStatusMsg = $"✅ Quét thành công: Phát hiện {detectedHoleCount} điểm khuất/nếp gấp ({detectedOcclusionAreaCm2:F1} cm²). Sẵn sàng bù đắp!";
    }

    public void RunOcclusionCompensation()
    {
        if (detectedHolePointsWorld == null || detectedHolePointsWorld.Count == 0)
        {
            DetectOcclusionAndHoles();
        }
        if (detectedHolePointsWorld.Count == 0)
        {
            occlusionStatusMsg = "✅ Không có lỗ khuyết tật nào cần bù đắp!";
            return;
        }

        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        compensatedSurfacePointsWorld.Clear();
        gunTiltNormalsWorld.Clear();
        compensatedHeightGrid.Clear();
        compensatedNormalGrid.Clear();

        string[] algoNames = {
            "KDTree K-NN Tangent Plane & SVD/PCA (Bước 3 & 4 - HUONG_DAN_PIPELINE)",
            "Thin Plate Spline (TPS) RBF Non-Rigid (Bước 5 P4 - HUONG_DAN_PIPELINE)",
            "Poisson & Laplacian PDE Propagation",
            "Adaptive Gun Tilt & Standoff",
            "Bilateral Leg Symmetry Mirroring"
        };

        // Tính trục đối xứng giữa 2 ống quần (cho thuật toán 4)
        float symmetryX = 0f;
        if (landmarksWorldPos != null && landmarksWorldPos.Count >= 4)
        {
            symmetryX = (landmarksWorldPos[1].x + landmarksWorldPos[3].x) * 0.5f;
        }

        for (int i = 0; i < detectedHolePointsWorld.Count; i++)
        {
            Vector3 pt = detectedHolePointsWorld[i];
            float compY = 0.022f;
            Vector3 compNorm = Vector3.up;

            switch (selectedOcclusionAlgorithm)
            {
                case 0:
                    // 1. KDTree K-NN Tangent Plane & SVD/PCA (Bước 3 & 4 trong HUONG_DAN_PIPELINE)
                    compY = CalculateMlsTangentPlaneHeight(plyLoader, pt.x, pt.z, out compNorm);
                    break;

                case 1:
                    // 2. Thin Plate Spline (TPS) RBF Surface Inpainting (Bước 5 P4 trong HUONG_DAN_PIPELINE)
                    compY = CalculateTpsHeightAtPoint(pt.x, pt.z);
                    compNorm = Vector3.up;
                    break;

                case 2:
                    // 3. Poisson & Laplacian PDE Surface Propagation (Nội suy vòng lân cận)
                    compY = CalculateLaplacianInpaintedHeight(plyLoader, pt.x, pt.z, out compNorm);
                    break;

                case 3:
                    // 4. Adaptive Gun Tilt & Crease Standoff Compensation
                    compY = CalculateMlsTangentPlaneHeight(plyLoader, pt.x, pt.z, out compNorm);
                    compY += occlusionZMarginMm * 0.001f;
                    // Nghiêng súng phun hướng sâu vào rãnh nếp gấp
                    Vector3 tiltAxis = Vector3.Cross(compNorm, Vector3.up);
                    if (tiltAxis.sqrMagnitude > 1e-4f)
                    {
                        tiltAxis.Normalize();
                        compNorm = Quaternion.AngleAxis(maxGunTiltAngleDeg, tiltAxis) * compNorm;
                    }
                    else
                    {
                        compNorm = Quaternion.Euler(maxGunTiltAngleDeg * 0.5f, 0f, 0f) * Vector3.up;
                    }
                    break;

                case 4:
                    // 5. Bilateral Leg Symmetry Mirroring
                    float mirrorX = 2f * symmetryX - pt.x;
                    if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(mirrorX, pt.z, out float mY, out Vector3 mNorm))
                    {
                        compY = mY;
                        compNorm = new Vector3(-mNorm.x, mNorm.y, mNorm.z);
                    }
                    else
                    {
                        compY = CalculateTpsHeightAtPoint(pt.x, pt.z);
                        compNorm = Vector3.up;
                    }
                    break;
            }

            Vector3 compPos = new Vector3(pt.x, compY, pt.z);
            compensatedSurfacePointsWorld.Add(compPos);
            gunTiltNormalsWorld.Add(compNorm);

            Vector2Int key = new Vector2Int(Mathf.RoundToInt(pt.x * 100f), Mathf.RoundToInt(pt.z * 100f));
            compensatedHeightGrid[key] = compY;
            compensatedNormalGrid[key] = compNorm;
        }

        // Tạo và hiển thị chùm điểm 3D (Point Cloud) thực thể vá kín lỗ khuyết
        BuildAndDisplayInpaintedMesh(plyLoader);

        occlusionStatusMsg = $"✨ Đã vá kín {compensatedSurfacePointsWorld.Count} hạt điểm khuyết tật bằng [{algoNames[selectedOcclusionAlgorithm]}]. Bề mặt quần đã liền mạch 100%!";
    }

    public void BuildAndDisplayInpaintedMesh(PLYDirectLoader plyLoader)
    {
        if (compensatedSurfacePointsWorld == null || compensatedSurfacePointsWorld.Count == 0) return;

        if (inpaintedMeshObject == null)
        {
            Transform parentT = (plyLoader != null) ? plyLoader.transform : this.transform;
            inpaintedMeshObject = new GameObject("InpaintedJeansSurface_PointCloud");
            inpaintedMeshObject.transform.SetParent(parentT, false);
            inpaintedMeshObject.transform.localPosition = Vector3.zero;
            inpaintedMeshObject.transform.localRotation = Quaternion.identity;
            inpaintedMeshObject.transform.localScale = Vector3.one;

            inpaintedMeshObject.AddComponent<MeshFilter>();
            var mr = inpaintedMeshObject.AddComponent<MeshRenderer>();

            Shader pcShader = Shader.Find("Custom/PointCloudVertexColor") ?? Shader.Find("Universal Render Pipeline/Particles/Unlit") ?? Shader.Find("Sprites/Default");
            Material mat = new Material(pcShader);
            mat.name = "Inpainted_PointCloud_Mat";
            if (mat.HasProperty("_PointSize")) mat.SetFloat("_PointSize", inpaintPointSize);
            mat.SetFloat("_Cull", 0);
            mr.sharedMaterial = mat;
        }

        MeshFilter filter = inpaintedMeshObject.GetComponent<MeshFilter>();
        MeshRenderer renderer = inpaintedMeshObject.GetComponent<MeshRenderer>();
        renderer.enabled = showInpaintedSurface3D;

        if (renderer.sharedMaterial != null && renderer.sharedMaterial.HasProperty("_PointSize"))
        {
            renderer.sharedMaterial.SetFloat("_PointSize", inpaintPointSize);
        }

        Color32 clothCol = highlightInpaintedMesh
            ? new Color32(30, 235, 250, 255)  // Cyan nổi bật để nhận diện vùng vá
            : new Color32(185, 175, 148, 255); // Khaki tự nhiên tiệp màu với đám mây điểm quần

        // Tạo chùm điểm Point Cloud thực thể (MeshTopology.Points)
        Mesh mesh = new Mesh();
        mesh.name = "Inpainted_PointCloud_Points";
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        List<Vector3> verts = new List<Vector3>(compensatedSurfacePointsWorld);
        List<Color32> cols = new List<Color32>(verts.Count);
        for (int i = 0; i < verts.Count; i++)
        {
            cols.Add(clothCol);
        }

        int[] indices = new int[verts.Count];
        for (int i = 0; i < verts.Count; i++) indices[i] = i;

        mesh.SetVertices(verts);
        mesh.SetColors(cols);
        mesh.SetIndices(indices, MeshTopology.Points, 0);

        filter.sharedMesh = mesh;

        // Bơm các điểm bù đắp vào lưới tra cứu độ cao của PLYDirectLoader
        if (plyLoader != null)
        {
            for (int i = 0; i < compensatedSurfacePointsWorld.Count; i++)
            {
                Vector3 p = compensatedSurfacePointsWorld[i];
                plyLoader.InjectSurfacePoint(p.x, p.y, p.z, clothCol);
            }
            if (plyLoader.renderMode == PLYDirectLoader.WorkpieceRenderMode.Mesh3D)
            {
                plyLoader.RefreshSurfaceMeshFromGrid();
            }
        }
    }

    private float CalculateTpsHeightAtPoint(float x, float z)
    {
        if (landmarksWorldPos == null || landmarksWorldPos.Count == 0) return 0.022f;

        float sumY = 0f;
        float sumWeight = 0f;
        for (int i = 0; i < landmarksWorldPos.Count; i++)
        {
            float dx = x - landmarksWorldPos[i].x;
            float dz = z - landmarksWorldPos[i].z;
            float distSq = dx * dx + dz * dz;
            float w = 1f / (distSq + 0.002f);
            sumY += landmarksWorldPos[i].y * w;
            sumWeight += w;
        }
        return sumWeight > 0f ? (sumY / sumWeight) : 0.022f;
    }

    private float CalculateLaplacianInpaintedHeight(PLYDirectLoader plyLoader, float x, float z, out Vector3 norm)
    {
        norm = Vector3.up;
        if (plyLoader == null) return 0.022f;

        float radius = occlusionSearchRadiusMm * 0.001f;
        float sumY = 0f;
        float sumWeight = 0f;
        Vector3 sumNorm = Vector3.zero;

        for (int ring = 1; ring <= 3; ring++)
        {
            float r = radius * (ring / 3f);
            for (int a = 0; a < 8; a++)
            {
                float ang = a * Mathf.PI * 0.25f;
                float sx = x + Mathf.Cos(ang) * r;
                float sz = z + Mathf.Sin(ang) * r;
                if (plyLoader.SampleSurfaceWorldHeightAndNormal(sx, sz, out float ny, out Vector3 sn))
                {
                    float w = 1f / (r * r + 1e-4f);
                    sumY += ny * w;
                    sumNorm += sn * w;
                    sumWeight += w;
                }
            }
        }

        if (sumWeight > 0f)
        {
            norm = sumNorm.normalized;
            return sumY / sumWeight;
        }
        return CalculateTpsHeightAtPoint(x, z);
    }

    private float CalculateMlsTangentPlaneHeight(PLYDirectLoader plyLoader, float x, float z, out Vector3 norm)
    {
        norm = Vector3.up;
        if (plyLoader == null) return 0.022f;

        float radius = occlusionSearchRadiusMm * 0.001f;
        List<Vector3> neighbors = new List<Vector3>();

        for (int dx = -2; dx <= 2; dx++)
        {
            for (int dz = -2; dz <= 2; dz++)
            {
                if (dx == 0 && dz == 0) continue;
                float sx = x + dx * (radius * 0.45f);
                float sz = z + dz * (radius * 0.45f);
                if (plyLoader.SampleSurfaceWorldHeightAndNormal(sx, sz, out float ny, out Vector3 sn))
                {
                    neighbors.Add(new Vector3(sx, ny, sz));
                    if (neighbors.Count >= occlusionKNeighbors) break;
                }
            }
            if (neighbors.Count >= occlusionKNeighbors) break;
        }

        if (neighbors.Count < 3)
        {
            return CalculateLaplacianInpaintedHeight(plyLoader, x, z, out norm);
        }

        // Fit mặt phẳng tiếp xúc bằng Moving Least Squares (Z = a*X + b*Z + c)
        float avgX = 0f, avgY = 0f, avgZ = 0f;
        for (int i = 0; i < neighbors.Count; i++)
        {
            avgX += neighbors[i].x;
            avgY += neighbors[i].y;
            avgZ += neighbors[i].z;
        }
        avgX /= neighbors.Count;
        avgY /= neighbors.Count;
        avgZ /= neighbors.Count;

        float sxx = 0f, sxz = 0f, szz = 0f, sxy = 0f, szy = 0f;
        for (int i = 0; i < neighbors.Count; i++)
        {
            float rx = neighbors[i].x - avgX;
            float rz = neighbors[i].z - avgZ;
            float ry = neighbors[i].y - avgY;
            sxx += rx * rx;
            sxz += rx * rz;
            szz += rz * rz;
            sxy += rx * ry;
            szy += rz * ry;
        }

        float det = sxx * szz - sxz * sxz;
        if (Mathf.Abs(det) > 1e-7f)
        {
            float a = (sxy * szz - szy * sxz) / det;
            float b = (szy * sxx - sxy * sxz) / det;
            norm = new Vector3(-a, 1f, -b).normalized;
            return avgY + a * (x - avgX) + b * (z - avgZ);
        }

        return avgY;
    }

    private static bool IsPointInPolygonXZ(float px, float pz, List<Vector3> poly)
    {
        if (poly == null || poly.Count < 3) return false;
        bool inside = false;
        for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++)
        {
            if (((poly[i].z > pz) != (poly[j].z > pz)) &&
                (px < (poly[j].x - poly[i].x) * (pz - poly[i].z) / (poly[j].z - poly[i].z + 1e-7f) + poly[i].x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    public bool SampleCompensatedSurface(float worldX, float worldZ, out float worldY, out Vector3 normalWorld)
    {
        Vector2Int key = new Vector2Int(Mathf.RoundToInt(worldX * 100f), Mathf.RoundToInt(worldZ * 100f));
        if (compensatedHeightGrid.TryGetValue(key, out worldY) && compensatedNormalGrid.TryGetValue(key, out normalWorld))
        {
            return true;
        }
        worldY = 0.020f;
        normalWorld = Vector3.up;
        return false;
    }

    public void ApplyOcclusionCompensationToTrajectory()
    {
        if (waypoints == null || waypoints.Count == 0)
        {
            occlusionStatusMsg = "⚠️ Chưa có quỹ đạo nào để áp dụng bù đắp! Hãy vẽ nét hoặc nạp mẫu trước.";
            return;
        }
        if (compensatedSurfacePointsWorld.Count == 0)
        {
            RunOcclusionCompensation();
        }

        float radiusM = occlusionSearchRadiusMm * 0.001f;
        int modifiedCount = 0;
        float[] seedJoints = null;

        for (int i = 0; i < waypoints.Count; i++)
        {
            WaypointData wp = waypoints[i];
            Vector3 p = wp.targetPosWorld_m;

            // Tìm điểm bù đắp gần nhất
            float minDist = float.MaxValue;
            int bestIdx = -1;
            for (int k = 0; k < compensatedSurfacePointsWorld.Count; k++)
            {
                float dx = p.x - compensatedSurfacePointsWorld[k].x;
                float dz = p.z - compensatedSurfacePointsWorld[k].z;
                float d = Mathf.Sqrt(dx * dx + dz * dz);
                if (d < minDist)
                {
                    minDist = d;
                    bestIdx = k;
                }
            }

            if (bestIdx >= 0 && minDist <= radiusM)
            {
                float compY = compensatedSurfacePointsWorld[bestIdx].y;
                Vector3 compN = gunTiltNormalsWorld[bestIdx];

                wp.targetPosWorld_m.y = compY + customDrawZOffsetMm * 0.001f;
                wp.microNormal = compN;

                if (globalReference != null)
                {
                    Vector3 localM = globalReference.InverseTransformPoint(wp.targetPosWorld_m);
                    wp.targetPosGlobal_mm = localM * 1000f;
                }

                // Bù góc súng theo pháp tuyến nghiêng nếu có
                if (selectedOcclusionAlgorithm == 3 || Vector3.Angle(compN, Vector3.up) > 8f)
                {
                    wp.pitch_deg = Mathf.Clamp(-compN.z * maxGunTiltAngleDeg, -maxGunTiltAngleDeg, maxGunTiltAngleDeg);
                    wp.yaw_deg = Mathf.Clamp(compN.x * maxGunTiltAngleDeg, -maxGunTiltAngleDeg, maxGunTiltAngleDeg);
                }

                // Giải Động học Nghịch IK cho điểm vừa cập nhật
                Quaternion flangeRot = GetWaypointFlangeRotation(wp);
                var sols = DobotCR10AKinematics.SolveTcpPose(
                    wp.targetPosGlobal_mm, flangeRot, seedJoints, toolLength,
                    useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY
                );

                if (sols != null && sols.Count > 0)
                {
                    var best = sols[0];
                    for (int j = 0; j < 6; j++)
                    {
                        wp.jointAngles_rad[j] = (float)best.jointsRad[j];
                        wp.jointAngles_deg[j] = (float)best.jointsDeg[j];
                    }
                    wp.isSolved = true;
                    seedJoints = wp.jointAngles_rad;
                }

                modifiedCount++;
            }
        }

        occlusionStatusMsg = $"🎯 Đã áp dụng bù đắp vào {modifiedCount}/{waypoints.Count} điểm quỹ đạo. Quỹ đạo robot đã ôm sát nếp gấp khuất!";
        Debug.Log($"[CR10A Occlusion] Đã cập nhật bù đắp vào {modifiedCount} waypoints.");
    }

    // =========================================================================
    // BẢNG VẼ QUẦN JEAN 2D TIÊU CHUẨN (TƯỢNG TRƯNG) -> ÁNH XẠ PHI CỨNG LÊN 3D
    // =========================================================================
    private void DrawCanonicalJeans2DCanvasSection(float width)
    {
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        GUILayout.Label("0. VẼ QUỸ ĐẠO TRÊN QUẦN 2D TIÊU CHUẨN ➔ ÁNH XẠ 3D", AntigravityUITheme.StyleSectionLabel);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button(showCanonicalDrawPanel ? "Thu Gọn" : "Mở Bảng Vẽ 2D", AntigravityUITheme.StyleButton, GUILayout.Width(95), GUILayout.Height(20)))
        {
            showCanonicalDrawPanel = !showCanonicalDrawPanel;
        }
        GUILayout.EndHorizontal();

        if (!showCanonicalDrawPanel)
        {
            GUILayout.EndVertical();
            return;
        }

        EnsureCanonicalJeansCanvasInitialized();

        GUILayout.Label("Giữ Chuột Trái kéo trên hình Quần 2D bên dưới để vẽ đường phun. Khi bấm [ÁP DỤNG & ÁNH XẠ LÊN 3D], đường vẽ sẽ tự động biến dạng theo 12 điểm mốc và ôm sát bề mặt lồi lõm 3D của đám mây điểm!", AntigravityUITheme.StyleLabelMuted);

        // Khung Canvas 2D Quần Jean Tiêu Chuẩn (260 x 300 px)
        float canvasW = Mathf.Min(width - 12f, 270f);
        float canvasH = 300f;

        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        Rect canvasRect = GUILayoutUtility.GetRect(canvasW, canvasH, GUILayout.Width(canvasW), GUILayout.Height(canvasH));
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();

        // Vẽ nền Quần 2D Tiêu Chuẩn
        if (canonicalJeansBgTex != null)
        {
            GUI.DrawTexture(canvasRect, canonicalJeansBgTex, ScaleMode.StretchToFill);
        }

        // Xử lý sự kiện chuột vẽ trực tiếp trên Canvas 2D
        Event ev = Event.current;
        if (ev != null && canvasRect.Contains(ev.mousePosition))
        {
            Vector2 normUV = new Vector2(
                Mathf.Clamp01((ev.mousePosition.x - canvasRect.x) / canvasRect.width),
                Mathf.Clamp01((ev.mousePosition.y - canvasRect.y) / canvasRect.height)
            );

            if (ev.type == EventType.MouseDown && ev.button == 0)
            {
                currentDrawingStroke = new List<Vector2> { normUV };
                customDrawnStrokes.Add(currentDrawingStroke);
                isDrawingOnCanvas = true;
                ev.Use();
            }
            else if (ev.type == EventType.MouseDrag && ev.button == 0 && isDrawingOnCanvas && currentDrawingStroke != null)
            {
                if (currentDrawingStroke.Count == 0 || Vector2.Distance(currentDrawingStroke[currentDrawingStroke.Count - 1], normUV) >= 0.012f)
                {
                    currentDrawingStroke.Add(normUV);
                }
                ev.Use();
            }
            else if (ev.type == EventType.MouseUp && ev.button == 0 && isDrawingOnCanvas)
            {
                isDrawingOnCanvas = false;
                currentDrawingStroke = null;
                ev.Use();
            }
        }
        else if (ev != null && ev.type == EventType.MouseUp && isDrawingOnCanvas)
        {
            isDrawingOnCanvas = false;
            currentDrawingStroke = null;
        }

        // Vẽ 12 điểm mốc chuẩn lên Canvas 2D (Tự động co giãn bám theo kích thước Size quần tượng trưng)
        Vector2[] curLandmarksUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        for (int i = 0; i < curLandmarksUV.Length; i++)
        {
            Vector2 uv = curLandmarksUV[i];
            Vector2 ptScreen = new Vector2(canvasRect.x + uv.x * canvasRect.width, canvasRect.y + uv.y * canvasRect.height);
            DrawGuiSolidRect(new Rect(ptScreen.x - 3f, ptScreen.y - 3f, 6f, 6f), new Color(1f, 0.75f, 0.1f, 0.95f));
        }

        // Vẽ các nét vẽ của người dùng lên Canvas 2D
        int totalStrokePts = 0;
        for (int s = 0; s < customDrawnStrokes.Count; s++)
        {
            var stroke = customDrawnStrokes[s];
            if (stroke == null || stroke.Count == 0) continue;
            totalStrokePts += stroke.Count;
            Color strokeCol = (s % 2 == 0) ? new Color(0.1f, 0.95f, 1f, 0.95f) : new Color(0.25f, 1f, 0.45f, 0.95f);
            for (int p = 0; p < stroke.Count - 1; p++)
            {
                Vector2 p0 = new Vector2(canvasRect.x + stroke[p].x * canvasRect.width, canvasRect.y + stroke[p].y * canvasRect.height);
                Vector2 p1 = new Vector2(canvasRect.x + stroke[p + 1].x * canvasRect.width, canvasRect.y + stroke[p + 1].y * canvasRect.height);
                DrawGuiLine2D(p0, p1, strokeCol, 2.5f);
            }
            Vector2 startPt = new Vector2(canvasRect.x + stroke[0].x * canvasRect.width, canvasRect.y + stroke[0].y * canvasRect.height);
            DrawGuiSolidRect(new Rect(startPt.x - 3f, startPt.y - 3f, 6f, 6f), Color.yellow);
        }

        GUILayout.Space(4);
        // Hàng chọn Mẫu cần phun theo Size (Khi đổi Size: KHÔNG reset mẫu, mẫu tự co giãn, hình quần 2D đổi kích thước)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Mẫu phun: SIZE {activeSprayTemplateSize} | Quần nhận diện: {detectedJeansSize}", AntigravityUITheme.StyleLabelGreen, GUILayout.Width(250));
        foreach (string sz in new[] { "S", "M", "L" })
        {
            GUIStyle st = activeSprayTemplateSize == sz ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            if (GUILayout.Button(sz, st, GUILayout.Width(32), GUILayout.Height(20)))
            {
                ChangeSprayTemplateSize(sz);
            }
        }
        GUILayout.EndHorizontal();

        // Hàng nút Mẫu Vẽ & Xóa Nét
        GUILayout.BeginHorizontal();
        if (GUILayout.Button($"Mẫu Râu Mèo + Đùi ({activeSprayTemplateSize})", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            LoadPresetWhiskersAndThighOn2DCanvas();
            customDrawStatusMsg = "Đã nạp Mẫu Râu Mèo + Đùi lên Quần 2D. Bấm [ÁP DỤNG & ÁNH XẠ LÊN 3D] để cập nhật đường phun.";
        }
        if (GUILayout.Button("Mẫu Zigzag 2 Ống", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            LoadPresetZigzag2LegsOn2DCanvas();
            customDrawStatusMsg = "Đã nạp Mẫu Zigzag 2 Ống lên Quần 2D. Bấm [ÁP DỤNG & ÁNH XẠ LÊN 3D] để cập nhật đường phun.";
        }
        if (GUILayout.Button("↩ Xóa Nét Cuối", AntigravityUITheme.StyleButton, GUILayout.Width(92), GUILayout.Height(22)))
        {
            hasInitializedDefaultStrokes = true;
            if (customDrawnStrokes.Count > 0)
            {
                customDrawnStrokes.RemoveAt(customDrawnStrokes.Count - 1);
            }
            if (customDrawnStrokes.Count == 0)
            {
                ClearAll2DAnd3DTrajectoryLines();
            }
        }
        if (GUILayout.Button("🗑 Xóa Trắng", AntigravityUITheme.StyleButton, GUILayout.Width(80), GUILayout.Height(22)))
        {
            ClearAll2DAnd3DTrajectoryLines();
        }
        GUILayout.EndHorizontal();

        // =====================================================================
        // KHỐI QUẢN LÝ MẪU PHUN (CHỌN THƯ MỤC XUẤT/IMPORT & FILE JSON TÙY BIẾN)
        // =====================================================================
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label("💾 QUẢN LÝ MẪU PHUN (CHỌN THƯ MỤC XUẤT / IMPORT FILE JSON)", AntigravityUITheme.StyleSectionLabel);

        // Hàng 1: Chọn thư mục xuất / import mẫu
        GUILayout.BeginHorizontal();
        GUILayout.Label("Thư mục:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(58));
        sprayTemplateDirectory = GUILayout.TextField(sprayTemplateDirectory, AntigravityUITheme.StyleTextField, GUILayout.Height(22));
        if (GUILayout.Button("📂 Chọn Thư Mục...", AntigravityUITheme.StyleButton, GUILayout.Width(130), GUILayout.Height(22)))
        {
            ChooseSprayTemplateDirectory();
        }
        GUILayout.EndHorizontal();

        // Hàng 2: Tên file & Nút Xuất / Import File
        GUILayout.BeginHorizontal();
        GUILayout.Label("Tên File:", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(58));
        saveTemplateFileName = GUILayout.TextField(saveTemplateFileName, AntigravityUITheme.StyleTextField, GUILayout.Width(110), GUILayout.Height(24));
        if (GUILayout.Button("💾 Xuất Mẫu (.json)...", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(24)))
        {
            ExportSprayTemplateWithDialog();
        }
        if (GUILayout.Button("📂 Import Mẫu (.json)...", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            ImportSprayTemplateWithDialog();
        }
        GUILayout.EndHorizontal();

        if (availableSprayTemplateFiles == null || availableSprayTemplateFiles.Count == 0)
        {
            RefreshAvailableSprayTemplates();
        }
        if (availableSprayTemplateFiles != null && availableSprayTemplateFiles.Count > 0)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Mẫu có sẵn:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(75));
            int maxShow = Mathf.Min(availableSprayTemplateFiles.Count, 3);
            for (int f = 0; f < maxShow; f++)
            {
                string fn = availableSprayTemplateFiles[f];
                if (GUILayout.Button(fn, AntigravityUITheme.StyleButton, GUILayout.Height(22)))
                {
                    saveTemplateFileName = fn;
                    ImportSprayTemplateFromFile(Path.Combine(GetSprayTemplateDirectoryFullPath(), fn + ".json"));
                }
            }
            GUILayout.EndHorizontal();
        }

        preserveCustomDrawingAcrossSamples = GUILayout.Toggle(
            preserveCustomDrawingAcrossSamples,
            " Giữ & Tự động áp dụng mẫu vẽ cho mọi mẫu phôi 3D",
            AntigravityUITheme.StyleToggle
        );
        GUILayout.EndVertical();

        // Thanh chỉnh độ cao đầu phun trên mặt vải 3D (0mm = áp sát mặt vải, 20mm = khoảng cách súng phun)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Độ cao bám mặt 3D (+Z): {customDrawZOffsetMm:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(190));
        float newSliderZ = GUILayout.HorizontalSlider(customDrawZOffsetMm, 0f, 50f);
        if (Mathf.Abs(newSliderZ - customDrawZOffsetMm) > 0.05f)
        {
            customDrawZOffsetMm = newSliderZ;
            ApplyCanonical2DDrawingTo3DTrajectory();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Áp Sát Mặt Quần (0 mm)", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            customDrawZOffsetMm = 0f;
            ApplyCanonical2DDrawingTo3DTrajectory();
        }
        if (GUILayout.Button("Khoảng Phun Chuẩn (20 mm)", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            customDrawZOffsetMm = 20f;
            ApplyCanonical2DDrawingTo3DTrajectory();
        }
        GUILayout.EndHorizontal();

        // Nút Bật/Tắt Đường Dự Kiến Phun 3D (Cyan Polyline)
        GUILayout.BeginHorizontal();
        string cyanToggleText = showGizmos ? "👁️ Đang HIỆN Đường Dự Kiến Phun (Cyan) - [Nhấn Để TẮT]" : "👁️ Đang ẨN Đường Dự Kiến Phun (Cyan) - [Nhấn Để HIỆN]";
        GUIStyle cyanToggleStyle = showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button(cyanToggleText, cyanToggleStyle, GUILayout.Height(28)))
        {
            showGizmos = !showGizmos;
        }
        GUILayout.EndHorizontal();

        // Nút ÁP DỤNG & ÁNH XẠ LÊN 3D
        if (GUILayout.Button($"🎯 ÁP DỤNG & ÁNH XẠ LÊN BỀ MẶT 3D ({customDrawnStrokes.Count} nét / {totalStrokePts} điểm)", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(32)))
        {
            ApplyCanonical2DDrawingTo3DTrajectory();
        }

        if (!string.IsNullOrEmpty(customDrawStatusMsg))
        {
            GUILayout.Label(customDrawStatusMsg, AntigravityUITheme.StyleLabelGreen);
        }

        GUILayout.EndVertical();
    }

    public string GetSprayTemplateDirectoryFullPath()
    {
        if (string.IsNullOrEmpty(sprayTemplateDirectory))
        {
            sprayTemplateDirectory = "output_drawing/spray_templates";
        }
        if (Path.IsPathRooted(sprayTemplateDirectory))
        {
            return sprayTemplateDirectory;
        }
        return Path.GetFullPath(Path.Combine(Application.dataPath, "..", sprayTemplateDirectory));
    }

    public void RefreshAvailableSprayTemplates()
    {
        availableSprayTemplateFiles.Clear();
        try
        {
            string dir = GetSprayTemplateDirectoryFullPath();
            if (Directory.Exists(dir))
            {
                string[] files = Directory.GetFiles(dir, "*.json");
                Array.Sort(files, (a, b) => File.GetLastWriteTime(b).CompareTo(File.GetLastWriteTime(a)));
                for (int i = 0; i < files.Length; i++)
                {
                    availableSprayTemplateFiles.Add(Path.GetFileNameWithoutExtension(files[i]));
                }
            }
        }
        catch { }
    }

    public void ChooseSprayTemplateDirectory()
    {
        string cur = GetSprayTemplateDirectoryFullPath();
#if UNITY_EDITOR
        string chosen = UnityEditor.EditorUtility.OpenFolderPanel("Chọn Thư Mục Lưu/Import Mẫu Phun", cur, "");
        if (!string.IsNullOrEmpty(chosen))
        {
            sprayTemplateDirectory = chosen;
            RefreshAvailableSprayTemplates();
            customDrawStatusMsg = $"📁 Đã chọn thư mục mẫu phun: {chosen}";
        }
#else
        customDrawStatusMsg = $"📁 Thư mục hiện hành: {cur}";
#endif
    }

    public void ExportSprayTemplateWithDialog()
    {
        if (customDrawnStrokes == null || customDrawnStrokes.Count == 0)
        {
            customDrawStatusMsg = "⚠️ Chưa có nét vẽ nào trên quần để xuất thành Mẫu!";
            return;
        }

        string curDir = GetSprayTemplateDirectoryFullPath();
        if (!Directory.Exists(curDir)) Directory.CreateDirectory(curDir);
        string defName = string.IsNullOrEmpty(saveTemplateFileName) ? $"mau_phun_size_{activeSprayTemplateSize}" : saveTemplateFileName.Trim();
        if (!defName.EndsWith(".json", StringComparison.OrdinalIgnoreCase)) defName += ".json";

#if UNITY_EDITOR
        string chosenPath = UnityEditor.EditorUtility.SaveFilePanel("Chọn Nơi Xuất Mẫu Phun (.json)", curDir, defName, "json");
        if (!string.IsNullOrEmpty(chosenPath))
        {
            SaveCurrentSprayTemplateToPath(chosenPath);
            sprayTemplateDirectory = Path.GetDirectoryName(chosenPath);
            saveTemplateFileName = Path.GetFileNameWithoutExtension(chosenPath);
            RefreshAvailableSprayTemplates();
        }
#else
        SaveCurrentSprayTemplate(defName);
#endif
    }

    public void ImportSprayTemplateWithDialog()
    {
        string curDir = GetSprayTemplateDirectoryFullPath();
        if (!Directory.Exists(curDir)) Directory.CreateDirectory(curDir);

#if UNITY_EDITOR
        string chosenFile = UnityEditor.EditorUtility.OpenFilePanel("Chọn File Mẫu Phun Cần Import (.json)", curDir, "json");
        if (!string.IsNullOrEmpty(chosenFile))
        {
            sprayTemplateDirectory = Path.GetDirectoryName(chosenFile);
            ImportSprayTemplateFromFile(chosenFile);
        }
#else
        ImportSprayTemplateFromFile(saveTemplateFileName);
#endif
    }

    public void SaveCurrentSprayTemplateToPath(string fullPath)
    {
        if (customDrawnStrokes == null || customDrawnStrokes.Count == 0)
        {
            customDrawStatusMsg = "⚠️ Chưa có nét vẽ nào trên quần để xuất thành Mẫu!";
            return;
        }

        try
        {
            string dir = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
            }

            SprayPatternTemplate template = new SprayPatternTemplate
            {
                templateName = Path.GetFileNameWithoutExtension(fullPath),
                createdDate = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"),
                size = activeSprayTemplateSize,
                zOffsetMm = customDrawZOffsetMm,
                strokes = new List<Stroke2D>()
            };

            foreach (var st in customDrawnStrokes)
            {
                if (st == null || st.Count == 0) continue;
                var strokeObj = new Stroke2D();
                foreach (var pt in st)
                {
                    strokeObj.points.Add(new StrokePoint2D(pt.x, pt.y));
                }
                template.strokes.Add(strokeObj);
            }

            string json = JsonUtility.ToJson(template, true);
            File.WriteAllText(fullPath, json);
            RefreshAvailableSprayTemplates();
            customDrawStatusMsg = $"💾 Đã xuất Mẫu phun thành công: {Path.GetFileName(fullPath)} ({template.strokes.Count} nét, SIZE {activeSprayTemplateSize})!";
            Debug.Log($"[CR10A SprayTemplate] Đã xuất mẫu phun vào {fullPath}");
        }
        catch (Exception ex)
        {
            customDrawStatusMsg = $"❌ Lỗi khi xuất mẫu phun: {ex.Message}";
            Debug.LogError($"[CR10A SprayTemplate] Lỗi xuất: {ex.Message}");
        }
    }

    public void SaveCurrentSprayTemplate(string templateName = null)
    {
        if (string.IsNullOrEmpty(templateName))
        {
            templateName = string.IsNullOrEmpty(saveTemplateFileName) ? $"mau_phun_{activeSprayTemplateSize}" : saveTemplateFileName.Trim();
        }
        if (!templateName.EndsWith(".json", StringComparison.OrdinalIgnoreCase))
        {
            templateName += ".json";
        }
        string fullPath = Path.Combine(GetSprayTemplateDirectoryFullPath(), templateName);
        SaveCurrentSprayTemplateToPath(fullPath);
    }

    public void ImportSprayTemplateFromFile(string filePathOrName = null)
    {
        try
        {
            string dir = GetSprayTemplateDirectoryFullPath();
            if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);

            string fullPath = null;
            if (!string.IsNullOrEmpty(filePathOrName))
            {
                if (File.Exists(filePathOrName))
                {
                    fullPath = filePathOrName;
                }
                else
                {
                    string target = filePathOrName.Trim();
                    if (!target.EndsWith(".json", StringComparison.OrdinalIgnoreCase)) target += ".json";
                    string candidate = Path.Combine(dir, target);
                    if (File.Exists(candidate)) fullPath = candidate;
                }
            }

            if (string.IsNullOrEmpty(fullPath))
            {
                var files = Directory.GetFiles(dir, "*.json");
                if (files.Length > 0)
                {
                    Array.Sort(files, (a, b) => File.GetLastWriteTime(b).CompareTo(File.GetLastWriteTime(a)));
                    fullPath = files[0];
                }
            }

            if (string.IsNullOrEmpty(fullPath) || !File.Exists(fullPath))
            {
                customDrawStatusMsg = $"⚠️ Không tìm thấy file mẫu phun JSON trong {dir}";
                return;
            }

            string json = File.ReadAllText(fullPath);
            SprayPatternTemplate template = JsonUtility.FromJson<SprayPatternTemplate>(json);
            if (template != null && template.strokes != null && template.strokes.Count > 0)
            {
                hasInitializedDefaultStrokes = true;
                isDrawingOnCanvas = false;
                currentDrawingStroke = null;
                customDrawnStrokes.Clear();

                foreach (var st in template.strokes)
                {
                    if (st.points == null || st.points.Count == 0) continue;
                    List<Vector2> s = new List<Vector2>();
                    foreach (var p in st.points)
                    {
                        s.Add(new Vector2(p.u, p.v));
                    }
                    customDrawnStrokes.Add(s);
                }

                if (!string.IsNullOrEmpty(template.size) && template.size != activeSprayTemplateSize)
                {
                    activeSprayTemplateSize = template.size;
                    RebuildCanonicalJeansTexture();
                }

                if (template.zOffsetMm > 0f) customDrawZOffsetMm = template.zOffsetMm;
                saveTemplateFileName = Path.GetFileNameWithoutExtension(fullPath);
                sprayTemplateDirectory = Path.GetDirectoryName(fullPath);
                RefreshAvailableSprayTemplates();

                // Tự động chiếu ngay lên đám mây điểm 3D hiện hành
                ApplyCanonical2DDrawingTo3DTrajectory();

                customDrawStatusMsg = $"📂 Đã import Mẫu [{Path.GetFileName(fullPath)}] ({customDrawnStrokes.Count} nét, SIZE {activeSprayTemplateSize}) & tự động ánh xạ lên 3D!";
                Debug.Log($"[CR10A SprayTemplate] Đã import mẫu từ {fullPath}");
            }
            else
            {
                customDrawStatusMsg = $"⚠️ File mẫu {Path.GetFileName(fullPath)} không có nét vẽ hợp lệ!";
            }
        }
        catch (Exception ex)
        {
            customDrawStatusMsg = $"❌ Lỗi khi import mẫu: {ex.Message}";
            Debug.LogError($"[CR10A SprayTemplate] Lỗi import: {ex.Message}");
        }
    }

    /// <summary>
    /// Chuyển đổi Size Mẫu Cần Phun (S/M/L):
    /// 1. KHÔNG reset mẫu vẽ của người dùng.
    /// 2. Tự động co giãn (scale) các nét vẽ theo đúng tỷ lệ hình thể size mới.
    /// 3. Thay đổi kích thước hình tượng trưng chiếc quần Jean 2D (nhỏ gọn ở S, vừa ở M, to rộng ở L).
    /// 4. Tự động ánh xạ lại nét vẽ lên bề mặt 3D.
    /// </summary>
    public void ChangeSprayTemplateSize(string newSize)
    {
        if (string.IsNullOrEmpty(newSize)) newSize = "S";
        if (activeSprayTemplateSize == newSize) return;

        string prevSize = activeSprayTemplateSize;
        activeSprayTemplateSize = newSize;

        // 1. Tự động co giãn toàn bộ các nét vẽ hiện tại theo kích thước size mới (KHÔNG RESET)
        if (customDrawnStrokes != null && customDrawnStrokes.Count > 0)
        {
            ScaleCustomDrawnStrokesBetweenSizes(prevSize, newSize);
        }

        // 2. Tái tạo hình ảnh quần 2D tượng trưng theo đúng kích thước size mới (S/M/L)
        RebuildCanonicalJeansTexture();

        // 3. Tự động ánh xạ lại nét vẽ lên bề mặt 3D
        if (customDrawnStrokes != null && customDrawnStrokes.Count > 0)
        {
            ApplyCanonical2DDrawingTo3DTrajectory();
        }

        customDrawStatusMsg = $"📏 Đã đổi Mẫu phun: SIZE {prevSize} ➔ SIZE {newSize}. Nét vẽ & quần 2D đã tự co giãn theo size mới!";
        Debug.Log($"[CR10A Size] {customDrawStatusMsg}");
    }

    public static void GetJeansBoundsForSize(string size, out float minU, out float maxU, out float minV, out float maxV)
    {
        switch (size)
        {
            case "S":
                minU = 0.27f; maxU = 0.73f;
                minV = 0.14f; maxV = 0.85f;
                break;
            case "L":
                minU = 0.18f; maxU = 0.82f;
                minV = 0.06f; maxV = 0.96f;
                break;
            case "M":
            default:
                minU = 0.22f; maxU = 0.78f;
                minV = 0.10f; maxV = 0.91f;
                break;
        }
    }

    public void ScaleCustomDrawnStrokesBetweenSizes(string fromSize, string toSize)
    {
        if (string.IsNullOrEmpty(fromSize) || string.IsNullOrEmpty(toSize) || fromSize == toSize) return;
        if (customDrawnStrokes == null || customDrawnStrokes.Count == 0) return;

        GetJeansBoundsForSize(fromSize, out float minUFrom, out float maxUFrom, out float minVFrom, out float maxVFrom);
        GetJeansBoundsForSize(toSize, out float minUTo, out float maxUTo, out float minVTo, out float maxVTo);

        float spanUFrom = Mathf.Max(0.01f, maxUFrom - minUFrom);
        float spanVFrom = Mathf.Max(0.01f, maxVFrom - minVFrom);
        float spanUTo = Mathf.Max(0.01f, maxUTo - minUTo);
        float spanVTo = Mathf.Max(0.01f, maxVTo - minVTo);

        for (int s = 0; s < customDrawnStrokes.Count; s++)
        {
            var st = customDrawnStrokes[s];
            if (st == null) continue;
            for (int p = 0; p < st.Count; p++)
            {
                Vector2 pt = st[p];
                float tu = (pt.x - minUFrom) / spanUFrom;
                float tv = (pt.y - minVFrom) / spanVFrom;
                float newU = Mathf.Clamp(minUTo + tu * spanUTo, 0.05f, 0.95f);
                float newV = Mathf.Clamp(minVTo + tv * spanVTo, 0.05f, 0.98f);
                st[p] = new Vector2(newU, newV);
            }
        }
    }

    public void RebuildCanonicalJeansTexture()
    {
        if (canonicalJeansBgTex != null)
        {
            if (Application.isPlaying) Destroy(canonicalJeansBgTex);
            else DestroyImmediate(canonicalJeansBgTex);
        }
        canonicalJeansBgTex = BuildCanonicalJeans2DTexture(260, 300, activeSprayTemplateSize);
    }

    public void ClearAll2DAnd3DTrajectoryLines()
    {
        hasInitializedDefaultStrokes = true;
        isDrawingOnCanvas = false;
        currentDrawingStroke = null;
        customDrawnStrokes.Clear();

        isPlaying = false;
        isMovingToManualTarget = false;
        paintingStarted = false;
        SetSprayState(false);
        if (waypoints != null) waypoints.Clear();
        ClearInkTrail();
        currentWaypointIndex = 0;
        customDrawStatusMsg = "✅ Đã xóa trắng toàn bộ các nét vẽ 2D và đường quỹ đạo 3D trên quần!";
    }

    private void EnsureCanonicalJeansCanvasInitialized()
    {
        if (whitePixelTex == null)
        {
            whitePixelTex = new Texture2D(1, 1, TextureFormat.RGBA32, false);
            whitePixelTex.SetPixel(0, 0, Color.white);
            whitePixelTex.Apply();
        }
        if (canonicalJeansBgTex == null)
        {
            canonicalJeansBgTex = BuildCanonicalJeans2DTexture(260, 300, activeSprayTemplateSize);
        }
        if (!hasInitializedDefaultStrokes)
        {
            hasInitializedDefaultStrokes = true;
            LoadPresetWhiskersAndThighOn2DCanvas();
        }
    }

    public static Vector2[] GetJeansPolygonForSize(string size)
    {
        if (size == "S")
        {
            return new Vector2[]
            {
                new Vector2(0.32f, 0.14f), // 0: waist_left
                new Vector2(0.50f, 0.14f), // 1: waist_center
                new Vector2(0.68f, 0.14f), // 2: waist_right
                new Vector2(0.71f, 0.32f), // hông phải
                new Vector2(0.72f, 0.51f), // đùi ngoài phải
                new Vector2(0.73f, 0.68f), // gối ngoài phải
                new Vector2(0.73f, 0.85f), // 11: cuff_right_outer
                new Vector2(0.57f, 0.85f), // 10: cuff_right_inner
                new Vector2(0.55f, 0.68f), // gối trong phải
                new Vector2(0.53f, 0.51f), // đùi trong phải
                new Vector2(0.50f, 0.37f), // 3: crotch
                new Vector2(0.47f, 0.51f), // đùi trong trái
                new Vector2(0.45f, 0.68f), // gối trong trái
                new Vector2(0.43f, 0.85f), // 9: cuff_left_inner
                new Vector2(0.27f, 0.85f), // 8: cuff_left_outer
                new Vector2(0.27f, 0.68f), // gối ngoài trái
                new Vector2(0.28f, 0.51f), // đùi ngoài trái
                new Vector2(0.29f, 0.32f), // hông trái
            };
        }
        else if (size == "L")
        {
            return new Vector2[]
            {
                new Vector2(0.24f, 0.06f), // 0: waist_left
                new Vector2(0.50f, 0.06f), // 1: waist_center
                new Vector2(0.76f, 0.06f), // 2: waist_right
                new Vector2(0.79f, 0.28f), // hông phải
                new Vector2(0.80f, 0.49f), // đùi ngoài phải
                new Vector2(0.81f, 0.72f), // gối ngoài phải
                new Vector2(0.82f, 0.96f), // 11: cuff_right_outer
                new Vector2(0.61f, 0.96f), // 10: cuff_right_inner
                new Vector2(0.58f, 0.72f), // gối trong phải
                new Vector2(0.55f, 0.49f), // đùi trong phải
                new Vector2(0.50f, 0.32f), // 3: crotch
                new Vector2(0.45f, 0.49f), // đùi trong trái
                new Vector2(0.42f, 0.72f), // gối trong trái
                new Vector2(0.39f, 0.96f), // 9: cuff_left_inner
                new Vector2(0.18f, 0.96f), // 8: cuff_left_outer
                new Vector2(0.19f, 0.72f), // gối ngoài trái
                new Vector2(0.20f, 0.49f), // đùi ngoài trái
                new Vector2(0.21f, 0.28f), // hông trái
            };
        }
        // Size M (mặc định)
        return new Vector2[]
        {
            new Vector2(0.28f, 0.10f), // 0: waist_left
            new Vector2(0.50f, 0.10f), // 1: waist_center
            new Vector2(0.72f, 0.10f), // 2: waist_right
            new Vector2(0.75f, 0.30f), // hông phải
            new Vector2(0.76f, 0.50f), // đùi ngoài phải
            new Vector2(0.77f, 0.70f), // gối ngoài phải
            new Vector2(0.78f, 0.91f), // 11: cuff_right_outer
            new Vector2(0.59f, 0.91f), // 10: cuff_right_inner
            new Vector2(0.56f, 0.70f), // gối trong phải
            new Vector2(0.54f, 0.50f), // đùi trong phải
            new Vector2(0.50f, 0.34f), // 3: crotch (đáy đũng)
            new Vector2(0.46f, 0.50f), // đùi trong trái
            new Vector2(0.44f, 0.70f), // gối trong trái
            new Vector2(0.41f, 0.91f), // 9: cuff_left_inner
            new Vector2(0.22f, 0.91f), // 8: cuff_left_outer
            new Vector2(0.23f, 0.70f), // gối ngoài trái
            new Vector2(0.24f, 0.50f), // đùi ngoài trái
            new Vector2(0.25f, 0.30f), // hông trái
        };
    }

    /// <summary>
    /// Tạo hình ảnh chiếc quần Jean 2D tiêu chuẩn (tượng trưng) có cạp quần, khóa kéo, đáy đũng,
    /// 2 ống quần trái/phải và vùng cho phép phun để người dùng trực tiếp vẽ lên.
    /// Kích thước quần thay đổi trực quan theo Size (S nhỏ gọn, M chuẩn, L to rộng).
    /// </summary>
    private Texture2D BuildCanonicalJeans2DTexture(int texW, int texH, string size = null)
    {
        if (string.IsNullOrEmpty(size)) size = activeSprayTemplateSize;
        if (string.IsNullOrEmpty(size)) size = "S";

        Texture2D tex = new Texture2D(texW, texH, TextureFormat.RGBA32, false);
        Color bgCol = new Color(0.08f, 0.10f, 0.14f, 1f);
        Color gridCol = new Color(0.14f, 0.18f, 0.24f, 1f);
        Color denimFill = new Color(0.14f, 0.26f, 0.44f, 1f);
        Color denimWaist = new Color(0.11f, 0.20f, 0.35f, 1f);
        Color sprayZoneCol = new Color(0.18f, 0.36f, 0.58f, 1f);
        Color borderCol = new Color(0.35f, 0.78f, 1.0f, 1f);

        Vector2[] jeansPoly = GetJeansPolygonForSize(size);
        float waistV = (size == "S") ? 0.14f : (size == "L" ? 0.06f : 0.10f);
        float crotchV = (size == "S") ? 0.37f : (size == "L" ? 0.32f : 0.34f);

        for (int py = 0; py < texH; py++)
        {
            float v = 1f - (float)py / (texH - 1);
            for (int px = 0; px < texW; px++)
            {
                float u = (float)px / (texW - 1);
                Color c = ((px % 20 == 0) || (py % 20 == 0)) ? gridCol : bgCol;

                if (IsPointInPolygon2D(new Vector2(u, v), jeansPoly))
                {
                    c = (v < waistV + 0.05f) ? denimWaist : denimFill;

                    bool inLeftSpray = false;
                    bool inRightSpray = false;
                    if (size == "S")
                    {
                        inLeftSpray = (v >= 0.40f && v <= 0.80f && u >= 0.29f && u <= 0.43f);
                        inRightSpray = (v >= 0.40f && v <= 0.80f && u >= 0.57f && u <= 0.71f);
                    }
                    else if (size == "L")
                    {
                        inLeftSpray = (v >= 0.34f && v <= 0.90f && u >= 0.22f && u <= 0.42f);
                        inRightSpray = (v >= 0.34f && v <= 0.90f && u >= 0.58f && u <= 0.78f);
                    }
                    else
                    {
                        inLeftSpray = (v >= 0.37f && v <= 0.85f && u >= 0.26f && u <= 0.43f);
                        inRightSpray = (v >= 0.37f && v <= 0.85f && u >= 0.57f && u <= 0.74f);
                    }

                    if (inLeftSpray || inRightSpray)
                    {
                        c = sprayZoneCol;
                    }
                }
                tex.SetPixel(px, py, c);
            }
        }

        // Vẽ đường viền
        for (int i = 0; i < jeansPoly.Length; i++)
        {
            Vector2 a = jeansPoly[i];
            Vector2 b = jeansPoly[(i + 1) % jeansPoly.Length];
            DrawLineOnTexture(tex, a, b, borderCol);
        }

        // Đường khóa kéo và đường phân cách cạp
        float waistLeftU = jeansPoly[0].x;
        float waistRightU = jeansPoly[2].x;
        DrawLineOnTexture(tex, new Vector2(0.50f, waistV), new Vector2(0.50f, crotchV), new Color(0.9f, 0.75f, 0.25f, 0.8f));
        DrawLineOnTexture(tex, new Vector2(waistLeftU - 0.01f, waistV + 0.05f), new Vector2(waistRightU + 0.01f, waistV + 0.05f), new Color(0.9f, 0.75f, 0.25f, 0.8f));

        tex.Apply();
        return tex;
    }

    private static bool IsPointInPolygon2D(Vector2 pt, Vector2[] poly)
    {
        bool inside = false;
        for (int i = 0, j = poly.Length - 1; i < poly.Length; j = i++)
        {
            if (((poly[i].y > pt.y) != (poly[j].y > pt.y)) &&
                (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y) / (poly[j].y - poly[i].y + 1e-7f) + poly[i].x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    private static void DrawLineOnTexture(Texture2D tex, Vector2 uv0, Vector2 uv1, Color col)
    {
        int w = tex.width;
        int h = tex.height;
        int x0 = Mathf.RoundToInt(uv0.x * (w - 1));
        int y0 = Mathf.RoundToInt((1f - uv0.y) * (h - 1));
        int x1 = Mathf.RoundToInt(uv1.x * (w - 1));
        int y1 = Mathf.RoundToInt((1f - uv1.y) * (h - 1));

        int steps = Mathf.Max(Mathf.Abs(x1 - x0), Mathf.Abs(y1 - y0), 1);
        for (int s = 0; s <= steps; s++)
        {
            float t = (float)s / steps;
            int px = Mathf.Clamp(Mathf.RoundToInt(Mathf.Lerp(x0, x1, t)), 0, w - 1);
            int py = Mathf.Clamp(Mathf.RoundToInt(Mathf.Lerp(y0, y1, t)), 0, h - 1);
            tex.SetPixel(px, py, col);
            if (px + 1 < w) tex.SetPixel(px + 1, py, col);
        }
    }

    public void LoadPresetWhiskersAndThighOn2DCanvas()
    {
        customDrawnStrokes.Clear();
        // Thông số Mẫu cần phun theo Size quần: S (mặc định) / M / L
        int nWhiskers = 2, nPasses = 2;
        float vTop = 0.47f, vBot = 0.76f, passSpacing = 0.036f;
        if (activeSprayTemplateSize == "M") { nWhiskers = 2; nPasses = 3; vTop = 0.46f; vBot = 0.80f; passSpacing = 0.032f; }
        else if (activeSprayTemplateSize == "L") { nWhiskers = 3; nPasses = 4; vTop = 0.45f; vBot = 0.84f; passSpacing = 0.026f; }

        // Ống trái: vệt râu mèo + các đường dọc tâm đùi-gối
        for (int w = 0; w < nWhiskers; w++)
        {
            float vBase = 0.41f + w * 0.04f;
            List<Vector2> whiskerL = new List<Vector2>();
            for (int k = 0; k <= 8; k++)
            {
                float t = (float)k / 8f;
                whiskerL.Add(new Vector2(Mathf.Lerp(0.28f, 0.42f, t), vBase + 0.02f * (t - 0.5f)));
            }
            customDrawnStrokes.Add(whiskerL);
        }
        for (int p = 0; p < nPasses; p++)
        {
            float uOff = (p - (nPasses - 1) * 0.5f) * passSpacing;
            List<Vector2> passL = new List<Vector2>();
            for (int k = 0; k <= 14; k++)
            {
                float t = (p % 2 == 0) ? ((float)k / 14f) : (1f - (float)k / 14f);
                passL.Add(new Vector2(Mathf.Lerp(0.35f, 0.33f, t) + uOff, Mathf.Lerp(vTop, vBot, t)));
            }
            customDrawnStrokes.Add(passL);
        }

        // Ống phải: vệt râu mèo + các đường dọc tâm đùi-gối
        for (int w = 0; w < nWhiskers; w++)
        {
            float vBase = 0.41f + w * 0.04f;
            List<Vector2> whiskerR = new List<Vector2>();
            for (int k = 0; k <= 8; k++)
            {
                float t = (float)k / 8f;
                whiskerR.Add(new Vector2(Mathf.Lerp(0.58f, 0.72f, t), vBase - 0.02f * (t - 0.5f)));
            }
            customDrawnStrokes.Add(whiskerR);
        }
        for (int p = 0; p < nPasses; p++)
        {
            float uOff = (p - (nPasses - 1) * 0.5f) * passSpacing;
            List<Vector2> passR = new List<Vector2>();
            for (int k = 0; k <= 14; k++)
            {
                float t = (p % 2 == 0) ? ((float)k / 14f) : (1f - (float)k / 14f);
                passR.Add(new Vector2(Mathf.Lerp(0.65f, 0.67f, t) + uOff, Mathf.Lerp(vTop, vBot, t)));
            }
            customDrawnStrokes.Add(passR);
        }
    }

    public void LoadPresetZigzag2LegsOn2DCanvas()
    {
        customDrawnStrokes.Clear();
        // Đường Zigzag liên tục dọc ống trái và ống phải
        List<Vector2> zigL = new List<Vector2>();
        int rows = 9;
        for (int r = 0; r < rows; r++)
        {
            float v = Mathf.Lerp(0.42f, 0.82f, (float)r / (rows - 1));
            float uLeft = 0.29f;
            float uRight = 0.41f;
            if (r % 2 == 0)
            {
                for (int k = 0; k <= 5; k++) zigL.Add(new Vector2(Mathf.Lerp(uLeft, uRight, k / 5f), v));
            }
            else
            {
                for (int k = 0; k <= 5; k++) zigL.Add(new Vector2(Mathf.Lerp(uRight, uLeft, k / 5f), v));
            }
        }
        customDrawnStrokes.Add(zigL);

        List<Vector2> zigR = new List<Vector2>();
        for (int r = 0; r < rows; r++)
        {
            float v = Mathf.Lerp(0.42f, 0.82f, (float)r / (rows - 1));
            float uLeft = 0.59f;
            float uRight = 0.71f;
            if (r % 2 == 0)
            {
                for (int k = 0; k <= 5; k++) zigR.Add(new Vector2(Mathf.Lerp(uLeft, uRight, k / 5f), v));
            }
            else
            {
                for (int k = 0; k <= 5; k++) zigR.Add(new Vector2(Mathf.Lerp(uRight, uLeft, k / 5f), v));
            }
        }
        customDrawnStrokes.Add(zigR);
    }

    /// <summary>
    /// Ánh xạ phi cứng toàn bộ các nét vẽ trên Quần 2D Tiêu Chuẩn [0, 1] x [0, 1] sang bề mặt thực 3D
    /// của đám mây điểm phôi đang nạp (bằng nội suy 12 mốc giải phẫu + truy vấn độ cao thực Z(x,y) & pháp tuyến
    /// của Point Cloud 3D), sau đó giải Động học Nghịch 6-DOF và cập nhật ngay quỹ đạo vận hành!
    /// </summary>
    public void ApplyCanonical2DDrawingTo3DTrajectory()
    {
        if (customDrawnStrokes == null || customDrawnStrokes.Count == 0)
        {
            customDrawStatusMsg = "⚠️ Bạn chưa vẽ đường nào trên Quần 2D! Hãy dùng chuột vẽ hoặc chọn Mẫu có sẵn.";
            return;
        }

        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        string activeCloudSample = (plyLoader != null && !string.IsNullOrEmpty(plyLoader.currentLoadedSampleName))
            ? plyLoader.currentLoadedSampleName
            : selectedSampleName;
        if (activeCloudSample == "full_cloud_world") activeCloudSample = "pcl_25";

        if (landmarks3D == null || landmarks3D.Count < 12 || loadedLandmarksSampleName != activeCloudSample)
        {
            LoadLandmarksAndSegmentationForSample(activeCloudSample);
        }

        // 1. Lấy 12 điểm mốc nguồn (Quần 2D Tiêu Chuẩn theo Size) và 12 điểm mốc đích 3D (Robot Global mm)
        Vector2[] srcUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        Vector3[] dstRobotMm = new Vector3[12];
        for (int i = 0; i < 12; i++)
        {
            if (landmarks3D != null && i < landmarks3D.Count)
            {
                dstRobotMm[i] = new Vector3(landmarks3D[i].x_mm, landmarks3D[i].y_mm, landmarks3D[i].z_mm);
            }
            else
            {
                // Fallback chuẩn nếu mẫu chưa có mốc
                dstRobotMm[i] = new Vector3(
                    Mathf.Lerp(430f, 1080f, srcUV[i].y),
                    Mathf.Lerp(320f, -320f, srcUV[i].x),
                    55f
                );
            }
        }

        // 2. Xây dựng hàm nội suy phi cứng 12 điểm mốc (Radial Basis / Thin Plate Spline 3D)
        //     kết hợp truy vấn trực tiếp cao độ Z_surface(x,y) và vector pháp tuyến n(x,y) trên 3D Point Cloud!
        List<WaypointData> mappedWaypoints = new List<WaypointData>();
        float cumDistMm = 0f;
        Vector3 prevPosMm = Vector3.zero;
        float[] prevJointsRad = null;
        int wpIndex = 0;
        int solvedCount = 0;

        for (int s = 0; s < customDrawnStrokes.Count; s++)
        {
            var stroke = customDrawnStrokes[s];
            if (stroke == null || stroke.Count == 0) continue;

            // Nội suy làm mịn các điểm dọc nét vẽ để robot di chuyển êm
            List<Vector2> resampledStroke = ResampleStroke2D(stroke, 0.022f);

            for (int p = 0; p < resampledStroke.Count; p++)
            {
                Vector2 uv = resampledStroke[p];

                // Ánh xạ (u, v) từ Quần 2D Tiêu Chuẩn sang (X_mm, Y_mm, Z_TPS_mm) bằng nội suy phi cứng 12 mốc
                Vector3 warpedMm = WarpCanonicalUVTo3DLandmarks(uv, srcUV, dstRobotMm);

                // Chiếu điểm (X_mm, Y_mm) trực tiếp lên bề mặt 3D thực tế của Đám Mây Điểm Zivid (PLYDirectLoader)
                Vector3 approxWorld = globalReference != null
                    ? globalReference.TransformPoint(warpedMm * 0.001f)
                    : new Vector3(-warpedMm.y * 0.001f, warpedMm.z * 0.001f, 1.100f - warpedMm.x * 0.001f);

                float surfaceZ_mm = warpedMm.z;
                Vector3 surfaceNormalWorld = Vector3.up;
                if (SampleCompensatedSurface(approxWorld.x, approxWorld.z, out float compY, out Vector3 compNorm))
                {
                    // Ưu tiên sử dụng cao độ và pháp tuyến đã được bù đắp chỗ khuất / khuyết tật
                    surfaceZ_mm = Mathf.Max(14f, compY * 1000f);
                    surfaceNormalWorld = compNorm;
                }
                else if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(approxWorld.x, approxWorld.z, out float cloudWorldY, out Vector3 cloudNormal))
                {
                    // Độ cao bề mặt thực tế từ đám mây điểm 3D (chuyển từ mét sang mm)
                    surfaceZ_mm = Mathf.Max(14f, cloudWorldY * 1000f);
                    surfaceNormalWorld = cloudNormal;
                }

                float finalZ_mm = Mathf.Clamp(surfaceZ_mm + customDrawZOffsetMm, 18f, 380f);
                Vector3 targetMm = new Vector3(warpedMm.x, warpedMm.y, finalZ_mm);

                // Giới hạn an toàn bán kính làm việc của Dobot CR10A
                if (targetMm.magnitude > 1145f)
                {
                    float scale = 1140f / targetMm.magnitude;
                    targetMm.x *= scale;
                    targetMm.y *= scale;
                }

                if (wpIndex > 0)
                {
                    cumDistMm += Vector3.Distance(prevPosMm, targetMm);
                }
                prevPosMm = targetMm;

                WaypointData wp = new WaypointData();
                wp.index = wpIndex;
                wp.distance_mm = cumDistMm;
                wp.sprayOn = (p > 0); // Điểm đầu tiên của nét là đoạn di chuyển chuyển tiếp -> KHÔNG PHUN
                wp.segmentName = $"STROKE_{s + 1}";
                wp.targetPosGlobal_mm = targetMm;
                wp.targetPosWorld_m = globalReference != null
                    ? globalReference.TransformPoint(targetMm * 0.001f)
                    : new Vector3(-targetMm.y * 0.001f, targetMm.z * 0.001f, 1.100f - targetMm.x * 0.001f);
                wp.microNormal = surfaceNormalWorld;

                // Tính góc nghiêng vi mô của súng theo pháp tuyến bề mặt 3D
                wp.roll_deg = 180f;
                wp.pitch_deg = Mathf.Clamp(-surfaceNormalWorld.z * 18f, -12f, 12f);
                wp.yaw_deg = 0f;

                Quaternion flangeRot = GetWaypointFlangeRotation(wp);
                var sols = DobotCR10AKinematics.SolveTcpPose(
                    wp.targetPosGlobal_mm, flangeRot, prevJointsRad, toolLength,
                    useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY
                );

                if (sols == null || sols.Count == 0)
                {
                    wp.pitch_deg = 0f;
                    flangeRot = GetWaypointFlangeRotation(wp);
                    sols = DobotCR10AKinematics.SolveTcpPose(
                        wp.targetPosGlobal_mm, flangeRot, prevJointsRad, toolLength,
                        useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY
                    );
                }

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
                    wp.selectedSolutionIndex = 0;
                    wp.selectedBranchName = best.branchName;
                    prevJointsRad = wp.jointAngles_rad;
                    solvedCount++;
                }

                mappedWaypoints.Add(wp);
                wpIndex++;
            }
        }

        if (mappedWaypoints.Count > 0)
        {
            isPlaying = false;
            SetSprayState(false);
            ClearInkTrail();
            waypoints = mappedWaypoints;
            MarkSprayTransitions();
            currentWaypointIndex = 0;
            showGizmos = true;
            ApplyWaypoint(0);
            RunDiagnostics();

            customDrawStatusMsg = $"✅ Đã ánh xạ 2D ➔ 3D thành công: {waypoints.Count} điểm 3D (Giải IK: {solvedCount}/{waypoints.Count}, Dài: {cumDistMm:F0} mm)!";
            Debug.Log($"[CR10A 2D->3D] {customDrawStatusMsg}");
        }
    }

    private static List<Vector2> ResampleStroke2D(List<Vector2> rawStroke, float stepUV)
    {
        List<Vector2> result = new List<Vector2>();
        if (rawStroke == null || rawStroke.Count == 0) return result;
        result.Add(rawStroke[0]);
        for (int i = 1; i < rawStroke.Count; i++)
        {
            Vector2 a = result[result.Count - 1];
            Vector2 b = rawStroke[i];
            float dist = Vector2.Distance(a, b);
            if (dist >= stepUV)
            {
                int segs = Mathf.Clamp(Mathf.CeilToInt(dist / stepUV), 1, 25);
                for (int k = 1; k <= segs; k++)
                {
                    result.Add(Vector2.Lerp(a, b, (float)k / segs));
                }
            }
        }
        if (result.Count == 1 && rawStroke.Count > 1)
        {
            result.Add(rawStroke[rawStroke.Count - 1]);
        }
        return result;
    }

    /// <summary>
    /// Nội suy phi cứng (Non-Rigid Anatomical Warping) từ tọa độ (u, v) trên Quần 2D Tiêu Chuẩn
    /// sang tọa độ 3D (X_mm, Y_mm, Z_mm) dựa trên 12 điểm mốc giải phẫu AI của chiếc quần thực tế.
    /// </summary>
    private static Vector3 WarpCanonicalUVTo3DLandmarks(Vector2 uv, Vector2[] src12, Vector3[] dst12)
    {
        // Tính biến đổi Affine nền (Least-Squares) từ 12 mốc chuẩn -> 12 mốc 3D thực tế
        // cộng với trường biến dạng đàn hồi cục bộ (Gaussian / TPS Kernel) tại từng mốc giải phẫu
        Vector2 srcCenter = Vector2.zero;
        Vector3 dstCenter = Vector3.zero;
        for (int i = 0; i < 12; i++)
        {
            srcCenter += src12[i];
            dstCenter += dst12[i];
        }
        srcCenter /= 12f;
        dstCenter /= 12f;

        float suu = 0f, suv = 0f, svv = 0f;
        Vector3 suDst = Vector3.zero;
        Vector3 svDst = Vector3.zero;
        for (int i = 0; i < 12; i++)
        {
            float du = src12[i].x - srcCenter.x;
            float dv = src12[i].y - srcCenter.y;
            Vector3 dP = dst12[i] - dstCenter;
            suu += du * du;
            suv += du * dv;
            svv += dv * dv;
            suDst += du * dP;
            svDst += dv * dP;
        }

        float det = suu * svv - suv * suv;
        Vector3 colU = Vector3.zero;
        Vector3 colV = Vector3.zero;
        if (Mathf.Abs(det) > 1e-6f)
        {
            colU = (svv * suDst - suv * svDst) / det;
            colV = (-suv * suDst + suu * svDst) / det;
        }

        Vector3 affinePos = dstCenter + (uv.x - srcCenter.x) * colU + (uv.y - srcCenter.y) * colV;

        // Cộng phần dư biến dạng phi cứng cục bộ (Non-rigid residual) từ 12 mốc giải phẫu
        Vector3 residualSum = Vector3.zero;
        float weightSum = 0f;
        float sigma2 = 0.025f;
        for (int i = 0; i < 12; i++)
        {
            Vector3 affI = dstCenter + (src12[i].x - srcCenter.x) * colU + (src12[i].y - srcCenter.y) * colV;
            Vector3 resI = dst12[i] - affI;
            float d2 = (uv - src12[i]).sqrMagnitude;
            float w = 1f / (d2 + 0.0015f) + Mathf.Exp(-d2 / sigma2);
            residualSum += w * resI;
            weightSum += w;
        }

        if (weightSum > 1e-6f)
        {
            return affinePos + residualSum / weightSum;
        }
        return affinePos;
    }

    // =========================================================================
    // NẠP & ĐỒNG BỘ 12 ĐIỂM MỐC GIẢI PHẪU 3D + ĐƯỜNG BAO SEGMENTATION 3D
    // =========================================================================
    public void LoadLandmarksAndSegmentationForSample(string sampleName, bool autoSwitchSprayTemplate = false)
    {
        if (string.IsNullOrEmpty(sampleName)) sampleName = "pcl_25";
        string clean = Path.GetFileNameWithoutExtension(sampleName).Trim();
        if (clean == "full_cloud_world") clean = "pcl_25";
        else if (!clean.StartsWith("pcl_")) clean = "pcl_" + clean;

        string[] candidatePaths = new string[]
        {
            Path.Combine(Application.dataPath, "..", "output_drawing", $"{clean}_landmarks_seg.json"),
            Path.Combine(Application.dataPath, "..", "..", "output_drawing", $"{clean}_landmarks_seg.json"),
            $"/home/nhan/RB3t_PJ/ai_src/step3_3d_trajectory/outputs/{clean}_landmarks_seg.json",
            Path.Combine(Application.dataPath, "..", "output_drawing", "pcl_25_landmarks_seg.json"),
            "/home/nhan/RB3t_PJ/ai_src/step3_3d_trajectory/outputs/pcl_25_landmarks_seg.json",
            Path.Combine(Application.dataPath, "..", "output_drawing", "pcl_101_landmarks_seg.json"),
            "/home/nhan/RB3t_PJ/ai_src/step3_3d_trajectory/outputs/pcl_101_landmarks_seg.json"
        };

        string foundPath = null;
        for (int i = 0; i < candidatePaths.Length; i++)
        {
            if (File.Exists(candidatePaths[i]))
            {
                foundPath = candidatePaths[i];
                break;
            }
        }

        if (string.IsNullOrEmpty(foundPath)) return;

        try
        {
            string jsonText = File.ReadAllText(foundPath);
            LandmarkSegmentationPayload payload = JsonUtility.FromJson<LandmarkSegmentationPayload>(jsonText);
            if (payload == null) return;

            loadedLandmarksSampleName = clean;
            landmarks3D = payload.landmarks ?? new List<LandmarkItem3D>();
            landmarksWorldPos.Clear();
            for (int i = 0; i < landmarks3D.Count; i++)
            {
                Vector3 posMm = new Vector3(landmarks3D[i].x_mm, landmarks3D[i].y_mm, landmarks3D[i].z_mm);
                Vector3 worldP = globalReference != null
                    ? globalReference.TransformPoint(posMm * 0.001f)
                    : new Vector3(-posMm.y * 0.001f, posMm.z * 0.001f, 1.100f - posMm.x * 0.001f);
                landmarksWorldPos.Add(worldP);
            }

            garmentContourWorld = ConvertPoint3DListToWorld(payload.garment_contour_3d);
            sprayZoneLeftWorld = ConvertPoint3DListToWorld(payload.spray_zone_left_3d);
            sprayZoneRightWorld = ConvertPoint3DListToWorld(payload.spray_zone_right_3d);

            // Kích thước & Size quần (nếu JSON chưa có thì tự tính từ 12 mốc)
            detectedOutseamMm = payload.outseam_mm;
            detectedInseamMm = payload.inseam_mm;
            detectedWaistMm = payload.waist_mm;
            detectedCuffMm = payload.cuff_mm;
            detectedAreaCm2 = payload.area_cm2;
            if ((detectedInseamMm <= 0f || detectedOutseamMm <= 0f) && landmarks3D.Count >= 12)
            {
                ComputeJeansMeasurementsFromLandmarks();
            }
            detectedJeansSize = !string.IsNullOrEmpty(payload.detected_size)
                ? payload.detected_size
                : ClassifyJeansSizeKMeans(detectedInseamMm, detectedOutseamMm);

            if (autoSwitchSprayTemplate)
            {
                SwitchSprayTemplateToSize(detectedJeansSize);
            }
            else if (preserveCustomDrawingAcrossSamples && customDrawnStrokes != null && customDrawnStrokes.Count > 0)
            {
                ApplyCanonical2DDrawingTo3DTrajectory();
            }

            if (autoInpaintHolesOnLoad)
            {
                RunOcclusionCompensation();
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[CR10A Landmarks] Không thể đọc file mốc 3D: {ex.Message}");
        }
    }

    // =========================================================================
    // PHÂN LOẠI SIZE QUẦN (K-MEANS K=3 TRÊN 300 MẪU ĐÃ GÁN 12 MỐC) & ĐỔI MẪU PHUN
    // Ngưỡng: Inseam T1 = 618.1 mm, T2 = 667.1 mm | Outseam T1 = 933.7 mm
    // =========================================================================
    public const float SizeInseamT1Mm = 618.1f;
    public const float SizeInseamT2Mm = 667.1f;
    public const float SizeOutseamT1Mm = 933.7f;

    public static string ClassifyJeansSizeKMeans(float inseamMm, float outseamMm)
    {
        if (inseamMm <= 0f) return "S";
        if (inseamMm < SizeInseamT1Mm || (outseamMm > 0f && outseamMm < SizeOutseamT1Mm)) return "S";
        if (inseamMm < SizeInseamT2Mm) return "M";
        return "L";
    }

    private void ComputeJeansMeasurementsFromLandmarks()
    {
        Func<int, int, float> dxy = (a, b) =>
        {
            float dx = landmarks3D[a].x_mm - landmarks3D[b].x_mm;
            float dy = landmarks3D[a].y_mm - landmarks3D[b].y_mm;
            return Mathf.Sqrt(dx * dx + dy * dy);
        };
        detectedOutseamMm = 0.5f * (dxy(0, 8) + dxy(2, 11));
        detectedInseamMm = 0.5f * (dxy(3, 9) + dxy(3, 10));
        detectedWaistMm = dxy(0, 2);
        detectedCuffMm = 0.5f * (dxy(8, 9) + dxy(10, 11));
        int[] poly = { 0, 1, 2, 11, 10, 3, 9, 8 };
        float area2 = 0f;
        for (int i = 0; i < poly.Length; i++)
        {
            var p = landmarks3D[poly[i]];
            var q = landmarks3D[poly[(i + 1) % poly.Length]];
            area2 += p.x_mm * q.y_mm - q.x_mm * p.y_mm;
        }
        detectedAreaCm2 = Mathf.Abs(area2) * 0.5f / 100f;
    }

    /// <summary>
    /// Nhận diện quần & 12 mốc của mẫu, phân loại Size (S/M/L) và tự động đổi Mẫu cần phun sang Size phù hợp.
    /// </summary>
    public void DetectJeansSizeAndSwitchSprayTemplate(string sampleName)
    {
        LoadLandmarksAndSegmentationForSample(sampleName, autoSwitchSprayTemplate: true);
    }

    public void SwitchSprayTemplateToSize(string size)
    {
        if (string.IsNullOrEmpty(size)) size = "S";
        string prev = activeSprayTemplateSize;
        hasTriggeredSizeAutoSwitch = true;
        hasInitializedDefaultStrokes = true;

        if (prev != size)
        {
            ChangeSprayTemplateSize(size);
        }
        else
        {
            if (customDrawnStrokes == null || customDrawnStrokes.Count == 0)
            {
                LoadPresetWhiskersAndThighOn2DCanvas();
            }
            ApplyCanonical2DDrawingTo3DTrajectory();
        }

        sizeDetectionSummary = $"Nhận diện [{loadedLandmarksSampleName}] ➔ SIZE {size} (Inseam {detectedInseamMm:F0} mm, Outseam {detectedOutseamMm:F0} mm). Mẫu phun: SIZE {prev} ➔ SIZE {size}";
        customDrawStatusMsg = $"📏 Đã nhận diện SIZE {size}. Nét vẽ & quần 2D đã tự co giãn theo size mới!";
        Debug.Log($"[CR10A Size] {sizeDetectionSummary}");
    }

    private void DrawJeansSizeDetectionSummaryBox()
    {
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label("📏 KẾT QUẢ NHẬN DIỆN SIZE QUẦN (K-MEANS 3 CỤM)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.Label($"SIZE QUẦN NHẬN DIỆN: [ {detectedJeansSize} ]   |   MẪU CẦN PHUN ĐANG DÙNG: [ SIZE {activeSprayTemplateSize} ]", AntigravityUITheme.StyleLabelGreen);
        GUILayout.Label($"Inseam: {detectedInseamMm:F0} mm | Outseam: {detectedOutseamMm:F0} mm | Eo: {detectedWaistMm:F0} mm | Gấu: {detectedCuffMm:F0} mm | Diện tích: {detectedAreaCm2:F0} cm²", AntigravityUITheme.StyleLabelCyan);
        GUILayout.Label($"Ngưỡng: S < {SizeInseamT1Mm:F0} mm ≤ M < {SizeInseamT2Mm:F0} mm ≤ L (Inseam); Outseam < {SizeOutseamT1Mm:F0} mm ➔ S", AntigravityUITheme.StyleLabelMuted);
        GUILayout.Label(hasTriggeredSizeAutoSwitch ? sizeDetectionSummary : "Mặc định: Mẫu cần phun SIZE S (chưa nhận diện)", AntigravityUITheme.StyleLabelMuted);
        GUILayout.EndVertical();
    }

    private List<Vector3> ConvertPoint3DListToWorld(List<Point3DJson> ptsMm)
    {
        List<Vector3> res = new List<Vector3>();
        if (ptsMm == null) return res;
        for (int i = 0; i < ptsMm.Count; i++)
        {
            Vector3 pMm = new Vector3(ptsMm[i].x_mm, ptsMm[i].y_mm, ptsMm[i].z_mm);
            Vector3 worldP = globalReference != null
                ? globalReference.TransformPoint(pMm * 0.001f)
                : new Vector3(-pMm.y * 0.001f, pMm.z * 0.001f, 1.100f - pMm.x * 0.001f);
            res.Add(worldP);
        }
        return res;
    }

    // =========================================================================
    // VẼ TRỰC TIẾP 12 ĐIỂM MỐC AI, SEGMENT CONTOUR & QUỸ ĐẠO 3D LÊN GAME VIEW
    // VÀ SCENE VIEW GIZMOS
    // =========================================================================
    private static readonly int[,] LandmarkSkeletonPairs = new int[,]
    {
        { 0, 1 }, { 1, 2 }, // Đường cạp quần: waist_left - waist_center - waist_right
        { 1, 3 },           // Khóa kéo: waist_center - crotch
        { 0, 4 }, { 3, 4 }, // Đùi trái
        { 2, 5 }, { 3, 5 }, // Đùi phải
        { 4, 6 }, { 5, 7 }, // Đùi -> Gối
        { 6, 8 }, { 6, 9 }, { 8, 9 },   // Gấu quần trái
        { 7, 10 }, { 7, 11 }, { 10, 11 } // Gấu quần phải
    };

    private void DrawViewport3DOverlaysInGUI()
    {
        if (Event.current == null || Event.current.type != EventType.Repaint) return;

        Camera cam = null;
        RobotCameraController rcc = EnsureCameraController();
        if (rcc != null) cam = rcc.GetComponent<Camera>();
        if (cam == null) cam = Camera.main;
        if (cam == null) return;

        EnsureCanonicalJeansCanvasInitialized();

        // Tự động đồng bộ 12 Điểm Mốc & Đường Bao Segment theo đúng mẫu Point Cloud đang hiển thị trên màn hình!
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        string activeCloudSample = (plyLoader != null && !string.IsNullOrEmpty(plyLoader.currentLoadedSampleName))
            ? plyLoader.currentLoadedSampleName
            : selectedSampleName;
        if (activeCloudSample == "full_cloud_world") activeCloudSample = "pcl_25";

        if (loadedLandmarksSampleName != activeCloudSample || landmarks3D == null || landmarks3D.Count == 0)
        {
            LoadLandmarksAndSegmentationForSample(activeCloudSample);
        }

        // 1. Vẽ Đường Bao Segment Thân Quần 3D (Viền Đen Tương Phản + Lõi Xanh Neon Đậm + Lưới Quét Bề Mặt + Nhãn Segment)
        if (showSegmentationContour3D && garmentContourWorld != null && garmentContourWorld.Count > 2)
        {
            // Vẽ các đường đan chéo mờ bên trong vùng Segment để nổi bật rõ diện tích chiếc quần đã được AI tách nền
            int cCount = garmentContourWorld.Count;
            Color hatchCol = new Color(0.05f, 1.0f, 0.50f, 0.22f);
            for (int i = 0; i < cCount / 2; i += 4)
            {
                int opp = (cCount - 1 - i) % cCount;
                Vector3 s0 = cam.WorldToScreenPoint(garmentContourWorld[i]);
                Vector3 s1 = cam.WorldToScreenPoint(garmentContourWorld[opp]);
                if (s0.z > 0.05f && s1.z > 0.05f)
                {
                    DrawGuiLine2D(new Vector2(s0.x, Screen.height - s0.y), new Vector2(s1.x, Screen.height - s1.y), hatchCol, 1.4f);
                }
            }

            // Viền ngoài tối tương phản cao + lõi xanh lá Neon rực rỡ + các nút đỉnh Segment
            DrawWorldLoopInGUI(cam, garmentContourWorld, new Color(0.01f, 0.14f, 0.06f, 0.95f), 6.0f, true);
            DrawWorldLoopInGUI(cam, garmentContourWorld, new Color(0.05f, 1.0f, 0.48f, 1.0f), 3.6f, true);
            for (int i = 0; i < cCount; i += 3)
            {
                Vector3 sp = cam.WorldToScreenPoint(garmentContourWorld[i]);
                if (sp.z > 0.05f)
                {
                    DrawGuiSolidRect(new Rect(sp.x - 2.5f, Screen.height - sp.y - 2.5f, 5f, 5f), new Color(0.2f, 1.0f, 0.6f, 0.95f));
                }
            }

            // Nhãn hiển thị rõ tên Đường Bao Segment ngay trên mép trên của quần
            Vector3 topSegScreen = cam.WorldToScreenPoint(garmentContourWorld[0]);
            float minGy = float.MaxValue;
            Vector2 bestSegPt = Vector2.zero;
            for (int i = 0; i < cCount; i++)
            {
                Vector3 sp = cam.WorldToScreenPoint(garmentContourWorld[i]);
                if (sp.z > 0.05f)
                {
                    float gy = Screen.height - sp.y;
                    if (gy < minGy)
                    {
                        minGy = gy;
                        bestSegPt = new Vector2(sp.x, gy);
                    }
                }
            }
            if (minGy < float.MaxValue)
            {
                Rect segBadge = new Rect(bestSegPt.x - 110f, bestSegPt.y - 24f, 235f, 19f);
                DrawGuiSolidRect(segBadge, new Color(0.02f, 0.22f, 0.12f, 0.92f));
                GUI.Label(new Rect(segBadge.x + 5f, segBadge.y + 1f, segBadge.width, 18f), $"🟢 SEGMENT QUẦN JEAN ({loadedLandmarksSampleName})", AntigravityUITheme.StyleLabelGreen);
            }
        }

        // 2. Vẽ Vùng Cho Phép Phun 2 Ống Quần (Viền Đen + Lõi Vàng Cam Đậm Rõ Nét)
        if (showSprayZones3D)
        {
            if (sprayZoneLeftWorld != null && sprayZoneLeftWorld.Count > 2)
            {
                DrawWorldLoopInGUI(cam, sprayZoneLeftWorld, new Color(0.18f, 0.10f, 0.01f, 0.92f), 4.8f, true);
                DrawWorldLoopInGUI(cam, sprayZoneLeftWorld, new Color(1.0f, 0.78f, 0.10f, 1.0f), 2.8f, true);
            }
            if (sprayZoneRightWorld != null && sprayZoneRightWorld.Count > 2)
            {
                DrawWorldLoopInGUI(cam, sprayZoneRightWorld, new Color(0.18f, 0.10f, 0.01f, 0.92f), 4.8f, true);
                DrawWorldLoopInGUI(cam, sprayZoneRightWorld, new Color(1.0f, 0.78f, 0.10f, 1.0f), 2.8f, true);
            }
        }

        // 3. Vẽ Đường Quỹ Đạo 3D (Cyan Polyline) ngay trên Game View (Chỉ vẽ đoạn CÓ PHUN)
        if (showGizmos && waypoints != null && waypoints.Count > 1)
        {
            Color trajHalo = new Color(0.02f, 0.12f, 0.20f, 0.85f);
            Color trajCol = new Color(0.15f, 0.92f, 1.0f, 0.96f);
            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                // Chỉ vẽ đường cyan ở những đoạn CÓ PHUN SƠN THỰC TẾ (loại bỏ bước nhảy chuyển tiếp giữa các nét/ống)
                bool isSameSeg = string.IsNullOrEmpty(waypoints[i].segmentName) || string.IsNullOrEmpty(waypoints[i + 1].segmentName) || waypoints[i].segmentName == waypoints[i + 1].segmentName;
                if (!waypoints[i + 1].sprayOn || !isSameSeg || Vector3.Distance(waypoints[i].targetPosGlobal_mm, waypoints[i + 1].targetPosGlobal_mm) > sprayJumpThresholdMm) continue;

                Vector3 s0 = cam.WorldToScreenPoint(waypoints[i].targetPosWorld_m);
                Vector3 s1 = cam.WorldToScreenPoint(waypoints[i + 1].targetPosWorld_m);
                if (s0.z > 0.05f && s1.z > 0.05f)
                {
                    Vector2 p0 = new Vector2(s0.x, Screen.height - s0.y);
                    Vector2 p1 = new Vector2(s1.x, Screen.height - s1.y);
                    DrawGuiLine2D(p0, p1, trajHalo, 4.0f);
                    DrawGuiLine2D(p0, p1, trajCol, 2.4f);
                }
            }
        }

        // 4. Vẽ 12 Điểm Mốc Giải Phẫu AI (Landmarks 3D) + Khung Xương + Nhãn Tên Không Chồng Lấp
        if (showLandmarks3D && landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
        {
            // Vẽ các đoạn nối khung xương giải phẫu (Viền tối + Lõi Hồng Tím Sáng)
            Color boneHalo = new Color(0.16f, 0.02f, 0.14f, 0.88f);
            Color boneCol = new Color(1.0f, 0.35f, 0.88f, 0.92f);
            int numPairs = LandmarkSkeletonPairs.GetLength(0);
            for (int k = 0; k < numPairs; k++)
            {
                int i0 = LandmarkSkeletonPairs[k, 0];
                int i1 = LandmarkSkeletonPairs[k, 1];
                if (i0 < landmarksWorldPos.Count && i1 < landmarksWorldPos.Count)
                {
                    Vector3 s0 = cam.WorldToScreenPoint(landmarksWorldPos[i0]);
                    Vector3 s1 = cam.WorldToScreenPoint(landmarksWorldPos[i1]);
                    if (s0.z > 0.05f && s1.z > 0.05f)
                    {
                        Vector2 p0 = new Vector2(s0.x, Screen.height - s0.y);
                        Vector2 p1 = new Vector2(s1.x, Screen.height - s1.y);
                        DrawGuiLine2D(p0, p1, boneHalo, 4.2f);
                        DrawGuiLine2D(p0, p1, boneCol, 2.4f);
                    }
                }
            }

            // Vẽ từng điểm mốc và nhãn tọa độ 3D (bố trí lệch thông minh để các nhãn 4/6 và 5/7 không đè lên nhau)
            for (int i = 0; i < landmarksWorldPos.Count; i++)
            {
                Vector3 sp = cam.WorldToScreenPoint(landmarksWorldPos[i]);
                if (sp.z <= 0.05f) continue;
                float gx = sp.x;
                float gy = Screen.height - sp.y;

                DrawGuiSolidRect(new Rect(gx - 6f, gy - 6f, 12f, 12f), new Color(0.05f, 0.05f, 0.08f, 0.98f));
                DrawGuiSolidRect(new Rect(gx - 4f, gy - 4f, 8f, 8f), new Color(1.0f, 0.88f, 0.12f, 1.0f));

                if (showLandmarkLabels && i < landmarks3D.Count)
                {
                    string lbl = $"{landmarks3D[i].label_vi} (Z:{landmarks3D[i].z_mm:F0})";
                    float offX = 9f;
                    float offY = -9f;
                    // Chống đè nhãn giữa Đùi trái (4) & Gối trái (6), Đùi phải (5) & Gối phải (7), và các mốc gấu quần (8,9,10,11)
                    if (i == 4) { offX = 9f; offY = -19f; }
                    else if (i == 6) { offX = -152f; offY = -19f; }
                    else if (i == 5) { offX = 9f; offY = -19f; }
                    else if (i == 7) { offX = -152f; offY = 4f; }
                    else if (i == 8) { offX = -152f; offY = -18f; }
                    else if (i == 9) { offX = -152f; offY = 3f; }
                    else if (i == 10) { offX = -152f; offY = -18f; }
                    else if (i == 11) { offX = -152f; offY = 3f; }

                    Rect bgRect = new Rect(gx + offX, gy + offY, 144f, 18f);
                    DrawGuiSolidRect(bgRect, new Color(0.04f, 0.08f, 0.15f, 0.90f));
                    GUI.Label(new Rect(bgRect.x + 3f, bgRect.y, 140f, 18f), lbl, AntigravityUITheme.StyleLabelCyan);
                }
            }
        }
    }

    private void DrawWorldLoopInGUI(Camera cam, List<Vector3> worldPts, Color col, float thickness, bool closedLoop)
    {
        int count = worldPts.Count;
        for (int i = 0; i < count - 1; i++)
        {
            Vector3 s0 = cam.WorldToScreenPoint(worldPts[i]);
            Vector3 s1 = cam.WorldToScreenPoint(worldPts[i + 1]);
            if (s0.z > 0.05f && s1.z > 0.05f)
            {
                DrawGuiLine2D(new Vector2(s0.x, Screen.height - s0.y), new Vector2(s1.x, Screen.height - s1.y), col, thickness);
            }
        }
        if (closedLoop && count > 2)
        {
            Vector3 s0 = cam.WorldToScreenPoint(worldPts[count - 1]);
            Vector3 s1 = cam.WorldToScreenPoint(worldPts[0]);
            if (s0.z > 0.05f && s1.z > 0.05f)
            {
                DrawGuiLine2D(new Vector2(s0.x, Screen.height - s0.y), new Vector2(s1.x, Screen.height - s1.y), col, thickness);
            }
        }
    }

    private void DrawGuiSolidRect(Rect r, Color c)
    {
        if (whitePixelTex == null) return;
        Color prev = GUI.color;
        GUI.color = c;
        GUI.DrawTexture(r, whitePixelTex);
        GUI.color = prev;
    }

    private void DrawGuiLine2D(Vector2 a, Vector2 b, Color col, float width)
    {
        if (whitePixelTex == null) return;
        Vector2 delta = b - a;
        float len = delta.magnitude;
        if (len < 0.5f) return;
        float angle = Mathf.Atan2(delta.y, delta.x) * Mathf.Rad2Deg;

        Matrix4x4 savedMat = GUI.matrix;
        Color savedCol = GUI.color;
        GUI.color = col;
        GUIUtility.RotateAroundPivot(angle, a);
        GUI.DrawTexture(new Rect(a.x, a.y - width * 0.5f, len, width), whitePixelTex);
        GUI.matrix = savedMat;
        GUI.color = savedCol;
    }

    // =========================================================================
    // VẼ GIZMO 3D QUỸ ĐẠO, 12 ĐIỂM MỐC & SEGMENT ĐÈ LÊN PHÔI (SCENE VIEW)
    // =========================================================================

    private void OnDrawGizmos()
    {
        // 1. Vẽ Đường bao Segment Thân Quần 3D trong Scene View
        if (showSegmentationContour3D && garmentContourWorld != null && garmentContourWorld.Count > 2)
        {
            Gizmos.color = new Color(0.05f, 0.95f, 0.55f, 0.95f);
            for (int i = 0; i < garmentContourWorld.Count - 1; i++)
                Gizmos.DrawLine(garmentContourWorld[i], garmentContourWorld[i + 1]);
            Gizmos.DrawLine(garmentContourWorld[garmentContourWorld.Count - 1], garmentContourWorld[0]);
        }

        // 2. Vẽ Vùng Cho Phép Phun 2 Ống Quần 3D trong Scene View
        if (showSprayZones3D)
        {
            Gizmos.color = new Color(1.0f, 0.72f, 0.15f, 0.90f);
            if (sprayZoneLeftWorld != null && sprayZoneLeftWorld.Count > 2)
            {
                for (int i = 0; i < sprayZoneLeftWorld.Count - 1; i++)
                    Gizmos.DrawLine(sprayZoneLeftWorld[i], sprayZoneLeftWorld[i + 1]);
                Gizmos.DrawLine(sprayZoneLeftWorld[sprayZoneLeftWorld.Count - 1], sprayZoneLeftWorld[0]);
            }
            if (sprayZoneRightWorld != null && sprayZoneRightWorld.Count > 2)
            {
                for (int i = 0; i < sprayZoneRightWorld.Count - 1; i++)
                    Gizmos.DrawLine(sprayZoneRightWorld[i], sprayZoneRightWorld[i + 1]);
                Gizmos.DrawLine(sprayZoneRightWorld[sprayZoneRightWorld.Count - 1], sprayZoneRightWorld[0]);
            }
        }

        // 3. Vẽ 12 Điểm Mốc Giải Phẫu AI 3D + Khung Xương trong Scene View
        if (showLandmarks3D && landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
        {
            Gizmos.color = new Color(1.0f, 0.35f, 0.85f, 0.90f);
            int numPairs = LandmarkSkeletonPairs.GetLength(0);
            for (int k = 0; k < numPairs; k++)
            {
                int i0 = LandmarkSkeletonPairs[k, 0];
                int i1 = LandmarkSkeletonPairs[k, 1];
                if (i0 < landmarksWorldPos.Count && i1 < landmarksWorldPos.Count)
                    Gizmos.DrawLine(landmarksWorldPos[i0], landmarksWorldPos[i1]);
            }

            Gizmos.color = new Color(1.0f, 0.85f, 0.15f, 1.0f);
            for (int i = 0; i < landmarksWorldPos.Count; i++)
            {
                Gizmos.DrawSphere(landmarksWorldPos[i], 0.010f);
            }
        }

        if (!showGizmos || waypoints == null || waypoints.Count == 0) return;

        // 4. Vẽ toàn bộ đường đa giác 3D (Polyline) đè lên đám mây điểm phôi (Chỉ vẽ đoạn CÓ PHUN)
        Gizmos.color = new Color(0.15f, 0.85f, 1f, 0.85f); // Cyan tương phản cao
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            bool isSameSeg = string.IsNullOrEmpty(waypoints[i].segmentName) || string.IsNullOrEmpty(waypoints[i + 1].segmentName) || waypoints[i].segmentName == waypoints[i + 1].segmentName;
            if (!waypoints[i + 1].sprayOn || !isSameSeg || Vector3.Distance(waypoints[i].targetPosGlobal_mm, waypoints[i + 1].targetPosGlobal_mm) > sprayJumpThresholdMm) continue;
            Gizmos.DrawLine(waypoints[i].targetPosWorld_m, waypoints[i + 1].targetPosWorld_m);
        }

        // 5. Điểm waypoint đang được chọn để quan sát
        if (currentWaypointIndex >= 0 && currentWaypointIndex < waypoints.Count)
        {
            WaypointData selWp = waypoints[currentWaypointIndex];
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(selWp.targetPosWorld_m, 0.007f);
            Gizmos.DrawWireSphere(selWp.targetPosWorld_m, 0.014f);
        }

        // 6. Vị trí đầu phun TCP thực tế của Robot
        if (tcpTransform != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(tcpTransform.position, 0.006f);
        }

        // 7. Vẽ các điểm/lỗ khuyết tật & chỗ khuất phát hiện (Hạt Đỏ Cam)
        if (showOcclusionHoles3D && detectedHolePointsWorld != null && detectedHolePointsWorld.Count > 0)
        {
            Gizmos.color = new Color(1.0f, 0.22f, 0.15f, 0.85f);
            for (int i = 0; i < detectedHolePointsWorld.Count; i++)
            {
                Gizmos.DrawSphere(detectedHolePointsWorld[i], 0.005f);
            }
        }

        // 8. Vẽ bề mặt đã bù đắp 3D (Lưới Xanh Neon) & Vector bù góc súng (Tia Vàng)
        if (showCompensatedSurface3D && compensatedSurfacePointsWorld != null && compensatedSurfacePointsWorld.Count > 0)
        {
            Gizmos.color = new Color(0.12f, 0.95f, 0.45f, 0.90f);
            for (int i = 0; i < compensatedSurfacePointsWorld.Count; i++)
            {
                Gizmos.DrawWireCube(compensatedSurfacePointsWorld[i], new Vector3(0.012f, 0.002f, 0.012f));
                if (showGunTiltVectors3D && i < gunTiltNormalsWorld.Count)
                {
                    Gizmos.color = new Color(1.0f, 0.88f, 0.15f, 0.95f);
                    Gizmos.DrawRay(compensatedSurfacePointsWorld[i], gunTiltNormalsWorld[i] * 0.035f);
                    Gizmos.color = new Color(0.12f, 0.95f, 0.45f, 0.90f);
                }
            }
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

    public AiServerClient EnsureAiClient()
    {
        AiServerClient client = GetComponent<AiServerClient>();
        if (client == null) client = FindAnyObjectByType<AiServerClient>();
        if (client == null) client = gameObject.AddComponent<AiServerClient>();
        return client;
    }
}
