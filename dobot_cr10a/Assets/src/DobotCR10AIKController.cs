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
    
    // Quản lý trạng thái các cửa sổ nổi Kéo Thả & Kéo Giãn (9 Module)
    // 0: SIM, 1: ROBOT, 2: NẠP PHÔI & HIỆU CHUẨN, 3: QUỸ ĐẠO, 4: XUẤT/NHẬP, 5: KIỂM TRA, 6: MỐC AI, 7: VÁ LỖ 3D, 8: ĐÁNH GIÁ & PHUN GAUSS
    public int interfaceTab = 0;
    public bool ribbonExpanded = true;
    public bool showDockPanel = true;
    public bool showJointsSubPanel = false;
    public bool showGizmos = true;
    private Vector2 dockScrollPosition;

    // 1 Bảng chi tiết duy nhất — gọn gàng, không mở chồng nhiều cửa sổ; có thể kéo thả & kéo giãn kích thước
    private DraggableUIManager.PanelState singleMainPanel;
    private bool floatingPanelsInitialized = false;

    [Header("4B. Nội suy vận tốc TCP đều (Cartesian Constant-Speed) & Telemetry")]
    public bool useConstantTcpSpeed = true;
    [Range(20f, 600f)]
    public float targetTcpSpeedMmPerSec = 220f;
    public float liveTcpSpeedMmPerSec = 0f;
    public float measuredCycleTimeSec = 0f;
    public float measuredTcpSpeedCvPct = 0f;
    private float cycleTimerSec = 0f;
    private float cartesianSegProgress = 0f;
    private Vector3 lastFrameTcpWorldPos;
    private bool hasLastFrameTcpWorldPos = false;
    private readonly List<float> recordedSpraySpeedsMmS = new List<float>();

    [Header("4C. Tham số Công Nghệ Phun, Đánh Giá P1/P3/P4 & Khuôn Đứng 3D")]
    public TrajectoryEvaluationEngine.SprayTechParams sprayTechParams = new TrajectoryEvaluationEngine.SprayTechParams();
    public List<TrajectoryEvaluationEngine.MethodEvaluationResult> comparisonResults = new List<TrajectoryEvaluationEngine.MethodEvaluationResult>();
    public bool showDepositionHeatmap3D = false;
    public bool showKeepOutZones3D = true;
    public bool showPhysicalWorktable = false;
    public bool showStandingMold3D = false;
    public string comparisonStatusMsg = "Bấm 'Chạy So Sánh P1 / P3 / P4' để tính toàn bộ chỉ tiêu theo file góp ý.";
    public string standingMoldReachSummary = "";
    private GameObject depositionHeatmapObj;
    private GameObject standingMoldObj;
    private GameObject worktableObj;
    private CalibrationManager calibrationManager;

    // Tương thích ngược cho các hệ thống cũ nếu có tham chiếu
    public bool showJointsPanel => showJointsSubPanel;
    public bool showFkMonitorPanel => false;
    public bool showIkTargetPanel => false;
    public bool showTrajectoryPanel => interfaceTab == 3 && showDockPanel;
    public bool showPipelinePanel => false;
    public bool showEvaluationPanel => interfaceTab == 8 && showDockPanel;

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

    [Header("7. Bảng Vẽ Trên Ảnh Camera Thực Tế 2D -> Ánh Xạ Lên Bề Mặt 3D")]
    public bool showCanonicalDrawPanel = true;
    public float customDrawZOffsetMm = 2.0f; // Mặc định 2mm (áp sát mặt quần)
    public string customDrawStatusMsg = "Vẽ trực tiếp trên ảnh thực tế từ camera hoặc bấm Áp dụng lên 3D.";
    private Texture2D canonicalJeansBgTex;
    private Texture2D realCameraFileTex;
    private string realCameraLoadedSample = "";
    private string explicitRealCameraImagePath = "";
    private bool canvasUsingRealCamera = false;
    private Vector2[] strokesReferenceLandmarksUV = null;
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
    public int selectedOcclusionAlgorithm = 0; // 0: MLS K-NN, 1: TPS RBF, 2: Poisson/Laplace, 3: Adaptive Gun Tilt, 4: Bilateral Symmetry
    public float occlusionSearchRadiusMm = 30.0f;
    public int occlusionKNeighbors = 16;
    public float maxGunTiltAngleDeg = 25.0f;
    public float occlusionZMarginMm = 12.0f;
    public float laplacianSmoothingFactor = 0.5f;
    public float steepSlopeAngleThresholdDeg = 55.0f;
    public bool showHoleRegionOutlines3D = true; // Bao quanh các vùng cần vá bằng đường màu đỏ
    public bool showOcclusionHoles3D = false;    // Tùy chọn hiện chấm đỏ từng ô bên trong vùng cần vá
    public bool showCompensatedSurface3D = false; // Tùy chọn hiện lưới chấm xanh (mặc định tắt để thấy rõ 3D Mesh đặc & viền đỏ)
    public bool showGunTiltVectors3D = true;
    public int detectedHoleCount = 0;
    public int detectedHoleRegionCount = 0;
    public float detectedOcclusionAreaCm2 = 0f;
    public float avgCreaseDepthMm = 0f;
    public string occlusionStatusMsg = "Sẵn sàng phân tích chỗ khuất trên bề mặt quần 3D.";
    public readonly List<Vector3> detectedHolePointsWorld = new List<Vector3>();
    public readonly List<Vector2Int> detectedHoleCells = new List<Vector2Int>();
    public readonly List<List<Vector3>> holeRegionOutlinesWorld = new List<List<Vector3>>();
    public readonly List<Vector3> compensatedSurfacePointsWorld = new List<Vector3>();
    public readonly List<Vector3> gunTiltNormalsWorld = new List<Vector3>();
    private readonly Dictionary<Vector2Int, float> compensatedHeightGrid = new Dictionary<Vector2Int, float>();
    private readonly Dictionary<Vector2Int, Vector3> compensatedNormalGrid = new Dictionary<Vector2Int, Vector3>();
    private float lastGarmentMinClothY = 0.015f;
    private float lastGarmentMedianY = 0.080f;

    [Header("10. Vá Điểm 3D & Chuyển Thành 3D Mesh (3D Inpainting & Solid Mesh)")]
    public bool autoInpaintHolesOnLoad = true;
    public bool showInpaintedSurface3D = true;
    public bool renderInpaintedAsSolidMesh = true; // true = tạo 3D Mesh tam giác đặc liền kín mép, false = hạt điểm Point Cloud
    public bool highlightInpaintedMesh = false; // false = màu RGB tiệp vải thật, true = màu cyan nổi bật
    public float inpaintPointSpacingMm = 3.5f; // Khoảng cách tham chiếu khi hiển thị hạt điểm
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

    [Serializable]
    private class WorkbenchSetupData
    {
        public bool showDockPanel = true;
        public bool ribbonExpanded = true;
        public int interfaceTab = 2;
        public int trajSimSubTab = 0;
        public int configSubTab = 0;
        public bool showGizmos = true;
        public bool showJointSpeedControls = false;
        public bool showJointsSubPanel = false;
        public bool showCanonicalDrawPanel = true;

        public float playbackSpeed = 1.0f;
        public bool useConstantTcpSpeed = true;
        public float targetTcpSpeedMmPerSec = 220f;
        public float maxJointSpeedDegPerSecond = 223f;
        public float maxJointAccelerationDegPerSecond2 = 12f;
        public float[] jointSpeedDegPerSecond = new float[] { 10f, 10f, 10f, 10f, 10f, 10f };

        public bool sprayPaintEnabled = true;
        public float sprayWidthMm = 4.0f;
        public float sprayPathToleranceMm = 20f;
        public float sprayJumpThresholdMm = 45f;

        public float toolLength = 100.0f;
        public float toolOffsetX = 0.0f;
        public float toolOffsetY = 0.0f;

        public string trajectoryCsvPath = "output_drawing/pants_outline_cr10a.csv";
        public string selectedSampleName = "pcl_101";
        public string customPlyPath = "Assets/PointCloud/full_cloud_world.ply";
        public string customImportCsvPath = "output_drawing/pants_outline_cr10a.csv";

        public string activeSprayTemplateSize = "S";
        public float customDrawZOffsetMm = 2.0f;
        public bool preserveCustomDrawingAcrossSamples = true;
        public string sprayTemplateDirectory = "output_drawing/spray_templates";
        public string saveTemplateFileName = "mau_phun_1";
        public List<Stroke2D> customDrawnStrokes = new List<Stroke2D>();

        public bool showLandmarks3D = true;
        public bool showLandmarkLabels = true;
        public bool showSegmentationContour3D = true;
        public bool showSprayZones3D = true;

        public int selectedOcclusionAlgorithm = 0;
        public float occlusionSearchRadiusMm = 30.0f;
        public int occlusionKNeighbors = 16;
        public float maxGunTiltAngleDeg = 25.0f;
        public float occlusionZMarginMm = 12.0f;
        public float laplacianSmoothingFactor = 0.5f;
        public float steepSlopeAngleThresholdDeg = 55.0f;
        public bool showHoleRegionOutlines3D = true;
        public bool showOcclusionHoles3D = false;
        public bool showCompensatedSurface3D = false;
        public bool showGunTiltVectors3D = true;

        public bool autoInpaintHolesOnLoad = true;
        public bool showInpaintedSurface3D = true;
        public bool renderInpaintedAsSolidMesh = true;
        public bool highlightInpaintedMesh = false;
        public float inpaintPointSpacingMm = 3.5f;
        public float inpaintPointSize = 3.5f;

        public TrajectoryEvaluationEngine.SprayTechParams sprayTechParams = new TrajectoryEvaluationEngine.SprayTechParams();
        public bool showDepositionHeatmap3D = false;
        public bool showKeepOutZones3D = true;
        public bool showPhysicalWorktable = false;
        public bool showStandingMold3D = false;

        public bool forceBypassSafetyInterlock = false;
    }

    private bool hasLoadedAllSettings = false;
    private bool pendingAutoSave = false;
    private float lastGuiChangeRealtime = -1f;
    private float lastSavedToastUntilRealtime = -1f;

    private static string GetWorkbenchSetupFilePath()
    {
#if UNITY_EDITOR
        string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "UserSettings"));
#else
        string dir = Application.persistentDataPath;
#endif
        return Path.Combine(dir, "CR10A_WorkbenchSetup.json");
    }

    public void MarkSettingsDirty()
    {
        if (!hasLoadedAllSettings) return;
        pendingAutoSave = true;
        lastGuiChangeRealtime = Time.realtimeSinceStartup;
    }

    public void SaveAllWorkbenchSetups(bool showToast = false)
    {
        pendingAutoSave = false;
        SaveAllSettings();

        CalibrationManager calib = calibrationManager ?? FindAnyObjectByType<CalibrationManager>();
        if (calib != null) calib.SavePoseParamsToPrefs();

        PLYDirectLoader ply = FindAnyObjectByType<PLYDirectLoader>();
        if (ply != null) ply.SaveSettings();

        AiServerClient ai = GetComponent<AiServerClient>() ?? FindAnyObjectByType<AiServerClient>();
        if (ai != null) ai.SaveSettings();

        FixedRigCamera rig = FindAnyObjectByType<FixedRigCamera>();
        if (rig != null) rig.SaveRigVisualSettings();

        if (robotManual != null) robotManual.SaveSettings();

        if (singleMainPanel != null) DraggableUIManager.SavePanelLayout(singleMainPanel);

        if (showToast)
        {
            lastSavedToastUntilRealtime = Time.realtimeSinceStartup + 2.5f;
        }
    }

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
        if (!hasLoadedAllSettings)
        {
            LoadAllSettings();
        }
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
        if (string.IsNullOrEmpty(activeSprayTemplateSize)) activeSprayTemplateSize = "S";
        hasTriggeredSizeAutoSwitch = false;
        LoadTrajectoryFile();
        LoadLandmarksAndSegmentationForSample(selectedSampleName, autoSwitchSprayTemplate: false);
        EnsureCanonicalJeansCanvasInitialized();
        EnsureCameraController();

        if (showPhysicalWorktable)
        {
            PLYDirectLoader pl = FindAnyObjectByType<PLYDirectLoader>();
            worktableObj = TrajectoryEvaluationEngine.EnsurePhysicalWorktable(null, pl != null ? pl.estimatedTableY : 0f);
        }

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
        if (Application.isPlaying && hasLoadedAllSettings)
        {
            SaveAllWorkbenchSetups();
        }
    }

    private void OnDestroy()
    {
        if (Application.isPlaying && hasLoadedAllSettings)
        {
            SaveAllWorkbenchSetups();
        }
        ReleaseRealCameraFileTexture();
        if (inkTrailMaterial != null)
        {
            if (Application.isPlaying) Destroy(inkTrailMaterial);
            else DestroyImmediate(inkTrailMaterial);
            inkTrailMaterial = null;
        }
    }

    private void OnApplicationQuit()
    {
        if (hasLoadedAllSettings)
        {
            SaveAllWorkbenchSetups();
        }
    }

    private void ReleaseRealCameraFileTexture()
    {
        if (realCameraFileTex != null)
        {
            if (Application.isPlaying) Destroy(realCameraFileTex);
            else DestroyImmediate(realCameraFileTex);
            realCameraFileTex = null;
        }
    }

    public void SaveAllSettings()
    {
        PlayerPrefs.SetInt("CR10A_ShowDock", showDockPanel ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_RibbonExpanded", ribbonExpanded ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_InterfaceTab", interfaceTab);
        PlayerPrefs.SetInt("CR10A_TrajSimSubTab", trajSimSubTab);
        PlayerPrefs.SetInt("CR10A_ConfigSubTab", configSubTab);
        PlayerPrefs.SetInt("CR10A_ShowGizmos", showGizmos ? 1 : 0);

        PlayerPrefs.SetFloat("CR10A_PlaybackSpeed", playbackSpeed);
        PlayerPrefs.SetFloat("CR10A_MaxJointSpeed", maxJointSpeedDegPerSecond);
        PlayerPrefs.SetFloat("CR10A_MaxJointAcc", maxJointAccelerationDegPerSecond2);
        PlayerPrefs.SetInt("CR10A_SprayPaint", sprayPaintEnabled ? 1 : 0);
        PlayerPrefs.SetFloat("CR10A_SprayWidth", sprayWidthMm);
        PlayerPrefs.SetString("CR10A_TrajectoryCsv", trajectoryCsvPath ?? "");

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

        try
        {
            string setupPath = GetWorkbenchSetupFilePath();
            string dir = Path.GetDirectoryName(setupPath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) Directory.CreateDirectory(dir);

            var data = new WorkbenchSetupData
            {
                showDockPanel = showDockPanel,
                ribbonExpanded = ribbonExpanded,
                interfaceTab = interfaceTab,
                trajSimSubTab = trajSimSubTab,
                configSubTab = configSubTab,
                showGizmos = showGizmos,
                showJointSpeedControls = showJointSpeedControls,
                showJointsSubPanel = showJointsSubPanel,
                showCanonicalDrawPanel = showCanonicalDrawPanel,

                playbackSpeed = playbackSpeed,
                useConstantTcpSpeed = useConstantTcpSpeed,
                targetTcpSpeedMmPerSec = targetTcpSpeedMmPerSec,
                maxJointSpeedDegPerSecond = maxJointSpeedDegPerSecond,
                maxJointAccelerationDegPerSecond2 = maxJointAccelerationDegPerSecond2,
                jointSpeedDegPerSecond = (jointSpeedDegPerSecond != null && jointSpeedDegPerSecond.Length == 6)
                    ? (float[])jointSpeedDegPerSecond.Clone()
                    : new float[] { 10f, 10f, 10f, 10f, 10f, 10f },

                sprayPaintEnabled = sprayPaintEnabled,
                sprayWidthMm = sprayWidthMm,
                sprayPathToleranceMm = sprayPathToleranceMm,
                sprayJumpThresholdMm = sprayJumpThresholdMm,

                toolLength = toolLength,
                toolOffsetX = toolOffsetX,
                toolOffsetY = toolOffsetY,

                trajectoryCsvPath = trajectoryCsvPath ?? "",
                selectedSampleName = selectedSampleName ?? "pcl_101",
                customPlyPath = customPlyPath ?? "",
                customImportCsvPath = customImportCsvPath ?? "",

                activeSprayTemplateSize = string.IsNullOrEmpty(activeSprayTemplateSize) ? "S" : activeSprayTemplateSize,
                customDrawZOffsetMm = customDrawZOffsetMm,
                preserveCustomDrawingAcrossSamples = preserveCustomDrawingAcrossSamples,
                sprayTemplateDirectory = sprayTemplateDirectory ?? "output_drawing/spray_templates",
                saveTemplateFileName = saveTemplateFileName ?? "mau_phun_1",

                showLandmarks3D = showLandmarks3D,
                showLandmarkLabels = showLandmarkLabels,
                showSegmentationContour3D = showSegmentationContour3D,
                showSprayZones3D = showSprayZones3D,

                selectedOcclusionAlgorithm = selectedOcclusionAlgorithm,
                occlusionSearchRadiusMm = occlusionSearchRadiusMm,
                occlusionKNeighbors = occlusionKNeighbors,
                maxGunTiltAngleDeg = maxGunTiltAngleDeg,
                occlusionZMarginMm = occlusionZMarginMm,
                laplacianSmoothingFactor = laplacianSmoothingFactor,
                steepSlopeAngleThresholdDeg = steepSlopeAngleThresholdDeg,
                showHoleRegionOutlines3D = showHoleRegionOutlines3D,
                showOcclusionHoles3D = showOcclusionHoles3D,
                showCompensatedSurface3D = showCompensatedSurface3D,
                showGunTiltVectors3D = showGunTiltVectors3D,

                autoInpaintHolesOnLoad = autoInpaintHolesOnLoad,
                showInpaintedSurface3D = showInpaintedSurface3D,
                renderInpaintedAsSolidMesh = renderInpaintedAsSolidMesh,
                highlightInpaintedMesh = highlightInpaintedMesh,
                inpaintPointSpacingMm = inpaintPointSpacingMm,
                inpaintPointSize = inpaintPointSize,

                sprayTechParams = sprayTechParams ?? new TrajectoryEvaluationEngine.SprayTechParams(),
                showDepositionHeatmap3D = showDepositionHeatmap3D,
                showKeepOutZones3D = showKeepOutZones3D,
                showPhysicalWorktable = showPhysicalWorktable,
                showStandingMold3D = showStandingMold3D,

                forceBypassSafetyInterlock = forceBypassSafetyInterlock
            };

            if (customDrawnStrokes != null)
            {
                for (int s = 0; s < customDrawnStrokes.Count; s++)
                {
                    var st = customDrawnStrokes[s];
                    if (st == null || st.Count == 0) continue;
                    var s2d = new Stroke2D();
                    for (int p = 0; p < st.Count; p++)
                    {
                        s2d.points.Add(new StrokePoint2D(st[p].x, st[p].y));
                    }
                    data.customDrawnStrokes.Add(s2d);
                }
            }

            File.WriteAllText(setupPath, JsonUtility.ToJson(data, true));
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[CR10A] Không thể lưu file cấu hình setup: {ex.Message}");
        }
    }

    public void LoadAllSettings()
    {
        bool loadedFromJson = false;
        try
        {
            string setupPath = GetWorkbenchSetupFilePath();
            if (File.Exists(setupPath))
            {
                string rawJson = File.ReadAllText(setupPath);
                var data = JsonUtility.FromJson<WorkbenchSetupData>(rawJson);
                if (data != null)
                {
                    showDockPanel = data.showDockPanel;
                    ribbonExpanded = data.ribbonExpanded;
                    interfaceTab = data.interfaceTab;
                    trajSimSubTab = data.trajSimSubTab;
                    configSubTab = data.configSubTab;
                    showGizmos = data.showGizmos;
                    showJointSpeedControls = data.showJointSpeedControls;
                    showJointsSubPanel = data.showJointsSubPanel;
                    showCanonicalDrawPanel = data.showCanonicalDrawPanel;

                    playbackSpeed = Mathf.Clamp(data.playbackSpeed, 0.1f, 2.0f);
                    useConstantTcpSpeed = data.useConstantTcpSpeed;
                    targetTcpSpeedMmPerSec = Mathf.Clamp(data.targetTcpSpeedMmPerSec, 20f, 600f);
                    maxJointSpeedDegPerSecond = Mathf.Clamp(data.maxJointSpeedDegPerSecond, 1f, 223f);
                    maxJointAccelerationDegPerSecond2 = Mathf.Clamp(data.maxJointAccelerationDegPerSecond2, 1f, 1000f);
                    if (data.jointSpeedDegPerSecond != null && data.jointSpeedDegPerSecond.Length == 6)
                    {
                        EnsureJointSpeedSettings();
                        for (int j = 0; j < 6; j++)
                        {
                            jointSpeedDegPerSecond[j] = Mathf.Clamp(data.jointSpeedDegPerSecond[j], 1f, 223f);
                        }
                    }

                    sprayPaintEnabled = data.sprayPaintEnabled;
                    sprayWidthMm = Mathf.Clamp(data.sprayWidthMm, 0.5f, 30f);
                    sprayPathToleranceMm = Mathf.Clamp(data.sprayPathToleranceMm, 2f, 60f);
                    sprayJumpThresholdMm = Mathf.Clamp(data.sprayJumpThresholdMm, 10f, 200f);

                    toolLength = Mathf.Max(0f, data.toolLength);
                    toolOffsetX = data.toolOffsetX;
                    toolOffsetY = data.toolOffsetY;

                    if (!string.IsNullOrEmpty(data.trajectoryCsvPath)) trajectoryCsvPath = data.trajectoryCsvPath;
                    if (!string.IsNullOrEmpty(data.selectedSampleName)) selectedSampleName = data.selectedSampleName;
                    if (!string.IsNullOrEmpty(data.customPlyPath)) customPlyPath = data.customPlyPath;
                    if (!string.IsNullOrEmpty(data.customImportCsvPath)) customImportCsvPath = data.customImportCsvPath;

                    if (!string.IsNullOrEmpty(data.activeSprayTemplateSize)) activeSprayTemplateSize = data.activeSprayTemplateSize;
                    customDrawZOffsetMm = Mathf.Clamp(data.customDrawZOffsetMm, 0f, 100f);
                    preserveCustomDrawingAcrossSamples = data.preserveCustomDrawingAcrossSamples;
                    if (!string.IsNullOrEmpty(data.sprayTemplateDirectory)) sprayTemplateDirectory = data.sprayTemplateDirectory;
                    if (!string.IsNullOrEmpty(data.saveTemplateFileName)) saveTemplateFileName = data.saveTemplateFileName;

                    if (data.customDrawnStrokes != null && data.customDrawnStrokes.Count > 0)
                    {
                        customDrawnStrokes.Clear();
                        for (int s = 0; s < data.customDrawnStrokes.Count; s++)
                        {
                            var st = data.customDrawnStrokes[s];
                            if (st == null || st.points == null || st.points.Count == 0) continue;
                            var list = new List<Vector2>(st.points.Count);
                            for (int p = 0; p < st.points.Count; p++)
                            {
                                list.Add(new Vector2(st.points[p].u, st.points[p].v));
                            }
                            customDrawnStrokes.Add(list);
                        }
                        if (customDrawnStrokes.Count > 0) hasInitializedDefaultStrokes = true;
                    }

                    showLandmarks3D = data.showLandmarks3D;
                    showLandmarkLabels = data.showLandmarkLabels;
                    showSegmentationContour3D = data.showSegmentationContour3D;
                    showSprayZones3D = data.showSprayZones3D;

                    selectedOcclusionAlgorithm = Mathf.Clamp(data.selectedOcclusionAlgorithm, 0, 4);
                    occlusionSearchRadiusMm = Mathf.Clamp(data.occlusionSearchRadiusMm, 12f, 80f);
                    occlusionKNeighbors = Mathf.Clamp(data.occlusionKNeighbors, 8, 48);
                    maxGunTiltAngleDeg = Mathf.Clamp(data.maxGunTiltAngleDeg, 10f, 45f);
                    occlusionZMarginMm = Mathf.Clamp(data.occlusionZMarginMm, 5f, 30f);
                    laplacianSmoothingFactor = Mathf.Clamp01(data.laplacianSmoothingFactor);
                    steepSlopeAngleThresholdDeg = Mathf.Clamp(data.steepSlopeAngleThresholdDeg, 15f, 85f);
                    showHoleRegionOutlines3D = data.showHoleRegionOutlines3D;
                    showOcclusionHoles3D = data.showOcclusionHoles3D;
                    showCompensatedSurface3D = data.showCompensatedSurface3D;
                    showGunTiltVectors3D = data.showGunTiltVectors3D;

                    autoInpaintHolesOnLoad = data.autoInpaintHolesOnLoad;
                    showInpaintedSurface3D = data.showInpaintedSurface3D;
                    renderInpaintedAsSolidMesh = data.renderInpaintedAsSolidMesh;
                    highlightInpaintedMesh = data.highlightInpaintedMesh;
                    inpaintPointSpacingMm = Mathf.Clamp(data.inpaintPointSpacingMm, 1f, 20f);
                    inpaintPointSize = Mathf.Clamp(data.inpaintPointSize, 1f, 20f);

                    if (data.sprayTechParams != null)
                    {
                        sprayTechParams = data.sprayTechParams;
                        sprayTechParams.targetSpeedMmPerSec = targetTcpSpeedMmPerSec;
                    }
                    showDepositionHeatmap3D = data.showDepositionHeatmap3D;
                    showKeepOutZones3D = data.showKeepOutZones3D;
                    showPhysicalWorktable = data.showPhysicalWorktable;
                    showStandingMold3D = data.showStandingMold3D;

                    forceBypassSafetyInterlock = data.forceBypassSafetyInterlock;
                    loadedFromJson = true;
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[CR10A] Không thể đọc file cấu hình setup: {ex.Message}");
        }

        if (!loadedFromJson)
        {
            showDockPanel = PlayerPrefs.GetInt("CR10A_ShowDock", 1) == 1;
            ribbonExpanded = PlayerPrefs.GetInt("CR10A_RibbonExpanded", 1) == 1;
            interfaceTab = PlayerPrefs.GetInt("CR10A_InterfaceTab", 0);
            trajSimSubTab = PlayerPrefs.GetInt("CR10A_TrajSimSubTab", 0);
            configSubTab = PlayerPrefs.GetInt("CR10A_ConfigSubTab", 0);
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
        }

        ApplyToolOffsetToTransforms();
        if (robotManual != null)
        {
            robotManual.toolLengthMm = toolLength;
            robotManual.toolOffsetX = toolOffsetX;
            robotManual.toolOffsetY = toolOffsetY;
            robotManual.toolOffsetZText = toolLength.ToString("F1", CultureInfo.InvariantCulture);
            robotManual.toolOffsetXText = toolOffsetX.ToString("F1", CultureInfo.InvariantCulture);
            robotManual.toolOffsetYText = toolOffsetY.ToString("F1", CultureInfo.InvariantCulture);
        }
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
        hasLoadedAllSettings = true;
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
            lineRenderer.useWorldSpace = true;
        }
        EnsureInkTrailMaterial();
        lineRenderer.sharedMaterial = inkTrailMaterial;
        lineRenderer.startWidth = sprayWidthMm * 0.001f;
        lineRenderer.endWidth = sprayWidthMm * 0.001f;
        lineRenderer.startColor = inkColor;
        lineRenderer.endColor = inkColor;
        lineRenderer.numCapVertices = 4;
        lineRenderer.numCornerVertices = 4;
        lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lineRenderer.receiveShadows = false;

        ApplyToolOffsetToTransforms();
    }

    private Material inkTrailMaterial;

    private void EnsureInkTrailMaterial()
    {
        if (inkTrailMaterial == null)
        {
            Shader sh = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Color");
            if (sh != null)
            {
                inkTrailMaterial = new Material(sh);
                inkTrailMaterial.name = "CR10A_SprayInk_RuntimeMat";
                inkTrailMaterial.hideFlags = HideFlags.DontSave;
            }
        }
        if (inkTrailMaterial != null)
        {
            inkTrailMaterial.color = inkColor;
            if (inkTrailMaterial.HasProperty("_BaseColor"))
            {
                inkTrailMaterial.SetColor("_BaseColor", inkColor);
            }
        }
    }

    public void UpdateSprayWidth(float widthMm)
    {
        sprayWidthMm = Mathf.Clamp(widthMm, 0.5f, 30.0f);
        float widthM = sprayWidthMm * 0.001f;
        if (lineRenderer != null)
        {
            lineRenderer.startWidth = widthM;
            lineRenderer.endWidth = widthM;
        }
        for (int i = 0; i < committedTrailRenderers.Count; i++)
        {
            if (committedTrailRenderers[i] != null)
            {
                committedTrailRenderers[i].startWidth = widthM;
                committedTrailRenderers[i].endWidth = widthM;
            }
        }
    }

    public void SetInkColor(Color color)
    {
        inkColor = color;
        EnsureInkTrailMaterial();
        if (lineRenderer != null)
        {
            lineRenderer.sharedMaterial = inkTrailMaterial;
            lineRenderer.startColor = inkColor;
            lineRenderer.endColor = inkColor;
        }
        for (int i = 0; i < committedTrailRenderers.Count; i++)
        {
            if (committedTrailRenderers[i] != null)
            {
                committedTrailRenderers[i].sharedMaterial = inkTrailMaterial;
                committedTrailRenderers[i].startColor = inkColor;
                committedTrailRenderers[i].endColor = inkColor;
            }
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
        if (pendingAutoSave && hasLoadedAllSettings && (Time.realtimeSinceStartup - lastGuiChangeRealtime) >= 0.35f)
        {
            SaveAllWorkbenchSetups();
        }
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
                cycleTimerSec = 0f;
                cartesianSegProgress = 0f;
                recordedSpraySpeedsMmS.Clear();
                if (robotManual != null)
                    robotManual.ikStatus = "Đang di chuyển từ Home đến điểm đầu quỹ đạo...";
            }
            else
            {
                cycleTimerSec += Time.deltaTime;
            }

            if (MoveTowardWaypoint(currentWaypointIndex, Time.deltaTime))
            {
                paintingStarted = true;
                cartesianSegProgress = 0f;
                if (currentWaypointIndex + 1 >= waypoints.Count)
                {
                    measuredCycleTimeSec = cycleTimerSec;
                    UpdateMeasuredTcpSpeedCv();
                    cycleTimerSec = 0f;
                    recordedSpraySpeedsMmS.Clear();
                }
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Count;
                if (robotManual != null)
                    robotManual.ikStatus = $"Đang vận hành quỹ đạo ({currentWaypointIndex + 1}/{waypoints.Count}) | v_TCP={liveTcpSpeedMmPerSec:F0} mm/s...";
            }

            // Đo vận tốc TCP thực tế thời gian thực để tính độ đều tốc độ V_cv (%)
            if (tcpTransform != null)
            {
                if (hasLastFrameTcpWorldPos && Time.deltaTime > 1e-4f)
                {
                    float instSpeedMmS = Vector3.Distance(tcpTransform.position, lastFrameTcpWorldPos) * 1000f / Time.deltaTime;
                    liveTcpSpeedMmPerSec = Mathf.Lerp(liveTcpSpeedMmPerSec, instSpeedMmS, 0.25f);
                    if (isSprayingNow && instSpeedMmS > 5f)
                    {
                        recordedSpraySpeedsMmS.Add(instSpeedMmS);
                        if (recordedSpraySpeedsMmS.Count > 2000) recordedSpraySpeedsMmS.RemoveAt(0);
                        UpdateMeasuredTcpSpeedCv();
                    }
                }
                lastFrameTcpWorldPos = tcpTransform.position;
                hasLastFrameTcpWorldPos = true;
            }
        }
        else
        {
            hasLastFrameTcpWorldPos = false;
            liveTcpSpeedMmPerSec = 0f;
        }

        UpdateInkTrail();
    }

    private void UpdateMeasuredTcpSpeedCv()
    {
        if (recordedSpraySpeedsMmS.Count < 3) return;
        double sum = 0;
        for (int i = 0; i < recordedSpraySpeedsMmS.Count; i++) sum += recordedSpraySpeedsMmS[i];
        double mean = sum / recordedSpraySpeedsMmS.Count;
        if (mean < 1e-3) return;
        double sq = 0;
        for (int i = 0; i < recordedSpraySpeedsMmS.Count; i++)
        {
            double d = recordedSpraySpeedsMmS[i] - mean;
            sq += d * d;
        }
        measuredTcpSpeedCvPct = (float)(Math.Sqrt(sq / recordedSpraySpeedsMmS.Count) / mean * 100.0);
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

        // Chế độ nội suy vận tốc TCP đều (Cartesian Linear Path) trên các đoạn đang phun:
        // Giữ v_TCP = const (không dừng giật tại mỗi waypoint), giải IK giải tích trực tuyến (< 0.04ms)
        if (useConstantTcpSpeed && paintingStarted && index > 0 && waypoint.sprayOn)
        {
            WaypointData prevWp = waypoints[index - 1];
            float segLenMm = Vector3.Distance(prevWp.targetPosGlobal_mm, waypoint.targetPosGlobal_mm);
            if (segLenMm >= 0.5f && segLenMm <= 250f)
            {
                float speedMmS = Mathf.Clamp(targetTcpSpeedMmPerSec * Mathf.Clamp(playbackSpeed, 0.1f, 2.0f), 15f, 800f);
                cartesianSegProgress += (speedMmS * Mathf.Max(0f, deltaTime)) / segLenMm;
                float t = Mathf.Clamp01(cartesianSegProgress);

                Vector3 interpPosMm = Vector3.Lerp(prevWp.targetPosGlobal_mm, waypoint.targetPosGlobal_mm, t);
                Quaternion qA = GetWaypointFlangeRotation(prevWp);
                Quaternion qB = GetWaypointFlangeRotation(waypoint);
                Quaternion interpRot = Quaternion.Slerp(qA, qB, t);

                float[] curQ = { robotManual.axis0, robotManual.axis1, robotManual.axis2, robotManual.axis3, robotManual.axis4, robotManual.axis5 };
                var sols = DobotCR10AKinematics.SolveTcpPose(
                    interpPosMm, interpRot, curQ, toolLength,
                    useAdaptiveTilt: true, maxTiltDeg: sprayTechParams != null ? sprayTechParams.maxTiltDeg : 30f,
                    toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);

                if (sols != null && sols.Count > 0)
                {
                    var best = sols[0];
                    robotManual.axis0 = (float)best.jointsRad[0];
                    robotManual.axis1 = (float)best.jointsRad[1];
                    robotManual.axis2 = (float)best.jointsRad[2];
                    robotManual.axis3 = (float)best.jointsRad[3];
                    robotManual.axis4 = (float)best.jointsRad[4];
                    robotManual.axis5 = (float)best.jointsRad[5];
                    robotManual.ApplyRotations();

                    bool sprayNow = sprayPaintEnabled && tcpTransform != null && waypoint.sprayOn;
                    if (sprayNow && !prevWp.sprayOn && cartesianSegProgress <= (speedMmS * Mathf.Max(0f, deltaTime)) / segLenMm + 1e-4f)
                    {
                        SetSprayState(false);
                    }
                    SetSprayState(sprayNow);
                    if (sprayNow)
                    {
                        RecordSprayInkPoint(tcpTransform.position);
                    }

                    return cartesianSegProgress >= 1f;
                }
            }
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

        if (sprayNow)
        {
            RecordSprayInkPoint(tcpTransform.position);
        }

        return arrived;
    }

    private void RecordSprayInkPoint(Vector3 tcpWorldPos)
    {
        Vector3 inkPt = ProjectSprayInkPointToSurface(tcpWorldPos);
        if (actualTcpTrail.Count > 0)
        {
            float d = Vector3.Distance(actualTcpTrail[actualTcpTrail.Count - 1], inkPt);
            if (d < 0.0015f) return;
            // Nếu điểm phun nhảy quá 5.5 cm (chuyển nét / vượt khe) -> ngắt đoạn cũ để không vẽ vệt đỏ nối ngang qua không trung
            if (d > 0.055f)
            {
                CommitCurrentTrailSegment();
            }
        }
        actualTcpTrail.Add(inkPt);
        if (actualTcpTrail.Count > 7000)
        {
            actualTcpTrail.RemoveRange(0, actualTcpTrail.Count - 7000);
        }
    }

    private PLYDirectLoader cachedPlyLoaderForSpray;

    /// <summary>
    /// Chiếu điểm phun từ đầu súng (TCP) theo đúng trục vòi phun (flange.forward = z6) xuống bề mặt thực
    /// của 3D Mesh / Point Cloud / Khuôn 3D (+1.8 mm theo pháp tuyến) để vệt mực bám khít mặt vải.
    /// Lưu ý: Không dùng tcpTransform.forward vì trong scene `tcp` xoay -90° quanh Y (hướng ngang)!
    /// </summary>
    private Vector3 ProjectSprayInkPointToSurface(Vector3 tcpWorldPos)
    {
        const float surfaceLiftM = 0.0018f;
        bool moldActive = (standingMoldObj != null && standingMoldObj.activeInHierarchy);

        // Trục vòi phun thực tế là trục +Z của mặt bích cuối (axis5 / parent của tcp)
        Transform flange = (tcpTransform != null) ? tcpTransform.parent : null;
        if (flange == null && robotManual != null && robotManual.axisTransforms != null && robotManual.axisTransforms.Length >= 6)
        {
            flange = robotManual.axisTransforms[5];
        }
        Vector3 sprayDir = (flange != null) ? flange.forward : Vector3.down;
        if (sprayDir.sqrMagnitude < 1e-5f) sprayDir = Vector3.down;
        sprayDir.Normalize();

        // Đảm bảo tia phun luôn hướng xuống bề mặt vải khi dùng phôi đặt nằm ngang
        if (!moldActive)
        {
            if (sprayDir.y > 0.1f) sprayDir = -sprayDir;
            if (sprayDir.y > -0.25f) sprayDir = Vector3.down;
        }

        Ray sprayRay = new Ray(tcpWorldPos - sprayDir * 0.015f, sprayDir);

        // 1. Ưu tiên khuôn chân đứng 3D nếu đang bật
        if (moldActive)
        {
            Collider moldCol = standingMoldObj.GetComponent<Collider>();
            if (moldCol != null && moldCol.Raycast(sprayRay, out RaycastHit moldHit, 0.50f))
            {
                return moldHit.point + moldHit.normal * surfaceLiftM;
            }
        }

        if (cachedPlyLoaderForSpray == null)
        {
            cachedPlyLoaderForSpray = FindAnyObjectByType<PLYDirectLoader>();
        }

        // 2. Raycast trực tiếp theo tia súng phun lên WorkpieceSurfaceMesh & lớp vá InpaintedJeansSurface_Mesh
        bool foundHit = false;
        float bestDist = float.MaxValue;
        Vector3 bestPt = tcpWorldPos;
        Vector3 bestNorm = Vector3.up;

        if (cachedPlyLoaderForSpray != null && cachedPlyLoaderForSpray.RaycastSurfaceMesh(sprayRay, 0.45f, out RaycastHit meshHit))
        {
            foundHit = true;
            bestDist = meshHit.distance;
            bestPt = meshHit.point;
            bestNorm = meshHit.normal;
        }
        if (inpaintedMeshObject != null && inpaintedMeshObject.activeInHierarchy)
        {
            MeshCollider patchCol = inpaintedMeshObject.GetComponent<MeshCollider>();
            if (patchCol != null && patchCol.Raycast(sprayRay, out RaycastHit patchHit, 0.45f) && patchHit.distance < bestDist)
            {
                foundHit = true;
                bestDist = patchHit.distance;
                bestPt = patchHit.point;
                bestNorm = patchHit.normal;
            }
        }
        if (foundHit)
        {
            float horizShift = Vector2.Distance(new Vector2(bestPt.x, bestPt.z), new Vector2(tcpWorldPos.x, tcpWorldPos.z));
            if (moldActive || horizShift <= 0.14f)
            {
                if (bestNorm.y < 0f && !moldActive) bestNorm = -bestNorm;
                return bestPt + bestNorm.normalized * surfaceLiftM;
            }
        }

        // 3. Chế độ Point Cloud (hoặc tia nghiêng ngoài mép collider): giao tia phun với lưới cao độ bề mặt vải 3D (2 bước lặp hội tụ)
        if (cachedPlyLoaderForSpray != null)
        {
            float queryX = tcpWorldPos.x;
            float queryZ = tcpWorldPos.z;
            float surfY = lastGarmentMedianY;
            Vector3 surfNorm = Vector3.up;

            if (sprayDir.y < -0.25f)
            {
                // Bước lặp 1 & 2: tìm giao điểm (X, Z) chính xác của tia súng nghiêng với bề mặt lồi lõm Point Cloud
                for (int iter = 0; iter < 2; iter++)
                {
                    if (SampleCompensatedSurface(queryX, queryZ, out float cy, out Vector3 cn))
                    {
                        surfY = cy; surfNorm = cn;
                    }
                    else if (cachedPlyLoaderForSpray.SampleSurfaceWorldHeightAndNormal(queryX, queryZ, out float gy, out Vector3 gn))
                    {
                        surfY = gy; surfNorm = gn;
                    }
                    float t = Mathf.Clamp((surfY - tcpWorldPos.y) / sprayDir.y, 0f, 0.45f);
                    Vector3 p = tcpWorldPos + sprayDir * t;
                    queryX = p.x;
                    queryZ = p.z;
                }
            }

            if (SampleCompensatedSurface(queryX, queryZ, out float finalCompY, out Vector3 finalCompN))
            {
                if (finalCompN.y < 0f) finalCompN = -finalCompN;
                return new Vector3(queryX, finalCompY, queryZ) + finalCompN.normalized * surfaceLiftM;
            }
            if (cachedPlyLoaderForSpray.SampleSurfaceWorldHeightAndNormal(queryX, queryZ, out float finalGridY, out Vector3 finalGridN))
            {
                if (finalGridN.y < 0f) finalGridN = -finalGridN;
                return new Vector3(queryX, finalGridY, queryZ) + finalGridN.normalized * surfaceLiftM;
            }
        }

        return tcpWorldPos;
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
            EnsureInkTrailMaterial();
            GameObject go = new GameObject($"SprayTrailSegment_{committedTrailRenderers.Count}");
            go.hideFlags = HideFlags.DontSave;
            go.transform.SetParent(transform, false);
            LineRenderer lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.sharedMaterial = inkTrailMaterial != null ? inkTrailMaterial : lineRenderer.sharedMaterial;
            lr.startWidth = sprayWidthMm * 0.001f;
            lr.endWidth = sprayWidthMm * 0.001f;
            lr.startColor = inkColor;
            lr.endColor = inkColor;
            lr.numCapVertices = 4;
            lr.numCornerVertices = 4;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
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
    public void StopPlaybackForNewFrame()
    {
        isPlaying = false;
        isMovingToManualTarget = false;
        isMovingToHome = false;
        SetSprayState(false);
    }

    public void SelectSampleByFullPath(string fullPath)
    {
        if (string.IsNullOrEmpty(fullPath) || !File.Exists(fullPath)) return;
        StopPlaybackForNewFrame();
        waypoints.Clear();
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
        SaveAllSettings();
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
        SaveAllSettings();
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

    public static float WorkbenchHeaderHeight { get; private set; } = 86f;
    public static float WorkbenchDockWidth { get; private set; } = 540f;
    public static bool WorkbenchDockVisible { get; private set; }

    public CalibrationManager EnsureCalibrationManager()
    {
        if (calibrationManager == null)
        {
            calibrationManager = GetComponent<CalibrationManager>() ?? FindAnyObjectByType<CalibrationManager>();
            if (calibrationManager == null)
            {
                calibrationManager = gameObject.AddComponent<CalibrationManager>();
            }
        }
        return calibrationManager;
    }

    private int trajSimSubTab = 0; // 0: Mô phỏng, 1: Quỹ đạo 2D->3D
    private int configSubTab = 0;  // 0: Robot & Đầu phun, 1: Hiệu chuẩn Camera
    public bool showCameraAxesInCalibration = true; // Hiện trục tọa độ XYZ của Camera khi đang hiệu chuẩn

    private bool ShouldDrawCameraAxes3D()
    {
        return showCameraAxesInCalibration && showDockPanel && (interfaceTab == 9 || (interfaceTab == 1 && configSubTab == 1));
    }

    private static readonly string[] SinglePanelTabTitles =
    {
        "4. Quỹ đạo & Mô phỏng",
        "7. Cấu hình & Hiệu chuẩn",
        "1. Nạp phôi 3D",
        "4. Quỹ đạo & Mô phỏng",
        "6. Xuất / Nhập",
        "8. Kiểm tra an toàn",
        "2. Views (Hiển thị 3D)",
        "3. Vá lỗ & 3D Mesh",
        "5. Đánh giá & Gauss",
        "7. Cấu hình & Hiệu chuẩn"
    };

    private void EnsureFloatingPanelsInitialized(float topSafeY)
    {
        if (floatingPanelsInitialized && singleMainPanel != null) return;

        // Quy đổi tab cũ sang cấu trúc gộp mới
        if (interfaceTab == 0) { interfaceTab = 3; trajSimSubTab = 0; }
        else if (interfaceTab == 9) { interfaceTab = 1; configSubTab = 1; }

        float w = Mathf.Clamp(Screen.width * 0.34f, 400f, 520f);
        float h = Mathf.Max(200f, Screen.height - topSafeY);
        float rightX = Mathf.Max(0f, Screen.width - w);

        singleMainPanel = new DraggableUIManager.PanelState(
            "main_inspector",
            SinglePanelTabTitles[Mathf.Clamp(interfaceTab, 0, SinglePanelTabTitles.Length - 1)],
            new Rect(rightX, topSafeY, w, h),
            new Vector2(360f, 260f),
            openByDefault: true
        );
        singleMainPanel.isDockedRight = true;
        DraggableUIManager.LoadPanelLayout(singleMainPanel, topSafeY);
        singleMainPanel.isDockedRight = true;
        singleMainPanel.isOpen = showDockPanel;
        floatingPanelsInitialized = true;
    }

    public void OpenOrFocusPanel(int panelIdx)
    {
        EnsureFloatingPanelsInitialized(WorkbenchHeaderHeight);
        if (panelIdx < 0 || panelIdx >= SinglePanelTabTitles.Length) return;

        if (panelIdx == 0)
        {
            interfaceTab = 3;
            trajSimSubTab = 0;
        }
        else if (panelIdx == 9)
        {
            interfaceTab = 1;
            configSubTab = 1;
        }
        else
        {
            interfaceTab = panelIdx;
        }

        showDockPanel = true;
        singleMainPanel.isDockedRight = true;
        singleMainPanel.isOpen = true;
        singleMainPanel.isCollapsed = false;
        singleMainPanel.scrollPos = Vector2.zero;
        singleMainPanel.title = SinglePanelTabTitles[Mathf.Clamp(interfaceTab, 0, SinglePanelTabTitles.Length - 1)];
        MarkSettingsDirty();
    }

    public void ResetAllPanelLayouts()
    {
        EnsureFloatingPanelsInitialized(WorkbenchHeaderHeight);
        float topY = WorkbenchHeaderHeight;
        float w = Mathf.Clamp(Screen.width * 0.34f, 400f, 520f);
        float h = Mathf.Max(200f, Screen.height - topY);
        singleMainPanel.isDockedRight = true;
        singleMainPanel.defaultRect = new Rect(Mathf.Max(0f, Screen.width - w), topY, w, h);
        DraggableUIManager.ResetPanelToDefault(singleMainPanel, topY);
        singleMainPanel.isOpen = true;
        showDockPanel = true;
        PlayerPrefs.Save();
    }

    private void OnGUI()
    {
        CleanupOrphanPhotoPlanes();
        AntigravityUITheme.EnsureInitialized();
        AntigravityUITheme.ApplyDarkScrollbarsToCurrentSkin();
        DraggableUIManager.BeginFrame();

        if (robotManual == null) EnsureComponents();
        EnsureCalibrationManager();
        if (landmarks3D.Count == 0) LoadLandmarksAndSegmentationForSample(selectedSampleName);
        EnsureCanonicalJeansCanvasInitialized();

        // Vẽ trực tiếp Đường bao Segment 3D, 12 Điểm Mốc Giải Phẫu AI & Quỹ Đạo 3D lên Viewport
        DrawViewport3DOverlaysInGUI();

        float screenW = Screen.width;
        bool narrow = screenW < 1180f;
        float navRows = narrow ? 2f : 1f;
        float headerTopH = 44f;
        float navH = navRows * 34f + 8f;
        float totalHeaderH = headerTopH + navH;

        WorkbenchHeaderHeight = totalHeaderH;
        EnsureFloatingPanelsInitialized(totalHeaderH);

        // Đăng ký thanh Header trên cùng vào vùng chặn chuột 3D
        DraggableUIManager.RegisterBlockingRect(new Rect(0f, 0f, screenW, totalHeaderH));

        // 1. NỀN THANH TIÊU ĐỀ & ĐIỀU HƯỚNG (GỌN GÀNG, KHÔNG CÓ THANH TRƯỢT NGANG!)
        GUI.DrawTexture(new Rect(0, 0, screenW, totalHeaderH), AntigravityUITheme.TexBgHeader);
        GUI.DrawTexture(new Rect(0, headerTopH - 1, screenW, 1), AntigravityUITheme.TexBorder);
        GUI.DrawTexture(new Rect(0, totalHeaderH - 1, screenW, 1), AntigravityUITheme.TexBorder);

        GUI.Label(new Rect(14, 6, 220, 22), "Dobot CR10A", AntigravityUITheme.StyleHeaderTitle);
        string savedNote = (Time.realtimeSinceStartup < lastSavedToastUntilRealtime) ? " | Đã lưu setup" : "";
        string subInfo = $"Phôi: {(string.IsNullOrEmpty(selectedSampleName) ? "-" : selectedSampleName)} | Size {detectedJeansSize} | {(hasCriticalErrors ? "Lỗi an toàn" : "Sẵn sàng")}{savedNote}";
        GUI.Label(new Rect(15, 26, 420, 16), subInfo, hasCriticalErrors ? AntigravityUITheme.StyleLabelRed : AntigravityUITheme.StyleLabelMuted);

        // Nút thao tác nhanh bên phải thanh trên cùng
        float btnH = 28f;
        float btnY = 8f;
        float rx = screenW - 12f;

        rx -= 82f;
        if (GUI.Button(new Rect(rx, btnY, 82f, btnH), showDockPanel ? "Ẩn bảng" : "Mở bảng", AntigravityUITheme.StyleButton))
        {
            showDockPanel = !showDockPanel;
            singleMainPanel.isOpen = showDockPanel;
            MarkSettingsDirty();
        }
        rx -= 94f;
        if (GUI.Button(new Rect(rx, btnY, 88f, btnH), "Cỡ mặc định", AntigravityUITheme.StyleButton))
        {
            ResetAllPanelLayouts();
        }
        rx -= 84f;
        if (GUI.Button(new Rect(rx, btnY, 78f, btnH), "Lưu setup", AntigravityUITheme.StyleButton))
        {
            SaveAllWorkbenchSetups(showToast: true);
        }
        rx -= 104f;
        if (GUI.Button(new Rect(rx, btnY, 98f, btnH), showGizmos ? "Quỹ đạo: Bật" : "Quỹ đạo: Tắt",
            showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton))
        {
            showGizmos = !showGizmos;
            MarkSettingsDirty();
        }
        rx -= 76f;
        if (GUI.Button(new Rect(rx, btnY, 70f, btnH), "Home", isMovingToHome ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton))
        {
            MoveToHomeGradually();
            ClearInkTrail();
        }
        rx -= 92f;
        if (GUI.Button(new Rect(rx, btnY, 86f, btnH), isPlaying ? "Tạm dừng" : "Chạy",
            isPlaying ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton))
        {
            isPlaying = !isPlaying;
            if (isPlaying) { isMovingToManualTarget = false; isMovingToHome = false; }
        }

        // 2. THANH CHỌN TAB (8 TAB GỌN TRÊN 1 HÀNG)
        string[] tabNames = {
            "1. Phôi", "2. Views", "3. Vá lỗ", "4. Quỹ đạo & MP",
            "5. Đánh giá", "6. Xuất", "7. Cấu hình", "8. An toàn"
        };
        int[] tabOrder = { 2, 6, 7, 3, 8, 4, 1, 5 };
        int cols = narrow ? 4 : 8;
        float padX = 12f;
        float gap = 5f;
        float tabW = (screenW - padX * 2f - (cols - 1) * gap) / cols;

        for (int n = 0; n < tabOrder.Length; n++)
        {
            int pIdx = tabOrder[n];
            int row = n / cols;
            int col = n % cols;
            Rect tRect = new Rect(padX + col * (tabW + gap), headerTopH + 4f + row * 34f, tabW, 28f);
            bool isCurrent = (interfaceTab == pIdx) && showDockPanel;

            if (GUI.Button(tRect, tabNames[n], isCurrent ? AntigravityUITheme.StyleTabActive : AntigravityUITheme.StyleTabInactive))
            {
                OpenOrFocusPanel(pIdx);
                if (pIdx == 5 && diagnosticResults.Count == 0) RunDiagnostics();
                if (pIdx == 6) LoadLandmarksAndSegmentationForSample(selectedSampleName);
                if (pIdx == 7 && detectedHoleCount == 0) DetectOcclusionAndHoles();
            }
            if (isCurrent)
            {
                GUI.DrawTexture(new Rect(tRect.x + 8f, tRect.yMax - 2f, tRect.width - 16f, 2f), AntigravityUITheme.TexSliderThumb);
            }
        }

        // 3. VẼ BẢNG TAB GẮN CỐ ĐỊNH BÊN PHẢI (KHÔNG DI CHUYỂN, CHỈ KÉO CẠNH TRÁI ĐỂ ĐỔI CỠ -> ĐẨY VIEWPORT 3D SANG TRÁI)
        singleMainPanel.isDockedRight = true;
        singleMainPanel.isOpen = showDockPanel;
        singleMainPanel.title = SinglePanelTabTitles[Mathf.Clamp(interfaceTab, 0, SinglePanelTabTitles.Length - 1)];

        if (showDockPanel)
        {
            bool prevGuiChanged = GUI.changed;
            GUI.changed = false;
            DraggableUIManager.DrawDraggablePanel(singleMainPanel, totalHeaderH, (innerW) =>
            {
                switch (interfaceTab)
                {
                    case 0:
                    case 3: DrawTrajectoryAndSimCombinedCard(innerW); break;
                    case 1:
                    case 9: DrawConfigAndCalibrationCombinedCard(innerW); break;
                    case 2: DrawCamPointCloudCard(innerW, null); break;
                    case 4: DrawExportImportCard(innerW, null); break;
                    case 5: DrawDiagnosticsCard(innerW, null); break;
                    case 6: DrawTopDownLandmarksAndSegCard(innerW, null); break;
                    case 7: DrawOcclusionCompensationCard(innerW, null); break;
                    case 8: DrawEvaluationAndSprayCard(innerW); break;
                }
            });
            if (GUI.changed)
            {
                MarkSettingsDirty();
            }
            GUI.changed = prevGuiChanged || GUI.changed;
            if (showDockPanel != singleMainPanel.isOpen)
            {
                showDockPanel = singleMainPanel.isOpen;
                MarkSettingsDirty();
            }
        }

        WorkbenchDockVisible = showDockPanel;
        WorkbenchDockWidth = showDockPanel ? singleMainPanel.rect.width : 0f;

        RobotCameraController rccViewport = EnsureCameraController();
        if (rccViewport != null)
        {
            rccViewport.UpdateCameraViewportRect();
        }
    }

    private void DrawTrajectoryAndSimCombinedCard(float width)
    {
        GUILayout.BeginHorizontal(GUILayout.Width(width));
        if (GUILayout.Button("Mô phỏng", trajSimSubTab == 0 ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            trajSimSubTab = 0;
        }
        if (GUILayout.Button("Quỹ đạo 2D -> 3D", trajSimSubTab == 1 ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            trajSimSubTab = 1;
        }
        GUILayout.EndHorizontal();
        GUILayout.Space(4);

        if (trajSimSubTab == 0)
            DrawSimCard(width, null);
        else
            DrawTrajectoryEditingCard(width, null);
    }

    private void DrawConfigAndCalibrationCombinedCard(float width)
    {
        GUILayout.BeginHorizontal(GUILayout.Width(width));
        if (GUILayout.Button("Robot & Đầu phun", configSubTab == 0 ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            configSubTab = 0;
        }
        if (GUILayout.Button("Hiệu chuẩn Camera", configSubTab == 1 ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            configSubTab = 1;
        }
        GUILayout.EndHorizontal();
        GUILayout.Space(4);

        if (configSubTab == 0)
            DrawRobotConfigCard(width, null);
        else
            DrawCameraCalibrationCard(width);
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
        GUILayout.Label("MÔ PHỎNG ROBOT", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(4);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(isPlaying ? "Tạm dừng" : "Chạy", isPlaying ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton, GUILayout.Height(30)))
        {
            isPlaying = !isPlaying;
            if (isPlaying) { isMovingToManualTarget = false; isMovingToHome = false; }
        }
        if (GUILayout.Button("Dừng", AntigravityUITheme.StyleButton, GUILayout.Height(30)))
        {
            isPlaying = false;
            isMovingToManualTarget = false;
        }
        if (GUILayout.Button("Home", isMovingToHome ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(30)))
        {
            MoveToHomeGradually();
            ClearInkTrail();
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(8);
        GUILayout.Label("TỐC ĐỘ TCP", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        useConstantTcpSpeed = GUILayout.Toggle(useConstantTcpSpeed, " Giữ tốc độ TCP đều (MovL)", AntigravityUITheme.StyleToggle);
        if (useConstantTcpSpeed)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Tốc độ TCP: {targetTcpSpeedMmPerSec:F0} mm/s", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(170));
            targetTcpSpeedMmPerSec = GUILayout.HorizontalSlider(targetTcpSpeedMmPerSec, 40f, 500f);
            if (sprayTechParams != null) sprayTechParams.targetSpeedMmPerSec = targetTcpSpeedMmPerSec;
            GUILayout.EndHorizontal();
        }
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Tốc độ phát: {playbackSpeed:F1}x", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(130));
        playbackSpeed = GUILayout.HorizontalSlider(playbackSpeed, 0.1f, 2.0f);
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("0.5x", AntigravityUITheme.StyleButton, GUILayout.Height(22))) playbackSpeed = 0.5f;
        if (GUILayout.Button("1.0x", Mathf.Abs(playbackSpeed - 1.0f) < 0.05f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22))) playbackSpeed = 1.0f;
        if (GUILayout.Button("1.5x", AntigravityUITheme.StyleButton, GUILayout.Height(22))) playbackSpeed = 1.5f;
        if (GUILayout.Button("2.0x", AntigravityUITheme.StyleButton, GUILayout.Height(22))) playbackSpeed = 2.0f;
        GUILayout.EndHorizontal();
        GUILayout.Label($"v={liveTcpSpeedMmPerSec:F0} mm/s | CV={measuredTcpSpeedCvPct:F1}% | t={(measuredCycleTimeSec > 0f ? measuredCycleTimeSec : cycleTimerSec):F1}s", AntigravityUITheme.StyleLabelGreen);
        GUILayout.EndVertical();

        GUILayout.Space(8);
        GUILayout.Label("TIẾN ĐỘ QUỸ ĐẠO", AntigravityUITheme.StyleSectionLabel);
        int wpTotal = Mathf.Max(1, waypoints.Count);
        int wpCur = Mathf.Clamp(currentWaypointIndex, 0, wpTotal - 1);
        float progress = (waypoints.Count <= 1 ? 0f : (float)wpCur / (wpTotal - 1) * 100f);
        float distMm = waypoints.Count > 0 ? waypoints[wpCur].distance_mm : 0f;
        GUILayout.Label($"Điểm: {wpCur + 1}/{wpTotal} ({progress:F0}%) | Quãng đường: {distMm:F0} mm", AntigravityUITheme.StyleLabelCyan);

        int newIdx = Mathf.RoundToInt(GUILayout.HorizontalSlider(wpCur, 0, wpTotal - 1));
        if (newIdx != currentWaypointIndex)
        {
            isPlaying = false;
            ApplyWaypoint(newIdx);
        }

        GUILayout.Space(8);
        GUILayout.Label("TỌA ĐỘ TCP (mm)", AntigravityUITheme.StyleSectionLabel);
        if (tcpTransform != null)
        {
            Vector3 worldTcp = tcpTransform.position;
            Vector3 robotTcpMm = globalReference != null ? globalReference.InverseTransformPoint(worldTcp) * 1000f : worldTcp * 1000f;
            GUILayout.Label($"X: {robotTcpMm.x:F1}   Y: {robotTcpMm.y:F1}   Z: {robotTcpMm.z:F1}", AntigravityUITheme.StyleLabelCyan);
        }

        GUILayout.Space(4);
        string statusText = robotManual != null && !string.IsNullOrEmpty(robotManual.ikStatus) ? robotManual.ikStatus : "Sẵn sàng.";
        GUILayout.Label(statusText, AntigravityUITheme.StyleLabelMuted);

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 1: CẤU HÌNH ROBOT (ROBOT CONFIG CARD)
    // =========================================================================
    private void DrawRobotConfigCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("CẤU HÌNH ROBOT", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);
        GUILayout.Label("1. VẬN TỐC & KHỚP", AntigravityUITheme.StyleSectionLabel);
        DrawAccelerationInput();
        GUILayout.Label($"Tốc độ tối đa: {Mathf.Clamp(maxJointSpeedDegPerSecond, 1f, 223f):F0}°/s", AntigravityUITheme.StyleLabelMuted);

        showJointSpeedControls = GUILayout.Toggle(showJointSpeedControls, " Chỉnh tốc độ từng khớp J1-J6", AntigravityUITheme.StyleToggle);
        if (showJointSpeedControls)
        {
            DrawJointSpeedInputs();
        }

        if (GUILayout.Button(showJointsSubPanel ? "Đóng chỉnh khớp q1-q6" : "Mở chỉnh khớp q1-q6", showJointsSubPanel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showJointsSubPanel = !showJointsSubPanel;
        }

        if (showJointsSubPanel && robotManual != null)
        {
            GUILayout.Space(4);
            robotManual.DrawJointsControlCard(width - 16, () => showJointsSubPanel = false);
        }

        GUILayout.Space(8);
        GUILayout.Label("2. OFFSET ĐẦU PHUN (BÍCH -> TCP)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Z: {toolLength:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(100));
        float rZ = GUILayout.HorizontalSlider(toolLength, 0f, 300f);
        if (Mathf.Abs(rZ - toolLength) > 0.05f) SetToolOffset(Mathf.Round(rZ * 10f) / 10f, toolOffsetX, toolOffsetY);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("0", Mathf.Abs(toolLength) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) SetToolOffset(0f, 0f, 0f);
        if (GUILayout.Button("50", Mathf.Abs(toolLength - 50f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) SetToolOffset(50f, 0f, 0f);
        if (GUILayout.Button("100", Mathf.Abs(toolLength - 100f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) SetToolOffset(100f, 0f, 0f);
        if (GUILayout.Button("150", Mathf.Abs(toolLength - 150f) < 0.1f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) SetToolOffset(150f, 0f, 0f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"X: {toolOffsetX:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(100));
        float rX = GUILayout.HorizontalSlider(toolOffsetX, -100f, 100f);
        if (Mathf.Abs(rX - toolOffsetX) > 0.1f) SetToolOffset(toolLength, Mathf.Round(rX * 10f) / 10f, toolOffsetY);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Y: {toolOffsetY:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(100));
        float rY = GUILayout.HorizontalSlider(toolOffsetY, -100f, 100f);
        if (Mathf.Abs(rY - toolOffsetY) > 0.1f) SetToolOffset(toolLength, toolOffsetX, Mathf.Round(rY * 10f) / 10f);
        GUILayout.EndHorizontal();

        GUILayout.Space(8);
        GUILayout.Label("3. VỆT PHUN MÔ PHỎNG", AntigravityUITheme.StyleSectionLabel);
        sprayPaintEnabled = GUILayout.Toggle(sprayPaintEnabled, " Hiện vệt phun", AntigravityUITheme.StyleToggle, GUILayout.Height(22));

        if (sprayPaintEnabled)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Bề rộng: {sprayWidthMm:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(120));
            float newWidth = GUILayout.HorizontalSlider(sprayWidthMm, 1f, 10f);
            if (Mathf.Abs(newWidth - sprayWidthMm) > 0.1f) UpdateSprayWidth(newWidth);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("2 mm", Mathf.Abs(sprayWidthMm - 2f) < 0.2f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) UpdateSprayWidth(2.0f);
            if (GUILayout.Button("4 mm", Mathf.Abs(sprayWidthMm - 4f) < 0.2f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) UpdateSprayWidth(4.0f);
            if (GUILayout.Button("Xóa vệt", AntigravityUITheme.StyleButton, GUILayout.Height(20))) { ClearInkTrail(); }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Dung sai: {sprayPathToleranceMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(120));
            sprayPathToleranceMm = GUILayout.HorizontalSlider(sprayPathToleranceMm, 5f, 50f);
            GUILayout.EndHorizontal();

            string sprayStateLabel = isSprayingNow ? "Trạng thái: Đang phun" : "Trạng thái: Tắt phun";
            GUILayout.Label(sprayStateLabel, isSprayingNow ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelRed);
        }

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 2: NẠP PHÔI 3D & CHỤP ZIVID / AI
    // =========================================================================
    private void DrawCamPointCloudCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("NẠP PHÔI 3D", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();

        GUILayout.Space(6);
        GUILayout.Label("1. CAMERA ZIVID", AntigravityUITheme.StyleSectionLabel);
        RobotCameraController rccCamCard = EnsureCameraController();
        bool oldEnabled = GUI.enabled;
        AiServerClient cameraClient = EnsureAiClient();
        GUI.enabled = oldEnabled && cameraClient != null && !cameraClient.isProcessing;
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Chụp Zivid", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(30)))
            cameraClient.CapturePreview();
        if (GUILayout.Button("Chụp + AI", AntigravityUITheme.StyleButton, GUILayout.Height(30)))
            cameraClient.CaptureAndDetect();
        GUILayout.EndHorizontal();
        GUI.enabled = oldEnabled;
        GUILayout.BeginHorizontal();
        GUIStyle zividCardBtnSt = (rccCamCard != null && rccCamCard.isZividCameraView) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("Góc nhìn Camera Zivid", zividCardBtnSt, GUILayout.Height(25))) { if (rccCamCard) rccCamCard.ToggleZividCameraView(); }
        GUIStyle pipCardBtnSt = (rccCamCard != null && rccCamCard.showZividPipWindow && !rccCamCard.isZividCameraView) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("Khung PiP", pipCardBtnSt, GUILayout.Width(96), GUILayout.Height(25))) { if (rccCamCard) rccCamCard.ToggleZividPip(); }
        GUILayout.EndHorizontal();
        if (cameraClient != null && !string.IsNullOrEmpty(cameraClient.lastStatusMessage))
            GUILayout.Label(cameraClient.lastStatusMessage, cameraClient.isServerOnline ? AntigravityUITheme.StyleLabelCyan : AntigravityUITheme.StyleLabelMuted);

        GUILayout.Space(8);
        GUILayout.Label("2. NẠP FILE .PLY & NHẬN DIỆN AI", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Chọn file .PLY...", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            OpenFileBrowserForSample();
        }
        if (!string.IsNullOrEmpty(customPlyPath) && GUILayout.Button("Tải lại", AntigravityUITheme.StyleButton, GUILayout.Width(72), GUILayout.Height(28)))
        {
            ReloadPointCloudFromPath(customPlyPath);
        }
        GUILayout.EndHorizontal();

        string displaySample = !string.IsNullOrEmpty(selectedSampleName) ? $"{selectedSampleName}.ply" : "Chưa chọn";
        int totalPts = plyLoader != null ? plyLoader.totalLoadedPoints : 0;
        GUILayout.Label($"File: {displaySample} ({totalPts:N0} điểm)", AntigravityUITheme.StyleLabelCyan);

        AiServerClient aiClient = EnsureAiClient();
        if (aiClient != null)
        {
            bool newAuto = GUILayout.Toggle(aiClient.autoModeEnabled, " Tự động nhận từ AI (UDP 5006)", AntigravityUITheme.StyleToggle);
            if (newAuto != aiClient.autoModeEnabled) aiClient.SetAutoMode(newAuto);

            if (!aiClient.autoModeEnabled)
            {
                bool isAiBusy = aiClient.isProcessing;
                string aiBtnText = isAiBusy ? "Đang nhận diện..." : $"Nhận diện AI [{selectedSampleName}]";
                GUIStyle aiStyle = isAiBusy ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton;
                if (GUILayout.Button(aiBtnText, aiStyle, GUILayout.Height(28)))
                {
                    DetectJeansSizeAndSwitchSprayTemplate(selectedSampleName);
                    aiClient.TriggerDetectionWithSample(detectedJeansSize, selectedSampleName);
                }
            }
        }

        GUILayout.Space(6);
        DrawJeansSizeDetectionSummaryBox();

        GUILayout.Space(8);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Mở Views (3D / Mesh / Mốc)", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            OpenOrFocusPanel(6);
        }
        if (GUILayout.Button("Hiệu chuẩn Camera", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            OpenOrFocusPanel(9);
        }
        GUILayout.EndHorizontal();

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 3: QUỸ ĐẠO & CHỈNH SỬA TỪNG ĐOẠN (TRAJECTORY EDITING CARD)
    // =========================================================================
    private void DrawTrajectoryEditingCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("QUỸ ĐẠO 2D -> 3D", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);
        DrawCanonicalJeans2DCanvasSection(width - 16f);

        GUILayout.Space(8);
        GUILayout.Label("1. SINH QUỸ ĐẠO (P1 / P3 / P4)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("P1: Zigzag", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            GenerateAndApplyMethodTrajectory("P1");
        }
        if (GUILayout.Button("P3: 12 Mốc", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            GenerateAndApplyMethodTrajectory("P3");
        }
        if (GUILayout.Button("P4: ICP/TPS", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
        {
            GenerateAndApplyMethodTrajectory("P4");
        }
        GUILayout.EndHorizontal();
        if (GUILayout.Button("So sánh P1 / P3 / P4", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            OpenOrFocusPanel(8);
            RunAllMethodsComparison();
        }
        if (!string.IsNullOrEmpty(comparisonStatusMsg))
        {
            GUILayout.Label(comparisonStatusMsg, AntigravityUITheme.StyleLabelCyan);
        }
        GUILayout.EndVertical();

        GUILayout.Space(8);
        GUILayout.Label("2. MÁY CHỦ AI", AntigravityUITheme.StyleSectionLabel);
        AiServerClient aiClient = EnsureAiClient();
        if (aiClient != null)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("IP:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(24));
            string newIp = GUILayout.TextField(aiClient.serverIp, AntigravityUITheme.StyleTextField, GUILayout.Width(115));
            if (newIp != aiClient.serverIp) { aiClient.serverIp = newIp; aiClient.SaveSettings(); }

            GUILayout.Label("Cổng:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(38));
            string portStr = GUILayout.TextField(aiClient.serverPort.ToString(), AntigravityUITheme.StyleTextField, GUILayout.Width(52));
            int parsedPort;
            if (int.TryParse(portStr, out parsedPort) && parsedPort != aiClient.serverPort) { aiClient.serverPort = parsedPort; aiClient.SaveSettings(); }

            if (GUILayout.Button("Ping", AntigravityUITheme.StyleButton, GUILayout.Width(56), GUILayout.Height(22)))
            {
                aiClient.CheckServerStatus();
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUIStyle bUdp = (aiClient.protocol == AiServerClient.ProtocolType.UDP) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            GUIStyle bHttp = (aiClient.protocol == AiServerClient.ProtocolType.HTTP) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            if (GUILayout.Button("UDP (5005)", bUdp, GUILayout.Height(22))) aiClient.SetProtocol(AiServerClient.ProtocolType.UDP);
            if (GUILayout.Button("HTTP (5000)", bHttp, GUILayout.Height(22))) aiClient.SetProtocol(AiServerClient.ProtocolType.HTTP);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            bool isBusy = aiClient.isProcessing;
            string aiBtnTitle = isBusy ? "Đang xử lý..." : $"Nhận diện [{selectedSampleName}]";
            GUIStyle btnSt = isBusy ? AntigravityUITheme.StyleButton : AntigravityUITheme.StylePrimaryButton;
            if (GUILayout.Button(aiBtnTitle, btnSt, GUILayout.Height(28)))
            {
                DetectJeansSizeAndSwitchSprayTemplate(selectedSampleName);
                aiClient.TriggerDetectionWithSample(detectedJeansSize, selectedSampleName);
            }
            if (GUILayout.Button("Chọn file...", AntigravityUITheme.StyleButton, GUILayout.Width(86), GUILayout.Height(28)))
            {
                OpenFileBrowserForSample();
            }
            if (GUILayout.Button("Mẫu mặc định", AntigravityUITheme.StyleButton, GUILayout.Width(92), GUILayout.Height(28)))
            {
                LoadPantsOutlineTrajectory();
            }
            GUILayout.EndHorizontal();

            bool newAutoMode = GUILayout.Toggle(aiClient.autoModeEnabled, " Tự động nhận từ AI (UDP 5006)", AntigravityUITheme.StyleToggle);
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
            if (GUILayout.Button("Nạp quỹ đạo mẫu", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26))) LoadPantsOutlineTrajectory();
        }

        GUILayout.Space(8);
        GUILayout.Label("3. CHỈNH ĐIỂM QUỸ ĐẠO", AntigravityUITheme.StyleSectionLabel);
        showGizmos = GUILayout.Toggle(showGizmos, " Hiện đường quỹ đạo 3D", AntigravityUITheme.StyleToggle);
        int wpCount = waypoints.Count;
        int wpIdx = wpCount > 0 ? Mathf.Clamp(currentWaypointIndex, 0, wpCount - 1) : 0;

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("-10", AntigravityUITheme.StyleButton, GUILayout.Width(38), GUILayout.Height(24))) ApplyWaypoint(Mathf.Max(0, wpIdx - 10));
        if (GUILayout.Button("-1", AntigravityUITheme.StyleButton, GUILayout.Width(34), GUILayout.Height(24))) ApplyWaypoint(Mathf.Max(0, wpIdx - 1));
        GUILayout.Label($"Điểm {wpIdx + 1}/{Mathf.Max(1, wpCount)}", AntigravityUITheme.StyleLabelCyan);
        if (GUILayout.Button("+1", AntigravityUITheme.StyleButton, GUILayout.Width(34), GUILayout.Height(24))) ApplyWaypoint(Mathf.Min(wpCount - 1, wpIdx + 1));
        if (GUILayout.Button("+10", AntigravityUITheme.StyleButton, GUILayout.Width(38), GUILayout.Height(24))) ApplyWaypoint(Mathf.Min(wpCount - 1, wpIdx + 10));
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
            GUILayout.Label($"TCP (mm): X={wp.targetPosGlobal_mm.x:F1}  Y={wp.targetPosGlobal_mm.y:F1}  Z={wp.targetPosGlobal_mm.z:F1}", AntigravityUITheme.StyleLabelMuted);
            GUILayout.Label($"Khớp (°): {wp.jointAngles_deg[0]:F1}, {wp.jointAngles_deg[1]:F1}, {wp.jointAngles_deg[2]:F1}, {wp.jointAngles_deg[3]:F1}, {wp.jointAngles_deg[4]:F1}, {wp.jointAngles_deg[5]:F1}", AntigravityUITheme.StyleLabelCyan);
            GUILayout.EndVertical();

            GUILayout.Space(6);
            GUILayout.Label("GÓC HƯỚNG ĐẦU PHUN (RPY)", AntigravityUITheme.StyleSectionLabel);

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Roll: {wp.roll_deg:F1}°", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(82));
            if (GUILayout.Button("-5", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg - 5f, wp.pitch_deg, wp.yaw_deg);
            if (GUILayout.Button("-1", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg - 1f, wp.pitch_deg, wp.yaw_deg);
            float rSlider = GUILayout.HorizontalSlider(wp.roll_deg, -180f, 180f);
            if (Mathf.Abs(rSlider - wp.roll_deg) > 0.1f) UpdateWaypointRpy(wpIdx, Mathf.Round(rSlider * 10f) / 10f, wp.pitch_deg, wp.yaw_deg);
            if (GUILayout.Button("+1", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg + 1f, wp.pitch_deg, wp.yaw_deg);
            if (GUILayout.Button("+5", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg + 5f, wp.pitch_deg, wp.yaw_deg);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Pitch: {wp.pitch_deg:F1}°", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(82));
            if (GUILayout.Button("-5", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg - 5f, wp.yaw_deg);
            if (GUILayout.Button("-1", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg - 1f, wp.yaw_deg);
            float pSlider = GUILayout.HorizontalSlider(wp.pitch_deg, -90f, 90f);
            if (Mathf.Abs(pSlider - wp.pitch_deg) > 0.1f) UpdateWaypointRpy(wpIdx, wp.roll_deg, Mathf.Round(pSlider * 10f) / 10f, wp.yaw_deg);
            if (GUILayout.Button("+1", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg + 1f, wp.yaw_deg);
            if (GUILayout.Button("+5", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg + 5f, wp.yaw_deg);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Yaw: {wp.yaw_deg:F1}°", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(82));
            if (GUILayout.Button("-5", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, wp.yaw_deg - 5f);
            if (GUILayout.Button("-1", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, wp.yaw_deg - 1f);
            float ySlider = GUILayout.HorizontalSlider(wp.yaw_deg, -180f, 180f);
            if (Mathf.Abs(ySlider - wp.yaw_deg) > 0.1f) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, Mathf.Round(ySlider * 10f) / 10f);
            if (GUILayout.Button("+1", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, wp.yaw_deg + 1f);
            if (GUILayout.Button("+5", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) UpdateWaypointRpy(wpIdx, wp.roll_deg, wp.pitch_deg, wp.yaw_deg + 5f);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Thẳng đứng", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                UpdateWaypointRpy(wpIdx, 180f, 0f, 0f);
            }
            if (GUILayout.Button("Áp dụng về sau", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                PropagateRpyForward(wpIdx);
            }
            if (GUILayout.Button("Áp dụng tất cả", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                ApplyRpyToAllWaypoints(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
            }
            GUILayout.EndHorizontal();

            Quaternion rot = GetWaypointFlangeRotation(wp);
            float[] seedAngles = wpIdx > 0 ? waypoints[wpIdx - 1].jointAngles_rad : null;
            List<DobotCR10AKinematics.IKSolution> sols = DobotCR10AKinematics.SolveTcpPose(
                wp.targetPosGlobal_mm, rot, seedAngles, toolLength, useAdaptiveTilt: true, toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY);

            if (sols != null && sols.Count > 0)
            {
                GUILayout.Space(4);
                GUILayout.Label($"NGHIỆM IK ({sols.Count})", AntigravityUITheme.StyleSectionLabel);
                waypointSolsScroll = GUILayout.BeginScrollView(waypointSolsScroll, GUILayout.Height(Mathf.Min(120f, sols.Count * 24f + 4f)));
                for (int i = 0; i < sols.Count; i++)
                {
                    var s = sols[i];
                    bool isCurrentChoice = IsMatchingConfiguration(wp.jointAngles_rad, s.jointsRad);
                    string pref = isCurrentChoice ? "[Đang chọn] " : $"#{i + 1} ";
                    string label = $"{pref}{s.branchName} ({s.positionErrorMm:F3} mm)";

                    GUIStyle btnStyle = isCurrentChoice ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
                    if (GUILayout.Button(label, btnStyle, GUILayout.Height(22)))
                    {
                        ApplyConfigurationToWaypoint(wpIdx, s, i);
                    }
                }
                GUILayout.EndScrollView();

                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Lan truyền về sau", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
                {
                    PropagateCurrentConfigurationForward(wpIdx);
                }
                if (GUILayout.Button("Mặc định Q*", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
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
        GUILayout.Label("XUẤT / NHẬP CHƯƠNG TRÌNH", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);

        if (hasCriticalErrors)
        {
            GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
            GUILayout.Label("KHÓA AN TOÀN: Quỹ đạo có lỗi, đang chặn xuất.", AntigravityUITheme.StyleLabelRed);
            forceBypassSafetyInterlock = GUILayout.Toggle(forceBypassSafetyInterlock, " Mở khóa cưỡng bức (Kỹ sư)", AntigravityUITheme.StyleToggle);
            GUILayout.EndVertical();
            GUILayout.Space(6);
        }

        bool canExport = !hasCriticalErrors || forceBypassSafetyInterlock;
        GUIStyle exportBtnStyle = canExport ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;

        GUILayout.Label("1. XUẤT CHƯƠNG TRÌNH", AntigravityUITheme.StyleSectionLabel);
        if (GUILayout.Button("Xuất Python Dobot CR10A (.py)", exportBtnStyle, GUILayout.Height(28)))
        {
            ExportDobotPythonScript();
        }
        if (GUILayout.Button("Xuất ROS 2 / MoveIt 2 (.yaml)", exportBtnStyle, GUILayout.Height(28)))
        {
            ExportRos2TrajectoryYaml();
        }
        if (GUILayout.Button("Xuất Quỹ Đạo 6-DOF (.csv)", exportBtnStyle, GUILayout.Height(28)))
        {
            ExportUpdatedCsvTrajectory();
        }
        if (GUILayout.Button("Mở thư mục xuất", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            OpenExportFolder();
        }

        GUILayout.Space(8);
        GUILayout.Label("2. NHẬP FILE QUỸ ĐẠO (.CSV)", AntigravityUITheme.StyleSectionLabel);
        customImportCsvPath = GUILayout.TextField(customImportCsvPath, AntigravityUITheme.StyleTextField);
        if (GUILayout.Button("Nạp file CSV", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
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
            GUILayout.Label("XEM TRƯỚC", AntigravityUITheme.StyleSectionLabel);
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
        GUILayout.Label("KIỂM TRA AN TOÀN", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        if (hasCriticalErrors)
        {
            GUILayout.Label("Phát hiện lỗi an toàn - Đã khóa xuất lệnh.", AntigravityUITheme.StyleLabelRed);
        }
        else
        {
            GUILayout.Label("Đạt chuẩn an toàn (không va chạm / kỳ dị).", AntigravityUITheme.StyleLabelGreen);
        }
        GUILayout.EndVertical();

        GUILayout.Space(6);
        if (GUILayout.Button("Quét lại quỹ đạo", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            RunDiagnostics();
        }

        GUILayout.Space(8);
        GUILayout.Label($"KẾT QUẢ QUÉT ({diagnosticResults.Count})", AntigravityUITheme.StyleSectionLabel);
        diagnosticsScroll = GUILayout.BeginScrollView(diagnosticsScroll, GUILayout.Height(240));

        for (int i = 0; i < diagnosticResults.Count; i++)
        {
            var item = diagnosticResults[i];
            GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
            GUILayout.BeginHorizontal();

            string tag = item.severity == DiagnosticItem.Severity.Error ? "[LỖI]" : (item.severity == DiagnosticItem.Severity.Warning ? "[CẢNH BÁO]" : "[OK]");
            GUIStyle textStyle = item.severity == DiagnosticItem.Severity.Error ? AntigravityUITheme.StyleLabelRed : (item.severity == DiagnosticItem.Severity.Warning ? AntigravityUITheme.StyleLabelYellow : AntigravityUITheme.StyleLabelGreen);

            GUILayout.Label($"{tag} {item.category}", textStyle, GUILayout.ExpandWidth(true));
            if (item.waypointIndex >= 0)
            {
                if (GUILayout.Button($"Điểm #{item.waypointIndex + 1}", AntigravityUITheme.StyleButton, GUILayout.Width(80), GUILayout.Height(20)))
                {
                    OpenOrFocusPanel(3);
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
            float speedRatioPct = Mathf.Clamp(playbackSpeed * 30f, 5f, 100f);
            string code = TrajectoryEvaluationEngine.BuildRealDobotPythonScript(
                waypoints, toolLength, toolOffsetX, toolOffsetY,
                sprayWidthMm, speedRatioPct, targetTcpSpeedMmPerSec);

            File.WriteAllText(outputPath, code);
            exportStatusMessage = $"Đã xuất Python ({waypoints.Count} điểm): {Path.GetFileName(outputPath)}";
            lastExportedCodePreview = code.Substring(0, Mathf.Min(1500, code.Length));
            Debug.Log($"[CR10A Export] {exportStatusMessage}");
        }
        catch (Exception ex)
        {
            exportStatusMessage = $"Lỗi khi xuất Python: {ex.Message}";
            Debug.LogError($"[CR10A Export] {exportStatusMessage}");
        }
    }

    public void ExportRos2TrajectoryYaml(string outputPath = null)
    {
        if (hasCriticalErrors && !forceBypassSafetyInterlock)
        {
            exportStatusMessage = "[CHẶN XUẤT] Không thể xuất ROS 2 YAML do có lỗi an toàn nghiêm trọng!";
            Debug.LogError(exportStatusMessage);
            return;
        }

        string exportDir = Path.Combine(Application.dataPath, "..", "output_drawing");
        if (!Directory.Exists(exportDir)) Directory.CreateDirectory(exportDir);
        if (string.IsNullOrEmpty(outputPath))
        {
            outputPath = Path.Combine(exportDir, "dobot_cr10a_ros2_trajectory.yaml");
        }

        try
        {
            string yaml = TrajectoryEvaluationEngine.BuildRos2TrajectoryYaml(waypoints, targetTcpSpeedMmPerSec);
            File.WriteAllText(outputPath, yaml);
            exportStatusMessage = $"Đã xuất ROS 2 YAML ({waypoints.Count} điểm): {Path.GetFileName(outputPath)}";
            lastExportedCodePreview = yaml.Substring(0, Mathf.Min(1200, yaml.Length));
            Debug.Log($"[CR10A Export ROS2] {exportStatusMessage}");
        }
        catch (Exception ex)
        {
            exportStatusMessage = $"Lỗi khi xuất ROS 2 YAML: {ex.Message}";
            Debug.LogError($"[CR10A Export ROS2] {exportStatusMessage}");
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
            exportStatusMessage = $"Đã xuất CSV ({waypoints.Count} điểm): {Path.GetFileName(outputPath)}";
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

        // 2. Kiểm tra quỹ đạo toàn diện (Khâu 7 — Safety Interlock theo file góp ý):
        //    IK, Tầm với, Giới hạn khớp cơ khí CR10A, Cú nhảy khớp lớn, Va chạm 3D toàn bộ khâu Link2..TCP với bàn,
        //    Điểm kỳ dị, Góc nghiêng súng <= θ_max, và Vùng cấm phun (Keep-Out Zones).
        int unsolvedCount = 0;
        int outOfReachCount = 0;
        int jointLimitCount = 0;
        int jointJumpCount = 0;
        int lowZCount = 0;
        int linkCollisionCount = 0;
        int tiltExceedCount = 0;
        int keepOutViolationCount = 0;
        int wristSingularityCount = 0;
        int elbowSingularityCount = 0;
        int invertedCount = 0;
        float maxAllowedTilt = sprayTechParams != null ? sprayTechParams.maxTiltDeg : 30f;

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
                        category = "Va chạm bàn — TCP (Collision)",
                        message = $"Waypoint #{i} có TCP Z = {wp.targetPosGlobal_mm.z:F1}mm quá sát mặt bàn (<10mm)!",
                        waypointIndex = i
                    });
                }
            }

            if (wp.isSolved && wp.jointAngles_deg != null && wp.jointAngles_deg.Length >= 6)
            {
                // Kiểm tra giới hạn cơ khí 6 khớp CR10A (J3 ±160°...)
                if (!DobotCR10AKinematics.IsWithinJointLimits(wp.jointAngles_deg, out int badJ, out float badAng))
                {
                    jointLimitCount++;
                    if (jointLimitCount <= 3)
                    {
                        diagnosticResults.Add(new DiagnosticItem
                        {
                            severity = DiagnosticItem.Severity.Error,
                            category = "Giới hạn khớp cơ khí (Joint Limit)",
                            message = $"Waypoint #{i}: Khớp J{badJ + 1} = {badAng:F1}° vượt giới hạn cơ khí Dobot CR10A!",
                            waypointIndex = i
                        });
                    }
                }

                // Kiểm tra va chạm 3D thực của các khâu Link2..Flange..TCP với mặt bàn (Z < 15mm) và tự va chạm đế (R < 110mm, Z < 180mm)
                Vector3[] linkPts = DobotCR10AKinematics.ForwardAllJointPositionsRobotMm(
                    wp.jointAngles_rad, toolLength, toolOffsetX, toolOffsetY);
                for (int l = 2; l <= 5; l++)
                {
                    float horizR = Mathf.Sqrt(linkPts[l].x * linkPts[l].x + linkPts[l].y * linkPts[l].y);
                    bool hitTable = linkPts[l].z < 20f;
                    bool hitBase = (l >= 3 && horizR < 115f && linkPts[l].z < 190f);
                    if (hitTable || hitBase)
                    {
                        linkCollisionCount++;
                        if (linkCollisionCount <= 3)
                        {
                            diagnosticResults.Add(new DiagnosticItem
                            {
                                severity = DiagnosticItem.Severity.Error,
                                category = hitTable ? "Va chạm khâu Robot với bàn (3D Link)" : "Tự va chạm thân đế Robot",
                                message = $"Waypoint #{i}: Khâu J{l + 1} tại ({linkPts[l].x:F0}, {linkPts[l].y:F0}, Z={linkPts[l].z:F0}mm) có nguy cơ va chạm!",
                                waypointIndex = i
                            });
                        }
                        break;
                    }
                }

                // Kiểm tra bước nhảy góc khớp đột ngột giữa 2 waypoint liền kề trên cùng đoạn phun
                if (i > 0 && wp.sprayOn && waypoints[i - 1].isSolved)
                {
                    float maxJump = 0f;
                    int jumpJoint = 0;
                    for (int j = 0; j < 6; j++)
                    {
                        float dq = Mathf.Abs(Mathf.DeltaAngle(waypoints[i - 1].jointAngles_deg[j], wp.jointAngles_deg[j]));
                        if (dq > maxJump) { maxJump = dq; jumpJoint = j; }
                    }
                    if (maxJump > 45f)
                    {
                        jointJumpCount++;
                        if (jointJumpCount <= 3)
                        {
                            diagnosticResults.Add(new DiagnosticItem
                            {
                                severity = DiagnosticItem.Severity.Warning,
                                category = "Bước nhảy khớp lớn (>45°)",
                                message = $"Waypoint #{i}: Khớp J{jumpJoint + 1} nhảy {maxJump:F1}° so với Wp #{i - 1} (dễ giật khi phun)!",
                                waypointIndex = i
                            });
                        }
                    }
                }

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

            // Kiểm tra góc nghiêng súng so với trục đứng <= θ_max (khi đang ở chế độ bàn phẳng)
            if (!showStandingMold3D)
            {
                Vector3 sprayAxisR = DobotCR10AKinematics.RpyToSprayAxisRobot(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                float tiltFromDown = Vector3.Angle(sprayAxisR, new Vector3(0f, 0f, -1f));
                if (tiltFromDown > maxAllowedTilt + 5f)
                {
                    tiltExceedCount++;
                    if (tiltExceedCount <= 3)
                    {
                        diagnosticResults.Add(new DiagnosticItem
                        {
                            severity = DiagnosticItem.Severity.Warning,
                            category = $"Góc nghiêng súng > θ_max ({maxAllowedTilt:F0}°)",
                            message = $"Waypoint #{i} nghiêng {tiltFromDown:F1}° vượt ngưỡng công nghệ {maxAllowedTilt:F0}°!",
                            waypointIndex = i
                        });
                    }
                }

                // Kiểm tra vùng cấm phun (Keep-Out Zones) cho các điểm đang bật phun
                if (wp.sprayOn && garmentContourWorld != null && garmentContourWorld.Count >= 4 && landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
                {
                    float marginMm = sprayTechParams != null ? sprayTechParams.borderMarginMm : 15f;
                    if (!TrajectoryEvaluationEngine.IsInsideValidSprayZone(
                        wp.targetPosWorld_m.x, wp.targetPosWorld_m.z,
                        garmentContourWorld, landmarksWorldPos, marginMm * 0.5f, out string reason))
                    {
                        keepOutViolationCount++;
                        if (keepOutViolationCount <= 3)
                        {
                            diagnosticResults.Add(new DiagnosticItem
                            {
                                severity = DiagnosticItem.Severity.Warning,
                                category = "Vi phạm Vùng Cấm / Biên vải",
                                message = $"Waypoint #{i} đang bật phun nhưng rơi vào [{reason}].",
                                waypointIndex = i
                            });
                        }
                    }
                }
            }

            if (Mathf.Abs(wp.pitch_deg) > 45.0f && Mathf.Abs(wp.pitch_deg - 90.0f) > 10.0f)
            {
                invertedCount++;
            }
        }

        if (unsolvedCount > 0 || outOfReachCount > 0 || lowZCount > 0 || jointLimitCount > 0 || linkCollisionCount > 0)
        {
            hasCriticalErrors = true;
        }

        if (keepOutViolationCount > 3)
        {
            diagnosticResults.Add(new DiagnosticItem
            {
                severity = DiagnosticItem.Severity.Warning,
                category = "Tổng hợp Vùng cấm phun",
                message = $"Có {keepOutViolationCount} điểm bật phun sát mép vải hoặc vùng cấm (cạp/đũng/gấu).",
                waypointIndex = -1
            });
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
            RefreshRealCameraImageForCurrentSample();
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
    // CARD 6: TAB VIEWS (GÓC NHÌN, 3D MESH / POINT CLOUD, MỐC AI & LỚP HIỂN THỊ)
    // =========================================================================
    private void DrawTopDownLandmarksAndSegCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("VIEWS - HIỂN THỊ 3D", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        RobotCameraController rcc = EnsureCameraController();
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();

        GUILayout.Space(6);
        GUILayout.Label("1. GÓC NHÌN CAMERA", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("3/4", AntigravityUITheme.StyleButton, GUILayout.Height(25))) { if (rcc) rcc.SetPresetPerspective(); }
        if (GUILayout.Button("Trên", AntigravityUITheme.StyleButton, GUILayout.Height(25))) { if (rcc) rcc.SetPresetTopDown(); }
        if (GUILayout.Button("Trước", AntigravityUITheme.StyleButton, GUILayout.Height(25))) { if (rcc) rcc.SetPresetFront(); }
        if (GUILayout.Button("Ngang", AntigravityUITheme.StyleButton, GUILayout.Height(25))) { if (rcc) rcc.SetPresetSide(); }
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        GUIStyle zividViewBtnSt = (rcc != null && rcc.isZividCameraView) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("Góc Camera Zivid", zividViewBtnSt, GUILayout.Height(25))) { if (rcc) rcc.ToggleZividCameraView(); }
        GUIStyle pipViewBtnSt = (rcc != null && rcc.showZividPipWindow && !rcc.isZividCameraView) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("Khung PiP Cam", pipViewBtnSt, GUILayout.Height(25))) { if (rcc) rcc.ToggleZividPip(); }
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Focus Phôi", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.FocusGarment(); }
        if (GUILayout.Button("Focus Robot", AntigravityUITheme.StyleButton, GUILayout.Height(24))) { if (rcc) rcc.ResetFocus(); }
        GUILayout.EndHorizontal();
        FixedRigCamera rigViewCam = rcc != null ? rcc.EnsureFixedRigCamera(createIfMissing: false) : FindAnyObjectByType<FixedRigCamera>();
        if (rigViewCam != null)
        {
            rigViewCam.showRuntimeRigVisual = GUILayout.Toggle(rigViewCam.showRuntimeRigVisual, " Hiện mô hình Camera Zivid & vùng FOV 3D", AntigravityUITheme.StyleToggle);
            rigViewCam.showRuntimeFovFrustum = rigViewCam.showRuntimeRigVisual;
        }

        GUILayout.Space(6);
        GUILayout.Label("2. CHẾ ĐỘ PHÔI (3D MESH / POINT CLOUD)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        bool isMeshMode = plyLoader != null && plyLoader.renderMode == PLYDirectLoader.WorkpieceRenderMode.Mesh3D;
        bool isCloudVis = plyLoader != null && plyLoader.IsVisible();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("3D Mesh", isMeshMode ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            if (plyLoader != null) plyLoader.SetRenderMode(PLYDirectLoader.WorkpieceRenderMode.Mesh3D);
        }
        if (GUILayout.Button("Point Cloud", !isMeshMode ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            if (plyLoader != null) plyLoader.SetRenderMode(PLYDirectLoader.WorkpieceRenderMode.PointCloud);
        }
        if (GUILayout.Button(isCloudVis ? "Ẩn phôi" : "Hiện phôi", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            TogglePointCloudVisible(!isCloudVis);
        }
        GUILayout.EndHorizontal();
        if (GUILayout.Button("Cập nhật 3D Mesh", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            if (plyLoader != null) plyLoader.ConvertPointCloudToSurfaceMesh();
        }
        int totalPts = plyLoader != null ? plyLoader.totalLoadedPoints : 0;
        GUILayout.Label($"Điểm 3D: {totalPts:N0} | Đang xem: {(isMeshMode ? "3D Mesh" : "Point Cloud")}", AntigravityUITheme.StyleLabelMuted);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("3. CÁC LỚP HIỂN THỊ 3D", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        showLandmarks3D = GUILayout.Toggle(showLandmarks3D, " 12 mốc AI & khung xương", AntigravityUITheme.StyleToggle);
        if (showLandmarks3D) showLandmarkLabels = GUILayout.Toggle(showLandmarkLabels, " Nhãn tên & cao độ Z", AntigravityUITheme.StyleToggle);
        showSegmentationContour3D = GUILayout.Toggle(showSegmentationContour3D, " Đường bao thân quần", AntigravityUITheme.StyleToggle);
        showSprayZones3D = GUILayout.Toggle(showSprayZones3D, " Vùng phun 2 ống quần", AntigravityUITheme.StyleToggle);
        showGizmos = GUILayout.Toggle(showGizmos, " Đường quỹ đạo 3D", AntigravityUITheme.StyleToggle);
        showHoleRegionOutlines3D = GUILayout.Toggle(showHoleRegionOutlines3D, " Viền đỏ vùng cần vá", AntigravityUITheme.StyleToggle);

        bool prevShowInp = showInpaintedSurface3D;
        showInpaintedSurface3D = GUILayout.Toggle(showInpaintedSurface3D, " Lớp vá 3D", AntigravityUITheme.StyleToggle);
        if (showInpaintedSurface3D != prevShowInp && inpaintedMeshObject != null)
        {
            var mr = inpaintedMeshObject.GetComponent<MeshRenderer>();
            if (mr != null) mr.enabled = showInpaintedSurface3D;
        }

        bool prevSolid = renderInpaintedAsSolidMesh;
        renderInpaintedAsSolidMesh = GUILayout.Toggle(renderInpaintedAsSolidMesh, " Lớp vá dạng 3D Mesh đặc", AntigravityUITheme.StyleToggle);
        if (renderInpaintedAsSolidMesh != prevSolid && compensatedSurfacePointsWorld.Count > 0)
        {
            BuildAndDisplayInpaintedMesh(plyLoader);
        }

        bool prevHl = highlightInpaintedMesh;
        highlightInpaintedMesh = GUILayout.Toggle(highlightInpaintedMesh, " Tô màu Cyan vùng vá", AntigravityUITheme.StyleToggle);
        if (highlightInpaintedMesh != prevHl && compensatedSurfacePointsWorld.Count > 0)
        {
            BuildAndDisplayInpaintedMesh(plyLoader);
        }

        showGunTiltVectors3D = GUILayout.Toggle(showGunTiltVectors3D, " Tia hướng súng", AntigravityUITheme.StyleToggle);

        bool prevHm = showDepositionHeatmap3D;
        showDepositionHeatmap3D = GUILayout.Toggle(showDepositionHeatmap3D, " Heatmap phun Gauss 3D", AntigravityUITheme.StyleToggle);
        if (showDepositionHeatmap3D != prevHm)
        {
            if (showDepositionHeatmap3D && waypoints.Count >= 2)
            {
                TrajectoryEvaluationEngine.SimulateGaussianDepositionAndHeatmap(
                    waypoints, this, plyLoader, sprayTechParams, true, ref depositionHeatmapObj, out _, out _, out _);
            }
            else if (depositionHeatmapObj != null)
            {
                depositionHeatmapObj.SetActive(false);
            }
        }

        bool prevTbl = showPhysicalWorktable;
        showPhysicalWorktable = GUILayout.Toggle(showPhysicalWorktable, " Bàn thao tác (Collider)", AntigravityUITheme.StyleToggle);
        if (showPhysicalWorktable != prevTbl)
        {
            if (showPhysicalWorktable)
            {
                worktableObj = TrajectoryEvaluationEngine.EnsurePhysicalWorktable(null, plyLoader != null ? plyLoader.estimatedTableY : 0f);
            }
            else if (worktableObj != null)
            {
                worktableObj.SetActive(false);
            }
        }
        GUILayout.EndVertical();

        GUILayout.Space(6);
        DrawJeansSizeDetectionSummaryBox();

        GUILayout.Space(6);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"4. TỌA ĐỘ 12 MỐC [{selectedSampleName}]", AntigravityUITheme.StyleSectionLabel);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Nhận diện Size", AntigravityUITheme.StylePrimaryButton, GUILayout.Width(110), GUILayout.Height(22)))
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
                GUILayout.Label(lm.label_vi, AntigravityUITheme.StyleLabelCyan, GUILayout.Width(130));
                GUILayout.Label($"X={lm.x_mm:F0}  Y={lm.y_mm:F0}  Z={lm.z_mm:F1} mm", AntigravityUITheme.StyleLabelMuted);
                GUILayout.EndHorizontal();
            }
        }
        else
        {
            GUILayout.Label("Chưa có dữ liệu 12 mốc.", AntigravityUITheme.StyleLabelMuted);
        }
        GUILayout.EndVertical();

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 7: THUẬT TOÁN BÙ ĐẮP CHỖ KHUẤT, VIỀN ĐỎ VÙNG VÁ & CHUYỂN THÀNH 3D MESH
    // =========================================================================
    private void DrawOcclusionCompensationCard(float width, Action onClose = null)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.BeginHorizontal();
        GUILayout.Label("VÁ LỖ 3D & CHUYỂN MESH", AntigravityUITheme.StyleCardHeader);
        GUILayout.FlexibleSpace();
        if (onClose != null && GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(22), GUILayout.Height(18))) onClose();
        GUILayout.EndHorizontal();

        GUILayout.Space(6);
        GUILayout.Label("1. QUÉT VÙNG BỊ CHE", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Quét & Khoanh đỏ", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            showHoleRegionOutlines3D = true;
            DetectOcclusionAndHoles();
        }
        if (GUILayout.Button("Khôi phục gốc", AntigravityUITheme.StyleButton, GUILayout.Width(115), GUILayout.Height(28)))
        {
            detectedHolePointsWorld.Clear();
            detectedHoleCells.Clear();
            holeRegionOutlinesWorld.Clear();
            compensatedSurfacePointsWorld.Clear();
            gunTiltNormalsWorld.Clear();
            compensatedHeightGrid.Clear();
            compensatedNormalGrid.Clear();
            detectedHoleCount = 0;
            detectedHoleRegionCount = 0;
            detectedOcclusionAreaCm2 = 0f;
            avgCreaseDepthMm = 0f;
            if (inpaintedMeshObject != null)
            {
                if (Application.isPlaying) Destroy(inpaintedMeshObject);
                else DestroyImmediate(inpaintedMeshObject);
                inpaintedMeshObject = null;
            }
            PLYDirectLoader plReset = FindAnyObjectByType<PLYDirectLoader>();
            if (plReset != null) plReset.ResetInpaintedPointsToRaw();
            occlusionStatusMsg = "Đã khôi phục bề mặt gốc.";
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(2);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Vùng: {detectedHoleRegionCount}", AntigravityUITheme.StyleLabelRed);
        GUILayout.Label($"Ô khuyết: {detectedHoleCount}", AntigravityUITheme.StyleLabelCyan);
        GUILayout.Label($"DT: {detectedOcclusionAreaCm2:F1} cm²", AntigravityUITheme.StyleLabelCyan);
        GUILayout.EndHorizontal();

        GUILayout.Label(occlusionStatusMsg, AntigravityUITheme.StyleLabelGreen);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("2. PHƯƠNG PHÁP VÁ", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        string[] algos = {
            "1. K-NN MLS & PCA",
            "2. Thin Plate Spline (TPS RBF)",
            "3. Poisson & Laplacian PDE",
            "4. Bù góc súng & nâng Z",
            "5. Đối xứng 2 ống quần"
        };
        for (int i = 0; i < algos.Length; i++)
        {
            bool isSel = (selectedOcclusionAlgorithm == i);
            GUIStyle st = isSel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            if (GUILayout.Button(algos[i], st, GUILayout.Height(25)))
            {
                selectedOcclusionAlgorithm = i;
                showHoleRegionOutlines3D = true;
                if (detectedHoleCells == null || detectedHoleCells.Count == 0 || holeRegionOutlinesWorld.Count == 0)
                {
                    DetectOcclusionAndHoles();
                }
                if (detectedHoleCells != null && detectedHoleCells.Count > 0)
                {
                    RunOcclusionCompensation();
                }
            }
        }
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("3. THÔNG SỐ", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Bán kính quét: {occlusionSearchRadiusMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(175));
        occlusionSearchRadiusMm = GUILayout.HorizontalSlider(occlusionSearchRadiusMm, 12f, 80f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Láng giềng K-NN: {occlusionKNeighbors}", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(175));
        occlusionKNeighbors = Mathf.RoundToInt(GUILayout.HorizontalSlider(occlusionKNeighbors, 8f, 48f));
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Góc nghiêng tối đa: {maxGunTiltAngleDeg:F0}°", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(175));
        maxGunTiltAngleDeg = GUILayout.HorizontalSlider(maxGunTiltAngleDeg, 10f, 45f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Nâng Z an toàn: +{occlusionZMarginMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(175));
        occlusionZMarginMm = GUILayout.HorizontalSlider(occlusionZMarginMm, 5f, 30f);
        GUILayout.EndHorizontal();
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("4. HIỂN THỊ", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        showHoleRegionOutlines3D = GUILayout.Toggle(showHoleRegionOutlines3D, " Viền đỏ bao quanh vùng cần vá", AntigravityUITheme.StyleToggle);

        bool prevSolid = renderInpaintedAsSolidMesh;
        renderInpaintedAsSolidMesh = GUILayout.Toggle(renderInpaintedAsSolidMesh, " Hiển thị lớp vá dạng 3D Mesh đặc", AntigravityUITheme.StyleToggle);
        if (renderInpaintedAsSolidMesh != prevSolid && compensatedSurfacePointsWorld.Count > 0)
        {
            PLYDirectLoader pl = FindAnyObjectByType<PLYDirectLoader>();
            BuildAndDisplayInpaintedMesh(pl);
        }

        bool prevShowInpainted = showInpaintedSurface3D;
        showInpaintedSurface3D = GUILayout.Toggle(showInpaintedSurface3D, " Hiện lớp vá 3D", AntigravityUITheme.StyleToggle);
        if (showInpaintedSurface3D != prevShowInpainted && inpaintedMeshObject != null)
        {
            var mr = inpaintedMeshObject.GetComponent<MeshRenderer>();
            if (mr != null) mr.enabled = showInpaintedSurface3D;
        }

        bool prevHighlight = highlightInpaintedMesh;
        highlightInpaintedMesh = GUILayout.Toggle(highlightInpaintedMesh, " Tô màu Cyan vùng vá", AntigravityUITheme.StyleToggle);
        if (highlightInpaintedMesh != prevHighlight && compensatedSurfacePointsWorld.Count > 0)
        {
            PLYDirectLoader pl = FindAnyObjectByType<PLYDirectLoader>();
            BuildAndDisplayInpaintedMesh(pl);
        }

        showGunTiltVectors3D = GUILayout.Toggle(showGunTiltVectors3D, " Hiện tia hướng súng", AntigravityUITheme.StyleToggle);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("5. THỰC THI & XUẤT MESH", AntigravityUITheme.StyleSectionLabel);
        if (GUILayout.Button("Vá kín 100% vùng bị che", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(30)))
        {
            showHoleRegionOutlines3D = true;
            if (detectedHoleCells.Count == 0) DetectOcclusionAndHoles();
            RunOcclusionCompensation();
        }
        GUILayout.Space(4);
        if (GUILayout.Button("Chuyển thành 3D Mesh hoàn chỉnh", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(32)))
        {
            ConvertRepairedWorkpieceToFull3DMesh();
        }
        GUILayout.Space(4);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Xuất Mesh (.OBJ)", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            ExportRepairedMeshToObj();
        }
        if (GUILayout.Button("Áp vào quỹ đạo", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            ApplyOcclusionCompensationToTrajectory();
        }
        GUILayout.EndHorizontal();

        GUILayout.EndVertical();
    }

    public void DetectOcclusionAndHoles()
    {
        detectedHolePointsWorld.Clear();
        detectedHoleCells.Clear();
        holeRegionOutlinesWorld.Clear();
        detectedHoleRegionCount = 0;

        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (plyLoader == null)
        {
            occlusionStatusMsg = "Không tìm thấy PLYDirectLoader.";
            return;
        }

        plyLoader.GetCellWorldSize(out float cellW, out float cellD);
        if (cellW <= 1e-6f || cellD <= 1e-6f)
        {
            plyLoader.RebuildSurfaceElevationGrid();
            plyLoader.GetCellWorldSize(out cellW, out cellD);
        }
        if (cellW <= 1e-6f || cellD <= 1e-6f)
        {
            occlusionStatusMsg = "Chưa nạp file PLY.";
            return;
        }

        // Xây dựng đa giác giải phẫu dự phòng đúng thứ tự đỉnh (không tự cắt ở ống trái!)
        List<Vector3> anatomicalPoly = null;
        if (landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
        {
            anatomicalPoly = new List<Vector3>
            {
                landmarksWorldPos[0],  // Eo trái
                landmarksWorldPos[1],  // Cạp giữa
                landmarksWorldPos[2],  // Eo phải
                landmarksWorldPos[5],  // Đùi phải
                landmarksWorldPos[7],  // Gối phải
                landmarksWorldPos[11], // Gấu ngoài phải
                landmarksWorldPos[10], // Gấu trong phải
                landmarksWorldPos[3],  // Đáy đũng
                landmarksWorldPos[9],  // Gấu trong trái
                landmarksWorldPos[8],  // Gấu ngoài trái
                landmarksWorldPos[6],  // Gối trái
                landmarksWorldPos[4]   // Đùi trái
            };
        }

        List<Vector3> activePoly = (garmentContourWorld != null && garmentContourWorld.Count > 5)
            ? garmentContourWorld
            : anatomicalPoly;

        if (activePoly == null || activePoly.Count < 3)
        {
            occlusionStatusMsg = "Chưa có đường bao quần hoặc 12 mốc.";
            return;
        }

        // SỬA LỖI QUÉT THIẾU VÙNG: Tính hộp bao (Bounding Box) từ HỢP của garmentContourWorld VÀ landmarksWorldPos
        // để quét trọn vẹn 100% thân quần (không bị cắt mất phần hông/mép gấu ngoài 12 mốc!)
        float minX = float.MaxValue, maxX = float.MinValue;
        float minZ = float.MaxValue, maxZ = float.MinValue;
        for (int i = 0; i < activePoly.Count; i++)
        {
            minX = Mathf.Min(minX, activePoly[i].x);
            maxX = Mathf.Max(maxX, activePoly[i].x);
            minZ = Mathf.Min(minZ, activePoly[i].z);
            maxZ = Mathf.Max(maxZ, activePoly[i].z);
        }
        if (landmarksWorldPos != null)
        {
            for (int i = 0; i < landmarksWorldPos.Count; i++)
            {
                minX = Mathf.Min(minX, landmarksWorldPos[i].x);
                maxX = Mathf.Max(maxX, landmarksWorldPos[i].x);
                minZ = Mathf.Min(minZ, landmarksWorldPos[i].z);
                maxZ = Mathf.Max(maxZ, landmarksWorldPos[i].z);
            }
        }
        minX -= cellW * 2f; maxX += cellW * 2f;
        minZ -= cellD * 2f; maxZ += cellD * 2f;

        Vector2Int cMin = plyLoader.WorldToGridCell(minX, minZ);
        Vector2Int cMax = plyLoader.WorldToGridCell(maxX, maxZ);
        int gxMin = Mathf.Clamp(Mathf.Min(cMin.x, cMax.x), 0, PLYDirectLoader.GRID_RES - 1);
        int gxMax = Mathf.Clamp(Mathf.Max(cMin.x, cMax.x), 0, PLYDirectLoader.GRID_RES - 1);
        int gzMin = Mathf.Clamp(Mathf.Min(cMin.y, cMax.y), 0, PLYDirectLoader.GRID_RES - 1);
        int gzMax = Mathf.Clamp(Mathf.Max(cMin.y, cMax.y), 0, PLYDirectLoader.GRID_RES - 1);

        // Pass A: Xác định từng ô lưới nằm trong thân quần & tính cao độ vải trung vị thực tế
        bool[,] insideMask = new bool[PLYDirectLoader.GRID_RES, PLYDirectLoader.GRID_RES];
        List<float> clothHeights = new List<float>(4096);

        for (int gx = gxMin; gx <= gxMax; gx++)
        {
            for (int gz = gzMin; gz <= gzMax; gz++)
            {
                Vector2 wxz = plyLoader.GridCellToWorldXZ(gx, gz);
                if (!IsPointInPolygonXZ(wxz.x, wxz.y, activePoly)) continue;
                insideMask[gx, gz] = true;
                if (plyLoader.IsRawCellValid(gx, gz))
                {
                    clothHeights.Add(plyLoader.GetRawCellY(gx, gz));
                }
            }
        }

        float tableY = plyLoader.estimatedTableY;
        if (clothHeights.Count > 10)
        {
            clothHeights.Sort();
            lastGarmentMedianY = clothHeights[clothHeights.Count / 2];
        }
        else if (landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
        {
            float sumLY = 0f;
            for (int i = 0; i < 12; i++) sumLY += landmarksWorldPos[i].y;
            lastGarmentMedianY = sumLY / 12f;
        }
        else
        {
            lastGarmentMedianY = tableY + 0.05f;
        }

        bool garmentAboveTable = (lastGarmentMedianY - tableY) > 0.010f;
        lastGarmentMinClothY = garmentAboveTable
            ? (tableY + Mathf.Clamp((lastGarmentMedianY - tableY) * 0.22f, 0.004f, 0.018f))
            : (tableY - 0.01f);

        // Pass B: CHỈ tìm các vùng thực sự CÓ LỖ trên bề mặt quần (lưới mịn 480x480 ~3mm):
        //   1) Ô mất điểm cảm biến thật (!IsRawCellValid)
        //   2) Ô rơi điểm xuống mặt bàn bên trong thân quần (rawY < lastGarmentMinClothY)
        //   3) Ô hụt độ sâu đột ngột (>= 4.2cm so với vải kề bên) gây rách/rớt tam giác trong 3D Mesh
        // KHÔNG gắn nhầm các nếp gấp vải nguyên vẹn (không bị thủng lỗ) là lỗ vá!
        bool[,] holeMask = new bool[PLYDirectLoader.GRID_RES, PLYDirectLoader.GRID_RES];
        float sumDepthDiffMm = 0f;
        int depthSamples = 0;
        int rawHoleCells = 0;
        int dropoutCells = 0;
        int cliffCells = 0;

        for (int gx = gxMin; gx <= gxMax; gx++)
        {
            for (int gz = gzMin; gz <= gzMax; gz++)
            {
                if (!insideMask[gx, gz]) continue;

                Vector2 wxz = plyLoader.GridCellToWorldXZ(gx, gz);
                bool isValid = plyLoader.IsRawCellValid(gx, gz);

                if (!isValid)
                {
                    holeMask[gx, gz] = true;
                    detectedHoleCells.Add(new Vector2Int(gx, gz));
                    detectedHolePointsWorld.Add(new Vector3(wxz.x, lastGarmentMedianY, wxz.y));
                    rawHoleCells++;
                }
                else
                {
                    float rawY = plyLoader.GetRawCellY(gx, gz);
                    if (garmentAboveTable && rawY < lastGarmentMinClothY)
                    {
                        holeMask[gx, gz] = true;
                        detectedHoleCells.Add(new Vector2Int(gx, gz));
                        detectedHolePointsWorld.Add(new Vector3(wxz.x, rawY, wxz.y));
                        sumDepthDiffMm += Mathf.Max(0f, (lastGarmentMedianY - rawY) * 1000f);
                        depthSamples++;
                        dropoutCells++;
                    }
                    else
                    {
                        // Kiểm tra ô tụt độ cao >= 4.2cm so với ô vải hợp lệ kề bên (sẽ bị cắt rách tam giác trong Mesh3D)
                        bool isBrokenTriangleLow = false;
                        const float cliffThreshM = 0.042f;
                        if (gx > 0 && insideMask[gx - 1, gz] && plyLoader.IsRawCellValid(gx - 1, gz) &&
                            (plyLoader.GetRawCellY(gx - 1, gz) - rawY) >= cliffThreshM) isBrokenTriangleLow = true;
                        else if (gx < PLYDirectLoader.GRID_RES - 1 && insideMask[gx + 1, gz] && plyLoader.IsRawCellValid(gx + 1, gz) &&
                            (plyLoader.GetRawCellY(gx + 1, gz) - rawY) >= cliffThreshM) isBrokenTriangleLow = true;
                        else if (gz > 0 && insideMask[gx, gz - 1] && plyLoader.IsRawCellValid(gx, gz - 1) &&
                            (plyLoader.GetRawCellY(gx, gz - 1) - rawY) >= cliffThreshM) isBrokenTriangleLow = true;
                        else if (gz < PLYDirectLoader.GRID_RES - 1 && insideMask[gx, gz + 1] && plyLoader.IsRawCellValid(gx, gz + 1) &&
                            (plyLoader.GetRawCellY(gx, gz + 1) - rawY) >= cliffThreshM) isBrokenTriangleLow = true;

                        if (isBrokenTriangleLow)
                        {
                            holeMask[gx, gz] = true;
                            detectedHoleCells.Add(new Vector2Int(gx, gz));
                            detectedHolePointsWorld.Add(new Vector3(wxz.x, rawY, wxz.y));
                            sumDepthDiffMm += Mathf.Max(0f, (lastGarmentMedianY - rawY) * 1000f);
                            depthSamples++;
                            cliffCells++;
                        }
                    }
                }
            }
        }

        // Pass C: Trích xuất các ĐƯỜNG VIỀN KÍN MÀU ĐỎ bao quanh các vùng thực sự có lỗ vá
        ExtractHoleRegionOutlines(plyLoader, holeMask, gxMin, gxMax, gzMin, gzMax, cellW, cellD);

        detectedHoleCount = detectedHoleCells.Count;
        detectedOcclusionAreaCm2 = detectedHoleCount * (cellW * cellD * 10000f);
        avgCreaseDepthMm = depthSamples > 0 ? (sumDepthDiffMm / depthSamples) : 0f;
        occlusionStatusMsg = $"Tìm thấy {detectedHoleRegionCount} vùng lỗ ({rawHoleCells + dropoutCells + cliffCells} ô, {detectedOcclusionAreaCm2:F1} cm²).";
    }

    /// <summary>
    /// Trích xuất chính xác các vòng đường viền kín (Closed Contour Loops) bao quanh các vùng thực sự có lỗ vá
    /// (lọc bỏ hạt nhiễu 1-2 ô khỏi đường viền đỏ cho gọn mắt, nhưng vẫn vá kín 100% mọi ô khuyết!).
    /// </summary>
    private void ExtractHoleRegionOutlines(
        PLYDirectLoader plyLoader,
        bool[,] holeMask,
        int gxMin, int gxMax, int gzMin, int gzMax,
        float cellW, float cellD)
    {
        holeRegionOutlinesWorld.Clear();
        detectedHoleRegionCount = 0;

        int res = PLYDirectLoader.GRID_RES;
        bool[,] visited = new bool[res, res];
        bool[,] outlineMask = new bool[res, res];
        Queue<Vector2Int> bfs = new Queue<Vector2Int>();
        List<Vector2Int> compCells = new List<Vector2Int>(256);

        // 1. Gom cụm 8-connected và chỉ đưa các vùng lỗ thật (>= 3 ô, ~25mm²) vào đường viền đỏ
        for (int x = gxMin; x <= gxMax; x++)
        {
            for (int z = gzMin; z <= gzMax; z++)
            {
                if (!holeMask[x, z] || visited[x, z]) continue;
                compCells.Clear();
                visited[x, z] = true;
                bfs.Enqueue(new Vector2Int(x, z));
                while (bfs.Count > 0)
                {
                    Vector2Int cur = bfs.Dequeue();
                    compCells.Add(cur);
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            if (dx == 0 && dz == 0) continue;
                            int nx = cur.x + dx;
                            int nz = cur.y + dz;
                            if (nx >= gxMin && nx <= gxMax && nz >= gzMin && nz <= gzMax &&
                                holeMask[nx, nz] && !visited[nx, nz])
                            {
                                visited[nx, nz] = true;
                                bfs.Enqueue(new Vector2Int(nx, nz));
                            }
                        }
                    }
                }

                if (compCells.Count >= 3)
                {
                    detectedHoleRegionCount++;
                    for (int k = 0; k < compCells.Count; k++)
                    {
                        outlineMask[compCells[k].x, compCells[k].y] = true;
                    }
                }
            }
        }

        // 2. Sinh các cạnh biên có hướng (CCW, vùng lỗ nằm bên trái cạnh) trên lưới góc [0..GRID_RES]
        Dictionary<Vector2Int, List<Vector2Int>> adj = new Dictionary<Vector2Int, List<Vector2Int>>();
        void AddEdge(Vector2Int a, Vector2Int b)
        {
            if (!adj.TryGetValue(a, out List<Vector2Int> list))
            {
                list = new List<Vector2Int>(2);
                adj[a] = list;
            }
            list.Add(b);
        }

        for (int x = gxMin; x <= gxMax; x++)
        {
            for (int z = gzMin; z <= gzMax; z++)
            {
                if (!outlineMask[x, z]) continue;
                if (z == 0 || !outlineMask[x, z - 1])
                    AddEdge(new Vector2Int(x, z), new Vector2Int(x + 1, z));
                if (x == res - 1 || !outlineMask[x + 1, z])
                    AddEdge(new Vector2Int(x + 1, z), new Vector2Int(x + 1, z + 1));
                if (z == res - 1 || !outlineMask[x, z + 1])
                    AddEdge(new Vector2Int(x + 1, z + 1), new Vector2Int(x, z + 1));
                if (x == 0 || !outlineMask[x - 1, z])
                    AddEdge(new Vector2Int(x, z + 1), new Vector2Int(x, z));
            }
        }

        Vector2 originXZ = plyLoader.GridCellToWorldXZ(0, 0);

        // 3. Nối các cạnh thành từng vòng khép kín, rút gọn cạnh thẳng hàng và làm mượt nhẹ Chaikin
        while (adj.Count > 0)
        {
            Vector2Int startNode = default;
            Vector2Int nextNode = default;
            foreach (var kv in adj)
            {
                if (kv.Value.Count > 0)
                {
                    startNode = kv.Key;
                    nextNode = kv.Value[0];
                    kv.Value.RemoveAt(0);
                    if (kv.Value.Count == 0) adj.Remove(startNode);
                    break;
                }
            }

            List<Vector2Int> cornerLoop = new List<Vector2Int> { startNode };
            Vector2Int prev = startNode;
            Vector2Int curr = nextNode;
            int guard = 0;

            while (curr != startNode && guard++ < 4096)
            {
                cornerLoop.Add(curr);
                if (!adj.TryGetValue(curr, out List<Vector2Int> nextList) || nextList.Count == 0)
                    break;

                int pickIdx = 0;
                if (nextList.Count > 1)
                {
                    // Ưu tiên rẽ trái 90° (-dIn.y, dIn.x) để tách gọn các vùng chạm góc chéo
                    Vector2Int dIn = curr - prev;
                    Vector2Int leftDir = new Vector2Int(-dIn.y, dIn.x);
                    for (int k = 0; k < nextList.Count; k++)
                    {
                        if (nextList[k] - curr == leftDir)
                        {
                            pickIdx = k;
                            break;
                        }
                    }
                }

                Vector2Int nxt = nextList[pickIdx];
                nextList.RemoveAt(pickIdx);
                if (nextList.Count == 0) adj.Remove(curr);
                prev = curr;
                curr = nxt;
            }

            if (cornerLoop.Count < 3) continue;

            // Rút gọn các đỉnh thẳng hàng trên cùng cạnh lưới
            List<Vector2> simplifiedXZ = new List<Vector2>(cornerLoop.Count);
            int n = cornerLoop.Count;
            for (int i = 0; i < n; i++)
            {
                Vector2Int pPrev = cornerLoop[(i - 1 + n) % n];
                Vector2Int pCur = cornerLoop[i];
                Vector2Int pNext = cornerLoop[(i + 1) % n];
                Vector2Int d1 = pCur - pPrev;
                Vector2Int d2 = pNext - pCur;
                if (d1.x * d2.y - d1.y * d2.x == 0) continue; // Thẳng hàng -> bỏ đỉnh giữa

                float wx = originXZ.x + (pCur.x - 0.5f) * cellW;
                float wz = originXZ.y + (pCur.y - 0.5f) * cellD;
                simplifiedXZ.Add(new Vector2(wx, wz));
            }
            if (simplifiedXZ.Count < 3) continue;

            // Làm mượt 1 nhịp Chaikin để đường viền đỏ ôm mềm mại quanh vùng cần vá (bớt bậc thang)
            List<Vector2> smoothXZ = simplifiedXZ;
            if (simplifiedXZ.Count >= 4 && simplifiedXZ.Count <= 220)
            {
                smoothXZ = new List<Vector2>(simplifiedXZ.Count * 2);
                int m = simplifiedXZ.Count;
                for (int i = 0; i < m; i++)
                {
                    Vector2 a = simplifiedXZ[i];
                    Vector2 b = simplifiedXZ[(i + 1) % m];
                    smoothXZ.Add(Vector2.Lerp(a, b, 0.25f));
                    smoothXZ.Add(Vector2.Lerp(a, b, 0.75f));
                }
            }

            // Nâng đường viền đỏ lên bề mặt quần (+4.5mm) để luôn nhìn rõ không bị vải che
            List<Vector3> worldLoop = new List<Vector3>(smoothXZ.Count);
            for (int i = 0; i < smoothXZ.Count; i++)
            {
                float wx = smoothXZ[i].x;
                float wz = smoothXZ[i].y;
                float wy = lastGarmentMedianY;
                if (plyLoader.TrySampleRawSurfaceAt(wx, wz, out float rY, out _) && rY >= lastGarmentMinClothY)
                {
                    wy = rY;
                }
                else if (plyLoader.SampleSurfaceWorldHeightAndNormal(wx, wz, out float sY, out _) && sY >= lastGarmentMinClothY)
                {
                    wy = sY;
                }
                worldLoop.Add(new Vector3(wx, wy + 0.0045f, wz));
            }

            holeRegionOutlinesWorld.Add(worldLoop);
        }
    }

    public void RunOcclusionCompensation()
    {
        if (detectedHoleCells == null || detectedHoleCells.Count == 0)
        {
            DetectOcclusionAndHoles();
        }
        if (detectedHoleCells.Count == 0)
        {
            occlusionStatusMsg = "Không phát hiện vùng bị che.";
            return;
        }

        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (plyLoader == null)
        {
            occlusionStatusMsg = "Không tìm thấy PLYDirectLoader.";
            return;
        }

        compensatedSurfacePointsWorld.Clear();
        gunTiltNormalsWorld.Clear();
        compensatedHeightGrid.Clear();
        compensatedNormalGrid.Clear();

        string[] algoNames = {
            "K-NN MLS",
            "TPS RBF",
            "Laplacian PDE",
            "Bù góc súng",
            "Đối xứng 2 ống"
        };

        int res = PLYDirectLoader.GRID_RES;
        plyLoader.GetCellWorldSize(out float cellW, out float cellD);
        HashSet<Vector2Int> holeCellSet = new HashSet<Vector2Int>(detectedHoleCells);

        // Khởi tạo lưới cao độ làm việc từ toàn bộ các ô vải cảm biến gốc hợp lệ (cố định làm điều kiện biên)
        float[,] filledGrid = new float[res, res];
        bool[,] filledValid = new bool[res, res];
        for (int x = 0; x < res; x++)
        {
            for (int z = 0; z < res; z++)
            {
                if (plyLoader.IsRawCellValid(x, z) && !holeCellSet.Contains(new Vector2Int(x, z)))
                {
                    float ry = plyLoader.GetRawCellY(x, z);
                    if (ry >= lastGarmentMinClothY)
                    {
                        filledGrid[x, z] = ry;
                        filledValid[x, z] = true;
                    }
                }
            }
        }

        // Giải trước PDE 100 vòng SOR nếu dùng Thuật toán 2 (Laplacian PDE)
        Dictionary<Vector2Int, float> laplacianPdeField = null;
        if (selectedOcclusionAlgorithm == 2)
        {
            laplacianPdeField = plyLoader.SolveLaplacianPdeOnHoleCells(
                holeCellSet,
                iterations: 100,
                relaxation: Mathf.Clamp(laplacianSmoothingFactor + 0.90f, 0.80f, 1.55f),
                minClothY: lastGarmentMinClothY);
        }

        // Trục đối xứng 2D thực của chiếc quần (đi qua waist_center -> crotch, bất biến góc xoay!)
        Vector2 symOriginXZ = Vector2.zero;
        Vector2 symAxisXZ = new Vector2(0f, 1f);
        if (landmarksWorldPos != null && landmarksWorldPos.Count >= 4)
        {
            Vector2 wC = new Vector2(landmarksWorldPos[1].x, landmarksWorldPos[1].z);
            Vector2 cr = new Vector2(landmarksWorldPos[3].x, landmarksWorldPos[3].z);
            symOriginXZ = cr;
            Vector2 dir = cr - wC;
            if (dir.sqrMagnitude > 1e-5f) symAxisXZ = dir.normalized;
        }

        // Bước 1: Tính cao độ vá ban đầu cho 100% TẤT CẢ các ô bị che theo thuật toán đã chọn
        Vector3[] perCellHintNormal = new Vector3[detectedHoleCells.Count];
        for (int i = 0; i < detectedHoleCells.Count; i++)
        {
            Vector2Int cell = detectedHoleCells[i];
            Vector2 wxz = plyLoader.GridCellToWorldXZ(cell.x, cell.y);
            float compY = lastGarmentMedianY;
            Vector3 compNorm = Vector3.up;

            switch (selectedOcclusionAlgorithm)
            {
                case 0:
                    compY = CalculateMlsTangentPlaneHeight(plyLoader, wxz.x, wxz.y, out compNorm);
                    break;

                case 1:
                    compY = CalculateTpsHeightAtPoint(plyLoader, wxz.x, wxz.y, out compNorm);
                    break;

                case 2:
                    if (laplacianPdeField != null && laplacianPdeField.TryGetValue(cell, out float pdeY))
                    {
                        compY = pdeY;
                    }
                    else
                    {
                        compY = CalculateLaplacianInpaintedHeight(plyLoader, wxz.x, wxz.y, out compNorm);
                    }
                    break;

                case 3:
                    compY = CalculateMlsTangentPlaneHeight(plyLoader, wxz.x, wxz.y, out compNorm);
                    compY += occlusionZMarginMm * 0.001f;
                    break;

                case 4:
                    Vector2 rel = wxz - symOriginXZ;
                    float proj = Vector2.Dot(rel, symAxisXZ);
                    Vector2 foot = symOriginXZ + symAxisXZ * proj;
                    Vector2 mirrorXZ = 2f * foot - wxz;

                    if (plyLoader.TrySampleRawSurfaceAt(mirrorXZ.x, mirrorXZ.y, out float mY, out _) && mY >= lastGarmentMinClothY)
                    {
                        // Hòa trộn độ cao đối xứng với độ cao cục bộ để triệt tiêu bậc thang tại viền lỗ
                        float localRefY = CalculateMlsTangentPlaneHeight(plyLoader, wxz.x, wxz.y, out _);
                        compY = Mathf.Lerp(mY, localRefY, 0.30f);
                        if (plyLoader.SampleSurfaceWorldHeightAndNormal(mirrorXZ.x, mirrorXZ.y, out _, out Vector3 mNorm))
                        {
                            Vector2 nXZ = new Vector2(mNorm.x, mNorm.z);
                            Vector2 nProj = symAxisXZ * Vector2.Dot(nXZ, symAxisXZ);
                            Vector2 nMirror = 2f * nProj - nXZ;
                            compNorm = new Vector3(nMirror.x, mNorm.y, nMirror.y).normalized;
                        }
                    }
                    else
                    {
                        compY = CalculateTpsHeightAtPoint(plyLoader, wxz.x, wxz.y, out compNorm);
                    }
                    break;
            }

            // Chống rớt độ cao xuống mặt bàn hoặc vọt lố quá biên vải
            float safeMinY = lastGarmentMinClothY;
            float safeMaxY = lastGarmentMedianY + 0.085f + (selectedOcclusionAlgorithm == 3 ? occlusionZMarginMm * 0.001f : 0f);
            compY = Mathf.Clamp(compY, safeMinY, safeMaxY);

            filledGrid[cell.x, cell.y] = compY;
            filledValid[cell.x, cell.y] = true;
            perCellHintNormal[i] = compNorm;
        }

        // Bước 2: Làm mượt biên hài (Harmonic Boundary Stitching) giữ cố định vải gốc — đảm bảo 100% kín khít không nứt mép!
        int stitchPasses = (selectedOcclusionAlgorithm == 2) ? 2 : (selectedOcclusionAlgorithm == 3 ? 4 : 8);
        float stitchAlpha = (selectedOcclusionAlgorithm == 3) ? 0.30f : 0.50f;
        for (int pass = 0; pass < stitchPasses; pass++)
        {
            for (int i = 0; i < detectedHoleCells.Count; i++)
            {
                Vector2Int c = detectedHoleCells[i];
                int x = c.x, z = c.y;
                float sum = 0f;
                int cnt = 0;
                if (x > 0 && filledValid[x - 1, z]) { sum += filledGrid[x - 1, z]; cnt++; }
                if (x < res - 1 && filledValid[x + 1, z]) { sum += filledGrid[x + 1, z]; cnt++; }
                if (z > 0 && filledValid[x, z - 1]) { sum += filledGrid[x, z - 1]; cnt++; }
                if (z < res - 1 && filledValid[x, z + 1]) { sum += filledGrid[x, z + 1]; cnt++; }
                if (cnt > 0)
                {
                    filledGrid[x, z] = Mathf.Lerp(filledGrid[x, z], sum / cnt, stitchAlpha);
                }
            }
        }

        // Bước 3: Tính pháp tuyến chính xác từ bề mặt đã điền kín, bơm 100% ô vào PLYDirectLoader
        for (int i = 0; i < detectedHoleCells.Count; i++)
        {
            Vector2Int cell = detectedHoleCells[i];
            int gx = cell.x, gz = cell.y;
            Vector2 wxz = plyLoader.GridCellToWorldXZ(gx, gz);
            float compY = filledGrid[gx, gz];

            float yL = (gx > 0 && filledValid[gx - 1, gz]) ? filledGrid[gx - 1, gz] : compY;
            float yR = (gx < res - 1 && filledValid[gx + 1, gz]) ? filledGrid[gx + 1, gz] : compY;
            float yD = (gz > 0 && filledValid[gx, gz - 1]) ? filledGrid[gx, gz - 1] : compY;
            float yU = (gz < res - 1 && filledValid[gx, gz + 1]) ? filledGrid[gx, gz + 1] : compY;

            Vector3 tx = new Vector3(2f * cellW, yR - yL, 0f).normalized;
            Vector3 tz = new Vector3(0f, yU - yD, 2f * cellD).normalized;
            Vector3 compNorm = Vector3.Cross(tz, tx).normalized;
            if (compNorm.y < 0f) compNorm = -compNorm;
            if (compNorm.sqrMagnitude < 1e-5f) compNorm = Vector3.up;

            if (selectedOcclusionAlgorithm == 3)
            {
                Vector3 tiltAxis = Vector3.Cross(compNorm, Vector3.up);
                if (tiltAxis.sqrMagnitude > 1e-4f)
                {
                    tiltAxis.Normalize();
                    float extraTilt = Mathf.Min(maxGunTiltAngleDeg, Vector3.Angle(compNorm, Vector3.up) * 0.5f + 10f);
                    compNorm = (Quaternion.AngleAxis(extraTilt, tiltAxis) * Vector3.up).normalized;
                }
            }
            else if (selectedOcclusionAlgorithm == 4 && perCellHintNormal[i].sqrMagnitude > 0.5f)
            {
                compNorm = Vector3.Slerp(compNorm, perCellHintNormal[i].normalized, 0.45f).normalized;
            }
            if (compNorm.y < 0f) compNorm = -compNorm;

            Vector3 compPos = new Vector3(wxz.x, compY, wxz.y);
            compensatedSurfacePointsWorld.Add(compPos);
            gunTiltNormalsWorld.Add(compNorm);

            // Lưu lưới độ phân giải mịn ~3mm (key * 333)
            Vector2Int key = new Vector2Int(Mathf.RoundToInt(wxz.x * 333f), Mathf.RoundToInt(wxz.y * 333f));
            compensatedHeightGrid[key] = compY;
            compensatedNormalGrid[key] = compNorm;

            // Bơm trực tiếp từng ô đã vá kín vào PLYDirectLoader kèm màu RGB tiệp vải gốc
            Color32 clothCol = plyLoader.SampleSurroundingRawColor(wxz.x, wxz.y, searchCells: 10);
            plyLoader.InjectSurfaceCell(gx, gz, compY, clothCol);
        }

        // Bước 4: Cập nhật cao độ đường viền đỏ ôm sát đúng bề mặt mới vá (+4.5mm)
        for (int r = 0; r < holeRegionOutlinesWorld.Count; r++)
        {
            List<Vector3> loop = holeRegionOutlinesWorld[r];
            for (int k = 0; k < loop.Count; k++)
            {
                Vector3 p = loop[k];
                Vector2Int c = plyLoader.WorldToGridCell(p.x, p.z);
                if (filledValid[c.x, c.y])
                {
                    loop[k] = new Vector3(p.x, filledGrid[c.x, c.y] + 0.0045f, p.z);
                }
            }
        }

        // Bước 5: Xây dựng Mesh vá đặc (Solid Triangulated Patch Mesh) & cập nhật WorkpieceSurfaceMesh nếu đang ở chế độ Mesh3D
        BuildAndDisplayInpaintedMesh(plyLoader);

        occlusionStatusMsg = $"Đã vá kín {detectedHoleRegionCount} vùng ({compensatedSurfacePointsWorld.Count} ô) - {algoNames[Mathf.Clamp(selectedOcclusionAlgorithm, 0, 4)]}.";
    }

    public void BuildAndDisplayInpaintedMesh(PLYDirectLoader plyLoader)
    {
        if (compensatedSurfacePointsWorld == null || compensatedSurfacePointsWorld.Count == 0) return;

        if (inpaintedMeshObject == null)
        {
            Transform parentT = (plyLoader != null) ? plyLoader.transform : this.transform;
            inpaintedMeshObject = new GameObject("InpaintedJeansSurface_Mesh");
            inpaintedMeshObject.transform.SetParent(parentT, false);
            inpaintedMeshObject.transform.localPosition = Vector3.zero;
            inpaintedMeshObject.transform.localRotation = Quaternion.identity;
            inpaintedMeshObject.transform.localScale = Vector3.one;

            inpaintedMeshObject.AddComponent<MeshFilter>();
            var mr = inpaintedMeshObject.AddComponent<MeshRenderer>();

            Shader pcShader = Shader.Find("Custom/PointCloudVertexColor") ?? Shader.Find("Universal Render Pipeline/Particles/Unlit") ?? Shader.Find("Sprites/Default");
            Material mat = new Material(pcShader);
            mat.name = "Inpainted_SurfaceMesh_Mat";
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

        Color32 cyanHighlight = new Color32(24, 232, 248, 255);
        Mesh mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        if (renderInpaintedAsSolidMesh && plyLoader != null && detectedHoleCells.Count == compensatedSurfacePointsWorld.Count)
        {
            // CHẾ ĐỘ 3D MESH ĐẶC (Solid Triangulated Patch Mesh + 1-ring khâu mép liền kín không khe hở!)
            mesh.name = "Inpainted_SolidPatch_Triangles";
            int res = PLYDirectLoader.GRID_RES;
            int[,] vertIdx = new int[res, res];
            bool[,] isHoleCell = new bool[res, res];
            for (int x = 0; x < res; x++)
                for (int z = 0; z < res; z++)
                    vertIdx[x, z] = -1;

            List<Vector3> localVerts = new List<Vector3>(detectedHoleCells.Count * 2);
            List<Color32> cols = new List<Color32>(detectedHoleCells.Count * 2);
            float zLift = highlightInpaintedMesh ? 0.0012f : 0.0005f;

            // Thêm toàn bộ đỉnh của các ô bị che đã vá
            for (int i = 0; i < detectedHoleCells.Count; i++)
            {
                Vector2Int c = detectedHoleCells[i];
                Vector3 wp = compensatedSurfacePointsWorld[i];
                wp.y += zLift;
                isHoleCell[c.x, c.y] = true;
                vertIdx[c.x, c.y] = localVerts.Count;
                localVerts.Add(inpaintedMeshObject.transform.InverseTransformPoint(wp));
                Color32 pointCol = highlightInpaintedMesh
                    ? cyanHighlight
                    : plyLoader.SampleSurroundingRawColor(wp.x, wp.z, searchCells: 10);
                cols.Add(pointCol);
            }

            // Thêm vành biên 1-ring các ô cảm biến gốc kề quanh lỗ để tam giác vá khâu kín 100% vào mặt quần thật!
            for (int i = 0; i < detectedHoleCells.Count; i++)
            {
                Vector2Int c = detectedHoleCells[i];
                float holeY = compensatedSurfacePointsWorld[i].y;
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        if (dx == 0 && dz == 0) continue;
                        int nx = c.x + dx;
                        int nz = c.y + dz;
                        if (nx < 0 || nx >= res || nz < 0 || nz >= res) continue;
                        if (vertIdx[nx, nz] >= 0) continue;
                        if (!plyLoader.IsRawCellValid(nx, nz)) continue;

                        float ry = plyLoader.GetRawCellY(nx, nz);
                        if (Mathf.Abs(ry - holeY) > 0.085f) continue;

                        Vector2 nXZ = plyLoader.GridCellToWorldXZ(nx, nz);
                        Vector3 wp = new Vector3(nXZ.x, ry + zLift * 0.5f, nXZ.y);
                        vertIdx[nx, nz] = localVerts.Count;
                        localVerts.Add(inpaintedMeshObject.transform.InverseTransformPoint(wp));

                        Color32 rimCol = cyanHighlight;
                        if (!highlightInpaintedMesh)
                        {
                            if (!plyLoader.TrySampleRawSurfaceAt(nXZ.x, nXZ.y, out _, out rimCol))
                                rimCol = plyLoader.SampleSurroundingRawColor(nXZ.x, nXZ.y, searchCells: 6);
                        }
                        cols.Add(rimCol);
                    }
                }
            }

            // Nối tam giác (2 tam giác mỗi ô 2x2 có ít nhất 1 đỉnh thuộc vùng vá)
            List<int> tris = new List<int>(detectedHoleCells.Count * 6);
            for (int x = 0; x < res - 1; x++)
            {
                for (int z = 0; z < res - 1; z++)
                {
                    int v00 = vertIdx[x, z];
                    int v10 = vertIdx[x + 1, z];
                    int v01 = vertIdx[x, z + 1];
                    int v11 = vertIdx[x + 1, z + 1];

                    if (v00 >= 0 && v01 >= 0 && v10 >= 0 &&
                        (isHoleCell[x, z] || isHoleCell[x, z + 1] || isHoleCell[x + 1, z]))
                    {
                        tris.Add(v00); tris.Add(v01); tris.Add(v10);
                    }
                    if (v10 >= 0 && v01 >= 0 && v11 >= 0 &&
                        (isHoleCell[x + 1, z] || isHoleCell[x, z + 1] || isHoleCell[x + 1, z + 1]))
                    {
                        tris.Add(v10); tris.Add(v01); tris.Add(v11);
                    }
                }
            }

            mesh.SetVertices(localVerts);
            mesh.SetColors(cols);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            filter.sharedMesh = mesh;

            // Gắn MeshCollider cho lớp vá 3D để Raycast và va chạm nhận diện chính xác
            MeshCollider mc = inpaintedMeshObject.GetComponent<MeshCollider>();
            if (mc == null) mc = inpaintedMeshObject.AddComponent<MeshCollider>();
            mc.sharedMesh = null;
            mc.sharedMesh = mesh;
        }
        else
        {
            // CHẾ ĐỘ HẠT ĐIỂM POINT CLOUD (Tùy chọn phụ)
            mesh.name = "Inpainted_PointCloud_Points";
            int count = compensatedSurfacePointsWorld.Count;
            List<Vector3> localVerts = new List<Vector3>(count);
            List<Color32> cols = new List<Color32>(count);

            for (int i = 0; i < count; i++)
            {
                Vector3 wp = compensatedSurfacePointsWorld[i];
                localVerts.Add(inpaintedMeshObject.transform.InverseTransformPoint(wp));
                Color32 pointCol = highlightInpaintedMesh
                    ? cyanHighlight
                    : (plyLoader != null ? plyLoader.SampleSurroundingRawColor(wp.x, wp.z) : new Color32(130, 140, 155, 255));
                cols.Add(pointCol);
            }

            int[] indices = new int[count];
            for (int i = 0; i < count; i++) indices[i] = i;

            mesh.SetVertices(localVerts);
            mesh.SetColors(cols);
            mesh.SetIndices(indices, MeshTopology.Points, 0);
            mesh.RecalculateBounds();
            filter.sharedMesh = mesh;
        }

        // Nếu phôi đang hiển thị ở chế độ 3D Mesh -> tự động dựng lại WorkpieceSurfaceMesh liền kín 100%!
        if (plyLoader != null && plyLoader.renderMode == PLYDirectLoader.WorkpieceRenderMode.Mesh3D)
        {
            plyLoader.RefreshSurfaceMeshFromGrid();
        }
    }

    /// <summary>
    /// Chuyển đổi toàn bộ phôi quần (đám mây điểm + 100% các vùng bị che đã vá) thành 1 khối 3D Surface Mesh hoàn chỉnh,
    /// kín lỗ, có pháp tuyến mượt và gắn MeshCollider sẵn sàng cho sinh quỹ đạo & xuất file .OBJ.
    /// </summary>
    public void ConvertRepairedWorkpieceToFull3DMesh()
    {
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (plyLoader == null)
        {
            occlusionStatusMsg = "Không tìm thấy PLYDirectLoader.";
            return;
        }

        // Đảm bảo đã phát hiện & điền kín 100% vùng bị che trước khi dựng Mesh tổng
        if (detectedHoleCells.Count == 0)
        {
            DetectOcclusionAndHoles();
        }
        if (detectedHoleCells.Count > 0 && compensatedSurfacePointsWorld.Count != detectedHoleCells.Count)
        {
            RunOcclusionCompensation();
        }
        else if (detectedHoleCells.Count > 0)
        {
            // Bơm lại toàn bộ ô đã vá vào lưới để bảo đảm đồng bộ tuyệt đối
            for (int i = 0; i < detectedHoleCells.Count; i++)
            {
                Vector2Int cell = detectedHoleCells[i];
                Vector3 wp = compensatedSurfacePointsWorld[i];
                Color32 col = plyLoader.SampleSurroundingRawColor(wp.x, wp.z, searchCells: 10);
                plyLoader.InjectSurfaceCell(cell.x, cell.y, wp.y, col);
            }
        }

        renderInpaintedAsSolidMesh = true;
        plyLoader.SetRenderMode(PLYDirectLoader.WorkpieceRenderMode.Mesh3D);
        GameObject meshObj = plyLoader.ConvertPointCloudToSurfaceMesh(applyPinHoleFill: true);
        plyLoader.SetVisible(true);

        int vCount = 0, tCount = 0;
        if (meshObj != null)
        {
            MeshFilter mf = meshObj.GetComponent<MeshFilter>();
            if (mf != null && mf.sharedMesh != null)
            {
                vCount = mf.sharedMesh.vertexCount;
                tCount = mf.sharedMesh.triangles.Length / 3;
            }
        }

        // Cập nhật lớp patch đồng bộ (nếu highlight cyan bật thì hiện đè nổi bật; nếu tắt thì WorkpieceSurfaceMesh đã chứa sẵn liền khối)
        if (compensatedSurfacePointsWorld.Count > 0)
        {
            BuildAndDisplayInpaintedMesh(plyLoader);
        }

        occlusionStatusMsg = $"Đã tạo 3D Mesh: {vCount:N0} đỉnh, {tCount:N0} tam giác.";
        Debug.Log($"[CR10A 3D Mesh] {occlusionStatusMsg}");
    }

    /// <summary>
    /// Xuất 3D Surface Mesh đã vá kín toàn bộ vùng bị che ra file chuẩn Wavefront .OBJ
    /// </summary>
    public void ExportRepairedMeshToObj()
    {
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (plyLoader == null)
        {
            occlusionStatusMsg = "Không tìm thấy PLYDirectLoader.";
            return;
        }

        // Đảm bảo đã chuyển sang 3D Mesh hoàn chỉnh trước khi xuất
        if (plyLoader.renderMode != PLYDirectLoader.WorkpieceRenderMode.Mesh3D || compensatedSurfacePointsWorld.Count == 0)
        {
            ConvertRepairedWorkpieceToFull3DMesh();
        }

        string sample = !string.IsNullOrEmpty(plyLoader.currentLoadedSampleName)
            ? plyLoader.currentLoadedSampleName
            : (string.IsNullOrEmpty(selectedSampleName) ? "garment" : selectedSampleName);
        string outPath = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "output_drawing", $"{sample}_repaired_mesh.obj"));
        string saved = plyLoader.ExportSurfaceMeshToObj(outPath);

        if (!string.IsNullOrEmpty(saved))
        {
            occlusionStatusMsg = $"Đã xuất OBJ: output_drawing/{Path.GetFileName(saved)}";
            Debug.Log($"[CR10A Export OBJ] Saved repaired 3D mesh to: {saved}");
        }
        else
        {
            occlusionStatusMsg = "Xuất file .OBJ thất bại.";
        }
    }

    private float CalculateTpsHeightAtPoint(PLYDirectLoader plyLoader, float x, float z, out Vector3 normalOut)
    {
        normalOut = Vector3.up;
        float radiusM = Mathf.Max(0.035f, occlusionSearchRadiusMm * 0.001f * 1.5f);
        List<Vector3> ctrlPts = plyLoader != null
            ? plyLoader.CollectRawSurfaceNeighbors(x, z, radiusM, Mathf.Clamp(occlusionKNeighbors, 8, 16), lastGarmentMinClothY)
            : null;

        if (ctrlPts != null && ctrlPts.Count >= 6)
        {
            if (TrajectoryEvaluationEngine.EvaluateTrueTpsSurface(ctrlPts, x, z, out float tpsY, out Vector3 tpsNorm))
            {
                float minY = float.MaxValue, maxY = float.MinValue;
                for (int i = 0; i < ctrlPts.Count; i++) { minY = Mathf.Min(minY, ctrlPts[i].y); maxY = Mathf.Max(maxY, ctrlPts[i].y); }
                normalOut = tpsNorm;
                return Mathf.Clamp(tpsY, minY - 0.015f, maxY + 0.015f);
            }
        }

        // Fallback về TPS trên 12 mốc giải phẫu nếu lỗ quá lớn không có điểm cảm biến trong bán kính tìm kiếm
        if (landmarksWorldPos != null && landmarksWorldPos.Count >= 4)
        {
            if (TrajectoryEvaluationEngine.EvaluateTrueTpsSurface(landmarksWorldPos, x, z, out float lmY, out Vector3 lmNorm))
            {
                normalOut = lmNorm;
                return lmY;
            }
        }
        return lastGarmentMedianY;
    }

    private float CalculateLaplacianInpaintedHeight(PLYDirectLoader plyLoader, float x, float z, out Vector3 norm)
    {
        norm = Vector3.up;
        if (plyLoader == null) return lastGarmentMedianY;

        float radius = Mathf.Max(0.03f, occlusionSearchRadiusMm * 0.001f);
        List<Vector3> rawNeighbors = plyLoader.CollectRawSurfaceNeighbors(x, z, radius * 1.6f, Mathf.Max(12, occlusionKNeighbors), lastGarmentMinClothY);
        if (rawNeighbors.Count == 0)
            return CalculateTpsHeightAtPoint(plyLoader, x, z, out norm);

        float sumY = 0f, sumW = 0f;
        for (int i = 0; i < rawNeighbors.Count; i++)
        {
            float dx = rawNeighbors[i].x - x;
            float dz = rawNeighbors[i].z - z;
            float w = 1f / (dx * dx + dz * dz + 1e-5f);
            sumY += rawNeighbors[i].y * w;
            sumW += w;
        }

        CalculateMlsTangentPlaneHeight(plyLoader, x, z, out norm);
        return sumW > 0f ? (sumY / sumW) : rawNeighbors[0].y;
    }

    private float CalculateMlsTangentPlaneHeight(PLYDirectLoader plyLoader, float x, float z, out Vector3 norm)
    {
        norm = Vector3.up;
        if (plyLoader == null) return lastGarmentMedianY;

        float radius = Mathf.Max(0.03f, occlusionSearchRadiusMm * 0.001f);
        // Thu thập K điểm cảm biến gốc đối xứng mọi hướng quanh (x, z) với bán kính tự mở rộng & lọc điểm mặt bàn
        List<Vector3> neighbors = plyLoader.CollectRawSurfaceNeighbors(x, z, radius, occlusionKNeighbors, lastGarmentMinClothY);
        if (neighbors.Count < 3)
        {
            return CalculateTpsHeightAtPoint(plyLoader, x, z, out norm);
        }

        // Độ rộng nhân Gauss thích ứng theo khoảng cách tới láng giềng xa nhất (không bị =0 khi lỗ lớn > 5cm!)
        float maxDistSq = 1e-4f;
        for (int i = 0; i < neighbors.Count; i++)
        {
            float dx = neighbors[i].x - x;
            float dz = neighbors[i].z - z;
            maxDistSq = Mathf.Max(maxDistSq, dx * dx + dz * dz);
        }
        float h2 = Mathf.Max(1e-4f, maxDistSq * 0.55f);

        float wSum = 0f, avgX = 0f, avgY = 0f, avgZ = 0f;
        float[] weights = new float[neighbors.Count];
        for (int i = 0; i < neighbors.Count; i++)
        {
            float dx = neighbors[i].x - x;
            float dz = neighbors[i].z - z;
            float w = Mathf.Exp(-(dx * dx + dz * dz) / h2);
            weights[i] = w;
            wSum += w;
            avgX += w * neighbors[i].x;
            avgY += w * neighbors[i].y;
            avgZ += w * neighbors[i].z;
        }
        if (wSum < 1e-8f) return lastGarmentMedianY;
        avgX /= wSum; avgY /= wSum; avgZ /= wSum;

        float sxx = 0f, sxz = 0f, szz = 0f, sxy = 0f, szy = 0f;
        for (int i = 0; i < neighbors.Count; i++)
        {
            float w = weights[i];
            float rx = neighbors[i].x - avgX;
            float rz = neighbors[i].z - avgZ;
            float ry = neighbors[i].y - avgY;
            sxx += w * rx * rx;
            sxz += w * rx * rz;
            szz += w * rz * rz;
            sxy += w * rx * ry;
            szy += w * rz * ry;
        }

        float det = sxx * szz - sxz * sxz;
        if (Mathf.Abs(det) > 1e-10f)
        {
            float a = (sxy * szz - szy * sxz) / det;
            float b = (szy * sxx - sxy * sxz) / det;
            norm = new Vector3(-a, 1f, -b).normalized;
            float yEst = avgY + a * (x - avgX) + b * (z - avgZ);
            float minY = float.MaxValue, maxY = float.MinValue;
            for (int i = 0; i < neighbors.Count; i++) { minY = Mathf.Min(minY, neighbors[i].y); maxY = Mathf.Max(maxY, neighbors[i].y); }
            return Mathf.Clamp(yEst, minY - 0.015f, maxY + 0.015f);
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
        Vector2Int key = new Vector2Int(Mathf.RoundToInt(worldX * 333f), Mathf.RoundToInt(worldZ * 333f));
        if (compensatedHeightGrid.TryGetValue(key, out worldY) && compensatedNormalGrid.TryGetValue(key, out normalWorld))
        {
            return true;
        }
        // Kiểm tra thêm 8 ô lân cận (±3mm) để không bị hụt điểm giữa các bước lưới
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dz == 0) continue;
                Vector2Int nk = new Vector2Int(key.x + dx, key.y + dz);
                if (compensatedHeightGrid.TryGetValue(nk, out worldY) && compensatedNormalGrid.TryGetValue(nk, out normalWorld))
                {
                    return true;
                }
            }
        }
        worldY = 0.020f;
        normalWorld = Vector3.up;
        return false;
    }

    public void ApplyOcclusionCompensationToTrajectory()
    {
        if (waypoints == null || waypoints.Count == 0)
        {
            occlusionStatusMsg = "Chưa có quỹ đạo để áp dụng.";
            return;
        }
        if (compensatedSurfacePointsWorld.Count == 0)
        {
            RunOcclusionCompensation();
        }

        float radiusM = occlusionSearchRadiusMm * 0.001f;
        float standoffM = customDrawZOffsetMm * 0.001f;
        int modifiedCount = 0;
        float[] seedJoints = null;

        for (int i = 0; i < waypoints.Count; i++)
        {
            WaypointData wp = waypoints[i];
            Vector3 p = wp.targetPosWorld_m;

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
                Vector3 surfP = new Vector3(p.x, compensatedSurfacePointsWorld[bestIdx].y, p.z);
                Vector3 compN = gunTiltNormalsWorld[bestIdx].normalized;

                // P_TCP = P_surface + d * normal (đúng hình học 3D theo pháp tuyến!)
                wp.targetPosWorld_m = surfP + compN * standoffM;
                wp.microNormal = compN;

                if (globalReference != null)
                {
                    wp.targetPosGlobal_mm = globalReference.InverseTransformPoint(wp.targetPosWorld_m) * 1000f;
                }

                // Tính góc định hướng 6-DOF chuẩn z_tool = -normal
                Vector3 normRobot = globalReference != null
                    ? globalReference.InverseTransformDirection(compN).normalized
                    : new Vector3(-compN.z, -compN.x, compN.y).normalized;
                Vector3 tanW = (i + 1 < waypoints.Count) ? (waypoints[i + 1].targetPosWorld_m - p) : Vector3.forward;
                Vector3 tanRobot = globalReference != null
                    ? globalReference.InverseTransformDirection(tanW).normalized
                    : new Vector3(-tanW.z, -tanW.x, tanW.y).normalized;
                Vector3 rpy = DobotCR10AKinematics.SurfaceNormalToRobotRpyDeg(normRobot, tanRobot, maxGunTiltAngleDeg);
                wp.roll_deg = rpy.x;
                wp.pitch_deg = rpy.y;
                wp.yaw_deg = rpy.z;

                Quaternion flangeRot = GetWaypointFlangeRotation(wp);
                var sols = DobotCR10AKinematics.SolveTcpPose(
                    wp.targetPosGlobal_mm, flangeRot, seedJoints, toolLength,
                    useAdaptiveTilt: true, maxTiltDeg: maxGunTiltAngleDeg,
                    toolOffsetX: toolOffsetX, toolOffsetY: toolOffsetY
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

        RunDiagnostics();
        occlusionStatusMsg = $"Đã áp dụng vào {modifiedCount}/{waypoints.Count} điểm quỹ đạo.";
        Debug.Log($"[CR10A Occlusion] Đã cập nhật bù đắp vào {modifiedCount} waypoints.");
    }

    // =========================================================================
    // BẢNG VẼ TRÊN ẢNH THỰC TẾ TỪ CAMERA (2D -> 3D)
    // =========================================================================
    public void SetRealCameraImageFromPath(string imagePath, string sampleName = "")
    {
        if (string.IsNullOrEmpty(imagePath) || !File.Exists(imagePath)) return;
        try
        {
            byte[] data = File.ReadAllBytes(imagePath);
            Texture2D tex = new Texture2D(2, 2, TextureFormat.RGBA32, false);
            if (tex.LoadImage(data))
            {
                tex.filterMode = FilterMode.Bilinear;
                tex.wrapMode = TextureWrapMode.Clamp;
                ReleaseRealCameraFileTexture();
                realCameraFileTex = tex;
                explicitRealCameraImagePath = imagePath;
                if (!string.IsNullOrEmpty(sampleName))
                {
                    realCameraLoadedSample = Path.GetFileNameWithoutExtension(sampleName).Trim();
                }
                else
                {
                    realCameraLoadedSample = GetActiveCloudSampleName();
                }
            }
            else
            {
                if (Application.isPlaying) Destroy(tex);
                else DestroyImmediate(tex);
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[CR10A Camera2D] Không thể nạp ảnh camera {imagePath}: {ex.Message}");
        }
    }

    public void RefreshRealCameraImageForCurrentSample()
    {
        realCameraLoadedSample = "";
        explicitRealCameraImagePath = "";
        TryLoadRealCameraImageForSample(GetActiveCloudSampleName());
    }

    private string GetActiveCloudSampleName()
    {
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        string active = (plyLoader != null && !string.IsNullOrEmpty(plyLoader.currentLoadedSampleName))
            ? plyLoader.currentLoadedSampleName
            : selectedSampleName;
        if (active == "full_cloud_world") active = "pcl_25";
        return string.IsNullOrEmpty(active) ? "" : Path.GetFileNameWithoutExtension(active).Trim();
    }

    private void TryLoadRealCameraImageForSample(string sampleName)
    {
        string clean = string.IsNullOrEmpty(sampleName) ? "" : Path.GetFileNameWithoutExtension(sampleName).Trim();
        if (clean == "full_cloud_world") clean = "pcl_25";
        realCameraLoadedSample = clean;

        if (!string.IsNullOrEmpty(explicitRealCameraImagePath) && File.Exists(explicitRealCameraImagePath))
        {
            string expStem = Path.GetFileNameWithoutExtension(explicitRealCameraImagePath);
            if (string.IsNullOrEmpty(clean) || expStem.Equals(clean, StringComparison.OrdinalIgnoreCase))
            {
                if (realCameraFileTex != null) return;
                SetRealCameraImageFromPath(explicitRealCameraImagePath, clean);
                if (realCameraFileTex != null) return;
            }
        }

        List<string> candidates = new List<string>();
        if (!string.IsNullOrEmpty(clean))
        {
            string numPart = clean.StartsWith("pcl_", StringComparison.OrdinalIgnoreCase) ? clean.Substring(4) : clean;
            string repoRoot = Path.GetFullPath(Path.Combine(Application.dataPath, "..", ".."));
            string unityRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));

            candidates.Add(Path.Combine(repoRoot, "ai_src", "dataset", "_2D", clean + ".png"));
            candidates.Add(Path.Combine(repoRoot, "ai_src", "dataset", "_2D", clean + ".jpg"));
            candidates.Add(Path.Combine(repoRoot, "ai_src", "dataset", "_2D", "image_" + numPart + ".png"));
            candidates.Add(Path.Combine(repoRoot, "dataset", "_2D", clean + ".png"));
            candidates.Add(Path.Combine(repoRoot, "dataset", "_2D", "image_" + numPart + ".png"));
            candidates.Add(Path.Combine(unityRoot, "dataset", "_2D", clean + ".png"));
            candidates.Add(Path.Combine(unityRoot, "dataset", "_2D", "image_" + numPart + ".png"));

            if (!string.IsNullOrEmpty(customPlyPath))
            {
                try
                {
                    string plyDir = Path.GetDirectoryName(customPlyPath);
                    if (!string.IsNullOrEmpty(plyDir))
                    {
                        string sibling2D = Path.GetFullPath(Path.Combine(plyDir, "..", "_2D"));
                        candidates.Add(Path.Combine(sibling2D, clean + ".png"));
                        candidates.Add(Path.Combine(sibling2D, "image_" + numPart + ".png"));
                    }
                }
                catch { }
            }
        }

        for (int i = 0; i < candidates.Count; i++)
        {
            if (File.Exists(candidates[i]))
            {
                SetRealCameraImageFromPath(candidates[i], clean);
                if (realCameraFileTex != null) return;
            }
        }

        // Không có file PNG rời -> nhường sang ảnh RGB trích xuất trực tiếp từ Zivid Point Cloud (.ply)
        ReleaseRealCameraFileTexture();
        explicitRealCameraImagePath = "";
    }

    private Texture2D GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out string sourceLabel)
    {
        plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        string activeSample = GetActiveCloudSampleName();
        if (realCameraLoadedSample != activeSample)
        {
            TryLoadRealCameraImageForSample(activeSample);
        }

        if (realCameraFileTex != null)
        {
            nativeW = realCameraFileTex.width;
            nativeH = realCameraFileTex.height;
            sourceLabel = string.IsNullOrEmpty(activeSample)
                ? $"Ảnh Camera Zivid ({nativeW}x{nativeH})"
                : $"Ảnh Camera Zivid: {activeSample} ({nativeW}x{nativeH})";
            return realCameraFileTex;
        }

        if (plyLoader != null)
        {
            plyLoader.EnsureCameraTextureReady();
            if (plyLoader.cameraRgbTexture != null)
            {
                nativeW = plyLoader.cameraImageWidth > 0 ? plyLoader.cameraImageWidth : plyLoader.cameraRgbTexture.width;
                nativeH = plyLoader.cameraImageHeight > 0 ? plyLoader.cameraImageHeight : plyLoader.cameraRgbTexture.height;
                string tag = plyLoader.hasOrganizedCameraFrame ? "Ảnh Camera RGB" : "Ảnh Thực Tế Camera (3D)";
                sourceLabel = string.IsNullOrEmpty(activeSample)
                    ? $"{tag} ({nativeW}x{nativeH})"
                    : $"{tag}: {activeSample}";
                return plyLoader.cameraRgbTexture;
            }
        }

        nativeW = 0;
        nativeH = 0;
        sourceLabel = "Chưa có ảnh camera. Bấm 'Chụp cam' hoặc nạp file .PLY.";
        return null;
    }

    private Vector2[] GetActiveCanvasLandmarks12UV(PLYDirectLoader plyLoader, int nativeW, int nativeH, bool isRealCamera, out bool hasValidRealLandmarks)
    {
        hasValidRealLandmarks = false;
        if (!isRealCamera)
        {
            return GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        }

        string activeSample = GetActiveCloudSampleName();
        if (landmarks3D == null || landmarks3D.Count != 12 ||
            (!string.IsNullOrEmpty(activeSample) && loadedLandmarksSampleName != activeSample))
        {
            return null;
        }

        bool isOrganizedPixelSpace = (realCameraFileTex != null) || (plyLoader != null && plyLoader.hasOrganizedCameraFrame);
        if (isOrganizedPixelSpace && nativeW > 1 && nativeH > 1)
        {
            bool hasPixels = false;
            for (int i = 0; i < 12; i++)
            {
                if (landmarks3D[i].pixel_u > 0 || landmarks3D[i].pixel_v > 0) { hasPixels = true; break; }
            }
            if (hasPixels)
            {
                Vector2[] uvs = new Vector2[12];
                for (int i = 0; i < 12; i++)
                {
                    uvs[i] = new Vector2(
                        Mathf.Clamp01(landmarks3D[i].pixel_u / (float)(nativeW - 1)),
                        Mathf.Clamp01(landmarks3D[i].pixel_v / (float)(nativeH - 1))
                    );
                }
                hasValidRealLandmarks = true;
                return uvs;
            }
        }

        if (plyLoader != null && landmarksWorldPos != null && landmarksWorldPos.Count == 12)
        {
            Vector2[] uvs = new Vector2[12];
            int ok = 0;
            for (int i = 0; i < 12; i++)
            {
                if (plyLoader.TryWorldToCameraUV(landmarksWorldPos[i], out uvs[i])) ok++;
            }
            if (ok == 12)
            {
                hasValidRealLandmarks = true;
                return uvs;
            }
        }

        return null;
    }

    private static Vector2 WarpUV2D(Vector2 uv, Vector2[] src12, Vector2[] dst12)
    {
        if (src12 == null || dst12 == null || src12.Length < 12 || dst12.Length < 12) return uv;

        Vector2 srcCenter = Vector2.zero;
        Vector2 dstCenter = Vector2.zero;
        for (int i = 0; i < 12; i++)
        {
            srcCenter += src12[i];
            dstCenter += dst12[i];
        }
        srcCenter /= 12f;
        dstCenter /= 12f;

        float suu = 0f, suv = 0f, svv = 0f;
        Vector2 suDst = Vector2.zero;
        Vector2 svDst = Vector2.zero;
        for (int i = 0; i < 12; i++)
        {
            float du = src12[i].x - srcCenter.x;
            float dv = src12[i].y - srcCenter.y;
            Vector2 dP = dst12[i] - dstCenter;
            suu += du * du;
            suv += du * dv;
            svv += dv * dv;
            suDst += du * dP;
            svDst += dv * dP;
        }

        float det = suu * svv - suv * suv;
        Vector2 colU = Vector2.zero;
        Vector2 colV = Vector2.zero;
        if (Mathf.Abs(det) > 1e-6f)
        {
            colU = (svv * suDst - suv * svDst) / det;
            colV = (-suv * suDst + suu * svDst) / det;
        }

        Vector2 affinePos = dstCenter + (uv.x - srcCenter.x) * colU + (uv.y - srcCenter.y) * colV;

        Vector2 residualSum = Vector2.zero;
        float weightSum = 0f;
        float sigma2 = 0.025f;
        for (int i = 0; i < 12; i++)
        {
            Vector2 affI = dstCenter + (src12[i].x - srcCenter.x) * colU + (src12[i].y - srcCenter.y) * colV;
            Vector2 resI = dst12[i] - affI;
            float d2 = (uv - src12[i]).sqrMagnitude;
            float w = 1f / (d2 + 0.0015f) + Mathf.Exp(-d2 / sigma2);
            residualSum += w * resI;
            weightSum += w;
        }

        Vector2 result = (weightSum > 1e-6f) ? (affinePos + residualSum / weightSum) : affinePos;
        return new Vector2(Mathf.Clamp01(result.x), Mathf.Clamp01(result.y));
    }

    private void WarpStrokesInPlace(List<List<Vector2>> strokes, Vector2[] src12, Vector2[] dst12)
    {
        if (strokes == null || src12 == null || dst12 == null || src12.Length < 12 || dst12.Length < 12) return;
        for (int s = 0; s < strokes.Count; s++)
        {
            var st = strokes[s];
            if (st == null) continue;
            for (int p = 0; p < st.Count; p++)
            {
                st[p] = WarpUV2D(st[p], src12, dst12);
            }
        }
    }

    private void DrawCanonicalJeans2DCanvasSection(float width)
    {
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        GUILayout.Label("VẼ MẪU PHUN 2D -> 3D", AntigravityUITheme.StyleSectionLabel);
        GUILayout.FlexibleSpace();
        AiServerClient camClient = EnsureAiClient();
        bool prevGuiEnabled = GUI.enabled;
        GUI.enabled = prevGuiEnabled && camClient != null && !camClient.isProcessing;
        if (GUILayout.Button("Chụp cam", AntigravityUITheme.StylePrimaryButton, GUILayout.Width(76), GUILayout.Height(20)))
        {
            camClient.CapturePreview();
        }
        GUI.enabled = prevGuiEnabled;
        if (GUILayout.Button(showCanonicalDrawPanel ? "Thu gọn" : "Mở", AntigravityUITheme.StyleButton, GUILayout.Width(68), GUILayout.Height(20)))
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

        Texture2D activeCamTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out string camSourceLabel);
        canvasUsingRealCamera = (activeCamTex != null);
        Vector2[] curLandmarksUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, canvasUsingRealCamera, out bool hasRealLandmarks);

        GUILayout.Label(camSourceLabel, canvasUsingRealCamera ? AntigravityUITheme.StyleLabelCyan : AntigravityUITheme.StyleLabelMuted);

        // Khung Canvas 2D theo đúng tỷ lệ ảnh thực tế từ camera (không làm méo ảnh)
        float maxCanvasW = Mathf.Clamp(width - 24f, 260f, 360f);
        float canvasW = maxCanvasW;
        float canvasH = canvasW * 0.65f;
        if (activeCamTex != null && activeCamTex.width > 0 && activeCamTex.height > 0)
        {
            float aspect = (float)activeCamTex.height / activeCamTex.width;
            canvasH = canvasW * aspect;
            if (canvasH > 360f)
            {
                canvasH = 360f;
                canvasW = canvasH / Mathf.Max(0.1f, aspect);
            }
            else if (canvasH < 150f)
            {
                canvasH = 150f;
            }
        }

        GUILayout.BeginHorizontal();
        GUILayout.FlexibleSpace();
        Rect canvasRect = GUILayoutUtility.GetRect(canvasW, canvasH, GUILayout.Width(canvasW), GUILayout.Height(canvasH));
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();

        // Vẽ ảnh thực tế từ camera lên Canvas 2D
        if (activeCamTex != null)
        {
            GUI.DrawTexture(canvasRect, activeCamTex, ScaleMode.StretchToFill);
        }
        else
        {
            DrawGuiSolidRect(canvasRect, new Color(0.08f, 0.10f, 0.14f, 1f));
            GUI.Label(
                new Rect(canvasRect.x + 12f, canvasRect.y + canvasRect.height * 0.42f, canvasRect.width - 24f, 44f),
                "Chưa có ảnh thực tế từ camera.\nBấm 'Chụp cam' hoặc nạp file .PLY.",
                AntigravityUITheme.StyleLabelMuted
            );
        }

        // Xử lý sự kiện chuột vẽ trực tiếp trên ảnh Camera 2D
        Event ev = Event.current;
        if (ev != null && canvasRect.Contains(ev.mousePosition))
        {
            Vector2 normUV = new Vector2(
                Mathf.Clamp01((ev.mousePosition.x - canvasRect.x) / canvasRect.width),
                Mathf.Clamp01((ev.mousePosition.y - canvasRect.y) / canvasRect.height)
            );

            if (ev.type == EventType.MouseDown && ev.button == 0)
            {
                hasInitializedDefaultStrokes = true;
                currentDrawingStroke = new List<Vector2> { normUV };
                customDrawnStrokes.Add(currentDrawingStroke);
                if (hasRealLandmarks && curLandmarksUV != null)
                {
                    strokesReferenceLandmarksUV = (Vector2[])curLandmarksUV.Clone();
                }
                isDrawingOnCanvas = true;
                ev.Use();
            }
            else if (ev.type == EventType.MouseDrag && ev.button == 0 && isDrawingOnCanvas && currentDrawingStroke != null)
            {
                if (currentDrawingStroke.Count == 0 || Vector2.Distance(currentDrawingStroke[currentDrawingStroke.Count - 1], normUV) >= 0.010f)
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

        // Vẽ 12 điểm mốc AI đúng vị trí thực tế trên ảnh Camera 2D
        if (curLandmarksUV != null && curLandmarksUV.Length == 12)
        {
            for (int i = 0; i < curLandmarksUV.Length; i++)
            {
                Vector2 uv = curLandmarksUV[i];
                Vector2 ptScreen = new Vector2(canvasRect.x + uv.x * canvasRect.width, canvasRect.y + uv.y * canvasRect.height);
                DrawGuiSolidRect(new Rect(ptScreen.x - 4f, ptScreen.y - 4f, 8f, 8f), new Color(0.05f, 0.05f, 0.08f, 0.9f));
                DrawGuiSolidRect(new Rect(ptScreen.x - 2.5f, ptScreen.y - 2.5f, 5f, 5f), new Color(1f, 0.80f, 0.1f, 0.98f));
            }
        }

        // Vẽ các nét vẽ của người dùng lên ảnh Camera 2D (có viền tối tương phản cao)
        int totalStrokePts = 0;
        for (int s = 0; s < customDrawnStrokes.Count; s++)
        {
            var stroke = customDrawnStrokes[s];
            if (stroke == null || stroke.Count == 0) continue;
            totalStrokePts += stroke.Count;
            Color strokeCol = (s % 2 == 0) ? new Color(0.1f, 0.95f, 1f, 0.98f) : new Color(0.25f, 1f, 0.45f, 0.98f);
            Color haloCol = new Color(0.02f, 0.06f, 0.12f, 0.85f);
            for (int p = 0; p < stroke.Count - 1; p++)
            {
                Vector2 p0 = new Vector2(canvasRect.x + stroke[p].x * canvasRect.width, canvasRect.y + stroke[p].y * canvasRect.height);
                Vector2 p1 = new Vector2(canvasRect.x + stroke[p + 1].x * canvasRect.width, canvasRect.y + stroke[p + 1].y * canvasRect.height);
                DrawGuiLine2D(p0, p1, haloCol, 4.2f);
                DrawGuiLine2D(p0, p1, strokeCol, 2.4f);
            }
            Vector2 startPt = new Vector2(canvasRect.x + stroke[0].x * canvasRect.width, canvasRect.y + stroke[0].y * canvasRect.height);
            DrawGuiSolidRect(new Rect(startPt.x - 3f, startPt.y - 3f, 6f, 6f), Color.yellow);
        }

        GUILayout.Space(4);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Size mẫu: {activeSprayTemplateSize} | Nhận diện: {detectedJeansSize}", AntigravityUITheme.StyleLabelGreen, GUILayout.Width(210));
        foreach (string sz in new[] { "S", "M", "L" })
        {
            GUIStyle st = activeSprayTemplateSize == sz ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            if (GUILayout.Button(sz, st, GUILayout.Width(32), GUILayout.Height(20)))
            {
                ChangeSprayTemplateSize(sz);
            }
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Râu mèo + Đùi", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            LoadPresetWhiskersAndThighOn2DCanvas();
            if (customDrawnStrokes.Count > 0)
            {
                customDrawStatusMsg = "Đã nạp mẫu Râu mèo + Đùi lên ảnh camera.";
            }
        }
        if (GUILayout.Button("Zigzag 2 ống", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            LoadPresetZigzag2LegsOn2DCanvas();
            if (customDrawnStrokes.Count > 0)
            {
                customDrawStatusMsg = "Đã nạp mẫu Zigzag 2 ống lên ảnh camera.";
            }
        }
        if (GUILayout.Button("Hoàn tác", AntigravityUITheme.StyleButton, GUILayout.Width(68), GUILayout.Height(22)))
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
        if (GUILayout.Button("Xóa hết", AntigravityUITheme.StyleButton, GUILayout.Width(62), GUILayout.Height(22)))
        {
            ClearAll2DAnd3DTrajectoryLines();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label("LƯU / NẠP MẪU (.JSON)", AntigravityUITheme.StyleSectionLabel);

        GUILayout.BeginHorizontal();
        GUILayout.Label("Thư mục:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(58));
        sprayTemplateDirectory = GUILayout.TextField(sprayTemplateDirectory, AntigravityUITheme.StyleTextField, GUILayout.Height(22));
        if (GUILayout.Button("Chọn...", AntigravityUITheme.StyleButton, GUILayout.Width(66), GUILayout.Height(22)))
        {
            ChooseSprayTemplateDirectory();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("Tên file:", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(58));
        saveTemplateFileName = GUILayout.TextField(saveTemplateFileName, AntigravityUITheme.StyleTextField, GUILayout.Width(110), GUILayout.Height(24));
        if (GUILayout.Button("Lưu mẫu", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(24)))
        {
            ExportSprayTemplateWithDialog();
        }
        if (GUILayout.Button("Nạp mẫu", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
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
            GUILayout.Label("Có sẵn:", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(52));
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
            " Giữ mẫu vẽ khi đổi phôi",
            AntigravityUITheme.StyleToggle
        );
        GUILayout.EndVertical();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Độ cao Z: {customDrawZOffsetMm:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(130));
        float newSliderZ = GUILayout.HorizontalSlider(customDrawZOffsetMm, 0f, 50f);
        if (Mathf.Abs(newSliderZ - customDrawZOffsetMm) > 0.05f)
        {
            customDrawZOffsetMm = newSliderZ;
            ApplyCanonical2DDrawingTo3DTrajectory();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("0 mm", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            customDrawZOffsetMm = 0f;
            ApplyCanonical2DDrawingTo3DTrajectory();
        }
        if (GUILayout.Button("20 mm", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            customDrawZOffsetMm = 20f;
            ApplyCanonical2DDrawingTo3DTrajectory();
        }
        if (GUILayout.Button(showGizmos ? "Ẩn đường 3D" : "Hiện đường 3D", showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showGizmos = !showGizmos;
        }
        GUILayout.EndHorizontal();

        if (GUILayout.Button($"Áp dụng lên 3D ({customDrawnStrokes.Count} nét)", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(30)))
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
            customDrawStatusMsg = $"Thư mục: {chosen}";
        }
#else
        customDrawStatusMsg = $"Thư mục: {cur}";
#endif
    }

    public void ExportSprayTemplateWithDialog()
    {
        if (customDrawnStrokes == null || customDrawnStrokes.Count == 0)
        {
            customDrawStatusMsg = "Chưa có nét vẽ để lưu.";
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
            customDrawStatusMsg = "Chưa có nét vẽ để lưu.";
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

            Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
            bool usingRealCam = (camTex != null);
            Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, usingRealCam, out bool hasRealLm);
            Vector2[] canonLmUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);

            foreach (var st in customDrawnStrokes)
            {
                if (st == null || st.Count == 0) continue;
                var strokeObj = new Stroke2D();
                foreach (var pt in st)
                {
                    Vector2 outPt = (usingRealCam && hasRealLm && camLmUV != null)
                        ? WarpUV2D(pt, camLmUV, canonLmUV)
                        : pt;
                    strokeObj.points.Add(new StrokePoint2D(outPt.x, outPt.y));
                }
                template.strokes.Add(strokeObj);
            }

            string json = JsonUtility.ToJson(template, true);
            File.WriteAllText(fullPath, json);
            RefreshAvailableSprayTemplates();
            customDrawStatusMsg = $"Đã lưu mẫu: {Path.GetFileName(fullPath)} ({template.strokes.Count} nét, Size {activeSprayTemplateSize})";
            Debug.Log($"[CR10A SprayTemplate] Đã xuất mẫu phun vào {fullPath}");
        }
        catch (Exception ex)
        {
            customDrawStatusMsg = $"Lỗi lưu mẫu: {ex.Message}";
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
                customDrawStatusMsg = $"Không tìm thấy file JSON trong {dir}";
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

                Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
                bool usingRealCam = (camTex != null);
                Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, usingRealCam, out bool hasRealLm);
                if (usingRealCam && hasRealLm && camLmUV != null)
                {
                    Vector2[] srcCanonUV = GetCanonicalLandmarks12UV(string.IsNullOrEmpty(template.size) ? activeSprayTemplateSize : template.size);
                    WarpStrokesInPlace(customDrawnStrokes, srcCanonUV, camLmUV);
                    strokesReferenceLandmarksUV = (Vector2[])camLmUV.Clone();
                }
                else
                {
                    strokesReferenceLandmarksUV = null;
                }

                if (template.zOffsetMm > 0f) customDrawZOffsetMm = template.zOffsetMm;
                saveTemplateFileName = Path.GetFileNameWithoutExtension(fullPath);
                sprayTemplateDirectory = Path.GetDirectoryName(fullPath);
                RefreshAvailableSprayTemplates();

                // Tự động chiếu ngay lên đám mây điểm 3D hiện hành
                ApplyCanonical2DDrawingTo3DTrajectory();

                customDrawStatusMsg = $"Đã nạp mẫu {Path.GetFileName(fullPath)} ({customDrawnStrokes.Count} nét, Size {activeSprayTemplateSize})";
                Debug.Log($"[CR10A SprayTemplate] Đã import mẫu từ {fullPath}");
            }
            else
            {
                customDrawStatusMsg = $"File {Path.GetFileName(fullPath)} không có nét vẽ.";
            }
        }
        catch (Exception ex)
        {
            customDrawStatusMsg = $"Lỗi nạp mẫu: {ex.Message}";
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

        // 1. Chỉ co giãn nét vẽ theo khung quần tượng trưng khi không dùng ảnh camera thực tế
        if (!canvasUsingRealCamera && customDrawnStrokes != null && customDrawnStrokes.Count > 0)
        {
            ScaleCustomDrawnStrokesBetweenSizes(prevSize, newSize);
        }

        // 2. Tái tạo hình ảnh dự phòng theo size mới (S/M/L)
        RebuildCanonicalJeansTexture();

        // 3. Tự động ánh xạ lại nét vẽ lên bề mặt 3D
        if (customDrawnStrokes != null && customDrawnStrokes.Count > 0)
        {
            ApplyCanonical2DDrawingTo3DTrajectory();
        }

        customDrawStatusMsg = $"Đã đổi mẫu phun: Size {prevSize} -> Size {newSize}.";
        SaveAllSettings();
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
        strokesReferenceLandmarksUV = null;

        isPlaying = false;
        isMovingToManualTarget = false;
        paintingStarted = false;
        SetSprayState(false);
        if (waypoints != null) waypoints.Clear();
        ClearInkTrail();
        currentWaypointIndex = 0;
        customDrawStatusMsg = "Đã xóa toàn bộ nét vẽ 2D và quỹ đạo 3D.";
    }

    private void EnsureCanonicalJeansCanvasInitialized()
    {
        if (whitePixelTex == null)
        {
            whitePixelTex = new Texture2D(1, 1, TextureFormat.RGBA32, false);
            whitePixelTex.SetPixel(0, 0, Color.white);
            whitePixelTex.Apply();
        }
        if (!hasInitializedDefaultStrokes)
        {
            string activeSample = GetActiveCloudSampleName();
            if (!string.IsNullOrEmpty(activeSample) &&
                (landmarks3D == null || landmarks3D.Count < 12 || loadedLandmarksSampleName != activeSample))
            {
                LoadLandmarksAndSegmentationForSample(activeSample);
            }

            Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
            if (camTex != null)
            {
                canvasUsingRealCamera = true;
                Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, true, out bool hasRealLm);
                hasInitializedDefaultStrokes = true;
                if (hasRealLm && camLmUV != null)
                {
                    LoadPresetWhiskersAndThighOn2DCanvas();
                }
            }
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

    private void ProjectCanonicalPresetStrokesOntoActiveCanvas()
    {
        Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
        canvasUsingRealCamera = (camTex != null);
        if (!canvasUsingRealCamera)
        {
            strokesReferenceLandmarksUV = null;
            return;
        }

        string activeSample = GetActiveCloudSampleName();
        if (!string.IsNullOrEmpty(activeSample) &&
            (landmarks3D == null || landmarks3D.Count < 12 || loadedLandmarksSampleName != activeSample))
        {
            LoadLandmarksAndSegmentationForSample(activeSample);
        }

        Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, true, out bool hasRealLm);
        if (hasRealLm && camLmUV != null)
        {
            Vector2[] srcCanonUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
            WarpStrokesInPlace(customDrawnStrokes, srcCanonUV, camLmUV);
            strokesReferenceLandmarksUV = (Vector2[])camLmUV.Clone();
        }
        else
        {
            customDrawnStrokes.Clear();
            strokesReferenceLandmarksUV = null;
            customDrawStatusMsg = "Ảnh camera chưa có 12 mốc AI. Hãy vẽ trực tiếp bằng chuột lên ảnh hoặc bấm Chụp + AI.";
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

        ProjectCanonicalPresetStrokesOntoActiveCanvas();
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

        ProjectCanonicalPresetStrokesOntoActiveCanvas();
    }

    /// <summary>
    /// Ánh xạ toàn bộ các nét vẽ trên ảnh Camera 2D thực tế [0, 1] x [0, 1] trực tiếp lên bề mặt 3D
    /// của đám mây điểm Zivid đang nạp (ưu tiên ánh xạ trực tiếp pixel RGB-D -> 3D, hỗ trợ bù đắp bề mặt
    /// & dự phòng nội suy 12 mốc giải phẫu), sau đó giải Động học Nghịch 6-DOF và cập nhật ngay quỹ đạo vận hành!
    /// </summary>
    public void ApplyCanonical2DDrawingTo3DTrajectory()
    {
        if (customDrawnStrokes == null || customDrawnStrokes.Count == 0)
        {
            customDrawStatusMsg = "Chưa có nét vẽ trên ảnh camera 2D.";
            return;
        }

        Texture2D activeCamTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
        canvasUsingRealCamera = (activeCamTex != null);

        string activeCloudSample = GetActiveCloudSampleName();
        if (!string.IsNullOrEmpty(activeCloudSample) &&
            (landmarks3D == null || landmarks3D.Count < 12 || loadedLandmarksSampleName != activeCloudSample))
        {
            LoadLandmarksAndSegmentationForSample(activeCloudSample);
        }

        Vector2[] camLandmarksUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, canvasUsingRealCamera, out bool hasRealLandmarks);
        bool has12Landmarks = (landmarks3D != null && landmarks3D.Count == 12 &&
            (string.IsNullOrEmpty(activeCloudSample) || loadedLandmarksSampleName == activeCloudSample));

        if (!canvasUsingRealCamera && !has12Landmarks)
        {
            customDrawStatusMsg = "Chưa có ảnh camera hoặc 12 mốc của phôi này. Chụp Zivid hoặc nạp .PLY trước.";
            return;
        }

        // Chuẩn bị 12 điểm mốc nguồn/đích cho nhánh dự phòng (khi 1 pixel thiếu chiều sâu hoặc chế độ canonical)
        Vector2[] srcUV = (canvasUsingRealCamera && hasRealLandmarks && camLandmarksUV != null)
            ? camLandmarksUV
            : GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        Vector3[] dstRobotMm = new Vector3[12];
        for (int i = 0; i < 12; i++)
        {
            if (has12Landmarks && i < landmarks3D.Count)
            {
                dstRobotMm[i] = new Vector3(landmarks3D[i].x_mm, landmarks3D[i].y_mm, landmarks3D[i].z_mm);
            }
            else
            {
                dstRobotMm[i] = new Vector3(
                    Mathf.Lerp(430f, 1080f, srcUV[i].y),
                    Mathf.Lerp(320f, -320f, srcUV[i].x),
                    55f
                );
            }
        }

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
            List<Vector2> resampledStroke = ResampleStroke2D(stroke, 0.018f);

            for (int p = 0; p < resampledStroke.Count; p++)
            {
                Vector2 uv = resampledStroke[p];

                Vector3 approxWorld = Vector3.zero;
                float surfaceZ_mm = 20f;
                Vector3 surfaceNormalWorld = Vector3.up;
                bool mappedDirectFromCamera = false;

                if (canvasUsingRealCamera && plyLoader != null &&
                    plyLoader.TrySampleCameraUVToWorld(uv, out Vector3 camSurfWorld, out Vector3 camSurfNormal))
                {
                    approxWorld = camSurfWorld;
                    surfaceZ_mm = Mathf.Max(14f, camSurfWorld.y * 1000f);
                    surfaceNormalWorld = camSurfNormal;
                    mappedDirectFromCamera = true;
                }
                else if (has12Landmarks)
                {
                    Vector3 warpedMm = WarpCanonicalUVTo3DLandmarks(uv, srcUV, dstRobotMm);
                    approxWorld = globalReference != null
                        ? globalReference.TransformPoint(warpedMm * 0.001f)
                        : new Vector3(-warpedMm.y * 0.001f, warpedMm.z * 0.001f, 1.100f - warpedMm.x * 0.001f);
                    surfaceZ_mm = warpedMm.z;
                }
                else
                {
                    continue;
                }

                if (SampleCompensatedSurface(approxWorld.x, approxWorld.z, out float compY, out Vector3 compNorm))
                {
                    if (!mappedDirectFromCamera || Mathf.Abs(compY - approxWorld.y) < 0.08f)
                    {
                        surfaceZ_mm = Mathf.Max(14f, compY * 1000f);
                        surfaceNormalWorld = compNorm;
                    }
                }
                else if (!mappedDirectFromCamera && plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(approxWorld.x, approxWorld.z, out float cloudWorldY, out Vector3 cloudNormal))
                {
                    surfaceZ_mm = Mathf.Max(14f, cloudWorldY * 1000f);
                    surfaceNormalWorld = cloudNormal;
                }
                if (surfaceNormalWorld.y < 0f) surfaceNormalWorld = -surfaceNormalWorld;
                surfaceNormalWorld.Normalize();

                // Giới hạn góc nghiêng pháp tuyến <= θ_max (ràng buộc công nghệ phun)
                float maxTiltAllowed = sprayTechParams != null ? sprayTechParams.maxTiltDeg : maxGunTiltAngleDeg;
                float tiltDeg = Vector3.Angle(surfaceNormalWorld, Vector3.up);
                if (tiltDeg > maxTiltAllowed && maxTiltAllowed > 0.1f)
                {
                    Vector3 rotAx = Vector3.Cross(Vector3.up, surfaceNormalWorld);
                    if (rotAx.sqrMagnitude > 1e-6f)
                        surfaceNormalWorld = (Quaternion.AngleAxis(maxTiltAllowed, rotAx.normalized) * Vector3.up).normalized;
                }

                // P_TCP = P_surface + d * n_surface (Lùi khoảng cách d dọc đúng vector pháp tuyến 3D!)
                Vector3 surfWorld = new Vector3(approxWorld.x, surfaceZ_mm * 0.001f, approxWorld.z);
                Vector3 tcpWorld = surfWorld + surfaceNormalWorld * (customDrawZOffsetMm * 0.001f);
                Vector3 targetMm = globalReference != null
                    ? globalReference.InverseTransformPoint(tcpWorld) * 1000f
                    : new Vector3((1.100f - tcpWorld.z) * 1000f, -tcpWorld.x * 1000f, tcpWorld.y * 1000f);
                targetMm.z = Mathf.Clamp(targetMm.z, 18f, 450f);

                // Giới hạn an toàn bán kính làm việc của Dobot CR10A
                if (targetMm.magnitude > 1145f)
                {
                    float scale = 1140f / targetMm.magnitude;
                    targetMm.x *= scale;
                    targetMm.y *= scale;
                }

                Vector3 prevPosForTangent = prevPosMm;
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
                    : tcpWorld;
                wp.microNormal = surfaceNormalWorld;

                // Tính góc định hướng 6-DOF đầy đủ từ pháp tuyến 3D: z_tool = -n_surface
                Vector3 normRobot = globalReference != null
                    ? globalReference.InverseTransformDirection(surfaceNormalWorld).normalized
                    : new Vector3(-surfaceNormalWorld.z, -surfaceNormalWorld.x, surfaceNormalWorld.y).normalized;
                Vector3 tanRobot = (p > 0) ? (targetMm - prevPosForTangent) : new Vector3(1f, 0f, 0f);
                if (tanRobot.sqrMagnitude < 1e-4f) tanRobot = new Vector3(1f, 0f, 0f);
                Vector3 rpyDeg = DobotCR10AKinematics.SurfaceNormalToRobotRpyDeg(normRobot, tanRobot, maxTiltAllowed);
                wp.roll_deg = rpyDeg.x;
                wp.pitch_deg = rpyDeg.y;
                wp.yaw_deg = rpyDeg.z;

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
            isMovingToManualTarget = false;
            isMovingToHome = false;
            paintingStarted = false;
            SetSprayState(false);
            ClearInkTrail();
            waypoints = mappedWaypoints;
            MarkSprayTransitions();
            currentWaypointIndex = 0;
            showGizmos = true;
            RunDiagnostics();
            if (robotManual != null)
            {
                robotManual.ikStatus = $"Đã nạp quỹ đạo 2D->3D ({waypoints.Count} điểm). Nhấn CHẠY để bắt đầu.";
            }

            customDrawStatusMsg = $"Đã ánh xạ 2D -> 3D: {waypoints.Count} điểm (IK: {solvedCount}/{waypoints.Count}, dài {cumDistMm:F0} mm). Bấm Chạy để vận hành.";
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
    public void LoadLandmarksAndSegmentationForSample(string sampleName, bool autoSwitchSprayTemplate = false, string exactJsonPath = null)
    {
        if (string.IsNullOrEmpty(sampleName)) sampleName = "pcl_25";
        string clean = Path.GetFileNameWithoutExtension(sampleName).Trim();
        if (clean == "full_cloud_world") clean = "pcl_25";
        else if (!clean.StartsWith("pcl_")) clean = "pcl_" + clean;

        string[] candidatePaths = new string[]
        {
            exactJsonPath ?? "",
            Path.Combine(Application.dataPath, "..", "output_drawing", $"{clean}_landmarks_seg.json"),
            Path.Combine(Application.dataPath, "..", "..", "ai_src", "step3_3d_trajectory", "outputs", $"{clean}_landmarks_seg.json")
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

        if (string.IsNullOrEmpty(foundPath))
        {
            landmarks3D.Clear(); landmarksWorldPos.Clear(); garmentContourWorld.Clear();
            sprayZoneLeftWorld.Clear(); sprayZoneRightWorld.Clear(); loadedLandmarksSampleName = "";
            return;
        }

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

            if (preserveCustomDrawingAcrossSamples && customDrawnStrokes != null && customDrawnStrokes.Count > 0 &&
                strokesReferenceLandmarksUV != null && strokesReferenceLandmarksUV.Length == 12)
            {
                Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyForWarp, out int nw, out int nh, out _);
                if (camTex != null)
                {
                    Vector2[] newCamLmUV = GetActiveCanvasLandmarks12UV(plyForWarp, nw, nh, true, out bool hasNewLm);
                    if (hasNewLm && newCamLmUV != null)
                    {
                        WarpStrokesInPlace(customDrawnStrokes, strokesReferenceLandmarksUV, newCamLmUV);
                        strokesReferenceLandmarksUV = (Vector2[])newCamLmUV.Clone();
                    }
                }
            }

            if (autoSwitchSprayTemplate)
            {
                SwitchSprayTemplateToSize(detectedJeansSize);
            }
            else if (string.IsNullOrEmpty(exactJsonPath) && preserveCustomDrawingAcrossSamples && customDrawnStrokes != null && customDrawnStrokes.Count > 0)
            {
                ApplyCanonical2DDrawingTo3DTrajectory();
            }

            if (autoInpaintHolesOnLoad)
            {
                DetectOcclusionAndHoles();
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

        sizeDetectionSummary = $"[{loadedLandmarksSampleName}] Size {size} (Inseam {detectedInseamMm:F0}, Outseam {detectedOutseamMm:F0} mm)";
        customDrawStatusMsg = $"Đã nhận diện Size {size}.";
        Debug.Log($"[CR10A Size] {sizeDetectionSummary}");
    }

    private void DrawJeansSizeDetectionSummaryBox()
    {
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label($"Size nhận diện: {detectedJeansSize} | Mẫu phun: {activeSprayTemplateSize}", AntigravityUITheme.StyleLabelGreen);
        GUILayout.Label($"Inseam: {detectedInseamMm:F0} mm | Outseam: {detectedOutseamMm:F0} mm | Eo: {detectedWaistMm:F0} mm", AntigravityUITheme.StyleLabelCyan);
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
                Rect segBadge = new Rect(bestSegPt.x - 85f, bestSegPt.y - 24f, 185f, 19f);
                DrawGuiSolidRect(segBadge, new Color(0.02f, 0.22f, 0.12f, 0.92f));
                GUI.Label(new Rect(segBadge.x + 6f, segBadge.y + 1f, segBadge.width, 18f), $"SEGMENT ({loadedLandmarksSampleName})", AntigravityUITheme.StyleLabelGreen);
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

        // 5. Vẽ ĐƯỜNG VIỀN MÀU ĐỎ BAO QUANH TỪNG VÙNG CẦN VÁ trực tiếp lên GAME VIEW
        if (showHoleRegionOutlines3D && holeRegionOutlinesWorld != null && holeRegionOutlinesWorld.Count > 0)
        {
            Color redOutlineHalo = new Color(0.24f, 0.01f, 0.01f, 0.94f);
            Color redOutlineCore = new Color(1.00f, 0.10f, 0.08f, 1.00f);

            float topGy = float.MaxValue;
            Vector2 badgeAnchor = Vector2.zero;

            for (int r = 0; r < holeRegionOutlinesWorld.Count; r++)
            {
                List<Vector3> loop = holeRegionOutlinesWorld[r];
                if (loop == null || loop.Count < 3) continue;

                DrawWorldLoopInGUI(cam, loop, redOutlineHalo, 4.8f, true);
                DrawWorldLoopInGUI(cam, loop, redOutlineCore, 2.8f, true);

                for (int k = 0; k < loop.Count; k += Mathf.Max(1, loop.Count / 6))
                {
                    Vector3 sp = cam.WorldToScreenPoint(loop[k]);
                    if (sp.z <= 0.05f) continue;
                    float gy = Screen.height - sp.y;
                    if (gy < topGy)
                    {
                        topGy = gy;
                        badgeAnchor = new Vector2(sp.x, gy);
                    }
                }
            }

            if (interfaceTab == 7 && topGy < float.MaxValue)
            {
                string[] algoBadges = { "K-NN MLS", "TPS RBF", "Laplacian PDE", "Gun Tilt", "Đối xứng" };
                string aTag = algoBadges[Mathf.Clamp(selectedOcclusionAlgorithm, 0, algoBadges.Length - 1)];
                Rect bRect = new Rect(badgeAnchor.x - 120f, Mathf.Max(WorkbenchHeaderHeight + 6f, badgeAnchor.y - 26f), 255f, 20f);
                DrawGuiSolidRect(bRect, new Color(0.28f, 0.03f, 0.03f, 0.93f));
                GUI.Label(new Rect(bRect.x + 6f, bRect.y + 1f, bRect.width - 8f, 18f),
                    $"VÙNG VÁ: {detectedHoleRegionCount} vùng ({detectedHoleCount} ô) [{aTag}]", AntigravityUITheme.StyleLabelRed);
            }
        }

        // 5b. Tia hướng súng phun bù góc (Vàng) khi mở Tab Vá lỗ
        if (showGunTiltVectors3D && interfaceTab == 7 &&
            compensatedSurfacePointsWorld != null && compensatedSurfacePointsWorld.Count > 0)
        {
            int cCount = compensatedSurfacePointsWorld.Count;
            int drawStep = Mathf.Max(1, cCount / 400);
            Color tiltYellow = new Color(1.00f, 0.88f, 0.15f, 0.90f);

            for (int i = 0; i < cCount; i += drawStep)
            {
                if (i >= gunTiltNormalsWorld.Count || (i % (drawStep * 5) != 0)) continue;
                Vector3 wp = compensatedSurfacePointsWorld[i];
                Vector3 sp = cam.WorldToScreenPoint(wp);
                if (sp.z <= 0.05f) continue;
                Vector3 tipW = wp + gunTiltNormalsWorld[i] * 0.032f;
                Vector3 spTip = cam.WorldToScreenPoint(tipW);
                if (spTip.z > 0.05f)
                {
                    DrawGuiLine2D(new Vector2(sp.x, Screen.height - sp.y), new Vector2(spTip.x, Screen.height - spTip.y), tiltYellow, 1.8f);
                }
            }
        }

        // 6. Khi đang ở bảng Hiệu chuẩn Camera: vẽ hệ trục XYZ của Camera (X Đỏ, Y Xanh lá, Z Xanh dương) + trục chiếu xuống bàn
        if (ShouldDrawCameraAxes3D())
        {
            DrawCameraCalibrationAxesInGUI(cam);
        }
    }

    private void Draw3DAxisArrowInGUI(Camera cam, Vector3 startW, Vector3 endW, Color axisCol, string label, float width = 3.2f)
    {
        Vector3 s0 = cam.WorldToScreenPoint(startW);
        Vector3 s1 = cam.WorldToScreenPoint(endW);
        if (s0.z <= 0.05f || s1.z <= 0.05f) return;

        Vector2 p0 = new Vector2(s0.x, Screen.height - s0.y);
        Vector2 p1 = new Vector2(s1.x, Screen.height - s1.y);
        Vector2 dir = p1 - p0;
        float len = dir.magnitude;
        if (len < 2f) return;
        dir /= len;

        // Viền tối tương phản + thân trục sáng màu
        DrawGuiLine2D(p0, p1, new Color(0.03f, 0.04f, 0.06f, 0.90f), width + 2.2f);
        DrawGuiLine2D(p0, p1, axisCol, width);

        // Mũi tên 2 cánh ở đầu trục
        Vector2 perp = new Vector2(-dir.y, dir.x);
        float headLen = Mathf.Clamp(len * 0.22f, 8f, 14f);
        float headW = headLen * 0.48f;
        Vector2 wingA = p1 - dir * headLen + perp * headW;
        Vector2 wingB = p1 - dir * headLen - perp * headW;
        DrawGuiLine2D(p1, wingA, axisCol, Mathf.Max(2.2f, width * 0.8f));
        DrawGuiLine2D(p1, wingB, axisCol, Mathf.Max(2.2f, width * 0.8f));

        if (!string.IsNullOrEmpty(label))
        {
            float bw = Mathf.Max(46f, label.Length * 7.5f + 10f);
            Rect lRect = new Rect(p1.x + 6f, p1.y - 10f, bw, 18f);
            DrawGuiSolidRect(lRect, new Color(0.05f, 0.07f, 0.11f, 0.90f));
            GUIStyle st = new GUIStyle(AntigravityUITheme.StyleLabelCyan)
            {
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter
            };
            st.normal.textColor = axisCol;
            GUI.Label(lRect, label, st);
        }
    }

    private void DrawCameraCalibrationAxesInGUI(Camera cam)
    {
        CalibrationManager calib = EnsureCalibrationManager();
        if (calib == null) return;

        Color colX = new Color(1.00f, 0.24f, 0.22f, 1.0f); // Đỏ: +X_cam
        Color colY = new Color(0.22f, 0.95f, 0.42f, 1.0f); // Xanh lá: +Y_cam
        Color colZ = new Color(0.24f, 0.68f, 1.00f, 1.0f); // Xanh dương: +Z_cam (quang trục)

        // 1. Hệ trục XYZ tại đúng tâm quang học Camera (X, Y = 320 mm; Z = 400 mm)
        Vector3 originW = calib.CameraMmToUnityWorld(Vector3.zero, globalReference);
        Vector3 tipXW = calib.CameraMmToUnityWorld(new Vector3(320f, 0f, 0f), globalReference);
        Vector3 tipYW = calib.CameraMmToUnityWorld(new Vector3(0f, 320f, 0f), globalReference);
        Vector3 tipZW = calib.CameraMmToUnityWorld(new Vector3(0f, 0f, 400f), globalReference);

        // Tia quang học từ tâm Camera xuống mặt bàn
        float zTableMm = Mathf.Clamp(calib.camPosRobotZ_mm - calib.tableHeightMm, 400f, 2600f);
        Vector3 tableHitW = calib.CameraMmToUnityWorld(new Vector3(0f, 0f, zTableMm), globalReference);
        Vector3 sOrig = cam.WorldToScreenPoint(originW);
        Vector3 sHit = cam.WorldToScreenPoint(tableHitW);
        if (sOrig.z > 0.05f && sHit.z > 0.05f)
        {
            DrawGuiLine2D(
                new Vector2(sOrig.x, Screen.height - sOrig.y),
                new Vector2(sHit.x, Screen.height - sHit.y),
                new Color(0.25f, 0.72f, 1.0f, 0.42f), 1.6f);
        }

        Draw3DAxisArrowInGUI(cam, originW, tipXW, colX, "+X_cam", 3.4f);
        Draw3DAxisArrowInGUI(cam, originW, tipYW, colY, "+Y_cam", 3.4f);
        Draw3DAxisArrowInGUI(cam, originW, tipZW, colZ, "+Z_cam", 3.4f);

        if (sOrig.z > 0.05f)
        {
            Vector2 po = new Vector2(sOrig.x, Screen.height - sOrig.y);
            DrawGuiSolidRect(new Rect(po.x - 5f, po.y - 5f, 10f, 10f), new Color(1f, 0.92f, 0.25f, 1f));
            Rect camTag = new Rect(po.x - 58f, po.y - 26f, 116f, 18f);
            DrawGuiSolidRect(camTag, new Color(0.06f, 0.09f, 0.14f, 0.92f));
            GUI.Label(new Rect(camTag.x + 4f, camTag.y, camTag.width - 8f, 18f), "GỐC CAMERA", AntigravityUITheme.StyleLabelYellow);
        }

        // 2. Hệ trục XY chiếu lên mặt bàn tại tâm quang học (giúp thấy rõ hướng X_cam / Y_cam ngay cả khi zoom sát phôi!)
        Vector3 projXW = calib.CameraMmToUnityWorld(new Vector3(160f, 0f, zTableMm), globalReference);
        Vector3 projYW = calib.CameraMmToUnityWorld(new Vector3(0f, 160f, zTableMm), globalReference);
        Draw3DAxisArrowInGUI(cam, tableHitW, projXW, new Color(1.0f, 0.35f, 0.30f, 0.85f), "X_cam", 2.4f);
        Draw3DAxisArrowInGUI(cam, tableHitW, projYW, new Color(0.30f, 0.95f, 0.48f, 0.85f), "Y_cam", 2.4f);
        if (sHit.z > 0.05f)
        {
            Vector2 ph = new Vector2(sHit.x, Screen.height - sHit.y);
            DrawGuiSolidRect(new Rect(ph.x - 4f, ph.y - 4f, 8f, 8f), colZ);
        }

        // 3. Hệ trục tham chiếu Gốc Robot (nhỏ gọn) để đối chiếu trực tiếp Cam -> Robot
        Vector3 robOrigW = globalReference != null ? globalReference.TransformPoint(Vector3.zero) : new Vector3(0f, 0f, calib.robotGlobalOriginZ_mm * 0.001f);
        Vector3 robXW = globalReference != null ? globalReference.TransformPoint(new Vector3(0.22f, 0f, 0f)) : robOrigW + new Vector3(0f, 0f, -0.22f);
        Vector3 robYW = globalReference != null ? globalReference.TransformPoint(new Vector3(0f, 0.22f, 0f)) : robOrigW + new Vector3(-0.22f, 0f, 0f);
        Vector3 robZW = globalReference != null ? globalReference.TransformPoint(new Vector3(0f, 0f, 0.22f)) : robOrigW + new Vector3(0f, 0.22f, 0f);
        Draw3DAxisArrowInGUI(cam, robOrigW, robXW, new Color(1.0f, 0.45f, 0.45f, 0.85f), "X_rob", 2.2f);
        Draw3DAxisArrowInGUI(cam, robOrigW, robYW, new Color(0.45f, 1.0f, 0.55f, 0.85f), "Y_rob", 2.2f);
        Draw3DAxisArrowInGUI(cam, robOrigW, robZW, new Color(0.45f, 0.78f, 1.0f, 0.85f), "Z_rob", 2.2f);
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

        // 3b. Vẽ ĐƯỜNG VIỀN MÀU ĐỎ bao quanh các vùng cần vá trong Scene View (luôn hiện dù chưa sinh quỹ đạo)
        if (showHoleRegionOutlines3D && holeRegionOutlinesWorld != null && holeRegionOutlinesWorld.Count > 0)
        {
            Gizmos.color = new Color(1.0f, 0.10f, 0.08f, 1.0f);
            for (int r = 0; r < holeRegionOutlinesWorld.Count; r++)
            {
                List<Vector3> loop = holeRegionOutlinesWorld[r];
                if (loop == null || loop.Count < 3) continue;
                for (int k = 0; k < loop.Count - 1; k++)
                    Gizmos.DrawLine(loop[k], loop[k + 1]);
                Gizmos.DrawLine(loop[loop.Count - 1], loop[0]);
            }
        }

        // 3c. Vẽ Vector bù góc súng (Tia Vàng) trong Scene View
        if (showGunTiltVectors3D && compensatedSurfacePointsWorld != null && compensatedSurfacePointsWorld.Count > 0)
        {
            Gizmos.color = new Color(1.0f, 0.88f, 0.15f, 0.95f);
            for (int i = 0; i < compensatedSurfacePointsWorld.Count; i += 4)
            {
                if (i < gunTiltNormalsWorld.Count)
                {
                    Gizmos.DrawRay(compensatedSurfacePointsWorld[i], gunTiltNormalsWorld[i] * 0.035f);
                }
            }
        }

        // 3d. Vẽ hệ trục XYZ của Camera trong Scene View khi đang hiệu chuẩn
        if (ShouldDrawCameraAxes3D())
        {
            CalibrationManager calibG = calibrationManager != null ? calibrationManager : FindAnyObjectByType<CalibrationManager>();
            if (calibG != null)
            {
                Vector3 oW = calibG.CameraMmToUnityWorld(Vector3.zero, globalReference);
                Vector3 xW = calibG.CameraMmToUnityWorld(new Vector3(320f, 0f, 0f), globalReference);
                Vector3 yW = calibG.CameraMmToUnityWorld(new Vector3(0f, 320f, 0f), globalReference);
                Vector3 zW = calibG.CameraMmToUnityWorld(new Vector3(0f, 0f, 400f), globalReference);
                float zTab = Mathf.Clamp(calibG.camPosRobotZ_mm - calibG.tableHeightMm, 400f, 2600f);
                Vector3 hitW = calibG.CameraMmToUnityWorld(new Vector3(0f, 0f, zTab), globalReference);

                Gizmos.color = Color.red; Gizmos.DrawLine(oW, xW);
                Gizmos.color = Color.green; Gizmos.DrawLine(oW, yW);
                Gizmos.color = Color.blue; Gizmos.DrawLine(oW, zW);
                Gizmos.color = new Color(0.25f, 0.72f, 1f, 0.45f); Gizmos.DrawLine(oW, hitW);
                Gizmos.color = Color.yellow; Gizmos.DrawSphere(oW, 0.018f);
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
                c.depth = -2;
                c.tag = "Untagged";
                if (c.targetTexture == null) c.enabled = false;
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

    // =========================================================================
    // CARD 8: ĐÁNH GIÁ CHỈ TIÊU, SO SÁNH P1/P3/P4, MÔ PHỎNG PHUN GAUSS & KHUÔN 3D
    // =========================================================================

    private List<List<Vector2>> BuildDefaultCanonicalStrokesForSize(string size)
    {
        var strokes = new List<List<Vector2>>();
        int nWhiskers = 2, nPasses = 2;
        float vTop = 0.47f, vBot = 0.76f, passSpacing = 0.036f;
        if (size == "M") { nWhiskers = 2; nPasses = 3; vTop = 0.46f; vBot = 0.80f; passSpacing = 0.032f; }
        else if (size == "L") { nWhiskers = 3; nPasses = 4; vTop = 0.45f; vBot = 0.84f; passSpacing = 0.026f; }

        for (int w = 0; w < nWhiskers; w++)
        {
            float vBase = 0.41f + w * 0.04f;
            var whiskerL = new List<Vector2>();
            for (int k = 0; k <= 8; k++)
            {
                float t = (float)k / 8f;
                whiskerL.Add(new Vector2(Mathf.Lerp(0.28f, 0.42f, t), vBase + 0.02f * (t - 0.5f)));
            }
            strokes.Add(whiskerL);
        }
        for (int p = 0; p < nPasses; p++)
        {
            float uOff = (p - (nPasses - 1) * 0.5f) * passSpacing;
            var passL = new List<Vector2>();
            for (int k = 0; k <= 14; k++)
            {
                float t = (p % 2 == 0) ? ((float)k / 14f) : (1f - (float)k / 14f);
                passL.Add(new Vector2(Mathf.Lerp(0.35f, 0.33f, t) + uOff, Mathf.Lerp(vTop, vBot, t)));
            }
            strokes.Add(passL);
        }
        for (int w = 0; w < nWhiskers; w++)
        {
            float vBase = 0.41f + w * 0.04f;
            var whiskerR = new List<Vector2>();
            for (int k = 0; k <= 8; k++)
            {
                float t = (float)k / 8f;
                whiskerR.Add(new Vector2(Mathf.Lerp(0.58f, 0.72f, t), vBase - 0.02f * (t - 0.5f)));
            }
            strokes.Add(whiskerR);
        }
        for (int p = 0; p < nPasses; p++)
        {
            float uOff = (p - (nPasses - 1) * 0.5f) * passSpacing;
            var passR = new List<Vector2>();
            for (int k = 0; k <= 14; k++)
            {
                float t = (p % 2 == 0) ? ((float)k / 14f) : (1f - (float)k / 14f);
                passR.Add(new Vector2(Mathf.Lerp(0.65f, 0.67f, t) + uOff, Mathf.Lerp(vTop, vBot, t)));
            }
            strokes.Add(passR);
        }
        return strokes;
    }

    /// <summary>
    /// Chuyển nét vẽ hiện tại (trên ảnh camera thực tế) về không gian chuẩn Canonical UV [0, 1]x[0, 1]
    /// để truyền chính xác cho các bộ sinh quỹ đạo P3 (12 mốc) và P4 (ICP + TPS).
    /// Nếu người dùng chưa vẽ nét nào thì tự sinh bộ nét Râu mèo + Đùi chuẩn.
    /// </summary>
    private List<List<Vector2>> GetCanonicalTemplateStrokesForMethods()
    {
        if (customDrawnStrokes == null || customDrawnStrokes.Count == 0)
        {
            return BuildDefaultCanonicalStrokesForSize(activeSprayTemplateSize);
        }

        Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
        bool usingRealCam = (camTex != null);
        Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, usingRealCam, out bool hasRealLm);
        Vector2[] canonLmUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);

        var canonicalList = new List<List<Vector2>>(customDrawnStrokes.Count);
        for (int s = 0; s < customDrawnStrokes.Count; s++)
        {
            var srcStroke = customDrawnStrokes[s];
            if (srcStroke == null || srcStroke.Count < 2) continue;
            var dstStroke = new List<Vector2>(srcStroke.Count);
            for (int p = 0; p < srcStroke.Count; p++)
            {
                Vector2 pt = srcStroke[p];
                if (usingRealCam && hasRealLm && camLmUV != null)
                {
                    pt = WarpUV2D(pt, camLmUV, canonLmUV);
                }
                dstStroke.Add(pt);
            }
            canonicalList.Add(dstStroke);
        }

        return canonicalList.Count > 0 ? canonicalList : BuildDefaultCanonicalStrokesForSize(activeSprayTemplateSize);
    }

    public void GenerateAndApplyMethodTrajectory(string method)
    {
        EnsureCanonicalJeansCanvasInitialized();
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        string activeSample = GetActiveCloudSampleName();
        if (string.IsNullOrEmpty(activeSample)) activeSample = selectedSampleName;

        if (landmarksWorldPos == null || landmarksWorldPos.Count < 12 ||
            (!string.IsNullOrEmpty(activeSample) && loadedLandmarksSampleName != activeSample))
        {
            LoadLandmarksAndSegmentationForSample(activeSample);
        }
        if (landmarksWorldPos == null || landmarksWorldPos.Count < 12)
        {
            comparisonStatusMsg = $"Chưa có 12 mốc AI cho phôi [{activeSample}]. Hãy bấm 'Chụp + AI' hoặc 'Nhận diện AI' trước.";
            return;
        }

        List<List<Vector2>> canonicalStrokes = GetCanonicalTemplateStrokesForMethods();
        List<WaypointData> generated = null;
        float tGen = 0f;
        float regRes = 0f;

        if (method == "P1")
        {
            generated = TrajectoryEvaluationEngine.GenerateP1Zigzag(this, plyLoader, sprayTechParams, out tGen);
        }
        else if (method == "P3")
        {
            generated = TrajectoryEvaluationEngine.GenerateP3LandmarkTemplate(this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, out tGen);
        }
        else
        {
            generated = TrajectoryEvaluationEngine.GenerateP4RegistrationTpsIcp(this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, out regRes, out tGen);
        }

        if (generated != null && generated.Count > 0)
        {
            isPlaying = false;
            isMovingToManualTarget = false;
            isMovingToHome = false;
            paintingStarted = false;
            SetSprayState(false);
            ClearInkTrail();
            waypoints = generated;
            MarkSprayTransitions();
            currentWaypointIndex = 0;
            showGizmos = true;
            RunDiagnostics();

            if (robotManual != null)
            {
                robotManual.ikStatus = $"Đã sinh quỹ đạo [{method}] ({waypoints.Count} điểm). Nhấn CHẠY để bắt đầu.";
            }

            if (showDepositionHeatmap3D)
            {
                TrajectoryEvaluationEngine.SimulateGaussianDepositionAndHeatmap(
                    waypoints, this, plyLoader, sprayTechParams, true, ref depositionHeatmapObj, out _, out _, out _);
            }

            int solvedCnt = 0;
            for (int i = 0; i < waypoints.Count; i++) if (waypoints[i].isSolved) solvedCnt++;

            string warnP4 = (method == "P4" && regRes > sprayTechParams.registrationResidualThresholdMm)
                ? $" [Cảnh báo dư={regRes:F1}mm]"
                : (method == "P4" ? $", dư={regRes:F2} mm" : "");
            comparisonStatusMsg = $"Đã sinh [{method}]: {waypoints.Count} điểm (IK {solvedCnt}/{waypoints.Count}, {tGen * 1000f:F0} ms{warnP4}). Bấm Chạy để vận hành.";
        }
        else
        {
            comparisonStatusMsg = $"Không sinh được điểm nào cho [{method}] trên phôi [{activeSample}]. Kiểm tra lại 12 mốc AI hoặc biên trừ hao.";
        }
    }

    public void RunAllMethodsComparison()
    {
        EnsureCanonicalJeansCanvasInitialized();
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        string activeSample = GetActiveCloudSampleName();
        if (string.IsNullOrEmpty(activeSample)) activeSample = selectedSampleName;

        if (landmarksWorldPos == null || landmarksWorldPos.Count < 12 ||
            (!string.IsNullOrEmpty(activeSample) && loadedLandmarksSampleName != activeSample))
        {
            LoadLandmarksAndSegmentationForSample(activeSample);
        }
        if (landmarksWorldPos == null || landmarksWorldPos.Count < 12)
        {
            comparisonStatusMsg = $"Chưa có 12 mốc AI cho [{activeSample}] để so sánh P1 / P3 / P4.";
            return;
        }

        comparisonResults.Clear();
        List<List<Vector2>> canonicalStrokes = GetCanonicalTemplateStrokesForMethods();

        var wpP1 = TrajectoryEvaluationEngine.GenerateP1Zigzag(this, plyLoader, sprayTechParams, out float t1);
        var wpP3 = TrajectoryEvaluationEngine.GenerateP3LandmarkTemplate(this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, out float t3);
        var wpP4 = TrajectoryEvaluationEngine.GenerateP4RegistrationTpsIcp(this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, out float res4, out float t4);

        // Dùng P4 làm đường chuẩn tham chiếu nội bộ nếu chưa nạp ground truth riêng
        var refGt = (wpP4 != null && wpP4.Count > 0) ? wpP4 : wpP3;

        GameObject dummyHm = null;
        var r1 = TrajectoryEvaluationEngine.EvaluateTrajectory("P1_Zigzag", wpP1, refGt, this, plyLoader, sprayTechParams, t1, 0f, 4.5f, false, ref dummyHm);
        var r3 = TrajectoryEvaluationEngine.EvaluateTrajectory("P3_12Moc", wpP3, refGt, this, plyLoader, sprayTechParams, t3, 0f, 4.0f, false, ref dummyHm);
        var r4 = TrajectoryEvaluationEngine.EvaluateTrajectory("P4_ICP_TPS", wpP4, refGt, this, plyLoader, sprayTechParams, t4, res4, 3.5f, showDepositionHeatmap3D, ref depositionHeatmapObj);

        comparisonResults.Add(r1);
        comparisonResults.Add(r3);
        comparisonResults.Add(r4);

        // Chọn phương án điểm cao nhất làm tóm tắt
        var best = r1;
        if (r3.weightedScore > best.weightedScore) best = r3;
        if (r4.weightedScore > best.weightedScore) best = r4;
        comparisonStatusMsg = $"Tốt nhất [{selectedSampleName}]: {best.methodName} ({best.weightedScore:F1}/100).";
    }

    private void DrawEvaluationAndSprayCard(float width)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.Label("ĐÁNH GIÁ & PHUN GAUSS", AntigravityUITheme.StyleCardHeader);

        GUILayout.Space(4);
        GUILayout.Label("1. THÔNG SỐ SÚNG PHUN", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Khoảng cách d: {sprayTechParams.standoffMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(170));
        sprayTechParams.standoffMm = GUILayout.HorizontalSlider(sprayTechParams.standoffMm, 15f, 250f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Bề rộng vệt w: {sprayTechParams.sprayWidthMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(170));
        sprayTechParams.sprayWidthMm = GUILayout.HorizontalSlider(sprayTechParams.sprayWidthMm, 25f, 150f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Chồng lấn r: {sprayTechParams.overlapRatio * 100f:F0}%", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(170));
        sprayTechParams.overlapRatio = GUILayout.HorizontalSlider(sprayTechParams.overlapRatio, 0.1f, 0.75f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Độ lệch chuẩn σ: {sprayTechParams.gaussSigmaMm:F1} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(170));
        sprayTechParams.gaussSigmaMm = GUILayout.HorizontalSlider(sprayTechParams.gaussSigmaMm, 8f, 45f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Nghiêng tối đa: {sprayTechParams.maxTiltDeg:F0}°", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(170));
        sprayTechParams.maxTiltDeg = GUILayout.HorizontalSlider(sprayTechParams.maxTiltDeg, 10f, 45f);
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Biên trừ hao e: {sprayTechParams.borderMarginMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(170));
        sprayTechParams.borderMarginMm = GUILayout.HorizontalSlider(sprayTechParams.borderMarginMm, 5f, 35f);
        GUILayout.EndHorizontal();

        float stepS = sprayTechParams.sprayWidthMm * (1f - sprayTechParams.overlapRatio);
        float ripplePct = TrajectoryEvaluationEngine.ComputeGaussRipplePercent(sprayTechParams.gaussSigmaMm, stepS);
        GUILayout.Label($"Bước s = {stepS:F1} mm | Dao động Gauss = {ripplePct:F2}%", ripplePct <= 5f ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("2. SO SÁNH P1 / P3 / P4", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Chạy so sánh", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            RunAllMethodsComparison();
        }
        if (GUILayout.Button("Xuất CSV", AntigravityUITheme.StyleButton, GUILayout.Width(95), GUILayout.Height(28)))
        {
            if (comparisonResults.Count == 0) RunAllMethodsComparison();
            string exportDir = Path.Combine(Application.dataPath, "..", "output_drawing");
            string csvPath = TrajectoryEvaluationEngine.ExportComparisonCsv(comparisonResults, exportDir);
            comparisonStatusMsg = $"Đã xuất: {Path.GetFileName(csvPath)}";
        }
        GUILayout.EndHorizontal();

        bool prevHm = showDepositionHeatmap3D;
        showDepositionHeatmap3D = GUILayout.Toggle(showDepositionHeatmap3D, " Hiện Heatmap phun Gauss 3D", AntigravityUITheme.StyleToggle);
        if (showDepositionHeatmap3D != prevHm)
        {
            PLYDirectLoader pl = FindAnyObjectByType<PLYDirectLoader>();
            if (showDepositionHeatmap3D && waypoints.Count >= 2)
            {
                TrajectoryEvaluationEngine.SimulateGaussianDepositionAndHeatmap(
                    waypoints, this, pl, sprayTechParams, true, ref depositionHeatmapObj, out _, out _, out _);
            }
            else if (depositionHeatmapObj != null)
            {
                depositionHeatmapObj.SetActive(false);
            }
        }

        GUILayout.Label(comparisonStatusMsg, AntigravityUITheme.StyleLabelCyan);

        if (comparisonResults.Count > 0)
        {
            GUILayout.Space(4);
            for (int i = 0; i < comparisonResults.Count; i++)
            {
                var r = comparisonResults[i];
                GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle());
                GUILayout.BeginHorizontal();
                GUILayout.Label($"{r.methodName} ({r.waypointCount} wp)", AntigravityUITheme.StyleLabelCyan);
                GUILayout.FlexibleSpace();
                GUILayout.Label($"{r.weightedScore:F1}/100 [{(r.passedGate ? "ĐẠT" : "KIỂM TRA")}]",
                    r.passedGate ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);
                GUILayout.EndHorizontal();

                GUILayout.Label($"E_geo={r.eGeoMeanMm:F2}mm | E_d={r.eDMeanMm:F2}mm | E_θ={r.eThetaMeanDeg:F1}°", AntigravityUITheme.StyleLabelMuted);
                GUILayout.Label($"E_T={r.eTPercent:F1}% | Phủ={r.coveragePercent:F1}% | Tràn={r.oversprayPercent:F1}% | t={r.tCycleSec:F1}s", AntigravityUITheme.StyleLabelMuted);

                string tag = r.methodName.StartsWith("P1") ? "P1" : (r.methodName.StartsWith("P3") ? "P3" : "P4");
                if (GUILayout.Button($"Nạp {tag}", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
                {
                    GenerateAndApplyMethodTrajectory(tag);
                }
                GUILayout.EndVertical();
            }
        }
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("3. KIỂM CHỨNG 3D", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        bool prevTbl = showPhysicalWorktable;
        showPhysicalWorktable = GUILayout.Toggle(showPhysicalWorktable, " Hiện bàn thao tác (Collider)", AntigravityUITheme.StyleToggle);
        if (showPhysicalWorktable != prevTbl)
        {
            if (showPhysicalWorktable)
            {
                PLYDirectLoader pl = FindAnyObjectByType<PLYDirectLoader>();
                worktableObj = TrajectoryEvaluationEngine.EnsurePhysicalWorktable(null, pl != null ? pl.estimatedTableY : 0f);
            }
            else if (worktableObj != null)
            {
                worktableObj.SetActive(false);
            }
        }

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Bật khuôn đứng 3D", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            showStandingMold3D = true;
            var moldWps = TrajectoryEvaluationEngine.GenerateStandingMold3DReachTest(this, sprayTechParams, ref standingMoldObj, out standingMoldReachSummary);
            if (moldWps.Count > 0)
            {
                isPlaying = false;
                isMovingToManualTarget = false;
                isMovingToHome = false;
                paintingStarted = false;
                SetSprayState(false);
                ClearInkTrail();
                waypoints = moldWps;
                MarkSprayTransitions();
                currentWaypointIndex = 0;
                showGizmos = true;
                RunDiagnostics();
                if (robotManual != null)
                {
                    robotManual.ikStatus = $"Đã nạp quỹ đạo khuôn đứng 3D ({waypoints.Count} điểm). Nhấn CHẠY để bắt đầu.";
                }
            }
        }
        if (showStandingMold3D && GUILayout.Button("Ẩn khuôn", AntigravityUITheme.StyleButton, GUILayout.Width(86), GUILayout.Height(28)))
        {
            showStandingMold3D = false;
            if (standingMoldObj != null) standingMoldObj.SetActive(false);
            RunDiagnostics();
        }
        GUILayout.EndHorizontal();

        if (!string.IsNullOrEmpty(standingMoldReachSummary))
        {
            GUILayout.Label(standingMoldReachSummary, AntigravityUITheme.StyleLabelGreen);
        }
        GUILayout.EndVertical();

        GUILayout.EndVertical();
    }

    // =========================================================================
    // CARD 9: HIỆU CHỈNH VỊ TRÍ CAMERA & QUY ĐỔI TỌA ĐỘ CAMERA -> ROBOT
    // =========================================================================

    private bool DrawCalibNumericSliderRow(string label, ref float value, float min, float max, float stepSmall, float stepBig, string unit)
    {
        bool changed = false;
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, AntigravityUITheme.StyleLabelCyan, GUILayout.Width(96));

        string txt = value.ToString("F1", CultureInfo.InvariantCulture);
        string newTxt = GUILayout.TextField(txt, AntigravityUITheme.StyleTextField, GUILayout.Width(60));
        if (newTxt != txt && float.TryParse(newTxt, NumberStyles.Float, CultureInfo.InvariantCulture, out float parsed))
        {
            value = Mathf.Clamp(parsed, min, max);
            changed = true;
        }
        GUILayout.Label(unit, AntigravityUITheme.StyleLabelMuted, GUILayout.Width(24));

        if (GUILayout.Button($"-{stepBig:0}", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) { value = Mathf.Clamp(value - stepBig, min, max); changed = true; }
        if (GUILayout.Button($"-{stepSmall:0}", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(20))) { value = Mathf.Clamp(value - stepSmall, min, max); changed = true; }
        float sVal = GUILayout.HorizontalSlider(value, min, max);
        if (Mathf.Abs(sVal - value) > 0.05f) { value = Mathf.Round(sVal * 10f) / 10f; changed = true; }
        if (GUILayout.Button($"+{stepSmall:0}", AntigravityUITheme.StyleButton, GUILayout.Width(26), GUILayout.Height(20))) { value = Mathf.Clamp(value + stepSmall, min, max); changed = true; }
        if (GUILayout.Button($"+{stepBig:0}", AntigravityUITheme.StyleButton, GUILayout.Width(30), GUILayout.Height(20))) { value = Mathf.Clamp(value + stepBig, min, max); changed = true; }
        GUILayout.EndHorizontal();
        return changed;
    }

    private Vector3 DrawVector3InputRow(string rowLabel, Vector3 vec)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(rowLabel, AntigravityUITheme.StyleLabelCyan, GUILayout.Width(96));

        GUILayout.Label("X", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(14));
        string sx = GUILayout.TextField(vec.x.ToString("F1", CultureInfo.InvariantCulture), AntigravityUITheme.StyleTextField, GUILayout.Width(64));
        if (float.TryParse(sx, NumberStyles.Float, CultureInfo.InvariantCulture, out float vx)) vec.x = vx;

        GUILayout.Label("Y", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(14));
        string sy = GUILayout.TextField(vec.y.ToString("F1", CultureInfo.InvariantCulture), AntigravityUITheme.StyleTextField, GUILayout.Width(64));
        if (float.TryParse(sy, NumberStyles.Float, CultureInfo.InvariantCulture, out float vy)) vec.y = vy;

        GUILayout.Label("Z", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(14));
        string sz = GUILayout.TextField(vec.z.ToString("F1", CultureInfo.InvariantCulture), AntigravityUITheme.StyleTextField, GUILayout.Width(64));
        if (float.TryParse(sz, NumberStyles.Float, CultureInfo.InvariantCulture, out float vz)) vec.z = vz;

        GUILayout.EndHorizontal();
        return vec;
    }

    private void DrawCameraCalibrationCard(float width)
    {
        CalibrationManager calib = EnsureCalibrationManager();
        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        RobotCameraController rcc = EnsureCameraController();
        FixedRigCamera rigCam = rcc != null ? rcc.EnsureFixedRigCamera(createIfMissing: false) : FindAnyObjectByType<FixedRigCamera>();

        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.Label("HIỆU CHUẨN CAMERA", AntigravityUITheme.StyleCardHeader);
        GUILayout.Label(calib.calibrationStatus, calib.isCalibrationValid ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);

        GUILayout.Space(6);
        GUILayout.Label("0. GÓC NHÌN & MÔ PHỎNG CAMERA ZIVID", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        GUIStyle povBtnSt = (rcc != null && rcc.isZividCameraView) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("Xem góc Camera", povBtnSt, GUILayout.Height(26))) { if (rcc != null) rcc.ToggleZividCameraView(); }
        GUIStyle pipBtnSt = (rcc != null && rcc.showZividPipWindow && !rcc.isZividCameraView) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
        if (GUILayout.Button("Khung PiP", pipBtnSt, GUILayout.Height(26))) { if (rcc != null) rcc.ToggleZividPip(); }
        if (GUILayout.Button("Về 3/4", AntigravityUITheme.StyleButton, GUILayout.Width(64), GUILayout.Height(26))) { if (rcc != null) rcc.SetPresetPerspective(); }
        GUILayout.EndHorizontal();

        showCameraAxesInCalibration = GUILayout.Toggle(showCameraAxesInCalibration, " Hiện trục XYZ Camera (X: Đỏ, Y: Xanh lá, Z: Xanh dương)", AntigravityUITheme.StyleToggle);

        if (rigCam != null)
        {
            bool prevRigVis = rigCam.showRuntimeRigVisual;
            bool prevFovVis = rigCam.showRuntimeFovFrustum;
            rigCam.showRuntimeRigVisual = GUILayout.Toggle(rigCam.showRuntimeRigVisual, " Hiện mô hình Camera Zivid & giá treo 3D", AntigravityUITheme.StyleToggle);
            if (rigCam.showRuntimeRigVisual)
            {
                rigCam.showRuntimeFovFrustum = GUILayout.Toggle(rigCam.showRuntimeFovFrustum, " Hiện vùng nhìn FOV & trục quang học", AntigravityUITheme.StyleToggle);
            }
            bool fovChanged = DrawCalibNumericSliderRow("FOV ống kính", ref rigCam.cameraFovDeg, 25f, 75f, 1f, 5f, "°");
            if (fovChanged)
            {
                rigCam.ApplyTransform();
            }
            if (fovChanged || rigCam.showRuntimeRigVisual != prevRigVis || rigCam.showRuntimeFovFrustum != prevFovVis)
            {
                rigCam.SaveRigVisualSettings();
            }
        }
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("1. VỊ TRÍ & GÓC CAMERA (ROBOT)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        bool poseChanged = false;
        poseChanged |= DrawCalibNumericSliderRow("Tx (X)", ref calib.camPosRobotX_mm, -500f, 1600f, 1f, 10f, "mm");
        poseChanged |= DrawCalibNumericSliderRow("Ty (Y)", ref calib.camPosRobotY_mm, -800f, 800f, 1f, 10f, "mm");
        poseChanged |= DrawCalibNumericSliderRow("Tz (Cao)", ref calib.camPosRobotZ_mm, 500f, 2500f, 1f, 10f, "mm");

        GUILayout.Space(2);
        poseChanged |= DrawCalibNumericSliderRow("Roll", ref calib.camRotRollDeg, -45f, 45f, 1f, 5f, "°");
        poseChanged |= DrawCalibNumericSliderRow("Pitch", ref calib.camRotPitchDeg, -45f, 45f, 1f, 5f, "°");
        poseChanged |= DrawCalibNumericSliderRow("Yaw", ref calib.camRotYawDeg, -180f, 180f, 1f, 15f, "°");

        GUILayout.Space(2);
        poseChanged |= DrawCalibNumericSliderRow("Z bàn", ref calib.tableHeightMm, -150f, 300f, 1f, 10f, "mm");
        poseChanged |= DrawCalibNumericSliderRow("Z gốc Rob", ref calib.robotGlobalOriginZ_mm, 500f, 1800f, 5f, 25f, "mm");

        if (poseChanged)
        {
            calib.RebuildMatrixFromPoseParams(syncScene: true, reloadPointCloud: false);
        }

        GUILayout.Space(4);
        bool prevUse = calib.useCalibratedTransform;
        calib.useCalibratedTransform = GUILayout.Toggle(calib.useCalibratedTransform, " Áp dụng ma trận 4x4 khi nạp .PLY", AntigravityUITheme.StyleToggle);
        if (calib.useCalibratedTransform != prevUse)
        {
            calib.SavePoseParamsToPrefs();
            if (plyLoader != null) plyLoader.ReloadFullPointCloud();
        }

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Áp dụng & Tải lại", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
        {
            calib.RebuildMatrixFromPoseParams(syncScene: true, reloadPointCloud: true);
            RunDiagnostics();
        }
        if (GUILayout.Button("Mặc định", AntigravityUITheme.StyleButton, GUILayout.Width(90), GUILayout.Height(26)))
        {
            calib.ResetToNominalRig(reloadCloud: true);
            RunDiagnostics();
        }
        GUILayout.EndHorizontal();

        float rigHeightCm = calib.camPosRobotZ_mm * 0.1f;
        float rigDistCm = Mathf.Abs(calib.robotGlobalOriginZ_mm - calib.camPosRobotX_mm) * 0.1f;
        GUILayout.Label($"Giá treo: Cao {rigHeightCm:F1} cm | Cách trục {rigDistCm:F1} cm | Lệch Y {calib.camPosRobotY_mm * 0.1f:F1} cm", AntigravityUITheme.StyleLabelMuted);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("2. MA TRẬN 4x4 (CAM -> ROBOT, mm)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        double[,] M = calib.GetCameraToRobotMatrix();
        for (int r = 0; r < 4; r++)
        {
            string rowStr = string.Format(CultureInfo.InvariantCulture,
                "[ {0,7:F4}  {1,7:F4}  {2,7:F4} | {3,7:F1} ]",
                M[r, 0], M[r, 1], M[r, 2], M[r, 3]);
            GUILayout.Label(rowStr, r < 3 ? AntigravityUITheme.StyleLabelCyan : AntigravityUITheme.StyleLabelMuted);
        }

        GUILayout.Space(3);
        string prevJsonPath = calib.calibrationJsonPath;
        calib.calibrationJsonPath = GUILayout.TextField(calib.calibrationJsonPath, AntigravityUITheme.StyleTextField);
        if (calib.calibrationJsonPath != prevJsonPath) calib.SavePoseParamsToPrefs();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Nạp JSON", AntigravityUITheme.StyleButton, GUILayout.Height(25)))
        {
            string resolved = calib.ResolveCalibrationPath(calib.calibrationJsonPath);
            calib.LoadCalibrationFromJson(resolved);
        }
        if (GUILayout.Button("Lưu JSON", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(25)))
        {
            calib.ExportCalibrationTemplateJson();
        }
        GUILayout.EndHorizontal();
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("3. KIỂM CHỨNG & BÙ LỆCH (CAM -> ROBOT)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        Vector3 prevVerifyCam = calib.verifyCameraPointMm;
        calib.verifyCameraPointMm = DrawVector3InputRow("Điểm Cam:", calib.verifyCameraPointMm);
        Vector3 livePredRob = calib.CameraMmToRobotMm(calib.verifyCameraPointMm);
        GUILayout.Label($"Quy đổi Robot: X={livePredRob.x:F1}  Y={livePredRob.y:F1}  Z={livePredRob.z:F1} mm", AntigravityUITheme.StyleLabelGreen);

        Vector3 prevVerifyRob = calib.verifyRobotTouchedMm;
        calib.verifyRobotTouchedMm = DrawVector3InputRow("TCP chạm:", calib.verifyRobotTouchedMm);
        if (calib.verifyCameraPointMm != prevVerifyCam || calib.verifyRobotTouchedMm != prevVerifyRob)
        {
            calib.SavePoseParamsToPrefs();
        }

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Lấy TCP", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            if (tcpTransform != null)
            {
                Vector3 tcpRobMm = globalReference != null
                    ? globalReference.InverseTransformPoint(tcpTransform.position) * 1000f
                    : tcpTransform.position * 1000f;
                calib.verifyRobotTouchedMm = tcpRobMm;
                calib.SavePoseParamsToPrefs();
            }
        }
        if (GUILayout.Button("Tính sai số", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            calib.RunTouchPointVerification();
        }
        if (GUILayout.Button("Bù tịnh tiến", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(24)))
        {
            calib.AutoCompensateTranslationFromTouchPoint();
        }
        GUILayout.EndHorizontal();
        GUILayout.Label(calib.touchVerificationSummary, AntigravityUITheme.StyleLabelCyan);
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("4. HIỆU CHUẨN ĐA ĐIỂM (>= 3 CẶP)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("+ Thêm cặp", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            calib.AddCurrentVerifyPairToList();
        }
        if (GUILayout.Button("Giải 4x4 (SVD)", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(24)))
        {
            calib.SolveRigidCalibrationFromPairs();
        }
        if (GUILayout.Button("Xóa", AntigravityUITheme.StyleButton, GUILayout.Width(50), GUILayout.Height(24)))
        {
            calib.calibrationPairs.Clear();
            calib.SavePoseParamsToPrefs();
            calib.multiPointCalibStatus = "Đã xóa danh sách.";
        }
        GUILayout.EndHorizontal();

        GUILayout.Label(calib.multiPointCalibStatus, AntigravityUITheme.StyleLabelCyan);
        for (int i = 0; i < calib.calibrationPairs.Count; i++)
        {
            var pr = calib.calibrationPairs[i];
            GUILayout.BeginHorizontal();
            GUILayout.Label($"{pr.label}: ({pr.cameraMm.x:F0},{pr.cameraMm.y:F0},{pr.cameraMm.z:F0}) -> ({pr.robotMm.x:F0},{pr.robotMm.y:F0},{pr.robotMm.z:F0}) | {pr.residualMm:F1}mm",
                AntigravityUITheme.StyleLabelMuted);
            if (GUILayout.Button("X", AntigravityUITheme.StyleCloseButton, GUILayout.Width(20), GUILayout.Height(18)))
            {
                calib.calibrationPairs.RemoveAt(i);
                calib.SavePoseParamsToPrefs();
                break;
            }
            GUILayout.EndHorizontal();
        }
        GUILayout.EndVertical();

        GUILayout.Space(6);
        GUILayout.Label("5. NGÂN SÁCH SAI SỐ (<= 5 mm)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        bool budgetChanged = false;
        budgetChanged |= DrawCalibNumericSliderRow("Cam-Bàn", ref calib.cameraToTableRmsMm, 0f, 5f, 0.1f, 0.5f, "mm");
        budgetChanged |= DrawCalibNumericSliderRow("Tay-Mắt t", ref calib.handEyeTransErrorMm, 0f, 5f, 0.1f, 0.5f, "mm");
        budgetChanged |= DrawCalibNumericSliderRow("Tay-Mắt r", ref calib.handEyeRotErrorDeg, 0f, 3f, 0.1f, 0.5f, "°");
        budgetChanged |= DrawCalibNumericSliderRow("TCP", ref calib.tcpCalibrationErrorMm, 0f, 5f, 0.1f, 0.5f, "mm");
        budgetChanged |= DrawCalibNumericSliderRow("Biên AI", ref calib.garmentEdgeErrorMm, 0f, 5f, 0.1f, 0.5f, "mm");
        if (budgetChanged) calib.SavePoseParamsToPrefs();

        float rss = calib.GetTotalErrorBudgetRssMm();
        float wc = calib.GetTotalErrorBudgetWorstCaseMm();
        GUILayout.Label($"RSS = {rss:F2} mm | Xấu nhất = {wc:F2} mm",
            rss <= 5f ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelRed);
        GUILayout.EndVertical();

        GUILayout.EndVertical();
    }
}
