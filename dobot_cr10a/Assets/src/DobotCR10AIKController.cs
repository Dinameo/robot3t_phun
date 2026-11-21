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
    public string activeTrajectoryMethod = "P4"; // Phương pháp đang chọn: "P1", "P3", "P4" (hoặc "CUSTOM" khi vẽ tay)
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
    public class Region2DJson
    {
        public int shapeType = 3;
        public float brushWidthMm = 4.0f;
        public float centerU = 0f;
        public float centerV = 0f;
        public float radiusU = 0f;
        public float radiusV = 0f;
        public List<StrokePoint2D> points = new List<StrokePoint2D>();
    }

    [Serializable]
    public class SprayPatternTemplate
    {
        public string templateName = "mau_phun_custom";
        public string createdDate = "";
        public string size = "S";
        public float zOffsetMm = 30f;
        public float sprayWidthMm = 4.0f;
        public List<Stroke2D> strokes = new List<Stroke2D>();
        public List<Region2DJson> regions = new List<Region2DJson>();
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
    public bool showSprayZones3D = true;        // Bật/Tắt hiển thị CHỖ CẦN PHUN (Vùng R & R_safe) trong View 2D & 3D
    public bool showTurnaroundConnectors = true; // Bật/Tắt hiển thị đoạn nối/quay đầu TẮT SÚNG (SPRAY OFF)
    public bool showP1ParameterPanel = false;    // Mở bảng thông số tùy chỉnh thuật toán P1 khi bấm P1
    public bool p1AutoRegenerateOnChange = true; // Tự sinh lại quỹ đạo P1 ngay khi kéo chỉnh thông số
    public TrajectoryEvaluationEngine.P1ZigzagConfig p1Config = new TrajectoryEvaluationEngine.P1ZigzagConfig();
    private TrajectoryEvaluationEngine.P1ExecutionResult lastP1Result = null;
    public bool showP2ParameterPanel = false;    // Mở bảng thông số tùy chỉnh thuật toán P2 khi bấm P2
    public bool p2AutoRegenerateOnChange = true; // Tự sinh lại quỹ đạo P2 ngay khi kéo chỉnh thông số
    public bool showP2SkeletonAxis = true;       // Bật/Tắt hiển thị trục giữa ống quần (Skeleton P2)
    public TrajectoryEvaluationEngine.P2SkeletonConfig p2Config = new TrajectoryEvaluationEngine.P2SkeletonConfig();
    private TrajectoryEvaluationEngine.P2ExecutionResult lastP2Result = null;
    public bool showP3ParameterPanel = false;    // Mở bảng thông số tùy chỉnh thuật toán P3 khi bấm P3
    public bool p3AutoRegenerateOnChange = true; // Tự sinh lại quỹ đạo P3 ngay khi chỉnh thông số
    public bool showP3TriangulationMesh = true;  // Bật/Tắt hiển thị lưới 32 tam giác Barycentric (P3)
    public TrajectoryEvaluationEngine.P3LandmarkConfig p3Config = new TrajectoryEvaluationEngine.P3LandmarkConfig();
    private TrajectoryEvaluationEngine.P3ExecutionResult lastP3Result = null;
    public bool showP4ParameterPanel = false;    // Mở bảng thông số tùy chỉnh thuật toán P4 khi bấm P4
    public bool p4AutoRegenerateOnChange = true; // Tự sinh lại quỹ đạo P4 ngay khi chỉnh thông số
    public bool showP4RegisteredCloud3D = true;  // Bật/Tắt hiển thị đám mây điểm mẫu 3D sau đăng ký (P4)
    public TrajectoryEvaluationEngine.P4RegistrationConfig p4Config = new TrajectoryEvaluationEngine.P4RegistrationConfig();
    private TrajectoryEvaluationEngine.P4ExecutionResult lastP4Result = null;

    // Bảng đánh giá toàn màn hình đầy đủ 12 chỉ tiêu + Ma trận quyết định 2 bước + Độ nhạy ±10% (Trang 8–12 file góp ý)
    private bool showFullscreenEvaluationReport = false;
    private Vector2 fullscreenEvalScroll = Vector2.zero;
    private TrajectoryEvaluationEngine.SensitivityAnalysisReport lastSensitivityReport = null;
    private float[] evalSetupTimeMin = new float[] { 0.3f, 0.5f, 2.0f, 3.0f };       // t_setup thực đo (phút) cho P1..P4
    private float[] evalEngineeringScores = new float[] { 4.5f, 4.2f, 4.0f, 3.5f };  // Điểm công cài đặt & bảo trì (1–5) cho P1..P4

    // Dữ liệu Đám mây điểm 3D chuẩn cho P4 (Mức D trang 5 & 8: lưu đám mây điểm 3D quần chuẩn + quỹ đạo mẫu 3D)
    public List<Vector3> templateReferenceCloud3DWorld = new List<Vector3>();
    public List<Vector3> templateReferenceLandmarks3D = new List<Vector3>();
    public List<List<Vector3>> templateReferenceStrokes3DWorld = new List<List<Vector3>>();
    public string templateReferenceSampleName = "";

    // Ground Truth thực nghiệm (Trang 9: quỹ đạo dạy tay/vẽ tay hoặc nạp từ file marker ngoài — KHÔNG lấy P4 tự chấm!)
    public List<WaypointData> groundTruthWaypoints = new List<WaypointData>();
    public string groundTruthSampleName = "";
    public string groundTruthSourceLabel = "";
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
    public string customDrawStatusMsg = "Vẽ vùng/nét trên ảnh 2D, sau đó chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
    private Texture2D canonicalJeansBgTex;
    private Texture2D realCameraFileTex;
    private string realCameraLoadedSample = "";
    private string explicitRealCameraImagePath = "";
    private bool canvasUsingRealCamera = false;
    private Vector2[] strokesReferenceLandmarksUV = null;
    private string strokesProjectedSampleName = "";
    private Texture2D whitePixelTex;
    private readonly List<List<Vector2>> customDrawnStrokes = new List<List<Vector2>>();
    private readonly List<List<Vector2>> canonicalPatternStrokes = new List<List<Vector2>>();
    // Tách biệt các lớp hiển thị trên 2D Canvas: CHỖ CẦN PHUN vs ĐƯỜNG QUỸ ĐẠO vs ĐƯỜNG PHUN + Trục P2 / Lưới P3 / Đám mây P4
    private readonly List<List<Vector2>> projectedTargetRegionR2D = new List<List<Vector2>>();
    private readonly List<List<Vector2>> projectedSafeRegionRsafe2D = new List<List<Vector2>>();
    private readonly List<List<Vector2>> projectedP2Skeleton2D = new List<List<Vector2>>();
    private readonly List<Vector2[]> projectedP3MeshTriangles2D = new List<Vector2[]>();
    private readonly List<bool> projectedP3TriangleFlipped2D = new List<bool>();
    private readonly List<Vector2> projectedP4RegisteredCloud2D = new List<Vector2>();
    private readonly List<List<Vector2>> projectedTrajectoryOn2D = new List<List<Vector2>>();
    private readonly List<List<Vector2>> projectedTrajectoryOff2D = new List<List<Vector2>>();
    private List<Vector2> currentDrawingStroke = null;
    private bool isDrawingOnCanvas = false;
    private bool hasInitializedDefaultStrokes = false;

    public enum CanvasDrawTool
    {
        Freehand = 0,
        Circle = 1,
        Rectangle = 2,
        Polygon = 3,
        Select = 4
    }

    private enum RegionEditMode
    {
        None = 0,
        MoveRegion = 1,
        ResizeCircleRadius = 2,
        ResizeRectHandle = 3,
        MovePolygonVertex = 4,
        ScalePolygonCorner = 5
    }

    public class DrawnRegion2D
    {
        public CanvasDrawTool shapeType = CanvasDrawTool.Polygon;
        public float brushWidthMm = 4.0f;
        public Vector2 centerUV = Vector2.zero;
        public Vector2 radiusUV = Vector2.zero;
        public Rect rectUV = Rect.zero;
        public List<Vector2> verticesUV = new List<Vector2>();
    }

    private CanvasDrawTool activeDrawTool = CanvasDrawTool.Freehand;
    private bool isDraggingShape = false;
    private bool draggingOnFullscreen = false;
    private Vector2 shapeStartUV = Vector2.zero;
    private Vector2 shapeCurrentUV = Vector2.zero;
    private readonly List<Vector2> activePolygonVerticesUV = new List<Vector2>();
    private readonly List<DrawnRegion2D> customDrawnRegions = new List<DrawnRegion2D>();
    private readonly List<bool> undoHistoryIsRegion = new List<bool>();
    private readonly List<float> scanlineScratchXs = new List<float>(32);
    private Texture2D smoothCircleFillTex;
    private Texture2D smoothCircleRingTex;
    private int activeCanvasControlId = 0;
    private int selectedRegionIndex = -1;
    private RegionEditMode activeRegionEditMode = RegionEditMode.None;
    private int activeEditHandleIndex = -1; // Rect: 0=TL,1=T,2=TR,3=R,4=BR,5=B,6=BL,7=L | PolyCorner: 0=TL,1=TR,2=BR,3=BL | PolyVertex: vertex index
    private Vector2 editDragStartMouseUV = Vector2.zero;
    private Vector2 editOrigCenterUV = Vector2.zero;
    private Rect editOrigRectUV = Rect.zero;
    private List<Vector2> editOrigVerticesUV = null;
    private bool lastPolygonHasRealLandmarks = false;
    private Vector2[] lastPolygonLandmarksUV = null;
    private bool layoutShowPolygonBar = false;
    private bool layoutShowSelectedRegionBar = false;

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
        public string activeTrajectoryMethod = "P4";
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
                activeTrajectoryMethod = string.IsNullOrEmpty(activeTrajectoryMethod) ? "P4" : activeTrajectoryMethod,
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
                    sprayWidthMm = Mathf.Clamp(data.sprayWidthMm, 0.5f, 50f);
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
                    customDrawZOffsetMm = Mathf.Clamp(data.customDrawZOffsetMm, 0f, 250f);
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
                        sprayTechParams.standoffMm = customDrawZOffsetMm;
                    }
                    if (!string.IsNullOrEmpty(data.activeTrajectoryMethod))
                    {
                        activeTrajectoryMethod = data.activeTrajectoryMethod;
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
        sprayWidthMm = Mathf.Clamp(widthMm, 0.5f, 50.0f);
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

        // Chế độ nội suy vận tốc TCP đều (Cartesian Linear Path) trên đoạn phun & đoạn quét Zigzag liên tục kiểu máy in 3D:
        // Giữ v_TCP = const (không dừng giật tại mỗi waypoint), chỉ bật súng khi waypoint.sprayOn == true!
        if (useConstantTcpSpeed && paintingStarted && index > 0)
        {
            WaypointData prevWp = waypoints[index - 1];
            float segLenMm = Vector3.Distance(prevWp.targetPosGlobal_mm, waypoint.targetPosGlobal_mm);
            bool isSameSegShortMove = !waypoint.sprayOn && segLenMm >= 0.5f && segLenMm <= 80f &&
                (string.IsNullOrEmpty(prevWp.segmentName) || string.IsNullOrEmpty(waypoint.segmentName) || prevWp.segmentName == waypoint.segmentName);
            if ((waypoint.sprayOn && segLenMm >= 0.5f && segLenMm <= 250f) || isSameSegShortMove)
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

        // Chỉ nạp file CSV dự phòng nếu chưa sinh được quỹ đạo từ mẫu/phương pháp hiện tại
        if (waypoints == null || waypoints.Count == 0)
        {
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

        // Chỉ nạp file quỹ đạo dự phòng nếu chưa sinh được quỹ đạo từ mẫu/phương pháp hiện tại
        if (waypoints == null || waypoints.Count == 0)
        {
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
        rx -= 114f;
        if (GUI.Button(new Rect(rx, btnY, 108f, btnH), showSprayZones3D ? "Vùng phun: Bật" : "Vùng phun: Tắt",
            showSprayZones3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton))
        {
            showSprayZones3D = !showSprayZones3D;
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

        if (showDockPanel && !showFullscreen2DCanvas)
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

        if (showFullscreen2DCanvas)
        {
            DrawFullscreen2DCanvasOverlay(totalHeaderH);
        }
        else if (showFullscreenEvaluationReport)
        {
            DrawFullscreenEvaluationReportOverlay(totalHeaderH);
        }

        WorkbenchDockVisible = showDockPanel && !showFullscreen2DCanvas && !showFullscreenEvaluationReport;
        WorkbenchDockWidth = (showDockPanel && !showFullscreen2DCanvas && !showFullscreenEvaluationReport) ? singleMainPanel.rect.width : 0f;

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
            float newWidth = GUILayout.HorizontalSlider(sprayWidthMm, 1f, 50f);
            if (Mathf.Abs(newWidth - sprayWidthMm) > 0.1f) { UpdateSprayWidth(newWidth); MarkSettingsDirty(); }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("2 mm", Mathf.Abs(sprayWidthMm - 2f) < 0.2f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) { UpdateSprayWidth(2.0f); MarkSettingsDirty(); }
            if (GUILayout.Button("5 mm", Mathf.Abs(sprayWidthMm - 5f) < 0.3f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) { UpdateSprayWidth(5.0f); MarkSettingsDirty(); }
            if (GUILayout.Button("15 mm", Mathf.Abs(sprayWidthMm - 15f) < 0.5f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(20))) { UpdateSprayWidth(15.0f); MarkSettingsDirty(); }
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
        GUILayout.Label("1. SINH QUỸ ĐẠO (P1 / P2 / P3 / P4)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("P1: Zigzag", activeTrajectoryMethod == "P1" ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            showP1ParameterPanel = true;
            showP2ParameterPanel = false;
            showP3ParameterPanel = false;
            showP4ParameterPanel = false;
            SyncP1ConfigFromTechParamsIfNeeded();
            GenerateAndApplyMethodTrajectory("P1");
        }
        if (GUILayout.Button("P2: Trục giữa", activeTrajectoryMethod == "P2" ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            showP1ParameterPanel = false;
            showP2ParameterPanel = true;
            showP3ParameterPanel = false;
            showP4ParameterPanel = false;
            SyncP2ConfigFromTechParamsIfNeeded();
            GenerateAndApplyMethodTrajectory("P2");
        }
        if (GUILayout.Button("P3: 12 Mốc", activeTrajectoryMethod == "P3" ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            showP1ParameterPanel = false;
            showP2ParameterPanel = false;
            showP3ParameterPanel = true;
            showP4ParameterPanel = false;
            SyncP3ConfigFromTechParamsIfNeeded();
            GenerateAndApplyMethodTrajectory("P3");
        }
        if (GUILayout.Button("P4: ICP/CPD", activeTrajectoryMethod == "P4" ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            showP1ParameterPanel = false;
            showP2ParameterPanel = false;
            showP3ParameterPanel = false;
            showP4ParameterPanel = true;
            SyncP4ConfigFromTechParamsIfNeeded();
            GenerateAndApplyMethodTrajectory("P4");
        }
        GUILayout.EndHorizontal();

        if (activeTrajectoryMethod == "P1" || showP1ParameterPanel)
        {
            DrawP1ParameterConfigSection();
        }
        if (activeTrajectoryMethod == "P2" || showP2ParameterPanel)
        {
            DrawP2ParameterConfigSection();
        }
        if (activeTrajectoryMethod == "P3" || showP3ParameterPanel)
        {
            DrawP3ParameterConfigSection();
        }
        if (activeTrajectoryMethod == "P4" || showP4ParameterPanel)
        {
            DrawP4ParameterConfigSection();
        }

        if (GUILayout.Button("So sánh P1 / P2 / P3 / P4", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
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

    public void ClearPreviousWorkpieceVisualArtifacts()
    {
        StopPlaybackForNewFrame();
        ClearInkTrail();
        if (waypoints != null) waypoints.Clear();
        currentWaypointIndex = 0;

        landmarks3D.Clear();
        landmarksWorldPos.Clear();
        garmentContourWorld.Clear();
        sprayZoneLeftWorld.Clear();
        sprayZoneRightWorld.Clear();
        loadedLandmarksSampleName = "";
        strokesProjectedSampleName = "";
        strokesReferenceLandmarksUV = null;

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
        comparisonResults.Clear();

        if (inpaintedMeshObject != null)
        {
            inpaintedMeshObject.transform.SetParent(null);
            inpaintedMeshObject.SetActive(false);
            if (Application.isPlaying) Destroy(inpaintedMeshObject);
            else DestroyImmediate(inpaintedMeshObject);
            inpaintedMeshObject = null;
        }
        if (depositionHeatmapObj != null)
        {
            depositionHeatmapObj.transform.SetParent(null);
            depositionHeatmapObj.SetActive(false);
            if (Application.isPlaying) Destroy(depositionHeatmapObj);
            else DestroyImmediate(depositionHeatmapObj);
            depositionHeatmapObj = null;
        }
    }

    public void ReloadPointCloudFromPath(string path)
    {
        ClearPreviousWorkpieceVisualArtifacts();
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
        GUILayout.Label("3. CÁC LỚP HIỂN THỊ (CHỖ CẦN PHUN & QUỸ ĐẠO)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        // Phân biệt rạch ròi 3 lớp hiển thị độc lập: Chỗ cần phun vs Đường quỹ đạo (quét/tắt súng) vs Đường phun (bật súng)
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(showSprayZones3D ? "Chỗ cần phun: BẬT" : "Chỗ cần phun: TẮT",
            showSprayZones3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showSprayZones3D = !showSprayZones3D;
            MarkSettingsDirty();
        }
        if (GUILayout.Button(showTurnaroundConnectors ? "Đường quỹ đạo: BẬT" : "Đường quỹ đạo: TẮT",
            showTurnaroundConnectors ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showTurnaroundConnectors = !showTurnaroundConnectors;
        }
        if (GUILayout.Button(showGizmos ? "Đường phun: BẬT" : "Đường phun: TẮT",
            showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showGizmos = !showGizmos;
            MarkSettingsDirty();
        }
        GUILayout.EndHorizontal();

        showSprayZones3D = GUILayout.Toggle(showSprayZones3D, " Chỗ cần phun (Mẫu vẽ / Vùng R: Vàng cam | R_safe: Xanh lá)", AntigravityUITheme.StyleToggle);
        showTurnaroundConnectors = GUILayout.Toggle(showTurnaroundConnectors, " Đường quỹ đạo (Quét qua lại & quay đầu — TẮT súng: Đứt cam)", AntigravityUITheme.StyleToggle);
        showGizmos = GUILayout.Toggle(showGizmos, " Đường phun (Đoạn BẬT súng phun lên mẫu: Cyan nét liền)", AntigravityUITheme.StyleToggle);
        showP2SkeletonAxis = GUILayout.Toggle(showP2SkeletonAxis, " Trục giữa ống quần Skeleton (P2: Hồng tím)", AntigravityUITheme.StyleToggle);
        showP3TriangulationMesh = GUILayout.Toggle(showP3TriangulationMesh, " Lưới 32 tam giác mốc Barycentric (P3: Xanh ngọc)", AntigravityUITheme.StyleToggle);
        showP4RegisteredCloud3D = GUILayout.Toggle(showP4RegisteredCloud3D, " Đám mây điểm 3D đã đăng ký (P4: Xanh chanh)", AntigravityUITheme.StyleToggle);

        GUILayout.Space(3);
        showLandmarks3D = GUILayout.Toggle(showLandmarks3D, " 12 mốc AI & khung xương", AntigravityUITheme.StyleToggle);
        if (showLandmarks3D) showLandmarkLabels = GUILayout.Toggle(showLandmarkLabels, " Nhãn tên & cao độ Z", AntigravityUITheme.StyleToggle);
        showSegmentationContour3D = GUILayout.Toggle(showSegmentationContour3D, " Đường bao thân quần", AntigravityUITheme.StyleToggle);
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

    public string GetActiveCloudSampleName()
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

    private static Vector2 WarpUV2D(Vector2 uv, Vector2[] src12, Vector2[] dst12, float srcAspectXOverY = 1f, float dstAspectXOverY = 1f)
    {
        if (src12 == null || dst12 == null || src12.Length < 12 || dst12.Length < 12) return uv;
        Vector2 res = TrajectoryEvaluationEngine.WarpPointBarycentric12(uv, src12, dst12, srcAspectXOverY, dstAspectXOverY);
        return new Vector2(Mathf.Clamp01(res.x), Mathf.Clamp01(res.y));
    }

    private float GetActiveCameraMetricAspect(PLYDirectLoader plyLoader, int nativeW, int nativeH)
    {
        if (nativeW > 0 && nativeH > 0)
        {
            return (float)nativeW / nativeH;
        }
        if (plyLoader != null && plyLoader.cameraRgbTexture != null && plyLoader.cameraRgbTexture.height > 0)
        {
            return (float)plyLoader.cameraRgbTexture.width / plyLoader.cameraRgbTexture.height;
        }
        return 1.62f;
    }

    private void CaptureCurrentStrokesToCanonical()
    {
        canonicalPatternStrokes.Clear();
        if (customDrawnStrokes == null || customDrawnStrokes.Count == 0) return;

        Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
        bool usingRealCam = (camTex != null);
        Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, usingRealCam, out bool hasRealLm);
        Vector2[] canonLmUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        float camAspect = GetActiveCameraMetricAspect(plyLoader, nativeW, nativeH);

        bool has3DLm = (landmarksWorldPos != null && landmarksWorldPos.Count == 12);
        Vector2[] worldLmXZ = null;
        if (has3DLm)
        {
            worldLmXZ = new Vector2[12];
            for (int i = 0; i < 12; i++) worldLmXZ[i] = new Vector2(landmarksWorldPos[i].x, landmarksWorldPos[i].z);
        }

        for (int s = 0; s < customDrawnStrokes.Count; s++)
        {
            var src = customDrawnStrokes[s];
            if (src == null || src.Count == 0) continue;
            var dst = new List<Vector2>(src.Count);
            for (int p = 0; p < src.Count; p++)
            {
                Vector2 pt = src[p];
                if (usingRealCam)
                {
                    if (has3DLm && plyLoader != null && plyLoader.TrySampleCameraUVToWorld(pt, out Vector3 surfW, out _))
                    {
                        Vector2 res = TrajectoryEvaluationEngine.WarpPointBarycentric12(new Vector2(surfW.x, surfW.z), worldLmXZ, canonLmUV, 1f, 1f);
                        pt = new Vector2(Mathf.Clamp01(res.x), Mathf.Clamp01(res.y));
                    }
                    else if (hasRealLm && camLmUV != null)
                    {
                        pt = WarpUV2D(pt, camLmUV, canonLmUV, camAspect, 1f);
                    }
                }
                dst.Add(pt);
            }
            canonicalPatternStrokes.Add(dst);
        }
        if (hasRealLm && camLmUV != null)
        {
            strokesReferenceLandmarksUV = (Vector2[])camLmUV.Clone();
            strokesProjectedSampleName = GetActiveCloudSampleName();
        }

        // Xóa cache mẫu 3D cũ của P4 để không bao giờ phát lại nét vẽ cũ sau khi người dùng sửa/vẽ nét mới!
        templateReferenceStrokes3DWorld.Clear();
        if (plyLoader != null && customDrawnStrokes.Count > 0)
        {
            var refCloud = plyLoader.ExtractGarmentSurfaceCloudVoxelized(0.005f, garmentContourWorld, 520);
            if (refCloud != null && refCloud.Count >= 30)
            {
                var liftedStrokes = new List<List<Vector3>>();
                for (int s = 0; s < customDrawnStrokes.Count; s++)
                {
                    var st = customDrawnStrokes[s];
                    if (st == null || st.Count < 2) continue;
                    var resamp = ResampleStroke2D(st, 0.018f);
                    var st3D = new List<Vector3>(resamp.Count);
                    for (int p = 0; p < resamp.Count; p++)
                    {
                        if (plyLoader.TrySampleCameraUVToWorld(resamp[p], out Vector3 surfW, out _))
                            st3D.Add(surfW);
                    }
                    if (st3D.Count >= 2) liftedStrokes.Add(st3D);
                }
                if (liftedStrokes.Count > 0)
                {
                    templateReferenceCloud3DWorld = refCloud;
                    templateReferenceStrokes3DWorld = liftedStrokes;
                    templateReferenceSampleName = GetActiveCloudSampleName();
                    templateReferenceLandmarks3D = (landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
                        ? new List<Vector3>(landmarksWorldPos)
                        : new List<Vector3>();
                }
            }
        }
    }

    private bool ReprojectCanonicalPatternOntoCurrentCameraImage()
    {
        if (canonicalPatternStrokes == null || canonicalPatternStrokes.Count == 0) return false;

        Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
        canvasUsingRealCamera = (camTex != null);
        Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, canvasUsingRealCamera, out bool hasRealLm);
        if (canvasUsingRealCamera && (!hasRealLm || camLmUV == null)) return false;

        Vector2[] canonLmUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        float camAspect = GetActiveCameraMetricAspect(plyLoader, nativeW, nativeH);

        bool use3DMetricProjection = (canvasUsingRealCamera && realCameraFileTex == null &&
            plyLoader != null && (plyLoader.hasTopDownCameraTexture || plyLoader.hasOrganizedCameraFrame) &&
            landmarksWorldPos != null && landmarksWorldPos.Count == 12);
        Vector2[] worldLmXZ = null;
        if (use3DMetricProjection)
        {
            worldLmXZ = new Vector2[12];
            for (int i = 0; i < 12; i++) worldLmXZ[i] = new Vector2(landmarksWorldPos[i].x, landmarksWorldPos[i].z);
        }

        customDrawnStrokes.Clear();
        for (int s = 0; s < canonicalPatternStrokes.Count; s++)
        {
            var src = canonicalPatternStrokes[s];
            if (src == null || src.Count == 0) continue;
            var dst = new List<Vector2>(src.Count);
            for (int p = 0; p < src.Count; p++)
            {
                Vector2 canPt = src[p];
                if (!canvasUsingRealCamera)
                {
                    dst.Add(canPt);
                }
                else if (use3DMetricProjection)
                {
                    Vector2 wXZ = TrajectoryEvaluationEngine.WarpPointBarycentric12(canPt, canonLmUV, worldLmXZ, 1f, 1f);
                    float surfY = 0.02f;
                    if (plyLoader.SampleSurfaceWorldHeightAndNormal(wXZ.x, wXZ.y, out float cy, out _)) surfY = cy;
                    if (plyLoader.TryWorldToCameraUV(new Vector3(wXZ.x, surfY, wXZ.y), out Vector2 camUv))
                        dst.Add(camUv);
                    else
                        dst.Add(WarpUV2D(canPt, canonLmUV, camLmUV, 1f, camAspect));
                }
                else
                {
                    dst.Add(WarpUV2D(canPt, canonLmUV, camLmUV, 1f, camAspect));
                }
            }
            customDrawnStrokes.Add(dst);
        }

        if (canvasUsingRealCamera && camLmUV != null)
        {
            strokesReferenceLandmarksUV = (Vector2[])camLmUV.Clone();
            strokesProjectedSampleName = GetActiveCloudSampleName();
        }
        return true;
    }

    private void WarpStrokesInPlace(List<List<Vector2>> strokes, Vector2[] src12, Vector2[] dst12, float srcAspectXOverY = 1f, float dstAspectXOverY = 1f)
    {
        if (strokes == null || src12 == null || dst12 == null || src12.Length < 12 || dst12.Length < 12) return;
        for (int s = 0; s < strokes.Count; s++)
        {
            var st = strokes[s];
            if (st == null) continue;
            for (int p = 0; p < st.Count; p++)
            {
                st[p] = WarpUV2D(st[p], src12, dst12, srcAspectXOverY, dstAspectXOverY);
            }
        }
    }

    public bool showFullscreen2DCanvas = false;

    private void DrawCanonicalJeans2DCanvasSection(float width)
    {
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        GUILayout.Label("VẼ MẪU PHUN 2D -> 3D", AntigravityUITheme.StyleSectionLabel);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Phóng to", AntigravityUITheme.StylePrimaryButton, GUILayout.Width(72), GUILayout.Height(20)))
        {
            showCanonicalDrawPanel = true;
            showFullscreen2DCanvas = true;
        }
        AiServerClient camClient = EnsureAiClient();
        bool prevGuiEnabled = GUI.enabled;
        GUI.enabled = prevGuiEnabled && camClient != null && !camClient.isProcessing;
        if (GUILayout.Button("Chụp cam", AntigravityUITheme.StyleButton, GUILayout.Width(72), GUILayout.Height(20)))
        {
            camClient.CapturePreview();
        }
        GUI.enabled = prevGuiEnabled;
        if (GUILayout.Button(showCanonicalDrawPanel ? "Thu gọn" : "Mở", AntigravityUITheme.StyleButton, GUILayout.Width(64), GUILayout.Height(20)))
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

        // Tự động ánh xạ lại mẫu vẽ / phương pháp đang chọn theo 12 mốc thực của chiếc quần mới mỗi khi đổi phôi!
        string activeSampleNow = GetActiveCloudSampleName();
        if (canvasUsingRealCamera && !isDrawingOnCanvas && !isDraggingShape && activePolygonVerticesUV.Count == 0 && preserveCustomDrawingAcrossSamples && canonicalPatternStrokes.Count > 0)
        {
            if (hasRealLandmarks && curLandmarksUV != null && strokesProjectedSampleName != activeSampleNow)
            {
                if (activeTrajectoryMethod == "P1" || activeTrajectoryMethod == "P2" || activeTrajectoryMethod == "P3" || activeTrajectoryMethod == "P4")
                {
                    GenerateAndApplyMethodTrajectory(activeTrajectoryMethod);
                }
                else
                {
                    ReprojectCanonicalPatternOntoCurrentCameraImage();
                }
            }
            else if (!hasRealLandmarks && !string.IsNullOrEmpty(strokesProjectedSampleName) && strokesProjectedSampleName != activeSampleNow)
            {
                // Phôi mới chưa có 12 mốc -> ẩn nét cũ khỏi vị trí sai, giữ canonicalPatternStrokes để tự khớp ngay khi có mốc
                customDrawnStrokes.Clear();
                strokesReferenceLandmarksUV = null;
                strokesProjectedSampleName = "";
            }
        }

        string methodBadge = activeTrajectoryMethod == "P1" ? " | Đang xem: P1 (Zigzag)"
            : (activeTrajectoryMethod == "P2" ? " | Đang xem: P2 (Trục giữa)"
            : (activeTrajectoryMethod == "P3" ? " | Đang xem: P3 (12 Mốc Barycentric)"
            : (activeTrajectoryMethod == "P4" ? " | Đang xem: P4 (ICP/TPS)" : " | Đang xem: Vẽ tay")));
        GUILayout.Label(camSourceLabel + methodBadge, canvasUsingRealCamera ? AntigravityUITheme.StyleLabelCyan : AntigravityUITheme.StyleLabelMuted);

        // Thanh công cụ chọn kiểu vẽ (Vẽ tay / Tròn / Vuông / Đa giác) & chỉnh cỡ cọ (mm)
        Draw2DCanvasBrushAndShapeToolbar(isFullscreen: false);

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

        // Xử lý sự kiện chuột vẽ trực tiếp (Vẽ tay / Hình tròn / Hình vuông / Đa giác) trên ảnh Camera 2D
        Handle2DCanvasMouseInput(canvasRect, isFullscreen: false, hasRealLandmarks, curLandmarksUV);

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

        // Vẽ tách biệt 2 lớp trên ảnh Camera 2D: (1) CHỖ CẦN PHUN (Vùng R & R_safe) và (2) ĐƯỜNG QUỸ ĐẠO (ON / OFF)
        DrawTargetAndTrajectoryOverlaysOn2DCanvas(canvasRect, isFullscreen: false);
        Draw2DCanvasToolPreview(canvasRect, isFullscreen: false);

        GUILayout.Space(4);
        // Nút bật/tắt độc lập 3 lớp hiển thị: Chỗ cần phun vs Đường quỹ đạo (quét/tắt súng) vs Đường phun (bật súng)
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(showSprayZones3D ? "Chỗ phun: BẬT" : "Chỗ phun: TẮT",
            showSprayZones3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showSprayZones3D = !showSprayZones3D;
            MarkSettingsDirty();
        }
        if (GUILayout.Button(showTurnaroundConnectors ? "Quỹ đạo: BẬT" : "Quỹ đạo: TẮT",
            showTurnaroundConnectors ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showTurnaroundConnectors = !showTurnaroundConnectors;
        }
        if (GUILayout.Button(showGizmos ? "Đường phun: BẬT" : "Đường phun: TẮT",
            showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showGizmos = !showGizmos;
            MarkSettingsDirty();
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(2);
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
                InvalidateGeneratedTrajectoryOnEdit();
                customDrawStatusMsg = "Đã nạp mẫu Râu mèo + Đùi. Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
            }
        }
        if (GUILayout.Button("Zigzag 2 ống", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            LoadPresetZigzag2LegsOn2DCanvas();
            if (customDrawnStrokes.Count > 0)
            {
                InvalidateGeneratedTrajectoryOnEdit();
                customDrawStatusMsg = "Đã nạp mẫu Zigzag 2 ống. Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
            }
        }
        if (GUILayout.Button("Hoàn tác", AntigravityUITheme.StyleButton, GUILayout.Width(68), GUILayout.Height(22)))
        {
            UndoLast2DCanvasStrokeOrShape();
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
            if (sprayTechParams != null) sprayTechParams.standoffMm = customDrawZOffsetMm;
            ReapplyActiveTrajectoryHeight();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("0 mm", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            customDrawZOffsetMm = 0f;
            if (sprayTechParams != null) sprayTechParams.standoffMm = customDrawZOffsetMm;
            ReapplyActiveTrajectoryHeight();
        }
        if (GUILayout.Button("20 mm", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            customDrawZOffsetMm = 20f;
            if (sprayTechParams != null) sprayTechParams.standoffMm = customDrawZOffsetMm;
            ReapplyActiveTrajectoryHeight();
        }
        if (GUILayout.Button(showGizmos ? "Ẩn đường 3D" : "Hiện đường 3D", showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showGizmos = !showGizmos;
        }
        GUILayout.EndHorizontal();

        if (!string.IsNullOrEmpty(customDrawStatusMsg))
        {
            GUILayout.Label(customDrawStatusMsg, AntigravityUITheme.StyleLabelGreen);
        }

        GUILayout.EndVertical();
    }

    private string Get2DPatternCountSummary()
    {
        int rCount = customDrawnRegions.Count;
        int sCount = customDrawnStrokes.Count;
        if (rCount > 0 && sCount > 0) return $"{rCount} vùng, {sCount} nét";
        if (rCount > 0) return $"{rCount} vùng";
        return $"{sCount} nét";
    }

    private void EnsureSmoothCircleTextures()
    {
        if (smoothCircleFillTex != null && smoothCircleRingTex != null) return;
        const int size = 128;
        float half = (size - 1) * 0.5f;
        float radius = half - 2.0f;
        float ringHalfW = 2.6f;

        smoothCircleFillTex = new Texture2D(size, size, TextureFormat.RGBA32, false)
        {
            wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear
        };
        smoothCircleRingTex = new Texture2D(size, size, TextureFormat.RGBA32, false)
        {
            wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear
        };

        Color[] fillPixels = new Color[size * size];
        Color[] ringPixels = new Color[size * size];
        for (int y = 0; y < size; y++)
        {
            float dy = y - half;
            for (int x = 0; x < size; x++)
            {
                float dx = x - half;
                float d = Mathf.Sqrt(dx * dx + dy * dy);
                float fillA = Mathf.Clamp01((radius - d) / 1.8f + 0.5f);
                float ringDist = Mathf.Abs(d - (radius - ringHalfW * 0.5f));
                float ringA = Mathf.Clamp01((ringHalfW - ringDist) / 1.5f);

                int idx = y * size + x;
                fillPixels[idx] = new Color(1f, 1f, 1f, fillA);
                ringPixels[idx] = new Color(1f, 1f, 1f, ringA);
            }
        }
        smoothCircleFillTex.SetPixels(fillPixels);
        smoothCircleFillTex.Apply(false, true);
        smoothCircleRingTex.SetPixels(ringPixels);
        smoothCircleRingTex.Apply(false, true);
    }

    private void DrawGuiRectOutline2D(Rect r, Color col, float thickness)
    {
        float t = Mathf.Max(1f, thickness);
        DrawGuiSolidRect(new Rect(r.x, r.y, r.width, t), col);
        DrawGuiSolidRect(new Rect(r.x, r.yMax - t, r.width, t), col);
        DrawGuiSolidRect(new Rect(r.x, r.y + t, t, Mathf.Max(0f, r.height - t * 2f)), col);
        DrawGuiSolidRect(new Rect(r.xMax - t, r.y + t, t, Mathf.Max(0f, r.height - t * 2f)), col);
    }

    private void DrawGuiSmoothCircle2D(Vector2 centerPx, float radiusX, float radiusY, Color fillCol, Color borderCol)
    {
        if (radiusX < 1f || radiusY < 1f) return;
        EnsureSmoothCircleTextures();
        Rect r = new Rect(centerPx.x - radiusX, centerPx.y - radiusY, radiusX * 2f, radiusY * 2f);
        Color prev = GUI.color;
        if (fillCol.a > 0.004f && smoothCircleFillTex != null)
        {
            GUI.color = fillCol;
            GUI.DrawTexture(r, smoothCircleFillTex);
        }
        if (borderCol.a > 0.004f && smoothCircleRingTex != null)
        {
            GUI.color = borderCol;
            GUI.DrawTexture(r, smoothCircleRingTex);
        }
        GUI.color = prev;
    }

    private void DrawGuiFilledPolygonScreen2D(List<Vector2> polyPx, Color fillCol, float stripHeightPx = 2f)
    {
        if (polyPx == null || polyPx.Count < 3 || fillCol.a <= 0.004f || whitePixelTex == null) return;
        float minY = float.MaxValue, maxY = float.MinValue;
        for (int i = 0; i < polyPx.Count; i++)
        {
            float y = polyPx[i].y;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
        }
        if (maxY - minY < 1f) return;

        float step = Mathf.Max(1.5f, stripHeightPx);
        Color prev = GUI.color;
        GUI.color = fillCol;
        int n = polyPx.Count;

        for (float y = minY + step * 0.5f; y < maxY; y += step)
        {
            scanlineScratchXs.Clear();
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                Vector2 a = polyPx[j];
                Vector2 b = polyPx[i];
                if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y))
                {
                    float t = (y - a.y) / (b.y - a.y);
                    scanlineScratchXs.Add(a.x + t * (b.x - a.x));
                }
            }
            if (scanlineScratchXs.Count < 2) continue;
            scanlineScratchXs.Sort();
            for (int k = 0; k + 1 < scanlineScratchXs.Count; k += 2)
            {
                float x0 = scanlineScratchXs[k];
                float x1 = scanlineScratchXs[k + 1];
                if (x1 - x0 >= 0.5f)
                {
                    GUI.DrawTexture(new Rect(x0, y - step * 0.5f, x1 - x0, step + 0.35f), whitePixelTex);
                }
            }
        }
        GUI.color = prev;
    }

    private void DrawDrawnRegion2D(DrawnRegion2D region, Rect canvasRect, bool isFullscreen, bool isSelected = false)
    {
        if (region == null) return;
        float borderW = isSelected ? (isFullscreen ? 3.0f : 2.3f) : (isFullscreen ? 2.4f : 1.9f);
        Color fillCol = isSelected
            ? new Color(1.0f, 0.86f, 0.18f, 0.20f)
            : new Color(0.10f, 0.88f, 1.0f, 0.22f);
        Color borderCol = isSelected
            ? new Color(1.0f, 0.90f, 0.22f, 0.98f)
            : new Color(0.18f, 0.96f, 1.0f, 0.96f);

        if (region.shapeType == CanvasDrawTool.Circle && region.radiusUV.x > 0.001f && region.radiusUV.y > 0.001f)
        {
            Vector2 centerPx = new Vector2(
                canvasRect.x + region.centerUV.x * canvasRect.width,
                canvasRect.y + region.centerUV.y * canvasRect.height
            );
            float rx = region.radiusUV.x * canvasRect.width;
            float ry = region.radiusUV.y * canvasRect.height;
            DrawGuiSmoothCircle2D(centerPx, rx, ry, fillCol, borderCol);
            return;
        }

        if (region.shapeType == CanvasDrawTool.Rectangle && region.rectUV.width > 0.001f && region.rectUV.height > 0.001f)
        {
            Rect rPx = new Rect(
                canvasRect.x + region.rectUV.x * canvasRect.width,
                canvasRect.y + region.rectUV.y * canvasRect.height,
                region.rectUV.width * canvasRect.width,
                region.rectUV.height * canvasRect.height
            );
            DrawGuiSolidRect(rPx, fillCol);
            DrawGuiRectOutline2D(rPx, borderCol, borderW);
            return;
        }

        if (region.verticesUV != null && region.verticesUV.Count >= 3)
        {
            List<Vector2> ptsPx = new List<Vector2>(region.verticesUV.Count);
            for (int i = 0; i < region.verticesUV.Count; i++)
            {
                ptsPx.Add(new Vector2(
                    canvasRect.x + region.verticesUV[i].x * canvasRect.width,
                    canvasRect.y + region.verticesUV[i].y * canvasRect.height
                ));
            }
            DrawGuiFilledPolygonScreen2D(ptsPx, fillCol, isFullscreen ? 2.0f : 2.5f);
            for (int i = 0; i < ptsPx.Count; i++)
            {
                DrawGuiLine2D(ptsPx[i], ptsPx[(i + 1) % ptsPx.Count], borderCol, borderW);
            }
        }
    }

    private float GetBrushDiameterPxOnCanvas(Rect canvasRect, float widthMm)
    {
        float scale = Mathf.Max(260f, canvasRect.width) / 480f;
        float px = 1.4f + Mathf.Max(0.5f, widthMm) * 0.55f * scale;
        return Mathf.Clamp(px, 2.0f, Mathf.Min(canvasRect.width, canvasRect.height) * 0.22f);
    }

    private void ReleaseCanvasHotControl()
    {
        if (activeCanvasControlId != 0 && GUIUtility.hotControl == activeCanvasControlId)
        {
            GUIUtility.hotControl = 0;
        }
        activeCanvasControlId = 0;
    }

    private void SetActiveCanvasDrawTool(CanvasDrawTool tool)
    {
        if (activeDrawTool == tool) return;
        activeDrawTool = tool;
        isDrawingOnCanvas = false;
        currentDrawingStroke = null;
        isDraggingShape = false;
        activeRegionEditMode = RegionEditMode.None;
        activeEditHandleIndex = -1;
        ReleaseCanvasHotControl();
        if (tool != CanvasDrawTool.Polygon)
        {
            activePolygonVerticesUV.Clear();
        }
    }

    private void DeleteSelectedRegion2D()
    {
        if (selectedRegionIndex < 0 || selectedRegionIndex >= customDrawnRegions.Count) return;
        customDrawnRegions.RemoveAt(selectedRegionIndex);
        selectedRegionIndex = Mathf.Clamp(selectedRegionIndex, -1, customDrawnRegions.Count - 1);
        activeRegionEditMode = RegionEditMode.None;
        activeEditHandleIndex = -1;
        ReleaseCanvasHotControl();
        customDrawStatusMsg = $"Đã xóa vùng chọn (còn {customDrawnRegions.Count} vùng). Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
        if (customDrawnRegions.Count == 0 && customDrawnStrokes.Count == 0)
        {
            ClearAll2DAnd3DTrajectoryLines();
        }
        else
        {
            InvalidateGeneratedTrajectoryOnEdit();
        }
    }

    private void Draw2DCanvasBrushAndShapeToolbar(bool isFullscreen)
    {
        float btnH = isFullscreen ? 25f : 22f;

        // Hàng 1: Chọn công cụ (Chọn/Sửa | Vẽ tay | Tròn | Vuông | Đa giác) — cấu trúc cố định tránh lệch layout IMGUI
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Sửa", activeDrawTool == CanvasDrawTool.Select ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(btnH)))
            SetActiveCanvasDrawTool(CanvasDrawTool.Select);
        if (GUILayout.Button("Vẽ tay", activeDrawTool == CanvasDrawTool.Freehand ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(btnH)))
            SetActiveCanvasDrawTool(CanvasDrawTool.Freehand);
        if (GUILayout.Button("Tròn", activeDrawTool == CanvasDrawTool.Circle ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(btnH)))
            SetActiveCanvasDrawTool(CanvasDrawTool.Circle);
        if (GUILayout.Button("Vuông", activeDrawTool == CanvasDrawTool.Rectangle ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(btnH)))
            SetActiveCanvasDrawTool(CanvasDrawTool.Rectangle);
        if (GUILayout.Button("Đa giác", activeDrawTool == CanvasDrawTool.Polygon ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(btnH)))
            SetActiveCanvasDrawTool(CanvasDrawTool.Polygon);
        GUILayout.EndHorizontal();

        // Hàng 2: Luôn hiển thị 1 dòng hướng dẫn cố định cho mọi công cụ (không gây nhảy layout khi bấm đổi công cụ!)
        string hint = activeDrawTool == CanvasDrawTool.Select
            ? "Click chọn vùng để di chuyển, kéo nút trắng để đổi kích thước (Delete = xóa)."
            : (activeDrawTool == CanvasDrawTool.Freehand
                ? "Kéo chuột trái trên ảnh để vẽ đường phun tự do."
                : (activeDrawTool == CanvasDrawTool.Circle
                    ? "Kéo từ tâm ra để vẽ hình tròn; kéo nút viền/tâm để chỉnh cỡ."
                    : (activeDrawTool == CanvasDrawTool.Rectangle
                        ? "Kéo chéo góc vẽ chữ nhật (Shift = vuông); kéo nút góc/cạnh để đổi cỡ."
                        : "Click trái thêm đỉnh | Click phải, Enter hoặc nối đỉnh đầu để đóng.")));
        GUILayout.Label(hint, AntigravityUITheme.StyleLabelMuted);

        // Hàng 2b: Snapshot trạng thái hiển thị ở pha Layout để Layout/Mouse/Repaint cùng 1 cấu trúc IMGUI (không nhảy tọa độ)
        if (Event.current != null && Event.current.type == EventType.Layout)
        {
            layoutShowPolygonBar = (activeDrawTool == CanvasDrawTool.Polygon && activePolygonVerticesUV.Count > 0);
            layoutShowSelectedRegionBar = !layoutShowPolygonBar && (selectedRegionIndex >= 0 && selectedRegionIndex < customDrawnRegions.Count);
        }

        if (layoutShowPolygonBar)
        {
            GUILayout.BeginHorizontal();
            bool prevEn = GUI.enabled;
            GUI.enabled = prevEn && activePolygonVerticesUV.Count >= 3;
            if (GUILayout.Button($"Đóng đa giác ({activePolygonVerticesUV.Count} đỉnh)", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(22f)))
            {
                CommitPolygonRegion2D(new List<Vector2>(activePolygonVerticesUV), lastPolygonHasRealLandmarks, lastPolygonLandmarksUV);
                activePolygonVerticesUV.Clear();
            }
            GUI.enabled = prevEn;
            if (GUILayout.Button("Bỏ đỉnh", AntigravityUITheme.StyleButton, GUILayout.Width(64f), GUILayout.Height(22f)))
            {
                if (activePolygonVerticesUV.Count > 0) activePolygonVerticesUV.RemoveAt(activePolygonVerticesUV.Count - 1);
            }
            if (GUILayout.Button("Hủy", AntigravityUITheme.StyleButton, GUILayout.Width(48f), GUILayout.Height(22f)))
            {
                activePolygonVerticesUV.Clear();
                customDrawStatusMsg = "Đã hủy đa giác đang vẽ.";
            }
            GUILayout.EndHorizontal();
        }
        else if (layoutShowSelectedRegionBar)
        {
            bool validSel = selectedRegionIndex >= 0 && selectedRegionIndex < customDrawnRegions.Count;
            string selName = "Vùng";
            if (validSel)
            {
                var sel = customDrawnRegions[selectedRegionIndex];
                selName = sel.shapeType == CanvasDrawTool.Circle ? "Tròn"
                    : (sel.shapeType == CanvasDrawTool.Rectangle ? "Vuông/CN" : $"Đa giác ({sel.verticesUV.Count} đỉnh)");
            }
            GUILayout.BeginHorizontal();
            GUILayout.Label(validSel ? $"Đang chọn: {selName} #{selectedRegionIndex + 1}" : "Đang chọn: -", AntigravityUITheme.StyleLabelYellow);
            if (GUILayout.Button("Xóa vùng", AntigravityUITheme.StyleButton, GUILayout.Width(68f), GUILayout.Height(20f)))
            {
                DeleteSelectedRegion2D();
            }
            if (GUILayout.Button("Bỏ chọn", AntigravityUITheme.StyleButton, GUILayout.Width(60f), GUILayout.Height(20f)))
            {
                selectedRegionIndex = -1;
                activeRegionEditMode = RegionEditMode.None;
            }
            GUILayout.EndHorizontal();
        }

        // Hàng 3: Chỉnh cỡ cọ (kích cỡ đường phun mm) + các mức chọn nhanh
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Cỡ cọ: {sprayWidthMm:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(104f));
        float newBrushMm = GUILayout.HorizontalSlider(sprayWidthMm, 1.0f, 50.0f);
        if (Mathf.Abs(newBrushMm - sprayWidthMm) > 0.05f)
        {
            UpdateSprayWidth(Mathf.Round(newBrushMm * 2f) * 0.5f);
            MarkSettingsDirty();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        float[] brushPresets = { 2f, 5f, 10f, 20f, 30f };
        for (int i = 0; i < brushPresets.Length; i++)
        {
            float p = brushPresets[i];
            GUIStyle st = Mathf.Abs(sprayWidthMm - p) < 0.4f ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            if (GUILayout.Button($"{p:F0}mm", st, GUILayout.Height(20f)))
            {
                UpdateSprayWidth(p);
                MarkSettingsDirty();
            }
        }
        GUILayout.EndHorizontal();
    }

    private Vector2[] GetRectHandlesScreenPx(Rect rUV, Rect canvasRect)
    {
        float x0 = canvasRect.x + rUV.xMin * canvasRect.width;
        float x1 = canvasRect.x + rUV.xMax * canvasRect.width;
        float xm = (x0 + x1) * 0.5f;
        float y0 = canvasRect.y + rUV.yMin * canvasRect.height;
        float y1 = canvasRect.y + rUV.yMax * canvasRect.height;
        float ym = (y0 + y1) * 0.5f;
        return new Vector2[]
        {
            new Vector2(x0, y0), // 0: TL
            new Vector2(xm, y0), // 1: T
            new Vector2(x1, y0), // 2: TR
            new Vector2(x1, ym), // 3: R
            new Vector2(x1, y1), // 4: BR
            new Vector2(xm, y1), // 5: B
            new Vector2(x0, y1), // 6: BL
            new Vector2(x0, ym)  // 7: L
        };
    }

    private Vector2[] GetCircleEdgeHandlesScreenPx(DrawnRegion2D reg, Rect canvasRect)
    {
        Vector2 c = new Vector2(canvasRect.x + reg.centerUV.x * canvasRect.width, canvasRect.y + reg.centerUV.y * canvasRect.height);
        float rx = reg.radiusUV.x * canvasRect.width;
        float ry = reg.radiusUV.y * canvasRect.height;
        return new Vector2[]
        {
            new Vector2(c.x + rx, c.y),
            new Vector2(c.x - rx, c.y),
            new Vector2(c.x, c.y - ry),
            new Vector2(c.x, c.y + ry)
        };
    }

    private Rect ComputePolygonBoundsUV(List<Vector2> verts)
    {
        if (verts == null || verts.Count == 0) return Rect.zero;
        float minU = float.MaxValue, maxU = float.MinValue, minV = float.MaxValue, maxV = float.MinValue;
        for (int i = 0; i < verts.Count; i++)
        {
            if (verts[i].x < minU) minU = verts[i].x;
            if (verts[i].x > maxU) maxU = verts[i].x;
            if (verts[i].y < minV) minV = verts[i].y;
            if (verts[i].y > maxV) maxV = verts[i].y;
        }
        return new Rect(minU, minV, Mathf.Max(0.001f, maxU - minU), Mathf.Max(0.001f, maxV - minV));
    }

    private bool HitTestSelectedRegionHandles(Vector2 mousePx, Rect canvasRect, out RegionEditMode editMode, out int handleIdx)
    {
        editMode = RegionEditMode.None;
        handleIdx = -1;
        if (selectedRegionIndex < 0 || selectedRegionIndex >= customDrawnRegions.Count) return false;
        var reg = customDrawnRegions[selectedRegionIndex];
        if (reg == null) return false;
        const float hitR = 11f;

        if (reg.shapeType == CanvasDrawTool.Circle)
        {
            Vector2[] edges = GetCircleEdgeHandlesScreenPx(reg, canvasRect);
            for (int i = 0; i < edges.Length; i++)
            {
                if (Vector2.Distance(mousePx, edges[i]) <= hitR)
                {
                    editMode = RegionEditMode.ResizeCircleRadius;
                    handleIdx = i;
                    return true;
                }
            }
            Vector2 centerPx = new Vector2(canvasRect.x + reg.centerUV.x * canvasRect.width, canvasRect.y + reg.centerUV.y * canvasRect.height);
            if (Vector2.Distance(mousePx, centerPx) <= hitR)
            {
                editMode = RegionEditMode.MoveRegion;
                handleIdx = 0;
                return true;
            }
            return false;
        }

        if (reg.shapeType == CanvasDrawTool.Rectangle)
        {
            Vector2[] handles = GetRectHandlesScreenPx(reg.rectUV, canvasRect);
            for (int i = 0; i < handles.Length; i++)
            {
                if (Vector2.Distance(mousePx, handles[i]) <= hitR)
                {
                    editMode = RegionEditMode.ResizeRectHandle;
                    handleIdx = i;
                    return true;
                }
            }
            return false;
        }

        if (reg.verticesUV != null && reg.verticesUV.Count >= 3)
        {
            // Ưu tiên kéo từng đỉnh đa giác
            for (int i = 0; i < reg.verticesUV.Count; i++)
            {
                Vector2 vp = new Vector2(canvasRect.x + reg.verticesUV[i].x * canvasRect.width, canvasRect.y + reg.verticesUV[i].y * canvasRect.height);
                if (Vector2.Distance(mousePx, vp) <= hitR)
                {
                    editMode = RegionEditMode.MovePolygonVertex;
                    handleIdx = i;
                    return true;
                }
            }
            // Hoặc kéo 4 góc khung bao để phóng to / thu nhỏ toàn bộ đa giác
            Rect bUV = ComputePolygonBoundsUV(reg.verticesUV);
            Vector2[] corners = new Vector2[]
            {
                new Vector2(canvasRect.x + bUV.xMin * canvasRect.width, canvasRect.y + bUV.yMin * canvasRect.height),
                new Vector2(canvasRect.x + bUV.xMax * canvasRect.width, canvasRect.y + bUV.yMin * canvasRect.height),
                new Vector2(canvasRect.x + bUV.xMax * canvasRect.width, canvasRect.y + bUV.yMax * canvasRect.height),
                new Vector2(canvasRect.x + bUV.xMin * canvasRect.width, canvasRect.y + bUV.yMax * canvasRect.height)
            };
            for (int i = 0; i < corners.Length; i++)
            {
                if (Vector2.Distance(mousePx, corners[i]) <= hitR)
                {
                    editMode = RegionEditMode.ScalePolygonCorner;
                    handleIdx = i;
                    return true;
                }
            }
        }
        return false;
    }

    private int HitTestAnyRegionInterior(Vector2 mouseUV, Rect canvasRect)
    {
        for (int i = customDrawnRegions.Count - 1; i >= 0; i--)
        {
            var reg = customDrawnRegions[i];
            if (reg == null) continue;
            if (reg.shapeType == CanvasDrawTool.Circle)
            {
                float dx = (mouseUV.x - reg.centerUV.x) / Mathf.Max(0.001f, reg.radiusUV.x);
                float dy = (mouseUV.y - reg.centerUV.y) / Mathf.Max(0.001f, reg.radiusUV.y);
                if (dx * dx + dy * dy <= 1.05f) return i;
            }
            else if (reg.shapeType == CanvasDrawTool.Rectangle)
            {
                if (reg.rectUV.Contains(mouseUV)) return i;
            }
            else if (reg.verticesUV != null && reg.verticesUV.Count >= 3)
            {
                if (IsPointInPolygon2D(mouseUV, reg.verticesUV)) return i;
            }
        }
        return -1;
    }

    private void BeginRegionEdit(int regionIdx, RegionEditMode mode, int handleIdx, Vector2 mouseUV)
    {
        selectedRegionIndex = regionIdx;
        activeRegionEditMode = mode;
        activeEditHandleIndex = handleIdx;
        editDragStartMouseUV = mouseUV;
        var reg = customDrawnRegions[regionIdx];
        editOrigCenterUV = reg.centerUV;
        editOrigRectUV = reg.shapeType == CanvasDrawTool.Polygon ? ComputePolygonBoundsUV(reg.verticesUV) : reg.rectUV;
        editOrigVerticesUV = reg.verticesUV != null ? new List<Vector2>(reg.verticesUV) : null;
    }

    private void ApplyActiveRegionEditDrag(Vector2 curMouseUV, Rect canvasRect, bool shiftHeld)
    {
        if (selectedRegionIndex < 0 || selectedRegionIndex >= customDrawnRegions.Count) return;
        var reg = customDrawnRegions[selectedRegionIndex];
        if (reg == null) return;

        if (activeRegionEditMode == RegionEditMode.MoveRegion)
        {
            Vector2 delta = curMouseUV - editDragStartMouseUV;
            if (reg.shapeType == CanvasDrawTool.Circle)
            {
                float cx = Mathf.Clamp(editOrigCenterUV.x + delta.x, reg.radiusUV.x, 1f - reg.radiusUV.x);
                float cy = Mathf.Clamp(editOrigCenterUV.y + delta.y, reg.radiusUV.y, 1f - reg.radiusUV.y);
                reg.centerUV = new Vector2(cx, cy);
                reg.verticesUV = BuildCirclePolygonFromCenterRadiusUV(reg.centerUV, reg.radiusUV, 48);
            }
            else if (reg.shapeType == CanvasDrawTool.Rectangle)
            {
                float nx = Mathf.Clamp(editOrigRectUV.x + delta.x, 0f, Mathf.Max(0f, 1f - editOrigRectUV.width));
                float ny = Mathf.Clamp(editOrigRectUV.y + delta.y, 0f, Mathf.Max(0f, 1f - editOrigRectUV.height));
                reg.rectUV = new Rect(nx, ny, editOrigRectUV.width, editOrigRectUV.height);
                reg.verticesUV = new List<Vector2>
                {
                    new Vector2(reg.rectUV.xMin, reg.rectUV.yMin),
                    new Vector2(reg.rectUV.xMax, reg.rectUV.yMin),
                    new Vector2(reg.rectUV.xMax, reg.rectUV.yMax),
                    new Vector2(reg.rectUV.xMin, reg.rectUV.yMax)
                };
            }
            else if (editOrigVerticesUV != null && editOrigVerticesUV.Count >= 3)
            {
                float dx = Mathf.Clamp(delta.x, -editOrigRectUV.xMin, 1f - editOrigRectUV.xMax);
                float dy = Mathf.Clamp(delta.y, -editOrigRectUV.yMin, 1f - editOrigRectUV.yMax);
                reg.verticesUV = new List<Vector2>(editOrigVerticesUV.Count);
                for (int i = 0; i < editOrigVerticesUV.Count; i++)
                {
                    reg.verticesUV.Add(new Vector2(editOrigVerticesUV[i].x + dx, editOrigVerticesUV[i].y + dy));
                }
                reg.rectUV = ComputePolygonBoundsUV(reg.verticesUV);
            }
            return;
        }

        if (activeRegionEditMode == RegionEditMode.ResizeCircleRadius && reg.shapeType == CanvasDrawTool.Circle)
        {
            ComputeClampedCircleGeometry(reg.centerUV, curMouseUV, canvasRect, out float radPx, out Vector2 radUV);
            if (radPx >= 4f)
            {
                reg.radiusUV = radUV;
                reg.verticesUV = BuildCirclePolygonFromCenterRadiusUV(reg.centerUV, reg.radiusUV, 48);
            }
            return;
        }

        if (activeRegionEditMode == RegionEditMode.ResizeRectHandle && reg.shapeType == CanvasDrawTool.Rectangle)
        {
            float minU = editOrigRectUV.xMin;
            float maxU = editOrigRectUV.xMax;
            float minV = editOrigRectUV.yMin;
            float maxV = editOrigRectUV.yMax;
            float minSizeU = 6f / Mathf.Max(1f, canvasRect.width);
            float minSizeV = 6f / Mathf.Max(1f, canvasRect.height);

            // Handles: 0=TL, 1=T, 2=TR, 3=R, 4=BR, 5=B, 6=BL, 7=L
            bool moveL = (activeEditHandleIndex == 0 || activeEditHandleIndex == 6 || activeEditHandleIndex == 7);
            bool moveR = (activeEditHandleIndex == 2 || activeEditHandleIndex == 3 || activeEditHandleIndex == 4);
            bool moveT = (activeEditHandleIndex == 0 || activeEditHandleIndex == 1 || activeEditHandleIndex == 2);
            bool moveB = (activeEditHandleIndex == 4 || activeEditHandleIndex == 5 || activeEditHandleIndex == 6);

            if (moveL) minU = Mathf.Clamp(curMouseUV.x, 0f, maxU - minSizeU);
            if (moveR) maxU = Mathf.Clamp(curMouseUV.x, minU + minSizeU, 1f);
            if (moveT) minV = Mathf.Clamp(curMouseUV.y, 0f, maxV - minSizeV);
            if (moveB) maxV = Mathf.Clamp(curMouseUV.y, minV + minSizeV, 1f);

            if (shiftHeld && (moveL || moveR) && (moveT || moveB))
            {
                float wPx = (maxU - minU) * canvasRect.width;
                float hPx = (maxV - minV) * canvasRect.height;
                float sidePx = Mathf.Max(wPx, hPx);
                float sideU = sidePx / Mathf.Max(1f, canvasRect.width);
                float sideV = sidePx / Mathf.Max(1f, canvasRect.height);
                if (moveL) minU = Mathf.Clamp01(maxU - sideU); else maxU = Mathf.Clamp01(minU + sideU);
                if (moveT) minV = Mathf.Clamp01(maxV - sideV); else maxV = Mathf.Clamp01(minV + sideV);
            }

            reg.rectUV = new Rect(minU, minV, Mathf.Max(minSizeU, maxU - minU), Mathf.Max(minSizeV, maxV - minV));
            reg.verticesUV = new List<Vector2>
            {
                new Vector2(reg.rectUV.xMin, reg.rectUV.yMin),
                new Vector2(reg.rectUV.xMax, reg.rectUV.yMin),
                new Vector2(reg.rectUV.xMax, reg.rectUV.yMax),
                new Vector2(reg.rectUV.xMin, reg.rectUV.yMax)
            };
            return;
        }

        if (activeRegionEditMode == RegionEditMode.MovePolygonVertex && reg.verticesUV != null &&
            activeEditHandleIndex >= 0 && activeEditHandleIndex < reg.verticesUV.Count)
        {
            reg.verticesUV[activeEditHandleIndex] = new Vector2(Mathf.Clamp01(curMouseUV.x), Mathf.Clamp01(curMouseUV.y));
            reg.rectUV = ComputePolygonBoundsUV(reg.verticesUV);
            return;
        }

        if (activeRegionEditMode == RegionEditMode.ScalePolygonCorner && editOrigVerticesUV != null && editOrigVerticesUV.Count >= 3)
        {
            // Corners: 0=TL (anchor BR), 1=TR (anchor BL), 2=BR (anchor TL), 3=BL (anchor TR)
            Vector2 anchor = activeEditHandleIndex == 0 ? new Vector2(editOrigRectUV.xMax, editOrigRectUV.yMax)
                : (activeEditHandleIndex == 1 ? new Vector2(editOrigRectUV.xMin, editOrigRectUV.yMax)
                : (activeEditHandleIndex == 2 ? new Vector2(editOrigRectUV.xMin, editOrigRectUV.yMin)
                : new Vector2(editOrigRectUV.xMax, editOrigRectUV.yMin)));

            float origW = Mathf.Max(0.01f, editOrigRectUV.width);
            float origH = Mathf.Max(0.01f, editOrigRectUV.height);
            float newW = Mathf.Max(0.01f, Mathf.Abs(curMouseUV.x - anchor.x));
            float newH = Mathf.Max(0.01f, Mathf.Abs(curMouseUV.y - anchor.y));
            float sx = newW / origW;
            float sy = newH / origH;
            if (shiftHeld)
            {
                float sUni = Mathf.Max(sx, sy);
                sx = sUni;
                sy = sUni;
            }

            reg.verticesUV = new List<Vector2>(editOrigVerticesUV.Count);
            for (int i = 0; i < editOrigVerticesUV.Count; i++)
            {
                float nx = Mathf.Clamp01(anchor.x + (editOrigVerticesUV[i].x - anchor.x) * sx);
                float ny = Mathf.Clamp01(anchor.y + (editOrigVerticesUV[i].y - anchor.y) * sy);
                reg.verticesUV.Add(new Vector2(nx, ny));
            }
            reg.rectUV = ComputePolygonBoundsUV(reg.verticesUV);
        }
    }

    private void Handle2DCanvasMouseInput(Rect canvasRect, bool isFullscreen, bool hasRealLandmarks, Vector2[] curLandmarksUV)
    {
        Event ev = Event.current;
        if (ev == null || ev.type == EventType.Layout || canvasRect.width < 4f || canvasRect.height < 4f) return;

        int canvasCtrlId = GUIUtility.GetControlID(FocusType.Passive);
        bool insideCanvas = canvasRect.Contains(ev.mousePosition);
        Vector2 normUV = new Vector2(
            Mathf.Clamp01((ev.mousePosition.x - canvasRect.x) / canvasRect.width),
            Mathf.Clamp01((ev.mousePosition.y - canvasRect.y) / canvasRect.height)
        );

        // Phím tắt (Enter / Backspace / Delete / ESC)
        if (ev.type == EventType.KeyDown)
        {
            if (activeDrawTool == CanvasDrawTool.Polygon && activePolygonVerticesUV.Count > 0)
            {
                if ((ev.keyCode == KeyCode.Return || ev.keyCode == KeyCode.KeypadEnter) && activePolygonVerticesUV.Count >= 3)
                {
                    CommitPolygonRegion2D(new List<Vector2>(activePolygonVerticesUV), hasRealLandmarks, curLandmarksUV);
                    activePolygonVerticesUV.Clear();
                    ev.Use();
                    return;
                }
                if (ev.keyCode == KeyCode.Backspace)
                {
                    activePolygonVerticesUV.RemoveAt(activePolygonVerticesUV.Count - 1);
                    ev.Use();
                    return;
                }
                if (ev.keyCode == KeyCode.Escape)
                {
                    activePolygonVerticesUV.Clear();
                    customDrawStatusMsg = "Đã hủy đa giác đang vẽ.";
                    ev.Use();
                    return;
                }
            }
            else if (ev.keyCode == KeyCode.Delete && selectedRegionIndex >= 0 && selectedRegionIndex < customDrawnRegions.Count)
            {
                DeleteSelectedRegion2D();
                ev.Use();
                return;
            }
            else if ((isDraggingShape || activeRegionEditMode != RegionEditMode.None) && ev.keyCode == KeyCode.Escape)
            {
                isDraggingShape = false;
                activeRegionEditMode = RegionEditMode.None;
                activeEditHandleIndex = -1;
                ReleaseCanvasHotControl();
                customDrawStatusMsg = "Đã hủy thao tác.";
                ev.Use();
                return;
            }
        }

        // 1. Đang kéo chỉnh kích thước / di chuyển vùng đã vẽ
        if (activeRegionEditMode != RegionEditMode.None)
        {
            if (draggingOnFullscreen != isFullscreen) return;
            if (ev.type == EventType.MouseDrag)
            {
                ApplyActiveRegionEditDrag(normUV, canvasRect, ev.shift);
                ev.Use();
            }
            else if (ev.type == EventType.MouseUp && ev.button == 0)
            {
                ApplyActiveRegionEditDrag(normUV, canvasRect, ev.shift);
                activeRegionEditMode = RegionEditMode.None;
                activeEditHandleIndex = -1;
                ReleaseCanvasHotControl();
                InvalidateGeneratedTrajectoryOnEdit();
                ev.Use();
            }
            return;
        }

        // 2. Ưu tiên bắt nút kéo đổi kích thước / chỉnh đỉnh của vùng đang chọn (hoạt động cả khi đang ở công cụ Tròn / Vuông / Đa giác / Sửa)
        if (insideCanvas && ev.type == EventType.MouseDown && ev.button == 0 &&
            activeDrawTool != CanvasDrawTool.Freehand && activePolygonVerticesUV.Count == 0 &&
            HitTestSelectedRegionHandles(ev.mousePosition, canvasRect, out RegionEditMode hitMode, out int hitHandle))
        {
            GUIUtility.hotControl = canvasCtrlId;
            activeCanvasControlId = canvasCtrlId;
            draggingOnFullscreen = isFullscreen;
            BeginRegionEdit(selectedRegionIndex, hitMode, hitHandle, normUV);
            ev.Use();
            return;
        }

        // 3. Công cụ Sửa (Chọn vùng, di chuyển, kéo đổi kích thước)
        if (activeDrawTool == CanvasDrawTool.Select)
        {
            if (insideCanvas && ev.type == EventType.MouseDown && ev.button == 0)
            {
                int hitIdx = HitTestAnyRegionInterior(normUV, canvasRect);
                if (hitIdx >= 0)
                {
                    GUIUtility.hotControl = canvasCtrlId;
                    activeCanvasControlId = canvasCtrlId;
                    draggingOnFullscreen = isFullscreen;
                    BeginRegionEdit(hitIdx, RegionEditMode.MoveRegion, 0, normUV);
                    customDrawStatusMsg = $"Đã chọn vùng #{hitIdx + 1} (kéo để di chuyển, kéo nút trắng để đổi kích thước).";
                }
                else
                {
                    selectedRegionIndex = -1;
                }
                ev.Use();
            }
            return;
        }

        if (activeDrawTool == CanvasDrawTool.Freehand)
        {
            float minDistUV = isFullscreen ? 0.004f : 0.008f;
            if (insideCanvas && ev.type == EventType.MouseDown && ev.button == 0)
            {
                GUIUtility.hotControl = canvasCtrlId;
                activeCanvasControlId = canvasCtrlId;
                draggingOnFullscreen = isFullscreen;
                hasInitializedDefaultStrokes = true;
                currentDrawingStroke = new List<Vector2> { normUV };
                customDrawnStrokes.Add(currentDrawingStroke);
                undoHistoryIsRegion.Add(false);
                if (hasRealLandmarks && curLandmarksUV != null)
                {
                    strokesReferenceLandmarksUV = (Vector2[])curLandmarksUV.Clone();
                }
                isDrawingOnCanvas = true;
                ev.Use();
            }
            else if (isDrawingOnCanvas && draggingOnFullscreen == isFullscreen &&
                     ev.type == EventType.MouseDrag && ev.button == 0 && currentDrawingStroke != null)
            {
                if (currentDrawingStroke.Count == 0 || Vector2.Distance(currentDrawingStroke[currentDrawingStroke.Count - 1], normUV) >= minDistUV)
                {
                    currentDrawingStroke.Add(normUV);
                }
                ev.Use();
            }
            else if (isDrawingOnCanvas && draggingOnFullscreen == isFullscreen &&
                     ev.type == EventType.MouseUp && ev.button == 0)
            {
                ReleaseCanvasHotControl();
                isDrawingOnCanvas = false;
                currentDrawingStroke = null;
                CaptureCurrentStrokesToCanonical();
                InvalidateGeneratedTrajectoryOnEdit();
                ev.Use();
            }
            return;
        }

        if (activeDrawTool == CanvasDrawTool.Circle || activeDrawTool == CanvasDrawTool.Rectangle)
        {
            if (insideCanvas && ev.type == EventType.MouseDown && ev.button == 0)
            {
                GUIUtility.hotControl = canvasCtrlId;
                activeCanvasControlId = canvasCtrlId;
                draggingOnFullscreen = isFullscreen;
                hasInitializedDefaultStrokes = true;
                isDraggingShape = true;
                shapeStartUV = normUV;
                shapeCurrentUV = normUV;
                ev.Use();
            }
            else if (isDraggingShape && draggingOnFullscreen == isFullscreen && ev.type == EventType.MouseDown && ev.button == 1)
            {
                ReleaseCanvasHotControl();
                isDraggingShape = false;
                customDrawStatusMsg = "Đã hủy vùng đang kéo.";
                ev.Use();
            }
            else if (isDraggingShape && draggingOnFullscreen == isFullscreen && ev.type == EventType.MouseDrag && ev.button == 0)
            {
                shapeCurrentUV = normUV;
                ev.Use();
            }
            else if (isDraggingShape && draggingOnFullscreen == isFullscreen && ev.type == EventType.MouseUp && ev.button == 0)
            {
                ReleaseCanvasHotControl();
                isDraggingShape = false;
                shapeCurrentUV = normUV;
                if (activeDrawTool == CanvasDrawTool.Circle)
                {
                    CommitCircleRegion2D(shapeStartUV, shapeCurrentUV, canvasRect, hasRealLandmarks, curLandmarksUV);
                }
                else
                {
                    CommitRectangleRegion2D(shapeStartUV, shapeCurrentUV, canvasRect, ev.shift, hasRealLandmarks, curLandmarksUV);
                }
                ev.Use();
            }
            return;
        }

        if (activeDrawTool == CanvasDrawTool.Polygon && insideCanvas)
        {
            if (ev.type == EventType.MouseDown && ev.button == 0)
            {
                hasInitializedDefaultStrokes = true;
                lastPolygonHasRealLandmarks = hasRealLandmarks;
                lastPolygonLandmarksUV = (hasRealLandmarks && curLandmarksUV != null) ? (Vector2[])curLandmarksUV.Clone() : null;

                if (activePolygonVerticesUV.Count >= 3)
                {
                    Vector2 firstPx = new Vector2(
                        canvasRect.x + activePolygonVerticesUV[0].x * canvasRect.width,
                        canvasRect.y + activePolygonVerticesUV[0].y * canvasRect.height
                    );
                    if (Vector2.Distance(ev.mousePosition, firstPx) <= 13f)
                    {
                        CommitPolygonRegion2D(new List<Vector2>(activePolygonVerticesUV), hasRealLandmarks, curLandmarksUV);
                        activePolygonVerticesUV.Clear();
                        ev.Use();
                        return;
                    }
                }

                bool farEnough = true;
                if (activePolygonVerticesUV.Count > 0)
                {
                    Vector2 prevPx = new Vector2(
                        canvasRect.x + activePolygonVerticesUV[activePolygonVerticesUV.Count - 1].x * canvasRect.width,
                        canvasRect.y + activePolygonVerticesUV[activePolygonVerticesUV.Count - 1].y * canvasRect.height
                    );
                    if (Vector2.Distance(ev.mousePosition, prevPx) < 4f) farEnough = false;
                }

                if (farEnough)
                {
                    activePolygonVerticesUV.Add(normUV);
                    customDrawStatusMsg = $"Đang vẽ đa giác ({activePolygonVerticesUV.Count} đỉnh) - Click phải hoặc nối đỉnh đầu để đóng.";
                }
                ev.Use();
            }
            else if (ev.type == EventType.MouseDown && ev.button == 1)
            {
                if (activePolygonVerticesUV.Count >= 3)
                {
                    CommitPolygonRegion2D(new List<Vector2>(activePolygonVerticesUV), hasRealLandmarks, curLandmarksUV);
                    activePolygonVerticesUV.Clear();
                }
                else if (activePolygonVerticesUV.Count > 0)
                {
                    activePolygonVerticesUV.Clear();
                    customDrawStatusMsg = "Đã hủy đa giác đang vẽ.";
                }
                ev.Use();
            }
        }
    }

    private void ComputeClampedCircleGeometry(Vector2 centerUV, Vector2 edgeUV, Rect canvasRect, out float radiusPx, out Vector2 radiusUV)
    {
        float dxPx = (edgeUV.x - centerUV.x) * canvasRect.width;
        float dyPx = (edgeUV.y - centerUV.y) * canvasRect.height;
        float rawRadiusPx = Mathf.Sqrt(dxPx * dxPx + dyPx * dyPx);

        float cxPx = centerUV.x * canvasRect.width;
        float cyPx = centerUV.y * canvasRect.height;
        float maxFitPx = Mathf.Min(
            Mathf.Min(cxPx, canvasRect.width - cxPx),
            Mathf.Min(cyPx, canvasRect.height - cyPx)
        );
        radiusPx = Mathf.Min(rawRadiusPx, Mathf.Max(4f, maxFitPx));
        radiusUV = new Vector2(radiusPx / Mathf.Max(1f, canvasRect.width), radiusPx / Mathf.Max(1f, canvasRect.height));
    }

    private Rect ComputeRectangleUV(Vector2 startUV, Vector2 endUV, Rect canvasRect, bool lockSquare)
    {
        float u1 = endUV.x;
        float v1 = endUV.y;
        if (lockSquare)
        {
            float dxPx = (endUV.x - startUV.x) * canvasRect.width;
            float dyPx = (endUV.y - startUV.y) * canvasRect.height;
            float sidePx = Mathf.Max(Mathf.Abs(dxPx), Mathf.Abs(dyPx));
            float sx = dxPx >= 0f ? 1f : -1f;
            float sy = dyPx >= 0f ? 1f : -1f;
            u1 = Mathf.Clamp01(startUV.x + (sx * sidePx) / canvasRect.width);
            v1 = Mathf.Clamp01(startUV.y + (sy * sidePx) / canvasRect.height);
        }

        float minU = Mathf.Min(startUV.x, u1);
        float maxU = Mathf.Max(startUV.x, u1);
        float minV = Mathf.Min(startUV.y, v1);
        float maxV = Mathf.Max(startUV.y, v1);
        return new Rect(minU, minV, maxU - minU, maxV - minV);
    }

    private List<Vector2> BuildCirclePolygonFromCenterRadiusUV(Vector2 centerUV, Vector2 radiusUV, int segments = 48)
    {
        int seg = Mathf.Clamp(segments, 16, 72);
        List<Vector2> poly = new List<Vector2>(seg);
        for (int i = 0; i < seg; i++)
        {
            float ang = (i / (float)seg) * Mathf.PI * 2f;
            poly.Add(new Vector2(
                Mathf.Clamp01(centerUV.x + Mathf.Cos(ang) * radiusUV.x),
                Mathf.Clamp01(centerUV.y + Mathf.Sin(ang) * radiusUV.y)
            ));
        }
        return poly;
    }

    private void CommitCircleRegion2D(Vector2 centerUV, Vector2 edgeUV, Rect canvasRect, bool hasRealLandmarks, Vector2[] curLandmarksUV)
    {
        ComputeClampedCircleGeometry(centerUV, edgeUV, canvasRect, out float radiusPx, out Vector2 radiusUV);
        if (radiusPx < 4f) return;

        if (hasRealLandmarks && curLandmarksUV != null && strokesReferenceLandmarksUV == null)
        {
            strokesReferenceLandmarksUV = (Vector2[])curLandmarksUV.Clone();
        }

        DrawnRegion2D region = new DrawnRegion2D
        {
            shapeType = CanvasDrawTool.Circle,
            brushWidthMm = sprayWidthMm,
            centerUV = centerUV,
            radiusUV = radiusUV,
            verticesUV = BuildCirclePolygonFromCenterRadiusUV(centerUV, radiusUV, 48)
        };
        customDrawnRegions.Add(region);
        undoHistoryIsRegion.Add(true);
        selectedRegionIndex = customDrawnRegions.Count - 1;
        customDrawStatusMsg = $"Đã thêm vùng Hình tròn ({customDrawnRegions.Count} vùng). Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
        InvalidateGeneratedTrajectoryOnEdit();
    }

    private void CommitRectangleRegion2D(Vector2 startUV, Vector2 endUV, Rect canvasRect, bool lockSquare, bool hasRealLandmarks, Vector2[] curLandmarksUV)
    {
        Rect rUV = ComputeRectangleUV(startUV, endUV, canvasRect, lockSquare);
        if (rUV.width * canvasRect.width < 4f || rUV.height * canvasRect.height < 4f) return;

        if (hasRealLandmarks && curLandmarksUV != null && strokesReferenceLandmarksUV == null)
        {
            strokesReferenceLandmarksUV = (Vector2[])curLandmarksUV.Clone();
        }

        DrawnRegion2D region = new DrawnRegion2D
        {
            shapeType = CanvasDrawTool.Rectangle,
            brushWidthMm = sprayWidthMm,
            rectUV = rUV,
            verticesUV = new List<Vector2>
            {
                new Vector2(rUV.xMin, rUV.yMin),
                new Vector2(rUV.xMax, rUV.yMin),
                new Vector2(rUV.xMax, rUV.yMax),
                new Vector2(rUV.xMin, rUV.yMax)
            }
        };
        customDrawnRegions.Add(region);
        undoHistoryIsRegion.Add(true);
        selectedRegionIndex = customDrawnRegions.Count - 1;
        string label = lockSquare ? "Hình vuông" : "Hình chữ nhật";
        customDrawStatusMsg = $"Đã thêm vùng {label} ({customDrawnRegions.Count} vùng). Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
        InvalidateGeneratedTrajectoryOnEdit();
    }

    private void CommitPolygonRegion2D(List<Vector2> polyUV, bool hasRealLandmarks, Vector2[] curLandmarksUV)
    {
        if (polyUV == null || polyUV.Count < 3) return;
        float minU = float.MaxValue, maxU = float.MinValue;
        float minV = float.MaxValue, maxV = float.MinValue;
        for (int i = 0; i < polyUV.Count; i++)
        {
            if (polyUV[i].x < minU) minU = polyUV[i].x;
            if (polyUV[i].x > maxU) maxU = polyUV[i].x;
            if (polyUV[i].y < minV) minV = polyUV[i].y;
            if (polyUV[i].y > maxV) maxV = polyUV[i].y;
        }
        if ((maxU - minU) < 0.004f || (maxV - minV) < 0.004f) return;

        if (hasRealLandmarks && curLandmarksUV != null && strokesReferenceLandmarksUV == null)
        {
            strokesReferenceLandmarksUV = (Vector2[])curLandmarksUV.Clone();
        }

        DrawnRegion2D region = new DrawnRegion2D
        {
            shapeType = CanvasDrawTool.Polygon,
            brushWidthMm = sprayWidthMm,
            rectUV = new Rect(minU, minV, maxU - minU, maxV - minV),
            verticesUV = new List<Vector2>(polyUV)
        };
        customDrawnRegions.Add(region);
        undoHistoryIsRegion.Add(true);
        selectedRegionIndex = customDrawnRegions.Count - 1;
        customDrawStatusMsg = $"Đã thêm vùng Đa giác {polyUV.Count} đỉnh ({customDrawnRegions.Count} vùng). Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
        InvalidateGeneratedTrajectoryOnEdit();
    }

    private static void AppendSubdividedSegmentUV(List<Vector2> dst, Vector2 a, Vector2 b, float maxStepUV = 0.020f)
    {
        float dist = Vector2.Distance(a, b);
        int steps = Mathf.Max(1, Mathf.CeilToInt(dist / Mathf.Max(0.004f, maxStepUV)));
        for (int k = 0; k <= steps; k++)
        {
            Vector2 pt = Vector2.Lerp(a, b, k / (float)steps);
            if (dst.Count == 0 || Vector2.Distance(dst[dst.Count - 1], pt) > 0.0008f)
            {
                dst.Add(pt);
            }
        }
    }

    private static bool IsPointInPolygon2D(Vector2 pt, List<Vector2> poly)
    {
        if (poly == null || poly.Count < 3) return false;
        bool inside = false;
        int n = poly.Count;
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            Vector2 pi = poly[i];
            Vector2 pj = poly[j];
            bool intersect = ((pi.y > pt.y) != (pj.y > pt.y)) &&
                             (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / ((pj.y - pi.y) + 1e-9f) + pi.x);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    public bool HasCustomDrawnRegions2D => customDrawnRegions != null && customDrawnRegions.Count > 0;
    public bool HasCustomDrawnStrokes2D => customDrawnStrokes != null && customDrawnStrokes.Count > 0;

    /// <summary>
    /// Ánh xạ trực tiếp từng vùng hình cơ bản (Tròn / Vuông / Đa giác) từ tọa độ Canvas UV [0,1]x[0,1]
    /// sang đa giác thực tế 3D World XZ (mét) theo đúng tỷ lệ 1:1 từ ảnh camera — KHÔNG biến dạng qua lưới 12 mốc.
    /// </summary>
    public List<List<Vector2>> GetCustomDrawnRegionPolygonsWorldXZ(PLYDirectLoader plyLoader)
    {
        var result = new List<List<Vector2>>();
        if (customDrawnRegions == null || customDrawnRegions.Count == 0) return result;

        Texture2D activeCamTex = GetActiveRealCameraTexture(out PLYDirectLoader foundPly, out int nativeW, out int nativeH, out _);
        if (plyLoader == null) plyLoader = foundPly;
        bool usingRealCam = (activeCamTex != null);
        Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, usingRealCam, out bool hasRealLm);
        bool has12Lm = (landmarksWorldPos != null && landmarksWorldPos.Count >= 12);
        Vector2[] canonLmUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        Vector2[] dstWorldXZ = new Vector2[12];
        if (has12Lm)
        {
            for (int i = 0; i < 12; i++) dstWorldXZ[i] = new Vector2(landmarksWorldPos[i].x, landmarksWorldPos[i].z);
        }

        // Xây dựng phép ánh xạ tuyến tính toàn cục (Affine 2D) từ UV camera -> World XZ để bù các pixel thiếu chiều sâu
        // mà vẫn bảo toàn 100% độ thẳng cạnh hình vuông và độ tròn đều của hình tròn!
        bool hasDirectCameraGrid = (realCameraFileTex == null && plyLoader != null &&
                                    (plyLoader.hasOrganizedCameraFrame || plyLoader.hasTopDownCameraTexture));
        bool hasAffineCamToWorld = false;
        double[] affX = null, affZ = null;

        if (hasDirectCameraGrid)
        {
            var sampleUV = new List<Vector2>(81);
            var sampleXZ = new List<Vector2>(81);
            for (int gy = 1; gy <= 9; gy++)
            {
                float v = gy / 10f;
                for (int gx = 1; gx <= 9; gx++)
                {
                    float u = gx / 10f;
                    if (plyLoader.TrySampleCameraUVToWorld(new Vector2(u, v), out Vector3 wp, out _))
                    {
                        sampleUV.Add(new Vector2(u, v));
                        sampleXZ.Add(new Vector2(wp.x, wp.z));
                    }
                }
            }
            if (sampleUV.Count >= 6)
            {
                hasAffineCamToWorld = FitAffine2D(sampleUV.ToArray(), sampleXZ.ToArray(), out affX, out affZ);
            }
        }
        if (!hasAffineCamToWorld && has12Lm)
        {
            Vector2[] srcRef = (usingRealCam && hasRealLm && camLmUV != null) ? camLmUV : canonLmUV;
            hasAffineCamToWorld = FitAffine2D(srcRef, dstWorldXZ, out affX, out affZ);
        }

        bool TryMapUvExact(Vector2 uv, out Vector2 worldXZ)
        {
            uv = new Vector2(Mathf.Clamp01(uv.x), Mathf.Clamp01(uv.y));
            if (hasDirectCameraGrid && plyLoader.TrySampleCameraUVToWorld(uv, out Vector3 wp, out _))
            {
                worldXZ = new Vector2(wp.x, wp.z);
                return true;
            }
            if (hasAffineCamToWorld && affX != null && affZ != null)
            {
                // FitAffine2D giải hệ: c[0] * 1 + c[1] * u + c[2] * v = worldCoord
                float wx = (float)(affX[0] + affX[1] * uv.x + affX[2] * uv.y);
                float wz = (float)(affZ[0] + affZ[1] * uv.x + affZ[2] * uv.y);
                worldXZ = new Vector2(wx, wz);
                return true;
            }
            if (has12Lm)
            {
                Vector2[] srcRef = (usingRealCam && hasRealLm && camLmUV != null) ? camLmUV : canonLmUV;
                worldXZ = TrajectoryEvaluationEngine.WarpPointBarycentric12(uv, srcRef, dstWorldXZ, 1f, 1f);
                return true;
            }
            worldXZ = Vector2.zero;
            return false;
        }

        for (int r = 0; r < customDrawnRegions.Count; r++)
        {
            var reg = customDrawnRegions[r];
            if (reg == null) continue;

            if (reg.shapeType == CanvasDrawTool.Circle && reg.radiusUV.x > 0.001f && reg.radiusUV.y > 0.001f)
            {
                if (!TryMapUvExact(reg.centerUV, out Vector2 cXZ)) continue;
                Vector2 uStep = new Vector2(Mathf.Clamp01(reg.centerUV.x + reg.radiusUV.x), reg.centerUV.y);
                Vector2 vStep = new Vector2(reg.centerUV.x, Mathf.Clamp01(reg.centerUV.y + reg.radiusUV.y));
                if (!TryMapUvExact(uStep, out Vector2 pU)) pU = cXZ + new Vector2(reg.radiusUV.x * 0.9f, 0f);
                if (!TryMapUvExact(vStep, out Vector2 pV)) pV = cXZ + new Vector2(0f, reg.radiusUV.y * 0.9f);

                float du = Mathf.Max(1e-4f, Mathf.Abs(uStep.x - reg.centerUV.x));
                float dv = Mathf.Max(1e-4f, Mathf.Abs(vStep.y - reg.centerUV.y));
                float rU = Vector2.Distance(cXZ, pU) * (reg.radiusUV.x / du);
                float rV = Vector2.Distance(cXZ, pV) * (reg.radiusUV.y / dv);
                float radiusM = Mathf.Clamp(0.5f * (rU + rV), 0.005f, 0.55f);

                const int segs = 48;
                var circlePoly = new List<Vector2>(segs);
                for (int i = 0; i < segs; i++)
                {
                    float ang = (i / (float)segs) * Mathf.PI * 2f;
                    circlePoly.Add(new Vector2(cXZ.x + Mathf.Cos(ang) * radiusM, cXZ.y + Mathf.Sin(ang) * radiusM));
                }
                result.Add(circlePoly);
                continue;
            }

            if (reg.shapeType == CanvasDrawTool.Rectangle && reg.rectUV.width > 0.001f && reg.rectUV.height > 0.001f)
            {
                Vector2[] cornersUV = new Vector2[]
                {
                    new Vector2(reg.rectUV.xMin, reg.rectUV.yMin),
                    new Vector2(reg.rectUV.xMax, reg.rectUV.yMin),
                    new Vector2(reg.rectUV.xMax, reg.rectUV.yMax),
                    new Vector2(reg.rectUV.xMin, reg.rectUV.yMax)
                };
                var rectPoly = new List<Vector2>(4);
                bool ok = true;
                for (int i = 0; i < 4; i++)
                {
                    if (TryMapUvExact(cornersUV[i], out Vector2 wPt)) rectPoly.Add(wPt);
                    else ok = false;
                }
                if (ok && rectPoly.Count == 4) result.Add(rectPoly);
                continue;
            }

            if (reg.verticesUV != null && reg.verticesUV.Count >= 3)
            {
                var polyXZ = new List<Vector2>(reg.verticesUV.Count);
                for (int i = 0; i < reg.verticesUV.Count; i++)
                {
                    if (TryMapUvExact(reg.verticesUV[i], out Vector2 wPt))
                    {
                        if (polyXZ.Count == 0 || Vector2.Distance(polyXZ[polyXZ.Count - 1], wPt) > 0.001f)
                            polyXZ.Add(wPt);
                    }
                }
                if (polyXZ.Count >= 3) result.Add(polyXZ);
            }
        }
        return result;
    }

    private static void AppendSubdividedMetricSegment(List<Vector2> dst, Vector2 a, Vector2 b, float maxStepM = 0.008f)
    {
        float dist = Vector2.Distance(a, b);
        int steps = Mathf.Max(1, Mathf.CeilToInt(dist / Mathf.Max(0.003f, maxStepM)));
        for (int k = 0; k <= steps; k++)
        {
            Vector2 pt = Vector2.Lerp(a, b, k / (float)steps);
            if (dst.Count == 0 || Vector2.Distance(dst[dst.Count - 1], pt) > 0.0005f)
                dst.Add(pt);
        }
    }

    /// <summary>
    /// Sinh tập đường phun 3D chuẩn hình trong không gian mét World XZ cho 1 vùng hình (Tròn / Vuông / Đa giác):
    /// (1) 1 vòng chạy viền khép kín đúng 100% hình dạng vùng vẽ,
    /// (2) Các lượt quét song song bên trong vùng cách nhau theo cỡ cọ sprayWidthMm.
    /// </summary>
    private static List<List<Vector2>> BuildMetricSprayPathsForWorldPolygon(List<Vector2> polyXZ, float widthMm)
    {
        var strokes = new List<List<Vector2>>();
        if (polyXZ == null || polyXZ.Count < 3) return strokes;

        float minX = float.MaxValue, maxX = float.MinValue;
        float minZ = float.MaxValue, maxZ = float.MinValue;
        for (int i = 0; i < polyXZ.Count; i++)
        {
            if (polyXZ[i].x < minX) minX = polyXZ[i].x;
            if (polyXZ[i].x > maxX) maxX = polyXZ[i].x;
            if (polyXZ[i].y < minZ) minZ = polyXZ[i].y;
            if (polyXZ[i].y > maxZ) maxZ = polyXZ[i].y;
        }
        float wM = maxX - minX;
        float hM = maxZ - minZ;
        if (wM < 0.004f || hM < 0.004f) return strokes;

        // 1. Đường viền khép kín bám chính xác 100% chu vi hình (giữ tròn đều / giữ sắc 4 góc vuông / giữ mọi đỉnh đa giác)
        var perimeter = new List<Vector2>();
        for (int i = 0; i < polyXZ.Count; i++)
        {
            AppendSubdividedMetricSegment(perimeter, polyXZ[i], polyXZ[(i + 1) % polyXZ.Count], 0.008f);
        }
        if (perimeter.Count >= 3)
        {
            strokes.Add(perimeter);
        }

        // 2. Các lượt quét song song bên trong vùng cách nhau theo cỡ cọ (72% bề rộng cọ)
        float stepM = Mathf.Clamp(widthMm * 0.72f, 4.0f, 40.0f) * 0.001f;
        if (hM > stepM * 1.4f)
        {
            float insetM = Mathf.Min(stepM * 0.35f, Mathf.Min(wM, hM) * 0.12f);
            List<Vector2> activeZigzag = null;
            List<float> xs = new List<float>(16);
            int rowIdx = 0;

            for (float z = minZ + stepM * 0.65f; z <= maxZ - stepM * 0.55f; z += stepM)
            {
                xs.Clear();
                int n = polyXZ.Count;
                for (int i = 0, j = n - 1; i < n; j = i++)
                {
                    Vector2 a = polyXZ[j];
                    Vector2 b = polyXZ[i];
                    if ((a.y <= z && b.y > z) || (b.y <= z && a.y > z))
                    {
                        float t = (z - a.y) / (b.y - a.y);
                        xs.Add(a.x + t * (b.x - a.x));
                    }
                }
                if (xs.Count < 2)
                {
                    if (activeZigzag != null && activeZigzag.Count >= 2) strokes.Add(activeZigzag);
                    activeZigzag = null;
                    continue;
                }

                xs.Sort();
                int pairCount = xs.Count / 2;
                bool leftToRight = (rowIdx % 2 == 0);

                if (pairCount == 1)
                {
                    float x0 = xs[0] + insetM;
                    float x1 = xs[1] - insetM;
                    if (x1 - x0 > 0.004f)
                    {
                        Vector2 segA = leftToRight ? new Vector2(x0, z) : new Vector2(x1, z);
                        Vector2 segB = leftToRight ? new Vector2(x1, z) : new Vector2(x0, z);
                        bool canContinue = false;
                        if (activeZigzag != null && activeZigzag.Count >= 2)
                        {
                            Vector2 prevEnd = activeZigzag[activeZigzag.Count - 1];
                            Vector2 mid = (prevEnd + segA) * 0.5f;
                            if (IsPointInPolygon2D(mid, polyXZ) && Vector2.Distance(prevEnd, segA) <= stepM * 2.4f)
                                canContinue = true;
                        }
                        if (!canContinue)
                        {
                            if (activeZigzag != null && activeZigzag.Count >= 2) strokes.Add(activeZigzag);
                            activeZigzag = new List<Vector2>();
                        }
                        AppendSubdividedMetricSegment(activeZigzag, segA, segB, 0.008f);
                    }
                }
                else
                {
                    if (activeZigzag != null && activeZigzag.Count >= 2) strokes.Add(activeZigzag);
                    activeZigzag = null;
                    for (int p = 0; p < pairCount; p++)
                    {
                        float x0 = xs[p * 2] + insetM;
                        float x1 = xs[p * 2 + 1] - insetM;
                        if (x1 - x0 <= 0.004f) continue;
                        Vector2 segA = leftToRight ? new Vector2(x0, z) : new Vector2(x1, z);
                        Vector2 segB = leftToRight ? new Vector2(x1, z) : new Vector2(x0, z);
                        var span = new List<Vector2>();
                        AppendSubdividedMetricSegment(span, segA, segB, 0.008f);
                        if (span.Count >= 2) strokes.Add(span);
                    }
                }
                rowIdx++;
            }
            if (activeZigzag != null && activeZigzag.Count >= 2) strokes.Add(activeZigzag);
        }

        return strokes;
    }

    /// <summary>
    /// Chỉ được gọi cho P3/P4 khi cần nét mẫu dạng UV: trả về vòng viền khép kín chính xác của từng vùng hình.
    /// </summary>
    private List<List<Vector2>> BuildTemporarySprayStrokesFromDrawnRegions(float widthMm)
    {
        List<List<Vector2>> result = new List<List<Vector2>>();
        if (customDrawnRegions == null || customDrawnRegions.Count == 0) return result;

        for (int r = 0; r < customDrawnRegions.Count; r++)
        {
            var region = customDrawnRegions[r];
            if (region == null || region.verticesUV == null || region.verticesUV.Count < 3) continue;
            var polyUV = region.verticesUV;
            List<Vector2> closedLoop = new List<Vector2>(polyUV.Count + 1);
            for (int i = 0; i < polyUV.Count; i++)
            {
                AppendSubdividedSegmentUV(closedLoop, polyUV[i], polyUV[(i + 1) % polyUV.Count], 0.015f);
            }
            if (closedLoop.Count >= 3) result.Add(closedLoop);
        }
        return result;
    }

    private void DrawResizeHandleSquare(Vector2 centerPx, float halfSize, Color fillCol, Color borderCol)
    {
        Rect outer = new Rect(centerPx.x - halfSize - 1f, centerPx.y - halfSize - 1f, (halfSize + 1f) * 2f, (halfSize + 1f) * 2f);
        Rect inner = new Rect(centerPx.x - halfSize, centerPx.y - halfSize, halfSize * 2f, halfSize * 2f);
        DrawGuiSolidRect(outer, borderCol);
        DrawGuiSolidRect(inner, fillCol);
    }

    private void Draw2DCanvasToolPreview(Rect canvasRect, bool isFullscreen)
    {
        Event ev = Event.current;
        if (ev == null || ev.type != EventType.Repaint) return;
        float lineW = isFullscreen ? 2.4f : 1.9f;

        // 0. Vẽ các nút kéo đổi kích thước / di chuyển cho vùng hình đang chọn
        if (selectedRegionIndex >= 0 && selectedRegionIndex < customDrawnRegions.Count &&
            activeDrawTool != CanvasDrawTool.Freehand && activePolygonVerticesUV.Count == 0)
        {
            var sel = customDrawnRegions[selectedRegionIndex];
            float hSize = isFullscreen ? 4.5f : 3.8f;
            Color hFill = Color.white;
            Color hBorder = new Color(0.06f, 0.14f, 0.24f, 0.98f);
            Color cornerFill = new Color(1.0f, 0.88f, 0.20f, 1.0f);

            if (sel.shapeType == CanvasDrawTool.Circle)
            {
                Vector2[] edges = GetCircleEdgeHandlesScreenPx(sel, canvasRect);
                for (int i = 0; i < edges.Length; i++)
                {
                    DrawResizeHandleSquare(edges[i], hSize, hFill, hBorder);
                }
                Vector2 centerPx = new Vector2(canvasRect.x + sel.centerUV.x * canvasRect.width, canvasRect.y + sel.centerUV.y * canvasRect.height);
                DrawResizeHandleSquare(centerPx, hSize, cornerFill, hBorder);
            }
            else if (sel.shapeType == CanvasDrawTool.Rectangle)
            {
                Vector2[] handles = GetRectHandlesScreenPx(sel.rectUV, canvasRect);
                for (int i = 0; i < handles.Length; i++)
                {
                    DrawResizeHandleSquare(handles[i], hSize, (i % 2 == 0) ? cornerFill : hFill, hBorder);
                }
            }
            else if (sel.verticesUV != null && sel.verticesUV.Count >= 3)
            {
                Rect bUV = ComputePolygonBoundsUV(sel.verticesUV);
                Vector2 pTL = new Vector2(canvasRect.x + bUV.xMin * canvasRect.width, canvasRect.y + bUV.yMin * canvasRect.height);
                Vector2 pTR = new Vector2(canvasRect.x + bUV.xMax * canvasRect.width, canvasRect.y + bUV.yMin * canvasRect.height);
                Vector2 pBR = new Vector2(canvasRect.x + bUV.xMax * canvasRect.width, canvasRect.y + bUV.yMax * canvasRect.height);
                Vector2 pBL = new Vector2(canvasRect.x + bUV.xMin * canvasRect.width, canvasRect.y + bUV.yMax * canvasRect.height);
                Color boxCol = new Color(1.0f, 0.88f, 0.20f, 0.55f);
                DrawGuiDashedLine2D(pTL, pTR, boxCol, 1.2f, 5f, 4f);
                DrawGuiDashedLine2D(pTR, pBR, boxCol, 1.2f, 5f, 4f);
                DrawGuiDashedLine2D(pBR, pBL, boxCol, 1.2f, 5f, 4f);
                DrawGuiDashedLine2D(pBL, pTL, boxCol, 1.2f, 5f, 4f);
                DrawResizeHandleSquare(pTL, hSize, cornerFill, hBorder);
                DrawResizeHandleSquare(pTR, hSize, cornerFill, hBorder);
                DrawResizeHandleSquare(pBR, hSize, cornerFill, hBorder);
                DrawResizeHandleSquare(pBL, hSize, cornerFill, hBorder);

                for (int i = 0; i < sel.verticesUV.Count; i++)
                {
                    Vector2 vp = new Vector2(canvasRect.x + sel.verticesUV[i].x * canvasRect.width, canvasRect.y + sel.verticesUV[i].y * canvasRect.height);
                    DrawResizeHandleSquare(vp, hSize - 0.5f, hFill, hBorder);
                }
            }
        }

        // 1. Xem trước hình tròn hoặc hình vuông / chữ nhật đang kéo chuột (mượt bằng texture anti-alias / solid rect)
        if (isDraggingShape && draggingOnFullscreen == isFullscreen &&
            (activeDrawTool == CanvasDrawTool.Circle || activeDrawTool == CanvasDrawTool.Rectangle))
        {
            Color previewBorder = new Color(1.0f, 0.88f, 0.18f, 0.98f);
            Color previewFill = new Color(0.18f, 0.88f, 1.0f, 0.24f);

            if (activeDrawTool == CanvasDrawTool.Circle)
            {
                ComputeClampedCircleGeometry(shapeStartUV, shapeCurrentUV, canvasRect, out float radiusPx, out _);
                Vector2 centerPx = new Vector2(canvasRect.x + shapeStartUV.x * canvasRect.width, canvasRect.y + shapeStartUV.y * canvasRect.height);
                if (radiusPx >= 2f)
                {
                    DrawGuiSmoothCircle2D(centerPx, radiusPx, radiusPx, previewFill, previewBorder);
                }
                DrawGuiSolidRect(new Rect(centerPx.x - 3f, centerPx.y - 3f, 6f, 6f), previewBorder);
            }
            else
            {
                bool lockSquare = ev.shift;
                Rect rUV = ComputeRectangleUV(shapeStartUV, shapeCurrentUV, canvasRect, lockSquare);
                Rect rPx = new Rect(
                    canvasRect.x + rUV.x * canvasRect.width,
                    canvasRect.y + rUV.y * canvasRect.height,
                    rUV.width * canvasRect.width,
                    rUV.height * canvasRect.height
                );
                if (rPx.width >= 2f && rPx.height >= 2f)
                {
                    DrawGuiSolidRect(rPx, previewFill);
                    DrawGuiRectOutline2D(rPx, previewBorder, lineW);
                }
            }
        }

        // 2. Xem trước đa giác đang chấm đỉnh (tô nền mờ trực tiếp khi >= 3 đỉnh)
        if (activeDrawTool == CanvasDrawTool.Polygon && activePolygonVerticesUV.Count > 0)
        {
            Color polyFillCol = new Color(0.18f, 0.88f, 1.0f, 0.20f);
            Color polyEdgeCol = new Color(1.0f, 0.88f, 0.18f, 0.98f);
            Color rubberCol = new Color(0.25f, 0.95f, 1.0f, 0.88f);

            List<Vector2> ptsPx = new List<Vector2>(activePolygonVerticesUV.Count + 1);
            for (int i = 0; i < activePolygonVerticesUV.Count; i++)
            {
                ptsPx.Add(new Vector2(
                    canvasRect.x + activePolygonVerticesUV[i].x * canvasRect.width,
                    canvasRect.y + activePolygonVerticesUV[i].y * canvasRect.height
                ));
            }
            bool mouseIn = canvasRect.Contains(ev.mousePosition);
            if (ptsPx.Count >= 3)
            {
                DrawGuiFilledPolygonScreen2D(ptsPx, polyFillCol, isFullscreen ? 2.5f : 3.0f);
            }
            for (int i = 0; i < ptsPx.Count - 1; i++)
            {
                DrawGuiLine2D(ptsPx[i], ptsPx[i + 1], polyEdgeCol, lineW);
            }

            Vector2 firstPx = ptsPx[0];
            Vector2 lastPx = ptsPx[ptsPx.Count - 1];
            if (mouseIn)
            {
                DrawGuiDashedLine2D(lastPx, ev.mousePosition, rubberCol, 1.6f, 6f, 4f);
                if (ptsPx.Count >= 2)
                {
                    DrawGuiDashedLine2D(ev.mousePosition, firstPx, new Color(0.35f, 1f, 0.45f, 0.65f), 1.3f, 5f, 5f);
                }
            }

            for (int i = 0; i < ptsPx.Count; i++)
            {
                Vector2 vp = ptsPx[i];
                float r = (i == 0 && ptsPx.Count >= 3) ? 5f : 3.5f;
                Color vc = (i == 0 && ptsPx.Count >= 3) ? new Color(0.30f, 1.0f, 0.45f, 1f) : polyEdgeCol;
                DrawGuiSolidRect(new Rect(vp.x - r, vp.y - r, r * 2f, r * 2f), vc);
            }

            if (ptsPx.Count >= 3)
            {
                bool hoveringFirst = mouseIn && Vector2.Distance(ev.mousePosition, firstPx) <= 13f;
                float snapR = hoveringFirst ? 11f : 8f;
                DrawGuiSmoothCircle2D(firstPx, snapR, snapR, new Color(0.30f, 1.0f, 0.45f, hoveringFirst ? 0.25f : 0f), new Color(0.30f, 1.0f, 0.45f, 0.96f));
            }
        }

        // 3. Vòng tròn xem trước kích cỡ cọ phun quanh con trỏ chuột (ẩn khi đang ở chế độ Sửa/Kéo đổi cỡ)
        if (activeDrawTool != CanvasDrawTool.Select && activeRegionEditMode == RegionEditMode.None && canvasRect.Contains(ev.mousePosition))
        {
            float brushRadiusPx = Mathf.Max(4.5f, GetBrushDiameterPxOnCanvas(canvasRect, sprayWidthMm) * 0.5f);
            Color cursorCol = activeDrawTool == CanvasDrawTool.Freehand
                ? new Color(0.15f, 0.95f, 1.0f, 0.92f)
                : new Color(1.0f, 0.88f, 0.20f, 0.92f);
            DrawGuiSmoothCircle2D(ev.mousePosition, brushRadiusPx, brushRadiusPx, Color.clear, cursorCol);
            DrawGuiSolidRect(new Rect(ev.mousePosition.x - 1.5f, ev.mousePosition.y - 1.5f, 3f, 3f), cursorCol);
        }
    }

    private void UndoLast2DCanvasStrokeOrShape()
    {
        hasInitializedDefaultStrokes = true;
        if (activeDrawTool == CanvasDrawTool.Polygon && activePolygonVerticesUV.Count > 0)
        {
            activePolygonVerticesUV.RemoveAt(activePolygonVerticesUV.Count - 1);
            customDrawStatusMsg = $"Đã bỏ đỉnh cuối của đa giác (còn {activePolygonVerticesUV.Count} đỉnh).";
            return;
        }

        bool undoRegion = false;
        if (undoHistoryIsRegion.Count > 0)
        {
            undoRegion = undoHistoryIsRegion[undoHistoryIsRegion.Count - 1];
            undoHistoryIsRegion.RemoveAt(undoHistoryIsRegion.Count - 1);
        }
        else if (customDrawnRegions.Count > 0 && customDrawnStrokes.Count == 0)
        {
            undoRegion = true;
        }

        if (undoRegion && customDrawnRegions.Count > 0)
        {
            customDrawnRegions.RemoveAt(customDrawnRegions.Count - 1);
            selectedRegionIndex = Mathf.Clamp(selectedRegionIndex, -1, customDrawnRegions.Count - 1);
            activeRegionEditMode = RegionEditMode.None;
            customDrawStatusMsg = $"Đã hoàn tác vùng vừa vẽ (còn {customDrawnRegions.Count} vùng).";
        }
        else if (customDrawnStrokes.Count > 0)
        {
            customDrawnStrokes.RemoveAt(customDrawnStrokes.Count - 1);
            CaptureCurrentStrokesToCanonical();
        }

        if (customDrawnStrokes.Count == 0 && customDrawnRegions.Count == 0)
        {
            ClearAll2DAnd3DTrajectoryLines();
        }
        else
        {
            InvalidateGeneratedTrajectoryOnEdit();
        }
    }

    private Vector2 fullscreenSidebarScroll = Vector2.zero;
    private float fullscreenSidebarWidth = 480f;
    private bool isResizingFullscreenSidebar = false;

    private void DrawFullscreen2DCanvasOverlay(float topSafeY)
    {
        EnsureCanonicalJeansCanvasInitialized();

        Event ev = Event.current;
        if (ev != null && ev.type == EventType.KeyDown && ev.keyCode == KeyCode.Escape)
        {
            if (isDraggingShape || activePolygonVerticesUV.Count > 0 || activeRegionEditMode != RegionEditMode.None)
            {
                isDraggingShape = false;
                activeRegionEditMode = RegionEditMode.None;
                activeEditHandleIndex = -1;
                ReleaseCanvasHotControl();
                activePolygonVerticesUV.Clear();
                customDrawStatusMsg = "Đã hủy thao tác đang vẽ.";
                ev.Use();
                return;
            }
            isDrawingOnCanvas = false;
            currentDrawingStroke = null;
            showFullscreen2DCanvas = false;
            ev.Use();
            return;
        }

        Rect fullRect = new Rect(0f, topSafeY, Screen.width, Mathf.Max(240f, Screen.height - topSafeY));
        DraggableUIManager.RegisterBlockingRect(fullRect);

        // Nền tối toàn màn hình cho chế độ vẽ chuyên sâu
        DrawGuiSolidRect(fullRect, new Color(0.05f, 0.07f, 0.10f, 0.98f));

        Texture2D activeCamTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out string camSourceLabel);
        canvasUsingRealCamera = (activeCamTex != null);
        Vector2[] curLandmarksUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, canvasUsingRealCamera, out bool hasRealLandmarks);

        string activeSampleNow = GetActiveCloudSampleName();
        if (canvasUsingRealCamera && !isDrawingOnCanvas && !isDraggingShape && activePolygonVerticesUV.Count == 0 && preserveCustomDrawingAcrossSamples && canonicalPatternStrokes.Count > 0)
        {
            if (hasRealLandmarks && curLandmarksUV != null && strokesProjectedSampleName != activeSampleNow)
            {
                if (activeTrajectoryMethod == "P1" || activeTrajectoryMethod == "P2" || activeTrajectoryMethod == "P3" || activeTrajectoryMethod == "P4")
                {
                    GenerateAndApplyMethodTrajectory(activeTrajectoryMethod);
                }
                else
                {
                    ReprojectCanonicalPatternOntoCurrentCameraImage();
                }
            }
            else if (!hasRealLandmarks && !string.IsNullOrEmpty(strokesProjectedSampleName) && strokesProjectedSampleName != activeSampleNow)
            {
                customDrawnStrokes.Clear();
                strokesReferenceLandmarksUV = null;
                strokesProjectedSampleName = "";
            }
        }

        string methodBadge = activeTrajectoryMethod == "P1" ? " | Đang xem: P1 (Zigzag)"
            : (activeTrajectoryMethod == "P2" ? " | Đang xem: P2 (Trục giữa)"
            : (activeTrajectoryMethod == "P3" ? " | Đang xem: P3 (12 Mốc Barycentric)"
            : (activeTrajectoryMethod == "P4" ? " | Đang xem: P4 (ICP/TPS)" : " | Đang xem: Vẽ tay")));

        // 1. Thanh tiêu đề trên cùng của bảng vẽ phóng to
        float topBarH = 40f;
        Rect topBarRect = new Rect(fullRect.x + 12f, fullRect.y + 8f, fullRect.width - 24f, topBarH);
        GUI.DrawTexture(topBarRect, AntigravityUITheme.TexBgCard);
        GUI.Label(new Rect(topBarRect.x + 14f, topBarRect.y + 4f, 420f, 20f), "BẢNG VẼ MẪU PHUN TOÀN MÀN HÌNH (2D -> 3D)", AntigravityUITheme.StyleCardHeader);
        string activeToolHint = activeDrawTool == CanvasDrawTool.Select ? "Sửa vùng: Click chọn để di chuyển, kéo nút để đổi cỡ"
            : (activeDrawTool == CanvasDrawTool.Freehand ? "Vẽ tay: Kéo chuột trái"
            : (activeDrawTool == CanvasDrawTool.Circle ? "Hình tròn: Kéo từ tâm ra ngoài"
            : (activeDrawTool == CanvasDrawTool.Rectangle ? "Hình vuông/CN: Kéo chéo góc (Shift = Vuông)"
            : "Đa giác: Click trái thêm đỉnh, Click phải/Enter để đóng")));
        GUI.Label(new Rect(topBarRect.x + 14f, topBarRect.y + 21f, 680f, 16f), $"{camSourceLabel}{methodBadge} | Cọ {sprayWidthMm:F1}mm | {activeToolHint}", AntigravityUITheme.StyleLabelCyan);

        float btnRight = topBarRect.xMax - 10f;
        btnRight -= 130f;
        if (GUI.Button(new Rect(btnRight, topBarRect.y + 6f, 130f, 28f), "Thu nhỏ (ESC)", AntigravityUITheme.StyleButton))
        {
            isDrawingOnCanvas = false;
            currentDrawingStroke = null;
            isResizingFullscreenSidebar = false;
            showFullscreen2DCanvas = false;
            return;
        }
        AiServerClient camClient = EnsureAiClient();
        bool prevEn = GUI.enabled;
        GUI.enabled = prevEn && camClient != null && !camClient.isProcessing;
        btnRight -= 104f;
        if (GUI.Button(new Rect(btnRight, topBarRect.y + 6f, 96f, 28f), "Chụp cam", AntigravityUITheme.StyleButton))
        {
            camClient.CapturePreview();
        }
        GUI.enabled = prevEn;

        // 2. Bố cục: Khung vẽ lớn bên trái + Thanh kéo đổi độ rộng + Bảng công cụ rộng bên phải
        float contentY = topBarRect.yMax + 8f;
        float contentH = Mathf.Max(180f, fullRect.yMax - contentY - 10f);
        float minSideW = 380f;
        float maxSideW = Mathf.Clamp(fullRect.width * 0.58f, 480f, 760f);
        float splitterW = 10f;

        if (fullscreenSidebarWidth < minSideW)
            fullscreenSidebarWidth = Mathf.Clamp(fullRect.width * 0.34f, 460f, 560f);
        float sideW = Mathf.Clamp(fullscreenSidebarWidth, minSideW, maxSideW);
        float canvasAreaW = Mathf.Max(300f, fullRect.width - sideW - splitterW - 24f);

        Rect canvasAreaRect = new Rect(fullRect.x + 12f, contentY, canvasAreaW, contentH);
        Rect splitterRect = new Rect(canvasAreaRect.xMax, contentY, splitterW, contentH);
        Rect sideAreaRect = new Rect(splitterRect.xMax, contentY, sideW, contentH);

        // Xử lý kéo thanh ngăn cách để người dùng tự mở rộng/thu hẹp bảng công cụ phóng to
        if (ev != null)
        {
            if (ev.type == EventType.MouseDown && ev.button == 0 && splitterRect.Contains(ev.mousePosition))
            {
                isResizingFullscreenSidebar = true;
                ev.Use();
            }
            else if (isResizingFullscreenSidebar && ev.type == EventType.MouseDrag && ev.button == 0)
            {
                float targetSideW = (fullRect.xMax - 12f) - ev.mousePosition.x - splitterW * 0.5f;
                fullscreenSidebarWidth = Mathf.Clamp(targetSideW, minSideW, maxSideW);
                sideW = fullscreenSidebarWidth;
                canvasAreaW = Mathf.Max(300f, fullRect.width - sideW - splitterW - 24f);
                canvasAreaRect.width = canvasAreaW;
                splitterRect.x = canvasAreaRect.xMax;
                sideAreaRect.x = splitterRect.xMax;
                sideAreaRect.width = sideW;
                ev.Use();
            }
            else if (isResizingFullscreenSidebar && ev.type == EventType.MouseUp && ev.button == 0)
            {
                isResizingFullscreenSidebar = false;
                ev.Use();
            }
        }

        // Vẽ thanh kéo ngăn cách rõ nét
        bool splitterHover = ev != null && splitterRect.Contains(ev.mousePosition);
        DrawGuiSolidRect(splitterRect, (isResizingFullscreenSidebar || splitterHover)
            ? new Color(0.22f, 0.72f, 0.98f, 0.45f)
            : new Color(0.14f, 0.18f, 0.25f, 0.85f));
        Rect gripRect = new Rect(splitterRect.x + 3f, splitterRect.y + contentH * 0.5f - 28f, 4f, 56f);
        DrawGuiSolidRect(gripRect, (isResizingFullscreenSidebar || splitterHover)
            ? new Color(0.45f, 0.92f, 1.0f, 0.95f)
            : new Color(0.45f, 0.55f, 0.68f, 0.70f));

        DrawGuiSolidRect(canvasAreaRect, new Color(0.07f, 0.09f, 0.13f, 1f));

        // Tính khung ảnh giữ nguyên 100% tỷ lệ gốc của camera, phóng tối đa vừa khít vùng vẽ lớn
        float pad = 16f;
        float availW = Mathf.Max(200f, canvasAreaRect.width - pad * 2f);
        float availH = Mathf.Max(160f, canvasAreaRect.height - pad * 2f);
        float aspect = (activeCamTex != null && activeCamTex.width > 0 && activeCamTex.height > 0)
            ? ((float)activeCamTex.height / activeCamTex.width)
            : 0.65f;

        float drawW = availW;
        float drawH = drawW * aspect;
        if (drawH > availH)
        {
            drawH = availH;
            drawW = drawH / Mathf.Max(0.1f, aspect);
        }

        Rect canvasRect = new Rect(
            canvasAreaRect.x + (canvasAreaRect.width - drawW) * 0.5f,
            canvasAreaRect.y + (canvasAreaRect.height - drawH) * 0.5f,
            drawW,
            drawH
        );

        // Viền khung canvas
        DrawGuiSolidRect(new Rect(canvasRect.x - 2f, canvasRect.y - 2f, canvasRect.width + 4f, canvasRect.height + 4f), new Color(0.20f, 0.72f, 0.95f, 0.65f));
        if (activeCamTex != null)
        {
            GUI.DrawTexture(canvasRect, activeCamTex, ScaleMode.StretchToFill);
        }
        else
        {
            DrawGuiSolidRect(canvasRect, new Color(0.08f, 0.10f, 0.14f, 1f));
            GUI.Label(
                new Rect(canvasRect.x + 24f, canvasRect.y + canvasRect.height * 0.45f, canvasRect.width - 48f, 48f),
                "Chưa có ảnh thực tế từ camera. Bấm 'Chụp cam' hoặc nạp file .PLY để hiển thị.",
                AntigravityUITheme.StyleLabelMuted
            );
        }

        // Xử lý chuột vẽ mịn (Vẽ tay / Tròn / Vuông / Đa giác) trên khung phóng to
        Handle2DCanvasMouseInput(canvasRect, isFullscreen: true, hasRealLandmarks, curLandmarksUV);

        // Vẽ 12 mốc AI + khung nối giải phẫu mờ trên khung phóng to
        if (curLandmarksUV != null && curLandmarksUV.Length == 12)
        {
            int numPairs = LandmarkSkeletonPairs.GetLength(0);
            Color skelCol = new Color(1f, 0.78f, 0.15f, 0.42f);
            for (int k = 0; k < numPairs; k++)
            {
                int i0 = LandmarkSkeletonPairs[k, 0];
                int i1 = LandmarkSkeletonPairs[k, 1];
                Vector2 s0 = new Vector2(canvasRect.x + curLandmarksUV[i0].x * canvasRect.width, canvasRect.y + curLandmarksUV[i0].y * canvasRect.height);
                Vector2 s1 = new Vector2(canvasRect.x + curLandmarksUV[i1].x * canvasRect.width, canvasRect.y + curLandmarksUV[i1].y * canvasRect.height);
                DrawGuiLine2D(s0, s1, skelCol, 1.5f);
            }
            for (int i = 0; i < curLandmarksUV.Length; i++)
            {
                Vector2 uv = curLandmarksUV[i];
                Vector2 ptScreen = new Vector2(canvasRect.x + uv.x * canvasRect.width, canvasRect.y + uv.y * canvasRect.height);
                DrawGuiSolidRect(new Rect(ptScreen.x - 5f, ptScreen.y - 5f, 10f, 10f), new Color(0.05f, 0.05f, 0.08f, 0.92f));
                DrawGuiSolidRect(new Rect(ptScreen.x - 3.5f, ptScreen.y - 3.5f, 7f, 7f), new Color(1f, 0.82f, 0.12f, 1f));
                GUI.Label(new Rect(ptScreen.x + 6f, ptScreen.y - 8f, 28f, 16f), $"M{i}", AntigravityUITheme.StyleLabelYellow);
            }
        }

        // Vẽ tách biệt 2 lớp trên khung phóng to: (1) CHỖ CẦN PHUN (R & R_safe) và (2) ĐƯỜNG QUỸ ĐẠO (ON / OFF)
        DrawTargetAndTrajectoryOverlaysOn2DCanvas(canvasRect, isFullscreen: true);
        Draw2DCanvasToolPreview(canvasRect, isFullscreen: true);

        // 3. Bảng điều khiển bên phải trong chế độ toàn màn hình
        GUILayout.BeginArea(sideAreaRect, AntigravityUITheme.CardBoxStyle());
        fullscreenSidebarScroll = GUILayout.BeginScrollView(fullscreenSidebarScroll);

        GUILayout.Label("HIỂN THỊ LỚP (QUỸ ĐẠO vs ĐƯỜNG PHUN)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(showSprayZones3D ? "Chỗ phun: BẬT" : "Chỗ phun: TẮT",
            showSprayZones3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showSprayZones3D = !showSprayZones3D;
            MarkSettingsDirty();
        }
        if (GUILayout.Button(showTurnaroundConnectors ? "Quỹ đạo: BẬT" : "Quỹ đạo: TẮT",
            showTurnaroundConnectors ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showTurnaroundConnectors = !showTurnaroundConnectors;
        }
        if (GUILayout.Button(showGizmos ? "Đường phun: BẬT" : "Đường phun: TẮT",
            showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            showGizmos = !showGizmos;
            MarkSettingsDirty();
        }
        GUILayout.EndHorizontal();
        if (activeTrajectoryMethod == "P2" || showP2ParameterPanel)
        {
            showP2SkeletonAxis = GUILayout.Toggle(showP2SkeletonAxis, " Hiện trục giữa ống quần Skeleton (P2: Hồng tím)", AntigravityUITheme.StyleToggle);
        }
        if (activeTrajectoryMethod == "P3" || showP3ParameterPanel)
        {
            showP3TriangulationMesh = GUILayout.Toggle(showP3TriangulationMesh, " Hiện lưới 32 tam giác Barycentric (P3: Xanh ngọc)", AntigravityUITheme.StyleToggle);
        }
        if (activeTrajectoryMethod == "P4" || showP4ParameterPanel)
        {
            showP4RegisteredCloud3D = GUILayout.Toggle(showP4RegisteredCloud3D, " Hiện đám mây 3D đã đăng ký (P4: Xanh chanh)", AntigravityUITheme.StyleToggle);
        }
        GUILayout.Label("Vàng cam: Chỗ cần phun | Đứt cam: Đường quỹ đạo (Tắt súng) | Cyan: Đường phun (Bật súng)", AntigravityUITheme.StyleLabelMuted);
        GUILayout.EndVertical();
        GUILayout.Space(6);

        GUILayout.Label("1. CÔNG CỤ VẼ & MẪU CÓ SẴN", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        Draw2DCanvasBrushAndShapeToolbar(isFullscreen: true);
        GUILayout.Space(4);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Size mẫu: {activeSprayTemplateSize} (AI: {detectedJeansSize})", AntigravityUITheme.StyleLabelGreen);
        foreach (string sz in new[] { "S", "M", "L" })
        {
            GUIStyle st = activeSprayTemplateSize == sz ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
            if (GUILayout.Button(sz, st, GUILayout.Width(34), GUILayout.Height(24)))
            {
                ChangeSprayTemplateSize(sz);
            }
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(4);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Râu mèo + Đùi", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            LoadPresetWhiskersAndThighOn2DCanvas();
            InvalidateGeneratedTrajectoryOnEdit();
            customDrawStatusMsg = "Đã nạp mẫu Râu mèo + Đùi. Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
        }
        if (GUILayout.Button("Zigzag 2 ống", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            LoadPresetZigzag2LegsOn2DCanvas();
            InvalidateGeneratedTrajectoryOnEdit();
            customDrawStatusMsg = "Đã nạp mẫu Zigzag 2 ống. Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Hoàn tác nét cuối", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
        {
            UndoLast2DCanvasStrokeOrShape();
        }
        if (GUILayout.Button("Xóa hết", AntigravityUITheme.StyleButton, GUILayout.Width(86), GUILayout.Height(26)))
        {
            ClearAll2DAnd3DTrajectoryLines();
        }
        GUILayout.EndHorizontal();
        GUILayout.EndVertical();

        GUILayout.Space(8);
        GUILayout.Label("2. PHƯƠNG PHÁP SINH QUỸ ĐẠO", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("P1: Zigzag", activeTrajectoryMethod == "P1" ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(28)))
        {
            showP1ParameterPanel = true;
            showP2ParameterPanel = false;
            showP3ParameterPanel = false;
            showP4ParameterPanel = false;
            SyncP1ConfigFromTechParamsIfNeeded();
            GenerateAndApplyMethodTrajectory("P1");
        }
        if (GUILayout.Button("P2: Trục giữa", activeTrajectoryMethod == "P2" ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(28)))
        {
            showP1ParameterPanel = false;
            showP2ParameterPanel = true;
            showP3ParameterPanel = false;
            showP4ParameterPanel = false;
            SyncP2ConfigFromTechParamsIfNeeded();
            GenerateAndApplyMethodTrajectory("P2");
        }
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("P3: 12 Mốc", activeTrajectoryMethod == "P3" ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(28)))
        {
            showP1ParameterPanel = false;
            showP2ParameterPanel = false;
            showP3ParameterPanel = true;
            showP4ParameterPanel = false;
            SyncP3ConfigFromTechParamsIfNeeded();
            GenerateAndApplyMethodTrajectory("P3");
        }
        if (GUILayout.Button("P4: ICP/CPD", activeTrajectoryMethod == "P4" ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(28)))
        {
            showP1ParameterPanel = false;
            showP2ParameterPanel = false;
            showP3ParameterPanel = false;
            showP4ParameterPanel = true;
            SyncP4ConfigFromTechParamsIfNeeded();
            GenerateAndApplyMethodTrajectory("P4");
        }
        GUILayout.EndHorizontal();
        if (activeTrajectoryMethod == "P1" || showP1ParameterPanel)
        {
            DrawP1ParameterConfigSection();
        }
        if (activeTrajectoryMethod == "P2" || showP2ParameterPanel)
        {
            DrawP2ParameterConfigSection();
        }
        if (activeTrajectoryMethod == "P3" || showP3ParameterPanel)
        {
            DrawP3ParameterConfigSection();
        }
        if (activeTrajectoryMethod == "P4" || showP4ParameterPanel)
        {
            DrawP4ParameterConfigSection();
        }
        if (!string.IsNullOrEmpty(comparisonStatusMsg))
        {
            GUILayout.Label(comparisonStatusMsg, AntigravityUITheme.StyleLabelCyan);
        }
        GUILayout.EndVertical();

        GUILayout.Space(8);
        GUILayout.Label("3. ĐỘ CAO LỚP PHUN (Z OFFSET)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Độ cao Z: {customDrawZOffsetMm:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(130));
        float newFsZ = GUILayout.HorizontalSlider(customDrawZOffsetMm, 0f, 50f);
        if (Mathf.Abs(newFsZ - customDrawZOffsetMm) > 0.05f)
        {
            customDrawZOffsetMm = newFsZ;
            if (sprayTechParams != null) sprayTechParams.standoffMm = customDrawZOffsetMm;
            ReapplyActiveTrajectoryHeight();
        }
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("0 mm", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            customDrawZOffsetMm = 0f;
            if (sprayTechParams != null) sprayTechParams.standoffMm = customDrawZOffsetMm;
            ReapplyActiveTrajectoryHeight();
        }
        if (GUILayout.Button("10 mm", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            customDrawZOffsetMm = 10f;
            if (sprayTechParams != null) sprayTechParams.standoffMm = customDrawZOffsetMm;
            ReapplyActiveTrajectoryHeight();
        }
        if (GUILayout.Button("20 mm", AntigravityUITheme.StyleButton, GUILayout.Height(24)))
        {
            customDrawZOffsetMm = 20f;
            if (sprayTechParams != null) sprayTechParams.standoffMm = customDrawZOffsetMm;
            ReapplyActiveTrajectoryHeight();
        }
        GUILayout.EndHorizontal();
        GUILayout.EndVertical();

        GUILayout.Space(8);
        GUILayout.Label("4. LƯU / NẠP MẪU (.JSON)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.BeginHorizontal();
        GUILayout.Label("Tên file:", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(58));
        saveTemplateFileName = GUILayout.TextField(saveTemplateFileName, AntigravityUITheme.StyleTextField, GUILayout.Height(24));
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Lưu mẫu", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
            ExportSprayTemplateWithDialog();
        if (GUILayout.Button("Nạp mẫu...", AntigravityUITheme.StyleButton, GUILayout.Height(26)))
            ImportSprayTemplateWithDialog();
        GUILayout.EndHorizontal();
        if (availableSprayTemplateFiles != null && availableSprayTemplateFiles.Count > 0)
        {
            GUILayout.Label("Mẫu có sẵn:", AntigravityUITheme.StyleLabelMuted);
            int maxShow = Mathf.Min(availableSprayTemplateFiles.Count, 4);
            for (int f = 0; f < maxShow; f++)
            {
                string fn = availableSprayTemplateFiles[f];
                if (GUILayout.Button(fn, AntigravityUITheme.StyleButton, GUILayout.Height(22)))
                {
                    saveTemplateFileName = fn;
                    ImportSprayTemplateFromFile(Path.Combine(GetSprayTemplateDirectoryFullPath(), fn + ".json"));
                }
            }
        }
        GUILayout.EndVertical();

        if (!string.IsNullOrEmpty(customDrawStatusMsg))
        {
            GUILayout.Space(6);
            GUILayout.Label(customDrawStatusMsg, AntigravityUITheme.StyleLabelGreen);
        }

        GUILayout.EndScrollView();
        GUILayout.EndArea();
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
        if ((customDrawnStrokes == null || customDrawnStrokes.Count == 0) &&
            (customDrawnRegions == null || customDrawnRegions.Count == 0))
        {
            customDrawStatusMsg = "Chưa có nét vẽ hoặc vùng vẽ để lưu.";
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
        if ((customDrawnStrokes == null || customDrawnStrokes.Count == 0) &&
            (customDrawnRegions == null || customDrawnRegions.Count == 0))
        {
            customDrawStatusMsg = "Chưa có nét vẽ hoặc vùng vẽ để lưu.";
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
                sprayWidthMm = sprayWidthMm,
                strokes = new List<Stroke2D>(),
                regions = new List<Region2DJson>()
            };

            Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
            bool usingRealCam = (camTex != null);
            Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, usingRealCam, out bool hasRealLm);
            Vector2[] canonLmUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
            float camAspect = GetActiveCameraMetricAspect(plyLoader, nativeW, nativeH);

            List<List<Vector2>> sourceStrokesToSave = (canonicalPatternStrokes != null && canonicalPatternStrokes.Count > 0)
                ? canonicalPatternStrokes
                : customDrawnStrokes;
            bool alreadyCanonical = (sourceStrokesToSave == canonicalPatternStrokes);

            foreach (var st in sourceStrokesToSave)
            {
                if (st == null || st.Count == 0) continue;
                var strokeObj = new Stroke2D();
                foreach (var pt in st)
                {
                    Vector2 outPt = (!alreadyCanonical && usingRealCam && hasRealLm && camLmUV != null)
                        ? WarpUV2D(pt, camLmUV, canonLmUV, camAspect, 1f)
                        : pt;
                    strokeObj.points.Add(new StrokePoint2D(outPt.x, outPt.y));
                }
                template.strokes.Add(strokeObj);
            }

            for (int r = 0; r < customDrawnRegions.Count; r++)
            {
                var reg = customDrawnRegions[r];
                if (reg == null || reg.verticesUV == null || reg.verticesUV.Count < 3) continue;
                var regJson = new Region2DJson
                {
                    shapeType = (int)reg.shapeType,
                    brushWidthMm = reg.brushWidthMm,
                    centerU = reg.centerUV.x,
                    centerV = reg.centerUV.y,
                    radiusU = reg.radiusUV.x,
                    radiusV = reg.radiusUV.y,
                    points = new List<StrokePoint2D>(reg.verticesUV.Count)
                };
                for (int i = 0; i < reg.verticesUV.Count; i++)
                {
                    regJson.points.Add(new StrokePoint2D(reg.verticesUV[i].x, reg.verticesUV[i].y));
                }
                template.regions.Add(regJson);
            }

            string json = JsonUtility.ToJson(template, true);
            File.WriteAllText(fullPath, json);
            RefreshAvailableSprayTemplates();
            customDrawStatusMsg = $"Đã lưu mẫu: {Path.GetFileName(fullPath)} ({template.regions.Count} vùng, {template.strokes.Count} nét, Size {activeSprayTemplateSize})";
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
            bool hasTplStrokes = template != null && template.strokes != null && template.strokes.Count > 0;
            bool hasTplRegions = template != null && template.regions != null && template.regions.Count > 0;
            if (template != null && (hasTplStrokes || hasTplRegions))
            {
                hasInitializedDefaultStrokes = true;
                isDrawingOnCanvas = false;
                currentDrawingStroke = null;
                isDraggingShape = false;
                activePolygonVerticesUV.Clear();
                customDrawnRegions.Clear();
                undoHistoryIsRegion.Clear();
                customDrawnStrokes.Clear();

                if (hasTplStrokes)
                {
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
                }

                if (hasTplRegions)
                {
                    foreach (var rj in template.regions)
                    {
                        if (rj == null || rj.points == null || rj.points.Count < 3) continue;
                        var reg = new DrawnRegion2D
                        {
                            shapeType = (CanvasDrawTool)Mathf.Clamp(rj.shapeType, 1, 3),
                            brushWidthMm = rj.brushWidthMm > 0.4f ? rj.brushWidthMm : sprayWidthMm,
                            centerUV = new Vector2(rj.centerU, rj.centerV),
                            radiusUV = new Vector2(rj.radiusU, rj.radiusV),
                            verticesUV = new List<Vector2>(rj.points.Count)
                        };
                        float minU = float.MaxValue, maxU = float.MinValue, minV = float.MaxValue, maxV = float.MinValue;
                        foreach (var p in rj.points)
                        {
                            Vector2 v = new Vector2(p.u, p.v);
                            reg.verticesUV.Add(v);
                            if (v.x < minU) minU = v.x;
                            if (v.x > maxU) maxU = v.x;
                            if (v.y < minV) minV = v.y;
                            if (v.y > maxV) maxV = v.y;
                        }
                        reg.rectUV = new Rect(minU, minV, maxU - minU, maxV - minV);
                        customDrawnRegions.Add(reg);
                        undoHistoryIsRegion.Add(true);
                    }
                }

                string tplSize = string.IsNullOrEmpty(template.size) ? activeSprayTemplateSize : template.size;
                if (string.IsNullOrEmpty(tplSize)) tplSize = "M";

                // Nếu mẫu JSON được lưu ở Size khác với activeSprayTemplateSize hiện tại, chuẩn hóa về hệ mốc Canonical của activeSprayTemplateSize
                if (tplSize != activeSprayTemplateSize)
                {
                    Vector2[] srcTplCanon = GetCanonicalLandmarks12UV(tplSize);
                    Vector2[] dstCurCanon = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
                    WarpStrokesInPlace(customDrawnStrokes, srcTplCanon, dstCurCanon, 1f, 1f);
                }

                // Lưu bản gốc trong không gian chuẩn Canonical để tự thích nghi khi đổi sang quần khác
                canonicalPatternStrokes.Clear();
                for (int s = 0; s < customDrawnStrokes.Count; s++)
                {
                    canonicalPatternStrokes.Add(new List<Vector2>(customDrawnStrokes[s]));
                }

                ReprojectCanonicalPatternOntoCurrentCameraImage();

                if (template.zOffsetMm > 0f)
                {
                    customDrawZOffsetMm = template.zOffsetMm;
                    if (sprayTechParams != null) sprayTechParams.standoffMm = customDrawZOffsetMm;
                }
                if (template.sprayWidthMm > 0.4f)
                {
                    UpdateSprayWidth(template.sprayWidthMm);
                }
                saveTemplateFileName = Path.GetFileNameWithoutExtension(fullPath);
                sprayTemplateDirectory = Path.GetDirectoryName(fullPath);
                RefreshAvailableSprayTemplates();

                // Nếu người dùng đang chọn P1/P2/P3/P4 thì áp dụng lại phương pháp đó với mẫu mới; ngược lại ánh xạ mẫu 3D trực tiếp
                if (waypoints != null) waypoints.Clear();
                ClearInkTrail();
                if (depositionHeatmapObj != null) depositionHeatmapObj.SetActive(false);
                if (activeTrajectoryMethod == "P1" || activeTrajectoryMethod == "P2" ||
                    activeTrajectoryMethod == "P3" || activeTrajectoryMethod == "P4")
                {
                    GenerateAndApplyMethodTrajectory(activeTrajectoryMethod);
                }
                else
                {
                    InvalidateGeneratedTrajectoryOnEdit();
                }

                customDrawStatusMsg = $"Đã nạp mẫu {Path.GetFileName(fullPath)} ({canonicalPatternStrokes.Count} nét -> phôi {GetActiveCloudSampleName()}). Chọn P1 / P2 / P3 / P4 để sinh quỹ đạo.";
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

        // 1. Bảo toàn vị trí giải phẫu của mẫu chuẩn khi chuyển giữa các hệ mốc chuẩn S / M / L
        Vector2[] prevCanonUV = GetCanonicalLandmarks12UV(prevSize);
        Vector2[] newCanonUV = GetCanonicalLandmarks12UV(newSize);
        if (canonicalPatternStrokes != null && canonicalPatternStrokes.Count > 0)
        {
            WarpStrokesInPlace(canonicalPatternStrokes, prevCanonUV, newCanonUV);
        }

        // 2. Chỉ co giãn nét vẽ theo khung quần tượng trưng khi không dùng ảnh camera thực tế
        if (!canvasUsingRealCamera && customDrawnStrokes != null && customDrawnStrokes.Count > 0)
        {
            WarpStrokesInPlace(customDrawnStrokes, prevCanonUV, newCanonUV);
        }

        // 3. Tái tạo hình ảnh dự phòng theo size mới (S/M/L)
        RebuildCanonicalJeansTexture();

        // 4. Chiếu lại mẫu chuẩn lên ảnh camera 2D theo Size mới và tự động ánh xạ lại quỹ đạo 3D
        if (canvasUsingRealCamera && canonicalPatternStrokes != null && canonicalPatternStrokes.Count > 0 &&
            activeTrajectoryMethod != "P1" && activeTrajectoryMethod != "P2")
        {
            ReprojectCanonicalPatternOntoCurrentCameraImage();
        }
        if ((canonicalPatternStrokes != null && canonicalPatternStrokes.Count > 0) ||
            (customDrawnStrokes != null && customDrawnStrokes.Count > 0))
        {
            ReapplyActiveTrajectoryHeight();
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
        isDraggingShape = false;
        activeRegionEditMode = RegionEditMode.None;
        activeEditHandleIndex = -1;
        selectedRegionIndex = -1;
        ReleaseCanvasHotControl();
        activePolygonVerticesUV.Clear();
        customDrawnRegions.Clear();
        undoHistoryIsRegion.Clear();
        activeTrajectoryMethod = "CUSTOM";
        showP1ParameterPanel = false;
        showP2ParameterPanel = false;
        showP3ParameterPanel = false;
        showP4ParameterPanel = false;
        lastP1Result = null;
        lastP2Result = null;
        lastP3Result = null;
        lastP4Result = null;
        templateReferenceCloud3DWorld.Clear();
        templateReferenceLandmarks3D.Clear();
        templateReferenceStrokes3DWorld.Clear();
        templateReferenceSampleName = "";
        customDrawnStrokes.Clear();
        canonicalPatternStrokes.Clear();
        projectedTargetRegionR2D.Clear();
        projectedSafeRegionRsafe2D.Clear();
        projectedP2Skeleton2D.Clear();
        projectedP3MeshTriangles2D.Clear();
        projectedP3TriangleFlipped2D.Clear();
        projectedP4RegisteredCloud2D.Clear();
        projectedTrajectoryOn2D.Clear();
        projectedTrajectoryOff2D.Clear();
        strokesReferenceLandmarksUV = null;
        strokesProjectedSampleName = "";

        isPlaying = false;
        isMovingToManualTarget = false;
        paintingStarted = false;
        SetSprayState(false);
        if (waypoints != null) waypoints.Clear();
        ClearInkTrail();
        if (depositionHeatmapObj != null) depositionHeatmapObj.SetActive(false);
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
        // Lưu bản mẫu chuẩn Canonical trước khi chiếu lên ảnh camera thực tế
        canonicalPatternStrokes.Clear();
        for (int s = 0; s < customDrawnStrokes.Count; s++)
        {
            if (customDrawnStrokes[s] != null && customDrawnStrokes[s].Count > 0)
            {
                canonicalPatternStrokes.Add(new List<Vector2>(customDrawnStrokes[s]));
            }
        }

        Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
        canvasUsingRealCamera = (camTex != null);
        if (!canvasUsingRealCamera)
        {
            strokesReferenceLandmarksUV = null;
            strokesProjectedSampleName = "";
            return;
        }

        string activeSample = GetActiveCloudSampleName();
        if (!string.IsNullOrEmpty(activeSample) &&
            (landmarks3D == null || landmarks3D.Count < 12 || loadedLandmarksSampleName != activeSample))
        {
            LoadLandmarksAndSegmentationForSample(activeSample);
        }

        if (!ReprojectCanonicalPatternOntoCurrentCameraImage())
        {
            customDrawnStrokes.Clear();
            strokesReferenceLandmarksUV = null;
            strokesProjectedSampleName = "";
            customDrawStatusMsg = "Ảnh camera chưa có 12 mốc AI. Hãy vẽ trực tiếp bằng chuột lên ảnh hoặc bấm Chụp + AI.";
        }
    }

    public void LoadPresetWhiskersAndThighOn2DCanvas()
    {
        activeTrajectoryMethod = "CUSTOM";
        isDraggingShape = false;
        activePolygonVerticesUV.Clear();
        customDrawnRegions.Clear();
        undoHistoryIsRegion.Clear();
        if (waypoints != null) waypoints.Clear();
        ClearInkTrail();
        if (depositionHeatmapObj != null) depositionHeatmapObj.SetActive(false);

        customDrawnStrokes.Clear();
        var presetStrokes = BuildDefaultCanonicalStrokesForSize(activeSprayTemplateSize);
        for (int s = 0; s < presetStrokes.Count; s++)
        {
            customDrawnStrokes.Add(new List<Vector2>(presetStrokes[s]));
        }

        ProjectCanonicalPresetStrokesOntoActiveCanvas();
        InvalidateGeneratedTrajectoryOnEdit();
    }

    public void LoadPresetZigzag2LegsOn2DCanvas()
    {
        activeTrajectoryMethod = "CUSTOM";
        isDraggingShape = false;
        activePolygonVerticesUV.Clear();
        customDrawnRegions.Clear();
        undoHistoryIsRegion.Clear();
        if (waypoints != null) waypoints.Clear();
        ClearInkTrail();
        if (depositionHeatmapObj != null) depositionHeatmapObj.SetActive(false);

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
        InvalidateGeneratedTrajectoryOnEdit();
    }

    /// <summary>
    /// Ánh xạ toàn bộ các nét vẽ trên ảnh Camera 2D thực tế [0, 1] x [0, 1] trực tiếp lên bề mặt 3D
    /// của đám mây điểm Zivid đang nạp (ưu tiên ánh xạ trực tiếp pixel RGB-D -> 3D, hỗ trợ bù đắp bề mặt
    /// & dự phòng nội suy 12 mốc giải phẫu), sau đó giải Động học Nghịch 6-DOF và cập nhật ngay quỹ đạo vận hành!
    /// </summary>
    public void ApplyCanonical2DDrawingTo3DTrajectory()
    {
        if ((customDrawnStrokes == null || customDrawnStrokes.Count == 0) &&
            (canonicalPatternStrokes == null || canonicalPatternStrokes.Count == 0) &&
            (customDrawnRegions == null || customDrawnRegions.Count == 0))
        {
            customDrawStatusMsg = "Chưa có nét vẽ hoặc vùng vẽ trên ảnh camera 2D.";
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

        if (canvasUsingRealCamera && preserveCustomDrawingAcrossSamples &&
            canonicalPatternStrokes.Count > 0 && strokesProjectedSampleName != activeCloudSample)
        {
            ReprojectCanonicalPatternOntoCurrentCameraImage();
        }

        Vector2[] camLandmarksUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, canvasUsingRealCamera, out bool hasRealLandmarks);
        bool has12Landmarks = (landmarks3D != null && landmarks3D.Count == 12 &&
            landmarksWorldPos != null && landmarksWorldPos.Count == 12 &&
            (string.IsNullOrEmpty(activeCloudSample) || loadedLandmarksSampleName == activeCloudSample));

        if (!canvasUsingRealCamera && !has12Landmarks)
        {
            customDrawStatusMsg = "Chưa có ảnh camera hoặc 12 mốc của phôi này. Chụp Zivid hoặc nạp .PLY trước.";
            return;
        }

        // Khi đã có 12 mốc 3D của phôi mới và có mẫu chuẩn canonicalPatternStrokes:
        // Ánh xạ trực tiếp từ Canonical -> Không gian 3D đẳng hướng (World XZ) bằng Lưới Barycentric 32 Tam Giác (P3)
        // để loại bỏ hoàn toàn sai số tỷ lệ khung hình ảnh 1944x1200 và đặt mẫu đúng tâm từng ống quần!
        bool useCanonical3DBarycentric = (has12Landmarks && canonicalPatternStrokes != null && canonicalPatternStrokes.Count > 0);
        Vector2[] srcCanonUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        Vector2[] dstWorldXZ = new Vector2[12];
        if (has12Landmarks)
        {
            for (int i = 0; i < 12; i++) dstWorldXZ[i] = new Vector2(landmarksWorldPos[i].x, landmarksWorldPos[i].z);
        }

        // Chuẩn bị 12 điểm mốc nguồn/đích cho nhánh dự phòng (khi 1 pixel thiếu chiều sâu hoặc chế độ canonical)
        Vector2[] srcUV = (canvasUsingRealCamera && hasRealLandmarks && camLandmarksUV != null)
            ? camLandmarksUV
            : srcCanonUV;
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

        List<List<Vector2>> activeSourceStrokes = new List<List<Vector2>>();
        bool hasUserRegions = (customDrawnRegions != null && customDrawnRegions.Count > 0);
        bool hasUserFreehand = (customDrawnStrokes != null && customDrawnStrokes.Count > 0);

        // Nếu người dùng chỉ vẽ vùng cơ bản (Tròn / Vuông / Đa giác), không trộn lẫn các nét cũ còn sót trong canonicalPatternStrokes
        if (hasUserFreehand)
        {
            if (useCanonical3DBarycentric && canonicalPatternStrokes != null && canonicalPatternStrokes.Count > 0)
            {
                for (int i = 0; i < canonicalPatternStrokes.Count; i++) activeSourceStrokes.Add(canonicalPatternStrokes[i]);
            }
            else
            {
                for (int i = 0; i < customDrawnStrokes.Count; i++) activeSourceStrokes.Add(customDrawnStrokes[i]);
            }
        }
        else if (!hasUserRegions && useCanonical3DBarycentric && canonicalPatternStrokes != null)
        {
            for (int i = 0; i < canonicalPatternStrokes.Count; i++) activeSourceStrokes.Add(canonicalPatternStrokes[i]);
        }

        // Xây dựng đường phun trực tiếp trong hệ tọa độ thực 3D World XZ (mét) cho các vùng hình cơ bản (không méo qua lưới 12 mốc!)
        List<List<Vector2>> directWorldXZStrokes = new List<List<Vector2>>();
        if (hasUserRegions)
        {
            var regionPolysXZ = GetCustomDrawnRegionPolygonsWorldXZ(plyLoader);
            for (int r = 0; r < regionPolysXZ.Count; r++)
            {
                var regionMetricPaths = BuildMetricSprayPathsForWorldPolygon(regionPolysXZ[r], sprayWidthMm);
                for (int k = 0; k < regionMetricPaths.Count; k++)
                {
                    directWorldXZStrokes.Add(regionMetricPaths[k]);
                }
            }
        }

        List<WaypointData> mappedWaypoints = new List<WaypointData>();
        float cumDistMm = 0f;
        Vector3 prevPosMm = Vector3.zero;
        float[] prevJointsRad = null;
        int wpIndex = 0;
        int solvedCount = 0;
        float maxTiltAllowedGlobal = sprayTechParams != null ? sprayTechParams.maxTiltDeg : maxGunTiltAngleDeg;

        // A. Giải quỹ đạo 3D chính xác cho các vùng hình cơ bản (Tròn / Vuông / Đa giác) theo đúng tọa độ mét World XZ
        for (int s = 0; s < directWorldXZStrokes.Count; s++)
        {
            var pathXZ = directWorldXZStrokes[s];
            if (pathXZ == null || pathXZ.Count < 2) continue;

            for (int p = 0; p < pathXZ.Count; p++)
            {
                Vector2 wXZ = pathXZ[p];
                float surfaceZ_mm = 20f;
                Vector3 surfaceNormalWorld = Vector3.up;

                if (SampleCompensatedSurface(wXZ.x, wXZ.y, out float compY, out Vector3 compNorm))
                {
                    surfaceZ_mm = Mathf.Max(14f, compY * 1000f);
                    surfaceNormalWorld = compNorm;
                }
                else if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(wXZ.x, wXZ.y, out float cloudWorldY, out Vector3 cloudNormal))
                {
                    surfaceZ_mm = Mathf.Max(14f, cloudWorldY * 1000f);
                    surfaceNormalWorld = cloudNormal;
                }
                else if (plyLoader != null)
                {
                    surfaceZ_mm = Mathf.Max(14f, (plyLoader.estimatedTableY + 0.020f) * 1000f);
                }

                if (surfaceNormalWorld.y < 0f) surfaceNormalWorld = -surfaceNormalWorld;
                surfaceNormalWorld.Normalize();

                float tiltDeg = Vector3.Angle(surfaceNormalWorld, Vector3.up);
                if (tiltDeg > maxTiltAllowedGlobal && maxTiltAllowedGlobal > 0.1f)
                {
                    Vector3 rotAx = Vector3.Cross(Vector3.up, surfaceNormalWorld);
                    if (rotAx.sqrMagnitude > 1e-6f)
                        surfaceNormalWorld = (Quaternion.AngleAxis(maxTiltAllowedGlobal, rotAx.normalized) * Vector3.up).normalized;
                }

                Vector3 surfWorld = new Vector3(wXZ.x, surfaceZ_mm * 0.001f, wXZ.y);
                Vector3 tcpWorld = surfWorld + surfaceNormalWorld * (customDrawZOffsetMm * 0.001f);
                Vector3 targetMm = globalReference != null
                    ? globalReference.InverseTransformPoint(tcpWorld) * 1000f
                    : new Vector3((1.100f - tcpWorld.z) * 1000f, -tcpWorld.x * 1000f, tcpWorld.y * 1000f);
                targetMm.z = Mathf.Clamp(targetMm.z, 18f, 450f);

                if (targetMm.magnitude > 1145f)
                {
                    float scale = 1140f / targetMm.magnitude;
                    targetMm.x *= scale;
                    targetMm.y *= scale;
                }

                Vector3 prevPosForTangent = prevPosMm;
                if (wpIndex > 0) cumDistMm += Vector3.Distance(prevPosMm, targetMm);
                prevPosMm = targetMm;

                WaypointData wp = new WaypointData
                {
                    index = wpIndex,
                    distance_mm = cumDistMm,
                    sprayOn = (p > 0),
                    segmentName = $"REGION_{s + 1}",
                    targetPosGlobal_mm = targetMm,
                    targetPosWorld_m = globalReference != null
                        ? globalReference.TransformPoint(targetMm * 0.001f)
                        : tcpWorld,
                    microNormal = surfaceNormalWorld
                };

                Vector3 normRobot = globalReference != null
                    ? globalReference.InverseTransformDirection(surfaceNormalWorld).normalized
                    : new Vector3(-surfaceNormalWorld.z, -surfaceNormalWorld.x, surfaceNormalWorld.y).normalized;
                Vector3 tanRobot = (p > 0) ? (targetMm - prevPosForTangent) : new Vector3(1f, 0f, 0f);
                if (tanRobot.sqrMagnitude < 1e-4f) tanRobot = new Vector3(1f, 0f, 0f);
                Vector3 rpyDeg = DobotCR10AKinematics.SurfaceNormalToRobotRpyDeg(normRobot, tanRobot, maxTiltAllowedGlobal);
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

        for (int s = 0; s < activeSourceStrokes.Count; s++)
        {
            var stroke = activeSourceStrokes[s];
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

                if (useCanonical3DBarycentric)
                {
                    Vector2 wXZ = TrajectoryEvaluationEngine.WarpPointBarycentric12(uv, srcCanonUV, dstWorldXZ, 1f, 1f);
                    approxWorld = new Vector3(wXZ.x, 0.020f, wXZ.y);
                    surfaceZ_mm = 20f;
                }
                else if (canvasUsingRealCamera && plyLoader != null &&
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
            activeTrajectoryMethod = "CUSTOM";
            ProjectWaypointsOntoActiveCameraCanvas(waypoints, activeCloudSample);
            if (templateReferenceCloud3DWorld.Count < 30 || templateReferenceSampleName == activeCloudSample)
            {
                CaptureReferenceTemplateCloud3DForP4(plyLoader, activeCloudSample);
            }
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
            if (dist < 1e-5f) continue;
            if (dist > stepUV)
            {
                int segs = Mathf.Clamp(Mathf.CeilToInt(dist / stepUV), 1, 32);
                for (int k = 1; k <= segs; k++)
                {
                    result.Add(Vector2.Lerp(a, b, (float)k / segs));
                }
            }
            else
            {
                result.Add(b);
            }
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
            lastP1Result = null;
            projectedTargetRegionR2D.Clear();
            projectedSafeRegionRsafe2D.Clear();
            projectedTrajectoryOn2D.Clear();
            projectedTrajectoryOff2D.Clear();
            if (preserveCustomDrawingAcrossSamples && canonicalPatternStrokes.Count > 0)
            {
                // Ẩn nét vẽ pixel của quần cũ khỏi vị trí sai trên ảnh quần mới (giữ nguyên canonicalPatternStrokes để tự khớp khi có 12 mốc)
                customDrawnStrokes.Clear();
                strokesReferenceLandmarksUV = null;
                strokesProjectedSampleName = "";
            }
            return;
        }

        try
        {
            string jsonText = File.ReadAllText(foundPath);
            LandmarkSegmentationPayload payload = JsonUtility.FromJson<LandmarkSegmentationPayload>(jsonText);
            if (payload == null) return;

            lastP1Result = null;
            projectedTargetRegionR2D.Clear();
            projectedSafeRegionRsafe2D.Clear();
            projectedTrajectoryOn2D.Clear();
            projectedTrajectoryOff2D.Clear();
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

            bool isMethodActive = (activeTrajectoryMethod == "P1" || activeTrajectoryMethod == "P2" ||
                                   activeTrajectoryMethod == "P3" || activeTrajectoryMethod == "P4");

            if (autoSwitchSprayTemplate)
            {
                SwitchSprayTemplateToSize(detectedJeansSize);
            }
            else if (preserveCustomDrawingAcrossSamples)
            {
                if (canonicalPatternStrokes.Count > 0)
                {
                    if (!isMethodActive)
                    {
                        ReprojectCanonicalPatternOntoCurrentCameraImage();
                    }
                }
                else if (!isMethodActive && customDrawnStrokes != null && customDrawnStrokes.Count > 0 &&
                         strokesReferenceLandmarksUV != null && strokesReferenceLandmarksUV.Length == 12)
                {
                    Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyForWarp, out int nw, out int nh, out _);
                    if (camTex != null)
                    {
                        Vector2[] newCamLmUV = GetActiveCanvasLandmarks12UV(plyForWarp, nw, nh, true, out bool hasNewLm);
                        if (hasNewLm && newCamLmUV != null)
                        {
                            float camAsp = GetActiveCameraMetricAspect(plyForWarp, nw, nh);
                            WarpStrokesInPlace(customDrawnStrokes, strokesReferenceLandmarksUV, newCamLmUV, camAsp, camAsp);
                            strokesReferenceLandmarksUV = (Vector2[])newCamLmUV.Clone();
                            strokesProjectedSampleName = GetActiveCloudSampleName();
                            CaptureCurrentStrokesToCanonical();
                        }
                    }
                }

                if (isMethodActive)
                {
                    GenerateAndApplyMethodTrajectory(activeTrajectoryMethod);
                }
                else if ((canonicalPatternStrokes != null && canonicalPatternStrokes.Count > 0) ||
                         (customDrawnStrokes != null && customDrawnStrokes.Count > 0))
                {
                    ReprojectCanonicalPatternOntoCurrentCameraImage();
                    InvalidateGeneratedTrajectoryOnEdit();
                }
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

        if ((canonicalPatternStrokes == null || canonicalPatternStrokes.Count == 0) &&
            (customDrawnStrokes == null || customDrawnStrokes.Count == 0))
        {
            LoadPresetWhiskersAndThighOn2DCanvas();
        }

        if (prev != size)
        {
            ChangeSprayTemplateSize(size);
        }
        else
        {
            ReapplyActiveTrajectoryHeight();
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

        // 2. Vẽ CHỖ CẦN PHUN (Vùng mục tiêu R màu Vàng Cam & Vùng an toàn R_safe sau co biên e màu Xanh Lá)
        if (showSprayZones3D)
        {
            bool drewCustomTarget = false;
            if (activeTrajectoryMethod == "P1" && lastP1Result != null && lastP1Result.rawTargetRegionPolygonsWorld.Count > 0)
            {
                drewCustomTarget = true;
                Color rawHalo = new Color(0.22f, 0.10f, 0.01f, 0.94f);
                Color rawCore = new Color(1.0f, 0.72f, 0.08f, 1.0f);
                Color rawHatch = new Color(1.0f, 0.72f, 0.08f, 0.24f);
                Color safeHalo = new Color(0.02f, 0.20f, 0.06f, 0.94f);
                Color safeCore = new Color(0.32f, 1.0f, 0.36f, 1.0f);

                for (int r = 0; r < lastP1Result.rawTargetRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP1Result.rawTargetRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    // Gạch chéo mờ bên trong vùng R để phân biệt trực quan với đường quỹ đạo
                    int pc = poly.Count;
                    int stride = Mathf.Max(1, pc / 10);
                    for (int k = 0; k < pc / 2; k += stride)
                    {
                        int opp = (pc - 1 - k) % pc;
                        Vector3 s0 = cam.WorldToScreenPoint(poly[k]);
                        Vector3 s1 = cam.WorldToScreenPoint(poly[opp]);
                        if (s0.z > 0.05f && s1.z > 0.05f)
                            DrawGuiLine2D(new Vector2(s0.x, Screen.height - s0.y), new Vector2(s1.x, Screen.height - s1.y), rawHatch, 1.4f);
                    }
                    DrawWorldLoopInGUI(cam, poly, rawHalo, 5.2f, true);
                    DrawWorldLoopInGUI(cam, poly, rawCore, 3.0f, true);
                }

                for (int r = 0; r < lastP1Result.safeErodedRegionPolygonsWorld.Count; r++)
                {
                    var safePoly = lastP1Result.safeErodedRegionPolygonsWorld[r];
                    if (safePoly == null || safePoly.Count < 3) continue;
                    DrawWorldLoopInGUI(cam, safePoly, safeHalo, 4.0f, true);
                    DrawWorldLoopInGUI(cam, safePoly, safeCore, 2.2f, true);
                }
            }
            else if (activeTrajectoryMethod == "P2" && lastP2Result != null && lastP2Result.targetRegionPolygonsWorld.Count > 0)
            {
                drewCustomTarget = true;
                Color p2Halo = new Color(0.22f, 0.10f, 0.01f, 0.94f);
                Color p2Core = new Color(1.0f, 0.72f, 0.08f, 1.0f);
                Color p2Hatch = new Color(1.0f, 0.72f, 0.08f, 0.24f);

                for (int r = 0; r < lastP2Result.targetRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP2Result.targetRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    int pc = poly.Count;
                    int stride = Mathf.Max(1, pc / 10);
                    for (int k = 0; k < pc / 2; k += stride)
                    {
                        int opp = (pc - 1 - k) % pc;
                        Vector3 s0 = cam.WorldToScreenPoint(poly[k]);
                        Vector3 s1 = cam.WorldToScreenPoint(poly[opp]);
                        if (s0.z > 0.05f && s1.z > 0.05f)
                            DrawGuiLine2D(new Vector2(s0.x, Screen.height - s0.y), new Vector2(s1.x, Screen.height - s1.y), p2Hatch, 1.4f);
                    }
                    DrawWorldLoopInGUI(cam, poly, p2Halo, 5.2f, true);
                    DrawWorldLoopInGUI(cam, poly, p2Core, 3.0f, true);
                }
            }
            else if (activeTrajectoryMethod == "P3" && lastP3Result != null && lastP3Result.targetRegionPolygonsWorld.Count > 0)
            {
                drewCustomTarget = true;
                Color p3Halo = new Color(0.22f, 0.10f, 0.01f, 0.94f);
                Color p3Core = new Color(1.0f, 0.72f, 0.08f, 1.0f);
                for (int r = 0; r < lastP3Result.targetRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP3Result.targetRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    DrawWorldLoopInGUI(cam, poly, p3Halo, 5.0f, true);
                    DrawWorldLoopInGUI(cam, poly, p3Core, 2.8f, true);
                }
            }
            else if (activeTrajectoryMethod == "P4" && lastP4Result != null && lastP4Result.targetRegionPolygonsWorld.Count > 0)
            {
                drewCustomTarget = true;
                Color p4Halo = new Color(0.22f, 0.10f, 0.01f, 0.94f);
                Color p4Core = new Color(1.0f, 0.72f, 0.08f, 1.0f);
                for (int r = 0; r < lastP4Result.targetRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP4Result.targetRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    DrawWorldLoopInGUI(cam, poly, p4Halo, 5.0f, true);
                    DrawWorldLoopInGUI(cam, poly, p4Core, 2.8f, true);
                }
            }

            if (!drewCustomTarget)
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
        }

        // 2b. Vẽ TRỤC GIỮA ỐNG QUẦN (SKELETON P2 - Hồng tím) độc lập với Chỗ cần phun & Đường quỹ đạo
        if (showP2SkeletonAxis && activeTrajectoryMethod == "P2" && lastP2Result != null && lastP2Result.skeletonAxesWorld.Count > 0)
        {
            Color skelHalo = new Color(0.18f, 0.02f, 0.22f, 0.90f);
            Color skelCore = new Color(0.96f, 0.26f, 1.0f, 0.98f);
            for (int a = 0; a < lastP2Result.skeletonAxesWorld.Count; a++)
            {
                var axis = lastP2Result.skeletonAxesWorld[a];
                if (axis == null || axis.Count < 2) continue;
                for (int i = 0; i < axis.Count - 1; i++)
                {
                    Vector3 s0 = cam.WorldToScreenPoint(axis[i]);
                    Vector3 s1 = cam.WorldToScreenPoint(axis[i + 1]);
                    if (s0.z <= 0.05f || s1.z <= 0.05f) continue;
                    Vector2 p0 = new Vector2(s0.x, Screen.height - s0.y);
                    Vector2 p1 = new Vector2(s1.x, Screen.height - s1.y);
                    DrawGuiLine2D(p0, p1, skelHalo, 4.4f);
                    DrawGuiDashedLine2D(p0, p1, skelCore, 2.5f, 8f, 4f);
                }
            }
        }

        // 2c. Vẽ LƯỚI 32 TAM GIÁC BARYCENTRIC (P3 - Xanh ngọc, Đỏ nếu tam giác bị đảo chiều)
        if (showP3TriangulationMesh && activeTrajectoryMethod == "P3" && lastP3Result != null && lastP3Result.meshTrianglesWorld.Count > 0)
        {
            Color triOkCol = new Color(0.20f, 0.95f, 0.65f, 0.72f);
            Color triBadCol = new Color(1.0f, 0.15f, 0.15f, 0.95f);
            for (int t = 0; t < lastP3Result.meshTrianglesWorld.Count; t++)
            {
                Vector3[] tri = lastP3Result.meshTrianglesWorld[t];
                if (tri == null || tri.Length < 3) continue;
                bool isFlipped = (t < lastP3Result.meshTriangleFlipped.Count) && lastP3Result.meshTriangleFlipped[t];
                Color edgeCol = isFlipped ? triBadCol : triOkCol;
                float width = isFlipped ? 2.6f : 1.5f;
                for (int e = 0; e < 3; e++)
                {
                    Vector3 s0 = cam.WorldToScreenPoint(tri[e]);
                    Vector3 s1 = cam.WorldToScreenPoint(tri[(e + 1) % 3]);
                    if (s0.z <= 0.05f || s1.z <= 0.05f) continue;
                    DrawGuiLine2D(new Vector2(s0.x, Screen.height - s0.y), new Vector2(s1.x, Screen.height - s1.y), edgeCol, width);
                }
            }
            if (lastP3Result.detectedLandmarksWorld != null && lastP3Result.detectedLandmarksWorld.Count > 0)
            {
                Color lmDotCol = new Color(0.15f, 1.0f, 0.72f, 0.96f);
                for (int i = 0; i < lastP3Result.detectedLandmarksWorld.Count; i++)
                {
                    Vector3 sp = cam.WorldToScreenPoint(lastP3Result.detectedLandmarksWorld[i]);
                    if (sp.z <= 0.05f) continue;
                    DrawGuiSolidRect(new Rect(sp.x - 4f, Screen.height - sp.y - 4f, 8f, 8f), lmDotCol);
                }
            }
        }

        // 2d. Vẽ ĐÁM MÂY 3D QUẦN MẪU ĐÃ ĐĂNG KÝ LÊN QUẦN MỚI (P4 - Xanh chanh)
        if (showP4RegisteredCloud3D && activeTrajectoryMethod == "P4" && lastP4Result != null && lastP4Result.registeredCloudWorld.Count > 0)
        {
            Color cloudHalo = new Color(0.05f, 0.16f, 0.02f, 0.80f);
            Color cloudCore = new Color(0.55f, 1.0f, 0.18f, 0.90f);
            int total = lastP4Result.registeredCloudWorld.Count;
            int step = Mathf.Max(1, total / 420);
            for (int i = 0; i < total; i += step)
            {
                Vector3 sp = cam.WorldToScreenPoint(lastP4Result.registeredCloudWorld[i]);
                if (sp.z <= 0.05f) continue;
                float gx = sp.x;
                float gy = Screen.height - sp.y;
                DrawGuiSolidRect(new Rect(gx - 2.6f, gy - 2.6f, 5.2f, 5.2f), cloudHalo);
                DrawGuiSolidRect(new Rect(gx - 1.7f, gy - 1.7f, 3.4f, 3.4f), cloudCore);
            }
        }

        // 3. Vẽ ĐƯỜNG QUỸ ĐẠO (Đứt cam = Quét qua lại / TẮT SÚNG) và ĐƯỜNG PHUN (Cyan nét liền = BẬT SÚNG) ĐỘC LẬP NHAU
        if ((showGizmos || showTurnaroundConnectors) && waypoints != null && waypoints.Count > 1)
        {
            Color trajHalo = new Color(0.02f, 0.12f, 0.20f, 0.85f);
            Color trajCol = new Color(0.15f, 0.92f, 1.0f, 0.96f);
            Color offConnectorCol = new Color(1.0f, 0.44f, 0.24f, 0.88f);
            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                bool isSameSeg = string.IsNullOrEmpty(waypoints[i].segmentName) || string.IsNullOrEmpty(waypoints[i + 1].segmentName) || waypoints[i].segmentName == waypoints[i + 1].segmentName;
                bool isSprayOnMove = waypoints[i + 1].sprayOn && isSameSeg && Vector3.Distance(waypoints[i].targetPosGlobal_mm, waypoints[i + 1].targetPosGlobal_mm) <= sprayJumpThresholdMm;

                Vector3 s0 = cam.WorldToScreenPoint(waypoints[i].targetPosWorld_m);
                Vector3 s1 = cam.WorldToScreenPoint(waypoints[i + 1].targetPosWorld_m);
                if (s0.z <= 0.05f || s1.z <= 0.05f) continue;

                Vector2 p0 = new Vector2(s0.x, Screen.height - s0.y);
                Vector2 p1 = new Vector2(s1.x, Screen.height - s1.y);
                if (isSprayOnMove)
                {
                    if (showGizmos)
                    {
                        DrawGuiLine2D(p0, p1, trajHalo, 4.2f);
                        DrawGuiLine2D(p0, p1, trajCol, 2.6f);
                    }
                    else if (showTurnaroundConnectors)
                    {
                        // Khi người dùng tắt Đường phun và chỉ bật Đường quỹ đạo -> vẫn thấy toàn bộ lộ trình quét liên tục
                        DrawGuiDashedLine2D(p0, p1, offConnectorCol, 1.8f, 6f, 5f);
                    }
                }
                else if (showTurnaroundConnectors)
                {
                    DrawGuiDashedLine2D(p0, p1, offConnectorCol, 1.9f, 6f, 5f);
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

    private void DrawGuiDashedLine2D(Vector2 a, Vector2 b, Color col, float width, float dashPx = 7f, float gapPx = 5f)
    {
        Vector2 delta = b - a;
        float len = delta.magnitude;
        if (len < 1f) return;
        Vector2 dir = delta / len;
        float step = Mathf.Max(2f, dashPx + gapPx);
        float d = 0f;
        while (d < len)
        {
            float dEnd = Mathf.Min(len, d + dashPx);
            DrawGuiLine2D(a + dir * d, a + dir * dEnd, col, width);
            d += step;
        }
    }

    private void DrawGuiCircleOutline2D(Vector2 centerPx, float radiusPx, Color col, float width, int segments = 24)
    {
        if (radiusPx < 1f) return;
        int seg = Mathf.Clamp(segments, 12, 64);
        Vector2 prev = new Vector2(centerPx.x + radiusPx, centerPx.y);
        for (int i = 1; i <= seg; i++)
        {
            float ang = (i / (float)seg) * Mathf.PI * 2f;
            Vector2 next = new Vector2(centerPx.x + Mathf.Cos(ang) * radiusPx, centerPx.y + Mathf.Sin(ang) * radiusPx);
            DrawGuiLine2D(prev, next, col, width);
            prev = next;
        }
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

        // 2. Vẽ CHỖ CẦN PHUN (Vùng R màu Vàng Cam & Vùng R_safe màu Xanh Lá) trong Scene View
        if (showSprayZones3D)
        {
            bool drewCustom = false;
            if (activeTrajectoryMethod == "P1" && lastP1Result != null && lastP1Result.rawTargetRegionPolygonsWorld.Count > 0)
            {
                drewCustom = true;
                Gizmos.color = new Color(1.0f, 0.72f, 0.08f, 0.95f);
                for (int r = 0; r < lastP1Result.rawTargetRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP1Result.rawTargetRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    for (int i = 0; i < poly.Count; i++)
                        Gizmos.DrawLine(poly[i], poly[(i + 1) % poly.Count]);
                }
                Gizmos.color = new Color(0.32f, 1.0f, 0.36f, 0.95f);
                for (int r = 0; r < lastP1Result.safeErodedRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP1Result.safeErodedRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    for (int i = 0; i < poly.Count; i++)
                        Gizmos.DrawLine(poly[i], poly[(i + 1) % poly.Count]);
                }
            }
            else if (activeTrajectoryMethod == "P2" && lastP2Result != null && lastP2Result.targetRegionPolygonsWorld.Count > 0)
            {
                drewCustom = true;
                Gizmos.color = new Color(1.0f, 0.72f, 0.08f, 0.95f);
                for (int r = 0; r < lastP2Result.targetRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP2Result.targetRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    for (int i = 0; i < poly.Count; i++)
                        Gizmos.DrawLine(poly[i], poly[(i + 1) % poly.Count]);
                }
            }
            else if (activeTrajectoryMethod == "P3" && lastP3Result != null && lastP3Result.targetRegionPolygonsWorld.Count > 0)
            {
                drewCustom = true;
                Gizmos.color = new Color(1.0f, 0.72f, 0.08f, 0.95f);
                for (int r = 0; r < lastP3Result.targetRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP3Result.targetRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    for (int i = 0; i < poly.Count; i++)
                        Gizmos.DrawLine(poly[i], poly[(i + 1) % poly.Count]);
                }
            }
            else if (activeTrajectoryMethod == "P4" && lastP4Result != null && lastP4Result.targetRegionPolygonsWorld.Count > 0)
            {
                drewCustom = true;
                Gizmos.color = new Color(1.0f, 0.72f, 0.08f, 0.95f);
                for (int r = 0; r < lastP4Result.targetRegionPolygonsWorld.Count; r++)
                {
                    var poly = lastP4Result.targetRegionPolygonsWorld[r];
                    if (poly == null || poly.Count < 3) continue;
                    for (int i = 0; i < poly.Count; i++)
                        Gizmos.DrawLine(poly[i], poly[(i + 1) % poly.Count]);
                }
            }
            if (!drewCustom)
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
        }

        // 2b. Vẽ Trục giữa ống quần (Skeleton P2) trong Scene View
        if (showP2SkeletonAxis && activeTrajectoryMethod == "P2" && lastP2Result != null && lastP2Result.skeletonAxesWorld.Count > 0)
        {
            Gizmos.color = new Color(0.96f, 0.26f, 1.0f, 0.95f);
            for (int a = 0; a < lastP2Result.skeletonAxesWorld.Count; a++)
            {
                var axis = lastP2Result.skeletonAxesWorld[a];
                if (axis == null || axis.Count < 2) continue;
                for (int i = 0; i < axis.Count - 1; i++)
                    Gizmos.DrawLine(axis[i], axis[i + 1]);
            }
        }

        // 2c. Vẽ Lưới 32 tam giác Barycentric (P3) trong Scene View
        if (showP3TriangulationMesh && activeTrajectoryMethod == "P3" && lastP3Result != null && lastP3Result.meshTrianglesWorld.Count > 0)
        {
            for (int t = 0; t < lastP3Result.meshTrianglesWorld.Count; t++)
            {
                Vector3[] tri = lastP3Result.meshTrianglesWorld[t];
                if (tri == null || tri.Length < 3) continue;
                bool isFlipped = (t < lastP3Result.meshTriangleFlipped.Count) && lastP3Result.meshTriangleFlipped[t];
                Gizmos.color = isFlipped ? new Color(1.0f, 0.15f, 0.15f, 0.95f) : new Color(0.20f, 0.95f, 0.65f, 0.78f);
                Gizmos.DrawLine(tri[0], tri[1]);
                Gizmos.DrawLine(tri[1], tri[2]);
                Gizmos.DrawLine(tri[2], tri[0]);
            }
        }

        // 2d. Vẽ Đám mây 3D đã đăng ký (P4) trong Scene View
        if (showP4RegisteredCloud3D && activeTrajectoryMethod == "P4" && lastP4Result != null && lastP4Result.registeredCloudWorld.Count > 0)
        {
            Gizmos.color = new Color(0.55f, 1.0f, 0.18f, 0.88f);
            int total = lastP4Result.registeredCloudWorld.Count;
            int step = Mathf.Max(1, total / 420);
            for (int i = 0; i < total; i += step)
            {
                Gizmos.DrawSphere(lastP4Result.registeredCloudWorld[i], 0.0035f);
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

        if ((!showGizmos && !showTurnaroundConnectors) || waypoints == null || waypoints.Count == 0) return;

        // 4. Vẽ ĐƯỜNG QUỸ ĐẠO (Cam đỏ = Quét / TẮT SÚNG) và ĐƯỜNG PHUN (Cyan = BẬT SÚNG) ĐỘC LẬP NHAU
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            bool isSameSeg = string.IsNullOrEmpty(waypoints[i].segmentName) || string.IsNullOrEmpty(waypoints[i + 1].segmentName) || waypoints[i].segmentName == waypoints[i + 1].segmentName;
            bool isSprayOnMove = waypoints[i + 1].sprayOn && isSameSeg && Vector3.Distance(waypoints[i].targetPosGlobal_mm, waypoints[i + 1].targetPosGlobal_mm) <= sprayJumpThresholdMm;
            if (isSprayOnMove)
            {
                if (showGizmos)
                {
                    Gizmos.color = new Color(0.15f, 0.85f, 1f, 0.92f);
                    Gizmos.DrawLine(waypoints[i].targetPosWorld_m, waypoints[i + 1].targetPosWorld_m);
                }
                else if (showTurnaroundConnectors)
                {
                    Gizmos.color = new Color(1.0f, 0.44f, 0.24f, 0.75f);
                    Gizmos.DrawLine(waypoints[i].targetPosWorld_m, waypoints[i + 1].targetPosWorld_m);
                }
            }
            else if (showTurnaroundConnectors)
            {
                Gizmos.color = new Color(1.0f, 0.44f, 0.24f, 0.75f);
                Gizmos.DrawLine(waypoints[i].targetPosWorld_m, waypoints[i + 1].targetPosWorld_m);
            }
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
        // Định nghĩa mẫu chuẩn thống nhất (2 râu mèo + 3 đường dọc tâm đùi-gối mỗi ống) trên hệ mốc chuẩn Size M,
        // sau đó biến dạng Barycentric sang hệ mốc chuẩn của Size S/M/L để hình dạng mẫu 100% đồng nhất ở mọi size!
        var strokes = new List<List<Vector2>>();
        const int nWhiskers = 2;
        const int nPasses = 3;
        const float vTop = 0.46f, vBot = 0.80f, passSpacing = 0.032f;

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

        if (!string.IsNullOrEmpty(size) && size != "M")
        {
            Vector2[] srcM = GetCanonicalLandmarks12UV("M");
            Vector2[] dstSz = GetCanonicalLandmarks12UV(size);
            WarpStrokesInPlace(strokes, srcM, dstSz);
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
        if (canonicalPatternStrokes != null && canonicalPatternStrokes.Count > 0 &&
            (customDrawnRegions == null || customDrawnRegions.Count == 0))
        {
            var copy = new List<List<Vector2>>(canonicalPatternStrokes.Count);
            for (int s = 0; s < canonicalPatternStrokes.Count; s++)
            {
                if (canonicalPatternStrokes[s] != null && canonicalPatternStrokes[s].Count >= 2)
                    copy.Add(new List<Vector2>(canonicalPatternStrokes[s]));
            }
            if (copy.Count > 0) return copy;
        }

        if ((customDrawnStrokes == null || customDrawnStrokes.Count == 0) &&
            (customDrawnRegions == null || customDrawnRegions.Count == 0))
        {
            return BuildDefaultCanonicalStrokesForSize(activeSprayTemplateSize);
        }

        Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
        bool usingRealCam = (camTex != null);
        Vector2[] camLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, usingRealCam, out bool hasRealLm);
        Vector2[] canonLmUV = GetCanonicalLandmarks12UV(activeSprayTemplateSize);
        float camAspect = GetActiveCameraMetricAspect(plyLoader, nativeW, nativeH);

        var combinedSource = new List<List<Vector2>>();
        if (customDrawnStrokes != null) combinedSource.AddRange(customDrawnStrokes);
        if (customDrawnRegions != null && customDrawnRegions.Count > 0)
        {
            combinedSource.AddRange(BuildTemporarySprayStrokesFromDrawnRegions(sprayWidthMm));
        }

        var canonicalList = new List<List<Vector2>>(combinedSource.Count);
        for (int s = 0; s < combinedSource.Count; s++)
        {
            var srcStroke = combinedSource[s];
            if (srcStroke == null || srcStroke.Count < 2) continue;
            var dstStroke = new List<Vector2>(srcStroke.Count);
            for (int p = 0; p < srcStroke.Count; p++)
            {
                Vector2 pt = srcStroke[p];
                if (usingRealCam && hasRealLm && camLmUV != null)
                {
                    pt = WarpUV2D(pt, camLmUV, canonLmUV, camAspect, 1f);
                }
                dstStroke.Add(pt);
            }
            canonicalList.Add(dstStroke);
        }

        return canonicalList.Count > 0 ? canonicalList : BuildDefaultCanonicalStrokesForSize(activeSprayTemplateSize);
    }

    private static bool FitAffine2D(Vector2[] srcPts, Vector2[] dstPts, out double[] coefU, out double[] coefV)
    {
        coefU = null;
        coefV = null;
        if (srcPts == null || dstPts == null || srcPts.Length < 3 || dstPts.Length < srcPts.Length) return false;

        int n = srcPts.Length;
        double[,] ata = new double[3, 3];
        double[] atu = new double[3];
        double[] atv = new double[3];
        for (int i = 0; i < n; i++)
        {
            double x = srcPts[i].x, y = srcPts[i].y;
            double[] row = { 1.0, x, y };
            for (int r = 0; r < 3; r++)
            {
                atu[r] += row[r] * dstPts[i].x;
                atv[r] += row[r] * dstPts[i].y;
                for (int c = 0; c < 3; c++) ata[r, c] += row[r] * row[c];
            }
        }
        ata[0, 0] += 1e-9; ata[1, 1] += 1e-9; ata[2, 2] += 1e-9;
        return TrajectoryEvaluationEngine.SolveLinearSystem(ata, atu, out coefU)
            && TrajectoryEvaluationEngine.SolveLinearSystem(ata, atv, out coefV);
    }

    private bool p1ConfigInitializedFromTech = false;

    private void SyncP1ConfigFromTechParamsIfNeeded()
    {
        if (p1Config == null) p1Config = new TrajectoryEvaluationEngine.P1ZigzagConfig();
        if (!p1ConfigInitializedFromTech)
        {
            p1ConfigInitializedFromTech = true;
            p1Config.sprayDistanceDMm = customDrawZOffsetMm;
            // Giữ co biên mặc định 6 mm (an toàn cho cả vùng vẽ nhỏ lẫn 2 ống quần) và chồng lấn 50%
            p1Config.erosionEMm = 6f;
            if (sprayTechParams != null)
            {
                p1Config.sprayWidthWMm = Mathf.Clamp(sprayTechParams.sprayWidthMm, 15f, 150f);
                p1Config.overlapRatioR = Mathf.Clamp(Mathf.Max(sprayTechParams.overlapRatio, 0.45f), 0.05f, 0.85f);
                p1Config.scanSpacingSMm = p1Config.EffectiveSpacingSMm;
            }
        }
        else
        {
            p1Config.sprayDistanceDMm = customDrawZOffsetMm;
        }
    }

    /// <summary>
    /// Bảng thông số tùy chỉnh đầy đủ theo tài liệu thuật toán P1 – Quét Zigzag:
    /// Nguồn vùng R (Mẫu vẽ / 2 ống AI / Hình tròn / HCN), khoảng co biên e, bước lượt s (hoặc w & r),
    /// khoảng cách súng d, Douglas-Peucker eps, hướng quét (PCA / Min-Area Rect / Góc tự chọn),
    /// phân tách ô lồi Boustrophedon, và chế độ độ cao Z & hướng súng (2.5D / 2D).
    /// </summary>
    private void DrawP1ParameterConfigSection()
    {
        if (p1Config == null) p1Config = new TrajectoryEvaluationEngine.P1ZigzagConfig();
        bool changed = false;

        GUILayout.Space(4);
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle());
        GUILayout.BeginHorizontal();
        GUILayout.Label("THÔNG SỐ THUẬT TOÁN P1 – QUÉT ZIGZAG", AntigravityUITheme.StyleSectionLabel);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Thu gọn", AntigravityUITheme.StyleButton, GUILayout.Width(64), GUILayout.Height(18)))
        {
            showP1ParameterPanel = false;
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
            return;
        }
        GUILayout.EndHorizontal();

        // 0. Kiểu chạy quỹ đạo P1 (Kiểu máy in 3D quét qua lại toàn khung hay cắt gọn sát R_safe)
        GUILayout.Label("Kiểu quét Zigzag P1:", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isPrinterMode = (p1Config.zigzagStyle == TrajectoryEvaluationEngine.P1ZigzagStyle.PrinterRasterFullSweep);
        if (GUILayout.Button("Máy in 3D (Quét qua lại)", isPrinterMode ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(23)))
        {
            if (!isPrinterMode) { p1Config.zigzagStyle = TrajectoryEvaluationEngine.P1ZigzagStyle.PrinterRasterFullSweep; changed = true; }
        }
        if (GUILayout.Button("Cắt gọn theo R_safe", !isPrinterMode ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(23)))
        {
            if (isPrinterMode) { p1Config.zigzagStyle = TrajectoryEvaluationEngine.P1ZigzagStyle.TrimmedToSafeRegion; changed = true; }
        }
        GUILayout.EndHorizontal();

        if (isPrinterMode)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Lề quay đầu: {p1Config.turnaroundOverscanMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(135));
            float newOv = Mathf.Round(GUILayout.HorizontalSlider(p1Config.turnaroundOverscanMm, 4f, 45f));
            if (Mathf.Abs(newOv - p1Config.turnaroundOverscanMm) > 0.1f) { p1Config.turnaroundOverscanMm = newOv; changed = true; }
            GUILayout.EndHorizontal();
        }

        // 1. Nguồn vùng cần phun R (Mask đầu vào)
        GUILayout.Label("Nguồn chỗ cần phun (Mask R):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        string[] srcLabels = { "Mẫu vẽ", "2 ống AI", "Hình tròn", "Chữ nhật" };
        for (int i = 0; i < srcLabels.Length; i++)
        {
            bool sel = ((int)p1Config.regionSource == i);
            if (GUILayout.Button(srcLabels[i], sel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
            {
                if (!sel)
                {
                    p1Config.regionSource = (TrajectoryEvaluationEngine.P1RegionSourceMode)i;
                    changed = true;
                }
            }
        }
        GUILayout.EndHorizontal();

        if (p1Config.regionSource == TrajectoryEvaluationEngine.P1RegionSourceMode.FromTemplateStrokes)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Bề rộng nét hở: {p1Config.openStrokeBrushRadiusMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(135));
            float newBr = Mathf.Round(GUILayout.HorizontalSlider(p1Config.openStrokeBrushRadiusMm, 10f, 45f));
            if (Mathf.Abs(newBr - p1Config.openStrokeBrushRadiusMm) > 0.1f) { p1Config.openStrokeBrushRadiusMm = newBr; changed = true; }
            GUILayout.EndHorizontal();
        }
        else if (p1Config.regionSource == TrajectoryEvaluationEngine.P1RegionSourceMode.TestCircle)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Bán kính R0: {p1Config.testCircleRadiusMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(135));
            float nr = Mathf.Round(GUILayout.HorizontalSlider(p1Config.testCircleRadiusMm, 15f, 120f));
            if (Mathf.Abs(nr - p1Config.testCircleRadiusMm) > 0.1f) { p1Config.testCircleRadiusMm = nr; changed = true; }
            GUILayout.EndHorizontal();

            bool nb = GUILayout.Toggle(p1Config.testOnBothLegs, " Đặt trên cả 2 ống quần (R_safe = R0 - e)", AntigravityUITheme.StyleToggle);
            if (nb != p1Config.testOnBothLegs) { p1Config.testOnBothLegs = nb; changed = true; }
        }
        else if (p1Config.regionSource == TrajectoryEvaluationEngine.P1RegionSourceMode.TestRectangle)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Rộng W: {p1Config.testRectWidthMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(135));
            float nw = Mathf.Round(GUILayout.HorizontalSlider(p1Config.testRectWidthMm, 30f, 220f));
            if (Mathf.Abs(nw - p1Config.testRectWidthMm) > 0.1f) { p1Config.testRectWidthMm = nw; changed = true; }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Dài L: {p1Config.testRectLengthMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(135));
            float nl = Mathf.Round(GUILayout.HorizontalSlider(p1Config.testRectLengthMm, 50f, 450f));
            if (Mathf.Abs(nl - p1Config.testRectLengthMm) > 0.1f) { p1Config.testRectLengthMm = nl; changed = true; }
            GUILayout.EndHorizontal();

            bool nb = GUILayout.Toggle(p1Config.testOnBothLegs, " Đặt trên cả 2 ống quần", AntigravityUITheme.StyleToggle);
            if (nb != p1Config.testOnBothLegs) { p1Config.testOnBothLegs = nb; changed = true; }
        }

        // 2. Khoảng co biên e (tạo vùng an toàn R_safe bên trong R)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Co biên e: {p1Config.erosionEMm:F1} mm", AntigravityUITheme.StyleLabelGreen, GUILayout.Width(135));
        float newE = Mathf.Round(GUILayout.HorizontalSlider(p1Config.erosionEMm, 0f, 45f) * 2f) * 0.5f;
        if (Mathf.Abs(newE - p1Config.erosionEMm) > 0.05f)
        {
            p1Config.erosionEMm = newE;
            if (sprayTechParams != null) sprayTechParams.borderMarginMm = newE;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 3. Bề rộng vệt phun w & Bước lượt s
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Bề rộng vệt w: {p1Config.sprayWidthWMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(135));
        float newW = Mathf.Round(GUILayout.HorizontalSlider(p1Config.sprayWidthWMm, 15f, 150f));
        if (Mathf.Abs(newW - p1Config.sprayWidthWMm) > 0.1f)
        {
            p1Config.sprayWidthWMm = newW;
            if (sprayTechParams != null) sprayTechParams.sprayWidthMm = newW;
            changed = true;
        }
        GUILayout.EndHorizontal();

        bool newAutoS = GUILayout.Toggle(p1Config.autoSpacingFromWidthOverlap,
            $" Tính bước s = w*(1-r)  [s = {p1Config.EffectiveSpacingSMm:F1} mm]", AntigravityUITheme.StyleToggle);
        if (newAutoS != p1Config.autoSpacingFromWidthOverlap)
        {
            p1Config.autoSpacingFromWidthOverlap = newAutoS;
            if (!newAutoS) p1Config.scanSpacingSMm = p1Config.EffectiveSpacingSMm;
            changed = true;
        }

        if (p1Config.autoSpacingFromWidthOverlap)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Chồng lấn r: {p1Config.overlapRatioR * 100f:F0}%", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(135));
            float newR = Mathf.Round(GUILayout.HorizontalSlider(p1Config.overlapRatioR, 0.05f, 0.80f) * 20f) / 20f;
            if (Mathf.Abs(newR - p1Config.overlapRatioR) > 0.005f)
            {
                p1Config.overlapRatioR = newR;
                if (sprayTechParams != null) sprayTechParams.overlapRatio = newR;
                changed = true;
            }
            GUILayout.EndHorizontal();
        }
        else
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Bước lượt s: {p1Config.scanSpacingSMm:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(135));
            float newS = Mathf.Round(GUILayout.HorizontalSlider(p1Config.scanSpacingSMm, 4f, 80f) * 2f) * 0.5f;
            if (Mathf.Abs(newS - p1Config.scanSpacingSMm) > 0.05f)
            {
                p1Config.scanSpacingSMm = newS;
                changed = true;
            }
            GUILayout.EndHorizontal();
        }

        // 4. Khoảng cách súng d
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Khoảng cách d: {p1Config.sprayDistanceDMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(135));
        float newD = Mathf.Round(GUILayout.HorizontalSlider(p1Config.sprayDistanceDMm, 0f, 250f));
        if (Mathf.Abs(newD - p1Config.sprayDistanceDMm) > 0.1f)
        {
            p1Config.sprayDistanceDMm = newD;
            customDrawZOffsetMm = newD;
            if (sprayTechParams != null) sprayTechParams.standoffMm = newD;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 5. Ngưỡng Douglas-Peucker (đơn giản hóa biên)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Douglas-Peucker: {p1Config.douglasPeuckerEpsMm:F1} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(135));
        float newEps = Mathf.Round(GUILayout.HorizontalSlider(p1Config.douglasPeuckerEpsMm, 1f, 15f) * 2f) * 0.5f;
        if (Mathf.Abs(newEps - p1Config.douglasPeuckerEpsMm) > 0.05f)
        {
            p1Config.douglasPeuckerEpsMm = newEps;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 6. Hướng quét (Bước 4: PCA / HCN bao nhỏ nhất / Góc tùy chỉnh)
        GUILayout.Label("Hướng quét (Bước 4):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        string[] dirLabels = { "PCA (Dọc ống)", "HCN nhỏ nhất", "Góc tự chọn" };
        for (int i = 0; i < dirLabels.Length; i++)
        {
            bool sel = ((int)p1Config.scanDirection == i);
            if (GUILayout.Button(dirLabels[i], sel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(21)))
            {
                if (!sel) { p1Config.scanDirection = (TrajectoryEvaluationEngine.P1ScanDirectionMode)i; changed = true; }
            }
        }
        GUILayout.EndHorizontal();

        if (p1Config.scanDirection == TrajectoryEvaluationEngine.P1ScanDirectionMode.CustomAngleDeg)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Góc quét: {p1Config.customScanAngleDeg:F0}°", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(135));
            float newAng = Mathf.Round(GUILayout.HorizontalSlider(p1Config.customScanAngleDeg, 0f, 180f));
            if (Mathf.Abs(newAng - p1Config.customScanAngleDeg) > 0.1f) { p1Config.customScanAngleDeg = newAng; changed = true; }
            GUILayout.EndHorizontal();
        }

        // 7. Độ cao Z & Hướng súng (Bước 8)
        GUILayout.Label("Độ cao Z & Hướng súng (Bước 8):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        string[] zLabels = { "2.5D Đứng", "2.5D Pháp tuyến", "2D Phẳng" };
        for (int i = 0; i < zLabels.Length; i++)
        {
            bool sel = ((int)p1Config.zHeightMode == i);
            if (GUILayout.Button(zLabels[i], sel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(21)))
            {
                if (!sel) { p1Config.zHeightMode = (TrajectoryEvaluationEngine.P1ZHeightMode)i; changed = true; }
            }
        }
        GUILayout.EndHorizontal();

        // 8. Phân tách ô lồi Boustrophedon (Bước 5), 3 lớp hiển thị độc lập & Tự sinh lại
        bool newBoust = GUILayout.Toggle(p1Config.useBoustrophedonSplit, " Phân tách vùng lõm thành ô lồi (Boustrophedon)", AntigravityUITheme.StyleToggle);
        if (newBoust != p1Config.useBoustrophedonSplit) { p1Config.useBoustrophedonSplit = newBoust; changed = true; }

        GUILayout.Label("Tách lớp hiển thị (bật/tắt độc lập):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(showSprayZones3D ? "Chỗ phun: BẬT" : "Chỗ phun: TẮT",
            showSprayZones3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showSprayZones3D = !showSprayZones3D;
        }
        if (GUILayout.Button(showTurnaroundConnectors ? "Quỹ đạo: BẬT" : "Quỹ đạo: TẮT",
            showTurnaroundConnectors ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showTurnaroundConnectors = !showTurnaroundConnectors;
        }
        if (GUILayout.Button(showGizmos ? "Đường phun: BẬT" : "Đường phun: TẮT",
            showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showGizmos = !showGizmos;
            MarkSettingsDirty();
        }
        GUILayout.EndHorizontal();

        p1AutoRegenerateOnChange = GUILayout.Toggle(p1AutoRegenerateOnChange, " Tự sinh lại đường quỹ đạo khi chỉnh thông số", AntigravityUITheme.StyleToggle);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Sinh quỹ đạo P1", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
        {
            GenerateAndApplyMethodTrajectory("P1");
            changed = false;
        }
        if (GUILayout.Button("Mặc định P1", AntigravityUITheme.StyleButton, GUILayout.Width(88), GUILayout.Height(26)))
        {
            p1Config = new TrajectoryEvaluationEngine.P1ZigzagConfig();
            p1Config.sprayDistanceDMm = customDrawZOffsetMm;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 9. Hiển thị trực tiếp kết quả tách bạch Chỗ cần phun (R -> R_safe) và Quỹ đạo (ON / OFF)
        if (lastP1Result != null)
        {
            GUILayout.Space(3);
            if (!string.IsNullOrEmpty(lastP1Result.warningMessage))
            {
                GUILayout.Label(lastP1Result.warningMessage, AntigravityUITheme.StyleLabelRed);
            }
            GUILayout.Label(
                $"Vùng R: {lastP1Result.areaRawCm2:F1} cm² -> R_safe (e={lastP1Result.erosionEMm:F1}mm): {lastP1Result.areaSafeCm2:F1} cm²",
                AntigravityUITheme.StyleLabelGreen);
            GUILayout.Label(
                $"Ô lồi: {lastP1Result.cellCount} | Lượt: {lastP1Result.passCount} (s={lastP1Result.effectiveSpacingMm:F1}mm) | Bật súng: {lastP1Result.sprayOnSegmentCount} | Quay đầu tắt: {lastP1Result.sprayOffConnectorCount}",
                AntigravityUITheme.StyleLabelCyan);
        }

        GUILayout.EndVertical();

        if (changed && p1AutoRegenerateOnChange)
        {
            GenerateAndApplyMethodTrajectory("P1");
        }
    }

    private bool p2ConfigInitializedFromTech = false;

    private void SyncP2ConfigFromTechParamsIfNeeded()
    {
        if (p2Config == null) p2Config = new TrajectoryEvaluationEngine.P2SkeletonConfig();
        if (!p2ConfigInitializedFromTech)
        {
            p2ConfigInitializedFromTech = true;
            p2Config.sprayDistanceDMm = customDrawZOffsetMm;
            if (sprayTechParams != null)
            {
                p2Config.borderMarginEMm = Mathf.Clamp(sprayTechParams.borderMarginMm, 4f, 35f);
                p2Config.sprayWidthWMm = Mathf.Clamp(sprayTechParams.sprayWidthMm, 20f, 120f);
                p2Config.overlapRatioR = Mathf.Clamp(sprayTechParams.overlapRatio, 0.05f, 0.80f);
                p2Config.passSpacingSMm = p2Config.EffectiveSpacingSMm;
            }
        }
        else
        {
            p2Config.sprayDistanceDMm = customDrawZOffsetMm;
        }
    }

    /// <summary>
    /// Bảng thông số tùy chỉnh đầy đủ theo tài liệu thuật toán P2 – Đường phun theo hình dạng ống quần:
    /// Cách A (Song song trục giữa Skeleton) / Cách B (Offset đồng tâm từ biên),
    /// phạm vi vùng phun (vùng mẫu vẽ / toàn mask 2 ống tách cạp & đáy),
    /// bước khoảng cách s = w*(1-r), biên an toàn e, lọc đoạn ngắn khi ống thu hẹp,
    /// thứ tự sắp xếp nối lượt (tối ưu nối gần / trục ra 2 bên / biên vào trong),
    /// khoảng cách súng d & hướng súng (2.5D pháp tuyến mượt / 2.5D thẳng đứng / 2D phẳng),
    /// và công tắc bật/tắt độc lập Chỗ cần phun vs Đường quỹ đạo vs Trục giữa Skeleton.
    /// </summary>
    private void DrawP2ParameterConfigSection()
    {
        if (p2Config == null) p2Config = new TrajectoryEvaluationEngine.P2SkeletonConfig();
        bool changed = false;

        GUILayout.Space(4);
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle());
        GUILayout.BeginHorizontal();
        GUILayout.Label("THÔNG SỐ THUẬT TOÁN P2 – THEO HÌNH DẠNG ỐNG QUẦN", AntigravityUITheme.StyleSectionLabel);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Thu gọn", AntigravityUITheme.StyleButton, GUILayout.Width(64), GUILayout.Height(18)))
        {
            showP2ParameterPanel = false;
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
            return;
        }
        GUILayout.EndHorizontal();

        // 0. Chế độ quét quỹ đạo P2 (Máy in 3D quét qua lại + bật/tắt súng vs Cắt gọn)
        GUILayout.Label("Chế độ quét quỹ đạo P2:", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isPrinter3D = p2Config.continuousScanLike3DPrinter;
        if (GUILayout.Button("Máy in 3D (Quét & Bật/Tắt súng)", isPrinter3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(23)))
        {
            if (!isPrinter3D) { p2Config.continuousScanLike3DPrinter = true; changed = true; }
        }
        if (GUILayout.Button("Cắt gọn trong vùng phun", !isPrinter3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(23)))
        {
            if (isPrinter3D) { p2Config.continuousScanLike3DPrinter = false; changed = true; }
        }
        GUILayout.EndHorizontal();

        // 1. Biến thể hình học P2 (Cách A vs Cách B)
        GUILayout.Label("Phương pháp hình học P2 (Bước 3 & 4):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isSkel = (p2Config.geometryMode == TrajectoryEvaluationEngine.P2GeometryMode.SkeletonParallel);
        if (GUILayout.Button("Cách A: Trục giữa (Skeleton)", isSkel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(23)))
        {
            if (!isSkel) { p2Config.geometryMode = TrajectoryEvaluationEngine.P2GeometryMode.SkeletonParallel; changed = true; }
        }
        if (GUILayout.Button("Cách B: Offset đồng tâm biên", !isSkel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(23)))
        {
            if (isSkel) { p2Config.geometryMode = TrajectoryEvaluationEngine.P2GeometryMode.BoundaryConcentricOffset; changed = true; }
        }
        GUILayout.EndHorizontal();

        // 2. Phạm vi chỗ cần phun (Bước 1: tách riêng 2 ống quần)
        GUILayout.Label("Phạm vi chỗ cần phun (Tách riêng 2 ống):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isTemplateScope = (p2Config.regionScope == TrajectoryEvaluationEngine.P2RegionScopeMode.FromTemplateRegion);
        if (GUILayout.Button("Theo hình/mẫu đã vẽ", isTemplateScope ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isTemplateScope) { p2Config.regionScope = TrajectoryEvaluationEngine.P2RegionScopeMode.FromTemplateRegion; changed = true; }
        }
        if (GUILayout.Button("Toàn mask 2 ống (tách cạp/đáy)", !isTemplateScope ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (isTemplateScope) { p2Config.regionScope = TrajectoryEvaluationEngine.P2RegionScopeMode.FullLegMask; changed = true; }
        }
        GUILayout.EndHorizontal();

        if (isTemplateScope)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Độ rộng nét vẽ hở: {p2Config.openStrokeSprayWidthMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(140));
            float newBand = Mathf.Round(GUILayout.HorizontalSlider(p2Config.openStrokeSprayWidthMm, 12f, 60f));
            if (Mathf.Abs(newBand - p2Config.openStrokeSprayWidthMm) > 0.1f) { p2Config.openStrokeSprayWidthMm = newBand; changed = true; }
            GUILayout.EndHorizontal();
        }

        // 3. Bề rộng vệt phun w, tỷ lệ chồng lấn r & bước lượt s = w * (1 - r)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Bề rộng vệt w: {p2Config.sprayWidthWMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(140));
        float newW = Mathf.Round(GUILayout.HorizontalSlider(p2Config.sprayWidthWMm, 20f, 120f));
        if (Mathf.Abs(newW - p2Config.sprayWidthWMm) > 0.1f)
        {
            p2Config.sprayWidthWMm = newW;
            if (sprayTechParams != null) sprayTechParams.sprayWidthMm = newW;
            changed = true;
        }
        GUILayout.EndHorizontal();

        bool newAutoS = GUILayout.Toggle(p2Config.autoSpacingFromWidthOverlap,
            $" Tính bước s = w*(1-r)  [s = {p2Config.EffectiveSpacingSMm:F1} mm]", AntigravityUITheme.StyleToggle);
        if (newAutoS != p2Config.autoSpacingFromWidthOverlap)
        {
            p2Config.autoSpacingFromWidthOverlap = newAutoS;
            if (!newAutoS) p2Config.passSpacingSMm = p2Config.EffectiveSpacingSMm;
            changed = true;
        }

        if (p2Config.autoSpacingFromWidthOverlap)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Chồng lấn r: {p2Config.overlapRatioR * 100f:F0}%", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(140));
            float newR = Mathf.Round(GUILayout.HorizontalSlider(p2Config.overlapRatioR, 0.05f, 0.75f) * 20f) / 20f;
            if (Mathf.Abs(newR - p2Config.overlapRatioR) > 0.005f)
            {
                p2Config.overlapRatioR = newR;
                if (sprayTechParams != null) sprayTechParams.overlapRatio = newR;
                changed = true;
            }
            GUILayout.EndHorizontal();
        }
        else
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Bước lượt s: {p2Config.passSpacingSMm:F1} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(140));
            float newS = Mathf.Round(GUILayout.HorizontalSlider(p2Config.passSpacingSMm, 6f, 90f) * 2f) * 0.5f;
            if (Mathf.Abs(newS - p2Config.passSpacingSMm) > 0.05f)
            {
                p2Config.passSpacingSMm = newS;
                changed = true;
            }
            GUILayout.EndHorizontal();
        }

        // 4. Biên an toàn vùng phun e & Ngưỡng cắt ngắn khi ống quần thu hẹp (Bước 5)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Biên an toàn e: {p2Config.borderMarginEMm:F1} mm", AntigravityUITheme.StyleLabelGreen, GUILayout.Width(140));
        float newE = Mathf.Round(GUILayout.HorizontalSlider(p2Config.borderMarginEMm, 4f, 35f) * 2f) * 0.5f;
        if (Mathf.Abs(newE - p2Config.borderMarginEMm) > 0.05f)
        {
            p2Config.borderMarginEMm = newE;
            if (sprayTechParams != null) sprayTechParams.borderMarginMm = newE;
            changed = true;
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Đoạn ngắn tối thiểu: {p2Config.minPassLengthMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(140));
        float newMinLen = Mathf.Round(GUILayout.HorizontalSlider(p2Config.minPassLengthMm, 15f, 120f));
        if (Mathf.Abs(newMinLen - p2Config.minPassLengthMm) > 0.1f)
        {
            p2Config.minPassLengthMm = newMinLen;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 5. Thứ tự sắp xếp và nối các lượt (Bước 6)
        GUILayout.Label("Sắp xếp & nối các lượt (Bước 6):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        string[] ordLabels = { "Tối ưu nối gần", "Trục ra 2 bên", "Biên vào trong" };
        for (int i = 0; i < ordLabels.Length; i++)
        {
            bool sel = ((int)p2Config.passOrdering == i);
            if (GUILayout.Button(ordLabels[i], sel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(21)))
            {
                if (!sel) { p2Config.passOrdering = (TrajectoryEvaluationEngine.P2PassOrderingMode)i; changed = true; }
            }
        }
        GUILayout.EndHorizontal();

        // 6. Khoảng cách súng d & Quy tắc hướng súng (Bước 7)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Khoảng cách d: {p2Config.sprayDistanceDMm:F0} mm", AntigravityUITheme.StyleLabelCyan, GUILayout.Width(140));
        float newD = Mathf.Round(GUILayout.HorizontalSlider(p2Config.sprayDistanceDMm, 0f, 250f));
        if (Mathf.Abs(newD - p2Config.sprayDistanceDMm) > 0.1f)
        {
            p2Config.sprayDistanceDMm = newD;
            customDrawZOffsetMm = newD;
            if (sprayTechParams != null) sprayTechParams.standoffMm = newD;
            changed = true;
        }
        GUILayout.EndHorizontal();

        GUILayout.Label("Độ cao Z & Hướng súng (Bước 7):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        string[] oriLabels = { "2.5D Pháp tuyến", "2.5D Đứng", "2D Phẳng" };
        for (int i = 0; i < oriLabels.Length; i++)
        {
            bool sel = ((int)p2Config.gunOrientation == i);
            if (GUILayout.Button(oriLabels[i], sel ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(21)))
            {
                if (!sel) { p2Config.gunOrientation = (TrajectoryEvaluationEngine.P2GunOrientationMode)i; changed = true; }
            }
        }
        GUILayout.EndHorizontal();

        // 7. Công tắc phân biệt & bật/tắt độc lập trong View: Chỗ cần phun vs Đường quỹ đạo (Tắt súng) vs Đường phun (Bật súng)
        GUILayout.Space(3);
        GUILayout.Label("Phân biệt hiển thị (Chỗ cần phun / Đường quỹ đạo / Đường phun):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(showSprayZones3D ? "Chỗ phun: BẬT" : "Chỗ phun: TẮT",
            showSprayZones3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showSprayZones3D = !showSprayZones3D;
            MarkSettingsDirty();
        }
        if (GUILayout.Button(showTurnaroundConnectors ? "Đường quỹ đạo: BẬT" : "Đường quỹ đạo: TẮT",
            showTurnaroundConnectors ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showTurnaroundConnectors = !showTurnaroundConnectors;
        }
        if (GUILayout.Button(showGizmos ? "Đường phun: BẬT" : "Đường phun: TẮT",
            showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showGizmos = !showGizmos;
            MarkSettingsDirty();
        }
        GUILayout.EndHorizontal();
        showP2SkeletonAxis = GUILayout.Toggle(showP2SkeletonAxis, " Hiện trục giữa ống quần Skeleton (Hồng tím)", AntigravityUITheme.StyleToggle);

        // 8. Tự cập nhật & Nút sinh lại quỹ đạo P2
        p2AutoRegenerateOnChange = GUILayout.Toggle(p2AutoRegenerateOnChange, " Tự sinh lại đường quỹ đạo khi chỉnh thông số", AntigravityUITheme.StyleToggle);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Sinh quỹ đạo P2", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
        {
            GenerateAndApplyMethodTrajectory("P2");
            changed = false;
        }
        if (GUILayout.Button("Mặc định P2", AntigravityUITheme.StyleButton, GUILayout.Width(88), GUILayout.Height(26)))
        {
            p2Config = new TrajectoryEvaluationEngine.P2SkeletonConfig();
            p2Config.sprayDistanceDMm = customDrawZOffsetMm;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 9. Hiển thị trực tiếp chỉ số kiểm tra P2 (Mục 8 & 9)
        if (lastP2Result != null)
        {
            GUILayout.Space(3);
            if (!string.IsNullOrEmpty(lastP2Result.summaryText))
            {
                GUILayout.Label(lastP2Result.summaryText, AntigravityUITheme.StyleLabelGreen);
            }
            GUILayout.Label(
                $"Ống trái: {lastP2Result.leftLegPassCount} lượt | Ống phải: {lastP2Result.rightLegPassCount} lượt | Cắt ngắn do thu hẹp: {lastP2Result.truncatedOuterPassCount}",
                AntigravityUITheme.StyleLabelCyan);
            GUILayout.Label(
                $"Phun ON: {lastP2Result.totalSprayLengthMm:F0} mm | Không phun OFF: {lastP2Result.totalTravelLengthMm:F0} mm | Quay đầu: {lastP2Result.turnaroundCount} lần",
                AntigravityUITheme.StyleLabelMuted);
        }

        GUILayout.EndVertical();

        if (changed && p2AutoRegenerateOnChange)
        {
            GenerateAndApplyMethodTrajectory("P2");
        }
    }

    private void SyncP3ConfigFromTechParamsIfNeeded()
    {
        if (p3Config == null)
        {
            p3Config = new TrajectoryEvaluationEngine.P3LandmarkConfig();
        }
        p3Config.sprayDistanceDMm = Mathf.Clamp(customDrawZOffsetMm, 0f, 250f);
        if (sprayTechParams != null)
        {
            p3Config.borderMarginEMm = Mathf.Clamp(sprayTechParams.borderMarginMm, 0f, 35f);
        }
    }

    private void DrawP3ParameterConfigSection()
    {
        if (p3Config == null) SyncP3ConfigFromTechParamsIfNeeded();
        bool changed = false;

        GUILayout.Space(4);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        GUILayout.BeginHorizontal();
        GUILayout.Label("CẤU HÌNH THÔNG SỐ P3 (QUỸ ĐẠO MẪU THEO MỐC)", AntigravityUITheme.StyleLabelCyan);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Ẩn", AntigravityUITheme.StyleButton, GUILayout.Width(36), GUILayout.Height(18)))
        {
            showP3ParameterPanel = false;
        }
        GUILayout.EndHorizontal();

        // 1. Nguồn phát hiện mốc hình học (Mục 5.2 tài liệu P3)
        GUILayout.Label("Nguồn phát hiện mốc hình học:", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isAiLm = (p3Config.landmarkSource == TrajectoryEvaluationEngine.P3LandmarkSourceMode.AiKeypoint12);
        if (GUILayout.Button("AI Keypoint (12 mốc)", isAiLm ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isAiLm) { p3Config.landmarkSource = TrajectoryEvaluationEngine.P3LandmarkSourceMode.AiKeypoint12; changed = true; }
        }
        if (GUILayout.Button("Hình học Contour", !isAiLm ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (isAiLm) { p3Config.landmarkSource = TrajectoryEvaluationEngine.P3LandmarkSourceMode.ContourGeometry; changed = true; }
        }
        GUILayout.EndHorizontal();

        // 2. Độ cao Z & Hướng súng (2.5D Pháp tuyến / 2.5D Đứng / 2D Phẳng)
        GUILayout.Label("Chiếu 2D->3D & Hướng súng:", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isP3Norm = (p3Config.gunOrientation == TrajectoryEvaluationEngine.P3GunOrientationMode.SmoothSurfaceNormal25D);
        bool isP3Vert = (p3Config.gunOrientation == TrajectoryEvaluationEngine.P3GunOrientationMode.Vertical25D);
        bool isP3Flat = (p3Config.gunOrientation == TrajectoryEvaluationEngine.P3GunOrientationMode.FlatPlane2D);
        if (GUILayout.Button("2.5D Pháp tuyến", isP3Norm ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isP3Norm) { p3Config.gunOrientation = TrajectoryEvaluationEngine.P3GunOrientationMode.SmoothSurfaceNormal25D; changed = true; }
        }
        if (GUILayout.Button("2.5D Đứng", isP3Vert ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isP3Vert) { p3Config.gunOrientation = TrajectoryEvaluationEngine.P3GunOrientationMode.Vertical25D; changed = true; }
        }
        if (GUILayout.Button("2D Phẳng", isP3Flat ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isP3Flat) { p3Config.gunOrientation = TrajectoryEvaluationEngine.P3GunOrientationMode.FlatPlane2D; changed = true; }
        }
        GUILayout.EndHorizontal();

        // 3. Khoảng cách súng d (mm)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Khoảng cách súng d: {p3Config.sprayDistanceDMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
        float newD3 = Mathf.Round(GUILayout.HorizontalSlider(p3Config.sprayDistanceDMm, 0f, 250f));
        if (Mathf.Abs(newD3 - p3Config.sprayDistanceDMm) > 0.1f)
        {
            p3Config.sprayDistanceDMm = newD3;
            customDrawZOffsetMm = newD3;
            if (sprayTechParams != null) sprayTechParams.standoffMm = newD3;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 4. Biên an toàn cắt tràn mép e (mm)
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Biên an toàn mép e: {p3Config.borderMarginEMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
        float newE3 = Mathf.Round(GUILayout.HorizontalSlider(p3Config.borderMarginEMm, 0f, 35f));
        if (Mathf.Abs(newE3 - p3Config.borderMarginEMm) > 0.1f)
        {
            p3Config.borderMarginEMm = newE3;
            if (sprayTechParams != null) sprayTechParams.borderMarginMm = newE3;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 5. Độ mịn lấy mẫu dọc quỹ đạo mẫu
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Độ mịn lấy mẫu UV: {p3Config.resampleStepUV:F3}", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
        float newStep3 = Mathf.Round(GUILayout.HorizontalSlider(p3Config.resampleStepUV, 0.006f, 0.035f) * 1000f) / 1000f;
        if (Mathf.Abs(newStep3 - p3Config.resampleStepUV) > 0.0005f)
        {
            p3Config.resampleStepUV = newStep3;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 6. Tùy chọn vùng kín & cắt biên quần
        bool newFill3 = GUILayout.Toggle(p3Config.fillClosedDrawnRegions, " Phủ lượt phun trong hình kín (Tròn/Vuông/Đa giác)", AntigravityUITheme.StyleToggle);
        if (newFill3 != p3Config.fillClosedDrawnRegions) { p3Config.fillClosedDrawnRegions = newFill3; changed = true; }

        bool newClip3 = GUILayout.Toggle(p3Config.clipToGarmentContour, " Cắt điểm tràn ngoài đường bao quần (chống phun bàn)", AntigravityUITheme.StyleToggle);
        if (newClip3 != p3Config.clipToGarmentContour) { p3Config.clipToGarmentContour = newClip3; changed = true; }

        // 7. 3 lớp hiển thị độc lập + Lưới 32 tam giác Barycentric
        GUILayout.Label("Tách lớp hiển thị (bật/tắt độc lập):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(showSprayZones3D ? "Chỗ phun: BẬT" : "Chỗ phun: TẮT",
            showSprayZones3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showSprayZones3D = !showSprayZones3D;
        }
        if (GUILayout.Button(showTurnaroundConnectors ? "Quỹ đạo: BẬT" : "Quỹ đạo: TẮT",
            showTurnaroundConnectors ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showTurnaroundConnectors = !showTurnaroundConnectors;
        }
        if (GUILayout.Button(showGizmos ? "Đường phun: BẬT" : "Đường phun: TẮT",
            showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showGizmos = !showGizmos;
            MarkSettingsDirty();
        }
        GUILayout.EndHorizontal();
        showP3TriangulationMesh = GUILayout.Toggle(showP3TriangulationMesh, " Hiện lưới 32 tam giác Barycentric (Xanh ngọc / Đỏ nếu đảo)", AntigravityUITheme.StyleToggle);

        // 8. Tự cập nhật & Nút sinh lại quỹ đạo P3
        p3AutoRegenerateOnChange = GUILayout.Toggle(p3AutoRegenerateOnChange, " Tự sinh lại đường quỹ đạo khi chỉnh thông số", AntigravityUITheme.StyleToggle);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Sinh quỹ đạo P3", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
        {
            GenerateAndApplyMethodTrajectory("P3");
            changed = false;
        }
        if (GUILayout.Button("Mặc định P3", AntigravityUITheme.StyleButton, GUILayout.Width(88), GUILayout.Height(26)))
        {
            p3Config = new TrajectoryEvaluationEngine.P3LandmarkConfig();
            p3Config.sprayDistanceDMm = customDrawZOffsetMm;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 9. Hiển thị trực tiếp chỉ số kiểm tra P3 (Mục 9 tài liệu P3)
        if (lastP3Result != null)
        {
            GUILayout.Space(3);
            if (!string.IsNullOrEmpty(lastP3Result.summaryText))
            {
                GUILayout.Label(lastP3Result.summaryText, lastP3Result.flippedTriangleCount == 0
                    ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);
            }
            GUILayout.Label(
                $"Mốc: {lastP3Result.landmarkCount} ({lastP3Result.landmarkMethodUsed}) | Lưới: {lastP3Result.totalTriangles} tam giác (đảo: {lastP3Result.flippedTriangleCount}) | Biến dạng max: {lastP3Result.maxTriangleDistortion:F2}x",
                lastP3Result.flippedTriangleCount == 0 ? AntigravityUITheme.StyleLabelCyan : AntigravityUITheme.StyleLabelYellow);
            GUILayout.Label(
                $"Trong vùng quần: {lastP3Result.inTargetRatioPct:F1}% | Phun ON: {lastP3Result.totalSprayLengthMm:F0} mm | Di chuyển OFF: {lastP3Result.totalTravelLengthMm:F0} mm",
                AntigravityUITheme.StyleLabelMuted);
            if (!string.IsNullOrEmpty(lastP3Result.warningMessage))
            {
                GUILayout.Label(lastP3Result.warningMessage, AntigravityUITheme.StyleLabelYellow);
            }
        }

        GUILayout.EndVertical();

        if (changed && p3AutoRegenerateOnChange)
        {
            GenerateAndApplyMethodTrajectory("P3");
        }
    }

    private void SyncP4ConfigFromTechParamsIfNeeded()
    {
        if (p4Config == null)
        {
            p4Config = new TrajectoryEvaluationEngine.P4RegistrationConfig();
        }
        p4Config.sprayDistanceDMm = Mathf.Clamp(customDrawZOffsetMm, 0f, 250f);
        if (sprayTechParams != null)
        {
            p4Config.residualThresholdMm = Mathf.Clamp(sprayTechParams.registrationResidualThresholdMm, 3f, 35f);
        }
    }

    private void DrawP4ParameterConfigSection()
    {
        if (p4Config == null) SyncP4ConfigFromTechParamsIfNeeded();
        bool changed = false;

        GUILayout.Space(4);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        GUILayout.BeginHorizontal();
        GUILayout.Label("CẤU HÌNH THÔNG SỐ P4 (ĐĂNG KÝ ĐÁM MÂY ĐIỂM 3D)", AntigravityUITheme.StyleLabelCyan);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("Ẩn", AntigravityUITheme.StyleButton, GUILayout.Width(36), GUILayout.Height(18)))
        {
            showP4ParameterPanel = false;
        }
        GUILayout.EndHorizontal();

        // 0. Trạng thái Mẫu chuẩn 3D (Bước 1 tài liệu P4)
        bool hasRef3D = (templateReferenceCloud3DWorld != null && templateReferenceCloud3DWorld.Count >= 30);
        string refInfo = hasRef3D
            ? $"Mẫu 3D chuẩn: [{templateReferenceSampleName}] ({templateReferenceCloud3DWorld.Count} điểm, {templateReferenceStrokes3DWorld.Count} nét 3D)"
            : "Mẫu 3D chuẩn: Chưa chốt (tự tạo từ mốc/phôi hiện tại)";
        GUILayout.Label(refInfo, hasRef3D ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);
        if (GUILayout.Button("Chốt phôi & quỹ đạo hiện tại làm Mẫu Chuẩn 3D (P4)", AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            CaptureReferenceTemplateCloud3DForP4();
            comparisonStatusMsg = (templateReferenceCloud3DWorld != null && templateReferenceCloud3DWorld.Count >= 30)
                ? $"Đã chốt Mẫu 3D chuẩn [{templateReferenceSampleName}] ({templateReferenceCloud3DWorld.Count} điểm, voxel {p4Config.voxelSizeMm:F1}mm) cho P4!"
                : "Cần có quỹ đạo 3D (vẽ hoặc áp dụng 3D trước) để chốt Mẫu chuẩn 3D P4.";
        }

        // 1. Khởi tạo thô (Bước 3 tài liệu P4)
        GUILayout.Label("Bước 3 - Khởi tạo thô (Coarse Init):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isAutoInit = (p4Config.initMode == TrajectoryEvaluationEngine.P4InitMode.AutoLandmarksOrPca);
        bool isLmInit = (p4Config.initMode == TrajectoryEvaluationEngine.P4InitMode.LandmarksP3Only);
        bool isPcaInit = (p4Config.initMode == TrajectoryEvaluationEngine.P4InitMode.Pca3DOnly);
        if (GUILayout.Button("Tự động", isAutoInit ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isAutoInit) { p4Config.initMode = TrajectoryEvaluationEngine.P4InitMode.AutoLandmarksOrPca; changed = true; }
        }
        if (GUILayout.Button("Mốc P3", isLmInit ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isLmInit) { p4Config.initMode = TrajectoryEvaluationEngine.P4InitMode.LandmarksP3Only; changed = true; }
        }
        if (GUILayout.Button("PCA 3D", isPcaInit ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isPcaInit) { p4Config.initMode = TrajectoryEvaluationEngine.P4InitMode.Pca3DOnly; changed = true; }
        }
        GUILayout.EndHorizontal();

        // 2. Đăng ký phi cứng (Bước 5 tài liệu P4)
        GUILayout.Label("Bước 5 - Biến dạng phi cứng (Non-Rigid):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isCpd = (p4Config.nonRigidMode == TrajectoryEvaluationEngine.P4NonRigidMode.NonRigidCpd);
        bool isTps = (p4Config.nonRigidMode == TrajectoryEvaluationEngine.P4NonRigidMode.TpsRbf);
        bool isRigid = (p4Config.nonRigidMode == TrajectoryEvaluationEngine.P4NonRigidMode.RigidIcpOnly);
        if (GUILayout.Button("CPD phi cứng", isCpd ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isCpd) { p4Config.nonRigidMode = TrajectoryEvaluationEngine.P4NonRigidMode.NonRigidCpd; changed = true; }
        }
        if (GUILayout.Button("TPS RBF", isTps ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isTps) { p4Config.nonRigidMode = TrajectoryEvaluationEngine.P4NonRigidMode.TpsRbf; changed = true; }
        }
        if (GUILayout.Button("Chỉ ICP cứng", isRigid ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isRigid) { p4Config.nonRigidMode = TrajectoryEvaluationEngine.P4NonRigidMode.RigidIcpOnly; changed = true; }
        }
        GUILayout.EndHorizontal();

        // 3. Tham số Voxel & ICP 3D
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Giảm điểm Voxel: {p4Config.voxelSizeMm:F1} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
        float newVox = Mathf.Round(GUILayout.HorizontalSlider(p4Config.voxelSizeMm, 2.5f, 12.0f) * 2f) / 2f;
        if (Mathf.Abs(newVox - p4Config.voxelSizeMm) > 0.1f) { p4Config.voxelSizeMm = newVox; changed = true; }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Vòng lặp ICP cứng: {p4Config.icpMaxIterations}", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
        int newIter = Mathf.RoundToInt(GUILayout.HorizontalSlider(p4Config.icpMaxIterations, 4, 30));
        if (newIter != p4Config.icpMaxIterations) { p4Config.icpMaxIterations = newIter; changed = true; }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Bán kính ghép ICP: {p4Config.icpMaxMatchRadiusMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
        float newRad = Mathf.Round(GUILayout.HorizontalSlider(p4Config.icpMaxMatchRadiusMm, 25f, 160f));
        if (Mathf.Abs(newRad - p4Config.icpMaxMatchRadiusMm) > 0.5f) { p4Config.icpMaxMatchRadiusMm = newRad; changed = true; }
        GUILayout.EndHorizontal();

        // 4. Tham số CPD hoặc TPS tùy chế độ
        if (isCpd)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Độ cứng CPD λ: {p4Config.cpdLambda:F2}", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
            float newLam = Mathf.Round(GUILayout.HorizontalSlider(p4Config.cpdLambda, 0.25f, 8.0f) * 20f) / 20f;
            if (Mathf.Abs(newLam - p4Config.cpdLambda) > 0.01f) { p4Config.cpdLambda = newLam; changed = true; }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Bán kính mượt β: {p4Config.cpdBetaMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
            float newBeta = Mathf.Round(GUILayout.HorizontalSlider(p4Config.cpdBetaMm, 60f, 350f));
            if (Mathf.Abs(newBeta - p4Config.cpdBetaMm) > 1f) { p4Config.cpdBetaMm = newBeta; changed = true; }
            GUILayout.EndHorizontal();
        }
        else if (isTps)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Điều chuẩn TPS λ: {p4Config.tpsLambda:F3}", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
            float newTps = Mathf.Round(GUILayout.HorizontalSlider(p4Config.tpsLambda, 0.002f, 0.080f) * 1000f) / 1000f;
            if (Mathf.Abs(newTps - p4Config.tpsLambda) > 0.0005f) { p4Config.tpsLambda = newTps; changed = true; }
            GUILayout.EndHorizontal();
        }

        // 5. Ngưỡng sai số dư an toàn & Khoảng lùi súng d
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Ngưỡng chặn an toàn: {p4Config.residualThresholdMm:F1} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
        float newThr = Mathf.Round(GUILayout.HorizontalSlider(p4Config.residualThresholdMm, 3f, 35f) * 2f) / 2f;
        if (Mathf.Abs(newThr - p4Config.residualThresholdMm) > 0.1f)
        {
            p4Config.residualThresholdMm = newThr;
            if (sprayTechParams != null) sprayTechParams.registrationResidualThresholdMm = newThr;
            changed = true;
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Khoảng cách súng d: {p4Config.sprayDistanceDMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(168));
        float newD4 = Mathf.Round(GUILayout.HorizontalSlider(p4Config.sprayDistanceDMm, 0f, 250f));
        if (Mathf.Abs(newD4 - p4Config.sprayDistanceDMm) > 0.1f)
        {
            p4Config.sprayDistanceDMm = newD4;
            customDrawZOffsetMm = newD4;
            if (sprayTechParams != null) sprayTechParams.standoffMm = newD4;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 6. Hướng súng (Bước 7 tài liệu P4)
        GUILayout.Label("Bước 7 - Bám mặt 3D & Hướng súng:", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        bool isP4Norm = (p4Config.gunOrientation == TrajectoryEvaluationEngine.P4GunOrientationMode.SmoothSurfaceNormal25D);
        bool isP4Vert = (p4Config.gunOrientation == TrajectoryEvaluationEngine.P4GunOrientationMode.Vertical25D);
        bool isP4Flat = (p4Config.gunOrientation == TrajectoryEvaluationEngine.P4GunOrientationMode.FlatPlane2D);
        if (GUILayout.Button("3D Pháp tuyến", isP4Norm ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isP4Norm) { p4Config.gunOrientation = TrajectoryEvaluationEngine.P4GunOrientationMode.SmoothSurfaceNormal25D; changed = true; }
        }
        if (GUILayout.Button("2.5D Đứng", isP4Vert ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isP4Vert) { p4Config.gunOrientation = TrajectoryEvaluationEngine.P4GunOrientationMode.Vertical25D; changed = true; }
        }
        if (GUILayout.Button("2D Phẳng", isP4Flat ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            if (!isP4Flat) { p4Config.gunOrientation = TrajectoryEvaluationEngine.P4GunOrientationMode.FlatPlane2D; changed = true; }
        }
        GUILayout.EndHorizontal();

        bool newFill4 = GUILayout.Toggle(p4Config.fillClosedDrawnRegions, " Phủ lượt phun trong hình kín (Tròn/Vuông/Đa giác)", AntigravityUITheme.StyleToggle);
        if (newFill4 != p4Config.fillClosedDrawnRegions) { p4Config.fillClosedDrawnRegions = newFill4; changed = true; }

        bool newClip4 = GUILayout.Toggle(p4Config.clipToGarmentContour, " Cắt điểm tràn ngoài đường bao quần", AntigravityUITheme.StyleToggle);
        if (newClip4 != p4Config.clipToGarmentContour) { p4Config.clipToGarmentContour = newClip4; changed = true; }

        // 7. 3 lớp hiển thị độc lập + Đám mây đăng ký P4
        GUILayout.Label("Tách lớp hiển thị (bật/tắt độc lập):", AntigravityUITheme.StyleLabelMuted);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(showSprayZones3D ? "Chỗ phun: BẬT" : "Chỗ phun: TẮT",
            showSprayZones3D ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showSprayZones3D = !showSprayZones3D;
        }
        if (GUILayout.Button(showTurnaroundConnectors ? "Quỹ đạo: BẬT" : "Quỹ đạo: TẮT",
            showTurnaroundConnectors ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showTurnaroundConnectors = !showTurnaroundConnectors;
        }
        if (GUILayout.Button(showGizmos ? "Đường phun: BẬT" : "Đường phun: TẮT",
            showGizmos ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton, GUILayout.Height(22)))
        {
            showGizmos = !showGizmos;
            MarkSettingsDirty();
        }
        GUILayout.EndHorizontal();
        showP4RegisteredCloud3D = GUILayout.Toggle(showP4RegisteredCloud3D, " Hiện đám mây 3D đã đăng ký (Xanh chanh)", AntigravityUITheme.StyleToggle);

        // 8. Tự cập nhật & Nút sinh lại quỹ đạo P4
        p4AutoRegenerateOnChange = GUILayout.Toggle(p4AutoRegenerateOnChange, " Tự sinh lại đường quỹ đạo khi chỉnh thông số", AntigravityUITheme.StyleToggle);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Sinh quỹ đạo P4", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(26)))
        {
            GenerateAndApplyMethodTrajectory("P4");
            changed = false;
        }
        if (GUILayout.Button("Mặc định P4", AntigravityUITheme.StyleButton, GUILayout.Width(88), GUILayout.Height(26)))
        {
            p4Config = new TrajectoryEvaluationEngine.P4RegistrationConfig();
            p4Config.sprayDistanceDMm = customDrawZOffsetMm;
            changed = true;
        }
        GUILayout.EndHorizontal();

        // 9. Hiển thị trực tiếp chỉ số kiểm tra P4 (Mục 8 tài liệu P4)
        if (lastP4Result != null)
        {
            GUILayout.Space(3);
            if (!string.IsNullOrEmpty(lastP4Result.summaryText))
            {
                GUILayout.Label(lastP4Result.summaryText, lastP4Result.rejectedByResidualGate
                    ? AntigravityUITheme.StyleLabelYellow : AntigravityUITheme.StyleLabelGreen);
            }
            GUILayout.Label(
                $"Sai số dư: Init {lastP4Result.residualAfterInitMm:F1} -> ICP {lastP4Result.residualAfterIcpMm:F1} -> Cuối {lastP4Result.residualAfterNonRigidMm:F2} mm",
                lastP4Result.rejectedByResidualGate ? AntigravityUITheme.StyleLabelYellow : AntigravityUITheme.StyleLabelCyan);
            GUILayout.Label(
                $"Bám mặt TB: {lastP4Result.meanSurfaceSnapShiftMm:F2} mm | Ổn định góc n: ±{lastP4Result.normalStabilityStdDeg:F1}° | Voxel: {lastP4Result.sourceVoxelCount}->{lastP4Result.targetVoxelCount}",
                AntigravityUITheme.StyleLabelMuted);
            GUILayout.Label(
                $"Phun ON: {lastP4Result.totalSprayLengthMm:F0} mm | Di chuyển OFF: {lastP4Result.totalTravelLengthMm:F0} mm | Gate: {(lastP4Result.rejectedByResidualGate ? "CHẶN AN TOÀN" : "ĐẠT")}",
                lastP4Result.rejectedByResidualGate ? AntigravityUITheme.StyleLabelYellow : AntigravityUITheme.StyleLabelMuted);
            if (!string.IsNullOrEmpty(lastP4Result.warningMessage))
            {
                GUILayout.Label(lastP4Result.warningMessage, AntigravityUITheme.StyleLabelYellow);
            }
        }

        GUILayout.EndVertical();

        if (changed && p4AutoRegenerateOnChange)
        {
            GenerateAndApplyMethodTrajectory("P4");
        }
    }

    private bool TryProjectWorldPointToActiveCanvasUV(
        Vector3 worldPt,
        bool usePlyDirectUv,
        PLYDirectLoader plyLoader,
        bool useAffineWorldToUv,
        double[] coefU,
        double[] coefV,
        out Vector2 uv)
    {
        if (usePlyDirectUv && plyLoader != null)
        {
            return plyLoader.TryWorldToCameraUV(worldPt, out uv);
        }
        if (useAffineWorldToUv && coefU != null && coefV != null)
        {
            double u = coefU[0] + coefU[1] * worldPt.x + coefU[2] * worldPt.z;
            double v = coefV[0] + coefV[1] * worldPt.x + coefV[2] * worldPt.z;
            uv = new Vector2(Mathf.Clamp01((float)u), Mathf.Clamp01((float)v));
            return true;
        }
        uv = Vector2.zero;
        return false;
    }

    private void ProjectTargetRegionsOntoActiveCameraCanvas(
        PLYDirectLoader plyLoader,
        bool usePlyDirectUv,
        bool useAffineWorldToUv,
        double[] coefU,
        double[] coefV)
    {
        projectedTargetRegionR2D.Clear();
        projectedSafeRegionRsafe2D.Clear();
        projectedP2Skeleton2D.Clear();
        projectedP3MeshTriangles2D.Clear();
        projectedP3TriangleFlipped2D.Clear();
        projectedP4RegisteredCloud2D.Clear();
        if (!usePlyDirectUv && !useAffineWorldToUv) return;

        List<List<Vector3>> rawWorldPolys = null;
        List<List<Vector3>> safeWorldPolys = null;

        if (activeTrajectoryMethod == "P1" && lastP1Result != null && lastP1Result.rawTargetRegionPolygonsWorld.Count > 0)
        {
            rawWorldPolys = lastP1Result.rawTargetRegionPolygonsWorld;
            safeWorldPolys = lastP1Result.safeErodedRegionPolygonsWorld;
        }
        else if (activeTrajectoryMethod == "P2" && lastP2Result != null && lastP2Result.targetRegionPolygonsWorld.Count > 0)
        {
            rawWorldPolys = lastP2Result.targetRegionPolygonsWorld;
            if (lastP2Result.skeletonAxesWorld != null)
            {
                for (int a = 0; a < lastP2Result.skeletonAxesWorld.Count; a++)
                {
                    var axis = lastP2Result.skeletonAxesWorld[a];
                    if (axis == null || axis.Count < 2) continue;
                    var uvAxis = new List<Vector2>(axis.Count);
                    for (int i = 0; i < axis.Count; i++)
                    {
                        if (TryProjectWorldPointToActiveCanvasUV(axis[i], usePlyDirectUv, plyLoader, useAffineWorldToUv, coefU, coefV, out Vector2 uv))
                            uvAxis.Add(uv);
                    }
                    if (uvAxis.Count >= 2) projectedP2Skeleton2D.Add(uvAxis);
                }
            }
        }
        else if (activeTrajectoryMethod == "P3" && lastP3Result != null)
        {
            if (lastP3Result.targetRegionPolygonsWorld != null && lastP3Result.targetRegionPolygonsWorld.Count > 0)
            {
                rawWorldPolys = lastP3Result.targetRegionPolygonsWorld;
            }
            if (lastP3Result.meshTrianglesWorld != null)
            {
                for (int t = 0; t < lastP3Result.meshTrianglesWorld.Count; t++)
                {
                    Vector3[] tri = lastP3Result.meshTrianglesWorld[t];
                    if (tri == null || tri.Length < 3) continue;
                    if (TryProjectWorldPointToActiveCanvasUV(tri[0], usePlyDirectUv, plyLoader, useAffineWorldToUv, coefU, coefV, out Vector2 u0) &&
                        TryProjectWorldPointToActiveCanvasUV(tri[1], usePlyDirectUv, plyLoader, useAffineWorldToUv, coefU, coefV, out Vector2 u1) &&
                        TryProjectWorldPointToActiveCanvasUV(tri[2], usePlyDirectUv, plyLoader, useAffineWorldToUv, coefU, coefV, out Vector2 u2))
                    {
                        projectedP3MeshTriangles2D.Add(new Vector2[] { u0, u1, u2 });
                        bool isFl = (t < lastP3Result.meshTriangleFlipped.Count) && lastP3Result.meshTriangleFlipped[t];
                        projectedP3TriangleFlipped2D.Add(isFl);
                    }
                }
            }
        }
        else if (activeTrajectoryMethod == "P4" && lastP4Result != null)
        {
            if (lastP4Result.targetRegionPolygonsWorld != null && lastP4Result.targetRegionPolygonsWorld.Count > 0)
            {
                rawWorldPolys = lastP4Result.targetRegionPolygonsWorld;
            }
            if (lastP4Result.registeredCloudWorld != null && lastP4Result.registeredCloudWorld.Count > 0)
            {
                int total = lastP4Result.registeredCloudWorld.Count;
                int step = Mathf.Max(1, total / 360);
                for (int i = 0; i < total; i += step)
                {
                    if (TryProjectWorldPointToActiveCanvasUV(lastP4Result.registeredCloudWorld[i], usePlyDirectUv, plyLoader, useAffineWorldToUv, coefU, coefV, out Vector2 uv))
                        projectedP4RegisteredCloud2D.Add(uv);
                }
            }
        }
        else
        {
            rawWorldPolys = new List<List<Vector3>>();
            if (sprayZoneLeftWorld != null && sprayZoneLeftWorld.Count >= 3) rawWorldPolys.Add(sprayZoneLeftWorld);
            if (sprayZoneRightWorld != null && sprayZoneRightWorld.Count >= 3) rawWorldPolys.Add(sprayZoneRightWorld);
        }

        if (rawWorldPolys != null)
        {
            for (int r = 0; r < rawWorldPolys.Count; r++)
            {
                var poly = rawWorldPolys[r];
                if (poly == null || poly.Count < 3) continue;
                var uvLoop = new List<Vector2>(poly.Count);
                for (int i = 0; i < poly.Count; i++)
                {
                    if (TryProjectWorldPointToActiveCanvasUV(poly[i], usePlyDirectUv, plyLoader, useAffineWorldToUv, coefU, coefV, out Vector2 uv))
                        uvLoop.Add(uv);
                }
                if (uvLoop.Count >= 3) projectedTargetRegionR2D.Add(uvLoop);
            }
        }

        if (safeWorldPolys != null)
        {
            for (int r = 0; r < safeWorldPolys.Count; r++)
            {
                var poly = safeWorldPolys[r];
                if (poly == null || poly.Count < 3) continue;
                var uvLoop = new List<Vector2>(poly.Count);
                for (int i = 0; i < poly.Count; i++)
                {
                    if (TryProjectWorldPointToActiveCanvasUV(poly[i], usePlyDirectUv, plyLoader, useAffineWorldToUv, coefU, coefV, out Vector2 uv))
                        uvLoop.Add(uv);
                }
                if (uvLoop.Count >= 3) projectedSafeRegionRsafe2D.Add(uvLoop);
            }
        }
    }

    /// <summary>
    /// Vẽ tách biệt hoàn toàn 2 lớp lên 2D Camera Canvas & Fullscreen 2D Canvas:
    /// - Lớp 1 (showSprayZones3D): CHỖ CẦN PHUN (Vùng R màu vàng cam + Vùng an toàn R_safe sau co biên e màu xanh lá).
    /// - Lớp 2 (showGizmos): ĐƯỜNG QUỸ ĐẠO (Cyan liền nét = BẬT SÚNG; Đứt cam = Quay đầu / TẮT SÚNG khi showTurnaroundConnectors bật).
    /// </summary>
    private void DrawTargetAndTrajectoryOverlaysOn2DCanvas(Rect canvasRect, bool isFullscreen)
    {
        if (Event.current != null && Event.current.type != EventType.Repaint) return;
        float haloMul = isFullscreen ? 1.30f : 1.0f;
        float coreMul = isFullscreen ? 1.25f : 1.0f;

        // Tự chiếu vùng phun AI nếu chưa có cache vùng mục tiêu 2D
        if (showSprayZones3D && projectedTargetRegionR2D.Count == 0 &&
            ((sprayZoneLeftWorld != null && sprayZoneLeftWorld.Count >= 3) || (sprayZoneRightWorld != null && sprayZoneRightWorld.Count >= 3)))
        {
            Texture2D camTex = GetActiveRealCameraTexture(out PLYDirectLoader pl, out int nw, out int nh, out _);
            bool usingCam = (camTex != null);
            Vector2[] camLm = GetActiveCanvasLandmarks12UV(pl, nw, nh, usingCam, out bool hasLm);
            bool useDirect = (realCameraFileTex == null && pl != null && (pl.hasTopDownCameraTexture || pl.hasOrganizedCameraFrame));
            bool useAff = false;
            double[] cU = null, cV = null;
            if (!useDirect && landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
            {
                Vector2[] wXZ = new Vector2[12];
                for (int i = 0; i < 12; i++) wXZ[i] = new Vector2(landmarksWorldPos[i].x, landmarksWorldPos[i].z);
                Vector2[] dstU = (usingCam && hasLm && camLm != null) ? camLm : GetCanonicalLandmarks12UV(activeSprayTemplateSize);
                useAff = FitAffine2D(wXZ, dstU, out cU, out cV);
            }
            ProjectTargetRegionsOntoActiveCameraCanvas(pl, useDirect, useAff, cU, cV);
        }

        // 1. LỚP CHỖ CẦN PHUN (Vùng R gốc vàng cam & Vùng an toàn R_safe xanh lá sau co biên e)
        if (showSprayZones3D)
        {
            Color rawHalo = new Color(0.20f, 0.09f, 0.01f, 0.90f);
            Color rawCore = new Color(1.0f, 0.72f, 0.08f, 0.98f);
            Color rawHatch = new Color(1.0f, 0.72f, 0.08f, 0.25f);
            for (int r = 0; r < projectedTargetRegionR2D.Count; r++)
            {
                var loop = projectedTargetRegionR2D[r];
                if (loop == null || loop.Count < 3) continue;
                int lc = loop.Count;
                int stride = Mathf.Max(1, lc / 8);
                for (int k = 0; k < lc / 2; k += stride)
                {
                    int opp = (lc - 1 - k) % lc;
                    Vector2 h0 = new Vector2(canvasRect.x + loop[k].x * canvasRect.width, canvasRect.y + loop[k].y * canvasRect.height);
                    Vector2 h1 = new Vector2(canvasRect.x + loop[opp].x * canvasRect.width, canvasRect.y + loop[opp].y * canvasRect.height);
                    DrawGuiLine2D(h0, h1, rawHatch, 1.2f);
                }
                for (int p = 0; p < lc; p++)
                {
                    Vector2 a = loop[p];
                    Vector2 b = loop[(p + 1) % lc];
                    Vector2 p0 = new Vector2(canvasRect.x + a.x * canvasRect.width, canvasRect.y + a.y * canvasRect.height);
                    Vector2 p1 = new Vector2(canvasRect.x + b.x * canvasRect.width, canvasRect.y + b.y * canvasRect.height);
                    DrawGuiLine2D(p0, p1, rawHalo, 4.2f * haloMul);
                    DrawGuiLine2D(p0, p1, rawCore, 2.3f * coreMul);
                }
            }

            Color safeHalo = new Color(0.02f, 0.20f, 0.06f, 0.90f);
            Color safeCore = new Color(0.32f, 1.0f, 0.36f, 0.98f);
            for (int r = 0; r < projectedSafeRegionRsafe2D.Count; r++)
            {
                var loop = projectedSafeRegionRsafe2D[r];
                if (loop == null || loop.Count < 3) continue;
                int lc = loop.Count;
                for (int p = 0; p < lc; p++)
                {
                    Vector2 a = loop[p];
                    Vector2 b = loop[(p + 1) % lc];
                    Vector2 p0 = new Vector2(canvasRect.x + a.x * canvasRect.width, canvasRect.y + a.y * canvasRect.height);
                    Vector2 p1 = new Vector2(canvasRect.x + b.x * canvasRect.width, canvasRect.y + b.y * canvasRect.height);
                    DrawGuiLine2D(p0, p1, safeHalo, 3.4f * haloMul);
                    DrawGuiLine2D(p0, p1, safeCore, 1.8f * coreMul);
                }
            }
        }

        // 1a. CÁC VÙNG HÌNH CƠ BẢN DO NGƯỜI DÙNG VẼ (Tròn / Vuông / Đa giác) — luôn hiển thị giữ nguyên hình mượt mà
        if (customDrawnRegions != null && customDrawnRegions.Count > 0)
        {
            for (int r = 0; r < customDrawnRegions.Count; r++)
            {
                DrawDrawnRegion2D(customDrawnRegions[r], canvasRect, isFullscreen, isSelected: (r == selectedRegionIndex));
            }
        }

        // 1b. LỚP TRỤC GIỮA ỐNG QUẦN (SKELETON P2 - Hồng tím) độc lập với Chỗ cần phun & Đường quỹ đạo
        if (showP2SkeletonAxis && activeTrajectoryMethod == "P2" && projectedP2Skeleton2D.Count > 0)
        {
            Color skelHalo = new Color(0.18f, 0.02f, 0.22f, 0.88f);
            Color skelCore = new Color(0.96f, 0.26f, 1.0f, 0.98f);
            for (int a = 0; a < projectedP2Skeleton2D.Count; a++)
            {
                var axis = projectedP2Skeleton2D[a];
                if (axis == null || axis.Count < 2) continue;
                for (int p = 0; p < axis.Count - 1; p++)
                {
                    Vector2 p0 = new Vector2(canvasRect.x + axis[p].x * canvasRect.width, canvasRect.y + axis[p].y * canvasRect.height);
                    Vector2 p1 = new Vector2(canvasRect.x + axis[p + 1].x * canvasRect.width, canvasRect.y + axis[p + 1].y * canvasRect.height);
                    DrawGuiLine2D(p0, p1, skelHalo, 3.8f * haloMul);
                    DrawGuiDashedLine2D(p0, p1, skelCore, 2.2f * coreMul, 7f, 4f);
                }
            }
        }

        // 1c. LỚP LƯỚI 32 TAM GIÁC BARYCENTRIC (P3 - Xanh ngọc / Đỏ nếu tam giác bị đảo)
        if (showP3TriangulationMesh && activeTrajectoryMethod == "P3" && projectedP3MeshTriangles2D.Count > 0)
        {
            Color triOk = new Color(0.20f, 0.95f, 0.65f, 0.72f);
            Color triBad = new Color(1.0f, 0.15f, 0.15f, 0.95f);
            for (int t = 0; t < projectedP3MeshTriangles2D.Count; t++)
            {
                Vector2[] tri = projectedP3MeshTriangles2D[t];
                if (tri == null || tri.Length < 3) continue;
                bool isFl = (t < projectedP3TriangleFlipped2D.Count) && projectedP3TriangleFlipped2D[t];
                Color col = isFl ? triBad : triOk;
                float w = (isFl ? 2.2f : 1.2f) * coreMul;
                for (int e = 0; e < 3; e++)
                {
                    Vector2 a = tri[e];
                    Vector2 b = tri[(e + 1) % 3];
                    Vector2 p0 = new Vector2(canvasRect.x + a.x * canvasRect.width, canvasRect.y + a.y * canvasRect.height);
                    Vector2 p1 = new Vector2(canvasRect.x + b.x * canvasRect.width, canvasRect.y + b.y * canvasRect.height);
                    DrawGuiLine2D(p0, p1, col, w);
                }
            }
        }

        // 1d. LỚP ĐÁM MÂY 3D QUẦN MẪU ĐÃ ĐĂNG KÝ (P4 - Xanh chanh)
        if (showP4RegisteredCloud3D && activeTrajectoryMethod == "P4" && projectedP4RegisteredCloud2D.Count > 0)
        {
            Color cHalo = new Color(0.04f, 0.14f, 0.02f, 0.75f);
            Color cCore = new Color(0.55f, 1.0f, 0.18f, 0.90f);
            float sz = isFullscreen ? 2.8f : 1.8f;
            for (int i = 0; i < projectedP4RegisteredCloud2D.Count; i++)
            {
                Vector2 uv = projectedP4RegisteredCloud2D[i];
                float px = canvasRect.x + uv.x * canvasRect.width;
                float py = canvasRect.y + uv.y * canvasRect.height;
                DrawGuiSolidRect(new Rect(px - sz - 0.6f, py - sz - 0.6f, (sz + 0.6f) * 2f, (sz + 0.6f) * 2f), cHalo);
                DrawGuiSolidRect(new Rect(px - sz, py - sz, sz * 2f, sz * 2f), cCore);
            }
        }

        // 2 & 3. LỚP ĐƯỜNG QUỸ ĐẠO (Đứt cam = Quét qua lại / TẮT SÚNG) và LỚP ĐƯỜNG PHUN (Cyan liền nét = BẬT SÚNG) ĐỘC LẬP NHAU
        bool isMethodTraj = ((activeTrajectoryMethod == "P1" || activeTrajectoryMethod == "P2" ||
                              activeTrajectoryMethod == "P3" || activeTrajectoryMethod == "P4")
                             || (activeTrajectoryMethod == "CUSTOM" && HasCustomDrawnRegions2D))
                            && (projectedTrajectoryOn2D.Count > 0 || projectedTrajectoryOff2D.Count > 0);

        if (isMethodTraj)
        {
            // Lớp 2: ĐƯỜNG QUỸ ĐẠO (Tắt súng / Quét qua lại / Quay đầu — nét đứt cam)
            if (showTurnaroundConnectors)
            {
                Color offCol = new Color(1.0f, 0.44f, 0.24f, 0.90f);
                for (int c = 0; c < projectedTrajectoryOff2D.Count; c++)
                {
                    var conn = projectedTrajectoryOff2D[c];
                    if (conn == null || conn.Count < 2) continue;
                    for (int p = 0; p < conn.Count - 1; p++)
                    {
                        Vector2 p0 = new Vector2(canvasRect.x + conn[p].x * canvasRect.width, canvasRect.y + conn[p].y * canvasRect.height);
                        Vector2 p1 = new Vector2(canvasRect.x + conn[p + 1].x * canvasRect.width, canvasRect.y + conn[p + 1].y * canvasRect.height);
                        DrawGuiDashedLine2D(p0, p1, offCol, 1.8f * coreMul, 5.5f, 4.5f);
                    }
                }

                // Nếu người dùng TẮT Đường phun và chỉ BẬT Đường quỹ đạo -> vẽ nối tiếp đoạn ON thành nét đứt cam để thấy trọn vẹn lộ trình quét!
                if (!showGizmos)
                {
                    for (int s = 0; s < projectedTrajectoryOn2D.Count; s++)
                    {
                        var stroke = projectedTrajectoryOn2D[s];
                        if (stroke == null || stroke.Count < 2) continue;
                        for (int p = 0; p < stroke.Count - 1; p++)
                        {
                            Vector2 p0 = new Vector2(canvasRect.x + stroke[p].x * canvasRect.width, canvasRect.y + stroke[p].y * canvasRect.height);
                            Vector2 p1 = new Vector2(canvasRect.x + stroke[p + 1].x * canvasRect.width, canvasRect.y + stroke[p + 1].y * canvasRect.height);
                            DrawGuiDashedLine2D(p0, p1, offCol, 1.7f * coreMul, 5.5f, 4.5f);
                        }
                    }
                }
            }

            // Lớp 3: ĐƯỜNG PHUN (Chỉ các đoạn BẬT SÚNG thực sự phun lên mẫu — Cyan nét liền đậm)
            if (showGizmos)
            {
                Color haloCol = new Color(0.02f, 0.08f, 0.16f, 0.90f);
                Color onCol = new Color(0.10f, 0.95f, 1.0f, 0.98f);
                for (int s = 0; s < projectedTrajectoryOn2D.Count; s++)
                {
                    var stroke = projectedTrajectoryOn2D[s];
                    if (stroke == null || stroke.Count < 2) continue;
                    for (int p = 0; p < stroke.Count - 1; p++)
                    {
                        Vector2 p0 = new Vector2(canvasRect.x + stroke[p].x * canvasRect.width, canvasRect.y + stroke[p].y * canvasRect.height);
                        Vector2 p1 = new Vector2(canvasRect.x + stroke[p + 1].x * canvasRect.width, canvasRect.y + stroke[p + 1].y * canvasRect.height);
                        DrawGuiLine2D(p0, p1, haloCol, 4.0f * haloMul);
                        DrawGuiLine2D(p0, p1, onCol, 2.4f * coreMul);
                    }
                    if (s == 0)
                    {
                        Vector2 startPt = new Vector2(canvasRect.x + stroke[0].x * canvasRect.width, canvasRect.y + stroke[0].y * canvasRect.height);
                        float dot = isFullscreen ? 4.5f : 3f;
                        DrawGuiSolidRect(new Rect(startPt.x - dot, startPt.y - dot, dot * 2f, dot * 2f), Color.yellow);
                    }
                }
            }
        }
        else if (showGizmos)
        {
            float brushPx = GetBrushDiameterPxOnCanvas(canvasRect, sprayWidthMm);
            float coreLinePx = Mathf.Clamp(brushPx, 2.0f, isFullscreen ? 4.2f : 3.2f);
            float haloLinePx = coreLinePx + (isFullscreen ? 2.0f : 1.5f);
            bool showWideSprayBand = brushPx > coreLinePx + 1.2f;

            for (int s = 0; s < customDrawnStrokes.Count; s++)
            {
                var stroke = customDrawnStrokes[s];
                if (stroke == null || stroke.Count == 0) continue;
                Color strokeCol = (s % 2 == 0) ? new Color(0.1f, 0.95f, 1f, 0.98f) : new Color(0.25f, 1f, 0.45f, 0.98f);
                Color bandCol = new Color(strokeCol.r, strokeCol.g, strokeCol.b, 0.30f);
                Color haloCol = new Color(0.02f, 0.06f, 0.12f, 0.85f);
                for (int p = 0; p < stroke.Count - 1; p++)
                {
                    Vector2 p0 = new Vector2(canvasRect.x + stroke[p].x * canvasRect.width, canvasRect.y + stroke[p].y * canvasRect.height);
                    Vector2 p1 = new Vector2(canvasRect.x + stroke[p + 1].x * canvasRect.width, canvasRect.y + stroke[p + 1].y * canvasRect.height);
                    if (showWideSprayBand)
                    {
                        DrawGuiLine2D(p0, p1, bandCol, brushPx);
                    }
                    DrawGuiLine2D(p0, p1, haloCol, haloLinePx);
                    DrawGuiLine2D(p0, p1, strokeCol, coreLinePx);
                }
                Vector2 startPt = new Vector2(canvasRect.x + stroke[0].x * canvasRect.width, canvasRect.y + stroke[0].y * canvasRect.height);
                float dot = isFullscreen ? 4.5f : 3f;
                DrawGuiSolidRect(new Rect(startPt.x - dot, startPt.y - dot, dot * 2f, dot * 2f), Color.yellow);
            }
        }

        if (isDrawingOnCanvas && currentDrawingStroke != null && currentDrawingStroke.Count > 1)
        {
            // Luôn hiện nét đang kéo chuột khi người dùng tự khoanh vùng phun
            float brushPx = GetBrushDiameterPxOnCanvas(canvasRect, sprayWidthMm);
            Color drawBandCol = new Color(1.0f, 0.85f, 0.20f, 0.32f);
            Color drawCol = new Color(1.0f, 0.85f, 0.20f, 0.98f);
            for (int p = 0; p < currentDrawingStroke.Count - 1; p++)
            {
                Vector2 p0 = new Vector2(canvasRect.x + currentDrawingStroke[p].x * canvasRect.width, canvasRect.y + currentDrawingStroke[p].y * canvasRect.height);
                Vector2 p1 = new Vector2(canvasRect.x + currentDrawingStroke[p + 1].x * canvasRect.width, canvasRect.y + currentDrawingStroke[p + 1].y * canvasRect.height);
                if (brushPx > 4.2f) DrawGuiLine2D(p0, p1, drawBandCol, brushPx);
                DrawGuiLine2D(p0, p1, drawCol, Mathf.Clamp(brushPx, 2.2f, 3.8f));
            }
        }
    }

    /// <summary>
    /// Chiếu trực tiếp quỹ đạo 3D vừa sinh (P1 / P2 / P3 / P4) và vùng cần phun (R & R_safe)
    /// ngược lên ảnh Camera 2D thực tế vào các lớp tách biệt mà KHÔNG ghi đè mẫu gốc `customDrawnStrokes` / `canonicalPatternStrokes`.
    /// </summary>
    private void ProjectWaypointsOntoActiveCameraCanvas(List<WaypointData> generatedWaypoints, string activeSample)
    {
        Texture2D activeCamTex = GetActiveRealCameraTexture(out PLYDirectLoader plyLoader, out int nativeW, out int nativeH, out _);
        canvasUsingRealCamera = (activeCamTex != null);
        Vector2[] targetCanvasLmUV = GetActiveCanvasLandmarks12UV(plyLoader, nativeW, nativeH, canvasUsingRealCamera, out bool hasRealLm);

        bool usePlyDirectUv = (realCameraFileTex == null && plyLoader != null && (plyLoader.hasTopDownCameraTexture || plyLoader.hasOrganizedCameraFrame));
        bool useAffineWorldToUv = false;
        double[] coefU = null, coefV = null;

        if (!usePlyDirectUv && landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
        {
            Vector2[] worldLmXZ = new Vector2[12];
            for (int i = 0; i < 12; i++) worldLmXZ[i] = new Vector2(landmarksWorldPos[i].x, landmarksWorldPos[i].z);
            Vector2[] dstUv12 = (canvasUsingRealCamera && hasRealLm && targetCanvasLmUV != null)
                ? targetCanvasLmUV
                : GetCanonicalLandmarks12UV(activeSprayTemplateSize);
            useAffineWorldToUv = FitAffine2D(worldLmXZ, dstUv12, out coefU, out coefV);
        }

        // Luôn chiếu lớp CHỖ CẦN PHUN (Vùng R & R_safe) dù quỹ đạo có trống hay không (ví dụ e quá lớn)
        ProjectTargetRegionsOntoActiveCameraCanvas(plyLoader, usePlyDirectUv, useAffineWorldToUv, coefU, coefV);

        projectedTrajectoryOn2D.Clear();
        projectedTrajectoryOff2D.Clear();
        if (generatedWaypoints == null || generatedWaypoints.Count == 0) return;
        if (!usePlyDirectUv && !useAffineWorldToUv) return;

        float standoffMmUsed = customDrawZOffsetMm;
        if (activeTrajectoryMethod == "P1" && p1Config != null) standoffMmUsed = p1Config.sprayDistanceDMm;
        else if (activeTrajectoryMethod == "P2" && p2Config != null) standoffMmUsed = p2Config.sprayDistanceDMm;
        else if (activeTrajectoryMethod == "P3" && p3Config != null) standoffMmUsed = p3Config.sprayDistanceDMm;
        else if (activeTrajectoryMethod == "P4" && p4Config != null) standoffMmUsed = p4Config.sprayDistanceDMm;
        float standoffM = Mathf.Max(0f, standoffMmUsed) * 0.001f;

        // Chiếu trước toàn bộ waypoint sang UV trên ảnh camera 2D
        int wCount = generatedWaypoints.Count;
        Vector2[] wpUvs = new Vector2[wCount];
        bool[] wpOk = new bool[wCount];
        for (int i = 0; i < wCount; i++)
        {
            WaypointData wp = generatedWaypoints[i];
            Vector3 nrm = wp.microNormal.sqrMagnitude > 1e-4f ? wp.microNormal.normalized : Vector3.up;
            Vector3 surfW = wp.targetPosWorld_m - nrm * standoffM;
            wpOk[i] = TryProjectWorldPointToActiveCanvasUV(surfW, usePlyDirectUv, plyLoader, useAffineWorldToUv, coefU, coefV, out wpUvs[i]);
        }

        // Tách chính xác từng đoạn nối (i -> i+1):
        // - SPRAY ON (liền nét Cyan): wp[i+1].sprayOn == true & cùng segment & không nhảy lớn
        // - SPRAY OFF (nét đứt cam đỏ): các đoạn chạy trống ngoài vùng phun / quay đầu kiểu máy in 3D / nối giữa 2 vùng
        var onStrokes = new List<List<Vector2>>();
        var offStrokes = new List<List<Vector2>>();
        List<Vector2> curOn = null;
        List<Vector2> curOff = null;

        for (int i = 0; i < wCount - 1; i++)
        {
            if (!wpOk[i] || !wpOk[i + 1])
            {
                if (curOn != null && curOn.Count >= 2) onStrokes.Add(curOn);
                if (curOff != null && curOff.Count >= 2) offStrokes.Add(curOff);
                curOn = null; curOff = null;
                continue;
            }

            WaypointData w0 = generatedWaypoints[i];
            WaypointData w1 = generatedWaypoints[i + 1];
            bool isSameSeg = string.IsNullOrEmpty(w0.segmentName) || string.IsNullOrEmpty(w1.segmentName) || w0.segmentName == w1.segmentName;
            bool isSprayOnMove = w1.sprayOn && isSameSeg && Vector3.Distance(w0.targetPosGlobal_mm, w1.targetPosGlobal_mm) <= sprayJumpThresholdMm;

            if (isSprayOnMove)
            {
                if (curOff != null && curOff.Count >= 2) offStrokes.Add(curOff);
                curOff = null;

                if (curOn == null)
                {
                    curOn = new List<Vector2> { wpUvs[i], wpUvs[i + 1] };
                }
                else if (Vector2.Distance(curOn[curOn.Count - 1], wpUvs[i + 1]) >= 0.0015f)
                {
                    curOn.Add(wpUvs[i + 1]);
                }
            }
            else
            {
                if (curOn != null && curOn.Count >= 2) onStrokes.Add(curOn);
                curOn = null;

                if (curOff == null)
                {
                    curOff = new List<Vector2> { wpUvs[i], wpUvs[i + 1] };
                }
                else if (Vector2.Distance(curOff[curOff.Count - 1], wpUvs[i + 1]) >= 0.0015f)
                {
                    curOff.Add(wpUvs[i + 1]);
                }
            }
        }
        if (curOn != null && curOn.Count >= 2) onStrokes.Add(curOn);
        if (curOff != null && curOff.Count >= 2) offStrokes.Add(curOff);

        if (offStrokes.Count > 0)
        {
            projectedTrajectoryOff2D.AddRange(offStrokes);
        }
        if (onStrokes.Count > 0)
        {
            projectedTrajectoryOn2D.AddRange(onStrokes);
            hasInitializedDefaultStrokes = true;
            if (canvasUsingRealCamera && hasRealLm && targetCanvasLmUV != null)
            {
                strokesReferenceLandmarksUV = (Vector2[])targetCanvasLmUV.Clone();
                strokesProjectedSampleName = activeSample;
            }
        }
    }

    /// <summary>
    /// Khi người dùng vẽ / sửa / xóa vùng trên 2D Canvas: CHỈ xóa đường quỹ đạo cũ không còn khớp,
    /// KHÔNG tự tính quỹ đạo hay giải IK. Chỉ khi người dùng chọn P1 / P2 / P3 / P4 mới sinh đường quỹ đạo!
    /// </summary>
    public void InvalidateGeneratedTrajectoryOnEdit()
    {
        isPlaying = false;
        isMovingToManualTarget = false;
        isMovingToHome = false;
        paintingStarted = false;
        SetSprayState(false);
        ClearInkTrail();
        if (waypoints != null) waypoints.Clear();
        currentWaypointIndex = 0;

        activeTrajectoryMethod = "CUSTOM";
        lastP1Result = null;
        lastP2Result = null;
        lastP3Result = null;
        lastP4Result = null;
        projectedTargetRegionR2D.Clear();
        projectedSafeRegionRsafe2D.Clear();
        projectedP2Skeleton2D.Clear();
        projectedP3MeshTriangles2D.Clear();
        projectedP3TriangleFlipped2D.Clear();
        projectedP4RegisteredCloud2D.Clear();
        projectedTrajectoryOn2D.Clear();
        projectedTrajectoryOff2D.Clear();
        if (depositionHeatmapObj != null) depositionHeatmapObj.SetActive(false);
    }

    private void ReapplyActiveTrajectoryHeight()
    {
        if (p1Config != null) p1Config.sprayDistanceDMm = customDrawZOffsetMm;
        if (p2Config != null) p2Config.sprayDistanceDMm = customDrawZOffsetMm;
        if (p3Config != null) p3Config.sprayDistanceDMm = customDrawZOffsetMm;
        if (p4Config != null) p4Config.sprayDistanceDMm = customDrawZOffsetMm;
        if (activeTrajectoryMethod == "P1" || activeTrajectoryMethod == "P2" || activeTrajectoryMethod == "P3" || activeTrajectoryMethod == "P4")
        {
            GenerateAndApplyMethodTrajectory(activeTrajectoryMethod);
        }
    }

    /// <summary>
    /// Lưu đám mây điểm 3D giảm mẫu (~5 mm) và các nét quỹ đạo 3D trên mặt vải của chiếc quần chuẩn hiện tại
    /// để P4 thực hiện đăng ký đám mây điểm 3D thật (ICP 3D + CPD/TPS 3D) sang quần mới theo đúng Mức D trang 5 & 8 file góp ý.
    /// </summary>
    public void CaptureReferenceTemplateCloud3DForP4(PLYDirectLoader plyLoader = null, string sampleName = null)
    {
        if (plyLoader == null) plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (string.IsNullOrEmpty(sampleName)) sampleName = GetActiveCloudSampleName();
        if (string.IsNullOrEmpty(sampleName)) sampleName = selectedSampleName;

        if (plyLoader == null || waypoints == null || waypoints.Count < 2) return;

        float voxelM = (p4Config != null) ? Mathf.Clamp(p4Config.voxelSizeMm, 2.5f, 15f) * 0.001f : 0.005f;
        var cloud3D = plyLoader.ExtractGarmentSurfaceCloudVoxelized(voxelM, garmentContourWorld, 560);
        if (cloud3D == null || cloud3D.Count < 30) return;

        templateReferenceCloud3DWorld = cloud3D;
        templateReferenceSampleName = sampleName;
        templateReferenceLandmarks3D = (landmarksWorldPos != null && landmarksWorldPos.Count >= 12)
            ? new List<Vector3>(landmarksWorldPos)
            : new List<Vector3>();

        templateReferenceStrokes3DWorld.Clear();
        float standoffM = Mathf.Max(0f, customDrawZOffsetMm) * 0.001f;
        List<Vector3> cur = null;
        string curSeg = null;
        for (int i = 0; i < waypoints.Count; i++)
        {
            var wp = waypoints[i];
            Vector3 nrm = wp.microNormal.sqrMagnitude > 1e-4f ? wp.microNormal.normalized : Vector3.up;
            Vector3 surfPt = wp.targetPosWorld_m - nrm * standoffM;
            bool startNew = (cur == null) || (!wp.sprayOn) || (!string.IsNullOrEmpty(wp.segmentName) && wp.segmentName != curSeg);
            if (startNew)
            {
                if (cur != null && cur.Count >= 2) templateReferenceStrokes3DWorld.Add(cur);
                cur = new List<Vector3>();
                curSeg = wp.segmentName;
            }
            cur.Add(surfPt);
        }
        if (cur != null && cur.Count >= 2) templateReferenceStrokes3DWorld.Add(cur);
    }

    public void SetCurrentTrajectoryAsGroundTruth()
    {
        if (waypoints == null || waypoints.Count < 2)
        {
            comparisonStatusMsg = "Chưa có quỹ đạo vẽ tay hiện tại để chốt làm Ground Truth.";
            return;
        }
        string activeSample = GetActiveCloudSampleName();
        if (string.IsNullOrEmpty(activeSample)) activeSample = selectedSampleName;

        groundTruthWaypoints.Clear();
        for (int i = 0; i < waypoints.Count; i++)
        {
            var w = waypoints[i];
            groundTruthWaypoints.Add(new WaypointData
            {
                index = w.index,
                distance_mm = w.distance_mm,
                targetPosGlobal_mm = w.targetPosGlobal_mm,
                targetPosWorld_m = w.targetPosWorld_m,
                microNormal = w.microNormal,
                roll_deg = w.roll_deg,
                pitch_deg = w.pitch_deg,
                yaw_deg = w.yaw_deg,
                sprayOn = w.sprayOn,
                segmentName = w.segmentName
            });
        }
        groundTruthSampleName = activeSample;
        groundTruthSourceLabel = $"Dạy/vẽ tay [{activeSample}] ({groundTruthWaypoints.Count} wp)";
        comparisonStatusMsg = $"Đã chốt Ground Truth thật: {groundTruthSourceLabel}";
    }

    public void ImportGroundTruthCsvWithDialog()
    {
#if UNITY_EDITOR
        string defaultDir = Path.Combine(Application.dataPath, "..", "output_drawing");
        string path = UnityEditor.EditorUtility.OpenFilePanel("Chọn file Ground Truth (.csv chứa X_mm, Y_mm, Z_mm)", defaultDir, "csv");
        if (string.IsNullOrEmpty(path) || !File.Exists(path)) return;
        try
        {
            string[] lines = File.ReadAllLines(path);
            var loaded = new List<WaypointData>();
            int idx = 0;
            for (int i = 0; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line) || line.StartsWith("#")) continue;
                string[] parts = line.Split(',');
                if (parts.Length < 3) continue;
                if (!float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float xMm)) continue;
                if (!float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float yMm)) continue;
                if (!float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float zMm)) continue;
                Vector3 posMm = new Vector3(xMm, yMm, zMm);
                Vector3 posW = globalReference != null
                    ? globalReference.TransformPoint(posMm * 0.001f)
                    : new Vector3(-posMm.y * 0.001f, posMm.z * 0.001f, 1.100f - posMm.x * 0.001f);
                loaded.Add(new WaypointData
                {
                    index = idx++,
                    targetPosGlobal_mm = posMm,
                    targetPosWorld_m = posW,
                    microNormal = Vector3.up,
                    sprayOn = (idx > 1),
                    segmentName = "GT_EXT"
                });
            }
            if (loaded.Count >= 2)
            {
                groundTruthWaypoints = loaded;
                groundTruthSampleName = GetActiveCloudSampleName();
                groundTruthSourceLabel = $"File ngoài: {Path.GetFileName(path)} ({loaded.Count} wp)";
                comparisonStatusMsg = $"Đã nạp Ground Truth: {groundTruthSourceLabel}";
            }
            else
            {
                comparisonStatusMsg = "File CSV Ground Truth không hợp lệ (cần ít nhất 2 dòng X_mm,Y_mm,Z_mm).";
            }
        }
        catch (Exception ex)
        {
            comparisonStatusMsg = $"Lỗi đọc Ground Truth CSV: {ex.Message}";
        }
#else
        comparisonStatusMsg = "Hộp thoại nạp file Ground Truth chỉ hỗ trợ trong Unity Editor.";
#endif
    }

    public void ClearGroundTruth()
    {
        groundTruthWaypoints.Clear();
        groundTruthSampleName = "";
        groundTruthSourceLabel = "";
        comparisonStatusMsg = "Đã xóa Ground Truth (E_geo sẽ báo N/A khi so sánh, không lấy P4 tự chấm).";
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

        bool has12Lm = (landmarksWorldPos != null && landmarksWorldPos.Count >= 12);
        bool hasContourForP3 = (garmentContourWorld != null && garmentContourWorld.Count >= 8);
        bool hasMaskData = (garmentContourWorld != null && garmentContourWorld.Count >= 6)
            || (sprayZoneLeftWorld != null && sprayZoneLeftWorld.Count >= 4)
            || (sprayZoneRightWorld != null && sprayZoneRightWorld.Count >= 4)
            || HasCustomDrawnRegions2D;
        bool hasP4Ref3D = (templateReferenceCloud3DWorld != null && templateReferenceCloud3DWorld.Count >= 30
            && templateReferenceStrokes3DWorld != null && templateReferenceStrokes3DWorld.Count > 0);

        if (method == "P3" && !has12Lm && !(p3Config != null && p3Config.landmarkSource == TrajectoryEvaluationEngine.P3LandmarkSourceMode.ContourGeometry && hasContourForP3))
        {
            comparisonStatusMsg = $"P3 yêu cầu đủ 12 mốc AI (hoặc Contour quần khi chọn 'Hình học Contour') cho phôi [{activeSample}]. Hãy bấm 'Chụp + AI' hoặc 'Nhận diện AI' trước.";
            return;
        }
        if ((method == "P1" || method == "P2") && !hasMaskData && !has12Lm)
        {
            comparisonStatusMsg = $"Chưa có Mask 2D (đường bao quần / vùng ống) cho [{method}] trên phôi [{activeSample}].";
            return;
        }
        if (method == "P4" && !has12Lm && !hasP4Ref3D)
        {
            comparisonStatusMsg = $"P4 cần mẫu 3D chuẩn hoặc 12 mốc AI cho phôi [{activeSample}]. Hãy chốt mẫu chuẩn 3D hoặc bấm 'Nhận diện AI'.";
            return;
        }

        List<List<Vector2>> canonicalStrokes = GetCanonicalTemplateStrokesForMethods();
        if (canonicalPatternStrokes.Count == 0 && canonicalStrokes != null && canonicalStrokes.Count > 0)
        {
            for (int s = 0; s < canonicalStrokes.Count; s++)
            {
                if (canonicalStrokes[s] != null && canonicalStrokes[s].Count >= 2)
                    canonicalPatternStrokes.Add(new List<Vector2>(canonicalStrokes[s]));
            }
        }

        activeTrajectoryMethod = method;
        List<WaypointData> generated = null;
        float tGen = 0f;
        float regRes = 0f;

        if (method == "P1")
        {
            SyncP1ConfigFromTechParamsIfNeeded();
            generated = TrajectoryEvaluationEngine.GenerateP1Zigzag(
                this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, p1Config, out lastP1Result, out tGen);
        }
        else if (method == "P2")
        {
            SyncP2ConfigFromTechParamsIfNeeded();
            generated = TrajectoryEvaluationEngine.GenerateP2ShapeSkeleton(
                this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, p2Config, out lastP2Result, out tGen);
        }
        else if (method == "P3")
        {
            SyncP3ConfigFromTechParamsIfNeeded();
            generated = TrajectoryEvaluationEngine.GenerateP3LandmarkTemplate(
                this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, p3Config, out lastP3Result, out tGen);
        }
        else
        {
            SyncP4ConfigFromTechParamsIfNeeded();
            generated = TrajectoryEvaluationEngine.GenerateP4RegistrationTpsIcp(
                this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, p4Config, out lastP4Result, out regRes, out tGen);
            float resLimit = p4Config != null ? p4Config.residualThresholdMm : sprayTechParams.registrationResidualThresholdMm;
            if (regRes > resLimit && !forceBypassSafetyInterlock)
            {
                if (lastP4Result != null) lastP4Result.rejectedByResidualGate = true;
                ProjectWaypointsOntoActiveCameraCanvas(null, activeSample);
                comparisonStatusMsg = $"[CHẶN AN TOÀN P4] Sai số dư đăng ký = {regRes:F2} mm > ngưỡng {resLimit:F1} mm. Dừng nạp & báo kỹ sư!";
                return;
            }
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
            ProjectWaypointsOntoActiveCameraCanvas(waypoints, activeSample);
            RunDiagnostics();
            MarkSettingsDirty();

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

            string extraNote = "";
            if (method == "P1" && lastP1Result != null)
            {
                extraNote = $", e={lastP1Result.erosionEMm:F0}mm, s={lastP1Result.effectiveSpacingMm:F1}mm, {lastP1Result.passCount} lượt";
            }
            else if (method == "P2" && lastP2Result != null)
            {
                string modeTag = p2Config != null && p2Config.geometryMode == TrajectoryEvaluationEngine.P2GeometryMode.BoundaryConcentricOffset
                    ? "Offset biên"
                    : "Trục giữa";
                extraNote = $", {modeTag}, s={lastP2Result.effectiveSpacingMm:F1}mm, {lastP2Result.leftLegPassCount + lastP2Result.rightLegPassCount} lượt";
            }
            else if (method == "P3" && lastP3Result != null)
            {
                extraNote = $", {lastP3Result.landmarkMethodUsed}, lưới {lastP3Result.totalTriangles} tam giác (đảo={lastP3Result.flippedTriangleCount})";
            }
            else if (method == "P4" && lastP4Result != null)
            {
                float resLimit = p4Config != null ? p4Config.residualThresholdMm : sprayTechParams.registrationResidualThresholdMm;
                extraNote = regRes > resLimit
                    ? $" [Cảnh báo dư={regRes:F1}mm]"
                    : $", {lastP4Result.nonRigidMethodUsed}, dư={lastP4Result.residualAfterInitMm:F1}->{lastP4Result.residualAfterIcpMm:F1}->{regRes:F2} mm";
            }
            comparisonStatusMsg = $"Đã sinh [{method}]: {waypoints.Count} điểm (IK {solvedCnt}/{waypoints.Count}, {tGen * 1000f:F0} ms{extraNote}). Bấm Chạy để vận hành.";
        }
        else
        {
            if (waypoints != null) waypoints.Clear();
            ProjectWaypointsOntoActiveCameraCanvas(waypoints, activeSample);
            if (method == "P1" && lastP1Result != null && !string.IsNullOrEmpty(lastP1Result.warningMessage))
            {
                comparisonStatusMsg = lastP1Result.warningMessage;
            }
            else if (method == "P3" && lastP3Result != null && !string.IsNullOrEmpty(lastP3Result.warningMessage))
            {
                comparisonStatusMsg = lastP3Result.warningMessage;
            }
            else if (method == "P4" && lastP4Result != null && !string.IsNullOrEmpty(lastP4Result.warningMessage))
            {
                comparisonStatusMsg = lastP4Result.warningMessage;
            }
            else
            {
                comparisonStatusMsg = $"Không sinh được điểm nào cho [{method}] trên phôi [{activeSample}]. Kiểm tra lại 12 mốc AI hoặc biên trừ hao e.";
            }
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

        comparisonResults.Clear();
        List<List<Vector2>> canonicalStrokes = GetCanonicalTemplateStrokesForMethods();

        SyncP1ConfigFromTechParamsIfNeeded();
        SyncP2ConfigFromTechParamsIfNeeded();
        SyncP3ConfigFromTechParamsIfNeeded();
        SyncP4ConfigFromTechParamsIfNeeded();

        var wpP1 = TrajectoryEvaluationEngine.GenerateP1Zigzag(this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, p1Config, out lastP1Result, out float t1);
        var wpP2 = TrajectoryEvaluationEngine.GenerateP2ShapeSkeleton(this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, p2Config, out lastP2Result, out float t2);
        var wpP3 = TrajectoryEvaluationEngine.GenerateP3LandmarkTemplate(this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, p3Config, out lastP3Result, out float t3);
        var wpP4 = TrajectoryEvaluationEngine.GenerateP4RegistrationTpsIcp(this, plyLoader, canonicalStrokes, activeSprayTemplateSize, sprayTechParams, p4Config, out lastP4Result, out float res4, out float t4);

        // TUYỆT ĐỐI KHÔNG tự lấy P4 làm Ground Truth (đúng trang 9 file góp ý: chỉ dùng GT dạy tay hoặc marker vật lý ngoài!)
        bool hasValidGt = (groundTruthWaypoints != null && groundTruthWaypoints.Count >= 2
            && (string.IsNullOrEmpty(groundTruthSampleName) || groundTruthSampleName == activeSample));
        var refGt = hasValidGt ? groundTruthWaypoints : null;
        string gtLabel = hasValidGt ? groundTruthSourceLabel : "Chưa có GT";

        GameObject dummyHm = null;
        var r1 = TrajectoryEvaluationEngine.EvaluateTrajectory(
            "P1_Zigzag", wpP1, refGt, this, plyLoader, sprayTechParams, t1, 0f,
            evalEngineeringScores[0], false, ref dummyHm, gtLabel, evalSetupTimeMin[0],
            "Đơn giản, nhanh, không cần mẫu chuẩn; dễ chỉnh w, r, e nhưng không mang ý đồ hiệu ứng đậm-nhạt.");
        var r2 = TrajectoryEvaluationEngine.EvaluateTrajectory(
            "P2_Skeleton", wpP2, refGt, this, plyLoader, sprayTechParams, t2, 0f,
            evalEngineeringScores[1], false, ref dummyHm, gtLabel, evalSetupTimeMin[1],
            "Ít quay đầu, giống thợ phun dọc ống; vùng hội tụ đũng/cạp phức tạp và chưa mang ý đồ hiệu ứng.");
        var r3 = TrajectoryEvaluationEngine.EvaluateTrajectory(
            "P3_Barycentric12", wpP3, refGt, this, plyLoader, sprayTechParams, t3, 0f,
            evalEngineeringScores[2], false, ref dummyHm, gtLabel, evalSetupTimeMin[2],
            "Giữ đúng ý đồ hiệu ứng mẫu dạy, rất nhanh, dễ giải thích & kỹ sư chỉnh được; phụ thuộc độ chính xác mốc.");
        var r4 = TrajectoryEvaluationEngine.EvaluateTrajectory(
            "P4_ICP3D_CPD", wpP4, refGt, this, plyLoader, sprayTechParams, t4, res4,
            evalEngineeringScores[3], showDepositionHeatmap3D, ref depositionHeatmapObj, gtLabel, evalSetupTimeMin[3],
            "Xử lý được nếp nhăn 3D & đổi cỡ, giữ ý đồ hiệu ứng; chậm hơn P3, cần khởi tạo tốt & chỉnh λ, β của CPD.");

        comparisonResults.Add(r1);
        comparisonResults.Add(r2);
        comparisonResults.Add(r3);
        comparisonResults.Add(r4);

        lastSensitivityReport = TrajectoryEvaluationEngine.RunWeightSensitivityAnalysis(comparisonResults);

        // Bước 1: Ưu tiên chọn phương án ĐẠT ngưỡng loại có điểm Ma trận Quyết định cao nhất; nếu chưa phương án nào đạt hết thì lấy cao nhất
        TrajectoryEvaluationEngine.MethodEvaluationResult best = null;
        for (int i = 0; i < comparisonResults.Count; i++)
        {
            var cur = comparisonResults[i];
            if (!cur.passedGate) continue;
            if (best == null || cur.weightedScore > best.weightedScore) best = cur;
        }
        if (best == null)
        {
            best = r1;
            for (int i = 1; i < comparisonResults.Count; i++)
                if (comparisonResults[i].weightedScore > best.weightedScore) best = comparisonResults[i];
        }
        string gtNote = hasValidGt ? $" [GT: {gtLabel}]" : " [Chưa GT -> E_geo=N/A]";
        string gateBadge = best.passedGate ? "ĐẠT" : "CẦN KIỂM TRA";
        comparisonStatusMsg = $"Hạng 1 [{activeSample}]: {best.methodName} ({best.weightedScore:F1}/100, {gateBadge}){gtNote}";
    }

    private void DrawEvaluationAndSprayCard(float width)
    {
        GUILayout.BeginVertical(AntigravityUITheme.CardBoxStyle(), GUILayout.Width(width));
        GUILayout.Label("ĐÁNH GIÁ & PHUN GAUSS", AntigravityUITheme.StyleCardHeader);

        GUILayout.Space(4);
        GUILayout.Label("1. THÔNG SỐ SÚNG PHUN", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        GUILayout.BeginHorizontal();
        GUILayout.Label($"Khoảng cách d: {customDrawZOffsetMm:F0} mm", AntigravityUITheme.StyleLabelMuted, GUILayout.Width(170));
        float newStandoff = GUILayout.HorizontalSlider(customDrawZOffsetMm, 0f, 250f);
        if (Mathf.Abs(newStandoff - customDrawZOffsetMm) > 0.1f)
        {
            customDrawZOffsetMm = Mathf.Round(newStandoff);
            sprayTechParams.standoffMm = customDrawZOffsetMm;
        }
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
        GUILayout.Label("2. MẪU CHUẨN 3D (P4) & GROUND TRUTH THỰC NGHIỆM", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());

        string ref3DInfo = (templateReferenceCloud3DWorld != null && templateReferenceCloud3DWorld.Count >= 30)
            ? $"Mẫu 3D P4: [{templateReferenceSampleName}] ({templateReferenceCloud3DWorld.Count} điểm 3D, {templateReferenceStrokes3DWorld.Count} nét)"
            : "Mẫu 3D P4: Chưa chốt (đang dùng nhánh dự phòng 12 mốc)";
        GUILayout.Label(ref3DInfo, (templateReferenceCloud3DWorld != null && templateReferenceCloud3DWorld.Count >= 30)
            ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);
        if (GUILayout.Button("Chốt phôi & nét hiện tại làm Mẫu Chuẩn 3D (P4)", AntigravityUITheme.StyleButton, GUILayout.Height(23)))
        {
            CaptureReferenceTemplateCloud3DForP4();
            comparisonStatusMsg = (templateReferenceCloud3DWorld != null && templateReferenceCloud3DWorld.Count >= 30)
                ? $"Đã chốt Mẫu 3D chuẩn [{templateReferenceSampleName}] ({templateReferenceCloud3DWorld.Count} điểm, voxel 5mm) cho P4!"
                : "Không đủ điểm 3D hoặc chưa có nét vẽ trên phôi hiện tại.";
        }

        GUILayout.Space(3);
        bool hasGtNow = (groundTruthWaypoints != null && groundTruthWaypoints.Count >= 2);
        GUILayout.Label(hasGtNow ? $"Ground Truth: {groundTruthSourceLabel}" : "Ground Truth: Chưa có (E_geo = N/A, KHÔNG dùng P4 tự chấm)",
            hasGtNow ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Chốt vẽ tay làm GT", AntigravityUITheme.StyleButton, GUILayout.Height(23)))
        {
            SetCurrentTrajectoryAsGroundTruth();
        }
        if (GUILayout.Button("Nạp GT (.csv)", AntigravityUITheme.StyleButton, GUILayout.Width(86), GUILayout.Height(23)))
        {
            ImportGroundTruthCsvWithDialog();
        }
        if (hasGtNow && GUILayout.Button("Xóa", AntigravityUITheme.StyleButton, GUILayout.Width(42), GUILayout.Height(23)))
        {
            ClearGroundTruth();
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(4);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Chạy so sánh P1/P2/P3/P4", AntigravityUITheme.StylePrimaryButton, GUILayout.Height(28)))
        {
            RunAllMethodsComparison();
        }
        if (GUILayout.Button("Xuất CSV", AntigravityUITheme.StyleButton, GUILayout.Width(84), GUILayout.Height(28)))
        {
            if (comparisonResults.Count == 0) RunAllMethodsComparison();
            string exportDir = Path.Combine(Application.dataPath, "..", "output_drawing");
            string csvPath = TrajectoryEvaluationEngine.ExportComparisonCsv(comparisonResults, exportDir);
            comparisonStatusMsg = $"Đã xuất: {Path.GetFileName(csvPath)}";
        }
        GUILayout.EndHorizontal();

        if (GUILayout.Button("Phóng to Bảng Đánh Giá (Đủ 12 chỉ tiêu + Ma trận QĐ + Độ nhạy)", AntigravityUITheme.StyleButton, GUILayout.Height(25)))
        {
            if (comparisonResults.Count == 0) RunAllMethodsComparison();
            showFullscreen2DCanvas = false;
            showFullscreenEvaluationReport = true;
        }

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
                GUILayout.Label($"{r.weightedScore:F1}/100 [{(r.passedGate ? "ĐẠT" : "LOẠI")}]",
                    r.passedGate ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);
                GUILayout.EndHorizontal();

                if (!r.passedGate && !string.IsNullOrEmpty(r.gateFailReason))
                {
                    GUILayout.Label($"Ngưỡng loại (B1): {r.gateFailReason}", AntigravityUITheme.StyleLabelYellow);
                }

                string geoStr = r.hasGroundTruth
                    ? $"E_geo={r.eGeoMeanMm:F2}/P95={r.eGeoP95Mm:F1}/H={r.eGeoHausdorffMm:F1}mm"
                    : "E_geo=N/A (chưa GT)";
                GUILayout.Label($"{geoStr} | E_d={r.eDMeanMm:F2}(max {r.eDMaxMm:F1})mm | E_θ={r.eThetaMeanDeg:F1}(max {r.eThetaMaxDeg:F1})°", AntigravityUITheme.StyleLabelMuted);
                GUILayout.Label($"E_T={r.eTPercent:F1}% | Phủ C={r.coveragePercent:F1}% | Tràn O={r.oversprayPercent:F1}% | Hàm J={r.objectiveJ:F3}", AntigravityUITheme.StyleLabelMuted);
                string extraP4 = r.methodName.StartsWith("P4") ? $" | Dư ĐK={r.regResidualMm:F2}mm" : "";
                GUILayout.Label($"F={r.feasibilityPct:F0}% (VC={r.collisionCount}, KD={r.singularityCount}) | V_cv={r.vCvPercent:F1}% | t_gen={r.tGenSec:F2}s | t_cyc={r.tCycleSec:F1}s | R_drop={r.rDropMm:F2}mm | t_set={r.tSetupMin:F1}p{extraP4}", AntigravityUITheme.StyleLabelMuted);
                GUILayout.Label($"Điểm B2: Phun(30%)={r.scoreSprayQuality:F0} | HH(20%)={r.scoreGeoAcc:F0} | Bền(15%)={r.scoreRobust:F0} | KT(15%)={r.scoreFeasCycle:F0} | Tốc(10%)={r.scoreSpeedSetup:F0} | Công(10%)={r.scoreEng:F0} ({r.engineeringScore:F1}/5)", AntigravityUITheme.StyleLabelCyan);

                string tag = r.methodName.StartsWith("P1") ? "P1" : (r.methodName.StartsWith("P2") ? "P2" : (r.methodName.StartsWith("P3") ? "P3" : "P4"));
                GUIStyle loadBtnStyle = (activeTrajectoryMethod == tag) ? AntigravityUITheme.StylePrimaryButton : AntigravityUITheme.StyleButton;
                if (GUILayout.Button($"Nạp {tag}", loadBtnStyle, GUILayout.Height(22)))
                {
                    showP1ParameterPanel = (tag == "P1");
                    showP2ParameterPanel = (tag == "P2");
                    showP3ParameterPanel = (tag == "P3");
                    showP4ParameterPanel = (tag == "P4");
                    if (tag == "P1") SyncP1ConfigFromTechParamsIfNeeded();
                    else if (tag == "P2") SyncP2ConfigFromTechParamsIfNeeded();
                    else if (tag == "P3") SyncP3ConfigFromTechParamsIfNeeded();
                    else if (tag == "P4") SyncP4ConfigFromTechParamsIfNeeded();
                    GenerateAndApplyMethodTrajectory(tag);
                }
                GUILayout.EndVertical();
            }

            if (lastSensitivityReport != null && !string.IsNullOrEmpty(lastSensitivityReport.summaryText))
            {
                GUILayout.Space(3);
                GUILayout.Label("Độ nhạy trọng số (±10%): " + lastSensitivityReport.summaryText,
                    lastSensitivityReport.isStable ? AntigravityUITheme.StyleLabelGreen : AntigravityUITheme.StyleLabelYellow);
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

    private void DrawEvalTableRow7(
        string c0, string c1, string c2,
        string m0, string m1, string m2, string m3,
        float w0, float w1, float w2, float wM,
        GUIStyle s0, GUIStyle s1, GUIStyle s2,
        GUIStyle sm0, GUIStyle sm1, GUIStyle sm2, GUIStyle sm3)
    {
        GUILayout.BeginHorizontal(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label(c0, s0, GUILayout.Width(w0));
        GUILayout.Label(c1, s1, GUILayout.Width(w1));
        GUILayout.Label(c2, s2, GUILayout.Width(w2));
        GUILayout.Label(m0, sm0, GUILayout.Width(wM));
        GUILayout.Label(m1, sm1, GUILayout.Width(wM));
        GUILayout.Label(m2, sm2, GUILayout.Width(wM));
        GUILayout.Label(m3, sm3, GUILayout.Width(wM));
        GUILayout.EndHorizontal();
    }

    private void DrawFullscreenEvaluationReportOverlay(float topSafeY)
    {
        Event ev = Event.current;
        if (ev != null && ev.type == EventType.KeyDown && ev.keyCode == KeyCode.Escape)
        {
            showFullscreenEvaluationReport = false;
            ev.Use();
            return;
        }

        Rect fullRect = new Rect(0f, topSafeY, Screen.width, Mathf.Max(260f, Screen.height - topSafeY));
        DraggableUIManager.RegisterBlockingRect(fullRect);
        DrawGuiSolidRect(fullRect, new Color(0.05f, 0.07f, 0.10f, 0.98f));

        // 1. Thanh tiêu đề trên cùng
        float topBarH = 44f;
        Rect topBarRect = new Rect(fullRect.x + 12f, fullRect.y + 8f, fullRect.width - 24f, topBarH);
        GUI.DrawTexture(topBarRect, AntigravityUITheme.TexBgCard);
        GUI.Label(new Rect(topBarRect.x + 14f, topBarRect.y + 4f, 680f, 20f),
            "BẢNG ĐÁNH GIÁ ĐẦY ĐỦ 12 CHỈ TIÊU & MA TRẬN QUYẾT ĐỊNH P1 / P2 / P3 / P4 (THEO FILE GÓP Ý)",
            AntigravityUITheme.StyleCardHeader);
        string activeSample = GetActiveCloudSampleName();
        if (string.IsNullOrEmpty(activeSample)) activeSample = selectedSampleName;
        bool hasGt = (groundTruthWaypoints != null && groundTruthWaypoints.Count >= 2);
        GUI.Label(new Rect(topBarRect.x + 14f, topBarRect.y + 23f, 780f, 16f),
            $"Phôi: [{activeSample}] | Ground Truth: {(hasGt ? groundTruthSourceLabel : "Chưa có (E_geo = N/A — KHÔNG lấy P4 tự chấm)")} | d={customDrawZOffsetMm:F0}mm, w={sprayTechParams.sprayWidthMm:F0}mm, r={sprayTechParams.overlapRatio * 100f:F0}%, σ={sprayTechParams.gaussSigmaMm:F1}mm",
            AntigravityUITheme.StyleLabelCyan);

        float btnX = topBarRect.xMax - 10f;
        btnX -= 110f;
        if (GUI.Button(new Rect(btnX, topBarRect.y + 8f, 110f, 28f), "Đóng (ESC)", AntigravityUITheme.StyleButton))
        {
            showFullscreenEvaluationReport = false;
            return;
        }
        btnX -= 132f;
        if (GUI.Button(new Rect(btnX, topBarRect.y + 8f, 124f, 28f), "Xuất CSV đầy đủ", AntigravityUITheme.StyleButton))
        {
            if (comparisonResults.Count == 0) RunAllMethodsComparison();
            string exportDir = Path.Combine(Application.dataPath, "..", "output_drawing");
            string csvPath = TrajectoryEvaluationEngine.ExportComparisonCsv(comparisonResults, exportDir);
            comparisonStatusMsg = $"Đã xuất: {Path.GetFileName(csvPath)}";
        }
        btnX -= 154f;
        if (GUI.Button(new Rect(btnX, topBarRect.y + 8f, 146f, 28f), "Chạy so sánh lại", AntigravityUITheme.StylePrimaryButton))
        {
            RunAllMethodsComparison();
        }

        // 2. Vùng nội dung bảng cuộn
        Rect bodyRect = new Rect(fullRect.x + 12f, topBarRect.yMax + 8f, fullRect.width - 24f, Mathf.Max(160f, fullRect.yMax - topBarRect.yMax - 18f));
        GUILayout.BeginArea(bodyRect);
        fullscreenEvalScroll = GUILayout.BeginScrollView(fullscreenEvalScroll);

        if (comparisonResults == null || comparisonResults.Count < 4)
        {
            GUILayout.Label("Chưa có dữ liệu so sánh. Nhấn 'Chạy so sánh lại' để đo toàn bộ P1 / P2 / P3 / P4 trên cùng bộ tham số.", AntigravityUITheme.StyleLabelYellow);
            GUILayout.EndScrollView();
            GUILayout.EndArea();
            return;
        }

        var r1 = comparisonResults[0];
        var r2 = comparisonResults[1];
        var r3 = comparisonResults[2];
        var r4 = comparisonResults[3];

        float usableW = Mathf.Max(900f, bodyRect.width - 36f);
        float w0 = 86f;
        float w1 = Mathf.Clamp(usableW * 0.17f, 155f, 210f);
        float w2 = Mathf.Clamp(usableW * 0.22f, 190f, 280f);
        float wM = Mathf.Max(115f, (usableW - w0 - w1 - w2 - 42f) / 4f);

        GUIStyle hdr = AntigravityUITheme.StyleLabelCyan;
        GUIStyle lbl = new GUIStyle(AntigravityUITheme.StyleLabelMuted);
        lbl.normal.textColor = new Color(0.92f, 0.95f, 1.0f, 1f);
        GUIStyle mut = AntigravityUITheme.StyleLabelMuted;
        GUIStyle grn = AntigravityUITheme.StyleLabelGreen;
        GUIStyle yel = AntigravityUITheme.StyleLabelYellow;

        // =====================================================================
        // BẢNG 1: 12 CHỈ TIÊU ĐO CHUẨN (TRANG 10–11) + HÀM MỤC TIÊU J (TRANG 7) + NGƯỠNG LOẠI BƯỚC 1
        // =====================================================================
        GUILayout.Label("BẢNG 1. SỐ ĐO THÔ ĐẦY ĐỦ 12 CHỈ TIÊU ĐO (TRANG 10–11), HÀM MỤC TIÊU J (TRANG 7) & NGƯỠNG LOẠI BƯỚC 1", AntigravityUITheme.StyleSectionLabel);
        DrawEvalTableRow7(
            "Ký hiệu", "Chỉ tiêu đo", "Cách tính / Tầng kiểm thử",
            $"P1 Zigzag ({r1.waypointCount} wp)", $"P2 Trục giữa ({r2.waypointCount} wp)",
            $"P3 12 Mốc ({r3.waypointCount} wp)", $"P4 ICP/CPD ({r4.waypointCount} wp)",
            w0, w1, w2, wM, hdr, hdr, hdr, hdr, hdr, hdr, hdr);

        string FmtGeo(TrajectoryEvaluationEngine.MethodEvaluationResult r) =>
            r.hasGroundTruth ? $"TB {r.eGeoMeanMm:F2} | P95 {r.eGeoP95Mm:F1} | H {r.eGeoHausdorffMm:F1}" : "N/A (chưa GT)";

        DrawEvalTableRow7(
            "1. E_geo", "Sai số hình học (mm)", "TB / P95 / Hausdorff so với GT [Tầng 1, 3]",
            FmtGeo(r1), FmtGeo(r2), FmtGeo(r3), FmtGeo(r4),
            w0, w1, w2, wM, hdr, lbl, mut,
            r1.gateGeoOk ? grn : yel, r2.gateGeoOk ? grn : yel, r3.gateGeoOk ? grn : yel, r4.gateGeoOk ? grn : yel);

        DrawEvalTableRow7(
            "2. E_d", "Sai số khoảng cách súng (mm)", "|d_thực(TCP->bề mặt) - d| (TB / Max) [Tầng 1, 3]",
            $"{r1.eDMeanMm:F2} (max {r1.eDMaxMm:F1})", $"{r2.eDMeanMm:F2} (max {r2.eDMaxMm:F1})",
            $"{r3.eDMeanMm:F2} (max {r3.eDMaxMm:F1})", $"{r4.eDMeanMm:F2} (max {r4.eDMaxMm:F1})",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "3. E_θ", "Sai số góc súng (°)", "Góc trục súng vs pháp tuyến (TB / Max) [Tầng 1]",
            $"{r1.eThetaMeanDeg:F1}° (max {r1.eThetaMaxDeg:F1}°)", $"{r2.eThetaMeanDeg:F1}° (max {r2.eThetaMaxDeg:F1}°)",
            $"{r3.eThetaMeanDeg:F1}° (max {r3.eThetaMaxDeg:F1}°)", $"{r4.eThetaMeanDeg:F1}° (max {r4.eThetaMaxDeg:F1}°)",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "4. E_T", "Sai lệch mật độ phun (%)", "RMS(T - T*) / mean(T*) trong R [Tầng 1, 4]",
            $"{r1.eTPercent:F1}%", $"{r2.eTPercent:F1}%", $"{r3.eTPercent:F1}%", $"{r4.eTPercent:F1}%",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "5. C", "Độ phủ vùng phun (%)", "Diện tích R có T >= 50% T* [Tầng 1, 4]",
            $"{r1.coveragePercent:F1}%", $"{r2.coveragePercent:F1}%", $"{r3.coveragePercent:F1}%", $"{r4.coveragePercent:F1}%",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "6. O", "Tràn ra ngoài (%)", "Lượng phun rơi ngoài R / tổng phun [Tầng 1, 4]",
            $"{r1.oversprayPercent:F1}%", $"{r2.oversprayPercent:F1}%", $"{r3.oversprayPercent:F1}%", $"{r4.oversprayPercent:F1}%",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "7. F", "Khả thi (%) / Va chạm / Kỳ dị", "IK + khớp + 0 va chạm + góc <= θ_max [Tầng 2]",
            $"{r1.feasibilityPct:F1}% (VC:{r1.collisionCount} KD:{r1.singularityCount})",
            $"{r2.feasibilityPct:F1}% (VC:{r2.collisionCount} KD:{r2.singularityCount})",
            $"{r3.feasibilityPct:F1}% (VC:{r3.collisionCount} KD:{r3.singularityCount})",
            $"{r4.feasibilityPct:F1}% (VC:{r4.collisionCount} KD:{r4.singularityCount})",
            w0, w1, w2, wM, hdr, lbl, mut,
            (r1.gateFeasibilityOk && r1.gateCollisionOk) ? grn : yel,
            (r2.gateFeasibilityOk && r2.gateCollisionOk) ? grn : yel,
            (r3.gateFeasibilityOk && r3.gateCollisionOk) ? grn : yel,
            (r4.gateFeasibilityOk && r4.gateCollisionOk) ? grn : yel);

        DrawEvalTableRow7(
            "8. V_cv", "Độ đều tốc độ TCP (%)", "Hệ số biến thiên tốc độ trong đoạn phun [Tầng 2]",
            $"{r1.vCvPercent:F2}%", $"{r2.vCvPercent:F2}%", $"{r3.vCvPercent:F2}%", $"{r4.vCvPercent:F2}%",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "9. t_gen", "Thời gian sinh (s)", "Từ lúc có ảnh đến xuất lệnh (<= 60s) [Tầng 1]",
            $"{r1.tGenSec:F3} s", $"{r2.tGenSec:F3} s", $"{r3.tGenSec:F3} s", $"{r4.tGenSec:F3} s",
            w0, w1, w2, wM, hdr, lbl, mut,
            r1.gateTimeOk ? grn : yel, r2.gateTimeOk ? grn : yel, r3.gateTimeOk ? grn : yel, r4.gateTimeOk ? grn : yel);

        DrawEvalTableRow7(
            "10. t_cycle", "Thời gian chu kỳ (s)", "Thời gian robot chạy hết quỹ đạo [Tầng 2, 3]",
            $"{r1.tCycleSec:F2} s", $"{r2.tCycleSec:F2} s", $"{r3.tCycleSec:F2} s", $"{r4.tCycleSec:F2} s",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "11. R_drop", "Độ bền vững (mm)", "Mức tăng sai số khi lệch tư thế/nếp [Tầng 1]",
            $"{r1.rDropMm:F2} mm", $"{r2.rDropMm:F2} mm", $"{r3.rDropMm:F2} mm", $"{r4.rDropMm:F2} mm",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "12. t_setup", "Thiết lập mã/cỡ mới (phút)", "Bấm giờ từ có quần mới -> chạy (vs dạy tay) [T3]",
            $"{r1.tSetupMin:F1} phút", $"{r2.tSetupMin:F1} phút", $"{r3.tSetupMin:F1} phút", $"{r4.tSetupMin:F1} phút",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "Hàm J", "Hàm mục tiêu chung (Tr.7)", "J = MSE(T-T*) + λ_o·O + λ_t·t_cycle (nhỏ tốt)",
            $"{r1.objectiveJ:F4}", $"{r2.objectiveJ:F4}", $"{r3.objectiveJ:F4}", $"{r4.objectiveJ:F4}",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        DrawEvalTableRow7(
            "Dư ĐK 3D", "Sai số dư đăng ký P4 (mm)", "Khoảng cách RMS sau ICP+CPD/TPS (ngưỡng an toàn)",
            "—", "—", "—", $"{r4.regResidualMm:F2} mm",
            w0, w1, w2, wM, hdr, lbl, mut, mut, mut, mut,
            r4.regResidualMm <= (p4Config != null ? p4Config.residualThresholdMm : 12f) ? grn : yel);

        DrawEvalTableRow7(
            "BƯỚC 1", "NGƯỠNG LOẠI BẮT BUỘC", "F>=95%, 0 va chạm, E_geo<=5/P95<=8, t_gen<=60s",
            r1.passedGate ? "ĐẠT" : $"LOẠI ({r1.gateFailReason})",
            r2.passedGate ? "ĐẠT" : $"LOẠI ({r2.gateFailReason})",
            r3.passedGate ? "ĐẠT" : $"LOẠI ({r3.gateFailReason})",
            r4.passedGate ? "ĐẠT" : $"LOẠI ({r4.gateFailReason})",
            w0, w1, w2, wM, hdr, hdr, mut,
            r1.passedGate ? grn : yel, r2.passedGate ? grn : yel, r3.passedGate ? grn : yel, r4.passedGate ? grn : yel);

        // =====================================================================
        // BẢNG 2: MA TRẬN QUYẾT ĐỊNH BƯỚC 2 (6 NHÓM TIÊU CHÍ × TRỌNG SỐ — TRANG 11–12)
        // =====================================================================
        GUILayout.Space(10);
        GUILayout.Label("BẢNG 2. MA TRẬN QUYẾT ĐỊNH BƯỚC 2 — CHẤM ĐIỂM CÓ TRỌNG SỐ S_k = 100·clip((x_worst-x)/(x_worst-x_target), 0, 1) (TRANG 11–12)", AntigravityUITheme.StyleSectionLabel);
        DrawEvalTableRow7(
            "Trọng số", "Nhóm tiêu chí", "Chỉ tiêu dùng",
            "P1 (0–100)", "P2 (0–100)", "P3 (0–100)", "P4 (0–100)",
            w0, w1, w2, wM, hdr, hdr, hdr, hdr, hdr, hdr, hdr);

        DrawEvalTableRow7("30%", "1. Chất lượng phun", "E_T, C, O",
            $"{r1.scoreSprayQuality:F1}", $"{r2.scoreSprayQuality:F1}", $"{r3.scoreSprayQuality:F1}", $"{r4.scoreSprayQuality:F1}",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("20%", "2. Độ chính xác hình học", "E_geo, E_d, E_θ",
            $"{r1.scoreGeoAcc:F1}", $"{r2.scoreGeoAcc:F1}", $"{r3.scoreGeoAcc:F1}", $"{r4.scoreGeoAcc:F1}",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("15%", "3. Bền vững nếp/tư thế/cỡ", "R_drop, E_geo theo nhóm",
            $"{r1.scoreRobust:F1}", $"{r2.scoreRobust:F1}", $"{r3.scoreRobust:F1}", $"{r4.scoreRobust:F1}",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("15%", "4. Khả thi và chu kỳ", "F, V_cv, t_cycle",
            $"{r1.scoreFeasCycle:F1}", $"{r2.scoreFeasCycle:F1}", $"{r3.scoreFeasCycle:F1}", $"{r4.scoreFeasCycle:F1}",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("10%", "5. Tốc độ sinh & thiết lập", "t_gen (s), t_setup (phút)",
            $"{r1.scoreSpeedSetup:F1}", $"{r2.scoreSpeedSetup:F1}", $"{r3.scoreSpeedSetup:F1}", $"{r4.scoreSpeedSetup:F1}",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("10%", "6. Công cài đặt & chỉnh tay", "Đánh giá 1–5 có lý do",
            $"{r1.scoreEng:F1} ({r1.engineeringScore:F1}/5)", $"{r2.scoreEng:F1} ({r2.engineeringScore:F1}/5)",
            $"{r3.scoreEng:F1} ({r3.engineeringScore:F1}/5)", $"{r4.scoreEng:F1} ({r4.engineeringScore:F1}/5)",
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);

        // Xác định hạng 1..4 (ưu tiên đạt Bước 1)
        int[] ranks = new int[4];
        for (int i = 0; i < 4; i++)
        {
            int rank = 1;
            var ri = comparisonResults[i];
            for (int j = 0; j < 4; j++)
            {
                if (i == j) continue;
                var rj = comparisonResults[j];
                bool jBetter = (rj.passedGate && !ri.passedGate) ||
                               (rj.passedGate == ri.passedGate && rj.weightedScore > ri.weightedScore);
                if (jBetter) rank++;
            }
            ranks[i] = rank;
        }
        DrawEvalTableRow7(
            "100%", "TỔNG ĐIỂM & XẾP HẠNG", "Điểm = Σ w_k · S_k (ưu tiên Đạt Bước 1)",
            $"{r1.weightedScore:F1}/100 [Hạng #{ranks[0]}]",
            $"{r2.weightedScore:F1}/100 [Hạng #{ranks[1]}]",
            $"{r3.weightedScore:F1}/100 [Hạng #{ranks[2]}]",
            $"{r4.weightedScore:F1}/100 [Hạng #{ranks[3]}]",
            w0, w1, w2, wM, hdr, hdr, mut,
            ranks[0] == 1 ? grn : hdr, ranks[1] == 1 ? grn : hdr, ranks[2] == 1 ? grn : hdr, ranks[3] == 1 ? grn : hdr);

        // =====================================================================
        // MỤC 3: KIỂM TRA ĐỘ NHẠY TRỌNG SỐ (±10 ĐIỂM PHẦN TRĂM — TRANG 12)
        // =====================================================================
        GUILayout.Space(10);
        GUILayout.Label("MỤC 3. KIỂM TRA ĐỘ NHẠY TRỌNG SỐ (±10 ĐIỂM PHẦN TRĂM — TRANG 12 FILE GÓP Ý)", AntigravityUITheme.StyleSectionLabel);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        if (lastSensitivityReport == null) lastSensitivityReport = TrajectoryEvaluationEngine.RunWeightSensitivityAnalysis(comparisonResults);
        GUILayout.Label(lastSensitivityReport.summaryText, lastSensitivityReport.isStable ? grn : yel);
        GUILayout.Space(3);
        for (int i = 0; i < lastSensitivityReport.scenarioLines.Count; i += 2)
        {
            GUILayout.BeginHorizontal();
            string leftTxt = lastSensitivityReport.scenarioLines[i];
            GUILayout.Label("• " + leftTxt, leftTxt.Contains("[ĐỔI HẠNG]") ? yel : mut, GUILayout.Width(usableW * 0.49f));
            if (i + 1 < lastSensitivityReport.scenarioLines.Count)
            {
                string rightTxt = lastSensitivityReport.scenarioLines[i + 1];
                GUILayout.Label("• " + rightTxt, rightTxt.Contains("[ĐỔI HẠNG]") ? yel : mut, GUILayout.Width(usableW * 0.49f));
            }
            GUILayout.EndHorizontal();
        }
        GUILayout.EndVertical();

        // =====================================================================
        // BẢNG 4: SO SÁNH ĐỊNH TÍNH (TRANG 8) & NHẬP SỐ ĐO t_setup / ĐIỂM CÔNG 1–5
        // =====================================================================
        GUILayout.Space(10);
        GUILayout.Label("BẢNG 4. SO SÁNH ĐỊNH TÍNH (TRANG 8) & CẬP NHẬT SỐ ĐO t_setup (PHÚT) / ĐIỂM CÔNG CÀI ĐẶT (1–5 CÓ LÝ DO)", AntigravityUITheme.StyleSectionLabel);
        DrawEvalTableRow7(
            "Thuộc tính", "Tiêu chí định tính", "Ý nghĩa khi đưa vào báo cáo",
            "P1 Zigzag", "P2 Trục giữa", "P3 12 Mốc", "P4 ICP/CPD",
            w0, w1, w2, wM, hdr, hdr, hdr, hdr, hdr, hdr, hdr);
        DrawEvalTableRow7("Đầu vào", "Đầu vào tối thiểu", "Dữ liệu cần thiết để chạy",
            r1.minInputDesc, r2.minInputDesc, r3.minInputDesc, r4.minInputDesc,
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("Mặt cong", "Khả năng trên bề mặt cong", "Phôi trên khuôn chân 3D / nếp lớn",
            r1.curvedSurfaceRating, r2.curvedSurfaceRating, r3.curvedSurfaceRating, r4.curvedSurfaceRating,
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("Đổi cỡ", "Thích nghi đổi cỡ / vị trí", "Cỡ S/M/L, xoay góc, lệch bàn",
            r1.sizePoseAdaptRating, r2.sizePoseAdaptRating, r3.sizePoseAdaptRating, r4.sizePoseAdaptRating,
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("Hiệu ứng", "Giữ ý đồ hiệu ứng đậm-nhạt", "Râu mèo, mài đùi không đều T*(x)",
            r1.effectIntentRating, r2.effectIntentRating, r3.effectIntentRating, r4.effectIntentRating,
            w0, w1, w2, wM, hdr, lbl, mut, lbl, lbl, lbl, lbl);
        DrawEvalTableRow7("Lý do 1–5", "Công cài đặt, bảo trì, chỉnh tay", "Giải thích điểm Nhóm 6 (10%)",
            r1.engineeringReason, r2.engineeringReason, r3.engineeringReason, r4.engineeringReason,
            w0, w1, w2, wM, hdr, lbl, mut, mut, mut, mut, mut);

        GUILayout.Space(6);
        GUILayout.BeginVertical(AntigravityUITheme.InnerBoxStyle());
        GUILayout.Label("Chỉnh số đo bấm giờ t_setup (phút) & Điểm công cài đặt (1–5) từ thực nghiệm rồi bấm 'Chạy so sánh lại':", AntigravityUITheme.StyleLabelCyan);
        string[] mTags = { "P1 Zigzag", "P2 Trục giữa", "P3 12 Mốc", "P4 ICP/CPD" };
        for (int i = 0; i < 4; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(mTags[i], hdr, GUILayout.Width(100f));
            GUILayout.Label($"t_setup: {evalSetupTimeMin[i]:F1} phút", lbl, GUILayout.Width(115f));
            evalSetupTimeMin[i] = Mathf.Round(GUILayout.HorizontalSlider(evalSetupTimeMin[i], 0.1f, 20f, GUILayout.Width(160f)) * 10f) / 10f;
            GUILayout.Space(16f);
            GUILayout.Label($"Công cài đặt: {evalEngineeringScores[i]:F1}/5", lbl, GUILayout.Width(135f));
            evalEngineeringScores[i] = Mathf.Round(GUILayout.HorizontalSlider(evalEngineeringScores[i], 1.0f, 5.0f, GUILayout.Width(140f)) * 10f) / 10f;
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
        }
        GUILayout.EndVertical();

        GUILayout.EndScrollView();
        GUILayout.EndArea();
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
