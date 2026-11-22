using System;
using System.IO;
using System.Text;
using System.Globalization;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// BỘ CÔNG CỤ ĐÁNH GIÁ, SO SÁNH THUẬT TOÁN (P1 / P3 / P4), MÔ PHỎNG VỆT PHUN GAUSS 3D,
/// VÙNG CẤM PHUN, KHUÔN ĐỨNG 3D & XUẤT LỆNH DOBOT / ROS 2
/// Đáp ứng trực tiếp các yêu cầu còn thiếu trong `Gop-y-Bao-cao-lan-1-Tu-dong-sinh-chuong-trinh-phun.pdf`.
/// </summary>
public static class TrajectoryEvaluationEngine
{
    [Serializable]
    public class SprayTechParams
    {
        public float standoffMm = 150f;         // Khoảng cách súng d (mm)
        public float sprayWidthMm = 80f;        // Bề rộng vệt w tại khoảng cách d (mm)
        public float overlapRatio = 0.40f;      // Tỷ lệ chồng lấn r (0..0.8) -> s = w*(1-r)
        public float gaussSigmaMm = 22f;        // Độ lệch chuẩn σ của vệt Gauss đo từ phun thử lên giấy (mm)
        public float flowRateQ = 100f;          // Lưu lượng Q danh nghĩa
        public float targetSpeedMmPerSec = 220f;// Tốc độ TCP mục tiêu v (mm/s)
        public float maxTiltDeg = 28f;          // Góc nghiêng súng tối đa θ_max (°)
        public float borderMarginMm = 18f;      // Biên trừ hao e (mm) chống tràn mép vải
        public float registrationResidualThresholdMm = 12f; // Ngưỡng chặn sai số dư đăng ký P4
    }

    public enum P1RegionSourceMode
    {
        FromTemplateStrokes = 0, // Vùng R từ mẫu vẽ / mask hiện tại
        JeansLegZones = 1,       // Vùng 2 ống quần AI (đùi - gối)
        TestCircle = 2,          // Hình tròn kiểm chứng (bán kính R0, co e -> R0 - e)
        TestRectangle = 3        // Hình chữ nhật kiểm chứng (W x L)
    }

    public enum P1ScanDirectionMode
    {
        PcaMajorAxis = 0,        // Trục dài PCA tự động
        MinAreaBoundingBox = 1,  // Hình chữ nhật bao nhỏ nhất (Min-Area Rect)
        CustomAngleDeg = 2       // Góc quét tùy chỉnh (0..180°)
    }

    public enum P1ZHeightMode
    {
        Surface25DVertical = 0,     // 2.5D từ Point Cloud + súng thẳng đứng (Baseline)
        Surface25DSmoothNormal = 1, // 2.5D từ Point Cloud + pháp tuyến làm mượt
        Flat2DPlane = 2             // 2D mặt phẳng bàn + súng thẳng đứng
    }

    public enum P1ZigzagStyle
    {
        PrinterRasterFullSweep = 0, // Kiểu máy in 3D: đi zigzag song song qua lại toàn khung, vào chỗ cần phun mới bật súng
        TrimmedToSafeRegion = 1     // Cắt gọn sát biên R_safe (chỉ di chuyển trong vùng phun + đoạn nối quay đầu)
    }

    [Serializable]
    public class P1ZigzagConfig
    {
        public P1RegionSourceMode regionSource = P1RegionSourceMode.FromTemplateStrokes;
        public P1ZigzagStyle zigzagStyle = P1ZigzagStyle.PrinterRasterFullSweep;
        public float erosionEMm = 8f;                   // e – khoảng co biên vào trong (mm)
        public bool autoSpacingFromWidthOverlap = true; // true: s = w*(1-r), false: nhập tay s
        public float scanSpacingSMm = 18f;              // s – bước lượt quét song song (mm)
        public float sprayDistanceDMm = 20f;            // d – khoảng cách súng/TCP đến bề mặt (mm)
        public float sprayWidthWMm = 40f;               // w – bề rộng vệt phun tại khoảng cách d (mm)
        public float overlapRatioR = 0.50f;             // r – tỷ lệ chồng lấn (0..0.85)
        public float douglasPeuckerEpsMm = 4.0f;        // ngưỡng đơn giản hóa biên Douglas-Peucker (mm)
        public float turnaroundOverscanMm = 16f;        // lề quay đầu bên ngoài vùng phun (mm, tắt súng)
        public float openStrokeBrushRadiusMm = 20f;     // bề rộng nở nét khi người dùng vẽ nét hở (mm)
        public P1ScanDirectionMode scanDirection = P1ScanDirectionMode.PcaMajorAxis;
        public float customScanAngleDeg = 0f;           // góc quét tùy chỉnh (độ)
        public bool useBoustrophedonSplit = true;       // phân tách vùng lõm thành ô lồi (Boustrophedon)
        public P1ZHeightMode zHeightMode = P1ZHeightMode.Flat2DPlane; // Theo tài liệu: P1 mặc định 2D phẳng + súng thẳng đứng (không tính pháp tuyến lân cận nhỏ)
        public float testCircleRadiusMm = 55f;          // R0 khi kiểm chứng hình tròn (mm)
        public float testRectWidthMm = 110f;            // Chiều rộng HCN kiểm chứng (mm)
        public float testRectLengthMm = 260f;           // Chiều dài HCN kiểm chứng (mm)
        public bool testOnBothLegs = true;              // Kiểm chứng trên cả 2 ống hay 1 ống trái

        public float EffectiveSpacingSMm => autoSpacingFromWidthOverlap
            ? Mathf.Clamp(sprayWidthWMm * (1f - Mathf.Clamp(overlapRatioR, 0.05f, 0.85f)), 4f, 120f)
            : Mathf.Clamp(scanSpacingSMm, 4f, 120f);
    }

    public class P1ExecutionResult
    {
        public List<List<Vector3>> rawTargetRegionPolygonsWorld = new List<List<Vector3>>();  // Chỗ cần phun R (gốc)
        public List<List<Vector3>> safeErodedRegionPolygonsWorld = new List<List<Vector3>>(); // Vùng an toàn R_safe (sau co e)
        public List<List<Vector3>> sprayOnSegmentsWorld = new List<List<Vector3>>();          // Các đoạn quỹ đạo BẬT súng
        public List<List<Vector3>> sprayOffConnectorsWorld = new List<List<Vector3>>();       // Các đoạn nối/quay đầu TẮT súng
        public float areaRawCm2;
        public float areaSafeCm2;
        public int cellCount;
        public int passCount;
        public int sprayOnSegmentCount;
        public int sprayOffConnectorCount;
        public float scanAngleDegUsed;
        public float effectiveSpacingMm;
        public float erosionEMm;
        public string warningMessage = "";
    }

    public enum P2GeometryMode
    {
        SkeletonParallel = 0,        // Cách A: Song song trục giữa (Skeleton / Medial Axis)
        BoundaryConcentricOffset = 1 // Cách B: Offset đồng tâm lùi dần từ biên mask vào trong
    }

    public enum P2RegionScopeMode
    {
        FromTemplateRegion = 0,      // Theo vùng chỗ cần phun đã chọn/vẽ trên từng ống
        FullLegMask = 1              // Toàn bộ mask 2 ống quần (tự tách cạp & đáy quần)
    }

    public enum P2PassOrderingMode
    {
        MinimumTravelGreedy = 0,     // Tối ưu giảm quãng đường di chuyển không phun (order_for_minimum_travel)
        CenterOutAlternating = 1,    // Từ trục giữa lần lượt ra hai bên
        OuterToInner = 2             // Từ biên ngoài thu dần vào trục giữa
    }

    public enum P2GunOrientationMode
    {
        SmoothSurfaceNormal25D = 0,  // 2.5D/3D Pháp tuyến bề mặt đã làm mượt
        Vertical25D = 1,             // 2.5D Hướng súng thẳng đứng cố định
        FlatPlane2D = 2              // 2D Mặt phẳng chuẩn + súng thẳng đứng
    }

    [Serializable]
    public class P2SkeletonConfig
    {
        public P2GeometryMode geometryMode = P2GeometryMode.SkeletonParallel;
        public P2RegionScopeMode regionScope = P2RegionScopeMode.FromTemplateRegion;
        public bool continuousScanLike3DPrinter = true; // true: quét qua lại liên tục kiểu máy in 3D, chỉ BẬT súng khi đi qua chỗ cần phun
        public float openStrokeSprayWidthMm = 26f;      // bề rộng vùng cần phun (mm) cho các nét vẽ hở (râu mèo, đường cong)
        public bool autoSpacingFromWidthOverlap = true; // true: s = w * (1 - r)
        public float sprayWidthWMm = 45f;               // w – bề rộng vệt phun (mm)
        public float overlapRatioR = 0.35f;             // r – tỷ lệ chồng lấn (0..0.80)
        public float passSpacingSMm = 29f;              // s – khoảng cách giữa 2 đường phun lân cận (mm)
        public float borderMarginEMm = 16f;             // e – biên an toàn vùng cần phun (mm)
        public float sprayDistanceDMm = 20f;            // d – khoảng cách súng đến bề mặt (mm)
        public float minPassLengthMm = 35f;             // loại bỏ đoạn quá ngắn khi ống thu hẹp (mm)
        public P2PassOrderingMode passOrdering = P2PassOrderingMode.MinimumTravelGreedy;
        public P2GunOrientationMode gunOrientation = P2GunOrientationMode.FlatPlane2D; // Theo tài liệu: P2 mặc định 2D phẳng + súng thẳng đứng (không tính pháp tuyến lân cận nhỏ)

        public float EffectiveSpacingSMm => autoSpacingFromWidthOverlap
            ? Mathf.Clamp(sprayWidthWMm * (1f - Mathf.Clamp(overlapRatioR, 0.05f, 0.80f)), 6f, 110f)
            : Mathf.Clamp(passSpacingSMm, 6f, 110f);
    }

    public class P2ExecutionResult
    {
        public List<List<Vector3>> targetRegionPolygonsWorld = new List<List<Vector3>>(); // Chỗ cần phun (hình/vùng mục tiêu trên mặt vải)
        public List<List<Vector3>> skeletonAxesWorld = new List<List<Vector3>>();         // Trục giữa (Skeleton chính) của 2 ống quần
        public List<List<Vector3>> sprayOnPathsWorld = new List<List<Vector3>>();         // Đường phun (các đoạn BẬT súng ON)
        public List<List<Vector3>> sprayOffConnectorsWorld = new List<List<Vector3>>();   // Đường quỹ đạo không phun (quét OFF & quay đầu)
        public int leftLegPassCount;
        public int rightLegPassCount;
        public int truncatedOuterPassCount;
        public int sprayOnSegmentCount;
        public int turnaroundCount;
        public float totalSprayLengthMm;
        public float totalTravelLengthMm;
        public float effectiveSpacingMm;
        public string summaryText = "";
    }

    // =========================================================================
    // CẤU HÌNH & KẾT QUẢ KIỂM TRA THUẬT TOÁN P3 (QUỸ ĐẠO MẪU THEO MỐC HÌNH HỌC)
    // =========================================================================
    public enum P3LandmarkSourceMode
    {
        AiKeypoint12 = 0,       // 12 mốc từ AI (YOLO11-pose / file landmarks)
        ContourGeometry = 1     // Tự phát hiện mốc bằng hình học trên contour (điểm lồi/lõm, giao trục giữa)
    }

    public enum P3GunOrientationMode
    {
        SmoothSurfaceNormal25D = 0, // 2.5D/3D Pháp tuyến bề mặt đã làm mượt
        Vertical25D = 1,            // 2.5D Thẳng đứng cố định
        FlatPlane2D = 2             // 2D Mặt phẳng chuẩn
    }

    [Serializable]
    public class P3LandmarkConfig
    {
        public P3LandmarkSourceMode landmarkSource = P3LandmarkSourceMode.AiKeypoint12;
        public P3GunOrientationMode gunOrientation = P3GunOrientationMode.SmoothSurfaceNormal25D;
        public float sprayDistanceDMm = 20f;      // Khoảng cách súng d (mm)
        public float borderMarginEMm = 10f;       // Biên an toàn cắt tràn ngoài mép vải (mm)
        public float resampleStepUV = 0.014f;     // Bước lấy mẫu mịn dọc quỹ đạo mẫu
        public bool fillClosedDrawnRegions = true;// true: vùng Tròn/Vuông/Đa giác có cả viền + lượt phủ; false: chỉ giữ đường viền/nét dạy
        public bool clipToGarmentContour = true;  // Cắt điểm tràn ngoài đường bao quần
    }

    public class P3ExecutionResult
    {
        public List<Vector3[]> meshTrianglesWorld = new List<Vector3[]>();                // 32 tam giác Barycentric 3D trên quần mới
        public List<bool> meshTriangleFlipped = new List<bool>();                         // Cờ cảnh báo tam giác bị đảo ngược
        public List<Vector3> detectedLandmarksWorld = new List<Vector3>();                // Bộ mốc đã phát hiện trên quần mới
        public List<List<Vector3>> targetRegionPolygonsWorld = new List<List<Vector3>>(); // Vùng mục tiêu (nếu có hình vẽ kín)
        public string landmarkMethodUsed = "";
        public int landmarkCount;
        public bool landmarkOrderConsistent;
        public int totalTriangles;
        public int flippedTriangleCount;
        public float maxTriangleDistortion;
        public float inTargetRatioPct;
        public int strokeCount;
        public float totalSprayLengthMm;
        public float totalTravelLengthMm;
        public string summaryText = "";
        public string warningMessage = "";
    }

    // =========================================================================
    // CẤU HÌNH & KẾT QUẢ KIỂM TRA THUẬT TOÁN P4 (QUỸ ĐẠO MẪU + ĐĂNG KÝ ĐÁM MÂY ĐIỂM 3D)
    // =========================================================================
    public enum P4InitMode
    {
        AutoLandmarksOrPca = 0, // Tự động: ưu tiên mốc P3, thiếu mốc dùng PCA 3D
        LandmarksP3Only = 1,    // Khởi tạo thô bằng bộ mốc P3
        Pca3DOnly = 2           // Khởi tạo thô bằng PCA 3D (không dùng mốc)
    }

    public enum P4NonRigidMode
    {
        NonRigidCpd = 0,        // CPD phi cứng (Coherent Point Drift - Gaussian Motion Coherence)
        TpsRbf = 1,             // TPS RBF (Thin-Plate Spline 3D/2.5D chuẩn hóa)
        RigidIcpOnly = 2        // Chỉ ICP cứng 3D (ablation so sánh)
    }

    public enum P4GunOrientationMode
    {
        SmoothSurfaceNormal25D = 0, // 3D/2.5D Pháp tuyến bề mặt đã làm mượt (p_TCP = p_surface + d*n)
        Vertical25D = 1,            // 2.5D Thẳng đứng cố định
        FlatPlane2D = 2             // 2D Mặt phẳng chuẩn
    }

    [Serializable]
    public class P4RegistrationConfig
    {
        public P4InitMode initMode = P4InitMode.AutoLandmarksOrPca;
        public P4NonRigidMode nonRigidMode = P4NonRigidMode.NonRigidCpd;
        public P4GunOrientationMode gunOrientation = P4GunOrientationMode.SmoothSurfaceNormal25D;
        public float voxelSizeMm = 5.0f;           // Kích thước voxel giảm điểm (~5 mm theo tài liệu)
        public int icpMaxIterations = 14;          // Số vòng lặp ICP cứng 3D
        public float icpMaxMatchRadiusMm = 85f;    // Bán kính ghép cặp ICP tối đa (mm)
        public float cpdLambda = 2.0f;             // Độ cứng biến dạng CPD phi cứng (lớn -> cứng hơn)
        public float cpdBetaMm = 180f;             // Bán kính tương quan mượt CPD beta (mm)
        public float tpsLambda = 0.012f;           // Tham số điều chuẩn TPS
        public float residualThresholdMm = 12.0f;  // Ngưỡng sai số dư đăng ký -> vượt ngưỡng thì dừng & báo kỹ sư
        public float sprayDistanceDMm = 20f;       // Khoảng lùi súng d dọc pháp tuyến (mm)
        public bool fillClosedDrawnRegions = true; // Với vùng Tròn/Vuông/Đa giác: phủ kín hay chỉ viền
        public bool clipToGarmentContour = true;
    }

    public class P4ExecutionResult
    {
        public List<Vector3> registeredCloudWorld = new List<Vector3>();                  // Đám mây quần mẫu đã đăng ký lên quần mới
        public List<List<Vector3>> targetRegionPolygonsWorld = new List<List<Vector3>>(); // Vùng mục tiêu sau đăng ký
        public string initMethodUsed = "";
        public string nonRigidMethodUsed = "";
        public int sourceVoxelCount;
        public int targetVoxelCount;
        public float residualAfterInitMm;
        public float residualAfterIcpMm;
        public float residualAfterNonRigidMm;
        public float meanSurfaceSnapShiftMm;
        public float normalStabilityStdDeg;
        public bool rejectedByResidualGate;
        public int strokeCount;
        public float totalSprayLengthMm;
        public float totalTravelLengthMm;
        public string summaryText = "";
        public string warningMessage = "";
    }

    [Serializable]
    public class MethodEvaluationResult
    {
        public string methodName;
        public string sampleName;
        public int waypointCount;
        public bool hasGroundTruth;
        public string groundTruthSource = "";

        // 12 chỉ tiêu đo chuẩn (Trang 10–11 file góp ý) + Hàm mục tiêu J (Trang 7)
        public float eGeoMeanMm;      // 1. E_geo trung bình (mm)
        public float eGeoP95Mm;       //    E_geo P95 (mm)
        public float eGeoHausdorffMm; //    E_geo Hausdorff (mm)
        public float eDMeanMm;        // 2. E_d trung bình (mm)
        public float eDMaxMm;         //    E_d lớn nhất (mm)
        public float eThetaMeanDeg;   // 3. E_θ trung bình (°)
        public float eThetaMaxDeg;    //    E_θ lớn nhất (°)
        public float eTPercent;       // 4. E_T sai lệch mật độ RMS(T - T*) / mean(T*) (%)
        public float coveragePercent; // 5. C độ phủ (% diện tích R có T >= 50% T*)
        public float oversprayPercent;// 6. O tràn ra ngoài (% lượng phun rơi ngoài R)
        public float feasibilityPct;  // 7. F khả thi (% điểm qua IK, giới hạn khớp, 0 va chạm, góc <= θ_max)
        public int collisionCount;    //    Số điểm va chạm
        public int singularityCount;  //    Số điểm gần kỳ dị / sát biên tầm với
        public float vCvPercent;      // 8. V_cv độ đều tốc độ TCP (% hệ số biến thiên)
        public float tGenSec;         // 9. t_gen thời gian sinh quỹ đạo (s)
        public float tCycleSec;       // 10. t_cycle thời gian chu kỳ robot chạy hết quỹ đạo (s)
        public float rDropMm;         // 11. R_drop độ bền vững khi lệch tư thế/nếp (mm)
        public float tSetupMin;       // 12. t_setup thời gian thiết lập mã/cỡ mới (phút, so với dạy tay)
        public float objectiveJ;      // Hàm mục tiêu chung J = MSE_R + λ_o*O + λ_t*t_cycle (Trang 7)
        public float regResidualMm;   // Sai số dư sau đăng ký (riêng P4)

        // Bước 1 — Chi tiết ngưỡng loại (Trang 11)
        public bool gateFeasibilityOk;
        public bool gateCollisionOk;
        public bool gateGeoOk;
        public bool gateTimeOk;
        public bool passedGate;       // Đạt Bước 1 ngưỡng loại (F>=95%, 0 va chạm, E_geo<=5/8, t_gen<=60)
        public string gateFailReason = "";

        // Bước 2 — 6 điểm thành phần Ma trận Quyết định (0..100) & Trọng số (30/20/15/15/10/10, Trang 11–12)
        public float scoreSprayQuality; // Nhóm 1 (30%): E_T, C, O
        public float scoreGeoAcc;       // Nhóm 2 (20%): E_geo, E_d, E_θ
        public float scoreRobust;       // Nhóm 3 (15%): R_drop
        public float scoreFeasCycle;    // Nhóm 4 (15%): F, V_cv, t_cycle
        public float scoreSpeedSetup;   // Nhóm 5 (10%): t_gen, t_setup
        public float scoreEng;          // Nhóm 6 (10%): Công cài đặt, bảo trì, chỉnh tay (1-5)
        public float engineeringScore;  // Điểm công cài đặt & kỹ sư chỉnh tay (1-5)
        public string engineeringReason = "";
        public float weightedScore;     // Tổng điểm Bước 2 Ma trận quyết định (0..100)

        // Bảng so sánh định tính (Trang 8)
        public string minInputDesc = "";
        public string curvedSurfaceRating = "";
        public string sizePoseAdaptRating = "";
        public string effectIntentRating = "";
        public string implEffortRating = "";
        public string notes;
    }

    [Serializable]
    public class SensitivityAnalysisReport
    {
        public string baseWinnerName = "";
        public int totalScenarios;
        public int stableWinCount;
        public bool isStable;
        public string summaryText = "";
        public List<string> scenarioLines = new List<string>();
    }

    /// <summary>
    /// Dao động độ dày tương đối xấp xỉ theo mô hình Gauss (trang 6 file góp ý):
    /// ΔT / T̄ ≈ 2 * exp(-2 * π² * σ² / s²)
    /// </summary>
    public static float ComputeGaussRipplePercent(float sigmaMm, float stepSMm)
    {
        if (stepSMm <= 1e-3f || sigmaMm <= 1e-3f) return 0f;
        double ratio = (double)sigmaMm / stepSMm;
        double ripple = 2.0 * Math.Exp(-2.0 * Math.PI * Math.PI * ratio * ratio);
        return (float)(ripple * 100.0);
    }

    // =========================================================================
    // 1. BỘ GIẢI HỆ PHƯƠNG TRÌNH TUYẾN TÍNH GAUSS-JORDAN & THIN PLATE SPLINE (TPS) THỰC
    // =========================================================================

    public static bool SolveLinearSystem(double[,] A, double[] b, out double[] x)
    {
        int n = b.Length;
        x = new double[n];
        double[,] M = new double[n, n + 1];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++) M[i, j] = A[i, j];
            M[i, n] = b[i];
        }

        for (int col = 0; col < n; col++)
        {
            int pivot = col;
            double maxVal = Math.Abs(M[col, col]);
            for (int row = col + 1; row < n; row++)
            {
                double val = Math.Abs(M[row, col]);
                if (val > maxVal)
                {
                    maxVal = val;
                    pivot = row;
                }
            }
            if (maxVal < 1e-12) return false;

            if (pivot != col)
            {
                for (int j = col; j <= n; j++)
                {
                    double tmp = M[col, j];
                    M[col, j] = M[pivot, j];
                    M[pivot, j] = tmp;
                }
            }

            double diag = M[col, col];
            for (int j = col; j <= n; j++) M[col, j] /= diag;

            for (int row = 0; row < n; row++)
            {
                if (row == col) continue;
                double factor = M[row, col];
                if (Math.Abs(factor) < 1e-15) continue;
                for (int j = col; j <= n; j++)
                    M[row, j] -= factor * M[col, j];
            }
        }

        for (int i = 0; i < n; i++) x[i] = M[i, n];
        return true;
    }

    private static double TpsKernel(double r2)
    {
        if (r2 < 1e-12) return 0.0;
        return 0.5 * r2 * Math.Log(r2); // r^2 * ln(r)
    }

    /// <summary>
    /// Fit bề mặt Thin Plate Spline (TPS) RBF thực sự y(x, z) = sum(w_i * r_i^2 ln r_i) + a0 + a1*x + a2*z
    /// trên tập điểm biên hợp lệ (world space) và trả về độ cao + pháp tuyến giải tích (-∂y/∂x, 1, -∂y/∂z)!
    /// </summary>
    public static bool EvaluateTrueTpsSurface(
        List<Vector3> controlPtsWorld,
        float queryX,
        float queryZ,
        out float outY,
        out Vector3 outNormal,
        double regularization = 1e-4)
    {
        outY = 0.020f;
        outNormal = Vector3.up;
        if (controlPtsWorld == null || controlPtsWorld.Count < 4) return false;

        // Giới hạn tối đa 28 điểm điều khiển gần nhất để giải tức thì (< 0.1 ms)
        int n = Mathf.Min(controlPtsWorld.Count, 28);
        int dim = n + 3;
        double[,] K = new double[dim, dim];
        double[] rhs = new double[dim];

        // Chuẩn hóa tọa độ cục bộ quanh điểm query để ổn định số học
        double[] px = new double[n];
        double[] pz = new double[n];
        for (int i = 0; i < n; i++)
        {
            px[i] = controlPtsWorld[i].x - queryX;
            pz[i] = controlPtsWorld[i].z - queryZ;
            rhs[i] = controlPtsWorld[i].y;
        }

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i == j)
                {
                    K[i, j] = regularization;
                }
                else
                {
                    double dx = px[i] - px[j];
                    double dz = pz[i] - pz[j];
                    K[i, j] = TpsKernel(dx * dx + dz * dz);
                }
            }
            K[i, n] = 1.0;
            K[i, n + 1] = px[i];
            K[i, n + 2] = pz[i];

            K[n, i] = 1.0;
            K[n + 1, i] = px[i];
            K[n + 2, i] = pz[i];
        }

        if (!SolveLinearSystem(K, rhs, out double[] w))
            return false;

        // Đánh giá tại (0, 0) vì đã dịch gốc về (queryX, queryZ)
        double yVal = w[n];
        double dydx = w[n + 1];
        double dydz = w[n + 2];

        for (int i = 0; i < n; i++)
        {
            double dx = -px[i];
            double dz = -pz[i];
            double r2 = dx * dx + dz * dz;
            if (r2 > 1e-12)
            {
                double lnR = 0.5 * Math.Log(r2);
                yVal += w[i] * r2 * lnR;
                // d/dr(r^2 ln r) / r = 2 ln r + 1
                double dFactor = w[i] * (2.0 * lnR + 1.0);
                dydx += dFactor * dx;
                dydz += dFactor * dz;
            }
        }

        if (double.IsNaN(yVal) || double.IsInfinity(yVal)) return false;

        // Chống ngoại suy bùng nổ quá biên lân cận
        float minY = float.MaxValue, maxY = float.MinValue;
        for (int i = 0; i < n; i++)
        {
            minY = Mathf.Min(minY, controlPtsWorld[i].y);
            maxY = Mathf.Max(maxY, controlPtsWorld[i].y);
        }
        outY = Mathf.Clamp((float)yVal, minY - 0.03f, maxY + 0.03f);
        outNormal = new Vector3((float)-dydx, 1f, (float)-dydz).normalized;
        if (outNormal.y < 0f) outNormal = -outNormal;
        return true;
    }

    // =========================================================================
    // 2. VÙNG CẤM PHUN (KEEP-OUT ZONES) & KIỂM TRA BIÊN VẢI AN TOÀN
    // =========================================================================

    /// <summary>
    /// Kiểm tra một điểm (worldX, worldZ) có nằm trong VÙNG PHUN HỢP LỆ R hay không:
    /// - Nằm trong đường bao quần & cách biên vải ít nhất `borderMarginMm` (co biên e)
    /// - KHÔNG rơi vào vùng cấm phun: Cạp/Túi/Khóa kéo (trên đáy đũng), vùng đệm đáy đũng (35mm), vùng gấu quần (20% cuối)
    /// </summary>
    public static bool IsInsideValidSprayZone(
        float worldX,
        float worldZ,
        List<Vector3> garmentContourWorld,
        List<Vector3> landmarksWorld,
        float borderMarginMm,
        out string keepOutReason)
    {
        keepOutReason = "";
        Vector2 p = new Vector2(worldX, worldZ);

        // 1. Kiểm tra nằm trong đa giác quần & khoảng cách tới biên >= borderMarginMm
        if (garmentContourWorld != null && garmentContourWorld.Count >= 4)
        {
            if (!IsPointInPolyXZ(worldX, worldZ, garmentContourWorld))
            {
                keepOutReason = "Ngoài biên quần";
                return false;
            }
            float minEdgeDistM = MinDistanceToPolygonEdgesXZ(p, garmentContourWorld);
            if (minEdgeDistM < borderMarginMm * 0.001f)
            {
                keepOutReason = $"Sát mép vải (< {borderMarginMm:F0}mm)";
                return false;
            }
        }

        // 2. Kiểm tra các vùng cấm giải phẫu dựa trên 12 mốc AI
        if (landmarksWorld != null && landmarksWorld.Count >= 12)
        {
            Vector2 crotch = new Vector2(landmarksWorld[3].x, landmarksWorld[3].z);
            Vector2 cuffL = 0.5f * (new Vector2(landmarksWorld[8].x, landmarksWorld[8].z) + new Vector2(landmarksWorld[9].x, landmarksWorld[9].z));
            Vector2 cuffR = 0.5f * (new Vector2(landmarksWorld[10].x, landmarksWorld[10].z) + new Vector2(landmarksWorld[11].x, landmarksWorld[11].z));
            Vector2 cuffMid = 0.5f * (cuffL + cuffR);

            Vector2 vLeg = cuffMid - crotch;
            float legLen = vLeg.magnitude;
            if (legLen > 0.1f)
            {
                Vector2 vUnit = vLeg / legLen;
                float proj = Vector2.Dot(p - crotch, vUnit);

                if (proj < 0.05f * legLen)
                {
                    keepOutReason = "Vùng cấm: Cạp / Túi / Khóa kéo";
                    return false;
                }
                if (proj > 0.82f * legLen)
                {
                    keepOutReason = "Vùng cấm: Gấu quần";
                    return false;
                }
                if (Vector2.Distance(p, crotch) < 0.035f)
                {
                    keepOutReason = "Vùng cấm: Đường may đáy đũng (R<35mm)";
                    return false;
                }
            }
        }

        return true;
    }

    public static bool IsPointInPolyXZ(float px, float pz, List<Vector3> poly)
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

    private static float MinDistanceToPolygonEdgesXZ(Vector2 p, List<Vector3> poly)
    {
        float minD2 = float.MaxValue;
        int n = poly.Count;
        for (int i = 0; i < n; i++)
        {
            Vector2 a = new Vector2(poly[i].x, poly[i].z);
            Vector2 b = new Vector2(poly[(i + 1) % n].x, poly[(i + 1) % n].z);
            Vector2 ab = b - a;
            float len2 = ab.sqrMagnitude;
            float t = len2 > 1e-9f ? Mathf.Clamp01(Vector2.Dot(p - a, ab) / len2) : 0f;
            float d2 = (p - (a + ab * t)).sqrMagnitude;
            if (d2 < minD2) minD2 = d2;
        }
        return Mathf.Sqrt(minD2);
    }

    // =========================================================================
    // 3. SINH QUỸ ĐẠO 3 PHƯƠNG PHÁP SO SÁNH CÔNG BẰNG TRÊN CÙNG PHÔI (P1, P3, P4)
    // =========================================================================

    /// <summary>
    /// Xây dựng Waypoint 3D chuẩn vật lý phun:
    ///   P_TCP = P_surface + d * n_surface (lùi đúng khoảng cách d dọc pháp tuyến 3D!)
    ///   z_tool = -n_surface (vuông góc mặt vải, giới hạn góc nghiêng <= θ_max)
    /// </summary>
    /// <summary>
    /// Độ cao lớp phun thống nhất (mm) dùng chung cho cả Vẽ tay 2D->3D và các phương pháp P1 / P3 / P4,
    /// tránh việc đổi phương pháp làm nhảy độ cao đột ngột giữa customDrawZOffsetMm và standoffMm.
    /// </summary>
    public static float GetEffectiveSprayHeightMm(DobotCR10AIKController ctrl, SprayTechParams tech)
    {
        if (ctrl != null) return Mathf.Max(0f, ctrl.customDrawZOffsetMm);
        return tech != null ? Mathf.Max(0f, tech.standoffMm) : 20f;
    }

    public static List<DobotCR10AIKController.WaypointData> BuildSprayWaypointsFromWorldStrokes(
        List<List<Vector3>> worldSurfaceStrokes,
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        SprayTechParams tech,
        string methodTag,
        bool useFixedVerticalNormal = false,
        bool useFlatPlaneZ = false,
        float flatPlaneWorldY = 0.02f,
        float overrideStandoffMm = -1f,
        List<List<bool>> perPointSprayOn = null)
    {
        var result = new List<DobotCR10AIKController.WaypointData>();
        if (worldSurfaceStrokes == null || worldSurfaceStrokes.Count == 0 || ctrl == null) return result;

        float effectiveStandoffMm = (overrideStandoffMm >= 0f) ? overrideStandoffMm : GetEffectiveSprayHeightMm(ctrl, tech);
        float standoffM = effectiveStandoffMm * 0.001f;
        Transform gRef = ctrl.globalReference;

        // 1. Lấy điểm bề mặt + pháp tuyến thô cho từng nét, sau đó làm mượt pháp tuyến dọc nét
        int wpIdx = 0;
        float cumDistMm = 0f;
        Vector3 prevTcpRobotMm = Vector3.zero;
        float[] prevJoints = null;

        for (int s = 0; s < worldSurfaceStrokes.Count; s++)
        {
            var stroke = worldSurfaceStrokes[s];
            if (stroke == null || stroke.Count < 2) continue;

            int m = stroke.Count;
            Vector3[] surfPtsWorld = new Vector3[m];
            Vector3[] surfNormsWorld = new Vector3[m];

            for (int p = 0; p < m; p++)
            {
                Vector3 pt = stroke[p];
                Vector3 nrm = Vector3.up;
                if (useFlatPlaneZ)
                {
                    pt.y = flatPlaneWorldY;
                    nrm = Vector3.up;
                }
                else if (ctrl.SampleCompensatedSurface(pt.x, pt.z, out float cy, out Vector3 cn))
                {
                    pt.y = cy;
                    nrm = useFixedVerticalNormal ? Vector3.up : cn;
                }
                else if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(pt.x, pt.z, out float py, out Vector3 pn))
                {
                    pt.y = py;
                    nrm = useFixedVerticalNormal ? Vector3.up : pn;
                }
                if (nrm.y < 0f) nrm = -nrm;
                surfPtsWorld[p] = pt;
                surfNormsWorld[p] = nrm.normalized;
            }

            // Làm mượt trường pháp tuyến (3-point moving average trên mặt cầu) để tránh cổ tay xoay giật
            Vector3[] smoothNormsWorld = new Vector3[m];
            for (int p = 0; p < m; p++)
            {
                Vector3 sum = surfNormsWorld[p] * 2f;
                if (p > 0) sum += surfNormsWorld[p - 1];
                if (p < m - 1) sum += surfNormsWorld[p + 1];
                smoothNormsWorld[p] = sum.normalized;
            }

            for (int p = 0; p < m; p++)
            {
                Vector3 surfW = surfPtsWorld[p];
                Vector3 normW = smoothNormsWorld[p];

                // Giới hạn góc nghiêng pháp tuyến so với trục đứng <= maxTiltDeg
                float tiltDeg = Vector3.Angle(normW, Vector3.up);
                if (tiltDeg > tech.maxTiltDeg && tech.maxTiltDeg > 0.1f)
                {
                    Vector3 axis = Vector3.Cross(Vector3.up, normW);
                    if (axis.sqrMagnitude > 1e-6f)
                        normW = (Quaternion.AngleAxis(tech.maxTiltDeg, axis.normalized) * Vector3.up).normalized;
                }

                // P_TCP = P_surface + d * normal (lùi dọc pháp tuyến 3D thực!)
                Vector3 tcpW = surfW + normW * standoffM;

                // Chuyển vị trí, pháp tuyến & tiếp tuyến sang hệ Robot Global (mm)
                Vector3 tcpRobotMm = gRef != null
                    ? gRef.InverseTransformPoint(tcpW) * 1000f
                    : new Vector3((1.100f - tcpW.z) * 1000f, -tcpW.x * 1000f, tcpW.y * 1000f);
                tcpRobotMm.z = Mathf.Clamp(tcpRobotMm.z, 18f, 550f);
                if (tcpRobotMm.magnitude > 1145f)
                {
                    float rScale = 1140f / tcpRobotMm.magnitude;
                    tcpRobotMm.x *= rScale;
                    tcpRobotMm.y *= rScale;
                    tcpW = gRef != null
                        ? gRef.TransformPoint(tcpRobotMm * 0.001f)
                        : new Vector3(-tcpRobotMm.y * 0.001f, tcpRobotMm.z * 0.001f, 1.100f - tcpRobotMm.x * 0.001f);
                }

                Vector3 normRobot = gRef != null
                    ? gRef.InverseTransformDirection(normW).normalized
                    : new Vector3(-normW.z, -normW.x, normW.y).normalized;

                Vector3 tanW = (p < m - 1) ? (surfPtsWorld[p + 1] - surfPtsWorld[p]) : (surfPtsWorld[p] - surfPtsWorld[p - 1]);
                if (tanW.sqrMagnitude < 1e-7f) tanW = Vector3.forward;
                Vector3 tanRobot = gRef != null
                    ? gRef.InverseTransformDirection(tanW).normalized
                    : new Vector3(-tanW.z, -tanW.x, tanW.y).normalized;

                Vector3 rpyDeg = DobotCR10AKinematics.SurfaceNormalToRobotRpyDeg(normRobot, tanRobot, tech.maxTiltDeg);

                if (wpIdx > 0) cumDistMm += Vector3.Distance(prevTcpRobotMm, tcpRobotMm);
                prevTcpRobotMm = tcpRobotMm;

                bool customSprayFlag = (perPointSprayOn == null || s >= perPointSprayOn.Count || perPointSprayOn[s] == null || p >= perPointSprayOn[s].Count)
                    ? true
                    : perPointSprayOn[s][p];

                var wp = new DobotCR10AIKController.WaypointData
                {
                    index = wpIdx,
                    distance_mm = cumDistMm,
                    targetPosGlobal_mm = tcpRobotMm,
                    targetPosWorld_m = tcpW,
                    microNormal = normW,
                    roll_deg = rpyDeg.x,
                    pitch_deg = rpyDeg.y,
                    yaw_deg = rpyDeg.z,
                    sprayOn = (p > 0) && customSprayFlag, // Điểm đầu mỗi nét là đoạn di chuyển chuyển tiếp -> TẮT PHUN
                    segmentName = $"{methodTag}_S{s + 1}"
                };

                Quaternion qRot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                var sols = DobotCR10AKinematics.SolveTcpPose(
                    wp.targetPosGlobal_mm, qRot, prevJoints,
                    ctrl.toolLength, useAdaptiveTilt: true, maxTiltDeg: tech.maxTiltDeg,
                    toolOffsetX: ctrl.toolOffsetX, toolOffsetY: ctrl.toolOffsetY);

                if (sols == null || sols.Count == 0)
                {
                    wp.roll_deg = 180f;
                    wp.pitch_deg = 0f;
                    wp.yaw_deg = 0f;
                    qRot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                    sols = DobotCR10AKinematics.SolveTcpPose(
                        wp.targetPosGlobal_mm, qRot, prevJoints,
                        ctrl.toolLength, useAdaptiveTilt: true, maxTiltDeg: tech.maxTiltDeg,
                        toolOffsetX: ctrl.toolOffsetX, toolOffsetY: ctrl.toolOffsetY);
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
                    prevJoints = wp.jointAngles_rad;
                }

                result.Add(wp);
                wpIdx++;
            }
        }

        return result;
    }

    // =========================================================================
    // 2B. HỆ TỌA ĐỘ GIẢI PHẪU ĐỘC LẬP 2 ỐNG QUẦN (ANATOMICAL LEG-AWARE WARPING)
    //     Tự động thích nghi chính xác khi mỗi chiếc quần có góc xòe 2 ống,
    //     độ cong ống, bề rộng ống, vị trí và hướng đặt khác nhau!
    // =========================================================================

    private struct AnatomicalLegChain
    {
        public Vector2[] C; // 5 điểm trục tâm: 0=Eo, 1=Hông/Đũng, 2=Đùi, 3=Gối, 4=Gấu
        public Vector2[] N; // 5 vector ngang đơn vị hướng từ mép ngoài -> mép trong ống
        public float[] R;   // 5 bán kính bề rộng ống quần tại từng trạm
    }

    private static AnatomicalLegChain BuildLegChain(Vector2[] lm12, bool isLeftLeg)
    {
        var chain = new AnatomicalLegChain
        {
            C = new Vector2[5],
            N = new Vector2[5],
            R = new float[5]
        };

        Vector2 waistOuter = isLeftLeg ? lm12[0] : lm12[2];
        Vector2 waistCenter = lm12[1];
        Vector2 crotch = lm12[3];
        Vector2 thigh = isLeftLeg ? lm12[4] : lm12[5];
        Vector2 knee = isLeftLeg ? lm12[6] : lm12[7];
        Vector2 cuffOuter = isLeftLeg ? lm12[8] : lm12[11];
        Vector2 cuffInner = isLeftLeg ? lm12[9] : lm12[10];

        Vector2 vWaist = waistCenter - waistOuter;
        float rWaist = Mathf.Max(1e-4f, 0.5f * vWaist.magnitude);
        Vector2 nWaist = vWaist.sqrMagnitude > 1e-8f ? vWaist.normalized : (isLeftLeg ? Vector2.right : Vector2.left);

        Vector2 vCuff = cuffInner - cuffOuter;
        float rCuff = Mathf.Max(1e-4f, 0.5f * vCuff.magnitude);
        Vector2 nCuff = vCuff.sqrMagnitude > 1e-8f ? vCuff.normalized : nWaist;

        chain.C[0] = 0.5f * (waistOuter + waistCenter);
        chain.C[1] = crotch + 0.5f * (waistOuter - waistCenter);
        chain.C[2] = thigh;
        chain.C[3] = knee;
        chain.C[4] = 0.5f * (cuffOuter + cuffInner);

        Vector2 refInner = nWaist + nCuff;
        if (refInner.sqrMagnitude < 1e-6f) refInner = nWaist;
        else refInner.Normalize();

        chain.N[0] = nWaist;
        chain.R[0] = rWaist;
        chain.N[4] = nCuff;
        chain.R[4] = rCuff;

        for (int k = 1; k <= 3; k++)
        {
            Vector2 dIn = chain.C[k] - chain.C[k - 1];
            Vector2 dOut = chain.C[k + 1] - chain.C[k];
            Vector2 tIn = dIn.sqrMagnitude > 1e-8f ? dIn.normalized : Vector2.up;
            Vector2 tOut = dOut.sqrMagnitude > 1e-8f ? dOut.normalized : tIn;
            Vector2 tan = (tIn + tOut);
            if (tan.sqrMagnitude < 1e-6f) tan = tOut;
            else tan.Normalize();

            Vector2 nPerp = new Vector2(-tan.y, tan.x);
            if (Vector2.Dot(nPerp, refInner) < 0f) nPerp = -nPerp;

            if (k == 1)
            {
                Vector2 nHip = 0.55f * nPerp + 0.45f * nWaist;
                nHip = nHip.sqrMagnitude > 1e-6f ? nHip.normalized : nPerp;
                if (Vector2.Dot(nHip, chain.N[0]) < 0f) nHip = -nHip;
                chain.N[1] = nHip;
                float projCrotch = Vector2.Dot(crotch - chain.C[1], nHip);
                chain.R[1] = Mathf.Max(rWaist * 0.65f, Mathf.Abs(projCrotch));
            }
            else
            {
                if (Vector2.Dot(nPerp, chain.N[k - 1]) < 0f) nPerp = -nPerp;
                chain.N[k] = nPerp;
                chain.R[k] = (k == 2)
                    ? Mathf.Lerp(chain.R[1], chain.R[4], 0.30f)
                    : Mathf.Lerp(chain.R[1], chain.R[4], 0.65f);
            }
        }
        if (Vector2.Dot(chain.N[4], chain.N[3]) < 0f) chain.N[4] = -chain.N[4];

        return chain;
    }

    private static void ProjectOntoLegChain(
        Vector2 pt,
        in AnatomicalLegChain chain,
        out int bestSeg,
        out float bestU,
        out float bestW,
        out float bestDistSq)
    {
        bestSeg = 0;
        bestU = 0f;
        bestW = 0f;
        bestDistSq = float.MaxValue;
        float bestRawU = 0f;

        for (int k = 0; k < 4; k++)
        {
            Vector2 a = chain.C[k];
            Vector2 ab = chain.C[k + 1] - a;
            float len2 = Mathf.Max(1e-8f, ab.sqrMagnitude);
            float uRaw = Vector2.Dot(pt - a, ab) / len2;
            float uClamp = Mathf.Clamp01(uRaw);
            Vector2 cPt = a + ab * uClamp;
            float d2 = (pt - cPt).sqrMagnitude;
            if (d2 < bestDistSq)
            {
                bestDistSq = d2;
                bestSeg = k;
                bestU = uClamp;
                bestRawU = uRaw;
            }
        }

        // Cho phép ngoại suy mượt phía trên cạp (seg 0) hoặc phía dưới gấu (seg 3)
        float uFinal = bestU;
        if (bestSeg == 0 && bestRawU < 0f) uFinal = Mathf.Clamp(bestRawU, -0.5f, 1f);
        else if (bestSeg == 3 && bestRawU > 1f) uFinal = Mathf.Clamp(bestRawU, 0f, 1.5f);

        float uInterp = Mathf.Clamp01(uFinal);
        Vector2 cFinal = chain.C[bestSeg] + (chain.C[bestSeg + 1] - chain.C[bestSeg]) * uFinal;
        Vector2 nVec = Vector2.Lerp(chain.N[bestSeg], chain.N[bestSeg + 1], uInterp);
        if (nVec.sqrMagnitude < 1e-6f) nVec = chain.N[bestSeg];
        else nVec.Normalize();
        float rVal = Mathf.Max(1e-4f, Mathf.Lerp(chain.R[bestSeg], chain.R[bestSeg + 1], uInterp));

        bestU = uFinal;
        bestW = Vector2.Dot(pt - cFinal, nVec) / rVal;
    }

    private static Vector2 ReconstructOnLegChain(in AnatomicalLegChain chain, int seg, float u, float w)
    {
        float uInterp = Mathf.Clamp01(u);
        Vector2 cDst = chain.C[seg] + (chain.C[seg + 1] - chain.C[seg]) * u;
        Vector2 nDst = Vector2.Lerp(chain.N[seg], chain.N[seg + 1], uInterp);
        if (nDst.sqrMagnitude < 1e-6f) nDst = chain.N[seg];
        else nDst.Normalize();
        float rDst = Mathf.Max(1e-4f, Mathf.Lerp(chain.R[seg], chain.R[seg + 1], uInterp));
        return cDst + nDst * (w * rDst);
    }

    /// <summary>
    /// Ánh xạ 2D giải phẫu độc lập 2 ống quần giữa bất kỳ 2 bộ 12 điểm mốc nào (Canvas chuẩn <-> Ảnh Camera <-> Unity XZ):
    /// Đảm bảo ống trái và ống phải tự xoay, co giãn, xòe rộng/hẹp độc lập theo đúng tư thế chiếc quần đích!
    /// Hỗ trợ truyền tỷ lệ khung hình (aspect ratio X/Y của nguồn và đích) để pháp tuyến vuông góc 90° không bị méo trên ảnh 1944x1200.
    /// </summary>
    public static Vector2 WarpPointAnatomical2D(Vector2 pt, Vector2[] src12, Vector2[] dst12, float srcAspectXOverY = 1f, float dstAspectXOverY = 1f)
    {
        if (src12 == null || dst12 == null || src12.Length < 12 || dst12.Length < 12) return pt;

        float sAx = Mathf.Max(0.05f, srcAspectXOverY);
        float dAx = Mathf.Max(0.05f, dstAspectXOverY);
        Vector2 ptIso = new Vector2(pt.x * sAx, pt.y);
        Vector2[] srcIso = src12;
        if (Mathf.Abs(sAx - 1f) > 1e-4f)
        {
            srcIso = new Vector2[12];
            for (int i = 0; i < 12; i++) srcIso[i] = new Vector2(src12[i].x * sAx, src12[i].y);
        }
        Vector2[] dstIso = dst12;
        if (Mathf.Abs(dAx - 1f) > 1e-4f)
        {
            dstIso = new Vector2[12];
            for (int i = 0; i < 12; i++) dstIso[i] = new Vector2(dst12[i].x * dAx, dst12[i].y);
        }

        AnatomicalLegChain srcL = BuildLegChain(srcIso, true);
        AnatomicalLegChain srcR = BuildLegChain(srcIso, false);

        ProjectOntoLegChain(ptIso, srcL, out int segL, out float uL, out float wL, out float d2L);
        ProjectOntoLegChain(ptIso, srcR, out int segR, out float uR, out float wR, out float d2R);

        float absWL = Mathf.Abs(wL);
        float absWR = Mathf.Abs(wR);
        bool useLeft = (absWL <= 1.15f || absWR <= 1.15f) ? (absWL <= absWR) : (d2L <= d2R);

        Vector2 resIso;
        if (useLeft)
        {
            AnatomicalLegChain dstL = BuildLegChain(dstIso, true);
            resIso = ReconstructOnLegChain(dstL, segL, uL, wL);
        }
        else
        {
            AnatomicalLegChain dstR = BuildLegChain(dstIso, false);
            resIso = ReconstructOnLegChain(dstR, segR, uR, wR);
        }
        return new Vector2(resIso.x / dAx, resIso.y);
    }

    // =========================================================================
    // 2C. LƯỚI TAM GIÁC GIẢI PHẪU 32 MẢNH (STRUCTURED ANATOMICAL TRIANGLE MESH)
    //     & TỌA ĐỘ TRỌNG TÂM (BARYCENTRIC COORDINATES - THUẬT TOÁN P3 CHUẨN)
    //     Đúng mô tả P3 trang 7 file góp ý:
    //     "Chia quần thành lưới tam giác theo mốc; biểu diễn mỗi điểm quỹ đạo mẫu
    //      bằng tọa độ trọng tâm (barycentric) trong tam giác chứa nó -> với quần mới:
    //      phát hiện mốc -> dựng lại lưới -> biến đổi affine từng mảnh."
    //
    //     Từ 12 mốc YOLO11-pose, mỗi ống quần (Trái & Phải) được chia thành 5 trạm
    //     ngang (0:Eo, 1:Hông/Đũng, 2:Đùi, 3:Gối, 4:Gấu) x 3 đỉnh (Ngoài, Tâm, Trong)
    //     = 15 đỉnh -> 4 dải dọc x 4 tam giác đều đặn = 16 tam giác/ống (Tổng 32 tam giác).
    //     Loại bỏ hoàn toàn tam giác suy biến/chéo ống khi quần xòe hoặc xoay ngang!
    // =========================================================================

    private static readonly int[,] LegStripTriangles16 = new int[16, 3]
    {
        // Dải 0: Eo (k=0) -> Hông/Đũng (k=1)
        { 0, 1, 3 }, { 1, 4, 3 }, { 1, 2, 4 }, { 2, 5, 4 },
        // Dải 1: Hông/Đũng (k=1) -> Đùi (k=2)
        { 3, 4, 6 }, { 4, 7, 6 }, { 4, 5, 7 }, { 5, 8, 7 },
        // Dải 2: Đùi (k=2) -> Gối (k=3)
        { 6, 7, 9 }, { 7, 10, 9 }, { 7, 8, 10 }, { 8, 11, 10 },
        // Dải 3: Gối (k=3) -> Gấu (k=4)
        { 9, 10, 12 }, { 10, 13, 12 }, { 10, 11, 13 }, { 11, 14, 13 }
    };

    private static void BuildLegStructuredVertices15(in AnatomicalLegChain chain, Vector2[] verts15)
    {
        for (int k = 0; k < 5; k++)
        {
            Vector2 c = chain.C[k];
            Vector2 n = chain.N[k];
            float r = chain.R[k];
            verts15[k * 3 + 0] = c - n * r; // Đỉnh biên ngoài (Outer, w = -1)
            verts15[k * 3 + 1] = c;         // Đỉnh trục giữa (Center, w = 0)
            verts15[k * 3 + 2] = c + n * r; // Đỉnh biên trong (Inner, w = +1)
        }
    }

    private static bool ComputeBarycentric2D(Vector2 p, Vector2 a, Vector2 b, Vector2 c, out float u, out float v, out float w)
    {
        Vector2 v0 = b - a;
        Vector2 v1 = c - a;
        Vector2 v2 = p - a;
        float den = v0.x * v1.y - v1.x * v0.y;
        if (Mathf.Abs(den) < 1e-8f)
        {
            u = 1f; v = 0f; w = 0f;
            return false;
        }
        float invDen = 1.0f / den;
        v = (v2.x * v1.y - v1.x * v2.y) * invDen;
        w = (v0.x * v2.y - v2.x * v0.y) * invDen;
        u = 1.0f - v - w;
        return true;
    }

    /// <summary>
    /// Biến đổi Affine từng mảnh theo Tọa độ Trọng tâm (Barycentric Coordinates) trên lưới 32 tam giác giải phẫu
    /// được dựng từ 12 mốc (Thuật toán P3 chuẩn trang 7). Đảm bảo nội suy chính xác 100% tại 12 mốc và trên cả 2 ống quần!
    /// </summary>
    public static Vector2 WarpPointBarycentric12(Vector2 pt, Vector2[] src12, Vector2[] dst12, float srcAspectXOverY = 1f, float dstAspectXOverY = 1f)
    {
        if (src12 == null || dst12 == null || src12.Length < 12 || dst12.Length < 12) return pt;

        float sAx = Mathf.Max(0.05f, srcAspectXOverY);
        float dAx = Mathf.Max(0.05f, dstAspectXOverY);
        Vector2 ptIso = new Vector2(pt.x * sAx, pt.y);
        Vector2[] srcIso = src12;
        if (Mathf.Abs(sAx - 1f) > 1e-4f)
        {
            srcIso = new Vector2[12];
            for (int i = 0; i < 12; i++) srcIso[i] = new Vector2(src12[i].x * sAx, src12[i].y);
        }
        Vector2[] dstIso = dst12;
        if (Mathf.Abs(dAx - 1f) > 1e-4f)
        {
            dstIso = new Vector2[12];
            for (int i = 0; i < 12; i++) dstIso[i] = new Vector2(dst12[i].x * dAx, dst12[i].y);
        }

        // Phân định ống trái / ống phải để hai ống không bao giờ kéo méo nhau qua khe đũng quần
        AnatomicalLegChain srcChainL = BuildLegChain(srcIso, true);
        AnatomicalLegChain srcChainR = BuildLegChain(srcIso, false);
        ProjectOntoLegChain(ptIso, srcChainL, out int segL, out float uL, out float wL, out float d2L);
        ProjectOntoLegChain(ptIso, srcChainR, out int segR, out float uR, out float wR, out float d2R);

        float absWL = Mathf.Abs(wL);
        float absWR = Mathf.Abs(wR);
        bool useLeft = (absWL <= 1.15f || absWR <= 1.15f) ? (absWL <= absWR) : (d2L <= d2R);

        AnatomicalLegChain activeSrcChain = useLeft ? srcChainL : srcChainR;
        AnatomicalLegChain activeDstChain = BuildLegChain(dstIso, useLeft);

        Vector2[] srcVerts = new Vector2[15];
        Vector2[] dstVerts = new Vector2[15];
        BuildLegStructuredVertices15(activeSrcChain, srcVerts);
        BuildLegStructuredVertices15(activeDstChain, dstVerts);

        int bestTri = -1;
        float bestMinCoord = float.MinValue;
        float bestU = 1f, bestV = 0f, bestW = 0f;
        float bestCenterDistSq = float.MaxValue;

        for (int t = 0; t < 16; t++)
        {
            int i0 = LegStripTriangles16[t, 0];
            int i1 = LegStripTriangles16[t, 1];
            int i2 = LegStripTriangles16[t, 2];

            if (!ComputeBarycentric2D(ptIso, srcVerts[i0], srcVerts[i1], srcVerts[i2], out float u, out float v, out float w))
                continue;

            float minCoord = Mathf.Min(u, Mathf.Min(v, w));
            if (minCoord >= -1e-4f)
            {
                // Điểm nằm trọn bên trong tam giác t -> Biến đổi Affine từng mảnh chuẩn Barycentric: P' = u*A' + v*B' + w*C'
                Vector2 mapped = u * dstVerts[i0] + v * dstVerts[i1] + w * dstVerts[i2];
                return new Vector2(mapped.x / dAx, mapped.y);
            }

            Vector2 triCenter = (srcVerts[i0] + srcVerts[i1] + srcVerts[i2]) / 3f;
            float d2 = (ptIso - triCenter).sqrMagnitude;
            if (minCoord > bestMinCoord - 0.05f && d2 < bestCenterDistSq)
            {
                bestMinCoord = Mathf.Max(bestMinCoord, minCoord);
                bestCenterDistSq = d2;
                bestTri = t;
                bestU = Mathf.Clamp(u, -0.45f, 1.45f);
                bestV = Mathf.Clamp(v, -0.45f, 1.45f);
                bestW = 1f - bestU - bestV;
            }
        }

        if (bestTri >= 0 && bestMinCoord >= -0.45f)
        {
            int j0 = LegStripTriangles16[bestTri, 0];
            int j1 = LegStripTriangles16[bestTri, 1];
            int j2 = LegStripTriangles16[bestTri, 2];
            Vector2 mapped = bestU * dstVerts[j0] + bestV * dstVerts[j1] + bestW * dstVerts[j2];
            return new Vector2(mapped.x / dAx, mapped.y);
        }

        Vector2 fallback = ReconstructOnLegChain(activeDstChain, useLeft ? segL : segR, useLeft ? uL : uR, useLeft ? wL : wR);
        return new Vector2(fallback.x / dAx, fallback.y);
    }

    // =========================================================================
    // 2D. ĐƠN GIẢN HÓA BIÊN DOUGLAS-PEUCKER, PCA TRỤC ỐNG & VÙNG MẪU R (P1 & P2)
    // =========================================================================

    public static List<Vector2> SimplifyPolygonDouglasPeucker(List<Vector2> pts, float epsilon)
    {
        if (pts == null || pts.Count < 3) return pts != null ? new List<Vector2>(pts) : new List<Vector2>();
        int n = pts.Count;
        bool[] keep = new bool[n];
        keep[0] = true;
        keep[n - 1] = true;

        var stack = new Stack<Vector2Int>();
        stack.Push(new Vector2Int(0, n - 1));
        while (stack.Count > 0)
        {
            Vector2Int seg = stack.Pop();
            int first = seg.x, last = seg.y;
            if (last <= first + 1) continue;

            float maxDist = -1f;
            int index = -1;
            Vector2 a = pts[first], b = pts[last];
            for (int i = first + 1; i < last; i++)
            {
                float d = DistToSegment2D(pts[i], a, b);
                if (d > maxDist) { maxDist = d; index = i; }
            }
            if (maxDist > epsilon && index >= 0)
            {
                keep[index] = true;
                stack.Push(new Vector2Int(first, index));
                stack.Push(new Vector2Int(index, last));
            }
        }

        var result = new List<Vector2>(n);
        for (int i = 0; i < n; i++) if (keep[i]) result.Add(pts[i]);
        return result;
    }

    /// <summary>
    /// Tính trục chính PCA (Principal Component Analysis) của tập điểm 2D để chọn hướng quét giảm số lượt (Mục P1.2 trang 7 file góp ý).
    /// </summary>
    private static void ComputePcaAxes2D(List<Vector2> pts, Vector2 fallbackMajor, out Vector2 centroid, out Vector2 majorAxis, out Vector2 minorAxis)
    {
        centroid = Vector2.zero;
        if (pts == null || pts.Count < 2)
        {
            majorAxis = fallbackMajor.sqrMagnitude > 1e-6f ? fallbackMajor.normalized : Vector2.up;
            minorAxis = new Vector2(-majorAxis.y, majorAxis.x);
            return;
        }
        for (int i = 0; i < pts.Count; i++) centroid += pts[i];
        centroid /= pts.Count;

        double cxx = 0.0, cxy = 0.0, cyy = 0.0;
        for (int i = 0; i < pts.Count; i++)
        {
            double dx = pts[i].x - centroid.x;
            double dy = pts[i].y - centroid.y;
            cxx += dx * dx;
            cxy += dx * dy;
            cyy += dy * dy;
        }
        double trace = cxx + cyy;
        double det = cxx * cyy - cxy * cxy;
        double disc = Math.Max(0.0, trace * trace * 0.25 - det);
        double lambda1 = trace * 0.5 + Math.Sqrt(disc);

        Vector2 v;
        if (Math.Abs(cxy) > 1e-8)
            v = new Vector2((float)(lambda1 - cyy), (float)cxy);
        else
            v = (cxx >= cyy) ? Vector2.right : Vector2.up;

        if (v.sqrMagnitude < 1e-8f) v = fallbackMajor;
        v.Normalize();
        if (fallbackMajor.sqrMagnitude > 1e-6f && Vector2.Dot(v, fallbackMajor) < 0f) v = -v;
        majorAxis = v;
        minorAxis = new Vector2(-majorAxis.y, majorAxis.x);
    }

    /// <summary>
    /// Trích xuất vùng mục tiêu giải phẫu R_leg = [sMin..sMax] x [wMin..wMax] trên từng ống quần từ mẫu vẽ `templateStrokesUV`.
    /// Nếu mẫu vẽ chỉ đặt trên đùi hoặc cả đùi-gối, P1 và P2 sẽ biết chính xác cần đặt vùng quét ở đâu trên ống quần mới!
    /// </summary>
    private static void ExtractLegAnatomicalRegionBounds(
        List<List<Vector2>> templateStrokesUV,
        Vector2[] src12,
        Vector2[] dst12XZ,
        out bool[] hasLegRegion,
        out float[] sMinLeg,
        out float[] sMaxLeg,
        out float[] wMinLeg,
        out float[] wMaxLeg,
        out List<Vector2>[] legRegionPolygonPoints)
    {
        hasLegRegion = new bool[2];
        sMinLeg = new float[2] { float.MaxValue, float.MaxValue };
        sMaxLeg = new float[2] { float.MinValue, float.MinValue };
        wMinLeg = new float[2] { float.MaxValue, float.MaxValue };
        wMaxLeg = new float[2] { float.MinValue, float.MinValue };
        legRegionPolygonPoints = new List<Vector2>[2] { new List<Vector2>(), new List<Vector2>() };

        AnatomicalLegChain srcChainL = BuildLegChain(src12, true);
        AnatomicalLegChain srcChainR = BuildLegChain(src12, false);
        AnatomicalLegChain dstChainL = BuildLegChain(dst12XZ, true);
        AnatomicalLegChain dstChainR = BuildLegChain(dst12XZ, false);

        int totalPts = 0;
        if (templateStrokesUV != null)
        {
            for (int s = 0; s < templateStrokesUV.Count; s++)
            {
                var raw = templateStrokesUV[s];
                if (raw == null || raw.Count < 2) continue;
                var resampled = ResampleUV(raw, 0.015f);
                for (int p = 0; p < resampled.Count; p++)
                {
                    Vector2 pt = resampled[p];
                    ProjectOntoLegChain(pt, srcChainL, out int segL, out float uL, out float wL, out float d2L);
                    ProjectOntoLegChain(pt, srcChainR, out int segR, out float uR, out float wR, out float d2R);

                    float absWL = Mathf.Abs(wL);
                    float absWR = Mathf.Abs(wR);
                    int leg = (absWL <= 1.15f || absWR <= 1.15f) ? (absWL <= absWR ? 0 : 1) : (d2L <= d2R ? 0 : 1);
                    float sVal = (leg == 0 ? segL + uL : segR + uR);
                    float wVal = (leg == 0 ? wL : wR);

                    hasLegRegion[leg] = true;
                    if (sVal < sMinLeg[leg]) sMinLeg[leg] = sVal;
                    if (sVal > sMaxLeg[leg]) sMaxLeg[leg] = sVal;
                    if (wVal < wMinLeg[leg]) wMinLeg[leg] = wVal;
                    if (wVal > wMaxLeg[leg]) wMaxLeg[leg] = wVal;
                    totalPts++;
                }
            }
        }

        // Nếu chưa có mẫu vẽ nào, mặc định vùng phun chuẩn là vùng đùi-gối trên cả 2 ống
        if (totalPts == 0)
        {
            for (int leg = 0; leg < 2; leg++)
            {
                hasLegRegion[leg] = true;
                sMinLeg[leg] = 1.15f;
                sMaxLeg[leg] = 3.70f;
                wMinLeg[leg] = -0.82f;
                wMaxLeg[leg] = 0.82f;
            }
        }

        for (int leg = 0; leg < 2; leg++)
        {
            if (!hasLegRegion[leg]) continue;
            // Mở rộng nhẹ biên bao giải phẫu để vùng quét liền mạch, phủ trọn các nét mẫu trên ống quần
            sMinLeg[leg] = Mathf.Clamp(sMinLeg[leg] - 0.08f, 0.85f, 3.85f);
            sMaxLeg[leg] = Mathf.Clamp(Mathf.Max(sMinLeg[leg] + 0.35f, sMaxLeg[leg] + 0.08f), 1.05f, 3.92f);
            wMinLeg[leg] = Mathf.Clamp(wMinLeg[leg] - 0.12f, -0.92f, 0.60f);
            wMaxLeg[leg] = Mathf.Clamp(Mathf.Max(wMinLeg[leg] + 0.45f, wMaxLeg[leg] + 0.12f), -0.60f, 0.92f);

            AnatomicalLegChain dstChain = (leg == 0) ? dstChainL : dstChainR;
            int nS = 10;
            int nW = 5;
            for (int i = 0; i <= nS; i++)
            {
                float sVal = Mathf.Lerp(sMinLeg[leg], sMaxLeg[leg], (float)i / nS);
                int seg = Mathf.Clamp(Mathf.FloorToInt(sVal), 0, 3);
                float u = sVal - seg;
                for (int j = 0; j <= nW; j++)
                {
                    float wVal = Mathf.Lerp(wMinLeg[leg], wMaxLeg[leg], (float)j / nW);
                    legRegionPolygonPoints[leg].Add(ReconstructOnLegChain(dstChain, seg, u, wVal));
                }
            }
        }
    }

    /// <summary>
    /// Kiểm tra điểm 2D `ptXZ` trên ống `leg` (0=trái, 1=phải) có nằm trong vùng phun mục tiêu `R_leg`
    /// (giao giữa vùng mẫu hiệu ứng giải phẫu `[sMin..sMax, wMin..wMax]` trên ống đích và biên quần Douglas-Peucker co khoảng `e`) hay không.
    /// </summary>
    private static bool IsPointInAnatomicalRegionR(
        Vector2 ptXZ,
        int leg,
        in AnatomicalLegChain dstChainL,
        in AnatomicalLegChain dstChainR,
        float sMin,
        float sMax,
        float wMin,
        float wMax,
        List<Vector3> contourWorld,
        List<Vector3> landmarksWorldPos,
        float borderMarginMm)
    {
        if (!IsInsideValidSprayZone(ptXZ.x, ptXZ.y, contourWorld, landmarksWorldPos, borderMarginMm, out _))
            return false;

        // Phân tách ô lồi Boustrophedon tại đáy đũng: điểm thuộc ống nào phải gần trục giữa ống đó hơn
        ProjectOntoLegChain(ptXZ, dstChainL, out int segL, out float uL, out float wL, out float d2L);
        ProjectOntoLegChain(ptXZ, dstChainR, out int segR, out float uR, out float wR, out float d2R);

        if (leg == 0)
        {
            if (d2L > d2R + 0.0004f && Mathf.Abs(wL) > Mathf.Abs(wR)) return false;
            float sVal = segL + uL;
            return (sVal >= sMin && sVal <= sMax && wL >= wMin && wL <= wMax);
        }
        else
        {
            if (d2R > d2L + 0.0004f && Mathf.Abs(wR) > Mathf.Abs(wL)) return false;
            float sVal = segR + uR;
            return (sVal >= sMin && sVal <= sMax && wR >= wMin && wR <= wMax);
        }
    }

    // =========================================================================
    // 2E. CÔNG CỤ HÌNH HỌC & MASK CHO THUẬT TOÁN P1 – QUÉT ZIGZAG ĐẦY ĐỦ 10 BƯỚC
    //     (Douglas-Peucker, Co biên e bằng EDT chuẩn, PCA / Min-Area Rect,
    //      Phân tách ô lồi Boustrophedon, Cắt đường song song bước s, Bật/Tắt súng)
    // =========================================================================

    private static bool IsPointInPolygonXZ(Vector2 pt, List<Vector2> poly)
    {
        if (poly == null || poly.Count < 3) return false;
        bool inside = false;
        int n = poly.Count;
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            Vector2 pi = poly[i];
            Vector2 pj = poly[j];
            if (((pi.y > pt.y) != (pj.y > pt.y)) &&
                (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y + 1e-9f) + pi.x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    private static float ComputePolygonAreaSqM(List<Vector2> poly)
    {
        if (poly == null || poly.Count < 3) return 0f;
        double sum = 0.0;
        int n = poly.Count;
        for (int i = 0; i < n; i++)
        {
            Vector2 a = poly[i];
            Vector2 b = poly[(i + 1) % n];
            sum += (double)a.x * b.y - (double)b.x * a.y;
        }
        return (float)(Math.Abs(sum) * 0.5);
    }

    private static List<Vector2> SimplifyClosedPolygonDouglasPeucker(List<Vector2> poly, float epsilonM)
    {
        if (poly == null || poly.Count <= 4 || epsilonM <= 1e-5f)
            return poly != null ? new List<Vector2>(poly) : new List<Vector2>();

        // Chia vòng kín tại 2 điểm xa nhau nhất rồi áp Douglas-Peucker lên từng nửa để bảo toàn hình kín
        int n = poly.Count;
        int idxA = 0, idxB = n / 2;
        float bestD2 = -1f;
        for (int i = 0; i < n; i++)
        {
            float d2 = (poly[i] - poly[0]).sqrMagnitude;
            if (d2 > bestD2) { bestD2 = d2; idxB = i; }
        }
        if (idxB <= 1 || idxB >= n - 1) idxB = n / 2;

        var half1 = new List<Vector2>(idxB - idxA + 1);
        for (int i = idxA; i <= idxB; i++) half1.Add(poly[i]);
        var half2 = new List<Vector2>(n - idxB + 1);
        for (int i = idxB; i < n; i++) half2.Add(poly[i]);
        half2.Add(poly[idxA]);

        var s1 = SimplifyPolygonDouglasPeucker(half1, epsilonM);
        var s2 = SimplifyPolygonDouglasPeucker(half2, epsilonM);

        var merged = new List<Vector2>(s1.Count + s2.Count);
        for (int i = 0; i < s1.Count; i++) merged.Add(s1[i]);
        for (int i = 1; i < s2.Count - 1; i++) merged.Add(s2[i]);
        return merged.Count >= 3 ? merged : new List<Vector2>(poly);
    }

    private static List<Vector2> ComputeConvexHull2D(List<Vector2> pts)
    {
        if (pts == null || pts.Count < 3) return pts != null ? new List<Vector2>(pts) : new List<Vector2>();
        var sorted = new List<Vector2>(pts);
        sorted.Sort((a, b) => Math.Abs(a.x - b.x) > 1e-6f ? a.x.CompareTo(b.x) : a.y.CompareTo(b.y));

        var lower = new List<Vector2>();
        for (int i = 0; i < sorted.Count; i++)
        {
            Vector2 p = sorted[i];
            while (lower.Count >= 2)
            {
                Vector2 o = lower[lower.Count - 2];
                Vector2 a = lower[lower.Count - 1];
                if ((a.x - o.x) * (p.y - o.y) - (a.y - o.y) * (p.x - o.x) <= 1e-9f) lower.RemoveAt(lower.Count - 1);
                else break;
            }
            lower.Add(p);
        }

        var upper = new List<Vector2>();
        for (int i = sorted.Count - 1; i >= 0; i--)
        {
            Vector2 p = sorted[i];
            while (upper.Count >= 2)
            {
                Vector2 o = upper[upper.Count - 2];
                Vector2 a = upper[upper.Count - 1];
                if ((a.x - o.x) * (p.y - o.y) - (a.y - o.y) * (p.x - o.x) <= 1e-9f) upper.RemoveAt(upper.Count - 1);
                else break;
            }
            upper.Add(p);
        }

        lower.RemoveAt(lower.Count - 1);
        upper.RemoveAt(upper.Count - 1);
        lower.AddRange(upper);
        return lower.Count >= 3 ? lower : new List<Vector2>(pts);
    }

    /// <summary>
    /// Bước 4 P1: Tìm hướng quét bằng Hình chữ nhật bao nhỏ nhất (Minimum-Area Bounding Rectangle) trên bao lồi.
    /// Trục majorAxis nằm theo cạnh dài nhất (giảm số lượt quét), minorAxis vuông góc (hướng dịch bước s).
    /// </summary>
    private static void ComputeMinAreaRectAxes2D(List<Vector2> pts, Vector2 fallbackMajor, out Vector2 centroid, out Vector2 majorAxis, out Vector2 minorAxis)
    {
        centroid = Vector2.zero;
        if (pts == null || pts.Count < 3)
        {
            ComputePcaAxes2D(pts, fallbackMajor, out centroid, out majorAxis, out minorAxis);
            return;
        }
        for (int i = 0; i < pts.Count; i++) centroid += pts[i];
        centroid /= pts.Count;

        var hull = ComputeConvexHull2D(pts);
        float bestArea = float.MaxValue;
        majorAxis = fallbackMajor.sqrMagnitude > 1e-6f ? fallbackMajor.normalized : Vector2.up;
        minorAxis = new Vector2(-majorAxis.y, majorAxis.x);

        int hn = hull.Count;
        for (int i = 0; i < hn; i++)
        {
            Vector2 edge = hull[(i + 1) % hn] - hull[i];
            if (edge.sqrMagnitude < 1e-10f) continue;
            Vector2 axU = edge.normalized;
            Vector2 axV = new Vector2(-axU.y, axU.x);

            float minU = float.MaxValue, maxU = float.MinValue;
            float minV = float.MaxValue, maxV = float.MinValue;
            for (int k = 0; k < hn; k++)
            {
                float u = Vector2.Dot(hull[k], axU);
                float v = Vector2.Dot(hull[k], axV);
                if (u < minU) minU = u; if (u > maxU) maxU = u;
                if (v < minV) minV = v; if (v > maxV) maxV = v;
            }
            float spanU = maxU - minU;
            float spanV = maxV - minV;
            float area = spanU * spanV;
            if (area < bestArea)
            {
                bestArea = area;
                if (spanU >= spanV)
                {
                    majorAxis = axU;
                    minorAxis = axV;
                }
                else
                {
                    majorAxis = axV;
                    minorAxis = new Vector2(-axV.y, axV.x);
                }
            }
        }
        if (fallbackMajor.sqrMagnitude > 1e-6f && Vector2.Dot(majorAxis, fallbackMajor) < 0f)
        {
            majorAxis = -majorAxis;
            minorAxis = -minorAxis;
        }
    }

    // 1D Squared Euclidean Distance Transform (Felzenszwalb & Huttenlocher, O(n))
    private static void Edt1D(float[] f, float[] d, int n, int[] v, float[] z)
    {
        int k = 0;
        v[0] = 0;
        z[0] = float.NegativeInfinity;
        z[1] = float.PositiveInfinity;
        for (int q = 1; q < n; q++)
        {
            float s;
            while (true)
            {
                int r = v[k];
                s = ((f[q] + q * q) - (f[r] + r * r)) / (2f * (q - r));
                if (s <= z[k])
                {
                    k--;
                    if (k < 0) { k = 0; break; }
                }
                else break;
            }
            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = float.PositiveInfinity;
        }
        k = 0;
        for (int q = 0; q < n; q++)
        {
            while (z[k + 1] < q) k++;
            int r = v[k];
            float diff = q - r;
            d[q] = diff * diff + f[r];
        }
    }

    /// <summary>
    /// Bước 3 P1: Co vùng phun R vào trong khoảng e (mm) bằng biến đổi khoảng cách Euclid chính xác (Exact EDT)
    /// trên lưới Mask raster và trích xuất đa giác biên R_safe (mỗi thành phần liên thông -> 1 đa giác kín sạch).
    /// </summary>
    private static List<List<Vector2>> ErodePolygonByDistanceE2D(
        List<Vector2> rawPoly,
        float erosionEM,
        List<Vector2> garmentClipPoly,
        float dpEpsM,
        out float rawAreaSqM,
        out float safeAreaSqM)
    {
        rawAreaSqM = ComputePolygonAreaSqM(rawPoly);
        safeAreaSqM = 0f;
        var safePolys = new List<List<Vector2>>();
        if (rawPoly == null || rawPoly.Count < 3) return safePolys;

        if (erosionEM <= 0.0005f)
        {
            safeAreaSqM = rawAreaSqM;
            safePolys.Add(new List<Vector2>(rawPoly));
            return safePolys;
        }

        float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
        for (int i = 0; i < rawPoly.Count; i++)
        {
            if (rawPoly[i].x < minX) minX = rawPoly[i].x;
            if (rawPoly[i].x > maxX) maxX = rawPoly[i].x;
            if (rawPoly[i].y < minY) minY = rawPoly[i].y;
            if (rawPoly[i].y > maxY) maxY = rawPoly[i].y;
        }

        // Độ phân giải lưới raster k pixel/mm (~3 mm/ô, tự co nhẹ nếu vùng quá lớn để luôn mượt < 2ms)
        float spanX = Mathf.Max(0.01f, maxX - minX);
        float spanY = Mathf.Max(0.01f, maxY - minY);
        float cell = Mathf.Clamp(Mathf.Max(spanX, spanY) / 180f, 0.0025f, 0.0045f);
        int pad = 3;
        int gw = Mathf.Clamp(Mathf.CeilToInt(spanX / cell) + pad * 2, 10, 260);
        int gh = Mathf.Clamp(Mathf.CeilToInt(spanY / cell) + pad * 2, 10, 260);
        float origX = minX - pad * cell;
        float origY = minY - pad * cell;

        bool[,] maskRaw = new bool[gw, gh];
        const float INF = 1e12f;
        float[,] distGrid = new float[gw, gh];
        int rawCount = 0;
        bool useClip = (garmentClipPoly != null && garmentClipPoly.Count >= 4);

        for (int y = 0; y < gh; y++)
        {
            float wy = origY + (y + 0.5f) * cell;
            for (int x = 0; x < gw; x++)
            {
                float wx = origX + (x + 0.5f) * cell;
                Vector2 p = new Vector2(wx, wy);
                bool inside = IsPointInPolygonXZ(p, rawPoly);
                if (inside && useClip && !IsPointInPolygonXZ(p, garmentClipPoly))
                    inside = false;
                maskRaw[x, y] = inside;
                if (inside) rawCount++;
                distGrid[x, y] = inside ? INF : 0f;
            }
        }

        // Nếu clip theo garmentClipPoly làm rỗng (ví dụ mẫu hình tròn thử nghiệm đặt ra ngoài contour), bỏ clip garment
        if (rawCount == 0 && useClip)
        {
            for (int y = 0; y < gh; y++)
            {
                float wy = origY + (y + 0.5f) * cell;
                for (int x = 0; x < gw; x++)
                {
                    float wx = origX + (x + 0.5f) * cell;
                    bool inside = IsPointInPolygonXZ(new Vector2(wx, wy), rawPoly);
                    maskRaw[x, y] = inside;
                    if (inside) rawCount++;
                    distGrid[x, y] = inside ? INF : 0f;
                }
            }
        }

        rawAreaSqM = rawCount * cell * cell;
        if (rawCount == 0) return safePolys;

        // Pass 1 (cột) + Pass 2 (hàng) -> Khoảng cách Euclid chính xác tới biên ngoài của Mask R
        int maxDim = Mathf.Max(gw, gh);
        float[] fBuf = new float[maxDim];
        float[] dBuf = new float[maxDim];
        int[] vBuf = new int[maxDim];
        float[] zBuf = new float[maxDim + 1];

        for (int x = 0; x < gw; x++)
        {
            for (int y = 0; y < gh; y++) fBuf[y] = distGrid[x, y];
            Edt1D(fBuf, dBuf, gh, vBuf, zBuf);
            for (int y = 0; y < gh; y++) distGrid[x, y] = dBuf[y];
        }
        for (int y = 0; y < gh; y++)
        {
            for (int x = 0; x < gw; x++) fBuf[x] = distGrid[x, y];
            Edt1D(fBuf, dBuf, gw, vBuf, zBuf);
            for (int x = 0; x < gw; x++) distGrid[x, y] = dBuf[x];
        }

        float ePixelsSq = (erosionEM / cell) * (erosionEM / cell);
        bool[,] maskSafe = new bool[gw, gh];
        int safeCount = 0;
        for (int y = 1; y < gh - 1; y++)
        {
            for (int x = 1; x < gw - 1; x++)
            {
                if (maskRaw[x, y] && distGrid[x, y] >= ePixelsSq)
                {
                    maskSafe[x, y] = true;
                    safeCount++;
                }
            }
        }
        safeAreaSqM = safeCount * cell * cell;
        if (safeCount < 3) return safePolys;

        // Trích xuất từng thành phần liên thông của R_safe rồi lần biên (Moore-Neighbor contour tracing)
        int[,] compId = new int[gw, gh];
        int curComp = 0;
        var queue = new Queue<Vector2Int>();
        int[] dx4 = { 1, -1, 0, 0 };
        int[] dy4 = { 0, 0, 1, -1 };

        for (int y = 1; y < gh - 1; y++)
        {
            for (int x = 1; x < gw - 1; x++)
            {
                if (!maskSafe[x, y] || compId[x, y] != 0) continue;
                curComp++;
                int compPixels = 0;
                queue.Clear();
                queue.Enqueue(new Vector2Int(x, y));
                compId[x, y] = curComp;
                int startX = x, startY = y;

                while (queue.Count > 0)
                {
                    Vector2Int c = queue.Dequeue();
                    compPixels++;
                    if (c.y < startY || (c.y == startY && c.x < startX)) { startX = c.x; startY = c.y; }
                    for (int d = 0; d < 4; d++)
                    {
                        int nx = c.x + dx4[d], ny = c.y + dy4[d];
                        if (nx > 0 && nx < gw - 1 && ny > 0 && ny < gh - 1 && maskSafe[nx, ny] && compId[nx, ny] == 0)
                        {
                            compId[nx, ny] = curComp;
                            queue.Enqueue(new Vector2Int(nx, ny));
                        }
                    }
                }

                if (compPixels < 4) continue;

                // Moore-Neighbor tracing theo chiều kim đồng hồ xung quanh thành phần liên thông curComp
                int[] mdx = { -1, 0, 1, 1, 1, 0, -1, -1 };
                int[] mdy = { 0, -1, -1, 0, 1, 1, 1, 0 };
                var traced = new List<Vector2>();
                int cx = startX, cy = startY;
                int backDir = 0; // bên trái của (startX, startY) chắc chắn ngoài comp
                int maxSteps = compPixels * 4 + 32;

                for (int step = 0; step < maxSteps; step++)
                {
                    traced.Add(new Vector2(origX + (cx + 0.5f) * cell, origY + (cy + 0.5f) * cell));
                    bool foundNext = false;
                    int searchStart = (backDir + 1) % 8;
                    for (int k = 0; k < 8; k++)
                    {
                        int dir = (searchStart + k) % 8;
                        int nx = cx + mdx[dir];
                        int ny = cy + mdy[dir];
                        if (nx >= 0 && nx < gw && ny >= 0 && ny < gh && compId[nx, ny] == curComp)
                        {
                            cx = nx; cy = ny;
                            backDir = (dir + 5) % 8; // quay lại ô trống ngay trước hướng đi vào
                            foundNext = true;
                            break;
                        }
                    }
                    if (!foundNext || (cx == startX && cy == startY && step > 1)) break;
                }

                if (traced.Count >= 3)
                {
                    // Làm mượt nhẹ (1 pass Laplacian) rồi đơn giản hóa bằng Douglas-Peucker
                    var smoothed = new List<Vector2>(traced.Count);
                    for (int i = 0; i < traced.Count; i++)
                    {
                        Vector2 prev = traced[(i - 1 + traced.Count) % traced.Count];
                        Vector2 curr = traced[i];
                        Vector2 next = traced[(i + 1) % traced.Count];
                        smoothed.Add(0.25f * prev + 0.50f * curr + 0.25f * next);
                    }
                    var simple = SimplifyClosedPolygonDouglasPeucker(smoothed, Mathf.Max(0.002f, dpEpsM * 0.65f));
                    if (simple.Count >= 3 && ComputePolygonAreaSqM(simple) > 1e-5f)
                    {
                        safePolys.Add(simple);
                    }
                }
            }
        }
        return safePolys;
    }

    private static List<Vector2> BuildCirclePolygon2D(Vector2 center, float radiusM, int segments = 48)
    {
        var poly = new List<Vector2>(segments);
        if (radiusM <= 1e-4f) return poly;
        for (int i = 0; i < segments; i++)
        {
            float ang = (2f * Mathf.PI * i) / segments;
            poly.Add(center + new Vector2(Mathf.Cos(ang) * radiusM, Mathf.Sin(ang) * radiusM));
        }
        return poly;
    }

    private static List<Vector2> BuildOrientedRectPolygon2D(Vector2 center, Vector2 longAxis, float widthM, float lengthM)
    {
        var poly = new List<Vector2>(4);
        if (widthM <= 1e-4f || lengthM <= 1e-4f) return poly;
        Vector2 u = longAxis.sqrMagnitude > 1e-6f ? longAxis.normalized : Vector2.up;
        Vector2 v = new Vector2(-u.y, u.x);
        float hw = widthM * 0.5f;
        float hl = lengthM * 0.5f;
        poly.Add(center - u * hl - v * hw);
        poly.Add(center + u * hl - v * hw);
        poly.Add(center + u * hl + v * hw);
        poly.Add(center - u * hl + v * hw);
        return poly;
    }

    private static List<Vector2> BuildAnatomicalLegStripPolygon2D(in AnatomicalLegChain chain, float sMin, float sMax, float wMin, float wMax)
    {
        var poly = new List<Vector2>(28);
        int nSteps = 10;
        for (int i = 0; i <= nSteps; i++)
        {
            float sVal = Mathf.Lerp(sMin, sMax, (float)i / nSteps);
            int seg = Mathf.Clamp(Mathf.FloorToInt(sVal), 0, 3);
            poly.Add(ReconstructOnLegChain(chain, seg, sVal - seg, wMin));
        }
        for (int i = nSteps; i >= 0; i--)
        {
            float sVal = Mathf.Lerp(sMin, sMax, (float)i / nSteps);
            int seg = Mathf.Clamp(Mathf.FloorToInt(sVal), 0, 3);
            poly.Add(ReconstructOnLegChain(chain, seg, sVal - seg, wMax));
        }
        return poly;
    }

    /// <summary>
    /// Bước 6 P1: Cắt chính xác đa giác kín `poly` (đã chuyển sang hệ tọa độ (u, v) theo majorAxis/minorAxis)
    /// bằng đường thẳng quét song song tại `v = vLine`. Trả về danh sách các giao đoạn hợp lệ `[uStart, uEnd]`.
    /// </summary>
    private static List<Vector2> IntersectPolygonWithScanlineUV(List<Vector2> polyUV, float vLine)
    {
        var hitsU = new List<float>(8);
        int n = polyUV.Count;
        for (int i = 0; i < n; i++)
        {
            Vector2 a = polyUV[i];
            Vector2 b = polyUV[(i + 1) % n];
            // Dùng quy ước half-open [min, max) để không đếm đúp tại đỉnh
            if ((a.y <= vLine && b.y > vLine) || (b.y <= vLine && a.y > vLine))
            {
                float t = (vLine - a.y) / (b.y - a.y);
                hitsU.Add(a.x + t * (b.x - a.x));
            }
        }
        hitsU.Sort();
        var intervals = new List<Vector2>(hitsU.Count / 2);
        for (int i = 0; i + 1 < hitsU.Count; i += 2)
        {
            float u0 = hitsU[i];
            float u1 = hitsU[i + 1];
            if (u1 - u0 >= 0.004f) // bỏ các đoạn vi mô < 4 mm
            {
                intervals.Add(new Vector2(u0, u1));
            }
        }
        return intervals;
    }

    private struct BoustrophedonSpan
    {
        public float v;
        public float u0;
        public float u1;
    }

    private static List<Vector3> LiftPolygon2DToWorldSurface(List<Vector2> poly2D, DobotCR10AIKController ctrl, PLYDirectLoader plyLoader, bool useFlatPlane, float flatY)
    {
        var res = new List<Vector3>(poly2D.Count);
        for (int i = 0; i < poly2D.Count; i++)
        {
            float y = flatY;
            if (!useFlatPlane)
            {
                if (ctrl.SampleCompensatedSurface(poly2D[i].x, poly2D[i].y, out float cy, out _)) y = cy;
                else if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(poly2D[i].x, poly2D[i].y, out float py, out _)) y = py;
            }
            res.Add(new Vector3(poly2D[i].x, y + 0.002f, poly2D[i].y));
        }
        return res;
    }

    private struct TypedSurfacePoint
    {
        public Vector3 worldPos;
        public bool sprayOnToHere;
        public string segName;
    }

    /// <summary>
    /// Chuyển các nét vẽ hở của người dùng thành đa giác Mask R chính xác theo đúng hình dáng nét vẽ
    /// (thay vì ép thành hình chữ nhật lớn bao cả ống quần).
    /// </summary>
    private static List<List<Vector2>> BuildMaskPolygonsFromOpenStrokes2D(List<List<Vector2>> strokes2D, float brushRadiusM, float dpEpsM)
    {
        var polys = new List<List<Vector2>>();
        if (strokes2D == null || strokes2D.Count == 0) return polys;

        float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
        int validSegs = 0;
        for (int s = 0; s < strokes2D.Count; s++)
        {
            var st = strokes2D[s];
            if (st == null || st.Count < 2) continue;
            validSegs += st.Count - 1;
            for (int p = 0; p < st.Count; p++)
            {
                if (st[p].x < minX) minX = st[p].x; if (st[p].x > maxX) maxX = st[p].x;
                if (st[p].y < minY) minY = st[p].y; if (st[p].y > maxY) maxY = st[p].y;
            }
        }
        if (validSegs == 0) return polys;

        float rBrush = Mathf.Clamp(brushRadiusM, 0.008f, 0.060f);
        float spanX = Mathf.Max(0.02f, (maxX - minX) + 2f * rBrush);
        float spanY = Mathf.Max(0.02f, (maxY - minY) + 2f * rBrush);
        float cell = Mathf.Clamp(Mathf.Max(spanX, spanY) / 180f, 0.0025f, 0.0045f);
        int pad = 3;
        int gw = Mathf.Clamp(Mathf.CeilToInt(spanX / cell) + pad * 2, 12, 260);
        int gh = Mathf.Clamp(Mathf.CeilToInt(spanY / cell) + pad * 2, 12, 260);
        float origX = minX - rBrush - pad * cell;
        float origY = minY - rBrush - pad * cell;

        bool[,] mask = new bool[gw, gh];
        float r2 = rBrush * rBrush;
        int rCells = Mathf.CeilToInt(rBrush / cell) + 1;

        for (int s = 0; s < strokes2D.Count; s++)
        {
            var st = strokes2D[s];
            if (st == null || st.Count < 2) continue;
            for (int p = 0; p < st.Count - 1; p++)
            {
                Vector2 a = st[p];
                Vector2 b = st[p + 1];
                int ix0 = Mathf.Clamp(Mathf.FloorToInt((Mathf.Min(a.x, b.x) - origX) / cell) - rCells, 1, gw - 2);
                int ix1 = Mathf.Clamp(Mathf.CeilToInt((Mathf.Max(a.x, b.x) - origX) / cell) + rCells, 1, gw - 2);
                int iy0 = Mathf.Clamp(Mathf.FloorToInt((Mathf.Min(a.y, b.y) - origY) / cell) - rCells, 1, gh - 2);
                int iy1 = Mathf.Clamp(Mathf.CeilToInt((Mathf.Max(a.y, b.y) - origY) / cell) + rCells, 1, gh - 2);

                for (int y = iy0; y <= iy1; y++)
                {
                    float wy = origY + (y + 0.5f) * cell;
                    for (int x = ix0; x <= ix1; x++)
                    {
                        if (mask[x, y]) continue;
                        float wx = origX + (x + 0.5f) * cell;
                        float d = DistToSegment2D(new Vector2(wx, wy), a, b);
                        if (d * d <= r2) mask[x, y] = true;
                    }
                }
            }
        }

        // Lần biên từng thành phần liên thông của Mask nét vẽ
        int[,] compId = new int[gw, gh];
        int curComp = 0;
        var queue = new Queue<Vector2Int>();
        int[] dx4 = { 1, -1, 0, 0 };
        int[] dy4 = { 0, 0, 1, -1 };
        int[] mdx = { -1, 0, 1, 1, 1, 0, -1, -1 };
        int[] mdy = { 0, -1, -1, 0, 1, 1, 1, 0 };

        for (int y = 1; y < gh - 1; y++)
        {
            for (int x = 1; x < gw - 1; x++)
            {
                if (!mask[x, y] || compId[x, y] != 0) continue;
                curComp++;
                int compPixels = 0;
                queue.Clear();
                queue.Enqueue(new Vector2Int(x, y));
                compId[x, y] = curComp;
                int startX = x, startY = y;

                while (queue.Count > 0)
                {
                    Vector2Int c = queue.Dequeue();
                    compPixels++;
                    if (c.y < startY || (c.y == startY && c.x < startX)) { startX = c.x; startY = c.y; }
                    for (int d = 0; d < 4; d++)
                    {
                        int nx = c.x + dx4[d], ny = c.y + dy4[d];
                        if (nx > 0 && nx < gw - 1 && ny > 0 && ny < gh - 1 && mask[nx, ny] && compId[nx, ny] == 0)
                        {
                            compId[nx, ny] = curComp;
                            queue.Enqueue(new Vector2Int(nx, ny));
                        }
                    }
                }
                if (compPixels < 6) continue;

                var traced = new List<Vector2>();
                int cx = startX, cy = startY;
                int backDir = 0;
                int maxSteps = compPixels * 4 + 32;
                for (int step = 0; step < maxSteps; step++)
                {
                    traced.Add(new Vector2(origX + (cx + 0.5f) * cell, origY + (cy + 0.5f) * cell));
                    bool foundNext = false;
                    int searchStart = (backDir + 1) % 8;
                    for (int k = 0; k < 8; k++)
                    {
                        int dir = (searchStart + k) % 8;
                        int nx = cx + mdx[dir];
                        int ny = cy + mdy[dir];
                        if (nx >= 0 && nx < gw && ny >= 0 && ny < gh && compId[nx, ny] == curComp)
                        {
                            cx = nx; cy = ny;
                            backDir = (dir + 5) % 8;
                            foundNext = true;
                            break;
                        }
                    }
                    if (!foundNext || (cx == startX && cy == startY && step > 1)) break;
                }

                if (traced.Count >= 3)
                {
                    var smoothed = new List<Vector2>(traced.Count);
                    for (int i = 0; i < traced.Count; i++)
                    {
                        Vector2 prev = traced[(i - 1 + traced.Count) % traced.Count];
                        Vector2 curr = traced[i];
                        Vector2 next = traced[(i + 1) % traced.Count];
                        smoothed.Add(0.25f * prev + 0.50f * curr + 0.25f * next);
                    }
                    var simple = SimplifyClosedPolygonDouglasPeucker(smoothed, Mathf.Max(0.002f, dpEpsM * 0.7f));
                    if (simple.Count >= 3 && ComputePolygonAreaSqM(simple) > 2e-5f)
                        polys.Add(simple);
                }
            }
        }
        return polys;
    }

    private static List<DobotCR10AIKController.WaypointData> BuildSprayWaypointsFromTypedPolyline(
        List<TypedSurfacePoint> typedPath,
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        SprayTechParams tech,
        bool useFixedVerticalNormal,
        bool useFlatPlaneZ,
        float flatPlaneWorldY,
        float overrideStandoffMm)
    {
        var result = new List<DobotCR10AIKController.WaypointData>();
        if (typedPath == null || typedPath.Count < 2 || ctrl == null) return result;

        float effectiveStandoffMm = (overrideStandoffMm >= 0f) ? overrideStandoffMm : GetEffectiveSprayHeightMm(ctrl, tech);
        float standoffM = effectiveStandoffMm * 0.001f;
        Transform gRef = ctrl.globalReference;
        int m = typedPath.Count;

        Vector3[] surfPtsWorld = new Vector3[m];
        Vector3[] surfNormsWorld = new Vector3[m];
        for (int p = 0; p < m; p++)
        {
            Vector3 pt = typedPath[p].worldPos;
            Vector3 nrm = Vector3.up;
            if (useFlatPlaneZ)
            {
                pt.y = flatPlaneWorldY;
                nrm = Vector3.up;
            }
            else if (ctrl.SampleCompensatedSurface(pt.x, pt.z, out float cy, out Vector3 cn))
            {
                pt.y = cy;
                nrm = useFixedVerticalNormal ? Vector3.up : cn;
            }
            else if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(pt.x, pt.z, out float py, out Vector3 pn))
            {
                pt.y = py;
                nrm = useFixedVerticalNormal ? Vector3.up : pn;
            }
            if (nrm.y < 0f) nrm = -nrm;
            surfPtsWorld[p] = pt;
            surfNormsWorld[p] = nrm.normalized;
        }

        Vector3[] smoothNormsWorld = new Vector3[m];
        for (int p = 0; p < m; p++)
        {
            Vector3 sum = surfNormsWorld[p] * 2f;
            if (p > 0) sum += surfNormsWorld[p - 1];
            if (p < m - 1) sum += surfNormsWorld[p + 1];
            smoothNormsWorld[p] = sum.normalized;
        }

        float cumDistMm = 0f;
        Vector3 prevTcpRobotMm = Vector3.zero;
        float[] prevJoints = null;
        float maxTilt = tech != null ? tech.maxTiltDeg : 28f;

        for (int p = 0; p < m; p++)
        {
            Vector3 surfW = surfPtsWorld[p];
            Vector3 normW = smoothNormsWorld[p];
            float tiltDeg = Vector3.Angle(normW, Vector3.up);
            if (tiltDeg > maxTilt && maxTilt > 0.1f)
            {
                Vector3 axis = Vector3.Cross(Vector3.up, normW);
                if (axis.sqrMagnitude > 1e-6f)
                    normW = (Quaternion.AngleAxis(maxTilt, axis.normalized) * Vector3.up).normalized;
            }

            Vector3 tcpW = surfW + normW * standoffM;
            Vector3 tcpRobotMm = gRef != null
                ? gRef.InverseTransformPoint(tcpW) * 1000f
                : new Vector3((1.100f - tcpW.z) * 1000f, -tcpW.x * 1000f, tcpW.y * 1000f);
            tcpRobotMm.z = Mathf.Clamp(tcpRobotMm.z, 18f, 550f);
            if (tcpRobotMm.magnitude > 1145f)
            {
                float rScale = 1140f / tcpRobotMm.magnitude;
                tcpRobotMm.x *= rScale;
                tcpRobotMm.y *= rScale;
                tcpW = gRef != null
                    ? gRef.TransformPoint(tcpRobotMm * 0.001f)
                    : new Vector3(-tcpRobotMm.y * 0.001f, tcpRobotMm.z * 0.001f, 1.100f - tcpRobotMm.x * 0.001f);
            }

            Vector3 normRobot = gRef != null
                ? gRef.InverseTransformDirection(normW).normalized
                : new Vector3(-normW.z, -normW.x, normW.y).normalized;
            Vector3 tanW = (p < m - 1) ? (surfPtsWorld[p + 1] - surfPtsWorld[p]) : (surfPtsWorld[p] - surfPtsWorld[p - 1]);
            if (tanW.sqrMagnitude < 1e-7f) tanW = Vector3.forward;
            Vector3 tanRobot = gRef != null
                ? gRef.InverseTransformDirection(tanW).normalized
                : new Vector3(-tanW.z, -tanW.x, tanW.y).normalized;

            Vector3 rpyDeg = DobotCR10AKinematics.SurfaceNormalToRobotRpyDeg(normRobot, tanRobot, maxTilt);

            if (p > 0) cumDistMm += Vector3.Distance(prevTcpRobotMm, tcpRobotMm);
            prevTcpRobotMm = tcpRobotMm;

            var wp = new DobotCR10AIKController.WaypointData
            {
                index = p,
                distance_mm = cumDistMm,
                targetPosGlobal_mm = tcpRobotMm,
                targetPosWorld_m = tcpW,
                microNormal = normW,
                roll_deg = rpyDeg.x,
                pitch_deg = rpyDeg.y,
                yaw_deg = rpyDeg.z,
                sprayOn = (p > 0) && typedPath[p].sprayOnToHere,
                segmentName = typedPath[p].segName
            };

            Quaternion qRot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
            var sols = DobotCR10AKinematics.SolveTcpPose(
                wp.targetPosGlobal_mm, qRot, prevJoints,
                ctrl.toolLength, useAdaptiveTilt: true, maxTiltDeg: maxTilt,
                toolOffsetX: ctrl.toolOffsetX, toolOffsetY: ctrl.toolOffsetY);
            if (sols == null || sols.Count == 0)
            {
                wp.roll_deg = 180f; wp.pitch_deg = 0f; wp.yaw_deg = 0f;
                qRot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                sols = DobotCR10AKinematics.SolveTcpPose(
                    wp.targetPosGlobal_mm, qRot, prevJoints,
                    ctrl.toolLength, useAdaptiveTilt: true, maxTiltDeg: maxTilt,
                    toolOffsetX: ctrl.toolOffsetX, toolOffsetY: ctrl.toolOffsetY);
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
                prevJoints = wp.jointAngles_rad;
            }
            result.Add(wp);
        }

        return result;
    }

    private static void AppendSampledLinePoints(
        List<TypedSurfacePoint> list,
        Vector2 startXZ,
        Vector2 endXZ,
        bool sprayOn,
        string segName,
        float stepM,
        float defaultY,
        bool includeStart)
    {
        float len = Vector2.Distance(startXZ, endXZ);
        int nSteps = Mathf.Clamp(Mathf.CeilToInt(len / Mathf.Max(0.005f, stepM)), 1, 60);
        int kStart = includeStart ? 0 : 1;
        for (int k = kStart; k <= nSteps; k++)
        {
            float t = (float)k / nSteps;
            Vector2 p = Vector2.Lerp(startXZ, endXZ, t);
            list.Add(new TypedSurfacePoint
            {
                worldPos = new Vector3(p.x, defaultY, p.y),
                sprayOnToHere = (k == 0) ? false : sprayOn,
                segName = segName
            });
        }
    }

    /// <summary>
    /// Overload tương thích ngược cho bộ so sánh P1 / P2 / P3 / P4.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP1Zigzag(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        out float genTimeSec)
    {
        var cfg = ctrl != null && ctrl.p1Config != null ? ctrl.p1Config : new P1ZigzagConfig
        {
            erosionEMm = tech != null ? tech.borderMarginMm : 12f,
            sprayWidthWMm = tech != null ? tech.sprayWidthMm : 80f,
            overlapRatioR = tech != null ? tech.overlapRatio : 0.40f,
            sprayDistanceDMm = GetEffectiveSprayHeightMm(ctrl, tech),
            autoSpacingFromWidthOverlap = true
        };
        return GenerateP1Zigzag(ctrl, plyLoader, templateStrokesUV, templateSize, tech, cfg, out _, out genTimeSec);
    }

    /// <summary>
    /// P1 — Quét Zigzag / Raster kiểu máy in 3D trên chỗ cần phun R (Đầy đủ 10 bước chuẩn tài liệu P1):
    /// 1. Nhận chỗ cần phun R (hình/vòng tròn/nét vẽ thực tế của người dùng, 2 ống AI, hình tròn hoặc HCN kiểm chứng).
    /// 2. Đơn giản hóa biên bằng Douglas-Peucker (ngưỡng eps mm).
    /// 3. Co vùng phun R vào trong khoảng e -> R_safe (phân tách rõ R gốc và R_safe để hiển thị độc lập trong View).
    /// 4. Chọn hướng quét tối ưu (PCA trục dài / HCN bao nhỏ nhất / góc tùy chỉnh).
    /// 5. Phân tách vùng lõm thành các ô lồi (Boustrophedon).
    /// 6-7. Quét đường thẳng song song cách bước s qua lại kiểu Zigzag:
    ///      - Kiểu Máy in 3D (mặc định): Đầu phun chạy Zigzag qua lại liên tục trên toàn khung bao vùng phun;
    ///        đi qua chỗ cần phun (R_safe) thì BẬT SÚNG (sprayOn=true), đi ngoài vùng hoặc quay đầu thì TẮT SÚNG (sprayOn=false).
    ///      - Kiểu Cắt gọn R_safe: Chỉ đi trong giao đoạn R_safe + nối quay đầu tắt súng.
    /// 8-10. Gán Z (2.5D từ Point Cloud / 2D mặt phẳng), hướng súng, lùi d, giải IK 6-DOF.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP1Zigzag(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        P1ZigzagConfig p1Cfg,
        out P1ExecutionResult p1Result,
        out float genTimeSec)
    {
        float t0 = Time.realtimeSinceStartup;
        p1Result = new P1ExecutionResult();
        var strokesWorld = new List<List<Vector3>>();

        if (ctrl == null)
        {
            p1Result.warningMessage = "Chưa có bộ điều khiển phôi.";
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        if (p1Cfg == null) p1Cfg = new P1ZigzagConfig();
        float erosionEM = Mathf.Max(0f, p1Cfg.erosionEMm) * 0.001f;
        float stepSM = Mathf.Clamp(p1Cfg.EffectiveSpacingSMm, 4f, 120f) * 0.001f;
        float dpEpsM = Mathf.Clamp(p1Cfg.douglasPeuckerEpsMm, 0.5f, 20f) * 0.001f;
        float overscanM = Mathf.Clamp(p1Cfg.turnaroundOverscanMm, 4f, 50f) * 0.001f;
        float sampleStepM = 0.015f;
        p1Result.effectiveSpacingMm = stepSM * 1000f;
        p1Result.erosionEMm = p1Cfg.erosionEMm;

        bool has12Landmarks = (ctrl.landmarksWorldPos != null && ctrl.landmarksWorldPos.Count >= 12);
        Vector2[] src12 = DobotCR10AIKController.GetCanonicalLandmarks12UV(templateSize);
        Vector2[] dst12XZ = null;
        float avgClothY = (plyLoader != null) ? (plyLoader.estimatedTableY + 0.025f) : 0.025f;

        if (has12Landmarks)
        {
            dst12XZ = new Vector2[12];
            avgClothY = 0f;
            for (int i = 0; i < 12; i++)
            {
                dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);
                avgClothY += ctrl.landmarksWorldPos[i].y;
            }
            avgClothY /= 12f;
        }
        else if (ctrl.garmentContourWorld != null && ctrl.garmentContourWorld.Count >= 3)
        {
            float sumY = 0f;
            for (int i = 0; i < ctrl.garmentContourWorld.Count; i++) sumY += ctrl.garmentContourWorld[i].y;
            avgClothY = sumY / ctrl.garmentContourWorld.Count;
        }

        List<Vector2> garmentClip2D = null;
        if (ctrl.garmentContourWorld != null && ctrl.garmentContourWorld.Count >= 6)
        {
            var rawG = new List<Vector2>(ctrl.garmentContourWorld.Count);
            for (int i = 0; i < ctrl.garmentContourWorld.Count; i++)
                rawG.Add(new Vector2(ctrl.garmentContourWorld[i].x, ctrl.garmentContourWorld[i].z));
            garmentClip2D = SimplifyClosedPolygonDouglasPeucker(rawG, dpEpsM);
        }

        bool hasMaskFallback = (garmentClip2D != null && garmentClip2D.Count >= 3)
            || (ctrl.sprayZoneLeftWorld != null && ctrl.sprayZoneLeftWorld.Count >= 3)
            || (ctrl.sprayZoneRightWorld != null && ctrl.sprayZoneRightWorld.Count >= 3)
            || ctrl.HasCustomDrawnRegions2D;
        if (!has12Landmarks && !hasMaskFallback)
        {
            p1Result.warningMessage = "Chưa có Mask 2D (đường bao quần / vùng ống) hoặc 12 mốc AI cho phôi hiện tại.";
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        AnatomicalLegChain dstChainL = has12Landmarks ? BuildLegChain(dst12XZ, true) : default;
        AnatomicalLegChain dstChainR = has12Landmarks ? BuildLegChain(dst12XZ, false) : default;

        // Tách sẵn mask 2 ống quần thuần túy từ Mask 2D (phục vụ P1 chạy độc lập không cần 12 mốc!)
        var pureMaskLegs = ExtractLegMaskPolygonsFromMask2D(ctrl, dpEpsM, preferFullGarmentSplit: true);

        // BƯỚC 1 & 2 & 3: Xây dựng vùng cần phun R gốc + đơn giản hóa Douglas-Peucker + co biên e -> R_safe
        var rawRegions2D = new List<List<Vector2>>();
        var safeRegions2D = new List<List<Vector2>>();
        var safeToRawIdx = new List<int>();
        var regionFallbackAxes = new List<Vector2>();
        float totalRawSqM = 0f;
        float totalSafeSqM = 0f;

        if (p1Cfg.regionSource == P1RegionSourceMode.TestCircle)
        {
            float r0 = Mathf.Clamp(p1Cfg.testCircleRadiusMm, 10f, 180f) * 0.001f;
            float rSafe = r0 - erosionEM;
            int nLegs = p1Cfg.testOnBothLegs ? 2 : 1;
            for (int leg = 0; leg < nLegs; leg++)
            {
                Vector2 center, longAx;
                if (has12Landmarks)
                {
                    Vector2 thigh = dst12XZ[leg == 0 ? 4 : 5];
                    Vector2 knee = dst12XZ[leg == 0 ? 6 : 7];
                    center = 0.5f * (thigh + knee);
                    longAx = (knee - thigh).sqrMagnitude > 1e-6f ? (knee - thigh).normalized : Vector2.up;
                }
                else if (leg < pureMaskLegs.Count && pureMaskLegs[leg].Count >= 3)
                {
                    ComputePcaAxes2D(pureMaskLegs[leg], Vector2.up, out center, out longAx, out _);
                }
                else continue;

                int rawIdx = rawRegions2D.Count;
                rawRegions2D.Add(BuildCirclePolygon2D(center, r0, 56));
                totalRawSqM += Mathf.PI * r0 * r0;
                if (rSafe > 0.003f)
                {
                    safeRegions2D.Add(BuildCirclePolygon2D(center, rSafe, 56));
                    safeToRawIdx.Add(rawIdx);
                    regionFallbackAxes.Add(longAx);
                    totalSafeSqM += Mathf.PI * rSafe * rSafe;
                }
            }
            if (rSafe <= 0.003f)
                p1Result.warningMessage = $"Co biên e = {p1Cfg.erosionEMm:F1} mm >= bán kính R0 = {p1Cfg.testCircleRadiusMm:F1} mm -> vùng R_safe biến mất! Hãy giảm e.";
        }
        else if (p1Cfg.regionSource == P1RegionSourceMode.TestRectangle)
        {
            float w0 = Mathf.Clamp(p1Cfg.testRectWidthMm, 20f, 300f) * 0.001f;
            float l0 = Mathf.Clamp(p1Cfg.testRectLengthMm, 30f, 600f) * 0.001f;
            float wSafe = w0 - 2f * erosionEM;
            float lSafe = l0 - 2f * erosionEM;
            int nLegs = p1Cfg.testOnBothLegs ? 2 : 1;
            for (int leg = 0; leg < nLegs; leg++)
            {
                Vector2 center, longAx;
                if (has12Landmarks)
                {
                    Vector2 thigh = dst12XZ[leg == 0 ? 4 : 5];
                    Vector2 knee = dst12XZ[leg == 0 ? 6 : 7];
                    center = 0.5f * (thigh + knee);
                    longAx = (knee - thigh).sqrMagnitude > 1e-6f ? (knee - thigh).normalized : Vector2.up;
                }
                else if (leg < pureMaskLegs.Count && pureMaskLegs[leg].Count >= 3)
                {
                    ComputePcaAxes2D(pureMaskLegs[leg], Vector2.up, out center, out longAx, out _);
                }
                else continue;

                int rawIdx = rawRegions2D.Count;
                rawRegions2D.Add(BuildOrientedRectPolygon2D(center, longAx, w0, l0));
                totalRawSqM += w0 * l0;
                if (wSafe > 0.005f && lSafe > 0.005f)
                {
                    safeRegions2D.Add(BuildOrientedRectPolygon2D(center, longAx, wSafe, lSafe));
                    safeToRawIdx.Add(rawIdx);
                    regionFallbackAxes.Add(longAx);
                    totalSafeSqM += wSafe * lSafe;
                }
            }
            if (wSafe <= 0.005f || lSafe <= 0.005f)
                p1Result.warningMessage = $"Co biên 2*e = {2f * p1Cfg.erosionEMm:F1} mm >= kích thước HCN ({p1Cfg.testRectWidthMm:F0}x{p1Cfg.testRectLengthMm:F0} mm) -> vùng R_safe biến mất!";
        }
        else
        {
            var candidateRawPolys = new List<List<Vector2>>();
            var candidateAxes = new List<Vector2>();
            bool clipToGarment = (p1Cfg.regionSource == P1RegionSourceMode.JeansLegZones);

            bool usedUserDrawnRegions = false;
            if (p1Cfg.regionSource == P1RegionSourceMode.FromTemplateStrokes)
            {
                // Nếu người dùng vẽ trực tiếp vùng hình cơ bản (Tròn / Vuông / Đa giác) trên ảnh 2D,
                // dùng thẳng đa giác thực 3D World XZ (không bị méo qua biến dạng 12 mốc và không bị giãn nét hở!)
                if (ctrl.HasCustomDrawnRegions2D)
                {
                    var directRegionsXZ = ctrl.GetCustomDrawnRegionPolygonsWorldXZ(plyLoader);
                    for (int i = 0; i < directRegionsXZ.Count; i++)
                    {
                        if (directRegionsXZ[i] != null && directRegionsXZ[i].Count >= 3)
                        {
                            candidateRawPolys.Add(directRegionsXZ[i]);
                            candidateAxes.Add(Vector2.up);
                            usedUserDrawnRegions = true;
                        }
                    }
                }
                if (has12Landmarks && templateStrokesUV != null && templateStrokesUV.Count > 0 &&
                    (!usedUserDrawnRegions || ctrl.HasCustomDrawnStrokes2D))
                {
                    var closedUserLoops = new List<List<Vector2>>();
                    var openStrokesWorld2D = new List<List<Vector2>>();

                    for (int s = 0; s < templateStrokesUV.Count; s++)
                    {
                        var st = templateStrokesUV[s];
                        if (st == null || st.Count < 2) continue;

                        var warped = new List<Vector2>(st.Count);
                        float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
                        float arcLen = 0f;
                        for (int p = 0; p < st.Count; p++)
                        {
                            Vector2 wp = WarpPointBarycentric12(st[p], src12, dst12XZ);
                            if (warped.Count > 0) arcLen += Vector2.Distance(warped[warped.Count - 1], wp);
                            warped.Add(wp);
                            if (wp.x < minX) minX = wp.x; if (wp.x > maxX) maxX = wp.x;
                            if (wp.y < minY) minY = wp.y; if (wp.y > maxY) maxY = wp.y;
                        }
                        float diag = Mathf.Sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY));
                        float endGap = Vector2.Distance(warped[0], warped[warped.Count - 1]);
                        float area = ComputePolygonAreaSqM(warped);

                        // Nhận diện thông minh vòng khép kín vẽ tay (hình tròn, oval, hình vuông, hình tự do...)
                        bool isLoopShape = (warped.Count >= 6 && diag > 0.025f && area >= 0.00025f)
                            && (endGap <= Mathf.Max(0.065f, diag * 0.42f) || (endGap > 1e-4f && arcLen / endGap >= 2.2f));
                        if (isLoopShape)
                        {
                            closedUserLoops.Add(warped);
                        }
                        else
                        {
                            openStrokesWorld2D.Add(warped);
                        }
                    }

                    for (int i = 0; i < closedUserLoops.Count; i++)
                    {
                        var simp = SimplifyClosedPolygonDouglasPeucker(closedUserLoops[i], dpEpsM);
                        if (simp.Count >= 3)
                        {
                            candidateRawPolys.Add(simp);
                            candidateAxes.Add(Vector2.up);
                        }
                    }

                    // Với các nét vẽ hở (chữ, đường cong, râu mèo, mẫu nhiều nét): chuyển đúng hình dáng nét vẽ thành vùng Mask R!
                    if (openStrokesWorld2D.Count > 0)
                    {
                        float brushR = Mathf.Max(0.012f, p1Cfg.openStrokeBrushRadiusMm * 0.001f);
                        if (erosionEM > 0.001f && brushR < erosionEM + 0.008f)
                            brushR = Mathf.Min(0.045f, erosionEM + 0.010f);
                        var strokeMaskPolys = BuildMaskPolygonsFromOpenStrokes2D(openStrokesWorld2D, brushR, dpEpsM);
                        for (int i = 0; i < strokeMaskPolys.Count; i++)
                        {
                            candidateRawPolys.Add(strokeMaskPolys[i]);
                            candidateAxes.Add(Vector2.up);
                        }
                    }
                }
            }

            if (candidateRawPolys.Count == 0)
            {
                clipToGarment = true;
                // Ưu tiên lấy trực tiếp từ Mask 2D thuần (sprayZoneLeft/Right hoặc tự tách 2 ống từ garmentContourWorld — KHÔNG cần 12 mốc!)
                if (pureMaskLegs.Count > 0)
                {
                    for (int m = 0; m < pureMaskLegs.Count; m++)
                    {
                        ComputePcaAxes2D(pureMaskLegs[m], Vector2.up, out _, out Vector2 pcaAx, out _);
                        candidateRawPolys.Add(pureMaskLegs[m]);
                        candidateAxes.Add(pcaAx);
                    }
                }
                else if (has12Landmarks)
                {
                    var stripL = BuildAnatomicalLegStripPolygon2D(dstChainL, 1.08f, 3.80f, -0.88f, 0.88f);
                    var stripR = BuildAnatomicalLegStripPolygon2D(dstChainR, 1.08f, 3.80f, -0.88f, 0.88f);
                    candidateRawPolys.Add(SimplifyClosedPolygonDouglasPeucker(stripL, dpEpsM));
                    candidateAxes.Add((dstChainL.C[4] - dstChainL.C[1]).normalized);
                    candidateRawPolys.Add(SimplifyClosedPolygonDouglasPeucker(stripR, dpEpsM));
                    candidateAxes.Add((dstChainR.C[4] - dstChainR.C[1]).normalized);
                }
            }

            int autoClampedSmallCount = 0;
            for (int r = 0; r < candidateRawPolys.Count; r++)
            {
                int rawIdx = rawRegions2D.Count;
                rawRegions2D.Add(candidateRawPolys[r]);
                var erodedList = ErodePolygonByDistanceE2D(candidateRawPolys[r], erosionEM, clipToGarment ? garmentClip2D : null, dpEpsM, out float aRaw, out float aSafe);

                // Bảo vệ các vùng nhỏ do người dùng tự vẽ (Tròn/Vuông/Đa giác): không để co biên e xóa mất hoàn toàn vùng vẽ!
                if (erodedList.Count == 0 && usedUserDrawnRegions && aRaw > 1e-5f)
                {
                    float approxR = Mathf.Sqrt(aRaw / Mathf.PI);
                    float safeSmallE = Mathf.Clamp(approxR * 0.25f, 0f, erosionEM * 0.4f);
                    if (safeSmallE >= 0.002f)
                        erodedList = ErodePolygonByDistanceE2D(candidateRawPolys[r], safeSmallE, null, Mathf.Min(dpEpsM, 0.002f), out _, out aSafe);
                    if (erodedList.Count == 0)
                    {
                        erodedList = new List<List<Vector2>> { new List<Vector2>(candidateRawPolys[r]) };
                        aSafe = aRaw;
                    }
                    autoClampedSmallCount++;
                }

                totalRawSqM += aRaw;
                totalSafeSqM += aSafe;
                for (int k = 0; k < erodedList.Count; k++)
                {
                    safeRegions2D.Add(erodedList[k]);
                    safeToRawIdx.Add(rawIdx);
                    regionFallbackAxes.Add(candidateAxes[r]);
                }
            }

            if (safeRegions2D.Count == 0 && rawRegions2D.Count > 0)
            {
                p1Result.warningMessage = $"Co biên e = {p1Cfg.erosionEMm:F1} mm quá lớn so với bề rộng vùng R -> vùng R_safe biến mất! Hãy giảm e.";
            }
            else if (autoClampedSmallCount > 0)
            {
                p1Result.warningMessage = $"Lưu ý: {autoClampedSmallCount} vùng vẽ nhỏ hơn 2×e ({2f * p1Cfg.erosionEMm:F0} mm) đã tự giảm co biên để không bị mất quỹ đạo.";
            }
        }

        // Lưu đa giác 3D của CHỖ CẦN PHUN (R gốc & R_safe sau co biên e) để hiển thị độc lập trong View 2D & 3D
        bool flatPlane = (p1Cfg.zHeightMode == P1ZHeightMode.Flat2DPlane);
        for (int i = 0; i < rawRegions2D.Count; i++)
            p1Result.rawTargetRegionPolygonsWorld.Add(LiftPolygon2DToWorldSurface(rawRegions2D[i], ctrl, plyLoader, flatPlane, avgClothY));
        for (int i = 0; i < safeRegions2D.Count; i++)
            p1Result.safeErodedRegionPolygonsWorld.Add(LiftPolygon2DToWorldSurface(safeRegions2D[i], ctrl, plyLoader, flatPlane, avgClothY));

        p1Result.areaRawCm2 = totalRawSqM * 10000f;
        p1Result.areaSafeCm2 = totalSafeSqM * 10000f;

        if (safeRegions2D.Count == 0)
        {
            genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
            return new List<DobotCR10AIKController.WaypointData>();
        }

        bool fixedVert = (p1Cfg.zHeightMode != P1ZHeightMode.Surface25DSmoothNormal);
        float firstAngleDeg = 0f;
        bool hasFirstAngle = false;

        // =====================================================================
        // CHẾ ĐỘ A (MẶC ĐỊNH): QUÉT ZIGZAG KIỂU MÁY IN 3D (PRINTER RASTER FULL SWEEP)
        // Quỹ đạo đi Zigzag song song qua lại liên tục trên toàn khung bao của chỗ cần phun;
        // đoạn nào đi VÀO R_safe thì BẬT SÚNG (sprayOn=true), ngoài vùng & đoạn quay đầu thì TẮT SÚNG (sprayOn=false)!
        // =====================================================================
        if (p1Cfg.zigzagStyle == P1ZigzagStyle.PrinterRasterFullSweep)
        {
            var typedPath = new List<TypedSurfacePoint>(512);

            var groupedSafeByRaw = new Dictionary<int, List<List<Vector2>>>();
            for (int i = 0; i < safeRegions2D.Count; i++)
            {
                int rKey = safeToRawIdx[i];
                if (!groupedSafeByRaw.TryGetValue(rKey, out var lst))
                {
                    lst = new List<List<Vector2>>();
                    groupedSafeByRaw[rKey] = lst;
                }
                lst.Add(safeRegions2D[i]);
            }

            int groupNum = 0;
            foreach (var kv in groupedSafeByRaw)
            {
                groupNum++;
                int rawIdx = kv.Key;
                List<Vector2> framePoly = (rawIdx >= 0 && rawIdx < rawRegions2D.Count) ? rawRegions2D[rawIdx] : kv.Value[0];
                List<List<Vector2>> safePolysInGroup = kv.Value;

                Vector2 fallbackMajor = Vector2.up;
                for (int i = 0; i < safeToRawIdx.Count; i++)
                {
                    if (safeToRawIdx[i] == rawIdx && i < regionFallbackAxes.Count)
                    {
                        fallbackMajor = regionFallbackAxes[i];
                        break;
                    }
                }

                Vector2 centroid, majorAxis, minorAxis;
                if (p1Cfg.scanDirection == P1ScanDirectionMode.MinAreaBoundingBox)
                    ComputeMinAreaRectAxes2D(framePoly, fallbackMajor, out centroid, out majorAxis, out minorAxis);
                else if (p1Cfg.scanDirection == P1ScanDirectionMode.CustomAngleDeg)
                {
                    centroid = Vector2.zero;
                    for (int i = 0; i < framePoly.Count; i++) centroid += framePoly[i];
                    centroid /= framePoly.Count;
                    float rad = p1Cfg.customScanAngleDeg * Mathf.Deg2Rad;
                    majorAxis = new Vector2(Mathf.Cos(rad), Mathf.Sin(rad));
                    minorAxis = new Vector2(-majorAxis.y, majorAxis.x);
                }
                else
                    ComputePcaAxes2D(framePoly, fallbackMajor, out centroid, out majorAxis, out minorAxis);

                if (!hasFirstAngle)
                {
                    firstAngleDeg = Mathf.Atan2(majorAxis.y, majorAxis.x) * Mathf.Rad2Deg;
                    hasFirstAngle = true;
                }

                float minU = float.MaxValue, maxU = float.MinValue;
                for (int i = 0; i < framePoly.Count; i++)
                {
                    Vector2 rel = framePoly[i] - centroid;
                    float u = Vector2.Dot(rel, majorAxis);
                    if (u < minU) minU = u; if (u > maxU) maxU = u;
                }

                float safeMinV = float.MaxValue, safeMaxV = float.MinValue;
                var safePolysUV = new List<List<Vector2>>(safePolysInGroup.Count);
                for (int p = 0; p < safePolysInGroup.Count; p++)
                {
                    var sp = safePolysInGroup[p];
                    var uvList = new List<Vector2>(sp.Count);
                    for (int i = 0; i < sp.Count; i++)
                    {
                        Vector2 rel = sp[i] - centroid;
                        float u = Vector2.Dot(rel, majorAxis);
                        float v = Vector2.Dot(rel, minorAxis);
                        uvList.Add(new Vector2(u, v));
                        if (u < minU) minU = u; if (u > maxU) maxU = u;
                        if (v < safeMinV) safeMinV = v; if (v > safeMaxV) safeMaxV = v;
                    }
                    safePolysUV.Add(uvList);
                }

                // Đặt các đường quét song song CHẮC CHẮN cắt qua R_safe (kể cả vùng nhỏ/hẹp hơn bước lượt s!)
                float safeSpanV = Mathf.Max(0f, safeMaxV - safeMinV);
                int numLines;
                float vFirst, vLast;
                if (safeSpanV <= stepSM * 0.95f)
                {
                    numLines = 1;
                    vFirst = vLast = 0.5f * (safeMinV + safeMaxV);
                }
                else
                {
                    float edgeInset = Mathf.Min(stepSM * 0.22f, safeSpanV * 0.12f);
                    vFirst = safeMinV + edgeInset;
                    vLast = safeMaxV - edgeInset;
                    float innerSpan = Mathf.Max(0f, vLast - vFirst);
                    numLines = Mathf.Clamp(Mathf.CeilToInt(innerSpan / stepSM) + 1, 2, 95);
                }
                float uLeftFrame = minU - overscanM;
                float uRightFrame = maxU + overscanM;

                string segTag = $"P1_RASTER_{groupNum}";
                p1Result.cellCount++;
                Vector2? prevRowEndXZ = null;

                for (int lineIdx = 0; lineIdx < numLines; lineIdx++)
                {
                    float vLine = (numLines == 1)
                        ? vFirst
                        : Mathf.Lerp(vFirst, vLast, (float)lineIdx / (numLines - 1));
                    bool reverse = (lineIdx % 2 == 1);

                    var rawIntervals = new List<Vector2>();
                    for (int spIdx = 0; spIdx < safePolysUV.Count; spIdx++)
                    {
                        var ivs = IntersectPolygonWithScanlineUV(safePolysUV[spIdx], vLine);
                        rawIntervals.AddRange(ivs);
                    }

                    // Gộp các giao đoạn chồng lấn/sát nhau trước khi sắp xếp để KHÔNG BAO GIỜ chạy lùi trên cùng 1 hàng
                    var rowOnIntervals = new List<Vector2>(rawIntervals.Count);
                    if (rawIntervals.Count > 0)
                    {
                        rawIntervals.Sort((a, b) => a.x.CompareTo(b.x));
                        Vector2 curIv = rawIntervals[0];
                        for (int k = 1; k < rawIntervals.Count; k++)
                        {
                            if (rawIntervals[k].x <= curIv.y + 0.002f)
                            {
                                curIv.y = Mathf.Max(curIv.y, rawIntervals[k].y);
                            }
                            else
                            {
                                rowOnIntervals.Add(curIv);
                                curIv = rawIntervals[k];
                            }
                        }
                        rowOnIntervals.Add(curIv);
                        if (reverse) rowOnIntervals.Reverse();
                    }

                    float uStart = reverse ? uRightFrame : uLeftFrame;
                    float uEnd = reverse ? uLeftFrame : uRightFrame;
                    Vector2 rowStartXZ = centroid + uStart * majorAxis + vLine * minorAxis;
                    Vector2 rowEndXZ = centroid + uEnd * majorAxis + vLine * minorAxis;

                    if (prevRowEndXZ.HasValue)
                    {
                        AppendSampledLinePoints(typedPath, prevRowEndXZ.Value, rowStartXZ, sprayOn: false, segTag, sampleStepM, avgClothY, includeStart: false);
                        p1Result.sprayOffConnectorsWorld.Add(new List<Vector3>
                        {
                            new Vector3(prevRowEndXZ.Value.x, avgClothY, prevRowEndXZ.Value.y),
                            new Vector3(rowStartXZ.x, avgClothY, rowStartXZ.y)
                        });
                        p1Result.sprayOffConnectorCount++;
                    }
                    else
                    {
                        if (typedPath.Count > 0)
                        {
                            Vector3 prevGroupEnd = typedPath[typedPath.Count - 1].worldPos;
                            p1Result.sprayOffConnectorsWorld.Add(new List<Vector3>
                            {
                                prevGroupEnd,
                                new Vector3(rowStartXZ.x, avgClothY, rowStartXZ.y)
                            });
                            p1Result.sprayOffConnectorCount++;
                        }
                        typedPath.Add(new TypedSurfacePoint
                        {
                            worldPos = new Vector3(rowStartXZ.x, avgClothY, rowStartXZ.y),
                            sprayOnToHere = false,
                            segName = segTag
                        });
                    }

                    float curU = uStart;
                    Vector2 curXZ = rowStartXZ;
                    for (int j = 0; j < rowOnIntervals.Count; j++)
                    {
                        float uEnter = reverse ? rowOnIntervals[j].y : rowOnIntervals[j].x;
                        float uExit = reverse ? rowOnIntervals[j].x : rowOnIntervals[j].y;
                        // Đảm bảo đơn điệu theo chiều quét (không lùi ngược)
                        if (!reverse)
                        {
                            uEnter = Mathf.Max(uEnter, curU);
                            if (uExit <= uEnter + 0.002f) continue;
                        }
                        else
                        {
                            uEnter = Mathf.Min(uEnter, curU);
                            if (uExit >= uEnter - 0.002f) continue;
                        }
                        Vector2 enterXZ = centroid + uEnter * majorAxis + vLine * minorAxis;
                        Vector2 exitXZ = centroid + uExit * majorAxis + vLine * minorAxis;

                        // 1. Đoạn chạy trống tới mép chỗ cần phun -> TẮT SÚNG
                        if (Vector2.Distance(curXZ, enterXZ) > 0.002f)
                        {
                            AppendSampledLinePoints(typedPath, curXZ, enterXZ, sprayOn: false, segTag, sampleStepM, avgClothY, includeStart: false);
                            p1Result.sprayOffConnectorsWorld.Add(new List<Vector3>
                            {
                                new Vector3(curXZ.x, avgClothY, curXZ.y),
                                new Vector3(enterXZ.x, avgClothY, enterXZ.y)
                            });
                            p1Result.sprayOffConnectorCount++;
                        }

                        // 2. Đoạn đi NGANG QUA chỗ cần phun R_safe -> BẬT SÚNG!
                        if (Vector2.Distance(enterXZ, exitXZ) > 0.002f)
                        {
                            AppendSampledLinePoints(typedPath, enterXZ, exitXZ, sprayOn: true, segTag, sampleStepM, avgClothY, includeStart: false);
                            p1Result.sprayOnSegmentsWorld.Add(new List<Vector3>
                            {
                                new Vector3(enterXZ.x, avgClothY, enterXZ.y),
                                new Vector3(exitXZ.x, avgClothY, exitXZ.y)
                            });
                            p1Result.sprayOnSegmentCount++;
                        }
                        curU = uExit;
                        curXZ = exitXZ;
                    }

                    // 3. Đoạn chạy trống ra cuối khung chuẩn bị quay đầu -> TẮT SÚNG
                    if (Vector2.Distance(curXZ, rowEndXZ) > 0.002f)
                    {
                        AppendSampledLinePoints(typedPath, curXZ, rowEndXZ, sprayOn: false, segTag, sampleStepM, avgClothY, includeStart: false);
                        p1Result.sprayOffConnectorsWorld.Add(new List<Vector3>
                        {
                            new Vector3(curXZ.x, avgClothY, curXZ.y),
                            new Vector3(rowEndXZ.x, avgClothY, rowEndXZ.y)
                        });
                        p1Result.sprayOffConnectorCount++;
                    }

                    prevRowEndXZ = rowEndXZ;
                    p1Result.passCount++;
                }
            }

            p1Result.scanAngleDegUsed = firstAngleDeg;
            var rasterWps = BuildSprayWaypointsFromTypedPolyline(
                typedPath, ctrl, plyLoader, tech,
                useFixedVerticalNormal: fixedVert,
                useFlatPlaneZ: flatPlane,
                flatPlaneWorldY: avgClothY,
                overrideStandoffMm: p1Cfg.sprayDistanceDMm);
            genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
            return rasterWps;
        }

        // =====================================================================
        // CHẾ ĐỘ B: CẮT GỌN SÁT BIÊN R_safe + PHÂN TÁCH Ô LỒI BOUSTROPHEDON
        // =====================================================================
        Vector3? prevSegEndWorld = null;
        for (int rIdx = 0; rIdx < safeRegions2D.Count; rIdx++)
        {
            List<Vector2> safePoly = safeRegions2D[rIdx];
            if (safePoly == null || safePoly.Count < 3) continue;

            Vector2 fallbackMajor = rIdx < regionFallbackAxes.Count ? regionFallbackAxes[rIdx] : Vector2.up;
            Vector2 centroid, majorAxis, minorAxis;
            if (p1Cfg.scanDirection == P1ScanDirectionMode.MinAreaBoundingBox)
                ComputeMinAreaRectAxes2D(safePoly, fallbackMajor, out centroid, out majorAxis, out minorAxis);
            else if (p1Cfg.scanDirection == P1ScanDirectionMode.CustomAngleDeg)
            {
                centroid = Vector2.zero;
                for (int i = 0; i < safePoly.Count; i++) centroid += safePoly[i];
                centroid /= safePoly.Count;
                float rad = p1Cfg.customScanAngleDeg * Mathf.Deg2Rad;
                majorAxis = new Vector2(Mathf.Cos(rad), Mathf.Sin(rad));
                minorAxis = new Vector2(-majorAxis.y, majorAxis.x);
            }
            else
                ComputePcaAxes2D(safePoly, fallbackMajor, out centroid, out majorAxis, out minorAxis);

            if (!hasFirstAngle)
            {
                firstAngleDeg = Mathf.Atan2(majorAxis.y, majorAxis.x) * Mathf.Rad2Deg;
                hasFirstAngle = true;
            }

            var polyUV = new List<Vector2>(safePoly.Count);
            float minV = float.MaxValue, maxV = float.MinValue;
            for (int i = 0; i < safePoly.Count; i++)
            {
                Vector2 rel = safePoly[i] - centroid;
                float u = Vector2.Dot(rel, majorAxis);
                float v = Vector2.Dot(rel, minorAxis);
                polyUV.Add(new Vector2(u, v));
                if (v < minV) minV = v;
                if (v > maxV) maxV = v;
            }

            float spanV = maxV - minV;
            int numLines;
            float vFirstB, vLastB;
            if (spanV <= stepSM * 0.95f)
            {
                numLines = 1;
                vFirstB = vLastB = 0.5f * (minV + maxV);
            }
            else
            {
                float edgeInset = Mathf.Min(stepSM * 0.22f, spanV * 0.12f);
                vFirstB = minV + edgeInset;
                vLastB = maxV - edgeInset;
                numLines = Mathf.Clamp(Mathf.CeilToInt(Mathf.Max(0f, vLastB - vFirstB) / stepSM) + 1, 2, 90);
            }

            var lineIntervals = new List<List<BoustrophedonSpan>>(numLines);
            for (int lineIdx = 0; lineIdx < numLines; lineIdx++)
            {
                float vLine = (numLines == 1)
                    ? vFirstB
                    : Mathf.Lerp(vFirstB, vLastB, (float)lineIdx / (numLines - 1));

                var ivs = IntersectPolygonWithScanlineUV(polyUV, vLine);
                var spans = new List<BoustrophedonSpan>(ivs.Count);
                for (int k = 0; k < ivs.Count; k++)
                    spans.Add(new BoustrophedonSpan { v = vLine, u0 = ivs[k].x, u1 = ivs[k].y });
                lineIntervals.Add(spans);
            }

            var orderedCells = new List<List<BoustrophedonSpan>>();
            if (p1Cfg.useBoustrophedonSplit)
            {
                var activeCells = new List<List<BoustrophedonSpan>>();
                for (int lineIdx = 0; lineIdx < lineIntervals.Count; lineIdx++)
                {
                    var curSpans = lineIntervals[lineIdx];
                    if (activeCells.Count == 0)
                    {
                        for (int c = 0; c < curSpans.Count; c++)
                            activeCells.Add(new List<BoustrophedonSpan> { curSpans[c] });
                        continue;
                    }

                    int nAct = activeCells.Count;
                    int nCur = curSpans.Count;
                    var actMatches = new List<int>[nAct];
                    var curMatches = new List<int>[nCur];
                    for (int a = 0; a < nAct; a++) actMatches[a] = new List<int>();
                    for (int c = 0; c < nCur; c++) curMatches[c] = new List<int>();

                    for (int a = 0; a < nAct; a++)
                    {
                        BoustrophedonSpan prevSp = activeCells[a][activeCells[a].Count - 1];
                        for (int c = 0; c < nCur; c++)
                        {
                            BoustrophedonSpan curSp = curSpans[c];
                            if (curSp.u0 <= prevSp.u1 + 0.005f && curSp.u1 >= prevSp.u0 - 0.005f)
                            {
                                actMatches[a].Add(c);
                                curMatches[c].Add(a);
                            }
                        }
                    }

                    var nextActive = new List<List<BoustrophedonSpan>>();
                    bool[] curConsumed = new bool[nCur];
                    for (int a = 0; a < nAct; a++)
                    {
                        if (actMatches[a].Count == 1 && curMatches[actMatches[a][0]].Count == 1)
                        {
                            int cIdx = actMatches[a][0];
                            activeCells[a].Add(curSpans[cIdx]);
                            nextActive.Add(activeCells[a]);
                            curConsumed[cIdx] = true;
                        }
                        else
                        {
                            orderedCells.Add(activeCells[a]);
                        }
                    }
                    for (int c = 0; c < nCur; c++)
                        if (!curConsumed[c]) nextActive.Add(new List<BoustrophedonSpan> { curSpans[c] });
                    activeCells = nextActive;
                }
                for (int a = 0; a < activeCells.Count; a++) orderedCells.Add(activeCells[a]);
            }
            else
            {
                var singleCell = new List<BoustrophedonSpan>();
                for (int lineIdx = 0; lineIdx < lineIntervals.Count; lineIdx++)
                    singleCell.AddRange(lineIntervals[lineIdx]);
                if (singleCell.Count > 0) orderedCells.Add(singleCell);
            }

            p1Result.cellCount += orderedCells.Count;
            for (int cIdx = 0; cIdx < orderedCells.Count; cIdx++)
            {
                var cellSpans = orderedCells[cIdx];
                for (int pIdx = 0; pIdx < cellSpans.Count; pIdx++)
                {
                    BoustrophedonSpan sp = cellSpans[pIdx];
                    bool reverse = (pIdx % 2 == 1);
                    float uA = reverse ? sp.u1 : sp.u0;
                    float uB = reverse ? sp.u0 : sp.u1;
                    float segLen = Mathf.Abs(uB - uA);
                    int steps = Mathf.Clamp(Mathf.CeilToInt(segLen / sampleStepM), 2, 80);

                    var segWorld = new List<Vector3>(steps + 1);
                    for (int k = 0; k <= steps; k++)
                    {
                        float t = (float)k / steps;
                        float uVal = Mathf.Lerp(uA, uB, t);
                        Vector2 pXZ = centroid + uVal * majorAxis + sp.v * minorAxis;
                        float surfY = avgClothY;
                        if (!flatPlane)
                        {
                            if (ctrl.SampleCompensatedSurface(pXZ.x, pXZ.y, out float cy, out _)) surfY = cy;
                            else if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(pXZ.x, pXZ.y, out float py, out _)) surfY = py;
                        }
                        segWorld.Add(new Vector3(pXZ.x, surfY, pXZ.y));
                    }

                    if (segWorld.Count >= 2)
                    {
                        if (prevSegEndWorld.HasValue)
                        {
                            p1Result.sprayOffConnectorsWorld.Add(new List<Vector3> { prevSegEndWorld.Value, segWorld[0] });
                            p1Result.sprayOffConnectorCount++;
                        }
                        prevSegEndWorld = segWorld[segWorld.Count - 1];
                        strokesWorld.Add(segWorld);
                        p1Result.sprayOnSegmentsWorld.Add(segWorld);
                        p1Result.sprayOnSegmentCount++;
                        p1Result.passCount++;
                    }
                }
            }
        }

        p1Result.scanAngleDegUsed = firstAngleDeg;
        var wps = BuildSprayWaypointsFromWorldStrokes(
            strokesWorld, ctrl, plyLoader, tech, "P1_ZIGZAG",
            useFixedVerticalNormal: fixedVert,
            useFlatPlaneZ: flatPlane,
            flatPlaneWorldY: avgClothY,
            overrideStandoffMm: p1Cfg.sprayDistanceDMm);

        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
    }


    /// <summary>
    /// P2 — Đường phun theo hình dạng ống quần (Song song trục giữa Skeleton hoặc Offset đồng tâm từ biên, Chuẩn 8 bước trang 7):
    /// Bước 1: Nhận mask & tách 2 vùng ống trái / phải riêng biệt (loại bỏ vùng cạp & đáy quần), trích xuất đa giác "Chỗ cần phun".
    /// Bước 2: Tính trục giữa (skeleton) chính chạy từ đùi xuống gấu của từng ống quần.
    /// Bước 3 & 4: Sinh các đường chạy cách nhau bước s = w*(1-r) theo Cách A (song song trục giữa) hoặc Cách B (offset đồng tâm lùi dần từ biên).
    /// Bước 5: Xử lý thay đổi bề rộng ống quần (cắt ngắn lượt ngoài khi ống thu hẹp từ đùi xuống gấu và loại bỏ đoạn quá ngắn).
    /// Bước 6: Sắp xếp thứ tự các lượt để giảm quãng đường di chuyển không phun (order_for_minimum_travel) và tắt súng ở đoạn nối.
    /// Bước 7 & 8: Gán độ cao Z + hướng súng (pháp tuyến làm mượt hoặc thẳng đứng) và kiểm tra xuất quỹ đạo TCP.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP2ShapeSkeleton(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        out float genTimeSec)
    {
        var cfg = (ctrl != null && ctrl.p2Config != null) ? ctrl.p2Config : new P2SkeletonConfig
        {
            sprayWidthWMm = tech != null ? tech.sprayWidthMm : 45f,
            overlapRatioR = tech != null ? tech.overlapRatio : 0.35f,
            borderMarginEMm = tech != null ? tech.borderMarginMm : 16f,
            sprayDistanceDMm = GetEffectiveSprayHeightMm(ctrl, tech),
            autoSpacingFromWidthOverlap = true
        };
        return GenerateP2ShapeSkeleton(ctrl, plyLoader, templateStrokesUV, templateSize, tech, cfg, out _, out genTimeSec);
    }

    private struct MaskSkeletonSpine
    {
        public List<Vector2> C; // Trục giữa thật (Medial Axis) từ đùi xuống gấu, đã làm mượt & loại nhánh nhiễu
        public List<Vector2> N; // Pháp tuyến ngang vuông góc trục giữa tại mỗi điểm
        public List<float> R;   // Nửa bề rộng thật của ống quần đo từ Exact Euclidean Distance Transform (EDT)
        public float totalLengthM;
        public float maxHalfWidthM;
    }

    /// <summary>
    /// Tách riêng 2 vùng mask ống quần trái / phải trực tiếp từ Mask 2D (`garmentContourWorld` hoặc `sprayZoneLeftWorld`/`sprayZoneRightWorld`).
    /// Khi `preferFullGarmentSplit = true` (dùng cho P2): ưu tiên tách toàn bộ 2 ống quần đầy đủ từ đùi xuống gấu dựa trên `garmentContourWorld`,
    /// tự nhận diện chính xác đầu cạp (đặc ở trục giữa v=0) vs đầu gấu (tách 2 ống trống ở v=0) — KHÔNG cần 12 mốc AI!
    /// </summary>
    private static List<List<Vector2>> ExtractLegMaskPolygonsFromMask2D(DobotCR10AIKController ctrl, float dpEpsM, bool preferFullGarmentSplit = false)
    {
        var legs = new List<List<Vector2>>();
        if (ctrl == null) return legs;

        if (!preferFullGarmentSplit)
        {
            if (ctrl.sprayZoneLeftWorld != null && ctrl.sprayZoneLeftWorld.Count >= 4)
            {
                var pL = new List<Vector2>(ctrl.sprayZoneLeftWorld.Count);
                for (int i = 0; i < ctrl.sprayZoneLeftWorld.Count; i++)
                    pL.Add(new Vector2(ctrl.sprayZoneLeftWorld[i].x, ctrl.sprayZoneLeftWorld[i].z));
                var simp = SimplifyClosedPolygonDouglasPeucker(pL, dpEpsM);
                if (simp.Count >= 3) legs.Add(simp);
            }
            if (ctrl.sprayZoneRightWorld != null && ctrl.sprayZoneRightWorld.Count >= 4)
            {
                var pR = new List<Vector2>(ctrl.sprayZoneRightWorld.Count);
                for (int i = 0; i < ctrl.sprayZoneRightWorld.Count; i++)
                    pR.Add(new Vector2(ctrl.sprayZoneRightWorld[i].x, ctrl.sprayZoneRightWorld[i].z));
                var simp = SimplifyClosedPolygonDouglasPeucker(pR, dpEpsM);
                if (simp.Count >= 3) legs.Add(simp);
            }
            if (legs.Count > 0) return legs;
        }

        // Tách đầy đủ 2 ống quần trái / phải từ đường bao toàn thân quần `garmentContourWorld` (không cần 12 mốc!)
        if (ctrl.garmentContourWorld != null && ctrl.garmentContourWorld.Count >= 8)
        {
            var rawG = new List<Vector2>(ctrl.garmentContourWorld.Count);
            for (int i = 0; i < ctrl.garmentContourWorld.Count; i++)
                rawG.Add(new Vector2(ctrl.garmentContourWorld[i].x, ctrl.garmentContourWorld[i].z));

            ComputePcaAxes2D(rawG, Vector2.up, out Vector2 centroid, out Vector2 majorAx, out Vector2 minorAx);
            float minU = float.MaxValue, maxU = float.MinValue;
            float minVAll = float.MaxValue, maxVAll = float.MinValue;
            for (int i = 0; i < rawG.Count; i++)
            {
                Vector2 rel = rawG[i] - centroid;
                float u = Vector2.Dot(rel, majorAx);
                float v = Vector2.Dot(rel, minorAx);
                if (u < minU) minU = u; if (u > maxU) maxU = u;
                if (v < minVAll) minVAll = v; if (v > maxVAll) maxVAll = v;
            }
            float uSpan = Mathf.Max(0.10f, maxU - minU);

            // Tự xác định chiều Cạp -> Gấu thuần túy từ mask:
            // Đầu Cạp đặc liền ở trục giữa (v = 0 nằm TRONG mask), còn đầu Gấu chẻ 2 ống (v = 0 nằm NGOÀI mask)!
            int midInsideAtMin = 0, midInsideAtMax = 0;
            for (int k = 1; k <= 5; k++)
            {
                float frac = 0.08f + 0.04f * k; // 12%..28% từ mỗi đầu
                Vector2 pMin = centroid + majorAx * (minU + frac * uSpan);
                Vector2 pMax = centroid + majorAx * (maxU - frac * uSpan);
                if (IsPointInPolygonXZ(pMin, rawG)) midInsideAtMin++;
                if (IsPointInPolygonXZ(pMax, rawG)) midInsideAtMax++;
            }
            if (midInsideAtMax > midInsideAtMin)
            {
                // Đảo trục để minU luôn là đầu Cạp (Waist), maxU luôn là đầu Gấu (Cuff)
                majorAx = -majorAx;
                minorAx = -minorAx;
                float oldMin = minU;
                minU = -maxU;
                maxU = -oldMin;
                float oldMinV = minVAll;
                minVAll = -maxVAll;
                maxVAll = -oldMinV;
            }

            // Tìm vị trí chẻ đũng (crotch notch) uCrotch dọc theo v = 0 thuần túy từ mask
            float uCrotch = minU + 0.32f * uSpan;
            for (int s = 10; s <= 65; s++)
            {
                float uTest = minU + (s / 100f) * uSpan;
                Vector2 pMid = centroid + majorAx * uTest;
                if (IsPointInPolygonXZ(pMid, rawG))
                {
                    uCrotch = uTest;
                }
                else if (uTest > minU + 0.22f * uSpan)
                {
                    break;
                }
            }

            float uTopCut = Mathf.Clamp(uCrotch - 0.05f * uSpan, minU + 0.16f * uSpan, minU + 0.48f * uSpan);
            float uBotCut = maxU - 0.02f * uSpan;
            float vSearchMax = Mathf.Max(0.28f, Mathf.Max(Mathf.Abs(minVAll), Mathf.Abs(maxVAll)) + 0.03f);

            for (int side = -1; side <= 1; side += 2)
            {
                var leftB = new List<Vector2>();
                var rightB = new List<Vector2>();
                int nSlices = 28;
                for (int s = 0; s <= nSlices; s++)
                {
                    float u = Mathf.Lerp(uTopCut, uBotCut, (float)s / nSlices);
                    float vMinSide = float.MaxValue, vMaxSide = float.MinValue;
                    for (int k = 0; k <= 48; k++)
                    {
                        float v = (side < 0)
                            ? Mathf.Lerp(-vSearchMax, -0.004f, k / 48f)
                            : Mathf.Lerp(0.004f, vSearchMax, k / 48f);
                        Vector2 p = centroid + majorAx * u + minorAx * v;
                        if (IsPointInPolygonXZ(p, rawG))
                        {
                            if (v < vMinSide) vMinSide = v;
                            if (v > vMaxSide) vMaxSide = v;
                        }
                    }
                    if (vMaxSide - vMinSide > 0.025f)
                    {
                        leftB.Add(centroid + majorAx * u + minorAx * vMinSide);
                        rightB.Add(centroid + majorAx * u + minorAx * vMaxSide);
                    }
                }
                if (leftB.Count >= 4)
                {
                    var legPoly = new List<Vector2>(leftB.Count + rightB.Count);
                    legPoly.AddRange(leftB);
                    for (int k = rightB.Count - 1; k >= 0; k--) legPoly.Add(rightB[k]);
                    var simp = SimplifyClosedPolygonDouglasPeucker(legPoly, dpEpsM);
                    if (simp.Count >= 3) legs.Add(simp);
                }
            }
            if (legs.Count > 0) return legs;
        }

        // Dự phòng cuối cùng nếu không có garmentContourWorld: dùng sprayZoneLeft/RightWorld
        if (ctrl.sprayZoneLeftWorld != null && ctrl.sprayZoneLeftWorld.Count >= 4)
        {
            var pL = new List<Vector2>(ctrl.sprayZoneLeftWorld.Count);
            for (int i = 0; i < ctrl.sprayZoneLeftWorld.Count; i++)
                pL.Add(new Vector2(ctrl.sprayZoneLeftWorld[i].x, ctrl.sprayZoneLeftWorld[i].z));
            var simp = SimplifyClosedPolygonDouglasPeucker(pL, dpEpsM);
            if (simp.Count >= 3) legs.Add(simp);
        }
        if (ctrl.sprayZoneRightWorld != null && ctrl.sprayZoneRightWorld.Count >= 4)
        {
            var pR = new List<Vector2>(ctrl.sprayZoneRightWorld.Count);
            for (int i = 0; i < ctrl.sprayZoneRightWorld.Count; i++)
                pR.Add(new Vector2(ctrl.sprayZoneRightWorld[i].x, ctrl.sprayZoneRightWorld[i].z));
            var simp = SimplifyClosedPolygonDouglasPeucker(pR, dpEpsM);
            if (simp.Count >= 3) legs.Add(simp);
        }
        return legs;
    }

    /// <summary>
    /// Tính TRỤC GIỮA (SKELETON / MEDIAL AXIS) THẬT trực tiếp từ ảnh nhị phân Mask ống quần:
    /// 1. Rasterize đa giác mask sang lưới nhị phân (~6 mm).
    /// 2. Tính Exact Euclidean Distance Transform (EDT) khoảng cách tới biên mask.
    /// 3. Trích xuất dãy đỉnh cực đại EDT (Medial Axis) dọc theo trục dài PCA từ đùi xuống gấu + lọc nhánh nhiễu (làm mượt 3 điểm).
    /// </summary>
    private static bool ExtractRealMaskSkeletonByEDT(List<Vector2> legMaskPoly, float gridResM, out MaskSkeletonSpine spine)
    {
        spine = new MaskSkeletonSpine
        {
            C = new List<Vector2>(),
            N = new List<Vector2>(),
            R = new List<float>(),
            totalLengthM = 0f,
            maxHalfWidthM = 0.05f
        };
        if (legMaskPoly == null || legMaskPoly.Count < 3) return false;

        float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
        for (int i = 0; i < legMaskPoly.Count; i++)
        {
            Vector2 p = legMaskPoly[i];
            if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
        }
        float step = Mathf.Clamp(gridResM, 0.004f, 0.012f);
        float pad = step * 2f;
        minX -= pad; maxX += pad; minY -= pad; maxY += pad;
        int nx = Mathf.Clamp(Mathf.CeilToInt((maxX - minX) / step) + 1, 16, 180);
        int ny = Mathf.Clamp(Mathf.CeilToInt((maxY - minY) / step) + 1, 16, 180);
        float dx = (maxX - minX) / (nx - 1);
        float dy = (maxY - minY) / (ny - 1);
        float avgStep = 0.5f * (dx + dy);

        bool[,] inside = new bool[nx, ny];
        var insidePts = new List<Vector2>(nx * ny / 2);
        for (int ix = 0; ix < nx; ix++)
        {
            float wx = minX + ix * dx;
            for (int iy = 0; iy < ny; iy++)
            {
                float wy = minY + iy * dy;
                if (IsPointInPolygonXZ(new Vector2(wx, wy), legMaskPoly))
                {
                    inside[ix, iy] = true;
                    insidePts.Add(new Vector2(wx, wy));
                }
            }
        }
        if (insidePts.Count < 16) return false;

        // 2-pass Chamfer / Euclidean Distance Transform (EDT) tính khoảng cách tới biên mask
        float[,] dist = new float[nx, ny];
        const float INF = 1e6f;
        const float DIAG = 1.41421356f;
        for (int ix = 0; ix < nx; ix++)
            for (int iy = 0; iy < ny; iy++)
                dist[ix, iy] = inside[ix, iy] ? INF : 0f;

        for (int ix = 0; ix < nx; ix++)
        {
            for (int iy = 0; iy < ny; iy++)
            {
                float d = dist[ix, iy];
                if (d == 0f) continue;
                if (ix > 0) d = Mathf.Min(d, dist[ix - 1, iy] + 1f);
                if (iy > 0) d = Mathf.Min(d, dist[ix, iy - 1] + 1f);
                if (ix > 0 && iy > 0) d = Mathf.Min(d, dist[ix - 1, iy - 1] + DIAG);
                if (ix > 0 && iy + 1 < ny) d = Mathf.Min(d, dist[ix - 1, iy + 1] + DIAG);
                dist[ix, iy] = d;
            }
        }
        for (int ix = nx - 1; ix >= 0; ix--)
        {
            for (int iy = ny - 1; iy >= 0; iy--)
            {
                float d = dist[ix, iy];
                if (d == 0f) continue;
                if (ix + 1 < nx) d = Mathf.Min(d, dist[ix + 1, iy] + 1f);
                if (iy + 1 < ny) d = Mathf.Min(d, dist[ix, iy + 1] + 1f);
                if (ix + 1 < nx && iy + 1 < ny) d = Mathf.Min(d, dist[ix + 1, iy + 1] + DIAG);
                if (ix + 1 < nx && iy > 0) d = Mathf.Min(d, dist[ix + 1, iy - 1] + DIAG);
                dist[ix, iy] = d;
            }
        }

        // Tính trục dài PCA của mask ống quần để quét lát cắt trích xuất sống trục Medial Axis từ đùi -> gấu
        ComputePcaAxes2D(insidePts, Vector2.up, out Vector2 centroid, out Vector2 uAx, out Vector2 vAx);
        float minU = float.MaxValue, maxU = float.MinValue;
        for (int i = 0; i < insidePts.Count; i++)
        {
            float u = Vector2.Dot(insidePts[i] - centroid, uAx);
            if (u < minU) minU = u;
            if (u > maxU) maxU = u;
        }
        float uSpan = maxU - minU;
        if (uSpan < 0.04f) return false;

        int nSlices = 30;
        float sliceHalf = Mathf.Max(avgStep * 1.15f, uSpan / (nSlices * 1.4f));
        var rawC = new List<Vector2>(nSlices + 1);
        var rawR = new List<float>(nSlices + 1);

        for (int s = 0; s <= nSlices; s++)
        {
            float uTarget = Mathf.Lerp(minU + avgStep, maxU - avgStep, (float)s / nSlices);
            float maxD = 0f;
            float vMinSl = float.MaxValue, vMaxSl = float.MinValue;
            // Thu tập điểm trong lát cắt quanh uTarget
            var slicePts = new List<(Vector2 pt, float edtM)>();
            for (int ix = 0; ix < nx; ix++)
            {
                float wx = minX + ix * dx;
                for (int iy = 0; iy < ny; iy++)
                {
                    if (!inside[ix, iy]) continue;
                    float wy = minY + iy * dy;
                    Vector2 p = new Vector2(wx, wy);
                    float u = Vector2.Dot(p - centroid, uAx);
                    if (Mathf.Abs(u - uTarget) > sliceHalf) continue;
                    float edtM = dist[ix, iy] * avgStep;
                    if (edtM > maxD) maxD = edtM;
                    float v = Vector2.Dot(p - centroid, vAx);
                    if (v < vMinSl) vMinSl = v;
                    if (v > vMaxSl) vMaxSl = v;
                    slicePts.Add((p, edtM));
                }
            }
            if (slicePts.Count == 0 || maxD < 0.004f) continue;

            // Lấy tâm trọng số của các pixel gần cực đại EDT (>= 82% maxD) -> trục giữa Medial Axis trơn, không răng cưa!
            Vector2 sumP = Vector2.zero;
            float sumW = 0f;
            float thresh = maxD * 0.82f;
            for (int i = 0; i < slicePts.Count; i++)
            {
                if (slicePts[i].edtM >= thresh)
                {
                    float w = slicePts[i].edtM;
                    sumP += slicePts[i].pt * w;
                    sumW += w;
                }
            }
            if (sumW > 1e-6f)
            {
                Vector2 cPt = sumP / sumW;
                float halfSpan = 0.5f * Mathf.Max(0.012f, vMaxSl - vMinSl);
                float localR = Mathf.Max(maxD, halfSpan * 0.92f);
                rawC.Add(cPt);
                rawR.Add(localR);
            }
        }
        if (rawC.Count < 4) return false;

        // Lọc nhánh nhiễu & làm mượt trục giữa (2 lượt moving average 3 điểm)
        for (int pass = 0; pass < 2; pass++)
        {
            var smC = new List<Vector2>(rawC.Count);
            var smR = new List<float>(rawR.Count);
            for (int i = 0; i < rawC.Count; i++)
            {
                if (i == 0 || i == rawC.Count - 1)
                {
                    smC.Add(rawC[i]);
                    smR.Add(rawR[i]);
                }
                else
                {
                    smC.Add(0.25f * rawC[i - 1] + 0.50f * rawC[i] + 0.25f * rawC[i + 1]);
                    smR.Add(0.25f * rawR[i - 1] + 0.50f * rawR[i] + 0.25f * rawR[i + 1]);
                }
            }
            rawC = smC;
            rawR = smR;
        }

        // Hướng trục thống nhất (đầu rộng hơn là đùi -> đầu hẹp hơn là gấu)
        if (rawR[0] < rawR[rawR.Count - 1])
        {
            rawC.Reverse();
            rawR.Reverse();
        }

        float totalLen = 0f;
        float maxR = 0.02f;
        for (int i = 0; i < rawC.Count; i++)
        {
            if (i > 0) totalLen += Vector2.Distance(rawC[i - 1], rawC[i]);
            if (rawR[i] > maxR) maxR = rawR[i];
            Vector2 tan = (i == 0) ? (rawC[1] - rawC[0])
                : (i == rawC.Count - 1 ? (rawC[i] - rawC[i - 1]) : (rawC[i + 1] - rawC[i - 1]));
            if (tan.sqrMagnitude < 1e-8f) tan = uAx;
            else tan.Normalize();
            Vector2 nrm = new Vector2(-tan.y, tan.x);
            if (spine.N.Count > 0 && Vector2.Dot(nrm, spine.N[spine.N.Count - 1]) < 0f) nrm = -nrm;
            spine.C.Add(rawC[i]);
            spine.N.Add(nrm);
            spine.R.Add(rawR[i]);
        }
        spine.totalLengthM = Mathf.Max(0.05f, totalLen);
        spine.maxHalfWidthM = maxR;
        return spine.C.Count >= 4;
    }

    private static void SampleSpineAtT(MaskSkeletonSpine spine, float t01, out Vector2 centerPt, out Vector2 normalVec, out float halfWidthM)
    {
        int n = spine.C.Count;
        float f = Mathf.Clamp01(t01) * (n - 1);
        int i0 = Mathf.Clamp(Mathf.FloorToInt(f), 0, n - 2);
        float u = Mathf.Clamp01(f - i0);
        centerPt = Vector2.Lerp(spine.C[i0], spine.C[i0 + 1], u);
        normalVec = Vector2.Lerp(spine.N[i0], spine.N[i0 + 1], u);
        if (normalVec.sqrMagnitude < 1e-6f) normalVec = spine.N[i0];
        else normalVec.Normalize();
        halfWidthM = Mathf.Max(0.008f, Mathf.Lerp(spine.R[i0], spine.R[i0 + 1], u));
    }

    public static List<DobotCR10AIKController.WaypointData> GenerateP2ShapeSkeleton(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        P2SkeletonConfig p2Cfg,
        out P2ExecutionResult p2Exec,
        out float genTimeSec)
    {
        float t0 = Time.realtimeSinceStartup;
        p2Exec = new P2ExecutionResult();
        var strokesWorld = new List<List<Vector3>>();

        if (ctrl == null)
        {
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        if (p2Cfg == null) p2Cfg = new P2SkeletonConfig();
        float spacingMm = p2Cfg.EffectiveSpacingSMm;
        p2Exec.effectiveSpacingMm = spacingMm;
        float stepSM = Mathf.Clamp(spacingMm, 6f, 110f) * 0.001f;
        float borderEMm = Mathf.Clamp(p2Cfg.borderMarginEMm, 4f, 40f);
        float minPassLenM = Mathf.Clamp(p2Cfg.minPassLengthMm, 12f, 180f) * 0.001f;
        float sampleStepM = 0.009f; // Lấy mẫu mịn 9mm để cắt biên bật/tắt súng sắc nét

        bool has12Landmarks = (ctrl.landmarksWorldPos != null && ctrl.landmarksWorldPos.Count >= 12);
        Vector2[] src12 = DobotCR10AIKController.GetCanonicalLandmarks12UV(templateSize);
        Vector2[] dst12XZ = null;
        if (has12Landmarks)
        {
            dst12XZ = new Vector2[12];
            for (int i = 0; i < 12; i++) dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);
        }

        // Bước 1: Tách đầy đủ 2 vùng Mask ống quần trái/phải từ toàn bộ `garmentContourWorld` (đùi -> gấu, KHÔNG bị bó vào ô đùi nhỏ!)
        var legMaskPolys = ExtractLegMaskPolygonsFromMask2D(ctrl, 0.004f, preferFullGarmentSplit: true);
        if (legMaskPolys.Count == 0 && has12Landmarks)
        {
            AnatomicalLegChain chL = BuildLegChain(dst12XZ, true);
            AnatomicalLegChain chR = BuildLegChain(dst12XZ, false);
            var pL = BuildAnatomicalLegStripPolygon2D(chL, 0.65f, 2.75f, -0.88f, 0.88f);
            var pR = BuildAnatomicalLegStripPolygon2D(chR, 0.65f, 2.75f, -0.88f, 0.88f);
            if (pL != null && pL.Count >= 3) legMaskPolys.Add(pL);
            if (pR != null && pR.Count >= 3) legMaskPolys.Add(pR);
        }
        if (legMaskPolys.Count == 0)
        {
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        // Bước 2: Tính Trục giữa THẬT (Skeleton / Medial Axis) trực tiếp từ Mask nhị phân + Exact Distance Transform (EDT)
        var legSpines = new List<MaskSkeletonSpine>();
        var validLegMasks = new List<List<Vector2>>();
        for (int li = 0; li < legMaskPolys.Count; li++)
        {
            if (ExtractRealMaskSkeletonByEDT(legMaskPolys[li], 0.006f, out MaskSkeletonSpine sp))
            {
                legSpines.Add(sp);
                validLegMasks.Add(legMaskPolys[li]);
            }
        }
        if (legSpines.Count == 0)
        {
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        // Xác định hình học "Chỗ cần phun" (theo hình/mẫu đã vẽ hoặc toàn mask 2 ống quần)
        var targetPolysXZ = new List<List<Vector2>>();
        bool flat2DMode = (p2Cfg.gunOrientation == P2GunOrientationMode.FlatPlane2D);
        float flatY = (plyLoader != null) ? (plyLoader.estimatedTableY + 0.025f) : 0.025f;
        if (has12Landmarks)
        {
            float sumY = 0f;
            for (int i = 0; i < 12; i++) sumY += ctrl.landmarksWorldPos[i].y;
            flatY = sumY / 12f;
        }
        else if (ctrl.garmentContourWorld != null && ctrl.garmentContourWorld.Count >= 3)
        {
            float sumY = 0f;
            for (int i = 0; i < ctrl.garmentContourWorld.Count; i++) sumY += ctrl.garmentContourWorld[i].y;
            flatY = sumY / ctrl.garmentContourWorld.Count;
        }

        if (p2Cfg.regionScope == P2RegionScopeMode.FromTemplateRegion)
        {
            if (ctrl.HasCustomDrawnRegions2D)
            {
                var directRegionsXZ = ctrl.GetCustomDrawnRegionPolygonsWorldXZ(plyLoader);
                for (int i = 0; i < directRegionsXZ.Count; i++)
                {
                    if (directRegionsXZ[i] != null && directRegionsXZ[i].Count >= 3)
                        targetPolysXZ.Add(directRegionsXZ[i]);
                }
            }
            else if (has12Landmarks && templateStrokesUV != null && templateStrokesUV.Count > 0)
            {
                var closedUserLoops = new List<List<Vector2>>();
                var openStrokesWorld2D = new List<List<Vector2>>();
                float dpEpsM = 0.003f;

                for (int s = 0; s < templateStrokesUV.Count; s++)
                {
                    var st = templateStrokesUV[s];
                    if (st == null || st.Count < 2) continue;

                    var warped = new List<Vector2>(st.Count);
                    float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
                    float arcLen = 0f;
                    for (int p = 0; p < st.Count; p++)
                    {
                        Vector2 wp = WarpPointBarycentric12(st[p], src12, dst12XZ);
                        if (warped.Count > 0) arcLen += Vector2.Distance(warped[warped.Count - 1], wp);
                        warped.Add(wp);
                        if (wp.x < minX) minX = wp.x; if (wp.x > maxX) maxX = wp.x;
                        if (wp.y < minY) minY = wp.y; if (wp.y > maxY) maxY = wp.y;
                    }
                    float diag = Mathf.Sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY));
                    float endGap = Vector2.Distance(warped[0], warped[warped.Count - 1]);
                    float area = ComputePolygonAreaSqM(warped);

                    bool isLoopShape = (warped.Count >= 6 && diag > 0.020f && area >= 0.00020f)
                        && (endGap <= Mathf.Max(0.065f, diag * 0.42f) || (endGap > 1e-4f && arcLen / endGap >= 2.2f));
                    if (isLoopShape) closedUserLoops.Add(warped);
                    else openStrokesWorld2D.Add(warped);
                }

                for (int i = 0; i < closedUserLoops.Count; i++)
                {
                    var simp = SimplifyClosedPolygonDouglasPeucker(closedUserLoops[i], dpEpsM);
                    if (simp.Count >= 3) targetPolysXZ.Add(simp);
                }

                if (openStrokesWorld2D.Count > 0)
                {
                    float bandRadiusM = Mathf.Clamp(p2Cfg.openStrokeSprayWidthMm * 0.5f, 6f, 60f) * 0.001f;
                    var strokePolys = BuildMaskPolygonsFromOpenStrokes2D(openStrokesWorld2D, bandRadiusM, dpEpsM);
                    for (int i = 0; i < strokePolys.Count; i++)
                    {
                        if (strokePolys[i] != null && strokePolys[i].Count >= 3)
                            targetPolysXZ.Add(strokePolys[i]);
                    }
                }
            }
        }

        bool isFullMaskMode = (p2Cfg.regionScope == P2RegionScopeMode.FullLegMask || targetPolysXZ.Count == 0);
        if (isFullMaskMode)
        {
            targetPolysXZ.Clear();
            for (int li = 0; li < validLegMasks.Count; li++) targetPolysXZ.Add(validLegMasks[li]);
        }

        // Lưu đa giác 3D "Chỗ cần phun" và Trục giữa (Skeleton) thật của 2 ống quần trên bề mặt vải
        for (int i = 0; i < targetPolysXZ.Count; i++)
        {
            p2Exec.targetRegionPolygonsWorld.Add(LiftPolygon2DToWorldSurface(targetPolysXZ[i], ctrl, plyLoader, flat2DMode, flatY));
        }
        for (int li = 0; li < legSpines.Count; li++)
        {
            var sp = legSpines[li];
            var skelPts = new List<Vector3>(sp.C.Count);
            for (int k = 0; k < sp.C.Count; k++)
            {
                Vector2 cPt = sp.C[k];
                float yC = flat2DMode ? flatY : SampleSurfaceHeightOnly(ctrl, plyLoader, cPt.x, cPt.y);
                skelPts.Add(new Vector3(cPt.x, yC + 0.003f, cPt.y));
            }
            p2Exec.skeletonAxesWorld.Add(skelPts);
        }

        // Bước 3..6: Sinh các lượt QUỸ ĐẠO QUÉT dọc theo Trục giữa Mask thật (Cách A) hoặc Offset đồng tâm từ biên (Cách B)
        var perPassSprayFlags = new List<List<bool>>();
        float[] marginAttempts = { borderEMm, borderEMm * 0.35f, 0f };
        for (int attempt = 0; attempt < marginAttempts.Length && strokesWorld.Count == 0; attempt++)
        {
            float activeMargin = marginAttempts[attempt];
            List<Vector3> activeContour = (attempt < 2) ? ctrl.garmentContourWorld : null;
            float marginM = activeMargin * 0.001f;
            p2Exec.leftLegPassCount = 0;
            p2Exec.rightLegPassCount = 0;
            p2Exec.truncatedOuterPassCount = 0;

            for (int leg = 0; leg < legSpines.Count; leg++)
            {
                MaskSkeletonSpine sp = legSpines[leg];
                List<Vector2> legMaskPoly = validLegMasks[leg];
                int steps = Mathf.Clamp(Mathf.CeilToInt(sp.totalLengthM / sampleStepM), 24, 120);
                var candidateLegPaths = new List<List<Vector3>>();

                if (p2Cfg.geometryMode == P2GeometryMode.SkeletonParallel)
                {
                    // CÁCH A — Song song với Trục giữa thật (EDT Medial Axis): khoảng cách kPass * s ra 2 bên
                    int maxAbsIdx = Mathf.Clamp(Mathf.CeilToInt((sp.maxHalfWidthM - marginM * 0.4f) / stepSM) + 1, 1, 14);
                    List<int> passIndices = BuildOrderedPassIndices(-maxAbsIdx, maxAbsIdx, p2Cfg.passOrdering);

                    for (int pIdx = 0; pIdx < passIndices.Count; pIdx++)
                    {
                        int kPass = passIndices[pIdx];
                        float offsetMeters = kPass * stepSM;
                        var currentSeg = new List<Vector3>();
                        bool wasTruncatedByNarrowing = false;

                        for (int k = 0; k <= steps; k++)
                        {
                            float frac = (float)k / steps;
                            SampleSpineAtT(sp, frac, out Vector2 centerPt, out Vector2 normalVec, out float localHalfWidthM);

                            // Bước 5: Xử lý thay đổi bề rộng thật của mask ống quần — cắt lượt ngoài khi ống thu hẹp xuống gấu
                            float usableHalfWidth = Mathf.Max(0.010f, localHalfWidthM - marginM * 0.55f);
                            if (Mathf.Abs(offsetMeters) > usableHalfWidth)
                            {
                                wasTruncatedByNarrowing = true;
                                FlushValidCandidatePath(currentSeg, minPassLenM, candidateLegPaths);
                                currentSeg = new List<Vector3>();
                                continue;
                            }

                            Vector2 ptXZ = centerPt + normalVec * offsetMeters;
                            bool inMask = IsPointInPolygonXZ(ptXZ, legMaskPoly);
                            bool inSafe = inMask && (activeContour == null || IsInsideValidSprayZone(ptXZ.x, ptXZ.y, activeContour, ctrl.landmarksWorldPos, activeMargin, out _));
                            if (inSafe)
                            {
                                currentSeg.Add(new Vector3(ptXZ.x, 0.02f, ptXZ.y));
                            }
                            else
                            {
                                FlushValidCandidatePath(currentSeg, minPassLenM, candidateLegPaths);
                                currentSeg = new List<Vector3>();
                            }
                        }
                        FlushValidCandidatePath(currentSeg, minPassLenM, candidateLegPaths);
                        if (wasTruncatedByNarrowing && kPass != 0) p2Exec.truncatedOuterPassCount++;
                    }
                }
                else
                {
                    // CÁCH B — Offset đồng tâm từ biên mask thật vào phía trong theo từng bước ring * s
                    int maxRings = Mathf.Clamp(Mathf.FloorToInt((sp.maxHalfWidthM - marginM * 0.45f) / stepSM) + 1, 1, 10);

                    for (int ring = 0; ring < maxRings; ring++)
                    {
                        float inwardOffsetM = ring * stepSM;
                        bool anyTruncatedInRing = false;

                        for (int side = -1; side <= 1; side += 2)
                        {
                            var currentSeg = new List<Vector3>();
                            for (int k = 0; k <= steps; k++)
                            {
                                float frac = (float)k / steps;
                                SampleSpineAtT(sp, frac, out Vector2 centerPt, out Vector2 normalVec, out float localHalfWidthM);

                                float outerBoundM = (side < 0)
                                    ? -(localHalfWidthM - marginM * 0.55f)
                                    : +(localHalfWidthM - marginM * 0.55f);
                                float offsetMeters = (side < 0) ? (outerBoundM + inwardOffsetM) : (outerBoundM - inwardOffsetM);

                                if ((side < 0 && offsetMeters > -0.25f * stepSM && ring > 0) ||
                                    (side > 0 && offsetMeters < +0.25f * stepSM && ring > 0))
                                {
                                    anyTruncatedInRing = true;
                                    FlushValidCandidatePath(currentSeg, minPassLenM, candidateLegPaths);
                                    currentSeg = new List<Vector3>();
                                    continue;
                                }

                                Vector2 ptXZ = centerPt + normalVec * offsetMeters;
                                bool inMask = IsPointInPolygonXZ(ptXZ, legMaskPoly);
                                bool inSafe = inMask && (activeContour == null || IsInsideValidSprayZone(ptXZ.x, ptXZ.y, activeContour, ctrl.landmarksWorldPos, activeMargin, out _));
                                if (inSafe)
                                {
                                    currentSeg.Add(new Vector3(ptXZ.x, 0.02f, ptXZ.y));
                                }
                                else
                                {
                                    FlushValidCandidatePath(currentSeg, minPassLenM, candidateLegPaths);
                                    currentSeg = new List<Vector3>();
                                }
                            }
                            FlushValidCandidatePath(currentSeg, minPassLenM, candidateLegPaths);
                        }
                        if (anyTruncatedInRing) p2Exec.truncatedOuterPassCount++;
                    }
                }

                // Bước 6: Sắp xếp thứ tự và đảo chiều đầu-cuối các lượt để đi quét qua lại liên tục kiểu máy in 3D
                List<List<Vector3>> orderedLegPaths = OrderPathsForP2(candidateLegPaths, p2Cfg.passOrdering);

                // Bước 6b: Gắn cờ BẬT/TẮT súng chính xác tại giao cắt với Chỗ cần phun (targetPolysXZ)
                var legFinalPaths = new List<List<Vector3>>();
                var legFinalFlags = new List<List<bool>>();
                bool gateByTarget = (!isFullMaskMode && targetPolysXZ.Count > 0);

                for (int pi = 0; pi < orderedLegPaths.Count; pi++)
                {
                    if (!gateByTarget)
                    {
                        var fullPass = orderedLegPaths[pi];
                        var allOn = new List<bool>(fullPass.Count);
                        for (int k = 0; k < fullPass.Count; k++) allOn.Add(k > 0);
                        legFinalPaths.Add(fullPass);
                        legFinalFlags.Add(allOn);
                    }
                    else if (p2Cfg.continuousScanLike3DPrinter)
                    {
                        GateContinuousScanPassWithTarget(
                            orderedLegPaths[pi], targetPolysXZ,
                            out List<Vector3> gatedPts, out List<bool> gatedFlags, out bool anySprayOn);
                        if (anySprayOn && gatedPts.Count >= 2)
                        {
                            legFinalPaths.Add(gatedPts);
                            legFinalFlags.Add(gatedFlags);
                        }
                    }
                    else
                    {
                        ExtractClippedSpraySegmentsFromPass(orderedLegPaths[pi], targetPolysXZ, 0.004f, out List<List<Vector3>> clippedSegs);
                        for (int ci = 0; ci < clippedSegs.Count; ci++)
                        {
                            var segPts = clippedSegs[ci];
                            var segFlags = new List<bool>(segPts.Count);
                            for (int k = 0; k < segPts.Count; k++) segFlags.Add(k > 0);
                            legFinalPaths.Add(segPts);
                            legFinalFlags.Add(segFlags);
                        }
                    }
                }

                if (leg == 0) p2Exec.leftLegPassCount = legFinalPaths.Count;
                else p2Exec.rightLegPassCount += legFinalPaths.Count;

                strokesWorld.AddRange(legFinalPaths);
                perPassSprayFlags.AddRange(legFinalFlags);
            }
        }

        // Bước 7 & 8: Gán tọa độ Z, hướng súng, bật/tắt súng theo từng đoạn và kiểm tra quỹ đạo
        bool useFixedVert = (p2Cfg.gunOrientation != P2GunOrientationMode.SmoothSurfaceNormal25D);
        bool useFlat2D = (p2Cfg.gunOrientation == P2GunOrientationMode.FlatPlane2D);

        var wps = BuildSprayWaypointsFromWorldStrokes(
            strokesWorld, ctrl, plyLoader, tech,
            p2Cfg.geometryMode == P2GeometryMode.SkeletonParallel ? "P2_SKELETON" : "P2_OFFSET",
            useFixedVerticalNormal: useFixedVert,
            useFlatPlaneZ: useFlat2D,
            flatPlaneWorldY: flatY,
            overrideStandoffMm: p2Cfg.sprayDistanceDMm,
            perPointSprayOn: perPassSprayFlags);

        // Trích xuất các đoạn BẬT súng (Đường phun) vs đoạn TẮT súng (Đường quỹ đạo không phun / quay đầu)
        PopulateP2ExecutionMetricsFromWaypoints(wps, p2Exec);

        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
    }


    private static bool IsPointInAnyPolygonXZ(Vector2 ptXZ, List<List<Vector2>> polys)
    {
        if (polys == null) return false;
        for (int i = 0; i < polys.Count; i++)
        {
            var poly = polys[i];
            if (poly != null && poly.Count >= 3 && IsPointInPolygonXZ(ptXZ, poly)) return true;
        }
        return false;
    }

    /// <summary>
    /// Kiểu máy in 3D: giữ nguyên toàn bộ lượt quét liên tục qua lại, tìm chính xác điểm cắt mép
    /// Chỗ cần phun (binary search 6 bước) để bật súng (sprayOn = true) ĐÚNG trong vùng mẫu và tắt súng (sprayOn = false) ngoài vùng mẫu.
    /// </summary>
    private static void GateContinuousScanPassWithTarget(
        List<Vector3> passPts,
        List<List<Vector2>> targetPolysXZ,
        out List<Vector3> outPts,
        out List<bool> outFlags,
        out bool anySprayOn)
    {
        outPts = new List<Vector3>();
        outFlags = new List<bool>();
        anySprayOn = false;
        if (passPts == null || passPts.Count < 2) return;

        Vector3 prevP = passPts[0];
        bool prevIn = IsPointInAnyPolygonXZ(new Vector2(prevP.x, prevP.z), targetPolysXZ);
        outPts.Add(prevP);
        outFlags.Add(false); // Điểm bắt đầu lượt luôn là di chuyển tới (TẮT súng)

        for (int i = 1; i < passPts.Count; i++)
        {
            Vector3 curP = passPts[i];
            bool curIn = IsPointInAnyPolygonXZ(new Vector2(curP.x, curP.z), targetPolysXZ);

            if (curIn != prevIn)
            {
                // Tìm chính xác điểm cắt biên giữa prevP và curP
                float lo = 0f, hi = 1f;
                for (int it = 0; it < 6; it++)
                {
                    float mid = 0.5f * (lo + hi);
                    Vector3 mp = Vector3.Lerp(prevP, curP, mid);
                    bool mIn = IsPointInAnyPolygonXZ(new Vector2(mp.x, mp.z), targetPolysXZ);
                    if (mIn == prevIn) lo = mid;
                    else hi = mid;
                }
                Vector3 crossP = Vector3.Lerp(prevP, curP, 0.5f * (lo + hi));
                if (Vector3.Distance(outPts[outPts.Count - 1], crossP) > 0.0008f)
                {
                    // Đoạn từ prevP -> crossP mang trạng thái của prevIn
                    outPts.Add(crossP);
                    outFlags.Add(prevIn);
                    if (prevIn) anySprayOn = true;
                }
                // Đoạn từ crossP -> curP mang trạng thái của curIn
                if (Vector3.Distance(outPts[outPts.Count - 1], curP) > 0.0008f)
                {
                    outPts.Add(curP);
                    outFlags.Add(curIn);
                    if (curIn) anySprayOn = true;
                }
            }
            else
            {
                outPts.Add(curP);
                outFlags.Add(curIn);
                if (curIn) anySprayOn = true;
            }

            prevP = curP;
            prevIn = curIn;
        }
    }

    private static void ExtractClippedSpraySegmentsFromPass(
        List<Vector3> passPts,
        List<List<Vector2>> targetPolysXZ,
        float minOnSegLenM,
        out List<List<Vector3>> outSegments)
    {
        outSegments = new List<List<Vector3>>();
        GateContinuousScanPassWithTarget(passPts, targetPolysXZ, out List<Vector3> pts, out List<bool> flags, out bool anyOn);
        if (!anyOn || pts.Count < 2) return;

        List<Vector3> cur = null;
        for (int i = 1; i < pts.Count; i++)
        {
            if (flags[i])
            {
                if (cur == null) cur = new List<Vector3> { pts[i - 1] };
                cur.Add(pts[i]);
            }
            else if (cur != null)
            {
                FlushValidCandidatePath(cur, minOnSegLenM, outSegments);
                cur = null;
            }
        }
        if (cur != null) FlushValidCandidatePath(cur, minOnSegLenM, outSegments);
    }

    private static float SampleSurfaceHeightOnly(DobotCR10AIKController ctrl, PLYDirectLoader plyLoader, float wx, float wz)
    {
        if (ctrl != null && ctrl.SampleCompensatedSurface(wx, wz, out float cy, out _)) return cy;
        if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(wx, wz, out float py, out _)) return py;
        return 0.020f;
    }

    private static void FlushValidCandidatePath(List<Vector3> seg, float minLenM, List<List<Vector3>> outList)
    {
        if (seg == null || seg.Count < 2) return;
        float lenM = 0f;
        for (int i = 1; i < seg.Count; i++) lenM += Vector3.Distance(seg[i - 1], seg[i]);
        if (lenM >= minLenM)
        {
            outList.Add(new List<Vector3>(seg));
        }
    }

    private static List<int> BuildOrderedPassIndices(int minIdx, int maxIdx, P2PassOrderingMode mode)
    {
        var list = new List<int>();
        if (mode == P2PassOrderingMode.CenterOutAlternating)
        {
            if (0 >= minIdx && 0 <= maxIdx) list.Add(0);
            int maxAbs = Mathf.Max(Mathf.Abs(minIdx), Mathf.Abs(maxIdx));
            for (int d = 1; d <= maxAbs; d++)
            {
                if (-d >= minIdx && -d <= maxIdx) list.Add(-d);
                if (+d >= minIdx && +d <= maxIdx) list.Add(+d);
            }
        }
        else if (mode == P2PassOrderingMode.OuterToInner)
        {
            int l = minIdx, r = maxIdx;
            while (l <= r)
            {
                list.Add(l);
                if (r != l) list.Add(r);
                l++;
                r--;
            }
        }
        else
        {
            for (int k = minIdx; k <= maxIdx; k++) list.Add(k);
        }
        return list;
    }

    private static List<List<Vector3>> OrderPathsForP2(List<List<Vector3>> paths, P2PassOrderingMode mode)
    {
        if (paths == null || paths.Count <= 1) return paths ?? new List<List<Vector3>>();

        if (mode != P2PassOrderingMode.MinimumTravelGreedy)
        {
            var alt = new List<List<Vector3>>(paths.Count);
            for (int i = 0; i < paths.Count; i++)
            {
                var p = new List<Vector3>(paths[i]);
                if (i % 2 == 1) p.Reverse();
                alt.Add(p);
            }
            return alt;
        }

        // Greedy nearest-neighbor endpoint ordering (order_for_minimum_travel)
        var remaining = new List<List<Vector3>>(paths);
        var ordered = new List<List<Vector3>>(paths.Count);
        ordered.Add(new List<Vector3>(remaining[0]));
        remaining.RemoveAt(0);

        while (remaining.Count > 0)
        {
            Vector3 lastEnd = ordered[ordered.Count - 1][ordered[ordered.Count - 1].Count - 1];
            int bestIdx = 0;
            bool bestReverse = false;
            float bestDist = float.MaxValue;

            for (int i = 0; i < remaining.Count; i++)
            {
                var cand = remaining[i];
                float dStart = Vector3.Distance(lastEnd, cand[0]);
                float dEnd = Vector3.Distance(lastEnd, cand[cand.Count - 1]);
                if (dStart < bestDist)
                {
                    bestDist = dStart;
                    bestIdx = i;
                    bestReverse = false;
                }
                if (dEnd < bestDist)
                {
                    bestDist = dEnd;
                    bestIdx = i;
                    bestReverse = true;
                }
            }

            var chosen = new List<Vector3>(remaining[bestIdx]);
            if (bestReverse) chosen.Reverse();
            ordered.Add(chosen);
            remaining.RemoveAt(bestIdx);
        }

        return ordered;
    }

    private static void PopulateP2ExecutionMetricsFromWaypoints(
        List<DobotCR10AIKController.WaypointData> wps,
        P2ExecutionResult exec)
    {
        if (exec == null) return;
        exec.sprayOnPathsWorld.Clear();
        exec.sprayOffConnectorsWorld.Clear();
        exec.totalSprayLengthMm = 0f;
        exec.totalTravelLengthMm = 0f;
        exec.turnaroundCount = 0;
        exec.sprayOnSegmentCount = 0;

        if (wps == null || wps.Count < 2) return;

        List<Vector3> curSprayOn = null;
        List<Vector3> curTravelOff = null;
        for (int i = 1; i < wps.Count; i++)
        {
            float dMm = Vector3.Distance(wps[i - 1].targetPosGlobal_mm, wps[i].targetPosGlobal_mm);
            bool sameSeg = string.IsNullOrEmpty(wps[i - 1].segmentName) || string.IsNullOrEmpty(wps[i].segmentName)
                || wps[i - 1].segmentName == wps[i].segmentName;
            bool isOn = wps[i].sprayOn && sameSeg;

            if (isOn)
            {
                if (curTravelOff != null && curTravelOff.Count >= 2)
                {
                    exec.sprayOffConnectorsWorld.Add(curTravelOff);
                }
                curTravelOff = null;

                exec.totalSprayLengthMm += dMm;
                if (curSprayOn == null) curSprayOn = new List<Vector3> { wps[i - 1].targetPosWorld_m };
                curSprayOn.Add(wps[i].targetPosWorld_m);
            }
            else
            {
                if (curSprayOn != null && curSprayOn.Count >= 2)
                {
                    exec.sprayOnPathsWorld.Add(curSprayOn);
                }
                curSprayOn = null;

                exec.totalTravelLengthMm += dMm;
                if (!sameSeg) exec.turnaroundCount++;

                if (curTravelOff == null) curTravelOff = new List<Vector3> { wps[i - 1].targetPosWorld_m };
                curTravelOff.Add(wps[i].targetPosWorld_m);
            }
        }
        if (curSprayOn != null && curSprayOn.Count >= 2) exec.sprayOnPathsWorld.Add(curSprayOn);
        if (curTravelOff != null && curTravelOff.Count >= 2) exec.sprayOffConnectorsWorld.Add(curTravelOff);
        exec.sprayOnSegmentCount = exec.sprayOnPathsWorld.Count;

        exec.summaryText =
            $"Lượt quét: Trái {exec.leftLegPassCount} + Phải {exec.rightLegPassCount} | Đoạn phun ON: {exec.sprayOnSegmentCount} | " +
            $"Phun: {exec.totalSprayLengthMm:F0} mm | Quỹ đạo tắt: {exec.totalTravelLengthMm:F0} mm";
    }

    /// <summary>
    /// Phát hiện tự động 12 mốc hình học trực tiếp trên Contour quần (điểm lồi/lõm, điểm chẻ đáy đũng, giao trục giữa)
    /// KHÔNG cần mô hình keypoint YOLO11-pose (đúng Mục 2 & Bước 3 tài liệu P3).
    /// </summary>
    public static bool DetectLandmarksFromContourGeometry(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        out Vector2[] landmarksXZ,
        out List<Vector3> landmarksWorld)
    {
        landmarksXZ = null;
        landmarksWorld = new List<Vector3>();
        if (ctrl == null || ctrl.garmentContourWorld == null || ctrl.garmentContourWorld.Count < 8) return false;

        var rawG = new List<Vector2>(ctrl.garmentContourWorld.Count);
        for (int i = 0; i < ctrl.garmentContourWorld.Count; i++)
            rawG.Add(new Vector2(ctrl.garmentContourWorld[i].x, ctrl.garmentContourWorld[i].z));

        ComputePcaAxes2D(rawG, Vector2.up, out Vector2 centroid, out Vector2 majorAx, out Vector2 minorAx);
        float minU = float.MaxValue, maxU = float.MinValue;
        float minVAll = float.MaxValue, maxVAll = float.MinValue;
        for (int i = 0; i < rawG.Count; i++)
        {
            Vector2 rel = rawG[i] - centroid;
            float u = Vector2.Dot(rel, majorAx);
            float v = Vector2.Dot(rel, minorAx);
            if (u < minU) minU = u; if (u > maxU) maxU = u;
            if (v < minVAll) minVAll = v; if (v > maxVAll) maxVAll = v;
        }
        float uSpan = Mathf.Max(0.10f, maxU - minU);

        // Xác định chiều Cạp (liền khối ở trục giữa v=0) -> Gấu (tách 2 ống trống ở v=0)
        int midAtMin = 0, midAtMax = 0;
        for (int k = 1; k <= 5; k++)
        {
            float frac = 0.08f + 0.04f * k;
            if (IsPointInPolygonXZ(centroid + majorAx * (minU + frac * uSpan), rawG)) midAtMin++;
            if (IsPointInPolygonXZ(centroid + majorAx * (maxU - frac * uSpan), rawG)) midAtMax++;
        }
        if (midAtMax > midAtMin)
        {
            majorAx = -majorAx;
            minorAx = -minorAx;
            float oldMin = minU; minU = -maxU; maxU = -oldMin;
            float oldMinV = minVAll; minVAll = -maxVAll; maxVAll = -oldMinV;
        }

        // Tìm điểm lõm đáy đũng (crotch notch) dọc theo trục giữa v = 0
        float uCrotch = minU + 0.34f * uSpan;
        for (int s = 12; s <= 62; s++)
        {
            float uTest = minU + (s / 100f) * uSpan;
            if (IsPointInPolygonXZ(centroid + majorAx * uTest, rawG)) uCrotch = uTest;
            else if (uTest > minU + 0.22f * uSpan) break;
        }

        float vSearchMax = Mathf.Max(0.28f, Mathf.Max(Mathf.Abs(minVAll), Mathf.Abs(maxVAll)) + 0.03f);
        Func<float, float, float, Vector2> findExtremeV = (uPos, vLo, vHi) =>
        {
            float bestMin = float.MaxValue, bestMax = float.MinValue;
            for (int k = 0; k <= 48; k++)
            {
                float v = Mathf.Lerp(vLo, vHi, k / 48f);
                if (IsPointInPolygonXZ(centroid + majorAx * uPos + minorAx * v, rawG))
                {
                    if (v < bestMin) bestMin = v;
                    if (v > bestMax) bestMax = v;
                }
            }
            if (bestMin > bestMax) return new Vector2(0.5f * (vLo + vHi), 0.5f * (vLo + vHi));
            return new Vector2(bestMin, bestMax);
        };

        float uWaist = minU + 0.04f * uSpan;
        Vector2 wSpan = findExtremeV(uWaist, -vSearchMax, vSearchMax);
        Vector2 waistL = centroid + majorAx * uWaist + minorAx * wSpan.x;
        Vector2 waistC = centroid + majorAx * uWaist + minorAx * (0.5f * (wSpan.x + wSpan.y));
        Vector2 waistR = centroid + majorAx * uWaist + minorAx * wSpan.y;
        Vector2 crotch = centroid + majorAx * uCrotch;

        float legLenU = Mathf.Max(0.15f, (maxU - 0.03f * uSpan) - uCrotch);
        float uThigh = uCrotch + 0.26f * legLenU;
        float uKnee = uCrotch + 0.58f * legLenU;
        float uCuff = maxU - 0.03f * uSpan;

        Vector2 thL = findExtremeV(uThigh, -vSearchMax, -0.004f);
        Vector2 thR = findExtremeV(uThigh, 0.004f, vSearchMax);
        Vector2 knL = findExtremeV(uKnee, -vSearchMax, -0.004f);
        Vector2 knR = findExtremeV(uKnee, 0.004f, vSearchMax);
        Vector2 cfL = findExtremeV(uCuff, -vSearchMax, -0.004f);
        Vector2 cfR = findExtremeV(uCuff, 0.004f, vSearchMax);

        landmarksXZ = new Vector2[12]
        {
            waistL,
            waistC,
            waistR,
            crotch,
            centroid + majorAx * uThigh + minorAx * (0.5f * (thL.x + thL.y)), // 4: thigh_left
            centroid + majorAx * uThigh + minorAx * (0.5f * (thR.x + thR.y)), // 5: thigh_right
            centroid + majorAx * uKnee  + minorAx * (0.5f * (knL.x + knL.y)), // 6: knee_left
            centroid + majorAx * uKnee  + minorAx * (0.5f * (knR.x + knR.y)), // 7: knee_right
            centroid + majorAx * uCuff  + minorAx * cfL.x,                    // 8: cuff_left_outer
            centroid + majorAx * uCuff  + minorAx * cfL.y,                    // 9: cuff_left_inner
            centroid + majorAx * uCuff  + minorAx * cfR.x,                    // 10: cuff_right_inner
            centroid + majorAx * uCuff  + minorAx * cfR.y                     // 11: cuff_right_outer
        };

        for (int i = 0; i < 12; i++)
        {
            float y = SampleSurfaceHeightOnly(ctrl, plyLoader, landmarksXZ[i].x, landmarksXZ[i].y);
            landmarksWorld.Add(new Vector3(landmarksXZ[i].x, y, landmarksXZ[i].y));
        }
        return true;
    }

    private static float SignedArea2D(Vector2 a, Vector2 b, Vector2 c)
    {
        return 0.5f * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    }

    /// <summary>
    /// Overload tương thích ngược cho bộ so sánh P1 / P2 / P3 / P4.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP3LandmarkTemplate(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        out float genTimeSec)
    {
        var cfg = (ctrl != null && ctrl.p3Config != null) ? ctrl.p3Config : new P3LandmarkConfig
        {
            sprayDistanceDMm = GetEffectiveSprayHeightMm(ctrl, tech),
            borderMarginEMm = tech != null ? tech.borderMarginMm * 0.5f : 10f
        };
        return GenerateP3LandmarkTemplate(ctrl, plyLoader, templateStrokesUV, templateSize, tech, cfg, out _, out genTimeSec);
    }

    /// <summary>
    /// P3 — Quỹ đạo mẫu theo mốc (Tham số hóa Tọa độ Trọng tâm Barycentric trên Lưới 32 Tam Giác, Đầy đủ 8 bước tài liệu P3):
    /// Bước 1-2: Nhận quỹ đạo mẫu & bộ mốc hình học trên quần chuẩn (giữ đúng thứ tự mốc).
    /// Bước 3: Phát hiện mốc trên quần mới bằng AI Keypoint (YOLO11-pose) HOẶC hình học trực tiếp trên Contour.
    /// Bước 4: Chia lưới tam giác (32 tam giác cho 2 ống) trên quần chuẩn và quần mới; kiểm tra tam giác đảo ngược / méo.
    /// Bước 5-6: Tham số hóa từng điểm mẫu bằng tọa độ trọng tâm (alpha, beta, gamma) và tái tạo P' = alpha*A' + beta*B' + gamma*C'.
    /// Bước 7-8: Gán độ cao Z (2.5D/2D), hướng súng (pháp tuyến mượt / thẳng đứng), lùi khoảng d, kiểm tra robot & thống kê Mục 9.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP3LandmarkTemplate(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        P3LandmarkConfig p3Cfg,
        out P3ExecutionResult p3Exec,
        out float genTimeSec)
    {
        float t0 = Time.realtimeSinceStartup;
        p3Exec = new P3ExecutionResult();
        var strokesWorld = new List<List<Vector3>>();

        if (ctrl == null)
        {
            p3Exec.warningMessage = "Thiếu bộ điều khiển phôi.";
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }
        if (p3Cfg == null) p3Cfg = new P3LandmarkConfig();

        // Bước 3: Lấy bộ mốc hình học trên quần mới (AI Keypoint 12 mốc HOẶC hình học trực tiếp trên Contour)
        Vector2[] dst12XZ = null;
        bool useContourLm = (p3Cfg.landmarkSource == P3LandmarkSourceMode.ContourGeometry);
        if (!useContourLm && ctrl.landmarksWorldPos != null && ctrl.landmarksWorldPos.Count >= 12)
        {
            dst12XZ = new Vector2[12];
            for (int i = 0; i < 12; i++)
            {
                dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);
                p3Exec.detectedLandmarksWorld.Add(ctrl.landmarksWorldPos[i]);
            }
            p3Exec.landmarkMethodUsed = "AI Keypoint (12 mốc)";
        }
        else if (DetectLandmarksFromContourGeometry(ctrl, plyLoader, out Vector2[] geoLmXZ, out List<Vector3> geoLmW))
        {
            dst12XZ = geoLmXZ;
            p3Exec.detectedLandmarksWorld.AddRange(geoLmW);
            p3Exec.landmarkMethodUsed = useContourLm ? "Hình học Contour (12 mốc)" : "Hình học Contour (dự phòng)";
        }
        else if (ctrl.landmarksWorldPos != null && ctrl.landmarksWorldPos.Count >= 12)
        {
            dst12XZ = new Vector2[12];
            for (int i = 0; i < 12; i++)
            {
                dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);
                p3Exec.detectedLandmarksWorld.Add(ctrl.landmarksWorldPos[i]);
            }
            p3Exec.landmarkMethodUsed = "AI Keypoint (12 mốc)";
        }

        if (dst12XZ == null)
        {
            p3Exec.warningMessage = "Không phát hiện được bộ mốc hình học cho P3 (cần 12 mốc AI hoặc đường bao contour).";
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }
        p3Exec.landmarkCount = 12;

        // Bước 4: Dựng lưới tam giác chuẩn & mới (16 tam giác ống trái + 16 tam giác ống phải = 32 tam giác), kiểm tra đảo/méo tam giác
        Vector2[] src12 = DobotCR10AIKController.GetCanonicalLandmarks12UV(templateSize);
        bool flat2D = (p3Cfg.gunOrientation == P3GunOrientationMode.FlatPlane2D);
        float flatY = (plyLoader != null) ? (plyLoader.estimatedTableY + 0.025f) : 0.025f;

        p3Exec.totalTriangles = 0;
        p3Exec.flippedTriangleCount = 0;
        p3Exec.maxTriangleDistortion = 1f;
        float totalAreaSrc = 0f, totalAreaDst = 0f;
        var triAreaRatios = new List<float>(32);

        for (int leg = 0; leg < 2; leg++)
        {
            bool isLeft = (leg == 0);
            AnatomicalLegChain sCh = BuildLegChain(src12, isLeft);
            AnatomicalLegChain dCh = BuildLegChain(dst12XZ, isLeft);
            Vector2[] sV = new Vector2[15];
            Vector2[] dV = new Vector2[15];
            BuildLegStructuredVertices15(sCh, sV);
            BuildLegStructuredVertices15(dCh, dV);

            for (int t = 0; t < 16; t++)
            {
                int i0 = LegStripTriangles16[t, 0];
                int i1 = LegStripTriangles16[t, 1];
                int i2 = LegStripTriangles16[t, 2];

                float aSrc = SignedArea2D(sV[i0], sV[i1], sV[i2]);
                float aDst = SignedArea2D(dV[i0], dV[i1], dV[i2]);
                // Trong hệ World XZ so với UV (v đi xuống), xác định dấu nhất quán theo ống (so với tam giác đầu)
                bool flipped = (Mathf.Abs(aDst) < 1e-6f);
                if (!flipped && t > 0)
                {
                    float refSign = SignedArea2D(dV[LegStripTriangles16[0, 0]], dV[LegStripTriangles16[0, 1]], dV[LegStripTriangles16[0, 2]]);
                    if (refSign * aDst < 0f) flipped = true;
                }
                if (flipped) p3Exec.flippedTriangleCount++;

                totalAreaSrc += Mathf.Abs(aSrc);
                totalAreaDst += Mathf.Abs(aDst);
                if (Mathf.Abs(aSrc) > 1e-7f) triAreaRatios.Add(Mathf.Abs(aDst) / Mathf.Abs(aSrc));

                Vector3 w0 = new Vector3(dV[i0].x, (flat2D ? flatY : SampleSurfaceHeightOnly(ctrl, plyLoader, dV[i0].x, dV[i0].y)) + 0.002f, dV[i0].y);
                Vector3 w1 = new Vector3(dV[i1].x, (flat2D ? flatY : SampleSurfaceHeightOnly(ctrl, plyLoader, dV[i1].x, dV[i1].y)) + 0.002f, dV[i1].y);
                Vector3 w2 = new Vector3(dV[i2].x, (flat2D ? flatY : SampleSurfaceHeightOnly(ctrl, plyLoader, dV[i2].x, dV[i2].y)) + 0.002f, dV[i2].y);
                p3Exec.meshTrianglesWorld.Add(new Vector3[] { w0, w1, w2 });
                p3Exec.meshTriangleFlipped.Add(flipped);
                p3Exec.totalTriangles++;
            }
        }

        float meanRatio = (totalAreaSrc > 1e-7f) ? (totalAreaDst / totalAreaSrc) : 1f;
        for (int i = 0; i < triAreaRatios.Count; i++)
        {
            float rel = (meanRatio > 1e-6f) ? (triAreaRatios[i] / meanRatio) : 1f;
            float distor = rel >= 1f ? rel : (rel > 1e-4f ? 1f / rel : 10f);
            if (distor > p3Exec.maxTriangleDistortion) p3Exec.maxTriangleDistortion = distor;
        }
        p3Exec.landmarkOrderConsistent = (p3Exec.flippedTriangleCount == 0);
        if (p3Exec.flippedTriangleCount > 0)
            p3Exec.warningMessage = $"Cảnh báo P3: có {p3Exec.flippedTriangleCount}/{p3Exec.totalTriangles} tam giác bị đảo ngược hoặc suy biến (kiểm tra thứ tự mốc)!";

        // Bước 5 & 6: Lấy quỹ đạo mẫu (hỗ trợ cả vùng hình Tròn/Vuông/Đa giác đã vẽ LẪN các nét mẫu Canonical UV) và biến đổi Barycentric!
        var activeCanonicalStrokes = new List<List<Vector2>>();
        var regionPolysForOverlay = new List<List<Vector2>>();

        if (ctrl.HasCustomDrawnRegions2D)
        {
            // P3 chuẩn: lấy tọa độ tương đối (canonical UV) của từng hình Tròn/Vuông/Đa giác và biến đổi
            // qua đúng lưới 32 tam giác Barycentric sang quần mới (tự dịch/xoay/co giãn khi đổi phôi!)
            var canonRegionPolysUV = ctrl.GetCustomDrawnRegionCanonicalPolygonsUV();
            float fillStepMm = ctrl.ComputeEffectiveRegionFillSpacingMm();
            for (int r = 0; r < canonRegionPolysUV.Count; r++)
            {
                var canonPoly = canonRegionPolysUV[r];
                if (canonPoly == null || canonPoly.Count < 3) continue;

                var warpedPolyXZ = new List<Vector2>(canonPoly.Count);
                for (int i = 0; i < canonPoly.Count; i++)
                {
                    warpedPolyXZ.Add(WarpPointBarycentric12(canonPoly[i], src12, dst12XZ));
                }
                if (warpedPolyXZ.Count < 3) continue;
                regionPolysForOverlay.Add(warpedPolyXZ);

                var outline = new List<Vector3>(warpedPolyXZ.Count + 1);
                for (int i = 0; i <= warpedPolyXZ.Count; i++)
                {
                    Vector2 p2 = warpedPolyXZ[i % warpedPolyXZ.Count];
                    outline.Add(new Vector3(p2.x, 0.02f, p2.y));
                }
                strokesWorld.Add(outline);

                if (p3Cfg.fillClosedDrawnRegions)
                {
                    var fillPasses = BuildSimpleFillPassesInsidePolygon2D(warpedPolyXZ, fillStepMm * 0.001f);
                    for (int f = 0; f < fillPasses.Count; f++) strokesWorld.Add(fillPasses[f]);
                }
            }
        }
        else if (templateStrokesUV != null && templateStrokesUV.Count > 0)
        {
            activeCanonicalStrokes.AddRange(templateStrokesUV);
        }

        int totalCandidatePts = 0;
        int insideGarmentPts = 0;

        if (strokesWorld.Count == 0 && activeCanonicalStrokes.Count > 0)
        {
            float stepUV = Mathf.Clamp(p3Cfg.resampleStepUV, 0.006f, 0.04f);
            float marginMm = Mathf.Clamp(p3Cfg.borderMarginEMm, 0f, 35f);
            for (int attempt = 0; attempt < 2 && strokesWorld.Count == 0; attempt++)
            {
                totalCandidatePts = 0;
                insideGarmentPts = 0;
                for (int s = 0; s < activeCanonicalStrokes.Count; s++)
                {
                    var raw = activeCanonicalStrokes[s];
                    if (raw == null || raw.Count < 2) continue;
                    var resampled = ResampleUV(raw, stepUV);
                    var worldStroke = new List<Vector3>();

                    for (int p = 0; p < resampled.Count; p++)
                    {
                        // Biến đổi Affine từng mảnh chuẩn Barycentric: P' = alpha*A' + beta*B' + gamma*C'
                        Vector2 wXZ = WarpPointBarycentric12(resampled[p], src12, dst12XZ);
                        Vector3 w = new Vector3(wXZ.x, 0.02f, wXZ.y);
                        totalCandidatePts++;
                        bool isIn = IsInsideValidSprayZone(w.x, w.z, ctrl.garmentContourWorld, ctrl.landmarksWorldPos, marginMm * 0.5f, out _);
                        if (isIn) insideGarmentPts++;

                        bool keep = (!p3Cfg.clipToGarmentContour || attempt == 1) ? true : isIn;
                        if (keep)
                        {
                            worldStroke.Add(w);
                        }
                        else if (worldStroke.Count >= 2)
                        {
                            strokesWorld.Add(worldStroke);
                            worldStroke = new List<Vector3>();
                        }
                        else
                        {
                            worldStroke.Clear();
                        }
                    }
                    if (worldStroke.Count >= 2) strokesWorld.Add(worldStroke);
                }
            }
        }
        else if (strokesWorld.Count > 0)
        {
            for (int s = 0; s < strokesWorld.Count; s++)
            {
                for (int p = 0; p < strokesWorld[s].Count; p++)
                {
                    totalCandidatePts++;
                    if (IsInsideValidSprayZone(strokesWorld[s][p].x, strokesWorld[s][p].z, ctrl.garmentContourWorld, ctrl.landmarksWorldPos, 0f, out _))
                        insideGarmentPts++;
                }
            }
        }

        for (int i = 0; i < regionPolysForOverlay.Count; i++)
            p3Exec.targetRegionPolygonsWorld.Add(LiftPolygon2DToWorldSurface(regionPolysForOverlay[i], ctrl, plyLoader, flat2D, flatY));

        // Bước 7 & 8: Gán độ cao Z, hướng súng, lùi khoảng d, giải IK 6-DOF & tính tiêu chí kiểm tra Mục 9
        bool useFixedVert = (p3Cfg.gunOrientation != P3GunOrientationMode.SmoothSurfaceNormal25D);
        var wps = BuildSprayWaypointsFromWorldStrokes(
            strokesWorld, ctrl, plyLoader, tech, "P3_BARYCENTRIC12",
            useFixedVerticalNormal: useFixedVert,
            useFlatPlaneZ: flat2D,
            flatPlaneWorldY: flatY,
            overrideStandoffMm: p3Cfg.sprayDistanceDMm);

        p3Exec.strokeCount = strokesWorld.Count;
        p3Exec.inTargetRatioPct = totalCandidatePts > 0 ? (100f * insideGarmentPts / totalCandidatePts) : 100f;
        ComputeSprayAndTravelLengths(wps, out p3Exec.totalSprayLengthMm, out p3Exec.totalTravelLengthMm);
        p3Exec.summaryText =
            $"Mốc: {p3Exec.landmarkMethodUsed} | Lưới: {p3Exec.totalTriangles} tam giác (đảo: {p3Exec.flippedTriangleCount}, méo max: {p3Exec.maxTriangleDistortion:F2}x) | " +
            $"Trong vùng: {p3Exec.inTargetRatioPct:F0}% | Phun: {p3Exec.totalSprayLengthMm:F0} mm";

        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
    }


    private static List<List<Vector3>> BuildSimpleFillPassesInsidePolygon2D(List<Vector2> polyXZ, float spacingM)
    {
        var passes = new List<List<Vector3>>();
        if (polyXZ == null || polyXZ.Count < 3 || spacingM <= 1e-4f) return passes;
        ComputePcaAxes2D(polyXZ, Vector2.up, out Vector2 centroid, out Vector2 majorAx, out Vector2 minorAx);
        var polyUV = new List<Vector2>(polyXZ.Count);
        float minV = float.MaxValue, maxV = float.MinValue;
        for (int i = 0; i < polyXZ.Count; i++)
        {
            Vector2 rel = polyXZ[i] - centroid;
            float u = Vector2.Dot(rel, majorAx);
            float v = Vector2.Dot(rel, minorAx);
            polyUV.Add(new Vector2(u, v));
            if (v < minV) minV = v;
            if (v > maxV) maxV = v;
        }

        float spanV = Mathf.Max(0f, maxV - minV);
        if (spanV < 0.002f) return passes;

        int numLines;
        float vFirst, vLast;
        if (spanV <= spacingM * 0.90f)
        {
            numLines = 1;
            vFirst = vLast = 0.5f * (minV + maxV);
        }
        else
        {
            float edgeInset = Mathf.Min(spacingM * 0.30f, spanV * 0.12f);
            vFirst = minV + edgeInset;
            vLast = maxV - edgeInset;
            float innerSpan = Mathf.Max(0f, vLast - vFirst);
            numLines = Mathf.Clamp(Mathf.CeilToInt(innerSpan / spacingM) + 1, 2, 120);
        }

        float alongStepM = Mathf.Clamp(spacingM * 0.75f, 0.005f, 0.014f);
        for (int li = 0; li < numLines; li++)
        {
            float vLine = (numLines == 1)
                ? vFirst
                : Mathf.Lerp(vFirst, vLast, (float)li / (numLines - 1));
            var ivs = IntersectPolygonWithScanlineUV(polyUV, vLine);
            for (int k = 0; k < ivs.Count; k++)
            {
                float u0 = (li % 2 == 0) ? ivs[k].x : ivs[k].y;
                float u1 = (li % 2 == 0) ? ivs[k].y : ivs[k].x;
                int sub = Mathf.Clamp(Mathf.CeilToInt(Mathf.Abs(u1 - u0) / alongStepM), 2, 60);
                var seg = new List<Vector3>(sub + 1);
                for (int p = 0; p <= sub; p++)
                {
                    Vector2 pt = centroid + majorAx * Mathf.Lerp(u0, u1, (float)p / sub) + minorAx * vLine;
                    seg.Add(new Vector3(pt.x, 0.02f, pt.y));
                }
                if (seg.Count >= 2) passes.Add(seg);
            }
        }
        return passes;
    }

    private static void ComputeSprayAndTravelLengths(List<DobotCR10AIKController.WaypointData> wps, out float sprayMm, out float travelMm)
    {
        sprayMm = 0f;
        travelMm = 0f;
        if (wps == null || wps.Count < 2) return;
        for (int i = 1; i < wps.Count; i++)
        {
            float d = Vector3.Distance(wps[i - 1].targetPosGlobal_mm, wps[i].targetPosGlobal_mm);
            bool sameSeg = string.IsNullOrEmpty(wps[i - 1].segmentName) || string.IsNullOrEmpty(wps[i].segmentName)
                || wps[i - 1].segmentName == wps[i].segmentName;
            if (wps[i].sprayOn && sameSeg) sprayMm += d;
            else travelMm += d;
        }
    }

    /// <summary>
    /// Thuật toán ICP cứng 2D (Iterative Closest Point + Kabsch 2D SVD):
    /// Khử sai lệch tịnh tiến & góc xoay cứng giữa đám mây biên mẫu đã khởi tạo P3 và đám mây biên thực của quần mới (voxel downsample ~5 mm).
    /// </summary>
    private static void RunRigidKabschIcp2D(
        List<Vector2> sourceCloud,
        List<Vector2> targetCloud,
        int maxIterations,
        float maxMatchRadiusM,
        out float cosTheta,
        out float sinTheta,
        out Vector2 centroidSrc,
        out Vector2 translation)
    {
        cosTheta = 1f;
        sinTheta = 0f;
        centroidSrc = Vector2.zero;
        translation = Vector2.zero;
        if (sourceCloud == null || targetCloud == null || sourceCloud.Count < 3 || targetCloud.Count < 3) return;

        int n = sourceCloud.Count;
        var curPts = new Vector2[n];
        for (int i = 0; i < n; i++)
        {
            curPts[i] = sourceCloud[i];
            centroidSrc += sourceCloud[i];
        }
        centroidSrc /= n;

        float maxD2 = maxMatchRadiusM * maxMatchRadiusM;
        var matchedSrc = new List<Vector2>(n);
        var matchedDst = new List<Vector2>(n);

        for (int iter = 0; iter < maxIterations; iter++)
        {
            matchedSrc.Clear();
            matchedDst.Clear();

            for (int i = 0; i < n; i++)
            {
                Vector2 p = curPts[i];
                float bestD2 = float.MaxValue;
                Vector2 bestQ = p;
                for (int j = 0; j < targetCloud.Count; j++)
                {
                    float d2 = (targetCloud[j] - p).sqrMagnitude;
                    if (d2 < bestD2) { bestD2 = d2; bestQ = targetCloud[j]; }
                }
                if (bestD2 <= maxD2)
                {
                    matchedSrc.Add(p);
                    matchedDst.Add(bestQ);
                }
            }

            if (matchedSrc.Count < 3) break;

            Vector2 meanP = Vector2.zero, meanQ = Vector2.zero;
            for (int i = 0; i < matchedSrc.Count; i++)
            {
                meanP += matchedSrc[i];
                meanQ += matchedDst[i];
            }
            meanP /= matchedSrc.Count;
            meanQ /= matchedSrc.Count;

            // Kabsch 2D: cực đại hóa trace(R^T * H) -> tan(dTheta) = (sum(px*qy - py*qx)) / (sum(px*qx + py*qy))
            double sDot = 0.0, sCross = 0.0;
            for (int i = 0; i < matchedSrc.Count; i++)
            {
                double px = matchedSrc[i].x - meanP.x;
                double py = matchedSrc[i].y - meanP.y;
                double qx = matchedDst[i].x - meanQ.x;
                double qy = matchedDst[i].y - meanQ.y;
                sDot += px * qx + py * qy;
                sCross += px * qy - py * qx;
            }

            double dAngle = Math.Atan2(sCross, sDot);
            // Giới hạn bước xoay ICP cứng trong vùng hợp lý để tránh trượt biên
            dAngle = Math.Max(-0.18, Math.Min(0.18, dAngle));
            float c = (float)Math.Cos(dAngle);
            float s = (float)Math.Sin(dAngle);

            for (int i = 0; i < n; i++)
            {
                Vector2 rel = curPts[i] - meanP;
                Vector2 rot = new Vector2(c * rel.x - s * rel.y, s * rel.x + c * rel.y);
                curPts[i] = rot + meanQ;
            }

            if (Math.Abs(dAngle) < 1e-4 && (meanQ - meanP).sqrMagnitude < 1e-7f)
                break;
        }

        // Tổng hợp phép biến đổi cứng tổng cộng: p_icp = R_total * (p - centroidSrc) + centroidSrc + translation
        Vector2 meanAfter = Vector2.zero;
        for (int i = 0; i < n; i++) meanAfter += curPts[i];
        meanAfter /= n;

        double totDot = 0.0, totCross = 0.0;
        for (int i = 0; i < n; i++)
        {
            Vector2 a = sourceCloud[i] - centroidSrc;
            Vector2 b = curPts[i] - meanAfter;
            totDot += a.x * b.x + a.y * b.y;
            totCross += a.x * b.y - a.y * b.x;
        }
        double totAngle = Math.Atan2(totCross, totDot);
        cosTheta = (float)Math.Cos(totAngle);
        sinTheta = (float)Math.Sin(totAngle);
        translation = meanAfter - centroidSrc;
    }

    private static Vector2 ApplyRigid2D(Vector2 pt, float cosT, float sinT, Vector2 center, Vector2 trans)
    {
        Vector2 rel = pt - center;
        Vector2 rot = new Vector2(cosT * rel.x - sinT * rel.y, sinT * rel.x + cosT * rel.y);
        return center + rot + trans;
    }

    /// <summary>
    /// Giải phép quay cứng 3D Kabsch (R thuộc SO(3), t thuộc R^3) tối thiểu hóa sum || R * (p_i - cP) + cQ - q_i ||^2
    /// bằng phân rã cực (Polar Decomposition: R = H * (H^T H)^(-1/2)) qua Jacobi 3x3 chuẩn xác.
    /// </summary>
    private static void SolveKabschRigid3D(
        List<Vector3> srcPts,
        List<Vector3> dstPts,
        out Quaternion rotation,
        out Vector3 centroidSrc,
        out Vector3 centroidDst)
    {
        rotation = Quaternion.identity;
        centroidSrc = Vector3.zero;
        centroidDst = Vector3.zero;
        int n = Mathf.Min(srcPts != null ? srcPts.Count : 0, dstPts != null ? dstPts.Count : 0);
        if (n < 3) return;

        for (int i = 0; i < n; i++) { centroidSrc += srcPts[i]; centroidDst += dstPts[i]; }
        centroidSrc /= n;
        centroidDst /= n;

        // Ma trận hiệp phương sai theo đúng chuẩn Horn (1987): M = sum (p_i - cP) * (q_i - cQ)^T  (3x3)
        double[,] H = new double[3, 3];
        for (int i = 0; i < n; i++)
        {
            Vector3 p = srcPts[i] - centroidSrc;
            Vector3 q = dstPts[i] - centroidDst;
            double[] pv = { p.x, p.y, p.z };
            double[] qv = { q.x, q.y, q.z };
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    H[r, c] += pv[r] * qv[c];
        }

        // Horn 1987: tìm quaternion quay tối ưu (eigenvector lớn nhất của ma trận đối xứng 4x4 Davenport/Horn N)
        double Sxx = H[0, 0], Sxy = H[0, 1], Sxz = H[0, 2];
        double Syx = H[1, 0], Syy = H[1, 1], Syz = H[1, 2];
        double Szx = H[2, 0], Szy = H[2, 1], Szz = H[2, 2];

        double[,] N = new double[4, 4]
        {
            { Sxx + Syy + Szz, Syz - Szy,       Szx - Sxz,       Sxy - Syx       },
            { Syz - Szy,       Sxx - Syy - Szz, Sxy + Syx,       Szx + Sxz       },
            { Szx - Sxz,       Sxy + Syx,      -Sxx + Syy - Szz, Syz + Szy       },
            { Sxy - Syx,       Szx + Sxz,       Syz + Szy,      -Sxx - Syy + Szz }
        };

        // Jacobi eigendecomposition cho ma trận đối xứng 4x4 (20 lượt -> chính xác < 1e-12)
        double[,] V = new double[4, 4];
        for (int i = 0; i < 4; i++) V[i, i] = 1.0;

        for (int sweep = 0; sweep < 20; sweep++)
        {
            for (int p = 0; p < 3; p++)
            {
                for (int q = p + 1; q < 4; q++)
                {
                    double apq = N[p, q];
                    if (Math.Abs(apq) < 1e-12) continue;
                    double tau = (N[q, q] - N[p, p]) / (2.0 * apq);
                    double t = (tau >= 0.0)
                        ? (1.0 / (tau + Math.Sqrt(1.0 + tau * tau)))
                        : (-1.0 / (-tau + Math.Sqrt(1.0 + tau * tau)));
                    double c = 1.0 / Math.Sqrt(1.0 + t * t);
                    double s = t * c;

                    double app = N[p, p], aqq = N[q, q];
                    N[p, p] = app - t * apq;
                    N[q, q] = aqq + t * apq;
                    N[p, q] = N[q, p] = 0.0;

                    for (int k = 0; k < 4; k++)
                    {
                        if (k == p || k == q) continue;
                        double akp = N[k, p], akq = N[k, q];
                        N[k, p] = N[p, k] = c * akp - s * akq;
                        N[k, q] = N[q, k] = s * akp + c * akq;
                    }
                    for (int k = 0; k < 4; k++)
                    {
                        double vkp = V[k, p], vkq = V[k, q];
                        V[k, p] = c * vkp - s * vkq;
                        V[k, q] = s * vkp + c * vkq;
                    }
                }
            }
        }

        int bestCol = 0;
        double bestEig = N[0, 0];
        for (int i = 1; i < 4; i++)
        {
            if (N[i, i] > bestEig) { bestEig = N[i, i]; bestCol = i; }
        }

        // Quaternion dạng (w, x, y, z) từ eigenvector cột bestCol + kiểm chứng trực tiếp chiều quay tối ưu
        float qw = (float)V[0, bestCol];
        float qx = (float)V[1, bestCol];
        float qy = (float)V[2, bestCol];
        float qz = (float)V[3, bestCol];
        Quaternion qBest = new Quaternion(qx, qy, qz, qw);
        float mag = Mathf.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        qBest = mag > 1e-6f ? Quaternion.Normalize(qBest) : Quaternion.identity;
        Quaternion qInv = Quaternion.Inverse(qBest);

        float errFwd = 0f, errInv = 0f;
        for (int i = 0; i < n; i++)
        {
            Vector3 p = srcPts[i] - centroidSrc;
            Vector3 q = dstPts[i] - centroidDst;
            errFwd += (qBest * p - q).sqrMagnitude;
            errInv += (qInv * p - q).sqrMagnitude;
        }
        rotation = (errInv < errFwd) ? qInv : qBest;
    }

    private static void RunRigidKabschIcp3D(
        List<Vector3> sourceCloud,
        List<Vector3> targetCloud,
        int maxIterations,
        float maxMatchRadiusM,
        out Quaternion totalRot,
        out Vector3 totalTrans)
    {
        totalRot = Quaternion.identity;
        totalTrans = Vector3.zero;
        if (sourceCloud == null || targetCloud == null || sourceCloud.Count < 6 || targetCloud.Count < 6) return;

        int n = sourceCloud.Count;
        Vector3[] cur = new Vector3[n];
        for (int i = 0; i < n; i++) cur[i] = sourceCloud[i];

        float maxR2 = maxMatchRadiusM * maxMatchRadiusM;
        var matchedSrc = new List<Vector3>(n);
        var matchedDst = new List<Vector3>(n);

        for (int iter = 0; iter < maxIterations; iter++)
        {
            matchedSrc.Clear();
            matchedDst.Clear();
            for (int i = 0; i < n; i++)
            {
                Vector3 p = cur[i];
                float bestD2 = maxR2;
                int bestIdx = -1;
                for (int j = 0; j < targetCloud.Count; j++)
                {
                    float d2 = (targetCloud[j] - p).sqrMagnitude;
                    if (d2 < bestD2) { bestD2 = d2; bestIdx = j; }
                }
                if (bestIdx >= 0)
                {
                    matchedSrc.Add(p);
                    matchedDst.Add(targetCloud[bestIdx]);
                }
            }
            if (matchedSrc.Count < 6) break;

            SolveKabschRigid3D(matchedSrc, matchedDst, out Quaternion dR, out Vector3 cSrc, out Vector3 cDst);
            Vector3 dT = cDst - (dR * cSrc);

            for (int i = 0; i < n; i++) cur[i] = dR * cur[i] + dT;
            totalRot = dR * totalRot;
            totalTrans = dR * totalTrans + dT;

            if (Quaternion.Angle(Quaternion.identity, dR) < 0.05f && dT.magnitude < 0.0005f) break;
        }
    }

    private static void ComputePcaInit3D(
        List<Vector3> srcCloud,
        List<Vector3> dstCloud,
        out Quaternion initRot,
        out Vector3 initCentroidSrc,
        out Vector3 initCentroidDst,
        out float initScale)
    {
        initRot = Quaternion.identity;
        initCentroidSrc = Vector3.zero;
        initCentroidDst = Vector3.zero;
        initScale = 1f;
        if (srcCloud == null || dstCloud == null || srcCloud.Count < 6 || dstCloud.Count < 6) return;

        var src2D = new List<Vector2>(srcCloud.Count);
        var dst2D = new List<Vector2>(dstCloud.Count);
        for (int i = 0; i < srcCloud.Count; i++) { initCentroidSrc += srcCloud[i]; src2D.Add(new Vector2(srcCloud[i].x, srcCloud[i].z)); }
        for (int i = 0; i < dstCloud.Count; i++) { initCentroidDst += dstCloud[i]; dst2D.Add(new Vector2(dstCloud[i].x, dstCloud[i].z)); }
        initCentroidSrc /= srcCloud.Count;
        initCentroidDst /= dstCloud.Count;

        ComputePcaAxes2D(src2D, Vector2.up, out _, out Vector2 uS, out _);
        ComputePcaAxes2D(dst2D, Vector2.up, out _, out Vector2 uD, out _);
        if (Vector2.Dot(uS, uD) < 0f) uD = -uD;

        float angDeg = Vector2.SignedAngle(uS, uD);
        // Trong mặt phẳng XZ, quay quanh trục Y
        initRot = Quaternion.AngleAxis(-angDeg, Vector3.up);

        float radS = 0f, radD = 0f;
        for (int i = 0; i < srcCloud.Count; i++) radS += (srcCloud[i] - initCentroidSrc).magnitude;
        for (int i = 0; i < dstCloud.Count; i++) radD += (dstCloud[i] - initCentroidDst).magnitude;
        radS /= srcCloud.Count;
        radD /= dstCloud.Count;
        if (radS > 1e-4f) initScale = Mathf.Clamp(radD / radS, 0.72f, 1.38f);
    }

    private struct RegularizedTps3DModel
    {
        public bool isValid;
        public bool is25DFlatMode; // true nếu phôi phẳng 2.5D -> tránh suy biến cột Y trong hệ đa thức TPS 3D!
        public Vector3 center;
        public float invScale;
        public List<Vector3> ctrlNorm;
        public double[] wX;
        public double[] wY;
        public double[] wZ;
    }

    private static bool SolveTps3D(
        List<Vector3> ctrlSrc,
        List<Vector3> ctrlDst,
        double lambda,
        out RegularizedTps3DModel model)
    {
        model = new RegularizedTps3DModel { isValid = false };
        int n = Mathf.Min(ctrlSrc != null ? ctrlSrc.Count : 0, ctrlDst != null ? ctrlDst.Count : 0);
        if (n < 4) return false;

        Vector3 minP = ctrlSrc[0], maxP = ctrlSrc[0], meanP = Vector3.zero;
        for (int i = 0; i < n; i++)
        {
            Vector3 p = ctrlSrc[i];
            minP = Vector3.Min(minP, p);
            maxP = Vector3.Max(maxP, p);
            meanP += p;
        }
        meanP /= n;
        float spanX = maxP.x - minP.x;
        float spanY = maxP.y - minP.y;
        float spanZ = maxP.z - minP.z;
        float maxHoriz = Mathf.Max(0.05f, Mathf.Max(spanX, spanZ));
        float scale = Mathf.Max(maxHoriz, spanY);
        float invScale = 1f / scale;

        // Nếu phôi nằm trên bàn phẳng (độ dày Y < 18% kích thước ngang XZ) -> giải TPS 2D chuẩn hóa trên (X, Z) + nội suy mượt Y
        // để loại bỏ hoàn toàn hiện tượng suy biến ma trận (ill-conditioned) làm nổ trọng số!
        bool use25D = (spanY < 0.18f * maxHoriz);

        var ctrlNorm = new List<Vector3>(n);
        for (int i = 0; i < n; i++) ctrlNorm.Add((ctrlSrc[i] - meanP) * invScale);

        int polyDim = use25D ? 3 : 4;
        int dim = n + polyDim;
        double[,] K = new double[dim, dim];
        double[] rhsX = new double[dim];
        double[] rhsY = new double[dim];
        double[] rhsZ = new double[dim];

        double reg = Math.Max(lambda, 0.008);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i == j) K[i, j] = reg;
                else
                {
                    double dx = ctrlNorm[i].x - ctrlNorm[j].x;
                    double dy = ctrlNorm[i].y - ctrlNorm[j].y;
                    double dz = ctrlNorm[i].z - ctrlNorm[j].z;
                    if (use25D)
                    {
                        K[i, j] = TpsKernel(dx * dx + dz * dz);
                    }
                    else
                    {
                        K[i, j] = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                    }
                }
            }

            if (use25D)
            {
                K[i, n] = 1.0; K[i, n + 1] = ctrlNorm[i].x; K[i, n + 2] = ctrlNorm[i].z;
                K[n, i] = 1.0; K[n + 1, i] = ctrlNorm[i].x; K[n + 2, i] = ctrlNorm[i].z;
            }
            else
            {
                K[i, n] = 1.0; K[i, n + 1] = ctrlNorm[i].x; K[i, n + 2] = ctrlNorm[i].y; K[i, n + 3] = ctrlNorm[i].z;
                K[n, i] = 1.0; K[n + 1, i] = ctrlNorm[i].x; K[n + 2, i] = ctrlNorm[i].y; K[n + 3, i] = ctrlNorm[i].z;
            }

            // Giải theo phần dư dịch chuyển (dst - src) để cực kỳ ổn định số học!
            Vector3 disp = ctrlDst[i] - ctrlSrc[i];
            rhsX[i] = disp.x;
            rhsY[i] = disp.y;
            rhsZ[i] = disp.z;
        }
        // Điều chuẩn nhỏ cho khối đa thức chống suy biến
        for (int p = 0; p < polyDim; p++) K[n + p, n + p] = 1e-6;

        if (!SolveLinearSystem(K, rhsX, out double[] wX) ||
            !SolveLinearSystem(K, rhsY, out double[] wY) ||
            !SolveLinearSystem(K, rhsZ, out double[] wZ))
        {
            return false;
        }

        model = new RegularizedTps3DModel
        {
            isValid = true,
            is25DFlatMode = use25D,
            center = meanP,
            invScale = invScale,
            ctrlNorm = ctrlNorm,
            wX = wX,
            wY = wY,
            wZ = wZ
        };
        return true;
    }

    private static Vector3 EvalTps3D(Vector3 q, RegularizedTps3DModel model, float maxDeformMeters = 0.045f)
    {
        if (!model.isValid || model.ctrlNorm == null) return q;
        Vector3 qn = (q - model.center) * model.invScale;
        int n = model.ctrlNorm.Count;

        double dxOut, dyOut, dzOut;
        if (model.is25DFlatMode)
        {
            dxOut = model.wX[n] + model.wX[n + 1] * qn.x + model.wX[n + 2] * qn.z;
            dyOut = model.wY[n] + model.wY[n + 1] * qn.x + model.wY[n + 2] * qn.z;
            dzOut = model.wZ[n] + model.wZ[n + 1] * qn.x + model.wZ[n + 2] * qn.z;
        }
        else
        {
            dxOut = model.wX[n] + model.wX[n + 1] * qn.x + model.wX[n + 2] * qn.y + model.wX[n + 3] * qn.z;
            dyOut = model.wY[n] + model.wY[n + 1] * qn.x + model.wY[n + 2] * qn.y + model.wY[n + 3] * qn.z;
            dzOut = model.wZ[n] + model.wZ[n + 1] * qn.x + model.wZ[n + 2] * qn.y + model.wZ[n + 3] * qn.z;
        }

        for (int i = 0; i < n; i++)
        {
            double dx = qn.x - model.ctrlNorm[i].x;
            double dy = qn.y - model.ctrlNorm[i].y;
            double dz = qn.z - model.ctrlNorm[i].z;
            double k = model.is25DFlatMode ? TpsKernel(dx * dx + dz * dz) : Math.Sqrt(dx * dx + dy * dy + dz * dz);
            dxOut += model.wX[i] * k;
            dyOut += model.wY[i] * k;
            dzOut += model.wZ[i] * k;
        }

        Vector3 disp = new Vector3((float)dxOut, (float)dyOut, (float)dzOut);
        if (float.IsNaN(disp.x) || float.IsNaN(disp.y) || float.IsNaN(disp.z) || float.IsInfinity(disp.sqrMagnitude))
            return q;
        if (maxDeformMeters > 0f && disp.magnitude > maxDeformMeters)
            disp = disp.normalized * maxDeformMeters;
        return q + disp;
    }

    private struct NonRigidCpd3DModel
    {
        public bool isValid;
        public List<Vector3> ctrlY; // Các điểm nguồn đại diện (M <= 42) sau ICP cứng
        public Vector3[] weightsW;  // Ma trận trọng số dịch chuyển W (M x 3)
        public float twoBetaSq;     // 2 * beta^2
    }

    /// <summary>
    /// Đăng ký phi cứng Non-Rigid CPD 3D (Myronenko & Song, 2010 — Coherent Point Drift):
    /// Tối thiểu hóa hàm năng lượng Gaussian Motion Coherence với tham số độ cứng lambda (cpdLambda) và độ mượt beta (cpdBetaM).
    /// Hệ tuyến tính (diag(P1) * G + lambda * sigma^2 * I) * W = PX - diag(P1) * Y luôn xác định dương -> ổn định tuyệt đối trên cả phôi 2.5D và 3D!
    /// </summary>
    private static bool SolveNonRigidCpd3D(
        List<Vector3> alignedSourceCloud,
        List<Vector3> targetCloud,
        float cpdLambda,
        float cpdBetaM,
        int maxIterations,
        out NonRigidCpd3DModel model)
    {
        model = new NonRigidCpd3DModel { isValid = false };
        if (alignedSourceCloud == null || targetCloud == null || alignedSourceCloud.Count < 8 || targetCloud.Count < 8)
            return false;

        // Chọn tối đa M = 40 điểm điều khiển phân bố đều trên đám mây nguồn để giải hệ 40x40 trong ~10 ms
        var Y = new List<Vector3>(42);
        int stepY = Mathf.Max(1, alignedSourceCloud.Count / 40);
        for (int i = 0; i < alignedSourceCloud.Count && Y.Count < 40; i += stepY) Y.Add(alignedSourceCloud[i]);

        // Giảm đám mây đích X xuống <= 220 điểm để E-step nhanh (< 1 ms/vòng)
        var X = new List<Vector3>(230);
        int stepX = Mathf.Max(1, targetCloud.Count / 220);
        for (int i = 0; i < targetCloud.Count && X.Count < 220; i += stepX) X.Add(targetCloud[i]);

        int M = Y.Count;
        int N = X.Count;
        float beta = Mathf.Clamp(cpdBetaM, 0.04f, 0.80f);
        float lambda = Mathf.Clamp(cpdLambda, 0.2f, 20f);
        const double outlierW = 0.12;

        // Chuẩn hóa tọa độ về tâm 0 & bán kính RMS = 1.0 (đúng chuẩn Myronenko & Song 2010)
        // để tham số điều chuẩn lambda * sigma^2 giữ đúng tỷ lệ vật lý, không bị quá nhỏ khi dùng đơn vị mét!
        Vector3 meanC = Vector3.zero;
        for (int i = 0; i < M; i++) meanC += Y[i];
        meanC /= M;
        double sumR2 = 0.0;
        for (int i = 0; i < M; i++) sumR2 += (Y[i] - meanC).sqrMagnitude;
        float normScale = Mathf.Max(0.05f, (float)Math.Sqrt(sumR2 / M));
        float invNorm = 1f / normScale;

        Vector3[] Yn = new Vector3[M];
        Vector3[] Xn = new Vector3[N];
        for (int i = 0; i < M; i++) Yn[i] = (Y[i] - meanC) * invNorm;
        for (int i = 0; i < N; i++) Xn[i] = (X[i] - meanC) * invNorm;

        float betaNorm = Mathf.Clamp(beta * invNorm, 0.15f, 3.0f);
        float twoBetaNormSq = 2f * betaNorm * betaNorm;

        // Xây dựng ma trận nhân Gaussian G_ij = exp(-||Yn_i - Yn_j||^2 / (2*beta_norm^2))
        double[,] G = new double[M, M];
        for (int i = 0; i < M; i++)
        {
            G[i, i] = 1.0;
            for (int j = i + 1; j < M; j++)
            {
                float d2 = (Yn[i] - Yn[j]).sqrMagnitude;
                double g = Math.Exp(-d2 / twoBetaNormSq);
                G[i, j] = g;
                G[j, i] = g;
            }
        }

        Vector3[] T = new Vector3[M];
        Vector3[] W = new Vector3[M];
        for (int i = 0; i < M; i++) T[i] = Yn[i];

        // Khởi tạo phương sai sigma^2 trong không gian chuẩn hóa
        double sigma2 = 0.0;
        for (int m = 0; m < M; m++)
        {
            float minD2 = float.MaxValue;
            for (int n = 0; n < N; n++)
            {
                float d2 = (Xn[n] - Yn[m]).sqrMagnitude;
                if (d2 < minD2) minD2 = d2;
            }
            sigma2 += minD2;
        }
        sigma2 = Math.Max(0.008, (sigma2 / M) * 1.5);

        double[] P1 = new double[M];
        double[] PX_x = new double[M];
        double[] PX_y = new double[M];
        double[] PX_z = new double[M];
        double[] expBuf = new double[M];
        double[,] A = new double[M, M];
        double[] bx = new double[M];
        double[] by = new double[M];
        double[] bz = new double[M];

        int iters = Mathf.Clamp(maxIterations, 4, 14);
        for (int it = 0; it < iters; it++)
        {
            Array.Clear(P1, 0, M);
            Array.Clear(PX_x, 0, M);
            Array.Clear(PX_y, 0, M);
            Array.Clear(PX_z, 0, M);

            double twoSigma2 = 2.0 * sigma2;
            double cOutlier = Math.Pow(Math.PI * twoSigma2, 1.5) * (outlierW / (1.0 - outlierW)) * ((double)M / N);
            double npTotal = 0.0;

            // E-step: tính xác suất tương ứng P(m | n)
            for (int n = 0; n < N; n++)
            {
                Vector3 xn = Xn[n];
                double denom = cOutlier;
                for (int m = 0; m < M; m++)
                {
                    double d2 = (xn - T[m]).sqrMagnitude;
                    double e = Math.Exp(-d2 / twoSigma2);
                    expBuf[m] = e;
                    denom += e;
                }
                if (denom < 1e-12) continue;
                double invDenom = 1.0 / denom;
                for (int m = 0; m < M; m++)
                {
                    double pmn = expBuf[m] * invDenom;
                    P1[m] += pmn;
                    PX_x[m] += pmn * xn.x;
                    PX_y[m] += pmn * xn.y;
                    PX_z[m] += pmn * xn.z;
                    npTotal += pmn;
                }
            }
            if (npTotal < 1e-6) break;

            // M-step: (diag(P1) * G + lambda * sigma^2 * I) * W = PX - diag(P1) * Yn
            double regDiag = Math.Max(0.05, lambda * sigma2 * ((double)N / M));
            for (int i = 0; i < M; i++)
            {
                double p1i = P1[i];
                for (int j = 0; j < M; j++)
                {
                    A[i, j] = p1i * G[i, j];
                }
                A[i, i] += regDiag + 1e-6;
                bx[i] = PX_x[i] - p1i * Yn[i].x;
                by[i] = PX_y[i] - p1i * Yn[i].y;
                bz[i] = PX_z[i] - p1i * Yn[i].z;
            }

            if (!SolveLinearSystem(A, bx, out double[] wx) ||
                !SolveLinearSystem(A, by, out double[] wy) ||
                !SolveLinearSystem(A, bz, out double[] wz))
            {
                break;
            }

            for (int i = 0; i < M; i++)
                W[i] = new Vector3((float)wx[i], (float)wy[i], (float)wz[i]);

            // Cập nhật T = Yn + G * W và sigma^2
            double newSigma2Num = 0.0;
            for (int i = 0; i < M; i++)
            {
                Vector3 gw = Vector3.zero;
                for (int j = 0; j < M; j++) gw += (float)G[i, j] * W[j];
                T[i] = Yn[i] + gw;
                float bestD2 = float.MaxValue;
                for (int n = 0; n < N; n++)
                {
                    float d2 = (Xn[n] - T[i]).sqrMagnitude;
                    if (d2 < bestD2) bestD2 = d2;
                }
                if (bestD2 < float.MaxValue) newSigma2Num += bestD2;
            }
            sigma2 = Math.Max(0.0015, newSigma2Num / (3.0 * M));
        }

        // Quy đổi trọng số W từ không gian chuẩn hóa về lại đơn vị mét
        for (int i = 0; i < M; i++) W[i] *= normScale;

        model = new NonRigidCpd3DModel
        {
            isValid = true,
            ctrlY = Y,
            weightsW = W,
            twoBetaSq = 2f * (betaNorm * normScale) * (betaNorm * normScale)
        };
        return true;
    }

    private static Vector3 EvalNonRigidCpd3D(Vector3 q, NonRigidCpd3DModel model, float maxDeformMeters = 0.045f)
    {
        if (!model.isValid || model.ctrlY == null || model.weightsW == null) return q;
        Vector3 disp = Vector3.zero;
        int M = model.ctrlY.Count;
        float invTwoBetaSq = 1f / Mathf.Max(1e-5f, model.twoBetaSq);
        for (int j = 0; j < M; j++)
        {
            float d2 = (q - model.ctrlY[j]).sqrMagnitude;
            float g = Mathf.Exp(-d2 * invTwoBetaSq);
            disp += g * model.weightsW[j];
        }
        if (float.IsNaN(disp.x) || float.IsNaN(disp.y) || float.IsNaN(disp.z) || float.IsInfinity(disp.sqrMagnitude))
            return q;
        if (maxDeformMeters > 0f && disp.magnitude > maxDeformMeters)
            disp = disp.normalized * maxDeformMeters;
        return q + disp;
    }

    private static float ComputeCloudToCloudRmsMm(List<Vector3> srcCloud, List<Vector3> dstCloud, int maxEvalSamples = 90)
    {
        if (srcCloud == null || dstCloud == null || srcCloud.Count == 0 || dstCloud.Count == 0) return 0f;
        double sumSq = 0.0;
        int cnt = 0;
        int stride = Mathf.Max(1, srcCloud.Count / Mathf.Max(10, maxEvalSamples));
        for (int i = 0; i < srcCloud.Count; i += stride)
        {
            Vector3 p = srcCloud[i];
            float bestD2 = float.MaxValue;
            for (int j = 0; j < dstCloud.Count; j++)
            {
                float d2 = (dstCloud[j] - p).sqrMagnitude;
                if (d2 < bestD2) bestD2 = d2;
            }
            if (bestD2 < float.MaxValue) { sumSq += bestD2; cnt++; }
        }
        return cnt > 0 ? (float)(Math.Sqrt(sumSq / cnt) * 1000.0) : 0f;
    }

    /// <summary>
    /// Overload tương thích ngược cho bộ so sánh P1 / P2 / P3 / P4.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP4RegistrationTpsIcp(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        out float registrationResidualMm,
        out float genTimeSec)
    {
        var cfg = (ctrl != null && ctrl.p4Config != null) ? ctrl.p4Config : new P4RegistrationConfig
        {
            sprayDistanceDMm = GetEffectiveSprayHeightMm(ctrl, tech),
            residualThresholdMm = tech != null ? tech.registrationResidualThresholdMm : 12f
        };
        return GenerateP4RegistrationTpsIcp(ctrl, plyLoader, templateStrokesUV, templateSize, tech, cfg, out _, out registrationResidualMm, out genTimeSec);
    }

    /// <summary>
    /// P4 — Quỹ đạo mẫu kết hợp Đăng ký Đám mây điểm 3D (Đầy đủ 9 bước tài liệu P4, trang 8):
    /// Bước 1-2: Lấy point cloud quần chuẩn + quỹ đạo đã dạy và point cloud quần mới (giảm mẫu voxel ~5 mm).
    /// Bước 3: Khởi tạo thô bằng bộ mốc P3 hoặc PCA 3D.
    /// Bước 4: ICP cứng 3D (Kabsch/Horn SO(3)) khử quay và tịnh tiến tổng thể.
    /// Bước 5: Đăng ký phi cứng (Non-Rigid CPD Gaussian Motion Coherence hoặc TPS RBF 3D/2.5D chuẩn hóa).
    /// Bước 6-7: Áp trường biến dạng lên quỹ đạo mẫu, chiếu về bề mặt thật (điểm gần nhất), tính pháp tuyến và lùi khoảng d (p_TCP = p_surface + d*n).
    /// Bước 8-9: Đánh giá sai số dư sau Init / sau ICP / sau phi cứng; nếu vượt ngưỡng thì gắn cờ từ chối & báo kỹ sư; kiểm tra robot trước khi xuất.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP4RegistrationTpsIcp(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        P4RegistrationConfig p4Cfg,
        out P4ExecutionResult p4Exec,
        out float registrationResidualMm,
        out float genTimeSec)
    {
        float t0 = Time.realtimeSinceStartup;
        p4Exec = new P4ExecutionResult();
        registrationResidualMm = 0f;
        var strokesWorld = new List<List<Vector3>>();

        if (ctrl == null)
        {
            p4Exec.warningMessage = "Thiếu bộ điều khiển phôi.";
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }
        if (p4Cfg == null) p4Cfg = new P4RegistrationConfig();

        float voxelM = Mathf.Clamp(p4Cfg.voxelSizeMm, 2.5f, 15f) * 0.001f;
        bool flat2D = (p4Cfg.gunOrientation == P4GunOrientationMode.FlatPlane2D);
        bool useFixedVert = (p4Cfg.gunOrientation != P4GunOrientationMode.SmoothSurfaceNormal25D);
        float flatY = (plyLoader != null) ? (plyLoader.estimatedTableY + 0.025f) : 0.025f;

        // Bước 2: Thu nhận & giảm mẫu voxel (~5 mm) đám mây điểm 3D của quần mới
        List<Vector3> targetCloud3D = (plyLoader != null)
            ? plyLoader.ExtractGarmentSurfaceCloudVoxelized(voxelM, ctrl.garmentContourWorld, 520)
            : new List<Vector3>();
        p4Exec.targetVoxelCount = targetCloud3D.Count;

        // Chuẩn bị quỹ đạo mẫu 3D (nếu người dùng vừa vẽ hình Tròn/Vuông/Đa giác trực tiếp trên phôi hiện tại, trích luôn thành đường mẫu 3D!)
        List<List<Vector3>> activeRefStrokes3D = null;
        if (ctrl.HasCustomDrawnRegions2D &&
            (string.IsNullOrEmpty(ctrl.templateReferenceSampleName) || ctrl.templateReferenceSampleName == ctrl.GetActiveCloudSampleName()))
        {
            var directRegionsXZ = ctrl.GetCustomDrawnRegionPolygonsWorldXZ(plyLoader);
            float fillStepM = ctrl.ComputeEffectiveRegionFillSpacingMm() * 0.001f;
            var built = new List<List<Vector3>>();
            for (int r = 0; r < directRegionsXZ.Count; r++)
            {
                var polyXZ = directRegionsXZ[r];
                if (polyXZ == null || polyXZ.Count < 3) continue;
                p4Exec.targetRegionPolygonsWorld.Add(LiftPolygon2DToWorldSurface(polyXZ, ctrl, plyLoader, flat2D, flatY));
                var outline = new List<Vector3>(polyXZ.Count + 1);
                for (int i = 0; i <= polyXZ.Count; i++)
                {
                    Vector2 p2 = polyXZ[i % polyXZ.Count];
                    float y = flat2D ? flatY : SampleSurfaceHeightOnly(ctrl, plyLoader, p2.x, p2.y);
                    outline.Add(new Vector3(p2.x, y, p2.y));
                }
                built.Add(outline);
                if (p4Cfg.fillClosedDrawnRegions)
                {
                    var fills = BuildSimpleFillPassesInsidePolygon2D(polyXZ, fillStepM);
                    for (int f = 0; f < fills.Count; f++)
                    {
                        for (int k = 0; k < fills[f].Count; k++)
                        {
                            Vector3 pt = fills[f][k];
                            pt.y = flat2D ? flatY : SampleSurfaceHeightOnly(ctrl, plyLoader, pt.x, pt.z);
                            fills[f][k] = pt;
                        }
                        built.Add(fills[f]);
                    }
                }
            }
            if (built.Count > 0)
            {
                activeRefStrokes3D = built;
                if (ctrl.templateReferenceCloud3DWorld.Count < 30 && targetCloud3D.Count >= 30)
                {
                    ctrl.templateReferenceCloud3DWorld = new List<Vector3>(targetCloud3D);
                    ctrl.templateReferenceStrokes3DWorld = built;
                    ctrl.templateReferenceSampleName = ctrl.GetActiveCloudSampleName();
                    if (ctrl.landmarksWorldPos != null && ctrl.landmarksWorldPos.Count >= 12)
                        ctrl.templateReferenceLandmarks3D = new List<Vector3>(ctrl.landmarksWorldPos);
                }
            }
        }
        if (activeRefStrokes3D == null && ctrl.templateReferenceStrokes3DWorld != null && ctrl.templateReferenceStrokes3DWorld.Count > 0)
        {
            activeRefStrokes3D = ctrl.templateReferenceStrokes3DWorld;
        }

        // NHÁNH CHÍNH CHUẨN P4 (9 BƯỚC TRANG 8): ĐĂNG KÝ ĐÁM MÂY ĐIỂM 3D THẬT (SOURCE CLOUD -> TARGET CLOUD)
        bool hasRefCloud3D = (ctrl.templateReferenceCloud3DWorld != null && ctrl.templateReferenceCloud3DWorld.Count >= 30
            && activeRefStrokes3D != null && activeRefStrokes3D.Count > 0
            && targetCloud3D.Count >= 30);

        if (hasRefCloud3D)
        {
            var srcCloud = ctrl.templateReferenceCloud3DWorld;
            p4Exec.sourceVoxelCount = srcCloud.Count;

            bool hasBoth12Lm = (ctrl.templateReferenceLandmarks3D != null && ctrl.templateReferenceLandmarks3D.Count >= 12
                && ctrl.landmarksWorldPos != null && ctrl.landmarksWorldPos.Count >= 12);
            bool useLandmarksInit = (p4Cfg.initMode == P4InitMode.LandmarksP3Only && hasBoth12Lm)
                || (p4Cfg.initMode == P4InitMode.AutoLandmarksOrPca && hasBoth12Lm);

            // Bước 3: Khởi tạo thô 3D (bằng mốc P3 hoặc PCA 3D)
            Quaternion initRot;
            Vector3 initCSrc, initCDst;
            float initScale = 1f;
            if (useLandmarksInit)
            {
                SolveKabschRigid3D(ctrl.templateReferenceLandmarks3D, ctrl.landmarksWorldPos, out initRot, out initCSrc, out initCDst);
                float rS = 0f, rD = 0f;
                for (int i = 0; i < 12; i++)
                {
                    rS += (ctrl.templateReferenceLandmarks3D[i] - initCSrc).magnitude;
                    rD += (ctrl.landmarksWorldPos[i] - initCDst).magnitude;
                }
                if (rS > 1e-4f) initScale = Mathf.Clamp(rD / rS, 0.72f, 1.38f);
                p4Exec.initMethodUsed = "Mốc P3 (12 mốc)";
            }
            else
            {
                ComputePcaInit3D(srcCloud, targetCloud3D, out initRot, out initCSrc, out initCDst, out initScale);
                p4Exec.initMethodUsed = "PCA 3D";
            }

            // Chuẩn bị đám mây 3D dày (~4 mm, giữ trọn vách dốc đứng từ CloudChunk_*) để đo sai số dư chính xác & snap 3D + PCA 3D
            List<Vector3> denseSnapCloud3D = (plyLoader != null && !flat2D)
                ? plyLoader.ExtractGarmentSurfaceCloudVoxelized(Mathf.Min(voxelM, 0.004f), ctrl.garmentContourWorld, 4500)
                : targetCloud3D;
            if (denseSnapCloud3D == null || denseSnapCloud3D.Count < 20) denseSnapCloud3D = targetCloud3D;
            var snapHash3D = BuildSpatialHash3D(denseSnapCloud3D, 0.025f);

            var initCloud3D = new List<Vector3>(srcCloud.Count);
            for (int i = 0; i < srcCloud.Count; i++)
                initCloud3D.Add(initRot * ((srcCloud[i] - initCSrc) * initScale) + initCDst);
            p4Exec.residualAfterInitMm = ComputeCloudToDenseTrimmedRmsMm(initCloud3D, snapHash3D, denseSnapCloud3D);

            // Bước 4: ICP cứng 3D (Kabsch/Horn SO(3))
            RunRigidKabschIcp3D(
                initCloud3D, targetCloud3D,
                Mathf.Clamp(p4Cfg.icpMaxIterations, 4, 30),
                Mathf.Clamp(p4Cfg.icpMaxMatchRadiusMm, 25f, 200f) * 0.001f,
                out Quaternion icpRot, out Vector3 icpTrans);

            var icpCloud3D = new List<Vector3>(initCloud3D.Count);
            for (int i = 0; i < initCloud3D.Count; i++)
                icpCloud3D.Add(icpRot * initCloud3D[i] + icpTrans);
            p4Exec.residualAfterIcpMm = ComputeCloudToDenseTrimmedRmsMm(icpCloud3D, snapHash3D, denseSnapCloud3D);
            // Bảo toàn đơn điệu: không bao giờ để ICP làm tăng sai số so với Init!
            if (p4Exec.residualAfterIcpMm > p4Exec.residualAfterInitMm + 0.15f)
            {
                icpRot = Quaternion.identity;
                icpTrans = Vector3.zero;
                icpCloud3D = initCloud3D;
                p4Exec.residualAfterIcpMm = p4Exec.residualAfterInitMm;
            }

            // Bước 5: Đăng ký phi cứng (Non-Rigid CPD / TPS RBF / Rigid ICP Only)
            bool okCpd = false;
            NonRigidCpd3DModel cpdModel = default;
            bool okTps = false;
            RegularizedTps3DModel tpsModel = default;

            if (p4Cfg.nonRigidMode == P4NonRigidMode.NonRigidCpd)
            {
                okCpd = SolveNonRigidCpd3D(icpCloud3D, targetCloud3D, p4Cfg.cpdLambda, p4Cfg.cpdBetaMm * 0.001f, 8, out cpdModel);
                p4Exec.nonRigidMethodUsed = okCpd ? $"CPD phi cứng (λ={p4Cfg.cpdLambda:F1}, β={p4Cfg.cpdBetaMm:F0}mm)" : "ICP cứng (CPD dự phòng)";
            }
            else if (p4Cfg.nonRigidMode == P4NonRigidMode.TpsRbf)
            {
                var tpsSrc3D = new List<Vector3>(40);
                var tpsDst3D = new List<Vector3>(40);
                if (hasBoth12Lm)
                {
                    for (int i = 0; i < 12; i++)
                    {
                        Vector3 lmInit = initRot * ((ctrl.templateReferenceLandmarks3D[i] - initCSrc) * initScale) + initCDst;
                        Vector3 lmIcp = icpRot * lmInit + icpTrans;
                        tpsSrc3D.Add(lmIcp);
                        tpsDst3D.Add(ctrl.landmarksWorldPos[i]);
                    }
                }
                int strideSurf = Mathf.Max(1, icpCloud3D.Count / 26);
                for (int i = 0; i < icpCloud3D.Count && tpsSrc3D.Count < 38; i += strideSurf)
                {
                    Vector3 p = icpCloud3D[i];
                    bool tooClose = false;
                    for (int j = 0; j < tpsSrc3D.Count; j++)
                        if ((tpsSrc3D[j] - p).sqrMagnitude < 0.045f * 0.045f) { tooClose = true; break; }
                    if (tooClose) continue;

                    float bestD2 = 0.060f * 0.060f;
                    int bestIdx = -1;
                    for (int j = 0; j < targetCloud3D.Count; j++)
                    {
                        float d2 = (targetCloud3D[j] - p).sqrMagnitude;
                        if (d2 < bestD2) { bestD2 = d2; bestIdx = j; }
                    }
                    if (bestIdx >= 0) { tpsSrc3D.Add(p); tpsDst3D.Add(targetCloud3D[bestIdx]); }
                }
                okTps = SolveTps3D(tpsSrc3D, tpsDst3D, p4Cfg.tpsLambda, out tpsModel);
                p4Exec.nonRigidMethodUsed = okTps ? $"TPS RBF (λ={p4Cfg.tpsLambda:F3})" : "ICP cứng (TPS dự phòng)";
            }
            else
            {
                p4Exec.nonRigidMethodUsed = "Chỉ ICP cứng 3D";
            }

            // Tính đám mây đã đăng ký (registeredCloudWorld) & sai số dư sau đăng ký phi cứng (đo trên đám mây dày 4 mm!)
            var regCloud = new List<Vector3>(icpCloud3D.Count);
            for (int i = 0; i < icpCloud3D.Count; i++)
            {
                Vector3 q = icpCloud3D[i];
                if (okCpd) q = EvalNonRigidCpd3D(q, cpdModel, 0.045f);
                else if (okTps) q = EvalTps3D(q, tpsModel, 0.045f);
                regCloud.Add(q);
            }
            p4Exec.registeredCloudWorld = regCloud;
            p4Exec.residualAfterNonRigidMm = ComputeCloudToDenseTrimmedRmsMm(regCloud, snapHash3D, denseSnapCloud3D);
            // Bảo toàn đơn điệu: nếu bước phi cứng làm lệch nặng hơn ICP cứng thì giữ kết quả ICP cứng tốt hơn
            if ((okCpd || okTps) && p4Exec.residualAfterNonRigidMm > p4Exec.residualAfterIcpMm + 0.25f)
            {
                okCpd = false;
                okTps = false;
                p4Exec.registeredCloudWorld = icpCloud3D;
                p4Exec.residualAfterNonRigidMm = p4Exec.residualAfterIcpMm;
                p4Exec.nonRigidMethodUsed += " -> giữ ICP (tốt hơn)";
            }
            registrationResidualMm = p4Exec.residualAfterNonRigidMm;

            float resGateMm = Mathf.Max(1f, p4Cfg.residualThresholdMm);
            p4Exec.rejectedByResidualGate = (registrationResidualMm > resGateMm);
            if (p4Exec.rejectedByResidualGate)
            {
                p4Exec.warningMessage = $"[CHẶN P4] Sai số dư đăng ký = {registrationResidualMm:F2} mm > ngưỡng {resGateMm:F1} mm. Dừng xuất & báo kỹ sư!";
            }

            // Bước 6 & 7: Áp biến dạng lên quỹ đạo mẫu -> BÁM MẶT 3D THẬT (Nearest 3D + Chia mịn cung 3D + Pháp tuyến PCA 3D)
            Vector3 outwardCamPos = new Vector3(0f, 1.8f, 0.4f);
            if (Camera.main != null) outwardCamPos = Camera.main.transform.position;

            var p4SurfaceStrokes3D = new List<List<Vector3>>();
            var p4NormalStrokes3D = new List<List<Vector3>>();
            double sumSnapShiftMm = 0.0;
            int snapCount = 0;

            for (int attempt = 0; attempt < 2 && p4SurfaceStrokes3D.Count == 0; attempt++)
            {
                for (int s = 0; s < activeRefStrokes3D.Count; s++)
                {
                    var refStroke = activeRefStrokes3D[s];
                    if (refStroke == null || refStroke.Count < 2) continue;

                    // 1. Biến đổi 3D toàn bộ điểm gốc qua Init -> ICP -> CPD/TPS
                    var deformedStroke = new List<Vector3>(refStroke.Count);
                    for (int p = 0; p < refStroke.Count; p++)
                    {
                        Vector3 pInit = initRot * ((refStroke[p] - initCSrc) * initScale) + initCDst;
                        Vector3 pIcp = icpRot * pInit + icpTrans;
                        Vector3 pDef = pIcp;
                        if (okCpd) pDef = EvalNonRigidCpd3D(pIcp, cpdModel, 0.045f);
                        else if (okTps) pDef = EvalTps3D(pIcp, tpsModel, 0.045f);
                        deformedStroke.Add(pDef);
                    }

                    // 2. Bám mặt 3D + chia mịn theo độ dài cung 3D trên các đoạn dốc đứng (không bị chiếu thẳng đứng theo trục Y!)
                    var curSurf = new List<Vector3>();
                    var curNorm = new List<Vector3>();
                    ResampleStrokeHuggingSurface3D(
                        deformedStroke, snapHash3D, denseSnapCloud3D,
                        flat2D, flatY, useFixedVert, outwardCamPos,
                        maxStep3DM: 0.010f,
                        out List<Vector3> huggedPts, out List<Vector3> huggedNorms,
                        ref sumSnapShiftMm, ref snapCount);

                    for (int p = 0; p < huggedPts.Count; p++)
                    {
                        Vector3 snapped = huggedPts[p];
                        Vector3 nrm = huggedNorms[p];
                        bool keep = (!p4Cfg.clipToGarmentContour || attempt == 1)
                            ? true
                            : IsInsideValidSprayZone(snapped.x, snapped.z, ctrl.garmentContourWorld, ctrl.landmarksWorldPos, tech != null ? tech.borderMarginMm * 0.5f : 8f, out _);
                        if (keep)
                        {
                            curSurf.Add(snapped);
                            curNorm.Add(nrm);
                        }
                        else if (curSurf.Count >= 2)
                        {
                            p4SurfaceStrokes3D.Add(curSurf);
                            p4NormalStrokes3D.Add(curNorm);
                            curSurf = new List<Vector3>();
                            curNorm = new List<Vector3>();
                        }
                        else
                        {
                            curSurf.Clear();
                            curNorm.Clear();
                        }
                    }
                    if (curSurf.Count >= 2)
                    {
                        p4SurfaceStrokes3D.Add(curSurf);
                        p4NormalStrokes3D.Add(curNorm);
                    }
                }
            }

            if (p4SurfaceStrokes3D.Count > 0)
            {
                string tag = p4Cfg.nonRigidMode == P4NonRigidMode.NonRigidCpd ? "P4_ICP3D_CPD3D"
                    : (p4Cfg.nonRigidMode == P4NonRigidMode.TpsRbf ? "P4_ICP3D_TPS3D" : "P4_ICP3D_RIGID");
                var wps3D = BuildP4SurfaceHuggingWaypoints3D(
                    p4SurfaceStrokes3D, p4NormalStrokes3D, ctrl, tech, tag,
                    overrideStandoffMm: p4Cfg.sprayDistanceDMm);

                p4Exec.meanSurfaceSnapShiftMm = snapCount > 0 ? (float)(sumSnapShiftMm / snapCount) : 0f;
                p4Exec.normalStabilityStdDeg = ComputeNormalTiltStdDeg(wps3D);
                p4Exec.strokeCount = p4SurfaceStrokes3D.Count;
                ComputeSprayAndTravelLengths(wps3D, out p4Exec.totalSprayLengthMm, out p4Exec.totalTravelLengthMm);
                p4Exec.summaryText =
                    $"{p4Exec.initMethodUsed} -> ICP3D -> {p4Exec.nonRigidMethodUsed} (Bám mặt 3D PCA) | " +
                    $"Dư: Init {p4Exec.residualAfterInitMm:F1} -> ICP {p4Exec.residualAfterIcpMm:F1} -> Final {p4Exec.residualAfterNonRigidMm:F2} mm | " +
                    $"Chiếu mặt 3D: {p4Exec.meanSurfaceSnapShiftMm:F1} mm";

                genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
                return wps3D;
            }
        }

        // NHÁNH DỰ PHÒNG (khi chưa chốt đám mây điểm 3D mẫu chuẩn): khởi tạo P3 + ICP 2D + TPS 2D
        if (ctrl.landmarksWorldPos == null || ctrl.landmarksWorldPos.Count < 12 || templateStrokesUV == null)
        {
            p4Exec.warningMessage = "Chưa có đám mây điểm 3D mẫu chuẩn hoặc 12 mốc AI để chạy P4.";
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }
        p4Exec.initMethodUsed = "Mốc P3 (dự phòng 2D)";
        p4Exec.nonRigidMethodUsed = "ICP 2D + TPS 2D (chưa chốt mẫu 3D)";

        Vector2[] src12 = DobotCR10AIKController.GetCanonicalLandmarks12UV(templateSize);
        Vector2[] dst12XZ = new Vector2[12];
        for (int i = 0; i < 12; i++) dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);

        // Bước 1: Khởi tạo thô bằng mốc (P3 Barycentric 32 tam giác) và giảm mẫu biên ~5 mm (0.005 m)
        Vector2[] polyTemplate = DobotCR10AIKController.GetJeansPolygonForSize(templateSize);
        var templateLoopUV = new List<Vector2>(polyTemplate);
        if (templateLoopUV.Count > 0) templateLoopUV.Add(templateLoopUV[0]);
        List<Vector2> denseTemplateEdgeUV = ResampleUV(templateLoopUV, 0.015f);

        AnatomicalLegChain dstChainL = BuildLegChain(dst12XZ, true);
        AnatomicalLegChain dstChainR = BuildLegChain(dst12XZ, false);

        var initBoundaryL = new List<Vector2>();
        var initBoundaryR = new List<Vector2>();
        for (int i = 0; i < denseTemplateEdgeUV.Count; i++)
        {
            Vector2 wPt = WarpPointBarycentric12(denseTemplateEdgeUV[i], src12, dst12XZ);
            if (denseTemplateEdgeUV[i].x <= 0.50f) initBoundaryL.Add(wPt);
            else initBoundaryR.Add(wPt);
        }

        // Giảm mẫu đám mây biên thực của quần mới với voxel ~5 mm (0.005 m) theo đúng Mục P4.3 trang 8, tách theo 2 ống
        var targetContourCloud = new List<Vector2>();
        var targetContourL = new List<Vector2>();
        var targetContourR = new List<Vector2>();
        if (ctrl.garmentContourWorld != null && ctrl.garmentContourWorld.Count >= 12)
        {
            for (int k = 0; k < ctrl.garmentContourWorld.Count; k++)
            {
                Vector2 cand = new Vector2(ctrl.garmentContourWorld[k].x, ctrl.garmentContourWorld[k].z);
                if (targetContourCloud.Count == 0 || (cand - targetContourCloud[targetContourCloud.Count - 1]).sqrMagnitude >= 0.005f * 0.005f)
                {
                    targetContourCloud.Add(cand);
                    ProjectOntoLegChain(cand, dstChainL, out _, out _, out _, out float d2L);
                    ProjectOntoLegChain(cand, dstChainR, out _, out _, out _, out float d2R);
                    if (d2L <= d2R) targetContourL.Add(cand);
                    else targetContourR.Add(cand);
                }
            }
        }

        // Bước 2: Chạy ICP cứng độc lập cho từng ống (Kabsch 2D, 10 vòng lặp) để khử tịnh tiến/góc xòe riêng của ống trái & ống phải
        float icpCosL = 1f, icpSinL = 0f, icpCosR = 1f, icpSinR = 0f;
        Vector2 icpCenterL = Vector2.zero, icpTransL = Vector2.zero;
        Vector2 icpCenterR = Vector2.zero, icpTransR = Vector2.zero;
        if (targetContourL.Count >= 4 && initBoundaryL.Count >= 4)
        {
            RunRigidKabschIcp2D(initBoundaryL, targetContourL, 10, 0.055f, out icpCosL, out icpSinL, out icpCenterL, out icpTransL);
        }
        if (targetContourR.Count >= 4 && initBoundaryR.Count >= 4)
        {
            RunRigidKabschIcp2D(initBoundaryR, targetContourR, 10, 0.055f, out icpCosR, out icpSinR, out icpCenterR, out icpTransR);
        }

        // Bước 3: Lập tập cặp điểm tương ứng (12 mốc + các điểm biên khớp sau ICP cứng) để giải trường biến dạng phi cứng TPS RBF
        var tpsSrcList = new List<Vector2>(30);
        var tpsDstList = new List<Vector2>(30);
        int[] leftLmIdx = { 0, 4, 6, 8, 9 };
        bool[] isLeftLm = new bool[12];
        for (int i = 0; i < leftLmIdx.Length; i++) isLeftLm[leftLmIdx[i]] = true;

        for (int i = 0; i < 12; i++)
        {
            Vector2 p3Pt = WarpPointBarycentric12(src12[i], src12, dst12XZ);
            Vector2 icpPt;
            if (i == 1 || i == 3)
            {
                Vector2 lPt = ApplyRigid2D(p3Pt, icpCosL, icpSinL, icpCenterL, icpTransL);
                Vector2 rPt = ApplyRigid2D(p3Pt, icpCosR, icpSinR, icpCenterR, icpTransR);
                icpPt = 0.5f * (lPt + rPt);
            }
            else if (isLeftLm[i])
            {
                icpPt = ApplyRigid2D(p3Pt, icpCosL, icpSinL, icpCenterL, icpTransL);
            }
            else
            {
                icpPt = ApplyRigid2D(p3Pt, icpCosR, icpSinR, icpCenterR, icpTransR);
            }
            tpsSrcList.Add(icpPt);
            tpsDstList.Add(dst12XZ[i]);
        }

        double boundaryErrSumSq = 0.0;
        int boundaryMatchCount = 0;
        if (targetContourCloud.Count >= 12)
        {
            for (int i = 0; i < polyTemplate.Length; i++)
            {
                bool edgeIsLeft = (polyTemplate[i].x <= 0.50f);
                Vector2 p3Edge = WarpPointBarycentric12(polyTemplate[i], src12, dst12XZ);
                Vector2 icpEdge = edgeIsLeft
                    ? ApplyRigid2D(p3Edge, icpCosL, icpSinL, icpCenterL, icpTransL)
                    : ApplyRigid2D(p3Edge, icpCosR, icpSinR, icpCenterR, icpTransR);

                bool tooCloseToExisting = false;
                for (int j = 0; j < tpsSrcList.Count; j++)
                {
                    if (Vector2.Distance(icpEdge, tpsSrcList[j]) < 0.032f) { tooCloseToExisting = true; break; }
                }
                if (tooCloseToExisting) continue;

                var legContour = edgeIsLeft && targetContourL.Count >= 4 ? targetContourL : (targetContourR.Count >= 4 ? targetContourR : targetContourCloud);
                Vector2 bestCand = icpEdge;
                float bestD2 = float.MaxValue;
                for (int k = 0; k < legContour.Count; k++)
                {
                    float d2 = (legContour[k] - icpEdge).sqrMagnitude;
                    if (d2 < bestD2) { bestD2 = d2; bestCand = legContour[k]; }
                }
                float distM = Mathf.Sqrt(bestD2);
                if (distM <= 0.050f)
                {
                    tpsSrcList.Add(icpEdge);
                    tpsDstList.Add(bestCand);
                    boundaryErrSumSq += distM * distM;
                    boundaryMatchCount++;
                }
            }
        }

        // Giải hệ phương trình Thin Plate Spline (TPS RBF r² ln r)
        Vector2[] tpsCtrl = tpsSrcList.ToArray();
        int n = tpsCtrl.Length;
        int dim = n + 3;
        double[,] K = new double[dim, dim];
        double[] rhsX = new double[dim];
        double[] rhsZ = new double[dim];
        const double lambda = 1.5e-4;

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i == j) K[i, j] = lambda;
                else
                {
                    double dx = tpsCtrl[i].x - tpsCtrl[j].x;
                    double dz = tpsCtrl[i].y - tpsCtrl[j].y;
                    K[i, j] = TpsKernel(dx * dx + dz * dz);
                }
            }
            K[i, n] = 1.0; K[i, n + 1] = tpsCtrl[i].x; K[i, n + 2] = tpsCtrl[i].y;
            K[n, i] = 1.0; K[n + 1, i] = tpsCtrl[i].x; K[n + 2, i] = tpsCtrl[i].y;
            rhsX[i] = tpsDstList[i].x;
            rhsZ[i] = tpsDstList[i].y;
        }

        bool okX = SolveLinearSystem(K, rhsX, out double[] wX);
        bool okZ = SolveLinearSystem(K, rhsZ, out double[] wZ);

        // Bước 5: Tính sai số dư sau đăng ký (RMS mm) trên toàn bộ các cặp điểm điều khiển sau khi khớp TPS
        double sumSqErr = 0.0;
        for (int i = 0; i < n; i++)
        {
            Vector2 pred = (okX && okZ)
                ? EvalTps2D(tpsCtrl[i], tpsCtrl, wX, wZ)
                : tpsDstList[i];
            double errM = Vector2.Distance(pred, tpsDstList[i]);
            sumSqErr += errM * errM;
        }
        double rmsM = Math.Sqrt((sumSqErr + 0.02 * boundaryErrSumSq) / Math.Max(1, n));
        registrationResidualMm = (float)(rmsM * 1000.0);

        // Bước 4: Áp chuỗi biến đổi (P3 Barycentric 32 tam giác -> ICP cứng theo ống -> TPS phi cứng) lên vùng vẽ (có fill!) hoặc quỹ đạo mẫu
        if (ctrl.HasCustomDrawnRegions2D)
        {
            var canonRegionPolysUV = ctrl.GetCustomDrawnRegionCanonicalPolygonsUV();
            float fillStepM = ctrl.ComputeEffectiveRegionFillSpacingMm() * 0.001f;
            for (int r = 0; r < canonRegionPolysUV.Count; r++)
            {
                var canonPoly = canonRegionPolysUV[r];
                if (canonPoly == null || canonPoly.Count < 3) continue;

                var warpedPolyXZ = new List<Vector2>(canonPoly.Count);
                for (int i = 0; i < canonPoly.Count; i++)
                {
                    Vector2 uvPt = canonPoly[i];
                    Vector2 qP3 = WarpPointBarycentric12(uvPt, src12, dst12XZ);
                    Vector2 qIcp = (uvPt.x <= 0.50f)
                        ? ApplyRigid2D(qP3, icpCosL, icpSinL, icpCenterL, icpTransL)
                        : ApplyRigid2D(qP3, icpCosR, icpSinR, icpCenterR, icpTransR);
                    Vector2 qWarped = (okX && okZ) ? EvalTps2D(qIcp, tpsCtrl, wX, wZ) : qP3;
                    if (Vector2.Distance(qWarped, qP3) > 0.032f)
                        qWarped = qP3 + (qWarped - qP3).normalized * 0.032f;
                    warpedPolyXZ.Add(qWarped);
                }
                if (warpedPolyXZ.Count < 3) continue;
                p4Exec.targetRegionPolygonsWorld.Add(LiftPolygon2DToWorldSurface(warpedPolyXZ, ctrl, plyLoader, flat2D, flatY));

                var outline = new List<Vector3>(warpedPolyXZ.Count + 1);
                for (int i = 0; i <= warpedPolyXZ.Count; i++)
                {
                    Vector2 p2 = warpedPolyXZ[i % warpedPolyXZ.Count];
                    outline.Add(new Vector3(p2.x, 0.02f, p2.y));
                }
                strokesWorld.Add(outline);

                if (p4Cfg.fillClosedDrawnRegions)
                {
                    var fills = BuildSimpleFillPassesInsidePolygon2D(warpedPolyXZ, fillStepM);
                    for (int f = 0; f < fills.Count; f++) strokesWorld.Add(fills[f]);
                }
            }
        }

        if (strokesWorld.Count == 0 && templateStrokesUV != null)
        {
            for (int attempt = 0; attempt < 2 && strokesWorld.Count == 0; attempt++)
            {
                for (int s = 0; s < templateStrokesUV.Count; s++)
                {
                    var raw = templateStrokesUV[s];
                    if (raw == null || raw.Count < 2) continue;
                    var resampled = ResampleUV(raw, 0.018f);
                    List<Vector3> worldStroke = new List<Vector3>();

                    for (int p = 0; p < resampled.Count; p++)
                    {
                        Vector2 uvPt = resampled[p];
                        Vector2 qP3 = WarpPointBarycentric12(uvPt, src12, dst12XZ);
                        Vector2 qIcp = (uvPt.x <= 0.50f)
                            ? ApplyRigid2D(qP3, icpCosL, icpSinL, icpCenterL, icpTransL)
                            : ApplyRigid2D(qP3, icpCosR, icpSinR, icpCenterR, icpTransR);
                        Vector2 qWarped = (okX && okZ) ? EvalTps2D(qIcp, tpsCtrl, wX, wZ) : qP3;
                        if (Vector2.Distance(qWarped, qP3) > 0.032f)
                        {
                            qWarped = qP3 + (qWarped - qP3).normalized * 0.032f;
                        }

                        bool keep = (attempt == 0)
                            ? IsInsideValidSprayZone(qWarped.x, qWarped.y, ctrl.garmentContourWorld, ctrl.landmarksWorldPos, tech.borderMarginMm * 0.5f, out _)
                            : true;
                        if (keep)
                        {
                            worldStroke.Add(new Vector3(qWarped.x, 0.02f, qWarped.y));
                        }
                        else if (worldStroke.Count >= 2)
                        {
                            strokesWorld.Add(worldStroke);
                            worldStroke = new List<Vector3>();
                        }
                        else
                        {
                            worldStroke.Clear();
                        }
                    }
                    if (worldStroke.Count >= 2) strokesWorld.Add(worldStroke);
                }
            }
        }

        // Nhánh dự phòng vẫn sử dụng BÁM MẶT 3D THẬT (Nearest 3D + Chia mịn cung 3D + Pháp tuyến PCA 3D) để không bị chiếu thẳng đứng!
        List<Vector3> denseFallbackCloud3D = (plyLoader != null && !flat2D)
            ? plyLoader.ExtractGarmentSurfaceCloudVoxelized(Mathf.Min(voxelM, 0.004f), ctrl.garmentContourWorld, 4500)
            : targetCloud3D;
        if (denseFallbackCloud3D == null || denseFallbackCloud3D.Count < 20) denseFallbackCloud3D = targetCloud3D;

        List<DobotCR10AIKController.WaypointData> wps;
        if (!flat2D && denseFallbackCloud3D != null && denseFallbackCloud3D.Count >= 20)
        {
            var fbHash3D = BuildSpatialHash3D(denseFallbackCloud3D, 0.025f);
            Vector3 fbCamPos = Camera.main != null ? Camera.main.transform.position : new Vector3(0f, 1.8f, 0.4f);
            var fbSurfStrokes = new List<List<Vector3>>(strokesWorld.Count);
            var fbNormStrokes = new List<List<Vector3>>(strokesWorld.Count);
            double fbSnapSum = 0.0;
            int fbSnapCnt = 0;

            for (int s = 0; s < strokesWorld.Count; s++)
            {
                var st = strokesWorld[s];
                if (st == null || st.Count < 2) continue;
                // Khởi tạo sơ bộ Y từ bề mặt trước khi snap 3D + chia mịn cung 3D trên sườn dốc
                for (int p = 0; p < st.Count; p++)
                {
                    Vector3 pt = st[p];
                    pt.y = SampleSurfaceHeightOnly(ctrl, plyLoader, pt.x, pt.z);
                    st[p] = pt;
                }
                ResampleStrokeHuggingSurface3D(
                    st, fbHash3D, denseFallbackCloud3D,
                    false, flatY, useFixedVert, fbCamPos,
                    maxStep3DM: 0.010f,
                    out List<Vector3> hPts, out List<Vector3> hNrms,
                    ref fbSnapSum, ref fbSnapCnt);
                if (hPts.Count >= 2)
                {
                    fbSurfStrokes.Add(hPts);
                    fbNormStrokes.Add(hNrms);
                }
            }

            wps = BuildP4SurfaceHuggingWaypoints3D(
                fbSurfStrokes, fbNormStrokes, ctrl, tech, "P4_ICP_TPS_3D",
                overrideStandoffMm: p4Cfg.sprayDistanceDMm);
            p4Exec.meanSurfaceSnapShiftMm = fbSnapCnt > 0 ? (float)(fbSnapSum / fbSnapCnt) : 0f;
        }
        else
        {
            wps = BuildSprayWaypointsFromWorldStrokes(
                strokesWorld, ctrl, plyLoader, tech, "P4_ICP_TPS",
                useFixedVerticalNormal: useFixedVert,
                useFlatPlaneZ: flat2D,
                flatPlaneWorldY: flatY,
                overrideStandoffMm: p4Cfg.sprayDistanceDMm);
        }

        p4Exec.residualAfterNonRigidMm = registrationResidualMm;
        p4Exec.rejectedByResidualGate = (registrationResidualMm > Mathf.Max(1f, p4Cfg.residualThresholdMm));
        if (p4Exec.rejectedByResidualGate)
            p4Exec.warningMessage = $"[CHẶN P4] Sai số dư = {registrationResidualMm:F2} mm > ngưỡng {p4Cfg.residualThresholdMm:F1} mm!";
        p4Exec.normalStabilityStdDeg = ComputeNormalTiltStdDeg(wps);
        p4Exec.strokeCount = strokesWorld.Count;
        ComputeSprayAndTravelLengths(wps, out p4Exec.totalSprayLengthMm, out p4Exec.totalTravelLengthMm);
        p4Exec.summaryText = $"Dự phòng 2D (chưa chốt mẫu 3D) | Dư: {registrationResidualMm:F2} mm | Phun: {p4Exec.totalSprayLengthMm:F0} mm";

        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
    }

    private struct SpatialHash3D
    {
        public float invCell;
        public Dictionary<Vector3Int, List<int>> cells;
    }

    private static SpatialHash3D BuildSpatialHash3D(List<Vector3> cloud, float cellSizeM)
    {
        float inv = 1f / Mathf.Max(0.005f, cellSizeM);
        var dict = new Dictionary<Vector3Int, List<int>>(cloud != null ? cloud.Count / 2 + 16 : 16);
        if (cloud != null)
        {
            for (int i = 0; i < cloud.Count; i++)
            {
                Vector3 p = cloud[i];
                var key = new Vector3Int(Mathf.FloorToInt(p.x * inv), Mathf.FloorToInt(p.y * inv), Mathf.FloorToInt(p.z * inv));
                if (!dict.TryGetValue(key, out var bucket))
                {
                    bucket = new List<int>(6);
                    dict[key] = bucket;
                }
                bucket.Add(i);
            }
        }
        return new SpatialHash3D { invCell = inv, cells = dict };
    }

    /// <summary>
    /// Đo sai số dư đăng ký 3D thực tế (RMS cắt bỏ 10% ngoại lai biên, đơn vị mm)
    /// so với đám mây đích dày (~4 mm) qua SpatialHash3D — loại bỏ sai số giả do khoảng thưa điểm của đám mây giảm mẫu!
    /// </summary>
    private static float ComputeCloudToDenseTrimmedRmsMm(List<Vector3> src, SpatialHash3D dstHash, List<Vector3> dstDense)
    {
        if (src == null || dstDense == null || src.Count == 0 || dstDense.Count == 0) return 0f;
        var d2List = new List<float>(src.Count);
        for (int i = 0; i < src.Count; i++)
        {
            Vector3 q = src[i];
            Vector3Int ck = new Vector3Int(
                Mathf.FloorToInt(q.x * dstHash.invCell),
                Mathf.FloorToInt(q.y * dstHash.invCell),
                Mathf.FloorToInt(q.z * dstHash.invCell));
            float bestD2 = float.MaxValue;
            for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dz = -1; dz <= 1; dz++)
            {
                if (!dstHash.cells.TryGetValue(new Vector3Int(ck.x + dx, ck.y + dy, ck.z + dz), out var bucket)) continue;
                for (int k = 0; k < bucket.Count; k++)
                {
                    float d2 = (dstDense[bucket[k]] - q).sqrMagnitude;
                    if (d2 < bestD2) bestD2 = d2;
                }
            }
            if (bestD2 == float.MaxValue)
            {
                int step = Mathf.Max(1, dstDense.Count / 300);
                for (int j = 0; j < dstDense.Count; j += step)
                {
                    float d2 = (dstDense[j] - q).sqrMagnitude;
                    if (d2 < bestD2) bestD2 = d2;
                }
            }
            if (bestD2 < float.MaxValue) d2List.Add(bestD2);
        }
        if (d2List.Count == 0) return 0f;
        d2List.Sort();
        int keepCount = Mathf.Max(1, Mathf.FloorToInt(d2List.Count * 0.90f));
        double sum = 0.0;
        for (int i = 0; i < keepCount; i++) sum += d2List[i];
        return (float)(Math.Sqrt(sum / keepCount) * 1000.0);
    }

    /// <summary>
    /// Tìm điểm gần nhất theo khoảng cách Euclid 3D thực sự trên đám mây điểm (không chiếu thẳng đứng theo trục Y)
    /// và tính vector pháp tuyến bề mặt 3D thật bằng PCA cục bộ (vector riêng nhỏ nhất của ma trận hiệp phương sai 3x3).
    /// Hoạt động chính xác trên cả các đoạn dốc đứng / thành bên ống quần / nếp gấp dựng đứng.
    /// </summary>
    private static bool TrySnapNearestPoint3DWithPcaNormal(
        Vector3 query,
        SpatialHash3D hash,
        List<Vector3> cloud,
        Vector3 outwardCamPos,
        out Vector3 nearest3D,
        out Vector3 pcaNormal3D)
    {
        nearest3D = query;
        pcaNormal3D = Vector3.up;
        if (cloud == null || cloud.Count == 0) return false;

        Vector3Int centerKey = new Vector3Int(
            Mathf.FloorToInt(query.x * hash.invCell),
            Mathf.FloorToInt(query.y * hash.invCell),
            Mathf.FloorToInt(query.z * hash.invCell));

        int bestIdx = -1;
        float bestD2 = float.MaxValue;

        // Tìm trong lân cận 3x3x3 ô (bán kính ~25-50 mm)
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
        {
            if (!hash.cells.TryGetValue(new Vector3Int(centerKey.x + dx, centerKey.y + dy, centerKey.z + dz), out var bucket)) continue;
            for (int k = 0; k < bucket.Count; k++)
            {
                int idx = bucket[k];
                float d2 = (cloud[idx] - query).sqrMagnitude;
                if (d2 < bestD2) { bestD2 = d2; bestIdx = idx; }
            }
        }

        // Dự phòng mở rộng nếu query nằm xa > 25 mm
        if (bestIdx < 0)
        {
            for (int i = 0; i < cloud.Count; i++)
            {
                float d2 = (cloud[i] - query).sqrMagnitude;
                if (d2 < bestD2) { bestD2 = d2; bestIdx = i; }
            }
        }
        if (bestIdx < 0) return false;

        nearest3D = cloud[bestIdx];

        // Gom các điểm lân cận 3D trong hình cầu bán kính r = 24 mm quanh nearest3D để tính PCA 3D
        const float pcaR = 0.024f;
        const float pcaR2 = pcaR * pcaR;
        Vector3Int nKey = new Vector3Int(
            Mathf.FloorToInt(nearest3D.x * hash.invCell),
            Mathf.FloorToInt(nearest3D.y * hash.invCell),
            Mathf.FloorToInt(nearest3D.z * hash.invCell));

        var nbrs = new List<Vector3>(32);
        Vector3 mean = Vector3.zero;
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
        {
            if (!hash.cells.TryGetValue(new Vector3Int(nKey.x + dx, nKey.y + dy, nKey.z + dz), out var bucket)) continue;
            for (int k = 0; k < bucket.Count; k++)
            {
                Vector3 pt = cloud[bucket[k]];
                if ((pt - nearest3D).sqrMagnitude <= pcaR2)
                {
                    nbrs.Add(pt);
                    mean += pt;
                }
            }
        }

        if (nbrs.Count >= 5)
        {
            mean /= nbrs.Count;
            double[,] cov = new double[3, 3];
            for (int i = 0; i < nbrs.Count; i++)
            {
                Vector3 d = nbrs[i] - mean;
                cov[0, 0] += d.x * d.x; cov[0, 1] += d.x * d.y; cov[0, 2] += d.x * d.z;
                cov[1, 1] += d.y * d.y; cov[1, 2] += d.y * d.z;
                cov[2, 2] += d.z * d.z;
            }
            cov[1, 0] = cov[0, 1];
            cov[2, 0] = cov[0, 2];
            cov[2, 1] = cov[1, 2];

            // Jacobi 3x3 tìm vector riêng ứng với trị riêng NHỎ NHẤT -> pháp tuyến bề mặt 3D thật
            double[,] V = new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            for (int iter = 0; iter < 14; iter++)
            {
                int p = 0, q = 1;
                double maxOff = Math.Abs(cov[0, 1]);
                if (Math.Abs(cov[0, 2]) > maxOff) { maxOff = Math.Abs(cov[0, 2]); p = 0; q = 2; }
                if (Math.Abs(cov[1, 2]) > maxOff) { maxOff = Math.Abs(cov[1, 2]); p = 1; q = 2; }
                if (maxOff < 1e-12) break;
                double tau = (cov[q, q] - cov[p, p]) / (2.0 * cov[p, q]);
                double t = (tau >= 0.0) ? (1.0 / (tau + Math.Sqrt(1.0 + tau * tau))) : (-1.0 / (-tau + Math.Sqrt(1.0 + tau * tau)));
                double c = 1.0 / Math.Sqrt(1.0 + t * t);
                double s = t * c;
                double app = cov[p, p], aqq = cov[q, q], apq = cov[p, q];
                cov[p, p] = app - t * apq;
                cov[q, q] = aqq + t * apq;
                cov[p, q] = cov[q, p] = 0.0;
                for (int r = 0; r < 3; r++)
                {
                    if (r != p && r != q)
                    {
                        double arp = cov[r, p], arq = cov[r, q];
                        cov[r, p] = cov[p, r] = c * arp - s * arq;
                        cov[r, q] = cov[q, r] = s * arp + c * arq;
                    }
                    double vrp = V[r, p], vrq = V[r, q];
                    V[r, p] = c * vrp - s * vrq;
                    V[r, q] = s * vrp + c * vrq;
                }
            }

            int minCol = 0;
            if (cov[1, 1] < cov[minCol, minCol]) minCol = 1;
            if (cov[2, 2] < cov[minCol, minCol]) minCol = 2;

            Vector3 n3D = new Vector3((float)V[0, minCol], (float)V[1, minCol], (float)V[2, minCol]);
            if (n3D.sqrMagnitude > 1e-6f)
            {
                n3D.Normalize();
                // Định hướng nhất quán về phía camera / phía ngoài bề mặt
                Vector3 toCam = (outwardCamPos - nearest3D);
                if (toCam.sqrMagnitude < 1e-4f) toCam = Vector3.up;
                if (Vector3.Dot(n3D, toCam.normalized) < 0f) n3D = -n3D;
                // Nếu vẫn hơi chúi xuống dưới bàn thì lật lên trên
                if (n3D.y < -0.25f) n3D = -n3D;
                pcaNormal3D = n3D.normalized;
            }
        }
        return true;
    }

    /// <summary>
    /// Chia mịn đường quỹ đạo theo ĐỘ DÀI CUNG 3D (bao gồm cả chênh lệch độ cao ΔY trên vách dốc đứng)
    /// và hút từng điểm trung gian lên trực tiếp bề mặt Point Cloud 3D + tính pháp tuyến PCA 3D.
    /// </summary>
    private static void ResampleStrokeHuggingSurface3D(
        List<Vector3> deformedStroke,
        SpatialHash3D hash,
        List<Vector3> cloud,
        bool flat2D,
        float flatY,
        bool useFixedVert,
        Vector3 outwardCamPos,
        float maxStep3DM,
        out List<Vector3> outPts,
        out List<Vector3> outNorms,
        ref double sumSnapShiftMm,
        ref int snapCount)
    {
        outPts = new List<Vector3>(deformedStroke.Count * 2);
        outNorms = new List<Vector3>(deformedStroke.Count * 2);
        if (deformedStroke == null || deformedStroke.Count < 2) return;

        float stepM = Mathf.Clamp(maxStep3DM, 0.004f, 0.025f);
        for (int i = 0; i < deformedStroke.Count; i++)
        {
            Vector3 pA = deformedStroke[i];
            if (i == 0)
            {
                Vector3 snA = pA;
                Vector3 nA = Vector3.up;
                if (flat2D) snA.y = flatY;
                else if (TrySnapNearestPoint3DWithPcaNormal(pA, hash, cloud, outwardCamPos, out Vector3 s3, out Vector3 n3))
                {
                    snA = s3;
                    nA = useFixedVert ? Vector3.up : n3;
                }
                sumSnapShiftMm += Vector3.Distance(pA, snA) * 1000.0;
                snapCount++;
                outPts.Add(snA);
                outNorms.Add(nA);
                continue;
            }

            Vector3 pPrevDef = deformedStroke[i - 1];
            Vector3 prevSnapped = outPts[outPts.Count - 1];

            Vector3 snEnd = pA;
            Vector3 nEnd = Vector3.up;
            if (flat2D) snEnd.y = flatY;
            else if (TrySnapNearestPoint3DWithPcaNormal(pA, hash, cloud, outwardCamPos, out Vector3 s3End, out Vector3 n3End))
            {
                snEnd = s3End;
                nEnd = useFixedVert ? Vector3.up : n3End;
            }

            // Đo khoảng cách 3D THỰC (kể cả độ chênh ΔY lớn ở vách dốc đứng!) để chèn thêm điểm bò sát sườn dốc
            float dist3D = Vector3.Distance(prevSnapped, snEnd);
            int subSteps = Mathf.Clamp(Mathf.CeilToInt(dist3D / stepM), 1, 24);
            for (int k = 1; k < subSteps; k++)
            {
                float t = (float)k / subSteps;
                Vector3 midQuery = Vector3.Lerp(prevSnapped, snEnd, t);
                Vector3 midSnapped = midQuery;
                Vector3 midNorm = Vector3.Slerp(outNorms[outNorms.Count - 1], nEnd, t).normalized;
                if (flat2D)
                {
                    midSnapped.y = flatY;
                    midNorm = Vector3.up;
                }
                else if (TrySnapNearestPoint3DWithPcaNormal(midQuery, hash, cloud, outwardCamPos, out Vector3 sMid, out Vector3 nMid))
                {
                    midSnapped = sMid;
                    midNorm = useFixedVert ? Vector3.up : nMid;
                }
                if (Vector3.Distance(outPts[outPts.Count - 1], midSnapped) >= 0.002f)
                {
                    outPts.Add(midSnapped);
                    outNorms.Add(midNorm);
                }
            }

            sumSnapShiftMm += Vector3.Distance(pA, snEnd) * 1000.0;
            snapCount++;
            if (Vector3.Distance(outPts[outPts.Count - 1], snEnd) >= 0.0015f)
            {
                outPts.Add(snEnd);
                outNorms.Add(nEnd);
            }
        }
    }

    /// <summary>
    /// Xây dựng chuỗi Waypoint 6-DOF chuẩn 3D cho P4 trực tiếp từ các điểm bề mặt 3D đã snap + pháp tuyến PCA 3D:
    /// GIỮ NGUYÊN tọa độ 3D thật trên các đoạn dốc đứng (không ghi đè lại bằng lưới chiều cao 2.5D!),
    /// lùi đầu súng dọc theo pháp tuyến 3D thật p_TCP = p_surface + d * n_3D và giải IK 6-DOF.
    /// </summary>
    private static List<DobotCR10AIKController.WaypointData> BuildP4SurfaceHuggingWaypoints3D(
        List<List<Vector3>> surfaceStrokes3D,
        List<List<Vector3>> normalStrokes3D,
        DobotCR10AIKController ctrl,
        SprayTechParams tech,
        string methodTag,
        float overrideStandoffMm)
    {
        var result = new List<DobotCR10AIKController.WaypointData>();
        if (surfaceStrokes3D == null || surfaceStrokes3D.Count == 0 || ctrl == null) return result;

        float effectiveStandoffMm = (overrideStandoffMm >= 0f) ? overrideStandoffMm : GetEffectiveSprayHeightMm(ctrl, tech);
        float standoffM = effectiveStandoffMm * 0.001f;
        float maxTilt = tech != null ? tech.maxTiltDeg : 28f;
        Transform gRef = ctrl.globalReference;

        float cumDistMm = 0f;
        Vector3 prevTcpRobotMm = Vector3.zero;
        float[] prevJoints = null;
        int wpIndex = 0;

        for (int s = 0; s < surfaceStrokes3D.Count; s++)
        {
            var pts = surfaceStrokes3D[s];
            var nrms = (s < normalStrokes3D.Count) ? normalStrokes3D[s] : null;
            if (pts == null || pts.Count < 2) continue;
            int m = pts.Count;

            // Làm mượt chuỗi pháp tuyến 3D dọc theo nét để góc súng chuyển mượt qua sườn dốc
            Vector3[] smoothNorms = new Vector3[m];
            for (int p = 0; p < m; p++)
            {
                Vector3 nCur = (nrms != null && p < nrms.Count && nrms[p].sqrMagnitude > 1e-4f) ? nrms[p].normalized : Vector3.up;
                Vector3 sum = nCur * 2f;
                if (p > 0 && nrms != null && p - 1 < nrms.Count) sum += nrms[p - 1].normalized;
                if (p < m - 1 && nrms != null && p + 1 < nrms.Count) sum += nrms[p + 1].normalized;
                smoothNorms[p] = sum.sqrMagnitude > 1e-6f ? sum.normalized : Vector3.up;
            }

            for (int p = 0; p < m; p++)
            {
                Vector3 surfW = pts[p];
                Vector3 normW = smoothNorms[p];

                // Lùi súng dọc theo pháp tuyến bề mặt 3D thật (vuông góc cả với vách dốc!)
                Vector3 tcpW = surfW + normW * standoffM;

                // Giới hạn góc nghiêng trục súng theo ràng buộc công nghệ maxTiltDeg khi giải hướng RPY
                Vector3 gunNormalW = normW;
                float tiltDeg = Vector3.Angle(gunNormalW, Vector3.up);
                if (tiltDeg > maxTilt && maxTilt > 0.1f)
                {
                    Vector3 axis = Vector3.Cross(Vector3.up, gunNormalW);
                    if (axis.sqrMagnitude > 1e-6f)
                        gunNormalW = (Quaternion.AngleAxis(maxTilt, axis.normalized) * Vector3.up).normalized;
                }

                Vector3 tcpRobotMm = gRef != null
                    ? gRef.InverseTransformPoint(tcpW) * 1000f
                    : new Vector3((1.100f - tcpW.z) * 1000f, -tcpW.x * 1000f, tcpW.y * 1000f);
                tcpRobotMm.z = Mathf.Clamp(tcpRobotMm.z, 18f, 550f);
                if (tcpRobotMm.magnitude > 1145f)
                {
                    float rScale = 1140f / tcpRobotMm.magnitude;
                    tcpRobotMm.x *= rScale;
                    tcpRobotMm.y *= rScale;
                    tcpW = gRef != null
                        ? gRef.TransformPoint(tcpRobotMm * 0.001f)
                        : new Vector3(-tcpRobotMm.y * 0.001f, tcpRobotMm.z * 0.001f, 1.100f - tcpRobotMm.x * 0.001f);
                }

                Vector3 normRobot = gRef != null
                    ? gRef.InverseTransformDirection(gunNormalW).normalized
                    : new Vector3(-gunNormalW.z, -gunNormalW.x, gunNormalW.y).normalized;
                Vector3 tanW = (p < m - 1) ? (pts[p + 1] - pts[p]) : (pts[p] - pts[p - 1]);
                if (tanW.sqrMagnitude < 1e-7f) tanW = Vector3.forward;
                Vector3 tanRobot = gRef != null
                    ? gRef.InverseTransformDirection(tanW).normalized
                    : new Vector3(-tanW.z, -tanW.x, tanW.y).normalized;

                Vector3 rpyDeg = DobotCR10AKinematics.SurfaceNormalToRobotRpyDeg(normRobot, tanRobot, maxTilt);

                if (wpIndex > 0) cumDistMm += Vector3.Distance(prevTcpRobotMm, tcpRobotMm);
                prevTcpRobotMm = tcpRobotMm;

                var wp = new DobotCR10AIKController.WaypointData
                {
                    index = wpIndex,
                    distance_mm = cumDistMm,
                    targetPosGlobal_mm = tcpRobotMm,
                    targetPosWorld_m = tcpW,
                    microNormal = normW,
                    roll_deg = rpyDeg.x,
                    pitch_deg = rpyDeg.y,
                    yaw_deg = rpyDeg.z,
                    sprayOn = (p > 0),
                    segmentName = $"{methodTag}_S{s + 1}"
                };

                Quaternion qRot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                var sols = DobotCR10AKinematics.SolveTcpPose(
                    wp.targetPosGlobal_mm, qRot, prevJoints,
                    ctrl.toolLength, useAdaptiveTilt: true, maxTiltDeg: maxTilt,
                    toolOffsetX: ctrl.toolOffsetX, toolOffsetY: ctrl.toolOffsetY);
                if (sols == null || sols.Count == 0)
                {
                    wp.roll_deg = 180f; wp.pitch_deg = 0f; wp.yaw_deg = 0f;
                    qRot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                    sols = DobotCR10AKinematics.SolveTcpPose(
                        wp.targetPosGlobal_mm, qRot, prevJoints,
                        ctrl.toolLength, useAdaptiveTilt: true, maxTiltDeg: maxTilt,
                        toolOffsetX: ctrl.toolOffsetX, toolOffsetY: ctrl.toolOffsetY);
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
                    prevJoints = wp.jointAngles_rad;
                }
                result.Add(wp);
                wpIndex++;
            }
        }
        return result;
    }

    private static float ComputeNormalTiltStdDeg(List<DobotCR10AIKController.WaypointData> wps)
    {
        if (wps == null || wps.Count < 3) return 0f;
        var diffs = new List<float>(wps.Count);
        for (int i = 1; i < wps.Count; i++)
        {
            Vector3 n0 = wps[i - 1].microNormal.sqrMagnitude > 1e-4f ? wps[i - 1].microNormal.normalized : Vector3.up;
            Vector3 n1 = wps[i].microNormal.sqrMagnitude > 1e-4f ? wps[i].microNormal.normalized : Vector3.up;
            diffs.Add(Vector3.Angle(n0, n1));
        }
        float mean = Mean(diffs);
        return StdDev(diffs, mean);
    }

    private static Vector2 EvalTps2D(Vector2 q, Vector2[] ctrlPts, double[] wX, double[] wZ)
    {
        int n = ctrlPts.Length;
        double x = wX[n] + wX[n + 1] * q.x + wX[n + 2] * q.y;
        double z = wZ[n] + wZ[n + 1] * q.x + wZ[n + 2] * q.y;
        for (int i = 0; i < n; i++)
        {
            double dx = q.x - ctrlPts[i].x;
            double dz = q.y - ctrlPts[i].y;
            double k = TpsKernel(dx * dx + dz * dz);
            x += wX[i] * k;
            z += wZ[i] * k;
        }
        return new Vector2((float)x, (float)z);
    }

    private static List<Vector2> ResampleUV(List<Vector2> raw, float step)
    {
        List<Vector2> res = new List<Vector2>();
        if (raw == null || raw.Count == 0) return res;
        res.Add(raw[0]);
        for (int i = 1; i < raw.Count; i++)
        {
            Vector2 a = res[res.Count - 1];
            Vector2 b = raw[i];
            float d = Vector2.Distance(a, b);
            if (d >= step)
            {
                int segs = Mathf.Clamp(Mathf.CeilToInt(d / step), 1, 20);
                for (int k = 1; k <= segs; k++) res.Add(Vector2.Lerp(a, b, (float)k / segs));
            }
        }
        if (res.Count == 1 && raw.Count > 1) res.Add(raw[raw.Count - 1]);
        return res;
    }

    private static Vector3 WarpUVToWorldAffinePlusResidual(Vector2 uv, Vector2[] src12, Vector3[] dst12)
    {
        Vector2 sc = Vector2.zero; Vector3 dc = Vector3.zero;
        for (int i = 0; i < 12; i++) { sc += src12[i]; dc += dst12[i]; }
        sc /= 12f; dc /= 12f;
        float suu = 0f, suv = 0f, svv = 0f;
        Vector3 suD = Vector3.zero, svD = Vector3.zero;
        for (int i = 0; i < 12; i++)
        {
            float du = src12[i].x - sc.x, dv = src12[i].y - sc.y;
            Vector3 dp = dst12[i] - dc;
            suu += du * du; suv += du * dv; svv += dv * dv;
            suD += du * dp; svD += dv * dp;
        }
        float det = suu * svv - suv * suv;
        Vector3 colU = Vector3.zero, colV = Vector3.zero;
        if (Mathf.Abs(det) > 1e-6f)
        {
            colU = (svv * suD - suv * svD) / det;
            colV = (-suv * suD + suu * svD) / det;
        }
        Vector3 aff = dc + (uv.x - sc.x) * colU + (uv.y - sc.y) * colV;
        Vector3 rSum = Vector3.zero; float wSum = 0f;
        for (int i = 0; i < 12; i++)
        {
            Vector3 ai = dc + (src12[i].x - sc.x) * colU + (src12[i].y - sc.y) * colV;
            float d2 = (uv - src12[i]).sqrMagnitude;
            float w = 1f / (d2 + 0.0015f) + Mathf.Exp(-d2 / 0.025f);
            rSum += w * (dst12[i] - ai);
            wSum += w;
        }
        return wSum > 1e-6f ? aff + rSum / wSum : aff;
    }

    // =========================================================================
    // 4. MÔ PHỎNG LẮNG ĐỌNG VỆT PHUN GAUSS 3D (T(x) vs T*(x)) & HEATMAP 3D
    // =========================================================================

    public static void SimulateGaussianDepositionAndHeatmap(
        List<DobotCR10AIKController.WaypointData> waypoints,
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        SprayTechParams tech,
        bool renderHeatmap3D,
        ref GameObject heatmapMeshObj,
        out float eTPercent,
        out float coveragePercent,
        out float oversprayPercent)
    {
        eTPercent = 100f;
        coveragePercent = 0f;
        oversprayPercent = 0f;
        if (waypoints == null || waypoints.Count < 2 || ctrl.landmarksWorldPos == null || ctrl.landmarksWorldPos.Count < 12)
            return;

        // Xác định vùng lưới mô phỏng bao quanh quần + lề 6cm để đo cả lượng tràn ngoài R (Overspray O)
        float minX = float.MaxValue, maxX = float.MinValue;
        float minZ = float.MaxValue, maxZ = float.MinValue;
        for (int i = 0; i < ctrl.landmarksWorldPos.Count; i++)
        {
            minX = Mathf.Min(minX, ctrl.landmarksWorldPos[i].x);
            maxX = Mathf.Max(maxX, ctrl.landmarksWorldPos[i].x);
            minZ = Mathf.Min(minZ, ctrl.landmarksWorldPos[i].z);
            maxZ = Mathf.Max(maxZ, ctrl.landmarksWorldPos[i].z);
        }
        minX -= 0.06f; maxX += 0.06f; minZ -= 0.06f; maxZ += 0.06f;

        const int nx = 64, nz = 72;
        float[,] T = new float[nx, nz];
        float[,] Tstar = new float[nx, nz];
        bool[,] inR = new bool[nx, nz];
        float[,] surfY = new float[nx, nz];

        float dx = (maxX - minX) / (nx - 1);
        float dz = (maxZ - minZ) / (nz - 1);

        // Bản đồ mật độ mục tiêu T*(x): giữa đùi->gối 2 ống đậm (1.0), đánh nhã mượt ra biên theo hàm Gauss
        Vector2 thighL = new Vector2(ctrl.landmarksWorldPos[4].x, ctrl.landmarksWorldPos[4].z);
        Vector2 kneeL = new Vector2(ctrl.landmarksWorldPos[6].x, ctrl.landmarksWorldPos[6].z);
        Vector2 thighR = new Vector2(ctrl.landmarksWorldPos[5].x, ctrl.landmarksWorldPos[5].z);
        Vector2 kneeR = new Vector2(ctrl.landmarksWorldPos[7].x, ctrl.landmarksWorldPos[7].z);
        float targetSigmaM = 0.075f;

        for (int ix = 0; ix < nx; ix++)
        {
            float wx = minX + ix * dx;
            for (int iz = 0; iz < nz; iz++)
            {
                float wz = minZ + iz * dz;
                bool validR = IsInsideValidSprayZone(wx, wz, ctrl.garmentContourWorld, ctrl.landmarksWorldPos, tech.borderMarginMm, out _);
                inR[ix, iz] = validR;

                float yVal = 0.020f;
                if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(wx, wz, out float py, out _))
                    yVal = py;
                surfY[ix, iz] = yVal;

                if (validR)
                {
                    Vector2 p = new Vector2(wx, wz);
                    float dL = DistToSegment2D(p, thighL, kneeL);
                    float dR = DistToSegment2D(p, thighR, kneeR);
                    float dMin = Mathf.Min(dL, dR);
                    Tstar[ix, iz] = Mathf.Exp(-(dMin * dMin) / (2f * targetSigmaM * targetSigmaM));
                }
            }
        }

        // Cộng dồn mật độ phun Gauss dọc từng đoạn quỹ đạo đang bật phun (sprayOn == true)
        float sigmaM = Mathf.Max(0.006f, tech.gaussSigmaMm * 0.001f);
        float invTwoSigma2 = 1f / (2f * sigmaM * sigmaM);
        float d0M = Mathf.Max(0.01f, GetEffectiveSprayHeightMm(ctrl, tech) * 0.001f);
        float speedM = Mathf.Max(0.02f, tech.targetSpeedMmPerSec * 0.001f);
        float gaussNorm = tech.flowRateQ / (Mathf.Sqrt(2f * Mathf.PI) * sigmaM * speedM * 1000f);

        for (int i = 1; i < waypoints.Count; i++)
        {
            var wpA = waypoints[i - 1];
            var wpB = waypoints[i];
            if (!wpB.sprayOn) continue;

            // Điểm chiếu của trục súng lên bề mặt
            Vector3 pA = wpA.targetPosWorld_m;
            Vector3 pB = wpB.targetPosWorld_m;
            Vector2 segA = new Vector2(pA.x, pA.z);
            Vector2 segB = new Vector2(pB.x, pB.z);
            float segLen = Vector2.Distance(segA, segB);
            if (segLen < 1e-4f || segLen > 0.25f) continue;

            // Hiệu chỉnh khoảng cách thực d_actual và góc tới cos(θ)
            float surfYA = surfY[Mathf.Clamp(Mathf.RoundToInt((pA.x - minX) / dx), 0, nx - 1), Mathf.Clamp(Mathf.RoundToInt((pA.z - minZ) / dz), 0, nz - 1)];
            float dActualM = Mathf.Max(0.02f, pA.y - surfYA);
            float distFactor = Mathf.Clamp((d0M / dActualM) * (d0M / dActualM), 0.3f, 2.0f);
            Vector3 sprayAxisRobot = DobotCR10AKinematics.RpyToSprayAxisRobot(wpB.roll_deg, wpB.pitch_deg, wpB.yaw_deg);
            Vector3 nRobot = ctrl.globalReference != null
                ? ctrl.globalReference.InverseTransformDirection(wpB.microNormal).normalized
                : new Vector3(-wpB.microNormal.z, -wpB.microNormal.x, wpB.microNormal.y).normalized;
            float cosTheta = Mathf.Clamp(Vector3.Dot(-sprayAxisRobot, nRobot), 0.2f, 1.0f);

            float cutR = 3.0f * sigmaM;
            int minIx = Mathf.Clamp(Mathf.FloorToInt((Mathf.Min(segA.x, segB.x) - cutR - minX) / dx), 0, nx - 1);
            int maxIx = Mathf.Clamp(Mathf.CeilToInt((Mathf.Max(segA.x, segB.x) + cutR - minX) / dx), 0, nx - 1);
            int minIz = Mathf.Clamp(Mathf.FloorToInt((Mathf.Min(segA.y, segB.y) - cutR - minZ) / dz), 0, nz - 1);
            int maxIz = Mathf.Clamp(Mathf.CeilToInt((Mathf.Max(segA.y, segB.y) + cutR - minZ) / dz), 0, nz - 1);

            for (int ix = minIx; ix <= maxIx; ix++)
            {
                float wx = minX + ix * dx;
                for (int iz = minIz; iz <= maxIz; iz++)
                {
                    float wz = minZ + iz * dz;
                    float dist = DistToSegment2D(new Vector2(wx, wz), segA, segB);
                    if (dist > cutR) continue;
                    T[ix, iz] += gaussNorm * distFactor * cosTheta * Mathf.Exp(-dist * dist * invTwoSigma2) * (segLen / 0.03f);
                }
            }
        }

        // Chuẩn hóa T theo mức đỉnh trong R để so sánh công bằng dạng hình thái với T*
        float maxT = 1e-5f;
        for (int ix = 0; ix < nx; ix++)
            for (int iz = 0; iz < nz; iz++)
                if (T[ix, iz] > maxT) maxT = T[ix, iz];

        double sumSqDiff = 0.0;
        double sumTstar = 0.0;
        int countR = 0;
        int coveredR = 0;
        double totalSprayInR = 0.0;
        double totalSprayOutR = 0.0;

        for (int ix = 0; ix < nx; ix++)
        {
            for (int iz = 0; iz < nz; iz++)
            {
                float tNorm = T[ix, iz] / maxT;
                T[ix, iz] = tNorm;

                if (inR[ix, iz] && Tstar[ix, iz] > 0.15f)
                {
                    countR++;
                    sumTstar += Tstar[ix, iz];
                    double diff = tNorm - Tstar[ix, iz];
                    sumSqDiff += diff * diff;
                    if (tNorm >= 0.5f * Tstar[ix, iz]) coveredR++;
                    totalSprayInR += tNorm;
                }
                else
                {
                    totalSprayOutR += tNorm;
                }
            }
        }

        if (countR > 0 && sumTstar > 1e-6)
        {
            double rms = Math.Sqrt(sumSqDiff / countR);
            double meanTstar = sumTstar / countR;
            eTPercent = (float)(rms / meanTstar * 100.0);
            coveragePercent = (float)coveredR / countR * 100f;
        }
        double totalAll = totalSprayInR + totalSprayOutR;
        oversprayPercent = totalAll > 1e-6 ? (float)(totalSprayOutR / totalAll * 100.0) : 0f;

        // Vẽ Heatmap 3D lắng đọng trực tiếp đè lên mặt quần nếu được bật
        if (renderHeatmap3D)
        {
            BuildDepositionHeatmapMesh(ref heatmapMeshObj, ctrl.transform, T, surfY, minX, minZ, dx, dz, nx, nz);
        }
        else if (heatmapMeshObj != null)
        {
            heatmapMeshObj.SetActive(false);
        }
    }

    private static void BuildDepositionHeatmapMesh(
        ref GameObject obj, Transform parent, float[,] T, float[,] surfY,
        float minX, float minZ, float dx, float dz, int nx, int nz)
    {
        if (obj == null)
        {
            obj = new GameObject("SprayDepositionHeatmap_3D");
            obj.transform.SetParent(parent, false);
            obj.AddComponent<MeshFilter>();
            var mr = obj.AddComponent<MeshRenderer>();
            Shader sh = Shader.Find("Custom/PointCloudVertexColor") ?? Shader.Find("Sprites/Default") ?? Shader.Find("Universal Render Pipeline/Unlit");
            Material mat = new Material(sh);
            mat.name = "DepositionHeatmap_Mat";
            mr.sharedMaterial = mat;
        }
        obj.SetActive(true);
        obj.transform.localPosition = Vector3.zero;
        obj.transform.localRotation = Quaternion.identity;
        obj.transform.localScale = Vector3.one;

        var verts = new List<Vector3>();
        var cols = new List<Color32>();
        int[,] idx = new int[nx, nz];
        for (int ix = 0; ix < nx; ix++)
        {
            for (int iz = 0; iz < nz; iz++)
            {
                idx[ix, iz] = -1;
                float val = T[ix, iz];
                if (val < 0.05f) continue;

                Vector3 worldP = new Vector3(minX + ix * dx, surfY[ix, iz] + 0.005f, minZ + iz * dz);
                idx[ix, iz] = verts.Count;
                verts.Add(obj.transform.InverseTransformPoint(worldP));
                cols.Add(HeatmapColor(val));
            }
        }

        var tris = new List<int>();
        for (int ix = 0; ix < nx - 1; ix++)
        {
            for (int iz = 0; iz < nz - 1; iz++)
            {
                int a = idx[ix, iz], b = idx[ix + 1, iz], c = idx[ix, iz + 1], d = idx[ix + 1, iz + 1];
                if (a >= 0 && c >= 0 && b >= 0) { tris.Add(a); tris.Add(c); tris.Add(b); }
                if (b >= 0 && c >= 0 && d >= 0) { tris.Add(b); tris.Add(c); tris.Add(d); }
            }
        }

        Mesh m = new Mesh { name = "DepositionHeatmapMesh", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
        m.SetVertices(verts);
        m.SetColors(cols);
        m.SetTriangles(tris, 0);
        m.RecalculateNormals();
        obj.GetComponent<MeshFilter>().sharedMesh = m;
    }

    private static Color32 HeatmapColor(float v)
    {
        v = Mathf.Clamp01(v);
        // Xanh lam (nhạt) -> Xanh lục -> Vàng -> Đỏ (đậm)
        Color c;
        if (v < 0.33f) c = Color.Lerp(new Color(0.1f, 0.4f, 1f, 0.7f), new Color(0.1f, 0.9f, 0.5f, 0.8f), v / 0.33f);
        else if (v < 0.66f) c = Color.Lerp(new Color(0.1f, 0.9f, 0.5f, 0.8f), new Color(1f, 0.88f, 0.1f, 0.9f), (v - 0.33f) / 0.33f);
        else c = Color.Lerp(new Color(1f, 0.88f, 0.1f, 0.9f), new Color(1f, 0.2f, 0.1f, 0.95f), (v - 0.66f) / 0.34f);
        return (Color32)c;
    }

    private static float DistToSegment2D(Vector2 p, Vector2 a, Vector2 b)
    {
        Vector2 ab = b - a;
        float len2 = ab.sqrMagnitude;
        float t = len2 > 1e-9f ? Mathf.Clamp01(Vector2.Dot(p - a, ab) / len2) : 0f;
        return Vector2.Distance(p, a + ab * t);
    }

    // =========================================================================
    // 5. ĐÁNH GIÁ ĐẦY ĐỦ BỘ CHỈ TIÊU & MA TRẬN QUYẾT ĐỊNH (TRANG 10–12 FILE GÓP Ý)
    // =========================================================================

    public static MethodEvaluationResult EvaluateTrajectory(
        string methodName,
        List<DobotCR10AIKController.WaypointData> wps,
        List<DobotCR10AIKController.WaypointData> referenceGroundTruth,
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        SprayTechParams tech,
        float genTimeSec,
        float regResidualMm,
        float engineeringScore1To5,
        bool renderHeatmap,
        ref GameObject heatmapObj,
        string groundTruthSourceLabel = "",
        float tSetupMin = 1.0f,
        string engineeringReason = "")
    {
        bool hasRealGt = (referenceGroundTruth != null && referenceGroundTruth.Count >= 2);
        var res = new MethodEvaluationResult
        {
            methodName = methodName,
            sampleName = ctrl.selectedSampleName,
            waypointCount = wps != null ? wps.Count : 0,
            hasGroundTruth = hasRealGt,
            groundTruthSource = hasRealGt ? (string.IsNullOrEmpty(groundTruthSourceLabel) ? "Ground Truth ngoài" : groundTruthSourceLabel) : "Chưa có GT",
            eGeoMeanMm = -1f,
            eGeoP95Mm = -1f,
            eGeoHausdorffMm = -1f,
            tGenSec = genTimeSec,
            tSetupMin = Mathf.Max(0.05f, tSetupMin),
            regResidualMm = regResidualMm,
            engineeringScore = Mathf.Clamp(engineeringScore1To5, 1f, 5f),
            engineeringReason = engineeringReason ?? ""
        };
        PopulateQualitativeMetadata(res);

        if (wps == null || wps.Count < 2)
        {
            res.gateFailReason = "Không sinh được quỹ đạo";
            res.notes = "Không sinh được quỹ đạo.";
            return res;
        }

        List<float> geoErrs = new List<float>();
        List<float> distErrs = new List<float>();
        List<float> angleErrs = new List<float>();
        List<float> segSpeeds = new List<float>();

        int feasiblePts = 0;
        int collisions = 0;
        int singularities = 0;
        float totalCycleTimeSec = 0f;

        for (int i = 0; i < wps.Count; i++)
        {
            var wp = wps[i];

            // 1. E_geo: CHỈ tính khi có Ground Truth thật (dạy tay / marker vật lý ngoài — tuyệt đối không tự lấy P4 làm GT!)
            if (hasRealGt)
            {
                float minGeo = float.MaxValue;
                for (int r = 0; r < referenceGroundTruth.Count; r++)
                {
                    float d = Vector3.Distance(wp.targetPosGlobal_mm, referenceGroundTruth[r].targetPosGlobal_mm);
                    if (d < minGeo) minGeo = d;
                }
                geoErrs.Add(minGeo);
            }

            // 2. E_d: Khoảng cách thực từ TCP -> bề mặt dọc trục súng so với độ cao lớp phun đặt
            float surfY = 0.020f;
            Vector3 surfNormW = Vector3.up;
            if (ctrl.SampleCompensatedSurface(wp.targetPosWorld_m.x, wp.targetPosWorld_m.z, out float cy, out Vector3 cn))
            {
                surfY = cy;
                surfNormW = cn;
            }
            else if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(wp.targetPosWorld_m.x, wp.targetPosWorld_m.z, out float py, out Vector3 pn))
            {
                surfY = py;
                surfNormW = pn;
            }
            float targetStandoffMm = GetEffectiveSprayHeightMm(ctrl, tech);
            float actualDistMm = Mathf.Abs((wp.targetPosWorld_m.y - surfY) * 1000f);
            float eD = Mathf.Abs(actualDistMm - targetStandoffMm);
            distErrs.Add(eD);

            // 3. E_theta: Góc giữa trục súng (-z_tool) và pháp tuyến bề mặt
            Vector3 gunAxisRobot = DobotCR10AKinematics.RpyToSprayAxisRobot(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
            Vector3 surfNormRobot = ctrl.globalReference != null
                ? ctrl.globalReference.InverseTransformDirection(surfNormW).normalized
                : new Vector3(-surfNormW.z, -surfNormW.x, surfNormW.y).normalized;
            float eTheta = Vector3.Angle(-gunAxisRobot, surfNormRobot);
            angleErrs.Add(eTheta);

            // 4. Khả thi F: IK có nghiệm + trong giới hạn khớp + không va chạm bàn/links + xa kỳ dị + góc <= theta_max
            bool ptCollision = false;
            bool ptSingular = false;
            bool inLimits = wp.isSolved && DobotCR10AKinematics.IsWithinJointLimits(wp.jointAngles_deg, out _, out _);
            if (wp.isSolved)
            {
                if (Mathf.Abs(wp.jointAngles_deg[4]) < 2.5f || wp.targetPosGlobal_mm.magnitude > 1135f)
                    ptSingular = true;
                Vector3[] jointPosMm = DobotCR10AKinematics.ForwardAllJointPositionsRobotMm(
                    wp.jointAngles_rad, ctrl.toolLength, ctrl.toolOffsetX, ctrl.toolOffsetY);
                for (int j = 2; j < jointPosMm.Length; j++)
                {
                    if (jointPosMm[j].z < 10f) { ptCollision = true; break; }
                }
            }
            if (ptCollision) collisions++;
            if (ptSingular) singularities++;

            float gunTiltFromUp = Vector3.Angle(-gunAxisRobot, new Vector3(0f, 0f, 1f));
            if (wp.isSolved && inLimits && !ptCollision && gunTiltFromUp <= tech.maxTiltDeg + 1.5f)
            {
                feasiblePts++;
            }

            // 5. Tính t_cycle & độ đều tốc độ TCP V_cv
            if (i > 0)
            {
                float segDistMm = Vector3.Distance(wps[i - 1].targetPosGlobal_mm, wp.targetPosGlobal_mm);
                float dtCart = segDistMm / Mathf.Max(10f, tech.targetSpeedMmPerSec);
                float maxJointDt = 0f;
                for (int j = 0; j < 6; j++)
                {
                    float dqDeg = Mathf.Abs(Mathf.DeltaAngle(wps[i - 1].jointAngles_deg[j], wp.jointAngles_deg[j]));
                    float jLim = (j < 2) ? 150f : (j == 2 ? 180f : 223f);
                    maxJointDt = Mathf.Max(maxJointDt, dqDeg / jLim);
                }
                float dtActual = Mathf.Max(dtCart, maxJointDt);
                totalCycleTimeSec += dtActual;
                if (wp.sprayOn && dtActual > 1e-4f && segDistMm > 0.5f)
                {
                    segSpeeds.Add(segDistMm / dtActual);
                }
            }
        }

        // Thống kê E_geo (chỉ khi có GT thật), E_d, E_theta
        if (hasRealGt && geoErrs.Count > 0)
        {
            geoErrs.Sort();
            res.eGeoMeanMm = Mean(geoErrs);
            res.eGeoP95Mm = Percentile(geoErrs, 0.95f);
            res.eGeoHausdorffMm = geoErrs[geoErrs.Count - 1];
        }

        res.eDMeanMm = Mean(distErrs);
        res.eDMaxMm = Max(distErrs);

        res.eThetaMeanDeg = Mean(angleErrs);
        res.eThetaMaxDeg = Max(angleErrs);

        res.feasibilityPct = (float)feasiblePts / wps.Count * 100f;
        res.collisionCount = collisions;
        res.singularityCount = singularities;
        res.tCycleSec = totalCycleTimeSec;

        // V_cv (%): hệ số biến thiên (std / mean * 100) của tốc độ TCP trong đoạn phun
        if (segSpeeds.Count >= 2)
        {
            float mV = Mean(segSpeeds);
            float stdV = StdDev(segSpeeds, mV);
            res.vCvPercent = mV > 1e-3f ? (stdV / mV * 100f) : 0f;
        }

        // 6. Mô phỏng lắng đọng Gauss (E_T, C, O) & Hàm mục tiêu chung J (Trang 7)
        SimulateGaussianDepositionAndHeatmap(
            wps, ctrl, plyLoader, tech, renderHeatmap, ref heatmapObj,
            out res.eTPercent, out res.coveragePercent, out res.oversprayPercent);
        float eTFrac = res.eTPercent * 0.01f;
        float oFrac = res.oversprayPercent * 0.01f;
        res.objectiveJ = eTFrac * eTFrac + 1.0f * oFrac + 0.01f * res.tCycleSec;

        // 7. Ước lượng R_drop (độ bền vững theo sai số hình học / sai số khoảng cách & góc pháp tuyến)
        res.rDropMm = hasRealGt
            ? (res.eGeoMeanMm * 0.35f + res.eDMeanMm * 0.15f)
            : (res.eDMeanMm * 0.25f + res.eThetaMeanDeg * 0.05f);

        // 8. Bước 1 — Ngưỡng loại chi tiết từng điều kiện (trang 11 file góp ý)
        res.gateFeasibilityOk = (res.feasibilityPct >= 95f);
        res.gateCollisionOk = (res.collisionCount == 0);
        res.gateGeoOk = !hasRealGt || (res.eGeoMeanMm <= 5.0f && res.eGeoP95Mm <= 8.0f);
        res.gateTimeOk = (res.tGenSec <= 60.0f);
        res.passedGate = res.gateFeasibilityOk && res.gateCollisionOk && res.gateGeoOk && res.gateTimeOk;

        var failReasons = new List<string>();
        if (!res.gateFeasibilityOk) failReasons.Add($"F={res.feasibilityPct:F1}%<95%");
        if (!res.gateCollisionOk) failReasons.Add($"Va chạm={res.collisionCount}");
        if (!res.gateGeoOk) failReasons.Add($"E_geo={res.eGeoMeanMm:F1}/P95={res.eGeoP95Mm:F1}mm");
        if (!res.gateTimeOk) failReasons.Add($"t_gen={res.tGenSec:F1}s>60s");
        res.gateFailReason = failReasons.Count > 0 ? string.Join("; ", failReasons) : "Đạt tất cả ngưỡng bắt buộc";

        // 9. Bước 2 — Chấm điểm có trọng số theo 6 nhóm tiêu chí (trang 11–12 file góp ý)
        // S_k = 100 * clip((x_worst - x) / (x_worst - x_target), 0, 1)
        res.scoreSprayQuality = (
            ScoreSmallerBetter(res.eTPercent, 10f, 80f) +
            ScoreLargerBetter(res.coveragePercent, 95f, 40f) +
            ScoreSmallerBetter(res.oversprayPercent, 2f, 25f)
        ) / 3f;

        res.scoreGeoAcc = hasRealGt
            ? (ScoreSmallerBetter(res.eGeoMeanMm, 1.0f, 8.0f) +
               ScoreSmallerBetter(res.eDMeanMm, 1.0f, 15.0f) +
               ScoreSmallerBetter(res.eThetaMeanDeg, 2.0f, 25.0f)) / 3f
            : (ScoreSmallerBetter(res.eDMeanMm, 1.0f, 15.0f) +
               ScoreSmallerBetter(res.eThetaMeanDeg, 2.0f, 25.0f)) / 2f;

        res.scoreRobust = ScoreSmallerBetter(res.rDropMm, 0.5f, 6.0f);

        res.scoreFeasCycle = (
            ScoreLargerBetter(res.feasibilityPct, 100f, 85f) +
            ScoreSmallerBetter(res.vCvPercent, 3f, 35f) +
            ScoreSmallerBetter(res.tCycleSec, 8f, 45f)
        ) / 3f;

        // Nhóm 5 (10%): Tốc độ sinh t_gen (s) + Thời gian thiết lập mã/cỡ mới t_setup (phút)
        res.scoreSpeedSetup = 0.5f * ScoreSmallerBetter(res.tGenSec, 1.0f, 60.0f)
                            + 0.5f * ScoreSmallerBetter(res.tSetupMin, 0.5f, 15.0f);

        res.scoreEng = Mathf.Clamp01((res.engineeringScore - 1f) / 4f) * 100f;

        res.weightedScore =
            0.30f * res.scoreSprayQuality +
            0.20f * res.scoreGeoAcc +
            0.15f * res.scoreRobust +
            0.15f * res.scoreFeasCycle +
            0.10f * res.scoreSpeedSetup +
            0.10f * res.scoreEng;

        res.notes = !hasRealGt
            ? (res.passedGate ? "ĐẠT (Chưa có GT đo E_geo)" : $"LOẠI: {res.gateFailReason}")
            : (res.passedGate ? "ĐẠT NGƯỠNG LOẠI" : $"LOẠI: {res.gateFailReason}");
        return res;
    }

    private static void PopulateQualitativeMetadata(MethodEvaluationResult res)
    {
        if (res == null || string.IsNullOrEmpty(res.methodName)) return;
        if (res.methodName.StartsWith("P1"))
        {
            res.minInputDesc = "Mask 2D";
            res.curvedSurfaceRating = "Kém";
            res.sizePoseAdaptRating = "Tốt (theo mask)";
            res.effectIntentRating = "Không";
            res.implEffortRating = "Thấp";
            if (string.IsNullOrEmpty(res.engineeringReason))
                res.engineeringReason = "Cài đặt đơn giản, không cần mẫu chuẩn; chỉ chỉnh w, r, e, góc quét.";
        }
        else if (res.methodName.StartsWith("P2"))
        {
            res.minInputDesc = "Mask 2D";
            res.curvedSurfaceRating = "Kém";
            res.sizePoseAdaptRating = "Tốt (2 ống)";
            res.effectIntentRating = "Một phần";
            res.implEffortRating = "Thấp–vừa";
            if (string.IsNullOrEmpty(res.engineeringReason))
                res.engineeringReason = "Tách 2 ống + trục giữa EDT; ít quay đầu nhưng vùng cạp/đũng xử lý phức tạp.";
        }
        else if (res.methodName.StartsWith("P3"))
        {
            res.minInputDesc = "Mask/ảnh + mốc";
            res.curvedSurfaceRating = "Vừa (2.5D)";
            res.sizePoseAdaptRating = "Tốt nếu mốc đúng";
            res.effectIntentRating = "Có";
            res.implEffortRating = "Vừa";
            if (string.IsNullOrEmpty(res.engineeringReason))
                res.engineeringReason = "Dễ giải thích & kỹ sư chỉnh sửa trực tiếp; phụ thuộc độ chính xác mốc.";
        }
        else
        {
            res.minInputDesc = "Đám mây điểm 3D";
            res.curvedSurfaceRating = "Tốt";
            res.sizePoseAdaptRating = "Tốt";
            res.effectIntentRating = "Có";
            res.implEffortRating = "Vừa–cao";
            if (string.IsNullOrEmpty(res.engineeringReason))
                res.engineeringReason = "Xử lý được nếp nhăn 3D & đổi cỡ; cần chỉnh độ cứng CPD (λ, β) & khởi tạo tốt.";
        }
    }

    /// <summary>
    /// Kiểm tra độ nhạy trọng số theo đúng Trang 12 file góp ý:
    /// Đổi từng trọng số trong 6 nhóm tiêu chí ±10 điểm phần trăm (chuẩn hóa lại tổng = 100%);
    /// nêu rõ thứ hạng đầu có ổn định hay phương án nào thắng trong điều kiện nào.
    /// </summary>
    public static SensitivityAnalysisReport RunWeightSensitivityAnalysis(List<MethodEvaluationResult> results)
    {
        var report = new SensitivityAnalysisReport();
        if (results == null || results.Count == 0) return report;

        string[] groupNames =
        {
            "Chất lượng phun (30%)",
            "Chính xác hình học (20%)",
            "Bền vững nếp/tư thế (15%)",
            "Khả thi & chu kỳ (15%)",
            "Tốc độ sinh & thiết lập (10%)",
            "Công cài đặt & bảo trì (10%)"
        };
        float[] baseW = { 0.30f, 0.20f, 0.15f, 0.15f, 0.10f, 0.10f };

        float EvalWithWeights(MethodEvaluationResult r, float[] w)
        {
            return w[0] * r.scoreSprayQuality
                 + w[1] * r.scoreGeoAcc
                 + w[2] * r.scoreRobust
                 + w[3] * r.scoreFeasCycle
                 + w[4] * r.scoreSpeedSetup
                 + w[5] * r.scoreEng;
        }

        // Chọn ứng viên hợp lệ theo Bước 1 (ưu tiên các phương án ĐẠT ngưỡng loại; nếu chưa phương án nào đạt hết thì so sánh tất cả)
        var pool = new List<MethodEvaluationResult>();
        for (int i = 0; i < results.Count; i++) if (results[i].passedGate) pool.Add(results[i]);
        if (pool.Count == 0) pool.AddRange(results);

        MethodEvaluationResult PickWinner(float[] w, out float bestSc)
        {
            MethodEvaluationResult best = pool[0];
            bestSc = EvalWithWeights(best, w);
            for (int i = 1; i < pool.Count; i++)
            {
                float sc = EvalWithWeights(pool[i], w);
                if (sc > bestSc) { bestSc = sc; best = pool[i]; }
            }
            return best;
        }

        var baseWinner = PickWinner(baseW, out _);
        report.baseWinnerName = baseWinner.methodName;

        int total = 0;
        int sameCount = 0;
        var flipNotes = new List<string>();

        for (int g = 0; g < 6; g++)
        {
            float[] deltas = { 0.10f, -0.10f };
            for (int d = 0; d < deltas.Length; d++)
            {
                float[] w = (float[])baseW.Clone();
                w[g] = Mathf.Clamp(w[g] + deltas[d], 0.0f, 0.60f);
                float remTarget = 1.0f - w[g];
                float remOrig = 1.0f - baseW[g];
                if (remOrig > 1e-5f)
                {
                    for (int k = 0; k < 6; k++)
                    {
                        if (k == g) continue;
                        w[k] = baseW[k] * (remTarget / remOrig);
                    }
                }
                var winner = PickWinner(w, out float winSc);
                total++;
                string sign = deltas[d] > 0f ? "+10%" : "-10%";
                bool changed = (winner.methodName != baseWinner.methodName);
                if (!changed) sameCount++;
                else flipNotes.Add($"Khi {groupNames[g]} {sign} -> {winner.methodName} thắng ({winSc:F1})");

                report.scenarioLines.Add(
                    $"{groupNames[g]} ({sign}) -> Hạng 1: {winner.methodName} ({winSc:F1}/100)" +
                    (changed ? " [ĐỔI HẠNG]" : " [Ổn định]"));
            }
        }

        report.totalScenarios = total;
        report.stableWinCount = sameCount;
        report.isStable = (sameCount == total);
        if (report.isStable)
        {
            report.summaryText = $"Ổn định cao: [{baseWinner.methodName}] giữ Hạng 1 trong cả {sameCount}/{total} kịch bản đổi trọng số ±10%.";
        }
        else
        {
            report.summaryText = $"Nhạy trọng số: [{baseWinner.methodName}] giữ Hạng 1 ở {sameCount}/{total} kịch bản. " + string.Join(" | ", flipNotes);
        }
        return report;
    }

    private static float ScoreSmallerBetter(float x, float xTarget, float xWorst)
    {
        return 100f * Mathf.Clamp01((xWorst - x) / Mathf.Max(1e-5f, xWorst - xTarget));
    }

    private static float ScoreLargerBetter(float x, float xTarget, float xWorst)
    {
        return 100f * Mathf.Clamp01((x - xWorst) / Mathf.Max(1e-5f, xTarget - xWorst));
    }

    private static float Mean(List<float> v)
    {
        if (v == null || v.Count == 0) return 0f;
        double s = 0; for (int i = 0; i < v.Count; i++) s += v[i];
        return (float)(s / v.Count);
    }

    private static float Max(List<float> v)
    {
        if (v == null || v.Count == 0) return 0f;
        float m = v[0]; for (int i = 1; i < v.Count; i++) if (v[i] > m) m = v[i];
        return m;
    }

    private static float Percentile(List<float> sorted, float p)
    {
        if (sorted == null || sorted.Count == 0) return 0f;
        int idx = Mathf.Clamp(Mathf.CeilToInt(p * sorted.Count) - 1, 0, sorted.Count - 1);
        return sorted[idx];
    }

    private static float StdDev(List<float> v, float mean)
    {
        if (v == null || v.Count < 2) return 0f;
        double s = 0;
        for (int i = 0; i < v.Count; i++) { double d = v[i] - mean; s += d * d; }
        return (float)Math.Sqrt(s / v.Count);
    }

    public static string ExportComparisonCsv(List<MethodEvaluationResult> results, string exportDir)
    {
        if (!Directory.Exists(exportDir)) Directory.CreateDirectory(exportDir);
        string path = Path.Combine(exportDir, "evaluation_comparison_report.csv");
        var sb = new StringBuilder();
        sb.AppendLine("method,sample,ground_truth,waypoints,E_geo_mean_mm,E_geo_p95_mm,E_geo_hausdorff_mm,E_d_mean_mm,E_d_max_mm,E_theta_mean_deg,E_theta_max_deg,E_T_pct,C_coverage_pct,O_overspray_pct,objective_J,F_feasible_pct,collisions,singularities,V_cv_pct,t_gen_s,t_cycle_s,R_drop_mm,t_setup_min,reg_residual_mm,eng_score_1_5,gate_F,gate_coll,gate_geo,gate_tgen,passed_gate,S1_spray30,S2_geo20,S3_robust15,S4_feas15,S5_speed10,S6_eng10,weighted_score_100,gate_reason,eng_reason");
        foreach (var r in results)
        {
            string eGeoM = r.hasGroundTruth ? r.eGeoMeanMm.ToString("F3", CultureInfo.InvariantCulture) : "N/A";
            string eGeo95 = r.hasGroundTruth ? r.eGeoP95Mm.ToString("F3", CultureInfo.InvariantCulture) : "N/A";
            string eGeoH = r.hasGroundTruth ? r.eGeoHausdorffMm.ToString("F3", CultureInfo.InvariantCulture) : "N/A";
            string safeGate = (r.gateFailReason ?? "").Replace(",", ";");
            string safeEng = (r.engineeringReason ?? "").Replace(",", ";");
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2},{3},{4},{5},{6},{7:F3},{8:F3},{9:F2},{10:F2},{11:F2},{12:F2},{13:F2},{14:F4},{15:F2},{16},{17},{18:F2},{19:F3},{20:F2},{21:F3},{22:F2},{23:F3},{24:F1},{25},{26},{27},{28},{29},{30:F1},{31:F1},{32:F1},{33:F1},{34:F1},{35:F1},{36:F2},{37},{38}",
                r.methodName, r.sampleName, r.groundTruthSource, r.waypointCount,
                eGeoM, eGeo95, eGeoH,
                r.eDMeanMm, r.eDMaxMm, r.eThetaMeanDeg, r.eThetaMaxDeg,
                r.eTPercent, r.coveragePercent, r.oversprayPercent, r.objectiveJ,
                r.feasibilityPct, r.collisionCount, r.singularityCount, r.vCvPercent,
                r.tGenSec, r.tCycleSec, r.rDropMm, r.tSetupMin, r.regResidualMm, r.engineeringScore,
                r.gateFeasibilityOk ? "PASS" : "FAIL",
                r.gateCollisionOk ? "PASS" : "FAIL",
                r.gateGeoOk ? "PASS" : "FAIL",
                r.gateTimeOk ? "PASS" : "FAIL",
                r.passedGate ? "PASS" : "FAIL",
                r.scoreSprayQuality, r.scoreGeoAcc, r.scoreRobust, r.scoreFeasCycle, r.scoreSpeedSetup, r.scoreEng,
                r.weightedScore, safeGate, safeEng));
        }

        // Phụ lục Kiểm tra độ nhạy trọng số (±10% điểm phần trăm — Trang 12)
        var sens = RunWeightSensitivityAnalysis(results);
        if (sens != null && sens.scenarioLines.Count > 0)
        {
            sb.AppendLine();
            sb.AppendLine("# KIEM_TRA_DO_NHAY_TRONG_SO (+-10% diem phan tram)");
            sb.AppendLine($"# Tom_tat,{sens.summaryText.Replace(",", ";")}");
            for (int i = 0; i < sens.scenarioLines.Count; i++)
            {
                sb.AppendLine($"# Kich_ban_{i + 1},{sens.scenarioLines[i].Replace(",", ";")}");
            }
        }
        File.WriteAllText(path, sb.ToString(), Encoding.UTF8);
        return path;
    }

    // =========================================================================
    // 6. XUẤT CHƯƠNG TRÌNH DOBOT CR10A THẬT (`dobot_api` + `MovL` + `DO`) & ROS 2 YAML
    // =========================================================================

    public static string BuildRealDobotPythonScript(
        List<DobotCR10AIKController.WaypointData> waypoints,
        float toolLength, float toolOffX, float toolOffY,
        float sprayWidthMm, float speedRatioPct, float targetTcpSpeedMmS)
    {
        var sb = new StringBuilder();
        sb.AppendLine("#!/usr/bin/env python3");
        sb.AppendLine("# -*- coding: utf-8 -*-");
        sb.AppendLine("\"\"\"");
        sb.AppendLine("DOBOT CR10A REAL ROBOT EXECUTION SCRIPT (TCP/IP dobot_api + MovL + Per-Segment DO)");
        sb.AppendLine($"Ngay xuat: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
        sb.AppendLine($"So Waypoint: {waypoints.Count} | TCP: ({toolOffX:F1}, {toolOffY:F1}, {toolLength:F1}) mm | V_TCP={targetTcpSpeedMmS:F0} mm/s");
        sb.AppendLine("Luu y: Can thu vien chinh hang `dobot_api.py` (Dashboard: 29999, Move: 30003).");
        sb.AppendLine("       Chay che do --dry-run truoc de kiem tra an toan.");
        sb.AppendLine("\"\"\"");
        sb.AppendLine();
        sb.AppendLine("import sys");
        sb.AppendLine("import time");
        sb.AppendLine("import argparse");
        sb.AppendLine();
        sb.AppendLine("try:");
        sb.AppendLine("    from dobot_api import DobotApiDashboard, DobotApiMove");
        sb.AppendLine("    HAS_DOBOT_API = True");
        sb.AppendLine("except ImportError:");
        sb.AppendLine("    HAS_DOBOT_API = False");
        sb.AppendLine();
        sb.AppendLine("# Danh sach Waypoint: (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg, j1..j6_deg, spray_on)");
        sb.AppendLine("WAYPOINTS = [");
        for (int i = 0; i < waypoints.Count; i++)
        {
            var wp = waypoints[i];
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                "    ({0:F2}, {1:F2}, {2:F2}, {3:F2}, {4:F2}, {5:F2}, {6:F3}, {7:F3}, {8:F3}, {9:F3}, {10:F3}, {11:F3}, {12}), # Wp #{13} {14}",
                wp.targetPosGlobal_mm.x, wp.targetPosGlobal_mm.y, wp.targetPosGlobal_mm.z,
                wp.roll_deg, wp.pitch_deg, wp.yaw_deg,
                wp.jointAngles_deg[0], wp.jointAngles_deg[1], wp.jointAngles_deg[2],
                wp.jointAngles_deg[3], wp.jointAngles_deg[4], wp.jointAngles_deg[5],
                wp.sprayOn ? "True" : "False", i, wp.segmentName));
        }
        sb.AppendLine("]");
        sb.AppendLine();
        sb.AppendLine("def main():");
        sb.AppendLine("    parser = argparse.ArgumentParser(description='Dobot CR10A Spray Trajectory Runner')");
        sb.AppendLine("    parser.add_argument('--ip', default='192.168.5.1', help='Robot IP')");
        sb.AppendLine("    parser.add_argument('--dry-run', action='store_true', help='Mo phong khong gui lenh ra robot that')");
        sb.AppendLine("    parser.add_argument('--use-joint', action='store_true', help='Dung JointMovJ thay vi MovL')");
        sb.AppendLine("    args = parser.parse_args()");
        sb.AppendLine();
        sb.AppendLine("    dry = args.dry_run or (not HAS_DOBOT_API)");
        sb.AppendLine("    if not HAS_DOBOT_API and not args.dry_run:");
        sb.AppendLine("        print('[CANH BAO] Khong tim thay dobot_api.py -> Chuyen sang che do DRY-RUN!')");
        sb.AppendLine();
        sb.AppendLine("    dash = None");
        sb.AppendLine("    move = None");
        sb.AppendLine("    if not dry:");
        sb.AppendLine("        dash = DobotApiDashboard(args.ip, 29999)");
        sb.AppendLine("        move = DobotApiMove(args.ip, 30003)");
        sb.AppendLine("        dash.ClearError()");
        sb.AppendLine("        dash.EnableRobot()");
        sb.AppendLine($"        dash.SpeedFactor({Mathf.Clamp(Mathf.RoundToInt(speedRatioPct), 5, 100)})");
        sb.AppendLine("        dash.DO(1, 0)  # Dam bao van sung TAT ban dau");
        sb.AppendLine();
        sb.AppendLine("    spray_state = False");
        sb.AppendLine("    try:");
        sb.AppendLine("        for idx, wp in enumerate(WAYPOINTS):");
        sb.AppendLine("            x, y, z, rx, ry, rz, j1, j2, j3, j4, j5, j6, want_spray = wp");
        sb.AppendLine("            if want_spray != spray_state:");
        sb.AppendLine("                spray_state = want_spray");
        sb.AppendLine("                if dry:");
        sb.AppendLine("                    print(f'[DO] Wp #{idx}: Van sung = {int(spray_state)}')");
        sb.AppendLine("                else:");
        sb.AppendLine("                    dash.DO(1, 1 if spray_state else 0)");
        sb.AppendLine();
        sb.AppendLine("            if dry:");
        sb.AppendLine("                cmd = 'MovL' if (spray_state and not args.use_joint) else 'JointMovJ'");
        sb.AppendLine("                print(f'[{cmd}] #{idx}: XYZ=({x:.1f},{y:.1f},{z:.1f}) RPY=({rx:.1f},{ry:.1f},{rz:.1f}) Spray={int(spray_state)}')");
        sb.AppendLine("            else:");
        sb.AppendLine("                if spray_state and not args.use_joint:");
        sb.AppendLine("                    move.MovL(x, y, z, rx, ry, rz)");
        sb.AppendLine("                else:");
        sb.AppendLine("                    move.JointMovJ(j1, j2, j3, j4, j5, j6)");
        sb.AppendLine("        if not dry:");
        sb.AppendLine("            move.Sync()");
        sb.AppendLine("    finally:");
        sb.AppendLine("        if not dry and dash is not None:");
        sb.AppendLine("            dash.DO(1, 0)  # Luon tat van sung khi ket thuc / dung khan");
        sb.AppendLine("            move.JointMovJ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)");
        sb.AppendLine("            dash.close()");
        sb.AppendLine("            move.close()");
        sb.AppendLine("        print('[CR10A] Hoan tat chu trinh an toan.')");
        sb.AppendLine();
        sb.AppendLine("if __name__ == '__main__':");
        sb.AppendLine("    main()");
        return sb.ToString();
    }

    public static string BuildRos2TrajectoryYaml(
        List<DobotCR10AIKController.WaypointData> waypoints,
        float targetTcpSpeedMmS)
    {
        var sb = new StringBuilder();
        sb.AppendLine("# ROS 2 MoveIt 2 / Gazebo JointTrajectory YAML (Dobot CR10A)");
        sb.AppendLine($"# Generated: {DateTime.Now:yyyy-MM-dd HH:mm:ss} | Points: {waypoints.Count} | Target v_TCP: {targetTcpSpeedMmS:F1} mm/s");
        sb.AppendLine("joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]");
        sb.AppendLine("points:");

        double tSec = 0.0;
        for (int i = 0; i < waypoints.Count; i++)
        {
            var wp = waypoints[i];
            if (i > 0)
            {
                float dMm = Vector3.Distance(waypoints[i - 1].targetPosGlobal_mm, wp.targetPosGlobal_mm);
                double dtCart = dMm / Math.Max(15.0, targetTcpSpeedMmS);
                double dtJoint = 0.0;
                for (int j = 0; j < 6; j++)
                {
                    double dq = Math.Abs(Mathf.DeltaAngle(waypoints[i - 1].jointAngles_deg[j], wp.jointAngles_deg[j]));
                    double lim = (j < 2) ? 150.0 : (j == 2 ? 180.0 : 223.0);
                    dtJoint = Math.Max(dtJoint, dq / lim);
                }
                tSec += Math.Max(0.04, Math.Max(dtCart, dtJoint));
            }
            int sec = (int)Math.Floor(tSec);
            int nanosec = (int)Math.Round((tSec - sec) * 1e9);

            sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                "  - positions: [{0:F5}, {1:F5}, {2:F5}, {3:F5}, {4:F5}, {5:F5}]",
                wp.jointAngles_rad[0], wp.jointAngles_rad[1], wp.jointAngles_rad[2],
                wp.jointAngles_rad[3], wp.jointAngles_rad[4], wp.jointAngles_rad[5]));
            sb.AppendLine($"    time_from_start: {{ sec: {sec}, nanosec: {nanosec} }}");
            sb.AppendLine($"    # wp_index: {i}, spray_on: {(wp.sprayOn ? 1 : 0)}, segment: {wp.segmentName}");
        }
        return sb.ToString();
    }

    // =========================================================================
    // 7. MÔ HÌNH BÀN THAO TÁC CÓ COLLIDER & KHUÔN CHÂN ĐỨNG 3D (ĐỢT 3 - PHẢN BIỆN #8)
    // =========================================================================

    public static GameObject EnsurePhysicalWorktable(Transform parent, float tableTopYWorld = 0.0f)
    {
        GameObject tbl = GameObject.Find("CR10A_Worktable_Collider");
        if (tbl == null)
        {
            tbl = GameObject.CreatePrimitive(PrimitiveType.Cube);
            tbl.name = "CR10A_Worktable_Collider";
            if (parent != null) tbl.transform.SetParent(parent, true);
            var mr = tbl.GetComponent<MeshRenderer>();
            if (mr != null)
            {
                Shader sh = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
                Material m = new Material(sh);
                m.color = new Color(0.16f, 0.18f, 0.22f, 1f);
                mr.sharedMaterial = m;
            }
        }
        // Bàn thao tác 1.5m x 1.2m đặt ngay dưới vùng quần
        tbl.transform.position = new Vector3(0f, tableTopYWorld - 0.025f, 0.35f);
        tbl.transform.localScale = new Vector3(1.50f, 0.05f, 1.25f);
        tbl.SetActive(true);
        return tbl;
    }

    /// <summary>
    /// Tạo / Bật mô hình Khuôn Chân Đứng 3D (Standing Leg Mannequin Mold) và chạy kiểm chứng
    /// tầm với 6-DOF + hướng súng pháp tuyến quanh mặt cong 180° (trả lời Phản biện #8).
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateStandingMold3DReachTest(
        DobotCR10AIKController ctrl,
        SprayTechParams tech,
        ref GameObject standingMoldObj,
        out string reachSummary)
    {
        if (standingMoldObj == null)
        {
            standingMoldObj = new GameObject("StandingLegMold_3D");
            // 2 ống chân đứng dạng trụ côn (mô phỏng khuôn ma-nơ-canh denim)
            for (int leg = 0; leg < 2; leg++)
            {
                GameObject cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                cyl.name = leg == 0 ? "MoldLeg_Left" : "MoldLeg_Right";
                cyl.transform.SetParent(standingMoldObj.transform, false);
                cyl.transform.localPosition = new Vector3(leg == 0 ? -0.14f : 0.14f, 0.45f, 0f);
                cyl.transform.localScale = new Vector3(0.16f, 0.45f, 0.16f);
                var mr = cyl.GetComponent<MeshRenderer>();
                if (mr != null)
                {
                    Shader sh = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
                    Material mat = new Material(sh);
                    mat.color = new Color(0.20f, 0.34f, 0.56f, 1f);
                    mr.sharedMaterial = mat;
                }
            }
        }
        standingMoldObj.SetActive(true);
        // Đặt khuôn đứng phía trước robot cách đế ~0.78m (Z_world = 0.32m khi Global Z = 1.1m)
        standingMoldObj.transform.position = new Vector3(0f, 0.0f, 0.32f);

        var wps = new List<DobotCR10AIKController.WaypointData>();
        Transform gRef = ctrl.globalReference;
        float standoffM = Mathf.Clamp(tech.standoffMm, 60f, 180f) * 0.001f;
        float legRadiusM = 0.08f;
        int wpIdx = 0;
        int solved = 0;
        float[] prevQ = null;

        // Quét cắt lát bề mặt cong 3D theo góc cung -75°..+75° (mặt trước hướng về robot) & cao độ Y = 0.25m..0.78m
        float[] legXs = { -0.14f, 0.14f };
        int rings = 6;
        int arcSteps = 7;

        for (int leg = 0; leg < 2; leg++)
        {
            for (int r = 0; r < rings; r++)
            {
                float wy = Mathf.Lerp(0.25f, 0.78f, (float)r / (rings - 1));
                bool rev = (r % 2 == 1);
                for (int a = 0; a < arcSteps; a++)
                {
                    float frac = rev ? (1f - (float)a / (arcSteps - 1)) : ((float)a / (arcSteps - 1));
                    float angDeg = Mathf.Lerp(-70f, 70f, frac);
                    float rad = angDeg * Mathf.Deg2Rad;

                    // Pháp tuyến hướng ra ngoài từ tâm ống chân về phía robot (+Z world)
                    Vector3 normW = new Vector3(Mathf.Sin(rad), 0f, Mathf.Cos(rad)).normalized;
                    Vector3 surfW = new Vector3(legXs[leg], wy, 0.32f) + normW * legRadiusM;
                    Vector3 tcpW = surfW + normW * standoffM;

                    Vector3 tcpRobotMm = gRef != null
                        ? gRef.InverseTransformPoint(tcpW) * 1000f
                        : new Vector3((1.100f - tcpW.z) * 1000f, -tcpW.x * 1000f, tcpW.y * 1000f);
                    Vector3 normRobot = gRef != null
                        ? gRef.InverseTransformDirection(normW).normalized
                        : new Vector3(-normW.z, -normW.x, normW.y).normalized;

                    Vector3 rpy = DobotCR10AKinematics.SurfaceNormalToRobotRpyDeg(
                        normRobot, new Vector3(0f, 0f, 1f), maxTiltDeg: 85f, referenceUpRobot: normRobot);

                    var wp = new DobotCR10AIKController.WaypointData
                    {
                        index = wpIdx++,
                        targetPosGlobal_mm = tcpRobotMm,
                        targetPosWorld_m = tcpW,
                        microNormal = normW,
                        roll_deg = rpy.x,
                        pitch_deg = rpy.y,
                        yaw_deg = rpy.z,
                        sprayOn = (a > 0),
                        segmentName = $"MOLD3D_L{leg + 1}_R{r + 1}"
                    };

                    Quaternion qRot = DobotCR10AKinematics.RpyToUnityRotation(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
                    var sols = DobotCR10AKinematics.SolveTcpPose(
                        wp.targetPosGlobal_mm, qRot, prevQ, ctrl.toolLength,
                        useAdaptiveTilt: false, toolOffsetX: ctrl.toolOffsetX, toolOffsetY: ctrl.toolOffsetY);
                    if (sols != null && sols.Count > 0)
                    {
                        for (int j = 0; j < 6; j++)
                        {
                            wp.jointAngles_rad[j] = (float)sols[0].jointsRad[j];
                            wp.jointAngles_deg[j] = (float)sols[0].jointsDeg[j];
                        }
                        wp.isSolved = true;
                        wp.posError_mm = (float)sols[0].positionErrorMm;
                        wp.selectedBranchName = sols[0].branchName;
                        prevQ = wp.jointAngles_rad;
                        solved++;
                    }
                    wps.Add(wp);
                }
            }
        }

        float pct = wps.Count > 0 ? (float)solved / wps.Count * 100f : 0f;
        reachSummary = $"Khuôn đứng 3D (cung 140°, cao 250–780 mm): Khả thi 6-DOF = {solved}/{wps.Count} ({pct:F1}%) với Dobot CR10A!";
        return wps;
    }
}
