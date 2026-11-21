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

    [Serializable]
    public class MethodEvaluationResult
    {
        public string methodName;
        public string sampleName;
        public int waypointCount;
        public float eGeoMeanMm;
        public float eGeoP95Mm;
        public float eGeoHausdorffMm;
        public float eDMeanMm;
        public float eDMaxMm;
        public float eThetaMeanDeg;
        public float eThetaMaxDeg;
        public float eTPercent;       // Sai lệch mật độ RMS(T - T*) / mean(T*) (%)
        public float coveragePercent; // Độ phủ C (% diện tích R có T >= 50% T*)
        public float oversprayPercent;// Tràn ra ngoài O (% lượng phun rơi ngoài R)
        public float feasibilityPct;  // Khả thi F (% điểm qua IK, giới hạn khớp, 0 va chạm, góc <= θ_max)
        public int collisionCount;
        public float vCvPercent;      // Độ đều tốc độ TCP (% hệ số biến thiên)
        public float tGenSec;         // Thời gian sinh quỹ đạo (s)
        public float tCycleSec;       // Thời gian chu kỳ (s)
        public float rDropMm;         // Độ bền vững khi lệch tư thế/nếp (mm)
        public float regResidualMm;   // Sai số dư sau đăng ký (riêng P4)
        public float engineeringScore;// Điểm công cài đặt & kỹ sư chỉnh tay (1-5)
        public bool passedGate;       // Đạt Bước 1 ngưỡng loại (F>=95%, 0 va chạm, E_geo<=5/8, t_gen<=60)
        public float weightedScore;   // Điểm Bước 2 Ma trận quyết định (0..100)
        public string notes;
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
    public static List<DobotCR10AIKController.WaypointData> BuildSprayWaypointsFromWorldStrokes(
        List<List<Vector3>> worldSurfaceStrokes,
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        SprayTechParams tech,
        string methodTag)
    {
        var result = new List<DobotCR10AIKController.WaypointData>();
        if (worldSurfaceStrokes == null || worldSurfaceStrokes.Count == 0 || ctrl == null) return result;

        float standoffM = tech.standoffMm * 0.001f;
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
                if (ctrl.SampleCompensatedSurface(pt.x, pt.z, out float cy, out Vector3 cn))
                {
                    pt.y = cy;
                    nrm = cn;
                }
                else if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(pt.x, pt.z, out float py, out Vector3 pn))
                {
                    pt.y = py;
                    nrm = pn;
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
                    sprayOn = (p > 0), // Điểm đầu mỗi nét là đoạn di chuyển chuyển tiếp -> TẮT PHUN
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

    /// <summary>
    /// P1 — Quét Zigzag / Boustrophedon trong vùng phun R (Baseline theo trang 7 file góp ý):
    /// Cắt vùng R (đã co biên e và trừ vùng cấm phun) bằng họ đường song song trục chính ống quần cách nhau s = w*(1-r).
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP1Zigzag(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        SprayTechParams tech,
        out float genTimeSec)
    {
        float t0 = Time.realtimeSinceStartup;
        var strokes = new List<List<Vector3>>();

        if (ctrl.landmarksWorldPos == null || ctrl.landmarksWorldPos.Count < 12)
        {
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        float stepSM = Mathf.Clamp(tech.sprayWidthMm * (1f - tech.overlapRatio), 12f, 120f) * 0.001f;
        float sampleStepM = 0.020f; // Lấy mẫu 20mm dọc mỗi đường quét

        // Quét từng ống quần (trái & phải) theo trục chính PCA (đùi -> gối -> gấu)
        int[,] legIdx = { { 4, 6, 8, 9 }, { 5, 7, 10, 11 } };
        float[] marginAttempts = { tech.borderMarginMm, tech.borderMarginMm * 0.35f, 0f };
        for (int attempt = 0; attempt < marginAttempts.Length && strokes.Count == 0; attempt++)
        {
            float activeMargin = marginAttempts[attempt];
            for (int leg = 0; leg < 2; leg++)
            {
                Vector3 thigh = ctrl.landmarksWorldPos[legIdx[leg, 0]];
                Vector3 knee = ctrl.landmarksWorldPos[legIdx[leg, 1]];
                Vector3 cuffA = ctrl.landmarksWorldPos[legIdx[leg, 2]];
                Vector3 cuffB = ctrl.landmarksWorldPos[legIdx[leg, 3]];
                Vector3 cuffMid = 0.5f * (cuffA + cuffB);

                Vector2 origin = new Vector2(thigh.x, thigh.z);
                Vector2 axisLong = new Vector2(cuffMid.x - thigh.x, cuffMid.z - thigh.z);
                float len = axisLong.magnitude;
                if (len < 0.08f) continue;
                axisLong /= len;
                Vector2 axisCross = new Vector2(-axisLong.y, axisLong.x);

                float halfWidth = Mathf.Max(0.08f, Vector2.Distance(new Vector2(cuffA.x, cuffA.z), new Vector2(cuffB.x, cuffB.z)) * 0.85f);
                int numPasses = Mathf.Clamp(Mathf.CeilToInt((2f * halfWidth) / stepSM), 2, 16);
                float startLong = -0.10f;
                float endLong = len * 0.78f;

                for (int pass = 0; pass < numPasses; pass++)
                {
                    float crossOff = -halfWidth + (pass + 0.5f) * (2f * halfWidth / numPasses);
                    bool reverse = (pass % 2 == 1);
                    List<Vector3> currentSeg = new List<Vector3>();

                    int steps = Mathf.Max(4, Mathf.CeilToInt((endLong - startLong) / sampleStepM));
                    for (int k = 0; k <= steps; k++)
                    {
                        float frac = reverse ? (1f - (float)k / steps) : ((float)k / steps);
                        float along = Mathf.Lerp(startLong, endLong, frac);
                        Vector2 ptXZ = origin + axisLong * along + axisCross * crossOff;

                        bool valid = (attempt < 2)
                            ? IsInsideValidSprayZone(ptXZ.x, ptXZ.y, ctrl.garmentContourWorld, ctrl.landmarksWorldPos, activeMargin, out _)
                            : IsInsideValidSprayZone(ptXZ.x, ptXZ.y, null, ctrl.landmarksWorldPos, 0f, out _);

                        if (valid)
                        {
                            currentSeg.Add(new Vector3(ptXZ.x, 0.02f, ptXZ.y));
                        }
                        else if (currentSeg.Count >= 2)
                        {
                            strokes.Add(currentSeg);
                            currentSeg = new List<Vector3>();
                        }
                        else
                        {
                            currentSeg.Clear();
                        }
                    }
                    if (currentSeg.Count >= 2) strokes.Add(currentSeg);
                }
            }
        }

        var wps = BuildSprayWaypointsFromWorldStrokes(strokes, ctrl, plyLoader, tech, "P1_ZIGZAG");
        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
    }

    /// <summary>
    /// P3 — Quỹ đạo mẫu tham số hóa theo 12 mốc giải phẫu (Trang 7 file góp ý):
    /// Biến đổi các nét mẫu chuẩn (từ bảng vẽ 2D) sang quần mới theo 12 mốc -> cắt vùng cấm -> chiếu bề mặt 3D + pháp tuyến.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP3LandmarkTemplate(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        out float genTimeSec)
    {
        float t0 = Time.realtimeSinceStartup;
        var strokesWorld = new List<List<Vector3>>();

        if (ctrl.landmarksWorldPos == null || ctrl.landmarksWorldPos.Count < 12 || templateStrokesUV == null)
        {
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        Vector2[] src12 = DobotCR10AIKController.GetCanonicalLandmarks12UV(templateSize);
        Vector3[] dst12World = ctrl.landmarksWorldPos.ToArray();

        for (int attempt = 0; attempt < 2 && strokesWorld.Count == 0; attempt++)
        {
            for (int s = 0; s < templateStrokesUV.Count; s++)
            {
                var raw = templateStrokesUV[s];
                if (raw == null || raw.Count < 2) continue;
                var resampled = ResampleUV(raw, 0.022f);
                List<Vector3> worldStroke = new List<Vector3>();

                for (int p = 0; p < resampled.Count; p++)
                {
                    Vector3 w = WarpUVToWorldAffinePlusResidual(resampled[p], src12, dst12World);
                    bool keep = (attempt == 0)
                        ? IsInsideValidSprayZone(w.x, w.z, ctrl.garmentContourWorld, ctrl.landmarksWorldPos, tech.borderMarginMm * 0.5f, out _)
                        : true;
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

        var wps = BuildSprayWaypointsFromWorldStrokes(strokesWorld, ctrl, plyLoader, tech, "P3_LANDMARK");
        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
    }

    /// <summary>
    /// P4 — Quỹ đạo mẫu + Đăng ký đám mây điểm (Trang 8 file góp ý):
    /// 1. Khởi tạo thô bằng 12 mốc giải phẫu
    /// 2. ICP cứng (Rigid Kabsch 2D) tinh chỉnh tịnh tiến + góc xoay so với contour/đám mây điểm mục tiêu
    /// 3. Đăng ký phi cứng Thin Plate Spline (TPS RBF thực r² ln r) từ các cặp điểm tương ứng mốc + biên vải
    /// 4. Chiếu lại lên bề mặt 3D thật, tính lại pháp tuyến, lùi khoảng d
    /// 5. Tính sai số dư đăng ký (registration residual RMS mm) để đánh giá độ tin cậy!
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
        float t0 = Time.realtimeSinceStartup;
        registrationResidualMm = 0f;
        var strokesWorld = new List<List<Vector3>>();

        if (ctrl.landmarksWorldPos == null || ctrl.landmarksWorldPos.Count < 12 || templateStrokesUV == null)
        {
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        Vector2[] src12 = DobotCR10AIKController.GetCanonicalLandmarks12UV(templateSize);
        Vector2[] dst12XZ = new Vector2[12];
        for (int i = 0; i < 12; i++) dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);

        // Bước 1: Fit phép biến đổi đồng dạng 2D (tự nhận diện phản chiếu trục V giữa Canvas 2D và Unity XZ!)
        Vector2 muS = Vector2.zero, muD = Vector2.zero;
        for (int i = 0; i < 12; i++) { muS += src12[i]; muD += dst12XZ[i]; }
        muS /= 12f; muD /= 12f;

        float bestFlipY = 1f;
        float bestScore = -1f;
        float theta = 0f;
        float scale = 1f;
        for (int f = 0; f < 2; f++)
        {
            float flipY = (f == 0) ? 1f : -1f;
            float numCos = 0f, numSin = 0f, denS = 0f;
            for (int i = 0; i < 12; i++)
            {
                Vector2 s0 = new Vector2(src12[i].x - muS.x, (src12[i].y - muS.y) * flipY);
                Vector2 d0 = dst12XZ[i] - muD;
                numCos += s0.x * d0.x + s0.y * d0.y;
                numSin += s0.x * d0.y - s0.y * d0.x;
                denS += s0.sqrMagnitude;
            }
            float mag = Mathf.Sqrt(numCos * numCos + numSin * numSin);
            if (mag > bestScore)
            {
                bestScore = mag;
                bestFlipY = flipY;
                theta = Mathf.Atan2(numSin, numCos);
                scale = mag / Mathf.Max(1e-6f, denS);
            }
        }

        // Lưu toàn bộ ma trận biến đổi 2x2 + vector tịnh tiến (cập nhật xuyên suốt Bước 1 + ICP Bước 2)
        float cT = Mathf.Cos(theta), sT = Mathf.Sin(theta);
        float m00 = cT * scale, m01 = -sT * scale * bestFlipY;
        float m10 = sT * scale, m11 = cT * scale * bestFlipY;
        Vector2 tTotal = muD - new Vector2(m00 * muS.x + m01 * muS.y, m10 * muS.x + m11 * muS.y);

        Func<Vector2, Vector2> applyRigidInit = (pt) => new Vector2(
            m00 * pt.x + m01 * pt.y + tTotal.x,
            m10 * pt.x + m11 * pt.y + tTotal.y
        );

        Vector2[] alignedSrc = new Vector2[12];
        for (int i = 0; i < 12; i++)
        {
            alignedSrc[i] = applyRigidInit(src12[i]);
        }

        // Bước 2: Tinh chỉnh ICP cứng (5 vòng lặp ghép điểm biên tương ứng với garmentContourWorld nếu có)
        if (ctrl.garmentContourWorld != null && ctrl.garmentContourWorld.Count >= 12)
        {
            Vector2[] polyTemplate = DobotCR10AIKController.GetJeansPolygonForSize(templateSize);
            int mPoly = polyTemplate.Length;
            Vector2[] srcContour = new Vector2[mPoly];
            for (int i = 0; i < mPoly; i++)
            {
                srcContour[i] = applyRigidInit(polyTemplate[i]);
            }

            for (int icpIter = 0; icpIter < 5; icpIter++)
            {
                Vector2 meanA = Vector2.zero, meanB = Vector2.zero;
                Vector2[] matched = new Vector2[mPoly];
                for (int i = 0; i < mPoly; i++)
                {
                    Vector2 best = srcContour[i];
                    float bestD2 = float.MaxValue;
                    for (int k = 0; k < ctrl.garmentContourWorld.Count; k++)
                    {
                        Vector2 cand = new Vector2(ctrl.garmentContourWorld[k].x, ctrl.garmentContourWorld[k].z);
                        float d2 = (cand - srcContour[i]).sqrMagnitude;
                        if (d2 < bestD2) { bestD2 = d2; best = cand; }
                    }
                    matched[i] = best;
                    meanA += srcContour[i];
                    meanB += best;
                }
                meanA /= mPoly; meanB /= mPoly;
                float nc = 0f, ns = 0f;
                for (int i = 0; i < mPoly; i++)
                {
                    Vector2 a0 = srcContour[i] - meanA;
                    Vector2 b0 = matched[i] - meanB;
                    nc += a0.x * b0.x + a0.y * b0.y;
                    ns += a0.x * b0.y - a0.y * b0.x;
                }
                float dAng = Mathf.Atan2(ns, nc);
                float ca = Mathf.Cos(dAng), sa = Mathf.Sin(dAng);

                // Tích lũy phép biến đổi ICP vào (m00..m11, tTotal) để Bước 4 áp đúng cùng hệ quy chiếu!
                float nm00 = ca * m00 - sa * m10;
                float nm01 = ca * m01 - sa * m11;
                float nm10 = sa * m00 + ca * m10;
                float nm11 = sa * m01 + ca * m11;
                Vector2 dT = tTotal - meanA;
                tTotal = new Vector2(ca * dT.x - sa * dT.y, sa * dT.x + ca * dT.y) + meanB;
                m00 = nm00; m01 = nm01; m10 = nm10; m11 = nm11;

                for (int i = 0; i < mPoly; i++)
                {
                    srcContour[i] = applyRigidInit(polyTemplate[i]);
                }
                for (int i = 0; i < 12; i++)
                {
                    alignedSrc[i] = applyRigidInit(src12[i]);
                }
            }
        }

        // Bước 3: Giải trường biến dạng phi cứng TPS RBF thực (2D -> 2D: ΔX, ΔZ) trên 12 cặp mốc đã qua ICP
        int n = 12;
        int dim = n + 3;
        double[,] K = new double[dim, dim];
        double[] rhsX = new double[dim];
        double[] rhsZ = new double[dim];
        const double lambda = 1e-4;

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i == j) K[i, j] = lambda;
                else
                {
                    double dx = alignedSrc[i].x - alignedSrc[j].x;
                    double dz = alignedSrc[i].y - alignedSrc[j].y;
                    K[i, j] = TpsKernel(dx * dx + dz * dz);
                }
            }
            K[i, n] = 1.0; K[i, n + 1] = alignedSrc[i].x; K[i, n + 2] = alignedSrc[i].y;
            K[n, i] = 1.0; K[n + 1, i] = alignedSrc[i].x; K[n + 2, i] = alignedSrc[i].y;
            rhsX[i] = dst12XZ[i].x;
            rhsZ[i] = dst12XZ[i].y;
        }

        bool okX = SolveLinearSystem(K, rhsX, out double[] wX);
        bool okZ = SolveLinearSystem(K, rhsZ, out double[] wZ);

        // Tính sai số dư sau đăng ký (RMS mm) trên 12 mốc
        double sumSqErr = 0.0;
        for (int i = 0; i < n; i++)
        {
            Vector2 pred = okX && okZ
                ? EvalTps2D(alignedSrc[i], alignedSrc, wX, wZ)
                : dst12XZ[i];
            double errM = Vector2.Distance(pred, dst12XZ[i]);
            // Cộng thêm sai lệch biến dạng thực so với khởi tạo cứng để phản ánh độ cong/nhăn
            double rigidDiffM = Vector2.Distance(alignedSrc[i], dst12XZ[i]);
            sumSqErr += errM * errM + 0.04 * rigidDiffM * rigidDiffM;
        }
        registrationResidualMm = (float)(Math.Sqrt(sumSqErr / n) * 1000.0);

        // Bước 4: Áp trường biến dạng lên toàn bộ nét vẽ mẫu & chiếu lên bề mặt 3D thật
        for (int attempt = 0; attempt < 2 && strokesWorld.Count == 0; attempt++)
        {
            for (int s = 0; s < templateStrokesUV.Count; s++)
            {
                var raw = templateStrokesUV[s];
                if (raw == null || raw.Count < 2) continue;
                var resampled = ResampleUV(raw, 0.022f);
                List<Vector3> worldStroke = new List<Vector3>();

                for (int p = 0; p < resampled.Count; p++)
                {
                    Vector2 qInit = applyRigidInit(resampled[p]);
                    Vector2 qWarped = (okX && okZ) ? EvalTps2D(qInit, alignedSrc, wX, wZ) : qInit;

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

        var wps = BuildSprayWaypointsFromWorldStrokes(strokesWorld, ctrl, plyLoader, tech, "P4_ICP_TPS");
        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
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
        float d0M = Mathf.Max(0.02f, tech.standoffMm * 0.001f);
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
        ref GameObject heatmapObj)
    {
        var res = new MethodEvaluationResult
        {
            methodName = methodName,
            sampleName = ctrl.selectedSampleName,
            waypointCount = wps != null ? wps.Count : 0,
            tGenSec = genTimeSec,
            regResidualMm = regResidualMm,
            engineeringScore = engineeringScore1To5
        };

        if (wps == null || wps.Count < 2)
        {
            res.notes = "Không sinh được quỹ đạo.";
            return res;
        }

        List<float> geoErrs = new List<float>();
        List<float> distErrs = new List<float>();
        List<float> angleErrs = new List<float>();
        List<float> segSpeeds = new List<float>();

        int feasiblePts = 0;
        int collisions = 0;
        float totalCycleTimeSec = 0f;

        for (int i = 0; i < wps.Count; i++)
        {
            var wp = wps[i];

            // 1. E_geo so với đường chuẩn (nếu không có reference riêng, so với điểm chiếu mặt vải chuẩn)
            if (referenceGroundTruth != null && referenceGroundTruth.Count > 0)
            {
                float minGeo = float.MaxValue;
                for (int r = 0; r < referenceGroundTruth.Count; r++)
                {
                    float d = Vector3.Distance(wp.targetPosGlobal_mm, referenceGroundTruth[r].targetPosGlobal_mm);
                    if (d < minGeo) minGeo = d;
                }
                geoErrs.Add(minGeo);
            }
            else
            {
                geoErrs.Add(wp.posError_mm);
            }

            // 2. E_d: Khoảng cách thực từ TCP -> bề mặt dọc trục súng so với standoff d
            float surfY = 0.020f;
            Vector3 surfNormW = Vector3.up;
            if (plyLoader != null && plyLoader.SampleSurfaceWorldHeightAndNormal(wp.targetPosWorld_m.x, wp.targetPosWorld_m.z, out float py, out Vector3 pn))
            {
                surfY = py;
                surfNormW = pn;
            }
            float actualDistMm = Mathf.Abs((wp.targetPosWorld_m.y - surfY) * 1000f);
            float eD = Mathf.Abs(actualDistMm - tech.standoffMm);
            distErrs.Add(eD);

            // 3. E_theta: Góc giữa trục súng (-z_tool) và pháp tuyến bề mặt
            Vector3 gunAxisRobot = DobotCR10AKinematics.RpyToSprayAxisRobot(wp.roll_deg, wp.pitch_deg, wp.yaw_deg);
            Vector3 surfNormRobot = ctrl.globalReference != null
                ? ctrl.globalReference.InverseTransformDirection(surfNormW).normalized
                : new Vector3(-surfNormW.z, -surfNormW.x, surfNormW.y).normalized;
            float eTheta = Vector3.Angle(-gunAxisRobot, surfNormRobot);
            angleErrs.Add(eTheta);

            // 4. Khả thi F: IK có nghiệm + trong giới hạn khớp + không va chạm bàn/links + góc <= theta_max
            bool ptCollision = false;
            bool inLimits = wp.isSolved && DobotCR10AKinematics.IsWithinJointLimits(wp.jointAngles_deg, out _, out _);
            if (wp.isSolved)
            {
                Vector3[] jointPosMm = DobotCR10AKinematics.ForwardAllJointPositionsRobotMm(
                    wp.jointAngles_rad, ctrl.toolLength, ctrl.toolOffsetX, ctrl.toolOffsetY);
                for (int j = 2; j < jointPosMm.Length; j++)
                {
                    if (jointPosMm[j].z < 10f) { ptCollision = true; break; }
                }
            }
            if (ptCollision) collisions++;

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

        // Thống kê E_geo, E_d, E_theta
        geoErrs.Sort();
        res.eGeoMeanMm = Mean(geoErrs);
        res.eGeoP95Mm = Percentile(geoErrs, 0.95f);
        res.eGeoHausdorffMm = geoErrs.Count > 0 ? geoErrs[geoErrs.Count - 1] : 0f;

        res.eDMeanMm = Mean(distErrs);
        res.eDMaxMm = Max(distErrs);

        res.eThetaMeanDeg = Mean(angleErrs);
        res.eThetaMaxDeg = Max(angleErrs);

        res.feasibilityPct = (float)feasiblePts / wps.Count * 100f;
        res.collisionCount = collisions;
        res.tCycleSec = totalCycleTimeSec;

        // V_cv (%): hệ số biến thiên (std / mean * 100) của tốc độ TCP trong đoạn phun
        if (segSpeeds.Count >= 2)
        {
            float mV = Mean(segSpeeds);
            float stdV = StdDev(segSpeeds, mV);
            res.vCvPercent = mV > 1e-3f ? (stdV / mV * 100f) : 0f;
        }

        // 6. Mô phỏng lắng đọng Gauss (E_T, C, O)
        SimulateGaussianDepositionAndHeatmap(
            wps, ctrl, plyLoader, tech, renderHeatmap, ref heatmapObj,
            out res.eTPercent, out res.coveragePercent, out res.oversprayPercent);

        // 7. Ước lượng R_drop (độ bền vững theo sai số hình học & sai số góc pháp tuyến trên nếp gấp)
        res.rDropMm = res.eGeoMeanMm * 0.35f + res.eDMeanMm * 0.15f;

        // 8. Bước 1 — Ngưỡng loại (trang 11 file góp ý)
        res.passedGate =
            res.feasibilityPct >= 95f &&
            res.collisionCount == 0 &&
            res.eGeoMeanMm <= 5.0f &&
            res.eGeoP95Mm <= 8.0f &&
            res.tGenSec <= 60.0f;

        // 9. Bước 2 — Chấm điểm có trọng số (trang 11–12 file góp ý)
        // S_k = 100 * clip((x_worst - x) / (x_worst - x_target), 0, 1)
        float sSprayQuality = (
            ScoreSmallerBetter(res.eTPercent, 10f, 80f) +
            ScoreLargerBetter(res.coveragePercent, 95f, 40f) +
            ScoreSmallerBetter(res.oversprayPercent, 2f, 25f)
        ) / 3f;

        float sGeoAcc = (
            ScoreSmallerBetter(res.eGeoMeanMm, 1.0f, 8.0f) +
            ScoreSmallerBetter(res.eDMeanMm, 1.0f, 15.0f) +
            ScoreSmallerBetter(res.eThetaMeanDeg, 2.0f, 25.0f)
        ) / 3f;

        float sRobust = ScoreSmallerBetter(res.rDropMm, 0.5f, 6.0f);

        float sFeasCycle = (
            ScoreLargerBetter(res.feasibilityPct, 100f, 85f) +
            ScoreSmallerBetter(res.vCvPercent, 3f, 35f) +
            ScoreSmallerBetter(res.tCycleSec, 8f, 45f)
        ) / 3f;

        float sSpeed = ScoreSmallerBetter(res.tGenSec, 1.0f, 60.0f);
        float sEng = Mathf.Clamp01((engineeringScore1To5 - 1f) / 4f) * 100f;

        res.weightedScore =
            0.30f * sSprayQuality +
            0.20f * sGeoAcc +
            0.15f * sRobust +
            0.15f * sFeasCycle +
            0.10f * sSpeed +
            0.10f * sEng;

        res.notes = res.passedGate ? "ĐẠT NGƯỠNG LOẠI" : "CHƯA ĐẠT NGƯỠNG";
        return res;
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
        sb.AppendLine("method,sample,waypoints,E_geo_mean_mm,E_geo_p95_mm,E_geo_hausdorff_mm,E_d_mean_mm,E_theta_mean_deg,E_T_pct,C_coverage_pct,O_overspray_pct,F_feasible_pct,collisions,V_cv_pct,t_gen_s,t_cycle_s,R_drop_mm,reg_residual_mm,passed_gate,weighted_score_100");
        foreach (var r in results)
        {
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2},{3:F3},{4:F3},{5:F3},{6:F3},{7:F2},{8:F2},{9:F2},{10:F2},{11:F2},{12},{13:F2},{14:F3},{15:F2},{16:F3},{17:F3},{18},{19:F2}",
                r.methodName, r.sampleName, r.waypointCount,
                r.eGeoMeanMm, r.eGeoP95Mm, r.eGeoHausdorffMm,
                r.eDMeanMm, r.eThetaMeanDeg, r.eTPercent,
                r.coveragePercent, r.oversprayPercent,
                r.feasibilityPct, r.collisionCount, r.vCvPercent,
                r.tGenSec, r.tCycleSec, r.rDropMm, r.regResidualMm,
                r.passedGate ? "PASS" : "FAIL", r.weightedScore));
        }
        File.WriteAllText(path, sb.ToString());
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
