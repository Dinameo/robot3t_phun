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
        bool useFixedVerticalNormal = false)
    {
        var result = new List<DobotCR10AIKController.WaypointData>();
        if (worldSurfaceStrokes == null || worldSurfaceStrokes.Count == 0 || ctrl == null) return result;

        float standoffM = GetEffectiveSprayHeightMm(ctrl, tech) * 0.001f;
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
    /// </summary>
    public static Vector2 WarpPointAnatomical2D(Vector2 pt, Vector2[] src12, Vector2[] dst12)
    {
        if (src12 == null || dst12 == null || src12.Length < 12 || dst12.Length < 12) return pt;

        AnatomicalLegChain srcL = BuildLegChain(src12, true);
        AnatomicalLegChain srcR = BuildLegChain(src12, false);

        ProjectOntoLegChain(pt, srcL, out int segL, out float uL, out float wL, out float d2L);
        ProjectOntoLegChain(pt, srcR, out int segR, out float uR, out float wR, out float d2R);

        float absWL = Mathf.Abs(wL);
        float absWR = Mathf.Abs(wR);
        bool useLeft = (absWL <= 1.15f || absWR <= 1.15f) ? (absWL <= absWR) : (d2L <= d2R);

        if (useLeft)
        {
            AnatomicalLegChain dstL = BuildLegChain(dst12, true);
            return ReconstructOnLegChain(dstL, segL, uL, wL);
        }
        else
        {
            AnatomicalLegChain dstR = BuildLegChain(dst12, false);
            return ReconstructOnLegChain(dstR, segR, uR, wR);
        }
    }

    // =========================================================================
    // 2C. LƯỚI TAM GIÁC 12 MỐC GIẢI PHẪU & TỌA ĐỘ TRỌNG TÂM (BARYCENTRIC - P3)
    //     Đúng mô tả P3 trang 7 file góp ý:
    //     "Chia quần thành lưới tam giác theo mốc; biểu diễn mỗi điểm quỹ đạo mẫu
    //      bằng tọa độ trọng tâm (barycentric) trong tam giác chứa nó -> với quần mới:
    //      phát hiện mốc -> dựng lại lưới -> biến đổi affine từng mảnh."
    // =========================================================================

    // 14 tam giác giải phẫu cố định không chồng lấp phủ kín toàn bộ thân quần & 2 ống
    // (0:waist_L, 1:waist_C, 2:waist_R, 3:crotch, 4:thigh_L, 5:thigh_R,
    //  6:knee_L, 7:knee_R, 8:cuff_L_out, 9:cuff_L_in, 10:cuff_R_in, 11:cuff_R_out)
    private static readonly int[,] AnatomicalTriangles12 = new int[14, 3]
    {
        // Vùng cạp - hông - đáy đũng
        { 0, 1, 3 },
        { 1, 2, 3 },
        // Ống quần trái (quanh tâm đùi 4 & tâm gối 6)
        { 0, 3, 4 },
        { 0, 4, 6 },
        { 0, 6, 8 },
        { 3, 9, 4 },
        { 4, 9, 6 },
        { 6, 9, 8 },
        // Ống quần phải (quanh tâm đùi 5 & tâm gối 7)
        { 2, 5, 3 },
        { 2, 7, 5 },
        { 2, 11, 7 },
        { 3, 5, 10 },
        { 5, 7, 10 },
        { 7, 11, 10 }
    };

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
    /// Biến đổi Affine từng mảnh theo Tọa độ Trọng tâm (Barycentric Coordinates) trên lưới 14 tam giác của 12 mốc giải phẫu (Thuật toán P3 chuẩn).
    /// </summary>
    public static Vector2 WarpPointBarycentric12(Vector2 pt, Vector2[] src12, Vector2[] dst12)
    {
        if (src12 == null || dst12 == null || src12.Length < 12 || dst12.Length < 12) return pt;

        int bestTri = -1;
        float bestMinCoord = float.MinValue;
        float bestU = 1f, bestV = 0f, bestW = 0f;
        float bestCenterDistSq = float.MaxValue;

        int numTris = AnatomicalTriangles12.GetLength(0);
        for (int t = 0; t < numTris; t++)
        {
            int i0 = AnatomicalTriangles12[t, 0];
            int i1 = AnatomicalTriangles12[t, 1];
            int i2 = AnatomicalTriangles12[t, 2];

            if (!ComputeBarycentric2D(pt, src12[i0], src12[i1], src12[i2], out float u, out float v, out float w))
                continue;

            float minCoord = Mathf.Min(u, Mathf.Min(v, w));
            if (minCoord >= -1e-4f)
            {
                // Điểm nằm trọn bên trong tam giác t
                return u * dst12[i0] + v * dst12[i1] + w * dst12[i2];
            }

            // Nếu nằm ngoài biên nhẹ, ưu tiên tam giác gần nhất có độ âm nhỏ nhất
            Vector2 triCenter = (src12[i0] + src12[i1] + src12[i2]) / 3f;
            float d2 = (pt - triCenter).sqrMagnitude;
            if (minCoord > bestMinCoord - 0.08f && d2 < bestCenterDistSq)
            {
                bestMinCoord = Mathf.Max(bestMinCoord, minCoord);
                bestCenterDistSq = d2;
                bestTri = t;
                bestU = Mathf.Clamp(u, -0.35f, 1.35f);
                bestV = Mathf.Clamp(v, -0.35f, 1.35f);
                bestW = 1f - bestU - bestV;
            }
        }

        if (bestTri >= 0)
        {
            int j0 = AnatomicalTriangles12[bestTri, 0];
            int j1 = AnatomicalTriangles12[bestTri, 1];
            int j2 = AnatomicalTriangles12[bestTri, 2];
            return bestU * dst12[j0] + bestV * dst12[j1] + bestW * dst12[j2];
        }

        return WarpPointAnatomical2D(pt, src12, dst12);
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
    /// Kiểm tra điểm 2D `ptXZ` trên ống `leg` (0=trái, 1=phải) có nằm trong vùng phun mục tiêu `R`
    /// (giao giữa vùng mẫu hiệu ứng đã ánh xạ và biên quần Douglas-Peucker co khoảng `e`) hay không.
    /// </summary>
    private static bool IsPointInTemplateRegionR(
        Vector2 ptXZ,
        int leg,
        List<Vector2> legTemplatePts,
        float corridorRadiusM,
        List<Vector3> contourWorld,
        List<Vector3> landmarksWorldPos,
        float borderMarginMm)
    {
        if (!IsInsideValidSprayZone(ptXZ.x, ptXZ.y, contourWorld, landmarksWorldPos, borderMarginMm, out _))
            return false;

        // Phân tách ô lồi Boustrophedon tại đáy đũng: điểm của ống nào phải gần trục ống đó hơn
        Vector2 thighL = new Vector2(landmarksWorldPos[4].x, landmarksWorldPos[4].z);
        Vector2 kneeL = new Vector2(landmarksWorldPos[6].x, landmarksWorldPos[6].z);
        Vector2 thighR = new Vector2(landmarksWorldPos[5].x, landmarksWorldPos[5].z);
        Vector2 kneeR = new Vector2(landmarksWorldPos[7].x, landmarksWorldPos[7].z);
        float dL = DistToSegment2D(ptXZ, thighL, kneeL);
        float dR = DistToSegment2D(ptXZ, thighR, kneeR);
        if (leg == 0 && dL > dR + 0.008f) return false;
        if (leg == 1 && dR > dL + 0.008f) return false;

        if (legTemplatePts == null || legTemplatePts.Count == 0)
            return true;

        // Kiểm tra điểm nằm trong bao vùng mẫu hiệu ứng R trên ống chân hiện tại
        float r2 = corridorRadiusM * corridorRadiusM;
        for (int i = 0; i < legTemplatePts.Count; i++)
        {
            if ((ptXZ - legTemplatePts[i]).sqrMagnitude <= r2)
                return true;
        }
        return false;
    }

    /// <summary>
    /// P1 — Quét Zigzag / Boustrophedon trong vùng phun R (Chuẩn trang 7 file góp ý):
    /// 1. Lấy vùng R từ mask/mẫu hiệu ứng, đơn giản hóa biên bằng Douglas-Peucker và co biên một khoảng e.
    /// 2. Tách vùng lõm đáy quần thành 2 ô lồi từng ống (Boustrophedon decomposition), tìm trục dài từng ống bằng PCA để giảm số lượt.
    /// 3. Cắt vùng R bằng họ đường song song cách nhau bước lượt s = w*(1-r), nối zigzag và tắt súng ở đoạn quay đầu ngoài R.
    /// 4. Gán Z bề mặt (2.5D) với độ cao lớp phun thống nhất và hướng súng thẳng đứng (Baseline).
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP1Zigzag(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        out float genTimeSec)
    {
        float t0 = Time.realtimeSinceStartup;
        var strokesWorld = new List<List<Vector3>>();

        if (ctrl.landmarksWorldPos == null || ctrl.landmarksWorldPos.Count < 12)
        {
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        Vector2[] src12 = DobotCR10AIKController.GetCanonicalLandmarks12UV(templateSize);
        Vector2[] dst12XZ = new Vector2[12];
        for (int i = 0; i < 12; i++) dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);

        // Bước 1: Đơn giản hóa biên quần bằng Douglas-Peucker (epsilon = 4 mm)
        List<Vector3> simplifiedContourWorld = ctrl.garmentContourWorld;
        if (ctrl.garmentContourWorld != null && ctrl.garmentContourWorld.Count >= 8)
        {
            var raw2D = new List<Vector2>(ctrl.garmentContourWorld.Count);
            for (int i = 0; i < ctrl.garmentContourWorld.Count; i++)
                raw2D.Add(new Vector2(ctrl.garmentContourWorld[i].x, ctrl.garmentContourWorld[i].z));
            var dp2D = SimplifyPolygonDouglasPeucker(raw2D, 0.004f);
            simplifiedContourWorld = new List<Vector3>(dp2D.Count);
            for (int i = 0; i < dp2D.Count; i++)
                simplifiedContourWorld.Add(new Vector3(dp2D[i].x, 0.02f, dp2D[i].y));
        }

        // Ánh xạ mẫu vẽ sang từng ống để xác định vùng cần phun R_leg (theo mẫu hiệu ứng của mã hàng)
        var legTemplatePts = new List<Vector2>[2] { new List<Vector2>(), new List<Vector2>() };
        if (templateStrokesUV != null)
        {
            Vector2 thighL = dst12XZ[4], kneeL = dst12XZ[6];
            Vector2 thighR = dst12XZ[5], kneeR = dst12XZ[7];
            for (int s = 0; s < templateStrokesUV.Count; s++)
            {
                var raw = templateStrokesUV[s];
                if (raw == null || raw.Count < 2) continue;
                var resampled = ResampleUV(raw, 0.015f);
                for (int p = 0; p < resampled.Count; p++)
                {
                    Vector2 wpt = WarpPointBarycentric12(resampled[p], src12, dst12XZ);
                    float dL = DistToSegment2D(wpt, thighL, kneeL);
                    float dR = DistToSegment2D(wpt, thighR, kneeR);
                    legTemplatePts[dL <= dR ? 0 : 1].Add(wpt);
                }
            }
        }

        float stepSM = Mathf.Clamp(tech.sprayWidthMm * (1f - tech.overlapRatio), 10f, 90f) * 0.001f;
        float sampleStepM = 0.016f;
        float corridorRadiusM = Mathf.Max(stepSM * 0.85f, 0.026f);

        // Bước 2 & 3: Boustrophedon decomposition cho 2 ô lồi (ống trái & ống phải), quét dọc trục dài PCA
        float[] marginAttempts = { tech.borderMarginMm, tech.borderMarginMm * 0.35f, 0f };
        for (int attempt = 0; attempt < marginAttempts.Length && strokesWorld.Count == 0; attempt++)
        {
            float activeMargin = marginAttempts[attempt];
            List<Vector3> activeContour = (attempt < 2) ? simplifiedContourWorld : null;

            for (int leg = 0; leg < 2; leg++)
            {
                Vector2 crotch = dst12XZ[3];
                Vector2 thigh = dst12XZ[leg == 0 ? 4 : 5];
                Vector2 knee = dst12XZ[leg == 0 ? 6 : 7];
                Vector2 cuffMid = 0.5f * (dst12XZ[leg == 0 ? 8 : 10] + dst12XZ[leg == 0 ? 9 : 11]);
                Vector2 fallbackLong = (cuffMid - crotch).sqrMagnitude > 1e-6f ? (cuffMid - crotch).normalized : Vector2.up;

                List<Vector2> pcaInput = legTemplatePts[leg].Count >= 3
                    ? legTemplatePts[leg]
                    : new List<Vector2> { crotch, thigh, knee, cuffMid };
                ComputePcaAxes2D(pcaInput, fallbackLong, out Vector2 centroid, out Vector2 majorAxis, out Vector2 minorAxis);

                // Xác định hộp bao của vùng R_leg trên hệ trục PCA (majorAxis = hướng quét dọc, minorAxis = bước dịch ngang s)
                float minMajor = float.MaxValue, maxMajor = float.MinValue;
                float minMinor = float.MaxValue, maxMinor = float.MinValue;
                for (int i = 0; i < pcaInput.Count; i++)
                {
                    Vector2 rel = pcaInput[i] - centroid;
                    float u = Vector2.Dot(rel, majorAxis);
                    float v = Vector2.Dot(rel, minorAxis);
                    if (u < minMajor) minMajor = u;
                    if (u > maxMajor) maxMajor = u;
                    if (v < minMinor) minMinor = v;
                    if (v > maxMinor) maxMinor = v;
                }
                minMajor -= corridorRadiusM * 0.7f;
                maxMajor += corridorRadiusM * 0.7f;
                minMinor -= corridorRadiusM * 0.7f;
                maxMinor += corridorRadiusM * 0.7f;

                float spanMinor = Mathf.Max(stepSM, maxMinor - minMinor);
                int numPasses = Mathf.Clamp(Mathf.CeilToInt(spanMinor / stepSM) + 1, 2, 18);
                int numSteps = Mathf.Clamp(Mathf.CeilToInt((maxMajor - minMajor) / sampleStepM), 8, 90);

                for (int pass = 0; pass < numPasses; pass++)
                {
                    float vOff = (numPasses == 1)
                        ? 0.5f * (minMinor + maxMinor)
                        : Mathf.Lerp(minMinor, maxMinor, (float)pass / (numPasses - 1));
                    bool reverse = (pass % 2 == 1);
                    var currentSeg = new List<Vector3>();

                    for (int k = 0; k <= numSteps; k++)
                    {
                        float t = reverse ? (1f - (float)k / numSteps) : ((float)k / numSteps);
                        float uOff = Mathf.Lerp(minMajor, maxMajor, t);
                        Vector2 ptXZ = centroid + uOff * majorAxis + vOff * minorAxis;

                        if (IsPointInTemplateRegionR(ptXZ, leg, legTemplatePts[leg], corridorRadiusM, activeContour, ctrl.landmarksWorldPos, activeMargin))
                        {
                            currentSeg.Add(new Vector3(ptXZ.x, 0.02f, ptXZ.y));
                        }
                        else if (currentSeg.Count >= 2)
                        {
                            strokesWorld.Add(currentSeg);
                            currentSeg = new List<Vector3>();
                        }
                        else
                        {
                            currentSeg.Clear();
                        }
                    }
                    if (currentSeg.Count >= 2) strokesWorld.Add(currentSeg);
                }
            }
        }

        // Bước 4: Gán Z (2.5D) và hướng súng thẳng đứng (Baseline P1)
        var wps = BuildSprayWaypointsFromWorldStrokes(strokesWorld, ctrl, plyLoader, tech, "P1_ZIGZAG", useFixedVerticalNormal: true);
        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
    }

    /// <summary>
    /// P2 — Đường theo hình dạng (Song song biên / theo trục giữa — Skeleton, Chuẩn trang 7 file góp ý):
    /// 1. Tính trục giữa (medial axis / skeleton) của từng ống quần từ mask/12 mốc kèm bề rộng thực của ống từ đùi xuống gấu.
    /// 2. Sinh các đường cong song song với trục giữa cách nhau bước s = w*(1-r) dọc theo ống.
    /// 3. Bề rộng ống thu hẹp từ đùi xuống gấu -> tự động cắt bớt các lượt ngoài khi vượt quá biên vùng R hoặc bề rộng ống tại trạm đó.
    /// 4. Gán Z bề mặt và hướng súng theo pháp tuyến đã làm mượt.
    /// </summary>
    public static List<DobotCR10AIKController.WaypointData> GenerateP2ShapeSkeleton(
        DobotCR10AIKController ctrl,
        PLYDirectLoader plyLoader,
        List<List<Vector2>> templateStrokesUV,
        string templateSize,
        SprayTechParams tech,
        out float genTimeSec)
    {
        float t0 = Time.realtimeSinceStartup;
        var strokesWorld = new List<List<Vector3>>();

        if (ctrl.landmarksWorldPos == null || ctrl.landmarksWorldPos.Count < 12)
        {
            genTimeSec = Time.realtimeSinceStartup - t0;
            return new List<DobotCR10AIKController.WaypointData>();
        }

        Vector2[] src12 = DobotCR10AIKController.GetCanonicalLandmarks12UV(templateSize);
        Vector2[] dst12XZ = new Vector2[12];
        for (int i = 0; i < 12; i++) dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);

        var legTemplatePts = new List<Vector2>[2] { new List<Vector2>(), new List<Vector2>() };
        if (templateStrokesUV != null)
        {
            Vector2 thighL = dst12XZ[4], kneeL = dst12XZ[6];
            Vector2 thighR = dst12XZ[5], kneeR = dst12XZ[7];
            for (int s = 0; s < templateStrokesUV.Count; s++)
            {
                var raw = templateStrokesUV[s];
                if (raw == null || raw.Count < 2) continue;
                var resampled = ResampleUV(raw, 0.015f);
                for (int p = 0; p < resampled.Count; p++)
                {
                    Vector2 wpt = WarpPointBarycentric12(resampled[p], src12, dst12XZ);
                    float dL = DistToSegment2D(wpt, thighL, kneeL);
                    float dR = DistToSegment2D(wpt, thighR, kneeR);
                    legTemplatePts[dL <= dR ? 0 : 1].Add(wpt);
                }
            }
        }

        float stepSM = Mathf.Clamp(tech.sprayWidthMm * (1f - tech.overlapRatio), 10f, 90f) * 0.001f;
        float sampleStepM = 0.016f;
        float corridorRadiusM = Mathf.Max(stepSM * 0.85f, 0.026f);

        float[] marginAttempts = { tech.borderMarginMm, tech.borderMarginMm * 0.35f, 0f };
        for (int attempt = 0; attempt < marginAttempts.Length && strokesWorld.Count == 0; attempt++)
        {
            float activeMargin = marginAttempts[attempt];
            List<Vector3> activeContour = (attempt < 2) ? ctrl.garmentContourWorld : null;
            float marginM = activeMargin * 0.001f;

            for (int leg = 0; leg < 2; leg++)
            {
                // Bước 1: Tính trục giữa (Skeleton) cong mượt dọc ống (Hông/Đũng -> Đùi -> Gối -> Gấu) và bán kính bề rộng ống R(s)
                AnatomicalLegChain chain = BuildLegChain(dst12XZ, leg == 0);
                float maxHalfWidthM = Mathf.Max(chain.R[1], Mathf.Max(chain.R[2], Mathf.Max(chain.R[3], chain.R[4])));
                int maxSidePasses = Mathf.Clamp(Mathf.CeilToInt(maxHalfWidthM / stepSM), 1, 8);

                float legLenM = Vector2.Distance(chain.C[1], chain.C[2])
                              + Vector2.Distance(chain.C[2], chain.C[3])
                              + Vector2.Distance(chain.C[3], chain.C[4]);
                int steps = Mathf.Clamp(Mathf.CeilToInt(legLenM / sampleStepM), 14, 90);

                // Bước 2: Sinh các đường song song với trục giữa cách đều bước s (từ ngoài vào tâm rồi sang mép đối diện)
                for (int kPass = -maxSidePasses; kPass <= maxSidePasses; kPass++)
                {
                    float offsetMeters = kPass * stepSM;
                    bool reverse = ((kPass + maxSidePasses) % 2 == 1);
                    var currentSeg = new List<Vector3>();

                    for (int k = 0; k <= steps; k++)
                    {
                        float frac = reverse ? (1f - (float)k / steps) : ((float)k / steps);
                        float sVal = Mathf.Lerp(1.05f, 3.85f, frac);
                        int seg = Mathf.Clamp(Mathf.FloorToInt(sVal), 1, 3);
                        float uLocal = Mathf.Clamp01(sVal - seg);

                        Vector2 centerPt = chain.C[seg] + (chain.C[seg + 1] - chain.C[seg]) * uLocal;
                        Vector2 normalVec = Vector2.Lerp(chain.N[seg], chain.N[seg + 1], uLocal);
                        if (normalVec.sqrMagnitude < 1e-6f) normalVec = chain.N[seg];
                        else normalVec.Normalize();
                        float localHalfWidthM = Mathf.Lerp(chain.R[seg], chain.R[seg + 1], uLocal);

                        // Bước 3: Cắt lượt ngoài khi ống quần thu hẹp từ đùi xuống gấu (|offset| > bề rộng ống tại trạm - biên e)
                        float usableHalfWidth = Mathf.Max(0.015f, localHalfWidthM - marginM * 0.6f);
                        if (Mathf.Abs(offsetMeters) > usableHalfWidth)
                        {
                            if (currentSeg.Count >= 2)
                            {
                                strokesWorld.Add(currentSeg);
                                currentSeg = new List<Vector3>();
                            }
                            else
                            {
                                currentSeg.Clear();
                            }
                            continue;
                        }

                        Vector2 ptXZ = centerPt + normalVec * offsetMeters;
                        if (IsPointInTemplateRegionR(ptXZ, leg, legTemplatePts[leg], corridorRadiusM, activeContour, ctrl.landmarksWorldPos, activeMargin))
                        {
                            currentSeg.Add(new Vector3(ptXZ.x, 0.02f, ptXZ.y));
                        }
                        else if (currentSeg.Count >= 2)
                        {
                            strokesWorld.Add(currentSeg);
                            currentSeg = new List<Vector3>();
                        }
                        else
                        {
                            currentSeg.Clear();
                        }
                    }
                    if (currentSeg.Count >= 2) strokesWorld.Add(currentSeg);
                }
            }
        }

        var wps = BuildSprayWaypointsFromWorldStrokes(strokesWorld, ctrl, plyLoader, tech, "P2_SKELETON", useFixedVerticalNormal: false);
        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
    }

    /// <summary>
    /// P3 — Quỹ đạo mẫu theo mốc (Tham số hóa Tọa độ Trọng tâm Barycentric trên Lưới Tam Giác 12 Mốc, Chuẩn trang 7 file góp ý):
    /// 1. Chia quần chuẩn thành lưới 14 tam giác theo 12 mốc giải phẫu (YOLO11-pose).
    /// 2. Biểu diễn mỗi điểm quỹ đạo mẫu bằng tọa độ trọng tâm (u, v, w) trong tam giác chứa nó.
    /// 3. Với quần mới: dựng lại lưới từ 12 mốc thực -> biến đổi affine từng mảnh (u*A' + v*B' + w*C') -> cắt vùng cấm -> gán Z + pháp tuyến 3D.
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
        Vector2[] dst12XZ = new Vector2[12];
        for (int i = 0; i < 12; i++) dst12XZ[i] = new Vector2(ctrl.landmarksWorldPos[i].x, ctrl.landmarksWorldPos[i].z);

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
                    // Biến đổi Affine từng mảnh bằng tọa độ trọng tâm (Barycentric) trên lưới tam giác 12 mốc
                    Vector2 wXZ = WarpPointBarycentric12(resampled[p], src12, dst12XZ);
                    Vector3 w = new Vector3(wXZ.x, 0.02f, wXZ.y);
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

        var wps = BuildSprayWaypointsFromWorldStrokes(strokesWorld, ctrl, plyLoader, tech, "P3_BARYCENTRIC12");
        genTimeSec = Mathf.Max(0.001f, Time.realtimeSinceStartup - t0);
        return wps;
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
    /// P4 — Quỹ đạo mẫu + Đăng ký đám mây điểm (Chuẩn 5 bước trang 8 file góp ý):
    /// 1. Khởi tạo thô bằng lưới mốc Barycentric (P3).
    /// 2. ICP cứng (Iterative Closest Point + Kabsch 2D) trên đám mây biên giảm mẫu (~5 mm) để khử tịnh tiến/quay.
    /// 3. Đăng ký phi cứng Thin Plate Spline (TPS RBF thực r² ln r) từ các cặp điểm tương ứng sau ICP.
    /// 4. Áp trường biến dạng lên quỹ đạo mẫu, chiếu lại lên bề mặt 3D thật, tính lại pháp tuyến, lùi khoảng d.
    /// 5. Tính sai số dư sau đăng ký (registrationResidualMm); nếu vượt ngưỡng thì chặn & báo kỹ sư.
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

        // Bước 1: Khởi tạo thô bằng mốc (P3 Barycentric) và giảm mẫu biên ~5 mm (0.005 m)
        Vector2[] polyTemplate = DobotCR10AIKController.GetJeansPolygonForSize(templateSize);
        var templateLoopUV = new List<Vector2>(polyTemplate);
        if (templateLoopUV.Count > 0) templateLoopUV.Add(templateLoopUV[0]);
        List<Vector2> denseTemplateEdgeUV = ResampleUV(templateLoopUV, 0.015f);

        var initBoundaryCloud = new List<Vector2>(denseTemplateEdgeUV.Count);
        for (int i = 0; i < denseTemplateEdgeUV.Count; i++)
        {
            initBoundaryCloud.Add(WarpPointBarycentric12(denseTemplateEdgeUV[i], src12, dst12XZ));
        }

        // Giảm mẫu đám mây biên thực của quần mới với voxel ~5 mm (0.005 m) theo đúng Mục P4.3 trang 8
        var targetContourCloud = new List<Vector2>();
        if (ctrl.garmentContourWorld != null && ctrl.garmentContourWorld.Count >= 12)
        {
            for (int k = 0; k < ctrl.garmentContourWorld.Count; k++)
            {
                Vector2 cand = new Vector2(ctrl.garmentContourWorld[k].x, ctrl.garmentContourWorld[k].z);
                if (targetContourCloud.Count == 0 || (cand - targetContourCloud[targetContourCloud.Count - 1]).sqrMagnitude >= 0.005f * 0.005f)
                {
                    targetContourCloud.Add(cand);
                }
            }
        }

        // Bước 2: Chạy ICP cứng (Kabsch 2D, 10 vòng lặp) để khử tịnh tiến/quay giữa mẫu khởi tạo và đám mây biên thực
        float icpCos = 1f, icpSin = 0f;
        Vector2 icpCenter = Vector2.zero, icpTrans = Vector2.zero;
        if (targetContourCloud.Count >= 6)
        {
            RunRigidKabschIcp2D(initBoundaryCloud, targetContourCloud, 10, 0.065f, out icpCos, out icpSin, out icpCenter, out icpTrans);
        }

        // Bước 3: Lập tập cặp điểm tương ứng (12 mốc + các điểm biên sau ICP cứng) để giải trường biến dạng phi cứng TPS RBF
        var tpsSrcList = new List<Vector2>(28);
        var tpsDstList = new List<Vector2>(28);
        for (int i = 0; i < 12; i++)
        {
            Vector2 p3Pt = WarpPointBarycentric12(src12[i], src12, dst12XZ);
            Vector2 icpPt = ApplyRigid2D(p3Pt, icpCos, icpSin, icpCenter, icpTrans);
            tpsSrcList.Add(icpPt);
            tpsDstList.Add(dst12XZ[i]);
        }

        double boundaryErrSumSq = 0.0;
        int boundaryMatchCount = 0;
        if (targetContourCloud.Count >= 12)
        {
            for (int i = 0; i < polyTemplate.Length; i++)
            {
                Vector2 p3Edge = WarpPointBarycentric12(polyTemplate[i], src12, dst12XZ);
                Vector2 icpEdge = ApplyRigid2D(p3Edge, icpCos, icpSin, icpCenter, icpTrans);

                bool tooCloseToExisting = false;
                for (int j = 0; j < tpsSrcList.Count; j++)
                {
                    if (Vector2.Distance(icpEdge, tpsSrcList[j]) < 0.035f) { tooCloseToExisting = true; break; }
                }
                if (tooCloseToExisting) continue;

                Vector2 bestCand = icpEdge;
                float bestD2 = float.MaxValue;
                for (int k = 0; k < targetContourCloud.Count; k++)
                {
                    float d2 = (targetContourCloud[k] - icpEdge).sqrMagnitude;
                    if (d2 < bestD2) { bestD2 = d2; bestCand = targetContourCloud[k]; }
                }
                float distM = Mathf.Sqrt(bestD2);
                if (distM <= 0.055f)
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
        const double lambda = 2e-4;

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

        // Bước 5: Tính sai số dư sau đăng ký (RMS mm) trên 12 mốc + các cặp biên ICP
        double sumSqErr = 0.0;
        for (int i = 0; i < n; i++)
        {
            Vector2 pred = (okX && okZ)
                ? EvalTps2D(tpsCtrl[i], tpsCtrl, wX, wZ)
                : tpsDstList[i];
            double errM = Vector2.Distance(pred, tpsDstList[i]);
            sumSqErr += errM * errM;
        }
        double rmsM = Math.Sqrt((sumSqErr + 0.05 * boundaryErrSumSq) / Math.Max(1, n));
        registrationResidualMm = (float)(rmsM * 1000.0);

        // Bước 4: Áp chuỗi biến đổi (P3 Barycentric -> ICP cứng Kabsch -> TPS phi cứng) lên quỹ đạo mẫu & chiếu lên bề mặt 3D thật
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
                    Vector2 qP3 = WarpPointBarycentric12(resampled[p], src12, dst12XZ);
                    Vector2 qIcp = ApplyRigid2D(qP3, icpCos, icpSin, icpCenter, icpTrans);
                    Vector2 qWarped = (okX && okZ) ? EvalTps2D(qIcp, tpsCtrl, wX, wZ) : qP3;
                    if (Vector2.Distance(qWarped, qP3) > 0.035f)
                    {
                        qWarped = qP3 + (qWarped - qP3).normalized * 0.035f;
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
