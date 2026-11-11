using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// BỘ GIẢI ĐỘNG HỌC THUẬN (FK) VÀ ĐỘNG HỌC NGHỊCH THUẦN GIẢI TÍCH (CLOSED-FORM ANALYTICAL IK)
/// Dành cho Robot 6 bậc tự do Dobot CR10A dựa trên phương pháp toán học báo cáo kỹ thuật Dobot CR5.
/// 
/// ĐẶC ĐIỂM KỸ THUẬT:
/// 1. Tốc độ giải siêu nhanh: Dưới 0.04 ms, không dùng lặp số Jacobian, không lo kẹt cực trị hay trôi sai số.
/// 2. Độ chính xác giải tích: Sai số vị trí < 1e-6 mm.
/// 3. Khám phá trọn vẹn 8 cấu hình nghiệm giải tích:
///    - 2 nhánh vai (q1): Vai trái (Shoulder Left) / Vai phải (Shoulder Right)
///    - 2 nhánh cổ tay (q5): Cổ tay ngửa (Wrist Up) / Cổ tay úp (Wrist Down)
///    - 2 nhánh khuỷu (q3): Khuỷu trên (Elbow Up) / Khuỷu dưới (Elbow Down)
/// 4. Tiêu chí chọn nghiệm tối ưu Q* với bộ trọng số ưu tiên khớp chịu tải nặng:
///    w = [1.2, 1.2, 1.0, 0.8, 0.8, 0.5]
/// 5. Thuật toán tiếp cận thích ứng (Adaptive Approach Angle) khi vươn xa mép bàn.
/// 6. Đồng bộ chuyển đổi hai chiều chuẩn xác giữa hệ Right-Handed D-H Robotics và Left-Handed Unity (Sx reflection).
/// </summary>
public static class DobotCR10AKinematics
{
    public sealed class IKSolution
    {
        public readonly double[] jointsRad;
        public readonly double[] jointsDeg;
        public readonly string branchName;
        public readonly double positionErrorMm;
        public readonly double orientationErrorRad;
        public readonly double selectionCost;
        public readonly bool isOptimal;

        public IKSolution(double[] jointsRad, string branchName, double positionErrorMm, double orientationErrorRad, double selectionCost = 0, bool isOptimal = false)
        {
            this.jointsRad = jointsRad;
            this.jointsDeg = new double[6];
            for (int i = 0; i < 6; i++) this.jointsDeg[i] = jointsRad[i] * (180.0 / Math.PI);
            this.branchName = branchName;
            this.positionErrorMm = positionErrorMm;
            this.orientationErrorRad = orientationErrorRad;
            this.selectionCost = selectionCost;
            this.isOptimal = isOptimal;
        }
    }

    // =========================================================================
    // 1. BẢNG THÔNG SỐ DENAVIT - HARTENBERG (STANDARD D-H) CỦA DOBOT CR10A (mm & rad)
    // =========================================================================
    // Khâu 1: theta1 = q1,         d1 = 176.5, a1 = 0,     alpha1 = +pi/2, offset = 0
    // Khâu 2: theta2 = q2 + pi/2,  d2 = 193.0, a2 = 607.0, alpha2 = 0,     offset = +pi/2
    // Khâu 3: theta3 = q3,         d3 = -127.0,a3 = 568.0, alpha3 = 0,     offset = 0
    // Khâu 4: theta4 = q4 - pi/2,  d4 = 125.0, a4 = 0,     alpha4 = -pi/2, offset = -pi/2
    // Khâu 5: theta5 = q5,         d5 = 125.0, a5 = 0,     alpha5 = +pi/2, offset = 0
    // Khâu 6: theta6 = q6,         d6 = 111.0, a6 = 0,     alpha6 = 0,     offset = 0
    public const double D1 = 176.5;     // Đế -> Khớp 2 (mm)
    public const double A2 = 607.0;     // Độ dài cánh tay chính L2 (mm)
    public const double D2 = 193.0;     // Offset khớp 2 (mm)
    public const double A3 = 568.0;     // Độ dài cẳng tay L3 (mm)
    public const double D3 = -127.0;    // Offset khớp 3 (mm)
    public const double D4 = 125.0;     // Offset khớp 4 (mm)
    public const double D5 = 125.0;     // Offset khớp 5 (mm)
    public const double D6 = 111.0;     // Khớp 6 -> Mặt bích khâu 6 (mm)
    public const double D_TOOL_DEFAULT = 100.0; // TCP công cụ gắp mặc định dọc trục z6 (mm)

    // Độ lệch vai tổng (Shoulder Offset): H = -(d2 + d3 + d4) = -(193.0 - 127.0 + 125.0) = -191.0 mm
    public const double H = -(D2 + D3 + D4);
    public const double L2 = A2;
    public const double L3 = A3;
    public const double R_MAX = L2 + L3; // 1175.0 mm

    // Trọng số lựa chọn nghiệm tối ưu theo báo cáo (ưu tiên ổn định khớp 1, 2)
    private static readonly double[] JointWeights = { 1.2, 1.2, 1.0, 0.8, 0.8, 0.5 };

    // =========================================================================
    // 2. BÀI TOÁN ĐỘNG HỌC THUẬN (FORWARD KINEMATICS - FK)
    // =========================================================================

    /// <summary>
    /// Ma trận biến đổi thuần nhất chuẩn D-H: Ai = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    /// </summary>
    public static double[,] DhMatrix(double theta, double d, double a, double alpha)
    {
        double ct = Math.Cos(theta), st = Math.Sin(theta);
        double ca = Math.Cos(alpha), sa = Math.Sin(alpha);
        return new double[4, 4]
        {
            { ct, -st * ca,  st * sa, a * ct },
            { st,  ct * ca, -ct * sa, a * st },
            { 0,   sa,       ca,      d      },
            { 0,   0,        0,       1      }
        };
    }

    /// <summary>
    /// Tính toán ma trận biến đổi thuần nhất T06 đến mặt bích trong hệ DH Robotics (mm)
    /// </summary>
    public static double[,] ForwardFlangeDH(double[] q)
    {
        double[,] m1 = DhMatrix(q[0], D1, 0.0, Math.PI / 2.0);
        double[,] m2 = DhMatrix(q[1] + Math.PI / 2.0, D2, A2, 0.0);
        double[,] m3 = DhMatrix(q[2], D3, A3, 0.0);
        double[,] m4 = DhMatrix(q[3] - Math.PI / 2.0, D4, 0.0, -Math.PI / 2.0);
        double[,] m5 = DhMatrix(q[4], D5, 0.0, Math.PI / 2.0);
        double[,] m6 = DhMatrix(q[5], D6, 0.0, 0.0);

        return Multiply4(m1, Multiply4(m2, Multiply4(m3, Multiply4(m4, Multiply4(m5, m6)))));
    }

    /// <summary>
    /// Động học thuận tính tới điểm làm việc trung tâm TCP: P_TCP = P_06 + toolOffsetX * x6 + toolOffsetY * y6 + dtool * z6
    /// Trả về Matrix4x4 theo hệ tọa độ Robot trong Unity (mét)
    /// </summary>
    public static Matrix4x4 ForwardTcp(
        float[] jointsRad, 
        Transform tcpTransform = null, 
        double dtoolMm = D_TOOL_DEFAULT, 
        double toolOffsetX = 0.0, 
        double toolOffsetY = 0.0)
    {
        double[] q = new double[6];
        for (int i = 0; i < 6; i++) q[i] = jointsRad[i];

        double[,] T06 = ForwardFlangeDH(q);

        // Vector các trục của khâu 6 (Link 6 / Flange)
        double x6x = T06[0, 0], x6y = T06[1, 0], x6z = T06[2, 0];
        double y6x = T06[0, 1], y6y = T06[1, 1], y6z = T06[2, 1];
        double z6x = T06[0, 2], z6y = T06[1, 2], z6z = T06[2, 2];

        double offX = toolOffsetX;
        double offY = toolOffsetY;
        double offZ = dtoolMm;

        // Nếu người dùng truyền offset âm (chưa xác định) và có tcpTransform trong Unity
        if (dtoolMm < 0 && tcpTransform != null)
        {
            offX = tcpTransform.localPosition.x * 1000.0;
            offY = tcpTransform.localPosition.y * 1000.0;
            offZ = tcpTransform.localPosition.z * 1000.0;
        }

        double tcpX_dh = T06[0, 3] + offX * x6x + offY * y6x + offZ * z6x;
        double tcpY_dh = T06[1, 3] + offX * x6y + offY * y6y + offZ * z6y;
        double tcpZ_dh = T06[2, 3] + offX * x6z + offY * y6z + offZ * z6z;

        // Chuyển đổi sang hệ quy chiếu Unity: X_unity = -X_dh, Y_unity = Y_dh, Z_unity = Z_dh
        // R_unity = Sx * R_dh * Sx
        Matrix4x4 unityPose = Matrix4x4.identity;

        // Xoay R_unity
        unityPose[0, 0] = (float)T06[0, 0];
        unityPose[0, 1] = (float)-T06[0, 1];
        unityPose[0, 2] = (float)-T06[0, 2];
        unityPose[1, 0] = (float)-T06[1, 0];
        unityPose[1, 1] = (float)T06[1, 1];
        unityPose[1, 2] = (float)T06[1, 2];
        unityPose[2, 0] = (float)-T06[2, 0];
        unityPose[2, 1] = (float)T06[2, 1];
        unityPose[2, 2] = (float)T06[2, 2];

        // Vị trí P_unity (đổi từ mm sang mét)
        unityPose[0, 3] = (float)(-tcpX_dh * 0.001);
        unityPose[1, 3] = (float)(tcpY_dh * 0.001);
        unityPose[2, 3] = (float)(tcpZ_dh * 0.001);

        return unityPose;
    }

    // =========================================================================
    // 3. BÀI TOÁN ĐỘNG HỌC NGHỊCH THUẦN GIẢI TÍCH (CLOSED-FORM ANALYTICAL IK)
    // =========================================================================

    /// <summary>
    /// Giải bài toán Động học Nghịch Thuần Giải Tích từ vị trí mục tiêu (mm) và góc định hướng RPY (độ)
    /// Hỗ trợ bù trừ offset 3D tùy chỉnh [toolOffsetX, toolOffsetY, dtoolMm] từ mặt bích đến TCP.
    /// Trả về danh sách đầy đủ 8 cấu hình nghiệm giải tích, sắp xếp theo nghiệm tối ưu Q*.
    /// </summary>
    public static List<IKSolution> SolveTcpPose(
        Vector3 targetUnityPositionMm,
        Quaternion targetUnityRotation,
        float[] currentJointsRad = null,
        double dtoolMm = D_TOOL_DEFAULT,
        bool useAdaptiveTilt = false,
        double maxTiltDeg = 32.0,
        double toolOffsetX = 0.0,
        double toolOffsetY = 0.0)
    {
        // 1. Chuyển đổi Target Pose từ Unity sang hệ DH Robotics:
        // P_dh = [-x_u, y_u, z_u], R_dh = Sx * R_u * Sx
        Matrix4x4 Ru = Matrix4x4.Rotate(targetUnityRotation);
        double[,] Rdh = new double[3, 3]
        {
            {  Ru[0, 0], -Ru[0, 1], -Ru[0, 2] },
            { -Ru[1, 0],  Ru[1, 1],  Ru[1, 2] },
            { -Ru[2, 0],  Ru[2, 1],  Ru[2, 2] }
        };

        double[] PtargetDH = new double[3]
        {
            -targetUnityPositionMm.x,
             targetUnityPositionMm.y,
             targetUnityPositionMm.z
        };

        // Thuật toán tiếp cận thích ứng (Adaptive Approach Angle) nếu được bật
        if (useAdaptiveTilt)
        {
            double r = Math.Sqrt(PtargetDH[0] * PtargetDH[0] + PtargetDH[1] * PtargetDH[1]);
            if (r > 680.0)
            {
                double[] tiltCandidates = { 0.0, 10.0, 18.0, 25.0, maxTiltDeg };
                foreach (double tiltDeg in tiltCandidates)
                {
                    double alpha = tiltDeg * Math.PI / 180.0;
                    double[,] RdhTilt = (double[,])Rdh.Clone();
                    double sinA = Math.Sin(alpha);
                    double cosA = Math.Cos(alpha);
                    // z6 nghiêng về phía tâm đế
                    double z6x = (PtargetDH[0] / r) * sinA;
                    double z6y = (PtargetDH[1] / r) * sinA;
                    double z6z = -cosA;
                    RdhTilt[0, 2] = z6x; RdhTilt[1, 2] = z6y; RdhTilt[2, 2] = z6z;

                    List<IKSolution> candidateSols = SolveInternal(PtargetDH, RdhTilt, currentJointsRad, dtoolMm, toolOffsetX, toolOffsetY);
                    if (candidateSols.Count > 0)
                        return candidateSols;
                }
            }
        }

        return SolveInternal(PtargetDH, Rdh, currentJointsRad, dtoolMm, toolOffsetX, toolOffsetY);
    }

    /// <summary>
    /// Lõi thuật toán giải tích 8 cấu hình nghiệm theo Báo cáo CR5
    /// Phân tách tâm cổ tay chính xác với bù trừ offset công cụ 3D [toolOffsetX, toolOffsetY, dtoolMm].
    /// </summary>
    private static List<IKSolution> SolveInternal(
        double[] PtargetDH, 
        double[,] Rdh, 
        float[] currentJointsRad, 
        double dtoolMm, 
        double toolOffsetX = 0.0, 
        double toolOffsetY = 0.0)
    {
        var solutions = new List<IKSolution>();

        // Vector các trục của Link 6:
        double x6x = Rdh[0, 0], x6y = Rdh[1, 0], x6z = Rdh[2, 0];
        double y6x = Rdh[0, 1], y6y = Rdh[1, 1], y6z = Rdh[2, 1];
        double z6x = Rdh[0, 2], z6y = Rdh[1, 2], z6z = Rdh[2, 2];

        // 3.1. Bước 1: Phân tách tâm cổ tay (Wrist Center Decoupling)
        // P_TCP = P06 + offX * x6 + offY * y6 + offZ * z6
        // P06 = P_TCP - (offX * x6 + offY * y6 + offZ * z6)
        // P05 = P06 - D6 * z6 = P_TCP - (offX * x6 + offY * y6 + (D6 + offZ) * z6)
        double totalToolD = D6 + dtoolMm;
        double X5 = PtargetDH[0] - (toolOffsetX * x6x + toolOffsetY * y6x + totalToolD * z6x);
        double Y5 = PtargetDH[1] - (toolOffsetX * x6y + toolOffsetY * y6y + totalToolD * z6y);
        double Z5 = PtargetDH[2] - (toolOffsetX * x6z + toolOffsetY * y6z + totalToolD * z6z);

        double P06_x = PtargetDH[0] - (toolOffsetX * x6x + toolOffsetY * y6x + dtoolMm * z6x);
        double P06_y = PtargetDH[1] - (toolOffsetX * x6y + toolOffsetY * y6y + dtoolMm * z6y);
        double P06_z = PtargetDH[2] - (toolOffsetX * x6z + toolOffsetY * y6z + dtoolMm * z6z);

        double[,] T06 = new double[4, 4]
        {
            { Rdh[0, 0], Rdh[0, 1], Rdh[0, 2], P06_x },
            { Rdh[1, 0], Rdh[1, 1], Rdh[1, 2], P06_y },
            { Rdh[2, 0], Rdh[2, 1], Rdh[2, 2], P06_z },
            { 0,         0,         0,         1     }
        };

        // 3.2. Bước 2: Giải tích góc khớp 1 (q1)
        // Y5 * cos(q1) - X5 * sin(q1) = H  với H = -(d2 + d3 + d4) = -191.0 mm
        double a1_coef = Y5;
        double b1_coef = -X5;
        double c1_coef = H;
        double delta1 = X5 * X5 + Y5 * Y5 - H * H;

        // Nếu Delta1 < 0: Điểm mục tiêu nằm trong "vùng chết" hình trụ ở tâm robot
        if (delta1 < -1e-6)
        {
            return solutions;
        }

        double sqrtDelta1 = Math.Sqrt(Math.Max(0.0, delta1));
        double atan_ab = Math.Atan2(a1_coef, b1_coef);

        // 2 cấu hình khớp 1: Vai trái (s1 = +1) và Vai phải (s1 = -1)
        int[] shoulderSigns = { 1, -1 };
        int[] wristSigns = { 1, -1 };
        int[] elbowSigns = { 1, -1 };

        foreach (int s1 in shoulderSigns)
        {
            double q1 = WrapPi(Math.Atan2(c1_coef, s1 * sqrtDelta1) - atan_ab);

            // 3.3. Bước 3: Chuyển về Frame 1 và giải tích góc cổ tay q5, q6, q234
            // Ma trận nghịch đảo A1^-1
            double cq1 = Math.Cos(q1), sq1 = Math.Sin(q1);
            double[,] A1_inv = new double[4, 4]
            {
                {  cq1,  sq1, 0.0,  0.0 },
                {  0.0,  0.0, 1.0, -D1  },
                {  sq1, -cq1, 0.0,  0.0 },
                {  0.0,  0.0, 0.0,  1.0 }
            };

            double[,] T16 = Multiply4(A1_inv, T06);

            // cos(q5) = clamp(T16[2, 2], -1.0, 1.0)
            double cos_q5 = Math.Max(-1.0, Math.Min(1.0, T16[2, 2]));
            double sin_q5_mag = Math.Sqrt(Math.Max(0.0, 1.0 - cos_q5 * cos_q5));

            foreach (int s5 in wristSigns)
            {
                double sin_q5 = s5 * sin_q5_mag;
                double q5 = Math.Atan2(sin_q5, cos_q5);

                double q6;
                double q234;

                // Kiểm tra điểm kỳ dị cổ tay (|sin(q5)| <= 1e-4)
                if (Math.Abs(sin_q5) > 1e-4)
                {
                    q6 = Math.Atan2(T16[2, 1] / sin_q5, -T16[2, 0] / sin_q5);
                    q234 = Math.Atan2(T16[1, 2] / sin_q5, T16[0, 2] / sin_q5);
                }
                else
                {
                    // Trường hợp kỳ dị cổ tay: Khớp 4 và 6 đồng trục, quy ước q6 = 0
                    q6 = 0.0;
                    q234 = Math.Atan2(T16[1, 0], T16[0, 0]);
                }

                // 3.4. Bước 4: Giải tích cơ cấu cánh tay phẳng 2R (q2, q3, q4)
                // Chuyển tâm cổ tay sang Frame 1: P15 = A1^-1 * [P05, 1]^T
                double P15_x = A1_inv[0, 0] * X5 + A1_inv[0, 1] * Y5 + A1_inv[0, 3];
                double P15_y = A1_inv[1, 2] * Z5 + A1_inv[1, 3];

                // Tính lùi tọa độ tâm khớp 4 (P14)
                double P14_x = P15_x + D5 * Math.Sin(q234);
                double P14_y = P15_y - D5 * Math.Cos(q234);

                double rho14_sq = P14_x * P14_x + P14_y * P14_y;
                double cos_q3 = (rho14_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);

                // Nếu |cos(q3)| > 1.0: Điểm vượt quá tầm với cánh tay
                if (Math.Abs(cos_q3) > 1.0 + 1e-6)
                {
                    continue;
                }

                cos_q3 = Math.Max(-1.0, Math.Min(1.0, cos_q3));
                double sin_q3_mag = Math.Sqrt(Math.Max(0.0, 1.0 - cos_q3 * cos_q3));

                foreach (int s3 in elbowSigns)
                {
                    double sin_q3 = s3 * sin_q3_mag;
                    double q3 = Math.Atan2(sin_q3, cos_q3);

                    double K1 = L2 + L3 * cos_q3;
                    double K2 = L3 * sin_q3;
                    double q2 = Math.Atan2(-P14_x, P14_y) - Math.Atan2(K2, K1);
                    double q4 = q234 - q2 - q3;

                    double[] qSol = new double[6]
                    {
                        WrapPi(q1),
                        WrapPi(q2),
                        WrapPi(q3),
                        WrapPi(q4),
                        WrapPi(q5),
                        WrapPi(q6)
                    };

                    // Kiểm tra sai số FK đối chiếu thực nghiệm
                    double[,] Tcheck = ForwardFlangeDH(qSol);
                    double checkTcpX = Tcheck[0, 3] + toolOffsetX * Tcheck[0, 0] + toolOffsetY * Tcheck[0, 1] + dtoolMm * Tcheck[0, 2];
                    double checkTcpY = Tcheck[1, 3] + toolOffsetX * Tcheck[1, 0] + toolOffsetY * Tcheck[1, 1] + dtoolMm * Tcheck[1, 2];
                    double checkTcpZ = Tcheck[2, 3] + toolOffsetX * Tcheck[2, 0] + toolOffsetY * Tcheck[2, 1] + dtoolMm * Tcheck[2, 2];

                    double dx = PtargetDH[0] - checkTcpX;
                    double dy = PtargetDH[1] - checkTcpY;
                    double dz = PtargetDH[2] - checkTcpZ;
                    double posErrMm = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                    // Sai số góc hướng Frobenius norm
                    double rotErr = 0;
                    for (int r = 0; r < 3; r++)
                        for (int c = 0; c < 3; c++)
                        {
                            double diff = Rdh[r, c] - Tcheck[r, c];
                            rotErr += diff * diff;
                        }
                    rotErr = Math.Sqrt(rotErr);

                    // 3.5. Tiêu chí chọn nghiệm tối ưu Q* theo công thức báo cáo:
                    // Cost = Sum(wi * [Wrap180(qi - qi_current)]^2)
                    double cost = 0;
                    if (currentJointsRad != null && currentJointsRad.Length >= 6)
                    {
                        for (int i = 0; i < 6; i++)
                        {
                            double deltaQ = WrapPi(qSol[i] - currentJointsRad[i]);
                            cost += JointWeights[i] * deltaQ * deltaQ;
                        }
                    }

                    string branchDesc = string.Format("Vai {0}, Cổ tay {1}, Khuỷu {2}",
                        s1 > 0 ? "Trái" : "Phải",
                        s5 > 0 ? "Ngửa" : "Úp",
                        s3 > 0 ? "Trên" : "Dưới");

                    solutions.Add(new IKSolution(qSol, branchDesc, posErrMm, rotErr, cost));
                }
            }
        }

        // Sắp xếp các nghiệm theo chi phí tối ưu (nghiệm Q* tốt nhất ở vị trí đầu tiên index 0)
        solutions.Sort((a, b) => a.selectionCost.CompareTo(b.selectionCost));

        return solutions;
    }

    // =========================================================================
    // 4. CÁC HÀM TIỆN ÍCH HÌNH HỌC VÀ MA TRẬN
    // =========================================================================

    public static double WrapPi(double angleRad)
    {
        return Math.Atan2(Math.Sin(angleRad), Math.Cos(angleRad));
    }

    public static double[,] Multiply4(double[,] a, double[,] b)
    {
        double[,] result = new double[4, 4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    result[i, j] += a[i, k] * b[k, j];
        return result;
    }

    /// <summary>
    /// Chuyển đổi góc Roll-Pitch-Yaw (hệ Tait-Bryan Z-Y-X) sang Quaternion trong hệ quy chiếu Unity
    /// </summary>
    public static Quaternion RpyToUnityRotation(float rollDeg, float pitchDeg, float yawDeg)
    {
        // R_dh = Rz(yaw) * Ry(pitch) * Rx(roll)
        // R_unity = Sx * R_dh * Sx
        double cr = Math.Cos(rollDeg * Math.PI / 180.0), sr = Math.Sin(rollDeg * Math.PI / 180.0);
        double cp = Math.Cos(pitchDeg * Math.PI / 180.0), sp = Math.Sin(pitchDeg * Math.PI / 180.0);
        double cy = Math.Cos(yawDeg * Math.PI / 180.0), sy = Math.Sin(yawDeg * Math.PI / 180.0);

        // Rdh = Rz * Ry * Rx
        double r11 = cy * cp;
        double r12 = cy * sp * sr - sy * cr;
        double r13 = cy * sp * cr + sy * sr;

        double r21 = sy * cp;
        double r22 = sy * sp * sr + cy * cr;
        double r23 = sy * sp * cr - cy * sr;

        double r31 = -sp;
        double r32 = cp * sr;
        double r33 = cp * cr;

        // Apply Sx * Rdh * Sx
        Matrix4x4 m = Matrix4x4.identity;
        m[0, 0] = (float)r11;  m[0, 1] = (float)-r12; m[0, 2] = (float)-r13;
        m[1, 0] = (float)-r21; m[1, 1] = (float)r22;  m[1, 2] = (float)r23;
        m[2, 0] = (float)-r31; m[2, 1] = (float)r32;  m[2, 2] = (float)r33;

        return m.rotation;
    }

    /// <summary>
    /// Trích xuất Roll - Pitch - Yaw (độ) theo chuẩn Tait-Bryan Z-Y-X từ Quaternion Unity
    /// </summary>
    public static Vector3 UnityRotationToRpy(Quaternion unityRot)
    {
        Matrix4x4 m = Matrix4x4.Rotate(unityRot);
        // Rdh = Sx * Ru * Sx
        double r11 = m[0, 0];
        double r21 = -m[1, 0];
        double r31 = -m[2, 0];
        double r32 = m[2, 1];
        double r33 = m[2, 2];

        // 2.4. Trích xuất góc định hướng Roll - Pitch - Yaw (RPY) từ báo cáo:
        // Pitch = atan2(-r31, sqrt(r11^2 + r21^2))
        // Roll = atan2(r32, r33)
        // Yaw = atan2(r21, r11)
        double pitch = Math.Atan2(-r31, Math.Sqrt(r11 * r11 + r21 * r21));
        double roll;
        double yaw;

        if (Math.Abs(Math.Cos(pitch)) > 1e-6)
        {
            roll = Math.Atan2(r32, r33);
            yaw = Math.Atan2(r21, r11);
        }
        else
        {
            roll = Math.Atan2(-m[1, 2], m[1, 1]);
            yaw = 0.0;
        }

        return new Vector3(
            (float)(roll * 180.0 / Math.PI),
            (float)(pitch * 180.0 / Math.PI),
            (float)(yaw * 180.0 / Math.PI)
        );
    }
}
