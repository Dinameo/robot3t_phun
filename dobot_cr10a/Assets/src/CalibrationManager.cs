using System;
using System.IO;
using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;
using UnityEngine;

/// <summary>
/// QUẢN LÝ HIỆU CHỈNH VỊ TRÍ CAMERA & CHUỖI QUY ĐỔI TỌA ĐỘ CAMERA ➔ ROBOT (EYE-TO-HAND CALIBRATION)
/// Đáp ứng Mục 4 & Phản biện #4 trong file góp ý:
/// 1. Hiệu chỉnh trực tiếp vị trí lắp đặt Camera (Tx, Ty, Tz mm) và góc lệch 3 trục (Roll, Pitch, Yaw °).
/// 2. Đồng bộ trực tiếp với `FixedRigCamera` và `PLYDirectLoader` trong cảnh 3D Unity.
/// 3. Giải hiệu chuẩn 3D Rigid (Kabsch / SVD) từ danh sách ≥ 3 cặp điểm chạm mốc (P_cam ↔ P_robot) ngay trong Unity.
/// 4. Nạp / Xuất ma trận 4×4 trực chuẩn `camera_to_robot` khớp 100% schema `ai_src/step3_3d_trajectory/calibration.py`.
/// 5. Tính ngân sách sai số cả chuỗi (Cam–Bàn, Tay–Mắt, TCP, Biên vải, Robot) và kiểm chứng chạm mốc độc lập E_calib.
/// </summary>
[ExecuteAlways]
public class CalibrationManager : MonoBehaviour
{
    [Serializable]
    public class CalibrationPointPair
    {
        public string label = "Moc_1";
        public Vector3 cameraMm;
        public Vector3 robotMm;
        public float residualMm;
    }

    [Header("1. Vị trí & Góc đặt Camera trong hệ tọa độ Robot Global (mm & độ)")]
    [Tooltip("Tọa độ X của tâm quang học Camera trong hệ Robot (mặc định: 1100 - 335 = 765 mm)")]
    public float camPosRobotX_mm = 765.0f;
    [Tooltip("Tọa độ Y của tâm quang học Camera trong hệ Robot (lệch ngang trái/phải, mặc định: 0 mm)")]
    public float camPosRobotY_mm = 0.0f;
    [Tooltip("Tọa độ Z của tâm quang học Camera trong hệ Robot (chiều cao cách mặt đất/bàn, mặc định: 1685 mm)")]
    public float camPosRobotZ_mm = 1685.0f;

    [Tooltip("Góc tinh chỉnh nghiêng quanh trục X robot (Roll, độ)")]
    public float camRotRollDeg = 0.0f;
    [Tooltip("Góc tinh chỉnh nghiêng quanh trục Y robot (Pitch, độ)")]
    public float camRotPitchDeg = 0.0f;
    [Tooltip("Góc xoay ngang của Camera quanh trục Z robot (Yaw, độ)")]
    public float camRotYawDeg = 0.0f;

    [Header("2. File hiệu chuẩn Eye-to-Hand (JSON chuẩn calibration.py)")]
    public string calibrationJsonPath = "ai_src/step3_3d_trajectory/camera_to_robot_calibration.json";
    public bool useCalibratedTransform = true;
    public bool isCalibrationValid = true;
    public bool isFromVerifiedJson = false;
    public string calibrationStatus = "Đang dùng thông số Camera Rig có thể hiệu chỉnh trực tiếp.";

    [Header("3. Thông số chuỗi hiệu chuẩn & Ngân sách sai số (Slide 10)")]
    public string handEyeMethod = "Tsai-Lenz / Kabsch-SVD (Eye-to-Hand)";
    public int calibrationPoseCount = 15;
    public float cameraToTableRmsMm = 0.45f;
    public float handEyeTransErrorMm = 0.85f;
    public float handEyeRotErrorDeg = 0.25f;
    public float tcpCalibrationErrorMm = 0.35f;
    public float garmentEdgeErrorMm = 0.80f;
    public float robotRepeatabilityMm = 0.03f; // Dobot CR10A ±0.03 mm

    [Header("4. Phương trình mặt bàn trong hệ Robot (Z_table)")]
    public Vector3 tableNormalRobot = new Vector3(0f, 0f, 1f);
    public float tableHeightMm = 0.0f;
    public float robotGlobalOriginZ_mm = 1100.0f; // Vị trí Z gốc Robot trong Unity World (SampleScene Global.z = 1.1m)

    [Header("5. Kiểm chứng chạm mốc & Hiệu chuẩn đa điểm (Touch-Point & Multi-Point SVD)")]
    public Vector3 verifyCameraPointMm = new Vector3(0f, 0f, 1650f);
    public Vector3 verifyRobotTouchedMm = new Vector3(765f, 0f, 35f);
    public float lastTouchErrorMm = 0f;
    public string touchVerificationSummary = "Nhập điểm Camera & TCP chạm để kiểm chứng.";

    public List<CalibrationPointPair> calibrationPairs = new List<CalibrationPointPair>();
    public string multiPointCalibStatus = "Cần >= 3 cặp điểm Cam - Robot để giải 4x4.";

    // Ma trận 4×4 Camera (mm) -> Robot Global (mm)
    // Mặc định (Top-down Zivid): X_r = 765 + Y_cam, Y_r = -X_cam, Z_r = 1685 - Z_cam
    private double[,] cameraToRobot4x4 = new double[4, 4]
    {
        {  0.0,  1.0,  0.0,  765.0 },
        { -1.0,  0.0,  0.0,    0.0 },
        {  0.0,  0.0, -1.0, 1685.0 },
        {  0.0,  0.0,  0.0,    1.0 }
    };

    [Serializable]
    private class CalibrationSetupSnapshot
    {
        public float camPosRobotX_mm = 765.0f;
        public float camPosRobotY_mm = 0.0f;
        public float camPosRobotZ_mm = 1685.0f;
        public float camRotRollDeg = 0.0f;
        public float camRotPitchDeg = 0.0f;
        public float camRotYawDeg = 0.0f;
        public float tableHeightMm = 0.0f;
        public float robotGlobalOriginZ_mm = 1100.0f;
        public bool useCalibratedTransform = true;
        public string calibrationJsonPath = "ai_src/step3_3d_trajectory/camera_to_robot_calibration.json";
        public string handEyeMethod = "Tsai-Lenz / Kabsch-SVD (Eye-to-Hand)";
        public int calibrationPoseCount = 15;
        public float cameraToTableRmsMm = 0.45f;
        public float handEyeTransErrorMm = 0.85f;
        public float handEyeRotErrorDeg = 0.25f;
        public float tcpCalibrationErrorMm = 0.35f;
        public float garmentEdgeErrorMm = 0.80f;
        public float robotRepeatabilityMm = 0.03f;
        public Vector3 verifyCameraPointMm = new Vector3(0f, 0f, 1650f);
        public Vector3 verifyRobotTouchedMm = new Vector3(765f, 0f, 35f);
        public List<CalibrationPointPair> calibrationPairs = new List<CalibrationPointPair>();
    }

    private bool hasLoadedStorage = false;

    private static string GetSetupFilePath()
    {
#if UNITY_EDITOR
        string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "UserSettings"));
#else
        string dir = Application.persistentDataPath;
#endif
        return Path.Combine(dir, "CR10A_CalibrationSetup.json");
    }

    public void EnsureLoadedFromStorage()
    {
        if (hasLoadedStorage) return;
        bool loadedUserPrefs = LoadPoseParamsFromPrefs();
        if (!loadedUserPrefs)
        {
            string resolved = ResolveCalibrationPath(calibrationJsonPath);
            if (File.Exists(resolved))
            {
                LoadCalibrationFromJson(resolved);
                hasLoadedStorage = true;
                return;
            }
        }
        RebuildMatrixFromPoseParams(syncScene: false, reloadPointCloud: false, saveToStorage: false);
        hasLoadedStorage = true;
    }

    private void Awake()
    {
        EnsureLoadedFromStorage();
    }

    private void OnEnable()
    {
        if (!hasLoadedStorage)
        {
            EnsureLoadedFromStorage();
        }
    }

    private void Start()
    {
        EnsureLoadedFromStorage();
        SyncToSceneCameraAndLoader(reloadPointCloud: false);
    }

    private void OnDisable()
    {
        if (Application.isPlaying && hasLoadedStorage)
        {
            SavePoseParamsToPrefs();
        }
    }

    private void OnDestroy()
    {
        if (Application.isPlaying && hasLoadedStorage)
        {
            SavePoseParamsToPrefs();
        }
    }

    private void OnApplicationQuit()
    {
        if (hasLoadedStorage)
        {
            SavePoseParamsToPrefs();
        }
    }

    public double[,] GetCameraToRobotMatrix()
    {
        return cameraToRobot4x4;
    }

    public void SavePoseParamsToPrefs()
    {
        PlayerPrefs.SetFloat("CR10A_Calib_Tx", camPosRobotX_mm);
        PlayerPrefs.SetFloat("CR10A_Calib_Ty", camPosRobotY_mm);
        PlayerPrefs.SetFloat("CR10A_Calib_Tz", camPosRobotZ_mm);
        PlayerPrefs.SetFloat("CR10A_Calib_Roll", camRotRollDeg);
        PlayerPrefs.SetFloat("CR10A_Calib_Pitch", camRotPitchDeg);
        PlayerPrefs.SetFloat("CR10A_Calib_Yaw", camRotYawDeg);
        PlayerPrefs.SetFloat("CR10A_Calib_TableZ", tableHeightMm);
        PlayerPrefs.SetFloat("CR10A_Calib_GlobalZ", robotGlobalOriginZ_mm);
        PlayerPrefs.SetInt("CR10A_Calib_UseMat", useCalibratedTransform ? 1 : 0);
        PlayerPrefs.Save();

        try
        {
            string filePath = GetSetupFilePath();
            string dir = Path.GetDirectoryName(filePath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) Directory.CreateDirectory(dir);

            var snap = new CalibrationSetupSnapshot
            {
                camPosRobotX_mm = camPosRobotX_mm,
                camPosRobotY_mm = camPosRobotY_mm,
                camPosRobotZ_mm = camPosRobotZ_mm,
                camRotRollDeg = camRotRollDeg,
                camRotPitchDeg = camRotPitchDeg,
                camRotYawDeg = camRotYawDeg,
                tableHeightMm = tableHeightMm,
                robotGlobalOriginZ_mm = robotGlobalOriginZ_mm,
                useCalibratedTransform = useCalibratedTransform,
                calibrationJsonPath = calibrationJsonPath,
                handEyeMethod = handEyeMethod,
                calibrationPoseCount = calibrationPoseCount,
                cameraToTableRmsMm = cameraToTableRmsMm,
                handEyeTransErrorMm = handEyeTransErrorMm,
                handEyeRotErrorDeg = handEyeRotErrorDeg,
                tcpCalibrationErrorMm = tcpCalibrationErrorMm,
                garmentEdgeErrorMm = garmentEdgeErrorMm,
                robotRepeatabilityMm = robotRepeatabilityMm,
                verifyCameraPointMm = verifyCameraPointMm,
                verifyRobotTouchedMm = verifyRobotTouchedMm,
                calibrationPairs = calibrationPairs != null ? new List<CalibrationPointPair>(calibrationPairs) : new List<CalibrationPointPair>()
            };
            File.WriteAllText(filePath, JsonUtility.ToJson(snap, true));
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[CalibrationManager] Không thể ghi file cấu hình hiệu chuẩn: {ex.Message}");
        }
    }

    public bool LoadPoseParamsFromPrefs()
    {
        try
        {
            string filePath = GetSetupFilePath();
            if (File.Exists(filePath))
            {
                string raw = File.ReadAllText(filePath);
                var snap = JsonUtility.FromJson<CalibrationSetupSnapshot>(raw);
                if (snap != null)
                {
                    camPosRobotX_mm = snap.camPosRobotX_mm;
                    camPosRobotY_mm = snap.camPosRobotY_mm;
                    camPosRobotZ_mm = snap.camPosRobotZ_mm;
                    camRotRollDeg = snap.camRotRollDeg;
                    camRotPitchDeg = snap.camRotPitchDeg;
                    camRotYawDeg = snap.camRotYawDeg;
                    tableHeightMm = snap.tableHeightMm;
                    robotGlobalOriginZ_mm = snap.robotGlobalOriginZ_mm;
                    useCalibratedTransform = snap.useCalibratedTransform;
                    if (!string.IsNullOrEmpty(snap.calibrationJsonPath)) calibrationJsonPath = snap.calibrationJsonPath;
                    if (!string.IsNullOrEmpty(snap.handEyeMethod)) handEyeMethod = snap.handEyeMethod;
                    calibrationPoseCount = snap.calibrationPoseCount;
                    cameraToTableRmsMm = snap.cameraToTableRmsMm;
                    handEyeTransErrorMm = snap.handEyeTransErrorMm;
                    handEyeRotErrorDeg = snap.handEyeRotErrorDeg;
                    tcpCalibrationErrorMm = snap.tcpCalibrationErrorMm;
                    garmentEdgeErrorMm = snap.garmentEdgeErrorMm;
                    robotRepeatabilityMm = snap.robotRepeatabilityMm;
                    verifyCameraPointMm = snap.verifyCameraPointMm;
                    verifyRobotTouchedMm = snap.verifyRobotTouchedMm;
                    if (snap.calibrationPairs != null)
                        calibrationPairs = new List<CalibrationPointPair>(snap.calibrationPairs);
                    return true;
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[CalibrationManager] Không thể đọc file cấu hình hiệu chuẩn: {ex.Message}");
        }

        if (!PlayerPrefs.HasKey("CR10A_Calib_Tz")) return false;
        camPosRobotX_mm = PlayerPrefs.GetFloat("CR10A_Calib_Tx", 765.0f);
        camPosRobotY_mm = PlayerPrefs.GetFloat("CR10A_Calib_Ty", 0.0f);
        camPosRobotZ_mm = PlayerPrefs.GetFloat("CR10A_Calib_Tz", 1685.0f);
        camRotRollDeg = PlayerPrefs.GetFloat("CR10A_Calib_Roll", 0.0f);
        camRotPitchDeg = PlayerPrefs.GetFloat("CR10A_Calib_Pitch", 0.0f);
        camRotYawDeg = PlayerPrefs.GetFloat("CR10A_Calib_Yaw", 0.0f);
        tableHeightMm = PlayerPrefs.GetFloat("CR10A_Calib_TableZ", 0.0f);
        robotGlobalOriginZ_mm = PlayerPrefs.GetFloat("CR10A_Calib_GlobalZ", 1100.0f);
        useCalibratedTransform = PlayerPrefs.GetInt("CR10A_Calib_UseMat", 1) == 1;
        return true;
    }

    public void ResetToNominalRig(bool reloadCloud = true)
    {
        camPosRobotX_mm = 765.0f;
        camPosRobotY_mm = 0.0f;
        camPosRobotZ_mm = 1685.0f;
        camRotRollDeg = 0.0f;
        camRotPitchDeg = 0.0f;
        camRotYawDeg = 0.0f;
        tableHeightMm = 0.0f;
        robotGlobalOriginZ_mm = 1100.0f;
        isFromVerifiedJson = false;
        RebuildMatrixFromPoseParams(syncScene: true, reloadPointCloud: reloadCloud);
        calibrationStatus = "Đã khôi phục thông số Camera Rig chuẩn mặc định (Tx=765, Ty=0, Tz=1685 mm).";
    }

    /// <summary>
    /// Xây dựng ma trận trực chuẩn 4×4 `T_camera_to_robot` (det(R) = +1) từ:
    /// - Ma trận định hướng gốc Top-Down Zivid -> Robot: R_base = [[0, 1, 0], [-1, 0, 0], [0, 0, -1]]
    /// - Ma trận xoay tinh chỉnh R_delta = Rz(yaw) * Ry(pitch) * Rx(roll)
    /// - Vector tịnh tiến t = [camPosRobotX_mm, camPosRobotY_mm, camPosRobotZ_mm]
    /// </summary>
    public void RebuildMatrixFromPoseParams(bool syncScene = true, bool reloadPointCloud = false, bool saveToStorage = true)
    {
        double cr = Math.Cos(camRotRollDeg * Math.PI / 180.0), sr = Math.Sin(camRotRollDeg * Math.PI / 180.0);
        double cp = Math.Cos(camRotPitchDeg * Math.PI / 180.0), sp = Math.Sin(camRotPitchDeg * Math.PI / 180.0);
        double cy = Math.Cos(camRotYawDeg * Math.PI / 180.0), sy = Math.Sin(camRotYawDeg * Math.PI / 180.0);

        // R_delta = Rz(yaw) * Ry(pitch) * Rx(roll)
        double[,] rDelta = new double[3, 3]
        {
            { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
            { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
            {    -sp,                 cp * sr,                cp * cr }
        };

        // R_base (Zivid top-down -> Robot Global): X_r = +Y_c, Y_r = -X_c, Z_r = -Z_c (det = +1)
        double[,] rBase = new double[3, 3]
        {
            {  0.0,  1.0,  0.0 },
            { -1.0,  0.0,  0.0 },
            {  0.0,  0.0, -1.0 }
        };

        double[,] R = Multiply3x3(rDelta, rBase);
        cameraToRobot4x4 = new double[4, 4]
        {
            { R[0, 0], R[0, 1], R[0, 2], camPosRobotX_mm },
            { R[1, 0], R[1, 1], R[1, 2], camPosRobotY_mm },
            { R[2, 0], R[2, 1], R[2, 2], camPosRobotZ_mm },
            {     0.0,     0.0,     0.0,             1.0 }
        };

        isCalibrationValid = true;
        if (saveToStorage)
        {
            SavePoseParamsToPrefs();
        }

        if (syncScene)
        {
            SyncToSceneCameraAndLoader(reloadPointCloud);
        }
    }

    /// <summary>
    /// Trích xuất ngược (Tx, Ty, Tz, Roll, Pitch, Yaw) từ ma trận 4×4 khi nạp file JSON hoặc giải SVD
    /// </summary>
    public void ExtractPoseParamsFromMatrix()
    {
        camPosRobotX_mm = (float)cameraToRobot4x4[0, 3];
        camPosRobotY_mm = (float)cameraToRobot4x4[1, 3];
        camPosRobotZ_mm = (float)cameraToRobot4x4[2, 3];

        // R_delta = R * R_base^T
        double[,] rBaseT = new double[3, 3]
        {
            { 0.0, -1.0,  0.0 },
            { 1.0,  0.0,  0.0 },
            { 0.0,  0.0, -1.0 }
        };
        double[,] R = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R[i, j] = cameraToRobot4x4[i, j];

        double[,] rDelta = Multiply3x3(R, rBaseT);
        double pitch = Math.Atan2(-rDelta[2, 0], Math.Sqrt(rDelta[0, 0] * rDelta[0, 0] + rDelta[1, 0] * rDelta[1, 0]));
        double roll, yaw;
        if (Math.Abs(Math.Cos(pitch)) > 1e-5)
        {
            roll = Math.Atan2(rDelta[2, 1], rDelta[2, 2]);
            yaw = Math.Atan2(rDelta[1, 0], rDelta[0, 0]);
        }
        else
        {
            roll = Math.Atan2(-rDelta[1, 2], rDelta[1, 1]);
            yaw = 0.0;
        }
        camRotRollDeg = (float)(roll * 180.0 / Math.PI);
        camRotPitchDeg = (float)(pitch * 180.0 / Math.PI);
        camRotYawDeg = (float)(yaw * 180.0 / Math.PI);
    }

    /// <summary>
    /// Đồng bộ trực tiếp vị trí & góc Camera sang `FixedRigCamera` và `PLYDirectLoader` trong cảnh 3D Unity!
    /// </summary>
    public void SyncToSceneCameraAndLoader(bool reloadPointCloud = false)
    {
        // Quy đổi vị trí Camera trong Robot Global (mm) sang Unity World (m):
        // X_world = -Y_robot / 1000, Y_world = Z_robot / 1000, Z_world = (GlobalZ - X_robot) / 1000
        Vector3 camWorldPos = new Vector3(
            -camPosRobotY_mm * 0.001f,
             camPosRobotZ_mm * 0.001f,
            (robotGlobalOriginZ_mm - camPosRobotX_mm) * 0.001f
        );

        FixedRigCamera rigCam = FindAnyObjectByType<FixedRigCamera>();
        if (rigCam != null)
        {
            rigCam.ApplyCalibratedWorldPose(camWorldPos, camRotPitchDeg, camRotYawDeg, camRotRollDeg);
        }

        PLYDirectLoader plyLoader = FindAnyObjectByType<PLYDirectLoader>();
        if (plyLoader != null)
        {
            plyLoader.cameraOrigin = camWorldPos;
            if (reloadPointCloud)
            {
                plyLoader.ReloadFullPointCloud();
            }
        }
    }

    public float GetTotalErrorBudgetRssMm()
    {
        return Mathf.Sqrt(
            cameraToTableRmsMm * cameraToTableRmsMm +
            handEyeTransErrorMm * handEyeTransErrorMm +
            tcpCalibrationErrorMm * tcpCalibrationErrorMm +
            garmentEdgeErrorMm * garmentEdgeErrorMm +
            robotRepeatabilityMm * robotRepeatabilityMm
        );
    }

    public float GetTotalErrorBudgetWorstCaseMm()
    {
        return cameraToTableRmsMm + handEyeTransErrorMm + tcpCalibrationErrorMm + garmentEdgeErrorMm + robotRepeatabilityMm;
    }

    public string ResolveCalibrationPath(string path)
    {
        if (string.IsNullOrEmpty(path)) return "";
        if (Path.IsPathRooted(path) && File.Exists(path)) return path;

        string p1 = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", path));
        if (File.Exists(p1)) return p1;

        string p2 = Path.GetFullPath(Path.Combine(Application.dataPath, "..", path));
        if (File.Exists(p2)) return p2;

        return p1;
    }

    public bool LoadCalibrationFromJson(string fullPath)
    {
        try
        {
            if (!File.Exists(fullPath))
            {
                calibrationStatus = $"Không tìm thấy file hiệu chuẩn: {fullPath}";
                return false;
            }

            string text = File.ReadAllText(fullPath);
            bool calibratedFlag = Regex.IsMatch(text, "\"calibrated\"\\s*:\\s*true", RegexOptions.IgnoreCase);
            bool unitsMm = Regex.IsMatch(text, "\"units\"\\s*:\\s*\"mm\"", RegexOptions.IgnoreCase);
            if (!calibratedFlag || !unitsMm)
            {
                calibrationStatus = "Lỗi JSON: cần calibrated=true và units=\"mm\".";
                return false;
            }

            Match matMatch = Regex.Match(text, "\"camera_to_robot\"\\s*:\\s*\\[(.*?)\\]\\s*(,|})", RegexOptions.Singleline);
            if (!matMatch.Success)
            {
                calibrationStatus = "Thiếu trường camera_to_robot (4x4) trong JSON.";
                return false;
            }

            MatchCollection nums = Regex.Matches(matMatch.Groups[1].Value, "[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?");
            if (nums.Count < 16)
            {
                calibrationStatus = $"Ma trận camera_to_robot thiếu phần tử ({nums.Count}/16).";
                return false;
            }

            double[,] m = new double[4, 4];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    m[r, c] = double.Parse(nums[r * 4 + c].Value, CultureInfo.InvariantCulture);

            if (!ValidateRigidTransform4x4(m, out string err))
            {
                calibrationStatus = "Lỗi ma trận: " + err;
                return false;
            }

            cameraToRobot4x4 = m;
            isCalibrationValid = true;
            isFromVerifiedJson = true;
            useCalibratedTransform = true;
            ExtractPoseParamsFromMatrix();

            TryParseJsonFloat(text, "hand_eye_trans_error_mm", ref handEyeTransErrorMm);
            TryParseJsonFloat(text, "hand_eye_rot_error_deg", ref handEyeRotErrorDeg);
            TryParseJsonFloat(text, "camera_to_table_rms_mm", ref cameraToTableRmsMm);
            TryParseJsonFloat(text, "tcp_error_mm", ref tcpCalibrationErrorMm);
            TryParseJsonInt(text, "pose_count", ref calibrationPoseCount);

            SavePoseParamsToPrefs();
            SyncToSceneCameraAndLoader(reloadPointCloud: true);
            calibrationStatus = $"Đã nạp {Path.GetFileName(fullPath)} (RSS={GetTotalErrorBudgetRssMm():F2} mm)";
            return true;
        }
        catch (Exception ex)
        {
            calibrationStatus = $"Lỗi nạp JSON: {ex.Message}";
            return false;
        }
    }

    public static bool ValidateRigidTransform4x4(double[,] m, out string error)
    {
        error = "";
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                if (double.IsNaN(m[r, c]) || double.IsInfinity(m[r, c]))
                { error = "Ma trận chứa NaN/Infinity."; return false; }

        if (Math.Abs(m[3, 0]) > 1e-5 || Math.Abs(m[3, 1]) > 1e-5 || Math.Abs(m[3, 2]) > 1e-5 || Math.Abs(m[3, 3] - 1.0) > 1e-5)
        { error = "Hàng cuối ma trận thuần nhất phải là [0, 0, 0, 1]."; return false; }

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                double dot = m[0, i] * m[0, j] + m[1, i] * m[1, j] + m[2, i] * m[2, j];
                if (Math.Abs(dot - (i == j ? 1.0 : 0.0)) > 1e-3)
                { error = "Ma trận xoay R không trực chuẩn (R^T·R != I)."; return false; }
            }

        double det =
            m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) -
            m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) +
            m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
        if (Math.Abs(det - 1.0) > 1e-3)
        { error = $"det(R) = {det:F4} != +1 (không phải phép xoay chuẩn)."; return false; }

        return true;
    }

    public string ExportCalibrationTemplateJson()
    {
        string outPath = ResolveCalibrationPath(calibrationJsonPath);
        string dir = Path.GetDirectoryName(outPath);
        if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) Directory.CreateDirectory(dir);

        string json = string.Format(CultureInfo.InvariantCulture,
@"{{
  ""calibrated"": true,
  ""units"": ""mm"",
  ""method"": ""{0}"",
  ""pose_count"": {1},
  ""camera_to_table_rms_mm"": {2:F3},
  ""hand_eye_trans_error_mm"": {3:F3},
  ""hand_eye_rot_error_deg"": {4:F3},
  ""tcp_error_mm"": {5:F3},
  ""camera_to_robot"": [
    [ {6:F6}, {7:F6}, {8:F6}, {9:F3} ],
    [ {10:F6}, {11:F6}, {12:F6}, {13:F3} ],
    [ {14:F6}, {15:F6}, {16:F6}, {17:F3} ],
    [ 0.0, 0.0, 0.0, 1.0 ]
  ]
}}",
            handEyeMethod, calibrationPoseCount,
            cameraToTableRmsMm, handEyeTransErrorMm, handEyeRotErrorDeg, tcpCalibrationErrorMm,
            cameraToRobot4x4[0, 0], cameraToRobot4x4[0, 1], cameraToRobot4x4[0, 2], cameraToRobot4x4[0, 3],
            cameraToRobot4x4[1, 0], cameraToRobot4x4[1, 1], cameraToRobot4x4[1, 2], cameraToRobot4x4[1, 3],
            cameraToRobot4x4[2, 0], cameraToRobot4x4[2, 1], cameraToRobot4x4[2, 2], cameraToRobot4x4[2, 3]);

        File.WriteAllText(outPath, json);
        isFromVerifiedJson = true;
        SavePoseParamsToPrefs();
        calibrationStatus = $"Đã lưu ma trận 4x4: {Path.GetFileName(outPath)}";
        return outPath;
    }

    public Vector3 CameraMmToRobotMm(Vector3 camMm)
    {
        double x = cameraToRobot4x4[0, 0] * camMm.x + cameraToRobot4x4[0, 1] * camMm.y + cameraToRobot4x4[0, 2] * camMm.z + cameraToRobot4x4[0, 3];
        double y = cameraToRobot4x4[1, 0] * camMm.x + cameraToRobot4x4[1, 1] * camMm.y + cameraToRobot4x4[1, 2] * camMm.z + cameraToRobot4x4[1, 3];
        double z = cameraToRobot4x4[2, 0] * camMm.x + cameraToRobot4x4[2, 1] * camMm.y + cameraToRobot4x4[2, 2] * camMm.z + cameraToRobot4x4[2, 3];
        return new Vector3((float)x, (float)y, (float)z);
    }

    public Vector3 CameraMmToUnityWorld(Vector3 camMm, Transform robotGlobalRef)
    {
        Vector3 robotMm = CameraMmToRobotMm(camMm);
        if (robotGlobalRef != null)
        {
            return robotGlobalRef.TransformPoint(robotMm * 0.001f);
        }
        return new Vector3(-robotMm.y * 0.001f, robotMm.z * 0.001f, (robotGlobalOriginZ_mm - robotMm.x) * 0.001f);
    }

    public float RunTouchPointVerification()
    {
        Vector3 predictedRobotMm = CameraMmToRobotMm(verifyCameraPointMm);
        Vector3 diff = predictedRobotMm - verifyRobotTouchedMm;
        lastTouchErrorMm = diff.magnitude;
        string passStr = lastTouchErrorMm <= 1.5f ? "Đạt" : (lastTouchErrorMm <= 3.0f ? "Chấp nhận" : "Vượt ngưỡng");
        touchVerificationSummary =
            $"Lệch d=({diff.x:F1}, {diff.y:F1}, {diff.z:F1}) mm | E_calib = {lastTouchErrorMm:F2} mm [{passStr}]";
        return lastTouchErrorMm;
    }

    /// <summary>
    /// Bù nhanh độ lệch tịnh tiến (ΔX, ΔY, ΔZ) từ 1 điểm chạm mốc thực tế
    /// </summary>
    public void AutoCompensateTranslationFromTouchPoint()
    {
        Vector3 predicted = CameraMmToRobotMm(verifyCameraPointMm);
        Vector3 delta = verifyRobotTouchedMm - predicted;
        camPosRobotX_mm += delta.x;
        camPosRobotY_mm += delta.y;
        camPosRobotZ_mm += delta.z;
        RebuildMatrixFromPoseParams(syncScene: true, reloadPointCloud: true);
        RunTouchPointVerification();
        calibrationStatus = $"Đã bù tịnh tiến ({delta.x:F1}, {delta.y:F1}, {delta.z:F1}) mm";
    }

    /// <summary>
    /// Thêm cặp điểm mẫu hiện tại từ ô kiểm chứng vào danh sách hiệu chuẩn đa điểm
    /// </summary>
    public void AddCurrentVerifyPairToList()
    {
        calibrationPairs.Add(new CalibrationPointPair
        {
            label = $"Moc_{calibrationPairs.Count + 1}",
            cameraMm = verifyCameraPointMm,
            robotMm = verifyRobotTouchedMm
        });
        SavePoseParamsToPrefs();
        multiPointCalibStatus = $"Đã thêm Moc_{calibrationPairs.Count} (Tổng {calibrationPairs.Count} cặp).";
    }

    /// <summary>
    /// Giải bài toán Hiệu chuẩn Cứng 3D (Kabsch / Polar Decomposition bằng Quaternion Horn)
    /// tìm ma trận 4×4 trực chuẩn (det(R) = +1) tối thiểu hóa ∑ ||R·p_cam + t - p_rob||² từ ≥ 3 cặp điểm chạm mốc!
    /// </summary>
    public bool SolveRigidCalibrationFromPairs()
    {
        if (calibrationPairs == null || calibrationPairs.Count < 3)
        {
            multiPointCalibStatus = "Cần tối thiểu 3 cặp điểm không thẳng hàng.";
            return false;
        }

        int n = calibrationPairs.Count;
        Vector3 meanC = Vector3.zero, meanR = Vector3.zero;
        for (int i = 0; i < n; i++)
        {
            meanC += calibrationPairs[i].cameraMm;
            meanR += calibrationPairs[i].robotMm;
        }
        meanC /= n;
        meanR /= n;

        // Ma trận hiệp phương sai chéo H = sum((c_i - meanC) * (r_i - meanR)^T)
        double[,] H = new double[3, 3];
        for (int k = 0; k < n; k++)
        {
            Vector3 c = calibrationPairs[k].cameraMm - meanC;
            Vector3 r = calibrationPairs[k].robotMm - meanR;
            H[0, 0] += c.x * r.x; H[0, 1] += c.x * r.y; H[0, 2] += c.x * r.z;
            H[1, 0] += c.y * r.x; H[1, 1] += c.y * r.y; H[1, 2] += c.y * r.z;
            H[2, 0] += c.z * r.x; H[2, 1] += c.z * r.y; H[2, 2] += c.z * r.z;
        }

        // Phương pháp Horn (1987): xây ma trận đối xứng 4×4 N, vector riêng ứng trị riêng lớn nhất là Quaternion xoay tối ưu q*!
        double sxx = H[0, 0], sxy = H[0, 1], sxz = H[0, 2];
        double syx = H[1, 0], syy = H[1, 1], syz = H[1, 2];
        double szx = H[2, 0], szy = H[2, 1], szz = H[2, 2];

        double[,] N = new double[4, 4]
        {
            { sxx + syy + szz, syz - szy,        szx - sxz,        sxy - syx        },
            { syz - szy,       sxx - syy - szz,  sxy + syx,        szx + sxz        },
            { szx - sxz,       sxy + syx,       -sxx + syy - szz,  syz + szy        },
            { sxy - syx,       szx + sxz,        syz + szy,       -sxx - syy + szz  }
        };

        // Tìm vector riêng ứng với trị riêng lớn nhất bằng Jacobi Eigenvalue (hội tụ chính xác 1e-12 cho ma trận đối xứng 4×4)
        double[] qBest = LargestEigenvectorSymmetric4x4(N);
        double qw = qBest[0], qx = qBest[1], qy = qBest[2], qz = qBest[3];

        double[,] R = new double[3, 3]
        {
            { 1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)     },
            { 2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)     },
            { 2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy) }
        };

        // t = meanR - R * meanC
        double tx = meanR.x - (R[0, 0] * meanC.x + R[0, 1] * meanC.y + R[0, 2] * meanC.z);
        double ty = meanR.y - (R[1, 0] * meanC.x + R[1, 1] * meanC.y + R[1, 2] * meanC.z);
        double tz = meanR.z - (R[2, 0] * meanC.x + R[2, 1] * meanC.y + R[2, 2] * meanC.z);

        double[,] M = new double[4, 4]
        {
            { R[0, 0], R[0, 1], R[0, 2], tx },
            { R[1, 0], R[1, 1], R[1, 2], ty },
            { R[2, 0], R[2, 1], R[2, 2], tz },
            { 0.0,     0.0,     0.0,     1.0 }
        };

        if (!ValidateRigidTransform4x4(M, out string err))
        {
            multiPointCalibStatus = "Lỗi phép xoay: " + err;
            return false;
        }

        cameraToRobot4x4 = M;
        isCalibrationValid = true;
        useCalibratedTransform = true;
        ExtractPoseParamsFromMatrix();

        // Tính sai số dư RMS (mm) trên toàn bộ các cặp điểm
        double sumSq = 0.0;
        float maxRes = 0f;
        for (int i = 0; i < n; i++)
        {
            Vector3 pred = CameraMmToRobotMm(calibrationPairs[i].cameraMm);
            float res = Vector3.Distance(pred, calibrationPairs[i].robotMm);
            calibrationPairs[i].residualMm = res;
            sumSq += res * res;
            if (res > maxRes) maxRes = res;
        }
        float rmsMm = (float)Math.Sqrt(sumSq / n);
        handEyeTransErrorMm = rmsMm;
        calibrationPoseCount = n;

        SavePoseParamsToPrefs();
        SyncToSceneCameraAndLoader(reloadPointCloud: true);
        multiPointCalibStatus = $"Đã giải 4x4 ({n} mốc): RMS={rmsMm:F2} mm | Max={maxRes:F2} mm";
        calibrationStatus = multiPointCalibStatus;
        return true;
    }

    private static double[] LargestEigenvectorSymmetric4x4(double[,] A)
    {
        double[,] a = (double[,])A.Clone();
        double[,] v = new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };

        for (int iter = 0; iter < 60; iter++)
        {
            int p = 0, q = 1;
            double maxOff = Math.Abs(a[0, 1]);
            for (int i = 0; i < 4; i++)
            {
                for (int j = i + 1; j < 4; j++)
                {
                    double val = Math.Abs(a[i, j]);
                    if (val > maxOff) { maxOff = val; p = i; q = j; }
                }
            }
            if (maxOff < 1e-12) break;

            double phi = 0.5 * Math.Atan2(2.0 * a[p, q], a[q, q] - a[p, p]);
            double c = Math.Cos(phi), s = Math.Sin(phi);

            double app = c * c * a[p, p] - 2 * s * c * a[p, q] + s * s * a[q, q];
            double aqq = s * s * a[p, p] + 2 * s * c * a[p, q] + c * c * a[q, q];
            a[p, p] = app; a[q, q] = aqq; a[p, q] = a[q, p] = 0.0;

            for (int k = 0; k < 4; k++)
            {
                if (k == p || k == q) continue;
                double akp = c * a[k, p] - s * a[k, q];
                double akq = s * a[k, p] + c * a[k, q];
                a[k, p] = a[p, k] = akp;
                a[k, q] = a[q, k] = akq;
            }
            for (int k = 0; k < 4; k++)
            {
                double vkp = c * v[k, p] - s * v[k, q];
                double vkq = s * v[k, p] + c * v[k, q];
                v[k, p] = vkp; v[k, q] = vkq;
            }
        }

        int bestCol = 0;
        double bestEig = a[0, 0];
        for (int i = 1; i < 4; i++)
        {
            if (a[i, i] > bestEig) { bestEig = a[i, i]; bestCol = i; }
        }

        double[] vec = { v[0, bestCol], v[1, bestCol], v[2, bestCol], v[3, bestCol] };
        double norm = Math.Sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2] + vec[3] * vec[3]);
        if (norm > 1e-9) for (int i = 0; i < 4; i++) vec[i] /= norm;
        return vec;
    }

    private static double[,] Multiply3x3(double[,] a, double[,] b)
    {
        double[,] res = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    res[i, j] += a[i, k] * b[k, j];
        return res;
    }

    private static void TryParseJsonFloat(string json, string key, ref float target)
    {
        Match m = Regex.Match(json, $"\"{key}\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+)");
        if (m.Success && float.TryParse(m.Groups[1].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out float v))
            target = v;
    }

    private static void TryParseJsonInt(string json, string key, ref int target)
    {
        Match m = Regex.Match(json, $"\"{key}\"\\s*:\\s*([-+]?[0-9]+)");
        if (m.Success && int.TryParse(m.Groups[1].Value, NumberStyles.Integer, CultureInfo.InvariantCulture, out int v))
            target = v;
    }
}
