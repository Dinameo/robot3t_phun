using UnityEngine;
using System;
using System.Globalization;
using System.Collections.Generic;

[ExecuteAlways]
public class manual : MonoBehaviour
{
    [Header("Joint Sliders: Axis 0 -> 5 (Range: -2π -> 2π rad)")]
    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay trục Z của Axis 0 (radian)")]
    public float axis0 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay trục Z của Axis 1 (radian)")]
    public float axis1 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay trục Z của Axis 2 (radian)")]
    public float axis2 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay trục Z của Axis 3 (radian)")]
    public float axis3 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay trục Z của Axis 4 (radian)")]
    public float axis4 = 0f;

    [Range(-2f * Mathf.PI, 2f * Mathf.PI)]
    [Tooltip("Góc xoay trục Z của Axis 5 (radian)")]
    public float axis5 = 0f;

    [Header("Axis Transforms (Khớp 0 -> 5)")]
    [Tooltip("Kéo thả 6 khớp vào đây (axis0 -> axis5 hoặc Link1 -> Link6)")]
    public Transform[] axisTransforms = new Transform[6];

    [Header("Offset ban đầu (Đã lưu để về 0)")]
    [SerializeField]
    private Quaternion[] initialRotations = new Quaternion[6];

    [SerializeField]
    private bool isInitialized = false;

    private readonly float[] targetPose = new float[6]; // Robot Global XYZ (mm), ZYX RPY (deg)
    private readonly string[] targetText = new string[6];
    private bool targetInitialized;
    private string ikStatus = "Nhập pose đích để giải IK.";
    private float[] lastIkSolution = new float[6];
    private bool hasIkSolution;
    private readonly List<DobotCR10AKinematics.IKSolution> ikSolutions = new List<DobotCR10AKinematics.IKSolution>();
    private Vector2 solutionScroll;
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

    /// <summary>
    /// Giữ nguyên offset đã lưu trong Scene. Chỉ chụp từ Transform khi chưa có offset.
    /// </summary>
    private void EnsureOffsetsInitialized()
    {
        AutoAssignTransforms();
        if (isInitialized && initialRotations != null && initialRotations.Length == 6)
            return;

        InitializeOffsets();
    }

    /// <summary>
    /// Ghi nhớ rotation ban đầu của từng khớp để khi slider = 0 thì góc quay về đúng vị trí ban đầu.
    /// </summary>
    [ContextMenu("Ghi nhớ góc ban đầu (Capture Zero Offset)")]
    public void InitializeOffsets()
    {
        if (axisTransforms == null || axisTransforms.Length != 6)
        {
            axisTransforms = new Transform[6];
        }

        // Tự động tìm kiếm trong hierarchy nếu chưa gán
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
    /// Tự động tìm kiếm các Transform theo tên axis0..5 hoặc Link1..6 nếu chưa kéo thả
    /// </summary>
    private void AutoAssignTransforms()
    {
        for (int i = 0; i < 6; i++)
        {
            if (axisTransforms[i] == null)
            {
                // Thử tìm trong con cháu hoặc theo tên
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
        if (parent.name.Equals(childName, System.StringComparison.OrdinalIgnoreCase)) return parent;

        foreach (Transform child in parent)
        {
            Transform result = FindDeepChild(child, childName);
            if (result != null) return result;
        }
        return null;
    }

    private void Update()
    {
        ApplyRotations();
    }

    /// <summary>
    /// Cập nhật ngay lập tức khi kéo slider trong Inspector (kể cả lúc chưa bấm Play)
    /// </summary>
    private void OnValidate()
    {
        if (!isInitialized)
        {
            EnsureOffsetsInitialized();
        }
        ApplyRotations();
    }

    // Runtime FK/IK panel in the Unity Game view.
    private void OnGUI()
    {
        if (axisTransforms == null || axisTransforms.Length != 6) return;
        if (!targetInitialized) InitializeTargetFromCurrentPose();
        ResolveKinematicsReferences();

        const float width = 410f;
        float height = Mathf.Min(Screen.height - 24f, 700f);
        Rect panel = new Rect(12f, 12f, width, height);
        GUI.Box(panel, GUIContent.none);
        GUILayout.BeginArea(new Rect(panel.x + 10f, panel.y + 8f, width - 20f, height - 16f));
        GUILayout.BeginVertical();
        GUILayout.Label("DOBOT CR10A — FK / IK", PanelTitleStyle());
        GUILayout.Label("Joint q1…q6 (degrees, −360° to 360°)");

        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"q{i + 1}: {GetJoint(i) * Mathf.Rad2Deg,7:F1}°", GUILayout.Width(108f));
            float value = GUILayout.HorizontalSlider(GetJoint(i), -2f * Mathf.PI, 2f * Mathf.PI);
            if (Mathf.Abs(value - GetJoint(i)) > 1e-5f)
            {
                SetJoint(i, value);
                ApplyRotations();
            }
            GUILayout.EndHorizontal();
        }

        Vector3 position;
        Quaternion rotation;
        GetForwardPose(GetJointVector(), out position, out rotation, null, null);
        Vector3 rpy = RotationToRpy(rotation);
        GUILayout.Space(4f);
        GUILayout.Label("FK hiện tại — Global robot (mm / độ)", PanelHeaderStyle());
        GUILayout.Label($"X {position.x * 1000f:F2}   Y {position.y * 1000f:F2}   Z {position.z * 1000f:F2} mm");
        GUILayout.Label($"Roll {rpy.x:F2}   Pitch {rpy.y:F2}   Yaw {rpy.z:F2}°");

        GUILayout.Label("Vị trí TCP (GameObject tcp) — Global robot (mm)", PanelHeaderStyle());
        Vector3 axis6GlobalPosition;
        if (TryGetAxis6PositionGlobal(out axis6GlobalPosition))
        {
            GUILayout.Label($"X {axis6GlobalPosition.x:F2}   Y {axis6GlobalPosition.y:F2}   Z {axis6GlobalPosition.z:F2} mm");
            Transform baseFrame = axisTransforms[0] != null ? axisTransforms[0].parent : null;
            if (baseFrame != null && globalReference != null)
            {
                Vector3 predictedGlobal = globalReference.InverseTransformPoint(baseFrame.TransformPoint(position));
                // predictedGlobal is in metres; the displayed axis6 value is in millimetres.
                float errorMm = Vector3.Distance(predictedGlobal * 1000f, axis6GlobalPosition);
                GUILayout.Label($"Sai số FK ↔ TCP: {errorMm:F3} mm");
            }
            else
            {
                GUILayout.Label("Không xác định được base frame để so sánh FK.");
            }
        }
        else
        {
            GUILayout.Label("Chưa tìm thấy Global hoặc GameObject tcp.");
        }

        GUILayout.Space(6f);
        GUILayout.Label("Pose đích — XYZ mm, RPY ZYX độ", PanelHeaderStyle());
        DrawPoseRow(0, 1, 2);
        DrawPoseRow(3, 4, 5);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Lấy pose FK hiện tại", GUILayout.Height(26f)))
            CopyCurrentPoseToTarget();
        if (GUILayout.Button("Tìm nghiệm IK", GUILayout.Height(26f)))
            SolveAndApplyIk();
        GUILayout.EndHorizontal();
        if (GUILayout.Button("Mặt bích hướng xuống + chọn nghiệm gần nhất", GUILayout.Height(26f)))
            SetFlangeDownTargetAndApplyIk();
        GUILayout.Label(ikStatus, PanelStatusStyle());

        if (hasIkSolution)
        {
            GUILayout.Label($"Nghiệm IK tìm được: {ikSolutions.Count}", PanelHeaderStyle());
            GUILayout.Label(string.Format(CultureInfo.InvariantCulture,
                "q1 {0:F2}   q2 {1:F2}   q3 {2:F2}",
                lastIkSolution[0] * Mathf.Rad2Deg, lastIkSolution[1] * Mathf.Rad2Deg,
                lastIkSolution[2] * Mathf.Rad2Deg));
            GUILayout.Label(string.Format(CultureInfo.InvariantCulture,
                "q4 {0:F2}   q5 {1:F2}   q6 {2:F2}",
                lastIkSolution[3] * Mathf.Rad2Deg, lastIkSolution[4] * Mathf.Rad2Deg,
                lastIkSolution[5] * Mathf.Rad2Deg));
            solutionScroll = GUILayout.BeginScrollView(solutionScroll, GUILayout.Height(Mathf.Min(150f, 26f * ikSolutions.Count)));
            for (int i = 0; i < ikSolutions.Count; i++)
            {
                double[] q = ikSolutions[i].jointsRad;
                string label = string.Format(CultureInfo.InvariantCulture,
                    "Áp dụng #{0}: {1:F1}, {2:F1}, {3:F1}, {4:F1}, {5:F1}, {6:F1}°",
                    i + 1, q[0] * Mathf.Rad2Deg, q[1] * Mathf.Rad2Deg, q[2] * Mathf.Rad2Deg,
                    q[3] * Mathf.Rad2Deg, q[4] * Mathf.Rad2Deg, q[5] * Mathf.Rad2Deg);
                if (GUILayout.Button(label, GUILayout.Height(23f))) ApplySolution(i);
            }
            GUILayout.EndScrollView();
        }
        GUILayout.EndVertical();
        GUILayout.EndArea();
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

        if (tcpTransform == null && axisTransforms != null && axisTransforms.Length >= 6)
            tcpTransform = FindDeepChild(axisTransforms[5], "tcp");
    }

    private bool TryGetAxis6PositionGlobal(out Vector3 position)
    {
        ResolveKinematicsReferences();
        if (globalReference == null || tcpTransform == null)
        {
            position = Vector3.zero;
            return false;
        }

        // TransformPoint returns metres in Unity; convert to millimetres for display.
        position = globalReference.InverseTransformPoint(tcpTransform.position) * 1000f;
        return true;
    }

    private GUIStyle PanelTitleStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontSize = 16,
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.cyan }
        };
    }

    private GUIStyle PanelHeaderStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.yellow }
        };
    }

    private GUIStyle PanelStatusStyle()
    {
        return new GUIStyle(GUI.skin.label)
        {
            wordWrap = true,
            normal = { textColor = ikStatus.StartsWith("IK OK") ? Color.green : Color.white }
        };
    }

    private void DrawPoseRow(int a, int b, int c)
    {
        string[] labels = { "X", "Y", "Z", "R", "P", "Y" };
        int[] indices = { a, b, c };
        GUILayout.BeginHorizontal();
        for (int k = 0; k < 3; k++)
        {
            int i = indices[k];
            GUILayout.Label(labels[i], GUILayout.Width(18f));
            targetText[i] = GUILayout.TextField(targetText[i] ?? "0", GUILayout.Width(95f));
            float parsed;
            if (float.TryParse(targetText[i], NumberStyles.Float, CultureInfo.CurrentCulture, out parsed) ||
                float.TryParse(targetText[i], NumberStyles.Float, CultureInfo.InvariantCulture, out parsed))
                targetPose[i] = parsed;
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
        Vector3 p;
        Quaternion r;
        GetForwardPose(GetJointVector(), out p, out r, null, null);
        Vector3 rpy = RotationToRpy(r);
        float[] current = { p.x * 1000f, p.y * 1000f, p.z * 1000f, rpy.x, rpy.y, rpy.z };
        for (int i = 0; i < 6; i++)
        {
            targetPose[i] = current[i];
            targetText[i] = current[i].ToString("F2", CultureInfo.InvariantCulture);
        }
        ikStatus = "Đã lấy pose đích từ FK hiện tại.";
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
        return new[] { axis0, axis1, axis2, axis3, axis4, axis5 };
    }

    /// <summary>FK from the ml_sim DH model, returned in Unity Global coordinates and metres.</summary>
    private void GetForwardPose(float[] q, out Vector3 position, out Quaternion rotation,
        Vector3[] origins, Vector3[] axes)
    {
        if (tcpTransform == null) ResolveKinematicsReferences();
        Matrix4x4 T = DobotCR10AKinematics.ForwardTcp(q, tcpTransform);
        position = new Vector3(T[0, 3], T[1, 3], T[2, 3]) * 0.001f;
        rotation = T.rotation;
    }

    private Quaternion RpyToRotation(float rollDeg, float pitchDeg, float yawDeg)
    {
        return Quaternion.AngleAxis(yawDeg, Vector3.forward)
             * Quaternion.AngleAxis(pitchDeg, Vector3.up)
             * Quaternion.AngleAxis(rollDeg, Vector3.right);
    }

    private Vector3 RotationToRpy(Quaternion rotation)
    {
        Matrix4x4 R = Matrix4x4.Rotate(rotation);
        float pitch = Mathf.Asin(Mathf.Clamp(-R[2, 0], -1f, 1f));
        float roll;
        float yaw;
        if (Mathf.Abs(Mathf.Cos(pitch)) > 1e-6f)
        {
            roll = Mathf.Atan2(R[2, 1], R[2, 2]);
            yaw = Mathf.Atan2(R[1, 0], R[0, 0]);
        }
        else
        {
            roll = Mathf.Atan2(-R[1, 2], R[1, 1]);
            yaw = 0f;
        }
        return new Vector3(roll, pitch, yaw) * Mathf.Rad2Deg;
    }

    private void SolveAndApplyIk()
    {
        ResolveKinematicsReferences();
        if (axisTransforms == null || axisTransforms.Length != 6 || tcpTransform == null)
        {
            ikStatus = "IK lỗi: cần gán đủ 6 khớp và GameObject tcp.";
            return;
        }
        for (int i = 0; i < 6; i++)
        {
            if (axisTransforms[i] == null)
            {
                ikStatus = $"IK lỗi: thiếu Transform khớp {i + 1}.";
                return;
            }
        }

        Quaternion targetR = RpyToRotation(targetPose[3], targetPose[4], targetPose[5]);
        ikSolutions.Clear();
        ikSolutions.AddRange(DobotCR10AKinematics.SolveTcp(
            new Vector3(targetPose[0], targetPose[1], targetPose[2]), targetR,
            tcpTransform.localPosition, tcpTransform.localRotation));
        if (ikSolutions.Count == 0)
        {
            hasIkSolution = false;
            ikStatus = "IK không tìm thấy nghiệm hội tụ cho pose này.";
            return;
        }

        hasIkSolution = true;
        for (int i = 0; i < 6; i++) lastIkSolution[i] = (float)ikSolutions[0].jointsRad[i];
        float bestPositionError = float.MaxValue;
        float bestOrientationError = float.MaxValue;
        for (int i = 0; i < ikSolutions.Count; i++)
        {
            if (ikSolutions[i].positionErrorMm < bestPositionError)
            {
                bestPositionError = (float)ikSolutions[i].positionErrorMm;
                bestOrientationError = (float)ikSolutions[i].orientationErrorRad;
            }
        }
        ikStatus = $"Tạo {ikSolutions.Count} nhánh theo phương pháp ma trận. Sai số tốt nhất: {bestPositionError:F2} mm, {bestOrientationError * Mathf.Rad2Deg:F2}°. Chọn nghiệm để áp dụng.";
    }

    private void SetFlangeDownTargetAndApplyIk()
    {
        ResolveKinematicsReferences();
        if (tcpTransform == null)
        {
            ikStatus = "IK lỗi: không tìm thấy GameObject tcp dưới axis6.";
            return;
        }

        // Keep the current flange yaw, set flange roll/pitch so its +Z axis
        // points along robot-frame -Z (down), then restore the TCP's local pose.
        Quaternion targetTcpRotation = RpyToRotation(targetPose[3], targetPose[4], targetPose[5]);
        Quaternion targetFlangeRotation = targetTcpRotation * Quaternion.Inverse(tcpTransform.localRotation);
        float flangeYaw = RotationToRpy(targetFlangeRotation).z;
        Quaternion downwardFlange = RpyToRotation(180f, 0f, flangeYaw);
        Quaternion downwardTcp = downwardFlange * tcpTransform.localRotation;
        Vector3 tcpRpy = RotationToRpy(downwardTcp);
        targetPose[3] = tcpRpy.x;
        targetPose[4] = tcpRpy.y;
        targetPose[5] = tcpRpy.z;
        for (int i = 3; i < 6; i++)
            targetText[i] = targetPose[i].ToString("F3", CultureInfo.InvariantCulture);

        SolveAndApplyIk();
        if (!hasIkSolution || ikSolutions.Count == 0) return;

        // All branches now have the same requested flange orientation. Among
        // numerically equivalent solutions, prefer the one requiring the
        // smallest wrapped joint motion from the robot's current pose.
        double bestPosition = double.MaxValue;
        double bestOrientation = double.MaxValue;
        for (int i = 0; i < ikSolutions.Count; i++)
        {
            bestPosition = Math.Min(bestPosition, ikSolutions[i].positionErrorMm);
            bestOrientation = Math.Min(bestOrientation, ikSolutions[i].orientationErrorRad);
        }
        int preferredIndex = -1;
        double preferredTravel = double.MaxValue;
        for (int i = 0; i < ikSolutions.Count; i++)
        {
            DobotCR10AKinematics.IKSolution candidate = ikSolutions[i];
            if (!IsFlangeZPointingDown(candidate.jointsRad, 1f))
                continue;
            if (candidate.positionErrorMm > bestPosition + 0.05 ||
                candidate.orientationErrorRad > bestOrientation + 0.1 * Mathf.Deg2Rad)
                continue;
            double travel = 0.0;
            for (int joint = 0; joint < 6; joint++)
            {
                double delta = Math.Atan2(Math.Sin(candidate.jointsRad[joint] - GetJoint(joint)),
                    Math.Cos(candidate.jointsRad[joint] - GetJoint(joint)));
                travel += delta * delta;
            }
            if (travel < preferredTravel)
            {
                preferredTravel = travel;
                preferredIndex = i;
            }
        }
        if (preferredIndex < 0)
        {
            ikStatus = "Không có nghiệm IK đạt điều kiện trục Z mặt bích hướng xuống trong sai số 1°. Robot giữ nguyên tư thế.";
            return;
        }
        if (preferredIndex > 0)
        {
            DobotCR10AKinematics.IKSolution preferred = ikSolutions[preferredIndex];
            ikSolutions.RemoveAt(preferredIndex);
            ikSolutions.Insert(0, preferred);
        }
        ApplySolution(0);
        ikStatus = $"Đã đặt mặt bích hướng xuống (roll 180°, pitch 0°, giữ yaw hiện tại) và áp dụng nhánh gần nhất. Sai số: {ikSolutions[0].positionErrorMm:F2} mm, {ikSolutions[0].orientationErrorRad * Mathf.Rad2Deg:F2}°. ";
    }

    private bool IsFlangeZPointingDown(double[] jointsRad, float toleranceDeg)
    {
        float[] q = new float[6];
        for (int i = 0; i < 6; i++) q[i] = (float)jointsRad[i];
        Matrix4x4 tcpPose = DobotCR10AKinematics.ForwardTcp(q, tcpTransform);
        Quaternion flangeRotation = tcpPose.rotation * Quaternion.Inverse(tcpTransform.localRotation);
        Vector3 flangeZ = flangeRotation * Vector3.forward;
        return Vector3.Dot(flangeZ.normalized, Vector3.back) >= Mathf.Cos(toleranceDeg * Mathf.Deg2Rad);
    }

    private void ApplySolution(int solutionIndex)
    {
        if (solutionIndex < 0 || solutionIndex >= ikSolutions.Count) return;
        double[] q = ikSolutions[solutionIndex].jointsRad;
        for (int i = 0; i < 6; i++) SetJoint(i, (float)q[i]);
        for (int i = 0; i < 6; i++) lastIkSolution[i] = (float)q[i];
        ApplyRotations();
        ikStatus = $"Đã áp dụng nhánh #{solutionIndex + 1}; sai số FK vị trí {ikSolutions[solutionIndex].positionErrorMm:F2} mm, hướng {ikSolutions[solutionIndex].orientationErrorRad * Mathf.Rad2Deg:F2}°. ";
    }

    [Header("Cấu hình chiều xoay")]
    [Tooltip("Đảo ngược chiều xoay để chuẩn theo quy tắc bàn tay phải (ROS/Robot chuẩn) thay vì quy tắc bàn tay trái của Unity")]
    public bool invertDirection = true;

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
                // Chuyển đổi từ radian sang độ và áp dụng chiều xoay ngược lại
                float angleDeg = dirMultiplier * sliders[i] * Mathf.Rad2Deg;
                // Rotate around local Z, preserving the original bind rotation.
                axisTransforms[i].localRotation = initialRotations[i]
                    * Quaternion.Euler(0f, 0f, angleDeg);
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
    }
}
