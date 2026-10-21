using UnityEngine;

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

    private void Awake()
    {
        InitializeOffsets();
    }

    private void Start()
    {
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
            InitializeOffsets();
        }
        ApplyRotations();
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

                // Xoay theo trục Z kết hợp offset ban đầu:
                // initialRotation * Quaternion.Euler(0, 0, angleDeg)
                axisTransforms[i].localRotation = initialRotations[i] * Quaternion.Euler(0f, 0f, angleDeg);
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
