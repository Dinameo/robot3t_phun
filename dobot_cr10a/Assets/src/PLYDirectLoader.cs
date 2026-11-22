using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Component tải và hiển thị TOÀN BỘ ĐIỂM ẢNH (100% Full Point Cloud) từ file PLY vào Unity.
/// - Giữ nguyên 100% màu sắc gốc (Original RGB).
/// - Tự động chia các Chunks để render mượt mà hàng triệu điểm (MeshTopology.Points).
/// - Khớp đúng với hệ quy chiếu Camera Fixed Rig (Y = 1.685m, Z = 0.335m).
/// </summary>
[ExecuteAlways]
public class PLYDirectLoader : MonoBehaviour
{
    [Header("1. Đường dẫn file PLY")]
    [Tooltip("Đường dẫn file PLY đầy đủ")]
    public string plyFilePath = "Assets/PointCloud/full_cloud_world.ply";

    [Header("2. Thư mục chứa dataset PLY gốc")]
    public string datasetDirectory = @"dataset\_3D";

    [Header("3. Chuyển đổi tọa độ vào Camera Fixed Rig")]
    [Tooltip("Tự động dịch chuyển theo hệ quy chiếu Camera")]
    public bool alignWithCameraRig = true;
    public Vector3 cameraOrigin = new Vector3(0f, 1.685f, 0.335f);

    [Header("4. Tùy chọn điểm (Đã tối ưu hóa hiệu năng)")]
    [Tooltip("Tải 100% toàn bộ điểm ảnh (Cảnh báo: Hàng triệu điểm có thể gây lag GPU/CPU)")]
    public bool loadAll100PercentPoints = false;

    [Tooltip("Bước lấy mẫu tối ưu hóa (2 = 50% điểm, 3 = 33% điểm, mượt mà và nhẹ nhàng)")]
    [Range(1, 4)]
    public int downsampleStep = 2;

    [Tooltip("Giới hạn số điểm tối đa mỗi Chunk Mesh")]
    public int maxPointsPerChunk = 500000;

    [Tooltip("Material hiển thị màu Vertex")]
    public Material pointCloudMaterial;

    [Header("5. Trạng thái & Chế độ hiển thị phôi")]
    public int totalLoadedPoints = 0;
    public bool isPointCloudVisible = true;

    public enum WorkpieceRenderMode
    {
        Mesh3D,
        PointCloud
    }

    [Tooltip("Chế độ hiển thị: 3D Mesh siêu nhẹ (mặc định) hoặc Point Cloud 1.7M điểm thô")]
    public WorkpieceRenderMode renderMode = WorkpieceRenderMode.Mesh3D;
    private GameObject surfaceMeshObject;
    private MeshCollider surfaceMeshCollider;

    private List<GameObject> chunkObjects = new List<GameObject>();
    private bool hasLoadedSettings = false;

    public void SaveSettings()
    {
        PlayerPrefs.SetString("CR10A_PLY_FilePath", plyFilePath ?? "");
        PlayerPrefs.SetString("CR10A_PLY_SampleName", currentLoadedSampleName ?? "");
        PlayerPrefs.SetString("CR10A_PLY_DatasetDir", datasetDirectory ?? "");
        PlayerPrefs.SetInt("CR10A_PLY_RenderMode", (int)renderMode);
        PlayerPrefs.SetInt("CR10A_PLY_Visible", isPointCloudVisible ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_PLY_AlignRig", alignWithCameraRig ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_PLY_Load100Pct", loadAll100PercentPoints ? 1 : 0);
        PlayerPrefs.SetInt("CR10A_PLY_Downsample", downsampleStep);
        PlayerPrefs.Save();
    }

    public void LoadSettings()
    {
        if (PlayerPrefs.HasKey("CR10A_PLY_RenderMode"))
        {
            string savedPath = PlayerPrefs.GetString("CR10A_PLY_FilePath", "");
            if (!string.IsNullOrEmpty(savedPath) && File.Exists(ResolvePath(savedPath)))
            {
                plyFilePath = savedPath;
            }
            string savedSample = PlayerPrefs.GetString("CR10A_PLY_SampleName", "");
            if (!string.IsNullOrEmpty(savedSample))
            {
                currentLoadedSampleName = savedSample;
            }
            string savedDatasetDir = PlayerPrefs.GetString("CR10A_PLY_DatasetDir", "");
            if (!string.IsNullOrEmpty(savedDatasetDir))
            {
                datasetDirectory = savedDatasetDir;
            }
            renderMode = (WorkpieceRenderMode)PlayerPrefs.GetInt("CR10A_PLY_RenderMode", (int)renderMode);
            isPointCloudVisible = PlayerPrefs.GetInt("CR10A_PLY_Visible", isPointCloudVisible ? 1 : 0) == 1;
            alignWithCameraRig = PlayerPrefs.GetInt("CR10A_PLY_AlignRig", alignWithCameraRig ? 1 : 0) == 1;
            loadAll100PercentPoints = PlayerPrefs.GetInt("CR10A_PLY_Load100Pct", loadAll100PercentPoints ? 1 : 0) == 1;
            downsampleStep = Mathf.Clamp(PlayerPrefs.GetInt("CR10A_PLY_Downsample", downsampleStep), 1, 4);
        }
        hasLoadedSettings = true;
    }

    private void DisableLegacyRootMeshOnSelf()
    {
        // Vô hiệu hóa MeshFilter/MeshRenderer tĩnh gắn trực tiếp trên root Garment_PLY_Model trong SampleScene.unity
        // (m_Mesh: full_cloud_world), tránh hiện đè mẫu phôi cũ nằm dưới phôi mới!
        MeshRenderer rootMr = GetComponent<MeshRenderer>();
        if (rootMr != null) rootMr.enabled = false;
        MeshFilter rootMf = GetComponent<MeshFilter>();
        if (rootMf != null) rootMf.sharedMesh = null;
        MeshCollider rootMc = GetComponent<MeshCollider>();
        if (rootMc != null)
        {
            rootMc.enabled = false;
            rootMc.sharedMesh = null;
        }
    }

    private void Awake()
    {
        DisableLegacyRootMeshOnSelf();
        LoadSettings();
    }

    private void OnDisable()
    {
        if (Application.isPlaying && hasLoadedSettings) SaveSettings();
    }

    private void OnDestroy()
    {
        if (Application.isPlaying && hasLoadedSettings) SaveSettings();
        ReleaseCameraRgbTexture();
    }

    private void OnApplicationQuit()
    {
        if (hasLoadedSettings) SaveSettings();
    }

    public void SetVisible(bool visible)
    {
        isPointCloudVisible = visible;
        ApplyRenderMode();
        if (hasLoadedSettings) SaveSettings();
    }

    public void SetPointCloudChunksVisible(bool visible)
    {
        for (int i = 0; i < transform.childCount; i++)
        {
            Transform child = transform.GetChild(i);
            if (child.name.StartsWith("CloudChunk_"))
            {
                var mr = child.GetComponent<MeshRenderer>();
                if (mr != null) mr.enabled = visible;
            }
        }
    }

    public void SetRenderMode(WorkpieceRenderMode mode)
    {
        renderMode = mode;
        if (renderMode == WorkpieceRenderMode.Mesh3D)
        {
            if (surfaceMeshObject == null)
            {
                ConvertPointCloudToSurfaceMesh();
            }
        }
        ApplyRenderMode();
        if (hasLoadedSettings) SaveSettings();
    }

    public void ApplyRenderMode()
    {
        DisableLegacyRootMeshOnSelf();
        if (surfaceMeshObject == null)
        {
            Transform found = transform.Find("WorkpieceSurfaceMesh");
            if (found != null)
            {
                surfaceMeshObject = found.gameObject;
                surfaceMeshCollider = surfaceMeshObject.GetComponent<MeshCollider>();
            }
        }

        if (renderMode == WorkpieceRenderMode.Mesh3D)
        {
            if (surfaceMeshObject != null) surfaceMeshObject.SetActive(isPointCloudVisible);
            SetPointCloudChunksVisible(false);
        }
        else
        {
            if (surfaceMeshObject != null) surfaceMeshObject.SetActive(false);
            SetPointCloudChunksVisible(isPointCloudVisible);
        }
    }

    public bool IsVisible()
    {
        if (renderMode == WorkpieceRenderMode.Mesh3D && surfaceMeshObject != null)
        {
            return surfaceMeshObject.activeSelf;
        }
        for (int i = 0; i < transform.childCount; i++)
        {
            Transform child = transform.GetChild(i);
            if (child.name.StartsWith("CloudChunk_"))
            {
                var mr = child.GetComponent<MeshRenderer>();
                if (mr != null) return mr.enabled;
            }
        }
        return isPointCloudVisible;
    }

    public int GetChunkCount()
    {
        return chunkObjects.Count;
    }

    public bool autoLoadOnStart = false; // Mặc định khi mở app không nạp sẵn phôi nào

    private void Start()
    {
        LoadSettings();
        CalibrationManager calibMgr = FindAnyObjectByType<CalibrationManager>();
        if (calibMgr != null)
        {
            calibMgr.EnsureLoadedFromStorage();
            calibMgr.SyncToSceneCameraAndLoader(reloadPointCloud: false);
        }
        EnsureMaterial();
        ClearExistingChunks();

        if (autoLoadOnStart)
        {
            string resolvedPath = ResolvePath(plyFilePath);
            if (File.Exists(resolvedPath))
            {
                LoadFullPointCloud(resolvedPath);
            }
        }
        else
        {
            plyFilePath = "";
            currentLoadedSampleName = "";
            hasOrganizedCameraFrame = false;
            hasTopDownCameraTexture = false;
            ReleaseCameraRgbTexture();
        }
    }

    private void EnsureMaterial()
    {
        if (pointCloudMaterial == null)
        {
            Shader shader = Shader.Find("Custom/PointCloudVertexColor") 
                         ?? Shader.Find("Universal Render Pipeline/Unlit") 
                         ?? Shader.Find("Sprites/Default") 
                         ?? Shader.Find("Standard");
            if (shader != null)
            {
                pointCloudMaterial = new Material(shader);
                pointCloudMaterial.name = "PointCloud_VertexColor_Mat";
            }
        }
    }

    [Header("6. Mẫu phôi hiện hành (Active Sample)")]
    public string currentLoadedSampleName = "pcl_101";

    [Header("7. Ảnh RGB thực tế từ Camera (2D -> 3D)")]
    [NonSerialized] public Texture2D cameraRgbTexture;
    public int cameraImageWidth = 0;
    public int cameraImageHeight = 0;
    public bool hasOrganizedCameraFrame = false;
    public bool hasTopDownCameraTexture = false;
    public string loadedCameraSampleName = "";

    private int camGridW = 0;
    private int camGridH = 0;
    private Vector3[] camGridWorldPos;
    private bool[] camGridWorldValid;
    private float topDownCropMinX = -0.75f;
    private float topDownCropMaxX = 0.75f;
    private float topDownCropMinZ = -0.35f;
    private float topDownCropMaxZ = 0.95f;
    private readonly Vector3[] actualCaptureFootprintWorld = new Vector3[4];
    private bool hasActualCaptureFootprint = false;

    [Serializable]
    private class FrameMetaJson
    {
        public int width;
        public int height;
    }

    private void ReleaseCameraRgbTexture()
    {
        if (cameraRgbTexture != null)
        {
            if (Application.isPlaying) Destroy(cameraRgbTexture);
            else DestroyImmediate(cameraRgbTexture);
            cameraRgbTexture = null;
        }
    }

    public string ResolvePath(string path)
    {
        if (string.IsNullOrEmpty(path)) return "";
        if (Path.IsPathRooted(path) && File.Exists(path)) return path;

        // 1. Thử trong thư mục gốc dự án cha (RB3t_PJ/path)
        try
        {
            string pParent = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", path));
            if (File.Exists(pParent)) return pParent;
        }
        catch { }

        // 2. Thử trong thư mục dobot_cr10a/path
        try
        {
            string p1 = Path.GetFullPath(Path.Combine(Application.dataPath, "..", path));
            if (File.Exists(p1)) return p1;
        }
        catch { }

        // 3. Thử trong Assets/
        try
        {
            string p2 = Path.GetFullPath(Path.Combine(Application.dataPath, path.Replace("Assets/", "")));
            if (File.Exists(p2)) return p2;
        }
        catch { }

        // 4. Thử tìm trực tiếp tên file trong dataset/_3D (bao gồm cả thư mục /home/nhan/RB3t_PJ/ai_src/dataset/_3D)
        try
        {
            string fileName = Path.GetFileName(path);
            if (!fileName.EndsWith(".ply", StringComparison.OrdinalIgnoreCase)) fileName += ".ply";
            string pDataset = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "dataset", "_3D", fileName));
            if (File.Exists(pDataset)) return pDataset;

            string pAiDataset = Path.Combine("/home/nhan/RB3t_PJ/ai_src/dataset/_3D", fileName);
            if (File.Exists(pAiDataset)) return pAiDataset;
        }
        catch { }

        return path;
    }

    /// <summary>
    /// Quét và lấy danh sách tất cả các mẫu PLY có trong dataset/_3D
    /// </summary>
    public List<string> GetAvailableSampleNames()
    {
        List<string> sampleList = new List<string>();
        string fullDirPath = datasetDirectory;
        if (!Path.IsPathRooted(fullDirPath))
        {
            fullDirPath = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", datasetDirectory));
            if (!Directory.Exists(fullDirPath))
            {
                fullDirPath = Path.GetFullPath(Path.Combine(Application.dataPath, "..", datasetDirectory));
            }
            if (!Directory.Exists(fullDirPath))
            {
                fullDirPath = "/home/nhan/RB3t_PJ/ai_src/dataset/_3D";
            }
        }

        if (Directory.Exists(fullDirPath))
        {
            string[] files = Directory.GetFiles(fullDirPath, "*.ply");
            foreach (string f in files)
            {
                sampleList.Add(Path.GetFileNameWithoutExtension(f));
            }
            sampleList.Sort();
        }

        // Nếu không quét được thư mục thực tế, dùng danh sách mẫu chuẩn có sẵn
        if (sampleList.Count == 0)
        {
            sampleList.AddRange(new string[] {
                "pcl_18", "pcl_25", "pcl_30", "pcl_35", "pcl_48", "pcl_58", "pcl_59", "pcl_60",
                "pcl_61", "pcl_66", "pcl_79", "pcl_83", "pcl_84", "pcl_101", "pcl_111", "pcl_112",
                "pcl_151", "pcl_157", "pcl_178", "pcl_183", "pcl_184", "pcl_194", "pcl_196", "pcl_204",
                "pcl_206", "pcl_208", "pcl_209", "pcl_217", "pcl_241", "pcl_243", "pcl_256", "pcl_261",
                "pcl_263", "pcl_271", "pcl_282"
            });
        }

        return sampleList;
    }

    /// <summary>
    /// Nạp một mẫu phôi theo tên (ví dụ: "pcl_151")
    /// </summary>
    public bool LoadSampleByName(string sampleName)
    {
        if (string.IsNullOrEmpty(sampleName)) return false;
        string cleanName = Path.GetFileNameWithoutExtension(sampleName);
        string relativePath = $"dataset/_3D/{cleanName}.ply";
        string resolved = ResolvePath(relativePath);

        if (File.Exists(resolved))
        {
            plyFilePath = relativePath;
            currentLoadedSampleName = cleanName;
            LoadFullPointCloud(resolved);
            SaveSettings();
            return true;
        }
        else
        {
            Debug.LogWarning($"[PLY Loader] Không tìm thấy mẫu {cleanName} tại: {resolved}");
            return false;
        }
    }

    /// <summary>
    /// Nạp TOÀN BỘ 100% điểm ảnh từ file PLY
    /// </summary>
    [ContextMenu("Nạp toàn bộ điểm ảnh (Load 100% Points Now)")]
    public void ReloadFullPointCloud()
    {
        EnsureMaterial();
        string resolved = ResolvePath(plyFilePath);
        if (File.Exists(resolved))
        {
            LoadFullPointCloud(resolved);
            if (hasLoadedSettings) SaveSettings();
        }
        else
        {
            Debug.LogError($"[PLY Loader] Không tìm thấy file: {resolved}");
        }
    }

    /// <summary>
    /// Đổi ngẫu nhiên một file PLY khác từ dataset và nạp toàn bộ điểm ảnh
    /// </summary>
    [ContextMenu("Chọn ngẫu nhiên file PLY mới từ dataset")]
    public void LoadRandomPLY()
    {
        string fullDirPath = datasetDirectory;
        if (!Path.IsPathRooted(fullDirPath))
        {
            fullDirPath = Path.Combine(Application.dataPath, "..", "..", datasetDirectory);
            if (!Directory.Exists(fullDirPath))
            {
                fullDirPath = Path.Combine(Application.dataPath, "..", datasetDirectory);
            }
        }

        if (Directory.Exists(fullDirPath))
        {
            string[] files = Directory.GetFiles(fullDirPath, "*.ply");
            if (files.Length > 0)
            {
                int randIdx = UnityEngine.Random.Range(0, files.Length);
                string chosenFile = files[randIdx];
                Debug.Log($"[PLY Loader] Đang nạp toàn bộ điểm ảnh từ file: {Path.GetFileName(chosenFile)}");

                // Nạp trực tiếp file được chọn
                LoadFullPointCloud(chosenFile);
            }
        }
    }

    public void LoadFullPointCloud(string filePath)
    {
        EnsureMaterial();
        ClearExistingChunks();

        if (!File.Exists(filePath))
        {
            Debug.LogError($"[PLY Loader] File không tồn tại: {filePath}");
            return;
        }

        int step = loadAll100PercentPoints ? 1 : downsampleStep;

        try
        {
            using (FileStream fs = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read, 65536))
            using (BinaryReader br = new BinaryReader(fs))
            {
                int vertexCount = 0;
                int headerWidth = 0;
                int headerHeight = 0;
                bool isBinary = false;

                // Đọc Header PLY
                while (true)
                {
                    string line = ReadLineASCII(fs);
                    if (string.IsNullOrEmpty(line)) break;
                    line = line.Trim();

                    if (line.StartsWith("format binary_little_endian")) isBinary = true;
                    else if (line.StartsWith("comment width "))
                    {
                        string[] parts = line.Split(' ');
                        if (parts.Length >= 3) int.TryParse(parts[2], out headerWidth);
                    }
                    else if (line.StartsWith("comment height "))
                    {
                        string[] parts = line.Split(' ');
                        if (parts.Length >= 3) int.TryParse(parts[2], out headerHeight);
                    }
                    else if (line.StartsWith("element vertex"))
                    {
                        string[] parts = line.Split(' ');
                        vertexCount = int.Parse(parts[parts.Length - 1]);
                    }
                    else if (line == "end_header") break;
                }

                if (!isBinary || vertexCount == 0)
                {
                    Debug.LogWarning("[PLY Loader] File không hợp lệ.");
                    return;
                }

                if (headerWidth <= 0 || headerHeight <= 0)
                {
                    string frameMetaPath = Path.ChangeExtension(filePath, ".frame.json");
                    if (File.Exists(frameMetaPath))
                    {
                        try
                        {
                            var meta = JsonUtility.FromJson<FrameMetaJson>(File.ReadAllText(frameMetaPath));
                            if (meta != null && meta.width > 0 && meta.height > 0)
                            {
                                headerWidth = meta.width;
                                headerHeight = meta.height;
                            }
                        }
                        catch { }
                    }
                }
                if ((headerWidth <= 0 || headerHeight <= 0) && vertexCount == 2332800)
                {
                    headerWidth = 1944;
                    headerHeight = 1200;
                }

                bool isPreconverted = filePath.Contains("full_cloud_world");
                bool isZividOrganized = !isPreconverted && headerWidth > 0 && headerHeight > 0 && (long)headerWidth * headerHeight == vertexCount;
                CalibrationManager calibMgr = FindAnyObjectByType<CalibrationManager>();
                bool useCalib = (calibMgr != null && calibMgr.useCalibratedTransform && calibMgr.isCalibrationValid);
                Transform robotGlobalRef = null;
                if (useCalib)
                {
                    var ctrl = FindAnyObjectByType<DobotCR10AIKController>();
                    if (ctrl != null) robotGlobalRef = ctrl.globalReference;
                    if (robotGlobalRef == null)
                    {
                        GameObject gObj = GameObject.Find("Global");
                        if (gObj != null) robotGlobalRef = gObj.transform;
                    }
                }

                hasOrganizedCameraFrame = false;
                hasTopDownCameraTexture = false;
                hasActualCaptureFootprint = false;
                camGridW = 0;
                camGridH = 0;
                camGridWorldPos = null;
                camGridWorldValid = null;

                List<Vector3> curVerts = new List<Vector3>(maxPointsPerChunk);
                List<Color32> curColors = new List<Color32>(maxPointsPerChunk);
                int totalLoaded = 0;
                int chunkIdx = 0;

                if (isZividOrganized)
                {
                    int h = headerHeight;
                    int w = headerWidth;
                    int texStep = Mathf.Max(1, Mathf.CeilToInt(Mathf.Max(w, h) / 648f));
                    int texW = (w + texStep - 1) / texStep;
                    int texH = (h + texStep - 1) / texStep;
                    Color32[] camPixels = new Color32[texW * texH];
                    Vector3[] gridPos = new Vector3[texW * texH];
                    bool[] gridValid = new bool[texW * texH];

                    for (int y = 0; y < h; y++)
                    {
                        bool sampleTexY = (y % texStep == 0);
                        int ty = y / texStep;
                        for (int x = 0; x < w; x++)
                        {
                            float vx = br.ReadSingle();
                            float vy = br.ReadSingle();
                            float vz = br.ReadSingle();
                            byte r = br.ReadByte();
                            byte g = br.ReadByte();
                            byte b = br.ReadByte();

                            bool validDepth = !float.IsNaN(vz) && vz > 400f && vz < 2500f;
                            bool sampleCloud = (y % step == 0 && x % step == 0);
                            bool sampleTex = sampleTexY && (x % texStep == 0);

                            Vector3 pos = Vector3.zero;
                            if (validDepth && (sampleCloud || sampleTex))
                            {
                                if (useCalib)
                                    pos = calibMgr.CameraMmToUnityWorld(new Vector3(vx, vy, vz), robotGlobalRef);
                                else if (alignWithCameraRig)
                                    pos = new Vector3(cameraOrigin.x + vx * 0.001f, cameraOrigin.y - vz * 0.001f, cameraOrigin.z - vy * 0.001f);
                                else
                                    pos = new Vector3(vx * 0.001f, vy * 0.001f, vz * 0.001f);
                            }

                            if (sampleTex)
                            {
                                int tx = x / texStep;
                                if (tx < texW && ty < texH)
                                {
                                    camPixels[(texH - 1 - ty) * texW + tx] = new Color32(r, g, b, 255);
                                    if (validDepth)
                                    {
                                        gridPos[ty * texW + tx] = transform.TransformPoint(pos);
                                        gridValid[ty * texW + tx] = true;
                                    }
                                }
                            }

                            if (sampleCloud && validDepth)
                            {
                                curVerts.Add(pos);
                                curColors.Add(new Color32(r, g, b, 255));
                                totalLoaded++;

                                if (curVerts.Count >= maxPointsPerChunk)
                                {
                                    CreateChunkObject(curVerts, curColors, chunkIdx++);
                                    curVerts.Clear();
                                    curColors.Clear();
                                }
                            }
                        }
                    }

                    ReleaseCameraRgbTexture();
                    cameraRgbTexture = new Texture2D(texW, texH, TextureFormat.RGBA32, false);
                    cameraRgbTexture.filterMode = FilterMode.Bilinear;
                    cameraRgbTexture.wrapMode = TextureWrapMode.Clamp;
                    cameraRgbTexture.SetPixels32(camPixels);
                    cameraRgbTexture.Apply();

                    camGridW = texW;
                    camGridH = texH;
                    camGridWorldPos = gridPos;
                    camGridWorldValid = gridValid;
                    cameraImageWidth = w;
                    cameraImageHeight = h;
                    hasOrganizedCameraFrame = true;
                }
                else
                {
                    // Đọc dạng vertex list thông thường
                    for (int i = 0; i < vertexCount; i++)
                    {
                        float vx = br.ReadSingle();
                        float vy = br.ReadSingle();
                        float vz = br.ReadSingle();
                        byte r = br.ReadByte();
                        byte g = br.ReadByte();
                        byte b = br.ReadByte();

                        if (i % step == 0 && !float.IsNaN(vz))
                        {
                            Vector3 pos;
                            if (isPreconverted)
                            {
                                pos = new Vector3(vx, vy, vz);
                            }
                            else if (useCalib)
                            {
                                pos = calibMgr.CameraMmToUnityWorld(new Vector3(vx, vy, vz), robotGlobalRef);
                            }
                            else if (alignWithCameraRig)
                            {
                                pos = new Vector3(cameraOrigin.x + vx * 0.001f, cameraOrigin.y - vz * 0.001f, cameraOrigin.z - vy * 0.001f);
                            }
                            else
                            {
                                pos = new Vector3(vx * 0.001f, vy * 0.001f, vz * 0.001f);
                            }

                            curVerts.Add(pos);
                            curColors.Add(new Color32(r, g, b, 255));
                            totalLoaded++;

                            if (curVerts.Count >= maxPointsPerChunk)
                            {
                                CreateChunkObject(curVerts, curColors, chunkIdx++);
                                curVerts.Clear();
                                curColors.Clear();
                            }
                        }
                    }
                }

                if (curVerts.Count > 0)
                {
                    CreateChunkObject(curVerts, curColors, chunkIdx++);
                }

                totalLoadedPoints = totalLoaded;
                isPointCloudVisible = true;
                string stemName = Path.GetFileNameWithoutExtension(filePath);
                currentLoadedSampleName = (stemName == "full_cloud_world") ? "pcl_25" : stemName;
                RebuildSurfaceElevationGrid();
                if (hasOrganizedCameraFrame)
                {
                    loadedCameraSampleName = currentLoadedSampleName;
                }
                else
                {
                    BuildTopDownCameraTextureFromElevationGrid();
                }
                RebuildActualCameraCaptureFootprint();
                ConvertPointCloudToSurfaceMesh();
                ApplyRenderMode();
                Debug.Log($"[PLY Loader] HOÀN TẤT: Đã tải {totalLoaded:N0} điểm ảnh 3D từ mẫu [{currentLoadedSampleName}] và tạo 3D Surface Mesh siêu nhẹ!");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[PLY Loader] Lỗi: {ex.Message}");
        }
    }

    public void EnsureCameraTextureReady()
    {
        if (cameraRgbTexture != null) return;
        if (rawElevationGridValid == null)
        {
            bool hasChunks = false;
            for (int i = 0; i < transform.childCount; i++)
            {
                if (transform.GetChild(i).name.StartsWith("CloudChunk_")) { hasChunks = true; break; }
            }
            if (hasChunks)
            {
                RebuildSurfaceElevationGrid();
            }
        }
        if (rawElevationGridValid != null)
        {
            BuildTopDownCameraTextureFromElevationGrid();
        }
    }

    public void BuildTopDownCameraTextureFromElevationGrid()
    {
        if (rawElevationGridValid == null || rawElevationGridColor == null) return;

        int minGx = GRID_RES, maxGx = -1, minGz = GRID_RES, maxGz = -1;
        for (int gx = 0; gx < GRID_RES; gx++)
        {
            for (int gz = 0; gz < GRID_RES; gz++)
            {
                if (!rawElevationGridValid[gx, gz]) continue;
                if (gx < minGx) minGx = gx;
                if (gx > maxGx) maxGx = gx;
                if (gz < minGz) minGz = gz;
                if (gz > maxGz) maxGz = gz;
            }
        }
        if (maxGx <= minGx || maxGz <= minGz) return;

        int pad = 4;
        minGx = Mathf.Max(0, minGx - pad);
        maxGx = Mathf.Min(GRID_RES - 1, maxGx + pad);
        minGz = Mathf.Max(0, minGz - pad);
        maxGz = Mathf.Min(GRID_RES - 1, maxGz + pad);

        int texW = maxGx - minGx + 1;
        int texH = maxGz - minGz + 1;
        Color32 bg = new Color32(18, 22, 30, 255);
        Color32[] pixels = new Color32[texW * texH];

        for (int py = 0; py < texH; py++)
        {
            int gz = minGz + py;
            for (int px = 0; px < texW; px++)
            {
                int gx = minGx + px;
                if (rawElevationGridValid[gx, gz])
                {
                    pixels[py * texW + px] = rawElevationGridColor[gx, gz];
                }
                else
                {
                    // Điền mượt các khe hở 1 ô nhỏ từ các ô lân cận để ảnh sắc nét như camera thật
                    int rSum = 0, gSum = 0, bSum = 0, cnt = 0;
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            int nx = gx + dx, nz = gz + dz;
                            if (nx >= 0 && nx < GRID_RES && nz >= 0 && nz < GRID_RES && rawElevationGridValid[nx, nz])
                            {
                                Color32 nc = rawElevationGridColor[nx, nz];
                                rSum += nc.r; gSum += nc.g; bSum += nc.b; cnt++;
                            }
                        }
                    }
                    pixels[py * texW + px] = (cnt >= 3)
                        ? new Color32((byte)(rSum / cnt), (byte)(gSum / cnt), (byte)(bSum / cnt), 255)
                        : bg;
                }
            }
        }

        ReleaseCameraRgbTexture();
        cameraRgbTexture = new Texture2D(texW, texH, TextureFormat.RGBA32, false);
        cameraRgbTexture.filterMode = FilterMode.Bilinear;
        cameraRgbTexture.wrapMode = TextureWrapMode.Clamp;
        cameraRgbTexture.SetPixels32(pixels);
        cameraRgbTexture.Apply();

        float cellW = (gridMax.x - gridMin.x) / (GRID_RES - 1);
        float cellD = (gridMax.z - gridMin.z) / (GRID_RES - 1);
        topDownCropMinX = gridMin.x + minGx * cellW;
        topDownCropMaxX = gridMin.x + maxGx * cellW;
        topDownCropMinZ = gridMin.z + minGz * cellD;
        topDownCropMaxZ = gridMin.z + maxGz * cellD;

        cameraImageWidth = texW;
        cameraImageHeight = texH;
        hasOrganizedCameraFrame = false;
        hasTopDownCameraTexture = true;
        loadedCameraSampleName = currentLoadedSampleName;
        RebuildActualCameraCaptureFootprint();
    }

    private static bool Solve3x3Linear(double[,] m, double[] rhs, out double[] sol)
    {
        sol = new double[3];
        double[,] a = new double[3, 4];
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++) a[r, c] = m[r, c];
            a[r, 3] = rhs[r];
        }
        for (int col = 0; col < 3; col++)
        {
            int piv = col;
            double maxAbs = Math.Abs(a[col, col]);
            for (int r = col + 1; r < 3; r++)
            {
                double v = Math.Abs(a[r, col]);
                if (v > maxAbs) { maxAbs = v; piv = r; }
            }
            if (maxAbs < 1e-11) return false;
            if (piv != col)
            {
                for (int c = col; c < 4; c++)
                {
                    double tmp = a[col, c];
                    a[col, c] = a[piv, c];
                    a[piv, c] = tmp;
                }
            }
            double diag = a[col, col];
            for (int r = col + 1; r < 3; r++)
            {
                double f = a[r, col] / diag;
                for (int c = col; c < 4; c++) a[r, c] -= f * a[col, c];
            }
        }
        for (int r = 2; r >= 0; r--)
        {
            double s = a[r, 3];
            for (int c = r + 1; c < 3; c++) s -= a[r, c] * sol[c];
            sol[r] = s / a[r, r];
        }
        return true;
    }

    /// <summary>
    /// Tính đúng 4 góc vùng thu nhận thực tế trên mặt bàn (TL, TR, BR, BL) khớp 1:1 với ảnh RGB từ file .PLY.
    /// </summary>
    public void RebuildActualCameraCaptureFootprint()
    {
        hasActualCaptureFootprint = false;
        float planeY = estimatedTableY + 0.005f;

        if (hasOrganizedCameraFrame && camGridWorldPos != null && camGridWorldValid != null && camGridW > 1 && camGridH > 1)
        {
            int strideX = Mathf.Max(1, camGridW / 28);
            int strideY = Mathf.Max(1, camGridH / 28);

            double[,] ata = new double[3, 3];
            double[] atx = new double[3];
            double[] atz = new double[3];
            int count = 0;
            float minX = float.MaxValue, maxX = float.MinValue;
            float minZ = float.MaxValue, maxZ = float.MinValue;

            for (int ty = 0; ty < camGridH; ty += strideY)
            {
                double v = ty / (double)(camGridH - 1);
                for (int tx = 0; tx < camGridW; tx += strideX)
                {
                    int idx = ty * camGridW + tx;
                    if (!camGridWorldValid[idx]) continue;
                    Vector3 wp = camGridWorldPos[idx];
                    if (wp.x < minX) minX = wp.x; if (wp.x > maxX) maxX = wp.x;
                    if (wp.z < minZ) minZ = wp.z; if (wp.z > maxZ) maxZ = wp.z;

                    double u = tx / (double)(camGridW - 1);
                    double[] row = { u, v, 1.0 };
                    for (int r = 0; r < 3; r++)
                    {
                        for (int c = 0; c < 3; c++) ata[r, c] += row[r] * row[c];
                        atx[r] += row[r] * wp.x;
                        atz[r] += row[r] * wp.z;
                    }
                    count++;
                }
            }

            if (count >= 6 && Solve3x3Linear(ata, atx, out double[] cx) && Solve3x3Linear(ata, atz, out double[] cz))
            {
                Vector2[] cornerUV = { new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(1f, 1f), new Vector2(0f, 1f) };
                for (int i = 0; i < 4; i++)
                {
                    float wx = (float)(cx[0] * cornerUV[i].x + cx[1] * cornerUV[i].y + cx[2]);
                    float wz = (float)(cz[0] * cornerUV[i].x + cz[1] * cornerUV[i].y + cz[2]);
                    actualCaptureFootprintWorld[i] = new Vector3(wx, planeY, wz);
                }
                hasActualCaptureFootprint = true;
                return;
            }

            if (count > 0 && maxX > minX && maxZ > minZ)
            {
                actualCaptureFootprintWorld[0] = new Vector3(minX, planeY, maxZ);
                actualCaptureFootprintWorld[1] = new Vector3(maxX, planeY, maxZ);
                actualCaptureFootprintWorld[2] = new Vector3(maxX, planeY, minZ);
                actualCaptureFootprintWorld[3] = new Vector3(minX, planeY, minZ);
                hasActualCaptureFootprint = true;
                return;
            }
        }

        if (hasTopDownCameraTexture && topDownCropMaxX > topDownCropMinX && topDownCropMaxZ > topDownCropMinZ)
        {
            actualCaptureFootprintWorld[0] = new Vector3(topDownCropMinX, planeY, topDownCropMaxZ);
            actualCaptureFootprintWorld[1] = new Vector3(topDownCropMaxX, planeY, topDownCropMaxZ);
            actualCaptureFootprintWorld[2] = new Vector3(topDownCropMaxX, planeY, topDownCropMinZ);
            actualCaptureFootprintWorld[3] = new Vector3(topDownCropMinX, planeY, topDownCropMinZ);
            hasActualCaptureFootprint = true;
        }
    }

    public bool TryGetActualCameraCaptureFootprintWorld(out Vector3 cTL, out Vector3 cTR, out Vector3 cBR, out Vector3 cBL)
    {
        if (!hasActualCaptureFootprint && (hasOrganizedCameraFrame || hasTopDownCameraTexture))
        {
            RebuildActualCameraCaptureFootprint();
        }
        if (hasActualCaptureFootprint)
        {
            cTL = actualCaptureFootprintWorld[0];
            cTR = actualCaptureFootprintWorld[1];
            cBR = actualCaptureFootprintWorld[2];
            cBL = actualCaptureFootprintWorld[3];
            return true;
        }
        cTL = cTR = cBR = cBL = Vector3.zero;
        return false;
    }

    public bool TrySampleCameraUVToWorld(Vector2 uv, out Vector3 worldPos, out Vector3 normalWorld)
    {
        worldPos = Vector3.zero;
        normalWorld = Vector3.up;
        float u = Mathf.Clamp01(uv.x);
        float v = Mathf.Clamp01(uv.y);

        if (hasOrganizedCameraFrame && camGridWorldPos != null && camGridWorldValid != null && camGridW > 0 && camGridH > 0)
        {
            int cx = Mathf.Clamp(Mathf.RoundToInt(u * (camGridW - 1)), 0, camGridW - 1);
            int cy = Mathf.Clamp(Mathf.RoundToInt(v * (camGridH - 1)), 0, camGridH - 1);

            bool found = false;
            Vector3 p = Vector3.zero;
            if (camGridWorldValid[cy * camGridW + cx])
            {
                p = camGridWorldPos[cy * camGridW + cx];
                found = true;
            }
            else
            {
                for (int r = 1; r <= 12 && !found; r++)
                {
                    Vector3 sum = Vector3.zero;
                    int cnt = 0;
                    for (int dy = -r; dy <= r; dy++)
                    {
                        int ny = cy + dy;
                        if (ny < 0 || ny >= camGridH) continue;
                        for (int dx = -r; dx <= r; dx++)
                        {
                            if (Mathf.Abs(dx) != r && Mathf.Abs(dy) != r) continue;
                            int nx = cx + dx;
                            if (nx < 0 || nx >= camGridW) continue;
                            int idx = ny * camGridW + nx;
                            if (camGridWorldValid[idx])
                            {
                                sum += camGridWorldPos[idx];
                                cnt++;
                            }
                        }
                    }
                    if (cnt > 0)
                    {
                        p = sum / cnt;
                        found = true;
                    }
                }
            }

            if (found)
            {
                worldPos = p;
                if (SampleSurfaceWorldHeightAndNormal(p.x, p.z, out float surfY, out Vector3 surfNorm))
                {
                    if (Mathf.Abs(surfY - p.y) < 0.08f)
                    {
                        worldPos.y = surfY;
                    }
                    normalWorld = surfNorm;
                }
                return true;
            }
        }

        if (hasTopDownCameraTexture)
        {
            float wx = Mathf.Lerp(topDownCropMinX, topDownCropMaxX, u);
            float wz = Mathf.Lerp(topDownCropMaxZ, topDownCropMinZ, v);
            if (SampleSurfaceWorldHeightAndNormal(wx, wz, out float wy, out normalWorld))
            {
                worldPos = new Vector3(wx, wy, wz);
                return true;
            }
            worldPos = new Vector3(wx, estimatedTableY + 0.02f, wz);
            return true;
        }

        return false;
    }

    public bool TryWorldToCameraUV(Vector3 worldPos, out Vector2 uv)
    {
        uv = Vector2.zero;
        if (hasTopDownCameraTexture)
        {
            float spanX = Mathf.Max(1e-4f, topDownCropMaxX - topDownCropMinX);
            float spanZ = Mathf.Max(1e-4f, topDownCropMaxZ - topDownCropMinZ);
            uv = new Vector2(
                Mathf.Clamp01((worldPos.x - topDownCropMinX) / spanX),
                Mathf.Clamp01((topDownCropMaxZ - worldPos.z) / spanZ)
            );
            return true;
        }

        if (hasOrganizedCameraFrame && camGridWorldPos != null && camGridWorldValid != null && camGridW > 0 && camGridH > 0)
        {
            float bestD2 = float.MaxValue;
            int bestX = -1, bestY = -1;
            int coarse = 4;
            for (int y = 0; y < camGridH; y += coarse)
            {
                for (int x = 0; x < camGridW; x += coarse)
                {
                    int idx = y * camGridW + x;
                    if (!camGridWorldValid[idx]) continue;
                    Vector3 p = camGridWorldPos[idx];
                    float dx = p.x - worldPos.x;
                    float dz = p.z - worldPos.z;
                    float d2 = dx * dx + dz * dz;
                    if (d2 < bestD2)
                    {
                        bestD2 = d2;
                        bestX = x;
                        bestY = y;
                    }
                }
            }
            if (bestX >= 0)
            {
                int x0 = Mathf.Max(0, bestX - coarse);
                int x1 = Mathf.Min(camGridW - 1, bestX + coarse);
                int y0 = Mathf.Max(0, bestY - coarse);
                int y1 = Mathf.Min(camGridH - 1, bestY + coarse);
                for (int y = y0; y <= y1; y++)
                {
                    for (int x = x0; x <= x1; x++)
                    {
                        int idx = y * camGridW + x;
                        if (!camGridWorldValid[idx]) continue;
                        Vector3 p = camGridWorldPos[idx];
                        float dx = p.x - worldPos.x;
                        float dz = p.z - worldPos.z;
                        float d2 = dx * dx + dz * dz;
                        if (d2 < bestD2)
                        {
                            bestD2 = d2;
                            bestX = x;
                            bestY = y;
                        }
                    }
                }
                if (bestD2 <= 0.01f)
                {
                    uv = new Vector2((float)bestX / Mathf.Max(1, camGridW - 1), (float)bestY / Mathf.Max(1, camGridH - 1));
                    return true;
                }
            }
        }

        return false;
    }

    // Lưới tra cứu độ cao bề mặt 3D (X_world, Z_world) -> Y_world (m) & pháp tuyến vi mô 3D
    // Tách biệt hoàn toàn Raw Grid (dữ liệu cảm biến gốc, bất biến) và Working Grid (sau khi vá lỗ)
    // để thuật toán phát hiện lỗ khuyết luôn thấy 100% lỗ thật và nút "Làm Mới" khôi phục được bề mặt gốc!
    public const int GRID_RES = 480;
    private float[,] rawElevationGridY;
    private bool[,] rawElevationGridValid;
    private Color32[,] rawElevationGridColor;

    private float[,] elevationGridY;
    private bool[,] elevationGridValid;
    private Color32[,] elevationGridColor;
    private bool[,] patchedCellMask;
    private Vector3 gridMin = new Vector3(-0.75f, 0f, -0.35f);
    private Vector3 gridMax = new Vector3(0.75f, 0f, 0.95f);
    public float estimatedTableY = 0.0f;

    public void GetCellWorldSize(out float cellW, out float cellD)
    {
        cellW = (gridMax.x - gridMin.x) / (GRID_RES - 1);
        cellD = (gridMax.z - gridMin.z) / (GRID_RES - 1);
    }

    /// <summary>
    /// Trích xuất đám mây điểm 3D bề mặt phôi THỰC SỰ theo lưới voxel 3D (vx, vy, vz) trực tiếp từ toàn bộ
    /// các đỉnh 3D trong CloudChunk_* (giữ nguyên đầy đủ các điểm trên vách dốc đứng / thành ống / nếp gấp 3D,
    /// không bị mất điểm như lưới chiều cao 2.5D!). Tự động dự phòng sang elevationGridY nếu chưa có CloudChunk.
    /// </summary>
    public List<Vector3> ExtractGarmentSurfaceCloudVoxelized(
        float voxelSizeM = 0.005f,
        List<Vector3> contourClipWorld = null,
        int maxPoints = 650,
        float heightAboveTableM = 0.006f)
    {
        var pts = new List<Vector3>();
        float vox = Mathf.Clamp(voxelSizeM, 0.002f, 0.030f);
        float invVox = 1f / vox;

        bool hasClip = (contourClipWorld != null && contourClipWorld.Count >= 3);
        float cMinX = float.MaxValue, cMaxX = float.MinValue, cMinZ = float.MaxValue, cMaxZ = float.MinValue;
        if (hasClip)
        {
            for (int i = 0; i < contourClipWorld.Count; i++)
            {
                Vector3 c = contourClipWorld[i];
                if (c.x < cMinX) cMinX = c.x; if (c.x > cMaxX) cMaxX = c.x;
                if (c.z < cMinZ) cMinZ = c.z; if (c.z > cMaxZ) cMaxZ = c.z;
            }
        }

        float yMinCut = estimatedTableY + Mathf.Max(0.002f, heightAboveTableM);

        // Ưu tiên 1: Voxel hóa 3D thật (vx, vy, vz) trực tiếp từ CloudChunk_* để giữ trọn vách dốc đứng & nếp gấp 3D
        var voxelSum = new Dictionary<Vector3Int, Vector3>(2048);
        var voxelCnt = new Dictionary<Vector3Int, int>(2048);
        int childCount = transform.childCount;
        for (int c = 0; c < childCount; c++)
        {
            Transform child = transform.GetChild(c);
            if (child == null || !child.gameObject.activeSelf || !child.name.StartsWith("CloudChunk_")) continue;
            MeshFilter mf = child.GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null) continue;
            Vector3[] verts = mf.sharedMesh.vertices;
            if (verts == null || verts.Length == 0) continue;

            Matrix4x4 l2w = child.localToWorldMatrix;
            int step = Mathf.Max(1, verts.Length / 45000);
            for (int i = 0; i < verts.Length; i += step)
            {
                Vector3 w = l2w.MultiplyPoint3x4(verts[i]);
                if (float.IsNaN(w.x) || float.IsNaN(w.y) || float.IsNaN(w.z)) continue;
                if (!hasClip && w.y < yMinCut) continue;
                if (hasClip)
                {
                    if (w.x < cMinX - 0.015f || w.x > cMaxX + 0.015f || w.z < cMinZ - 0.015f || w.z > cMaxZ + 0.015f) continue;
                    if (!IsPointInPolygonXZStatic(w.x, w.z, contourClipWorld)) continue;
                }

                Vector3Int key = new Vector3Int(
                    Mathf.FloorToInt(w.x * invVox),
                    Mathf.FloorToInt(w.y * invVox),
                    Mathf.FloorToInt(w.z * invVox));
                if (voxelSum.TryGetValue(key, out Vector3 acc))
                {
                    voxelSum[key] = acc + w;
                    voxelCnt[key] = voxelCnt[key] + 1;
                }
                else
                {
                    voxelSum[key] = w;
                    voxelCnt[key] = 1;
                }
            }
        }

        var raw = new List<Vector3>(voxelSum.Count > 0 ? voxelSum.Count : 1500);
        if (voxelSum.Count >= 30)
        {
            foreach (var kv in voxelSum)
            {
                int cnt = voxelCnt[kv.Key];
                raw.Add(kv.Value / Mathf.Max(1, cnt));
            }
        }
        else if (elevationGridY != null && elevationGridValid != null)
        {
            // Dự phòng: lấy từ lưới chiều cao nếu chưa có CloudChunk
            GetCellWorldSize(out float cellW, out float cellD);
            float avgCell = Mathf.Max(0.001f, 0.5f * (cellW + cellD));
            int stride = Mathf.Max(1, Mathf.RoundToInt(vox / avgCell));
            for (int gx = 0; gx < GRID_RES; gx += stride)
            {
                float wx = gridMin.x + ((float)gx / (GRID_RES - 1)) * (gridMax.x - gridMin.x);
                if (hasClip && (wx < cMinX - 0.01f || wx > cMaxX + 0.01f)) continue;
                for (int gz = 0; gz < GRID_RES; gz += stride)
                {
                    if (!elevationGridValid[gx, gz]) continue;
                    float wy = elevationGridY[gx, gz];
                    if (!hasClip && wy < yMinCut) continue;

                    float wz = gridMin.z + ((float)gz / (GRID_RES - 1)) * (gridMax.z - gridMin.z);
                    if (hasClip)
                    {
                        if (wz < cMinZ - 0.01f || wz > cMaxZ + 0.01f) continue;
                        if (!IsPointInPolygonXZStatic(wx, wz, contourClipWorld)) continue;
                    }
                    raw.Add(new Vector3(wx, wy, wz));
                }
            }
        }

        if (raw.Count == 0) return pts;
        if (maxPoints > 0 && raw.Count > maxPoints)
        {
            int dec = Mathf.Max(1, raw.Count / maxPoints);
            for (int i = 0; i < raw.Count; i += dec) pts.Add(raw[i]);
        }
        else
        {
            pts.AddRange(raw);
        }
        return pts;
    }

    private static bool IsPointInPolygonXZStatic(float x, float z, List<Vector3> poly)
    {
        bool inside = false;
        int n = poly.Count;
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            float xi = poly[i].x, zi = poly[i].z;
            float xj = poly[j].x, zj = poly[j].z;
            if (((zi > z) != (zj > z)) && (x < (xj - xi) * (z - zi) / (zj - zi + 1e-9f) + xi))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    public void RebuildSurfaceElevationGrid()
    {
        rawElevationGridY = new float[GRID_RES, GRID_RES];
        rawElevationGridValid = new bool[GRID_RES, GRID_RES];
        rawElevationGridColor = new Color32[GRID_RES, GRID_RES];

        elevationGridY = new float[GRID_RES, GRID_RES];
        elevationGridValid = new bool[GRID_RES, GRID_RES];
        elevationGridColor = new Color32[GRID_RES, GRID_RES];
        patchedCellMask = new bool[GRID_RES, GRID_RES];

        int[,] counts = new int[GRID_RES, GRID_RES];
        long[,] colorR = new long[GRID_RES, GRID_RES];
        long[,] colorG = new long[GRID_RES, GRID_RES];
        long[,] colorB = new long[GRID_RES, GRID_RES];
        List<float> allValidHeights = new List<float>(GRID_RES * GRID_RES);

        for (int c = 0; c < transform.childCount; c++)
        {
            Transform child = transform.GetChild(c);
            if (child == null || !child.gameObject.activeSelf || !child.name.StartsWith("CloudChunk_")) continue;
            MeshFilter mf = child.GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null) continue;
            Vector3[] verts = mf.sharedMesh.vertices;
            Color32[] cols = mf.sharedMesh.colors32;
            bool hasCols = (cols != null && cols.Length == verts.Length);

            // Quét mật độ dày & dùng RoundToInt đồng bộ 100% với WorldToGridCell / InjectSurfacePoint (không bị lệch nửa ô!)
            int step = Mathf.Max(1, verts.Length / 220000);
            for (int i = 0; i < verts.Length; i += step)
            {
                Vector3 v = child.TransformPoint(verts[i]);
                if (v.y < -0.15f || v.y > 0.70f) continue;
                int gx = Mathf.Clamp(Mathf.RoundToInt((v.x - gridMin.x) / (gridMax.x - gridMin.x) * (GRID_RES - 1)), 0, GRID_RES - 1);
                int gz = Mathf.Clamp(Mathf.RoundToInt((v.z - gridMin.z) / (gridMax.z - gridMin.z) * (GRID_RES - 1)), 0, GRID_RES - 1);
                rawElevationGridY[gx, gz] += v.y;
                counts[gx, gz]++;

                if (hasCols)
                {
                    Color32 c32 = cols[i];
                    colorR[gx, gz] += c32.r;
                    colorG[gx, gz] += c32.g;
                    colorB[gx, gz] += c32.b;
                }
            }
        }

        for (int x = 0; x < GRID_RES; x++)
        {
            for (int z = 0; z < GRID_RES; z++)
            {
                if (counts[x, z] > 0)
                {
                    float avgY = rawElevationGridY[x, z] / counts[x, z];
                    rawElevationGridY[x, z] = avgY;
                    rawElevationGridValid[x, z] = true;
                    byte r = (byte)Mathf.Clamp((int)(colorR[x, z] / counts[x, z]), 0, 255);
                    byte g = (byte)Mathf.Clamp((int)(colorG[x, z] / counts[x, z]), 0, 255);
                    byte b = (byte)Mathf.Clamp((int)(colorB[x, z] / counts[x, z]), 0, 255);
                    rawElevationGridColor[x, z] = new Color32(r, g, b, 255);
                    allValidHeights.Add(avgY);
                }
                else
                {
                    rawElevationGridColor[x, z] = new Color32(185, 175, 155, 255);
                }

                elevationGridY[x, z] = rawElevationGridY[x, z];
                elevationGridValid[x, z] = rawElevationGridValid[x, z];
                elevationGridColor[x, z] = rawElevationGridColor[x, z];
            }
        }

        // Ước lượng cao độ mặt bàn từ phân vị thấp (15%) của toàn bộ lưới quét
        if (allValidHeights.Count > 20)
        {
            allValidHeights.Sort();
            estimatedTableY = allValidHeights[Mathf.Clamp((int)(allValidHeights.Count * 0.15f), 0, allValidHeights.Count - 1)];
        }
        else
        {
            estimatedTableY = 0.0f;
        }
    }

    /// <summary>
    /// Khôi phục lưới độ cao bề mặt về đúng dữ liệu cảm biến gốc (xóa mọi điểm vá cũ)
    /// </summary>
    public void ResetInpaintedPointsToRaw()
    {
        if (rawElevationGridY == null || rawElevationGridValid == null) return;
        if (patchedCellMask == null) patchedCellMask = new bool[GRID_RES, GRID_RES];
        for (int x = 0; x < GRID_RES; x++)
        {
            for (int z = 0; z < GRID_RES; z++)
            {
                elevationGridY[x, z] = rawElevationGridY[x, z];
                elevationGridValid[x, z] = rawElevationGridValid[x, z];
                elevationGridColor[x, z] = rawElevationGridColor[x, z];
                patchedCellMask[x, z] = false;
            }
        }
        ConvertPointCloudToSurfaceMesh(applyPinHoleFill: false);
    }

    /// <summary>
    /// Chuyển đổi toàn bộ dữ liệu Point Cloud hoặc Elevation Grid thành 3D Surface Mesh siêu nhẹ.
    /// Lưu ý: Pin-hole fill chỉ chạy trên bản sao tạm cho mesh hiển thị, KHÔNG ghi đè `rawElevationGridValid`!
    /// </summary>
    [ContextMenu("Chuyển đổi sang 3D Surface Mesh")]
    public GameObject ConvertPointCloudToSurfaceMesh()
    {
        return ConvertPointCloudToSurfaceMesh(applyPinHoleFill: true);
    }

    public GameObject ConvertPointCloudToSurfaceMesh(bool applyPinHoleFill)
    {
        if (elevationGridY == null || elevationGridValid == null || elevationGridColor == null)
        {
            RebuildSurfaceElevationGrid();
        }
        if (elevationGridY == null) return null;

        // Xóa triệt để mọi mesh cũ nếu đã có (tách khỏi parent ngay để không còn tồn tại trong cùng frame Play Mode)
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Transform child = transform.GetChild(i);
            if (child != null && child.name == "WorkpieceSurfaceMesh")
            {
                child.SetParent(null);
                child.gameObject.SetActive(false);
                if (Application.isPlaying) Destroy(child.gameObject);
                else DestroyImmediate(child.gameObject);
            }
        }
        surfaceMeshObject = null;
        surfaceMeshCollider = null;

        // Tạo bản sao làm việc để pin-hole fill không phá hỏng cờ valid gốc
        float[,] meshY = (float[,])elevationGridY.Clone();
        bool[,] meshValid = (bool[,])elevationGridValid.Clone();
        Color32[,] meshColor = (Color32[,])elevationGridColor.Clone();

        List<Vector3> verts = new List<Vector3>();
        List<Color32> colors = new List<Color32>();
        int[,] gridVertIdx = new int[GRID_RES, GRID_RES];
        for (int x = 0; x < GRID_RES; x++)
            for (int z = 0; z < GRID_RES; z++)
                gridVertIdx[x, z] = -1;

        float cellW = (gridMax.x - gridMin.x) / (GRID_RES - 1);
        float cellD = (gridMax.z - gridMin.z) / (GRID_RES - 1);

        // Vá 2 pass lỗ kim 1-ô (5/8 hàng xóm) khi dựng mesh hiển thị để bịt kín khe vi mô; không làm thay đổi rawElevationGridValid!
        if (applyPinHoleFill)
        {
            for (int phPass = 0; phPass < 2; phPass++)
            {
                bool[,] nextValid = (bool[,])meshValid.Clone();
                float[,] nextY = (float[,])meshY.Clone();
                Color32[,] nextCol = (Color32[,])meshColor.Clone();

                for (int x = 1; x < GRID_RES - 1; x++)
                {
                    for (int z = 1; z < GRID_RES - 1; z++)
                    {
                        if (meshValid[x, z]) continue;

                        float sumY = 0f;
                        int validNeighbors = 0;
                        long sumR = 0, sumG = 0, sumB = 0;

                        for (int dx = -1; dx <= 1; dx++)
                        {
                            for (int dz = -1; dz <= 1; dz++)
                            {
                                if (dx == 0 && dz == 0) continue;
                                int nx = x + dx;
                                int nz = z + dz;
                                if (meshValid[nx, nz])
                                {
                                    sumY += meshY[nx, nz];
                                    sumR += meshColor[nx, nz].r;
                                    sumG += meshColor[nx, nz].g;
                                    sumB += meshColor[nx, nz].b;
                                    validNeighbors++;
                                }
                            }
                        }

                        if (validNeighbors >= 5)
                        {
                            nextY[x, z] = sumY / validNeighbors;
                            nextCol[x, z] = new Color32((byte)(sumR / validNeighbors), (byte)(sumG / validNeighbors), (byte)(sumB / validNeighbors), 255);
                            nextValid[x, z] = true;
                        }
                    }
                }

                meshY = nextY;
                meshValid = nextValid;
                meshColor = nextCol;
            }
        }

        // Pass 1: Tạo các đỉnh (Vertices) trên lưới
        for (int x = 0; x < GRID_RES; x++)
        {
            float wx = gridMin.x + x * cellW;
            for (int z = 0; z < GRID_RES; z++)
            {
                if (!meshValid[x, z]) continue;
                float wz = gridMin.z + z * cellD;
                float wy = meshY[x, z];

                Vector3 worldPos = new Vector3(wx, wy, wz);
                Vector3 localPos = transform.InverseTransformPoint(worldPos);

                gridVertIdx[x, z] = verts.Count;
                verts.Add(localPos);
                colors.Add(meshColor[x, z]);
            }
        }

        if (verts.Count == 0)
        {
            Debug.LogWarning("[PLY Loader] Không có đỉnh hợp lệ để tạo 3D Surface Mesh.");
            return null;
        }

        // Pass 2: Nối các tam giác (Triangles)
        List<int> triangles = new List<int>();
        float maxEdgeDy = 0.045f; // Ngăn nối giữa 2 ống quần hoặc nối từ quần rơi xuống mặt bàn (>4.5cm)

        for (int x = 0; x < GRID_RES - 1; x++)
        {
            for (int z = 0; z < GRID_RES - 1; z++)
            {
                int v00 = gridVertIdx[x, z];
                int v10 = gridVertIdx[x + 1, z];
                int v01 = gridVertIdx[x, z + 1];
                int v11 = gridVertIdx[x + 1, z + 1];

                bool has00 = v00 >= 0;
                bool has10 = v10 >= 0;
                bool has01 = v01 >= 0;
                bool has11 = v11 >= 0;

                // Tam giác 1: (v00, v01, v10)
                if (has00 && has01 && has10)
                {
                    bool anyPatched = patchedCellMask != null && (patchedCellMask[x, z] || patchedCellMask[x, z + 1] || patchedCellMask[x + 1, z]);
                    float edgeLimit = anyPatched ? 0.14f : maxEdgeDy;
                    float y00 = meshY[x, z];
                    float y01 = meshY[x, z + 1];
                    float y10 = meshY[x + 1, z];
                    if (Mathf.Abs(y00 - y01) < edgeLimit && Mathf.Abs(y00 - y10) < edgeLimit && Mathf.Abs(y01 - y10) < edgeLimit)
                    {
                        triangles.Add(v00);
                        triangles.Add(v01);
                        triangles.Add(v10);
                    }
                }

                // Tam giác 2: (v10, v01, v11)
                if (has10 && has01 && has11)
                {
                    bool anyPatched = patchedCellMask != null && (patchedCellMask[x + 1, z] || patchedCellMask[x, z + 1] || patchedCellMask[x + 1, z + 1]);
                    float edgeLimit = anyPatched ? 0.14f : maxEdgeDy;
                    float y10 = meshY[x + 1, z];
                    float y01 = meshY[x, z + 1];
                    float y11 = meshY[x + 1, z + 1];
                    if (Mathf.Abs(y10 - y01) < edgeLimit && Mathf.Abs(y10 - y11) < edgeLimit && Mathf.Abs(y01 - y11) < edgeLimit)
                    {
                        triangles.Add(v10);
                        triangles.Add(v01);
                        triangles.Add(v11);
                    }
                }
            }
        }

        Mesh surfaceMesh = new Mesh();
        surfaceMesh.name = "Workpiece_Cloth_SurfaceMesh";
        surfaceMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        surfaceMesh.SetVertices(verts);
        surfaceMesh.SetColors(colors);
        surfaceMesh.SetTriangles(triangles, 0);
        surfaceMesh.RecalculateNormals();
        surfaceMesh.RecalculateBounds();

        surfaceMeshObject = new GameObject("WorkpieceSurfaceMesh");
        surfaceMeshObject.transform.SetParent(this.transform, false);
        surfaceMeshObject.transform.localPosition = Vector3.zero;
        surfaceMeshObject.transform.localRotation = Quaternion.identity;
        surfaceMeshObject.transform.localScale = Vector3.one;

        MeshFilter mf = surfaceMeshObject.AddComponent<MeshFilter>();
        mf.sharedMesh = surfaceMesh;

        MeshRenderer mr = surfaceMeshObject.AddComponent<MeshRenderer>();
        mr.sharedMaterial = pointCloudMaterial;

        surfaceMeshCollider = surfaceMeshObject.AddComponent<MeshCollider>();
        surfaceMeshCollider.sharedMesh = surfaceMesh;

        Debug.Log($"[PLY Loader] Đã chuyển đổi thành công sang 3D Surface Mesh: {verts.Count:N0} đỉnh, {triangles.Count / 3:N0} tam giác (1 Draw Call duy nhất, siêu mượt!)");

        ApplyRenderMode();
        return surfaceMeshObject;
    }

    public Vector2Int WorldToGridCell(float worldX, float worldZ)
    {
        int gx = Mathf.Clamp(Mathf.RoundToInt((worldX - gridMin.x) / (gridMax.x - gridMin.x) * (GRID_RES - 1)), 0, GRID_RES - 1);
        int gz = Mathf.Clamp(Mathf.RoundToInt((worldZ - gridMin.z) / (gridMax.z - gridMin.z) * (GRID_RES - 1)), 0, GRID_RES - 1);
        return new Vector2Int(gx, gz);
    }

    public Vector2 GridCellToWorldXZ(int gx, int gz)
    {
        float cellW = (gridMax.x - gridMin.x) / (GRID_RES - 1);
        float cellD = (gridMax.z - gridMin.z) / (GRID_RES - 1);
        return new Vector2(gridMin.x + gx * cellW, gridMin.z + gz * cellD);
    }

    /// <summary>
    /// Kiểm tra trực tiếp trên LƯỚI CẢM BIẾN GỐC (rawElevationGridValid - chưa bị biến đổi bởi pin-hole fill hay vá lỗ).
    /// Đảm bảo phát hiện chính xác 100% các lỗ khuyết tật thật!
    /// </summary>
    public bool HasSurfacePointAt(float worldX, float worldZ)
    {
        if (rawElevationGridValid != null)
        {
            Vector2Int c = WorldToGridCell(worldX, worldZ);
            return rawElevationGridValid[c.x, c.y];
        }
        if (elevationGridValid == null) return false;
        Vector2Int cell = WorldToGridCell(worldX, worldZ);
        return elevationGridValid[cell.x, cell.y];
    }

    /// <summary>
    /// Lấy cao độ Y và màu RGB trực tiếp từ LƯỚI CẢM BIẾN GỐC tại (worldX, worldZ) — KHÔNG nội suy, KHÔNG lấy điểm đã vá.
    /// Dùng làm đầu vào sạch cho các thuật toán vá lỗ (K-NN MLS, TPS RBF, Laplacian PDE).
    /// </summary>
    public bool TrySampleRawSurfaceAt(float worldX, float worldZ, out float rawY, out Color32 rawColor)
    {
        rawY = estimatedTableY;
        rawColor = new Color32(140, 145, 155, 255);
        if (rawElevationGridValid == null || rawElevationGridY == null) return false;
        Vector2Int c = WorldToGridCell(worldX, worldZ);
        if (!rawElevationGridValid[c.x, c.y]) return false;
        rawY = rawElevationGridY[c.x, c.y];
        rawColor = rawElevationGridColor[c.x, c.y];
        return true;
    }

    /// <summary>
    /// Thu thập K điểm cảm biến gốc hợp lệ gần nhất đối xứng xung quanh (worldX, worldZ) theo vòng tròn đồng tâm mở rộng.
    /// Dừng ngay khi đủ K láng giềng (+2 vòng đối xứng), tốc độ nhanh gấp >10 lần ở độ phân giải lưới 480x480
    /// và tự động mở rộng tới ~28 cm cho các vùng bị che lớn, đồng thời lọc bỏ điểm mặt bàn (`minClothY`).
    /// </summary>
    public List<Vector3> CollectRawSurfaceNeighbors(float worldX, float worldZ, float searchRadiusM, int maxNeighbors, float minClothY = -10f)
    {
        List<Vector3> result = new List<Vector3>(maxNeighbors);
        if (rawElevationGridValid == null || rawElevationGridY == null) return result;

        Vector2Int center = WorldToGridCell(worldX, worldZ);
        float cellW = (gridMax.x - gridMin.x) / (GRID_RES - 1);
        float cellD = (gridMax.z - gridMin.z) / (GRID_RES - 1);
        float minCell = Mathf.Max(1e-4f, Mathf.Min(cellW, cellD));

        int baseRing = Mathf.Clamp(Mathf.CeilToInt(Mathf.Max(0.02f, searchRadiusM) / minCell), 4, 48);
        int hardMaxRing = Mathf.Clamp(Mathf.CeilToInt(0.28f / minCell), baseRing, 96);

        List<(float distSq, Vector3 pos)> candidates = new List<(float, Vector3)>(maxNeighbors * 3);
        int extraRingsLeft = -1;

        for (int pass = 0; pass < 2; pass++)
        {
            candidates.Clear();
            extraRingsLeft = -1;
            bool filterTable = (pass == 0 && minClothY > -10f);

            for (int r = 1; r <= hardMaxRing; r++)
            {
                int x0 = center.x - r;
                int x1 = center.x + r;
                int z0 = center.y - r;
                int z1 = center.y + r;

                // Cạnh trên & dưới của vòng r
                for (int dx = -r; dx <= r; dx++)
                {
                    int nx = center.x + dx;
                    if (nx < 0 || nx >= GRID_RES) continue;

                    if (z0 >= 0 && z0 < GRID_RES && rawElevationGridValid[nx, z0])
                    {
                        float ry = rawElevationGridY[nx, z0];
                        if (!filterTable || ry >= minClothY)
                        {
                            Vector2 wp = GridCellToWorldXZ(nx, z0);
                            float d2 = (wp.x - worldX) * (wp.x - worldX) + (wp.y - worldZ) * (wp.y - worldZ);
                            candidates.Add((d2, new Vector3(wp.x, ry, wp.y)));
                        }
                    }
                    if (z1 >= 0 && z1 < GRID_RES && rawElevationGridValid[nx, z1])
                    {
                        float ry = rawElevationGridY[nx, z1];
                        if (!filterTable || ry >= minClothY)
                        {
                            Vector2 wp = GridCellToWorldXZ(nx, z1);
                            float d2 = (wp.x - worldX) * (wp.x - worldX) + (wp.y - worldZ) * (wp.y - worldZ);
                            candidates.Add((d2, new Vector3(wp.x, ry, wp.y)));
                        }
                    }
                }

                // Cạnh trái & phải của vòng r (bỏ 4 góc đã duyệt)
                for (int dz = -r + 1; dz <= r - 1; dz++)
                {
                    int nz = center.y + dz;
                    if (nz < 0 || nz >= GRID_RES) continue;

                    if (x0 >= 0 && x0 < GRID_RES && rawElevationGridValid[x0, nz])
                    {
                        float ry = rawElevationGridY[x0, nz];
                        if (!filterTable || ry >= minClothY)
                        {
                            Vector2 wp = GridCellToWorldXZ(x0, nz);
                            float d2 = (wp.x - worldX) * (wp.x - worldX) + (wp.y - worldZ) * (wp.y - worldZ);
                            candidates.Add((d2, new Vector3(wp.x, ry, wp.y)));
                        }
                    }
                    if (x1 >= 0 && x1 < GRID_RES && rawElevationGridValid[x1, nz])
                    {
                        float ry = rawElevationGridY[x1, nz];
                        if (!filterTable || ry >= minClothY)
                        {
                            Vector2 wp = GridCellToWorldXZ(x1, nz);
                            float d2 = (wp.x - worldX) * (wp.x - worldX) + (wp.y - worldZ) * (wp.y - worldZ);
                            candidates.Add((d2, new Vector3(wp.x, ry, wp.y)));
                        }
                    }
                }

                if (extraRingsLeft < 0 && candidates.Count >= maxNeighbors)
                {
                    extraRingsLeft = 2; // Quét thêm 2 vòng để đảm bảo đối xứng tròn
                }
                else if (extraRingsLeft > 0)
                {
                    extraRingsLeft--;
                    if (extraRingsLeft == 0) break;
                }
            }

            if (candidates.Count >= Mathf.Min(6, maxNeighbors)) break;
        }

        candidates.Sort((a, b) => a.distSq.CompareTo(b.distSq));
        int take = Mathf.Min(maxNeighbors, candidates.Count);
        for (int i = 0; i < take; i++) result.Add(candidates[i].pos);
        return result;
    }

    public bool IsRawCellValid(int gx, int gz)
    {
        if (rawElevationGridValid == null || gx < 0 || gx >= GRID_RES || gz < 0 || gz >= GRID_RES) return false;
        return rawElevationGridValid[gx, gz];
    }

    public float GetRawCellY(int gx, int gz)
    {
        if (rawElevationGridY == null || gx < 0 || gx >= GRID_RES || gz < 0 || gz >= GRID_RES) return estimatedTableY;
        return rawElevationGridY[gx, gz];
    }

    public void InjectSurfaceCell(int gx, int gz, float worldY, Color32 color)
    {
        if (elevationGridY == null || elevationGridValid == null) RebuildSurfaceElevationGrid();
        if (elevationGridY == null || gx < 0 || gx >= GRID_RES || gz < 0 || gz >= GRID_RES) return;
        if (patchedCellMask == null) patchedCellMask = new bool[GRID_RES, GRID_RES];
        elevationGridY[gx, gz] = worldY;
        elevationGridValid[gx, gz] = true;
        patchedCellMask[gx, gz] = true;
        if (elevationGridColor != null) elevationGridColor[gx, gz] = color;
    }

    /// <summary>
    /// Xuất 3D Surface Mesh hiện tại (đã vá kín toàn bộ vùng bị che khuất) ra file chuẩn Wavefront .OBJ (có màu đỉnh & pháp tuyến)
    /// </summary>
    public string ExportSurfaceMeshToObj(string outputPath)
    {
        if (surfaceMeshObject == null) ConvertPointCloudToSurfaceMesh(applyPinHoleFill: true);
        if (surfaceMeshObject == null) return "";
        MeshFilter mf = surfaceMeshObject.GetComponent<MeshFilter>();
        if (mf == null || mf.sharedMesh == null) return "";

        Mesh m = mf.sharedMesh;
        Vector3[] verts = m.vertices;
        Vector3[] norms = m.normals;
        Color32[] cols = m.colors32;
        int[] tris = m.triangles;

        string dir = Path.GetDirectoryName(outputPath);
        if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) Directory.CreateDirectory(dir);

        using (StreamWriter sw = new StreamWriter(outputPath, false, System.Text.Encoding.UTF8))
        {
            sw.WriteLine("# Dobot CR10A Repaired 3D Garment Surface Mesh (.OBJ)");
            sw.WriteLine($"# Vertices: {verts.Length} | Triangles: {tris.Length / 3}");
            bool hasCols = (cols != null && cols.Length == verts.Length);
            for (int i = 0; i < verts.Length; i++)
            {
                Vector3 wp = surfaceMeshObject.transform.TransformPoint(verts[i]);
                if (hasCols)
                {
                    sw.WriteLine(string.Format(System.Globalization.CultureInfo.InvariantCulture,
                        "v {0:F5} {1:F5} {2:F5} {3:F3} {4:F3} {5:F3}",
                        wp.x, wp.y, wp.z, cols[i].r / 255f, cols[i].g / 255f, cols[i].b / 255f));
                }
                else
                {
                    sw.WriteLine(string.Format(System.Globalization.CultureInfo.InvariantCulture,
                        "v {0:F5} {1:F5} {2:F5}", wp.x, wp.y, wp.z));
                }
            }
            if (norms != null && norms.Length == verts.Length)
            {
                for (int i = 0; i < norms.Length; i++)
                {
                    sw.WriteLine(string.Format(System.Globalization.CultureInfo.InvariantCulture,
                        "vn {0:F4} {1:F4} {2:F4}", norms[i].x, norms[i].y, norms[i].z));
                }
            }
            for (int i = 0; i < tris.Length; i += 3)
            {
                int a = tris[i] + 1, b = tris[i + 1] + 1, c = tris[i + 2] + 1;
                sw.WriteLine($"f {a}//{a} {b}//{b} {c}//{c}");
            }
        }
        return outputPath;
    }

    /// <summary>
    /// Nội suy màu RGB thực của vải xung quanh lỗ khuyết theo vòng đồng tâm (dừng ngay khi đủ mẫu -> siêu nhanh ở 480x480).
    /// </summary>
    public Color32 SampleSurroundingRawColor(float worldX, float worldZ, int searchCells = 14)
    {
        if (rawElevationGridValid == null || rawElevationGridColor == null)
            return new Color32(120, 130, 148, 255);

        Vector2Int c = WorldToGridCell(worldX, worldZ);
        int maxR = Mathf.Max(48, searchCells * 4);
        float sumR = 0f, sumG = 0f, sumB = 0f, sumW = 0f;
        int found = 0;

        for (int r = 1; r <= maxR; r++)
        {
            int x0 = c.x - r, x1 = c.x + r;
            int z0 = c.y - r, z1 = c.y + r;

            for (int dx = -r; dx <= r; dx++)
            {
                int nx = c.x + dx;
                if (nx < 0 || nx >= GRID_RES) continue;
                if (z0 >= 0 && z0 < GRID_RES && rawElevationGridValid[nx, z0])
                {
                    float w = 1f / (1f + dx * dx + r * r);
                    Color32 col = rawElevationGridColor[nx, z0];
                    sumR += col.r * w; sumG += col.g * w; sumB += col.b * w; sumW += w; found++;
                }
                if (z1 >= 0 && z1 < GRID_RES && rawElevationGridValid[nx, z1])
                {
                    float w = 1f / (1f + dx * dx + r * r);
                    Color32 col = rawElevationGridColor[nx, z1];
                    sumR += col.r * w; sumG += col.g * w; sumB += col.b * w; sumW += w; found++;
                }
            }
            for (int dz = -r + 1; dz <= r - 1; dz++)
            {
                int nz = c.y + dz;
                if (nz < 0 || nz >= GRID_RES) continue;
                if (x0 >= 0 && x0 < GRID_RES && rawElevationGridValid[x0, nz])
                {
                    float w = 1f / (1f + r * r + dz * dz);
                    Color32 col = rawElevationGridColor[x0, nz];
                    sumR += col.r * w; sumG += col.g * w; sumB += col.b * w; sumW += w; found++;
                }
                if (x1 >= 0 && x1 < GRID_RES && rawElevationGridValid[x1, nz])
                {
                    float w = 1f / (1f + r * r + dz * dz);
                    Color32 col = rawElevationGridColor[x1, nz];
                    sumR += col.r * w; sumG += col.g * w; sumB += col.b * w; sumW += w; found++;
                }
            }

            if (found >= 12) break;
        }

        if (sumW > 1e-5f)
        {
            return new Color32(
                (byte)Mathf.Clamp(Mathf.RoundToInt(sumR / sumW), 0, 255),
                (byte)Mathf.Clamp(Mathf.RoundToInt(sumG / sumW), 0, 255),
                (byte)Mathf.Clamp(Mathf.RoundToInt(sumB / sumW), 0, 255),
                255);
        }
        return new Color32(120, 130, 148, 255);
    }

    /// <summary>
    /// Giải phương trình Laplace PDE (∇²Y = 0) bằng lặp Gauss-Seidel SOR nhiều vòng trên các ô khuyết
    /// với điều kiện biên Dirichlet cố định từ các ô cảm biến gốc hợp lệ xung quanh.
    /// Hỗ trợ lỗ rất lớn (>1400 ô ở lưới 480x480) hội tụ mượt, không bao giờ rớt độ cao!
    /// </summary>
    public Dictionary<Vector2Int, float> SolveLaplacianPdeOnHoleCells(HashSet<Vector2Int> holeCells, int iterations = 100, float relaxation = 1.45f, float minClothY = -10f)
    {
        Dictionary<Vector2Int, float> solved = new Dictionary<Vector2Int, float>();
        if (rawElevationGridY == null || rawElevationGridValid == null || holeCells == null || holeCells.Count == 0)
            return solved;

        float[,] fieldY = (float[,])rawElevationGridY.Clone();
        bool[,] known = new bool[GRID_RES, GRID_RES];
        for (int x = 0; x < GRID_RES; x++)
        {
            for (int z = 0; z < GRID_RES; z++)
            {
                if (rawElevationGridValid[x, z] && rawElevationGridY[x, z] >= minClothY)
                {
                    known[x, z] = true;
                }
            }
        }

        // Khởi tạo giá trị ban đầu cho mỗi ô khuyết bằng biên vải hợp lệ gần nhất (tới 72 ô ~20cm)
        List<Vector2Int> holeList = new List<Vector2Int>(holeCells);
        for (int i = 0; i < holeList.Count; i++)
        {
            Vector2Int cell = holeList[i];
            if (cell.x < 0 || cell.x >= GRID_RES || cell.y < 0 || cell.y >= GRID_RES) continue;
            known[cell.x, cell.y] = false;
        }

        for (int i = 0; i < holeList.Count; i++)
        {
            Vector2Int cell = holeList[i];
            if (cell.x < 0 || cell.x >= GRID_RES || cell.y < 0 || cell.y >= GRID_RES) continue;
            float sumY = 0f, sumW = 0f;
            for (int r = 1; r <= 72 && sumW <= 0f; r += (r < 10 ? 1 : 4))
            {
                int x0 = cell.x - r, x1 = cell.x + r;
                int z0 = cell.y - r, z1 = cell.y + r;
                for (int dx = -r; dx <= r; dx++)
                {
                    int nx = cell.x + dx;
                    if (nx < 0 || nx >= GRID_RES) continue;
                    if (z0 >= 0 && z0 < GRID_RES && known[nx, z0])
                    {
                        float w = 1f / (dx * dx + r * r + 0.1f);
                        sumY += fieldY[nx, z0] * w; sumW += w;
                    }
                    if (z1 >= 0 && z1 < GRID_RES && known[nx, z1])
                    {
                        float w = 1f / (dx * dx + r * r + 0.1f);
                        sumY += fieldY[nx, z1] * w; sumW += w;
                    }
                }
                for (int dz = -r + 1; dz <= r - 1; dz++)
                {
                    int nz = cell.y + dz;
                    if (nz < 0 || nz >= GRID_RES) continue;
                    if (x0 >= 0 && x0 < GRID_RES && known[x0, nz])
                    {
                        float w = 1f / (r * r + dz * dz + 0.1f);
                        sumY += fieldY[x0, nz] * w; sumW += w;
                    }
                    if (x1 >= 0 && x1 < GRID_RES && known[x1, nz])
                    {
                        float w = 1f / (r * r + dz * dz + 0.1f);
                        sumY += fieldY[x1, nz] * w; sumW += w;
                    }
                }
            }
            fieldY[cell.x, cell.y] = sumW > 0f ? (sumY / sumW) : (estimatedTableY + 0.06f);
        }

        for (int i = 0; i < holeList.Count; i++)
        {
            Vector2Int cell = holeList[i];
            if (cell.x >= 0 && cell.x < GRID_RES && cell.y >= 0 && cell.y < GRID_RES)
                known[cell.x, cell.y] = true;
        }

        // Lặp Gauss-Seidel SOR lan truyền hài (Harmonic / Laplace ∇²Y = 0)
        float omega = Mathf.Clamp(relaxation, 0.5f, 1.65f);
        for (int it = 0; it < iterations; it++)
        {
            for (int i = 0; i < holeList.Count; i++)
            {
                Vector2Int cell = holeList[i];
                int x = cell.x;
                int z = cell.y;
                if (x <= 0 || x >= GRID_RES - 1 || z <= 0 || z >= GRID_RES - 1) continue;

                float sum = 0f;
                int cnt = 0;
                if (known[x - 1, z]) { sum += fieldY[x - 1, z]; cnt++; }
                if (known[x + 1, z]) { sum += fieldY[x + 1, z]; cnt++; }
                if (known[x, z - 1]) { sum += fieldY[x, z - 1]; cnt++; }
                if (known[x, z + 1]) { sum += fieldY[x, z + 1]; cnt++; }

                if (cnt > 0)
                {
                    float avg = sum / cnt;
                    fieldY[x, z] = fieldY[x, z] + omega * (avg - fieldY[x, z]);
                }
            }
        }

        for (int i = 0; i < holeList.Count; i++)
        {
            Vector2Int cell = holeList[i];
            if (cell.x >= 0 && cell.x < GRID_RES && cell.y >= 0 && cell.y < GRID_RES)
                solved[cell] = fieldY[cell.x, cell.y];
        }
        return solved;
    }

    /// <summary>
    /// Cập nhật lại 3D Surface Mesh sau khi bù đắp/vá lỗ khuyết tật
    /// </summary>
    public void RefreshSurfaceMeshFromGrid()
    {
        ConvertPointCloudToSurfaceMesh(applyPinHoleFill: true);
    }

    /// <summary>
    /// Bắn tia 3D trực tiếp lên MeshCollider của bề mặt quần (3D Surface Mesh) để tìm giao điểm phun mực chính xác.
    /// </summary>
    public bool RaycastSurfaceMesh(Ray ray, float maxDistance, out RaycastHit hit)
    {
        hit = default;
        if (surfaceMeshCollider != null && surfaceMeshObject != null && surfaceMeshObject.activeInHierarchy)
        {
            return surfaceMeshCollider.Raycast(ray, out hit, maxDistance);
        }
        return false;
    }

    /// <summary>
    /// Nội suy độ cao bề mặt 3D thực tế Y_world (m) và Vector pháp tuyến 3D tại (worldX, worldZ)
    /// Ưu tiên Raycast trực tiếp lên 3D Surface MeshCollider nếu có để đạt độ chính xác tuyệt đối!
    /// </summary>
    public bool SampleSurfaceWorldHeightAndNormal(float worldX, float worldZ, out float worldY, out Vector3 normalWorld)
    {
        worldY = 0.020f;
        normalWorld = Vector3.up;

        // Ưu tiên 1: Raycast trực tiếp lên 3D Surface MeshCollider
        if (surfaceMeshCollider != null && surfaceMeshObject != null && surfaceMeshObject.activeInHierarchy)
        {
            Ray ray = new Ray(new Vector3(worldX, 0.70f, worldZ), Vector3.down);
            RaycastHit hit;
            if (surfaceMeshCollider.Raycast(ray, out hit, 1.0f))
            {
                worldY = hit.point.y;
                normalWorld = hit.normal;
                if (normalWorld.y < 0f) normalWorld = -normalWorld;
                return true;
            }
        }

        // Ưu tiên 2: Tra cứu Elevation Grid
        if (elevationGridY == null || elevationGridValid == null)
        {
            RebuildSurfaceElevationGrid();
        }
        if (elevationGridY == null) return false;

        int gx = Mathf.Clamp(Mathf.RoundToInt((worldX - gridMin.x) / (gridMax.x - gridMin.x) * (GRID_RES - 1)), 0, GRID_RES - 1);
        int gz = Mathf.Clamp(Mathf.RoundToInt((worldZ - gridMin.z) / (gridMax.z - gridMin.z) * (GRID_RES - 1)), 0, GRID_RES - 1);

        if (elevationGridValid[gx, gz])
        {
            worldY = elevationGridY[gx, gz];
        }
        else
        {
            float sumY = 0f;
            float weightSum = 0f;
            for (int dx = -3; dx <= 3; dx++)
            {
                for (int dz = -3; dz <= 3; dz++)
                {
                    int nx = gx + dx;
                    int nz = gz + dz;
                    if (nx >= 0 && nx < GRID_RES && nz >= 0 && nz < GRID_RES && elevationGridValid[nx, nz])
                    {
                        float w = 1f / (1f + dx * dx + dz * dz);
                        sumY += elevationGridY[nx, nz] * w;
                        weightSum += w;
                    }
                }
            }

            if (weightSum <= 0f) return false;
            worldY = sumY / weightSum;
        }

        // Ước lượng độ dốc vi mô để tính pháp tuyến bề mặt vải 3D
        float cellW = (gridMax.x - gridMin.x) / GRID_RES;
        float cellD = (gridMax.z - gridMin.z) / GRID_RES;
        float yL = (gx > 1 && elevationGridValid[gx - 1, gz]) ? elevationGridY[gx - 1, gz] : worldY;
        float yR = (gx < GRID_RES - 2 && elevationGridValid[gx + 1, gz]) ? elevationGridY[gx + 1, gz] : worldY;
        float yD = (gz > 1 && elevationGridValid[gx, gz - 1]) ? elevationGridY[gx, gz - 1] : worldY;
        float yU = (gz < GRID_RES - 2 && elevationGridValid[gx, gz + 1]) ? elevationGridY[gx, gz + 1] : worldY;

        Vector3 tx = new Vector3(2f * cellW, yR - yL, 0f).normalized;
        Vector3 tz = new Vector3(0f, yU - yD, 2f * cellD).normalized;
        normalWorld = Vector3.Cross(tz, tx).normalized;
        if (normalWorld.y < 0f) normalWorld = -normalWorld;
        return true;
    }

    public void InjectSurfacePoint(float worldX, float worldY, float worldZ, Color32? color = null)
    {
        if (elevationGridY == null || elevationGridValid == null) RebuildSurfaceElevationGrid();
        if (elevationGridY == null) return;
        int gx = Mathf.Clamp(Mathf.RoundToInt((worldX - gridMin.x) / (gridMax.x - gridMin.x) * (GRID_RES - 1)), 0, GRID_RES - 1);
        int gz = Mathf.Clamp(Mathf.RoundToInt((worldZ - gridMin.z) / (gridMax.z - gridMin.z) * (GRID_RES - 1)), 0, GRID_RES - 1);
        elevationGridY[gx, gz] = worldY;
        elevationGridValid[gx, gz] = true;
        if (color.HasValue && elevationGridColor != null)
        {
            elevationGridColor[gx, gz] = color.Value;
        }
    }

    private void CreateChunkObject(List<Vector3> verts, List<Color32> colors, int index)
    {
        GameObject chunk = new GameObject($"CloudChunk_{index}");
        chunk.transform.SetParent(this.transform, false);
        chunk.transform.localPosition = Vector3.zero;
        chunk.transform.localRotation = Quaternion.identity;
        chunk.transform.localScale = Vector3.one;

        MeshFilter mf = chunk.AddComponent<MeshFilter>();
        MeshRenderer mr = chunk.AddComponent<MeshRenderer>();
        mr.sharedMaterial = pointCloudMaterial;

        Mesh mesh = new Mesh();
        mesh.name = $"ChunkMesh_{index}";
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.SetVertices(verts);
        mesh.SetColors(colors);

        int[] indices = new int[verts.Count];
        for (int i = 0; i < verts.Count; i++) indices[i] = i;
        mesh.SetIndices(indices, MeshTopology.Points, 0);

        mf.sharedMesh = mesh;
        chunkObjects.Add(chunk);
    }

    private void ClearExistingChunks()
    {
        DisableLegacyRootMeshOnSelf();
        // Tách khỏi parent và tắt hiển thị NGAY LẬP TỨC trước khi Destroy để trong Play Mode
        // các chunk/mesh của phôi cũ không còn nằm trong transform ở cùng frame khi nạp phôi mới!
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Transform child = transform.GetChild(i);
            if (child == null) continue;
            if (child.name.StartsWith("CloudChunk_") ||
                child.name == "WorkpieceSurfaceMesh" ||
                child.name == "InpaintedJeansSurface_Mesh")
            {
                child.SetParent(null);
                child.gameObject.SetActive(false);
                if (Application.isPlaying) Destroy(child.gameObject);
                else DestroyImmediate(child.gameObject);
            }
        }
        chunkObjects.Clear();
        surfaceMeshObject = null;
        surfaceMeshCollider = null;
    }

    private string ReadLineASCII(Stream stream)
    {
        List<byte> bytes = new List<byte>();
        int b;
        while ((b = stream.ReadByte()) != -1)
        {
            if (b == '\n') break;
            if (b != '\r') bytes.Add((byte)b);
        }
        if (bytes.Count == 0 && b == -1) return null;
        return System.Text.Encoding.ASCII.GetString(bytes.ToArray());
    }
}
