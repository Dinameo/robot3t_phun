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

    [Header("4. Tùy chọn điểm")]
    [Tooltip("Tải TOÀN BỘ 100% điểm ảnh từ file PLY (Downsample Step = 1)")]
    public bool loadAll100PercentPoints = true;

    [Tooltip("Bước lấy mẫu (1 = 100% toàn bộ điểm, 2 = 50%, ...)")]
    [Range(1, 4)]
    public int downsampleStep = 1;

    [Tooltip("Giới hạn số điểm tối đa mỗi Chunk Mesh")]
    public int maxPointsPerChunk = 500000;

    [Tooltip("Material hiển thị màu Vertex")]
    public Material pointCloudMaterial;

    private List<GameObject> chunkObjects = new List<GameObject>();

    private void Start()
    {
        EnsureMaterial();
        string resolvedPath = ResolvePath(plyFilePath);
        if (File.Exists(resolvedPath))
        {
            LoadFullPointCloud(resolvedPath);
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

    private string ResolvePath(string path)
    {
        if (Path.IsPathRooted(path)) return path;

        string p1 = Path.Combine(Application.dataPath, "..", path);
        if (File.Exists(p1)) return p1;

        string p2 = Path.Combine(Application.dataPath, path.Replace("Assets/", ""));
        if (File.Exists(p2)) return p2;

        return path;
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
                bool isBinary = false;

                // Đọc Header PLY
                while (true)
                {
                    string line = ReadLineASCII(fs);
                    if (string.IsNullOrEmpty(line)) break;
                    line = line.Trim();

                    if (line.StartsWith("format binary_little_endian")) isBinary = true;
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

                bool isZividOrganized = (vertexCount == 2332800);
                bool isPreconverted = filePath.Contains("full_cloud_world");

                List<Vector3> curVerts = new List<Vector3>(maxPointsPerChunk);
                List<Color32> curColors = new List<Color32>(maxPointsPerChunk);
                int totalLoaded = 0;
                int chunkIdx = 0;

                if (isZividOrganized && !isPreconverted)
                {
                    int h = 1200;
                    int w = 1944;
                    for (int y = 0; y < h; y++)
                    {
                        for (int x = 0; x < w; x++)
                        {
                            float vx = br.ReadSingle();
                            float vy = br.ReadSingle();
                            float vz = br.ReadSingle();
                            byte r = br.ReadByte();
                            byte g = br.ReadByte();
                            byte b = br.ReadByte();

                            if (y % step == 0 && x % step == 0)
                            {
                                if (!float.IsNaN(vz) && vz > 400f && vz < 2500f)
                                {
                                    Vector3 pos = alignWithCameraRig
                                        ? new Vector3(cameraOrigin.x + vx * 0.001f, cameraOrigin.y - vz * 0.001f, cameraOrigin.z - vy * 0.001f)
                                        : new Vector3(vx * 0.001f, vy * 0.001f, vz * 0.001f);

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
                    }
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

                Debug.Log($"[PLY Loader] HOÀN TẤT: Đã hiển thị {totalLoaded:N0} điểm ảnh 3D với 100% màu sắc gốc trên {chunkIdx} Chunks!");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[PLY Loader] Lỗi: {ex.Message}");
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
        // Xóa các chunk cũ trong hierarchy
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Transform child = transform.GetChild(i);
            if (child.name.StartsWith("CloudChunk_"))
            {
                if (Application.isPlaying) Destroy(child.gameObject);
                else DestroyImmediate(child.gameObject);
            }
        }
        chunkObjects.Clear();
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
