using System;
using System.IO;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

/// <summary>
/// CLIENT KẾT NỐI MÁY CHỦ AI PYTHON (HỖ TRỢ CẢ UDP VÀ HTTP REST)
/// Cho phép cấu hình linh hoạt địa chỉ IP và Cổng ngay trên giao diện Unity:
/// 1. Kênh UDP (Mặc định - Khuyên dùng): Tốc độ < 1ms, không độ trễ.
/// 2. Kênh HTTP REST: Chuẩn giao tiếp web API truyền thống.
/// 3. Chế độ Tự Động (Auto Mode): Lắng nghe cổng UDP 5006, model tự động gửi kết quả vào mà không cần bấm nút.
/// </summary>
[ExecuteAlways]
public class AiServerClient : MonoBehaviour
{
    public enum ProtocolType { UDP, HTTP }

    [Header("1. Cấu hình máy chủ AI")]
    [Tooltip("Chọn giao thức kết nối: UDP (Nhanh < 1ms) hoặc HTTP (REST)")]
    public ProtocolType protocol = ProtocolType.UDP;

    [Tooltip("Địa chỉ IP máy chủ AI (Ví dụ: 127.0.0.1 hoặc IP mạng LAN 192.168.1.x)")]
    public string serverIp = "127.0.0.1";

    [Tooltip("Cổng máy chủ: 5005 cho UDP, 5000 cho HTTP")]
    public int serverPort = 5005;

    [Header("2. Trạng thái kết nối")]
    public bool isServerOnline = false;
    public bool isProcessing = false;
    public string lastStatusMessage = "Chưa kết nối máy chủ AI.";
    public float lastExecutionTimeSec = 0f;

    [Header("3. Tham chiếu hệ thống")]
    public DobotCR10AIKController ikController;

    [Header("4. Chế độ Tự Động (Auto Mode - Model tự gửi vào)")]
    [Tooltip("Bật chế độ tự động nhận kết quả từ AI Server đẩy vào qua cổng UDP 5006")]
    public bool autoModeEnabled = false;
    [Tooltip("Cổng UDP Unity lắng nghe kết quả tự động đẩy về")]
    public int autoListenPort = 5006;

    private UdpClient autoListenerClient;
    private Thread autoListenerThread;
    private bool isListeningAuto = false;
    private readonly Queue<string> incomingAutoMessages = new Queue<string>();
    private readonly object queueLock = new object();

    [Serializable]
    public class AiStatusResponse
    {
        public string status;
        public string service;
        public string version;
        public bool model_ready;
        public bool loading;
        public string model_error;
        public string protocol;
    }

    [Serializable]
    public class AiDetectionResult
    {
        public string status;
        public string message;
        public string garment_size;
        public float outseam_mm;
        public float inseam_mm;
        public float waist_mm;
        public float area_cm2;
        public string sample;
        public string pointcloud_ply;
        public int waypoints_count;
        public float perimeter_mm;
        public float time_generation_s;
        public string trajectory_csv;
        public string preview_image;
        public string landmarks_json;
        public string revision;
        public bool simulation_only;
        public bool frame_only;
        public int error_code;

        public bool IsSuccess => status == "SUCCESS" || string.Equals(status, "OK", StringComparison.OrdinalIgnoreCase);
    }

    private void Awake()
    {
        if (ikController == null) ikController = GetComponent<DobotCR10AIKController>() ?? FindAnyObjectByType<DobotCR10AIKController>();
        LoadSettings();
    }

    private void Start()
    {
        CheckServerStatus();
        if (autoModeEnabled) StartAutoListener();
    }

    private void OnEnable()
    {
        if (autoModeEnabled) StartAutoListener();
    }

    private void OnDisable()
    {
        StopAutoListener();
    }

    private void OnDestroy()
    {
        StopAutoListener();
    }

    private void Update()
    {
        // 1. Kiểm tra và đồng bộ trạng thái Listener
        if (autoModeEnabled && !isListeningAuto)
        {
            StartAutoListener();
        }
        else if (!autoModeEnabled && isListeningAuto)
        {
            StopAutoListener();
        }

        // 2. Lấy dữ liệu đẩy tự động từ luồng UDP vào main thread Unity
        if (incomingAutoMessages.Count > 0)
        {
            List<string> msgsToProcess = new List<string>();
            lock (queueLock)
            {
                while (incomingAutoMessages.Count > 0)
                    msgsToProcess.Add(incomingAutoMessages.Dequeue());
            }

            foreach (string msg in msgsToProcess)
            {
                ProcessIncomingAutoJson(msg);
            }
        }
    }

    public void SetAutoMode(bool enabled)
    {
        autoModeEnabled = enabled;
        PlayerPrefs.SetInt("CR10A_AI_AutoMode", autoModeEnabled ? 1 : 0);
        PlayerPrefs.Save();
        if (autoModeEnabled) StartAutoListener();
        else StopAutoListener();
    }

    public void StartAutoListener()
    {
        if (isListeningAuto) return;
        try
        {
            autoListenerClient = new UdpClient();
            autoListenerClient.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            autoListenerClient.Client.Bind(new IPEndPoint(IPAddress.Any, autoListenPort));
            autoListenerClient.Client.ReceiveTimeout = 1000;
            isListeningAuto = true;

            autoListenerThread = new Thread(AutoListenWorker)
            {
                IsBackground = true,
                Name = "Unity_AI_AutoListener"
            };
            autoListenerThread.Start();
            Debug.Log($"[AI Client] Đã kích hoạt Chế Độ Tự Động - Đang lắng nghe gói tin AI đẩy vào tại cổng UDP {autoListenPort}...");
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[AI Client] Không thể mở cổng UDP {autoListenPort} cho chế độ tự động: {ex.Message}");
            isListeningAuto = false;
        }
    }

    public void StopAutoListener()
    {
        isListeningAuto = false;
        if (autoListenerClient != null)
        {
            try { autoListenerClient.Close(); } catch { }
            autoListenerClient = null;
        }
        if (autoListenerThread != null && autoListenerThread.IsAlive)
        {
            try { autoListenerThread.Abort(); } catch { }
            autoListenerThread = null;
        }
    }

    private void AutoListenWorker()
    {
        while (isListeningAuto && autoListenerClient != null)
        {
            try
            {
                IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = autoListenerClient.Receive(ref remoteEP);
                if (data != null && data.Length > 0)
                {
                    string text = Encoding.UTF8.GetString(data).Trim();
                    lock (queueLock)
                    {
                        incomingAutoMessages.Enqueue(text);
                    }
                }
            }
            catch (SocketException)
            {
                // Timeout bình thường
            }
            catch (ThreadAbortException)
            {
                break;
            }
            catch (Exception ex)
            {
                if (isListeningAuto)
                {
                    Debug.LogWarning($"[AI Auto Listener] Lỗi: {ex.Message}");
                }
            }
        }
    }

    private void ProcessIncomingAutoJson(string jsonText)
    {
        if (string.IsNullOrEmpty(jsonText)) return;
        try
        {
            AiDetectionResult result = JsonUtility.FromJson<AiDetectionResult>(jsonText);
            if (result != null && result.IsSuccess)
            {
                lastStatusMessage = $"🟢 [TỰ ĐỘNG] Model vừa đẩy mẫu [{result.sample}] ({result.waypoints_count} điểm)!";
                ProcessAiResult(result, null);
                Debug.Log($"[AI Auto Receive] Đã tự động cập nhật phôi & quỹ đạo cho mẫu [{result.sample}]!");
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[AI Auto Receive] Lỗi phân tích gói tin JSON: {ex.Message}");
        }
    }

    public void SaveSettings()
    {
        PlayerPrefs.SetString("CR10A_AI_Server_IP", serverIp);
        PlayerPrefs.SetInt("CR10A_AI_Server_Port", serverPort);
        PlayerPrefs.SetInt("CR10A_AI_Protocol", (int)protocol);
        PlayerPrefs.SetInt("CR10A_AI_AutoMode", autoModeEnabled ? 1 : 0);
        PlayerPrefs.Save();
    }

    public void LoadSettings()
    {
        serverIp = PlayerPrefs.GetString("CR10A_AI_Server_IP", "127.0.0.1");
        int defaultPort = (protocol == ProtocolType.UDP) ? 5005 : 5000;
        serverPort = PlayerPrefs.GetInt("CR10A_AI_Server_Port", defaultPort);
        protocol = (ProtocolType)PlayerPrefs.GetInt("CR10A_AI_Protocol", (int)ProtocolType.UDP);
        autoModeEnabled = PlayerPrefs.GetInt("CR10A_AI_AutoMode", 0) == 1;
    }

    public void SetProtocol(ProtocolType newProtocol)
    {
        protocol = newProtocol;
        if (protocol == ProtocolType.UDP && serverPort == 5000) serverPort = 5005;
        if (protocol == ProtocolType.HTTP && serverPort == 5005) serverPort = 5000;
        SaveSettings();
    }

    // =========================================================================
    // 1. KIỂM TRA TRẠNG THÁI SERVER (PING STATUS)
    // =========================================================================

    public void CheckServerStatus(Action<bool, string> onComplete = null)
    {
        SaveSettings();
        if (protocol == ProtocolType.UDP)
        {
            StartCoroutine(PingUdpRoutine(onComplete));
        }
        else
        {
            StartCoroutine(PingHttpRoutine(onComplete));
        }
    }

    private IEnumerator PingUdpRoutine(Action<bool, string> onComplete)
    {
        bool success = false;
        string msg = "";
        string host = serverIp.Trim();
        int port = serverPort;

        Task<string> udpTask = Task.Run(() =>
        {
            try
            {
                using (UdpClient client = new UdpClient())
                {
                    client.Client.ReceiveTimeout = 2000; // 2 giây
                    byte[] pingBytes = Encoding.UTF8.GetBytes("PING");
                    client.Send(pingBytes, pingBytes.Length, host, port);

                    IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);
                    byte[] received = client.Receive(ref remoteEP);
                    return Encoding.UTF8.GetString(received);
                }
            }
            catch (Exception ex)
            {
                return "ERROR:" + (ex is SocketException se && se.SocketErrorCode == SocketError.ConnectionReset
                    ? "Không có AI Server nhận UDP tại " + host + ":" + port + ". Mở START_AI_SERVER.cmd rồi bấm Kiểm Tra."
                    : ex.Message);
            }
        });

        float timer = 0f;
        while (!udpTask.IsCompleted && timer < 2.5f)
        {
            timer += Time.unscaledDeltaTime;
            yield return null;
        }

        if (udpTask.IsCompleted && !udpTask.Result.StartsWith("ERROR:"))
        {
            AiStatusResponse status = null;
            try { status = JsonUtility.FromJson<AiStatusResponse>(udpTask.Result); } catch { }
            success = status != null && status.model_ready;
            isServerOnline = success;
            msg = status == null ? "Phản hồi server không hợp lệ" : !status.model_ready ?
                (status.loading ? "Server đang nạp model AI; chờ rồi kiểm tra lại." : "Lỗi nạp model: " + status.model_error) : $"🟢 AI Server UDP trực tuyến ({host}:{port})";
        }
        else
        {
            success = false;
            isServerOnline = false;
            msg = $"🔴 Không phản hồi UDP tại {host}:{port}. Hãy chạy run_ai_server.bat!";
        }

        lastStatusMessage = msg;
        onComplete?.Invoke(success, msg);
    }

    private IEnumerator PingHttpRoutine(Action<bool, string> onComplete)
    {
        string url = $"http://{serverIp.Trim()}:{serverPort}/api/status";
        using (UnityWebRequest req = UnityWebRequest.Get(url))
        {
            req.timeout = 2;
            yield return req.SendWebRequest();

            if (req.result == UnityWebRequest.Result.Success)
            {
                AiStatusResponse status = null;
                try { status = JsonUtility.FromJson<AiStatusResponse>(req.downloadHandler.text); } catch { }
                isServerOnline = status != null && status.model_ready;
                lastStatusMessage = !isServerOnline ? "Server chưa sẵn sàng: " + (status != null ? status.model_error : "JSON không hợp lệ") : $"🟢 AI Server HTTP trực tuyến ({serverIp}:{serverPort})";
                onComplete?.Invoke(isServerOnline, lastStatusMessage);
            }
            else
            {
                isServerOnline = false;
                lastStatusMessage = $"🔴 Không kết nối được HTTP {serverIp}:{serverPort} ({req.error})";
                onComplete?.Invoke(false, lastStatusMessage);
            }
        }
    }

    // =========================================================================
    // 2. KÍCH HOẠT NHẬN DẠNG & SINH QUỸ ĐẠO (TRIGGER DETECTION)
    // =========================================================================

    [Serializable]
    private class DetectionRequest
    {
        public string command;
        public string size;
        public string sample;
        public string sample_path;
    }

    public void CaptureAndDetect()
    {
        if (isProcessing) return;
        StartCoroutine(CaptureAndDetectRoutine(false));
    }

    public void CapturePreview()
    {
        if (isProcessing) return;
        StartCoroutine(CaptureAndDetectRoutine(true));
    }

    private IEnumerator CaptureAndDetectRoutine(bool previewOnly)
    {
        isProcessing = true;
        lastStatusMessage = "Đang chụp Zivid và xử lý RGB / XYZ...";
        string payload = previewOnly ? "{\"command\":\"CAPTURE_PREVIEW\"}" :
            "{\"command\":\"CAPTURE_DETECT\",\"preview_nominal_rig\":true}";
        // Camera capture uses HTTP for clear errors and the longer acquisition timeout.
        string url = $"http://{serverIp.Trim()}:5000/api/capture";
        using (UnityWebRequest request = new UnityWebRequest(url, "POST"))
        {
            request.uploadHandler = new UploadHandlerRaw(Encoding.UTF8.GetBytes(payload));
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/json");
            request.timeout = 180;
            yield return request.SendWebRequest();
            isProcessing = false;
            if (request.result != UnityWebRequest.Result.Success)
            {
                lastStatusMessage = "Không kết nối được camera server: " + request.error;
                yield break;
            }
            AiDetectionResult result = null;
            try { result = JsonUtility.FromJson<AiDetectionResult>(request.downloadHandler.text); }
            catch (Exception ex) { lastStatusMessage = "Phản hồi camera không hợp lệ: " + ex.Message; }
            if (result != null) ProcessAiResult(result, null);
        }
    }

    public void TriggerDetection(string size = "S", Action<AiDetectionResult> onComplete = null)
    {
        string currentSample = ikController != null ? ikController.selectedSampleName : "pcl_101";
        TriggerDetectionWithSample(size, currentSample, onComplete);
    }

    public void TriggerDetectionWithSample(string size = "S", string sample = "pcl_101", Action<AiDetectionResult> onComplete = null)
    {
        if (isProcessing) return;
        SaveSettings();

        if (string.IsNullOrEmpty(sample)) sample = "pcl_101";

        if (protocol == ProtocolType.UDP)
        {
            StartCoroutine(TriggerUdpRoutine(size, sample, onComplete));
        }
        else
        {
            StartCoroutine(TriggerHttpRoutine(size, sample, onComplete));
        }
    }

    /// <summary>
    /// Gửi danh sách nét vẽ trên Quần 2D Tiêu Chuẩn [0, 1] x [0, 1] sang AI Server để nội suy TPS 12 mốc
    /// và chiếu lên bề mặt thực 3D Zivid Point Cloud (X, Y, Z, pháp tuyến)
    /// </summary>
    public void SendCustomDraw3D(string sample, List<List<Vector2>> strokesNorm, float zOffsetMm = 22.0f, Action<AiDetectionResult> onComplete = null)
    {
        if (isProcessing || strokesNorm == null || strokesNorm.Count == 0) return;
        SaveSettings();
        if (string.IsNullOrEmpty(sample)) sample = ikController != null ? ikController.selectedSampleName : "pcl_101";

        StringBuilder sb = new StringBuilder();
        string baseJson = JsonUtility.ToJson(new DetectionRequest { command = "CUSTOM_DRAW_3D", sample = sample,
            sample_path = ikController != null && File.Exists(ikController.customPlyPath) ? Path.GetFullPath(ikController.customPlyPath) : "" });
        sb.Append(baseJson.Substring(0, baseJson.Length - 1)).Append(",");
        sb.Append("\"z_offset_mm\":").Append(zOffsetMm.ToString("F2", System.Globalization.CultureInfo.InvariantCulture)).Append(",\"strokes\":[");
        for (int s = 0; s < strokesNorm.Count; s++)
        {
            if (s > 0) sb.Append(",");
            sb.Append("[");
            var st = strokesNorm[s];
            for (int p = 0; p < st.Count; p++)
            {
                if (p > 0) sb.Append(",");
                sb.Append("{\"u\":").Append(st[p].x.ToString("F4", System.Globalization.CultureInfo.InvariantCulture))
                  .Append(",\"v\":").Append(st[p].y.ToString("F4", System.Globalization.CultureInfo.InvariantCulture))
                  .Append("}");
            }
            sb.Append("]");
        }
        sb.Append("]}");

        StartCoroutine(SendCustomDrawUdpOrHttpRoutine(sb.ToString(), sample, onComplete));
    }

    private IEnumerator SendCustomDrawUdpOrHttpRoutine(string jsonPayload, string sample, Action<AiDetectionResult> onComplete)
    {
        isProcessing = true;
        lastStatusMessage = $"🟡 Đang ánh xạ nét vẽ 2D -> 3D Point Cloud trên mẫu [{sample}]...";
        string host = serverIp.Trim();
        int port = serverPort;

        if (protocol == ProtocolType.HTTP)
        {
            using (UnityWebRequest request = new UnityWebRequest($"http://{host}:{port}/api/custom_draw_3d", "POST"))
            {
                request.uploadHandler = new UploadHandlerRaw(Encoding.UTF8.GetBytes(jsonPayload));
                request.downloadHandler = new DownloadHandlerBuffer();
                request.SetRequestHeader("Content-Type", "application/json");
                request.timeout = 180;
                yield return request.SendWebRequest();
                isProcessing = false;
                if (request.result != UnityWebRequest.Result.Success)
                {
                    lastStatusMessage = "Lỗi HTTP: " + request.error;
                    onComplete?.Invoke(null);
                    yield break;
                }
                AiDetectionResult result = null;
                try { result = JsonUtility.FromJson<AiDetectionResult>(request.downloadHandler.text); }
                catch (Exception ex) { lastStatusMessage = "JSON không hợp lệ: " + ex.Message; }
                if (result != null) ProcessAiResult(result, onComplete);
                yield break;
            }
        }

        Task<string> netTask = Task.Run(() =>
        {
            try
            {
                using (UdpClient client = new UdpClient())
                {
                    client.Client.ReceiveTimeout = 180000;
                    byte[] data = Encoding.UTF8.GetBytes(jsonPayload);
                    client.Send(data, data.Length, host, (protocol == ProtocolType.UDP) ? port : 5005);
                    IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);
                    byte[] received = client.Receive(ref remoteEP);
                    return Encoding.UTF8.GetString(received);
                }
            }
            catch (Exception ex)
            {
                return "ERROR:" + (ex is SocketException se && se.SocketErrorCode == SocketError.ConnectionReset
                    ? "Không có AI Server nhận UDP tại " + host + ":" + port + ". Mở START_AI_SERVER.cmd rồi bấm Kiểm Tra."
                    : ex.Message);
            }
        });

        while (!netTask.IsCompleted)
        {
            yield return null;
        }

        isProcessing = false;
        string resultText = netTask.Result;
        if (!resultText.StartsWith("ERROR:"))
        {
            isServerOnline = true;
            AiDetectionResult result = JsonUtility.FromJson<AiDetectionResult>(resultText);
            ProcessAiResult(result, onComplete);
        }
        else
        {
            onComplete?.Invoke(null);
        }
    }

    private IEnumerator TriggerUdpRoutine(string size, string sample, Action<AiDetectionResult> onComplete)
    {
        isProcessing = true;
        lastStatusMessage = $"🟡 [UDP] Đang gửi lệnh tới {serverIp}:{serverPort} kích hoạt AI mẫu {sample} (cỡ {size})...";

        string host = serverIp.Trim();
        int port = serverPort;
        string jsonCmd = JsonUtility.ToJson(new DetectionRequest {
            command = "TRIGGER_DETECT", size = size, sample = sample,
            sample_path = ikController != null && File.Exists(ikController.customPlyPath) ? Path.GetFullPath(ikController.customPlyPath) : ""
        });

        Task<string> udpTask = Task.Run(() =>
        {
            try
            {
                using (UdpClient client = new UdpClient())
                {
                    client.Client.ReceiveTimeout = 180000; // Timeout 60s theo chỉ tiêu t_gen <= 60s
                    byte[] data = Encoding.UTF8.GetBytes(jsonCmd);
                    client.Send(data, data.Length, host, port);

                    IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);
                    byte[] received = client.Receive(ref remoteEP);
                    return Encoding.UTF8.GetString(received);
                }
            }
            catch (Exception ex)
            {
                return "ERROR:" + (ex is SocketException se && se.SocketErrorCode == SocketError.ConnectionReset
                    ? "Không có AI Server nhận UDP tại " + host + ":" + port + ". Mở START_AI_SERVER.cmd rồi bấm Kiểm Tra."
                    : ex.Message);
            }
        });

        while (!udpTask.IsCompleted)
        {
            yield return null;
        }

        isProcessing = false;
        string resultText = udpTask.Result;

        if (!resultText.StartsWith("ERROR:"))
        {
            isServerOnline = true;
            AiDetectionResult result = null;
            try
            {
                result = JsonUtility.FromJson<AiDetectionResult>(resultText);
            }
            catch (Exception ex)
            {
                lastStatusMessage = $"Lỗi đọc JSON từ UDP: {ex.Message}";
            }

            ProcessAiResult(result, onComplete);
        }
        else
        {
            isServerOnline = false;
            lastStatusMessage = $"🔴 Lỗi gửi UDP: {resultText.Substring(6)}";
            onComplete?.Invoke(null);
        }
    }

    private IEnumerator TriggerHttpRoutine(string size, string sample, Action<AiDetectionResult> onComplete)
    {
        isProcessing = true;
        lastStatusMessage = $"🟡 [HTTP] Đang kích hoạt AI nhận dạng phôi mẫu {sample} (cỡ {size})...";

        string url = $"http://{serverIp.Trim()}:{serverPort}/api/detect";
        string jsonBody = JsonUtility.ToJson(new DetectionRequest {
            command = "TRIGGER_DETECT", size = size, sample = sample,
            sample_path = ikController != null && File.Exists(ikController.customPlyPath) ? Path.GetFullPath(ikController.customPlyPath) : ""
        });

        using (UnityWebRequest req = new UnityWebRequest(url, "POST"))
        {
            byte[] bodyRaw = Encoding.UTF8.GetBytes(jsonBody);
            req.uploadHandler = new UploadHandlerRaw(bodyRaw);
            req.downloadHandler = new DownloadHandlerBuffer();
            req.SetRequestHeader("Content-Type", "application/json");
            req.timeout = 180;

            yield return req.SendWebRequest();
            isProcessing = false;

            if (req.result == UnityWebRequest.Result.Success)
            {
                isServerOnline = true;
                AiDetectionResult result = null;
                try
                {
                    result = JsonUtility.FromJson<AiDetectionResult>(req.downloadHandler.text);
                }
                catch (Exception ex)
                {
                    lastStatusMessage = $"Lỗi đọc JSON từ HTTP: {ex.Message}";
                }
                ProcessAiResult(result, onComplete);
            }
            else
            {
                isServerOnline = false;
                lastStatusMessage = $"🔴 Lỗi kết nối HTTP: {req.error}";
                onComplete?.Invoke(null);
            }
        }
    }

    private void ProcessAiResult(AiDetectionResult result, Action<AiDetectionResult> onComplete)
    {
        if (result != null && result.IsSuccess)
        {
            isServerOnline = true;
            lastExecutionTimeSec = result.time_generation_s;
            string sName = !string.IsNullOrEmpty(result.sample) ? result.sample : "Phôi";
            string detSize = !string.IsNullOrEmpty(result.garment_size) && result.garment_size != "CUSTOM_3D" ? result.garment_size : "";

            if (ikController != null)
            {
                ikController.StopPlaybackForNewFrame();
                if (result.frame_only)
                {
                    ikController.SelectSampleByFullPath(result.pointcloud_ply);
                    RobotCameraController camera = FindAnyObjectByType<RobotCameraController>();
                    if (camera != null) camera.FocusGarment();
                    lastStatusMessage = "Đã chụp camera thật: " + result.sample + ". Chỉ xem phôi; chưa có quỹ đạo.";
                    onComplete?.Invoke(result);
                    return;
                }
                if (!string.IsNullOrEmpty(result.sample)) ikController.selectedSampleName = result.sample;
                if (!string.IsNullOrEmpty(result.pointcloud_ply))
                {
                    ikController.customPlyPath = result.pointcloud_ply;
                    ikController.ReloadPointCloudFromPath(result.pointcloud_ply);
                }
                ikController.LoadLandmarksAndSegmentationForSample(result.sample, false, result.landmarks_json);
                if (!string.IsNullOrEmpty(result.trajectory_csv))
                {
                    ikController.LoadNamedTrajectory(result.trajectory_csv);
                    ikController.RunDiagnostics();
                }
            }

            if (string.IsNullOrEmpty(detSize)) detSize = "S";
            lastStatusMessage = $"✓ AI nhận diện [{sName}] ➔ SIZE QUẦN: {detSize} | Tự động đổi Mẫu Phun sang SIZE {detSize} ({result.waypoints_count} điểm, {result.time_generation_s:F2}s)";
            onComplete?.Invoke(result);
        }
        else
        {
            lastStatusMessage = result != null ? $"Lỗi AI: {result.message}" : "Không nhận được phản hồi hợp lệ từ AI.";
            onComplete?.Invoke(null);
        }
    }
}
