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

    [Header("5. Sửa mốc AI & Tự động Train lại (Active Learning)")]
    public int feedbackPendingCount = 0;
    public int feedbackRetrainThreshold = 10;
    public int feedbackHistoryCount = 0;
    public int feedbackTotalRetrains = 0;
    public bool isRetrainingPose = false;
    public string feedbackStatusMessage = "";
    private float nextFeedbackPollTime = 0f;

    private UdpClient autoListenerClient;
    private Thread autoListenerThread;
    private bool isListeningAuto = false;
    private readonly Queue<string> incomingAutoMessages = new Queue<string>();
    private readonly object queueLock = new object();

    [Serializable]
    public class FeedbackStatusResponse
    {
        public string status;
        public string message;
        public int threshold;
        public int pending_count;
        public int history_count;
        public int total_retrains;
        public bool is_retraining;
        public bool ready_to_retrain;
        public bool retrain_triggered;
        public string last_retrain_time;
        public string last_retrain_status;
        public string status_message;
    }

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
        public string camera_image;
        public string landmarks_json;
        public string revision;
        public bool simulation_only;
        public bool frame_only;
        public int error_code;

        public bool IsSuccess => status == "SUCCESS" || string.Equals(status, "OK", StringComparison.OrdinalIgnoreCase);
    }

    private bool hasLoadedSettings = false;

    private void Awake()
    {
        if (ikController == null) ikController = GetComponent<DobotCR10AIKController>() ?? FindAnyObjectByType<DobotCR10AIKController>();
        LoadSettings();
    }

    private void Start()
    {
        if (!hasLoadedSettings) LoadSettings();
        LoadLocalFeedbackManifest();
        CheckServerStatus();
        RefreshFeedbackStatus();
        if (autoModeEnabled) StartAutoListener();
    }

    private void OnEnable()
    {
        if (!hasLoadedSettings) LoadSettings();
        LoadLocalFeedbackManifest();
        if (autoModeEnabled) StartAutoListener();
    }

    private void OnDisable()
    {
        if (Application.isPlaying && hasLoadedSettings) SaveSettings();
        StopAutoListener();
    }

    private void OnDestroy()
    {
        if (Application.isPlaying && hasLoadedSettings) SaveSettings();
        StopAutoListener();
    }

    private void OnApplicationQuit()
    {
        if (hasLoadedSettings) SaveSettings();
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

        // 3. Tự động cập nhật tiến độ khi đang train lại YOLO11-pose ở nền
        if (Application.isPlaying && isRetrainingPose && Time.unscaledTime >= nextFeedbackPollTime)
        {
            nextFeedbackPollTime = Time.unscaledTime + 4.0f;
            RefreshFeedbackStatus();
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
                lastStatusMessage = $"[Tự động] Nhận mẫu {result.sample} ({result.waypoints_count} điểm)";
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
        PlayerPrefs.SetInt("CR10A_AI_AutoListenPort", autoListenPort);
        PlayerPrefs.Save();
    }

    public void LoadSettings()
    {
        serverIp = PlayerPrefs.GetString("CR10A_AI_Server_IP", "127.0.0.1");
        protocol = (ProtocolType)PlayerPrefs.GetInt("CR10A_AI_Protocol", (int)ProtocolType.UDP);
        int defaultPort = (protocol == ProtocolType.UDP) ? 5005 : 5000;
        serverPort = PlayerPrefs.GetInt("CR10A_AI_Server_Port", defaultPort);
        autoModeEnabled = PlayerPrefs.GetInt("CR10A_AI_AutoMode", 0) == 1;
        autoListenPort = PlayerPrefs.GetInt("CR10A_AI_AutoListenPort", 5006);
        hasLoadedSettings = true;
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
                (status.loading ? "Server đang nạp model AI..." : "Lỗi nạp model: " + status.model_error) : $"Server UDP trực tuyến ({host}:{port})";
        }
        else
        {
            success = false;
            isServerOnline = false;
            msg = $"Không phản hồi UDP ({host}:{port}). Chạy START_AI_SERVER.cmd.";
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
                lastStatusMessage = !isServerOnline ? "Server chưa sẵn sàng: " + (status != null ? status.model_error : "JSON không hợp lệ") : $"Server HTTP trực tuyến ({serverIp}:{serverPort})";
                onComplete?.Invoke(isServerOnline, lastStatusMessage);
            }
            else
            {
                isServerOnline = false;
                lastStatusMessage = $"Lỗi kết nối HTTP {serverIp}:{serverPort} ({req.error})";
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
        lastStatusMessage = $"Đang ánh xạ 2D -> 3D [{sample}]...";
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
        lastStatusMessage = $"[UDP] Đang nhận diện {sample} (Size {size})...";

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
            lastStatusMessage = $"Lỗi UDP: {resultText.Substring(6)}";
            onComplete?.Invoke(null);
        }
    }

    private IEnumerator TriggerHttpRoutine(string size, string sample, Action<AiDetectionResult> onComplete)
    {
        isProcessing = true;
        lastStatusMessage = $"[HTTP] Đang nhận diện {sample} (Size {size})...";

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
                lastStatusMessage = $"Lỗi HTTP: {req.error}";
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
                    string camImg = !string.IsNullOrEmpty(result.camera_image) ? result.camera_image : result.preview_image;
                    if (!string.IsNullOrEmpty(camImg))
                    {
                        ikController.SetRealCameraImageFromPath(camImg, result.sample);
                    }
                    else
                    {
                        ikController.RefreshRealCameraImageForCurrentSample();
                    }
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
                if (!string.IsNullOrEmpty(result.camera_image))
                {
                    ikController.SetRealCameraImageFromPath(result.camera_image, result.sample);
                }
                else
                {
                    ikController.RefreshRealCameraImageForCurrentSample();
                }

                bool isCustom3dDrawRequest = (result.garment_size == "CUSTOM_3D");
                if (!isCustom3dDrawRequest)
                {
                    // Nhận diện AI xong chỉ nạp 12 mốc & đường bao Segment (hiển thị theo công tắc View), KHÔNG tự sinh/hiện đường quỹ đạo
                    ikController.InvalidateGeneratedTrajectoryOnEdit();
                    ikController.LoadLandmarksAndSegmentationForSample(result.sample, autoSwitchSprayTemplate: true, exactJsonPath: result.landmarks_json, generateTrajectory: false);
                }
                else
                {
                    ikController.LoadLandmarksAndSegmentationForSample(result.sample, autoSwitchSprayTemplate: false, exactJsonPath: result.landmarks_json, generateTrajectory: false);
                    if (!string.IsNullOrEmpty(result.trajectory_csv))
                    {
                        ikController.LoadNamedTrajectory(result.trajectory_csv);
                    }
                    ikController.RunDiagnostics();
                }
            }

            if (string.IsNullOrEmpty(detSize)) detSize = "S";
            lastStatusMessage = result.garment_size == "CUSTOM_3D"
                ? $"Ánh xạ 3D [{sName}]: {result.waypoints_count} điểm ({result.time_generation_s:F2}s)"
                : $"Đã nhận diện AI [{sName}]: Size {detSize} - 12 mốc & Segment ({result.time_generation_s:F2}s)";
            onComplete?.Invoke(result);
        }
        else
        {
            lastStatusMessage = result != null ? $"Lỗi AI: {result.message}" : "Không nhận được phản hồi hợp lệ từ AI.";
            onComplete?.Invoke(null);
        }
    }

    // =========================================================================
    // 3. HỌC CHỦ ĐỘNG: LƯU MẪU SỬA 12 MỐC AI & TỰ ĐỘNG TRAIN LẠI KHI ĐỦ 10 MẪU
    // =========================================================================

    private string GetFeedbackRootDir()
    {
        return Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "ai_src", "step2_train_yolo", "active_learning_feedback"));
    }

    private string GetRetrainScriptPath()
    {
        return Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "ai_src", "step2_train_yolo", "retrain_pose_feedback.py"));
    }

    private string ResolvePythonExePath()
    {
        string[] candidates = new string[]
        {
            @"D:\Program Files\miniconda3\envs\env\python.exe",
            Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), @"miniconda3\envs\env\python.exe"),
            Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), @"anaconda3\envs\env\python.exe"),
            "python"
        };
        for (int i = 0; i < candidates.Length; i++)
        {
            if (candidates[i] == "python" || File.Exists(candidates[i]))
                return candidates[i];
        }
        return "python";
    }

    private static string SanitizeSampleStem(string sampleName)
    {
        string raw = string.IsNullOrEmpty(sampleName) ? "sample" : Path.GetFileNameWithoutExtension(sampleName).Trim();
        StringBuilder sb = new StringBuilder(raw.Length);
        for (int i = 0; i < raw.Length; i++)
        {
            char c = raw[i];
            if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-')
                sb.Append(c);
            else
                sb.Append('_');
        }
        string cleaned = sb.ToString().Trim('_');
        if (string.IsNullOrEmpty(cleaned)) cleaned = "sample_" + DateTime.Now.ToString("yyyyMMdd_HHmmss");
        return cleaned.Length > 96 ? cleaned.Substring(0, 96) : cleaned;
    }

    private static byte[] EncodeTextureToJpgSafe(Texture2D tex)
    {
        if (tex == null || tex.width <= 0 || tex.height <= 0) return null;
        try
        {
            if (tex.isReadable)
            {
                return tex.EncodeToJPG(95);
            }
        }
        catch { }

        RenderTexture rt = RenderTexture.GetTemporary(tex.width, tex.height, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        RenderTexture prev = RenderTexture.active;
        try
        {
            Graphics.Blit(tex, rt);
            RenderTexture.active = rt;
            Texture2D copy = new Texture2D(tex.width, tex.height, TextureFormat.RGB24, false);
            copy.ReadPixels(new Rect(0, 0, tex.width, tex.height), 0, 0);
            copy.Apply();
            byte[] bytes = copy.EncodeToJPG(95);
            if (Application.isPlaying) Destroy(copy);
            else DestroyImmediate(copy);
            return bytes;
        }
        catch (Exception ex)
        {
            Debug.LogWarning("[AI Feedback] Không thể mã hóa ảnh camera sang JPG: " + ex.Message);
            return null;
        }
        finally
        {
            RenderTexture.active = prev;
            RenderTexture.ReleaseTemporary(rt);
        }
    }

    private void ApplyFeedbackResponse(FeedbackStatusResponse resp)
    {
        if (resp == null) return;
        if (resp.threshold > 0) feedbackRetrainThreshold = resp.threshold;
        feedbackPendingCount = Mathf.Max(0, resp.pending_count);
        feedbackHistoryCount = Mathf.Max(0, resp.history_count);
        feedbackTotalRetrains = Mathf.Max(0, resp.total_retrains);
        isRetrainingPose = resp.is_retraining;
        string msg = !string.IsNullOrEmpty(resp.status_message) ? resp.status_message : resp.message;
        if (!string.IsNullOrEmpty(msg)) feedbackStatusMessage = msg;
    }

    public void LoadLocalFeedbackManifest()
    {
        try
        {
            string rootDir = GetFeedbackRootDir();
            string manifestPath = Path.Combine(rootDir, "feedback_manifest.json");
            if (File.Exists(manifestPath))
            {
                string json = File.ReadAllText(manifestPath, Encoding.UTF8);
                FeedbackStatusResponse resp = JsonUtility.FromJson<FeedbackStatusResponse>(json);
                ApplyFeedbackResponse(resp);
            }
            string pendingLabelsDir = Path.Combine(rootDir, "pending", "labels");
            if (Directory.Exists(pendingLabelsDir))
            {
                int actualPending = Directory.GetFiles(pendingLabelsDir, "*.txt").Length;
                feedbackPendingCount = actualPending;
            }
            string historyLabelsDir = Path.Combine(rootDir, "history", "labels");
            if (Directory.Exists(historyLabelsDir))
            {
                feedbackHistoryCount = Directory.GetFiles(historyLabelsDir, "*.txt").Length;
            }
            if (feedbackRetrainThreshold < 1) feedbackRetrainThreshold = 10;
            if (string.IsNullOrEmpty(feedbackStatusMessage))
            {
                feedbackStatusMessage = $"Mẫu sửa mốc chờ train: {feedbackPendingCount}/{feedbackRetrainThreshold}.";
            }
        }
        catch { }
    }

    public void RefreshFeedbackStatus(Action<bool> onComplete = null)
    {
        LoadLocalFeedbackManifest();
        if (!Application.isPlaying || !gameObject.activeInHierarchy)
        {
            onComplete?.Invoke(true);
            return;
        }
        StartCoroutine(RefreshFeedbackStatusRoutine(onComplete));
    }

    private IEnumerator RefreshFeedbackStatusRoutine(Action<bool> onComplete)
    {
        string url = $"http://{serverIp.Trim()}:5000/api/feedback/status";
        using (UnityWebRequest req = UnityWebRequest.Get(url))
        {
            req.timeout = 2;
            yield return req.SendWebRequest();
            if (req.result == UnityWebRequest.Result.Success)
            {
                try
                {
                    FeedbackStatusResponse resp = JsonUtility.FromJson<FeedbackStatusResponse>(req.downloadHandler.text);
                    ApplyFeedbackResponse(resp);
                    onComplete?.Invoke(true);
                    yield break;
                }
                catch { }
            }
        }
        LoadLocalFeedbackManifest();
        onComplete?.Invoke(false);
    }

    public void SubmitCorrectedLandmarksFeedback(
        string sampleName,
        Vector2[] landmarksUV12,
        Texture2D cameraTex,
        string knownImagePath = "",
        bool autoRetrain = true,
        Action<bool, string> onComplete = null)
    {
        if (landmarksUV12 == null || landmarksUV12.Length != 12)
        {
            string err = "Cần đúng 12 mốc tọa độ để lưu mẫu sửa lỗi AI.";
            feedbackStatusMessage = err;
            onComplete?.Invoke(false, err);
            return;
        }

        string safeSample = SanitizeSampleStem(sampleName);
        byte[] jpgBytes = null;
        bool hasKnownImg = !string.IsNullOrEmpty(knownImagePath) && File.Exists(knownImagePath);
        if (!hasKnownImg && cameraTex != null)
        {
            jpgBytes = EncodeTextureToJpgSafe(cameraTex);
        }

        if (!Application.isPlaying || !gameObject.activeInHierarchy)
        {
            SaveFeedbackOfflineAndMaybeRetrain(safeSample, landmarksUV12, knownImagePath, jpgBytes, autoRetrain, onComplete);
            return;
        }

        StartCoroutine(SubmitCorrectedLandmarksRoutine(safeSample, landmarksUV12, knownImagePath, jpgBytes, autoRetrain, onComplete));
    }

    private IEnumerator SubmitCorrectedLandmarksRoutine(
        string safeSample,
        Vector2[] landmarksUV12,
        string knownImagePath,
        byte[] jpgBytes,
        bool autoRetrain,
        Action<bool, string> onComplete)
    {
        var inv = System.Globalization.CultureInfo.InvariantCulture;
        StringBuilder sb = new StringBuilder(1024);
        sb.Append("{\"command\":\"SAVE_LANDMARK_FEEDBACK\",\"sample\":\"").Append(EscapeJsonString(safeSample)).Append("\"");
        sb.Append(",\"threshold\":").Append(Mathf.Max(1, feedbackRetrainThreshold));
        sb.Append(",\"auto_retrain\":").Append(autoRetrain ? "true" : "false");
        if (!string.IsNullOrEmpty(knownImagePath) && File.Exists(knownImagePath))
        {
            sb.Append(",\"image_path\":\"").Append(EscapeJsonString(Path.GetFullPath(knownImagePath))).Append("\"");
        }
        else if (jpgBytes != null && jpgBytes.Length > 0)
        {
            sb.Append(",\"image_b64\":\"").Append(Convert.ToBase64String(jpgBytes)).Append("\"");
        }
        sb.Append(",\"landmarks_uv\":[");
        for (int i = 0; i < 12; i++)
        {
            if (i > 0) sb.Append(",");
            sb.Append("[").Append(Mathf.Clamp01(landmarksUV12[i].x).ToString("F6", inv))
              .Append(",").Append(Mathf.Clamp01(landmarksUV12[i].y).ToString("F6", inv)).Append("]");
        }
        sb.Append("]}");

        string url = $"http://{serverIp.Trim()}:5000/api/feedback/landmarks";
        using (UnityWebRequest req = new UnityWebRequest(url, "POST"))
        {
            req.uploadHandler = new UploadHandlerRaw(Encoding.UTF8.GetBytes(sb.ToString()));
            req.downloadHandler = new DownloadHandlerBuffer();
            req.SetRequestHeader("Content-Type", "application/json");
            req.timeout = 15;
            yield return req.SendWebRequest();

            if (req.result == UnityWebRequest.Result.Success)
            {
                try
                {
                    FeedbackStatusResponse resp = JsonUtility.FromJson<FeedbackStatusResponse>(req.downloadHandler.text);
                    if (resp != null && (resp.status == "SUCCESS" || resp.status == "STARTED"))
                    {
                        ApplyFeedbackResponse(resp);
                        if (isRetrainingPose) nextFeedbackPollTime = Time.unscaledTime + 4.0f;
                        onComplete?.Invoke(true, feedbackStatusMessage);
                        yield break;
                    }
                    else if (resp != null && !string.IsNullOrEmpty(resp.message))
                    {
                        feedbackStatusMessage = "Lỗi lưu mẫu sửa mốc: " + resp.message;
                        onComplete?.Invoke(false, feedbackStatusMessage);
                        yield break;
                    }
                }
                catch { }
            }
        }

        // Fallback ngoại tuyến: ghi thẳng vào thư mục active_learning_feedback/ và tự chạy Python retrain nếu đủ 10 mẫu
        SaveFeedbackOfflineAndMaybeRetrain(safeSample, landmarksUV12, knownImagePath, jpgBytes, autoRetrain, onComplete);
    }

    private void SaveFeedbackOfflineAndMaybeRetrain(
        string safeSample,
        Vector2[] landmarksUV12,
        string knownImagePath,
        byte[] jpgBytes,
        bool autoRetrain,
        Action<bool, string> onComplete)
    {
        try
        {
            string rootDir = GetFeedbackRootDir();
            string pendImgDir = Path.Combine(rootDir, "pending", "images");
            string pendLblDir = Path.Combine(rootDir, "pending", "labels");
            string pendMetaDir = Path.Combine(rootDir, "pending", "meta");
            Directory.CreateDirectory(pendImgDir);
            Directory.CreateDirectory(pendLblDir);
            Directory.CreateDirectory(pendMetaDir);

            string dstImg = Path.Combine(pendImgDir, safeSample + ".jpg");
            if (jpgBytes != null && jpgBytes.Length > 0)
            {
                File.WriteAllBytes(dstImg, jpgBytes);
            }
            else if (!string.IsNullOrEmpty(knownImagePath) && File.Exists(knownImagePath))
            {
                string ext = Path.GetExtension(knownImagePath).ToLowerInvariant();
                if (ext != ".jpg" && ext != ".jpeg" && ext != ".png") ext = ".jpg";
                dstImg = Path.Combine(pendImgDir, safeSample + ext);
                File.Copy(knownImagePath, dstImg, true);
            }
            else
            {
                string err = "Không có ảnh camera để lưu mẫu sửa mốc (hãy nạp PLY hoặc chụp camera trước).";
                feedbackStatusMessage = err;
                onComplete?.Invoke(false, err);
                return;
            }

            // Tính bounding box từ 12 mốc + đệm 10% (chuẩn YOLO11-pose: 0 cx cy bw bh u0 v0 2 ... u11 v11 2)
            float minU = 1f, maxU = 0f, minV = 1f, maxV = 0f;
            for (int i = 0; i < 12; i++)
            {
                float u = Mathf.Clamp01(landmarksUV12[i].x);
                float v = Mathf.Clamp01(landmarksUV12[i].y);
                if (u < minU) minU = u;
                if (u > maxU) maxU = u;
                if (v < minV) minV = v;
                if (v > maxV) maxV = v;
            }
            float spanU = Mathf.Max(0.08f, maxU - minU);
            float spanV = Mathf.Max(0.08f, maxV - minV);
            float x0 = Mathf.Clamp01(minU - Mathf.Max(0.03f, spanU * 0.10f));
            float y0 = Mathf.Clamp01(minV - Mathf.Max(0.03f, spanV * 0.08f));
            float x1 = Mathf.Clamp01(maxU + Mathf.Max(0.03f, spanU * 0.10f));
            float y1 = Mathf.Clamp01(maxV + Mathf.Max(0.03f, spanV * 0.08f));
            float bw = Mathf.Max(0.05f, x1 - x0);
            float bh = Mathf.Max(0.05f, y1 - y0);
            float cx = x0 + bw * 0.5f;
            float cy = y0 + bh * 0.5f;

            var inv = System.Globalization.CultureInfo.InvariantCulture;
            StringBuilder lbl = new StringBuilder(256);
            lbl.Append("0 ")
               .Append(cx.ToString("F6", inv)).Append(" ")
               .Append(cy.ToString("F6", inv)).Append(" ")
               .Append(bw.ToString("F6", inv)).Append(" ")
               .Append(bh.ToString("F6", inv));
            for (int i = 0; i < 12; i++)
            {
                lbl.Append(" ")
                   .Append(Mathf.Clamp01(landmarksUV12[i].x).ToString("F6", inv)).Append(" ")
                   .Append(Mathf.Clamp01(landmarksUV12[i].y).ToString("F6", inv)).Append(" 2");
            }
            lbl.Append("\n");
            File.WriteAllText(Path.Combine(pendLblDir, safeSample + ".txt"), lbl.ToString(), Encoding.UTF8);

            LoadLocalFeedbackManifest();
            int thresh = Mathf.Max(1, feedbackRetrainThreshold);
            if (autoRetrain && feedbackPendingCount >= thresh && !isRetrainingPose)
            {
                LaunchOfflineRetrainProcess(force: false);
                feedbackStatusMessage = $"Đã lưu [{safeSample}] ({feedbackPendingCount}/{thresh}) -> Đủ ngưỡng, đang chạy nền train lại YOLO11-pose...";
            }
            else
            {
                feedbackStatusMessage = $"Đã lưu mẫu sửa mốc [{safeSample}] cục bộ ({feedbackPendingCount}/{thresh}).";
            }
            onComplete?.Invoke(true, feedbackStatusMessage);
        }
        catch (Exception ex)
        {
            feedbackStatusMessage = "Lỗi lưu mẫu cục bộ: " + ex.Message;
            onComplete?.Invoke(false, feedbackStatusMessage);
        }
    }

    public void TriggerRetrainPoseModel(bool force = true, Action<bool, string> onComplete = null)
    {
        if (isRetrainingPose)
        {
            onComplete?.Invoke(false, "Đang train lại YOLO11-pose ở nền.");
            return;
        }
        if (!Application.isPlaying || !gameObject.activeInHierarchy)
        {
            bool started = LaunchOfflineRetrainProcess(force);
            onComplete?.Invoke(started, feedbackStatusMessage);
            return;
        }
        StartCoroutine(TriggerRetrainPoseRoutine(force, onComplete));
    }

    private IEnumerator TriggerRetrainPoseRoutine(bool force, Action<bool, string> onComplete)
    {
        string payload = $"{{\"command\":\"RETRAIN_POSE\",\"force\":{(force ? "true" : "false")},\"threshold\":{Mathf.Max(1, feedbackRetrainThreshold)},\"epochs\":25}}";
        string url = $"http://{serverIp.Trim()}:5000/api/feedback/retrain";
        using (UnityWebRequest req = new UnityWebRequest(url, "POST"))
        {
            req.uploadHandler = new UploadHandlerRaw(Encoding.UTF8.GetBytes(payload));
            req.downloadHandler = new DownloadHandlerBuffer();
            req.SetRequestHeader("Content-Type", "application/json");
            req.timeout = 8;
            yield return req.SendWebRequest();

            if (req.result == UnityWebRequest.Result.Success)
            {
                try
                {
                    FeedbackStatusResponse resp = JsonUtility.FromJson<FeedbackStatusResponse>(req.downloadHandler.text);
                    if (resp != null)
                    {
                        ApplyFeedbackResponse(resp);
                        bool ok = resp.status == "STARTED" || resp.status == "SUCCESS" || resp.is_retraining;
                        if (isRetrainingPose) nextFeedbackPollTime = Time.unscaledTime + 4.0f;
                        onComplete?.Invoke(ok, feedbackStatusMessage);
                        yield break;
                    }
                }
                catch { }
            }
        }

        bool launched = LaunchOfflineRetrainProcess(force);
        onComplete?.Invoke(launched, feedbackStatusMessage);
    }

    private bool LaunchOfflineRetrainProcess(bool force)
    {
        string script = GetRetrainScriptPath();
        if (!File.Exists(script))
        {
            feedbackStatusMessage = "Không tìm thấy script retrain_pose_feedback.py.";
            return false;
        }
        try
        {
            string pyExe = ResolvePythonExePath();
            string args = $"\"{script}\" --retrain --min-samples {Mathf.Max(1, feedbackRetrainThreshold)} --epochs 25" + (force ? " --force" : "");
            var psi = new System.Diagnostics.ProcessStartInfo
            {
                FileName = pyExe,
                Arguments = args,
                WorkingDirectory = Path.GetDirectoryName(script),
                UseShellExecute = false,
                CreateNoWindow = true
            };
            System.Diagnostics.Process.Start(psi);
            isRetrainingPose = true;
            nextFeedbackPollTime = Time.unscaledTime + 4.0f;
            feedbackStatusMessage = "Đang chạy nền train lại YOLO11-pose (offline Python)...";
            return true;
        }
        catch (Exception ex)
        {
            feedbackStatusMessage = "Không khởi chạy được Python train lại: " + ex.Message;
            return false;
        }
    }

    private static string EscapeJsonString(string s)
    {
        if (string.IsNullOrEmpty(s)) return "";
        return s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\r", "").Replace("\n", "\\n");
    }
}
