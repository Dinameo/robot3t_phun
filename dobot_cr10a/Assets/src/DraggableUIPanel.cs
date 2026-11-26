using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Hệ thống cửa sổ nổi KÉO THẢ & KÉO GIÃN KÍCH THƯỚC (Draggable & Resizable Floating Panels) cho Unity IMGUI.
/// Giải quyết triệt để tình trạng bảng bị nhỏ, bị ép thanh trượt ngang/dọc chồng chéo:
/// - Kéo thanh tiêu đề để di chuyển tự do mọi cửa sổ khắp màn hình.
/// - Kéo góc dưới-phải (◢) để phóng to / thu nhỏ chiều rộng & chiều cao tùy ý.
/// - Nút thu gọn (– / +) và đóng (×) trên từng bảng; cho phép mở nhiều bảng song song.
/// - Tự động ghi nhớ vị trí & kích thước qua PlayerPrefs và thông báo vùng chặn chuột sang RobotCameraController.
/// </summary>
public static class DraggableUIManager
{
    public class PanelState
    {
        public string id;
        public string title;
        public Rect rect;
        public Rect defaultRect;
        public Vector2 minSize;
        public bool isOpen;
        public bool isCollapsed;
        public bool isDockedRight = true; // Gắn cố định bên phải, không cho di chuyển, chỉ cho kéo giãn cạnh trái
        public Vector2 scrollPos;
        public int zOrder;

        internal bool isDragging;
        internal bool isResizing;
        internal bool isResizingLeft;
        internal Vector2 dragOffset;
        internal Vector2 resizeStartMouse;
        internal Vector2 resizeStartSize;
        internal float resizeStartX;

        public PanelState(string id, string title, Rect defaultRect, Vector2 minSize, bool openByDefault = false)
        {
            this.id = id;
            this.title = title;
            this.defaultRect = defaultRect;
            this.rect = defaultRect;
            this.minSize = minSize;
            this.isOpen = openByDefault;
            this.isCollapsed = false;
            this.scrollPos = Vector2.zero;
        }
    }

    private static readonly List<Rect> activeBlockingRects = new List<Rect>();
    private static int lastFrameCleared = -1;
    private static int nextZOrder = 10;
    private static string activeDragOrResizeId = null;

    public const float TitleBarHeight = 30f;
    public const float ResizeGripSize = 18f;

    public static void BeginFrame()
    {
        if (lastFrameCleared != Time.frameCount)
        {
            activeBlockingRects.Clear();
            lastFrameCleared = Time.frameCount;
        }
    }

    public static void RegisterBlockingRect(Rect r)
    {
        BeginFrame();
        activeBlockingRects.Add(r);
    }

    public static bool IsPointOverAnyUI(Vector2 guiMousePos)
    {
        if (!string.IsNullOrEmpty(activeDragOrResizeId)) return true;
        for (int i = 0; i < activeBlockingRects.Count; i++)
        {
            if (activeBlockingRects[i].Contains(guiMousePos))
                return true;
        }
        return false;
    }

    public static void BringToFront(PanelState panel)
    {
        if (panel == null) return;
        panel.zOrder = ++nextZOrder;
    }

    public static void SavePanelLayout(PanelState p)
    {
        if (p == null) return;
        string prefix = "CR10A_Win_" + p.id + "_";
        PlayerPrefs.SetFloat(prefix + "X", p.rect.x);
        PlayerPrefs.SetFloat(prefix + "Y", p.rect.y);
        PlayerPrefs.SetFloat(prefix + "W", p.rect.width);
        PlayerPrefs.SetFloat(prefix + "H", p.rect.height);
        PlayerPrefs.SetInt(prefix + "Collapsed", p.isCollapsed ? 1 : 0);
        PlayerPrefs.Save();
    }

    public static void LoadPanelLayout(PanelState p, float topSafeY)
    {
        if (p == null) return;
        string prefix = "CR10A_Win_" + p.id + "_";
        if (PlayerPrefs.HasKey(prefix + "W"))
        {
            float x = PlayerPrefs.GetFloat(prefix + "X", p.defaultRect.x);
            float y = PlayerPrefs.GetFloat(prefix + "Y", p.defaultRect.y);
            float w = PlayerPrefs.GetFloat(prefix + "W", p.defaultRect.width);
            float h = PlayerPrefs.GetFloat(prefix + "H", p.defaultRect.height);
            p.rect = new Rect(x, y, w, h);
            p.isCollapsed = PlayerPrefs.GetInt(prefix + "Collapsed", 0) == 1;
        }
        else
        {
            p.rect = p.defaultRect;
            p.isCollapsed = false;
        }
        ClampPanelToScreen(p, topSafeY);
    }

    public static void ResetPanelToDefault(PanelState p, float topSafeY)
    {
        if (p == null) return;
        p.rect = p.defaultRect;
        p.isCollapsed = false;
        p.scrollPos = Vector2.zero;
        ClampPanelToScreen(p, topSafeY);
        SavePanelLayout(p);
    }

    public static void ClampPanelToScreen(PanelState p, float topSafeY)
    {
        if (p.isDockedRight)
        {
            float maxW = Mathf.Max(p.minSize.x, Screen.width * 0.62f);
            p.rect.width = Mathf.Clamp(p.rect.width, p.minSize.x, maxW);
            p.rect.x = Mathf.Max(0f, Screen.width - p.rect.width);
            p.rect.y = topSafeY;
            p.rect.height = Mathf.Max(200f, Screen.height - topSafeY);
            p.isCollapsed = false;
            return;
        }

        float maxWFree = Mathf.Max(p.minSize.x, Screen.width - 20f);
        float maxH = Mathf.Max(p.minSize.y, Screen.height - topSafeY - 48f);

        p.rect.width = Mathf.Clamp(p.rect.width, p.minSize.x, maxWFree);
        p.rect.height = Mathf.Clamp(p.rect.height, p.minSize.y, maxH);

        float maxX = Mathf.Max(8f, Screen.width - Mathf.Min(p.rect.width, 160f));
        float maxY = Mathf.Max(topSafeY + 4f, Screen.height - TitleBarHeight - 40f);
        p.rect.x = Mathf.Clamp(p.rect.x, 8f, Mathf.Max(8f, Screen.width - p.rect.width - 8f));
        p.rect.y = Mathf.Clamp(p.rect.y, topSafeY + 4f, maxY);

        if (p.rect.x > maxX) p.rect.x = maxX;
    }

    /// <summary>
    /// Vẽ bảng điều khiển gắn cố định bên phải (chỉ kéo cạnh trái để đổi chiều rộng) hoặc cửa sổ nổi nếu isDockedRight=false.
    /// </summary>
    public static void DrawDraggablePanel(PanelState panel, float topSafeY, Action<float> drawContent)
    {
        if (panel == null || !panel.isOpen) return;

        AntigravityUITheme.EnsureInitialized();
        AntigravityUITheme.ApplyDarkScrollbarsToCurrentSkin();

        ClampPanelToScreen(panel, topSafeY);

        float visibleH = panel.isCollapsed ? TitleBarHeight : panel.rect.height;
        Rect drawRect = new Rect(panel.rect.x, panel.rect.y, panel.rect.width, visibleH);

        // Đăng ký vùng chặn chuột 3D
        RegisterBlockingRect(drawRect);

        Event e = Event.current;
        Rect titleRect = new Rect(drawRect.x + 8f, drawRect.y, drawRect.width - 42f, TitleBarHeight);
        Rect resizeRect = new Rect(drawRect.xMax - ResizeGripSize, drawRect.yMax - ResizeGripSize, ResizeGripSize, ResizeGripSize);
        Rect leftEdgeRect = new Rect(drawRect.x - 5f, drawRect.y, 12f, drawRect.height);
        bool hoverLeftEdge = (e != null && leftEdgeRect.Contains(e.mousePosition));

        // Xử lý kéo giãn (ở chế độ gắn phải: KHÓA di chuyển, CHỈ cho kéo cạnh trái để thay đổi kích thước!)
        if (e != null)
        {
            if (e.type == EventType.MouseDown && e.button == 0)
            {
                if (drawRect.Contains(e.mousePosition))
                {
                    BringToFront(panel);
                }

                if (!panel.isCollapsed && leftEdgeRect.Contains(e.mousePosition))
                {
                    panel.isResizingLeft = true;
                    panel.isResizing = false;
                    panel.isDragging = false;
                    panel.resizeStartMouse = e.mousePosition;
                    panel.resizeStartSize = new Vector2(panel.rect.width, panel.rect.height);
                    panel.resizeStartX = panel.rect.x;
                    activeDragOrResizeId = panel.id;
                    e.Use();
                }
                else if (!panel.isDockedRight && !panel.isCollapsed && resizeRect.Contains(e.mousePosition))
                {
                    panel.isResizing = true;
                    panel.isResizingLeft = false;
                    panel.isDragging = false;
                    panel.resizeStartMouse = e.mousePosition;
                    panel.resizeStartSize = new Vector2(panel.rect.width, panel.rect.height);
                    activeDragOrResizeId = panel.id;
                    e.Use();
                }
                else if (!panel.isDockedRight && titleRect.Contains(e.mousePosition))
                {
                    panel.isDragging = true;
                    panel.isResizing = false;
                    panel.isResizingLeft = false;
                    panel.dragOffset = e.mousePosition - new Vector2(panel.rect.x, panel.rect.y);
                    activeDragOrResizeId = panel.id;
                    e.Use();
                }
            }
            else if (e.type == EventType.MouseDrag && activeDragOrResizeId == panel.id)
            {
                if (!panel.isDockedRight && panel.isDragging)
                {
                    panel.rect.x = e.mousePosition.x - panel.dragOffset.x;
                    panel.rect.y = e.mousePosition.y - panel.dragOffset.y;
                    ClampPanelToScreen(panel, topSafeY);
                    e.Use();
                }
                else if (panel.isResizingLeft)
                {
                    float dx = e.mousePosition.x - panel.resizeStartMouse.x;
                    float maxW = panel.isDockedRight
                        ? Mathf.Max(panel.minSize.x, Screen.width * 0.62f)
                        : Mathf.Max(panel.minSize.x, Screen.width - 20f);
                    float newW = Mathf.Clamp(panel.resizeStartSize.x - dx, panel.minSize.x, maxW);
                    panel.rect.width = newW;
                    if (!panel.isDockedRight)
                    {
                        float actualDx = panel.resizeStartSize.x - newW;
                        panel.rect.x = Mathf.Max(8f, panel.resizeStartX + actualDx);
                    }
                    ClampPanelToScreen(panel, topSafeY);
                    e.Use();
                }
                else if (!panel.isDockedRight && panel.isResizing)
                {
                    Vector2 delta = e.mousePosition - panel.resizeStartMouse;
                    panel.rect.width = panel.resizeStartSize.x + delta.x;
                    panel.rect.height = panel.resizeStartSize.y + delta.y;
                    ClampPanelToScreen(panel, topSafeY);
                    e.Use();
                }
            }
            else if ((e.type == EventType.MouseUp || e.rawType == EventType.MouseUp) && activeDragOrResizeId == panel.id)
            {
                panel.isDragging = false;
                panel.isResizing = false;
                panel.isResizingLeft = false;
                activeDragOrResizeId = null;
                SavePanelLayout(panel);
            }
        }

        // Cập nhật lại drawRect sau khi resize
        visibleH = panel.isCollapsed ? TitleBarHeight : panel.rect.height;
        drawRect = new Rect(panel.rect.x, panel.rect.y, panel.rect.width, visibleH);

        // 1. Nền bảng + Viền kéo giãn cạnh trái nổi bật khi rê chuột hoặc kéo
        AntigravityUITheme.DrawSoftDock(drawRect);
        bool leftHot = panel.isResizingLeft || hoverLeftEdge;
        if (leftHot)
            GUI.DrawTexture(new Rect(drawRect.x + 1f, drawRect.y + 7f, 2f, drawRect.height - 14f), AntigravityUITheme.TexBorderActive);

        float midY = drawRect.y + drawRect.height * 0.5f;
        GUI.DrawTexture(new Rect(drawRect.x + 2f, midY - 12f, 2f, 24f), AntigravityUITheme.TexBorderActive);
        GUI.DrawTexture(new Rect(drawRect.x + 8f, drawRect.y + TitleBarHeight - 1f, drawRect.width - 16f, 1f), AntigravityUITheme.TexBorder);

        GUI.Label(
            new Rect(drawRect.x + 10f, drawRect.y + 2f, drawRect.width - (panel.isDockedRight ? 44f : 76f), TitleBarHeight - 4f),
            panel.title,
            AntigravityUITheme.StyleWindowTitle
        );

        // Nút Thu gọn chỉ hiển thị khi không gắn cố định
        if (!panel.isDockedRight)
        {
            if (GUI.Button(new Rect(drawRect.xMax - 56f, drawRect.y + 4f, 24f, 22f), panel.isCollapsed ? "+" : "-", AntigravityUITheme.StyleButton))
            {
                panel.isCollapsed = !panel.isCollapsed;
                SavePanelLayout(panel);
            }
        }

        // Nút Đóng (X) — đóng bảng và mở rộng toàn bộ màn hình xem phôi 3D
        if (GUI.Button(new Rect(drawRect.xMax - 28f, drawRect.y + 4f, 24f, 22f), "X", AntigravityUITheme.StyleCloseButton))
        {
            panel.isOpen = false;
            return;
        }

        if (panel.isCollapsed) return;

        // 3. Vùng nội dung bên trong (chỉ cuộn dọc khi cần)
        Rect bodyRect = new Rect(drawRect.x + 8f, drawRect.y + TitleBarHeight + 4f, drawRect.width - 14f, drawRect.height - TitleBarHeight - 10f);
        float innerWidth = Mathf.Max(260f, bodyRect.width - 18f);

        GUILayout.BeginArea(bodyRect);
        panel.scrollPos = GUILayout.BeginScrollView(
            panel.scrollPos,
            false,
            false,
            GUIStyle.none,
            AntigravityUITheme.StyleVerticalScrollbar,
            GUILayout.Width(bodyRect.width),
            GUILayout.Height(bodyRect.height)
        );

        drawContent?.Invoke(innerWidth);

        GUILayout.EndScrollView();
        GUILayout.EndArea();

        if (!panel.isDockedRight)
        {
            GUI.DrawTexture(new Rect(drawRect.xMax - 12f, drawRect.yMax - 4f, 9f, 2f), AntigravityUITheme.TexBorderActive);
            GUI.DrawTexture(new Rect(drawRect.xMax - 4f, drawRect.yMax - 12f, 2f, 9f), AntigravityUITheme.TexBorderActive);
        }
    }
}
