using System;
using UnityEngine;

/// <summary>
/// Bộ Theme giao diện Antigravity phong cách Dark Modern (Obsidian / Cyberpunk Clean / Fluent Dark).
/// Thay thế toàn bộ giao diện xám cổ điển của Unity IMGUI bằng giao diện phẳng, bo góc, tương phản cao.
/// </summary>
public static class AntigravityUITheme
{
    private static bool initialized = false;

    // Bảng màu chuẩn Antigravity
    public static readonly Color ColorBgDarkest    = new Color(0.06f, 0.07f, 0.09f, 0.98f); // #0f1217
    public static readonly Color ColorBgHeader     = new Color(0.09f, 0.10f, 0.13f, 0.98f); // #171a21
    public static readonly Color ColorBgStrip      = new Color(0.11f, 0.12f, 0.16f, 0.97f); // #1c1f29
    public static readonly Color ColorBgDock       = new Color(0.08f, 0.09f, 0.12f, 0.95f); // #14171f
    public static readonly Color ColorBgCard       = new Color(0.13f, 0.15f, 0.20f, 0.98f); // #212633
    public static readonly Color ColorBgCardInner  = new Color(0.09f, 0.10f, 0.14f, 0.90f); // #171a24

    public static readonly Color ColorBorder       = new Color(0.20f, 0.23f, 0.31f, 1.00f); // #333b4f
    public static readonly Color ColorBorderCard   = new Color(0.25f, 0.29f, 0.39f, 0.85f); // #404a63
    public static readonly Color ColorBorderActive = new Color(0.22f, 0.65f, 0.98f, 1.00f); // #38a6fa

    public static readonly Color ColorTextPrimary  = new Color(0.95f, 0.96f, 0.98f, 1.00f); // #f2f5fa
    public static readonly Color ColorTextMuted    = new Color(0.60f, 0.65f, 0.74f, 1.00f); // #99a6bd
    public static readonly Color ColorTextCyan     = new Color(0.22f, 0.78f, 1.00f, 1.00f); // #38c7ff
    public static readonly Color ColorTextGreen    = new Color(0.28f, 0.88f, 0.55f, 1.00f); // #47e08c
    public static readonly Color ColorTextYellow   = new Color(1.00f, 0.80f, 0.25f, 1.00f); // #ffcc40
    public static readonly Color ColorTextRed      = new Color(1.00f, 0.40f, 0.40f, 1.00f); // #ff6666

    // Textures phẳng được tạo trực tiếp trong RAM
    public static Texture2D TexBgHeader;
    public static Texture2D TexBgStrip;
    public static Texture2D TexBgDock;
    public static Texture2D TexBgCard;
    public static Texture2D TexBgCardInner;
    public static Texture2D TexBorder;
    public static Texture2D TexBtnNormal;
    public static Texture2D TexBtnHover;
    public static Texture2D TexBtnActive;
    public static Texture2D TexBtnPrimaryNormal;
    public static Texture2D TexBtnPrimaryHover;
    public static Texture2D TexTabActive;
    public static Texture2D TexTabInactive;
    public static Texture2D TexBadgeGreen;
    public static Texture2D TexBadgeBlue;
    public static Texture2D TexInput;
    public static Texture2D TexSliderTrack;
    public static Texture2D TexSliderThumb;

    // Styles lưu trữ sẵn
    public static GUIStyle StyleHeaderTitle;
    public static GUIStyle StyleTabActive;
    public static GUIStyle StyleTabInactive;
    public static GUIStyle StyleButton;
    public static GUIStyle StylePrimaryButton;
    public static GUIStyle StyleCloseButton;
    public static GUIStyle StyleCardHeader;
    public static GUIStyle StyleSectionLabel;
    public static GUIStyle StyleBadgeGreen;
    public static GUIStyle StyleBadgeBlue;
    public static GUIStyle StyleTextField;
    public static GUIStyle StyleToggle;
    public static GUIStyle StyleLabelMuted;
    public static GUIStyle StyleLabelCyan;
    public static GUIStyle StyleLabelGreen;
    public static GUIStyle StyleLabelYellow;
    public static GUIStyle StyleLabelRed;

    public static void EnsureInitialized()
    {
        if (initialized && TexBgHeader != null) return;

        // 1. Tạo Textures phẳng
        TexBgHeader         = MakeTex(2, 2, ColorBgHeader);
        TexBgStrip          = MakeTex(2, 2, ColorBgStrip);
        TexBgDock           = MakeTex(2, 2, ColorBgDock);
        TexBgCard           = MakeTex(2, 2, ColorBgCard);
        TexBgCardInner      = MakeTex(2, 2, ColorBgCardInner);
        TexBorder           = MakeTex(2, 2, ColorBorder);

        TexBtnNormal        = MakeTex(2, 2, new Color(0.16f, 0.18f, 0.24f, 1.0f));
        TexBtnHover         = MakeTex(2, 2, new Color(0.22f, 0.26f, 0.35f, 1.0f));
        TexBtnActive        = MakeTex(2, 2, new Color(0.12f, 0.45f, 0.85f, 1.0f));

        TexBtnPrimaryNormal = MakeTex(2, 2, new Color(0.08f, 0.48f, 0.90f, 1.0f));
        TexBtnPrimaryHover  = MakeTex(2, 2, new Color(0.14f, 0.58f, 1.00f, 1.0f));

        TexTabActive        = MakeTex(2, 2, new Color(0.15f, 0.26f, 0.42f, 1.0f));
        TexTabInactive      = MakeTex(2, 2, new Color(0.10f, 0.11f, 0.15f, 0.7f));

        TexBadgeGreen       = MakeTex(2, 2, new Color(0.06f, 0.28f, 0.16f, 0.85f));
        TexBadgeBlue        = MakeTex(2, 2, new Color(0.08f, 0.24f, 0.45f, 0.85f));

        TexInput            = MakeTex(2, 2, new Color(0.07f, 0.08f, 0.11f, 1.0f));
        TexSliderTrack      = MakeTex(2, 2, new Color(0.18f, 0.21f, 0.28f, 1.0f));
        TexSliderThumb      = MakeTex(2, 2, new Color(0.24f, 0.75f, 1.00f, 1.0f));

        // 2. Cấu hình Typography & Styles
        StyleHeaderTitle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 15,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft,
            normal = { textColor = ColorTextCyan }
        };

        StyleTabActive = new GUIStyle(GUI.skin.button)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            normal = { background = TexTabActive, textColor = ColorTextCyan },
            hover = { background = TexTabActive, textColor = Color.white },
            active = { background = TexTabActive, textColor = Color.white },
            margin = new RectOffset(2, 2, 2, 2),
            padding = new RectOffset(10, 10, 4, 4)
        };

        StyleTabInactive = new GUIStyle(GUI.skin.button)
        {
            fontSize = 12,
            fontStyle = FontStyle.Normal,
            alignment = TextAnchor.MiddleCenter,
            normal = { background = TexTabInactive, textColor = ColorTextMuted },
            hover = { background = TexBtnHover, textColor = ColorTextPrimary },
            active = { background = TexTabActive, textColor = Color.white },
            margin = new RectOffset(2, 2, 2, 2),
            padding = new RectOffset(10, 10, 4, 4)
        };

        StyleButton = new GUIStyle(GUI.skin.button)
        {
            fontSize = 11,
            alignment = TextAnchor.MiddleCenter,
            normal = { background = TexBtnNormal, textColor = ColorTextPrimary },
            hover = { background = TexBtnHover, textColor = Color.white },
            active = { background = TexBtnActive, textColor = Color.white },
            margin = new RectOffset(3, 3, 3, 3),
            padding = new RectOffset(8, 8, 4, 4)
        };

        StylePrimaryButton = new GUIStyle(GUI.skin.button)
        {
            fontSize = 11,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            normal = { background = TexBtnPrimaryNormal, textColor = Color.white },
            hover = { background = TexBtnPrimaryHover, textColor = Color.white },
            active = { background = TexBtnActive, textColor = Color.white },
            margin = new RectOffset(3, 3, 3, 3),
            padding = new RectOffset(8, 8, 4, 4)
        };

        StyleCloseButton = new GUIStyle(GUI.skin.button)
        {
            fontSize = 11,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            normal = { background = TexBtnNormal, textColor = ColorTextMuted },
            hover = { background = MakeTex(2, 2, new Color(0.7f, 0.15f, 0.15f, 1f)), textColor = Color.white },
            active = { background = MakeTex(2, 2, new Color(0.9f, 0.2f, 0.2f, 1f)), textColor = Color.white },
            margin = new RectOffset(2, 2, 2, 2),
            padding = new RectOffset(2, 2, 2, 2)
        };

        StyleCardHeader = new GUIStyle(GUI.skin.label)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold,
            normal = { textColor = ColorTextCyan },
            margin = new RectOffset(2, 2, 2, 4)
        };

        StyleSectionLabel = new GUIStyle(GUI.skin.label)
        {
            fontSize = 9,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            normal = { textColor = new Color(0.50f, 0.55f, 0.65f) }
        };

        StyleBadgeGreen = new GUIStyle(GUI.skin.box)
        {
            fontSize = 10,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            normal = { background = TexBadgeGreen, textColor = ColorTextGreen },
            padding = new RectOffset(8, 8, 4, 4)
        };

        StyleBadgeBlue = new GUIStyle(GUI.skin.box)
        {
            fontSize = 10,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            normal = { background = TexBadgeBlue, textColor = ColorTextCyan },
            padding = new RectOffset(8, 8, 4, 4)
        };

        StyleTextField = new GUIStyle(GUI.skin.textField)
        {
            fontSize = 11,
            alignment = TextAnchor.MiddleLeft,
            normal = { background = TexInput, textColor = ColorTextPrimary },
            focused = { background = TexInput, textColor = Color.white },
            padding = new RectOffset(6, 6, 3, 3)
        };

        StyleToggle = new GUIStyle(GUI.skin.toggle)
        {
            fontSize = 11,
            normal = { textColor = ColorTextPrimary },
            hover = { textColor = Color.white },
            onNormal = { textColor = ColorTextCyan },
            onHover = { textColor = Color.white }
        };

        StyleLabelMuted  = new GUIStyle(GUI.skin.label) { fontSize = 11, normal = { textColor = ColorTextMuted } };
        StyleLabelCyan   = new GUIStyle(GUI.skin.label) { fontSize = 11, fontStyle = FontStyle.Bold, normal = { textColor = ColorTextCyan } };
        StyleLabelGreen  = new GUIStyle(GUI.skin.label) { fontSize = 11, fontStyle = FontStyle.Bold, normal = { textColor = ColorTextGreen } };
        StyleLabelYellow = new GUIStyle(GUI.skin.label) { fontSize = 11, fontStyle = FontStyle.Bold, normal = { textColor = ColorTextYellow } };
        StyleLabelRed    = new GUIStyle(GUI.skin.label) { fontSize = 11, fontStyle = FontStyle.Bold, normal = { textColor = ColorTextRed } };

        initialized = true;
    }

    private static Texture2D MakeTex(int width, int height, Color col)
    {
        Color[] pix = new Color[width * height];
        for (int i = 0; i < pix.Length; ++i) pix[i] = col;
        Texture2D result = new Texture2D(width, height)
        {
            hideFlags = HideFlags.DontSave
        };
        result.SetPixels(pix);
        result.Apply();
        return result;
    }

    /// <summary>
    /// Vẽ vùng Card nền tối Antigravity với viền mỏng thanh lịch.
    /// </summary>
    public static void BeginCard(float width, string title = null, Action onClose = null)
    {
        EnsureInitialized();
        GUI.color = Color.white;
        GUILayout.BeginVertical(GUIStyle.none, GUILayout.Width(width));

        // Vẽ nền Card
        Rect cardRect = GUILayoutUtility.GetRect(width, 0f, GUILayout.ExpandHeight(true));
        // Lùi lại vì GetRect sẽ chiếm chỗ
        // Thay vì thế dùng GUILayout.BeginVertical với style tùy chỉnh:
    }

    /// <summary>
    /// Tạo một Box phong cách Card Antigravity
    /// </summary>
    public static GUIStyle CardBoxStyle()
    {
        EnsureInitialized();
        return new GUIStyle(GUI.skin.box)
        {
            normal = { background = TexBgCard },
            margin = new RectOffset(0, 0, 4, 4),
            padding = new RectOffset(10, 10, 8, 8)
        };
    }

    /// <summary>
    /// Tạo một Box phong cách Card phụ bên trong
    /// </summary>
    public static GUIStyle InnerBoxStyle()
    {
        EnsureInitialized();
        return new GUIStyle(GUI.skin.box)
        {
            normal = { background = TexBgCardInner },
            margin = new RectOffset(0, 0, 3, 3),
            padding = new RectOffset(8, 8, 6, 6)
        };
    }

    /// <summary>
    /// Nút Tab trên thanh Ribbon
    /// </summary>
    public static bool RibbonTab(string text, bool isCurrent, float width = 145f, float height = 30f)
    {
        EnsureInitialized();
        GUIStyle style = isCurrent ? StyleTabActive : StyleTabInactive;
        return GUILayout.Button(text, style, GUILayout.Width(width), GUILayout.Height(height));
    }

    /// <summary>
    /// Nút bấm Antigravity thông thường
    /// </summary>
    public static bool Button(string text, float height = 30f, float width = 0f)
    {
        EnsureInitialized();
        if (width > 0f)
            return GUILayout.Button(text, StyleButton, GUILayout.Width(width), GUILayout.Height(height));
        return GUILayout.Button(text, StyleButton, GUILayout.Height(height));
    }

    /// <summary>
    /// Nút bấm Antigravity nổi bật (Primary Action: Play, Solve, v.v.)
    /// </summary>
    public static bool PrimaryButton(string text, float height = 30f, float width = 0f)
    {
        EnsureInitialized();
        if (width > 0f)
            return GUILayout.Button(text, StylePrimaryButton, GUILayout.Width(width), GUILayout.Height(height));
        return GUILayout.Button(text, StylePrimaryButton, GUILayout.Height(height));
    }
}
