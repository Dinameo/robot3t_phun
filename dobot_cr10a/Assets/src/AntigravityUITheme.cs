using System;
using UnityEngine;

/// <summary>
/// Bộ Theme giao diện Antigravity phong cách Dark Modern (Obsidian / Cyberpunk Clean / Fluent Dark).
/// Tối ưu hóa cho các cửa sổ nổi kéo thả được (Draggable & Resizable Panels) với thanh cuộn tối tinh gọn.
/// </summary>
public static class AntigravityUITheme
{
    private static bool initialized = false;
    private static GUIStyle cachedCardStyle;
    private static GUIStyle cachedInnerStyle;

    // Bảng màu chuẩn Antigravity
    public static readonly Color ColorBgDarkest    = new Color(0.06f, 0.07f, 0.09f, 1.00f); // #0f1217
    public static readonly Color ColorBgHeader     = new Color(0.09f, 0.10f, 0.13f, 1.00f); // #171a21
    public static readonly Color ColorBgStrip      = new Color(0.11f, 0.12f, 0.16f, 1.00f); // #1c1f29
    public static readonly Color ColorBgDock       = new Color(0.08f, 0.09f, 0.12f, 1.00f); // #14171f
    public static readonly Color ColorBgCard       = new Color(0.13f, 0.15f, 0.20f, 0.98f); // #212633
    public static readonly Color ColorBgCardInner  = new Color(0.09f, 0.10f, 0.14f, 0.92f); // #171a24
    public static readonly Color ColorBgTitleBar   = new Color(0.12f, 0.15f, 0.22f, 1.00f);

    public static readonly Color ColorBorder       = new Color(0.20f, 0.23f, 0.31f, 1.00f); // #333b4f
    public static readonly Color ColorBorderCard   = new Color(0.25f, 0.29f, 0.39f, 0.85f); // #404a63
    public static readonly Color ColorBorderActive = new Color(0.22f, 0.65f, 0.98f, 1.00f); // #38a6fa

    public static readonly Color ColorTextPrimary  = new Color(0.95f, 0.96f, 0.98f, 1.00f); // #f2f5fa
    public static readonly Color ColorTextMuted    = new Color(0.65f, 0.70f, 0.78f, 1.00f); // #a6b2c7
    public static readonly Color ColorTextCyan     = new Color(0.43f, 0.79f, 0.98f, 1.00f); // #38c7ff
    public static readonly Color ColorTextGreen    = new Color(0.28f, 0.88f, 0.55f, 1.00f); // #47e08c
    public static readonly Color ColorTextYellow   = new Color(1.00f, 0.80f, 0.25f, 1.00f); // #ffcc40
    public static readonly Color ColorTextRed      = new Color(1.00f, 0.40f, 0.40f, 1.00f); // #ff6666

    // Textures phẳng được tạo trực tiếp trong RAM
    public static Texture2D TexBgHeader;
    public static Texture2D TexBgStrip;
    public static Texture2D TexBgDock;
    public static Texture2D TexBgCard;
    public static Texture2D TexBgCardInner;
    public static Texture2D TexBgTitleBar;
    public static Texture2D TexBorder;
    public static Texture2D TexBorderActive;
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
    public static Texture2D TexScrollTrack;
    public static Texture2D TexScrollThumb;

    // Styles lưu trữ sẵn
    public static GUIStyle StyleHeaderTitle;
    public static GUIStyle StyleWindowTitle;
    public static GUIStyle StyleTabActive;
    public static GUIStyle StyleTabInactive;
    public static GUIStyle StyleButton;
    public static GUIStyle StylePrimaryButton;
    public static GUIStyle StyleCloseButton;
    public static GUIStyle StyleCardHeader;
    public static GUIStyle StyleProgressLabel;
    public static GUIStyle StyleSprayColorButton;
    public static GUIStyle StyleSelectedSprayColorButton;
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
    public static GUIStyle StyleVerticalScrollbar;
    public static GUIStyle StyleVerticalScrollbarThumb;
    public static GUIStyle StyleHorizontalScrollbar;
    public static GUIStyle StyleHorizontalScrollbarThumb;

    public static void EnsureInitialized()
    {
        if (initialized && TexBgHeader != null && StyleVerticalScrollbar != null) return;

        // 1. Tạo Textures phẳng
        TexBgHeader         = MakeTex(2, 2, ColorBgHeader);
        TexBgStrip          = MakeTex(2, 2, ColorBgStrip);
        TexBgDock           = MakeTex(2, 2, ColorBgDock);
        TexBgCard           = MakeTex(2, 2, ColorBgCard);
        TexBgCardInner      = MakeTex(2, 2, ColorBgCardInner);
        TexBgTitleBar       = MakeTex(2, 2, ColorBgTitleBar);
        TexBorder           = MakeTex(2, 2, ColorBorder);
        TexBorderActive     = MakeTex(2, 2, ColorBorderActive);

        TexBtnNormal        = MakeTex(2, 2, new Color(0.16f, 0.18f, 0.24f, 1.0f));
        TexBtnHover         = MakeTex(2, 2, new Color(0.22f, 0.26f, 0.35f, 1.0f));
        TexBtnActive        = MakeTex(2, 2, new Color(0.12f, 0.45f, 0.85f, 1.0f));

        TexBtnPrimaryNormal = MakeTex(2, 2, new Color(0.10f, 0.40f, 0.66f, 1.0f));
        TexBtnPrimaryHover  = MakeTex(2, 2, new Color(0.15f, 0.51f, 0.80f, 1.0f));

        TexTabActive        = MakeTex(2, 2, new Color(0.15f, 0.28f, 0.46f, 1.0f));
        TexTabInactive      = MakeTex(2, 2, new Color(0.10f, 0.11f, 0.15f, 0.85f));

        TexBadgeGreen       = MakeTex(2, 2, new Color(0.06f, 0.28f, 0.16f, 0.85f));
        TexBadgeBlue        = MakeTex(2, 2, new Color(0.08f, 0.24f, 0.45f, 0.85f));

        TexInput            = MakeTex(2, 2, new Color(0.07f, 0.08f, 0.11f, 1.0f));
        TexSliderTrack      = MakeTex(2, 2, new Color(0.18f, 0.21f, 0.28f, 1.0f));
        TexSliderThumb      = MakeTex(2, 2, new Color(0.24f, 0.75f, 1.00f, 1.0f));

        TexScrollTrack      = MakeTex(2, 2, new Color(0.07f, 0.08f, 0.11f, 0.85f));
        TexScrollThumb      = MakeTex(2, 2, new Color(0.28f, 0.34f, 0.46f, 0.95f));

        // 2. Cấu hình Typography & Styles
        StyleHeaderTitle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 17,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft,
            wordWrap = false,
            normal = { textColor = ColorTextCyan }
        };

        StyleWindowTitle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 13,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft,
            wordWrap = false,
            normal = { textColor = ColorTextCyan },
            padding = new RectOffset(6, 4, 2, 2)
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
            padding = new RectOffset(8, 8, 4, 4)
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
            padding = new RectOffset(8, 8, 4, 4)
        };

        StyleButton = new GUIStyle(GUI.skin.button)
        {
            fontSize = 12,
            alignment = TextAnchor.MiddleCenter,
            wordWrap = false,
            normal = { background = TexBtnNormal, textColor = ColorTextPrimary },
            hover = { background = TexBtnHover, textColor = Color.white },
            active = { background = TexBtnActive, textColor = Color.white },
            margin = new RectOffset(3, 3, 3, 3),
            padding = new RectOffset(8, 8, 4, 4)
        };

        StylePrimaryButton = new GUIStyle(GUI.skin.button)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            wordWrap = false,
            normal = { background = TexBtnPrimaryNormal, textColor = Color.white },
            hover = { background = TexBtnPrimaryHover, textColor = Color.white },
            active = { background = TexBtnActive, textColor = Color.white },
            margin = new RectOffset(3, 3, 3, 3),
            padding = new RectOffset(8, 8, 4, 4)
        };

        StyleSprayColorButton = new GUIStyle(StyleButton)
        {
            margin = new RectOffset(2, 2, 3, 3),
            padding = new RectOffset(11, 11, 5, 8)
        };
        StyleSelectedSprayColorButton = new GUIStyle(StylePrimaryButton)
        {
            margin = new RectOffset(2, 2, 3, 3),
            padding = new RectOffset(11, 11, 5, 8)
        };

        StyleCloseButton = new GUIStyle(GUI.skin.button)
        {
            fontSize = 12,
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
            wordWrap = true,
            normal = { textColor = ColorTextCyan },
            margin = new RectOffset(2, 2, 2, 4)
        };

        StyleProgressLabel = new GUIStyle(StyleCardHeader)
        {
            wordWrap = false,
            clipping = TextClipping.Clip,
            alignment = TextAnchor.MiddleLeft,
            padding = new RectOffset()
        };

        StyleSectionLabel = new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft,
            wordWrap = true,
            normal = { textColor = new Color(0.68f, 0.74f, 0.83f) }
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
            fontSize = 12,
            alignment = TextAnchor.MiddleLeft,
            normal = { background = TexInput, textColor = ColorTextPrimary },
            focused = { background = TexInput, textColor = Color.white },
            padding = new RectOffset(6, 6, 3, 3)
        };

        StyleToggle = new GUIStyle(GUI.skin.toggle)
        {
            fontSize = 12,
            wordWrap = true,
            normal = { textColor = ColorTextPrimary },
            hover = { textColor = Color.white },
            onNormal = { textColor = ColorTextCyan },
            onHover = { textColor = Color.white }
        };

        StyleLabelMuted  = new GUIStyle(GUI.skin.label) { fontSize = 12, wordWrap = true, normal = { textColor = ColorTextMuted } };
        StyleLabelCyan   = new GUIStyle(GUI.skin.label) { fontSize = 12, fontStyle = FontStyle.Bold, wordWrap = true, normal = { textColor = ColorTextCyan } };
        StyleLabelGreen  = new GUIStyle(GUI.skin.label) { fontSize = 12, fontStyle = FontStyle.Bold, wordWrap = true, normal = { textColor = ColorTextGreen } };
        StyleLabelYellow = new GUIStyle(GUI.skin.label) { fontSize = 12, fontStyle = FontStyle.Bold, wordWrap = true, normal = { textColor = ColorTextYellow } };
        StyleLabelRed    = new GUIStyle(GUI.skin.label) { fontSize = 12, fontStyle = FontStyle.Bold, wordWrap = true, normal = { textColor = ColorTextRed } };

        // Thanh cuộn tối tinh gọn (mảnh 8px thay vì thanh xám to mặc định của Unity)
        StyleVerticalScrollbar = new GUIStyle(GUI.skin.verticalScrollbar)
        {
            fixedWidth = 8f,
            normal = { background = TexScrollTrack }
        };
        StyleVerticalScrollbarThumb = new GUIStyle(GUI.skin.verticalScrollbarThumb)
        {
            fixedWidth = 8f,
            normal = { background = TexScrollThumb },
            hover = { background = TexSliderThumb },
            active = { background = TexSliderThumb }
        };
        StyleHorizontalScrollbar = new GUIStyle(GUI.skin.horizontalScrollbar)
        {
            fixedHeight = 8f,
            normal = { background = TexScrollTrack }
        };
        StyleHorizontalScrollbarThumb = new GUIStyle(GUI.skin.horizontalScrollbarThumb)
        {
            fixedHeight = 8f,
            normal = { background = TexScrollThumb },
            hover = { background = TexSliderThumb },
            active = { background = TexSliderThumb }
        };

        cachedCardStyle = null;
        cachedInnerStyle = null;
        initialized = true;
    }

    public static void ApplyDarkScrollbarsToCurrentSkin()
    {
        EnsureInitialized();
        GUI.skin.verticalScrollbar = StyleVerticalScrollbar;
        GUI.skin.verticalScrollbarThumb = StyleVerticalScrollbarThumb;
        GUI.skin.horizontalScrollbar = StyleHorizontalScrollbar;
        GUI.skin.horizontalScrollbarThumb = StyleHorizontalScrollbarThumb;
        GUI.skin.verticalScrollbarUpButton = GUIStyle.none;
        GUI.skin.verticalScrollbarDownButton = GUIStyle.none;
        GUI.skin.horizontalScrollbarLeftButton = GUIStyle.none;
        GUI.skin.horizontalScrollbarRightButton = GUIStyle.none;
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

    public static GUIStyle CardBoxStyle()
    {
        EnsureInitialized();
        if (cachedCardStyle != null) return cachedCardStyle;
        cachedCardStyle = new GUIStyle(GUI.skin.box)
        {
            normal = { background = TexBgCard },
            margin = new RectOffset(0, 0, 2, 2),
            padding = new RectOffset(12, 12, 10, 10)
        };
        return cachedCardStyle;
    }

    public static GUIStyle InnerBoxStyle()
    {
        EnsureInitialized();
        if (cachedInnerStyle != null) return cachedInnerStyle;
        cachedInnerStyle = new GUIStyle(GUI.skin.box)
        {
            normal = { background = TexBgCardInner },
            margin = new RectOffset(0, 0, 3, 3),
            padding = new RectOffset(8, 8, 6, 6)
        };
        return cachedInnerStyle;
    }

    public static bool RibbonTab(string text, bool isCurrent, float width = 145f, float height = 30f)
    {
        EnsureInitialized();
        GUIStyle style = isCurrent ? StyleTabActive : StyleTabInactive;
        return GUILayout.Button(text, style, GUILayout.Width(width), GUILayout.Height(height));
    }

    public static bool Button(string text, float height = 30f, float width = 0f)
    {
        EnsureInitialized();
        if (width > 0f)
            return GUILayout.Button(text, StyleButton, GUILayout.Width(width), GUILayout.Height(height));
        return GUILayout.Button(text, StyleButton, GUILayout.Height(height));
    }

    public static bool PrimaryButton(string text, float height = 30f, float width = 0f)
    {
        EnsureInitialized();
        if (width > 0f)
            return GUILayout.Button(text, StylePrimaryButton, GUILayout.Width(width), GUILayout.Height(height));
        return GUILayout.Button(text, StylePrimaryButton, GUILayout.Height(height));
    }
}
