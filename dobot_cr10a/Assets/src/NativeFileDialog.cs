using System;
using System.IO;
using System.Runtime.InteropServices;
using UnityEngine;

/// <summary>
/// Hộp thoại chọn file / lưu file / chọn thư mục gốc của hệ điều hành,
/// hoạt động đầy đủ cả trong Unity Editor lẫn bản xuất ứng dụng Windows (.exe - Mono & IL2CPP).
/// </summary>
public static class NativeFileDialog
{
#if !UNITY_EDITOR && UNITY_STANDALONE_WIN
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void EnsureWindowedOnStartup()
    {
        // Đảm bảo bản xuất .exe luôn mở ở chế độ cửa sổ (kể cả khi Windows Registry còn lưu trạng thái Fullscreen cũ)
        if (Screen.fullScreenMode != FullScreenMode.Windowed)
        {
            int maxW = Mathf.Max(1024, Display.main.systemWidth - 100);
            int maxH = Mathf.Max(640, Display.main.systemHeight - 120);
            int targetW = Mathf.Min(1440, maxW);
            int targetH = Mathf.Min(860, maxH);
            Screen.SetResolution(targetW, targetH, FullScreenMode.Windowed);
        }
        Application.runInBackground = true;
    }
#endif

    public static string OpenFile(string title, string initialDirectory, string extension)
    {
        string cleanDir = ResolveValidInitialDir(initialDirectory);
        string cleanExt = CleanExtension(extension);

#if UNITY_EDITOR
        return UnityEditor.EditorUtility.OpenFilePanel(title ?? "Chọn file", cleanDir, cleanExt);
#elif UNITY_STANDALONE_WIN
        return Win32OpenOrSaveFile(isSave: false, title, cleanDir, "", cleanExt);
#else
        Debug.LogWarning("[NativeFileDialog] OpenFile chưa hỗ trợ trên nền tảng này.");
        return "";
#endif
    }

    /// <summary>
    /// Mở hộp thoại Windows File Explorer cho phép chọn NHIỀU file cùng lúc (Ctrl/Shift + Click).
    /// Hoạt động trên cả Unity Editor (Windows) và bản xuất .exe Windows.
    /// </summary>
    public static string[] OpenFiles(string title, string initialDirectory, string extension)
    {
        string cleanDir = ResolveValidInitialDir(initialDirectory);
        string cleanExt = CleanExtension(extension);

#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
        return Win32OpenMultipleFiles(title, cleanDir, cleanExt);
#else
        string single = OpenFile(title, cleanDir, cleanExt);
        return string.IsNullOrEmpty(single) ? new string[0] : new string[] { single };
#endif
    }

    public static void OpenFolderInExplorer(string folderPath)
    {
        try
        {
            if (string.IsNullOrWhiteSpace(folderPath)) return;
            string full = Path.GetFullPath(folderPath);
            if (!Directory.Exists(full)) Directory.CreateDirectory(full);
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            System.Diagnostics.Process.Start("explorer.exe", $"\"{full}\"");
#else
            Application.OpenURL("file://" + full);
#endif
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[NativeFileDialog] Không thể mở thư mục: {ex.Message}");
        }
    }

    public static string SaveFile(string title, string initialDirectory, string defaultFileName, string extension)
    {
        string cleanDir = ResolveValidInitialDir(initialDirectory);
        string cleanExt = CleanExtension(extension);

#if UNITY_EDITOR
        return UnityEditor.EditorUtility.SaveFilePanel(title ?? "Lưu file", cleanDir, defaultFileName ?? "", cleanExt);
#elif UNITY_STANDALONE_WIN
        return Win32OpenOrSaveFile(isSave: true, title, cleanDir, defaultFileName ?? "", cleanExt);
#else
        Debug.LogWarning("[NativeFileDialog] SaveFile chưa hỗ trợ trên nền tảng này.");
        return "";
#endif
    }

    public static string OpenFolder(string title, string initialDirectory)
    {
        string cleanDir = ResolveValidInitialDir(initialDirectory);

#if UNITY_EDITOR
        return UnityEditor.EditorUtility.OpenFolderPanel(title ?? "Chọn thư mục", cleanDir, "");
#elif UNITY_STANDALONE_WIN
        return Win32OpenFolder(title, cleanDir);
#else
        Debug.LogWarning("[NativeFileDialog] OpenFolder chưa hỗ trợ trên nền tảng này.");
        return "";
#endif
    }

    private static string CleanExtension(string ext)
    {
        if (string.IsNullOrWhiteSpace(ext)) return "";
        return ext.Trim().TrimStart('.', '*');
    }

    private static string ResolveValidInitialDir(string dir)
    {
        try
        {
            if (!string.IsNullOrWhiteSpace(dir))
            {
                string full = Path.GetFullPath(dir);
                if (Directory.Exists(full)) return full;
            }
        }
        catch { }

        try
        {
            string docs = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
            if (!string.IsNullOrEmpty(docs) && Directory.Exists(docs)) return docs;
        }
        catch { }

        return "";
    }

#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
    private const int OFN_OVERWRITEPROMPT = 0x00000002;
    private const int OFN_NOCHANGEDIR     = 0x00000008;
    private const int OFN_ALLOWMULTISELECT = 0x00000200;
    private const int OFN_PATHMUSTEXIST   = 0x00000800;
    private const int OFN_FILEMUSTEXIST   = 0x00001000;
    private const int OFN_EXPLORER        = 0x00080000;

    private const uint BIF_RETURNONLYFSDIRS = 0x00000001;
    private const uint BIF_NEWDIALOGSTYLE   = 0x00000040;

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    private struct OpenFileName
    {
        public int lStructSize;
        public IntPtr hwndOwner;
        public IntPtr hInstance;
        public string lpstrFilter;
        public string lpstrCustomFilter;
        public int nMaxCustFilter;
        public int nFilterIndex;
        public IntPtr lpstrFile;
        public int nMaxFile;
        public IntPtr lpstrFileTitle;
        public int nMaxFileTitle;
        public string lpstrInitialDir;
        public string lpstrTitle;
        public int Flags;
        public short nFileOffset;
        public short nFileExtension;
        public string lpstrDefExt;
        public IntPtr lCustData;
        public IntPtr lpfnHook;
        public string lpTemplateName;
        public IntPtr pvReserved;
        public int dwReserved;
        public int FlagsEx;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    private struct BrowseInfo
    {
        public IntPtr hwndOwner;
        public IntPtr pidlRoot;
        public IntPtr pszDisplayName;
        public string lpszTitle;
        public uint ulFlags;
        public IntPtr lpfn;
        public IntPtr lParam;
        public int iImage;
    }

    [DllImport("user32.dll")]
    private static extern IntPtr GetActiveWindow();

    [DllImport("comdlg32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool GetOpenFileNameW(ref OpenFileName ofn);

    [DllImport("comdlg32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool GetSaveFileNameW(ref OpenFileName ofn);

    [DllImport("shell32.dll", CharSet = CharSet.Unicode)]
    private static extern IntPtr SHBrowseForFolderW(ref BrowseInfo bi);

    [DllImport("shell32.dll", CharSet = CharSet.Unicode)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool SHGetPathFromIDListW(IntPtr pidl, IntPtr pszPath);

    [DllImport("ole32.dll")]
    private static extern void CoTaskMemFree(IntPtr pv);

    private static string Win32OpenOrSaveFile(bool isSave, string title, string initialDir, string defaultFileName, string ext)
    {
        const int maxChars = 4096;
        IntPtr fileBuf = IntPtr.Zero;
        try
        {
            fileBuf = Marshal.AllocHGlobal(maxChars * 2);
            // Xóa sạch buffer và ghi sẵn tên file mặc định nếu là hộp thoại Lưu
            for (int i = 0; i < maxChars * 2; i++) Marshal.WriteByte(fileBuf, i, 0);
            if (!string.IsNullOrEmpty(defaultFileName))
            {
                char[] chars = defaultFileName.ToCharArray();
                int len = Math.Min(chars.Length, maxChars - 1);
                Marshal.Copy(chars, 0, fileBuf, len);
            }

            string filter = string.IsNullOrEmpty(ext)
                ? "All Files (*.*)\0*.*\0\0"
                : $"{ext.ToUpperInvariant()} Files (*.{ext})\0*.{ext}\0All Files (*.*)\0*.*\0\0";

            OpenFileName ofn = new OpenFileName
            {
                lStructSize = Marshal.SizeOf(typeof(OpenFileName)),
                hwndOwner = GetActiveWindow(),
                hInstance = IntPtr.Zero,
                lpstrFilter = filter,
                nFilterIndex = 1,
                lpstrFile = fileBuf,
                nMaxFile = maxChars,
                lpstrInitialDir = string.IsNullOrEmpty(initialDir) ? null : initialDir.Replace('/', '\\'),
                lpstrTitle = string.IsNullOrEmpty(title) ? (isSave ? "Lưu file" : "Chọn file") : title,
                lpstrDefExt = string.IsNullOrEmpty(ext) ? null : ext,
                Flags = isSave
                    ? (OFN_EXPLORER | OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR)
                    : (OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR)
            };

            bool ok = isSave ? GetSaveFileNameW(ref ofn) : GetOpenFileNameW(ref ofn);
            if (!ok) return "";
            return Marshal.PtrToStringUni(fileBuf) ?? "";
        }
        catch (Exception ex)
        {
            Debug.LogError($"[NativeFileDialog] Lỗi mở hộp thoại Windows: {ex.Message}");
            return "";
        }
        finally
        {
            if (fileBuf != IntPtr.Zero) Marshal.FreeHGlobal(fileBuf);
        }
    }

    private static string[] Win32OpenMultipleFiles(string title, string initialDir, string ext)
    {
        const int maxChars = 65536;
        IntPtr fileBuf = IntPtr.Zero;
        try
        {
            fileBuf = Marshal.AllocHGlobal(maxChars * 2);
            // Xóa nhanh đầu buffer (2 ký tự null đầu là đủ cho Win32, vẫn zero vài byte an toàn)
            Marshal.WriteInt64(fileBuf, 0, 0);

            string filter = string.IsNullOrEmpty(ext)
                ? "All Files (*.*)\0*.*\0\0"
                : $"{ext.ToUpperInvariant()} Files (*.{ext})\0*.{ext}\0All Files (*.*)\0*.*\0\0";

            OpenFileName ofn = new OpenFileName
            {
                lStructSize = Marshal.SizeOf(typeof(OpenFileName)),
                hwndOwner = GetActiveWindow(),
                hInstance = IntPtr.Zero,
                lpstrFilter = filter,
                nFilterIndex = 1,
                lpstrFile = fileBuf,
                nMaxFile = maxChars,
                lpstrInitialDir = string.IsNullOrEmpty(initialDir) ? null : initialDir.Replace('/', '\\'),
                lpstrTitle = string.IsNullOrEmpty(title) ? "Chọn nhiều file" : title,
                lpstrDefExt = string.IsNullOrEmpty(ext) ? null : ext,
                Flags = OFN_EXPLORER | OFN_ALLOWMULTISELECT | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR
            };

            if (!GetOpenFileNameW(ref ofn)) return new string[0];

            var parts = new System.Collections.Generic.List<string>();
            int offsetChars = 0;
            while (offsetChars < maxChars)
            {
                IntPtr ptr = IntPtr.Add(fileBuf, offsetChars * 2);
                string token = Marshal.PtrToStringUni(ptr);
                if (string.IsNullOrEmpty(token)) break;
                parts.Add(token);
                offsetChars += token.Length + 1;
            }

            if (parts.Count == 0) return new string[0];
            if (parts.Count == 1) return new string[] { parts[0] };

            string folder = parts[0];
            string[] result = new string[parts.Count - 1];
            for (int i = 1; i < parts.Count; i++)
            {
                result[i - 1] = Path.Combine(folder, parts[i]);
            }
            return result;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[NativeFileDialog] Lỗi mở hộp thoại chọn nhiều file Windows: {ex.Message}");
            return new string[0];
        }
        finally
        {
            if (fileBuf != IntPtr.Zero) Marshal.FreeHGlobal(fileBuf);
        }
    }

    private static string Win32OpenFolder(string title, string initialDir)
    {
        const int maxChars = 4096;
        IntPtr displayBuf = IntPtr.Zero;
        IntPtr pathBuf = IntPtr.Zero;
        IntPtr pidl = IntPtr.Zero;
        try
        {
            displayBuf = Marshal.AllocHGlobal(maxChars * 2);
            pathBuf = Marshal.AllocHGlobal(maxChars * 2);
            for (int i = 0; i < maxChars * 2; i++)
            {
                Marshal.WriteByte(displayBuf, i, 0);
                Marshal.WriteByte(pathBuf, i, 0);
            }

            BrowseInfo bi = new BrowseInfo
            {
                hwndOwner = GetActiveWindow(),
                pidlRoot = IntPtr.Zero,
                pszDisplayName = displayBuf,
                lpszTitle = string.IsNullOrEmpty(title) ? "Chọn thư mục" : title,
                ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE
            };

            pidl = SHBrowseForFolderW(ref bi);
            if (pidl == IntPtr.Zero) return "";
            if (!SHGetPathFromIDListW(pidl, pathBuf)) return "";
            return Marshal.PtrToStringUni(pathBuf) ?? "";
        }
        catch (Exception ex)
        {
            Debug.LogError($"[NativeFileDialog] Lỗi mở hộp thoại thư mục Windows: {ex.Message}");
            return "";
        }
        finally
        {
            if (pidl != IntPtr.Zero) CoTaskMemFree(pidl);
            if (displayBuf != IntPtr.Zero) Marshal.FreeHGlobal(displayBuf);
            if (pathBuf != IntPtr.Zero) Marshal.FreeHGlobal(pathBuf);
        }
    }
#endif
}
