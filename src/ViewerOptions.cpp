#include "volume_surface/viewer/ViewerOptions.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#endif

namespace volume_surface::viewer {
namespace {

std::string trim(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool readJsonString(
    const std::string& json,
    const char* key,
    std::string& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) {
        return false;
    }
    const std::size_t colonPosition = json.find(
        ':', markerPosition + marker.size());
    const std::size_t quotePosition = json.find(
        '"', colonPosition == std::string::npos ? markerPosition : colonPosition + 1);
    if (colonPosition == std::string::npos || quotePosition == std::string::npos) {
        return false;
    }
    std::string parsed;
    bool escaped = false;
    for (std::size_t index = quotePosition + 1; index < json.size(); ++index) {
        const char character = json[index];
        if (escaped) {
            switch (character) {
                case '\\': parsed += '\\'; break;
                case '"': parsed += '"'; break;
                case 'n': parsed += '\n'; break;
                case 'r': parsed += '\r'; break;
                case 't': parsed += '\t'; break;
                default: parsed += character; break;
            }
            escaped = false;
        } else if (character == '\\') {
            escaped = true;
        } else if (character == '"') {
            value = std::move(parsed);
            return true;
        } else {
            parsed += character;
        }
    }
    return false;
}

bool readJsonBoolean(
    const std::string& json,
    const char* key,
    bool& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) {
        return false;
    }
    const std::size_t colonPosition = json.find(
        ':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) {
        return false;
    }
    const std::string remainder = json.substr(colonPosition + 1);
    const std::size_t truePosition = remainder.find("true");
    const std::size_t falsePosition = remainder.find("false");
    if (truePosition != std::string::npos &&
        (falsePosition == std::string::npos || truePosition < falsePosition)) {
        value = true;
        return true;
    }
    if (falsePosition != std::string::npos) {
        value = false;
        return true;
    }
    return false;
}

std::filesystem::path executableDirectory()
{
#ifdef _WIN32
    std::array<wchar_t, 32768> buffer{};
    const DWORD length = GetModuleFileNameW(
        nullptr,
        buffer.data(),
        static_cast<DWORD>(buffer.size()));
    if (length > 0 && length < buffer.size()) {
        return std::filesystem::path(
            std::wstring(buffer.data(), static_cast<std::size_t>(length)))
            .parent_path();
    }
#endif
    return {};
}

std::filesystem::path resolveCatalogEntryPath(
    const std::filesystem::path& rawPath,
    const std::filesystem::path& catalogPath)
{
    if (rawPath.is_absolute()) {
        return rawPath.lexically_normal();
    }
    const std::filesystem::path base = catalogPath.parent_path().empty()
        ? std::filesystem::current_path()
        : catalogPath.parent_path();
    return (base / rawPath).lexically_normal();
}

std::vector<ViewerInputEntry> fallbackViewerInputs()
{
    const std::array<ViewerInputEntry, 2> candidates{
        ViewerInputEntry{"male", R"(G:\images\male.vdb)", true},
        ViewerInputEntry{
            "rightArm.0224",
            R"(G:\transformed\rightArm\rightArm.0224.vdb)",
            false}};
    std::vector<ViewerInputEntry> entries;
    std::error_code errorCode;
    for (const auto& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate.path, errorCode)) {
            entries.push_back(candidate);
        }
        errorCode.clear();
    }
    return entries;
}

#ifdef _WIN32

constexpr int kInputComboId = 1001;

struct InputSelectorWindowState {
    std::vector<ViewerInputEntry> entries;
    HWND window = nullptr;
    HWND combo = nullptr;
    HWND pathLabel = nullptr;
    std::filesystem::path selection;
};

std::wstring utf8ToWide(const std::string& value)
{
    if (value.empty()) {
        return {};
    }
    const int required = MultiByteToWideChar(
        CP_UTF8,
        MB_ERR_INVALID_CHARS,
        value.data(),
        static_cast<int>(value.size()),
        nullptr,
        0);
    if (required <= 0) {
        return std::wstring(value.begin(), value.end());
    }
    std::wstring result(static_cast<std::size_t>(required), L'\0');
    MultiByteToWideChar(
        CP_UTF8,
        MB_ERR_INVALID_CHARS,
        value.data(),
        static_cast<int>(value.size()),
        result.data(),
        required);
    return result;
}

void updateInputSelectorPath(InputSelectorWindowState& state)
{
    const LRESULT selectedIndex = SendMessageW(
        state.combo,
        CB_GETCURSEL,
        0,
        0);
    if (selectedIndex < 0 ||
        static_cast<std::size_t>(selectedIndex) >= state.entries.size()) {
        return;
    }
    const std::wstring path = state.entries[
        static_cast<std::size_t>(selectedIndex)].path.wstring();
    const std::wstring text = L"Path: " + path;
    SetWindowTextW(state.pathLabel, text.c_str());
}

LRESULT CALLBACK inputSelectorWindowProc(
    HWND window,
    UINT message,
    WPARAM wParam,
    LPARAM lParam)
{
    auto* state = reinterpret_cast<InputSelectorWindowState*>(
        GetWindowLongPtrW(window, GWLP_USERDATA));
    if (message == WM_NCCREATE) {
        const auto* create = reinterpret_cast<CREATESTRUCTW*>(lParam);
        state = static_cast<InputSelectorWindowState*>(create->lpCreateParams);
        SetWindowLongPtrW(
            window,
            GWLP_USERDATA,
            reinterpret_cast<LONG_PTR>(state));
        state->window = window;
    }

    switch (message) {
        case WM_CREATE: {
            const HINSTANCE instance = reinterpret_cast<HINSTANCE>(
                GetWindowLongPtrW(window, GWLP_HINSTANCE));
            const HFONT font = static_cast<HFONT>(
                GetStockObject(DEFAULT_GUI_FONT));
            auto createControl = [&](DWORD extendedStyle,
                                     LPCWSTR className,
                                     LPCWSTR text,
                                     DWORD style,
                                     int x,
                                     int y,
                                     int width,
                                     int height,
                                     int id) {
                HWND control = CreateWindowExW(
                    extendedStyle,
                    className,
                    text,
                    style,
                    x,
                    y,
                    width,
                    height,
                    window,
                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
                    instance,
                    nullptr);
                if (control) {
                    SendMessageW(
                        control,
                        WM_SETFONT,
                        reinterpret_cast<WPARAM>(font),
                        TRUE);
                }
                return control;
            };

            createControl(
                0,
                L"STATIC",
                L"Select a VDB input before entering the viewer:",
                WS_CHILD | WS_VISIBLE,
                20,
                18,
                490,
                22,
                0);
            state->combo = createControl(
                WS_EX_CLIENTEDGE,
                L"COMBOBOX",
                L"",
                WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
                20,
                48,
                490,
                220,
                kInputComboId);
            state->pathLabel = createControl(
                0,
                L"STATIC",
                L"Path:",
                WS_CHILD | WS_VISIBLE | SS_LEFT | SS_NOPREFIX,
                20,
                84,
                490,
                22,
                1002);
            createControl(
                0,
                L"BUTTON",
                L"Open",
                WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
                330,
                125,
                85,
                30,
                IDOK);
            createControl(
                0,
                L"BUTTON",
                L"Cancel",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                425,
                125,
                85,
                30,
                IDCANCEL);

            for (const auto& entry : state->entries) {
                const std::wstring label = utf8ToWide(entry.label);
                SendMessageW(
                    state->combo,
                    CB_ADDSTRING,
                    0,
                    reinterpret_cast<LPARAM>(label.c_str()));
            }
            std::size_t defaultIndex = 0;
            for (std::size_t index = 0; index < state->entries.size(); ++index) {
                if (state->entries[index].defaultSelected) {
                    defaultIndex = index;
                    break;
                }
            }
            SendMessageW(
                state->combo,
                CB_SETCURSEL,
                static_cast<WPARAM>(defaultIndex),
                0);
            updateInputSelectorPath(*state);
            return 0;
        }
        case WM_COMMAND:
            if (LOWORD(wParam) == kInputComboId &&
                HIWORD(wParam) == CBN_SELCHANGE) {
                updateInputSelectorPath(*state);
                return 0;
            }
            if (LOWORD(wParam) == IDOK) {
                const LRESULT selectedIndex = SendMessageW(
                    state->combo,
                    CB_GETCURSEL,
                    0,
                    0);
                if (selectedIndex >= 0 &&
                    static_cast<std::size_t>(selectedIndex) < state->entries.size()) {
                    state->selection = state->entries[
                        static_cast<std::size_t>(selectedIndex)].path;
                }
                DestroyWindow(window);
                return 0;
            }
            if (LOWORD(wParam) == IDCANCEL) {
                state->selection.clear();
                DestroyWindow(window);
                return 0;
            }
            break;
        case WM_CLOSE:
            state->selection.clear();
            DestroyWindow(window);
            return 0;
        default:
            break;
    }
    return DefWindowProcW(window, message, wParam, lParam);
}

std::filesystem::path showInputSelector(
    const std::vector<ViewerInputEntry>& entries)
{
    if (entries.empty()) {
        return {};
    }
    const HINSTANCE instance = GetModuleHandleW(nullptr);
    constexpr wchar_t className[] = L"VolumeSurfaceInputSelector";
    WNDCLASSW windowClass{};
    windowClass.hInstance = instance;
    windowClass.lpfnWndProc = inputSelectorWindowProc;
    windowClass.lpszClassName = className;
    windowClass.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
    windowClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&windowClass);

    InputSelectorWindowState state;
    state.entries = entries;
    HWND window = CreateWindowExW(
        WS_EX_DLGMODALFRAME,
        className,
        L"VolumeSurface - Select Input",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        550,
        195,
        nullptr,
        nullptr,
        instance,
        &state);
    if (!window) {
        return {};
    }

    RECT workArea{};
    SystemParametersInfoW(SPI_GETWORKAREA, 0, &workArea, 0);
    constexpr int width = 550;
    constexpr int height = 195;
    const int workWidth = static_cast<int>(workArea.right - workArea.left);
    const int workHeight = static_cast<int>(workArea.bottom - workArea.top);
    const int x = static_cast<int>(workArea.left) +
        std::max(0, (workWidth - width) / 2);
    const int y = static_cast<int>(workArea.top) +
        std::max(0, (workHeight - height) / 2);
    SetWindowPos(window, HWND_TOP, x, y, width, height, SWP_SHOWWINDOW);
    SetForegroundWindow(window);

    MSG message{};
    while (IsWindow(window)) {
        const BOOL result = GetMessageW(&message, nullptr, 0, 0);
        if (result <= 0) {
            break;
        }
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return state.selection;
}

#endif

} // namespace

ViewerOptions parseViewerOptions(int argc, char** argv)
{
    ViewerOptions options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto requireValue = [&](const char* option) -> std::string {
            if (++index >= argc) {
                throw std::invalid_argument(
                    std::string("Missing value for ") + option);
            }
            return argv[index];
        };

        if (argument == "--input") {
            options.input = requireValue("--input");
        } else if (argument == "--grid") {
            options.gridName = requireValue("--grid");
        } else if (argument == "--iso") {
            options.isoValue = std::stod(requireValue("--iso"));
        } else if (argument == "--adaptivity") {
            options.adaptivity = std::stod(requireValue("--adaptivity"));
        } else if (argument == "--inspect-only") {
            options.inspectOnly = true;
        } else if (argument == "--headless-smoke") {
            options.headlessSmoke = true;
        } else if (argument == "--replay-brush-profile") {
            options.replayBrushProfile = requireValue("--replay-brush-profile");
        } else if (argument == "--help") {
            std::cout
                << "volume_surface_viewer [--input file.vdb] [--grid density] "
                   "[--iso 255] [--adaptivity 0.1] [--inspect-only] [--headless-smoke] "
                   "[--replay-brush-profile brush_profile.jsonl]\n"
                   "If --input is omitted, viewer_inputs.jsonl is shown in a startup selector.\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument("Unknown argument: " + argument);
        }
    }
    return options;
}

std::filesystem::path viewerInputCatalogPath()
{
    const std::filesystem::path fileName = "viewer_inputs.jsonl";
    const std::filesystem::path current = std::filesystem::current_path();
    const std::filesystem::path executable = executableDirectory();
    const std::array<std::filesystem::path, 4> candidates{
        current / fileName,
        current.parent_path() / fileName,
        executable / fileName,
        executable.parent_path() / fileName};
    std::error_code errorCode;
    for (const auto& candidate : candidates) {
        if (!candidate.empty() &&
            std::filesystem::is_regular_file(candidate, errorCode)) {
            return candidate;
        }
        errorCode.clear();
    }
    return current / fileName;
}

std::vector<ViewerInputEntry> loadViewerInputCatalog(
    const std::filesystem::path& catalogPath,
    std::string& status)
{
    status.clear();
    std::vector<ViewerInputEntry> entries;
    std::ifstream input(catalogPath);
    if (!input) {
        status = "Input catalog not found: " + catalogPath.string();
        return entries;
    }

    std::string line;
    std::size_t skippedCount = 0;
    while (std::getline(input, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty()) {
            continue;
        }
        ViewerInputEntry entry;
        std::string rawPath;
        bool defaultSelected = false;
        if (!readJsonString(trimmed, "path", rawPath) || rawPath.empty()) {
            ++skippedCount;
            continue;
        }
        std::string label;
        readJsonString(trimmed, "label", label);
        readJsonBoolean(trimmed, "default", defaultSelected);
        entry.path = resolveCatalogEntryPath(rawPath, catalogPath);
        entry.label = label.empty() ? entry.path.filename().string() : label;
        entry.defaultSelected = defaultSelected;

        std::error_code errorCode;
        if (!std::filesystem::is_regular_file(entry.path, errorCode)) {
            ++skippedCount;
            continue;
        }
        entries.push_back(std::move(entry));
    }
    if (entries.empty()) {
        status = "Input catalog contains no existing VDB files: " +
            catalogPath.string();
    } else if (skippedCount > 0) {
        status = "Input catalog skipped " + std::to_string(skippedCount) +
            " invalid or missing entries";
    } else {
        status = "Loaded " + std::to_string(entries.size()) +
            " input entries from " + catalogPath.string();
    }
    return entries;
}

std::filesystem::path selectVdbInputFromCatalog(
    const std::filesystem::path& catalogPath)
{
    const std::filesystem::path resolvedCatalog = catalogPath.empty()
        ? viewerInputCatalogPath()
        : catalogPath;
    std::string status;
    std::vector<ViewerInputEntry> entries = loadViewerInputCatalog(
        resolvedCatalog,
        status);
    if (entries.empty()) {
        entries = fallbackViewerInputs();
    }
#ifdef _WIN32
    return showInputSelector(entries);
#else
    (void)status;
    return {};
#endif
}

} // namespace volume_surface::viewer
