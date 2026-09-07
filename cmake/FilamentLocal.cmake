set(VOLUME_SURFACE_FILAMENT_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/.deps/filament-main"
    CACHE PATH
    "Filament source tree prepared inside the workspace"
)

if(NOT EXISTS "${VOLUME_SURFACE_FILAMENT_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR
        "Filament source was not found. Run scripts/prepare_filament.ps1 first: "
        "${VOLUME_SURFACE_FILAMENT_ROOT}"
    )
endif()

set(FILAMENT_SKIP_SAMPLES OFF CACHE BOOL "" FORCE)
set(FILAMENT_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(FILAMENT_ENABLE_LTO OFF CACHE BOOL "" FORCE)
set(FILAMENT_SUPPORTS_VULKAN OFF CACHE BOOL "" FORCE)
set(FILAMENT_SUPPORTS_OPENGL ON CACHE BOOL "" FORCE)
set(FILAMENT_SUPPORTS_WEBGPU OFF CACHE BOOL "" FORCE)
set(FILAMENT_ENABLE_MATDBG OFF CACHE BOOL "" FORCE)
set(USE_STATIC_CRT OFF CACHE BOOL "" FORCE)

# Dear ImGui defaults to Latin glyph ranges even when a CJK-capable font is loaded.
# Patch the bundled Filagui helper once so the viewer's UTF-8 labels include common
# simplified Chinese glyphs after a fresh dependency extraction.
set(_filament_imgui_helper
    "${VOLUME_SURFACE_FILAMENT_ROOT}/libs/filagui/src/ImGuiHelper.cpp")
if(EXISTS "${_filament_imgui_helper}")
    file(READ "${_filament_imgui_helper}" _filament_imgui_source)
    set(_filament_imgui_legacy_call
        "io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 16.0f);")
    set(_filament_imgui_cjk_call
        "io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 16.0f, nullptr, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());")
    string(FIND
        "${_filament_imgui_source}"
        "${_filament_imgui_legacy_call}"
        _filament_imgui_legacy_position)
    if(NOT _filament_imgui_legacy_position EQUAL -1)
        string(REPLACE
            "${_filament_imgui_legacy_call}"
            "${_filament_imgui_cjk_call}"
            _filament_imgui_source
            "${_filament_imgui_source}")
        file(WRITE "${_filament_imgui_helper}" "${_filament_imgui_source}")
    endif()
endif()

add_subdirectory(
    "${VOLUME_SURFACE_FILAMENT_ROOT}"
    "${CMAKE_CURRENT_BINARY_DIR}/filament"
    EXCLUDE_FROM_ALL
)
