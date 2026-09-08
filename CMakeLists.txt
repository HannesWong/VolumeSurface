cmake_minimum_required(VERSION 3.24)

project(VolumeSurface LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(VOLUME_SURFACE_BUILD_TESTS "Build VolumeSurface tests" ON)
option(VOLUME_SURFACE_BUILD_VIEWER "Build the Filament diagnostic viewer" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(OpenVDBLocal)

add_library(volume_surface_bricks
    src/BrickArena.cpp
)

target_include_directories(volume_surface_bricks
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(volume_surface_bricks
    PUBLIC
        OpenVDB::openvdb
)

target_compile_definitions(volume_surface_bricks
    PRIVATE
        NOMINMAX
)

add_library(volume_surface_mesh
    src/SurfaceMesh.cpp
    src/SurfaceBrush.cpp
    src/SliceDiagnostics.cpp
    src/SurfaceTarget.cpp
    src/SurfaceReconstruction.cpp
    src/SurfaceNormalField.cpp
    src/SurfaceMeshContinuity.cpp
    src/VdbSurfaceProbe.cpp
)

target_include_directories(volume_surface_mesh
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(volume_surface_mesh
    PUBLIC
        OpenVDB::openvdb
)

target_compile_definitions(volume_surface_mesh
    PRIVATE
        NOMINMAX
)

# Keep ABI-sensitive objects in sync even when localized MSVC include output
# cannot be parsed by Ninja's dependency scanner.
set(_surface_target_header
    "${CMAKE_CURRENT_SOURCE_DIR}/include/volume_surface/SurfaceTarget.h")
set(_surface_normal_field_header
    "${CMAKE_CURRENT_SOURCE_DIR}/include/volume_surface/SurfaceNormalField.h")
set(_surface_mesh_continuity_header
    "${CMAKE_CURRENT_SOURCE_DIR}/include/volume_surface/SurfaceMeshContinuity.h")
set_source_files_properties(
    src/SurfaceTarget.cpp
    src/SurfaceReconstruction.cpp
    src/SurfaceTargetCacheRepository.cpp
    src/reconstruction_info.cpp
    src/normal_diagnostics_info.cpp
    tests/SurfaceReconstructionTests.cpp
    tests/SurfaceTargetTests.cpp
    PROPERTIES OBJECT_DEPENDS
        "${_surface_target_header};${_surface_normal_field_header};${_surface_mesh_continuity_header}")

add_executable(vdb_brick_cache
    src/main.cpp
)

target_link_libraries(vdb_brick_cache
    PRIVATE
        volume_surface_bricks
)

volume_surface_copy_openvdb_runtime(vdb_brick_cache)

add_executable(vdb_surface_info
    src/surface_info.cpp
)

target_link_libraries(vdb_surface_info
    PRIVATE
        volume_surface_mesh
)

volume_surface_copy_openvdb_runtime(vdb_surface_info)

add_executable(vdb_surface_reconstruction_info
    src/reconstruction_info.cpp
)

target_link_libraries(vdb_surface_reconstruction_info
    PRIVATE
        volume_surface_mesh
)

volume_surface_copy_openvdb_runtime(vdb_surface_reconstruction_info)

add_executable(surface_normal_diagnostics_info
    src/normal_diagnostics_info.cpp
    src/SurfaceTargetCacheRepository.cpp
    src/SurfaceNormalSeedStore.cpp
)

target_link_libraries(surface_normal_diagnostics_info
    PRIVATE
        volume_surface_mesh
)

volume_surface_copy_openvdb_runtime(surface_normal_diagnostics_info)

if(VOLUME_SURFACE_BUILD_VIEWER)
    include(FilamentLocal)

    set(_viewer_material_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/viewer_materials")
    set(_viewer_resource_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/viewer_resources")
    file(MAKE_DIRECTORY "${_viewer_material_dir}")
    file(MAKE_DIRECTORY "${_viewer_resource_dir}")

    set(_brush_heatmap_source "${CMAKE_CURRENT_SOURCE_DIR}/materials/brushHeatmap.mat")
    set(_brush_heatmap_package "${_viewer_material_dir}/brushHeatmap.filamat")
    set(_brush_cursor_source "${CMAKE_CURRENT_SOURCE_DIR}/materials/brushCursor.mat")
    set(_brush_cursor_package "${_viewer_material_dir}/brushCursor.filamat")
    set(_surface_target_points_source "${CMAKE_CURRENT_SOURCE_DIR}/materials/surfaceTargetPoints.mat")
    set(_surface_target_points_package "${_viewer_material_dir}/surfaceTargetPoints.filamat")
    set(_surface_target_point_pick_source "${CMAKE_CURRENT_SOURCE_DIR}/materials/surfaceTargetPointPick.mat")
    set(_surface_target_point_pick_package "${_viewer_material_dir}/surfaceTargetPointPick.filamat")
    set(_surface_target_debug_source "${CMAKE_CURRENT_SOURCE_DIR}/materials/surfaceTargetDebug.mat")
    set(_surface_target_debug_package "${_viewer_material_dir}/surfaceTargetDebug.filamat")
    set(_surface_fit_expansion_points_source
        "${CMAKE_CURRENT_SOURCE_DIR}/materials/surfaceFitExpansionDebugPoints.mat")
    set(_surface_fit_expansion_points_package
        "${_viewer_material_dir}/surfaceFitExpansionDebugPoints.filamat")
    set(_surface_fit_neighborhood_points_source
        "${CMAKE_CURRENT_SOURCE_DIR}/materials/surfaceFitNeighborhoodDebugPoints.mat")
    set(_surface_fit_neighborhood_points_package
        "${_viewer_material_dir}/surfaceFitNeighborhoodDebugPoints.filamat")
    set(_surface_fit_plane_source "${CMAKE_CURRENT_SOURCE_DIR}/materials/surfaceFitPlane.mat")
    set(_surface_fit_plane_package "${_viewer_material_dir}/surfaceFitPlane.filamat")
    add_custom_command(
        OUTPUT "${_brush_heatmap_package}"
        COMMAND matc ${MATC_BASE_FLAGS}
            -o "${_brush_heatmap_package}"
            "${_brush_heatmap_source}"
        MAIN_DEPENDENCY "${_brush_heatmap_source}"
        DEPENDS matc
        COMMENT "Compiling brush heatmap material"
        VERBATIM
    )
    add_custom_command(
        OUTPUT "${_brush_cursor_package}"
        COMMAND matc ${MATC_BASE_FLAGS}
            -o "${_brush_cursor_package}"
            "${_brush_cursor_source}"
        MAIN_DEPENDENCY "${_brush_cursor_source}"
        DEPENDS matc
        COMMENT "Compiling brush cursor material"
        VERBATIM
    )
    add_custom_command(
        OUTPUT "${_surface_target_points_package}"
        COMMAND matc ${MATC_BASE_FLAGS}
            -o "${_surface_target_points_package}"
            "${_surface_target_points_source}"
        MAIN_DEPENDENCY "${_surface_target_points_source}"
        DEPENDS matc
        COMMENT "Compiling SurfaceTarget point material"
        VERBATIM
    )
    add_custom_command(
        OUTPUT "${_surface_target_point_pick_package}"
        COMMAND matc ${MATC_BASE_FLAGS}
            -o "${_surface_target_point_pick_package}"
            "${_surface_target_point_pick_source}"
        MAIN_DEPENDENCY "${_surface_target_point_pick_source}"
        DEPENDS matc
        COMMENT "Compiling SurfaceTarget point pick material"
        VERBATIM
    )
    add_custom_command(
        OUTPUT "${_surface_target_debug_package}"
        COMMAND matc ${MATC_BASE_FLAGS}
            -o "${_surface_target_debug_package}"
            "${_surface_target_debug_source}"
        MAIN_DEPENDENCY "${_surface_target_debug_source}"
        DEPENDS matc
        COMMENT "Compiling SurfaceTarget debug material"
        VERBATIM
    )
    add_custom_command(
        OUTPUT "${_surface_fit_plane_package}"
        COMMAND matc ${MATC_BASE_FLAGS}
            -o "${_surface_fit_plane_package}"
            "${_surface_fit_plane_source}"
        MAIN_DEPENDENCY "${_surface_fit_plane_source}"
        DEPENDS matc
        COMMENT "Compiling SurfaceFit plane material"
        VERBATIM
    )
    add_custom_command(
        OUTPUT "${_surface_fit_expansion_points_package}"
        COMMAND matc ${MATC_BASE_FLAGS}
            -o "${_surface_fit_expansion_points_package}"
            "${_surface_fit_expansion_points_source}"
        MAIN_DEPENDENCY "${_surface_fit_expansion_points_source}"
        DEPENDS matc
        COMMENT "Compiling SurfaceFit expansion selected point material"
        VERBATIM
    )
    add_custom_command(
        OUTPUT "${_surface_fit_neighborhood_points_package}"
        COMMAND matc ${MATC_BASE_FLAGS}
            -o "${_surface_fit_neighborhood_points_package}"
            "${_surface_fit_neighborhood_points_source}"
        MAIN_DEPENDENCY "${_surface_fit_neighborhood_points_source}"
        DEPENDS matc
        COMMENT "Compiling Surface Fit neighborhood debug point material"
        VERBATIM
    )

    get_resgen_vars("${_viewer_resource_dir}" volume_surface_viewer_resources)
    add_custom_command(
        OUTPUT ${RESGEN_OUTPUTS}
        COMMAND resgen ${RESGEN_FLAGS}
            "${_brush_heatmap_package}"
            "${_brush_cursor_package}"
            "${_surface_target_points_package}"
            "${_surface_target_point_pick_package}"
            "${_surface_target_debug_package}"
            "${_surface_fit_expansion_points_package}"
            "${_surface_fit_neighborhood_points_package}"
            "${_surface_fit_plane_package}"
        DEPENDS resgen
            "${_brush_heatmap_package}"
            "${_brush_cursor_package}"
            "${_surface_target_points_package}"
            "${_surface_target_point_pick_package}"
            "${_surface_target_debug_package}"
            "${_surface_fit_expansion_points_package}"
            "${_surface_fit_neighborhood_points_package}"
            "${_surface_fit_plane_package}"
        COMMENT "Embedding viewer materials"
        VERBATIM
    )
    if(DEFINED RESGEN_SOURCE_FLAGS)
        set_source_files_properties(
            "${RESGEN_SOURCE}"
            PROPERTIES COMPILE_FLAGS "${RESGEN_SOURCE_FLAGS}"
        )
    endif()
    add_library(volume_surface_viewer_resources STATIC ${RESGEN_SOURCE})
    target_include_directories(
        volume_surface_viewer_resources
        PUBLIC "${_viewer_resource_dir}"
    )

    add_executable(volume_surface_viewer
        src/viewer.cpp
        src/BrushCursorRenderer.cpp
        src/CameraPickController.cpp
        src/SurfaceAwareCameraController.cpp
        src/BrushHeatmapRenderer.cpp
        src/BrushInteractionController.cpp
        src/BrushPaintingPanel.cpp
        src/BrushProfileRecorder.cpp
        src/BrushSettingsStore.cpp
        src/LightingController.cpp
        src/MeshRenderer.cpp
        src/PresentationController.cpp
        src/ReconstructionPanel.cpp
        src/ReconstructionPick.cpp
        src/SliceRenderer.cpp
        src/SurfaceTargetPreview.cpp
        src/SurfaceTargetPointPicker.cpp
        src/SurfaceFitPlaneRenderer.cpp
        src/SurfaceFitExpansionDebugRenderer.cpp
        src/SurfaceFitNeighborhoodDebugRenderer.cpp
        src/SurfaceNormalSeedStore.cpp
        src/SurfaceTargetCacheRepository.cpp
        src/WorkflowController.cpp
        src/WorkflowPanel.cpp
        src/WeightFieldRepository.cpp
        src/WeightPaintingStage.cpp
        src/ViewerOptions.cpp
    )

    target_link_libraries(volume_surface_viewer
        PRIVATE
            volume_surface_mesh
            volume_surface_viewer_resources
            filamentapp
    )

    target_compile_definitions(volume_surface_viewer
        PRIVATE
            NOMINMAX
    )

    set(_viewer_cjk_font "")
    if(WIN32 AND EXISTS "C:/Windows/Fonts/simhei.ttf")
        set(_viewer_cjk_font "C:/Windows/Fonts/simhei.ttf")
    elseif(WIN32 AND EXISTS "C:/Windows/Fonts/msyh.ttc")
        set(_viewer_cjk_font "C:/Windows/Fonts/msyh.ttc")
    endif()
    if(NOT _viewer_cjk_font STREQUAL "")
        add_custom_command(
            TARGET volume_surface_viewer
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:volume_surface_viewer>/assets/fonts"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_viewer_cjk_font}"
                "$<TARGET_FILE_DIR:volume_surface_viewer>/assets/fonts/Roboto-Medium.ttf"
            COMMENT "Installing a CJK-capable ImGui font"
            VERBATIM
        )
    endif()

    set(_viewer_state_header
        "${CMAKE_CURRENT_SOURCE_DIR}/include/volume_surface/viewer/ViewerState.h")
    set(_document_session_header
        "${CMAKE_CURRENT_SOURCE_DIR}/include/volume_surface/viewer/DocumentSession.h")
    set_source_files_properties(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/viewer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/BrushInteractionController.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/BrushPaintingPanel.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/ReconstructionPanel.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/WorkflowPanel.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/SurfaceTargetCacheRepository.cpp"
        PROPERTIES OBJECT_DEPENDS
            "${_surface_target_header};${_surface_normal_field_header};${_surface_mesh_continuity_header};${_viewer_state_header};${_document_session_header}")

    set_source_files_properties(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/BrushCursorRenderer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/BrushHeatmapRenderer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/SurfaceFitExpansionDebugRenderer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/SurfaceFitNeighborhoodDebugRenderer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/SurfaceFitPlaneRenderer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/SurfaceTargetPointPicker.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/SurfaceTargetPreview.cpp"
        PROPERTIES OBJECT_DEPENDS "${RESGEN_HEADER}")

    volume_surface_copy_openvdb_runtime(volume_surface_viewer)
endif()

if(VOLUME_SURFACE_BUILD_TESTS)
    enable_testing()

    add_executable(brick_arena_tests
        tests/BrickArenaTests.cpp
    )

    target_link_libraries(brick_arena_tests
        PRIVATE
            volume_surface_bricks
    )

    volume_surface_copy_openvdb_runtime(brick_arena_tests)
    add_test(NAME brick_arena_tests COMMAND brick_arena_tests)

    add_executable(surface_mesh_tests
        tests/SurfaceMeshTests.cpp
    )

    target_link_libraries(surface_mesh_tests
        PRIVATE
            volume_surface_mesh
    )

    volume_surface_copy_openvdb_runtime(surface_mesh_tests)
    add_test(NAME surface_mesh_tests COMMAND surface_mesh_tests)

    add_executable(surface_mesh_continuity_tests
        tests/SurfaceMeshContinuityTests.cpp
    )

    target_link_libraries(surface_mesh_continuity_tests
        PRIVATE
            volume_surface_mesh
    )

    volume_surface_copy_openvdb_runtime(surface_mesh_continuity_tests)
    add_test(NAME surface_mesh_continuity_tests COMMAND surface_mesh_continuity_tests)

    add_executable(surface_brush_tests
        tests/SurfaceBrushTests.cpp
    )

    target_link_libraries(surface_brush_tests
        PRIVATE
            volume_surface_mesh
    )

    volume_surface_copy_openvdb_runtime(surface_brush_tests)
    add_test(NAME surface_brush_tests COMMAND surface_brush_tests)

    add_executable(slice_diagnostics_tests
        tests/SliceDiagnosticsTests.cpp
    )

    target_link_libraries(slice_diagnostics_tests
        PRIVATE
            volume_surface_mesh
    )

    volume_surface_copy_openvdb_runtime(slice_diagnostics_tests)
    add_test(NAME slice_diagnostics_tests COMMAND slice_diagnostics_tests)

    add_executable(surface_reconstruction_tests
        tests/SurfaceReconstructionTests.cpp
    )

    target_link_libraries(surface_reconstruction_tests
        PRIVATE
            volume_surface_mesh
    )

    volume_surface_copy_openvdb_runtime(surface_reconstruction_tests)
    add_test(NAME surface_reconstruction_tests COMMAND surface_reconstruction_tests)

    add_executable(surface_target_tests
        tests/SurfaceTargetTests.cpp
    )

    target_link_libraries(surface_target_tests
        PRIVATE
            volume_surface_mesh
    )

    volume_surface_copy_openvdb_runtime(surface_target_tests)
    add_test(NAME surface_target_tests COMMAND surface_target_tests)
endif()
