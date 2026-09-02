set(VOLUME_SURFACE_OPENVDB_SOURCE_ROOT
    "G:/downLoadProj/openvdb-master"
    CACHE PATH
    "OpenVDB source repository root"
)

set(VOLUME_SURFACE_OPENVDB_BUILD_ROOT
    "G:/downLoadProj/openvdb-master/build-vdb-from-slices"
    CACHE PATH
    "OpenVDB build tree root"
)

set(VOLUME_SURFACE_VCPKG_X64_ROOT
    "G:/downLoadProj/vcpkg-master/installed/x64-windows"
    CACHE PATH
    "vcpkg x64-windows installed root used by the OpenVDB build"
)

set(_openvdb_include_root "${VOLUME_SURFACE_OPENVDB_SOURCE_ROOT}/openvdb")
set(_openvdb_generated_root "${VOLUME_SURFACE_OPENVDB_BUILD_ROOT}/openvdb/openvdb")
set(_openvdb_release_root "${_openvdb_generated_root}/Release")
set(_vcpkg_include_root "${VOLUME_SURFACE_VCPKG_X64_ROOT}/include")
set(_vcpkg_library_root "${VOLUME_SURFACE_VCPKG_X64_ROOT}/lib")
set(_vcpkg_binary_root "${VOLUME_SURFACE_VCPKG_X64_ROOT}/bin")

set(_openvdb_header "${_openvdb_include_root}/openvdb/openvdb.h")
set(_openvdb_library "${_openvdb_release_root}/openvdb.lib")
set(_openvdb_runtime "${_openvdb_release_root}/openvdb.dll")

foreach(_required_path IN ITEMS
    "${_openvdb_header}"
    "${_openvdb_library}"
    "${_openvdb_runtime}"
    "${_vcpkg_include_root}"
)
    if(NOT EXISTS "${_required_path}")
        message(FATAL_ERROR "Required OpenVDB path does not exist: ${_required_path}")
    endif()
endforeach()

file(GLOB _boost_iostreams_libraries
    "${_vcpkg_library_root}/boost_iostreams-*.lib"
)
file(GLOB _boost_random_libraries
    "${_vcpkg_library_root}/boost_random-*.lib"
)

if(NOT _boost_iostreams_libraries OR NOT _boost_random_libraries)
    message(FATAL_ERROR "The OpenVDB Boost dependency libraries were not found")
endif()

list(GET _boost_iostreams_libraries 0 _boost_iostreams_library)
list(GET _boost_random_libraries 0 _boost_random_library)

add_library(OpenVDB::openvdb SHARED IMPORTED GLOBAL)

set_target_properties(OpenVDB::openvdb PROPERTIES
    IMPORTED_IMPLIB "${_openvdb_library}"
    IMPORTED_LOCATION "${_openvdb_runtime}"
    INTERFACE_INCLUDE_DIRECTORIES
        "${_openvdb_include_root};${_openvdb_generated_root};${_openvdb_generated_root}/openvdb;${VOLUME_SURFACE_OPENVDB_SOURCE_ROOT}/ext/imath;${_vcpkg_include_root}"
    INTERFACE_COMPILE_DEFINITIONS
        "OPENVDB_USE_DELAYED_LOADING;OPENVDB_DLL;BOOST_IOSTREAMS_NO_LIB;BOOST_IOSTREAMS_DYN_LINK;BOOST_RANDOM_NO_LIB;BOOST_RANDOM_DYN_LINK;BOOST_ALL_NO_LIB;WIN32_LEAN_AND_MEAN;_CRT_SECURE_NO_WARNINGS;_CRT_NONSTDC_NO_WARNINGS"
    INTERFACE_LINK_LIBRARIES
        "${_vcpkg_library_root}/tbbmalloc.lib;${_vcpkg_library_root}/tbb12.lib;${_vcpkg_library_root}/blosc.lib;${_vcpkg_library_root}/zlib.lib;${_boost_iostreams_library};${_boost_random_library}"
)

set(_volume_surface_runtime_files
    "${_openvdb_release_root}/openvdb.dll"
    "${_openvdb_release_root}/blosc.dll"
    "${_openvdb_release_root}/lz4.dll"
    "${_openvdb_release_root}/tbb12.dll"
    "${_openvdb_release_root}/zlib1.dll"
    "${_openvdb_release_root}/zstd.dll"
    "${_vcpkg_binary_root}/tbbmalloc.dll"
)

file(GLOB _boost_runtime_files
    "${_vcpkg_binary_root}/boost_iostreams-*.dll"
    "${_vcpkg_binary_root}/boost_random-*.dll"
)
list(APPEND _volume_surface_runtime_files ${_boost_runtime_files})

function(volume_surface_copy_openvdb_runtime target_name)
    foreach(_runtime_file IN LISTS _volume_surface_runtime_files)
        if(EXISTS "${_runtime_file}")
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "${_runtime_file}"
                    "$<TARGET_FILE_DIR:${target_name}>"
                VERBATIM
            )
        endif()
    endforeach()
endfunction()
