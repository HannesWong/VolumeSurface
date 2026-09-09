#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

#include "volume_surface/SurfaceMesh.h"

namespace volume_surface {

struct GltfExportResult {
    bool success = false;
    std::size_t bytesWritten = 0;
    std::string error;
};

// Writes a minimal glTF 2.0 binary containing POSITION, optional NORMAL, and triangle indices.
GltfExportResult writeSurfaceMeshGlb(
    const SurfaceMesh& mesh,
    const std::filesystem::path& outputPath,
    bool includeNormals = false);

} // namespace volume_surface
