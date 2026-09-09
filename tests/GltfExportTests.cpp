#include "volume_surface/GltfExport.h"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main()
{
    volume_surface::SurfaceMesh mesh;
    mesh.vertices.resize(3);
    mesh.vertices[0].position = {0.0f, 0.0f, 0.0f};
    mesh.vertices[1].position = {1.0f, 0.0f, 0.0f};
    mesh.vertices[2].position = {0.0f, 1.0f, 0.0f};
    for (auto& vertex : mesh.vertices) {
        vertex.normal = {0.0f, 0.0f, 1.0f};
    }
    mesh.indices = {0, 1, 2};

    const auto outputPath =
        std::filesystem::temp_directory_path() / "volume_surface_glb_export_test.glb";
    auto readExport = [&](bool includeNormals, std::size_t& byteCount) {
        const auto result = volume_surface::writeSurfaceMeshGlb(
            mesh,
            outputPath,
            includeNormals);
        if (!result.success || result.bytesWritten == 0) {
            std::cerr << "GLB export failed: " << result.error << '\n';
            return std::vector<std::uint8_t>{};
        }
        std::ifstream input(outputPath, std::ios::binary);
        if (!input) {
            std::cerr << "Could not reopen the GLB test file\n";
            return std::vector<std::uint8_t>{};
        }
        const std::istreambuf_iterator<char> begin(input);
        const std::istreambuf_iterator<char> end;
        std::vector<std::uint8_t> bytes(begin, end);
        input.close();
        std::filesystem::remove(outputPath);
        byteCount = result.bytesWritten;
        return bytes;
    };

    std::size_t positionsOnlyBytes = 0;
    const auto positionsOnly = readExport(false, positionsOnlyBytes);
    std::size_t withNormalsBytes = 0;
    const auto withNormals = readExport(true, withNormalsBytes);
    if (positionsOnly.size() != positionsOnlyBytes ||
        withNormals.size() != withNormalsBytes ||
        positionsOnly.size() < 20 ||
        withNormals.size() <= positionsOnly.size()) {
        std::cerr << "GLB byte count or optional normal payload is inconsistent\n";
        return EXIT_FAILURE;
    }

    const auto checkHeader = [](const std::vector<std::uint8_t>& bytes) {
        return bytes.size() >= 4 &&
            static_cast<std::uint32_t>(bytes[0]) == 0x67U &&
            static_cast<std::uint32_t>(bytes[1]) == 0x6CU &&
            static_cast<std::uint32_t>(bytes[2]) == 0x54U &&
            static_cast<std::uint32_t>(bytes[3]) == 0x46U;
    };
    if (!checkHeader(positionsOnly) || !checkHeader(withNormals)) {
        std::cerr << "GLB magic is invalid\n";
        return EXIT_FAILURE;
    }
    const std::string positionsOnlyText(
        reinterpret_cast<const char*>(positionsOnly.data()),
        positionsOnly.size());
    const std::string withNormalsText(
        reinterpret_cast<const char*>(withNormals.data()),
        withNormals.size());
    if (positionsOnlyText.find("\"POSITION\":0") == std::string::npos ||
        positionsOnlyText.find("\"NORMAL\":1") != std::string::npos ||
        positionsOnlyText.find("\"componentType\":5125") == std::string::npos ||
        withNormalsText.find("\"POSITION\":0") == std::string::npos ||
        withNormalsText.find("\"NORMAL\":1") == std::string::npos ||
        withNormalsText.find("\"componentType\":5125") == std::string::npos) {
        std::cerr << "GLB JSON does not describe the expected optional attributes\n";
        return EXIT_FAILURE;
    }

    std::cout << "glb_positions_only_bytes=" << positionsOnly.size()
              << " glb_with_normals_bytes=" << withNormals.size() << '\n';
    return EXIT_SUCCESS;
}
