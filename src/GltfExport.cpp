#include "volume_surface/GltfExport.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace volume_surface {
namespace {

constexpr std::uint32_t kGlbMagic = 0x46546C67U;
constexpr std::uint32_t kGlbVersion = 2U;
constexpr std::uint32_t kJsonChunkType = 0x4E4F534AU;
constexpr std::uint32_t kBinaryChunkType = 0x004E4942U;
constexpr std::uint64_t kMaxGlbLength =
    static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max());

std::size_t paddedSize(std::size_t size)
{
    constexpr std::size_t alignment = 4;
    const std::size_t remainder = size % alignment;
    return remainder == 0 ? size : size + (alignment - remainder);
}

std::size_t checkedMultiply(std::size_t left, std::size_t right)
{
    if (right != 0 && left > std::numeric_limits<std::size_t>::max() / right) {
        throw std::overflow_error("GLB buffer size overflow");
    }
    return left * right;
}

void writeU32(std::ofstream& output, std::uint32_t value)
{
    const std::array<char, 4> bytes{
        static_cast<char>(value & 0xFFU),
        static_cast<char>((value >> 8U) & 0xFFU),
        static_cast<char>((value >> 16U) & 0xFFU),
        static_cast<char>((value >> 24U) & 0xFFU)};
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

void writeFloat(std::ofstream& output, float value)
{
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeU32(output, bits);
}

void writePadding(std::ofstream& output, std::size_t size, char value)
{
    constexpr std::size_t chunkSize = 4096;
    const std::array<char, chunkSize> buffer = [&]() {
        std::array<char, chunkSize> result{};
        result.fill(value);
        return result;
    }();
    while (size > 0) {
        const std::size_t chunk = std::min(size, chunkSize);
        output.write(buffer.data(), static_cast<std::streamsize>(chunk));
        size -= chunk;
    }
}

std::string jsonNumber(double value)
{
    std::ostringstream stream;
    stream.precision(9);
    stream << value;
    return stream.str();
}

std::string makeJson(
    const SurfaceMesh& mesh,
    bool includeNormals,
    std::size_t positionBytes,
    std::size_t normalBytes,
    std::size_t indexBytes)
{
    const auto& first = mesh.vertices.front().position;
    std::array<float, 3> minimum = first;
    std::array<float, 3> maximum = first;
    for (const auto& vertex : mesh.vertices) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], vertex.position[axis]);
            maximum[axis] = std::max(maximum[axis], vertex.position[axis]);
        }
    }

    const std::size_t normalOffset = positionBytes;
    const std::size_t indexOffset = positionBytes + normalBytes;
    const std::size_t indexBufferView = includeNormals ? 2 : 1;
    const std::size_t indexAccessor = includeNormals ? 2 : 1;
    std::ostringstream json;
    json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"VolumeSurface\"},"
            "\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
            "\"nodes\":[{\"mesh\":0,\"name\":\"ReconstructedSurface\"}],"
            "\"meshes\":[{\"name\":\"ReconstructedSurface\",\"primitives\":[{"
            "\"attributes\":{\"POSITION\":0";
    if (includeNormals) {
        json << ",\"NORMAL\":1";
    }
    json << "},\"indices\":" << indexAccessor
         << ",\"mode\":4}]}],\"buffers\":[{\"byteLength\":"
        << (positionBytes + normalBytes + indexBytes) << "}],\"bufferViews\":["
        << "{\"buffer\":0,\"byteOffset\":0,\"byteLength\":"
        << positionBytes << "}";
    if (includeNormals) {
        json << ","
             << "{\"buffer\":0,\"byteOffset\":" << normalOffset
             << ",\"byteLength\":" << normalBytes << "}";
    }
    json << ","
        << "{\"buffer\":0,\"byteOffset\":" << indexOffset
        << ",\"byteLength\":" << indexBytes << "}],\"accessors\":["
        << "{\"bufferView\":0,\"componentType\":5126,\"count\":"
        << mesh.vertices.size() << ",\"type\":\"VEC3\",\"min\":["
        << jsonNumber(minimum[0]) << "," << jsonNumber(minimum[1]) << ","
        << jsonNumber(minimum[2]) << "],\"max\":["
        << jsonNumber(maximum[0]) << "," << jsonNumber(maximum[1]) << ","
        << jsonNumber(maximum[2]) << "]}";
    if (includeNormals) {
        json << ","
             << "{\"bufferView\":1,\"componentType\":5126,\"count\":"
             << mesh.vertices.size() << ",\"type\":\"VEC3\"}";
    }
    json << ","
        << "{\"bufferView\":" << indexBufferView
        << ",\"componentType\":5125,\"count\":"
        << mesh.indices.size() << ",\"type\":\"SCALAR\"}]}";
    return json.str();
}

void validateMesh(const SurfaceMesh& mesh, bool includeNormals)
{
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::invalid_argument("The reconstructed mesh is empty");
    }
    if (mesh.indices.size() % 3 != 0) {
        throw std::invalid_argument("The reconstructed mesh index count is not divisible by three");
    }
    for (const auto& vertex : mesh.vertices) {
        for (const float value : vertex.position) {
            if (!std::isfinite(value)) {
                throw std::invalid_argument("The reconstructed mesh contains a non-finite position");
            }
        }
        if (includeNormals) {
            for (const float value : vertex.normal) {
                if (!std::isfinite(value)) {
                    throw std::invalid_argument("The reconstructed mesh contains a non-finite normal");
                }
            }
        }
    }
    for (const std::uint32_t index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            throw std::invalid_argument("The reconstructed mesh contains an out-of-range index");
        }
    }
}

} // namespace

GltfExportResult writeSurfaceMeshGlb(
    const SurfaceMesh& mesh,
    const std::filesystem::path& outputPath,
    bool includeNormals)
{
    GltfExportResult result;
    try {
        validateMesh(mesh, includeNormals);
        if (outputPath.empty()) {
            throw std::invalid_argument("The GLB output path is empty");
        }

        const std::size_t positionBytes = checkedMultiply(mesh.vertices.size(), 3 * sizeof(float));
        const std::size_t normalBytes = includeNormals ? positionBytes : 0;
        const std::size_t indexBytes = checkedMultiply(mesh.indices.size(), sizeof(std::uint32_t));
        const std::string json = makeJson(
            mesh,
            includeNormals,
            positionBytes,
            normalBytes,
            indexBytes);
        const std::size_t jsonBytes = paddedSize(json.size());
        const std::size_t binaryBytes = positionBytes + normalBytes + indexBytes;
        const std::uint64_t totalBytes = 12ULL + 8ULL + jsonBytes + 8ULL + binaryBytes;
        if (totalBytes > kMaxGlbLength || jsonBytes > kMaxGlbLength || binaryBytes > kMaxGlbLength) {
            throw std::overflow_error("The GLB exceeds the 4 GiB format limit");
        }

        if (!outputPath.parent_path().empty()) {
            std::error_code directoryError;
            if (!std::filesystem::is_directory(outputPath.parent_path(), directoryError)) {
                throw std::runtime_error("The selected export folder does not exist");
            }
        }
        std::ofstream output(outputPath, std::ios::binary | std::ios::trunc);
        if (!output) {
            throw std::runtime_error("Could not open the GLB output file");
        }

        writeU32(output, kGlbMagic);
        writeU32(output, kGlbVersion);
        writeU32(output, static_cast<std::uint32_t>(totalBytes));
        writeU32(output, static_cast<std::uint32_t>(jsonBytes));
        writeU32(output, kJsonChunkType);
        output.write(json.data(), static_cast<std::streamsize>(json.size()));
        writePadding(output, jsonBytes - json.size(), ' ');
        writeU32(output, static_cast<std::uint32_t>(binaryBytes));
        writeU32(output, kBinaryChunkType);

        for (const auto& vertex : mesh.vertices) {
            for (const float value : vertex.position) {
                writeFloat(output, value);
            }
        }
        if (includeNormals) {
            for (const auto& vertex : mesh.vertices) {
                for (const float value : vertex.normal) {
                    writeFloat(output, value);
                }
            }
        }
        for (const std::uint32_t index : mesh.indices) {
            writeU32(output, index);
        }
        writePadding(output, paddedSize(binaryBytes) - binaryBytes, '\0');
        output.flush();
        if (!output) {
            throw std::runtime_error("The GLB output write failed");
        }

        result.success = true;
        result.bytesWritten = static_cast<std::size_t>(totalBytes);
    } catch (const std::exception& error) {
        result.error = error.what();
    }
    return result;
}

} // namespace volume_surface
