#include "volume_surface/viewer/BrushHeatmapRenderer.h"

#include "volume_surface_viewer_resources.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/VertexBuffer.h>
#include <utils/EntityManager.h>

namespace volume_surface::viewer {

namespace {

using filament::Engine;
using filament::IndexBuffer;
using filament::Material;
using filament::MaterialInstance;
using filament::RenderableManager;
using filament::Scene;
using filament::VertexBuffer;
using filament::math::float3;
using utils::Entity;
using utils::EntityManager;

struct VertexVoxelRecord {
    openvdb::Coord coordinate;
    std::uint32_t vertexIndex = 0;
};

void releaseUpload(void* buffer, std::size_t, void*)
{
    std::free(buffer);
}

template <typename T>
filament::backend::BufferDescriptor copyForUpload(const std::vector<T>& source)
{
    const std::size_t byteCount = source.size() * sizeof(T);
    void* copy = std::malloc(byteCount);
    if (!copy && byteCount != 0) {
        throw std::bad_alloc();
    }
    if (byteCount != 0) {
        std::memcpy(copy, source.data(), byteCount);
    }
    return filament::backend::BufferDescriptor(copy, byteCount, releaseUpload);
}

openvdb::Coord nearestVoxelCoordinate(
    const openvdb::FloatGrid& grid,
    const std::array<float, 3>& worldPosition)
{
    const openvdb::Vec3d indexPosition = grid.worldToIndex(openvdb::Vec3d{
        worldPosition[0], worldPosition[1], worldPosition[2]});
    return openvdb::Coord(
        static_cast<int>(std::lround(indexPosition.x())),
        static_cast<int>(std::lround(indexPosition.y())),
        static_cast<int>(std::lround(indexPosition.z())));
}

std::array<std::uint8_t, 4> heatmapColor(float normalizedWeight)
{
    const float t = std::clamp(normalizedWeight, 0.0f, 1.0f);
    if (t <= 0.0f) {
        return {0, 0, 0, 0};
    }

    const float red = std::clamp(2.0f * t, 0.0f, 1.0f);
    const float green = std::clamp(2.0f - std::abs(4.0f * t - 2.0f), 0.0f, 1.0f);
    const float blue = std::clamp(2.0f * (1.0f - t), 0.0f, 1.0f);
    const float alpha = 0.55f + 0.45f * t;
    return {
        static_cast<std::uint8_t>(std::lround(red * 255.0f)),
        static_cast<std::uint8_t>(std::lround(green * 255.0f)),
        static_cast<std::uint8_t>(std::lround(blue * 255.0f)),
        static_cast<std::uint8_t>(std::lround(alpha * 255.0f))};
}

} // namespace

struct BrushHeatmapRenderer::Resources {
    Material* material = nullptr;
    MaterialInstance* materialInstance = nullptr;
    VertexBuffer* vertexBuffer = nullptr;
    IndexBuffer* indexBuffer = nullptr;
    Entity entity;
    bool addedToScene = false;
    std::vector<VertexVoxelRecord> vertexLookup;
    std::vector<float> vertexWeights;
    std::vector<std::array<std::uint8_t, 4>> vertexColors;
    const SurfaceMesh* mesh = nullptr;
    float3 boundsCenter{};
    float3 boundsExtent{};
    Stats stats;
};

BrushHeatmapRenderer::BrushHeatmapRenderer() = default;

BrushHeatmapRenderer::~BrushHeatmapRenderer() = default;

void BrushHeatmapRenderer::clearGeometry(Engine& engine, Scene& scene)
{
    if (!mResources) {
        return;
    }
    Resources& heatmap = *mResources;
    if (heatmap.entity) {
        if (heatmap.addedToScene) {
            scene.remove(heatmap.entity);
            heatmap.addedToScene = false;
        }
        engine.destroy(heatmap.entity);
        EntityManager::get().destroy(heatmap.entity);
        heatmap.entity.clear();
    }
    if (heatmap.indexBuffer) {
        engine.destroy(heatmap.indexBuffer);
        heatmap.indexBuffer = nullptr;
    }
    heatmap.stats = {};
}

void BrushHeatmapRenderer::destroy(Engine& engine, Scene& scene)
{
    if (!mResources) {
        return;
    }
    Resources& heatmap = *mResources;
    clearGeometry(engine, scene);
    if (heatmap.materialInstance) {
        engine.destroy(heatmap.materialInstance);
        heatmap.materialInstance = nullptr;
    }
    if (heatmap.material) {
        engine.destroy(heatmap.material);
        heatmap.material = nullptr;
    }
    if (heatmap.vertexBuffer) {
        engine.destroy(heatmap.vertexBuffer);
        heatmap.vertexBuffer = nullptr;
    }
    heatmap.vertexLookup.clear();
    heatmap.vertexWeights.clear();
    heatmap.vertexColors.clear();
    heatmap.mesh = nullptr;
    heatmap.stats = {};
}

void BrushHeatmapRenderer::clear(Engine& engine, Scene& scene)
{
    if (!mResources) {
        return;
    }
    Resources& heatmap = *mResources;
    std::fill(heatmap.vertexWeights.begin(), heatmap.vertexWeights.end(), 0.0f);
    std::fill(
        heatmap.vertexColors.begin(),
        heatmap.vertexColors.end(),
        std::array<std::uint8_t, 4>{0, 0, 0, 0});
    if (heatmap.vertexBuffer && !heatmap.vertexColors.empty()) {
        heatmap.vertexBuffer->setBufferAt(
            engine,
            1,
            copyForUpload(heatmap.vertexColors));
    }
    clearGeometry(engine, scene);
}

void BrushHeatmapRenderer::create(
    Engine& engine,
    Scene& scene,
    const SurfaceMesh& mesh,
    const openvdb::FloatGrid& grid,
    const float3& referenceCenter,
    float displayScale)
{
    if (!mResources) {
        mResources = std::make_unique<Resources>();
    }
    destroy(engine, scene);
    if (mesh.empty()) {
        return;
    }

    Resources& heatmap = *mResources;
    heatmap.vertexLookup.resize(mesh.vertices.size());
    heatmap.vertexWeights.resize(mesh.vertices.size(), 0.0f);
    heatmap.vertexColors.resize(mesh.vertices.size(), {0, 0, 0, 0});
    heatmap.mesh = &mesh;

    const openvdb::Vec3d voxelSize = grid.voxelSize();
    const float surfaceOffset = static_cast<float>(
        std::min({voxelSize.x(), voxelSize.y(), voxelSize.z()}) * 0.15);
    std::vector<float3> positions(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const auto& source = mesh.vertices[i];
        const float3 worldPosition{
            source.position[0] + source.normal[0] * surfaceOffset,
            source.position[1] + source.normal[1] * surfaceOffset,
            source.position[2] + source.normal[2] * surfaceOffset};
        positions[i] = (worldPosition - referenceCenter) * displayScale
            + float3{0.0f, 0.0f, -4.0f};
        heatmap.vertexLookup[i] = {
            nearestVoxelCoordinate(grid, source.position),
            static_cast<std::uint32_t>(i)};
    }
    std::sort(
        heatmap.vertexLookup.begin(),
        heatmap.vertexLookup.end(),
        [](const VertexVoxelRecord& lhs, const VertexVoxelRecord& rhs) {
            if (lhs.coordinate != rhs.coordinate) {
                return lhs.coordinate < rhs.coordinate;
            }
            return lhs.vertexIndex < rhs.vertexIndex;
        });

    const float3 minimum =
        (float3{mesh.bounds.minimum[0], mesh.bounds.minimum[1], mesh.bounds.minimum[2]}
            - referenceCenter) * displayScale + float3{0.0f, 0.0f, -4.0f};
    const float3 maximum =
        (float3{mesh.bounds.maximum[0], mesh.bounds.maximum[1], mesh.bounds.maximum[2]}
            - referenceCenter) * displayScale + float3{0.0f, 0.0f, -4.0f};
    heatmap.boundsCenter = (minimum + maximum) * 0.5f;
    heatmap.boundsExtent = (maximum - minimum) * 0.5f;

    heatmap.vertexBuffer = VertexBuffer::Builder()
        .vertexCount(static_cast<std::uint32_t>(mesh.vertices.size()))
        .bufferCount(2)
        .attribute(
            filament::VertexAttribute::POSITION,
            0,
            VertexBuffer::AttributeType::FLOAT3)
        .attribute(
            filament::VertexAttribute::COLOR,
            1,
            VertexBuffer::AttributeType::UBYTE4)
        .normalized(filament::VertexAttribute::COLOR)
        .build(engine);
    if (!heatmap.vertexBuffer) {
        throw std::runtime_error("Failed to create brush heatmap vertex buffer");
    }
    heatmap.vertexBuffer->setBufferAt(engine, 0, copyForUpload(positions));
    heatmap.vertexBuffer->setBufferAt(
        engine,
        1,
        copyForUpload(heatmap.vertexColors));

    heatmap.material = Material::Builder()
        .package(
            VOLUME_SURFACE_VIEWER_RESOURCES_BRUSHHEATMAP_DATA,
            VOLUME_SURFACE_VIEWER_RESOURCES_BRUSHHEATMAP_SIZE)
        .build(engine);
    if (!heatmap.material) {
        throw std::runtime_error("Failed to create brush heatmap material");
    }
    heatmap.materialInstance = heatmap.material->createInstance();
}

bool BrushHeatmapRenderer::applySamples(
    Engine& engine,
    Scene& scene,
    const std::vector<SurfaceBrushSample>& samples)
{
    if (!mResources || !mResources->vertexBuffer ||
        !mResources->materialInstance || samples.empty()) {
        return false;
    }
    Resources& heatmap = *mResources;
    bool strokeApplied = false;
    for (const auto& sample : samples) {
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const openvdb::Coord lookupCoordinate =
                        sample.coordinate.offsetBy(dx, dy, dz);
                    auto found = std::lower_bound(
                        heatmap.vertexLookup.begin(),
                        heatmap.vertexLookup.end(),
                        lookupCoordinate,
                        [](const VertexVoxelRecord& record, const openvdb::Coord& coordinate) {
                            return record.coordinate < coordinate;
                        });
                    while (found != heatmap.vertexLookup.end() &&
                        found->coordinate == lookupCoordinate) {
                        heatmap.vertexWeights[found->vertexIndex] = std::max(
                            heatmap.vertexWeights[found->vertexIndex], sample.weight);
                        strokeApplied = true;
                        ++found;
                    }
                }
            }
        }
    }
    if (!strokeApplied) {
        return false;
    }

    clearGeometry(engine, scene);
    const float maximumWeight = *std::max_element(
        heatmap.vertexWeights.begin(), heatmap.vertexWeights.end());
    const float inverseMaximumWeight = maximumWeight > 1.0e-12f
        ? 1.0f / maximumWeight
        : 0.0f;
    heatmap.stats.affectedVertexCount = 0;
    for (std::size_t i = 0; i < heatmap.vertexWeights.size(); ++i) {
        heatmap.vertexColors[i] = heatmapColor(
            heatmap.vertexWeights[i] * inverseMaximumWeight);
        heatmap.stats.affectedVertexCount += heatmap.vertexWeights[i] > 0.0f ? 1 : 0;
    }
    heatmap.vertexBuffer->setBufferAt(
        engine,
        1,
        copyForUpload(heatmap.vertexColors));

    std::vector<std::uint32_t> visibleIndices;
    visibleIndices.reserve(heatmap.stats.affectedVertexCount * 6);
    if (!heatmap.mesh) {
        return false;
    }
    for (std::size_t i = 0; i < heatmap.mesh->indices.size(); i += 3) {
        const std::uint32_t a = heatmap.mesh->indices[i];
        const std::uint32_t b = heatmap.mesh->indices[i + 1];
        const std::uint32_t c = heatmap.mesh->indices[i + 2];
        if (heatmap.vertexWeights[a] > 0.0f ||
            heatmap.vertexWeights[b] > 0.0f ||
            heatmap.vertexWeights[c] > 0.0f) {
            visibleIndices.push_back(a);
            visibleIndices.push_back(b);
            visibleIndices.push_back(c);
        }
    }
    if (visibleIndices.empty()) {
        return false;
    }

    heatmap.stats.visibleTriangleCount = visibleIndices.size() / 3;
    heatmap.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(visibleIndices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    if (!heatmap.indexBuffer) {
        throw std::runtime_error("Failed to create brush heatmap index buffer");
    }
    heatmap.indexBuffer->setBuffer(engine, copyForUpload(visibleIndices));
    heatmap.entity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({heatmap.boundsCenter, heatmap.boundsExtent})
        .material(0, heatmap.materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::TRIANGLES,
            heatmap.vertexBuffer,
            heatmap.indexBuffer)
        .priority(7)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, heatmap.entity);
    scene.addEntity(heatmap.entity);
    heatmap.addedToScene = true;
    return true;
}

void BrushHeatmapRenderer::setVisible(Scene& scene, bool visible)
{
    if (!mResources || !mResources->entity) {
        return;
    }
    Resources& heatmap = *mResources;
    if (visible && !heatmap.addedToScene) {
        scene.addEntity(heatmap.entity);
        heatmap.addedToScene = true;
    } else if (!visible && heatmap.addedToScene) {
        scene.remove(heatmap.entity);
        heatmap.addedToScene = false;
    }
}

BrushHeatmapRenderer::Stats BrushHeatmapRenderer::stats() const noexcept
{
    return mResources ? mResources->stats : Stats{};
}

utils::Entity BrushHeatmapRenderer::entity() const noexcept
{
    return mResources ? mResources->entity : utils::Entity{};
}

bool BrushHeatmapRenderer::addedToScene() const noexcept
{
    return mResources && mResources->addedToScene;
}

bool BrushHeatmapRenderer::available() const noexcept
{
    return mResources && mResources->vertexBuffer && mResources->materialInstance;
}

} // namespace volume_surface::viewer
