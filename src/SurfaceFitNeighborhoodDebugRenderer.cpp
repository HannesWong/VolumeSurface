#include "volume_surface/viewer/SurfaceFitNeighborhoodDebugRenderer.h"
#include "volume_surface_viewer_resources.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
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
#include <geometry/SurfaceOrientation.h>
#include <math/vec3.h>
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
using filament::math::short4;
using utils::Entity;
using utils::EntityManager;

struct PointVertex
{
    float3 position{};
    short4 tangent{};
    std::array<std::uint8_t, 4> color{};
    float custom0[4]{};
};

struct PointRecord
{
    float3 position{};
    float3 normal{0.0f, 0.0f, 1.0f};
    std::array<std::uint8_t, 4> color{};
};

struct PrimitiveResources
{
    MaterialInstance* materialInstance = nullptr;
    VertexBuffer* vertexBuffer = nullptr;
    IndexBuffer* indexBuffer = nullptr;
    Entity entity;
    bool addedToScene = false;
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

void destroyPrimitive(
    Engine& engine,
    Scene& scene,
    PrimitiveResources& primitive)
{
    if (primitive.entity) {
        if (primitive.addedToScene) {
            scene.remove(primitive.entity);
            primitive.addedToScene = false;
        }
        engine.destroy(primitive.entity);
        EntityManager::get().destroy(primitive.entity);
        primitive.entity.clear();
    }
    if (primitive.materialInstance) {
        engine.destroy(primitive.materialInstance);
        primitive.materialInstance = nullptr;
    }
    if (primitive.vertexBuffer) {
        engine.destroy(primitive.vertexBuffer);
        primitive.vertexBuffer = nullptr;
    }
    if (primitive.indexBuffer) {
        engine.destroy(primitive.indexBuffer);
        primitive.indexBuffer = nullptr;
    }
}

void setPrimitiveVisible(
    Scene& scene,
    PrimitiveResources& primitive,
    bool visible)
{
    if (!primitive.entity) {
        return;
    }
    if (visible && !primitive.addedToScene) {
        scene.addEntity(primitive.entity);
        primitive.addedToScene = true;
    } else if (!visible && primitive.addedToScene) {
        scene.remove(primitive.entity);
        primitive.addedToScene = false;
    }
}

float3 toScene(
    const openvdb::Vec3d& worldPosition,
    const openvdb::Vec3d& referenceCenter,
    float displayScale)
{
    return (float3{
                static_cast<float>(worldPosition.x()),
                static_cast<float>(worldPosition.y()),
                static_cast<float>(worldPosition.z())} -
            float3{
                static_cast<float>(referenceCenter.x()),
                static_cast<float>(referenceCenter.y()),
                static_cast<float>(referenceCenter.z())}) *
            displayScale +
        float3{0.0f, 0.0f, -4.0f};
}

void createPointPrimitive(
    Engine& engine,
    Material* material,
    PrimitiveResources& primitive,
    const std::vector<PointRecord>& records)
{
    if (records.empty()) {
        return;
    }

    std::vector<PointVertex> vertices(records.size());
    std::vector<float3> normals;
    normals.reserve(records.size());
    for (std::size_t index = 0; index < records.size(); ++index) {
        vertices[index].position = records[index].position;
        vertices[index].color = records[index].color;
        vertices[index].custom0[0] = 0.0f;
        vertices[index].custom0[1] = 0.0f;
        vertices[index].custom0[2] = 0.0f;
        vertices[index].custom0[3] = 0.0f;
        normals.push_back(records[index].normal);
    }
    std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
        filament::geometry::SurfaceOrientation::Builder()
            .vertexCount(vertices.size())
            .normals(normals.data())
            .build());
    if (!orientation) {
        throw std::runtime_error("Failed to build Surface Fit neighborhood point frames");
    }
    orientation->getQuats(&vertices.front().tangent, vertices.size(), sizeof(PointVertex));

    float3 minimum = vertices.front().position;
    float3 maximum = minimum;
    for (const auto& vertex : vertices) {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], vertex.position[axis]);
            maximum[axis] = std::max(maximum[axis], vertex.position[axis]);
        }
    }
    const float3 extent = maximum - minimum;
    const float3 center = (minimum + maximum) * 0.5f;
    const float3 halfExtent{
        std::max(extent.x * 0.5f, 1.0e-4f),
        std::max(extent.y * 0.5f, 1.0e-4f),
        std::max(extent.z * 0.5f, 1.0e-4f)};

    primitive.vertexBuffer = VertexBuffer::Builder()
        .vertexCount(static_cast<std::uint32_t>(vertices.size()))
        .bufferCount(1)
        .attribute(
            filament::VertexAttribute::POSITION,
            0,
            VertexBuffer::AttributeType::FLOAT3,
            offsetof(PointVertex, position),
            sizeof(PointVertex))
        .attribute(
            filament::VertexAttribute::TANGENTS,
            0,
            VertexBuffer::AttributeType::SHORT4,
            offsetof(PointVertex, tangent),
            sizeof(PointVertex))
        .attribute(
            filament::VertexAttribute::COLOR,
            0,
            VertexBuffer::AttributeType::UBYTE4,
            offsetof(PointVertex, color),
            sizeof(PointVertex))
        .attribute(
            filament::VertexAttribute::CUSTOM0,
            0,
            VertexBuffer::AttributeType::FLOAT4,
            offsetof(PointVertex, custom0),
            sizeof(PointVertex))
        .normalized(filament::VertexAttribute::TANGENTS)
        .normalized(filament::VertexAttribute::COLOR)
        .build(engine);
    if (!primitive.vertexBuffer) {
        throw std::runtime_error("Failed to create Surface Fit neighborhood point vertices");
    }
    primitive.vertexBuffer->setBufferAt(engine, 0, copyForUpload(vertices));

    std::vector<std::uint32_t> indices(vertices.size());
    for (std::size_t index = 0; index < indices.size(); ++index) {
        indices[index] = static_cast<std::uint32_t>(index);
    }
    primitive.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    if (!primitive.indexBuffer) {
        throw std::runtime_error("Failed to create Surface Fit neighborhood point indices");
    }
    primitive.indexBuffer->setBuffer(engine, copyForUpload(indices));

    primitive.materialInstance = material->createInstance();
    if (!primitive.materialInstance) {
        throw std::runtime_error("Failed to create Surface Fit neighborhood point material");
    }
    primitive.entity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({center, halfExtent})
        .material(0, primitive.materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::POINTS,
            primitive.vertexBuffer,
            primitive.indexBuffer)
        .priority(10)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, primitive.entity);
}

std::array<std::uint8_t, 4> lerpColor(
    const std::array<std::uint8_t, 4>& first,
    const std::array<std::uint8_t, 4>& second,
    double amount)
{
    const double t = std::clamp(amount, 0.0, 1.0);
    std::array<std::uint8_t, 4> result{};
    for (int index = 0; index < 4; ++index) {
        result[index] = static_cast<std::uint8_t>(std::lround(
            static_cast<double>(first[index]) * (1.0 - t) +
            static_cast<double>(second[index]) * t));
    }
    return result;
}

} // namespace

struct SurfaceFitNeighborhoodDebugRenderer::Impl
{
    Settings settings;
    Statistics statistics;
    PrimitiveResources points;
    PrimitiveResources center;
    Material* material = nullptr;
    float pointRadiusScene = 0.0006f;
    bool hasSelection = false;

    void ensureMaterial(Engine& engine)
    {
        if (material) {
            return;
        }
        material = Material::Builder()
            .package(
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACEFITNEIGHBORHOODDEBUGPOINTS_DATA,
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACEFITNEIGHBORHOODDEBUGPOINTS_SIZE)
            .build(engine);
        if (!material) {
            throw std::runtime_error(
                "Failed to create Surface Fit neighborhood debug material");
        }
    }

    void destroyPrimitives(Engine& engine, Scene& scene)
    {
        destroyPrimitive(engine, scene, points);
        destroyPrimitive(engine, scene, center);
    }

    void updateMaterialSettings()
    {
        for (auto* instance : {points.materialInstance, center.materialInstance}) {
            if (instance) {
                instance->setParameter("POINT_RADIUS", pointRadiusScene);
            }
        }
    }
};

SurfaceFitNeighborhoodDebugRenderer::SurfaceFitNeighborhoodDebugRenderer()
    : mImpl(std::make_unique<Impl>())
{
}

SurfaceFitNeighborhoodDebugRenderer::~SurfaceFitNeighborhoodDebugRenderer() = default;

SurfaceFitNeighborhoodDebugRenderer::SurfaceFitNeighborhoodDebugRenderer(
    SurfaceFitNeighborhoodDebugRenderer&&) noexcept = default;

SurfaceFitNeighborhoodDebugRenderer& SurfaceFitNeighborhoodDebugRenderer::operator=(
    SurfaceFitNeighborhoodDebugRenderer&&) noexcept = default;

SurfaceFitNeighborhoodDebugRenderer::Settings&
SurfaceFitNeighborhoodDebugRenderer::settings() noexcept
{
    return mImpl->settings;
}

const SurfaceFitNeighborhoodDebugRenderer::Settings&
SurfaceFitNeighborhoodDebugRenderer::settings() const noexcept
{
    return mImpl->settings;
}

const SurfaceFitNeighborhoodDebugRenderer::Statistics&
SurfaceFitNeighborhoodDebugRenderer::statistics() const noexcept
{
    return mImpl->statistics;
}

void SurfaceFitNeighborhoodDebugRenderer::rebuild(
    Engine& engine,
    Scene& scene,
    const Inputs& inputs)
{
    mImpl->destroyPrimitives(engine, scene);
    mImpl->statistics = {};
    mImpl->hasSelection = false;
    if (settings().displayMode == SurfaceFitNeighborhoodDisplayMode::None ||
        !inputs.cache || !inputs.inspection || !inputs.inspection->valid ||
        inputs.inspection->centerSampleIndex >= inputs.cache->samples.size()) {
        return;
    }
    mImpl->ensureMaterial(engine);

    const double minimumVoxelSize = std::max(
        std::min({
            std::abs(inputs.voxelSize.x()),
            std::abs(inputs.voxelSize.y()),
            std::abs(inputs.voxelSize.z())}),
        1.0e-9);
    mImpl->pointRadiusScene = std::max(
        static_cast<float>(0.5 * minimumVoxelSize *
            std::max(inputs.displayScale, 1.0e-6f) *
            std::clamp(inputs.sourcePointScale, 0.05f, 20.0f) *
            std::clamp(settings().pointRadiusMultiplier, 0.1f, 20.0f)),
        1.0e-6f);

    const auto& inspection = *inputs.inspection;
    double maximumWeight = 0.0;
    for (const auto& sample : inspection.samples) {
        maximumWeight = std::max(maximumWeight, sample.weight);
    }
    maximumWeight = std::max(maximumWeight, 1.0e-12);

    const auto toRecord = [&](std::size_t sampleIndex,
                              const std::array<std::uint8_t, 4>& color) {
        const auto& sample = inputs.cache->samples[sampleIndex];
        PointRecord record;
        record.position = toScene(
            openvdb::Vec3d(sample.worldPosition),
            inputs.referenceCenter,
            inputs.displayScale);
        const openvdb::Vec3d normal = openvdb::Vec3d(sample.normal).lengthSqr() > 1.0e-20
            ? openvdb::Vec3d(sample.normal).unit()
            : openvdb::Vec3d{0.0, 0.0, 1.0};
        record.normal = {
            static_cast<float>(normal.x()),
            static_cast<float>(normal.y()),
            static_cast<float>(normal.z())};
        record.color = color;
        return record;
    };

    constexpr std::array<std::uint8_t, 4> candidateColor{35, 130, 255, 255};
    constexpr std::array<std::uint8_t, 4> keptColor{45, 220, 120, 255};
    constexpr std::array<std::uint8_t, 4> downweightedColor{255, 175, 25, 255};
    constexpr std::array<std::uint8_t, 4> rejectedTopologyColor{245, 55, 65, 255};
    constexpr std::array<std::uint8_t, 4> rejectedResidualColor{210, 70, 245, 255};
    constexpr std::array<std::uint8_t, 4> weightLowColor{40, 120, 255, 255};
    constexpr std::array<std::uint8_t, 4> weightHighColor{255, 70, 45, 255};

    std::vector<PointRecord> records;
    records.reserve(inspection.samples.size());
    for (const auto& sample : inspection.samples) {
        if (sample.sampleIndex >= inputs.cache->samples.size() ||
            sample.sampleIndex == inspection.centerSampleIndex) {
            continue;
        }
        const bool rejected =
            sample.state == SurfaceFitNeighborhoodInspection::SampleState::RejectedTopology ||
            sample.state == SurfaceFitNeighborhoodInspection::SampleState::RejectedResidual;
        bool visible = true;
        switch (settings().displayMode) {
            case SurfaceFitNeighborhoodDisplayMode::None:
                visible = false;
                break;
            case SurfaceFitNeighborhoodDisplayMode::Candidates:
                visible = true;
                break;
            case SurfaceFitNeighborhoodDisplayMode::Kept:
            case SurfaceFitNeighborhoodDisplayMode::Weight:
                visible = !rejected;
                break;
            case SurfaceFitNeighborhoodDisplayMode::Rejected:
                visible = rejected;
                break;
        }
        if (!visible) {
            continue;
        }

        std::array<std::uint8_t, 4> color = candidateColor;
        if (settings().displayMode == SurfaceFitNeighborhoodDisplayMode::Weight) {
            color = lerpColor(
                weightLowColor,
                weightHighColor,
                sample.weight / maximumWeight);
        } else if (settings().displayMode == SurfaceFitNeighborhoodDisplayMode::Kept) {
            color = sample.state == SurfaceFitNeighborhoodInspection::SampleState::Downweighted
                ? downweightedColor
                : keptColor;
        } else if (settings().displayMode == SurfaceFitNeighborhoodDisplayMode::Rejected) {
            color = sample.state == SurfaceFitNeighborhoodInspection::SampleState::RejectedTopology
                ? rejectedTopologyColor
                : rejectedResidualColor;
        }
        records.push_back(toRecord(sample.sampleIndex, color));
    }

    createPointPrimitive(engine, mImpl->material, mImpl->points, records);
    const auto centerRecord = toRecord(
        inspection.centerSampleIndex,
        {255, 255, 255, 255});
    createPointPrimitive(engine, mImpl->material, mImpl->center, {centerRecord});
    mImpl->statistics.valid = true;
    mImpl->statistics.candidateCount = inspection.samples.size();
    mImpl->statistics.keptCount = inspection.keptSampleCount;
    mImpl->statistics.downweightedCount = inspection.downweightedSampleCount;
    mImpl->statistics.rejectedCount = inspection.rejectedSampleCount;
    mImpl->hasSelection = true;
    mImpl->updateMaterialSettings();
}

void SurfaceFitNeighborhoodDebugRenderer::updateMaterialSettings(Engine&)
{
    mImpl->updateMaterialSettings();
}

void SurfaceFitNeighborhoodDebugRenderer::setVisible(Scene& scene, bool stageVisible)
{
    const bool visible = stageVisible && mImpl->hasSelection &&
        settings().displayMode != SurfaceFitNeighborhoodDisplayMode::None;
    setPrimitiveVisible(scene, mImpl->points, visible);
    setPrimitiveVisible(scene, mImpl->center, visible);
}

void SurfaceFitNeighborhoodDebugRenderer::destroy(Engine& engine, Scene& scene)
{
    mImpl->destroyPrimitives(engine, scene);
    if (mImpl->material) {
        engine.destroy(mImpl->material);
        mImpl->material = nullptr;
    }
    mImpl->statistics = {};
    mImpl->hasSelection = false;
}

} // namespace volume_surface::viewer

