#include "volume_surface/viewer/SurfaceFitExpansionDebugRenderer.h"
#include "volume_surface_viewer_resources.h"

#include <algorithm>
#include <array>
#include <cstdint>
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

struct PointVertex {
    float3 position{};
    short4 tangent{};
    std::array<std::uint8_t, 4> color{};
    float custom0[4]{};
};

struct PointRecord {
    float3 position{};
    float3 normal{0.0f, 0.0f, 1.0f};
    std::array<std::uint8_t, 4> color{};
};

struct LineVertex {
    float3 position{};
    std::array<std::uint8_t, 4> color{};
    float3 normal{0.0f, 0.0f, 1.0f};
    short4 tangent{};
};

struct PrimitiveResources {
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
        throw std::runtime_error("Failed to build SurfaceFit expansion point frames");
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
        throw std::runtime_error("Failed to create SurfaceFit expansion point vertices");
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
        throw std::runtime_error("Failed to create SurfaceFit expansion point indices");
    }
    primitive.indexBuffer->setBuffer(engine, copyForUpload(indices));

    primitive.materialInstance = material->createInstance();
    if (!primitive.materialInstance) {
        throw std::runtime_error("Failed to create SurfaceFit expansion point material");
    }
    primitive.materialInstance->setCullingMode(MaterialInstance::CullingMode::NONE);
    primitive.materialInstance->setParameter("NORMAL_CULL", false);
    primitive.materialInstance->setParameter("AMBIENT_LEVEL", 0.45f);
    primitive.materialInstance->setParameter("SPECULAR_STRENGTH", 0.05f);
    primitive.materialInstance->setParameter("SHININESS", 8.0f);
    primitive.entity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({center, halfExtent})
        .material(0, primitive.materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::POINTS,
            primitive.vertexBuffer,
            primitive.indexBuffer)
        .priority(8)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, primitive.entity);
}

void createLinePrimitive(
    Engine& engine,
    Material* material,
    PrimitiveResources& primitive,
    const std::vector<LineVertex>& vertices)
{
    if (vertices.empty()) {
        return;
    }

    std::vector<LineVertex> orientedVertices = vertices;
    std::vector<float3> normals;
    normals.reserve(orientedVertices.size());
    for (const auto& vertex : orientedVertices) {
        normals.push_back(vertex.normal);
    }
    std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
        filament::geometry::SurfaceOrientation::Builder()
            .vertexCount(orientedVertices.size())
            .normals(normals.data())
            .build());
    if (!orientation) {
        throw std::runtime_error("Failed to build SurfaceFit expansion normal frames");
    }
    orientation->getQuats(
        &orientedVertices.front().tangent,
        orientedVertices.size(),
        sizeof(LineVertex));

    float3 minimum = orientedVertices.front().position;
    float3 maximum = minimum;
    for (const auto& vertex : orientedVertices) {
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
        .vertexCount(static_cast<std::uint32_t>(orientedVertices.size()))
        .bufferCount(1)
        .attribute(
            filament::VertexAttribute::POSITION,
            0,
            VertexBuffer::AttributeType::FLOAT3,
            offsetof(LineVertex, position),
            sizeof(LineVertex))
        .attribute(
            filament::VertexAttribute::COLOR,
            0,
            VertexBuffer::AttributeType::UBYTE4,
            offsetof(LineVertex, color),
            sizeof(LineVertex))
        .attribute(
            filament::VertexAttribute::TANGENTS,
            0,
            VertexBuffer::AttributeType::SHORT4,
            offsetof(LineVertex, tangent),
            sizeof(LineVertex))
        .normalized(filament::VertexAttribute::COLOR)
        .normalized(filament::VertexAttribute::TANGENTS)
        .build(engine);
    if (!primitive.vertexBuffer) {
        throw std::runtime_error("Failed to create SurfaceFit expansion normal vertices");
    }
    primitive.vertexBuffer->setBufferAt(
        engine,
        0,
        copyForUpload(orientedVertices));

    std::vector<std::uint32_t> indices(orientedVertices.size());
    for (std::size_t index = 0; index < indices.size(); ++index) {
        indices[index] = static_cast<std::uint32_t>(index);
    }
    primitive.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    if (!primitive.indexBuffer) {
        throw std::runtime_error("Failed to create SurfaceFit expansion normal indices");
    }
    primitive.indexBuffer->setBuffer(engine, copyForUpload(indices));

    primitive.materialInstance = material->createInstance();
    if (!primitive.materialInstance) {
        throw std::runtime_error("Failed to create SurfaceFit expansion normal material");
    }
    primitive.materialInstance->setCullingMode(MaterialInstance::CullingMode::NONE);
    primitive.materialInstance->setParameter("NORMAL_CULL", false);
    primitive.entity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({center, halfExtent})
        .material(0, primitive.materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::LINES,
            primitive.vertexBuffer,
            primitive.indexBuffer)
        .priority(9)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, primitive.entity);
}

} // namespace

struct SurfaceFitExpansionDebugRenderer::Impl {
    Settings settings;
    Statistics statistics;
    PrimitiveResources targetNextPoints;
    PrimitiveResources sourceBatchPoints;
    PrimitiveResources parentPoints;
    PrimitiveResources targetPoint;
    PrimitiveResources sourcePoint;
    PrimitiveResources normalLines;
    float pointRadiusScene = 0.0006f;
    Material* selectedPointMaterial = nullptr;
    Material* normalMaterial = nullptr;
    bool hasSelection = false;

    void ensureMaterial(Engine& engine)
    {
        if (selectedPointMaterial) {
            return;
        }
        selectedPointMaterial = Material::Builder()
            .package(
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACEFITEXPANSIONDEBUGPOINTS_DATA,
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACEFITEXPANSIONDEBUGPOINTS_SIZE)
            .build(engine);
        if (!selectedPointMaterial) {
            throw std::runtime_error(
                "Failed to create SurfaceFit expansion selected point material");
        }
        normalMaterial = Material::Builder()
            .package(
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACETARGETDEBUG_DATA,
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACETARGETDEBUG_SIZE)
            .build(engine);
        if (!normalMaterial) {
            throw std::runtime_error(
                "Failed to create SurfaceFit expansion normal material");
        }
    }

    void destroyPrimitives(Engine& engine, Scene& scene)
    {
        destroyPrimitive(engine, scene, targetNextPoints);
        destroyPrimitive(engine, scene, sourceBatchPoints);
        destroyPrimitive(engine, scene, parentPoints);
        destroyPrimitive(engine, scene, targetPoint);
        destroyPrimitive(engine, scene, sourcePoint);
        destroyPrimitive(engine, scene, normalLines);
    }

    void updateMaterialSettings()
    {
        for (auto* instance : {
                 targetNextPoints.materialInstance,
                 sourceBatchPoints.materialInstance,
                 parentPoints.materialInstance}) {
            if (instance) {
                instance->setParameter("POINT_RADIUS", pointRadiusScene);
            }
        }
        for (auto* instance : {
                 targetPoint.materialInstance,
                 sourcePoint.materialInstance}) {
            if (instance) {
                instance->setParameter(
                    "POINT_RADIUS",
                pointRadiusScene * settings.selectedPointRadiusMultiplier);
            }
        }
        if (normalLines.materialInstance) {
            normalLines.materialInstance->setParameter("NORMAL_CULL", false);
        }
    }
};

SurfaceFitExpansionDebugRenderer::SurfaceFitExpansionDebugRenderer()
    : mImpl(std::make_unique<Impl>())
{
}

SurfaceFitExpansionDebugRenderer::~SurfaceFitExpansionDebugRenderer() = default;

SurfaceFitExpansionDebugRenderer::SurfaceFitExpansionDebugRenderer(
    SurfaceFitExpansionDebugRenderer&&) noexcept = default;

SurfaceFitExpansionDebugRenderer& SurfaceFitExpansionDebugRenderer::operator=(
    SurfaceFitExpansionDebugRenderer&&) noexcept = default;

SurfaceFitExpansionDebugRenderer::Settings& SurfaceFitExpansionDebugRenderer::settings() noexcept
{
    return mImpl->settings;
}

const SurfaceFitExpansionDebugRenderer::Settings&
SurfaceFitExpansionDebugRenderer::settings() const noexcept
{
    return mImpl->settings;
}

const SurfaceFitExpansionDebugRenderer::Statistics&
SurfaceFitExpansionDebugRenderer::statistics() const noexcept
{
    return mImpl->statistics;
}

void SurfaceFitExpansionDebugRenderer::rebuild(
    Engine& engine,
    Scene& scene,
    const Inputs& inputs)
{
    mImpl->destroyPrimitives(engine, scene);
    mImpl->statistics = {};
    mImpl->hasSelection = false;
    if (!inputs.cache || !inputs.neighborhood || !inputs.neighborhood->valid) {
        return;
    }
    mImpl->ensureMaterial(engine);
    const double minimumVoxelSize = std::max(
        std::min({
            std::abs(inputs.voxelSize.x()),
            std::abs(inputs.voxelSize.y()),
            std::abs(inputs.voxelSize.z())}),
        1.0e-9);
    const float sourcePointRadiusMillimeters = static_cast<float>(
        0.5 * minimumVoxelSize * 1000.0 *
        std::clamp(inputs.sourcePointScale, 0.05f, 20.0f));
    mImpl->pointRadiusScene = std::max(
        sourcePointRadiusMillimeters * settings().pointRadiusMultiplier * 0.001f *
            std::max(inputs.displayScale, 1.0e-6f),
        1.0e-6f);

    const auto toRecord = [&](std::size_t sampleIndex,
                              const std::array<std::uint8_t, 4>& color) {
        const auto& sample = inputs.cache->samples[sampleIndex];
        PointRecord record;
        record.position = toScene(
            openvdb::Vec3d(sample.worldPosition),
            inputs.referenceCenter,
            inputs.displayScale);
        const bool hasNormalOverride = inputs.normalOverrides &&
            inputs.normalOverrides->size() == inputs.cache->samples.size();
        const openvdb::Vec3d sourceNormal = hasNormalOverride
            ? openvdb::Vec3d((*inputs.normalOverrides)[sampleIndex])
            : openvdb::Vec3d(sample.normal);
        const openvdb::Vec3d normal = sourceNormal.lengthSqr() > 1.0e-20
            ? sourceNormal.unit()
            : openvdb::Vec3d{0.0, 0.0, 1.0};
        record.normal = {
            static_cast<float>(normal.x()),
            static_cast<float>(normal.y()),
            static_cast<float>(normal.z())};
        record.color = color;
        return record;
    };

    const bool unified = settings().showUnifiedNeighborhood;
    constexpr std::array<std::uint8_t, 4> unifiedColor{185, 85, 255, 255};
    const std::size_t targetIndex = inputs.neighborhood->targetSampleIndex;
    std::vector<PointRecord> targetNext;
    if (unified && inputs.fitNeighborhood && inputs.fitNeighborhood->valid) {
        targetNext.reserve(inputs.fitNeighborhood->sampleIndices.size());
        for (const std::size_t index : inputs.fitNeighborhood->sampleIndices) {
            if (index < inputs.cache->samples.size() && index != targetIndex) {
                targetNext.push_back(toRecord(index, unifiedColor));
            }
        }
    } else {
        targetNext.reserve(inputs.neighborhood->targetNextSampleIndices.size());
        for (const std::size_t index : inputs.neighborhood->targetNextSampleIndices) {
            if (index < inputs.cache->samples.size()) {
                targetNext.push_back(toRecord(
                    index,
                    unified ? unifiedColor : std::array<std::uint8_t, 4>{35, 120, 255, 255}));
            }
        }
    }
    std::vector<PointRecord> sourceBatch;
    if (!unified || !inputs.fitNeighborhood || !inputs.fitNeighborhood->valid) {
        sourceBatch.reserve(inputs.neighborhood->sourceBatchSampleIndices.size());
        for (const std::size_t index : inputs.neighborhood->sourceBatchSampleIndices) {
            if (index < inputs.cache->samples.size()) {
                sourceBatch.push_back(toRecord(
                    index,
                    unified ? unifiedColor : std::array<std::uint8_t, 4>{255, 145, 30, 255}));
            }
        }
    }
    constexpr std::array<std::uint8_t, 4> parentColor{70, 235, 175, 255};
    std::vector<PointRecord> parents;
    bool hasParentTrace = false;
    if (!unified && inputs.expansionTrace &&
        inputs.expansionTrace->matchesSampleCount(inputs.cache->samples.size()) &&
        targetIndex < inputs.expansionTrace->depthBySample.size() &&
        inputs.expansionTrace->depthBySample[targetIndex] >= 0) {
        hasParentTrace = true;
        const std::int32_t threshold = std::max(inputs.parentDepthThreshold, 0);
        std::size_t currentIndex = targetIndex;
        for (std::size_t guard = 0; guard < inputs.cache->samples.size(); ++guard) {
            const std::int32_t parentIndex =
                inputs.expansionTrace->parentBySample[currentIndex];
            if (parentIndex < 0 ||
                static_cast<std::size_t>(parentIndex) >= inputs.cache->samples.size()) {
                break;
            }
            currentIndex = static_cast<std::size_t>(parentIndex);
            const std::int32_t parentDepth =
                inputs.expansionTrace->depthBySample[currentIndex];
            if (parentDepth >= threshold) {
                parents.push_back(toRecord(currentIndex, parentColor));
            }
        }
    }
    std::vector<PointRecord> visibleRecords;
    if (settings().showCurrentPointNormals) {
        visibleRecords.reserve(
            targetNext.size() + sourceBatch.size() + parents.size() + 2);
        visibleRecords.insert(visibleRecords.end(), targetNext.begin(), targetNext.end());
        visibleRecords.insert(visibleRecords.end(), sourceBatch.begin(), sourceBatch.end());
        visibleRecords.insert(visibleRecords.end(), parents.begin(), parents.end());
    }
    createPointPrimitive(
        engine,
        mImpl->selectedPointMaterial,
        mImpl->targetNextPoints,
        targetNext);
    createPointPrimitive(
        engine,
        mImpl->selectedPointMaterial,
        mImpl->sourceBatchPoints,
        sourceBatch);
    createPointPrimitive(
        engine,
        mImpl->selectedPointMaterial,
        mImpl->parentPoints,
        parents);

    PointRecord targetPointRecord;
    if (targetIndex < inputs.cache->samples.size()) {
        targetPointRecord = toRecord(targetIndex, {255, 255, 255, 255});
        createPointPrimitive(
            engine,
            mImpl->selectedPointMaterial,
            mImpl->targetPoint,
            {targetPointRecord});
        if (settings().showCurrentPointNormals) {
            visibleRecords.push_back(targetPointRecord);
        }
    }
    const std::size_t sourceIndex = inputs.neighborhood->sourceSampleIndex;
    if (!unified && !hasParentTrace && sourceIndex < inputs.cache->samples.size()) {
        const PointRecord sourcePointRecord = toRecord(
            sourceIndex,
            {255, 60, 60, 255});
        createPointPrimitive(
            engine,
            mImpl->selectedPointMaterial,
            mImpl->sourcePoint,
            {sourcePointRecord});
        if (settings().showCurrentPointNormals) {
            visibleRecords.push_back(sourcePointRecord);
        }
    }
    if (settings().showCurrentPointNormals && !visibleRecords.empty()) {
        std::vector<LineVertex> normalVertices;
        normalVertices.reserve(visibleRecords.size() * 2);
        constexpr std::array<std::uint8_t, 4> normalBaseColor{35, 145, 255, 255};
        constexpr std::array<std::uint8_t, 4> normalTipColor{255, 70, 80, 255};
        const float normalLength = 4.0e-3f * std::max(inputs.displayScale, 1.0e-6f);
        for (const auto& record : visibleRecords) {
            normalVertices.push_back({record.position, normalBaseColor, record.normal});
            normalVertices.push_back({
                record.position + record.normal * normalLength,
                normalTipColor,
                record.normal});
        }
        createLinePrimitive(
            engine,
            mImpl->normalMaterial,
            mImpl->normalLines,
            normalVertices);
    }
    mImpl->statistics.valid = true;
    mImpl->statistics.targetNextPointCount = targetNext.size();
    mImpl->statistics.sourceBatchPointCount = sourceBatch.size();
    mImpl->statistics.parentPointCount = parents.size();
    mImpl->hasSelection = true;
    mImpl->updateMaterialSettings();
}

void SurfaceFitExpansionDebugRenderer::updateMaterialSettings(Engine&)
{
    mImpl->updateMaterialSettings();
}

void SurfaceFitExpansionDebugRenderer::setVisible(Scene& scene, bool stageVisible)
{
    const bool visible = stageVisible &&
        mImpl->hasSelection &&
        mImpl->settings.showNeighborhood;
    setPrimitiveVisible(scene, mImpl->targetNextPoints, visible);
    setPrimitiveVisible(scene, mImpl->sourceBatchPoints, visible);
    setPrimitiveVisible(scene, mImpl->parentPoints, visible);
    setPrimitiveVisible(scene, mImpl->targetPoint, visible);
    setPrimitiveVisible(scene, mImpl->sourcePoint, visible);
    setPrimitiveVisible(
        scene,
        mImpl->normalLines,
        visible && mImpl->settings.showCurrentPointNormals);
}

void SurfaceFitExpansionDebugRenderer::destroy(Engine& engine, Scene& scene)
{
    mImpl->destroyPrimitives(engine, scene);
    if (mImpl->selectedPointMaterial) {
        engine.destroy(mImpl->selectedPointMaterial);
        mImpl->selectedPointMaterial = nullptr;
    }
    if (mImpl->normalMaterial) {
        engine.destroy(mImpl->normalMaterial);
        mImpl->normalMaterial = nullptr;
    }
    mImpl->statistics = {};
    mImpl->hasSelection = false;
}

} // namespace volume_surface::viewer
