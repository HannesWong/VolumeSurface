#include "volume_surface/SurfaceTargetPreview.h"

#include "volume_surface_viewer_resources.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
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
#include <math/vec4.h>
#include <utils/EntityManager.h>

using filament::Engine;
using filament::IndexBuffer;
using filament::Material;
using filament::MaterialInstance;
using filament::RenderableManager;
using filament::Scene;
using filament::VertexBuffer;
using filament::math::float3;
using filament::math::float4;
using filament::math::short4;
using utils::Entity;
using utils::EntityManager;

namespace volume_surface {
namespace {

struct PointVertex {
    float3 position{};
    short4 tangent{};
    std::array<std::uint8_t, 4> color{};
    float4 custom0{};
};

struct PointRecord {
    float3 position{};
    float3 normal{};
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
    if (!copy) {
        throw std::bad_alloc();
    }
    std::memcpy(copy, source.data(), byteCount);
    return filament::backend::BufferDescriptor(copy, byteCount, releaseUpload);
}

void setPrimitiveVisible(Scene& scene, PrimitiveResources& primitive, bool visible)
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
    if (primitive.indexBuffer) {
        engine.destroy(primitive.indexBuffer);
        primitive.indexBuffer = nullptr;
    }
    if (primitive.vertexBuffer) {
        engine.destroy(primitive.vertexBuffer);
        primitive.vertexBuffer = nullptr;
    }
    if (primitive.materialInstance) {
        engine.destroy(primitive.materialInstance);
        primitive.materialInstance = nullptr;
    }
}

float3 transformPosition(
    const openvdb::Vec3d& position,
    const openvdb::Vec3d& referenceCenter,
    float displayScale)
{
    return (float3{
        static_cast<float>(position.x()),
        static_cast<float>(position.y()),
        static_cast<float>(position.z())} -
        float3{
            static_cast<float>(referenceCenter.x()),
            static_cast<float>(referenceCenter.y()),
            static_cast<float>(referenceCenter.z())}) * displayScale +
        float3{0.0f, 0.0f, -4.0f};
}

void updateBounds(
    const std::vector<PointVertex>& vertices,
    float3& center,
    float3& halfExtent)
{
    float3 minimum = vertices.front().position;
    float3 maximum = minimum;
    for (const auto& vertex : vertices) {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], vertex.position[axis]);
            maximum[axis] = std::max(maximum[axis], vertex.position[axis]);
        }
    }
    center = (minimum + maximum) * 0.5f;
    const float3 extent = maximum - minimum;
    constexpr float minimumExtent = 0.001f;
    halfExtent = {
        std::max(extent.x * 0.5f, minimumExtent),
        std::max(extent.y * 0.5f, minimumExtent),
        std::max(extent.z * 0.5f, minimumExtent)};
}

void updateBounds(
    const std::vector<LineVertex>& vertices,
    float3& center,
    float3& halfExtent)
{
    float3 minimum = vertices.front().position;
    float3 maximum = minimum;
    for (const auto& vertex : vertices) {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], vertex.position[axis]);
            maximum[axis] = std::max(maximum[axis], vertex.position[axis]);
        }
    }
    center = (minimum + maximum) * 0.5f;
    const float3 extent = maximum - minimum;
    constexpr float minimumExtent = 0.001f;
    halfExtent = {
        std::max(extent.x * 0.5f, minimumExtent),
        std::max(extent.y * 0.5f, minimumExtent),
        std::max(extent.z * 0.5f, minimumExtent)};
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
    std::vector<float3> sourceNormals;
    sourceNormals.reserve(records.size());
    for (std::size_t index = 0; index < records.size(); ++index) {
        vertices[index].position = records[index].position;
        vertices[index].color = records[index].color;
        sourceNormals.push_back(records[index].normal);
    }
    std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
        filament::geometry::SurfaceOrientation::Builder()
            .vertexCount(vertices.size())
            .normals(sourceNormals.data())
            .build());
    if (!orientation) {
        throw std::runtime_error("Failed to build SurfaceTarget point tangent frames");
    }
    orientation->getQuats(&vertices[0].tangent, vertices.size(), sizeof(PointVertex));

    float3 center{};
    float3 halfExtent{};
    updateBounds(vertices, center, halfExtent);

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
    primitive.vertexBuffer->setBufferAt(engine, 0, copyForUpload(vertices));

    std::vector<std::uint32_t> indices(vertices.size());
    for (std::size_t index = 0; index < indices.size(); ++index) {
        indices[index] = static_cast<std::uint32_t>(index);
    }
    primitive.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    primitive.indexBuffer->setBuffer(engine, copyForUpload(indices));

    primitive.materialInstance = material->createInstance();
    primitive.entity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({center, halfExtent})
        .material(0, primitive.materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::POINTS,
            primitive.vertexBuffer,
            primitive.indexBuffer)
        .priority(6)
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
    std::vector<float3> sourceNormals;
    sourceNormals.reserve(orientedVertices.size());
    for (const auto& vertex : orientedVertices) {
        sourceNormals.push_back(vertex.normal);
    }
    std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
        filament::geometry::SurfaceOrientation::Builder()
            .vertexCount(orientedVertices.size())
            .normals(sourceNormals.data())
            .build());
    if (!orientation) {
        throw std::runtime_error("Failed to build SurfaceTarget line tangent frames");
    }
    orientation->getQuats(
        &orientedVertices.front().tangent,
        orientedVertices.size(),
        sizeof(LineVertex));

    float3 center{};
    float3 halfExtent{};
    updateBounds(orientedVertices, center, halfExtent);
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
    primitive.vertexBuffer->setBufferAt(engine, 0, copyForUpload(orientedVertices));

    std::vector<std::uint32_t> indices(orientedVertices.size());
    for (std::size_t index = 0; index < indices.size(); ++index) {
        indices[index] = static_cast<std::uint32_t>(index);
    }
    primitive.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    primitive.indexBuffer->setBuffer(engine, copyForUpload(indices));

    primitive.materialInstance = material->createInstance();
    primitive.entity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({center, halfExtent})
        .material(0, primitive.materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::LINES,
            primitive.vertexBuffer,
            primitive.indexBuffer)
        .priority(7)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, primitive.entity);
}

} // namespace

struct SurfaceTargetPreview::Impl {
    Settings settings;
    Statistics statistics;
    Material* pointMaterial = nullptr;
    Material* debugMaterial = nullptr;
    PrimitiveResources corePoints;
    PrimitiveResources transitionPoints;
    PrimitiveResources normals;
    PrimitiveResources bvh;
    float basePointRadiusScene = 0.001f;

    void ensureMaterials(Engine& engine)
    {
        if (!pointMaterial) {
            pointMaterial = Material::Builder()
                .package(
                    VOLUME_SURFACE_VIEWER_RESOURCES_SURFACETARGETPOINTS_DATA,
                    VOLUME_SURFACE_VIEWER_RESOURCES_SURFACETARGETPOINTS_SIZE)
                .build(engine);
            if (!pointMaterial) {
                throw std::runtime_error("Failed to create SurfaceTarget point material");
            }
        }
        if (!debugMaterial) {
            debugMaterial = Material::Builder()
                .package(
                    VOLUME_SURFACE_VIEWER_RESOURCES_SURFACETARGETDEBUG_DATA,
                    VOLUME_SURFACE_VIEWER_RESOURCES_SURFACETARGETDEBUG_SIZE)
                .build(engine);
            if (!debugMaterial) {
                throw std::runtime_error("Failed to create SurfaceTarget debug material");
            }
        }
    }

    void destroyPrimitives(Engine& engine, Scene& scene)
    {
        destroyPrimitive(engine, scene, corePoints);
        destroyPrimitive(engine, scene, transitionPoints);
        destroyPrimitive(engine, scene, normals);
        destroyPrimitive(engine, scene, bvh);
    }

    void updateMaterialSettings()
    {
        const float radius = std::max(
            basePointRadiusScene * std::clamp(settings.pointScale, 0.05f, 20.0f),
            1.0e-6f);
        for (auto* instance : {
                 corePoints.materialInstance,
                 transitionPoints.materialInstance}) {
            if (!instance) {
                continue;
            }
            instance->setParameter("POINT_RADIUS", radius);
            instance->setParameter("NORMAL_CULL", false);
            instance->setParameter("AMBIENT_LEVEL", 0.12f);
            instance->setParameter("SPECULAR_STRENGTH", 0.22f);
            instance->setParameter("SHININESS", 24.0f);
        }
        if (normals.materialInstance) {
            // Normal diagnostics must remain visible from both sides.
            normals.materialInstance->setParameter("NORMAL_CULL", false);
        }
        if (bvh.materialInstance) {
            bvh.materialInstance->setParameter("NORMAL_CULL", false);
        }
    }
};

SurfaceTargetPreview::SurfaceTargetPreview()
    : mImpl(std::make_unique<Impl>())
{
}

SurfaceTargetPreview::~SurfaceTargetPreview() = default;

SurfaceTargetPreview::SurfaceTargetPreview(SurfaceTargetPreview&&) noexcept = default;

SurfaceTargetPreview& SurfaceTargetPreview::operator=(SurfaceTargetPreview&&) noexcept = default;

SurfaceTargetPreview::Settings& SurfaceTargetPreview::settings() noexcept
{
    return mImpl->settings;
}

const SurfaceTargetPreview::Settings& SurfaceTargetPreview::settings() const noexcept
{
    return mImpl->settings;
}

const SurfaceTargetPreview::Statistics& SurfaceTargetPreview::statistics() const noexcept
{
    return mImpl->statistics;
}

void SurfaceTargetPreview::rebuild(
    Engine& engine,
    Scene& scene,
    const Inputs& inputs)
{
    mImpl->destroyPrimitives(engine, scene);
    mImpl->statistics = {};

    const bool hasCache = inputs.cache && !inputs.cache->empty();
    const bool hasFallback = inputs.fallbackMesh && !inputs.fallbackMesh->empty();
    if (!hasCache && !hasFallback) {
        return;
    }
    mImpl->ensureMaterials(engine);

    const std::size_t stride = std::max<std::size_t>(1, mImpl->settings.pointStride);
    const double minimumVoxelSize = std::max(
        std::min({
            std::abs(inputs.voxelSize.x()),
            std::abs(inputs.voxelSize.y()),
            std::abs(inputs.voxelSize.z())}),
        1.0e-9);
    mImpl->basePointRadiusScene = static_cast<float>(
        0.5 * minimumVoxelSize * std::max(inputs.displayScale, 1.0e-6f));

    std::vector<PointRecord> coreVertices;
    std::vector<PointRecord> transitionVertices;
    std::vector<LineVertex> normalVertices;
    constexpr std::array<std::uint8_t, 4> normalBaseColor{35, 125, 255, 220};
    constexpr std::array<std::uint8_t, 4> normalTipColor{255, 55, 70, 245};
    if (hasCache) {
        const bool hasNormalOverrides = inputs.normalOverrides &&
            inputs.normalOverrides->size() == inputs.cache->samples.size();
        coreVertices.reserve(inputs.cache->coreCount / stride + 1);
        transitionVertices.reserve(inputs.cache->transitionCount / stride + 1);
        normalVertices.reserve(inputs.cache->samples.size() / stride * 2 + 2);
        for (std::size_t index = 0; index < inputs.cache->samples.size(); index += stride) {
            const auto& sample = inputs.cache->samples[index];
            const openvdb::Vec3d normal = hasNormalOverrides
                ? openvdb::Vec3d((*inputs.normalOverrides)[index])
                : openvdb::Vec3d(sample.normal);
            if (!normal.isFinite() || normal.lengthSqr() <= 1.0e-20) {
                continue;
            }
            const float3 position = transformPosition(
                openvdb::Vec3d(sample.worldPosition),
                inputs.referenceCenter,
                inputs.displayScale);
            const std::array<std::uint8_t, 4> color =
                sample.kind == SurfaceTargetSampleKind::Core
                ? std::array<std::uint8_t, 4>{255, 225, 70, 255}
                : std::array<std::uint8_t, 4>{255, 130, 70, 255};
            PointRecord vertex;
            vertex.position = position;
            vertex.normal = {
                static_cast<float>(normal.x()),
                static_cast<float>(normal.y()),
                static_cast<float>(normal.z())};
            vertex.color = color;
            if (sample.kind == SurfaceTargetSampleKind::Core) {
                coreVertices.push_back(vertex);
                ++mImpl->statistics.corePointCount;
            } else {
                transitionVertices.push_back(vertex);
                ++mImpl->statistics.transitionPointCount;
            }
            if (sample.kind == SurfaceTargetSampleKind::Core) {
                normalVertices.push_back({
                    position,
                    normalBaseColor,
                    vertex.normal});
                const float3 endpoint = position + float3{
                    static_cast<float>(normal.x()),
                    static_cast<float>(normal.y()),
                    static_cast<float>(normal.z())} *
                    (4.0f * 0.001f * inputs.displayScale);
                normalVertices.push_back({endpoint, normalTipColor, vertex.normal});
            }
        }
    } else {
        const auto& mesh = *inputs.fallbackMesh;
        coreVertices.reserve(mesh.vertices.size() / stride + 1);
        normalVertices.reserve(mesh.vertices.size() / stride * 2 + 2);
        for (std::size_t index = 0; index < mesh.vertices.size(); index += stride) {
            const auto& source = mesh.vertices[index];
            const openvdb::Vec3d position{
                source.position[0], source.position[1], source.position[2]};
            const openvdb::Vec3d normal{
                source.normal[0], source.normal[1], source.normal[2]};
            if (!normal.isFinite() || normal.lengthSqr() <= 1.0e-20) {
                continue;
            }
            const float3 point = transformPosition(
                position,
                inputs.referenceCenter,
                inputs.displayScale);
            PointRecord vertex;
            vertex.position = point;
            vertex.normal = {
                static_cast<float>(normal.x()),
                static_cast<float>(normal.y()),
                static_cast<float>(normal.z())};
            vertex.color = {255, 225, 70, 255};
            coreVertices.push_back(vertex);
            ++mImpl->statistics.corePointCount;
            normalVertices.push_back({point, normalBaseColor, vertex.normal});
            normalVertices.push_back({
                point + float3{
                    static_cast<float>(normal.x()),
                    static_cast<float>(normal.y()),
                    static_cast<float>(normal.z())} *
                    (4.0f * 0.001f * inputs.displayScale),
                normalTipColor,
                vertex.normal});
        }
    }

    createPointPrimitive(engine, mImpl->pointMaterial, mImpl->corePoints, std::move(coreVertices));
    createPointPrimitive(
        engine,
        mImpl->pointMaterial,
        mImpl->transitionPoints,
        std::move(transitionVertices));
    createLinePrimitive(engine, mImpl->debugMaterial, mImpl->normals, normalVertices);
    mImpl->statistics.normalSegmentCount = normalVertices.size() / 2;

    if (inputs.hierarchy) {
        const auto nodes = inputs.hierarchy->debugNodesAtDepth(mImpl->settings.bvhDepth);
        constexpr std::array<std::pair<int, int>, 12> boxEdges{{
            {0, 1}, {1, 3}, {3, 2}, {2, 0},
            {4, 5}, {5, 7}, {7, 6}, {6, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
        std::vector<LineVertex> bvhVertices;
        bvhVertices.reserve(nodes.size() * boxEdges.size() * 2);
        for (const auto& node : nodes) {
            const std::array<openvdb::Vec3d, 8> corners{{
                {node.worldMinimum.x(), node.worldMinimum.y(), node.worldMinimum.z()},
                {node.worldMaximum.x(), node.worldMinimum.y(), node.worldMinimum.z()},
                {node.worldMinimum.x(), node.worldMaximum.y(), node.worldMinimum.z()},
                {node.worldMaximum.x(), node.worldMaximum.y(), node.worldMinimum.z()},
                {node.worldMinimum.x(), node.worldMinimum.y(), node.worldMaximum.z()},
                {node.worldMaximum.x(), node.worldMinimum.y(), node.worldMaximum.z()},
                {node.worldMinimum.x(), node.worldMaximum.y(), node.worldMaximum.z()},
                {node.worldMaximum.x(), node.worldMaximum.y(), node.worldMaximum.z()}}};
            const std::array<std::uint8_t, 4> color = node.terminal
                ? std::array<std::uint8_t, 4>{255, 150, 70, 255}
                : std::array<std::uint8_t, 4>{90, 170, 255, 255};
            for (const auto [from, to] : boxEdges) {
                bvhVertices.push_back({
                    transformPosition(corners[from], inputs.referenceCenter, inputs.displayScale),
                    color});
                bvhVertices.push_back({
                    transformPosition(corners[to], inputs.referenceCenter, inputs.displayScale),
                    color});
            }
        }
        createLinePrimitive(engine, mImpl->debugMaterial, mImpl->bvh, bvhVertices);
        mImpl->statistics.bvhSegmentCount = bvhVertices.size() / 2;
    }

    mImpl->updateMaterialSettings();
}

void SurfaceTargetPreview::updateMaterialSettings(Engine&)
{
    mImpl->updateMaterialSettings();
}

void SurfaceTargetPreview::setVisible(Scene& scene, bool stageVisible)
{
    const auto& settings = mImpl->settings;
    setPrimitiveVisible(
        scene,
        mImpl->corePoints,
        stageVisible && settings.showPoints && settings.showCore);
    setPrimitiveVisible(
        scene,
        mImpl->transitionPoints,
        stageVisible && settings.showPoints && settings.showTransition);
    setPrimitiveVisible(
        scene,
        mImpl->normals,
        stageVisible && settings.showNormals);
    setPrimitiveVisible(
        scene,
        mImpl->bvh,
        stageVisible && settings.showBvh);
}

void SurfaceTargetPreview::destroy(Engine& engine, Scene& scene)
{
    mImpl->destroyPrimitives(engine, scene);
    if (mImpl->pointMaterial) {
        engine.destroy(mImpl->pointMaterial);
        mImpl->pointMaterial = nullptr;
    }
    if (mImpl->debugMaterial) {
        engine.destroy(mImpl->debugMaterial);
        mImpl->debugMaterial = nullptr;
    }
    mImpl->statistics = {};
}

} // namespace volume_surface
