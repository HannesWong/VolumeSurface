#include "volume_surface_viewer_resources.h"
#include "volume_surface/viewer/SurfaceNormalLocalSeedRenderer.h"

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

struct Primitive {
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
    const std::size_t bytes = source.size() * sizeof(T);
    void* copy = std::malloc(bytes);
    if (!copy && bytes != 0) {
        throw std::bad_alloc();
    }
    if (bytes != 0) {
        std::memcpy(copy, source.data(), bytes);
    }
    return filament::backend::BufferDescriptor(copy, bytes, releaseUpload);
}

void destroyPrimitive(Engine& engine, Scene& scene, Primitive& primitive)
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

void setPrimitiveVisible(Scene& scene, Primitive& primitive, bool visible)
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
    Primitive& primitive,
    const std::vector<PointVertex>& vertices,
    float pointRadius)
{
    if (vertices.empty() || !material) {
        return;
    }
    std::vector<PointVertex> upload = vertices;
    std::vector<float3> normals(upload.size(), float3{0.0f, 0.0f, 1.0f});
    std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
        filament::geometry::SurfaceOrientation::Builder()
            .vertexCount(upload.size())
            .normals(normals.data())
            .build());
    if (!orientation) {
        throw std::runtime_error("Failed to build local flip-point frames");
    }
    orientation->getQuats(&upload.front().tangent, upload.size(), sizeof(PointVertex));

    float3 minimum = upload.front().position;
    float3 maximum = minimum;
    for (const auto& vertex : upload) {
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
        .vertexCount(static_cast<std::uint32_t>(upload.size()))
        .bufferCount(1)
        .attribute(filament::VertexAttribute::POSITION, 0,
                   VertexBuffer::AttributeType::FLOAT3,
                   offsetof(PointVertex, position), sizeof(PointVertex))
        .attribute(filament::VertexAttribute::TANGENTS, 0,
                   VertexBuffer::AttributeType::SHORT4,
                   offsetof(PointVertex, tangent), sizeof(PointVertex))
        .attribute(filament::VertexAttribute::COLOR, 0,
                   VertexBuffer::AttributeType::UBYTE4,
                   offsetof(PointVertex, color), sizeof(PointVertex))
        .attribute(filament::VertexAttribute::CUSTOM0, 0,
                   VertexBuffer::AttributeType::FLOAT4,
                   offsetof(PointVertex, custom0), sizeof(PointVertex))
        .normalized(filament::VertexAttribute::TANGENTS)
        .normalized(filament::VertexAttribute::COLOR)
        .build(engine);
    if (!primitive.vertexBuffer) {
        throw std::runtime_error("Failed to create local flip-point vertices");
    }
    primitive.vertexBuffer->setBufferAt(engine, 0, copyForUpload(upload));

    std::vector<std::uint32_t> indices(upload.size());
    for (std::size_t index = 0; index < indices.size(); ++index) {
        indices[index] = static_cast<std::uint32_t>(index);
    }
    primitive.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    if (!primitive.indexBuffer) {
        throw std::runtime_error("Failed to create local flip-point indices");
    }
    primitive.indexBuffer->setBuffer(engine, copyForUpload(indices));
    primitive.materialInstance = material->createInstance();
    if (!primitive.materialInstance) {
        throw std::runtime_error("Failed to create local flip-point material");
    }
    primitive.materialInstance->setCullingMode(MaterialInstance::CullingMode::NONE);
    primitive.materialInstance->setParameter("POINT_RADIUS", pointRadius);
    primitive.entity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({center, halfExtent})
        .material(0, primitive.materialInstance)
        .geometry(0, RenderableManager::PrimitiveType::POINTS,
                  primitive.vertexBuffer, primitive.indexBuffer)
        .priority(10)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, primitive.entity);
}

std::vector<PointVertex> makeVertices(
    const SurfaceTargetCache& cache,
    const std::vector<std::size_t>& indices,
    const openvdb::Vec3d& referenceCenter,
    float displayScale,
    const std::array<std::uint8_t, 4>& color)
{
    std::vector<PointVertex> vertices;
    vertices.reserve(indices.size());
    for (const std::size_t index : indices) {
        if (index >= cache.samples.size()) {
            continue;
        }
        PointVertex vertex;
        vertex.position = toScene(
            openvdb::Vec3d(cache.samples[index].worldPosition),
            referenceCenter,
            displayScale);
        vertex.color = color;
        vertex.custom0[0] = 0.0f;
        vertex.custom0[1] = 0.0f;
        vertex.custom0[2] = 0.0f;
        vertex.custom0[3] = 0.0f;
        vertices.push_back(vertex);
    }
    return vertices;
}

std::vector<PointVertex> makeFlipPointVertices(
    const std::vector<SurfaceNormalLocalSeedRenderer::FlipPoint>& points,
    const openvdb::Vec3d& referenceCenter,
    float displayScale)
{
    std::vector<PointVertex> vertices;
    vertices.reserve(points.size());
    for (const auto& point : points) {
        PointVertex vertex;
        vertex.position = toScene(
            point.worldPosition,
            referenceCenter,
            displayScale);
        vertex.color = point.color;
        if (!point.enabled) {
            vertex.color[3] = static_cast<std::uint8_t>(
                std::min<unsigned int>(vertex.color[3], 72));
        }
        vertex.custom0[0] = 0.0f;
        vertex.custom0[1] = 0.0f;
        vertex.custom0[2] = 0.0f;
        vertex.custom0[3] = 0.0f;
        vertices.push_back(vertex);
    }
    return vertices;
}

} // namespace

struct SurfaceNormalLocalSeedRenderer::Impl {
    Settings settings;
    Primitive affected;
    Primitive boundary;
    Primitive seed;
    Primitive flipPoints;
    Material* material = nullptr;
    bool hasPreview = false;
    bool hasFlipPoints = false;

    void ensureMaterial(Engine& engine)
    {
        if (material) {
            return;
        }
        material = Material::Builder()
            .package(
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACENORMALLOCALSEEDPREVIEWPOINTS_DATA,
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACENORMALLOCALSEEDPREVIEWPOINTS_SIZE)
            .build(engine);
        if (!material) {
            throw std::runtime_error("Failed to create local flip-point preview material");
        }
    }

    void destroyPrimitives(Engine& engine, Scene& scene)
    {
        destroyPrimitive(engine, scene, affected);
        destroyPrimitive(engine, scene, boundary);
        destroyPrimitive(engine, scene, seed);
        destroyPrimitive(engine, scene, flipPoints);
    }
};

SurfaceNormalLocalSeedRenderer::SurfaceNormalLocalSeedRenderer()
    : mImpl(std::make_unique<Impl>())
{
}

SurfaceNormalLocalSeedRenderer::~SurfaceNormalLocalSeedRenderer() = default;

SurfaceNormalLocalSeedRenderer::SurfaceNormalLocalSeedRenderer(
    SurfaceNormalLocalSeedRenderer&&) noexcept = default;

SurfaceNormalLocalSeedRenderer& SurfaceNormalLocalSeedRenderer::operator=(
    SurfaceNormalLocalSeedRenderer&&) noexcept = default;

SurfaceNormalLocalSeedRenderer::Settings& SurfaceNormalLocalSeedRenderer::settings() noexcept
{
    return mImpl->settings;
}

const SurfaceNormalLocalSeedRenderer::Settings&
SurfaceNormalLocalSeedRenderer::settings() const noexcept
{
    return mImpl->settings;
}

void SurfaceNormalLocalSeedRenderer::rebuild(
    Engine& engine,
    Scene& scene,
    const Inputs& inputs)
{
    mImpl->destroyPrimitives(engine, scene);
    mImpl->hasPreview = false;
    if (!inputs.cache || !inputs.preview || !inputs.preview->valid) {
        return;
    }
    mImpl->ensureMaterial(engine);
    const float voxelRadius = std::max(inputs.minimumVoxelSize * 0.5f, 1.0e-6f);
    const float baseRadius = voxelRadius * std::clamp(inputs.sourcePointScale, 0.05f, 20.0f) *
        std::max(inputs.displayScale, 1.0e-6f);
    const auto affectedVertices = makeVertices(
        *inputs.cache,
        inputs.preview->affectedCoreSampleIndices,
        inputs.referenceCenter,
        inputs.displayScale,
        {40, 220, 255, 105});
    const auto boundaryVertices = makeVertices(
        *inputs.cache,
        inputs.preview->boundaryCoreSampleIndices,
        inputs.referenceCenter,
        inputs.displayScale,
        {255, 150, 40, 135});
    const std::vector<std::size_t> seedIndices{inputs.preview->seedSampleIndex};
    const auto seedVertices = makeVertices(
        *inputs.cache,
        seedIndices,
        inputs.referenceCenter,
        inputs.displayScale,
        {255, 80, 230, 255});
    createPointPrimitive(
        engine,
        mImpl->material,
        mImpl->affected,
        affectedVertices,
        baseRadius * mImpl->settings.affectedPointRadiusMultiplier);
    createPointPrimitive(
        engine,
        mImpl->material,
        mImpl->boundary,
        boundaryVertices,
        baseRadius * mImpl->settings.boundaryPointRadiusMultiplier);
    createPointPrimitive(
        engine,
        mImpl->material,
        mImpl->seed,
        seedVertices,
        baseRadius * mImpl->settings.seedPointRadiusMultiplier);
    mImpl->hasPreview = true;
}

void SurfaceNormalLocalSeedRenderer::rebuildFlipPoints(
    Engine& engine,
    Scene& scene,
    const FlipPointInputs& inputs)
{
    destroyPrimitive(engine, scene, mImpl->flipPoints);
    mImpl->hasFlipPoints = false;
    if (!inputs.points || inputs.points->empty()) {
        return;
    }
    mImpl->ensureMaterial(engine);
    const float voxelRadius = std::max(inputs.minimumVoxelSize * 0.5f, 1.0e-6f);
    const float baseRadius = voxelRadius *
        std::clamp(inputs.sourcePointScale, 0.05f, 20.0f) *
        std::max(inputs.displayScale, 1.0e-6f);
    const auto vertices = makeFlipPointVertices(
        *inputs.points,
        inputs.referenceCenter,
        inputs.displayScale);
    createPointPrimitive(
        engine,
        mImpl->material,
        mImpl->flipPoints,
        vertices,
        baseRadius * 2.0f);
    mImpl->hasFlipPoints = true;
}

void SurfaceNormalLocalSeedRenderer::setVisible(Scene& scene, bool stageVisible)
{
    // The affected/boundary local-seed debug primitives are intentionally
    // retired. Only explicit flip-point markers remain visible in this stage.
    setPrimitiveVisible(scene, mImpl->affected, false);
    setPrimitiveVisible(scene, mImpl->boundary, false);
    setPrimitiveVisible(scene, mImpl->seed, false);
    setPrimitiveVisible(
        scene,
        mImpl->flipPoints,
        stageVisible && mImpl->hasFlipPoints);
}

void SurfaceNormalLocalSeedRenderer::destroy(Engine& engine, Scene& scene)
{
    mImpl->destroyPrimitives(engine, scene);
    if (mImpl->material) {
        engine.destroy(mImpl->material);
        mImpl->material = nullptr;
    }
    mImpl->hasPreview = false;
    mImpl->hasFlipPoints = false;
}

} // namespace volume_surface::viewer
