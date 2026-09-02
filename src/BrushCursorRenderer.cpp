#include "volume_surface/viewer/BrushCursorRenderer.h"

#include "volume_surface_viewer_resources.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <vector>

#include <filament/Engine.h>
#include <filament/Camera.h>
#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <math/mat4.h>
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
using filament::View;
using filament::math::float3;
using filament::math::mat4;
using utils::Entity;
using utils::EntityManager;

template <typename T>
filament::backend::BufferDescriptor copyForUpload(const std::vector<T>& values)
{
    const std::size_t byteCount = values.size() * sizeof(T);
    void* copy = std::malloc(byteCount);
    if (!copy && byteCount != 0) {
        throw std::bad_alloc();
    }
    if (byteCount != 0) {
        std::memcpy(copy, values.data(), byteCount);
    }
    return filament::backend::BufferDescriptor(
        copy,
        byteCount,
        [](void* buffer, std::size_t, void*) { std::free(buffer); });
}

} // namespace

struct BrushCursorRenderer::Resources {
    Material* material = nullptr;
    MaterialInstance* materialInstance = nullptr;
    VertexBuffer* vertexBuffer = nullptr;
    IndexBuffer* indexBuffer = nullptr;
    Entity entity;
    bool addedToScene = false;
    std::vector<float3> positions;
};

BrushCursorRenderer::~BrushCursorRenderer() = default;

BrushCursorRenderer::BrushCursorRenderer() = default;

void BrushCursorRenderer::destroy(Engine& engine, Scene& scene)
{
    if (!mResources) {
        return;
    }
    Resources& cursor = *mResources;
    if (cursor.addedToScene && cursor.entity) {
        scene.remove(cursor.entity);
        cursor.addedToScene = false;
    }
    if (cursor.entity) {
        engine.destroy(cursor.entity);
        EntityManager::get().destroy(cursor.entity);
        cursor.entity.clear();
    }
    if (cursor.indexBuffer) {
        engine.destroy(cursor.indexBuffer);
        cursor.indexBuffer = nullptr;
    }
    if (cursor.vertexBuffer) {
        engine.destroy(cursor.vertexBuffer);
        cursor.vertexBuffer = nullptr;
    }
    if (cursor.materialInstance) {
        engine.destroy(cursor.materialInstance);
        cursor.materialInstance = nullptr;
    }
    if (cursor.material) {
        engine.destroy(cursor.material);
        cursor.material = nullptr;
    }
    cursor.positions.clear();
}

void BrushCursorRenderer::create(Engine& engine, Scene& scene)
{
    if (!mResources) {
        mResources = std::make_unique<Resources>();
    }
    destroy(engine, scene);
    Resources& cursor = *mResources;
    constexpr std::uint32_t segmentCount = 96;
    constexpr std::uint32_t ringCount = 2;
    constexpr std::uint32_t verticesPerRing = segmentCount * 2;
    constexpr std::uint32_t vertexCount = verticesPerRing * ringCount;

    cursor.positions.resize(vertexCount, float3{0.0f});
    std::vector<std::array<std::uint8_t, 4>> colors(vertexCount);
    for (std::uint32_t vertex = 0; vertex < verticesPerRing; ++vertex) {
        colors[vertex] = {255, 210, 55, 255};
        colors[verticesPerRing + vertex] = {55, 195, 255, 255};
    }

    std::vector<std::uint32_t> indices;
    indices.reserve(segmentCount * ringCount * 6);
    for (std::uint32_t ring = 0; ring < ringCount; ++ring) {
        const std::uint32_t base = ring * verticesPerRing;
        for (std::uint32_t segment = 0; segment < segmentCount; ++segment) {
            const std::uint32_t next = (segment + 1) % segmentCount;
            const std::uint32_t inner = base + segment * 2;
            const std::uint32_t outer = inner + 1;
            const std::uint32_t nextInner = base + next * 2;
            const std::uint32_t nextOuter = nextInner + 1;
            indices.insert(indices.end(), {
                inner, outer, nextOuter,
                inner, nextOuter, nextInner});
        }
    }

    cursor.vertexBuffer = VertexBuffer::Builder()
        .vertexCount(vertexCount)
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
    if (!cursor.vertexBuffer) {
        throw std::runtime_error("Failed to create brush cursor vertex buffer");
    }
    cursor.vertexBuffer->setBufferAt(engine, 0, copyForUpload(cursor.positions));
    cursor.vertexBuffer->setBufferAt(engine, 1, copyForUpload(colors));

    cursor.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    if (!cursor.indexBuffer) {
        throw std::runtime_error("Failed to create brush cursor index buffer");
    }
    cursor.indexBuffer->setBuffer(engine, copyForUpload(indices));

    cursor.material = Material::Builder()
        .package(
            VOLUME_SURFACE_VIEWER_RESOURCES_BRUSHCURSOR_DATA,
            VOLUME_SURFACE_VIEWER_RESOURCES_BRUSHCURSOR_SIZE)
        .build(engine);
    if (!cursor.material) {
        throw std::runtime_error("Failed to create brush cursor material");
    }
    cursor.materialInstance = cursor.material->createInstance();
    cursor.entity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({float3{0.0f, 0.0f, -4.0f}, float3{4.0f}})
        .material(0, cursor.materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::TRIANGLES,
            cursor.vertexBuffer,
            cursor.indexBuffer)
        .priority(7)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, cursor.entity);
}

void BrushCursorRenderer::update(
    Engine& engine,
    Scene& scene,
    View& view,
    bool visible,
    const float3& center,
    float displayScale,
    float coreRadiusMillimeters,
    float falloffRadiusMillimeters)
{
    if (!mResources || !mResources->entity) {
        return;
    }
    Resources& cursor = *mResources;
    if (!visible) {
        if (cursor.addedToScene) {
            scene.remove(cursor.entity);
            cursor.addedToScene = false;
        }
        return;
    }

    const mat4 cameraModel = view.getCamera().getModelMatrix();
    const float3 right{
        cameraModel[0][0], cameraModel[0][1], cameraModel[0][2]};
    const float3 up{
        cameraModel[1][0], cameraModel[1][1], cameraModel[1][2]};
    const float3 towardCamera{
        cameraModel[2][0], cameraModel[2][1], cameraModel[2][2]};
    const float sceneUnitsPerMillimeter = 0.001f * displayScale;
    const float cameraOffset = 0.20f * sceneUnitsPerMillimeter;
    const float ringHalfWidth = 0.35f * sceneUnitsPerMillimeter;
    const float3 ringCenter = center + towardCamera * cameraOffset;
    const std::array<float, 2> radii{
        coreRadiusMillimeters * sceneUnitsPerMillimeter,
        falloffRadiusMillimeters * sceneUnitsPerMillimeter};

    constexpr std::uint32_t segmentCount = 96;
    constexpr std::uint32_t verticesPerRing = segmentCount * 2;
    constexpr float twoPi = 6.2831853071795864769f;
    for (std::uint32_t ring = 0; ring < radii.size(); ++ring) {
        const float innerRadius = std::max(radii[ring] - ringHalfWidth, 0.0f);
        const float outerRadius = radii[ring] + ringHalfWidth;
        const std::uint32_t base = ring * verticesPerRing;
        for (std::uint32_t segment = 0; segment < segmentCount; ++segment) {
            const float angle = twoPi * segment / segmentCount;
            const float3 direction = right * std::cos(angle) + up * std::sin(angle);
            cursor.positions[base + segment * 2] =
                ringCenter + direction * innerRadius;
            cursor.positions[base + segment * 2 + 1] =
                ringCenter + direction * outerRadius;
        }
    }
    cursor.vertexBuffer->setBufferAt(engine, 0, copyForUpload(cursor.positions));

    const float boundingRadius = radii[1] + ringHalfWidth + cameraOffset;
    auto& renderableManager = engine.getRenderableManager();
    renderableManager.setAxisAlignedBoundingBox(
        renderableManager.getInstance(cursor.entity),
        {ringCenter, float3{boundingRadius}});
    if (!cursor.addedToScene) {
        scene.addEntity(cursor.entity);
        cursor.addedToScene = true;
    }
}

utils::Entity BrushCursorRenderer::entity() const noexcept
{
    return mResources ? mResources->entity : utils::Entity{};
}

std::size_t BrushCursorRenderer::vertexCount() const noexcept
{
    return mResources ? mResources->positions.size() : 0;
}

bool BrushCursorRenderer::addedToScene() const noexcept
{
    return mResources && mResources->addedToScene;
}

} // namespace volume_surface::viewer
