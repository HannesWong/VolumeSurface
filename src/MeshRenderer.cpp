#include "volume_surface/viewer/MeshRenderer.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
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
using filament::math::short4;
using utils::Entity;
using utils::EntityManager;

namespace volume_surface::viewer {
namespace {

struct GpuVertex {
    float3 position{};
    short4 tangent{};
};

struct Resources {
    MaterialInstance* opaqueMaterial = nullptr;
    MaterialInstance* transparentMaterial = nullptr;
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

void destroyResources(
    Engine& engine,
    Scene& scene,
    Resources& resources)
{
    if (resources.entity) {
        if (resources.addedToScene) {
            scene.remove(resources.entity);
            resources.addedToScene = false;
        }
        engine.destroy(resources.entity);
        EntityManager::get().destroy(resources.entity);
        resources.entity.clear();
    }
    if (resources.indexBuffer) {
        engine.destroy(resources.indexBuffer);
        resources.indexBuffer = nullptr;
    }
    if (resources.vertexBuffer) {
        engine.destroy(resources.vertexBuffer);
        resources.vertexBuffer = nullptr;
    }
    if (resources.opaqueMaterial) {
        engine.destroy(resources.opaqueMaterial);
        resources.opaqueMaterial = nullptr;
    }
    if (resources.transparentMaterial) {
        engine.destroy(resources.transparentMaterial);
        resources.transparentMaterial = nullptr;
    }
}

void setResourcesVisible(
    Scene& scene,
    Resources& resources,
    bool visible)
{
    if (!resources.entity) {
        return;
    }
    if (visible && !resources.addedToScene) {
        scene.addEntity(resources.entity);
        resources.addedToScene = true;
    } else if (!visible && resources.addedToScene) {
        scene.remove(resources.entity);
        resources.addedToScene = false;
    }
}

} // namespace

struct MeshRenderer::Impl {
    static constexpr std::size_t SlotCount = 4;
    std::array<Resources, SlotCount> resources;
};

MeshRenderer::MeshRenderer()
    : mImpl(std::make_unique<Impl>())
{
}

MeshRenderer::~MeshRenderer() = default;

MeshRenderer::MeshRenderer(MeshRenderer&&) noexcept = default;

MeshRenderer& MeshRenderer::operator=(MeshRenderer&&) noexcept = default;

void MeshRenderer::create(
    Engine& engine,
    Scene& scene,
    std::size_t slotIndex,
    const MeshSlot& slot,
    const SceneCoordinateMapper& mapper,
    const Material* opaqueMaterial,
    const Material* transparentMaterial)
{
    if (slotIndex >= Impl::SlotCount || !slot.available()) {
        return;
    }

    auto& resources = mImpl->resources[slotIndex];
    destroyResources(engine, scene, resources);

    std::vector<GpuVertex> vertices(slot.mesh.vertices.size());
    std::vector<float3> normals(slot.mesh.vertices.size());
    for (std::size_t index = 0; index < slot.mesh.vertices.size(); ++index) {
        const auto& source = slot.mesh.vertices[index];
        vertices[index].position = mapper.toScene(float3{
            source.position[0],
            source.position[1],
            source.position[2]});
        normals[index] = {
            source.normal[0],
            source.normal[1],
            source.normal[2]};
    }

    std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
        filament::geometry::SurfaceOrientation::Builder()
            .vertexCount(vertices.size())
            .normals(normals.data())
            .build());
    if (!orientation) {
        throw std::runtime_error("Failed to build Filament tangent frames");
    }
    orientation->getQuats(
        &vertices.front().tangent,
        vertices.size(),
        sizeof(GpuVertex));

    resources.vertexBuffer = VertexBuffer::Builder()
        .vertexCount(static_cast<std::uint32_t>(vertices.size()))
        .bufferCount(1)
        .attribute(
            filament::VertexAttribute::POSITION,
            0,
            VertexBuffer::AttributeType::FLOAT3,
            offsetof(GpuVertex, position),
            sizeof(GpuVertex))
        .attribute(
            filament::VertexAttribute::TANGENTS,
            0,
            VertexBuffer::AttributeType::SHORT4,
            offsetof(GpuVertex, tangent),
            sizeof(GpuVertex))
        .normalized(filament::VertexAttribute::TANGENTS)
        .build(engine);
    resources.vertexBuffer->setBufferAt(engine, 0, copyForUpload(vertices));

    resources.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(slot.mesh.indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    resources.indexBuffer->setBuffer(engine, copyForUpload(slot.mesh.indices));

    resources.opaqueMaterial = opaqueMaterial->createInstance();
    resources.transparentMaterial = transparentMaterial->createInstance();
    resources.opaqueMaterial->setCullingMode(MaterialInstance::CullingMode::NONE);
    resources.transparentMaterial->setCullingMode(MaterialInstance::CullingMode::NONE);
    resources.entity = EntityManager::get().create();

    const float3 minimum = mapper.toScene(float3{
        slot.mesh.bounds.minimum[0],
        slot.mesh.bounds.minimum[1],
        slot.mesh.bounds.minimum[2]});
    const float3 maximum = mapper.toScene(float3{
        slot.mesh.bounds.maximum[0],
        slot.mesh.bounds.maximum[1],
        slot.mesh.bounds.maximum[2]});
    const float3 low{
        std::min(minimum.x, maximum.x),
        std::min(minimum.y, maximum.y),
        std::min(minimum.z, maximum.z)};
    const float3 high{
        std::max(minimum.x, maximum.x),
        std::max(minimum.y, maximum.y),
        std::max(minimum.z, maximum.z)};
    const float3 extent = high - low;
    const float3 center = (low + high) * 0.5f;
    constexpr float minimumExtent = 0.001f;

    RenderableManager::Builder(1)
        .boundingBox({
            center,
            {
                std::max(extent.x * 0.5f, minimumExtent),
                std::max(extent.y * 0.5f, minimumExtent),
                std::max(extent.z * 0.5f, minimumExtent)}})
        .material(0, resources.opaqueMaterial)
        .geometry(
            0,
            RenderableManager::PrimitiveType::TRIANGLES,
            resources.vertexBuffer,
            resources.indexBuffer)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, resources.entity);
}

void MeshRenderer::applyStyle(
    Engine& engine,
    std::size_t slotIndex,
    const MeshSlot& slot)
{
    if (slotIndex >= Impl::SlotCount || !slot.available()) {
        return;
    }
    auto& resources = mImpl->resources[slotIndex];
    if (!resources.entity || !resources.opaqueMaterial ||
        !resources.transparentMaterial) {
        return;
    }

    const float3 color{slot.color[0], slot.color[1], slot.color[2]};
    resources.opaqueMaterial->setParameter("baseColor", color);
    resources.opaqueMaterial->setParameter("metallic", 0.0f);
    resources.opaqueMaterial->setParameter("roughness", 0.72f);
    resources.opaqueMaterial->setParameter("reflectance", 0.35f);
    resources.transparentMaterial->setParameter(
        "color",
        filament::math::float4{slot.color[0], slot.color[1], slot.color[2], slot.opacity});

    auto& manager = engine.getRenderableManager();
    const auto instance = manager.getInstance(resources.entity);
    manager.setMaterialInstanceAt(
        instance,
        0,
        slot.opacity >= 0.999f
            ? resources.opaqueMaterial
            : resources.transparentMaterial);
}

void MeshRenderer::setVisible(
    Scene& scene,
    std::size_t slotIndex,
    bool visible)
{
    if (slotIndex >= Impl::SlotCount) {
        return;
    }
    setResourcesVisible(scene, mImpl->resources[slotIndex], visible);
}

utils::Entity MeshRenderer::entity(std::size_t slotIndex) const noexcept
{
    if (slotIndex >= Impl::SlotCount) {
        return {};
    }
    return mImpl->resources[slotIndex].entity;
}

void MeshRenderer::destroy(
    Engine& engine,
    Scene& scene,
    std::size_t slotIndex)
{
    if (slotIndex >= Impl::SlotCount) {
        return;
    }
    destroyResources(engine, scene, mImpl->resources[slotIndex]);
}

void MeshRenderer::destroyAll(
    Engine& engine,
    Scene& scene)
{
    for (auto& resources : mImpl->resources) {
        destroyResources(engine, scene, resources);
    }
}

} // namespace volume_surface::viewer
