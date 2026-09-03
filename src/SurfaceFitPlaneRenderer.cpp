#include "volume_surface/viewer/SurfaceFitPlaneRenderer.h"

#include "volume_surface_viewer_resources.h"

#include <algorithm>
#include <array>
#include <cmath>
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
#include <math/vec3.h>
#include <math/vec4.h>
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
using filament::math::float4;
using utils::Entity;
using utils::EntityManager;

struct PlaneVertex {
    float3 position{};
};

struct PrimitiveResources {
    MaterialInstance* materialInstance = nullptr;
    VertexBuffer* vertexBuffer = nullptr;
    IndexBuffer* indexBuffer = nullptr;
    Entity entity;
    bool addedToScene = false;
};

enum class PlanePass : std::uint8_t {
    Probe,
    Success,
    Failure,
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

float3 toFloat3(const openvdb::Vec3d& value)
{
    return {
        static_cast<float>(value.x()),
        static_cast<float>(value.y()),
        static_cast<float>(value.z())};
}

std::array<std::uint8_t, 4> colorToBytes(const std::array<float, 4>& color)
{
    std::array<std::uint8_t, 4> result{};
    for (std::size_t index = 0; index < result.size(); ++index) {
        result[index] = static_cast<std::uint8_t>(std::lround(
            std::clamp(color[index], 0.0f, 1.0f) * 255.0f));
    }
    return result;
}

void configureMaterialInstance(
    MaterialInstance& instance,
    PlanePass pass,
    const float4& color,
    float depthBias)
{
    instance.setParameter("BASE_COLOR", color);
    instance.setCullingMode(MaterialInstance::CullingMode::NONE);
    instance.setDepthWrite(false);
    instance.setPolygonOffset(0.0f, depthBias);

    switch (pass) {
    case PlanePass::Probe:
        instance.setColorWrite(false);
        instance.setDepthCulling(true);
        instance.setStencilWrite(true);
        instance.setStencilCompareFunction(MaterialInstance::StencilCompareFunc::A);
        instance.setStencilReferenceValue(1);
        instance.setStencilOpStencilFail(MaterialInstance::StencilOperation::KEEP);
        instance.setStencilOpDepthFail(MaterialInstance::StencilOperation::REPLACE);
        instance.setStencilOpDepthStencilPass(MaterialInstance::StencilOperation::KEEP);
        break;
    case PlanePass::Success:
        instance.setColorWrite(true);
        instance.setDepthCulling(true);
        instance.setStencilWrite(false);
        instance.setStencilCompareFunction(MaterialInstance::StencilCompareFunc::A);
        break;
    case PlanePass::Failure:
        instance.setColorWrite(true);
        instance.setDepthCulling(false);
        instance.setStencilWrite(false);
        instance.setStencilCompareFunction(MaterialInstance::StencilCompareFunc::E);
        instance.setStencilReferenceValue(1);
        instance.setPolygonOffset(0.0f, 0.0f);
        break;
    }
}

void createPlanePrimitive(
    Engine& engine,
    Material* material,
    PrimitiveResources& primitive,
    const std::array<float3, 4>& corners,
    const std::array<float, 4>& color,
    PlanePass pass,
    std::uint8_t priority,
    std::uint16_t blendOrder,
    float depthBias)
{
    std::vector<PlaneVertex> vertices;
    vertices.reserve(corners.size());
    for (const auto& corner : corners) {
        vertices.push_back({corner});
    }
    const std::vector<std::uint32_t> indices{0, 1, 2, 0, 2, 3};

    primitive.vertexBuffer = VertexBuffer::Builder()
        .vertexCount(static_cast<std::uint32_t>(vertices.size()))
        .bufferCount(1)
        .attribute(
            filament::VertexAttribute::POSITION,
            0,
            VertexBuffer::AttributeType::FLOAT3,
            offsetof(PlaneVertex, position),
            sizeof(PlaneVertex))
        .build(engine);
    if (!primitive.vertexBuffer) {
        throw std::runtime_error("Failed to create SurfaceFit plane vertex buffer");
    }
    primitive.vertexBuffer->setBufferAt(engine, 0, copyForUpload(vertices));

    primitive.indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    if (!primitive.indexBuffer) {
        throw std::runtime_error("Failed to create SurfaceFit plane index buffer");
    }
    primitive.indexBuffer->setBuffer(engine, copyForUpload(indices));

    primitive.materialInstance = material->createInstance();
    if (!primitive.materialInstance) {
        throw std::runtime_error("Failed to create SurfaceFit plane material instance");
    }
    const auto bytes = colorToBytes(color);
    const float4 colorValue{
        static_cast<float>(bytes[0]) / 255.0f,
        static_cast<float>(bytes[1]) / 255.0f,
        static_cast<float>(bytes[2]) / 255.0f,
        static_cast<float>(bytes[3]) / 255.0f};
    configureMaterialInstance(
        *primitive.materialInstance,
        pass,
        colorValue,
        depthBias);

    primitive.entity = EntityManager::get().create();
    const float3 minimum{
        std::min({corners[0].x, corners[1].x, corners[2].x, corners[3].x}),
        std::min({corners[0].y, corners[1].y, corners[2].y, corners[3].y}),
        std::min({corners[0].z, corners[1].z, corners[2].z, corners[3].z})};
    const float3 maximum{
        std::max({corners[0].x, corners[1].x, corners[2].x, corners[3].x}),
        std::max({corners[0].y, corners[1].y, corners[2].y, corners[3].y}),
        std::max({corners[0].z, corners[1].z, corners[2].z, corners[3].z})};
    const float3 extent = maximum - minimum;
    const float3 boundCenter = (minimum + maximum) * 0.5f;
    RenderableManager::Builder(1)
        .boundingBox({
            boundCenter,
            {
                std::max(extent.x * 0.5f, 1.0e-5f),
                std::max(extent.y * 0.5f, 1.0e-5f),
                std::max(extent.z * 0.5f, 1.0e-5f)}})
        .material(0, primitive.materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::TRIANGLES,
            primitive.vertexBuffer,
            primitive.indexBuffer)
        .priority(priority)
        .blendOrder(0, blendOrder)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, primitive.entity);
}

} // namespace

struct SurfaceFitPlaneRenderer::Impl {
    Settings settings;
    Statistics statistics;
    Material* material = nullptr;
    PrimitiveResources probe;
    PrimitiveResources success;
    PrimitiveResources failure;

    void ensureMaterial(Engine& engine)
    {
        if (material) {
            return;
        }
        material = Material::Builder()
            .package(
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACEFITPLANE_DATA,
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACEFITPLANE_SIZE)
            .build(engine);
        if (!material) {
            throw std::runtime_error("Failed to create SurfaceFit plane material");
        }
    }

    void destroyPrimitives(Engine& engine, Scene& scene)
    {
        destroyPrimitive(engine, scene, probe);
        destroyPrimitive(engine, scene, success);
        destroyPrimitive(engine, scene, failure);
    }

    void updateMaterialSettings()
    {
        if (success.materialInstance) {
            success.materialInstance->setParameter(
                "BASE_COLOR",
                float4{
                    settings.depthSuccessColor[0],
                    settings.depthSuccessColor[1],
                    settings.depthSuccessColor[2],
                    settings.depthSuccessColor[3]});
            success.materialInstance->setPolygonOffset(0.0f, settings.depthBias);
        }
        if (failure.materialInstance) {
            failure.materialInstance->setParameter(
                "BASE_COLOR",
                float4{
                    settings.depthFailColor[0],
                    settings.depthFailColor[1],
                    settings.depthFailColor[2],
                    settings.depthFailColor[3]});
        }
        if (probe.materialInstance) {
            probe.materialInstance->setPolygonOffset(0.0f, settings.depthBias);
        }
    }
};

SurfaceFitPlaneRenderer::SurfaceFitPlaneRenderer()
    : mImpl(std::make_unique<Impl>())
{
}

SurfaceFitPlaneRenderer::~SurfaceFitPlaneRenderer() = default;

SurfaceFitPlaneRenderer::SurfaceFitPlaneRenderer(SurfaceFitPlaneRenderer&&) noexcept = default;

SurfaceFitPlaneRenderer& SurfaceFitPlaneRenderer::operator=(SurfaceFitPlaneRenderer&&) noexcept = default;

SurfaceFitPlaneRenderer::Settings& SurfaceFitPlaneRenderer::settings() noexcept
{
    return mImpl->settings;
}

const SurfaceFitPlaneRenderer::Settings& SurfaceFitPlaneRenderer::settings() const noexcept
{
    return mImpl->settings;
}

const SurfaceFitPlaneRenderer::Statistics& SurfaceFitPlaneRenderer::statistics() const noexcept
{
    return mImpl->statistics;
}

void SurfaceFitPlaneRenderer::rebuild(
    Engine& engine,
    Scene& scene,
    const Inputs& inputs)
{
    mImpl->destroyPrimitives(engine, scene);
    mImpl->statistics = {};
    if (!inputs.valid ||
        !inputs.centerWorld.isFinite() ||
        !inputs.normalWorld.isFinite() ||
        inputs.normalWorld.lengthSqr() <= 1.0e-20 ||
        !std::isfinite(inputs.displayScale) ||
        inputs.displayScale <= 0.0f) {
        return;
    }

    openvdb::Vec3d normal = inputs.normalWorld;
    normal.normalize();
    const openvdb::Vec3d helper =
        std::abs(normal.x()) < 0.8
        ? openvdb::Vec3d{1.0, 0.0, 0.0}
        : openvdb::Vec3d{0.0, 1.0, 0.0};
    openvdb::Vec3d tangentU = normal.cross(helper);
    if (!tangentU.isFinite() || tangentU.lengthSqr() <= 1.0e-20) {
        return;
    }
    tangentU.normalize();
    openvdb::Vec3d tangentV = normal.cross(tangentU);
    if (!tangentV.isFinite() || tangentV.lengthSqr() <= 1.0e-20) {
        return;
    }
    tangentV.normalize();

    const std::size_t side = inputs.neighborhoodSide == 9
        ? 9
        : inputs.neighborhoodSide == 5
            ? 5
            : 3;
    const double halfIndexExtent = 0.5 * static_cast<double>(side - 1);
    const double voxelX = std::isfinite(inputs.voxelSize.x()) &&
            std::abs(inputs.voxelSize.x()) > 1.0e-12
        ? std::abs(inputs.voxelSize.x())
        : 1.0;
    const double voxelY = std::isfinite(inputs.voxelSize.y()) &&
            std::abs(inputs.voxelSize.y()) > 1.0e-12
        ? std::abs(inputs.voxelSize.y())
        : 1.0;
    const double voxelZ = std::isfinite(inputs.voxelSize.z()) &&
            std::abs(inputs.voxelSize.z()) > 1.0e-12
        ? std::abs(inputs.voxelSize.z())
        : 1.0;
    const double tangentUExtent = halfIndexExtent * (
        std::abs(tangentU.x()) * voxelX +
        std::abs(tangentU.y()) * voxelY +
        std::abs(tangentU.z()) * voxelZ);
    const double tangentVExtent = halfIndexExtent * (
        std::abs(tangentV.x()) * voxelX +
        std::abs(tangentV.y()) * voxelY +
        std::abs(tangentV.z()) * voxelZ);
    const float tangentUHalfExtent = std::max(
        static_cast<float>(tangentUExtent * inputs.displayScale),
        1.0e-6f);
    const float tangentVHalfExtent = std::max(
        static_cast<float>(tangentVExtent * inputs.displayScale),
        1.0e-6f);
    const float3 center = toScene(
        inputs.centerWorld,
        inputs.referenceCenter,
        inputs.displayScale);
    const float3 u = toFloat3(tangentU) * tangentUHalfExtent;
    const float3 v = toFloat3(tangentV) * tangentVHalfExtent;
    const std::array<float3, 4> corners{
        center - u - v,
        center + u - v,
        center + u + v,
        center - u + v};

    mImpl->ensureMaterial(engine);
    createPlanePrimitive(
        engine,
        mImpl->material,
        mImpl->probe,
        corners,
        {1.0f, 1.0f, 1.0f, 0.0f},
        PlanePass::Probe,
        5,
        0,
        mImpl->settings.depthBias);
    createPlanePrimitive(
        engine,
        mImpl->material,
        mImpl->success,
        corners,
        mImpl->settings.depthSuccessColor,
        PlanePass::Success,
        6,
        1,
        mImpl->settings.depthBias);
    createPlanePrimitive(
        engine,
        mImpl->material,
        mImpl->failure,
        corners,
        mImpl->settings.depthFailColor,
        PlanePass::Failure,
        6,
        2,
        0.0f);
    mImpl->statistics.hasPlane = true;
    mImpl->statistics.vertexCount = corners.size();
    mImpl->updateMaterialSettings();
}

void SurfaceFitPlaneRenderer::setVisible(
    Scene& scene,
    bool stageVisible)
{
    const bool visible = stageVisible &&
        mImpl->settings.showPlane &&
        mImpl->statistics.hasPlane;
    setPrimitiveVisible(scene, mImpl->probe, visible);
    setPrimitiveVisible(scene, mImpl->success, visible);
    setPrimitiveVisible(scene, mImpl->failure, visible);
}

void SurfaceFitPlaneRenderer::updateMaterialSettings(Engine&)
{
    mImpl->updateMaterialSettings();
}

void SurfaceFitPlaneRenderer::destroy(
    Engine& engine,
    Scene& scene)
{
    mImpl->destroyPrimitives(engine, scene);
    if (mImpl->material) {
        engine.destroy(mImpl->material);
        mImpl->material = nullptr;
    }
    mImpl->statistics = {};
}

} // namespace volume_surface::viewer
