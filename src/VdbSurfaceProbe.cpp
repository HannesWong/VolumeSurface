#include "volume_surface/VdbSurfaceProbe.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <openvdb/tools/Interpolation.h>

namespace volume_surface {
namespace {

constexpr double kEpsilon = 1.0e-20;

struct SampledField
{
    double value = std::numeric_limits<double>::quiet_NaN();
    openvdb::Vec3d gradient{};
    bool valid = false;
};

double minimumVoxelSize(const openvdb::Vec3d& voxelSize)
{
    return std::min({
        std::abs(voxelSize.x()),
        std::abs(voxelSize.y()),
        std::abs(voxelSize.z())});
}

double maximumVoxelSize(const openvdb::Vec3d& voxelSize)
{
    return std::max({
        std::abs(voxelSize.x()),
        std::abs(voxelSize.y()),
        std::abs(voxelSize.z())});
}

SampledField sampleField(
    const openvdb::tools::GridSampler<
        openvdb::FloatGrid,
        openvdb::tools::BoxSampler>& sampler,
    const openvdb::Vec3d& position,
    const openvdb::Vec3d& step)
{
    SampledField result;
    result.value = sampler.wsSample(position);
    if (!std::isfinite(result.value)) {
        return result;
    }
    result.gradient = openvdb::Vec3d{
        (sampler.wsSample(position + openvdb::Vec3d(step.x(), 0.0, 0.0)) -
            sampler.wsSample(position - openvdb::Vec3d(step.x(), 0.0, 0.0))) /
            (2.0 * step.x()),
        (sampler.wsSample(position + openvdb::Vec3d(0.0, step.y(), 0.0)) -
            sampler.wsSample(position - openvdb::Vec3d(0.0, step.y(), 0.0))) /
            (2.0 * step.y()),
        (sampler.wsSample(position + openvdb::Vec3d(0.0, 0.0, step.z())) -
            sampler.wsSample(position - openvdb::Vec3d(0.0, 0.0, step.z()))) /
            (2.0 * step.z())};
    result.valid = result.gradient.isFinite() &&
        result.gradient.lengthSqr() > kEpsilon;
    return result;
}

double residualAt(
    const openvdb::tools::GridSampler<
        openvdb::FloatGrid,
        openvdb::tools::BoxSampler>& sampler,
    const openvdb::Vec3d& position,
    double isoValue)
{
    const double value = sampler.wsSample(position);
    return std::isfinite(value) ? value - isoValue
                                : std::numeric_limits<double>::infinity();
}

openvdb::Vec3d outwardNormal(
    const openvdb::FloatGrid& grid,
    openvdb::Vec3d gradient)
{
    if (grid.getGridClass() == openvdb::GRID_FOG_VOLUME) {
        gradient = -gradient;
    }
    const double length = gradient.length();
    if (!std::isfinite(length) || length <= kEpsilon) {
        return openvdb::Vec3d(0.0);
    }
    return gradient / length;
}

} // namespace

VdbSurfaceProbeResult projectVdbSurface(
    const openvdb::FloatGrid& grid,
    const openvdb::Vec3d& initialWorldPosition,
    const VdbSurfaceProbeSettings& settings)
{
    if (grid.getGridClass() != openvdb::GRID_FOG_VOLUME &&
        grid.getGridClass() != openvdb::GRID_LEVEL_SET) {
        throw std::invalid_argument("VDB surface probe requires a fog volume or level set grid");
    }
    if (!initialWorldPosition.isFinite() || !std::isfinite(settings.isoValue) ||
        settings.maximumIterations == 0 ||
        !std::isfinite(settings.valueTolerance) || settings.valueTolerance <= 0.0 ||
        !std::isfinite(settings.maximumStep) || settings.maximumStep < 0.0) {
        throw std::invalid_argument("VDB surface probe settings are invalid");
    }

    const openvdb::Vec3d voxelSize = grid.voxelSize();
    const double minimumSpacing = minimumVoxelSize(voxelSize);
    const double maximumSpacing = maximumVoxelSize(voxelSize);
    if (!std::isfinite(minimumSpacing) || minimumSpacing <= 0.0 ||
        !std::isfinite(maximumSpacing) || maximumSpacing <= 0.0) {
        throw std::invalid_argument("VDB surface probe requires a valid voxel size");
    }
    const openvdb::Vec3d gradientStep{
        std::max(std::abs(voxelSize.x()) * 0.5, 1.0e-9),
        std::max(std::abs(voxelSize.y()) * 0.5, 1.0e-9),
        std::max(std::abs(voxelSize.z()) * 0.5, 1.0e-9)};
    const double maximumStep = settings.maximumStep > 0.0
        ? settings.maximumStep
        : maximumSpacing * 2.0;

    openvdb::tools::GridSampler<
        openvdb::FloatGrid,
        openvdb::tools::BoxSampler> sampler(grid);
    VdbSurfaceProbeResult result;
    result.worldPosition = initialWorldPosition;
    const double initialResidual = residualAt(
        sampler,
        initialWorldPosition,
        settings.isoValue);
    result.sampledValue = initialResidual + settings.isoValue;
    result.isoResidual = initialResidual;
    result.indexPosition = grid.worldToIndex(initialWorldPosition);
    result.nearestVoxel = openvdb::Coord::round(result.indexPosition);
    openvdb::Vec3d position = initialWorldPosition;
    double residual = initialResidual;
    for (std::size_t iteration = 0;
         iteration < settings.maximumIterations;
         ++iteration) {
        const auto field = sampleField(sampler, position, gradientStep);
        if (!std::isfinite(field.value)) {
            break;
        }
        residual = field.value - settings.isoValue;
        result.iterations = iteration + 1;
        result.sampledValue = field.value;
        result.isoResidual = residual;
        if (std::abs(residual) <= settings.valueTolerance) {
            result.converged = true;
            result.normal = outwardNormal(grid, field.gradient);
            break;
        }
        if (!field.valid) {
            break;
        }

        const double gradientLengthSquared = field.gradient.lengthSqr();
        openvdb::Vec3d step = field.gradient * (residual / gradientLengthSquared);
        const double stepLength = step.length();
        if (!std::isfinite(stepLength) || stepLength <= kEpsilon) {
            break;
        }
        if (stepLength > maximumStep) {
            step *= maximumStep / stepLength;
        }
        openvdb::Vec3d candidate = position - step;
        double candidateResidual = residualAt(sampler, candidate, settings.isoValue);
        for (int lineSearch = 0;
             lineSearch < 4 && std::abs(candidateResidual) > std::abs(residual);
             ++lineSearch) {
            step *= 0.5;
            candidate = position - step;
            candidateResidual = residualAt(sampler, candidate, settings.isoValue);
        }
        if (!std::isfinite(candidateResidual)) {
            break;
        }
        position = candidate;
        residual = candidateResidual;
        result.normal = outwardNormal(grid, field.gradient);
    }

    if (!result.normal.isFinite() || result.normal.lengthSqr() <= kEpsilon) {
        const auto field = sampleField(sampler, position, gradientStep);
        if (field.valid) {
            result.normal = outwardNormal(grid, field.gradient);
        }
    }
    result.worldPosition = position;
    result.indexPosition = grid.worldToIndex(position);
    result.nearestVoxel = openvdb::Coord::round(result.indexPosition);
    result.displacement = (position - initialWorldPosition).length();
    const bool finiteProjection = position.isFinite() && result.indexPosition.isFinite() &&
        std::isfinite(result.sampledValue) && std::isfinite(result.isoResidual);
    result.valid = finiteProjection && result.converged;
    if (!result.valid) {
        result.worldPosition = initialWorldPosition;
        result.indexPosition = grid.worldToIndex(initialWorldPosition);
        result.nearestVoxel = openvdb::Coord::round(result.indexPosition);
        result.sampledValue = initialResidual + settings.isoValue;
        result.isoResidual = initialResidual;
        result.displacement = 0.0;
    }
    return result;
}

} // namespace volume_surface
