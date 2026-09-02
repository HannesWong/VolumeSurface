#include "volume_surface/SliceDiagnostics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace volume_surface {
namespace {

struct PlaneAxes {
    int fixed;
    int horizontal;
    int vertical;
};

PlaneAxes axesFor(SliceAxis axis)
{
    switch (axis) {
        case SliceAxis::X:
            return {0, 1, 2};
        case SliceAxis::Y:
            return {1, 0, 2};
        case SliceAxis::Z:
            return {2, 0, 1};
    }
    throw std::invalid_argument("Unknown slice axis");
}

openvdb::Coord makeCoordinate(
    const PlaneAxes& axes,
    int fixed,
    int horizontal,
    int vertical)
{
    openvdb::Coord coordinate;
    coordinate[axes.fixed] = fixed;
    coordinate[axes.horizontal] = horizontal;
    coordinate[axes.vertical] = vertical;
    return coordinate;
}

bool crosses(float a, float b, float level)
{
    return (a < level && b >= level) || (b < level && a >= level);
}

std::array<float, 2> interpolate(
    float ax,
    float ay,
    float av,
    float bx,
    float by,
    float bv,
    float level,
    std::uint32_t width,
    std::uint32_t height)
{
    const float denominator = bv - av;
    const float t = std::abs(denominator) > 1.0e-12f
        ? std::clamp((level - av) / denominator, 0.0f, 1.0f)
        : 0.5f;
    const float x = ax + (bx - ax) * t;
    const float y = ay + (by - ay) * t;
    return {
        width > 1 ? x / static_cast<float>(width - 1) : 0.0f,
        height > 1 ? 1.0f - y / static_cast<float>(height - 1) : 0.0f};
}

void appendContours(DensitySlice& slice, float level)
{
    if (!std::isfinite(level) || slice.width < 2 || slice.height < 2) {
        return;
    }

    for (std::uint32_t y = 0; y + 1 < slice.height; ++y) {
        for (std::uint32_t x = 0; x + 1 < slice.width; ++x) {
            const auto valueAt = [&](std::uint32_t px, std::uint32_t py) {
                return slice.values[static_cast<std::size_t>(py) * slice.width + px];
            };

            const float v0 = valueAt(x, y);
            const float v1 = valueAt(x + 1, y);
            const float v2 = valueAt(x + 1, y + 1);
            const float v3 = valueAt(x, y + 1);
            std::array<std::array<float, 2>, 4> intersections{};
            int count = 0;

            if (crosses(v0, v1, level)) {
                intersections[count++] = interpolate(
                    static_cast<float>(x), static_cast<float>(y), v0,
                    static_cast<float>(x + 1), static_cast<float>(y), v1,
                    level, slice.width, slice.height);
            }
            if (crosses(v1, v2, level)) {
                intersections[count++] = interpolate(
                    static_cast<float>(x + 1), static_cast<float>(y), v1,
                    static_cast<float>(x + 1), static_cast<float>(y + 1), v2,
                    level, slice.width, slice.height);
            }
            if (crosses(v2, v3, level)) {
                intersections[count++] = interpolate(
                    static_cast<float>(x + 1), static_cast<float>(y + 1), v2,
                    static_cast<float>(x), static_cast<float>(y + 1), v3,
                    level, slice.width, slice.height);
            }
            if (crosses(v3, v0, level)) {
                intersections[count++] = interpolate(
                    static_cast<float>(x), static_cast<float>(y + 1), v3,
                    static_cast<float>(x), static_cast<float>(y), v0,
                    level, slice.width, slice.height);
            }

            if (count == 2) {
                slice.contours.push_back({intersections[0], intersections[1], level});
            } else if (count == 4) {
                const float center = (v0 + v1 + v2 + v3) * 0.25f;
                if (center >= level) {
                    slice.contours.push_back({intersections[0], intersections[3], level});
                    slice.contours.push_back({intersections[1], intersections[2], level});
                } else {
                    slice.contours.push_back({intersections[0], intersections[1], level});
                    slice.contours.push_back({intersections[2], intersections[3], level});
                }
            }
        }
    }
}

} // namespace

DensitySlice extractDensitySlice(
    const openvdb::FloatGrid& grid,
    const openvdb::CoordBBox& bounds,
    SliceAxis fixedAxis,
    int fixedIndex,
    float displayMaximum,
    std::span<const float> contourLevels)
{
    if (bounds.empty()) {
        throw std::invalid_argument("Slice bounds must not be empty");
    }
    if (!std::isfinite(displayMaximum) || displayMaximum <= 0.0f) {
        throw std::invalid_argument("displayMaximum must be positive and finite");
    }

    const PlaneAxes axes = axesFor(fixedAxis);
    if (fixedIndex < bounds.min()[axes.fixed] || fixedIndex > bounds.max()[axes.fixed]) {
        throw std::out_of_range("Slice index is outside the supplied bounds");
    }

    const std::int64_t width =
        static_cast<std::int64_t>(bounds.max()[axes.horizontal])
        - bounds.min()[axes.horizontal] + 1;
    const std::int64_t height =
        static_cast<std::int64_t>(bounds.max()[axes.vertical])
        - bounds.min()[axes.vertical] + 1;
    constexpr std::int64_t maximumPixelCount = 16 * 1024 * 1024;
    if (width <= 0 || height <= 0 || width * height > maximumPixelCount) {
        throw std::length_error("Slice exceeds the 16-megapixel diagnostic limit");
    }

    DensitySlice slice;
    slice.fixedAxis = fixedAxis;
    slice.fixedIndex = fixedIndex;
    slice.horizontalAxis = axes.horizontal;
    slice.verticalAxis = axes.vertical;
    slice.horizontalMinimum = bounds.min()[axes.horizontal];
    slice.verticalMinimum = bounds.min()[axes.vertical];
    slice.width = static_cast<std::uint32_t>(width);
    slice.height = static_cast<std::uint32_t>(height);
    const auto spacing = grid.voxelSize();
    slice.horizontalSpacing = spacing[axes.horizontal];
    slice.verticalSpacing = spacing[axes.vertical];
    slice.minimumValue = std::numeric_limits<float>::infinity();
    slice.maximumValue = -std::numeric_limits<float>::infinity();
    slice.values.resize(static_cast<std::size_t>(width * height));
    slice.rgba.resize(static_cast<std::size_t>(width * height) * 4);

    const auto accessor = grid.getConstAccessor();
    for (std::uint32_t y = 0; y < slice.height; ++y) {
        for (std::uint32_t x = 0; x < slice.width; ++x) {
            const int horizontal = slice.horizontalMinimum + static_cast<int>(x);
            const int vertical = slice.verticalMinimum + static_cast<int>(y);
            const float value = accessor.getValue(
                makeCoordinate(axes, fixedIndex, horizontal, vertical));
            const std::size_t valueOffset = static_cast<std::size_t>(y) * slice.width + x;
            slice.values[valueOffset] = value;
            slice.minimumValue = std::min(slice.minimumValue, value);
            slice.maximumValue = std::max(slice.maximumValue, value);

            const std::uint32_t displayY = slice.height - 1 - y;
            const std::size_t pixelOffset =
                (static_cast<std::size_t>(displayY) * slice.width + x) * 4;
            const auto intensity = static_cast<std::uint8_t>(std::lround(
                std::clamp(value / displayMaximum, 0.0f, 1.0f) * 255.0f));
            slice.rgba[pixelOffset] = intensity;
            slice.rgba[pixelOffset + 1] = intensity;
            slice.rgba[pixelOffset + 2] = intensity;
            slice.rgba[pixelOffset + 3] = 255;
        }
    }

    for (const float level : contourLevels) {
        appendContours(slice, level);
    }
    return slice;
}

} // namespace volume_surface
