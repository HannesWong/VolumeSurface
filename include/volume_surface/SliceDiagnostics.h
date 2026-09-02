#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <openvdb/openvdb.h>

namespace volume_surface {

enum class SliceAxis : std::uint8_t {
    X,
    Y,
    Z
};

struct SliceContourSegment {
    std::array<float, 2> start{};
    std::array<float, 2> end{};
    float level = 0.0f;
};

struct DensitySlice {
    SliceAxis fixedAxis = SliceAxis::Z;
    int fixedIndex = 0;
    int horizontalAxis = 0;
    int verticalAxis = 1;
    int horizontalMinimum = 0;
    int verticalMinimum = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    double horizontalSpacing = 1.0;
    double verticalSpacing = 1.0;
    float minimumValue = 0.0f;
    float maximumValue = 0.0f;
    std::vector<float> values;
    std::vector<std::uint8_t> rgba;
    std::vector<SliceContourSegment> contours;
};

DensitySlice extractDensitySlice(
    const openvdb::FloatGrid& grid,
    const openvdb::CoordBBox& bounds,
    SliceAxis fixedAxis,
    int fixedIndex,
    float displayMaximum,
    std::span<const float> contourLevels);

} // namespace volume_surface
