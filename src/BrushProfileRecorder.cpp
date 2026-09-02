#include "volume_surface/viewer/BrushProfileRecorder.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace volume_surface::viewer {

namespace {

double elapsedMilliseconds(
    const std::chrono::steady_clock::time_point startTime)
{
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - startTime).count();
}

std::string escapeJsonString(const std::string& value)
{
    std::string escaped;
    escaped.reserve(value.size());
    for (const char character : value) {
        switch (character) {
        case '\\': escaped += "\\\\"; break;
        case '"': escaped += "\\\""; break;
        case '\n': escaped += "\\n"; break;
        case '\r': escaped += "\\r"; break;
        case '\t': escaped += "\\t"; break;
        default: escaped += character; break;
        }
    }
    return escaped;
}

void skipWhitespace(const std::string& text, std::size_t& position)
{
    while (position < text.size() &&
           std::strchr(" \t\r\n", text[position]) != nullptr) {
        ++position;
    }
}

bool parseNumber(const std::string& text, std::size_t& position, double& value)
{
    skipWhitespace(text, position);
    if (position >= text.size()) {
        return false;
    }
    const char* begin = text.c_str() + position;
    char* end = nullptr;
    const double parsed = std::strtod(begin, &end);
    if (end == begin || !std::isfinite(parsed)) {
        return false;
    }
    position = static_cast<std::size_t>(end - text.c_str());
    value = parsed;
    return true;
}

bool readNumber(const std::string& json, const char* key, double& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) {
        return false;
    }
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) {
        return false;
    }
    std::size_t position = colonPosition + 1;
    return parseNumber(json, position, value);
}

bool readString(const std::string& json, const char* key, std::string& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) {
        return false;
    }
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) {
        return false;
    }
    const std::size_t openingQuote = json.find('"', colonPosition + 1);
    if (openingQuote == std::string::npos) {
        return false;
    }
    const std::size_t closingQuote = json.find('"', openingQuote + 1);
    if (closingQuote == std::string::npos) {
        return false;
    }
    value = json.substr(openingQuote + 1, closingQuote - openingQuote - 1);
    return true;
}

bool parseNumberArray(
    const std::string& text,
    const char* marker,
    std::vector<double>& values)
{
    const std::size_t markerPosition = text.find(marker);
    if (markerPosition == std::string::npos) {
        return false;
    }
    std::size_t position = markerPosition + std::strlen(marker);
    values.clear();
    while (true) {
        skipWhitespace(text, position);
        if (position >= text.size()) {
            return false;
        }
        if (text[position] == ']') {
            ++position;
            return true;
        }
        double value = 0.0;
        if (!parseNumber(text, position, value)) {
            return false;
        }
        values.push_back(value);
        skipWhitespace(text, position);
        if (position >= text.size() ||
            (text[position] != ',' && text[position] != ']')) {
            return false;
        }
        if (text[position] == ',') {
            ++position;
        }
    }
}

bool parseTupleArray(
    const std::string& text,
    const char* marker,
    std::size_t tupleSize,
    std::vector<std::vector<double>>& tuples)
{
    const std::size_t markerPosition = text.find(marker);
    if (markerPosition == std::string::npos) {
        return false;
    }
    std::size_t position = markerPosition + std::strlen(marker);
    tuples.clear();
    while (true) {
        skipWhitespace(text, position);
        if (position >= text.size()) {
            return false;
        }
        if (text[position] == ']') {
            ++position;
            return true;
        }
        if (text[position++] != '[') {
            return false;
        }
        std::vector<double> tuple;
        tuple.reserve(tupleSize);
        for (std::size_t valueIndex = 0; valueIndex < tupleSize; ++valueIndex) {
            double value = 0.0;
            if (!parseNumber(text, position, value)) {
                return false;
            }
            tuple.push_back(value);
            skipWhitespace(text, position);
            if (valueIndex + 1 < tupleSize) {
                if (position >= text.size() || text[position++] != ',') {
                    return false;
                }
            }
        }
        skipWhitespace(text, position);
        if (position >= text.size() || text[position++] != ']') {
            return false;
        }
        tuples.push_back(std::move(tuple));
        skipWhitespace(text, position);
        if (position >= text.size() ||
            (text[position] != ',' && text[position] != ']')) {
            return false;
        }
        if (text[position] == ',') {
            ++position;
        }
    }
}

} // namespace

void BrushProfileRecorder::begin(
    const std::filesystem::path& input,
    const std::string& gridName,
    const float isoValue,
    const float adaptivity,
    const BrushProfileSettings& settings)
{
    mStroke = {};
    mStroke.active = true;
    mStroke.startTime = std::chrono::steady_clock::now();
    mStroke.input = input;
    mStroke.gridName = gridName;
    mStroke.isoValue = isoValue;
    mStroke.adaptivity = adaptivity;
    mStroke.settings = settings;
}

void BrushProfileRecorder::discard() noexcept
{
    mStroke = {};
}

void BrushProfileRecorder::addPoint(const openvdb::Vec3d& worldPosition)
{
    if (!mStroke.active) {
        return;
    }
    mStroke.points.push_back({elapsedMilliseconds(mStroke.startTime), worldPosition});
}

void BrushProfileRecorder::addFit(
    const std::size_t anchorCount,
    const bool finalFit,
    const SurfaceBrushResult& result,
    const double heatmapMilliseconds,
    const double endToEndMilliseconds,
    const std::size_t heatmapTriangleCount)
{
    if (!mStroke.active) {
        return;
    }
    std::size_t centerlineNodeCount = 0;
    for (const auto& path : result.centerlinePaths) {
        centerlineNodeCount += path.size();
    }
    mStroke.fits.push_back({
        elapsedMilliseconds(mStroke.startTime),
        anchorCount,
        finalFit,
        result.timings,
        heatmapMilliseconds,
        endToEndMilliseconds,
        result.candidateVoxelCount,
        result.candidateLeafCount,
        centerlineNodeCount,
        result.samples.size(),
        heatmapTriangleCount});
}

void BrushProfileRecorder::markLastFitFinal() noexcept
{
    if (!mStroke.fits.empty()) {
        mStroke.fits.back().finalFit = true;
    }
}

bool BrushProfileRecorder::append(
    const std::filesystem::path& path,
    std::string& error)
{
    error.clear();
    if (!mStroke.active || mStroke.points.empty() || mStroke.fits.empty()) {
        discard();
        return false;
    }
    std::ofstream output(path, std::ios::app);
    if (!output) {
        error = "Failed to append " + path.string();
        discard();
        return false;
    }

    const BrushProfileStroke& stroke = mStroke;
    output << std::setprecision(17)
           << "{\"schema\":2,\"type\":\"stroke\",\"input\":\""
           << escapeJsonString(stroke.input.string())
           << "\",\"grid\":\"" << escapeJsonString(stroke.gridName)
           << "\",\"iso\":" << stroke.isoValue
           << ",\"adaptivity\":" << stroke.adaptivity
           << ",\"settings\":["
           << stroke.settings.coreRadiusMillimeters << ','
           << stroke.settings.falloffRadiusMillimeters << ','
           << stroke.settings.strength << ','
           << stroke.settings.planarityRadiusMillimeters << ','
           << stroke.settings.planarityAngleDegrees << ','
           << stroke.settings.planeOffsetPenalty << ','
           << stroke.settings.normalChangePenalty << ','
           << stroke.settings.normalChangeAngleDegrees << ']'
           << ",\"points\":[";
    for (std::size_t pointIndex = 0; pointIndex < stroke.points.size(); ++pointIndex) {
        const BrushProfilePoint& point = stroke.points[pointIndex];
        if (pointIndex != 0) {
            output << ',';
        }
        output << '[' << point.inputMilliseconds << ','
               << point.worldPosition.x() << ','
               << point.worldPosition.y() << ','
               << point.worldPosition.z() << ']';
    }
    output << "],\"fits\":[";
    for (std::size_t fitIndex = 0; fitIndex < stroke.fits.size(); ++fitIndex) {
        const BrushProfileFit& fit = stroke.fits[fitIndex];
        if (fitIndex != 0) {
            output << ',';
        }
        output << '[' << fit.inputMilliseconds << ','
               << fit.anchorCount << ',' << (fit.finalFit ? 1 : 0) << ','
               << fit.coreTimings.hierarchyQueryMilliseconds << ','
               << fit.coreTimings.blockMilliseconds << ','
               << fit.coreTimings.anchorResolveMilliseconds << ','
               << fit.coreTimings.centerlineRouteMilliseconds << ','
               << fit.coreTimings.surfaceSweepMilliseconds << ','
               << fit.coreTimings.totalMilliseconds << ','
               << fit.heatmapMilliseconds << ','
               << fit.endToEndMilliseconds << ','
               << fit.candidateVoxelCount << ','
               << fit.candidateLeafCount << ','
               << fit.centerlineNodeCount << ','
               << fit.sampleCount << ','
               << fit.heatmapTriangleCount << ']';
    }
    output << "]}\n";
    if (!output) {
        error = "Failed while writing " + path.string();
        discard();
        return false;
    }
    discard();
    return true;
}

std::vector<BrushProfileStroke> BrushProfileRecorder::load(
    const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Unable to open brush profile: " + path.string());
    }

    std::vector<BrushProfileStroke> strokes;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty()) {
            continue;
        }
        std::vector<double> settings;
        std::vector<std::vector<double>> points;
        std::vector<std::vector<double>> fits;
        double schema = 0.0;
        if (!readNumber(line, "schema", schema) ||
            (schema != 1.0 && schema != 2.0)) {
            throw std::runtime_error(
                "Unsupported brush profile schema at line " + std::to_string(lineNumber));
        }
        const std::size_t fitTupleSize = schema == 2.0 ? 16 : 14;
        if (!parseNumberArray(line, "\"settings\":[", settings) ||
            settings.size() != 8 ||
            !parseTupleArray(line, "\"points\":[", 4, points) ||
            !parseTupleArray(line, "\"fits\":[", fitTupleSize, fits)) {
            throw std::runtime_error(
                "Invalid brush profile record at line " + std::to_string(lineNumber));
        }

        BrushProfileStroke stroke;
        double isoValue = 0.0;
        double adaptivity = 0.0;
        if (!readString(line, "grid", stroke.gridName) ||
            !readNumber(line, "iso", isoValue) ||
            !readNumber(line, "adaptivity", adaptivity)) {
            throw std::runtime_error(
                "Brush profile is missing mesh metadata at line " +
                std::to_string(lineNumber));
        }
        stroke.isoValue = static_cast<float>(isoValue);
        stroke.adaptivity = static_cast<float>(adaptivity);
        stroke.settings = {
            static_cast<float>(settings[0]),
            static_cast<float>(settings[1]),
            static_cast<float>(settings[2]),
            static_cast<float>(settings[3]),
            static_cast<float>(settings[4]),
            static_cast<float>(settings[5]),
            static_cast<float>(settings[6]),
            static_cast<float>(settings[7])};
        for (const std::vector<double>& point : points) {
            stroke.points.push_back({point[0], {point[1], point[2], point[3]}});
        }
        for (const std::vector<double>& fit : fits) {
            const std::size_t anchorCount = static_cast<std::size_t>(fit[1]);
            if (anchorCount == 0 || anchorCount > stroke.points.size()) {
                throw std::runtime_error(
                    "Brush profile fit has an invalid anchor count at line " +
                    std::to_string(lineNumber));
            }
            BrushProfileFit outputFit;
            outputFit.inputMilliseconds = fit[0];
            outputFit.anchorCount = anchorCount;
            outputFit.finalFit = fit[2] != 0.0;
            const std::size_t timingOffset = schema == 2.0 ? 1 : 0;
            outputFit.coreTimings.hierarchyQueryMilliseconds = schema == 2.0
                ? fit[3]
                : 0.0;
            outputFit.coreTimings.blockMilliseconds = fit[3 + timingOffset];
            outputFit.coreTimings.anchorResolveMilliseconds = fit[4 + timingOffset];
            outputFit.coreTimings.centerlineRouteMilliseconds = fit[5 + timingOffset];
            outputFit.coreTimings.surfaceSweepMilliseconds = fit[6 + timingOffset];
            outputFit.coreTimings.totalMilliseconds = fit[7 + timingOffset];
            outputFit.heatmapMilliseconds = fit[8 + timingOffset];
            outputFit.endToEndMilliseconds = fit[9 + timingOffset];
            outputFit.candidateVoxelCount = static_cast<std::size_t>(fit[10 + timingOffset]);
            outputFit.candidateLeafCount = schema == 2.0
                ? static_cast<std::size_t>(fit[12])
                : 0;
            outputFit.centerlineNodeCount = static_cast<std::size_t>(fit[11 + timingOffset]);
            outputFit.sampleCount = static_cast<std::size_t>(fit[12 + timingOffset]);
            outputFit.heatmapTriangleCount = static_cast<std::size_t>(fit[13 + timingOffset]);
            stroke.fits.push_back(outputFit);
        }
        if (!stroke.points.empty() && !stroke.fits.empty()) {
            strokes.push_back(std::move(stroke));
        }
    }
    if (strokes.empty()) {
        throw std::runtime_error("Brush profile contains no completed strokes: " + path.string());
    }
    return strokes;
}

} // namespace volume_surface::viewer
