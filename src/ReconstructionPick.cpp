#include "volume_surface/viewer/ReconstructionPick.h"

#include <iomanip>
#include <sstream>

namespace volume_surface::viewer {
namespace {

constexpr double kMetersToMillimeters = 1000.0;

void appendVec3(
    std::ostringstream& stream,
    const openvdb::Vec3d& value)
{
    stream << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

void appendVec3Millimeters(
    std::ostringstream& stream,
    const openvdb::Vec3d& value)
{
    stream << '[' << value.x() * kMetersToMillimeters << ','
           << value.y() * kMetersToMillimeters << ','
           << value.z() * kMetersToMillimeters << ']';
}

void appendCoord(
    std::ostringstream& stream,
    const openvdb::Coord& value)
{
    stream << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

std::string jsonString(const std::string& value)
{
    std::ostringstream stream;
    stream << '"';
    for (const char character : value) {
        switch (character) {
        case '\\': stream << "\\\\"; break;
        case '"': stream << "\\\""; break;
        case '\n': stream << "\\n"; break;
        case '\r': stream << "\\r"; break;
        case '\t': stream << "\\t"; break;
        default: stream << character; break;
        }
    }
    stream << '"';
    return stream.str();
}

const char* sampleKindName(SurfaceTargetSampleKind kind)
{
    return kind == SurfaceTargetSampleKind::Core ? "Core" : "Transition";
}

} // namespace

std::string formatReconstructionPickReportJsonl(
    const ReconstructionPickReport& report,
    const std::string& sourcePath,
    const std::string& gridName,
    double isoValue)
{
    std::ostringstream stream;
    stream << std::setprecision(12);
    stream << "{\"stage\":\"source\",\"path\":"
           << jsonString(sourcePath)
           << ",\"grid\":" << jsonString(gridName)
           << ",\"iso\":" << isoValue << "}\n";
    stream << "{\"stage\":\"pick\",\"hit\":"
           << (report.hit ? "true" : "false")
           << ",\"slot\":";
    if (report.hit) {
        stream << report.slotIndex;
    } else {
        stream << "null";
    }
    stream << ",\"triangle\":";
    if (report.hit) {
        stream << report.triangleIndex;
    } else {
        stream << "null";
    }
    stream << ",\"barycentric\":["
           << report.barycentric[0] << ','
           << report.barycentric[1] << ','
           << report.barycentric[2] << "]"
           << ",\"scene_position\":";
    appendVec3(stream, report.scenePosition);
    stream << ",\"mesh_world_mm\":";
    appendVec3Millimeters(stream, report.meshWorldPosition);
    stream << "}\n";

    stream << "{\"stage\":\"vdb_surface\",\"valid\":"
           << (report.vdb.valid ? "true" : "false")
           << ",\"converged\":"
           << (report.vdb.converged ? "true" : "false")
           << ",\"iterations\":" << report.vdb.iterations
           << ",\"world_mm\":";
    appendVec3Millimeters(stream, report.vdb.worldPosition);
    stream << ",\"index\":";
    appendVec3(stream, report.vdb.indexPosition);
    stream << ",\"nearest_voxel\":";
    appendCoord(stream, report.vdb.nearestVoxel);
    stream << ",\"normal\":";
    appendVec3(stream, report.vdb.normal);
    stream << ",\"sampled_value\":" << report.vdb.sampledValue
           << ",\"iso_residual\":" << report.vdb.isoResidual
           << ",\"displacement_mm\":"
           << report.vdb.displacement * kMetersToMillimeters << "}\n";

    stream << "{\"stage\":\"surface_target\",\"found\":"
           << (report.targetSampleFound ? "true" : "false");
    if (report.targetSampleFound) {
        stream << ",\"sample_index\":" << report.targetSampleIndex
               << ",\"distance_mm\":"
               << report.targetSampleDistance * kMetersToMillimeters
               << ",\"coordinate\":";
        appendCoord(stream, report.targetSample.coordinate);
        stream << ",\"kind\":" << jsonString(sampleKindName(report.targetSample.kind))
               << ",\"world_mm\":";
        appendVec3Millimeters(
            stream,
            openvdb::Vec3d(report.targetSample.worldPosition));
        stream << ",\"density\":" << report.targetSample.density
               << ",\"planarity\":" << report.targetSample.planarity
               << ",\"angular_deviation_rad\":"
               << report.targetSample.angularDeviationRadians
               << ",\"support_weight\":" << report.targetSample.supportWeight
               << ",\"transition_layer\":"
               << static_cast<unsigned int>(report.targetSample.transitionLayer)
               << ",\"raw_normal\":";
        appendVec3(stream, openvdb::Vec3d(report.targetSample.normal));
        stream << ",\"smoothed_normal_available\":"
               << (report.smoothedNormalFound ? "true" : "false")
               << ",\"smoothed_normal\":";
        appendVec3(stream, report.smoothedNormal);
        stream << ",\"normal_used_by_reconstruction\":"
               << jsonString(report.normalUsedByReconstruction);
    }
    stream << "}\n";
    return stream.str();
}

} // namespace volume_surface::viewer
