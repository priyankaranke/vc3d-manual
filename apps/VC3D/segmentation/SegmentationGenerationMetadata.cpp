#include "SegmentationGenerationMetadata.hpp"

#include "vc/core/util/QuadSurface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <opencv2/core.hpp>

namespace segmentation
{
namespace
{

constexpr int kDefaultGeneration = 1;
constexpr int kMaximumGeneration = std::numeric_limits<uint16_t>::max();

bool validPoint(const cv::Vec3f& point)
{
    return std::isfinite(point[0]) && std::isfinite(point[1]) &&
           std::isfinite(point[2]) && point[0] != -1.0f &&
           point[1] != -1.0f && point[2] != -1.0f;
}

void ensureMetaObject(QuadSurface* surface)
{
    if (surface->meta.is_null() || !surface->meta.is_object()) {
        surface->meta = utils::Json::object();
    }
}

int legacyMetadataGeneration(const QuadSurface& surface)
{
    if (surface.meta.is_null() || !surface.meta.is_object() ||
        !surface.meta.contains("max_gen") ||
        !surface.meta["max_gen"].is_number()) {
        return kDefaultGeneration;
    }

    const double value = surface.meta["max_gen"].get_double();
    if (!std::isfinite(value) || value < 1.0) {
        return kDefaultGeneration;
    }
    return static_cast<int>(std::lround(std::clamp(
        value, 1.0, static_cast<double>(kMaximumGeneration))));
}

} // namespace

std::optional<int> ensureGenerationChannel(QuadSurface* surface)
{
    if (!surface) {
        return std::nullopt;
    }

    const auto* points = surface->rawPointsPtr();
    if (!points || points->empty()) {
        return std::nullopt;
    }

    cv::Mat generations = surface->channel(
        "generations", SURF_CHANNEL_NORESIZE);
    const bool usableChannel = !generations.empty() &&
        generations.type() == CV_16UC1 &&
        generations.rows == points->rows && generations.cols == points->cols;
    if (usableChannel) {
        double maximum = 0.0;
        cv::minMaxLoc(generations, nullptr, &maximum);
        if (maximum >= 1.0) {
            const int currentGeneration = static_cast<int>(std::lround(maximum));
            ensureMetaObject(surface);
            surface->meta["max_gen"] = currentGeneration;
            return currentGeneration;
        }
    }

    const int baseline = legacyMetadataGeneration(*surface);
    cv::Mat_<uint16_t> seeded(
        points->rows, points->cols, static_cast<uint16_t>(0));
    bool hasValidPoint = false;
    for (int row = 0; row < points->rows; ++row) {
        for (int col = 0; col < points->cols; ++col) {
            if (validPoint((*points)(row, col))) {
                seeded(row, col) = static_cast<uint16_t>(baseline);
                hasValidPoint = true;
            }
        }
    }
    if (!hasValidPoint) {
        return std::nullopt;
    }

    surface->setChannel("generations", seeded);
    ensureMetaObject(surface);
    surface->meta["max_gen"] = baseline;
    return baseline;
}

int applyGuiGenerationWindow(
    utils::Json& params,
    int currentGeneration,
    int requestedSteps)
{
    const int start = std::clamp(
        currentGeneration, kDefaultGeneration, kMaximumGeneration);
    const int steps = std::max(0, requestedSteps);
    const long long unboundedTarget =
        static_cast<long long>(start) + static_cast<long long>(steps);
    const int target = static_cast<int>(std::min<long long>(
        unboundedTarget, kMaximumGeneration));

    // These values are written after custom JSON is merged. In particular,
    // rewind_gen must not be allowed to move the tracer's actual start behind
    // the generation used to calculate the GUI's stop value.
    params["rewind_gen"] = -1;
    params["grow_steps"] = steps;
    params["generations"] = target;
    return target;
}

} // namespace segmentation
