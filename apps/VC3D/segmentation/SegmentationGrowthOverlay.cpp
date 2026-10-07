#include "SegmentationGrowthOverlay.hpp"

#include "vc/core/util/QuadSurface.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace segmentation
{
namespace
{

bool validPoint(const cv::Vec3f& point)
{
    return std::isfinite(point[0]) && std::isfinite(point[1]) &&
           std::isfinite(point[2]) && point[0] != -1.0f &&
           point[1] != -1.0f && point[2] != -1.0f;
}

bool validGenerationChannel(
    const cv::Mat& generations,
    const cv::Mat_<cv::Vec3f>& points)
{
    return !generations.empty() && generations.channels() == 1 &&
           generations.rows == points.rows && generations.cols == points.cols;
}

std::optional<cv::Mat_<uint8_t>> generationGrowthMask(
    const QuadSurface& before,
    const QuadSurface& after,
    const cv::Mat_<cv::Vec3f>& beforePoints,
    const cv::Mat_<cv::Vec3f>& afterPoints)
{
    cv::Mat beforeGenerations = const_cast<QuadSurface&>(before).channel(
        "generations", SURF_CHANNEL_NORESIZE);
    cv::Mat afterGenerations = const_cast<QuadSurface&>(after).channel(
        "generations", SURF_CHANNEL_NORESIZE);
    if (!validGenerationChannel(beforeGenerations, beforePoints) ||
        !validGenerationChannel(afterGenerations, afterPoints)) {
        return std::nullopt;
    }

    double previousGeneration = 0.0;
    cv::minMaxLoc(beforeGenerations, nullptr, &previousGeneration);

    cv::Mat newer;
    cv::compare(afterGenerations, cv::Scalar(previousGeneration), newer, cv::CMP_GT);
    cv::Mat_<uint8_t> mask(
        afterPoints.rows, afterPoints.cols, static_cast<uint8_t>(0));
    for (int row = 0; row < afterPoints.rows; ++row) {
        const uint8_t* newerRow = newer.ptr<uint8_t>(row);
        for (int col = 0; col < afterPoints.cols; ++col) {
            if (newerRow[col] != 0 && validPoint(afterPoints(row, col))) {
                mask(row, col) = 255;
            }
        }
    }
    return mask;
}

std::optional<cv::Point> explicitGridOffset(const QuadSurface& surface)
{
    if (surface.meta.is_null() || !surface.meta.is_object() ||
        !surface.meta.contains("grid_offset") ||
        !surface.meta["grid_offset"].is_array() ||
        surface.meta["grid_offset"].size() < 2 ||
        !surface.meta["grid_offset"][0].is_number() ||
        !surface.meta["grid_offset"][1].is_number()) {
        return std::nullopt;
    }

    return cv::Point(
        static_cast<int>(std::lround(surface.meta["grid_offset"][0].get_double())),
        static_cast<int>(std::lround(surface.meta["grid_offset"][1].get_double())));
}

std::optional<cv::Point> mapAfterToBefore(
    int row,
    int col,
    const QuadSurface& before,
    const QuadSurface& after,
    const std::optional<cv::Point>& offset)
{
    if (offset) {
        return cv::Point(col - offset->x, row - offset->y);
    }

    const cv::Vec2f beforeScale = before.scale();
    const cv::Vec2f afterScale = after.scale();
    if (std::abs(beforeScale[0]) < 1e-6f ||
        std::abs(beforeScale[1]) < 1e-6f ||
        std::abs(afterScale[0]) < 1e-6f ||
        std::abs(afterScale[1]) < 1e-6f) {
        return std::nullopt;
    }

    const cv::Vec3f beforeCenter = before.center();
    const cv::Vec3f afterCenter = after.center();
    const float surfaceX = static_cast<float>(col) / afterScale[0] - afterCenter[0];
    const float surfaceY = static_cast<float>(row) / afterScale[1] - afterCenter[1];
    const int beforeCol = static_cast<int>(std::lround(
        (surfaceX + beforeCenter[0]) * beforeScale[0]));
    const int beforeRow = static_cast<int>(std::lround(
        (surfaceY + beforeCenter[1]) * beforeScale[1]));
    return cv::Point(beforeCol, beforeRow);
}

std::size_t alignmentScore(
    const cv::Mat_<cv::Vec3f>& beforePoints,
    const cv::Mat_<cv::Vec3f>& afterPoints,
    const QuadSurface& before,
    const QuadSurface& after,
    const std::optional<cv::Point>& offset)
{
    constexpr std::size_t kMaxSamples = 100000;
    const std::size_t total = static_cast<std::size_t>(afterPoints.rows) *
                              static_cast<std::size_t>(afterPoints.cols);
    const int stride = std::max(1, static_cast<int>(std::ceil(
        std::sqrt(static_cast<double>(total) / static_cast<double>(kMaxSamples)))));

    std::size_t matches = 0;
    for (int row = 0; row < afterPoints.rows; row += stride) {
        for (int col = 0; col < afterPoints.cols; col += stride) {
            if (!validPoint(afterPoints(row, col))) {
                continue;
            }
            const auto beforeGrid = mapAfterToBefore(
                row, col, before, after, offset);
            if (beforeGrid && beforeGrid->x >= 0 &&
                beforeGrid->x < beforePoints.cols &&
                beforeGrid->y >= 0 && beforeGrid->y < beforePoints.rows &&
                validPoint(beforePoints(beforeGrid->y, beforeGrid->x))) {
                ++matches;
            }
        }
    }
    return matches;
}

std::optional<cv::Point> selectedAlignmentOffset(
    const QuadSurface& before,
    const QuadSurface& after,
    const cv::Mat_<cv::Vec3f>& beforePoints,
    const cv::Mat_<cv::Vec3f>& afterPoints)
{
    auto offset = explicitGridOffset(after);
    if (offset) {
        const std::size_t explicitScore = alignmentScore(
            beforePoints, afterPoints, before, after, offset);
        const std::size_t parameterScore = alignmentScore(
            beforePoints, afterPoints, before, after, std::nullopt);
        if (parameterScore > explicitScore) {
            // Dense-displacement outputs and hand-authored segments can retain
            // stale grid_offset metadata. Prefer the alignment that preserves
            // more of the known pre-grow region.
            offset.reset();
        }
    }
    return offset;
}

float medianGridStep(const cv::Mat_<cv::Vec3f>& points)
{
    constexpr std::size_t kMaxSamples = 100000;
    const std::size_t total = static_cast<std::size_t>(points.rows) *
                              static_cast<std::size_t>(points.cols);
    const int stride = std::max(1, static_cast<int>(std::ceil(
        std::sqrt(static_cast<double>(total) / static_cast<double>(kMaxSamples)))));
    std::vector<float> lengths;
    lengths.reserve(std::min(total * 2, kMaxSamples * 2));
    for (int row = 0; row < points.rows; row += stride) {
        for (int col = 0; col < points.cols; col += stride) {
            if (!validPoint(points(row, col))) {
                continue;
            }
            if (col + 1 < points.cols && validPoint(points(row, col + 1))) {
                const float length = cv::norm(points(row, col + 1) - points(row, col));
                if (std::isfinite(length) && length > 1.0e-5f) {
                    lengths.push_back(length);
                }
            }
            if (row + 1 < points.rows && validPoint(points(row + 1, col))) {
                const float length = cv::norm(points(row + 1, col) - points(row, col));
                if (std::isfinite(length) && length > 1.0e-5f) {
                    lengths.push_back(length);
                }
            }
        }
    }
    if (lengths.empty()) {
        return 1.0f;
    }
    const std::size_t middle = lengths.size() / 2;
    std::nth_element(lengths.begin(),
                     lengths.begin() + static_cast<std::ptrdiff_t>(middle),
                     lengths.end());
    return std::max(lengths[middle], 1.0e-5f);
}

} // namespace

cv::Mat_<uint8_t> computeLastGrowthMask(
    const QuadSurface& before,
    const QuadSurface& after)
{
    const auto* beforePoints = before.rawPointsPtr();
    const auto* afterPoints = after.rawPointsPtr();
    if (!beforePoints || beforePoints->empty() ||
        !afterPoints || afterPoints->empty()) {
        return {};
    }

    cv::Mat_<uint8_t> mask(
        afterPoints->rows, afterPoints->cols, static_cast<uint8_t>(0));

    // A resize/crop necessarily changes grid coordinates, so inferred geometry
    // can mistake the tracer's transparent border for growth. Generation IDs
    // record exactly which vertices were created by this invocation and need no
    // before/after registration.
    if (auto generationMask = generationGrowthMask(
            before, after, *beforePoints, *afterPoints)) {
        return *generationMask;
    }

    const auto offset = selectedAlignmentOffset(
        before, after, *beforePoints, *afterPoints);

    for (int row = 0; row < afterPoints->rows; ++row) {
        for (int col = 0; col < afterPoints->cols; ++col) {
            if (!validPoint((*afterPoints)(row, col))) {
                continue;
            }

            const auto beforeGrid = mapAfterToBefore(
                row, col, before, after, offset);
            const bool existedBefore = beforeGrid &&
                beforeGrid->x >= 0 && beforeGrid->x < beforePoints->cols &&
                beforeGrid->y >= 0 && beforeGrid->y < beforePoints->rows &&
                validPoint((*beforePoints)(beforeGrid->y, beforeGrid->x));
            if (!existedBefore) {
                mask(row, col) = 255;
            }
        }
    }

    return mask;
}

std::size_t preserveExistingGrowthGeometry(
    const QuadSurface& before,
    QuadSurface& after)
{
    const auto* beforePoints = before.rawPointsPtr();
    auto* afterPoints = after.rawPointsPtr();
    if (!beforePoints || beforePoints->empty() ||
        !afterPoints || afterPoints->empty()) {
        return 0;
    }

    const auto offset = selectedAlignmentOffset(
        before, after, *beforePoints, *afterPoints);
    std::size_t restored = 0;
    for (int row = 0; row < afterPoints->rows; ++row) {
        for (int col = 0; col < afterPoints->cols; ++col) {
            const auto beforeGrid = mapAfterToBefore(
                row, col, before, after, offset);
            if (!beforeGrid || beforeGrid->x < 0 ||
                beforeGrid->x >= beforePoints->cols || beforeGrid->y < 0 ||
                beforeGrid->y >= beforePoints->rows) {
                continue;
            }

            const cv::Vec3f& original =
                (*beforePoints)(beforeGrid->y, beforeGrid->x);
            if (!validPoint(original)) {
                continue;
            }

            cv::Vec3f& candidate = (*afterPoints)(row, col);
            if (candidate[0] != original[0] ||
                candidate[1] != original[1] ||
                candidate[2] != original[2]) {
                ++restored;
            }
            candidate = original;
        }
    }

    if (restored > 0) {
        after.invalidateCache();
    }
    return restored;
}

cv::Mat_<uint8_t> computeLastGrowthCellMask(
    const QuadSurface& surface,
    const cv::Mat& vertexMask)
{
    const auto* points = surface.rawPointsPtr();
    if (!points || points->empty() || vertexMask.empty() ||
        vertexMask.rows != points->rows || vertexMask.cols != points->cols) {
        return {};
    }

    std::vector<cv::Mat> channels;
    cv::split(vertexMask, channels);
    cv::Mat binary(vertexMask.rows, vertexMask.cols, CV_8UC1, cv::Scalar(0));
    for (const cv::Mat& channel : channels) {
        cv::Mat nonzero;
        cv::compare(channel, 0, nonzero, cv::CMP_NE);
        cv::bitwise_or(binary, nonzero, binary);
    }

    cv::Mat_<uint8_t> cells(
        points->rows, points->cols, static_cast<uint8_t>(0));
    for (int row = 0; row + 1 < points->rows; ++row) {
        const uint8_t* top = binary.ptr<uint8_t>(row);
        const uint8_t* bottom = binary.ptr<uint8_t>(row + 1);
        for (int col = 0; col + 1 < points->cols; ++col) {
            const bool containsNewVertex = top[col] != 0 || top[col + 1] != 0 ||
                                           bottom[col] != 0 || bottom[col + 1] != 0;
            if (containsNewVertex && validPoint((*points)(row, col)) &&
                validPoint((*points)(row, col + 1)) &&
                validPoint((*points)(row + 1, col)) &&
                validPoint((*points)(row + 1, col + 1))) {
                cells(row, col) = 255;
            }
        }
    }
    return cells;
}

cv::Mat_<uint8_t> computeGrowthDisplacementMask(
    const QuadSurface& before,
    const QuadSurface& after,
    float relativeThreshold)
{
    const auto* beforePoints = before.rawPointsPtr();
    const auto* afterPoints = after.rawPointsPtr();
    if (!beforePoints || beforePoints->empty() ||
        !afterPoints || afterPoints->empty()) {
        return {};
    }

    cv::Mat_<uint8_t> mask(
        afterPoints->rows, afterPoints->cols, static_cast<uint8_t>(0));
    const auto offset = selectedAlignmentOffset(
        before, after, *beforePoints, *afterPoints);
    const float threshold = std::max(
        0.5f, medianGridStep(*beforePoints) * std::max(0.0f, relativeThreshold));

    for (int row = 0; row < afterPoints->rows; ++row) {
        for (int col = 0; col < afterPoints->cols; ++col) {
            if (!validPoint((*afterPoints)(row, col))) {
                continue;
            }
            const auto beforeGrid = mapAfterToBefore(
                row, col, before, after, offset);
            if (!beforeGrid || beforeGrid->x < 0 ||
                beforeGrid->x >= beforePoints->cols || beforeGrid->y < 0 ||
                beforeGrid->y >= beforePoints->rows ||
                !validPoint((*beforePoints)(beforeGrid->y, beforeGrid->x))) {
                continue;
            }
            if (cv::norm((*afterPoints)(row, col) -
                         (*beforePoints)(beforeGrid->y, beforeGrid->x)) > threshold) {
                mask(row, col) = 255;
            }
        }
    }
    return mask;
}

} // namespace segmentation
