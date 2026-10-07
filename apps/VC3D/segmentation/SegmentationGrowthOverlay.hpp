#pragma once

#include <cstddef>
#include <cstdint>

#include <opencv2/core/mat.hpp>

class QuadSurface;

namespace segmentation
{

inline constexpr const char* kLastGrowthChannel = "last_growth";

/**
 * Build a grid-sized vertex mask for the post-grow surface. A non-zero pixel
 * means that the corresponding post-grow vertex was created by this grow.
 * Tracer generation channels are authoritative when available; aligned
 * before/after geometry is used as a fallback for older growth outputs.
 */
[[nodiscard]] cv::Mat_<uint8_t> computeLastGrowthMask(
    const QuadSurface& before,
    const QuadSurface& after);

/**
 * Enforce append-only growth by copying every aligned, previously valid
 * vertex back into the post-grow surface. Returns the number of candidate
 * coordinates that did not already match the pre-grow geometry exactly.
 */
[[nodiscard]] std::size_t preserveExistingGrowthGeometry(
    const QuadSurface& before,
    QuadSurface& after);

/**
 * Convert a last-growth vertex mask to the surface cells it actually created.
 * Mask pixels are rendered as cells whose top-left corner is the matching grid
 * coordinate, so this conversion keeps Down/Right growth on the new surface
 * instead of painting the transparent padding beyond its newest vertices.
 */
[[nodiscard]] cv::Mat_<uint8_t> computeLastGrowthCellMask(
    const QuadSurface& surface,
    const cv::Mat& vertexMask);

/**
 * Mark aligned points that were already valid but moved substantially during
 * growth. This is kept separate from the new-area mask so warning detection
 * can cover dense/displacement growth without changing the cyan overlay's
 * meaning.
 */
[[nodiscard]] cv::Mat_<uint8_t> computeGrowthDisplacementMask(
    const QuadSurface& before,
    const QuadSurface& after,
    float relativeThreshold = 0.75f);

} // namespace segmentation
