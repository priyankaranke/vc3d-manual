#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <utility>

namespace segmentation {

// Ancillary rasters may cover cells (N*k) or vertices ((N-1)*k+1).
inline std::optional<std::pair<int,bool>> pointGrowChannelScale(int pixels, int nodes)
{
    if (nodes > 0 && pixels % nodes == 0 && pixels / nodes > 0)
        return {{pixels / nodes, false}};
    if (nodes > 1 && (pixels-1) % (nodes-1) == 0 && (pixels-1) / (nodes-1) > 0)
        return {{(pixels-1) / (nodes-1), true}};
    return std::nullopt;
}

inline cv::Mat padPointGrowChannel(const cv::Mat& source, cv::Size originalGrid,
                                  cv::Size paddedGrid, cv::Point offset)
{
    if (source.empty()) return {};
    const auto sx = pointGrowChannelScale(source.cols, originalGrid.width);
    const auto sy = pointGrowChannelScale(source.rows, originalGrid.height);
    if (!sx || !sy) {
        // Imported/flattened segments can retain a last_growth TIFF from an
        // older grid. It is already unusable as a live overlay (that reader
        // requires matching dimensions), but it must not veto XYZ editing.
        // Preserve its size and every pixel; never invent an alignment by
        // resizing, cropping, or replacing the historical raster.
        return source.clone();
    }
    const int cols = (paddedGrid.width - (sx->second ? 1 : 0))*sx->first + (sx->second ? 1 : 0);
    const int rows = (paddedGrid.height - (sy->second ? 1 : 0))*sy->first + (sy->second ? 1 : 0);
    cv::Mat padded = cv::Mat::zeros(rows, cols, source.type());
    source.copyTo(padded(cv::Rect(offset.x*sx->first, offset.y*sy->first, source.cols, source.rows)));
    return padded;
}

} // namespace segmentation
