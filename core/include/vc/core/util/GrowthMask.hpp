#pragma once

#include <opencv2/core/mat.hpp>

#include <algorithm>
#include <cstdint>

namespace vc::core::util
{

inline cv::Size scaledGridSize(const cv::Size& size, int factor)
{
    factor = std::max(1, factor);
    return cv::Size(
        std::max(1, (size.width + factor - 1) / factor),
        std::max(1, (size.height + factor - 1) / factor));
}

inline cv::Rect flaggedGrowthBounds(const cv::Mat_<uint8_t>& state, uint8_t flag)
{
    if (state.empty() || flag == 0) {
        return {};
    }

    int min_col = state.cols;
    int min_row = state.rows;
    int max_col = -1;
    int max_row = -1;
    for (int row = 0; row < state.rows; ++row) {
        for (int col = 0; col < state.cols; ++col) {
            if ((state(row, col) & flag) == 0) {
                continue;
            }
            min_col = std::min(min_col, col);
            min_row = std::min(min_row, row);
            max_col = std::max(max_col, col);
            max_row = std::max(max_row, row);
        }
    }

    if (max_col < min_col || max_row < min_row) {
        return {};
    }
    return cv::Rect(
        min_col, min_row, max_col - min_col + 1, max_row - min_row + 1);
}

inline cv::Mat downsampleAllowedGrowthMaskCovering(const cv::Mat& mask, int factor)
{
    if (mask.empty()) {
        return {};
    }
    factor = std::max(1, factor);
    const cv::Size dst_size = scaledGridSize(mask.size(), factor);
    cv::Mat normalized;
    if (mask.type() == CV_8UC1) {
        normalized = mask;
    } else {
        mask.convertTo(normalized, CV_8U);
    }

    cv::Mat_<uchar> result(dst_size, static_cast<uchar>(0));
    for (int r = 0; r < result.rows; ++r) {
        const int src_r0 = r * factor;
        const int src_r1 = std::min(mask.rows, src_r0 + factor);
        for (int c = 0; c < result.cols; ++c) {
            const int src_c0 = c * factor;
            const int src_c1 = std::min(mask.cols, src_c0 + factor);
            bool any_allowed = false;
            for (int sr = src_r0; sr < src_r1 && !any_allowed; ++sr) {
                for (int sc = src_c0; sc < src_c1; ++sc) {
                    if (normalized.at<uchar>(sr, sc) != 0) {
                        any_allowed = true;
                        break;
                    }
                }
            }
            result(r, c) = static_cast<uchar>(any_allowed ? 255 : 0);
        }
    }
    return result;
}

} // namespace vc::core::util
