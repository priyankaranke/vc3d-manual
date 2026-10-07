#pragma once

#include "vc/core/util/QuadSurface.hpp"
#include "vc/core/util/SurfacePatchIndex.hpp"
#include <cmath>
#include <optional>

// Navigation must project onto the current, drawable chart. An index built
// before a grid resize can report a nearby hit with obsolete grid indices.
inline std::optional<cv::Vec2f> projectSurfaceForNavigation(
    QuadSurface& surface, const cv::Vec3f& world, float tolerance,
    SurfacePatchIndex* index)
{
    for (int axis = 0; axis < 3; ++axis)
        if (!std::isfinite(world[axis])) return std::nullopt;

    cv::Vec3f ptr = surface.pointer();
    if (index && !index->empty()) {
        SurfacePatchIndex::PointQuery query;
        query.worldPoint = world;
        query.tolerance = tolerance;
        query.surfaces.only = SurfacePatchIndex::SurfacePtr(&surface, [](QuadSurface*) {});
        const auto hit = index->locate(query);
        if (!hit) return std::nullopt;
        ptr = hit->ptr;
    } else {
        const float distance = surface.pointTo(ptr, world, tolerance, 100, nullptr);
        if (!std::isfinite(distance) || distance < 0 || distance > tolerance)
            return std::nullopt;
    }

    const auto grid = surface.ptrToGrid(ptr);
    const auto* points = surface.rawPointsPtr();
    if (!points || !std::isfinite(grid[0]) || !std::isfinite(grid[1]) ||
        grid[0] < 0 || grid[1] < 0 ||
        grid[0] >= points->cols - 1 || grid[1] >= points->rows - 1)
        return std::nullopt;

    const int col = int(std::floor(grid[0])), row = int(std::floor(grid[1]));
    for (int dy = 0; dy < 2; ++dy) for (int dx = 0; dx < 2; ++dx) {
        const auto& p = (*points)(row + dy, col + dx);
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(p[axis]) || p[axis] < 0) return std::nullopt;
    }

    // Check the live bilinear sample, not just the index's cached distance.
    const float u = grid[0] - col, v = grid[1] - row;
    const auto current = (1-v)*((1-u)*(*points)(row,col) + u*(*points)(row,col+1)) +
                         v*((1-u)*(*points)(row+1,col) + u*(*points)(row+1,col+1));
    if (cv::norm(current - world) > tolerance) return std::nullopt;
    const auto loc = surface.loc(ptr);
    if (!std::isfinite(loc[0]) || !std::isfinite(loc[1])) return std::nullopt;
    return cv::Vec2f(loc[0], loc[1]);
}
