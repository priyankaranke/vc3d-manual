#pragma once

#include "vc/core/util/QuadSurface.hpp"
#include <cmath>
#include <limits>

// GUI-only footprint guard. A nearest-neighbour vertex mask cannot validate a
// bilinear XYZ sample: mixing a valid voxel position with (-1,-1,-1) samples
// unrelated CT along the edge. Do not change the pinned production renderer.
inline void guardSurfacePreviewFootprint(const QuadSurface& surface,
                                         cv::Mat_<cv::Vec3f>& coords,
                                         cv::Mat_<cv::Vec3f>& normals,
                                         float renderScale,
                                         const cv::Vec3f& offset)
{
    const auto& points = *surface.rawPointsPtr();
    const auto scale = surface.scale();
    const auto center = surface.center();
    const double ox = (offset[0]/renderScale + center[0])*scale[0];
    const double oy = (offset[1]/renderScale + center[1])*scale[1];
    const double sx = double(scale[0])/renderScale, sy = double(scale[1])/renderScale;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const cv::Vec3f invalid(nan,nan,nan);
    auto valid = [](const cv::Vec3f& p) {
        return p[0]>=0 && p[1]>=0 && p[2]>=0 &&
               std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
    };
    for (int y=0;y<coords.rows;++y) {
        const double gy = oy+y*sy;
        const int r = int(std::floor(gy));
        for (int x=0;x<coords.cols;++x) {
            const double gx = ox+x*sx;
            const int c = int(std::floor(gx));
            if (c<0 || r<0 || c+1>=points.cols || r+1>=points.rows ||
                !valid(points(r,c)) || !valid(points(r,c+1)) ||
                !valid(points(r+1,c)) || !valid(points(r+1,c+1))) {
                coords(y,x)=invalid;
                if (!normals.empty()) normals(y,x)=invalid;
                continue;
            }
            // The cached central-difference normal requires extra neighbours
            // outside this valid cell. A one-sided local normal keeps the
            // four-layer preview from punching extra holes at those edges.
            if (!normals.empty() && (!std::isfinite(normals(y,x)[0]) ||
                !std::isfinite(normals(y,x)[1]) || !std::isfinite(normals(y,x)[2]))) {
                const float u=float(gx-c),v=float(gy-r);
                const auto du=(1-v)*(points(r,c+1)-points(r,c))+v*(points(r+1,c+1)-points(r+1,c));
                const auto dv=(1-u)*(points(r+1,c)-points(r,c))+u*(points(r+1,c+1)-points(r,c+1));
                const auto n=du.cross(dv);
                const double length=cv::norm(n);
                if (length>1e-8) normals(y,x)=n/float(length);
            }
        }
    }
}
