#pragma once

#include <opencv2/core.hpp>
#include <memory>

class QuadSurface;

// Interactive UVs are a display layer, never a replacement for the saved XYZ
// grid or the production renderer's parameterization. Offsets are in voxels;
// zero padding therefore preserves both the mapping and ancillary image pixels.
namespace surface_display {
inline constexpr const char* uChannel = "manual_view_u";
inline constexpr const char* vChannel = "manual_view_v";
cv::Mat_<cv::Vec2f> readOffsets(QuadSurface& surface);
void writeOffsets(QuadSurface& surface, const cv::Mat_<cv::Vec2f>& offsets);

// Relax a bounded neighbourhood to its actual 3-D edge lengths, with a fixed
// perimeter and a positive-Jacobian line search. XYZ is strictly read-only.
cv::Rect relax(const cv::Mat_<cv::Vec3f>& points, cv::Mat_<cv::Vec2f>& offsets,
               cv::Vec2f gridScale, cv::Rect changed);
}

// Immutable, shared by the GUI and its asynchronous render job. Both directions
// use the same triangle map, including over holes, so picking/ROI overlays stay
// registered. Geometry sampling still uses the original bilinear XYZ cells.
class SurfaceDisplayMap
{
public:
    static std::shared_ptr<const SurfaceDisplayMap> fromSurface(QuadSurface& surface);
    SurfaceDisplayMap(const cv::Mat_<cv::Vec3f>& points,
                      const cv::Mat_<cv::Vec2f>& offsets,
                      cv::Vec2f scale, cv::Vec3f center);
    cv::Vec2f forward(cv::Vec2f nominal) const;
    cv::Vec2f inverse(cv::Vec2f display) const;
    cv::Mat_<cv::Vec2f> gridMap(cv::Size pixels, cv::Vec2f nominalCenter,
                              float pixelsPerVoxel) const;
    void gen(cv::Mat_<cv::Vec3f>& coords, cv::Mat_<cv::Vec3f>* normals,
             cv::Size pixels, cv::Vec2f nominalCenter, float pixelsPerVoxel) const;
    cv::Size gridSize() const { return _points.size(); }
private:
    cv::Vec2f vertex(int r, int c) const;
    cv::Vec2f toGrid(cv::Vec2f nominal) const;
    cv::Vec2f fromGrid(cv::Vec2f grid) const;
    cv::Mat_<cv::Vec3f> _points;
    cv::Mat_<cv::Vec2f> _offsets;
    cv::Vec2f _scale, _center, _maxOffset{0,0};
};
