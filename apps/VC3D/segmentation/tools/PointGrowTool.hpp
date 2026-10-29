#pragma once

#include "SegmentationEditManager.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

// Interactive extension in the imported surface's chart. World coordinates are
// native voxels; rows/columns retain their original meaning (only padding moves
// their integer origin). No CT I/O, global flattening or tracer runs on a click.
class PointGrowTool
{
public:
    using GridKey = SegmentationEditManager::GridKey;
    bool begin(const cv::Mat_<cv::Vec3f>& points, cv::Vec2f gridScale = {1,1},
               const cv::Mat_<cv::Vec2f>& displayOffsets = {});
    bool addPoint(const cv::Vec3f& world, int width, std::string& status);
    // Pick only close, visible vertices in a CT slice, including imported ones.
    std::optional<GridKey> pickPointNearSlice(const cv::Vec3f& world,
                                            const cv::Vec3f& normal,
                                            float pixelsPerVoxel,
                                            float radiusPixels = 12.0f,
                                            float sliceTolerance = 10.0f) const;
    bool removePoint(GridKey key, std::string& status);
    // Record one native brush stroke in the same history as CT clicks.
    bool adoptPreview(const cv::Mat_<cv::Vec3f>& points, bool* changed = nullptr);
    bool undo();
    bool redo();
    bool canRedo() const { return !_redo.empty(); }
    void clear();
    bool active() const { return !_points.empty(); }
    bool changed() const { return !_undo.empty(); }
    const cv::Mat_<cv::Vec3f>& points() const { return _points; }
    const cv::Mat_<cv::Vec3f>& original() const { return _original; }
    cv::Point offset() const { return _offset; }
    // Clicks whose nearest chart vertex lies outside this grid region are refused (stitched windings).
    void setEditRegion(std::optional<cv::Rect> region) { _editRegion = region; }
    std::optional<cv::Rect> editRegion() const { return _editRegion; }
    const cv::Mat_<cv::Vec2f>& displayOffsets() const { return _displayOffsets; }
    std::vector<GridKey> changedVertices() const;
    std::vector<cv::Vec3f> anchors() const;

private:
    struct Anchor { cv::Point grid; cv::Vec3f world; };
    struct Change { cv::Point grid; cv::Vec3f before; };
    struct DisplayChange { cv::Point grid; cv::Vec2f before; };
    struct Undo {
        std::vector<Change> changes;
        std::vector<Anchor> anchors;
        std::vector<DisplayChange> displayChanges;
    };
    void flattenDisplay(Undo& undo);
    bool moveHistory(std::vector<Undo>& from, std::vector<Undo>& to);
    void pad(int amount);
    cv::Mat_<cv::Vec3f> _points, _original;
    cv::Mat_<cv::Vec2f> _displayOffsets;
    cv::Vec2f _gridScale{1,1};
    cv::Point _offset;
    std::optional<cv::Rect> _editRegion;
    std::vector<Anchor> _anchors;
    std::vector<Undo> _undo, _redo;
};
