#include "SegmentationModule.hpp"
#include "SegmentationWidget.hpp"
#include "CState.hpp"
#include "ViewerManager.hpp"
#include "tools/SegmentationPushPullTool.hpp"
#include <QElapsedTimer>

namespace {
// Ancillary images can be stored on either a cell raster (N*k) or a vertex
// raster ((N-1)*k+1). Translate those rasters exactly; never resize their pixels.
std::optional<std::pair<int,bool>> channelScale(int pixels, int nodes) {
    if (nodes > 0 && pixels % nodes == 0) return {{pixels/nodes,false}};
    if (nodes > 1 && (pixels-1) % (nodes-1) == 0) return {{(pixels-1)/(nodes-1),true}};
    return std::nullopt;
}
}

bool SegmentationModule::publishPointGrowPreview(bool autosave)
{
    if (!_pointGrowTool.active() || !_pointGrowSnapshot || !_editManager) return false;
    auto surface = _editManager->baseSurface();
    if (!surface) return false;
    const auto& points = _pointGrowTool.points();
    const auto changed = _pointGrowTool.changedVertices();
    const bool resized = points.size() != _editManager->previewPoints().size();
    if (resized) {
        const auto oldSize = _pointGrowSnapshot->rawPoints().size();
        const auto offset = _pointGrowTool.offset();
        std::vector<std::pair<std::string,cv::Mat>> channels;
        for (const auto& name : _pointGrowSnapshot->channelNames()) {
            auto source = _pointGrowSnapshot->channel(name,SURF_CHANNEL_NORESIZE);
            if (source.empty()) continue;
            const auto sx = channelScale(source.cols,oldSize.width);
            const auto sy = channelScale(source.rows,oldSize.height);
            if (!sx || !sy || sx->first < 1 || sy->first < 1) {
                emit statusMessageRequested(tr("Channel %1 has an unknown grid mapping. Import a clean XYZ copy before growing.")
                    .arg(QString::fromStdString(name)),kStatusMedium);
                return false;
            }
            const int cols = (points.cols - (sx->second ? 1 : 0))*sx->first + (sx->second ? 1 : 0);
            const int rows = (points.rows - (sy->second ? 1 : 0))*sy->first + (sy->second ? 1 : 0);
            cv::Mat padded = cv::Mat::zeros(rows,cols,source.type());
            source.copyTo(padded(cv::Rect(offset.x*sx->first,offset.y*sy->first,source.cols,source.rows)));
            channels.emplace_back(name,std::move(padded));
        }
        if (!_editManager->setResizedPreviewPoints(points,_pointGrowTool.original(),changed,true)) return false;
        for (const auto& [name,channel] : channels) surface->setChannel(name,channel);
    } else if (!_editManager->setPreviewPointsOnly(points,changed,true)) return false;

    ++_pointGrowRevision;
    if (_state) _state->setSurface("segmentation",surface,false,true);
    emitPendingChanges();
    refreshOverlay();
    if (autosave) {
        const auto offset = _pointGrowTool.offset();
        auto gridOffset = utils::Json::array();
        gridOffset.push_back(offset.x); gridOffset.push_back(offset.y);
        surface->meta["grid_offset"] = gridOffset;
        surface->meta["manual_ct_extension"] = true;
        const auto delta = offset - _pointGrowPublishedOffset;
        if (delta != cv::Point()) {
            applyCorrectionAnchorOffset(delta.x, delta.y);
            saveCorrectionPoints(surface->path);
            _pointGrowPublishedOffset = offset;
        }
        // Undo can remove earlier additions, and padding changes grid indices.
        // Persist the current complete chart, never deltas against an old save.
        _forceFullSurfaceAutosave = true;
        markAutosaveNeeded(true);
    }
    return true;
}

bool SegmentationModule::finishPointGrow(bool apply)
{
    if (!_pointGrowSnapshot || !_editManager) return false;
    if (_pushPullTool) _pushPullTool->stopAll(false);
    auto surface = _editManager->baseSurface();
    if (!surface) return false;
    if (apply) {
        // Keep undone clicks/strokes when crossing into the older session
        // history. Redo remains ordered even after restoring an unpadded chart.
        std::vector<std::shared_ptr<QuadSurface>> nextStates;
        while (nextStates.size() < 20 && _pointGrowTool.redo()) {
            auto next = segmentation::snapshotSurface(*surface);
            _pointGrowTool.points().copyTo(*next->rawPointsPtr());
            next->invalidateCache();
            nextStates.push_back(std::move(next));
        }
        for (size_t i = 0; i < nextStates.size(); ++i) _pointGrowTool.undo();
        for (auto it = nextStates.rbegin(); it != nextStates.rend(); ++it)
            _redoHistory.captureSurface(*it);
    }
    if (apply && _pointGrowTool.changed()) {
        // Keep individual click/stroke undo after leaving the tool. The oldest
        // state also restores the original chart dimensions and channels.
        std::vector<std::shared_ptr<QuadSurface>> previousStates;
        while (previousStates.size() < 20 && _pointGrowTool.undo()) {
            auto previous = _pointGrowTool.changed()
                ? segmentation::snapshotSurface(*surface) : _pointGrowSnapshot;
            if (_pointGrowTool.changed()) {
                _pointGrowTool.points().copyTo(*previous->rawPointsPtr());
                previous->invalidateCache();
            }
            previousStates.push_back(std::move(previous));
        }
        for (auto it = previousStates.rbegin(); it != previousStates.rend(); ++it)
            _undoHistory.captureSurface(*it);
        _editManager->applyPreview();
        emit statusMessageRequested(tr("Editing finished. Changes save automatically."),kStatusShort);
    } else {
        // Clear queued vertex edits before refreshing. Otherwise refresh would
        // replay their padded grid indices onto the restored, smaller chart.
        _editManager->resetPreview();
        segmentation::restoreSurface(*surface,*_pointGrowSnapshot);
        _editManager->refreshFromBaseSurface();
        if (_pointGrowPublishedOffset != cv::Point()) {
            applyCorrectionAnchorOffset(-_pointGrowPublishedOffset.x,-_pointGrowPublishedOffset.y);
            saveCorrectionPoints(surface->path);
        }
        emit statusMessageRequested(tr("Editing finished; starting geometry restored."),kStatusShort);
    }
    _pointGrowTool.clear();
    _editManager->setAllowNewVertexEdits(false);
    _pointGrowSnapshot.reset();
    _pointGrowPublishedOffset = {};
    _manualAddTool->clear();
    _manualAddMode = false;
    emit manualAddModeChanged(false);
    _manualAddHoverThrottle.invalidate();
    if (_widget) {
        _widget->setManualAddActive(false);
        _widget->setGrowthMethod(_previousGrowthMethodBeforeManualAdd);
    }
    if (_viewerManager) _viewerManager->refreshSurfacePatchIndex(surface);
    if (_state) _state->setSurface("segmentation",surface,false,true);
    emitPendingChanges();
    refreshOverlay();
    _forceFullSurfaceAutosave = true;
    markAutosaveNeeded(true);
    return true;
}
