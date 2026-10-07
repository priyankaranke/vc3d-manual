#include "SegmentationModule.hpp"
#include "SegmentationWidget.hpp"
#include "CState.hpp"
#include "ViewerManager.hpp"
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

bool SegmentationModule::publishPointGrowPreview()
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
    return true;
}

bool SegmentationModule::finishPointGrow(bool apply)
{
    if (!_pointGrowSnapshot || !_editManager) return false;
    auto surface = _editManager->baseSurface();
    if (!surface) return false;
    if (apply && _pointGrowTool.changed()) {
        if (!publishPointGrowPreview()) return false;
        _undoHistory.captureSurface(_pointGrowSnapshot);
        const auto offset = _pointGrowTool.offset();
        if (!surface->meta.is_object()) surface->meta = utils::Json::object();
        auto gridOffset = utils::Json::array();
        gridOffset.push_back(offset.x); gridOffset.push_back(offset.y);
        surface->meta["grid_offset"] = gridOffset;
        surface->meta["manual_ct_extension"] = true;
        // Existing controls keep their physical location after chart padding.
        applyCorrectionAnchorOffset(offset.x,offset.y);
        saveCorrectionPoints(surface->path);
        _editManager->applyPreview();
        _pendingAutosaveVertexUpdates.clear();
        _pendingAutosaveVertexUpdateIndex.clear();
        _saveSnapshot.reset();
        // A complete snapshot is required because the grid and ancillary
        // channels may have grown. The ordinary atomic tifxyz save runs async.
        markAutosaveNeeded(true);
        emit statusMessageRequested(tr("Manual extension applied; saving the tifxyz segment."),kStatusMedium);
    } else {
        // Clear queued vertex edits before refreshing. Otherwise refresh would
        // replay their padded grid indices onto the restored, smaller chart.
        _editManager->resetPreview();
        segmentation::restoreSurface(*surface,*_pointGrowSnapshot);
        _editManager->refreshFromBaseSurface();
        emit statusMessageRequested(tr("Manual extension canceled; original surface restored."),kStatusShort);
    }
    _pointGrowTool.clear();
    _pointGrowSnapshot.reset();
    _manualAddTool->clear();
    _manualAddMode = false;
    emit manualAddModeChanged(false);
    _manualAddHoverThrottle.invalidate();
    if (_widget) {
        _widget->setManualAddActive(false);
        _widget->setGrowthMethod(_previousGrowthMethodBeforeManualAdd);
    }
    if (_state) _state->setSurface("segmentation",surface,false,true);
    emitPendingChanges();
    refreshOverlay();
    return true;
}
