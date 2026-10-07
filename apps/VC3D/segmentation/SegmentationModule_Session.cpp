#include "SegmentationModule.hpp"

#include "../CState.hpp"
#include "SegmentationWidget.hpp"
#include "tools/SegmentationEditManager.hpp"
#include "tools/ApprovalMaskBrushTool.hpp"
#include "tools/SurfaceMaskBrushTool.hpp"
#include "ViewerManager.hpp"
#include "overlays/SegmentationOverlayController.hpp"

#include <QLoggingCategory>
#include <QTimer>

#include "vc/core/util/QuadSurface.hpp"
#include "vc/core/util/SurfacePatchIndex.hpp"
#include "vc/ui/VCCollection.hpp"

bool SegmentationModule::beginEditingSession(std::shared_ptr<QuadSurface> surface)
{
    if (!_editManager || !surface) {
        return false;
    }

    stopAllPushPull();
    clearUndoStack();
    resetHoverLookupDetail();
    _hoverPointer.valid = false;
    _hoverPointer.viewer = nullptr;
    if (!_editManager->beginSession(surface)) {
        qCWarning(lcSegModule) << "Failed to begin segmentation editing session";
        return false;
    }
    _autosaveState.beginSession();

    if (_state) {
        _state->setSurface("segmentation", _editManager->previewSurface(), false, true);
    }

    if (_overlay) {
        _overlay->setEditingEnabled(_editingEnabled);
    }

    useFalloff(_activeFalloff);

    // Set surface on approval tool if edit approval mask mode is active
    if (isEditingApprovalMask() && _approvalTool) {
        _approvalTool->setSurface(_editManager->baseSurface().get());
    }
    if (_surfaceMaskTool) {
        _surfaceMaskTool->setSurface(_editManager->baseSurface().get());
        _surfaceMaskTool->setActive(_editingEnabled && _drawMaskEnabled);
    }

    // Reload approval mask image if showing OR editing approval mask
    // Ensures dimensions match the session's surface
    if ((_showApprovalMask || isEditingApprovalMask()) && _overlay) {
        _overlay->loadApprovalMaskImage(_editManager->baseSurface().get());
    }

    if (_overlay) {
        refreshOverlay();
    }

    // Auto-load persisted corrections and set path for auto-save
    if (_pointCollection && !surface->path.empty()) {
        qCInfo(lcSegModule) << "Loading correction points from segment path:" << QString::fromStdString(surface->path.string());
        _pointCollection->loadFromSegmentPath(surface->path);
        _correctionsSegmentPath = surface->path;
    } else {
        qCInfo(lcSegModule) << "No segment path for correction points auto-save (path empty:" << surface->path.empty() << ")";
        _correctionsSegmentPath.clear();
    }

    emitPendingChanges();
    _pendingAutosaveVertexUpdates.clear();
    _pendingAutosaveVertexUpdateIndex.clear();
    _saveSnapshot.reset();
    updateAutosaveState();
    return true;
}

void SegmentationModule::endEditingSession()
{
    if (!flushAutosave()) return;
    _forceFullSurfaceAutosave = false;
    _pendingGridOffsetColDelta = 0;
    _pendingGridOffsetRowDelta = 0;
    stopAllPushPull();
    clearUndoStack();
    cancelDrag();
    clearLineDragStroke();
    if (_surfaceMaskTool) {
        _surfaceMaskTool->setActive(false);
    }
    _lineDrawKeyActive = false;
    resetHoverLookupDetail();
    _hoverPointer.valid = false;
    _hoverPointer.viewer = nullptr;
    refreshOverlay();
    auto baseSurface = _editManager ? _editManager->baseSurface() : nullptr;
    auto previewSurface = _editManager ? _editManager->previewSurface() : nullptr;

    if (_state && previewSurface) {
        auto currentSurface = _state->surface("segmentation");
        if (currentSurface.get() == previewSurface.get()) {
            const bool previousGuard = _ignoreSegSurfaceChange;
            _ignoreSegSurfaceChange = true;
            _state->setSurface("segmentation", baseSurface, false, true);
            _ignoreSegSurfaceChange = previousGuard;
        }
    }

    // Save any pending corrections immediately and clear auto-save state
    if (_correctionsSaveTimer && _correctionsSaveTimer->isActive()) {
        _correctionsSaveTimer->stop();
    }
    if (_pointCollection && !_correctionsSegmentPath.empty()) {
        qCInfo(lcSegModule) << "Saving correction points on session end to:" << QString::fromStdString(_correctionsSegmentPath.string());
        _pointCollection->saveToSegmentPath(_correctionsSegmentPath);
    }
    _correctionsSegmentPath.clear();

    if (_autosaveState.pending()) {
        performAutosave();
    }

    if (_viewerManager && previewSurface) {
        _viewerManager->refreshSurfacePatchIndex(previewSurface);
    }

    if (_editManager) {
        _editManager->endSession();
    }
    _autosaveState.endSession();
    _saveSnapshot.reset();
    _pendingAutosaveVertexUpdates.clear();
    _pendingAutosaveVertexUpdateIndex.clear();
    updateAutosaveState();
}

void SegmentationModule::onSurfaceCollectionChanged(std::string name, std::shared_ptr<Surface> surface)
{
    if (name != "segmentation" || !_editingEnabled || _ignoreSegSurfaceChange) {
        return;
    }

    if (!_editManager) {
        setEditingEnabled(false);
        return;
    }

    auto previewSurface = _editManager->previewSurface();
    auto baseSurface = _editManager->baseSurface();

    if (surface.get() == previewSurface.get() || surface.get() == baseSurface.get()) {
        return;
    }

    qCInfo(lcSegModule) << "Segmentation surface changed externally; disabling editing.";
    emit statusMessageRequested(tr("Segmentation editing disabled because the surface changed."), kStatusMedium);
    endEditingSession();
    setEditingEnabled(false);
}

bool SegmentationModule::captureUndoSnapshot()
{
    if (_suppressUndoCapture || _growthInProgress) {
        return false;
    }
    if (!_editManager || !_editManager->hasSession()) {
        return false;
    }

    const auto& previewPoints = _editManager->previewPoints();
    if (previewPoints.empty()) {
        return false;
    }

    const bool captured = _undoHistory.capture(previewPoints);
    if (captured) _redoHistory.clear();
    return captured;
}

bool SegmentationModule::captureUndoDelta()
{
    if (_suppressUndoCapture) {
        return false;
    }
    if (!_editManager || !_editManager->hasSession()) {
        return false;
    }

    const auto editedVerts = _editManager->editedVertices();
    if (editedVerts.empty()) {
        return false;
    }

    // Convert to delta format (storing original positions for undo)
    std::vector<segmentation::VertexDelta> deltas;
    deltas.reserve(editedVerts.size());
    for (const auto& edit : editedVerts) {
        deltas.push_back({edit.row, edit.col, edit.originalWorld});
    }

    const bool captured = _undoHistory.captureDelta(deltas);
    if (captured) _redoHistory.clear();
    return captured;
}

void SegmentationModule::discardLastUndoSnapshot()
{
    _undoHistory.discardLast();
}

bool SegmentationModule::restoreUndoSnapshot(bool redo)
{
    if (_viewOnly) return false;
    if (_suppressUndoCapture || _growthInProgress) {
        return false;
    }
    if (!_editManager || !_editManager->hasSession()) {
        return false;
    }

    auto& history = redo ? _redoHistory : _undoHistory;
    auto& inverseHistory = redo ? _undoHistory : _redoHistory;
    if (history.empty()) {
        return false;
    }

    _suppressUndoCapture = true;
    bool applied = false;
    bool restoredGrowthTransaction = false;
    std::optional<cv::Rect> undoBounds;
    std::vector<SegmentationEditManager::VertexEdit> autosaveEdits;
    const bool editingWasEnabled = _editingEnabled;
    std::shared_ptr<QuadSurface> inverseSurface;
    cv::Mat_<cv::Vec3f> inversePoints;
    std::vector<segmentation::VertexDelta> inverseDeltas;

    if (history.lastIsSurface()) {
        auto snapshot = history.takeLastSurface();
        const auto restoredSurface = _editManager->baseSurface();
        if (snapshot && restoredSurface) {
            auto* restoredPoints = restoredSurface->rawPointsPtr();
            if (restoredPoints) {
                inverseSurface = segmentation::snapshotSurface(*restoredSurface);
                const auto& currentMeta = redo ? snapshot->meta : restoredSurface->meta;
                if (restoredPoints->size() != snapshot->rawPointsPtr()->size() &&
                    currentMeta.contains("grid_offset") && currentMeta["grid_offset"].is_array() &&
                    currentMeta["grid_offset"].size() == 2) {
                    const float sign = redo ? 1.0f : -1.0f;
                    applyCorrectionAnchorOffset(sign * currentMeta["grid_offset"][0].get_float(),
                                                sign * currentMeta["grid_offset"][1].get_float());
                    saveCorrectionPoints(restoredSurface->path);
                }
                segmentation::restoreSurface(*restoredSurface, *snapshot);

                // A geometry-size change cannot be persisted safely as stale
                // per-vertex deltas. Force the autosaver's full-surface path.
                _pendingAutosaveVertexUpdates.clear();
                _pendingAutosaveVertexUpdateIndex.clear();
                _saveSnapshot.reset();
                _forceFullSurfaceAutosave = true;
                restoredGrowthTransaction = true;
                applied = true;
            }
        }
        if (!applied) {
            history.pushBackSurface(std::move(snapshot));
        }
    } else if (history.lastIsDelta()) {
        // Check if this is a delta-based entry or a legacy point snapshot.
        auto deltas = history.takeLastDelta();
        if (deltas && !deltas->empty()) {
            // Apply deltas to restore previous positions
            auto& previewPoints = _editManager->previewPointsMutable();
            int minRow = INT_MAX, maxRow = INT_MIN;
            int minCol = INT_MAX, maxCol = INT_MIN;

            for (const auto& delta : *deltas) {
                if (delta.row >= 0 && delta.row < previewPoints.rows && delta.col >= 0 && delta.col < previewPoints.cols) {
                    inverseDeltas.push_back({delta.row, delta.col, previewPoints(delta.row, delta.col)});
                    previewPoints(delta.row, delta.col) = delta.previousWorld;
                    autosaveEdits.push_back({delta.row, delta.col, {}, delta.previousWorld, false});
                    minRow = std::min(minRow, delta.row);
                    maxRow = std::max(maxRow, delta.row);
                    minCol = std::min(minCol, delta.col);
                    maxCol = std::max(maxCol, delta.col);
                }
            }

            if (minRow <= maxRow && minCol <= maxCol) {
                undoBounds = cv::Rect(minCol, minRow, maxCol - minCol + 1, maxRow - minRow + 1);
            }

            _editManager->applyPreview();
            if (auto surface = _editManager->previewSurface()) {
                surface->invalidateCache();
            }
            applied = true;
        }
    } else {
        // Legacy full snapshot restore
        auto state = history.takeLast();
        if (state && !state->empty()) {
            inversePoints = _editManager->previewPoints().clone();
            applied = _editManager->setPreviewPoints(*state, false, &undoBounds);
            if (applied) {
                _editManager->applyPreview();
            } else {
                history.pushBack(std::move(*state));
            }
        }
    }

    if (applied) {
        if (inverseSurface) inverseHistory.captureSurface(std::move(inverseSurface));
        else if (!inverseDeltas.empty()) inverseHistory.captureDelta(inverseDeltas);
        else inverseHistory.pushBack(std::move(inversePoints));

        auto restoredSurface = _editManager->previewSurface();
        if (restoredSurface) {
            restoredSurface->invalidateCache();
        }

        if (_editManager) {
            _editManager->refreshFromBaseSurface();
        }

        restoredSurface = _editManager->previewSurface();
        if (restoredGrowthTransaction) {
            updateApprovalToolAfterGrowth(_editManager->baseSurface().get());
        }
        if (_surfaceMaskTool) {
            _surfaceMaskTool->setSurface(restoredSurface.get());
            _surfaceMaskTool->refreshFromSurface();
            _surfaceMaskTool->setActive(editingWasEnabled && _drawMaskEnabled && hasActiveSession());
        }

        if (restoredGrowthTransaction && _overlay) {
            _overlay->clearApprovalMaskUndoHistory();
            if (_showApprovalMask || isEditingApprovalMask()) {
                _overlay->loadApprovalMaskImage(restoredSurface.get());
            }
        }

        if (_state) {
            auto preview = _editManager->previewSurface();

            _state->setSurface("segmentation", preview, false, true);
        }
        if (_viewerManager && restoredSurface) _viewerManager->refreshSurfacePatchIndex(restoredSurface);

        if (editingWasEnabled && !_editingEnabled) {
            setEditingEnabled(true);
            if (_widget) {
                _widget->setEditingEnabled(true);
            }
        }

        // Also undo the corresponding auto-approval if approval mask is active
        if (!redo && _overlay && _overlay->hasApprovalMaskData() && _overlay->canUndoAutoApproval()) {
            _overlay->undoLastAutoApproval();
            // Schedule save to persist the undo
            if (_editManager && _editManager->baseSurface()) {
                _overlay->scheduleDebouncedSave(_editManager->baseSurface().get());
            }
        }

        refreshOverlay();
        emitPendingChanges();
        queueAutosaveVertexUpdates(autosaveEdits);
        markAutosaveNeeded(restoredGrowthTransaction);
    }

    _suppressUndoCapture = false;
    return applied;
}

void SegmentationModule::clearUndoStack()
{
    _undoHistory.clear();
    _redoHistory.clear();
}

bool SegmentationModule::hasActiveSession() const
{
    return _editManager && _editManager->hasSession();
}

QuadSurface* SegmentationModule::activeBaseSurface() const
{
    return _editManager ? _editManager->baseSurface().get() : nullptr;
}

std::shared_ptr<QuadSurface> SegmentationModule::activeBaseSurfaceShared() const
{
    return _editManager ? _editManager->baseSurface() : nullptr;
}

bool SegmentationModule::captureUndoSnapshotForGrowth()
{
    if (_suppressUndoCapture || !_editManager || !_editManager->hasSession()) {
        return false;
    }

    const auto surface = _editManager->baseSurface();
    if (!surface || surface->rawPoints().empty()) {
        return false;
    }

    const bool captured = _undoHistory.captureSurface(segmentation::snapshotSurface(*surface));
    if (captured) _redoHistory.clear();
    return captured;
}

void SegmentationModule::refreshSessionFromSurface(QuadSurface* surface)
{
    if (!_editManager || !surface) {
        return;
    }
    if (_editManager->baseSurface().get() != surface) {
        return;
    }
    cancelDrag();
    _editManager->clearInvalidatedEdits();
    _editManager->refreshFromBaseSurface();
    if (_state) {
        _state->setSurface("segmentation", _editManager->previewSurface(), false, true);
    }

    // Update approval tool surface if editing approval mask
    if (isEditingApprovalMask() && _approvalTool) {
        _approvalTool->setSurface(surface);
    }

    // Reload approval mask image if showing OR editing approval mask
    // Both modes need correct dimensions to render/paint properly
    if ((_showApprovalMask || isEditingApprovalMask()) && _overlay) {
        _overlay->loadApprovalMaskImage(surface);
    }

    refreshOverlay();
    emitPendingChanges();
}

bool SegmentationModule::applySurfaceUpdateFromGrowth(const cv::Rect& vertexRect)
{
    if (!_editManager || !_editManager->hasSession()) {
        return false;
    }
    if (!_editManager->applyExternalSurfaceUpdate(vertexRect)) {
        return false;
    }

    auto* baseSurf = _editManager->baseSurface().get();

    // IMPORTANT: Reload approval mask image FIRST to get correct dimensions.
    // The surface has already been updated with the preserved approval mask from growth,
    // but the overlay's QImages still have the old dimensions. We must reload before
    // doing any auto-approval painting, otherwise we'd paint into wrong-sized images
    // and overwrite the correctly-preserved mask with garbage.
    // Reload if showing OR editing - both modes need correct dimensions.
    if ((_showApprovalMask || isEditingApprovalMask()) && _overlay) {
        _overlay->loadApprovalMaskImage(baseSurf);
    }

    // Auto-approve the growth region if approval mask is active (growth = reviewed/corrected).
    // Now that images are correctly sized, we can safely paint the auto-approval.
    if (_autoApprovalEnabled && _overlay && _overlay->hasApprovalMaskData() && vertexRect.area() > 0) {
        std::vector<std::pair<int, int>> gridPositions;
        gridPositions.reserve(static_cast<size_t>(vertexRect.area()));
        for (int row = vertexRect.y; row < vertexRect.y + vertexRect.height; ++row) {
            for (int col = vertexRect.x; col < vertexRect.x + vertexRect.width; ++col) {
                gridPositions.emplace_back(row, col);
            }
        }

        if (!gridPositions.empty()) {
            // performAutoApproval() paints and schedules a debounced save.
            // A synchronous save here would rewrite approval.tif per growth
            // step on the GUI thread; the debounce coalesces the burst.
            performAutoApproval(gridPositions);
            _overlay->clearApprovalMaskUndoHistory();
            qCInfo(lcSegModule) << "Auto-approved growth region:" << gridPositions.size() << "vertices"
                                << "(rect:" << vertexRect.width << "x" << vertexRect.height << ")";
        }
    }

    // Update approval tool surface if editing approval mask
    if (isEditingApprovalMask() && _approvalTool) {
        _approvalTool->setSurface(baseSurf);
    }

    refreshOverlay();
    emitPendingChanges();
    return true;
}

void SegmentationModule::requestAutosaveFromGrowth()
{
    markAutosaveNeeded();
}

void SegmentationModule::updateApprovalToolAfterGrowth(QuadSurface* surface)
{
    if (!surface) {
        return;
    }

    // Use base surface if there's an active editing session, otherwise use the provided surface
    QuadSurface* approvalSurface = surface;
    if (_editManager && _editManager->hasSession()) {
        approvalSurface = _editManager->baseSurface().get();
    }

    if (!approvalSurface) {
        return;
    }

    // Update approval tool surface if editing approval mask
    if (isEditingApprovalMask() && _approvalTool) {
        _approvalTool->setSurface(approvalSurface);
    }

    // Reload approval mask image if showing OR editing approval mask
    // Both modes need correct dimensions to render/paint properly
    if ((_showApprovalMask || isEditingApprovalMask()) && _overlay) {
        _overlay->loadApprovalMaskImage(approvalSurface);
    }
    if (_showLastGrowthOverlay && _overlay) {
        _overlay->loadLastGrowthMaskImage(approvalSurface);
    }
}

void SegmentationModule::applyCorrectionAnchorOffset(float offsetX, float offsetY)
{
    if (_pointCollection) {
        _pointCollection->applyAnchorOffset(offsetX, offsetY);
    }
}

void SegmentationModule::saveCorrectionPoints(const std::filesystem::path& segmentPath)
{
    if (_pointCollection && !segmentPath.empty()) {
        _pointCollection->saveToSegmentPath(segmentPath);
    }
}
