# Changes from upstream

This fork starts from the `volume-cartographer/` folder of
[ScrollPrize/villa](https://github.com/ScrollPrize/villa) at commit
[`05ff9ea`](https://github.com/ScrollPrize/villa/tree/05ff9ea39bf1077f8749fc1fe06f43a548350143/volume-cartographer) (13 July 2026).
The first commit here (`5637e7a`) is that folder plus the growth-panel changes below;
the later commits are the manual editing work. All changes were made in 2026 and are
released under the same GNU GPL v3 or later as the original (see `LICENSE` and `NOTICE`).

## What changed

- **Growth panel** (in the first commit): a generation channel so a Grow request and the
  tracer agree on the current generation, a saved revision of the segment before each grow
  (kept beside it in `.vc3d-growth-history/`), and an overlay that marks what the last grow
  added.
- **Manual CT extension** (7-8 October 2026): a four-view workspace; click on the CT to extend
  or correct the surface (local thin-plate fit, the clicked point lands exactly on the CT);
  the native push/pull brush in the same mode; autosave after every edit; exact undo and
  redo; a view-only toggle; shading of edited areas; right-click point deletion; segment
  area and modified time kept up to date. `scripts/import_will_surface.py` imports a
  surface from a Will atlas as an editable project.
- **Per-winding editing** (21 October 2026): a Winding selector for stitched multi-winding
  segments, driven by `editing_regions` in `meta.json`.
- **Live flattened display** (written 14 October, added to this branch 22 October 2026): a local re-flattening of the view around each
  edit (`SurfaceDisplayMap`), saved as `manual_view_u.tif` / `manual_view_v.tif`, and ink
  overlays kept registered when the grid is padded.
- **Rotate** (29 October 2026): a display-only rotation of the flattened view, remembered per
  segment.

The README has a section for this fork and documents the features; the commit history has
the details.

## Files that differ from upstream 05ff9ea (77 including README.md)

Modified (51 plus `README.md`):

- `CMakeLists.txt`
- `apps/VC3D/AxisAlignedSliceController.cpp`
- `apps/VC3D/AxisAlignedSliceController.hpp`
- `apps/VC3D/CMakeLists.txt`
- `apps/VC3D/CWindow.cpp`
- `apps/VC3D/CWindow.hpp`
- `apps/VC3D/Keybinds.cpp`
- `apps/VC3D/Keybinds.hpp`
- `apps/VC3D/SurfacePanelController.cpp`
- `apps/VC3D/VCSettings.hpp`
- `apps/VC3D/ViewerManager.cpp`
- `apps/VC3D/ViewerManager.hpp`
- `apps/VC3D/overlays/InkDetectionOverlayController.cpp`
- `apps/VC3D/overlays/SegmentationOverlayController.cpp`
- `apps/VC3D/overlays/SegmentationOverlayController.hpp`
- `apps/VC3D/segmentation/SegmentationModule.cpp`
- `apps/VC3D/segmentation/SegmentationModule.hpp`
- `apps/VC3D/segmentation/SegmentationModule_Falloff.cpp`
- `apps/VC3D/segmentation/SegmentationModule_Input.cpp`
- `apps/VC3D/segmentation/SegmentationModule_Session.cpp`
- `apps/VC3D/segmentation/SegmentationUndoHistory.hpp`
- `apps/VC3D/segmentation/SegmentationWidget.cpp`
- `apps/VC3D/segmentation/SegmentationWidget.hpp`
- `apps/VC3D/segmentation/growth/SegmentationGrower.cpp`
- `apps/VC3D/segmentation/growth/SegmentationGrower.hpp`
- `apps/VC3D/segmentation/growth/SegmentationGrowth.cpp`
- `apps/VC3D/segmentation/growth/SegmentationGrowth.hpp`
- `apps/VC3D/segmentation/panels/SegmentationGrowthPanel.cpp`
- `apps/VC3D/segmentation/panels/SegmentationGrowthPanel.hpp`
- `apps/VC3D/segmentation/panels/SegmentationManualAddPanel.cpp`
- `apps/VC3D/segmentation/panels/SegmentationManualAddPanel.hpp`
- `apps/VC3D/segmentation/tools/ManualAddTool.hpp`
- `apps/VC3D/segmentation/tools/SegmentationEditManager.cpp`
- `apps/VC3D/segmentation/tools/SegmentationEditManager.hpp`
- `apps/VC3D/segmentation/tools/SegmentationPushPullTool.cpp`
- `apps/VC3D/segmentation/tools/SegmentationPushPullTool.hpp`
- `apps/VC3D/segmentation/tools/ThinPlateSpline3d.cpp`
- `apps/VC3D/volume_viewers/CChunkedVolumeViewer.cpp`
- `apps/VC3D/volume_viewers/CChunkedVolumeViewer.hpp`
- `apps/VC3D/volume_viewers/CVolumeViewerView.cpp`
- `apps/VC3D/volume_viewers/CVolumeViewerView.hpp`
- `apps/VC3D/volume_viewers/VolumeViewerBase.hpp`
- `apps/src/vc_grow_seg_from_seed.cpp`
- `core/include/vc/core/util/GrowthMask.hpp`
- `core/include/vc/core/util/QuadSurface.hpp`
- `core/src/GrowPatch.cpp`
- `core/src/QuadSurface.cpp`
- `core/test/CMakeLists.txt`
- `core/test/test_growth_helpers.cpp`
- `core/test/test_line_annotation_context_menu.cpp`
- `core/test/test_segmentation_autosave_state.cpp`

Added (25):

- `FORK_CHANGES.md`
- `apps/VC3D/segmentation/EditRegions.hpp`
- `apps/VC3D/segmentation/SegmentationGenerationMetadata.cpp`
- `apps/VC3D/segmentation/SegmentationGenerationMetadata.hpp`
- `apps/VC3D/segmentation/SegmentationGrowthHistory.hpp`
- `apps/VC3D/segmentation/SegmentationGrowthOverlay.cpp`
- `apps/VC3D/segmentation/SegmentationGrowthOverlay.hpp`
- `apps/VC3D/segmentation/SegmentationModule_PointGrow.cpp`
- `apps/VC3D/segmentation/tools/PointGrowChannels.hpp`
- `apps/VC3D/segmentation/tools/PointGrowTool.cpp`
- `apps/VC3D/segmentation/tools/PointGrowTool.hpp`
- `apps/VC3D/volume_viewers/SurfaceDisplayMap.cpp`
- `apps/VC3D/volume_viewers/SurfaceDisplayMap.hpp`
- `apps/VC3D/volume_viewers/SurfaceNavigation.hpp`
- `apps/VC3D/volume_viewers/SurfacePreviewFootprint.hpp`
- `core/test/test_growth_workflow.cpp`
- `core/test/test_keybinds.cpp`
- `core/test/test_point_grow.cpp`
- `core/test/test_segmentation_edit_smoothing.cpp`
- `core/test/test_segmentation_generation_metadata.cpp`
- `core/test/test_segmentation_growth_overlay.cpp`
- `core/test/test_segmentation_undo_history.cpp`
- `core/test/test_surface_display_map_rotation.cpp`
- `docs/imgs/vc3d-manual-four-views.jpg`
- `scripts/import_will_surface.py`
