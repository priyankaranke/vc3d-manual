![Volume Cartographer](docs/imgs/banner.svg)

**Volume Cartographer** is a toolkit and set of cross-platform C++ libraries for
virtually unwrapping volumetric datasets. It was designed to recover text from
CT scans of ancient, badly damaged manuscripts, but can be applied in many
volumetric analysis applications.

This repository (a.k.a. **"VC3D"**) is a version of [EduceLab](https://educelab.engr.uky.edu/)'s original.
Via Vesuvius Challenge, it has gone through a few forks:
[EduceLab](https://github.com/educelab/volume-cartographer) (original) ->
[Julian Schilliger](https://github.com/schillij95/volume-cartographer-papyrus) (optical flow segmentation) ->
[Philip Allgaier](https://github.com/spacegaier/volume-cartographer) (performance/usability improvements) ->
[Hendrik Schilling and Sean Johnson](https://github.com/hendrikschilling/volume-cartographer) (various improvements) ->
this repository (developed and maintained by the Vesuvius Challenge team).
The original Volume Cartographer is a general purpose toolkit for virtual unwrapping, while this version is specialized for the frontier challenges of the Herculaneum papyri.

### VC3D Manual (this fork)

A fork of VC3D for precise editing of segments by hand: extend or correct a
surface by clicking on the CT, push or pull it with a brush, and see the
flattened view update as you go. Every edit saves automatically and can be
undone. The features are described under [Manual CT extension](#manual-ct-extension).

- **Upstream:** the `volume-cartographer/` folder of
  [ScrollPrize/villa](https://github.com/ScrollPrize/villa) at commit
  [`05ff9ea`](https://github.com/ScrollPrize/villa/tree/05ff9ea39bf1077f8749fc1fe06f43a548350143/volume-cartographer) (13 July 2026).
  [`FORK_CHANGES.md`](FORK_CHANGES.md) summarises the changes and lists every
  file that differs from upstream.
- **Licence:** GNU GPL v3 or later, as upstream ([`LICENSE`](LICENSE),
  [`NOTICE`](NOTICE)). The changes in this fork were made in 2026 and are
  released under the same licence.
- **Build:** as upstream; see [Installation Instructions](#installation-instructions).

### Manual CT extension

This branch adds a four-pane workspace: the flattened surface at top left,
XY at top right, XZ at bottom left, and YZ at bottom right. **Four views**
restores this layout; **Focus patch** centers the views on the active focus.

Open a native tifxyz segment or import a Will atlas patch into a new project:

```sh
python scripts/import_will_surface.py \
  --atlas-url 'http://<atlas host>/?patch=YOUR_PATCH_ID' \
  --output /path/to/new-project
```

The importer needs NumPy and tifffile. Open the resulting `project.volpkg.json`
in VC3D, then double-click its segment. It copies the atlas's existing normalized,
flattened XYZ coordinates without resampling and connects to the native remote
CT Zarr. Existing VC3D chunk caching and progressive resolution remain available.
Imported surfaces start with the dashboard's four-layer MAX preview at native
offsets −1, 0, +1, +2 and contrast 0–200. The interactive preview rejects cells
touching invalid XYZ instead of blending the invalid sentinel into unrelated CT.
CT cross sections remain single slices so clicks have an exact depth.

- Editing is enabled when a segment opens. Click beyond an edge in any CT pane
  or in a hole to extend the sheet; click on existing geometry to correct it
  locally. Each click immediately updates the surface geometry and preview.
  Every completed click saves automatically in the background.
- **A/D:** hold to push/pull at the cursor in the flattened or CT views, including
  newly added geometry. **Q/E:** decrease/increase the brush radius. The toolbar
  also exposes radius, Gaussian falloff, and step size. Each held stroke is one
  undo operation and saves automatically when released.
- **Ctrl/Cmd+Z:** undo the last click or stroke and automatically save the restored
  geometry. **Ctrl/Cmd+Shift+Z:** redo the last undone click or stroke exactly,
  including its anchors, and autosave it. A new edit clears the redo branch.
  **View only / Shift+E:** pause CT clicks and A/D without losing edits
  or Undo history. **Escape:** stop a held stroke; editing remains available.
  Closing the app waits for the final save.
- **R** while hovering: center linked CT slices at that location. **Right-drag:**
  pan. **Wheel:** zoom. **Shift+wheel:** move through CT slices.
- The Manual Add panel exposes edit width; legacy hole-fill controls remain
  under **Advanced hole-fill settings**.

Extensions use a local thin-plate fit and retain existing XYZ values. Touch-ups
move the clicked vertex exactly onto the CT point, with a smooth displacement
of its connected neighbours that fades to zero within the edit width plus one
grid step. Other clicked anchors remain pinned; holes and distant vertices are
unchanged. The native chart scale is retained. Padding translates ancillary
channels and correction anchors without image resizing. Clicks that would
overstretch or fold the chart are rejected; grow in short steps (at most 12 grid
steps per click). A/D retains VC3D's native brush behaviour and moves any clicked
anchors with the edited surface.

After each CT click or brush stroke, a small local flattening solve re-spaces
the flattened view within 8 grid cells of the edit, so distances on screen match
the true 3-D edge lengths. The edge of that neighbourhood stays fixed and a fold
check limits the update. Only the display mapping changes: clicked XYZ positions
and the saved grid stay exactly as they are. CT, ink overlays, picking and
navigation all use the same mapping. The offsets are saved with the segment as
`manual_view_u.tif` / `manual_view_v.tif`, and Undo/Redo restores them together
with the geometry.

- **Rotate** (Segment tab, next to **Focus patch**): turns the flattened view on
  screen only, which helps when fibres run diagonally. Nodes never move and
  nothing is resampled; rendering, clicks, the ink overlay and pan/zoom all use
  the same rotation. With axis-aligned slices off, the side cuts turn with the
  view. The angle is remembered per segment; `VC3D_VIEW_ROTATION=<deg>` sets it
  at launch.
- **Winding selector** for segments that stitch several windings into one grid:
  list them in `meta.json` as `editing_regions`, for example
  `[{"label": "w027", "cols": [0, 700], "rows": [0, 410]}]` (grid-cell ranges,
  end exclusive), optionally with `editing_region_default` or
  `editing_region_selected`. Only the chosen winding answers CT clicks, drags and
  the A/D brush; the others are shaded and labelled in the flattened view and
  drawn dimmer in the slice views. Regions move with the grid padding.

This does not automatically trace the tissue.
Edits use VC3D's normal atomic tifxyz saving. Navigation and focus markers use
the live editing index, including after chart padding and Undo.

The saved segment is native geometry for the Will/VC3D pipeline. New production
renders still require the established 50-iteration flattening, UV20 normalization,
physical mapping, and layer order. The interactive preview is not a newly
certified production render, and there is no global flattening or inference on
the click path. `VC3D_SETTINGS_FILE` optionally isolates a development build's
settings from the installed app.

### Installation Instructions
Due to a complex set of dependencies, it is *highly* recommended to use the docker image. We host an up-to-date image on the [github container registry](https://github.com/ScrollPrize/villa/pkgs/container/villa%2Fvolume-cartographer) which can be pulled with a simple command :

```bash
docker pull ghcr.io/scrollprize/villa/volume-cartographer:edge
```

To install VC3D and all command-line tools from source on a recent Debian-family
Linux distribution using APT:

```bash
./build_from_src_debian.sh
```

The script installs build dependencies from system packages, builds VC3D and
flatboi, and installs the runtime under `/usr/local` so commands such as
`VC3D` and `vc_grow_seg_from_seed` are available from the terminal. The
distribution must provide CMake 3.28 or newer; Ubuntu 24.04+ and Debian 13+
meet that requirement. Set `JOBS`, `BUILD_DIR`, or `PREFIX` to override the
corresponding defaults.

#### Windows

A self-contained Windows 10/11 installer (`VC3D-<version>-win64.exe`, with all dependencies bundled) is attached to each [GitHub release](https://github.com/ScrollPrize/villa/releases) — download, run, and launch VC3D from the Start menu. A portable `.zip` of the same tree is published alongside it.

To build natively from source on Windows, use an [MSYS2](https://www.msys2.org/) UCRT64 shell (see [.github/workflows/vc3d-windows.yml](https://github.com/ScrollPrize/villa/blob/main/.github/workflows/vc3d-windows.yml) in the upstream repository for the package list), then:

```bash
cmake --preset ci-windows-mingw
cmake --build --preset ci-windows-mingw
cpack --config build/ci-windows-mingw/CPackConfig.cmake
```

Note: the flatboi SLIM flattening tool (PaStiX-based) is not available on Windows; Docker or WSL still covers that workflow.

#### macOS

A self-contained `VC3D.app` for Apple Silicon (`VC3D-<version>-macos-arm64.dmg`, with all dependencies bundled) is attached to each [GitHub release](https://github.com/ScrollPrize/villa/releases) — drag it to Applications. The app is not notarized, so the first launch is right-click → Open (or `xattr -dr com.apple.quarantine /Applications/VC3D.app`). The `vc_*` command-line tools ship inside the bundle at `VC3D.app/Contents/MacOS/`.

To build from source on macOS, use the Homebrew LLVM build helper:

```bash
scripts/build_macos.sh --install-deps
```

After dependencies are installed, rebuilds can use:

```bash
scripts/build_macos.sh
```

[installation instructions for docker](https://docs.docker.com/engine/install/)

[running docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/)


### Basic introduction to VC3D 
The document below is a basic introduction to the ui and keybindings available in VC3D, more detailed documentation is available in the [segmentation tutorial](https://scrollprize.org/segmentation) on the scrollprize website.
> [!WARNING]
> if you are using Ubuntu, the default open file limit is 1024, and you may encounter errors when running vc3d. To fix this, run `ulimit -n 750000` in your terminal.

### Launching VC3D

If you're using docker : 

```bash
xhost +local:docker
sudo docker run -it --rm \
  -v "/path/to/data/:/path/to/data/" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 \
ghcr.io/scrollprize/villa/volume-cartographer:edge
```

From source : 

```bash
/path/to/build/bin/VC3D
```

![vc3d initial ui](docs/imgs/vc3d_ui_initial.png)

### Data 
VC3D requires a few changes to the data you may already have downloaded. All data must be in OME-Zarr format, of dtype uint8, and contain a meta.json file. To check if your zarr is in uint8 already, open a resolution group zarray file (located at /path/to.zarr/0/.zarray) look at the dtype field. "|u1" is uint8, and "|u2" is uint16. 

The meta.json contains the following information. The only real change from a standard VC meta.json is the inclusion of the `format:"zarr"` key.
```json
{"height":3550,"max":65535.0,"min":0.0,"name":"PHerc0332, 7.91 - zarr - s3_raw","slices":9778,"type":"vol","uuid":"20231117143551-zarr-s3_raw","voxelsize":7.91,"width":3400, "format":"zarr"}
```
Your folder structure should resemble this: 
```
.
└── scroll1.volpkg/
    ├── volumes/
    │   ├── s1_uint8_ome.zarr -- this is your volume data/
    │   │   └── meta.json - REQUIRED!
    │   └── 050_entire_scroll1_ome.zarr -- this is your surface prediction/
    │       └── meta.json - REQUIRED!
    ├── paths 
    └── config.json - REQUIRED!
```
### Opening a volume package and navigating the UI
First, click `File -> Open volpkg` and select the volpkg you wish to work with (select the folder ending in .volpkg)

On the left side of the UI, you have a few dock widgets. The first being the Volume Package / Surface List. The most important things to note here are the `Volume` dropdown list, which selects the currently displayed volume, and the `paths` surface list, which selects the currently displayed segment. Selecting a `Surface ID` should populate the `Surface` volume view in the top left, and initialize the rest the volume viewers.

There are a number of widgets, each with their own purpose. From top to bottom, and left to right : 

**Volume Package**

_Main UI widget for interacting with the volume package and surface list_

<img src="docs/imgs/volume-package-ui.png" style="height: 500px">

- `Reload Surfaces` : checks for new segmentations in the currently selected directory (by default, this is {volpkg}/paths
- `Filters` : Apply a single (or multiple) filters to show or hide specific surface ids from the current set
  - the most important one to remember here is `Current Segment Only`, which will hide the intersection overlays of all other segmentations in the volume viewers (this can greatly speed up ui interaction)
- `Approved`, `Defective`, `Reviewed`, `Inspect`  : tags which can be applied to the surface meta.json, and filtered against
- `Generate Surface Mask` and `Append Surface Mask` : Create a binary mask representing the valid surface, and optionally append the current selected volume to it as a multipage tif

**Location**

_Main UI widget for adjusting the ROI and display of the volume viewers_

<img src="docs/imgs/location-dock-ui.png" style="height: 500px">

- `Focus` : Displays the current location of the focus point (in XYZ) -- also can be used to set the focus point by modifying the values in the text box
- `Zoom` : Can be used in place of scroll wheel based zooming, mostly exists to alleviate touchy track pad zooming
- `Overlay Volume` : Selects the volume to overlay onto the base volume 
- `Overlay opacity` : Modifies the opacity of the overlay volume 
- `Axis overlay opacity` : Modifies the opacity of the plane slice axis overlays
- `Overlay threshold` : The value of the overlay array below which will not be rendered on the base volume (useful for removing background)
- `Overlay colormap` : The colormap to use for the overlay volume
- `Use axis aligned slices` : by default, vc3d attempts to align your XZ and YZ planes orthogonal to the normal of the current selected surface, this checkbox will instead use the plane normal as the axis to slice along
- `Show axis overlays in XY view` : if you do not wish to view the axis overlays in the XY view, this checkbox will disable them

_Keybinds:_
> - `Right mouse + drag` : pans the current viewer
> - `Scroll wheel` : zooms the current viewer
> - `Ctrl + left mouse button` : centers the focus point on the cursor position
> - `Scroll wheel click + drag` : rotates the slicing pane in the XZ or XY volume viewers
> - `Spacebar` : toggles the overlay on / off
> - `C` : toggle a composite view of the current surface in the segmentation window (parameters of which can be located in the "composite" dock tab, next to the location tab)

**Segmentation**

_Primary entry point for interacting with the segmentation, more info in the [segmentation docs](docs/segmentation.md)_
 
<img src="docs/imgs/segmentation-widget-ui.png" style="height: 500px">

- `Enable editing` : Must be checked to enable any of the actions in this widget. Creates a copy of the base surface upon which we perform any of the following actions
- `Surface Growth` - grouping of settings mostly pertaining to the `Grow` action
  - `Steps` : The number of "generations" the growth action will undertake
  - `Allowed Directions` : Limits the directions of growth _relative to the flattened 2d quad surface!_ (i.e if you look in the segmentation window in the top left, and you set 'up' it will grow towards the top of the window)
  - `Limit Z range` : Constrains the growth to a selected `Z` range
  - `Volume` : the volume to pass to the `tracer()` call _should be a surface prediction volume!_ 
- `Editing` - grouping of settings which mostly apply to drag/push/pull actions for mesh deformation. All mesh deformation-type actions are performed on a gaussian-like area which has a circular shape centered at the current mouse location and whose strength is reduced as we reach the edges. _Radius is in quad vertices in the 2d flattened surface._
  - `Max Radius` : the maximum radius of the area to be affected by the action
  - `Sigma` : the _strength_ of the push/pull on affected vertices other the original one (aka how quickly the influence tapers as we step away from the original point)
  - `Push Pull Step` : The number of "offset steps" to take along the surface normal (in voxels) for each push/pull action (this parameter only affects the push/pull in the segmentation window that is applied with `F` and `G`)
- `Direction Fields`
  - `Zarr folder` : the path containing the direction field you want to use for optimization (ex: `/path/to/scroll.volpkg/fiber-directions.zarr/horizontal`) _these are optional_ 
    - `Orientation` : the orientation/type of the direction field (from horizontal, vertical, or normal)
    - `Scale level` : the zarr resolution from which these were computed
    - `Weight` : the weight to apply to the field when optimizing.
- `Corrections` : group which controls / manages the "correction points" for the current growth session
  - `New correction set` : creates a new point collection containing a single "correction" 
      
_Keybinds:_
> - `F` or `G` : push/pull the current surface in a positive or negative direction along the surface normal (only works in the surface
> - `1`, `2`, `3`, `4`, `5` : select a direction to grow in -- left, up, down, right, all, respectively
> - `T` : create a new correction set

**Seeding** 

_Widget for creating initial segmentations which can be used for traces or later growth actions_

<img src="docs/imgs/seeding-ui.png" style="height: 500px">

- `Switch to point mode` : toggles the seeding widget into draw mode (not recommended)
- `Source Collection` : the source point collection to use as seed points (will autofill if you draw/analyze)
- `Parallel processes` : the number of processes to run seeding with
- `OMP Threads` : limits the amount of threads each process can use (recommend to set this to 1)
- `Intensity threshold` : mostly unused, can leave default
- `Peak detection window` : how closely each peak detected along a path/raycast can be
- `Expansion iterations` : how many iterations to run expansion mode on after initial seeding
- `Show Preview Points` : unused, can be ignored
- `Clear` (both) : clears current points, paths, or raycasts
- `Analyze Paths` : after drawing paths, detect peaks along the line segment on which to place seed points
- `Run seeding` : creates segmentations from the current seed points
- `Expand seeds` : expands the current seed points to the given number of iterations
- `Clear all paths` : clears any user drawn paths / points (use this over the clear buttons most of the time)
- `Start label wraps` : this is used for absolute or relative wrap labels, is not used during the seeding step, will detail in another document
