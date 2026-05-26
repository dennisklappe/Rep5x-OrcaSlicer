> **⚠️ Work in progress.** This is alpha and is not done yet. Things may break. Test prints and feedback welcome, please open an issue with results.

<div align="center">

<picture>
  <img alt="Rep5x-OrcaSlicer" src="resources/images/OrcaSlicer.png" width="15%" height="15%">
</picture>

# Rep5x-OrcaSlicer

OrcaSlicer fork with multi-orientation planar slicing for the [Rep5x](https://github.com/dennisklappe/Rep5x) 5-axis retrofit (head-mounted B-tilt and C-pan).

<br>

[![Download](https://img.shields.io/github/v/release/dennisklappe/Rep5x-OrcaSlicer?include_prereleases&label=Download&color=brightgreen&style=for-the-badge)](https://github.com/dennisklappe/Rep5x-OrcaSlicer/releases)
[![Build](https://img.shields.io/github/actions/workflow/status/dennisklappe/Rep5x-OrcaSlicer/build_all.yml?branch=main&label=Build&style=for-the-badge)](https://github.com/dennisklappe/Rep5x-OrcaSlicer/actions/workflows/build_all.yml)
[![License](https://img.shields.io/badge/license-AGPL--3.0-blue?style=for-the-badge)](https://github.com/OrcaSlicer/OrcaSlicer/blob/main/LICENSE.txt)

</div>

---

## What this is, and what it isn't

Not true 5-axis slicing. The underlying engine is still OrcaSlicer's regular planar slicer. There are no curved non-planar toolpaths and the nozzle orientation does not vary within a layer.

What this fork adds is a way to break one model into multiple planar regions, slice each region in its own oriented frame, and stitch the per-region G-code back together. The 5-axis hardware reorients the head only *between* regions. Within a region, B and C are held fixed and the slicer prints normal horizontal layers in that rotated frame.

So you can print an upside-down `L` by slicing the vertical post bottom-up first, then the horizontal arm from its own root outward at 90 degrees. You can skip supports on parts you can reorient to face up. You can't print a continuously curving shell with varying nozzle orientation, and you don't get true non-planar layering.

---

## How it works

1. Drop a model into the slicer.
2. Right-click the model and pick **Add Rep5x direction modifier**. A cube-shaped modifier volume appears.
3. Rotate the modifier so its blue Z arrow points in the direction you want that region to grow. Move and resize it to overlap the part of the mesh that should print in that orientation.
4. Click the main Slice button.

Under the hood:

* The mesh is split into regions by intersecting it with each modifier volume. The background region is whatever isn't covered.
* Each region's mesh is rotated by the inverse of its build direction so the build axis lines up with +Z, then handed to OrcaSlicer's regular planar slicer.
* The per-region G-code is stitched back together. Each motion line gets `B<deg> C<deg>` appended to set the head orientation. Between regions, a transition sequence retracts, lifts above the tallest printed geometry plus the toolhead arm plus a margin, parks at a bed corner, and rotates B and C in free space.
* `G43.4 LB<...> LC<...>` is emitted once at the top so the firmware engages TCP inverse kinematics.

The output `.gcode` saves to your Downloads folder, and the [Rep5x web G-code viewer](https://tools.rep5x.com/gcode-viewer/) auto-opens with the file preloaded so you can sanity-check the toolpaths before committing to a print.

B and C angles follow the LinuxCNC convention used by the Rep5x firmware: B is tilt about Y (±135°), C is yaw about Z (±360°), and B=0 means nozzle straight down.

---

## Status

Working:

* Single main mesh with multiple Rep5x modifier volumes
* Mesh splitting via CGAL booleans
* Per-region planar slicing through subprocess calls to the bundled `orca-slicer` CLI
* Signed B/C G-code emission with TCP IK (`G43.4`)
* Dynamic safe-Z transitions (lift above tallest printed geometry + tool arm + margin, then park-then-rotate over empty bed)
* Bonding press: 0.1 mm overlap into the previously printed region for mechanical bond
* Auto-open in the Rep5x web G-code viewer

Not done yet:

* Hardware test on a real Rep5x. Algorithmically verified only so far.
* 3MF persistence of the Rep5x modifier flag (modifier type is lost on save and reload)
* Per-region print settings (all regions inherit the active profile)
* Multi-object scenes
* Envelope-collision check for the toolhead arc *during* the B/C rotation itself. Today only the post-rotation pose is verified.

The fork ships a single Rep5x profile (Ender 5 Pro with retrofit), but the workflow is profile-agnostic. Pick any printer profile and the 5-axis pipeline runs the same way. People are responsible for their own start/end G-code if they want to use it with a non-Rep5x machine.

---

## Download

Prebuilt binaries for tagged releases live on the [Releases page](https://github.com/dennisklappe/Rep5x-OrcaSlicer/releases). Linux AppImage, Windows portable zip and installer, macOS universal DMG.

---

## Building from source

Same as upstream OrcaSlicer. On Linux:

```bash
cmake --build build --config RelWithDebInfo --target all
```

See [`doc/`](doc/) for platform-specific instructions inherited from upstream.

---

## Credits

* **OrcaSlicer** ([OrcaSlicer/OrcaSlicer](https://github.com/OrcaSlicer/OrcaSlicer)). This fork is built entirely on top of their work; everything that makes a usable slicer is theirs.
* **Rep5x** retrofit project lives at [dennisklappe/Rep5x](https://github.com/dennisklappe/Rep5x).
