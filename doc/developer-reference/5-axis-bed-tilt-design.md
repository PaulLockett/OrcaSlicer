## 5-Axis (Pitch/Yaw Bed) Enablement in OrcaSlicer

### Who this is for
Recent CS/CE/EE grads who can code but are new to slicers, CNC kinematics, and G-code. We’ll explain the concepts, define jargon, and show exactly where to integrate changes in OrcaSlicer to support a 5-axis FFF printer with a tilting build plate (2 extra rotational degrees of freedom: pitch and yaw).

### What “5-axis” means here
- **Degrees of freedom (DOF)**: independent ways the machine can move. Traditional FFF printers are 4-DOF: X, Y, Z, and E (extruder). We’ll add two bed rotations:
  - **Pitch**: rotation about the X-axis (often called the A axis in CNC/G-code).
  - **Yaw**: rotation about the Y-axis (often called the B axis).

So the machine axes are X, Y, Z, E, plus A (pitch) and B (yaw). You’ll see A/B in CNC literature; some firmwares also support these as extra rotational axes.

### Why you might want this
- Reduce or eliminate supports by orienting the workpiece under a fixed nozzle.
- Print non-planar surfaces and stronger overhangs.
- Potentially improve surface quality and strength by aligning deposition with surface normals.

### Core idea at a glance
Add an "orientation planning" stage that computes desired bed angles (A/B) per move or per region, transform toolpaths from model/world coordinates into the machine coordinate system, then emit G-code with X/Y/Z/A/B/E. Update preview and collision checks to account for a rotated bed. Keep the feature off by default and fail-safe to standard planar slicing.

---

## Primer: Concepts and Vocabulary

- **Coordinate frames**
  - World/model frame W: where the object geometry lives in slicer math.
  - Machine frame M: axes you command in G-code (X, Y, Z, E, and now A, B).
  - Bed frame B: the build plate’s own frame; when you tilt the bed, you rotate this frame relative to W.

- **Forward/Inverse kinematics (FK/IK)**
  - FK: Given axis values (X, Y, Z, A, B), compute the pose of the bed/nozzle.
  - IK: Given a desired pose/orientation of the part under the nozzle, compute A/B (and sometimes X/Y/Z) to realize it.

- **Euler angles vs quaternions**
  - Euler (roll/pitch/yaw) is intuitive but can have singularities (gimbal lock). Quaternions are robust for interpolation. We can plan in quaternions and export A/B angles when emitting G-code.

- **Singularities**
  - Machine orientations where tiny angle changes cause big coordinate jumps. We’ll limit A/B ranges and smoothly plan angles to avoid abrupt changes.

---

## Where the relevant code lives (OrcaSlicer)

- G-code processing and writing
  - `src/libslic3r/GCodeWriter.hpp` and `GCodeWriter.cpp`: builds G0/G1 lines, emits axes and feedrates.
  - `src/libslic3r/GCode/GCodeProcessor.cpp`: parses and post-processes G-code; useful for preview/export pipeline.

- Print configuration and flavors
  - `src/libslic3r/PrintConfig.hpp` / `PrintConfig.cpp`: all printer/print settings; add new A/B axis options here, plus a new kinematics mode.
  - G-code flavor enum: `GCodeFlavor` in `PrintConfig.hpp`.

- Geometry and transforms
  - `src/libslic3r/Geometry.hpp` / `Geometry.cpp`: transforms, rotations, and utilities you will extend/reuse.
  - `src/libslic3r/Point.cpp`: transform helpers for point arrays.

- Build volume and bed shape
  - `src/libslic3r/BuildVolume.hpp` / `.cpp`: bed geometry and collision checks.
  - `src/slic3r/GUI/3DBed.hpp` / `.cpp`: renders the bed in the preview.

- Preview and toolpath visualization
  - `src/slic3r/GUI/GLCanvas3D.cpp`: loads and renders preview.
  - `src/slic3r/GUI/GCodeViewer.cpp` / `.hpp`: G-code preview and sliders.

Tip: Use these exact files as integration points. Search within them for the functions referenced below.

---

## High-level architecture for 5-axis support

1) New kinematics layer (planning A/B)
- A new module, e.g., `src/libslic3r/Kinematics/`:
  - `BedTiltKinematics.hpp/.cpp`: converts desired surface orientation into A/B angles; computes world→machine transforms.
  - `FiveAxisOrientationPlanner.hpp/.cpp`: decides when/how to tilt the bed (per layer, per region, or per segment), respecting velocity/acceleration/limit constraints.

2) Configuration extensions
- Add printer capabilities and limits:
  - `has_tilting_bed` (bool), `rotational_axes` (names, default A/B), `axis_limits_deg` (min/max for A/B), `axis_max_speed_deg_s`, `axis_max_accel_deg_s2`.
  - `bed_pivot_point` (Vec3 in mm): pivot relative to machine origin.
  - `orientation_mode` (enum): Off, PerLayer, PerIsland, PerSegment (increasing complexity).
  - `orientation_goals` (enum/flags): ReduceSupports, FollowSurfaceNormal, KeepNozzleUp, Custom.
  - Optional smoothing: `angle_smoothing_radius_mm`, `max_angle_step_deg`.

3) Toolpath transform and G-code emission
- Insert an orientation stage before `GCodeWriter` runs. For each move, compute A/B and transform X/Y/Z so the deposition in world space matches your intent with the bed tilted.
- Extend `GCodeWriter` to emit A/B coordinates on G0/G1.
- Handle absolute vs relative angles and homing state in the start G-code.

4) Preview and collision checks
- Update the preview to show a tilted bed and transformed toolpaths.
- Update build volume checks to account for bed rotation, not just flat Z.

5) Firmware compatibility
- Prefer firmwares that support extra axes (e.g., RepRapFirmware or Klipper with additional axes). Plan for two integration modes:
  - Direct A/B axes (G1 X.. Y.. Z.. A.. B.. E.. F..).
  - Macro-based for firmwares lacking native axes, where A/B are set via custom commands.

---

## Step-by-step implementation plan

### 1. Add configuration keys
Files to touch:
- `src/libslic3r/PrintConfig.hpp`
- `src/libslic3r/PrintConfig.cpp`

Add new options (names are suggestions; keep them concise/consistent):
- `has_tilting_bed` (coBool, default false)
- `rotational_axes` (coStrings, default ["A","B"]) — UI can validate these are single letters
- `a_min_deg`, `a_max_deg`, `b_min_deg`, `b_max_deg` (coFloat)
- `a_max_speed_deg_s`, `b_max_speed_deg_s`, `a_max_accel_deg_s2`, `b_max_accel_deg_s2` (coFloat)
- `bed_pivot_point` (coPoint3) — pivot location relative to machine origin
- `orientation_mode` (coEnum: Off, PerLayer, PerIsland, PerSegment)
- `orientation_goal` (coEnum: ReduceSupports, FollowSurfaceNormal, KeepNozzleUp, Custom)
- `angle_smoothing_radius_mm` (coFloat), `max_angle_step_deg` (coFloat)

Wire them into printer profiles and GUI later. Keep the defaults off so legacy behavior remains unchanged.

### 2. Create kinematics and planner modules
New files:
- `src/libslic3r/Kinematics/BedTiltKinematics.hpp/.cpp`
- `src/libslic3r/Kinematics/FiveAxisOrientationPlanner.hpp/.cpp`

Responsibilities:
- `BedTiltKinematics`
  - Inputs: desired local surface normal (or a simpler strategy), bed pivot, limits.
  - Outputs: feasible A/B angles and transform matrices:
    - T_Bed(W): rotation/translation of bed in world.
    - T_Machine(W): mapping to machine axes (X, Y, Z) given A/B.
  - Utility: clamp to limits, rate-limit with accel/vel constraints, interpolate using quaternions where helpful.

- `FiveAxisOrientationPlanner`
  - Decides when to change orientation: per layer, per island, or per segment.
  - Implements strategies:
    - ReduceSupports: aim bed normal towards overhang regions.
    - FollowSurfaceNormal: for top surfaces, align deposition to local normals.
    - KeepNozzleUp: mild tilts to avoid collisions but preserve traditional Z-up printing.
  - Provides a sequence of orientation keyframes (path index → A/B angles).

Leverage existing geometry helpers:
- `Geometry.cpp/.hpp` (e.g., `assemble_transform`, quaternion utils) and `Point.cpp` transforms.

### 3. Insert an orientation stage into the toolpath pipeline
Hook point candidates:
- After slicing/perimeters/infill are computed and before G-code writing. Toolpaths are accessible via `libslic3r` print objects/regions prior to `GCodeWriter`.

Implementation sketch:
- For each planned move (travel/extrude), request A/B from the planner.
- Compute world→machine transform using `BedTiltKinematics` with the bed pivot point.
- Transform the move’s coordinates: X/Y/Z’ so the deposited path in the world matches the intended geometry when the bed is rotated.
- Accumulate A/B into a smooth, rate-limited sequence; insert G1 angle updates only when A/B changes beyond a threshold to reduce code verbosity.

Edge cases:
- Retracts/unretracts: do not emit spurious A/B changes that cause ooze.
- Z hops: adjust hop direction relative to the tilted frame if needed.
- Arc moves (G2/G3): transform arcs consistently; if firmware doesn’t support arcs with additional axes, approximate with segments.

### 4. Extend G-code emission to include A/B
Files to modify:
- `src/libslic3r/GCodeWriter.hpp` / `.cpp`

Key places:
- Add helpers like `emit_a(double)`, `emit_b(double)` that delegate to the existing generic `emit_axis(char, ...)` inside `GCodeFormatter`.
- Where linear/arc moves are emitted (e.g., `extrude_to_xy`, travel emitters), include A/B when `has_tilting_bed` is true and the current planner state has updated angles.
- Respect absolute/relative angle modes; default to absolute angles and set with `G90`/`G91` semantics extended to rotational axes if your firmware supports it (many do by treating A/B like linear axes). If in doubt, emit absolute A/B and ensure homing angles are defined in start G-code.

Header/footer macros:
- In the print start G-code (profile), define homing and angle zero:
  - Example (RepRapFirmware-style): set up extra axes with `M584` (if needed), set limits `M208`, set current position `G92 A0 B0`.
- For Klipper: define `[extruder]` remains as-is, and add separate axes in `printer.cfg`. Slicer can assume they exist and just emit A/B on G1.

### 5. Update build volume checks and preview
Files to modify:
- `src/libslic3r/BuildVolume.hpp` / `.cpp`
- `src/slic3r/GUI/3DBed.hpp` / `.cpp`
- `src/slic3r/GUI/GLCanvas3D.cpp`
- `src/slic3r/GUI/GCodeViewer.cpp`

Collision/containment:
- Existing checks assume a flat bed. Add variants that take a transform (bed rotation) and test paths against the rotated volume. Practically: transform toolpaths into the bed frame before containment tests, or inflate bounds conservatively when bed tilts slightly.

Preview:
- Render the bed model with the current A/B applied per preview time. The preview time slider should interpolate A/B across moves (already supported for X/Y/Z/E; you’ll add two more channels).
- Show warnings if planned A/B exceed limits.

### 6. Firmware/dialect integration
Files to touch:
- `src/libslic3r/PrintConfig.hpp` (gcode flavor variants)
- `src/libslic3r/GCodeWriter.cpp` (flavor-dependent conditionals)

Two modes:
- Direct-axis mode: emit `G1 X.. Y.. Z.. A.. B.. E.. F..`.
- Macro-assisted mode: if firmware lacks A/B axes, emit custom sequences (e.g., `M290`-like custom M-codes or user-defined macros) to set angles, then standard `G1 X/Y/Z/E` for motion. Document this in profile notes.

Validation:
- Ensure `PrintConfig.cpp` validates that A/B settings are only used with compatible flavors (e.g., Klipper, RepRapFirmware). Provide clear error messages.

### 7. Testing strategy
- Unit tests (see `tests/`):
  - Kinematics: given target normals and pivot, produce A/B within limits.
  - Transform round-trips: world move + A/B → machine move; simulate FK and verify deposition point.
  - Emission: lines contain A/B when enabled; none when disabled.

- Integration tests:
  - Small model with overhang: verify reduced supports when orientation planner is on.
  - Preview correctness: A/B curves show up; no outside-of-bed warnings if within limits.

### 8. GUI options and UX
- Add an “Advanced → 5-axis (tilting bed)” panel with:
  - Enable toggle
  - Orientation mode (Off/PerLayer/PerIsland/PerSegment)
  - Limits and dynamics (min/max, speed/accel)
  - Pivot configuration helper (visual marker in bed view)
  - Safety switches: bound maximum A/B per-print; set “keep nozzle above part” constraint

Gradual rollout: hide PerSegment behind a “Developer/Experimental” toggle first; start with PerLayer (lowest complexity).

---

## Detailed integration notes (file-by-file)

### `src/libslic3r/GCodeWriter.hpp` / `.cpp`
- Add A/B emitters and state:
  - Track `current_A`, `current_B` in the writer.
  - When composing G1/G0 (see `GCodeG1Formatter`), call your new `emit_a()` / `emit_b()` if angles changed beyond an epsilon.
  - Ensure feedrate semantics remain consistent; angular velocity limits will be enforced by the planner (convert deg/s to equivalent feed limiting or emit separate synchronized moves if needed).

### `src/libslic3r/GCode/GCodeProcessor.cpp`
- Processor is used for post-processing and preview data extraction. Extend parsing to capture A/B on G0/G1 for the viewer data model so the preview can interpolate angles.

### `src/libslic3r/PrintConfig.hpp` / `.cpp`
- Define the new options and defaults.
- Validate flavor compatibility; add helpful error messages when misconfigured.

### `src/libslic3r/Geometry.cpp` / `.hpp` and `Point.cpp`
- Reuse `assemble_transform`, `rotation_transform`, quaternion helpers.
- Add helpers to build a transform for a bed rotated by A/B about a given pivot point.

### `src/libslic3r/BuildVolume.hpp` / `.cpp`
- Add functions to test points/paths against a rotated bed (accept a `Transform3d` for the current orientation). For initial implementation, a conservative 3D AABB inflated by max tilt is acceptable.

### `src/slic3r/GUI/3DBed.*`, `GLCanvas3D.cpp`, `GCodeViewer.*`
- Render the bed with current A/B.
- Extend the G-code preview path loader to record A/B per move (like X/Y/Z/E) and interpolate when scrubbing.

---

## Orientation planning strategies (from easiest to hardest)

1) Per-layer tilt
- Choose one A/B for an entire layer to keep most paths planar in that layer’s rotated frame.
- Good for reducing supports without heavy math.

2) Per-island tilt
- Different A/B for separate islands within a layer.
- Requires careful Z/seam transitions and collision checks.

3) Per-segment tilt (non-planar printing)
- Potentially change A/B along a perimeter or infill line to follow surface normals.
- Highest complexity: avoid abrupt changes, ensure E synchronization, and avoid nozzle crashes.

Recommended rollout: Implement Per-layer first, then Per-island, then Per-segment.

---

## Safety, limits, and synchronization

- Respect A/B min/max and rate limits; clamp and smooth angles.
- Synchronize linear and angular motion: either emit combined G1 with X/Y/Z/A/B or insert micro-segments such that both linear and angular constraints are respected.
- Keep nozzle clearance: ensure Z (or a virtual nozzle-to-surface distance) is adjusted when the bed tilts so the tip does not gouge the part.
- Homing and recovery: define start angles and a safe pose for pauses; ensure resume restores A/B consistently.

---

## Firmware notes

- RepRapFirmware (RRF):
  - Supports extra axes via `M584`, constraints via `M208`, and axis letters like A/B.
  - Slicer can emit `G1 X Y Z A B E F`. Configure machine kinematics in firmware.

- Klipper:
  - Configure additional axes in `printer.cfg` (custom stepper sections). Once configured, G-code with A/B is accepted.

- Marlin:
  - Some builds can be extended with `EXTRA_AXIS` patches, but support may vary. Prefer RRF/Klipper for early testing.

Provide profile templates with header snippets to define/zero A/B and soft limits. Document clearly in the printer profile.

---

## Testing checklist

- Unit tests (math): A/B planning returns feasible, smooth angles.
- Unit tests (emit): writer adds A/B only when changed; legacy off-mode emits no A/B.
- Integration: preview shows a tilted bed; no false “toolpath outside” when using bed tilt.
- Real hardware (with RRF/Klipper):
  - Dry-run: A/B move smoothly; no stutters.
  - Print a simple overhang twice (with/without 5-axis). Verify reduced supports and acceptable surface.

---

## Migration and backward compatibility

- Default is Off; no behavior changes for existing profiles.
- New config keys ignored if disabled.
- G-code flavor validation prevents emitting A/B on unsupported firmwares.

---

## Milestones (suggested)

1) Config + Preview scaffolding
- Add config keys; show A/B in preview from synthetic test data.

2) Per-layer tilt + G-code A/B emit
- Hardcode simple A/B per layer; emit combined moves; basic safety checks.

3) Per-island tilt + collision checks
- Distinct A/B per island; update BuildVolume checks for rotated bed.

4) Per-segment orientation (experimental)
- Follow normals on select perimeters; smoothing and rate limiting; arcs handling.

5) Production hardening
- Profiles, documentation, regression tests, and UX polish.

---

## Reading guide: exact reference points in code

- Emitters and move composition:
  - `src/libslic3r/GCodeWriter.hpp` (see `emit_axis`, `emit_xyz`, `emit_z`, `emit_e`, `GCodeG1Formatter`)
  - `src/libslic3r/GCodeWriter.cpp` (e.g., `extrude_to_xy`, arc emitters)

- G-code parsing and export:
  - `src/libslic3r/GCode/GCodeProcessor.cpp` (`process_G0/G1`, accumulation for preview)

- Config and flavors:
  - `src/libslic3r/PrintConfig.hpp` / `.cpp` (`GCodeFlavor`, bed/area options)

- Geometry:
  - `src/libslic3r/Geometry.cpp` / `.hpp` (`assemble_transform`, quaternion helpers)
  - `src/libslic3r/Point.cpp` (vectorized transforms)

- Build Volume and Preview:
  - `src/libslic3r/BuildVolume.hpp` / `.cpp`
  - `src/slic3r/GUI/3DBed.hpp` / `.cpp`
  - `src/slic3r/GUI/GLCanvas3D.cpp`, `src/slic3r/GUI/GCodeViewer.cpp`

---

## Closing thoughts

Start simple (Per-layer), test on firmware with native extra-axis support, and keep safety-first constraints. Once the pipeline is stable, iterate toward finer-grained orientation and non-planar strategies. The structure above isolates kinematics/planning from emission/preview, so future extensions (e.g., adding roll or nozzle tilt) can reuse the same architecture.

