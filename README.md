# AdaptiveCAD C++ Skeleton

This repository now contains a C++20 skeleton for the AdaptiveCAD kernel described in `adaptive_cad_flat_adasccsc.md`.

## Included

- `adaptivecad_core` static library
- `adaptivecad_geometry` backend adapter library
- `adaptivecad_tool` scene/document tool library
- Initial B-rep entity wrappers (`Vertex`, `Edge`, `Face`, `Body`) via `BRepKernel`
- Optional native OpenCascade topology handles attached to B-rep entities when available
- Kernel torus primitive with an analytic OpenCascade native solid when OpenCascade is active and a deterministic faceted fallback on the always-available stub backend
- Strict OpenCascade wire/face validation with detailed `TopologyDiagnostic` failure reporting
- Edge-loop healing helpers for face creation (reordering and near-gap bridge repair)
- History-aware geometry state attached to B-rep entities with position, normal, curvature, adaptive metric, `pi_a`, `pi_f`, ARP path trust, curve memory, Phase-Lift winding/parity, residuals, and manufacturing state
- Adaptive geodesic routing over graph/mesh states with penalties for adaptive length, residual, memory, winding/branch changes, support risk, heat, stress, and route trust
- Phase-Lift, curve-memory, ARP trust, flat `pi_f`, power-law radial, radial-flux recovery, and deterministic torus/Benchy-style benchmark helpers in the geometry layer
- Core model scaffolding for:
  - constant residual branch
  - power-law radial branch
  - gaussian defect branch
  - memory branch with time-aware evaluation helpers
  - angular weighted branch with phase-map and adaptive mode evaluators
  - shell weight and area functional queries
- Geometry adapter scaffolding for:
  - always-available Euclidean stub backend
  - OpenCascade-ready backend wrapper (optional)
- Inverse fitting scaffold from area samples
- Nonlinear Gaussian inverse fitting with robust weighting and uncertainty/confidence reporting
- Mixed-model comparison (constant, power-law, gaussian) with AIC/BIC and Akaike-weight confidence
- `adaptivecad` CLI executable with help/version, demo, and geometry inspection commands
- `adaptivecad_ui` Win32 UI executable
- `adaptivecad_tests`, `adaptivecad_geometry_tests`, `adaptivecad_brep_tests`, `adaptivecad_history_tests`, `adaptivecad_tool_tests`, and `adaptivecad_scaffold_tests` executables + CTest integration
- 2D fit plot panel in the UI for measured points, model curves, and best-model residuals
- CAD-style model tree panel in the UI with geometry hierarchy and synced selection
- Interactive geometry viewport in the UI with simple B-rep projection and entity selection
- Viewport pan/zoom and selectable top/front/right/isometric projections
- Snapshot export that writes both human-readable text and machine-readable JSON analysis reports
- Document-backed multi-body scene management with session persistence for embedded geometry
- Mesh import for OBJ, ASCII/binary STL, and ASCII/binary PLY in the UI and through startup flags
- Optional material/color metadata propagation for OBJ/MTL, binary STL facet colors, and PLY vertex/face colors
- Optional STEP import through the same document pipeline when OpenCascade is enabled
- Angular branch editor controls in the UI with coefficient/session/report persistence
- Dedicated tool-layer session bundle IO for `.ini` save/load and paired report generation
- Damage-aware Chern-Locked metamaterial scaffold generation in the tool layer with angular-biased strut thickness and rerouted backbone weighting
- Connector-prepared scaffold output with explicit connector bodies at surviving lattice nodes and lightweight strut/connector metadata

## Quick Start (Windows PowerShell)

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
cpack --config build/CPackConfig.cmake
```

Run the CLI:

```powershell
.\build\runbin\adaptivecad.exe --help
.\build\runbin\adaptivecad.exe --demo
.\build\runbin\adaptivecad.exe --inspect-geometry .\samples\simple_wedge.stl
.\build\runbin\adaptivecad.exe --inspect-geometry .\samples\simple_wedge.stl --healing strict --json
.\build\runbin\adaptivecad.exe --export-geometry .\samples\simple_wedge.stl --output .\sessions\simple_wedge_export.obj
.\build\runbin\adaptivecad.exe --create-primitive torus --output .\sessions\torus.obj
.\build\runbin\adaptivecad.exe --create-primitive box --width 2.0 --depth 1.2 --height 0.8 --output .\sessions\box_custom.obj
.\build\runbin\adaptivecad.exe --create-primitive scaffold --cells-x 4 --cells-y 2 --cells-z 1 --spacing 0.75 --damage-radius 0.2 --output .\sessions\scaffold_custom.obj
```

Run the UI executable on Windows:

```powershell
.\build\runbin\cad_workbench_live.exe
```

The CLI demo exercises the history-aware kernel primitives and generates a small Chern-Locked scaffold assembly with a surviving-strut build summary.
It also creates a torus through `BRepKernel::create_torus(...)`; OpenCascade builds it as a native analytic torus shape, while the stub backend emits a faceted torus body.
The geometry inspection command imports OBJ, STL, PLY, or optional STEP assets and prints a topology summary suitable for smoke testing releases; pass `--json` for machine-readable topology counts and diagnostics.
The geometry export command writes Wavefront OBJ from any imported scene document so personal work can move back into other CAD or mesh tools.
The primitive creation command can generate `box`, `wedge`, `plane`, `torus`, or `scaffold` scenes directly to OBJ, with optional dimensions/radii/segment counts/lattice settings supplied on the command line.

Optional scripted session actions:

```powershell
.\build\runbin\cad_workbench_live.exe --import-geometry .\samples\stacked_triangles.obj
.\build\runbin\cad_workbench_live.exe --import-geometry .\samples\simple_wedge.stl
.\build\runbin\cad_workbench_live.exe --import-geometry .\samples\simple_quad.ply
.\build\runbin\cad_workbench_live.exe --save-session .\sessions\demo.ini --exit-after-actions
.\build\runbin\cad_workbench_live.exe --load-session .\sessions\demo.ini
```

Current UI actions:

- `Refresh`: recompute and redraw adaptive summary output
- `Open Spec`: open `adaptive_cad_flat_adasccsc.md` from the detected workspace root
- `Create` menu: create box, wedge, plane, torus, or metamaterial scaffold geometry in the active scene
- `Primitive` panel: choose box, wedge, plane, torus, or scaffold and edit dimensions, radii, segment counts, cells, spacing, and damage radius before creation
- `Add to scene`: append newly created primitives as additional bodies instead of replacing the active scene
- `Selected Body` panel: rename, move X/Y/Z, scale, rotate around Z, duplicate, or delete the selected body; selecting a face, edge, or vertex targets its owning body
- `Import CSV`: load measurement dataset and use it for nonlinear Gaussian fitting
- `Import Geometry`: load an OBJ, ASCII/binary STL, ASCII/binary PLY, or optional STEP scene and rebuild the document-backed B-rep viewport/tree
- `Export Geometry`: write the active scene as Wavefront OBJ
- `Save Session`: write an `.ini` session file plus paired report files for the current workbench state
- `Load Session`: restore dataset, viewport, healing policy, and selection state from a saved session file
- `Refit`: rerun nonlinear fitting on the current active dataset source
- `Clear Dataset`: clear imported data and revert fitting to synthetic demo samples
- `Export Snapshot`: write current UI output to `snapshots/adaptivecad_snapshot_*.txt` and a paired `adaptivecad_snapshot_*.json`
- `Angular Branch` panel: edit `lambda0`, cosine coefficients, and sine coefficients directly in the UI with apply/revert support
- `Healing Policy` dropdown: choose `Strict`, `RepairFirst`, or `RepairOnly` topology strategy
- `Close`: close the UI window

Measurement dataset panel:

- Active CSV path display (or synthetic fallback)
- Valid row count and rejected row count
- Quick import summary / last import error status

Fit plot panel:

- Measured sample points overlaid with constant, power-law, and gaussian model curves
- Residual subplot for the currently selected best model
- Live best-model label and dataset source summary

Geometry viewport:

- 2D projection of the current document-backed B-rep scene
- Vertex, edge, and face hit-testing with selection feedback
- Mouse-wheel zoom, right-drag pan, and reset-view support
- Projection selector for top, front, right, and isometric views
- Live backend/policy and topology diagnostic summary in the viewport footer

Model tree panel:

- Hierarchical body -> face -> edge structure plus a vertex list and diagnostic branch
- Body labels come from the active scene document so imported multi-body assets remain distinguishable
- Tree selection drives viewport highlight and viewport selection updates the tree caret
- Refresh preserves selection when the same demo entity remains available

JSON analysis report:

- Dataset source, row counts, summary text, and exported measurement samples
- Demo field scalars, angular harmonic coefficients, nonlinear gaussian fit details, and model-comparison scores/weights
- Plot points/curves/residual data and geometry diagnostic/entity state
- Scene metadata, body labels/materials/colors, and per-face outline points for imported or embedded multi-body scenes

Session bundle:

- Save/load format uses an `.ini` session file under `sessions/` plus same-stem `_report.txt` and `_report.json` files
- Imported measurement datasets are embedded directly in the session so a restore does not depend on the original CSV still being present
- Embedded scene bodies/faces/vertices and angular harmonic coefficients are stored in the session, so imported geometry can reopen even if the original source asset is unavailable
- Restores healing policy, viewport projection/zoom/pan, and the active geometry selection
- Save/load/report generation is implemented in `adaptivecad_tool::SessionBundleIO` so the Win32 UI no longer owns the session serialization rules

Generated scaffold API:

- `adaptivecad::tool::generate_chern_locked_metamaterial_scaffold(...)` creates a damage-aware lattice scene as a multi-body strut and connector assembly
- Strut conductance is biased by the existing angular branch model and thickened along the surviving left-to-right transport backbone after damage removal
- Scaffold output includes explicit connector bodies at active lattice nodes so struts have connector-prepared junction geometry before future boolean fusion
- Scaffold output now validates through the `Strict` topology path after scene-body winding normalization, non-manifold edge rejection, and orientation-aware shared-edge reuse

History-aware geometry kernel:

- `adaptivecad::geometry::AdaptiveCADState` is the canonical per-entity state for history-aware CAD: position, normal, curvature, adaptive metric, `pi_a`, `pi_f`, ARP trust, curve memory, Phase-Lift state, residuals, and manufacturing state
- `adaptivecad::geometry::compute_adaptive_geodesic(...)` computes graph routes using adaptive length plus residual, memory, winding, support-risk, heat, stress, and trust terms
- `advance_phase_lift(...)`, `update_curve_memory(...)`, and `update_path_trust(...)` expose the Phase-Lift, curve-memory, and ARP update rules for toolpath and mesh-routing code
- `make_torus_phase_lift_benchmark(...)` and `make_benchy_manufacturing_benchmark()` provide reproducible routing benchmarks for winding/seam handling and manufacturing-aware path choice
- Current claim: AdaptiveCAD is now a software-buildable history-aware geometry kernel foundation. It should be judged against standard CAD/slicer methods through torus, Benchy-style, mesh-geodesic, slicer, and scan-correction benchmarks; it does not yet claim proven superior geometry quality.

Kernel torus primitive:

- `adaptivecad::geometry::BRepKernel::create_torus(...)` creates a torus body directly from major and minor radii
- With OpenCascade enabled, the torus body stores a native `TopoDS_Shape` produced by OpenCascade's torus primitive builder for analytic downstream operations
- Without OpenCascade, the same API produces a deterministic faceted fallback body so tests, CLI demos, and non-OCC workflows still have a real torus topology to work with

Sample imported geometry:

- `samples/stacked_triangles.obj`: small two-body Wavefront OBJ asset used for import and session verification
- `samples/simple_wedge.stl`: compact ASCII STL mesh for CLI/UI import smoke tests; binary STL is also supported
- `samples/simple_quad.ply`: compact ASCII PLY mesh for CLI/UI import smoke tests; binary little-endian and big-endian PLY are also supported

CSV import format:

- Expected columns: `radius,area` (first two columns are used)
- Delimiters supported: comma, semicolon, or tab
- Header/comment lines are allowed; comments can start with `#`
- Requires at least 3 valid rows with positive radius and area

To request OpenCascade integration at configure time:

```powershell
cmake -S . -B build -DADAPTIVECAD_ENABLE_OPENCASCADE=ON
```

If OpenCascade is not detected, the project still builds and falls back to the stub geometry adapter.

## Layout

```text
include/adaptivecad/core/
  Models.hpp
  AdaptiveField.hpp
  InverseRecovery.hpp
include/adaptivecad/geometry/
  GeometryAdapter.hpp
  BRepEntities.hpp
  BRepKernel.hpp
  HistoryAwareKernel.hpp
  HistoryAwareState.hpp
include/adaptivecad/tool/
  SceneDocument.hpp
  SessionBundleIO.hpp
src/core/
  AdaptiveField.cpp
  InverseRecovery.cpp
src/geometry/
  BRepKernel.cpp
  HistoryAwareKernel.cpp
  StubGeometryAdapter.cpp
  OpenCascadeGeometryAdapter.cpp
src/tool/
  SceneDocument.cpp
  SessionBundleIO.cpp
src/app/
  main.cpp
src/ui/
  main_win32.cpp
samples/
  stacked_triangles.obj
  simple_wedge.stl
  simple_quad.ply
tests/
  test_adaptive_field.cpp
  test_geometry_adapter.cpp
  test_brep_kernel.cpp
  test_scene_document.cpp
```

## Next Build Steps

1. Expand STEP coverage from import-only triangulation into richer persistent assembly/body metadata when OpenCascade is available.
2. Add larger import-scale benchmarks and malformed-file tests for OBJ/STL/PLY scenes.
3. Fuse generated scaffold struts and connector bodies through boolean operations when a robust CAD backend is active.
