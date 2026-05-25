# AdaptiveCAD Commercialization Plan

AdaptiveCAD is currently an engineering-preview product: it builds, runs, imports common mesh formats, has a Win32 workbench, and has CTest coverage for the core libraries. It is not yet a complete commercial CAD product.

## Current Release Artifact

- Build system: CMake with Ninja/MSVC-compatible targets
- Package target: CPack ZIP
- Runtime binaries: `adaptivecad.exe` and `cad_workbench_live.exe`
- Libraries: `adaptivecad_core`, `adaptivecad_geometry`, and `adaptivecad_tool`
- Public headers under `include/adaptivecad`
- Samples and documentation installed into the package

Build and package:

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
cpack --config build/CPackConfig.cmake
```

## Minimum Commercial Release Criteria

1. Product Positioning
   - Define the first paid use case: geometry inspection, adaptive fitting, scaffold generation, or CAD-kernel SDK.
   - Define the buyer and workflow: researcher, CAD plugin developer, additive manufacturing engineer, or internal R&D team.
   - Decide whether the first release is a desktop app, SDK, CLI utility, or paid preview.

2. Legal And Licensing
   - Choose the product license and EULA.
   - Confirm ownership of the AdaptiveCAD specification and source.
   - Audit OpenCascade and any future third-party dependencies for license compatibility.
   - Add copyright notices, third-party notices, and installer/package license text.

3. Quality
   - Add release smoke tests for CLI import, UI startup, session save/load, and CPack output.
   - Add larger real-world geometry import tests for OBJ/STL/PLY.
   - Add failure-mode tests for malformed geometry, large files, and invalid sessions.
   - Run static analysis and sanitizers in CI.

4. Product UX
   - Replace research-demo wording with user-task wording.
   - Add in-app status messages for long imports and failed operations.
   - Add recent files, save prompts, and deterministic default output locations.
   - Add a first-run sample workflow that proves value in under five minutes.

5. Distribution
   - Publish signed Windows ZIP or installer builds.
   - Add versioned release notes and checksums.
   - Add crash/error reporting only after a privacy policy is written.
   - Decide support channel and response expectations.

6. Commercial Validation
   - Benchmark against baseline CAD/slicer workflows with documented datasets.
   - Record before/after outputs for at least three target workflows.
   - Get pilot-user feedback before claiming production-grade geometry quality.

## Recommended First Paid SKU

Ship AdaptiveCAD first as a paid technical preview for geometry import, adaptive analysis, and scaffold generation rather than as a full CAD replacement. The current codebase is closest to a kernel/workbench preview, not a mature parametric CAD system.

## Next Engineering Milestones

1. Add package smoke tests that build, install, and run the installed binaries.
2. Add signed release packaging or an installer.
3. Add a real project/session format migration test suite.
4. Add import-scale benchmarks for large STL/PLY files.
5. Add user-facing documentation for the CLI, UI, and SDK headers.

