# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

Use `make/` as the CMake build directory. Dependencies are managed automatically via vcpkg (set `VCPKG_ROOT` or pass `-DCMAKE_TOOLCHAIN_FILE`).

```bash
mkdir make && cd make
cmake ..
cmake --build . -j4
```

Executables land in `make/bin/Debug/` or `make/bin/Release/`. Prefer incremental builds in the existing `make/` tree rather than creating new build directories.

Key optional CMake flags: `OpenMVS_USE_CUDA=ON`, `OpenMVS_USE_CERES=ON`, `OpenMVS_USE_PYTHON=ON`, `OpenMVS_BUILD_VIEWER=ON`, `OpenMVS_ENABLE_TESTS=ON`.

## Tests

```bash
cd make
ctest -R CommonUnitTests   # libs/Common changes
ctest -R SFMPipelineTest   # libs/SFM changes
ctest -R MVSPipelineTest   # libs/MVS changes
```

These map to the `Tests` executable with args `0`, `1`, `2`. Test data lives in `apps/Tests/data/`. Run all three for cross-cutting changes.

## Architecture

The library is split into four layers that must be built in dependency order:

```
libs/Common/  (SEACAVE namespace) — foundation: containers, threading, geometry, config
     ↓
libs/Math/  (MATH)  +  libs/IO/  (IO)
     ↓
libs/SFM/  (SFM)  +  libs/MVS/  (MVS)
     ↓
apps/*  +  apps/Viewer/
```

**`libs/Common/`** — Every other module pulls this in via the precompiled header `Common.h`. Provides `cList<T>` (the primary container, used instead of `std::vector`), geometry primitives (TAABB, TRay, TOctree, etc.), threading (`BS::light_thread_pool`, `EventQueue`), smart pointers (`CSharedPtr`, `CAutoPtr`), and logging/assertion macros. The `ConfigLocal.h` generated at CMake configure-time carries all `_USE_CUDA`, `_USE_CERES`, etc. flags.

**`libs/SFM/`** — Full Structure-from-Motion. Central class: `SFM::Scene`. Supports three strategies (Incremental, Hierarchical, Global). Key sub-systems: `FeaturesExtractor` (AKAZE/ORB/SIFT/SiftGPU), `PairsMatcher` (vocabulary tree or exhaustive), `BundleAdjustment` (Ceres), `GlobalRotationAveraging`, `GlobalPositioner`. Outputs a `.sfm` Boost-serialized scene that `ExportMVS()` converts to `.mvs` for the MVS pipeline.

**`libs/MVS/`** — Multi-View Stereo. Central class: `MVS::Scene`. Pipeline: `DenseReconstruction` (PatchMatch depth maps, CPU or CUDA) → `ReconstructMesh` (CGAL Delaunay + graph-cut) → `RefineMesh` (gradient descent, CPU or CUDA) → `TextureMesh` (MRF face selection, atlas packing, seam blending).

**File formats:** `.sfm`/`.mvs` are Boost-serialized binary scenes. `.dmap` is a custom binary depth map. PLY/OBJ/glTF are handled by `libs/IO/`.

**Pixel convention:** integer center at (0,0), top-left corner at (-0.5, -0.5). COLMAP import/export applies ±0.5 shift to `cx`/`cy`.

**Camera pose:** `P = K R [I | -C]` where R is world-to-camera and C is camera center in world coordinates.

## Code Conventions

- Formatting: tabs (width 4), `ColumnLimit: 0`, no auto-sorted includes. Classes/structs/functions: opening brace on next line. Control statements: brace on same line.
- Naming: `CamelCase` for types/functions, `lowerCamelCase` for variables; prefixes `n`/`f`/`b`/`p` for numeric/float/bool/pointer.
- Use `ASSERT(...)` for invariants, `VERBOSE`/`DEBUG`/`DEBUG_EXTRA` for logging — never `printf`/`cout`.
- Use `FOREACH(i, list)` / `FOREACHPTR(pItem, list)` macros and `cList<T>` container throughout.
- `NO_ID` (`(uint32_t)-1`) is the invalid-index sentinel used everywhere.
- Images pixels are lazy-loaded: call `Image::LoadPixels()` before accessing and `Image::ReleasePixels()` when done.

## Documentation

- [BUILD.md](BUILD.md) — full build, install, and Python API instructions
- [docs/architecture.md](docs/architecture.md) — detailed module architecture, threading model, data structures
- [docs/pipelines.md](docs/pipelines.md) — step-by-step pipeline data-flow traces
- [docs/features_catalog.md](docs/features_catalog.md) — per-module algorithm and config-knob catalog
- [.github/instructions/cpp-style-openmvs.instructions.md](.github/instructions/cpp-style-openmvs.instructions.md) — C++ style reference
- Per-library READMEs: [libs/Common/README.md](libs/Common/README.md), [libs/SFM/README.md](libs/SFM/README.md), [libs/MVS/README.md](libs/MVS/README.md)
