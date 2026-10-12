# Native curve memory: total-cost selection and process measurements

These measurements describe v2 at code commit `1480df2`. The follow-up
[native ARP cache and parametric curves](CURVE_MEMORY_ARP.md) adds optional
adaptive cache admission and retained curve definitions with separate validation.

This is the C++ continuation of the [shared CMA experiment](https://chatgpt.com/share/6acaf7c9-aa78-83e8-99c4-b38956f1da89).
The actual repository is [RDM3DC/CADadaptive2.5](https://github.com/RDM3DC/CADadaptive2.5).
The review branch is `codex/curve-memory-v2`. The original Python ZIP was not
available here; no original Python measurements are treated as native results.

Shared shapes, per-instance affine matrices, sparse local edits, and relationships
remain in place. Sampled coordinates retain every binary64 bit, including signed
zero and NaN payloads. Matrix transformations use ordinary floating-point
arithmetic. There is no new external CAD kernel and no learned curve alphabet.

## Storage policy

`--storage auto` first compares each curve's direct raw point vector with XOR
blocks. The comparison includes curve and block metadata, allocation capacities,
and the full reachable requested decoded cache. Compression must save at least
5% of that total. The scene adapter then applies the same 5% threshold to the
**complete imported scene**, including faces, indices, materials, instance data,
and other metadata. If the whole-scene saving is too small, its bank uses raw
vectors. This prevents a small point-payload saving inside a large topological
mesh from triggering expensive decompression.

`--storage raw` forces direct `Point3D` vectors while retaining exactly the same
sharing, matrices, edits, and scene container. It is the process comparison's
baseline. The separate `ordinary_shared_scene` estimate describes a hypothetical
minimal dense-vector container; `raw_policy_scene` describes the actual forced-raw
implementation and is the relevant estimate for raw/auto comparisons.

Selection occurs during ingestion. Adding placements later does not re-encode
shapes. Cache size is a bank-wide upper bound, limited by the number of compressed
blocks. Unlike v1, the cache is not throttled to manufacture a favorable storage
comparison. An individual cache miss decodes a block of up to 256 points. Raw
reads directly load a point without reconstructing bytes.

## Batched reads

`CurveMemoryBank::read_points(curve, indices, output)` accepts arbitrary order
and repeated indices. It validates indices/output length, groups requests by
block, and decodes each touched block once even with cache disabled. Raw batches
need no codec scratch allocation. Compressed batches use an ordinal array and
one transient decoded block, in addition to any retained cache.

`CurveMemoryScene::points(instance, indices, output)` applies local edits and the
instance matrix to those results. `points(instance)` returns an entire instance.
Materialization uses full batches for existing geometry build/export consumers.

The command benchmarks individual random/sequential reads and equivalent batches.
Both use identical requests and checksums. Reusable batch plans/output buffers
are prepared before timing for both baselines; internal block sorting, decoding,
scatter, and checksum work are included in the read time. A warm-start pass is
not a promise of cache hits when the working set exceeds the cache.

## Build and run

From the repository root in PowerShell:

```powershell
cmake -S . -B build-curve-memory -G Ninja -DCMAKE_BUILD_TYPE=Release -DADAPTIVECAD_ENABLE_OPENCASCADE=OFF
cmake --build build-curve-memory --target adaptivecad_cli adaptivecad_curve_memory_bank_tests adaptivecad_curve_memory_scene_tests
ctest --test-dir build-curve-memory -R curve_memory --output-on-failure
.\build-curve-memory\runbin\adaptivecad.exe --curve-memory-demo --instances 128 --report-json build-curve-memory\scene-validation.json
.\build-curve-memory\runbin\adaptivecad.exe --curve-memory samples\simple_wedge.stl --storage auto --output build-curve-memory\restored.obj
```

Process RAM measurements use a separate command and Python standard-library
runner. Each raw/auto case gets a fresh process; execution order alternates, and
three repetitions report median, minimum, and maximum. This example exercises
262,144 and 1,048,576 unique vertices with matching torus faces, plus four million
synthetic curve samples:

```powershell
python scripts\benchmark_curve_memory_process.py --exe build-curve-memory\runbin\adaptivecad.exe --output build-curve-memory\process-validation.json --torus-sizes 1024 2048 --dense-points 4194304 --repeats 3
```

Add `--input path\to\model.stl` to include a saved CAD model. The local Benchy
model is not distributed on this branch. `--instances N` adds shared placements;
this does not increase unique geometry and is reported separately.

The native worker can also run alone:

```powershell
.\build-curve-memory\runbin\adaptivecad.exe --curve-memory-process --storage auto --model torus --size 1024 --instances 1 --report-json build-curve-memory\one-process.json
```

## Interpreting process RAM

Windows snapshots come from `GetProcessMemoryInfo`: working-set bytes measure
resident RAM, and private committed bytes measure the process's private committed
memory, which need not all be resident. Linux uses `/proc/self/status` for RSS and
high-water RSS; private commitment is unavailable there. Unsupported platforms
fail explicitly instead of inventing a memory number.

Snapshots distinguish the source alone, conversion with the source alive, source
released, query buffers allocated, and the completed read workloads. The source
object is destroyed before `source_released`; allocator-retained free memory still
counts. Peak counters include import/conversion overlap and all earlier peaks.
No working-set eviction or heap trimming is forced. Both policies retain identical
query plans and outputs. Check the min/max spread before treating a small median
difference as a real saving. These measurements concern the native process
workload, not a running GUI renderer.

Dense generated curves are a synthetic cubic fixture with dyadic sampling and no
faces. Their result does not predict Benchy or general CAD-mesh performance. The
original 36–38% Python figure is neither a native baseline nor an acceptance target.

## Measured results

The uploaded [v2 validation JSON](../validation/curve-memory/v2-validation.json)
contains all 24 isolated process runs (three per policy and fixture), min/max
ranges, exact checksum comparisons, full Benchy recovery, and a 128-placement
scene check. These Windows Release results were collected on 2026-10-11 UTC.
Resident RAM below is the process working set after releasing the source and
before allocating query plans; MiB means 1,048,576 bytes.

| Fixture | Unique points | Auto selection | Raw RAM (MiB) | Auto RAM (MiB) | Raw / auto peak RAM (MiB) |
| --- | ---: | --- | ---: | ---: | ---: |
| Imported Benchy | 60,058 | Raw | 22.16 | 22.13 | 65.94 / 65.93 |
| Native torus mesh | 262,144 | Raw | 49.72 | 49.64 | 137.13 / 137.13 |
| Native torus mesh | 1,048,576 | Raw | 183.70 | 183.82 | 534.34 / 534.35 |
| Synthetic cubic curves, no faces | 4,194,304 | Compressed | 100.88 | 44.48 | 196.43 / 140.02 |

Benchy and both torus meshes have the same raw representation under either
policy; their small RAM differences do not establish a saving. Benchy's warm
individual random workload took a median 0.1550 ms forced raw versus 0.1547 ms
auto, and its auto batch workload took 0.0437 ms. Each workload reads 8,192 points.

The synthetic curve fixture reduced source-released resident RAM by 55.9%, but
compressed random reads remain expensive: 13.2569 ms individual versus 9.8330 ms
batched, compared with raw 0.2248 ms individual and 0.0720 ms batched. Batching
reduced compressed random time by 25.8% in these runs. This result does not
predict imported CAD savings. Peak figures include source construction and
conversion overlap; they are not the retained scene footprint.

The remaining codec bottleneck is block reconstruction for isolated requests.
Each compressed cache miss reconstructs up to 256 points to return one point;
an 8,192-point random workload far exceeds the four-block cache. Batch grouping
avoids decoding the same block repeatedly within a batch, but sparse requests
still pay for unused points. The measured timings include sorting and checksum
work; this is a code-path diagnosis, not a sampled CPU profile. Explicit raw
storage remains available for latency-sensitive workloads.

## Persistence and validation scope

The checksummed v1 bank archive remains readable and preserves coordinate bits.
Loading reapplies the requested storage policy. Bank persistence does not encode
the complete scene: matrices, topology, edits, and relationships still require a
scene format. Existing scene export/rendering expands to `SceneDocument`.

New tests cover total-cost decisions, direct raw fallback, exact binary recovery,
malformed archives, bounded caches, reordered/repeated batch indices, edits,
matrices, metadata, native build/export, and isolated-process command smoke tests.

The original validation JSON is retained as `validation/curve-memory/v1-validation.json`.
It describes the previous implementation and is not relabeled as a v2 result.
The v2 JSON records and source manifest in that directory identify the measured
implementation. Local absolute paths are replaced with repository-relative paths
or input filenames before upload; numeric measurements and checksums are retained.

The review branch starts from published `main` at `86f5a46` and includes only
this prototype and its integration. Other unpublished civil/UI/geometry work in
the local workspace is outside this branch. Its standalone Release build passed
all 18 CTests with assertions enabled; the updated local workspace also passed
all six curve-memory tests. The measured executable was built from the review
branch, whose code commit and SHA-256 source manifest are included in the JSON.
