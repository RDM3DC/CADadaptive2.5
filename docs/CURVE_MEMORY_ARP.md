# Native adaptive cache and retained parametric curves

This extends the exact native scene bank with two independent features:

- An optional ARP-inspired reinforce-and-decay cache admission policy.
- Authoritative line, cubic Bezier, and circle definitions evaluated on demand.

The actual repository is [RDM3DC/CADadaptive2.5](https://github.com/RDM3DC/CADadaptive2.5),
on review branch `codex/curve-memory-arp`. This work follows the
[updated conversation](https://chatgpt.com/share/6acbfae3-ab78-83e8-a83c-de9af4f61bb3).
The Python trial's reported 28–30% mixed-workload improvement is a separate
experiment. Native measurements below must establish their own result.

## Exact data and cache policies

LRU remains the default. ARP reinforces repeatedly accessed compressed blocks
and decays older history, using bounded policy state. It decides which decoded
blocks to retain; it never changes coordinate precision. This is an adaptation
of reinforce-and-decay control, not the TurboQuant numerical quantizer.

Both policies use the same lossless point codec and 256-point block granularity.
Byte budgets include decoded point allocations, entry metadata, and retained
controller history. Controller bytes are also reported separately. A transient
decode and caller-owned query/output buffers are reported separately from the
retained cache; process RAM includes their allocation effects.

`CurveCacheOptions` selects `CurveCachePolicy::Lru` or `CurveCachePolicy::Arp`
and a `budget_bytes` limit. Existing block-count constructors continue to work.
`CurveMemoryBank::set_cache_options` and `CurveMemoryScene::configure_cache`
clear cache/history and change the policy without re-encoding stored geometry.
`clear_cache` clears retained cache/history; `reset_cache_metrics` independently
resets counters. Cache state is mutable during reads, so external synchronization
is still required when sharing a bank across threads.

Raw-selected shapes bypass the decoded cache. In particular, a raw-selected
Benchy is a control for policy overhead; it cannot demonstrate a benefit from
ARP cache admission. The automatic storage selector continues to require a
5% estimated saving including the full reachable cache and scene metadata.

## Retained definitions

`ParametricCurveDefinition` supplies `line`, `cubic_bezier`, and `circle`
factories, validation, and indexed evaluation. A definition holds its parameters
and sample count. Line and Bezier sampling include both endpoints; circle
sampling covers one revolution without duplicating the endpoint.

`CurveMemoryScene::from_parametric` retains definitions without allocating the
expanded point arrays. Shared instances, matrices, sparse point edits, indexed
batches, and explicit materialization use the same scene API as imported data.
`revise_parametric` changes the selected instance's definition independently of
other instances and clears that instance's old point overrides.

These definitions are authoritative for newly generated geometry. Imported
vertices remain stored exactly; no fitted curve silently replaces them. Numeric
evaluation uses ordinary double arithmetic. Repeated evaluation is checked
against sampled results in the same build; transcendental circle evaluation is
not a promise of identical bits across different math libraries or compiler modes.

Explicit `materialize()` expands geometry for current export/UI consumers. The
sampled bank archive remains compatible with v1; it does not serialize complete
scenes or their parametric definitions. No application-wide persistence or GUI
memory reduction is implied by the memory-engine measurements.

## Workload comparison

The dedicated `--curve-memory-workload` command exercises individual or batched
reads, with ordered edit/scan, changing-active-region, random, streaming, and hot
reuse traces. Batches must not cross edit boundaries. Both access modes preserve
the same logical requests and output order.

The cache-only comparison constructs geometry using the same LRU selection
budget, then configures the requested cache policy. This keeps encoded geometry
identical while charging ARP state against its byte budget. A separate comparison
can let each policy participate in initial storage selection. These answer
different questions: replacement efficiency and complete automatic-policy cost.

Read timings, latency distributions, hits/misses, decoded blocks, admission
rejections, cache/controller costs, and actual process RAM are reported together.
The runner starts independent processes, alternates policy order, and reports
median/minimum/maximum across repetitions. Peaks include construction and source
overlap. Allocator-retained memory remains counted; no heap trimming is forced.

Generated line/Bezier fixtures are explicitly synthetic. Results on them do not
predict imported mesh savings. Benchy remains an imported-model control.

## Build and reproduce

```powershell
cmake -S . -B build-curve-memory -G Ninja -DCMAKE_BUILD_TYPE=Release -DADAPTIVECAD_ENABLE_OPENCASCADE=OFF
cmake --build build-curve-memory --parallel 4
ctest --test-dir build-curve-memory --output-on-failure

.\build-curve-memory\runbin\adaptivecad.exe --curve-memory-workload --help
.\build-curve-memory\runbin\adaptivecad.exe --curve-memory-workload --model sampled-curves --cache-policy arp --cache-bytes 262144 --trace mixed --access batch --report-json build-curve-memory\arp-mixed.json

python scripts\benchmark_curve_memory_cache.py --exe build-curve-memory\runbin\adaptivecad.exe --output build-curve-memory\cache-comparison.json --models sampled-curves parametric-curves --size 1048576 --queries 65536 --budgets 65536 262144 --traces mixed shift random scan hot --include-raw --repeats 3
```

Use `--models --input path\to\model.stl` in the Python command to benchmark
only an imported model. Add `--selection policy` for the separate experiment
that includes each cache policy in initial storage selection. The default
`--selection lru` holds the encoded representation fixed within each byte budget.

The full report preserves every child command, executable/input SHA-256,
per-run counters, latency summaries, process memory snapshots, and median/ranges.
Its `complete` field stays false if a child fails. The runner verifies individual
and batch coordinate checksums, ordered trace/edit checksums, and matching
encoded representations in the cache-only comparison. It also compares generated
sampled and parametric outputs when both models are selected in the same run.

## Measured native result

The validation bundle records three repetitions in fresh Windows Release
processes, with LRU/ARP execution order reversed on alternating repetitions.
Generated fixtures contain 1,048,576 samples from four line/Bezier definitions.
Every trace performs 65,536 ordered reads, with 16 independent sparse edits in
the mixed and shifting-region traces. These are controlled memory-engine
experiments, not measurements of GUI navigation or large production assemblies.

For the mixed trace at a 256 KiB retained-cache cap:

| Representation | Source-released process RAM (MiB) | Retained scene estimate (bytes) | Individual reads (ms) | Batched reads (ms) |
| --- | ---: | ---: | ---: | ---: |
| Forced raw samples | 28.26 | 25,168,008 | 1.2121 | 0.4868 |
| Compressed samples, LRU | 18.29 | 13,565,642 | 94.3261 | 93.5912 |
| Compressed samples, ARP | 18.29 | 13,565,642 | 88.8784 | 92.4881 |
| Retained definitions, default LRU setting | 4.21 | 2,467 | 2.0589 | 1.6085 |

RAM and read values are medians of three runs; MiB means 1,048,576 bytes. The
scene estimate is before reads, so compressed rows have not yet allocated their
bounded decoded cache. Full-cache estimates and post-read counters are in the
JSON. Source-released process RAM includes allocator-retained memory and process
overhead. Median peak working sets, including construction/read buffers, were
52.27 MiB for raw samples, 42.31 MiB for compressed samples, and 8.10 MiB for
retained definitions. The last row bypasses the decoded cache entirely.

Retained definitions reduce source-released process RAM by about 85.1% on this
fixture. Their reads still take longer than raw array reads. The result comes
from keeping authoritative geometry definitions and does not predict savings
for arbitrary imported vertices.

ARP/LRU median read-time ratios (below 1 favors ARP):

| Trace | 64 KiB individual | 64 KiB batch | 256 KiB individual | 256 KiB batch |
| --- | ---: | ---: | ---: | ---: |
| Mixed edits/scans | 1.043 | 1.015 | 0.942 | 0.988 |
| Shifting active region | 1.477 | 1.188 | 1.338 | 1.022 |
| Random | 1.019 | 1.021 | 0.965 | 0.942 |
| Sequential scan | 1.389 | 0.963 | 1.281 | 1.097 |
| Repeated hot reads | 1.461 | 1.024 | 1.487 | 1.019 |

At 256 KiB, ARP retains 41 decoded blocks and 8,192 controller bytes; LRU retains
42 decoded blocks and no controller allocation. Their full retained cache costs
are 261,736 and 259,728 bytes respectively, both below 262,144 bytes. The mixed
individual trace decoded 52,018 blocks with ARP and 53,077 with LRU. Its 5.8%
read-time improvement is useful but limited to this measured configuration;
the 64 KiB mixed case regresses, and other individual traces regress by up to
48.7%. LRU therefore remains the default. No native 28–30% general speedup is
claimed from the earlier Python trial.

The imported Benchy control also runs the mixed trace at 256 KiB. Both cache
settings select one raw curve, allocate zero cache/controller bytes, and perform
zero decodes. Individual LRU/ARP process RAM medians were both 22.70 MiB; read
medians were 1.1910/1.2231 ms, with overlapping ranges. Batch medians were
0.4478/0.4380 ms. These differences are not an ARP gain because the cache is
bypassed. Forced-raw controls are included in the JSON.

## Published validation

- [Complete v3 validation JSON](../validation/curve-memory/v3-validation.json):
  300 generated-model runs and 18 imported-Benchy runs, all in isolated processes;
  three repetitions per configuration, individual/batch exact coordinate and
  edit-replay checks, and the full timing/RAM/counter records.
- [Source manifest](../validation/curve-memory/v3-source-manifest.json):
  code commit `f7b4d5154c58b0bf2a7b019e10d5201700010ae5`, executable SHA-256,
  and hashes for every source/header and benchmark runner.
- All 23 standalone review-branch CTests passed with optimized-build assertions
  active. The updated workspace passed all 11 curve-memory CTests. Fourteen
  invalid-option/output-clobber checks and eight requested-policy selection
  checks at 0, 1, 65,536, and 262,144-byte caps also passed.

The review branch extends the published v2 prototype. Other unpublished
civil/UI/geometry workspace work is outside this branch. Local absolute path
prefixes are removed before publication; numeric measurements and checksums are
preserved. The Benchy source model itself is not distributed. Earlier exploratory
or interrupted benchmark runs are excluded from the published evidence.
