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
