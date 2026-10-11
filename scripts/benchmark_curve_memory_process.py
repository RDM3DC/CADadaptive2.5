#!/usr/bin/env python3
"""Compare raw/automatic native scenes in independent child processes.

Only the standard library is required. Each child releases its source document
before sampling steady process memory. Parent RSS is never used as model RAM.
"""

import argparse
import datetime
import hashlib
import json
import platform
import statistics
import subprocess
import sys
from pathlib import Path


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True, help="Native adaptivecad executable")
    parser.add_argument("--output", type=Path, required=True, help="Aggregate JSON with every child report")
    parser.add_argument("--models", nargs="+", choices=("torus", "dense-curves"),
                        default=["torus", "dense-curves"])
    parser.add_argument("--torus-sizes", nargs="+", type=int, default=[1024],
                        help="Unique torus mesh major segments (minor segments=max(8,size/4))")
    parser.add_argument("--dense-points", type=int, default=1048576)
    parser.add_argument("--input", type=Path, action="append", default=[],
                        help="Optional imported CAD input; may be repeated")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--instances", type=int, default=1,
                        help="Matrix placement copies; does not increase unique geometry")
    parser.add_argument("--cache-blocks", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=300,
                        help="Maximum seconds per isolated child")
    options = parser.parse_args()
    if not options.exe.is_file():
        parser.error("--exe must name an existing native executable")
    if options.exe.resolve() == options.output.resolve():
        parser.error("--output must differ from the executable")
    if not 1 <= options.repeats <= 20:
        parser.error("--repeats must be between 1 and 20")
    if not 1 <= options.instances <= 256:
        parser.error("--instances must be between 1 and 256")
    if any(size < 32 or size > 2048 for size in options.torus_sizes):
        parser.error("--torus-sizes must be between 32 and 2048")
    if not 32 <= options.dense_points <= 8 * 1024 * 1024:
        parser.error("--dense-points must be between 32 and 8388608")
    if not 0 <= options.cache_blocks <= 4096:
        parser.error("--cache-blocks must be between 0 and 4096")
    if options.timeout <= 0:
        parser.error("--timeout must be positive")
    for source in options.input:
        if not source.is_file():
            parser.error(f"input does not exist: {source}")
        if source.resolve() == options.output.resolve():
            parser.error("--output must differ from every --input")
    return options


def summary(values):
    values = [value for value in values if value is not None]
    if not values:
        return None
    return {"median": statistics.median(values), "min": min(values), "max": max(values)}


def aggregate(runs):
    first = runs[0]
    result = {"process_memory": {}, "timings_ms": {}, "estimated_capacity_bytes": {},
              "construction_ms": summary([run["construction_ms"] for run in runs])}
    for memory in first["process_memory"]:
        phase = memory["phase"]
        samples = [next(item for item in run["process_memory"] if item["phase"] == phase)
                   for run in runs]
        result["process_memory"][phase] = {
            field: summary([sample[field] for sample in samples])
            for field in ("resident_bytes", "peak_resident_bytes", "private_committed_bytes",
                          "peak_pagefile_bytes")
        }
    for key in first["timings"]:
        result["timings_ms"][key] = summary([run["timings"][key]["ms"] for run in runs])
    for key in first["estimated_capacity_bytes"]:
        result["estimated_capacity_bytes"][key] = summary(
            [run["estimated_capacity_bytes"][key] for run in runs])
    return result


def savings(raw, auto):
    if raw is None or auto is None or not raw["median"]:
        return None
    return 100.0 * (raw["median"] - auto["median"]) / raw["median"]


def verify(runs):
    expected = runs["raw"][0]
    for policy, reports in runs.items():
        for report in reports:
            for field in ("unique_source_points", "source_faces", "scene_instances", "unique_shapes"):
                if report[field] != expected[field]:
                    raise RuntimeError(f"raw/{policy} scene mismatch in {field}")
            if not report["individual_batch_exact"]:
                raise RuntimeError("native individual/batch verification failed")
            for name, timing in expected["timings"].items():
                actual = report["timings"][name]
                if (actual["checksum_hex"], actual["queries"]) != (timing["checksum_hex"], timing["queries"]):
                    raise RuntimeError(f"raw/{policy} checksum or query count mismatch: {name}")


def main():
    options = arguments()
    executable = options.exe.resolve()
    configurations = []
    for model in options.models:
        if model == "torus":
            configurations.extend((f"torus-{size}", ["--model", model, "--size", str(size)])
                                  for size in options.torus_sizes)
        else:
            configurations.append((f"dense-curves-{options.dense_points}",
                                   ["--model", model, "--size", str(options.dense_points)]))
    configurations.extend((f"input-{index}-{source.stem}",
                           ["--model", "input", "--input", str(source.resolve())])
                          for index, source in enumerate(options.input))
    output = {
        "schema": "adaptivecad.curve-memory-process-comparison.v1",
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "host_platform": platform.platform(),
        "host_machine": platform.machine(),
        "executable": str(executable),
        "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "repeats_per_policy": options.repeats,
        "measurement_note": "Each raw/auto run uses a new native process with identical shapes, topology, placements and deterministic queries. Policies alternate first-run order each repeat. Counter snapshots measure actual child process RAM; no source document remains at source_released. Allocator-retained memory remains counted. Peaks include construction. Small differences within min/max spread are not proven savings.",
        "baseline_note": "Raw policy uses shared shapes and the same scene metadata/matrix placement with contiguous binary64 point arrays. This compares codec/storage selection costs, not sharing against fully duplicated geometry. Dense generated curves are a synthetic fixture and do not predict imported CAD results.",
        "configurations": [],
    }
    for label, config in configurations:
        runs = {"raw": [], "auto": []}
        commands = []
        for repeat in range(options.repeats):
            for policy in (("raw", "auto") if repeat % 2 == 0 else ("auto", "raw")):
                command = [str(executable), "--curve-memory-process", "--storage", policy,
                           *config, "--instances", str(options.instances),
                           "--cache-blocks", str(options.cache_blocks)]
                print(f"{label}: {policy}, repeat {repeat + 1}/{options.repeats}", flush=True)
                completed = subprocess.run(command, capture_output=True, text=True,
                                           timeout=options.timeout, check=False)
                if completed.returncode:
                    raise RuntimeError(f"child failed ({completed.returncode}): {command!r}\n"
                                       f"{completed.stdout[-4000:]}\n{completed.stderr[-4000:]}")
                try:
                    report = json.loads(completed.stdout)
                except json.JSONDecodeError as error:
                    raise RuntimeError(f"child did not return a JSON report: {completed.stdout[-4000:]}") from error
                if report.get("schema") != "adaptivecad.curve-memory-process.v1":
                    raise RuntimeError("unexpected child report schema")
                if report.get("storage_policy") != policy:
                    raise RuntimeError("child used unexpected storage policy")
                runs[policy].append(report)
                commands.append(command)
        verify(runs)
        raw, auto = aggregate(runs["raw"]), aggregate(runs["auto"])
        comparison = {"label": label, "raw_auto_checksums_exact": True,
                      "raw_summary": raw, "auto_summary": auto,
                      "auto_reduction_percent": {}, "child_commands": commands, "runs": runs}
        for phase in ("source_released", "after_individual_reads", "after_batch_reads"):
            comparison["auto_reduction_percent"][phase] = {
                field: savings(raw["process_memory"][phase][field], auto["process_memory"][phase][field])
                for field in ("resident_bytes", "private_committed_bytes")
            }
        output["configurations"].append(comparison)
        print(f"{label}: checksums exact; steady RSS reduction "
              f"{comparison['auto_reduction_percent']['source_released']['resident_bytes']:.3f}%", flush=True)
        # Save completed configurations as they finish. A later failure does not
        # discard measurements already gathered, and complete remains false.
        output["complete"] = False
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    output["complete"] = True
    options.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {options.output.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"Process benchmark failed: {error}", file=sys.stderr)
        raise SystemExit(1)
