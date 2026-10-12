#!/usr/bin/env python3
"""Compare exact native LRU/ARP cache workloads in fresh child processes.

Each process uses one policy, byte budget and ordered trace. Both access modes
preserve edit/read chronology. Synthetic sampled/parametric models use identical
definitions; their results must not be generalized to imported CAD.
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


def same_file(first, second):
    return first.resolve() == second.resolve() or (
        first.exists() and second.exists() and first.samefile(second))


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--models", nargs="*", choices=("sampled-curves", "parametric-curves"),
                        default=["sampled-curves"])
    parser.add_argument("--input", type=Path, action="append", default=[])
    parser.add_argument("--size", type=int, default=1048576,
                        help="Total generated samples, shared by sampled/parametric definitions")
    parser.add_argument("--queries", type=int, default=65536)
    parser.add_argument("--budgets", nargs="+", type=int, default=[65536, 262144],
                        help="Total decoded cache bytes, including policy metadata")
    parser.add_argument("--traces", nargs="+", choices=("mixed", "shift", "random", "scan", "hot"),
                        default=["mixed", "shift", "random"])
    parser.add_argument("--access", nargs="+", choices=("individual", "batch"),
                        default=["individual", "batch"])
    parser.add_argument("--selection", choices=("lru", "policy"), default="lru")
    parser.add_argument("--include-raw", action="store_true",
                        help="Add forced raw controls for sampled/imported models")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=300)
    result = parser.parse_args()
    if not result.exe.is_file():
        parser.error("--exe must name an existing executable")
    if same_file(result.exe, result.output):
        parser.error("--output must differ from the executable")
    if not result.models and not result.input:
        parser.error("select at least one generated model or input")
    if not 32 <= result.size <= 8 * 1024 * 1024:
        parser.error("--size must be 32..8388608")
    if not 1 <= result.queries <= 2 * 1024 * 1024:
        parser.error("--queries must be 1..2097152")
    if any(budget < 0 or budget > 16 * 1024 * 1024 for budget in result.budgets):
        parser.error("--budgets must each be 0..16777216")
    if not 1 <= result.repeats <= 20:
        parser.error("--repeats must be 1..20")
    if result.timeout <= 0:
        parser.error("--timeout must be positive")
    for source in result.input:
        if not source.is_file():
            parser.error(f"input does not exist: {source}")
        if same_file(source, result.output):
            parser.error("--output must differ from every input")
    # Repeated options must not accidentally turn into extra weight in medians.
    for key in ("models", "budgets", "traces", "access"):
        setattr(result, key, list(dict.fromkeys(getattr(result, key))))
    return result


def summarize(values):
    values = [value for value in values if value is not None]
    return None if not values else {
        "median": statistics.median(values), "min": min(values), "max": max(values)}


def aggregate(reports):
    first = reports[0]
    result = {key: summarize([report[key] for report in reports]) for key in (
        "read_ms", "edit_ms", "trace_elapsed_ms", "construction_ms", "trace_plan_ms")}
    for category in ("cache_metrics", "estimated_capacity_bytes", "benchmark_buffers_bytes"):
        result[category] = {key: summarize([report[category][key] for report in reports])
                            for key in first[category]}
    result["chunk_latency_ms"] = {
        key: summarize([report["chunk_latency_ms"][key] for report in reports])
        for key in ("median", "p95", "p99")}
    result["process_memory"] = {}
    for snapshot in first["process_memory"]:
        phase = snapshot["phase"]
        samples = [next(item for item in report["process_memory"] if item["phase"] == phase)
                   for report in reports]
        result["process_memory"][phase] = {
            key: summarize([sample[key] for sample in samples])
            for key in ("resident_bytes", "peak_resident_bytes", "private_committed_bytes", "peak_pagefile_bytes")}
    return result


def check_report(report, expected, policy, access, storage, options):
    if report.get("schema") != "adaptivecad.curve-memory-workload.v1":
        raise RuntimeError("unexpected child report schema")
    if (report.get("cache_policy"), report.get("access"), report.get("storage_policy")) != (
            policy, access, storage):
        raise RuntimeError("child used an unexpected cache/access/storage configuration")
    if not report.get("bit_exact_source_and_edit_replay"):
        raise RuntimeError("child did not verify every returned point against the source/edit replay")
    if report["estimated_capacity_bytes"]["cache_after_reads"] > report["cache_budget_bytes"]:
        raise RuntimeError("cache retained more than the requested total byte budget")
    if expected is not None:
        for key in ("checksum_hex", "trace_checksum_hex", "queries", "edits", "source_points", "source_faces",
                    "scene_instances", "unique_shapes"):
            if report[key] != expected[key]:
                raise RuntimeError(f"identical workload verification differs in {key}")
        if options.selection == "lru" and storage == "auto" and expected["storage_policy"] == "auto":
            for key in ("raw_curves", "compressed_curves", "raw_blocks", "compressed_blocks", "encoded_payload_bytes"):
                if report["bank_storage"][key] != expected["bank_storage"][key]:
                    raise RuntimeError(f"frozen LRU selection changed encoded storage across policies: {key}")


def checkpoint(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def main():
    options = arguments()
    executable = options.exe.resolve()
    fixtures = [(model, ["--model", model, "--size", str(options.size)]) for model in options.models]
    fixtures.extend((f"input-{index}-{source.stem}", ["--model", "input", "--input", str(source.resolve())])
                    for index, source in enumerate(options.input))
    output = {
        "schema": "adaptivecad.curve-memory-cache-comparison.v1",
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "complete": False,
        "host_platform": platform.platform(),
        "host_machine": platform.machine(),
        "executable": str(executable),
        "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "input_sha256": {str(source.resolve()): hashlib.sha256(source.read_bytes()).hexdigest()
                         for source in options.input},
        "repeats_per_configuration": options.repeats,
        "measurement_note": "Every run is a fresh process. Policy and access order rotate between repeats. All policies, budgets and access modes use identical deterministic requests and edit chronology; checksums and every returned coordinate are verified. LRU/ARP use equal total cache byte caps including metadata; effective decoded capacities may differ. Min/max are retained, and small differences within run spread do not establish improvement.",
        "timing_note": "Latency units are identical chunks of at most 512 reads, not one call versus one batch. Batch locality can change decoder lookup count. Plans and exact source checks are outside read timing. Edit times and full trace elapsed times are reported separately. No batch crosses an edit. Single traces start cold. Raw and parametric paths bypass decoded caches.",
        "memory_note": "Windows working set/private commit or Linux RSS are child process counters. Steady source_released precedes trace buffers. Peaks include construction and source overlap. No heap trim is forced. Synthetic sampled/parametric curves are the same definitions and do not predict imported CAD savings.",
        "configurations": [],
    }
    checkpoint(options.output, output)
    workload_checksums = {}
    try:
        for label, fixture in fixtures:
            model = fixture[1]
            for budget in options.budgets:
                for trace in options.traces:
                    variants = [(policy, access, "auto") for access in options.access for policy in ("lru", "arp")]
                    if options.include_raw and model != "parametric-curves":
                        variants.extend(("lru", access, "raw") for access in options.access)
                    entry = {"label": label, "cache_budget_bytes": budget, "trace": trace,
                             "complete": False, "checksums_exact": False, "runs": {}, "child_commands": []}
                    output["configurations"].append(entry)
                    expected = None
                    for repeat in range(options.repeats):
                        ordered = variants[repeat % len(variants):] + variants[:repeat % len(variants)]
                        if repeat % 2:
                            ordered.reverse()
                        for policy, access, storage in ordered:
                            variant = f"{storage if storage == 'raw' else policy}-{access}"
                            command = [str(executable), "--curve-memory-workload", *fixture,
                                       "--cache-policy", policy, "--cache-bytes", str(budget),
                                       "--storage", storage, "--selection", options.selection,
                                       "--trace", trace, "--access", access, "--queries", str(options.queries)]
                            print(f"{label}/{trace}/{budget}: {variant}, repeat {repeat + 1}/{options.repeats}", flush=True)
                            completed = subprocess.run(command, capture_output=True, text=True,
                                                       timeout=options.timeout, check=False)
                            if completed.returncode:
                                raise RuntimeError(f"child failed ({completed.returncode}): {command!r}\n"
                                                   f"{completed.stdout[-4000:]}\n{completed.stderr[-4000:]}")
                            try:
                                report = json.loads(completed.stdout)
                            except json.JSONDecodeError as error:
                                raise RuntimeError(f"child did not return JSON: {completed.stdout[-4000:]}") from error
                            check_report(report, expected, policy, access, storage, options)
                            if expected is None:
                                expected = report
                            # Generated fixtures intentionally share geometry, so compare
                            # their results too; imported inputs each have their own key.
                            identity = (label if model == "input" else "generated", trace)
                            checksum = (report["checksum_hex"], report["trace_checksum_hex"], report["queries"], report["edits"])
                            if identity in workload_checksums and workload_checksums[identity] != checksum:
                                raise RuntimeError("checksums differ across budgets or generated representations")
                            workload_checksums[identity] = checksum
                            entry["runs"].setdefault(variant, []).append(report)
                            entry["child_commands"].append(command)
                            checkpoint(options.output, output)
                    entry["summary"] = {variant: aggregate(reports) for variant, reports in entry["runs"].items()}
                    entry["arp_over_lru_read_time_ratio"] = {
                        access: entry["summary"][f"arp-{access}"]["read_ms"]["median"]
                        / entry["summary"][f"lru-{access}"]["read_ms"]["median"]
                        if entry["summary"][f"lru-{access}"]["read_ms"]["median"] else None
                        for access in options.access}
                    entry["complete"] = True
                    entry["checksums_exact"] = True
                    checkpoint(options.output, output)
        output["complete"] = True
        output["cross_budget_and_generated_model_checksums_exact"] = True
        checkpoint(options.output, output)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        output["failure"] = str(error)
        checkpoint(options.output, output)
        raise
    print(f"Saved {options.output.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"Cache benchmark failed: {error}", file=sys.stderr)
        raise SystemExit(1)
