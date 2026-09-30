#!/usr/bin/env python3
"""Run the TMP108 TLA+ specifications as a quick verification gate.

Standard library only, so this adds no pip dependency, no cargo-vet entry
and no CI install step.

    python scripts/check-tla.py              # quick profile
    python scripts/check-tla.py --deep       # add the wide-domain profile
    python scripts/check-tla.py --only Hw    # one entry, by substring
    python scripts/check-tla.py --list

The gate exits 0 only when every entry matches its declared expectation.
Note that one entry is expected to FAIL: see Tmp108Probe.cfg.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import tempfile
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import NoReturn

REPO_ROOT = Path(__file__).resolve().parent.parent
TLA_DIR = REPO_ROOT / "docs" / "tla"

OK = "ok"
VIOLATION = "violation"

QUICK = "quick"
DEEP = "deep"

TLA_DOWNLOAD = "https://github.com/tlaplus/tlaplus/releases"


@dataclass(frozen=True)
class Spec:
    """One TLC invocation."""

    name: str
    module: str
    config: str
    expect: str = OK
    profile: str = QUICK
    # For expect=VIOLATION, the invariant that MUST be the one to fail.
    # Without this an expected-violation entry would be satisfied by any
    # error at all, including a spec that no longer parses.
    invariant: str | None = None
    why: str = ""


SPECS: list[Spec] = [
    Spec(
        name="Tmp108Codec",
        module="Tmp108CodecCheck",
        config="Tmp108CodecCheck.cfg",
        why="pure arithmetic over all 65,536 words and 4,096 temperatures",
    ),
    Spec(
        name="Tmp108Hw",
        module="Tmp108Hw",
        config="Tmp108Hw.cfg",
        why="the chip alone, so a chip defect is distinguishable from a driver one",
    ),
    Spec(
        name="Tmp108Driver",
        module="Tmp108Driver",
        config="Tmp108Driver.cfg",
        why="the driver composed with the chip",
    ),
    Spec(
        name="Tmp108Probe",
        module="Tmp108Driver",
        config="Tmp108Probe.cfg",
        expect=VIOLATION,
        invariant="ProbeDetectsPristineChip",
        why="Finding 1: probe() hardcodes 0x1022; a real part resets to 0x1026",
    ),
    Spec(
        name="Tmp108DriverDeep",
        module="Tmp108Driver",
        config="Tmp108DriverDeep.cfg",
        profile=DEEP,
        why="wide domains and the real 8-poll one-shot budget",
    ),
]


@dataclass
class Result:
    spec: Spec
    passed: bool
    seconds: float
    distinct: int | None
    summary: str
    output: str = field(repr=False, default="")


# --------------------------------------------------------------------------
# Locating the jar


def find_jar(explicit: str | None) -> Path:
    # An explicit --jar is an instruction, not a hint. Falling back to the
    # default when it does not exist would silently verify against a
    # different toolchain than the one asked for.
    if explicit:
        chosen = Path(explicit).expanduser()
        if not chosen.is_file():
            die(f"--jar {chosen} does not exist")
        return chosen

    candidates: list[Path] = []
    env = os.environ.get("TLA2TOOLS_JAR")
    if env:
        candidates.append(Path(env).expanduser())
    candidates.append(Path.home() / "Downloads" / "tla2tools.jar")
    on_path = shutil.which("tla2tools.jar")
    if on_path:
        candidates.append(Path(on_path))

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    looked = "\n".join(f"    {c}" for c in candidates)
    die(
        "Could not find tla2tools.jar. Looked in:\n"
        f"{looked}\n"
        "Pass --jar PATH, set TLA2TOOLS_JAR, or download it from\n"
        f"    {TLA_DOWNLOAD}"
    )


def find_java() -> str:
    java = shutil.which("java")
    if not java:
        die("Could not find `java` on PATH. A JDK or JRE 11+ is required.")
    return java


def die(message: str) -> NoReturn:
    print(f"error: {message}", file=sys.stderr)
    raise SystemExit(2)


# --------------------------------------------------------------------------
# Running and parsing

# "17894794 states generated, 1815240 distinct states found, 0 states left"
DISTINCT_RE = re.compile(r"([\d,]+)\s+distinct states found")
VIOLATED_RE = re.compile(r"Invariant (\w+) is violated")
TEMPORAL_RE = re.compile(r"Temporal properties were violated")
SUCCESS = "Model checking completed. No error has been found."


def run_tlc(spec: Spec, java: str, jar: Path, timeout: int, verbose: bool) -> Result:
    # -metadir keeps TLC's fingerprint and state-queue files out of the
    # repository. Without it TLC drops a `states/` tree next to the specs;
    # -cleanup only clears that BEFORE a run, not after.
    with tempfile.TemporaryDirectory(prefix="tlc-") as metadir:
        cmd = [
            java,
            "-XX:+UseParallelGC",
            "-cp",
            str(jar),
            "tlc2.TLC",
            "-workers",
            "auto",
            "-cleanup",
            "-metadir",
            metadir,
            "-config",
            spec.config,
            spec.module,
        ]

        started = time.monotonic()
        try:
            proc = subprocess.run(
                cmd,
                cwd=TLA_DIR,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
            output = proc.stdout + proc.stderr
        except subprocess.TimeoutExpired:
            elapsed = time.monotonic() - started
            return Result(
                spec,
                passed=False,
                seconds=elapsed,
                distinct=None,
                summary=f"TIMEOUT after {timeout}s",
                output="",
            )

    elapsed = time.monotonic() - started
    if verbose:
        print(output)

    # TLC prints a "N distinct states found" line for every progress report
    # as well as once at the end, so take the LAST match, not the first.
    distinct = None
    found = DISTINCT_RE.findall(output)
    if found:
        distinct = int(found[-1].replace(",", ""))

    violated = VIOLATED_RE.search(output)
    temporal = TEMPORAL_RE.search(output)
    succeeded = SUCCESS in output

    if spec.expect == OK:
        if succeeded:
            return Result(spec, True, elapsed, distinct, "no error")
        if violated:
            return Result(
                spec, False, elapsed, distinct,
                f"invariant {violated.group(1)} violated", output,
            )
        if temporal:
            return Result(
                spec, False, elapsed, distinct, "temporal property violated", output
            )
        return Result(spec, False, elapsed, distinct, "TLC did not complete", output)

    # expect == VIOLATION
    if succeeded:
        return Result(
            spec, False, elapsed, distinct,
            f"expected {spec.invariant} to be violated, but the spec held",
            output,
        )
    if violated and violated.group(1) == spec.invariant:
        return Result(
            spec, True, elapsed, distinct, f"expected violation: {spec.invariant}"
        )
    if violated:
        return Result(
            spec, False, elapsed, distinct,
            f"wrong invariant failed: expected {spec.invariant}, "
            f"got {violated.group(1)}",
            output,
        )
    return Result(
        spec, False, elapsed, distinct,
        f"expected {spec.invariant} to be violated, but TLC failed some other way",
        output,
    )


# --------------------------------------------------------------------------
# Reporting


def human(n: int | None) -> str:
    return "-" if n is None else f"{n:,}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the TMP108 TLA+ specifications as a verification gate."
    )
    parser.add_argument("--jar", help="path to tla2tools.jar")
    parser.add_argument(
        "--deep", action="store_true", help="also run the wide-domain profile"
    )
    parser.add_argument("--only", help="run entries whose name contains this substring")
    parser.add_argument(
        "--timeout", type=int, default=600, help="per-spec timeout in seconds"
    )
    parser.add_argument("--verbose", action="store_true", help="stream TLC output")
    parser.add_argument(
        "--list", action="store_true", help="list the manifest and exit"
    )
    args = parser.parse_args()

    if args.list:
        for spec in SPECS:
            marker = "!" if spec.expect == VIOLATION else " "
            print(f" {marker} {spec.name:18} {spec.profile:5} {spec.config}")
            print(f"     {spec.why}")
        print("\n ! = expected to fail; see the config file for why")
        return 0

    if not TLA_DIR.is_dir():
        die(f"{TLA_DIR} does not exist")

    selected = [s for s in SPECS if args.deep or s.profile == QUICK]
    if args.only:
        needle = args.only.lower()
        selected = [s for s in selected if needle in s.name.lower()]
    if not selected:
        die("no specs selected")

    java = find_java()
    jar = find_jar(args.jar)

    profile = "quick+deep" if args.deep else "quick"
    print(f"tmp108 TLA+ gate ({profile})   jar: {jar}")
    print()

    results: list[Result] = []
    for spec in selected:
        # Only paint a progress line on a terminal; when the output is
        # captured (CI, a pipe) the carriage return would just be noise.
        if sys.stdout.isatty():
            print(f"  .... {spec.name}".ljust(72), end="\r", flush=True)
        result = run_tlc(spec, java, jar, args.timeout, args.verbose)
        results.append(result)
        status = "PASS" if result.passed else "FAIL"
        print(
            f"  {status} {spec.name:18} {result.seconds:6.1f}s "
            f"{human(result.distinct):>12} distinct  {result.summary}"
        )

    failed = [r for r in results if not r.passed]
    for result in failed:
        if not result.output:
            continue
        print()
        print("=" * 78)
        print(f"{result.spec.name}  ({result.spec.module} / {result.spec.config})")
        print("=" * 78)
        print(result.output.strip())

    total = sum(r.seconds for r in results)
    print()
    print(
        f"  {len(results) - len(failed)} passed, {len(failed)} failed "
        f"in {total:.1f}s"
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
