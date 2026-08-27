#!/usr/bin/env python3
"""Generates instances.json manifests for the SHOT_benchmark_problems-derived instance subsets
(MINLP-convex, MINLP-nonconvex, MIQCQP-convex) from https://www.minlplib.org/minlplib.solu.

The .osil files themselves are committed directly under each category's folder in this directory;
this script only (re)generates the instances.json manifest that InstanceTest.cpp reads to get the
expected objective value / infeasibility status for each instance. Re-run this manually whenever the
instance files are updated.

Usage: python3 generate_manifest.py [--solu-url URL | --solu-file PATH]
"""

import argparse
import json
import sys
import urllib.request
from pathlib import Path

DEFAULT_SOLU_URL = "https://www.minlplib.org/minlplib.solu"

CATEGORIES = {
    "MINLP-convex-small": {
        "source": "SHOT_benchmark_problems",
        "url": "https://github.com/andreaslundell/SHOT_benchmark_problems",
        "description": "MINLP-convex-small category (osil format). Objectives sourced from "
        "https://www.minlplib.org/minlplib.solu.",
    },
    "MINLP-convex": {
        "source": "SHOT_benchmark_problems",
        "url": "https://github.com/andreaslundell/SHOT_benchmark_problems",
        "description": "MINLP-convex category (osil format). Objectives sourced from "
        "https://www.minlplib.org/minlplib.solu.",
    },
    "MINLP-nonconvex": {
        "source": "SHOT_benchmark_problems",
        "url": "https://github.com/andreaslundell/SHOT_benchmark_problems",
        "description": "MINLP-nonconvex category (osil format). Objectives sourced from "
        "https://www.minlplib.org/minlplib.solu.",
    },
    "MIQCQP-convex": {
        "source": "SHOT_benchmark_problems",
        "url": "https://github.com/andreaslundell/SHOT_benchmark_problems",
        "description": "MIQCQP-convex category (osil format). Objectives sourced from "
        "https://www.minlplib.org/minlplib.solu.",
    },
}

BEST_DESCRIPTION = "best known objective from minlplib.solu (not proven optimal)"


def parse_solu(text):
    """Parses minlplib.solu text into {name: {"objective": float}} or {name: {"status": "infeasible"}}."""
    results = {}

    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue

        tokens = line.split()
        status = tokens[0]
        name = tokens[1]

        if status == "=opt=":
            results[name] = {"objective": float(tokens[2])}
        elif status == "=best=":
            if name not in results:
                results[name] = {"objective": float(tokens[2]), "description": BEST_DESCRIPTION}
        elif status == "=bestdual=":
            continue  # lower-bound-only companion line, not an objective
        elif status == "=inf=":
            results[name] = {"status": "infeasible"}
        else:
            print(f"  Warning: unrecognized status keyword '{status}' on line: {line}", file=sys.stderr)

    return results


def generate_category_manifest(category_dir, solu_map):
    osil_files = sorted(p.name for p in category_dir.glob("*.osil"))

    solu_lower = {name.lower(): name for name in solu_map}

    instances = []
    unmatched = []

    for filename in osil_files:
        stem = filename[: -len(".osil")]

        name = stem if stem in solu_map else solu_lower.get(stem.lower())

        if name is None:
            unmatched.append(filename)
            continue

        entry = {"file": filename}
        entry.update(solu_map[name])
        instances.append(entry)

    manifest = dict(CATEGORIES[category_dir.name])
    manifest["instances"] = instances

    return manifest, unmatched


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--solu-url", default=DEFAULT_SOLU_URL)
    parser.add_argument("--solu-file", help="Use a local minlplib.solu file instead of fetching --solu-url")
    args = parser.parse_args()

    if args.solu_file:
        solu_text = Path(args.solu_file).read_text()
    else:
        with urllib.request.urlopen(args.solu_url) as response:
            solu_text = response.read().decode("utf-8")

    solu_map = parse_solu(solu_text)
    print(f"Parsed {len(solu_map)} entries from minlplib.solu")

    root = Path(__file__).parent

    for category in CATEGORIES:
        category_dir = root / category

        if not category_dir.is_dir():
            print(f"  Skipping '{category}': directory not found")
            continue

        manifest, unmatched = generate_category_manifest(category_dir, solu_map)

        out_path = category_dir / "instances.json"
        out_path.write_text(json.dumps(manifest, indent=4) + "\n")

        print(f"  {category}: {len(manifest['instances'])} matched, {len(unmatched)} unmatched -> {out_path}")
        if unmatched:
            for filename in unmatched:
                print(f"    no minlplib.solu match: {filename}")


if __name__ == "__main__":
    main()
