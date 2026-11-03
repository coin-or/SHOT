#!/usr/bin/env python3
"""Reproduce the AMPL MP or MC++ subset from a checksum-pinned archive.

Usage: python3 scripts/vendor-reader-dependencies.py {ampl,mc++} archive.tar.gz
Downloads are deliberately separate; source URLs/checksums are in versions.json.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


# MC++'s interval cosine and tangent shift the lower bound into the first period
# with an int multiple of the period, which overflows for bounds beyond about
# 1e9, and the shift loses the position in the period for large bounds anyway.
MCPP_INTERVAL_PATCHES = [
    ("""inline Interval
cos(Interval const& I)
{
""", """inline Interval
cos(Interval const& I)
{
  // A period or more contains the whole range, and for large bounds, k does not
  // fit in an int and the shift by 2*k*pi loses the position in the period
  if (I._u - I._l >= 2. * PI || std::fabs(I._l) > 1e9 || std::fabs(I._u) > 1e9)
    return Interval(-1., 1.);
"""),
    ("""inline Interval
tan(Interval const& I)
{
""", """inline Interval
tan(Interval const& I)
{
  // For large bounds, k does not fit in an int and the shift by k*pi loses the
  // position in the period, so whether the interval contains a pole is unknown
  if (std::fabs(I._l) > 1e9 || std::fabs(I._u) > 1e9)
    throw Interval::Exceptions(Interval::Exceptions::TAN);
"""),
]


def patch_mcpp_interval(text):
    for old, new in MCPP_INTERVAL_PATCHES:
        if text.count(old) != 1:
            raise SystemExit("MC++ interval.hpp does not match SHOT's patch: " + old.splitlines()[0])
        text = text.replace(old, new)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dependency", choices=["ampl", "mc++"])
    parser.add_argument("archive", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / "ThirdParty/versions.json").read_text())
    expected = manifest["dependencies"][args.dependency]["source_sha256"]
    if hashlib.sha256(args.archive.read_bytes()).hexdigest() != expected:
        parser.error("archive checksum does not match ThirdParty/versions.json")
    with tempfile.TemporaryDirectory() as temporary:
        work = Path(temporary)
        # Only the exact trusted upstream archive passes the checksum above.
        subprocess.run(["tar", "-xzf", str(args.archive.resolve()), "-C", str(work)], check=True)
        source, = work.iterdir()
        stage = work / "vendor"
        stage.mkdir()
        if args.dependency == "mc++":
            (stage / "include").mkdir()
            for name in ("interval.hpp", "mcfunc.hpp", "mcop.hpp"):
                shutil.copyfile(source / "src/mc" / name, stage / "include" / name)
            shutil.copyfile(source / "LICENSE", stage / "LICENSE")
            shutil.copytree(source / "LICENSES", stage / "LICENSES")
            interval = stage / "include/interval.hpp"
            interval.write_text(patch_mcpp_interval(interval.read_text()))
        else:
            seeds = ["src/" + name for name in (
                "expr-info.cc", "format.cc", "nl-reader.cc", "posix.cc",
                "problem.cc", "os.cc", "expr.cc", "sol.cc")]
            seeds += ["include/mp/" + name for name in (
                "nl.h", "problem.h", "nl-reader.h", "sol.h")]
            pending = [source / name for name in seeds]
            seen = set()
            while pending:
                path = pending.pop()
                if path in seen:
                    continue
                seen.add(path)
                target = stage / path.relative_to(source)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(path, target)
                for header in re.findall(r'^\s*#\s*include\s*[<"]([^">]+)[">]', path.read_text(), re.M):
                    for candidate in (path.parent / header, source / "include" / header, source / "src" / header):
                        if candidate.is_file():
                            pending.append(candidate)
                            break
            shutil.copyfile(source / "LICENSE.rst", stage / "LICENSE.rst")
            license_path = Path("thirdparty/gharveymn/small_vector/LICENSE")
            shutil.copyfile(source / license_path, stage / license_path)
            # Preserve SHOT's isolation of MP's legacy fmt from spdlog's fmt.
            # Rename both the namespace and macro/header-guard prefix.
            for path in stage.rglob("*"):
                if path.suffix in (".h", ".cc"):
                    text = re.sub(r"\bfmt\b", "fmtold", path.read_text())
                    text = re.sub(r"\bFMT_", "FMTOLD_", text)
                    path.write_text(text)
        destination = root / "ThirdParty" / args.dependency
        shutil.rmtree(destination)
        shutil.copytree(stage, destination)


if __name__ == "__main__":
    main()
