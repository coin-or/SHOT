#!/usr/bin/env python3
"""Reproduce the AMPL MP or MC++ subset from a checksum-pinned archive.

Usage: python3 misc/vendor-reader-dependencies.py {ampl,mc++} archive.tar.gz
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
