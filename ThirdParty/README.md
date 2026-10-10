# Vendored dependencies

`versions.json` records the release source, checksum or submodule commit, and
license for each dependency. Release builds use these fixed inputs. Update
the manifest and run the C++/Python tests whenever an input changes.

The five Git submodules remain pinned by the parent repository. Initialize
them with `git submodule update --init --recursive`; do not use `--remote`
when reproducing a release.

## Boost

Download the archive URL recorded in `versions.json`, then run:

```sh
bash misc/vendor-boost.sh /path/to/boost_1_92_0.tar.bz2
```

The script verifies the official archive SHA256, builds that release's `bcp`,
and replaces the header subset including transitive dependencies. The seed
headers cover SHOT's hashing, root finding, scalar minimization, integer
types, and version identification. No Boost binaries are linked. This update
does not change SHOT to Boost interval arithmetic.

## AMPL MP and MC++

Download the respective pinned archives from `versions.json`, then run:

```sh
python3 misc/vendor-reader-dependencies.py ampl /path/to/mp-4.1.0.tar.gz
python3 misc/vendor-reader-dependencies.py mc++ /path/to/mcpp-5.0.4.tar.gz
```

The importer verifies the checksum before replacing either directory. MP
contains the reader's source/header dependency closure. Its upstream
`gen-expr-info.cc` is a generator executable and is intentionally excluded;
the upstream generated `expr-info.cc` is included instead. MP retains its
own bundled formatting implementation; it is separate from spdlog's fmt.
The importer preserves SHOT's `fmtold` namespace and `FMTOLD_` macro prefix
patch so both formatting implementations can be included in the same source
file. MP's bundled small_vector header and MIT license are retained as well.

MC++ contains `interval.hpp`, `mcfunc.hpp`, and `mcop.hpp`. SHOT does not use
MC++'s relaxation, polynomial, linear algebra, or automatic differentiation
facilities. The old unused cpplapack/FADBAD++ trees were removed. Optional
`MC__USE_FADBAD` is not supported by this subset. MC++ interval arithmetic
remains non-verified (it does not account for rounding error).

## Other vendored files

JSON's single header is copied verbatim from the manifest URL, with its MIT
license. argh 1.3.2 and TinyXML2 11.0.0 match their upstream release sources
after CRLF-to-LF normalization. Keep their accompanying licenses. spdlog
1.17.0 carries its own fmt 12.1.0; do not replace fmt independently.

## External solvers

Cbc and Ipopt release pins apply to the coinbrew CI recipe. Distribution,
Homebrew, and Conda installations can have different versions and must be
reported in validation results. The manifest's Uno version is a compatibility
target, not a bundled installation. Commercial solver SDKs and licenses are
provided by the build environment.
