# Compiling SHOT

SHOT compiles on **Linux and macOS**. This guide covers getting the source,
installing dependencies, configuring with CMake, building, and running the
test suite. It's derived directly from the root `CMakeLists.txt`,
`test/CMakeLists.txt`, and the CI recipe in
`.github/workflows/build-workflow.yml`, so it should stay accurate as long
as those are the source of truth — if a command here stops working, check
those files first.

> **Windows is no longer supported.** SHOT previously could be
> cross-compiled for Windows from Linux using mingw (see
> `misc/toolchain-mingw.cmake` if you want to attempt reviving it), but this
> is unmaintained and untested — don't rely on it. If you want to use SHOT on Windows, consider using Windows Subsystem for Linux (WSL). 

## 1. Getting the source

```bash
git clone https://github.com/coin-or/SHOT.git
cd SHOT
git submodule update --init --recursive
```

Only 5 of the directories under `ThirdParty/` are actual git submodules
(per `.gitmodules`): `spdlog`, `CppAD`, `pybind11`, `HiGHS`, `eigen`. The
rest (`ampl`, `argh`, `boost` — a trimmed header subset, not full Boost —
`mc++`, `tinyxml2`, `nlohmann`) are committed directly in the main repo, so
there's nothing extra to fetch for those.

**Note on HiGHS**: HiGHS is **not always included** — it's controlled like
any other solver by the `HAS_HIGHS` option (default `ON`, see section 3).
When it's on (the default), CMake compiles HiGHS from source from the
local `ThirdParty/HiGHS` submodule, so run
`git submodule update --init --recursive` first. If that submodule
directory is empty/missing while `HAS_HIGHS=on`, CMake fails the configure
with a clear error telling you to initialize it. If you don't need HiGHS,
configure with `-DHAS_HIGHS=off` to skip it entirely (and skip
initializing that submodule) — this also noticeably shortens the build,
since HiGHS is compiled from scratch. To use a pre-built HiGHS instead of
compiling the submodule, configure with `-DUSE_EXTERNAL_HIGHS=on
-DHIGHS_EXTERNAL_DIR=<path>`.

## 2. Installing dependencies

SHOT needs at least one MIP solver (Cbc, CPLEX, Gurobi, or HiGHS) and an NLP
solver (Ipopt or GAMS) to be useful. 

Every optional dependency degrades gracefully: if
CMake can't find it, the corresponding `HAS_*` option is silently forced
back to `OFF` with a `message(WARNING ...)` rather than failing the
configure — always check the CMake output for
`"<solver> not found, setting HAS_<solver> to OFF"` lines to see what you
actually got, especially if `./SHOT --help` doesn't list a solver you
expected.

### Linux

Matches the packages installed in CI
(`.github/workflows/build-workflow.yml`):

```bash
sudo apt-get update
sudo apt-get install -y libc6-dev libbz2-dev zlib1g-dev liblapack-dev \
    libnauty2-dev libopenblas-base libopenblas-dev libmumps-dev pkgconf
```

If your distro's packaged Cbc/Ipopt are too old or lack pkg-config files,
build them from source with [coinbrew](https://github.com/coin-or/coinbrew):

```bash
git clone https://github.com/coin-or/coinbrew
cd coinbrew
./coinbrew build Ipopt --prefix=<SHOT>/ThirdParty/Ipopt --no-prompt --tests none
./coinbrew build Cbc --prefix=<SHOT>/ThirdParty/Cbc --no-prompt --tests none --no-third-party
```

then point CMake at them with `-DCBC_DIR=<SHOT>/ThirdParty/Cbc
-DIPOPT_DIR=<SHOT>/ThirdParty/Ipopt` (see section 3).

### macOS (Homebrew)

```bash
brew install cbc ipopt openblas
```

CMake locates Cbc and Ipopt automatically via `pkg-config` — no
`-DCBC_DIR`/`-DIPOPT_DIR` needed once they're installed this way.

**HiGHS via Homebrew (alternative to the submodule)**: `brew install highs`
also works — Homebrew's package ships proper CMake config files, so it's
usable as an "external HiGHS" (skips compiling HiGHS from source, which is
otherwise the slowest part of a default build):

```bash
brew install highs
cmake -S . -B build -DUSE_EXTERNAL_HIGHS=on -DHIGHS_EXTERNAL_DIR=/opt/homebrew/opt/highs
```

If Homebrew's installed version is old, run `brew upgrade highs` first —
older HiGHS releases may lack APIs `MIPSolverHighs.cpp` relies on.

### Proprietary solvers (GAMS, CPLEX, Gurobi) — either platform

These all require your own commercial (or free/academic) license to run,
but SHOT only needs their SDK/headers+libraries at compile time:

- **GAMS**: point CMake at the install with `-DGAMS_DIR=<path>` (default
  `/opt/gams`). No auto-detection.
- **CPLEX**: auto-detected (`misc/FindCPLEX.cmake`) under
  `/opt/ibm/ILOG`, `/opt/IBM/ILOG`, `$HOME/Applications/IBM/ILOG`, and
  `$HOME/Applications` (this is how a Mac install under
  `~/Applications/CPLEX_Studio<version>` gets found); override with
  `-DCPLEX_DIR=<path>/cplex` if installed elsewhere.
- **Gurobi**: auto-detected (`misc/FindGUROBI.cmake`) under
  `/opt/gurobi*/linux64` (Linux) or `/Library/gurobi*/mac64`,
  `/Library/gurobi*/macos_universal2`, `/Library/gurobi*/macos_arm64`
  (macOS) — these are Gurobi's own standard installer locations; override
  with `-DGUROBI_DIR=<path>` if installed elsewhere.

If you don't have one of these, just turn it off explicitly (see below) —
don't leave it on `ON` (the default) and hope it's silently skipped, since
a partial/broken install can still be *found* but fail later in the build.

## 3. Configuring with CMake

Key options (all `option(...)` declarations live at the top of the root
`CMakeLists.txt`):

| Option | Default | Meaning |
| --- | --- | --- |
| `HAS_CBC` | `ON` | Cbc MIP solver |
| `HAS_CPLEX` | `ON` | CPLEX MIP solver |
| `HAS_GUROBI` | `ON` | Gurobi MIP solver |
| `HAS_HIGHS` | `ON` | HiGHS MIP solver — like the others, optional; turn off with `-DHAS_HIGHS=off` (see section 1 re: `USE_EXTERNAL_HIGHS`) |
| `HAS_IPOPT` | `ON` | Ipopt NLP solver |
| `HAS_GAMS` | `ON` | GAMS interface (modeling system + NLP solver) |
| `HAS_AMPL` | `ON` | AMPL/ASL `.nl` file interface |
| `HAS_PYTHON` | `OFF` | Build the `SHOTpy` Python bindings (needs Python3 dev headers; pybind11 comes from the submodule) |
| `COMPILE_TESTS` | `OFF` | Build the `test_runner` test suite (section 5) |
| `GENERATE_EXE` | `ON` | Build the `./SHOT` console executable |
| `BUILD_DOCUMENTATION` | on if Doxygen found | Build the HTML API docs |

`<SOLVER>_DIR` cache variables (`CBC_DIR`, `CPLEX_DIR`, `GUROBI_DIR`,
`IPOPT_DIR`, `GAMS_DIR`, `HIGHS_EXTERNAL_DIR`) override where each
dependency is searched for, as described in section 2.

Also set `-DCMAKE_BUILD_TYPE=<Release|Debug|RelWithDebInfo>` explicitly —
it isn't set by default. Note that leaving it unset still disables
assertions (`-DNDEBUG` is added whenever `CMAKE_BUILD_TYPE` is not
`Debug`, and an *unset* build type is "not Debug" too), but you also won't
get `-O2`-style optimization flags either way, so an unset build type is
the worst of both — always pass it explicitly.

Minimal open-source-only build (Cbc + Ipopt + HiGHS, no proprietary
solvers):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DHAS_GAMS=off -DHAS_CPLEX=off -DHAS_GUROBI=off
```

Matching CI's Linux configure (Cbc/Ipopt built via coinbrew into
`ThirdParty/`, tests on, GAMS/Gurobi/CPLEX off):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DHAS_CBC=on -DCBC_DIR=ThirdParty/Cbc \
  -DHAS_IPOPT=on -DIPOPT_DIR=ThirdParty/Ipopt \
  -DHAS_GAMS=off -DHAS_GUROBI=off -DHAS_CPLEX=off \
  -DCOMPILE_TESTS=on
```

## 4. Building

```bash
cmake --build build -j<N>
```

(or `cd build && make -j<N>` for the default Unix Makefiles generator).
This produces the `./SHOT` console executable and `libSHOTSolver` inside
`build/` (plus `SHOTpy` if `HAS_PYTHON=on`). A full build including
compiling HiGHS from source (`HAS_HIGHS=on`, the default) can take several
minutes; pass `-DHAS_HIGHS=off` at configure time if you don't need it and
want a faster build.

## 5. Running the tests

Tests must be enabled at configure time, i.e. add `-DCOMPILE_TESTS=on` to
whichever `cmake` configure command from section 3 you used, then rebuild
and run from the build directory:

```bash
cmake --build build -j<N>
cd build
ctest --output-on-failure
```

This is genuinely useful when debugging, not just for CI: individual test
cases can be run directly (`./test/test_runner <Name>test <part>`), and
`InstanceTest` solves a batch of real problem instances against known
expected objectives — a fast way to check whether a change regressed
solver behavior. For the full breakdown of test structure, how to run a
single failing test in isolation, and how to use `InstanceTest` results to
drive a `--debug` investigation, see
[docs/DebuggingSHOT.md](DebuggingSHOT.md), section 7.

## 6. Next steps

- Running the compiled solver: [docs/CommandLineUsage.md](CommandLineUsage.md).
- Debugging solver behavior: [docs/DebuggingSHOT.md](DebuggingSHOT.md).
