# Compiling SHOT

SHOT compiles on **Linux and macOS**, and natively on **Windows with MSVC**
using HiGHS or Cbc, with optional Ipopt support. This guide covers getting the source,
installing dependencies, configuring with CMake, building, and running the
test suite. It's derived directly from the root `CMakeLists.txt`,
`test/CMakeLists.txt`, and the CI recipe in
`.github/workflows/build-workflow.yml`, so it should stay accurate as long
as those are the source of truth — if a command here stops working, check
those files first.

> For a native Windows build with HiGHS, see the MSVC instructions below.
> WSL remains an alternative for Linux solver dependencies.

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

Exact release pins, source checksums, and licenses are recorded in
[`ThirdParty/versions.json`](../ThirdParty/versions.json). See
[`ThirdParty/README.md`](../ThirdParty/README.md) for the vendoring procedures.
Boost is a coherent 1.92.0 header subset, and MC++ contains only its 5.0.4
interval arithmetic headers. The AMPL `.nl` reader uses selected AMPL MP 4.1.0
sources. CppAD's headers are installed into the build directory together with
its matching shared library; do not substitute a system CppAD header tree.

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
./coinbrew build Ipopt@releases/3.14.20 --prefix=<SHOT>/ThirdParty/Ipopt --no-prompt --tests none
./coinbrew build Cbc@releases/2.10.13 --prefix=<SHOT>/ThirdParty/Cbc --no-prompt --tests none --no-third-party
```

then point CMake at them with `-DCBC_DIR=<SHOT>/ThirdParty/Cbc
-DIPOPT_DIR=<SHOT>/ThirdParty/Ipopt` (see section 3).

#### Uno (optional)

[Uno](https://github.com/cvanaret/Uno) is an alternative NLP solver for the
fixed-integer primal heuristic. It is off by default and has to be built from
source; there is no distribution package.

Building Uno requires a **Fortran compiler** and BLAS/LAPACK. Beyond that, Uno
needs at least one subproblem solver to be compiled into it, or it cannot solve
constrained NLPs at all:

* an **interior-point** method (the `ipopt` preset) needs MUMPS, HSL or SPRAL
  SSIDS — note that MUMPS has to provide `dmumps_c.h`, which the MUMPS shipped
  inside some Ipopt distributions does not;
* an **SQP** method (the `filtersqp` preset) needs BQPD or HiGHS.

Which of these is available matters for how much of SHOT's workload Uno can
take on: HiGHS refuses negative curvature outright, so an Uno built with only
HiGHS solves convex fixed NLPs but fails on the nonconvex ones that SHOT
frequently produces. BQPD, or an interior-point build with MUMPS or HSL, is the
better fit.

Uno installs neither a CMake package configuration file nor a pkg-config file,
so SHOT locates it with `misc/FindUno.cmake` and the `UNO_DIR` cache variable.
A shared build is easiest, since a static `libuno.a` has to be linked together
with BLAS/LAPACK and the Fortran runtime.

```bash
git clone https://github.com/cvanaret/Uno
cmake -S Uno -B Uno/build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
    -DBUILD_STATIC_LIBS=OFF -DCMAKE_INSTALL_PREFIX=<SHOT>/ThirdParty/Uno
cmake --build Uno/build -j && cmake --install Uno/build
```

then configure SHOT with `-DHAS_UNO=on -DUNO_DIR=<SHOT>/ThirdParty/Uno`.

If Uno reports `Found HIGHS` but then fails to compile with `'Highs.h' file not
found`, pass `-DHIGHS_INCLUDE_DIR=<path to the HiGHS headers>` as well: Uno
locates the HiGHS library and its headers independently. On macOS, a Homebrew
GCC also needs `-DCMAKE_SHARED_LINKER_FLAGS=-L$(brew --prefix gcc)/lib/gcc/current`
so that the linker finds `libgfortran`.

**Uno with MUMPS and Ipopt in the same build.** When MUMPS (or BQPD) is given to
Uno as static archives, a shared `libuno` exports all their symbols. Ipopt
usually has its own MUMPS, and the MUMPS libraries of e.g. Homebrew's Ipopt
leave most of their symbols to be looked up in all loaded libraries at runtime.
Since SHOT loads `libuno` before them, Ipopt's MUMPS then calls into Uno's copy,
which is another MUMPS version. This crashes Ipopt on larger NLPs, either with a
segmentation fault in `mumps_ginp94_elim_tree_` or with
`Internal error 3 in DMUMPS_ANA_DRIVER` and `MPI_ABORT called`, after which SHOT
exits without a result. Build Uno so that it does not export the symbols of the
archives:

```bash
# macOS: list the symbols of the archives and leave them out of libuno's exports
nm -gUj <MUMPS and BQPD archives> | grep -v -E ':$|^$' | sort -u > hidden-symbols.txt
cmake ... "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-unexported_symbols_list,$PWD/hidden-symbols.txt"

# Linux (GNU ld): hide the symbols of all static archives
cmake ... "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,--exclude-libs,ALL"
```

where the archives are e.g. `libdmumps.a`, `libmumps_common.a`, `libmpiseq.a`,
`libpord.a` and `libbqpd.a`, and the flags are added to any other linker flags,
such as the `-L` for `libgfortran` above. Uno's own symbols, including its
`uno::MUMPSSolver`, stay exported, and `nm -gUj libuno.dylib | grep -c dmumps`
should then print 0. On macOS, SHOT also links Ipopt's MUMPS libraries before
`libuno` when it finds them next to Ipopt, which avoids the crash with a `libuno`
built without these flags.

### Windows (MSVC)

Install Visual Studio Build Tools with the C++ build tools, Windows SDK,
and CMake/Ninja components. Open an **x64 Native Tools Command Prompt** and
run these commands from the SHOT source directory:

```bat
cmake -S . -B build-msvc -G Ninja -DCMAKE_BUILD_TYPE=Release ^
  -DHAS_HIGHS=ON -DHAS_CBC=OFF -DHAS_CPLEX=OFF -DHAS_GUROBI=OFF ^
  -DHAS_GAMS=OFF -DHAS_IPOPT=OFF -DHAS_UNO=OFF -DHAS_PYTHON=OFF ^
  -DCOMPILE_TESTS=ON
cmake --build build-msvc --parallel 4
ctest --test-dir build-msvc --output-on-failure --timeout 300 -E FullInstance
build-msvc\SHOT.exe test\data\alan.osil --mip=highs
```

This configuration builds the bundled HiGHS and CppAD with `cl`; it needs
neither WSL nor pkg-config. AMPL `.nl` and OSiL input remain enabled. No
external NLP solver is included; NLP subproblems use SHOT's built-in fallback.
To add Cbc and Ipopt, use the release packages described below.
GAMS 54 is supported as described below. Windows builds with CPLEX, Gurobi,
and Python bindings have not been validated.

On MSVC, CMake stages CppAD's sources in the build directory and disables
its optional Eigen discovery there, avoiding upstream's unconditional
pkg-config requirement without changing the submodule. SHOT still uses its
bundled Eigen headers directly. CppAD is linked statically on Windows.
CMake generates `CppAD.h` in the build directory and installs it with the
public headers to keep the DLL's CppAD tape registry shared with C++ clients.

#### Adding Cbc and Ipopt

Download and extract these x64 MSVC release packages:

- [Ipopt 3.14.20](https://github.com/coin-or/Ipopt/releases/tag/releases%2F3.14.20):
  `Ipopt-3.14.20-win64-msvs2022-md.zip`.
- [Cbc 2.10.12](https://github.com/coin-or/Cbc/releases/tag/releases%2F2.10.12):
  `Cbc-releases.2.10.12-w64-msvc17-md.zip`.

`IPOPT_DIR` and `CBC_DIR` must point to the extracted directories containing
`include`, `lib`, and `bin`. Ipopt's archive has an extra enclosing directory.
The Cbc archive contains **static libraries built with `/MT`**, despite the
`md` in its filename. Select the matching runtime for SHOT and its bundled
dependencies with `CMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded`:

```bat
cmake -S . -B build-msvc-coin -G Ninja -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
  -DHAS_HIGHS=ON -DHAS_CBC=ON -DHAS_IPOPT=ON ^
  -DCBC_DIR=C:/solvers/cbc ^
  -DIPOPT_DIR=C:/solvers/Ipopt-3.14.20-win64-msvs2022-md ^
  -DHAS_CPLEX=OFF -DHAS_GUROBI=OFF -DHAS_GAMS=OFF ^
  -DHAS_UNO=OFF -DHAS_PYTHON=OFF -DCOMPILE_TESTS=ON
cmake --build build-msvc-coin --parallel 4
ctest --test-dir build-msvc-coin --output-on-failure --timeout 300 -E FullInstance
build-msvc-coin\SHOT.exe test\data\alan.osil --mip=cbc --nlp=ipopt
```

These are Release packages; a Debug build needs matching Debug solver libraries.
CMake discovers the libraries directly without pkg-config and copies package
DLLs, including Ipopt's Fortran and math runtimes, beside SHOT. It also installs
those DLLs into the installation's `bin` directory. Source-built libraries can
be used through the same directory options, with their matching MSVC runtime.

#### Adding Uno

Download the Windows x64 bundle from [Uno 2.9.1](https://github.com/cvanaret/Uno/releases/tag/v2.9.1):
`Uno_binaries.v2.9.1.x86_64-w64-mingw32-cxx11.zip`. Extract it to, for example,
`C:/solvers/uno`, preserving its `bin`, `deps`, `include`, and `lib` directories.
Enable Uno in an existing MSVC build:

```bat
cmake -S . -B build-msvc-coin -DHAS_UNO=ON -DUNO_DIR=C:/solvers/uno
cmake --build build-msvc-coin --parallel 4
ctest --test-dir build-msvc-coin --output-on-failure -R "^(Uno_|EqualityConstraint_11$)"
build-msvc-coin\SHOT.exe test\data\alan.osil --mip=highs --nlp=uno
```

Although the bundle is built with MinGW, SHOT calls Uno through its C API.
CMake uses MSVC's `dumpbin` and librarian to generate an import library from
`bin/libuno.dll` in the build directory. It copies that DLL and the runtime
DLLs in `deps` beside SHOT and installs them into `bin`. No MinGW or Fortran
compiler is needed to use this bundle. The bundled GNU static libraries
cannot be linked directly into an MSVC build.

#### Adding GAMS

With GAMS installed at `C:/gams/54`, enable its interface in an existing
MSVC build (retaining that build's MIP solvers and runtime settings):

```bat
cmake -S . -B build-msvc-coin -DHAS_GAMS=ON -DGAMS_DIR=C:/gams/54
cmake --build build-msvc-coin --parallel 4
set "PATH=C:\gams\54;%PATH%"
ctest --test-dir build-msvc-coin --output-on-failure --timeout 300 -R "^GAMS_"
build-msvc-coin\SHOT.exe test\data\alan.gms --mip=highs --nlp=gams ^
  Subsolver.GAMS.NLP.Solver=ipopt
```

The build compiles the C API loaders from GAMS's `apifiles/C/api` directory.
GAMS is loaded at runtime; its directory must be on `PATH` so Windows can
resolve the dependent DLLs. Use a working GAMS license, with network access
to its license service when required. 

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

SHOT requires standard C++17, with compiler extensions disabled. All SHOT C++
targets (including tests and Python bindings) use `-Wall -Wextra -Wpedantic
-pedantic-errors -Wformat=2 -Wimplicit-fallthrough` on GCC/Clang, or `/W4
/permissive- /Zc:__cplusplus` on MSVC. Third-party targets do not inherit these
warning flags.

For a strict development build, using the solver options appropriate to your installation:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DCOMPILE_TESTS=ON \
  -DSHOT_WARNINGS_AS_ERRORS=ON -Werror=dev -Werror=deprecated
cmake --build build --parallel
ctest --test-dir build --output-on-failure --timeout 300 -E FullInstance
```

`SHOT_WARNINGS_AS_ERRORS` defaults to `OFF`; enabling it makes SHOT compiler
warnings fatal. C++ standard violations diagnosed by `-pedantic-errors` are
fatal regardless. The CMake `-Werror` switches separately make developer and
deprecation warnings fatal during configuration.

For a broader GCC/Clang audit, add `-DSHOT_EXTRA_WARNINGS=ON` to enable
`-Wconversion -Wsign-conversion -Wshadow -Wnon-virtual-dtor -Woverloaded-virtual`.
This optional mode may produce substantial diagnostics in existing code; use
`-DSHOT_WARNINGS_AS_ERRORS=OFF` to collect them all before addressing them.
There is no portable "all relevant warnings" switch, and a successful build
checks only the enabled backends and code paths. Validate both Debug and Release
with GCC and Clang when changing portability-sensitive code.


Key options (all `option(...)` declarations live at the top of the root
`CMakeLists.txt`):

| Option | Default | Meaning |
| --- | --- | --- |
| `HAS_CBC` | `ON` | Cbc MIP solver |
| `HAS_CPLEX` | `ON` | CPLEX MIP solver |
| `HAS_GUROBI` | `ON` | Gurobi MIP solver |
| `HAS_HIGHS` | `ON` | HiGHS MIP solver — like the others, optional; turn off with `-DHAS_HIGHS=off` (see section 1 re: `USE_EXTERNAL_HIGHS`) |
| `HAS_IPOPT` | `ON` | Ipopt NLP solver |
| `HAS_UNO` | `OFF` | Uno NLP solver (see section 2) — off by default because building Uno needs a Fortran compiler |
| `HAS_GAMS` | `ON` | GAMS interface (modeling system + NLP solver) |
| `HAS_AMPL` | `ON` | AMPL MP `.nl` file interface |
| `HAS_PYTHON` | `OFF` | Build the `SHOTpy` Python bindings (needs Python3 dev headers; pybind11 comes from the submodule) |
| `SHOT_WARNINGS_AS_ERRORS` | `OFF` | Treat SHOT C++ warnings as errors |
| `SHOT_EXTRA_WARNINGS` | `OFF` | Additional GCC/Clang conversion, shadowing, and virtual-function diagnostics |
| `COMPILE_TESTS` | `OFF` | Build the `test_runner` test suite (section 6) |
| `GENERATE_EXE` | `ON` | Build the `./SHOT` console executable |
| `BUILD_DOCUMENTATION` | on if Doxygen found | Build the HTML API docs |

`<SOLVER>_DIR` cache variables (`CBC_DIR`, `CPLEX_DIR`, `GUROBI_DIR`,
`IPOPT_DIR`, `UNO_DIR`, `GAMS_DIR`, `HIGHS_EXTERNAL_DIR`) override where each
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

If `pybind11-stubgen` is installed in the Python that CMake finds
(`pip install pybind11-stubgen`), the build also generates the type stubs
`SHOTpy.pyi` next to the module, so that editors and type checkers know the
classes, signatures and docstrings of `SHOTpy`. A signature that cannot be
written as a Python type, e.g., one that shows a C++ type name, then fails
the build.

## 5. Installing SHOT

After building, install to a writable prefix:

```bash
cmake --install build --prefix "$HOME/.local"
"$HOME/.local/bin/SHOT" --help
```

The command-line program is installed in `bin`, while `libSHOTSolver` and the
CppAD library built with SHOT are installed in the platform's CMake library
directory (usually `lib` or `lib64`). A HiGHS library built
from the bundled submodule is installed there too. Dependencies supplied
externally, such as Ipopt, Cbc, CPLEX, Gurobi, GAMS, or an external HiGHS,
must remain available on the target system. `-DGENERATE_EXE=off` installs the
library without the command-line program. When `HAS_PYTHON=on`, use
`--component Runtime` followed by `--component Development` to install only
the native command-line program and C++ package; the Python component remains
separate.

The installation also contains a CMake package for C++ applications. In a
separate project's `CMakeLists.txt`:

```cmake
find_package(SHOT 1.1 CONFIG REQUIRED)
add_executable(my_solver main.cpp)
target_link_libraries(my_solver PRIVATE SHOT::SHOTSolver)
```

Configure that project with `-DCMAKE_PREFIX_PATH="$HOME/.local"` if the
prefix is not in CMake's default search paths. The imported target supplies
SHOT's include directories and C++17 requirement. Public headers that expose
CppAD, Eigen, spdlog, Boost, MC++, or tinyxml2 use the matching bundled headers
installed with SHOT. Code that includes an optional solver's own headers
still needs that solver's development package.

## 6. Running the tests

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
