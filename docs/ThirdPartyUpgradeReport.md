# Third-party release refresh

This report records the 2026-10-10 dependency refresh on branch
`maintenance/thirdparty-release-refresh`. Exact source commits and checksums
are in [`ThirdParty/versions.json`](../ThirdParty/versions.json). The release
table includes both updated and already-current dependencies so that the
distribution has a complete provenance record.

| Component | Release | Disposition |
| --- | --- | --- |
| Boost | 1.92.0 | Rebuilt coherent header subset with that release's `bcp` |
| CppAD | 20260000.0 | Updated submodule; SHOT now uses headers installed alongside `cppad_lib` |
| pybind11 | 3.1.0 | Updated submodule |
| HiGHS | 1.15.1 | Pinned stable release tag |
| Eigen | 5.0.1 | Already current; tag verified |
| spdlog / bundled fmt | 1.17.0 / 12.1.0 | Already current; release commit verified |
| nlohmann/json | 3.12.0 | Updated single header and license |
| AMPL MP reader | 4.1.0 | Updated selected sources/header closure; preserved isolated legacy fmt namespace |
| MC++ interval | 5.0.4 | Updated three used headers; removed unused third-party copies |
| TinyXML2 | 11.0.0 | Already current; files verified against release |
| argh | 1.3.2 | Already current; header verified against release |

CI coinbrew builds are pinned to Cbc 2.10.13 and Ipopt 3.14.20. The Python
build is pinned to scikit-build-core 1.1.1 and conditionally uses
pybind11-stubgen 3.0.0 on Python 3.10 or newer; the wheel workflow uses
cibuildwheel 4.3.1. GAMS, CPLEX, Gurobi, and Uno are supplied by the build
environment and are not part of SHOT's source checkout.

Clean Release and Debug configurations build on macOS Apple Clang. The Release
configuration enabled AMPL, Cbc, CPLEX, GAMS, Gurobi, HiGHS, Ipopt, and
Python. The Debug configuration enabled AMPL, Cbc, CPLEX, GAMS, Gurobi,
HiGHS, Ipopt, and Uno; its focused `Model_55`, `Solver_17`, and `Ipopt_1`
checks passed. The clean source archive contains the required vendor trees
and CPack inputs and excludes local build/Git objects and unrelated checkout
files. A wheel built from that archive was installed outside the checkout;
importing `SHOTpy` and constructing a solver passed.

The first Release suite run passed 231 of 232 tests. `Solver_17` exposed a
root-search corner case: TOMS748 lost its bracket when a perspective
constraint evaluated to infinity at an endpoint. The root-search method now
uses bisection for an infinite endpoint. The complete Release suite passed
232 of 232 tests after the fix.

The baseline suite passed all tests except 17 GAMS/Python cases when run
inside the sandbox. All 14 GAMS tests and the three affected Python groups
passed on rerun with local licensing-service access. The local Ipopt
installation used in these checks was 3.14.13; CI pins 3.14.20 for release
builds. Linux GCC/Clang and macOS validation are configured in
`.github/workflows/dependencies.yml` and require CI execution.

An installed macOS CPython 3.11 wheel passed 664 Python tests with 16 skips.
The final wheel was rebuilt from the explicit source archive after the
root-search fix and passed the install/import smoke test.
