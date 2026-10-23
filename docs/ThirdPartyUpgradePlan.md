# Third-party dependency upgrade plan

Prepared 2026-10-10 from the current checkout and upstream release pages. This is a plan only; no dependency pins or solver code have been changed.

Use the latest stable releases available at the implementation freeze, pin exact commits or archive checksums, and retain C++17 unless a dependency demonstrably requires a change. Recheck the versions below at that freeze. A library already at its latest release needs provenance verification and regression coverage rather than a gratuitous source change.

## 1. Inventory and proposed targets

| Dependency | Current checkout | Proposed target / action | Upstream source |
| --- | --- | --- | --- |
| Boost | Vendored header subset, 1.67.0 | 1.92.0; regenerate the complete required subset from one release | [Release](https://www.boost.org/releases/1.92.0/) |
| CppAD | Submodule, 20210000.6 (`c93554b1`) | 20260000.0; review intervening API and build changes | [Releases](https://github.com/coin-or/CppAD/releases) |
| pybind11 | Submodule, headers report 2.13.6 (`58c382a8`) | 3.1.0; treat as a major upgrade | [Release](https://github.com/pybind/pybind11/releases/tag/v3.1.0) |
| nlohmann/json | Vendored single header, 3.11.3 | 3.12.0 | [Release](https://github.com/nlohmann/json/releases/tag/v3.12.0) |
| spdlog | Submodule, 1.17.0 (`79524ddd`) | Already matches latest release; retain bundled fmt consistently | [Release](https://github.com/gabime/spdlog/releases/tag/v1.17.0) |
| HiGHS | Submodule `9a3d8db1`; `Version.txt` reports 1.15.1 | Verify commit against v1.15.1, pin release commit if different, inspect any lost post-release fixes | [Release](https://github.com/ERGO-Code/HiGHS/releases/tag/v1.15.1) |
| Eigen | Submodule, 5.0.1 (`bc3b3987`) | Retain pending final tag check; 5.0.1 verified upstream, latest-release listing was not readable | [Tag](https://gitlab.com/libeigen/eigen/-/tags/5.0.1) |
| TinyXML2 | Vendored sources, 11.0.0 | Already at latest version; compare source and local patches with release | [Release](https://github.com/leethomason/tinyxml2/releases/tag/11.0.0) |
| argh | Vendored header, updated locally in commit `e15cb78a`; no version macro identified | Compare against v1.3.2 and newer tags before replacing; preserve compiler fix | [Tags](https://github.com/adishavit/argh/tags) |
| AMPL MP reader | Vendored `mp` headers and selected `.cc` files; exact revision unidentified | Establish provenance, then choose latest stable MP release and adapt the reader integration | [Releases](https://github.com/ampl/mp/releases) |
| MC++ | Vendored sources and nested third-party copies; exact revision unidentified | Establish provenance and current upstream release/tag; update interval implementation with numerical validation | Target remains to be verified |

Do not infer versions from `git describe` alone: this checkout describes spdlog relative to v1.2.1 despite matching the v1.17.0 release commit, and HiGHS relative to v1.14.0 despite its 1.15.1 version file. Check upstream tag commit identities.

Include externally installed solvers in the release compatibility audit. Candidate versions observed upstream are [Cbc 2.10.13](https://github.com/coin-or/Cbc/releases/tag/releases/2.10.13), [Ipopt 3.14.20](https://github.com/coin-or/Ipopt/releases/tag/releases/3.14.20), and [Uno 2.8.0](https://github.com/cvanaret/Uno/releases/tag/v2.8.0). These are not tracked submodules. Inventory their transitive solver libraries through their build recipes. Verify current GAMS, CPLEX, and Gurobi SDK releases separately and test available licensed installations; record unavailable combinations explicitly. Local untracked Uno build/install directories are not vendored dependency sources to replace.

## 2. Establish a reproducible baseline

1. Record SHOT commit, submodule SHAs, compiler/CMake/Python versions, solver versions, build options, and existing failures. Preserve unrelated local work.
2. Make clean Release and Debug builds with tests enabled. Run CTest and `test/python`, plus a fixed representative `InstanceTest` set covering convex/nonconvex models and available input formats. Keep settings, threads, and instance inputs fixed.
3. Record status, objective and dual bounds, feasibility, iterations, and timing. Use these to distinguish numerical regressions from expected iteration differences after upgrades.
4. Add a dependency manifest (for example `ThirdParty/versions.json`) with origin, release, full SHA or archive SHA256, license, extraction command, and local patch list. Recover the unidentified vendored revisions by comparing upstream files and repository history. Enumerate nested copies, including fmt and MC++'s cpplapack/FADBAD++, and determine which are actually used or shipped.

Acceptance: reproducible baseline results and an explicit provenance/patch record. Unknown AMPL/MC++ targets must be resolved before claiming the refresh is complete.

## 3. Update the simpler dependencies first

- Refresh JSON from its official single-header release and exercise settings/results serialization and parsing, including malformed input handling where covered.
- Verify argh against its release; exercise help, positional arguments, negative values, and `--name=value` parsing. Preserve necessary local fixes as documented patches.
- Verify TinyXML2, spdlog, and Eigen against exact releases. Check OSiL parsing, formatted output/custom sinks, and the existing linear algebra tests. Account for fmt bundled inside spdlog; avoid mixing independently sourced fmt headers.
- Keep each dependency change and its necessary SHOT adaptation in a separate commit. Do not move submodules to floating branch heads.

## 4. Refresh Boost as a coherent vendored subset

Continue the existing vendored-header approach, keeping installation requirements stable.

1. Download the selected official Boost source archive and verify its checksum. Build/use the matching Boost `bcp` tool to extract explicit required entry headers and their transitive dependencies into a staging directory.
2. Start from actual includes: `boost/functional/hash/hash.hpp`, `boost/math/tools/roots.hpp`, `boost/math/tools/minima.hpp`, and `boost/cstdint.hpp`. Scan SHOT and retained dependencies for additional uses, including conditional includes. Adapt obsolete include paths if necessary.
3. Replace the old subset as one unit; remove stale headers. Preserve licenses and add a reproducible extraction script with pinned inputs. Validate on both platforms so missing headers cannot be masked by a system Boost installation.
4. Test root finding, scalar minimization, and point hashing/deduplication. Do not assume hash values remain identical across Boost releases; check any dependence on persistent hashes or ordering. Run solver cases that exercise supporting-hyperplane root search and the cutting-plane minimax NLP path.
5. Record resulting subset size, compiler requirements, and build impact.

The separate [Boost interval migration plan](BoostIntervalMigrationPlan.md) changes the numerical backend and should have its own commits and acceptance gates. If scheduled for this release, include Numeric.Interval in the extraction inputs and execute that plan after the basic Boost refresh. Otherwise retain and audit/update MC++ for this release. Refreshing Boost alone does not migrate interval arithmetic or establish verified bounds.

## 5. Upgrade numerical and modeling integrations

**CppAD:** move to the selected stable tag; review differentiation APIs, sparsity/Hessian support, and all custom integration points. Audit the `ExternalProject_Add(cppad)` configuration, generated configuration headers, installed include paths, library filename/byproducts, and runtime loading. SHOT currently combines source-tree headers with an externally built shared `cppad_lib`, so explicitly verify that headers and binary come from the same configuration. Test gradients/Hessians, expression/domain cases, and representative solves; add focused independent-reference checks where coverage is missing. Verify clean Ninja and Make builds and Python wheel runtime paths.

**HiGHS:** establish whether the current SHA is exactly the stable release before changing it. Inspect callback, solution/status, option, and model-modification APIs used by `MIPSolverHighs.cpp`. Test bundled HiGHS, external HiGHS at the chosen release, and a build with HiGHS disabled and another MIP backend enabled. Exercise infeasible/unbounded models, limits, callbacks, and repeated solves. Require a clear configure-time version error if a new minimum is needed.

**AMPL MP:** SHOT builds selected MP reader sources directly, rather than importing a complete modern MP project. Compare source layout/API and bundled formatting utilities with the target release. Preserve local adaptations, including the removal of an unnecessary `main` recorded in commit `168def36`. Verify text/binary `.nl` inputs where supported, nonlinear operators, bounds, and solution handling. Correct documentation that calls this vendored integration ASL if that misidentifies the actual implementation.

**MC++:** identify local interval fixes before replacing files. Validate power/domain behavior, exception handling, and forward/reverse bound propagation with model and FBBT tests. Preserve known feasible points and conservative bounds. Audit nested bundled libraries without introducing unused dependencies. If the separately validated interval migration removes MC++ first, record removal as the dependency disposition instead of doing an unnecessary upgrade.

## 6. Upgrade Python bindings and packaging

- Move pybind11 to the chosen 3.x release and review its migration notes, ABI, ownership/lifetime, callback/GIL, exception, and NumPy conversion implications.
- Run all Python tests, especially callbacks and object lifetimes. Generate and validate `SHOTpy.pyi` with the configured stub generator.
- Build an sdist, build a wheel from that sdist, install into a clean environment, and run tests outside the checkout. Check bundled CppAD/solver shared libraries and licenses.
- Test the declared Python floor (currently 3.9) and the supported wheel matrix (currently 3.9–3.13). Review newer stable Python support against upstream compatibility before adding wheel targets or changing the support promise.
- Audit build tooling in `pyproject.toml` and wheel workflows, including scikit-build-core, stubgen, cibuildwheel, and package-manager solver inputs. Record the versions used for release builds.

## 7. Make CI and release builds reproducible

- Replace floating Cbc/Ipopt coinbrew selections with release pins and record transitive revisions. The current Linux workflow uses `Cbc@stable/2.10` and unpinned Ipopt.
- Include dependency version/manifest digest, architecture, compiler, and relevant build options in cache keys. Current Cbc/Ipopt keys contain only runner OS and can silently reuse older builds.
- Align native CI, wheel builds, and compilation instructions. Homebrew/Conda provisioning currently floats; use recorded/resolvable release environments for published binaries.
- Test Linux GCC, Linux Clang, and macOS Apple Clang, with Release plus at least one Debug configuration. Include supported CPU architectures, optional-backend configurations, Python on/off, and AMPL on/off. Verify actual enabled backends because CMake may disable missing solvers with warnings.
- Validate the supported minimum CMake version (currently 3.20) and a current CMake version. If upstream requirements force a raised minimum, update CMake, documentation, and CI together.

## 8. Release acceptance and delivery

Run focused tests after each dependency change, then the combined CTest/Python suites, clean source-package and wheel builds, and the fixed numerical benchmark set. Investigate changed feasibility, invalid bounds, missing solutions, crashes, and material performance regressions; compare timing against baseline variability rather than demanding identical iteration counts.

Deliver separate commits for the manifest/baseline infrastructure, simple vendor updates, Boost, CppAD, HiGHS, AMPL/MC++, Python, and CI/documentation. Dependencies already current may need only manifest entries. Keep each update independently revertible. Document any intentionally retained older version with its concrete blocker and test evidence; such an exception must be visible in the release notes.

Completion means every shipped dependency has an identified disposition and provenance, required tests pass or pre-existing failures are explicitly accounted for, clean users can build/install release artifacts, and the compilation guide plus release notes state the tested versions and support requirements.
