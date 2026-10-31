# Changelog

User-visible changes to SHOT and SHOTpy. See [CONTRIBUTING.md](CONTRIBUTING.md)
for how to add entries. Changes in versions before 2.0 are described in the
[GitHub releases](https://github.com/coin-or/SHOT/releases).

## Unreleased

### Changed

- The C++ headers are installed in `include/SHOT` instead of directly in
  `include`, so that headers such as `Solver.h` and `Settings.h` no longer
  clash with other packages. Projects using the CMake target
  `SHOT::SHOTSolver` need no change; include headers as `<SHOT/Solver.h>` or,
  as before, `"Solver.h"`. Projects that do not use the CMake package must add
  `<prefix>/include/SHOT` to their include path.
