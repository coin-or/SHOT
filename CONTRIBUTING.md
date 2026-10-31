# Contributing to SHOT

These rules apply to human and agent contributions to `coin-or/SHOT`. Build
instructions are in [docs/CompilationInstructions.md](docs/CompilationInstructions.md).

## Issues

Use a short title stating the problem or wanted outcome, for example `HiGHS
ignores the infeasibility repair time limit`. For bugs, use the bug report
template: a reproducer, the expected and actual result, and the environment.
For solver behavior, a zipped `--debug` directory is the most useful evidence;
see [docs/DebuggingSHOT.md](docs/DebuggingSHOT.md).

## Branches

Branch from current `master` and keep each branch to one change. Name it
`<type>/<short-description>` in lowercase with hyphens, optionally with an
issue number, for example `fix/123-highs-time-limit`.

| Type | Use |
| --- | --- |
| `feat` | New functionality |
| `fix` | Bug fixes |
| `perf` | Performance improvements |
| `refactor` | Internal restructuring |
| `docs` | Documentation |
| `test` | Tests |
| `build` | Dependencies, build system, packaging, CI |
| `chore` | Other maintenance |

## Commits and pull requests

Write PR titles and commit subjects as `<type>(<optional scope>): <Description>`,
with a type from the table, a capitalized imperative description and no final
period:

```text
fix(highs): Use the time limit of the infeasibility repair
feat(python): Expose primal solution callbacks
build: Update the pinned HiGHS release
```

Add `!` after the type for incompatible changes to the public API, CLI or
settings, for example `feat(settings)!: Rename ...`, and describe the migration
in the changelog.

Open the PR into `master`. Describe the problem, the new behavior, the tests
run and any untested configurations; a few sentences suffice for a small
change. Write `Fixes #123` when the PR resolves an issue. PRs are squash merged
with the PR title as the commit subject.

Format only the code you change with `.clang-format` and `.cmake-format`, not
whole files. Keep dependency versions pinned; see
[ThirdParty/README.md](ThirdParty/README.md). Never force-push `master`.

## Validation

Build and run the tests with

```bash
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

and the checks that fit the change:

- Solver changes: add a small regression test; check feasibility, bounds and
  termination status on relevant instances.
- Backend changes: test with the affected MIP or NLP solver.
- Python changes: run the relevant tests in `test/python`.
- Performance changes: compare before and after with the same instances,
  settings, solver versions and hardware.
- Build and packaging changes: do a clean build and test the affected packages.
- Callback changes: follow [docs/Callbacks.md](docs/Callbacks.md).

State in the PR which checks were run and which were skipped.

## Changelog

`CHANGELOG.md` records user-visible changes. A PR with such a change adds a
bullet under `## Unreleased`, in an `Added`, `Changed`, `Fixed` or `Removed`
group, describing the effect for users. Internal refactoring and tests need
no entry.

## Versions and releases

SHOT and SHOTpy share one version, following [Semantic
Versioning](https://semver.org/). The version is `project(SHOT VERSION ...)`
together with `SHOT_VERSION_SUFFIX` (empty, `a<n>`, `b<n>` or `rc<n>`) in
`CMakeLists.txt`; version `2.0.0` with suffix `rc1` is tagged `v2.0.0-rc1` and
gives the Python version `2.0.0rc1`.

**Pushing a `v*` tag publishes SHOTpy to PyPI.** Only the project manager
decides on releases and pushes tags.

1. In a release PR, set the version and suffix, rename `## Unreleased` in
   the changelog to `## X.Y.Z — YYYY-MM-DD`, update `version`,
   `date-released` and `url` in `CITATION.cff`, and point the documentation
   links in `README.md` and `README` from `docs/dev/` to the release.
2. Run the regression tests and test the built wheels and executables.
3. After merging, tag the merge commit with an annotated tag, and publish the
   GitHub release notes from the changelog and the documentation
   ([docs/README.md](docs/README.md)).
4. Check that the published packages install. Never move a published tag; fix
   mistakes in a new release.
