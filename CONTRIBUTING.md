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

Keep dependency versions pinned; see
[ThirdParty/README.md](ThirdParty/README.md). Never force-push `master`.

## Formatting

Use Python 3.11 or newer, shown as `python` below. From the repository root,
the same commands work on Linux, macOS and Windows:

```bash
python scripts/format.py --setup
python scripts/format.py --apply
python scripts/format.py --check
```

Run `--setup` once, and again when the pins in `scripts/requirements-format.txt`
change. It creates or reuses `build/format-venv` and installs the pinned tools
there. Both formatting modes select that environment automatically, verify the
installed versions and run its executables. No environment activation or
platform-specific paths are needed. The formatting CI check runs the same
commands on Linux and Windows.

Both modes select the same tracked files using `git ls-files`: C++ sources,
headers and header templates in `src/` and `test/`; CMake files and templates
in the project root, `cmake/` and `test/`; and Python files in `src/`, `test/`,
`scripts/` and `docs/`. Add new source files to Git before running the script.
Dependencies in `ThirdParty/`, build outputs, model data, notebooks, symlinks and the
copied GAMS header `src/GAMS/GamsNLinstr.h` are excluded. The Python manifest
generator in `test/data/instances/` is included. Shell, PowerShell and website
assets are outside this formatter's scope.

The style is defined by `.clang-format`, `.cmake-format` and `.ruff.toml`.
C++ uses Allman braces with `else` and `else if` on their own lines, four-space
indentation and a 120-column limit.
CMake and Python use four-space indentation and a 120-column target too.
C++ formatting preserves include order and existing template declaration breaks,
and wraps comments. CMake formatting preserves argument order and comment text.
Python formatting preserves existing string quote choices and does not run lint
fixes or sort imports. All three formatters
preserve checkout line endings; Git normalizes text through `.gitattributes`.
The `project(...)` declaration
keeps its existing layout because packaging reads its version prefix.

The initial repository-wide formatting pass is a deliberate baseline reset.
Keep it separate from functional changes. After that, format the code you
change; the full-tree apply command should leave unrelated files unchanged.
Configure editor format-on-save to use the pinned tools and these configuration
files. The blank-line conventions in `AGENTS.md` still require review because
clang-format does not fully enforce them.

After the baseline PR is merged, add its full merged commit SHA to
`.git-blame-ignore-revs` in a follow-up change. PRs are squash merged, so use
the SHA on `master`, not a branch commit. To enable this file locally after
checking out a branch containing it:

```bash
git config blame.ignoreRevsFile .git-blame-ignore-revs
```

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
group, describing the effect for users. A change that requires users to
update their code, options files or builds goes in the `Breaking changes`
group, first in the section, and is not repeated in the other groups;
removals always go in `Removed`.
Internal refactoring and tests need no entry.

## Versions and releases

SHOT and SHOTpy share one version, following [Semantic
Versioning](https://semver.org/). The version is `project(SHOT VERSION ...)`
together with `SHOT_VERSION_SUFFIX` (empty, `a<n>`, `b<n>` or `rc<n>`) in
`CMakeLists.txt`; version `2.0.0` with suffix `rc1` is tagged `v2.0.0-rc1` and
gives the Python version `2.0.0rc1`.

**Pushing a `v*` tag publishes SHOTpy to PyPI** and attaches the executable
archives to the GitHub release, which is created as a draft if it does not
exist. Only the project manager decides on releases and pushes tags.

1. In a release PR, set the version and suffix, rename `## Unreleased` in
   the changelog to `## X.Y.Z — YYYY-MM-DD`, update `version`
   and `date-released` in `CITATION.cff`, and point the documentation
   links in `README.md` and `README` from `docs/dev/` to the release.
2. Run the regression tests and test the built wheels and executables.
3. After merging, tag the merge commit with an annotated tag. Check that the
   executable archives and their checksums are attached to the draft release,
   then publish it with release notes from the changelog, and publish the
   documentation ([docs/README.md](docs/README.md)).
4. Check that the published packages install. Never move a published tag; fix
   mistakes in a new release.
