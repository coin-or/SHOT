# Building the public documentation

The public site is assembled from `site/`, the guides explicitly listed in
`build.py`, C++ headers, and an importable SHOTpy module. Internal plans and reports
in this directory are never copied. Generated HTML is not committed.

## Local preview

Install Doxygen (for example, `brew install doxygen` or `apt-get install doxygen`),
then create a Python 3.11+ environment:

```bash
python3.11 -m venv .venv-docs
.venv-docs/bin/pip install -r docs/requirements.txt
# Build SHOTpy for this Python interpreter using CompilationInstructions.md.
PYTHONPATH="$PWD/build" .venv-docs/bin/python docs/build.py
python3 -m http.server 8000 --directory build/docs/html
```

Open http://localhost:8000. Use the matching Python interpreter and module from
the same source revision; do not silently substitute a different PyPI release.
The build fails on Sphinx warnings and missing module/Doxygen dependencies.
Notebook execution is disabled: run changed examples separately before publishing.

Run the complete Python examples in the first-model, results, configuration,
modeling, and MIP-capabilities guides with the matching module:

```bash
PYTHONPATH="$PWD/build" .venv-docs/bin/python docs/check_examples.py
```

This check also validates setting names in inline configuration examples against
that module. The documentation workflow runs it before building the site. The
notebook has its own execution test, `test/python/test_tutorial.py`; run it with
`pytest` when changing notebook code (it also needs NumPy, Matplotlib, IPython, and pandas).
For external-link maintenance, run Sphinx's `linkcheck` builder on an assembled
source tree when network access is available; the normal warning-as-error HTML
build checks internal document links without depending on external services.

## Website integration

The Astro website owns the single Pages deployment. Its workflow checks out SHOT,
installs/builds SHOTpy from that revision, installs `docs/requirements.txt`, and runs:

```bash
python shot/docs/build.py --source-ref COMMIT --output dist/docs/dev
```

Run this **after** Astro builds `dist/` and **before** uploading the Pages artifact.
The documentation uses relative internal links, so both `/SHOT-website/docs/dev/`
and `/docs/dev/` work. The homepage's `../../` link returns to the website root.
Add a `/docs/` entry page linking to `dev/` and, when released, `2.0/`.

For a release, build the matching tag/module into `dist/docs/2.0/` and set
`SHOT_DOCS_VERSION=2.0`; this also replaces the development notice on the start page.
Build every retained version on each deployment (or restore its archived build);
Pages deployments replace the entire published site. Do not label development
builds as stable or build release documentation from a moving branch.

## Branding

Official tokens and component styles are vendored unchanged under
`site/_static/branding/`, with their upstream revision and rights notice.
The logos come directly from SHOT-branding. `site/_static/shot.css` adapts the
Sphinx markup to those tokens: typography, navigation, tables, API signatures,
notes, code panels, focus states, and responsive spacing. `shot.js` synchronizes
Furo's theme control with the root-level theme used by the branding tokens.

Update the vendored tokens and components together from SHOT-branding; do not
edit the generated token file. Website-specific layouts remain in the website
repository. No deploy credentials are needed by SHOT's validation workflow.

The SHOTpy tutorial is the series of notebooks in `docs/tutorial`. The builder
publishes each notebook as a page, with the outputs stored in it, after an
overview made from `docs/tutorial/README.md`, whose numbered links give the
chapters and their order, and offers the folder as a zip download. The stored
outputs, including the figures, are what readers see on the site and on GitHub,
so execute the notebooks again, from their folder and with a Release build,
after changing them.

The settings page is progressively enhanced by `shot.js`: local filtering,
category navigation, collapsible groups, and copy controls. Without JavaScript,
the generated reference remains readable. Setting values come from the matching
SHOTpy module; copy controls preserve the displayed default. MathJax 4 uses Fira
Math to match the sans-serif branding. Browser math fonts load from its CDN.

The site-wide footer in `site/_templates/site-footer.html` mirrors the rendered
`SHOT-website/src/components/Footer.astro`, including its icons and link groups.
Keep it and the corresponding footer styles in sync when the website footer
changes. Its theme controls share the website's `shot-theme` preference and keep
Furo's startup preference synchronized. Section dropdowns use Furo's navigation
renderer with page sections enabled; the Furo version is pinned in requirements.
