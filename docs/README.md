# Building the public documentation

The public site is assembled from `site/`, the five guides explicitly listed in
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

For a release, build the matching tag/module into `dist/docs/2.0/`, set
`SHOT_DOCS_VERSION=2.0`, and replace the development notice in that release's source.
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

The SHOTpy notebook remains the single tutorial source. The builder splits its
numbered sections into `tutorial/*.ipynb` and creates a chapter overview and a
complete-notebook download. Fix prose and equations in the source notebook.

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
