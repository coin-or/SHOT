#!/usr/bin/env python3
"""Build the public docs; run with the matching SHOTpy on PYTHONPATH."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
GUIDES = ('CompilationInstructions.md', 'CommandLineUsage.md', 'Callbacks.md',
          'DebuggingSHOT.md', 'SHOTpy_Tutorial.ipynb')


def build_tutorial(notebook, source):
    """Publish one notebook source as a navigable series, preserving cell order."""
    sections = []
    for cell in notebook['cells']:
        text = ''.join(cell['source'])
        heading = re.match(r'^## \d+\. (.+?) \{#([^}]+)\}', text) if cell['cell_type'] == 'markdown' else None
        if heading:
            sections.append((heading[2], heading[1], []))
        if sections:
            sections[-1][2].append(cell)
    labels = {
        'getting-started': 'Getting started', 'creating-variables': 'Variables',
        'linear-problems': 'Linear models', 'mixed-integer-problems': 'Integer variables',
        'quadratic-problems': 'Quadratic models', 'nonlinear-problems': 'Nonlinear models',
        'reading-from-files': 'Problem files', 'solver-settings': 'Solver settings',
        'advanced-features': 'Advanced features', 'callbacks': 'Callbacks',
        'building-large-problems': 'Large models',
    }
    links = {slug: f'{slug}.ipynb' for slug, _, _ in sections}
    folder = source / 'tutorial'
    folder.mkdir()
    for slug, title, cells in sections:
        for cell in cells:
            if cell['cell_type'] != 'markdown':
                continue
            text = ''.join(cell['source'])
            text = re.sub(r'^## \d+\. (.+?) \{#([^}]+)\}', r'(\2)=\n# \1', text)
            # Promote subsection headings after splitting into individual pages.
            text = re.sub(r'^###', '##', text, flags=re.MULTILINE)
            for anchor, filename in links.items():
                text = text.replace(f'](#{anchor})', f']({filename})')
            cell['source'] = text.splitlines(keepends=True)
        page = dict(notebook, cells=cells)
        (folder / f'{slug}.ipynb').write_text(json.dumps(page))
    assets = source / '_assets'
    assets.mkdir()
    shutil.copyfile(ROOT / 'docs/SHOTpy_Tutorial.ipynb', assets / 'SHOTpy_Tutorial.ipynb')
    (source / 'SHOTpy_Tutorial.md').write_text(
        '# SHOTpy tutorial\n\nLearn to model and solve MINLP problems with SHOTpy. '
        'The chapters follow the original notebook in order; examples may reuse imports or objects from earlier chapters.\n\n'
        '{download}`Download the complete notebook <_assets/SHOTpy_Tutorial.ipynb>`\n\n'
        '```{toctree}\n:maxdepth: 1\n\n' +
        '\n'.join(f'{labels.get(slug, title)} <tutorial/{slug}>' for slug, title, _ in sections) + '\n```\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'build/docs/html')
    parser.add_argument('--source-ref', default='master', help='Git ref for source links')
    args = parser.parse_args()
    import SHOTpy
    output = args.output.resolve()
    with tempfile.TemporaryDirectory(prefix='shot-docs-') as directory:
        work = Path(directory)
        source = work / 'source'
        shutil.copytree(ROOT / 'docs/site', source)
        for guide in GUIDES:
            content = (ROOT / 'docs' / guide).read_text()
            # Repository-only links must remain usable on the published site.
            content = content.replace('](../ThirdParty/',
                f'](https://github.com/coin-or/SHOT/blob/{args.source_ref}/ThirdParty/')
            if guide.endswith('.ipynb'):
                build_tutorial(json.loads(content), source)
            else:
                (source / guide).write_text(content)
        settings = SHOTpy.Solver().getSettingsAsMarkup()
        settings = re.sub(r'^(#+) ', r'\1# ', settings, flags=re.MULTILINE)
        (source / 'reference/settings.md').write_text(
            '# Solver settings\n\nAll settings of SHOT, including those of the MIP and NLP solvers and of GAMS. '
            'These only have an effect if SHOT is built with them, as noted in the descriptions; a build without '
            'them warns when they are changed. Some settings have another default for nonconvex problems, applied '
            'unless `Strategy.UseRecommendedSettings` is false, and the default MIP solver depends on the build.\n'
            + settings)
        doxy = work / 'Doxyfile'
        headers = ['Solver.h', 'Settings.h', 'Results.h', 'Callback.h', 'Enums.h', 'Structs.h']
        doxy.write_text('\n'.join([
            'PROJECT_NAME = SHOT', f'OUTPUT_DIRECTORY = "{work / "doxygen"}"',
            'INPUT = ' + ' '.join(f'"{ROOT / "src" / h}"' for h in headers),
            'GENERATE_HTML = NO', 'GENERATE_LATEX = NO', 'GENERATE_XML = YES',
            'EXTRACT_ALL = YES', 'EXTRACT_PRIVATE = NO', 'EXTRACT_STATIC = NO',
            'QUIET = YES', 'WARN_IF_UNDOCUMENTED = NO',
            'ENABLE_PREPROCESSING = YES', 'MACRO_EXPANSION = YES', 'PREDEFINED = DllExport=',
        ]) + '\n')
        subprocess.run(['doxygen', str(doxy)], check=True)
        subprocess.run([sys.executable, '-m', 'sphinx', '-b', 'html', '-W', '--keep-going', '-d', str(work / 'doctrees'),
                        str(source), str(output)], check=True)
    print(f'Documentation: {output / "index.html"}')


if __name__ == '__main__':
    main()
