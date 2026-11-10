#!/usr/bin/env python3
"""Build the public docs; run with the matching SHOTpy on PYTHONPATH."""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
GUIDES = (
    'CompilationInstructions.md',
    'CommandLineUsage.md',
    'Callbacks.md',
    'UnderstandingOutput.md',
    'Configuration.md',
    'Modeling.md',
    'DebuggingSHOT.md',
    'DeveloperDebugging.md',
    'MIPSolverCapabilities.md',
)


def build_tutorial(source):
    """Publish the tutorial notebooks, with their stored outputs, as pages after the overview in their README."""
    tutorial = ROOT / 'docs/tutorial'
    shutil.copytree(tutorial, source / 'tutorial', ignore=shutil.ignore_patterns('README.md', '.ipynb_checkpoints'))
    overview = (tutorial / 'README.md').read_text()
    # The numbered links of the README give the chapters and their order
    chapters = re.findall(r'^\d+\. \[(.+?)\]\((\S+?)\.ipynb\)', overview, flags=re.MULTILINE)
    overview = re.sub(r'\]\((\S+?\.ipynb)\)', r'](tutorial/\1)', overview)
    assets = source / '_assets'
    assets.mkdir()
    shutil.make_archive(str(assets / 'SHOTpy_Tutorial'), 'zip', tutorial.parent, tutorial.name)
    (source / 'SHOTpy_Tutorial.md').write_text(
        overview + '\n{download}`Download the notebooks and their data files <_assets/SHOTpy_Tutorial.zip>`\n\n'
        '```{toctree}\n:hidden:\n:maxdepth: 1\n\n'
        + '\n'.join(f'{label} <tutorial/{name}>' for label, name in chapters)
        + '\n```\n'
    )


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
            content = content.replace(
                '](../ThirdParty/', f'](https://github.com/coin-or/SHOT/blob/{args.source_ref}/ThirdParty/'
            )
            (source / guide).write_text(content)
        build_tutorial(source)
        settings = SHOTpy.Solver().getSettingsAsMarkup()
        settings = re.sub(r'^(#+) ', r'\1# ', settings, flags=re.MULTILINE)
        (source / 'reference/settings.md').write_text(
            '# Solver settings\n\nAll settings of SHOT, including those of the MIP and NLP solvers and of GAMS. '
            'These only have an effect if SHOT is built with them, as noted in the descriptions; a build without '
            'them warns when they are changed. Some settings have another default for nonconvex problems, applied '
            'unless `Strategy.UseRecommendedSettings` is false, and the default MIP solver depends on the build.\n'
            + settings
        )
        doxy = work / 'Doxyfile'
        headers = [
            'Solver.h',
            'Settings.h',
            'Results.h',
            'Callback.h',
            'Enums.h',
            'Structs.h',
            'MIPSolver/MIPSolverCapabilities.h',
        ]
        doxy.write_text(
            '\n'.join(
                [
                    'PROJECT_NAME = SHOT',
                    f'OUTPUT_DIRECTORY = "{work / "doxygen"}"',
                    'INPUT = ' + ' '.join(f'"{ROOT / "src" / h}"' for h in headers),
                    'GENERATE_HTML = NO',
                    'GENERATE_LATEX = NO',
                    'GENERATE_XML = YES',
                    'EXTRACT_ALL = YES',
                    'EXTRACT_PRIVATE = NO',
                    'EXTRACT_STATIC = NO',
                    'QUIET = YES',
                    'WARN_IF_UNDOCUMENTED = NO',
                    'ENABLE_PREPROCESSING = YES',
                    'MACRO_EXPANSION = YES',
                    'PREDEFINED = DllExport=',
                ]
            )
            + '\n'
        )
        subprocess.run(['doxygen', str(doxy)], check=True)
        subprocess.run(
            [
                sys.executable,
                '-m',
                'sphinx',
                '-b',
                'html',
                '-W',
                '--keep-going',
                '-d',
                str(work / 'doctrees'),
                str(source),
                str(output),
            ],
            check=True,
        )
    print(f'Documentation: {output / "index.html"}')


if __name__ == '__main__':
    main()
