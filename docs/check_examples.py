#!/usr/bin/env python3
"""Check public guide setting names and run the complete introductory Python examples.

Use the SHOTpy module built from the same checkout, as for build.py.
The tutorial notebook has its own execution test in test/python/test_tutorial.py.
"""
import contextlib
import io
import json
import re
import tempfile

import SHOTpy

from build import GUIDES, ROOT


PYTHON_GUIDES = ('site/getting-started.md', 'site/concepts.md', 'Configuration.md', 'Modeling.md')
SETTING_NAME = re.compile(
    r'(?<![\w.])(?:Dual|Primal|Termination|Output|Strategy|Model|Subsolver)'
    r'(?:\.[A-Za-z][A-Za-z0-9]*)+'
)


def main():
    solver = SHOTpy.Solver()
    registered = set(re.findall(r'\|\*\*([\w.]+)\*\*', solver.getSettingsAsMarkup()))
    if not registered:
        raise RuntimeError('No settings found in the matching module')

    sources = [ROOT / 'docs' / guide for guide in GUIDES if guide != 'DeveloperDebugging.md']
    sources.extend((ROOT / 'docs/site').glob('*.md'))
    checked = set()
    for path in sources:
        if path.suffix == '.ipynb':
            notebook = json.loads(path.read_text())
            content = '\n'.join(''.join(cell['source']) for cell in notebook['cells'])
        else:
            content = path.read_text()
        for name in SETTING_NAME.findall(content):
            # Prose also names setting groups, e.g., Model.Reformulation.
            if name not in registered and not any(key.startswith(name + '.') for key in registered):
                # Internal paths mentioned in the file reference are omitted from public markup.
                for getter in (solver.getBoolSetting, solver.getIntSetting,
                               solver.getDoubleSetting, solver.getStringSetting):
                    try:
                        getter(name)
                        break
                    except RuntimeError:
                        pass
                else:
                    raise RuntimeError(f'{path.relative_to(ROOT)}: unknown setting or group {name}')
            checked.add(name)
    print(f'Checked {len(checked)} setting names/groups against SHOTpy')

    count = 0
    for guide in PYTHON_GUIDES:
        path = ROOT / 'docs' / guide
        content = path.read_text()
        for match in re.finditer(r'^```python\n(.*?)^```', content, re.MULTILINE | re.DOTALL):
            line = content[:match.start()].count('\n') + 2
            source = '\n' * (line - 1) + match[1]
            output = io.StringIO()
            try:
                # Examples are complete and must not rely on files in the checkout.
                with tempfile.TemporaryDirectory(prefix='shot-doc-example-') as directory:
                    with contextlib.chdir(directory), contextlib.redirect_stdout(output):
                        exec(compile(source, str(path), 'exec'), {'__name__': '__main__'})
            except Exception:
                print(output.getvalue())
                raise
            count += 1
            print(f'Passed {path.relative_to(ROOT)}:{line}')
    if count == 0:
        raise RuntimeError('No Python examples found')
    print(f'Executed {count} Python examples')


if __name__ == '__main__':
    main()
