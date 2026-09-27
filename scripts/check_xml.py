#!/usr/bin/env python3
"""Validate (and optionally repair) the project's XML files.

Exists because of one specific trap that broke this project four separate times:
XML forbids a double hyphen anywhere inside a comment body. Writing

    <!-- the wheels spin freely -- that is what makes them drive wheels -->

or using a row of hyphens as an underline is illegal, and xacro fails with a bare
"not well-formed (invalid token)" pointing at the line, which reads like a syntax error
in the robot description rather than a punctuation problem in a comment.

    scripts/check_xml.py            check every URDF/xacro/world/package.xml
    scripts/check_xml.py --fix      rewrite offending comments, then check

Run it after editing any of them; it takes well under a second.
"""

import pathlib
import re
import sys
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parent.parent
PATTERNS = ('src/**/*.xacro', 'src/**/*.urdf', 'src/**/*.world',
            'src/**/*.sdf', 'src/**/package.xml')


def repair(text: str) -> str:
    """Remove '--' from comment bodies while protecting the delimiters themselves."""
    text = text.replace('<!--', '\x01').replace('-->', '\x02')
    text = re.sub(r'-{3,}', lambda m: '=' * len(m.group()), text)   # ---- underlines
    text = text.replace('--', ' -')                                  # em-dash usage
    return text.replace('\x01', '<!--').replace('\x02', '-->')


def main() -> int:
    fix = '--fix' in sys.argv
    files = sorted({p for pat in PATTERNS for p in ROOT.glob(pat)})
    if not files:
        print('No XML files found.')
        return 1

    failed = []
    for path in files:
        original = path.read_text()
        if fix:
            repaired = repair(original)
            if repaired != original:
                path.write_text(repaired)
                print(f'  fixed   {path.relative_to(ROOT)}')
        try:
            ET.parse(path)
        except ET.ParseError as exc:
            print(f'  FAILED  {path.relative_to(ROOT)}: {exc}')
            failed.append(path)

    print(f'{len(files) - len(failed)}/{len(files)} files valid.')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
