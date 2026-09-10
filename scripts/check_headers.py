#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: assert that every source file carries the proprietary header.

"""
Check the proprietary licence header on every source file.

ament_copyright only knows the open-source licences it ships with, so it
cannot recognise our proprietary notice. This script replaces it and is used
from two places: the pre-commit hook, which passes the staged files as
arguments, and test/test_header.py, which calls check_paths() with the whole
package.

Usage:
    check_headers.py            check every source file in the repository
    check_headers.py FILE ...   check only the given files
"""

from __future__ import annotations

from pathlib import Path
import sys

#: Sentences that must all appear, in order, in the first lines of a file.
REQUIRED = (
    'Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.',
    'Proprietary and confidential. Unauthorized copying, distribution or use',
    'of this file, via any medium, is strictly prohibited.',
)

#: Extensions the header is mandatory on.
SUFFIXES = (
    '.py', '.yaml', '.yml', '.sh', '.cfg', '.cpp', '.hpp', '.c', '.h',
    '.xml', '.xacro', '.urdf', '.rviz', '.msg', '.md',
)

#: Files without a suffix that still carry the header.
NAMES = ('CMakeLists.txt', '.gitignore', '.pre-commit-config.yaml')

#: Directory names never scanned: build output, caches and ament markers.
SKIP_DIRS = frozenset({
    '.git', '.tmp_ws', '__pycache__', '.pytest_cache', 'build', 'install',
    'log', '.venv', 'resource', 'meshes', 'node_modules',
})

#: Files that carry no header: the notice itself, and long-form docs.
SKIP_NAMES = frozenset({'LICENSE'})

#: Third-party files kept verbatim under their own licence.
THIRD_PARTY: tuple[str, ...] = ()

#: How many leading lines may precede the header (shebang, XML declaration).
LOOKAHEAD = 12


def is_source(path: Path, root: Path) -> bool:
    """Return True when the header is mandatory on this path."""
    if any(part in SKIP_DIRS for part in path.parts):
        return False
    if path.name in SKIP_NAMES:
        return False
    try:
        relative = path.resolve().relative_to(root).as_posix()
    except ValueError:
        relative = path.as_posix()
    if relative in THIRD_PARTY:
        return False
    # Long-form documentation is prose, not source: docs/ markdown is exempt.
    if path.suffix == '.md' and 'docs' in path.parts:
        return False
    return path.suffix in SUFFIXES or path.name in NAMES


def has_header(path: Path) -> bool:
    """Return True when the file starts with the proprietary header."""
    try:
        with path.open(encoding='utf-8') as handle:
            head = ''.join(next(handle, '') for _ in range(LOOKAHEAD))
    except (OSError, UnicodeDecodeError):
        return False
    position = 0
    for sentence in REQUIRED:
        position = head.find(sentence, position)
        if position < 0:
            return False
        position += len(sentence)
    return True


def collect(paths: list[Path], root: Path) -> list[Path]:
    """Expand directories into the source files they contain."""
    files: list[Path] = []
    for path in paths:
        if path.is_dir():
            files.extend(
                p for p in sorted(path.rglob('*')) if is_source(p, root))
        elif is_source(path, root):
            files.append(path)
    return files


def check_paths(paths: list[Path], root: Path | None = None) -> list[Path]:
    """Return the files that are missing the header."""
    root = root or Path(__file__).resolve().parent.parent
    return [path for path in collect(paths, root) if not has_header(path)]


def main(argv: list[str] | None = None) -> int:
    """Run the check and report the offenders."""
    argv = sys.argv[1:] if argv is None else argv
    root = Path(__file__).resolve().parent.parent
    roots = [Path(a) for a in argv] or [root]
    missing = check_paths(roots, root)
    for path in missing:
        print(f'missing proprietary header: {path}')
    if missing:
        print(f'{len(missing)} file(s) without the proprietary header')
    return 1 if missing else 0


if __name__ == '__main__':
    sys.exit(main())
