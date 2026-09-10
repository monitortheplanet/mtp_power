# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: fail the build when a source file loses the proprietary header.

"""
Proprietary header check.

ament_copyright does not know our proprietary notice, so this test replaces
it. It calls scripts/check_headers.py, the same script the pre-commit hook
runs, so a file can never pass one and fail the other.
"""

import importlib.util
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
CHECKER = PACKAGE_ROOT / 'scripts' / 'check_headers.py'


def _load_checker():
    """Import check_headers.py by path, without installing it."""
    spec = importlib.util.spec_from_file_location('check_headers', CHECKER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_checker_script_is_present():
    """The pre-commit hook and this test must call the same script."""
    assert CHECKER.is_file(), f'{CHECKER} is missing'


def test_every_source_file_has_the_proprietary_header():
    """Every source file in the repository carries the header."""
    checker = _load_checker()
    missing = checker.check_paths([PACKAGE_ROOT], PACKAGE_ROOT)
    assert not missing, 'missing proprietary header: ' + ', '.join(
        str(path.relative_to(PACKAGE_ROOT)) for path in missing)


def test_the_checker_rejects_a_file_without_the_header(tmp_path):
    """A file without the header is reported, not silently accepted."""
    checker = _load_checker()
    offender = tmp_path / 'no_header.py'
    offender.write_text('print("hello")\n', encoding='utf-8')
    assert checker.check_paths([offender], tmp_path) == [offender]
