# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: run the ament pep257 docstring linter over the package.

"""Docstring check (ament_pep257)."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Assert every module, class and function is documented."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
