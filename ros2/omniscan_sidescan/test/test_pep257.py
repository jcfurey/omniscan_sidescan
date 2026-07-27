# -*- coding: utf-8 -*-
"""Docstring check, skipping the vendored ping-python subset."""

from ament_pep257.main import main

import pytest

# ament's convention picks a docstring dialect this package does not follow:
#   D213  wants the summary on the line after the opening quotes
#   D406/D407/D413  want numpy style sections with dashed underlines
# The docstrings here put the summary on the first line and use Google style
# 'Args:'/'Returns:'/'Raises:' sections, matching the ROS 1 package this was
# ported from.  Everything else in the convention is enforced.
DOCSTRING_DIALECT = ['D213', 'D406', 'D407', 'D413']


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # See the note in test_flake8.py about brping/.
    rc = main(argv=['.', 'test',
                    '--exclude', 'omniscan_sidescan/brping',
                    '--add-ignore'] + DOCSTRING_DIALECT)
    assert rc == 0, 'Found code style errors / warnings'
