# -*- coding: utf-8 -*-
"""Style check, skipping the vendored ping-python subset."""

from ament_flake8.main import main_with_errors

import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # brping/ is vendored third party code and is kept close to upstream so it
    # stays easy to diff, so it is not held to this package's style.
    rc, errors = main_with_errors(
        argv=['--exclude', 'omniscan_sidescan/brping'])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
