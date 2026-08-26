# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""Style check for the Python sources of this package."""

import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    ament_flake8 = pytest.importorskip('ament_flake8.main')
    return_code, errors = ament_flake8.main_with_errors(argv=[])
    assert return_code == 0, \
        '\n'.join(['found %d code style errors / warnings:' % len(errors)] + errors)
