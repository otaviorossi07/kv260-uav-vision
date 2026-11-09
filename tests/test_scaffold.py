"""Phase 1 smoke test: the package layout imports."""

import importlib

import pytest


@pytest.mark.parametrize(
    "module",
    [
        "fc_link",
        "fc_link.px4",
        "fc_link.ardupilot",
        "ingest",
        "perception",
        "perception.aruco",
        "perception.dpu",
    ],
)
def test_package_imports(module):
    importlib.import_module(module)
