"""Shared test setup.

Firmware modules (firmware/*.py) and the client (tapster.py) import as
top-level modules. CircuitPython-only modules are replaced by fakes.
Hardware tests are skipped unless pytest runs with --hardware.
"""

import sys

import pytest

from fakes import FIRMWARE, ROOT, install_circuitpython_stubs

# Appended, not prepended: firmware/code.py must not shadow the standard
# library's `code` module (pdb imports it).
for path in (str(ROOT), str(FIRMWARE)):
    if path not in sys.path:
        sys.path.append(path)

install_circuitpython_stubs()


def pytest_addoption(parser):
    parser.addoption(
        "--hardware", action="store_true",
        help="run end-to-end tests on a connected robot (moves it)")


def pytest_collection_modifyitems(config, items):
    if config.getoption("--hardware"):
        return
    skip = pytest.mark.skip(reason="needs a connected robot: make test-hw")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)
