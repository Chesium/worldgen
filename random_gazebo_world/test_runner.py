from __future__ import annotations

import os
import sys

import pytest


def main() -> None:
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    raise SystemExit(pytest.main(["-q", "-p", "pytest", *sys.argv[1:]]))
