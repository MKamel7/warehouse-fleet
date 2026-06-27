"""Make the package's scripts/ importable from tests (so ``import wb_common``
works whether pytest runs from the source tree or the colcon build space)."""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.normpath(os.path.join(_HERE, '..', 'scripts'))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)
