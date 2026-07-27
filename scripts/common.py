from __future__ import annotations

import importlib
import sys
from types import ModuleType


def get_station_module() -> ModuleType:
    """Get the active station module without re-importing the running script."""
    main_module = sys.modules.get("__main__")
    if main_module is not None and hasattr(main_module, "send_and_listen"):
        return main_module

    station_module = sys.modules.get("SDCStation")
    if station_module is not None:
        return station_module

    return importlib.import_module("SDCStation")
