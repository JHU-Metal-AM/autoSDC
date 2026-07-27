from __future__ import annotations

import importlib
from collections.abc import Callable

SCRIPT_REGISTRY: dict[str, str] = {
    "demo": "scripts.demo:script_demo",
    "demo2": "scripts.demo2:script_demo2_refactored",
    "wiggle_ls": "scripts.wiggle_ls:script_wiggle_ls",
    "exit": "scripts.program:program_exit",
    "quit": "scripts.program:program_exit",
}


def load_script_by_id(
    script_id: str, script_registry: dict[str, str] | None = None
) -> Callable[..., None] | None:
    """Load a script function by id from a module path registry."""
    registry = SCRIPT_REGISTRY if script_registry is None else script_registry
    script_path = registry.get(script_id)
    if script_path is None:
        return None

    try:
        module_path, function_name = script_path.split(":")
    except ValueError:
        print(
            f"ERROR: Script '{script_id}' has invalid registry path '{script_path}'."
        )
        return None

    try:
        module = importlib.import_module(module_path)
        script_function = getattr(module, function_name)
    except (ImportError, AttributeError) as error:
        print(f"ERROR: Failed to load script '{script_id}': {error}")
        return None

    if not callable(script_function):
        print(f"ERROR: Script target '{script_path}' is not callable.")
        return None

    return script_function
