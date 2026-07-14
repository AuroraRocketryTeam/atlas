"""Configuration helpers for RocketPy HiL simulations.

This module keeps the JSON parsing/resolution logic out of hil_rocketpy.py.

Rules:
- JSON sections named like RocketPy constructors/methods are intended to be
  passed with **kwargs.
- Keys starting with '_' are config-only helper values and are not forwarded
  to RocketPy.
- Plain numbers are used as-is.
- Plain strings are kept as strings.
- Strings starting with '=' are evaluated as numeric formulas.
- Formula references use '$path.to.value'. Local dict references are resolved
  first, then absolute root-config references are used.
"""

from __future__ import annotations

import ast
import json
import math
import re
from pathlib import Path
from typing import Any


FILE_PATH_KEYS = {
    "file",
    "thrust_source",
    "power_off_drag",
    "power_on_drag",
}

TUPLE_KEYS = {
    "date",
    "dry_inertia",
    "inertia",
    "reshape_thrust_curve",
    "noise",
    "orientation",
}

PAIR_LIST_KEYS = {
    "wind_u",
    "wind_v",
    "pressure",
    "temperature",
}

_ALLOWED_FORMULA_NAMES = {
    "pi": math.pi,
}

# Keep this intentionally small. These functions are pure numeric helpers only.
_ALLOWED_FORMULA_FUNCTIONS = {
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "exp": math.exp,
    "sqrt": math.sqrt,
    "log": math.log,
    "log10": math.log10,
    "radians": math.radians,
    "degrees": math.degrees,
}

_REFERENCE_RE = re.compile(
    r"\$([A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z0-9_]+)*)"
)


def load_hil_config(config_path: Path) -> dict[str, Any]:
    """Load a HiL RocketPy JSON config and resolve numeric formulas."""
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    return resolve_numeric_formulas(cfg)


def cfg_path(config_dir: Path, path_value: str) -> str:
    """Resolve a file path stored in the selected rocket config folder."""
    if path_value == "GFS":
        return path_value

    path = Path(path_value)
    if path.is_absolute():
        return str(path)

    return str(config_dir / path)


def rocketpy_kwargs(section: dict[str, Any]) -> dict[str, Any]:
    """
    Return arguments to forward to RocketPy.

    Keys starting with '_' are config-only helper values and are not forwarded
    to RocketPy.
    """
    return {
        key: value
        for key, value in section.items()
        if not key.startswith("_")
    }


def looks_like_pair_list(value: Any) -> bool:
    return (
        isinstance(value, list)
        and all(isinstance(item, list) and len(item) == 2 for item in value)
    )


def normalize_rocketpy_value(config_dir: Path, key: str, value: Any) -> Any:
    """Apply only mechanical JSON-to-Python conversions before **kwargs."""
    if key in FILE_PATH_KEYS and isinstance(value, str):
        return cfg_path(config_dir, value)

    if key in TUPLE_KEYS and isinstance(value, list):
        return tuple(value)

    if key in PAIR_LIST_KEYS and looks_like_pair_list(value):
        return [tuple(item) for item in value]

    if isinstance(value, list):
        return [normalize_rocketpy_value(config_dir, key, item) for item in value]

    if isinstance(value, dict):
        return {
            nested_key: normalize_rocketpy_value(config_dir, nested_key, nested_value)
            for nested_key, nested_value in value.items()
        }

    return value


def prepare_rocketpy_kwargs(section: dict[str, Any], config_dir: Path) -> dict[str, Any]:
    """Filter config-only keys and normalize JSON values for RocketPy."""
    return {
        key: normalize_rocketpy_value(config_dir, key, value)
        for key, value in rocketpy_kwargs(section).items()
    }


def _get_path_value(root: Any, dotted_path: str) -> Any:
    value = root

    for part in dotted_path.split("."):
        if isinstance(value, list):
            value = value[int(part)]
        else:
            value = value[part]

    return value


def _set_path_value(root: Any, dotted_path: str, new_value: Any) -> None:
    parts = dotted_path.split(".")
    value = root

    for part in parts[:-1]:
        if isinstance(value, list):
            value = value[int(part)]
        else:
            value = value[part]

    last = parts[-1]
    if isinstance(value, list):
        value[int(last)] = new_value
    else:
        value[last] = new_value


def _resolve_path_value(
    root: dict[str, Any],
    local: Any,
    dotted_path: str,
    cache: dict[Any, float],
    resolving: set[Any],
) -> float:
    """
    Resolve a $reference used by a formula.

    Examples:
    - $RocketV2.mass: absolute lookup from the root config.
    - $add_tail.length: absolute lookup from the root config.
    - $_cd: local lookup inside the current dict.
    - $lag_se: local lookup if present in current dict, otherwise root lookup.

    If the referenced value is itself a formula, it is resolved first, cached,
    and written back into the config to avoid stale reads.
    """
    first_part = dotted_path.split(".", 1)[0]

    if isinstance(local, dict) and first_part in local:
        lookup_root = local
        cache_key = ("local", id(local), dotted_path)
    else:
        lookup_root = root
        cache_key = ("root", dotted_path)

    if cache_key in cache:
        return cache[cache_key]

    if cache_key in resolving:
        raise ValueError(f"Cyclic formula reference detected while resolving ${dotted_path}")

    resolving.add(cache_key)

    value = _get_path_value(lookup_root, dotted_path)

    if isinstance(value, str) and value.startswith("="):
        value = _eval_numeric_formula(
            value,
            root=root,
            local=local,
            cache=cache,
            resolving=resolving,
        )
        _set_path_value(lookup_root, dotted_path, value)

    resolving.remove(cache_key)

    if not isinstance(value, (int, float)):
        raise TypeError(f"Formula reference ${dotted_path} is not numeric: {value!r}")

    numeric_value = float(value)
    cache[cache_key] = numeric_value
    return numeric_value


def _validate_formula_ast(tree: ast.AST, expression: str) -> None:
    allowed_nodes = (
        ast.Expression,
        ast.BinOp,
        ast.UnaryOp,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
        ast.Pow,
        ast.USub,
        ast.UAdd,
        ast.Constant,
        ast.Load,
        ast.Name,
        ast.Call,
    )

    for node in ast.walk(tree):
        if not isinstance(node, allowed_nodes):
            raise ValueError(f"Unsupported formula syntax: {expression!r}")

        if isinstance(node, ast.Name):
            if (
                node.id not in _ALLOWED_FORMULA_NAMES
                and node.id not in _ALLOWED_FORMULA_FUNCTIONS
            ):
                raise ValueError(f"Unsupported name in formula: {node.id!r}")

        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name):
                raise ValueError("Only simple whitelisted function calls are allowed")

            if node.func.id not in _ALLOWED_FORMULA_FUNCTIONS:
                raise ValueError(f"Function not allowed in formula: {node.func.id!r}")

            if node.keywords:
                raise ValueError("Keyword arguments are not allowed in formulas")

        if isinstance(node, ast.Constant) and not isinstance(node.value, (int, float)):
            raise ValueError(f"Only numeric constants are allowed in formulas: {expression!r}")


def _eval_numeric_formula(
    expression: str,
    root: dict[str, Any],
    local: Any,
    cache: dict[Any, float],
    resolving: set[Any],
) -> float:
    """
    Evaluate a small numeric-only formula.

    Supported syntax examples:
    - = $RocketV2._dry_mass + $RocketV2._ballast
    - = $add_tail.position + $add_tail.length
    - = $SolidMotor._grain_mass / (pi * ($SolidMotor.grain_outer_radius^2 - $SolidMotor.grain_initial_inner_radius^2) * $SolidMotor.grain_initial_height)
    - = $_cd * $_area
    - = sin(radians($_wind_heading_deg))

    This is intentionally not a Python eval replacement. It only accepts
    arithmetic operators, parentheses, numeric constants, whitelisted math
    functions, pi, and $field references.
    """
    if not expression.startswith("="):
        raise ValueError(f"Formula must start with '=': {expression!r}")

    expr = expression[1:].strip().replace("^", "**")

    def replace_reference(match: re.Match[str]) -> str:
        ref = match.group(1)
        value = _resolve_path_value(
            root=root,
            local=local,
            dotted_path=ref,
            cache=cache,
            resolving=resolving,
        )
        return repr(float(value))

    expr = _REFERENCE_RE.sub(replace_reference, expr)

    tree = ast.parse(expr, mode="eval")
    _validate_formula_ast(tree, expression)

    formula_env = {
        **_ALLOWED_FORMULA_NAMES,
        **_ALLOWED_FORMULA_FUNCTIONS,
    }

    # Safe after AST validation: only numeric constants, arithmetic operators,
    # whitelisted function calls and whitelisted names survive here.
    return eval(
        compile(tree, "<config-formula>", "eval"),
        {"__builtins__": {}},
        formula_env,
    )


def resolve_numeric_formulas(config: dict[str, Any]) -> dict[str, Any]:
    """
    Resolve only derived numeric fields in-place and return the config.

    Plain JSON numbers are used as-is.
    Plain strings are kept as strings.
    Strings starting with '=' are evaluated as numeric formulas.

    Formula references are resolved through a cache and cycle detector, so
    formulas may reference other formulas without reading stale unresolved values.
    """
    cache: dict[Any, float] = {}
    resolving: set[Any] = set()

    def walk(value: Any, root: dict[str, Any], local: Any) -> Any:
        if isinstance(value, dict):
            for key, item in list(value.items()):
                value[key] = walk(item, root, value)
            return value

        if isinstance(value, list):
            for i, item in enumerate(value):
                value[i] = walk(item, root, local)
            return value

        if isinstance(value, str) and value.startswith("="):
            return _eval_numeric_formula(
                value,
                root=root,
                local=local,
                cache=cache,
                resolving=resolving,
            )

        return value

    return walk(config, config, config)
