"""Configuration helpers for RocketPy HIL simulations.

This module owns the mechanical conversion from JSON to Python/RocketPy
arguments. The launcher should not know about JSON quirks, path resolution,
formula evaluation, or how clean/noisy sensor profiles are assembled.

NOTE: JSONC is JSON with support for comments

Configuration rules:
- JSON sections named like RocketPy constructors/methods are forwarded with
  ``**kwargs`` after normalization.
- Keys starting with ``_`` are HIL-only metadata and are never forwarded to
  RocketPy constructors.
- Plain numbers and strings are preserved.
- Strings starting with ``=`` are numeric formulas.
- Formula references use ``$path.to.value``. Local references are resolved
  before root-config references.
- ``Sensors`` must define profiles under ``_profiles``:
  ``{"_profiles": {"clean": {...}, "noisy": {"_inherits": "clean", ...}}}``.
- Sections may define ordered method calls under ``_calls``. Because the key
  starts with ``_``, method calls never leak into constructor kwargs.
- Mandatory sections are validated explicitly by callers through
  ``require_config_section`` instead of relying on raw ``KeyError`` messages.
"""

from __future__ import annotations

import ast
import copy
import jsonc
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


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

DEFAULT_CLEAN_SENSOR_PROFILE: dict[str, dict[str, Any]] = {
    "Accelerometer": {
        "consider_gravity": True,
        "orientation": [0, 0, 0],
        "noise_density": 0.0,
        "random_walk_density": 0.0,
        "constant_bias": 0.0,
        "temperature_bias": 0.0,
        "temperature_scale_factor": 0.0,
        "cross_axis_sensitivity": 0.0,
        "name": "Clean Accelerometer",
        "_position": 0,
    },
    "Barometer": {
        "noise_density": 0.0,
        "random_walk_density": 0.0,
        "constant_bias": 0.0,
        "temperature_bias": 0.0,
        "temperature_scale_factor": 0.0,
        "name": "Clean Barometer",
        "_position": 0,
    },
    "GnssReceiver": {
        "position_accuracy": 0.0,
        "altitude_accuracy": 0.0,
        "name": "Clean GPS",
        "_position": 0,
    },
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


@dataclass(frozen=True)
class PreparedSensorConfig:
    """Fully resolved RocketPy sensor configuration."""

    sensor_type: str
    constructor_kwargs: dict[str, Any]
    position: float
    profile_name: str

    def metadata(self) -> dict[str, Any]:
        """Return JSON-safe metadata for capture files."""
        return {
            "profile": self.profile_name,
            "position": self.position,
            "args": copy.deepcopy(self.constructor_kwargs),
        }


def load_hil_config(config_path: Path) -> dict[str, Any]:
    """Load a HIL RocketPy JSON config and resolve numeric formulas."""
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = jsonc.load(config_file)

    if not isinstance(config, dict):
        raise TypeError(f"HIL config root must be a JSON object: {config_path}")

    return resolve_numeric_formulas(config)


def require_config_section(config: dict[str, Any], section_name: str) -> dict[str, Any]:
    """Return a mandatory JSON object section with a clear config error."""
    if section_name not in config:
        raise KeyError(f"Missing required HIL config section: {section_name}")

    section = config[section_name]
    if not isinstance(section, dict):
        raise TypeError(
            f"HIL config section {section_name!r} must be a JSON object, "
            f"got {type(section).__name__}"
        )

    return section


def require_config_list(config: dict[str, Any], section_name: str) -> list[Any]:
    """Return a mandatory JSON array section with a clear config error."""
    if section_name not in config:
        raise KeyError(f"Missing required HIL config section: {section_name}")

    section = config[section_name]
    if not isinstance(section, list):
        raise TypeError(
            f"HIL config section {section_name!r} must be a JSON array, "
            f"got {type(section).__name__}"
        )

    return section


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

    Keys starting with ``_`` are HIL/config metadata and are not forwarded.
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
    """Apply only mechanical JSON-to-Python conversions before ``**kwargs``."""
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


def prepare_rocketpy_kwargs(
    section: dict[str, Any],
    config_dir: Path,
) -> dict[str, Any]:
    """Filter config-only keys and normalize JSON values for RocketPy."""
    return {
        key: normalize_rocketpy_value(config_dir, key, value)
        for key, value in rocketpy_kwargs(section).items()
    }


def apply_configured_calls(
    target: Any,
    section: dict[str, Any],
    config_dir: Path,
    section_name: str,
) -> list[dict[str, Any]]:
    """
    Apply ordered method calls declared in ``section["_calls"]``.

    Expected shape:

    ``{"_calls": [{"method": "method_name", "args": [], "kwargs": {}}]}``

    ``args`` and ``kwargs`` are optional. Values pass through the same
    normalization used for RocketPy constructor kwargs, so paths and tuples are
    handled consistently.
    """
    calls = section.get("_calls", [])
    if calls == []:
        return []

    if not isinstance(calls, list):
        raise TypeError(f"{section_name}._calls must be a JSON array")

    applied_calls: list[dict[str, Any]] = []

    for index, call_config in enumerate(calls):
        call_path = f"{section_name}._calls[{index}]"

        if not isinstance(call_config, dict):
            raise TypeError(f"{call_path} must be a JSON object")

        method_name = call_config.get("method")
        if not isinstance(method_name, str) or not method_name:
            raise TypeError(f"{call_path}.method must be a non-empty string")

        if not hasattr(target, method_name):
            raise AttributeError(f"{section_name} object has no method {method_name!r}")

        method = getattr(target, method_name)
        if not callable(method):
            raise TypeError(f"{section_name}.{method_name} is not callable")

        raw_args = call_config.get("args", [])
        raw_kwargs = call_config.get("kwargs", {})

        if not isinstance(raw_args, list):
            raise TypeError(f"{call_path}.args must be a JSON array when provided")

        if not isinstance(raw_kwargs, dict):
            raise TypeError(f"{call_path}.kwargs must be a JSON object when provided")

        args = [
            normalize_rocketpy_value(config_dir, "args", arg)
            for arg in raw_args
        ]
        kwargs = prepare_rocketpy_kwargs(raw_kwargs, config_dir)

        method(*args, **kwargs)
        applied_calls.append(
            {
                "method": method_name,
                "args": copy.deepcopy(args),
                "kwargs": copy.deepcopy(kwargs),
            }
        )

    return applied_calls


def prepare_sensor_configs(
    config: dict[str, Any],
    config_dir: Path,
    requested_profile_name: str,
    default_sampling_rate_hz: int,
    known_sensor_types: Iterable[str],
) -> tuple[str, list[PreparedSensorConfig]]:
    """
    Resolve a named sensor profile into RocketPy constructor arguments.

    Clean profile:
        Uses ``DEFAULT_CLEAN_SENSOR_PROFILE`` and applies optional JSON
        overrides. This makes the all-zero no-noise baseline centralized.

    Other profiles:
        Profile names are defined entirely by the JSON config. Non-clean
        profiles inherit ``clean`` by default, so custom configs can override
        only the real noise parameters instead of duplicating every clean field.

    A profile can disable inheritance with ``"_inherits": null`` or inherit
    another profile with ``"_inherits": "profile_name"``.
    """
    sensors_section = require_config_section(config, "Sensors")

    profile_map = _extract_sensor_profile_map(sensors_section)
    if "clean" not in profile_map:
        raise ValueError("Sensors._profiles.clean is required, even when it only overrides names")

    if not isinstance(requested_profile_name, str) or not requested_profile_name:
        raise TypeError("requested_profile_name must be a non-empty string")

    profile_name = requested_profile_name
    if profile_name not in profile_map:
        available = ", ".join(sorted(profile_map.keys())) or "none"
        raise ValueError(
            f"Unknown sensor profile {requested_profile_name!r}. "
            f"Available profiles: {available}"
        )

    if "_default_position" not in sensors_section:
        raise KeyError("Sensors._default_position is required")

    default_position = sensors_section["_default_position"]
    resolved_profile = _resolve_sensor_profile(profile_map, profile_name)
    known_sensor_type_set = set(known_sensor_types)

    prepared_sensors: list[PreparedSensorConfig] = []

    for sensor_type, sensor_config in resolved_profile.items():
        if sensor_type.startswith("_"):
            continue

        if sensor_type not in known_sensor_type_set:
            known = ", ".join(sorted(known_sensor_type_set))
            raise ValueError(f"Unknown sensor type {sensor_type!r}. Known sensor types: {known}")

        if not isinstance(sensor_config, dict):
            raise TypeError(
                f"Sensors._profiles.{profile_name}.{sensor_type} must be a JSON object"
            )

        if "sampling_rate" in sensor_config:
            raise ValueError(
                f"Sensors._profiles.{profile_name}.{sensor_type}.sampling_rate is not supported. "
                "Use --sampling-rate so all HIL components run at the same rate."
            )

        constructor_kwargs = prepare_rocketpy_kwargs(sensor_config, config_dir)
        constructor_kwargs["sampling_rate"] = default_sampling_rate_hz

        prepared_sensors.append(
            PreparedSensorConfig(
                sensor_type=sensor_type,
                constructor_kwargs=constructor_kwargs,
                position=sensor_config.get("_position", default_position),
                profile_name=profile_name,
            )
        )

    return profile_name, prepared_sensors


def _extract_sensor_profile_map(sensors_section: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """
    Return the available sensor profiles from the canonical JSON layout.
    """
    if "_profiles" not in sensors_section:
        raise ValueError(
            "Sensors must define profiles under Sensors._profiles. "
            "Expected: {'Sensors': {'_profiles': {'clean': {...}, 'noisy': {...}}}}"
        )

    profiles = sensors_section["_profiles"]
    if not isinstance(profiles, dict):
        raise TypeError("Sensors._profiles must be a JSON object")

    profile_map: dict[str, dict[str, Any]] = {}

    for profile_name, profile_config in profiles.items():
        if not isinstance(profile_name, str) or not profile_name:
            raise TypeError("Sensors._profiles keys must be non-empty strings")

        if not isinstance(profile_config, dict):
            raise TypeError(f"Sensors._profiles.{profile_name} must be a JSON object")

        profile_map[profile_name] = copy.deepcopy(profile_config)

    return profile_map


def _resolve_sensor_profile(
    profile_map: dict[str, dict[str, Any]],
    profile_name: str,
    resolving: set[str] | None = None,
) -> dict[str, dict[str, Any]]:
    """Resolve inheritance and merge defaults for one sensor profile."""
    if resolving is None:
        resolving = set()

    if profile_name in resolving:
        chain = " -> ".join((*resolving, profile_name))
        raise ValueError(f"Cyclic sensor profile inheritance detected: {chain}")

    if profile_name == "clean":
        base_profile = copy.deepcopy(DEFAULT_CLEAN_SENSOR_PROFILE)
        override_profile = copy.deepcopy(profile_map.get("clean", {}))
        return _deep_merge_sensor_profiles(base_profile, override_profile)

    if profile_name not in profile_map:
        raise ValueError(f"Missing sensor profile: {profile_name}")

    resolving.add(profile_name)
    override_profile = copy.deepcopy(profile_map[profile_name])
    inherited_profile_name = override_profile.pop("_inherits", "clean")

    if inherited_profile_name is None:
        base_profile = {}
    else:
        if not isinstance(inherited_profile_name, str):
            raise TypeError(
                f"Sensors._profiles.{profile_name}._inherits must be a string or null"
            )

        base_profile = _resolve_sensor_profile(
            profile_map,
            inherited_profile_name,
            resolving,
        )

    resolving.remove(profile_name)
    return _deep_merge_sensor_profiles(base_profile, override_profile)


def _deep_merge_sensor_profiles(
    base_profile: dict[str, dict[str, Any]],
    override_profile: dict[str, dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    """Merge sensor-profile dictionaries without mutating either input."""
    merged = copy.deepcopy(base_profile)

    for sensor_type, override_config in override_profile.items():
        if sensor_type.startswith("_"):
            merged[sensor_type] = copy.deepcopy(override_config)
            continue

        base_config = merged.get(sensor_type, {})
        if not isinstance(base_config, dict) or not isinstance(override_config, dict):
            merged[sensor_type] = copy.deepcopy(override_config)
            continue

        sensor_config = copy.deepcopy(base_config)
        sensor_config.update(copy.deepcopy(override_config))
        merged[sensor_type] = sensor_config

    return merged


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
    Resolve a ``$reference`` used by a formula.

    Examples:
    - ``$RocketV2.mass``: absolute lookup from the root config.
    - ``$add_tail.length``: absolute lookup from the root config.
    - ``$_cd``: local lookup inside the current dict.
    - ``$lag_se``: local lookup if present in current dict, otherwise root lookup.
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
    - ``= $RocketV2._dry_mass + $RocketV2._ballast``
    - ``= $add_tail.position + $add_tail.length``
    - ``= $_cd * $_area``
    - ``= sin(radians($_wind_heading_deg))``
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
    Resolve derived numeric fields in-place and return the config.

    Formula references are resolved through a cache and cycle detector, so
    formulas may reference other formulas without reading stale unresolved
    values.
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
