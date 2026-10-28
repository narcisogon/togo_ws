"""Configuration loading for the lunar world generator.

A configuration is a YAML file (see config/apollo16_core.yaml). It may define named
``presets`` whose keys are deep-merged over the base configuration, e.g. ``fast``.
Values are read through ``Config`` objects that allow both ``cfg["a"]["b"]`` and
``cfg.a.b`` access. Relative paths in ``paths`` are resolved against the package root
(the folder containing ``lunar_world_gen/`` and ``config/``) and ``~`` is expanded.
"""
from __future__ import annotations

import copy
import hashlib
import json
import zlib
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parent.parent  # .../src/lunar_world_gen
DEFAULT_CONFIG = PACKAGE_ROOT / "config" / "apollo16_core.yaml"


class Config(dict):
    """dict with attribute access; nested dicts are wrapped on access."""

    def __getattr__(self, name: str) -> Any:
        try:
            return _wrap(self[name])
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __getitem__(self, key: str) -> Any:
        return _wrap(dict.__getitem__(self, key))

    def get(self, key: str, default: Any = None) -> Any:  # type: ignore[override]
        return _wrap(dict.get(self, key, default))

    def require(self, dotted: str) -> Any:
        """Look up 'a.b.c'; raise a clear error if missing."""
        node: Any = self
        for part in dotted.split("."):
            if not isinstance(node, dict) or part not in node:
                raise KeyError(f"config key '{dotted}' is missing (failed at '{part}')")
            node = dict.__getitem__(node, part)
        return _wrap(node)

    def to_plain(self) -> dict:
        return json.loads(json.dumps(self, default=str))


def _wrap(value: Any) -> Any:
    if isinstance(value, dict) and not isinstance(value, Config):
        return Config(value)
    return value


def deep_merge(base: dict, over: dict) -> dict:
    """Return base with over merged in (dicts merge recursively, everything else replaces)."""
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _set_dotted(d: dict, dotted: str, value: Any) -> None:
    parts = dotted.split(".")
    node = d
    for p in parts[:-1]:
        node = node.setdefault(p, {})
    node[parts[-1]] = value


def load_config(path: str | Path | None = None, preset: str | None = None,
                overrides: Iterable[str] | None = None) -> Config:
    """Load a config file, apply a preset and 'dotted.key=yaml_value' overrides."""
    path = Path(path) if path else DEFAULT_CONFIG
    raw = yaml.safe_load(path.read_text())
    presets = raw.pop("presets", {}) or {}
    preset = preset or raw.get("world", {}).get("preset")
    if preset:
        if preset not in presets:
            raise KeyError(f"unknown preset '{preset}' (available: {sorted(presets)})")
        raw = deep_merge(raw, presets[preset])
        raw.setdefault("world", {})["preset"] = preset
    for item in overrides or []:
        key, _, val = item.partition("=")
        if not _:
            raise ValueError(f"override must be key=value, got {item!r}")
        _set_dotted(raw, key.strip(), yaml.safe_load(val))
    _resolve_paths(raw)
    raw["_source"] = {"config_file": str(path), "preset": preset, "overrides": list(overrides or [])}
    return Config(raw)


def _resolve_paths(raw: dict) -> None:
    paths = raw.get("paths", {})
    for key, value in list(paths.items()):
        if isinstance(value, str):
            p = Path(value).expanduser()
            if not p.is_absolute():
                p = (PACKAGE_ROOT / p).resolve()
            paths[key] = str(p)


def config_hash(cfg: dict, keys: Iterable[str] | None = None) -> str:
    """Stable short hash of the whole config or selected top-level sections (for stage caching)."""
    plain = {k: cfg[k] for k in keys} if keys else {k: v for k, v in cfg.items() if not k.startswith("_")}
    blob = json.dumps(plain, sort_keys=True, default=str).encode()
    return hashlib.sha256(blob).hexdigest()[:16]


def rng(cfg: dict, name: str) -> np.random.Generator:
    """Deterministic, independent random stream per named component.

    The same (world.seed, name) always gives the same stream, and adding a new component
    never changes the streams of existing ones.
    """
    seed = int(cfg["world"]["seed"])
    return np.random.default_rng(np.random.SeedSequence([seed, zlib.crc32(name.encode())]))
