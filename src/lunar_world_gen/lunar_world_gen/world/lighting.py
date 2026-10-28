"""Scene lighting for the Apollo 16 world (docs/DESIGN.md section 8).

Two modes, both built from ``cfg.lighting``:

* ``realistic`` - one directional sun (``lighting.sun_events[sun_event]``, casting shadows),
  a faint bluish earthshine from the Earth's direction (no shadows), a near-black ambient
  and a black background. Shadows on the Moon are almost black: there is no sky light.
* ``inspection`` - the ``inspection.sun_event`` sun (higher), plus an overhead fill light
  and a raised ambient, so camera images and the GUI show the terrain everywhere.

Conventions
-----------
World frame is ENU (x east, y north, z up). Azimuth is measured from north through east,
elevation above the local horizon. A Gazebo directional light's ``<direction>`` is the
propagation direction of the light, i.e. MINUS the unit vector pointing to the source::

    to_source = (sin(az) cos(el), cos(az) cos(el), sin(el))
    direction = -to_source

The config stores the sun's direction explicitly (``sun_events[event][2]``, computed by the
calibration report) together with its elevation and azimuth; :func:`sun_light` checks the two
agree (angle < ``DIRECTION_TOLERANCE_DEG``) and that the light points downward, then uses the
normalised config vector.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from ..config import Config

#: largest accepted angle between a configured direction and the one implied by el/az
DIRECTION_TOLERANCE_DEG = 0.5

#: light colours (linear RGB). The regolith albedo is white-balanced to sunlight, so the
#: sun is neutral white; earthshine is bluish (Earth's integrated colour).
SUN_COLOR = (1.0, 1.0, 1.0)
EARTHSHINE_COLOR = (0.55, 0.65, 0.90)
FILL_COLOR = (1.0, 1.0, 1.0)

MODES = ("realistic", "inspection")


@dataclass
class LightSpec:
    """One SDF ``<light>``; only directional lights are used by the generator."""

    name: str
    direction: tuple[float, float, float]
    intensity: float
    diffuse: tuple[float, float, float] = SUN_COLOR
    specular: tuple[float, float, float] = SUN_COLOR
    cast_shadows: bool = False
    kind: str = "directional"
    # The position only places the light's icon in the GUI; a directional light has no
    # position. The orientation must stay 0: <direction> is expressed in the light frame.
    pose: tuple[float, float, float, float, float, float] = (0.0, 0.0, 100.0, 0.0, 0.0, 0.0)
    elevation_deg: float | None = None
    azimuth_deg: float | None = None


@dataclass
class LightingSetup:
    mode: str
    sun_event: str
    ambient: tuple[float, float, float, float]
    background: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    shadows: bool = True
    lights: list[LightSpec] = field(default_factory=list)


def to_source_vector(elevation_deg: float, azimuth_deg: float) -> np.ndarray:
    """Unit ENU vector pointing from the ground to a source at (elevation, azimuth)."""
    el = math.radians(elevation_deg)
    az = math.radians(azimuth_deg)
    return np.array([math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el)])


def direction_from_el_az(elevation_deg: float, azimuth_deg: float) -> np.ndarray:
    """Gazebo ``<direction>`` (propagation, unit) of a light at (elevation, azimuth)."""
    return -to_source_vector(elevation_deg, azimuth_deg)


def el_az_from_direction(direction: Sequence[float]) -> tuple[float, float]:
    """Inverse of :func:`direction_from_el_az`: (elevation_deg, azimuth_deg in [0, 360))."""
    d = np.asarray(direction, dtype=np.float64)
    s = -d / np.linalg.norm(d)
    el = math.degrees(math.asin(max(-1.0, min(1.0, s[2]))))
    az = math.degrees(math.atan2(s[0], s[1])) % 360.0
    return el, az


def angle_between_deg(a: Sequence[float], b: Sequence[float]) -> float:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def check_direction(direction: Sequence[float], elevation_deg: float, azimuth_deg: float,
                    what: str = "light", tol_deg: float = DIRECTION_TOLERANCE_DEG) -> np.ndarray:
    """Validate a configured light direction and return it normalised.

    Raises ValueError if the vector is degenerate, does not point downward (z < 0), or
    deviates from the direction implied by elevation/azimuth by more than ``tol_deg``.
    """
    d = np.asarray(direction, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(d))
    if not np.isfinite(n) or n < 1e-9:
        raise ValueError(f"{what}: degenerate direction {direction!r}")
    d = d / n
    if d[2] >= 0.0:
        raise ValueError(f"{what}: direction {direction!r} does not point downward (z must be < 0)")
    expected = direction_from_el_az(elevation_deg, azimuth_deg)
    ang = angle_between_deg(d, expected)
    if ang > tol_deg:
        raise ValueError(
            f"{what}: configured direction {tuple(np.round(d, 4))} differs by {ang:.2f} deg from the "
            f"direction implied by elevation {elevation_deg} / azimuth {azimuth_deg} "
            f"({tuple(np.round(expected, 4))})")
    return d


def _sun_event(cfg: Config, event: str) -> tuple[float, float, np.ndarray]:
    events = cfg.lighting.sun_events
    if event not in events:
        raise KeyError(f"unknown sun event '{event}' (available: {sorted(events)})")
    el, az, vec = events[event]
    d = check_direction(vec, float(el), float(az), what=f"sun event '{event}'")
    return float(el), float(az), d


def sun_light(cfg: Config, event: str | None = None) -> LightSpec:
    """The sun for a ``lighting.sun_events`` key (default ``lighting.sun_event``)."""
    event = event or cfg.lighting.sun_event
    el, az, d = _sun_event(cfg, event)
    return LightSpec(name="sun", direction=tuple(float(v) for v in d),
                     intensity=float(cfg.lighting.get("sun_intensity", 1.0)),
                     diffuse=SUN_COLOR, specular=SUN_COLOR, cast_shadows=True,
                     elevation_deg=el, azimuth_deg=az)


def earth_light(cfg: Config, intensity: float) -> LightSpec:
    """Earthshine: faint directional light from the Earth's direction, no shadows."""
    el = float(cfg.lighting.earth.elevation_deg)
    az = float(cfg.lighting.earth.azimuth_deg)
    d = direction_from_el_az(el, az)
    if d[2] >= 0:
        raise ValueError(f"earth below the horizon (elevation {el}): no earthshine light possible")
    return LightSpec(name="earthshine", direction=tuple(float(v) for v in d), intensity=float(intensity),
                     diffuse=EARTHSHINE_COLOR, specular=(0.0, 0.0, 0.0), cast_shadows=False,
                     pose=(0.0, 0.0, 90.0, 0.0, 0.0, 0.0), elevation_deg=el, azimuth_deg=az)


def fill_light(intensity: float) -> LightSpec:
    """Overhead fill for inspection: straight down, no shadows, no specular."""
    return LightSpec(name="overhead_fill", direction=(0.0, 0.0, -1.0), intensity=float(intensity),
                     diffuse=FILL_COLOR, specular=(0.0, 0.0, 0.0), cast_shadows=False,
                     pose=(0.0, 0.0, 80.0, 0.0, 0.0, 0.0), elevation_deg=90.0, azimuth_deg=0.0)


def lighting_setup(cfg: Config, mode: str = "realistic") -> LightingSetup:
    """Scene ambient/background/shadows and the light list for a lighting mode."""
    if mode not in MODES:
        raise ValueError(f"unknown lighting mode '{mode}' (expected one of {MODES})")
    lc = cfg.lighting
    realistic = lc.realistic
    if mode == "realistic":
        event = lc.sun_event
        amb = float(realistic.ambient)
        lights = [sun_light(cfg, event), earth_light(cfg, float(realistic.earthshine_intensity))]
    else:
        insp = lc.inspection
        event = insp.get("sun_event", lc.sun_event)
        amb = float(insp.ambient)
        lights = [sun_light(cfg, event), earth_light(cfg, float(realistic.earthshine_intensity)),
                  fill_light(float(insp.fill_intensity))]
    return LightingSetup(mode=mode, sun_event=event, ambient=(amb, amb, amb, 1.0),
                         background=(0.0, 0.0, 0.0, 1.0), shadows=True, lights=lights)
