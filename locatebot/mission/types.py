"""Shared contracts for skills, the LLM agent and navigation (integrator-owned).

Lanes (``nav``, ``agent``, ``pi``, ``link.transport``) import from here and from
``locatebot.lidar.types`` / ``locatebot.safety.supervisor``, never from each other, so their
branches merge without conflicts. Change this file only through the integrator.

Conventions: robot/map frame +x forward, +y left, angles CCW in radians, metres, monotonic
seconds. Throttle and steering are normalised to [-1, 1]; positive steering turns left and
``steering = 1`` means ``VehicleConfig.max_steer_rad`` at the front wheels.
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field
from typing import Any, Mapping

from locatebot.lidar.types import Pose2D


class SkillName(str, enum.Enum):
    """The only actions the LLM (or a fallback routine) may request."""

    EXPLORE = "explore"          # args: {}  drive to frontiers until the map stops growing
    SEARCH = "search"            # args: {"items": (str, ...)}  patrol while vision looks for items
    GO_TO = "go_to"              # args: {"item": str} or {"x": float, "y": float}  (map frame)
    FOLLOW = "follow"            # args: {"item": str}  keep a set distance behind a tracked item
    LOOK_AROUND = "look_around"  # args: {}  slow turn so the camera sweeps the surroundings
    STOP = "stop"                # args: {}  stop and hold
    RETURN_HOME = "return_home"  # args: {}  drive back to the pose where the session started


# Argument schema per skill: name -> (python type, required). Used to validate LLM tool calls
# and to generate tool definitions. GO_TO needs either "item" or both "x" and "y".
# SEARCH items are a tuple in Python (immutable; the LLM sends a JSON array, the tool schema
# says "array" and ``agent.tools.parse_tool_call`` converts it).
SKILL_ARGS: dict[SkillName, dict[str, tuple[type, bool]]] = {
    SkillName.EXPLORE: {},
    SkillName.SEARCH: {"items": (tuple, True)},
    SkillName.GO_TO: {"item": (str, False), "x": (float, False), "y": (float, False)},
    SkillName.FOLLOW: {"item": (str, True)},
    SkillName.LOOK_AROUND: {},
    SkillName.STOP: {},
    SkillName.RETURN_HOME: {},
}


@dataclass(frozen=True)
class SkillCall:
    name: SkillName
    args: Mapping[str, Any] = field(default_factory=dict)
    reason: str = ""             # short free text from the LLM, for the dashboard/log
    source: str = "llm"          # "llm" | "fallback" | "operator"


class SkillState(str, enum.Enum):
    IDLE = "idle"
    PENDING_APPROVAL = "pending_approval"  # approve mode: waiting for the operator
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class SkillStatus:
    state: SkillState = SkillState.IDLE
    call: SkillCall | None = None
    detail: str = ""             # e.g. "no path to marker", "blocked by obstacle", "found 2/3 items"


@dataclass(frozen=True)
class ItemView:
    """One item as the agent sees it, built from the localize ItemTracker."""

    name: str
    confirmed: bool
    sightings: int
    x: float                     # map frame, metres
    y: float
    sigma_m: float
    range_m: float               # from the robot's current pose
    bearing_rad: float           # robot frame, CCW positive (left)
    last_seen_age_s: float


@dataclass(frozen=True)
class PersonView:
    """One tracked person as the agent sees it (from the LiDAR people tracker, ROADMAP M3).

    A LiDAR track is a person only by shape and motion until the camera confirms it, so
    ``confirmed_by_camera`` tells the LLM how much to trust it.
    """

    x: float                     # map frame, metres
    y: float
    range_m: float               # from the robot's current pose
    bearing_rad: float           # robot frame, CCW positive (left)
    speed_mps: float             # estimated ground speed, >= 0
    moving: bool                 # speed above the tracker's moving threshold
    confirmed_by_camera: bool    # a vision "person" fix matched this track
    last_seen_age_s: float       # time since the LiDAR last associated a detection


@dataclass(frozen=True)
class ChangeView:
    """Something new or moved compared with the saved room map (ROADMAP M2 change detection).

    Not classified: it may be a bag, a moved chair or a person (people also appear in
    ``WorldState.people`` once tracked).
    """

    x: float                     # map frame, metres (blob centroid)
    y: float
    width_m: float               # blob extent across the LiDAR's view
    range_m: float               # from the robot's current pose
    bearing_rad: float           # robot frame, CCW positive (left)
    moving: bool                 # the blob moved since it was first seen
    age_s: float                 # time since it was first seen


@dataclass(frozen=True)
class WorldState:
    """Snapshot handed to the LLM agent. Built on the Orin from pose, scan, map, items, skill."""

    stamp_s: float
    pose: Pose2D
    # Minimum LiDAR range per sector, ``detect.sector_clearance`` layout: sector 0 centred on
    # forward, then CCW. NaN = no return (unknown, never free).
    sector_clearance_m: tuple[float, ...]
    explored_frac: float | None  # known cells / reachable area estimate, None before a map exists
    items: tuple[ItemView, ...]
    skill: SkillStatus
    search_items: tuple[str, ...] = ()
    safety_reason: str = "ok"    # last Pi safety Reason value
    mission_state: str = "idle"
    last_event: str = ""         # newest notable event, e.g. "found marker", "blocked"
    goal: str = ""               # the operator's / Telegram request in words, "" = no goal
    people: tuple[PersonView, ...] = ()   # nearest first
    changes: tuple[ChangeView, ...] = ()  # new/moved things vs the saved room map, nearest first
    # Short extra facts for the LLM, e.g. "glass or low obstacle 0.6 m ahead (ultrasonic only)",
    # "wifi weak here". Producers keep each note short and the tuple small.
    notes: tuple[str, ...] = ()


@dataclass(frozen=True)
class VehicleConfig:
    """Car geometry and limits. Placeholders until measured on the car."""

    wheelbase_m: float = 0.26
    max_steer_rad: float = math.radians(30)  # must equal SafetyConfig.max_steer_rad
    half_width_m: float = 0.15               # half the car width plus margin
    max_speed_mps: float = 0.4               # speed at throttle == SafetyConfig.max_speed
    max_throttle: float = 0.5                # never command more than this

    def __post_init__(self) -> None:
        if self.wheelbase_m <= 0 or self.half_width_m <= 0 or self.max_speed_mps <= 0:
            raise ValueError("wheelbase_m, half_width_m and max_speed_mps must be > 0")
        if not 0.0 < self.max_steer_rad < math.pi / 2:
            raise ValueError("max_steer_rad must be in (0, pi/2)")
        if not 0.0 < self.max_throttle <= 1.0:
            raise ValueError("max_throttle must be in (0, 1]")

    @property
    def min_turn_radius_m(self) -> float:
        return self.wheelbase_m / math.tan(self.max_steer_rad)
