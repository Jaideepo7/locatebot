"""Live session mapping on the Orin (build step 2: map the room while driving manually).

``LiveMapper`` wraps ``lidar.mapping.Mapper`` for a running robot: one thread feeds scans as
they arrive from the Pi; other threads (vision localization, the skill executor, the agent,
the dashboard) read the pose, a copy of the grid, and pose/scan history at a frame's capture
time. All access goes through one lock. The map frame is the robot pose at the first accepted
scan, so "home" is (0, 0, 0).

``LiveMapper.from_saved(path, guess)`` (M1) instead loads a saved ``.lbmap`` and localizes in it
(``lidar.localize.LocalizingMapper``): the map frame is the saved map's, ``home`` its stored home
pose, and the map is updated slowly. ``stats()`` reports ``mode`` (``mapping``/``localizing``),
``lost``, ``ambiguous`` and ``last_score``.

In that mode the map is layered by default (M2, ``lidar.layers.LayeredMap``): new/moving things
go to a short-lived dynamic layer, ``change_blobs()`` lists them, and ``grid_snapshot()`` returns
the planning grid (room + dynamic + blocking extra layers) so skills plan around them.

M3 on top of the layers: a ``PeopleTracker`` fed with each scan's NEW-point clusters
(``people()``, camera confirmation hooks), ``on_range`` (ultrasonic -> ``glass_or_low`` layer,
created on the first reading) and ``add_wifi`` (RSSI -> ``wifi_rssi`` layer, created on the first
reading), and ``notes()`` for the LLM. Without layers (new-map mode) all of these are inert.
"""
from __future__ import annotations

import copy
import threading
from pathlib import Path

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.layers import ChangeBlob, LayeredMap
from locatebot.lidar.people import PeopleConfig, PeopleTracker, PersonTrack
from locatebot.lidar.ultrasonic import UltrasonicConfig, UltrasonicLayer
from locatebot.lidar.localize import LocalizerConfig, LocalizeResult, LocalizingMapper
from locatebot.lidar.mapping import Mapper
from locatebot.lidar.odometry import OdometryResult
from locatebot.lidar.types import Pose2D, Scan
from locatebot.localize.items import PoseHistory, ScanHistory


class LiveMapper:
    def __init__(self, size_m: float = 20.0, resolution_m: float = 0.05,
                 pose_history_s: float = 30.0, scan_match_dt_s: float = 0.06,
                 engine: Mapper | LocalizingMapper | None = None, people_config: PeopleConfig | None = None,
                 ultrasonic_config: UltrasonicConfig | None = None) -> None:
        self._lock = threading.Lock()
        self._mapper = engine if engine is not None else Mapper(size_m=size_m, resolution_m=resolution_m)
        self._poses = PoseHistory(max_age_s=pose_history_s)
        self._scans = ScanHistory(max_dt_s=scan_match_dt_s)
        self._latest: Scan | None = None
        self._last_result: OdometryResult | LocalizeResult | None = None
        self.home = getattr(self._mapper, "home", Pose2D(0.0, 0.0, 0.0))
        self.people_tracker = PeopleTracker(people_config)
        self.ultrasonic_config = ultrasonic_config
        self._ultrasonic: UltrasonicLayer | None = None
        self._wifi = None  # locatebot.orin.wifi.WifiLayer, created on the first reading

    @classmethod
    def from_saved(cls, path, guess: Pose2D | None = None, config: LocalizerConfig | None = None,
                   slow_rate: float = 0.2, layered: bool = True, **kw) -> "LiveMapper":
        """Localize in a saved ``.lbmap`` instead of starting a new map. ``guess`` is where the
        car is in that map (dock pose, operator click); None = global search over the map.
        ``home`` = the map's stored home pose (where it was first mapped), else the guess.
        ``layered``: static/dynamic layers and change detection (M2)."""
        engine = LocalizingMapper.from_file(path, guess=guess, config=config, slow_rate=slow_rate,
                                            layered=layered)
        return cls(engine=engine, **kw)

    @property
    def layers(self) -> LayeredMap | None:
        """The layered map (localizing + layered mode) or None. Mutate only under ``locked()``."""
        return getattr(self._mapper, "layers", None)

    def locked(self) -> threading.Lock:
        """The mapper lock, for modules that update an extra layer (``with lm.locked(): ...``)."""
        return self._lock

    def change_blobs(self, min_persistence: int = 3) -> list[ChangeBlob]:
        """Current new/moving things vs the saved room map, seen in >= ``min_persistence`` scans
        (hides one- and two-scan flickers seen on the real C1 replays). Empty when not layered."""
        with self._lock:
            lay = getattr(self._mapper, "layers", None)
            return lay.blobs(min_persistence=min_persistence) if lay is not None else []

    # ------------------------------------------------------------------ M3: people, ultrasonic, wifi
    def people(self, people_only: bool = True) -> list[PersonTrack]:
        """Tracked people (``people_only=False``: every confirmed track, objects included)."""
        with self._lock:
            return self.people_tracker.tracks(people_only=people_only)

    def people_wants_camera(self, now: float) -> PersonTrack | None:
        with self._lock:
            return self.people_tracker.wants_camera_check(now)

    def people_camera_requested(self, track_id: int, now: float) -> None:
        with self._lock:
            self.people_tracker.camera_requested(track_id, now)

    def people_camera_fix(self, xy, stamp_s: float) -> PersonTrack | None:
        with self._lock:
            return self.people_tracker.on_camera_fix(xy, stamp_s)

    def on_range(self, stamp_s: float, range_m: float | None, fov_rad: float) -> bool:
        """One ultrasonic reading (Pi stamp). Marks ``glass_or_low`` where only the ultrasonic
        sees something; needs layered mode and a pose/scan near the stamp. True = marked."""
        with self._lock:
            lay = getattr(self._mapper, "layers", None)
            if lay is None:
                return False
            pose = self._poses.at(stamp_s)
            scan = self._scans.nearest(stamp_s)
            if pose is None:
                return False
            if self._ultrasonic is None:
                self._ultrasonic = UltrasonicLayer(lay, self.ultrasonic_config)
            return self._ultrasonic.on_range(range_m, fov_rad, pose, scan, stamp_s)

    def add_wifi(self, dbm: float | None, stamp_s: float) -> bool:
        """One Wi-Fi RSSI reading at the current pose (layered, localized mode only)."""
        if dbm is None:
            return False
        with self._lock:
            lay = getattr(self._mapper, "layers", None)
            r = self._last_result
            if lay is None or r is None or not getattr(r, "accepted", False):
                return False
            if self._wifi is None:
                from .wifi import WifiLayer
                self._wifi = WifiLayer(lay)
            return self._wifi.add(dbm, self._pose_locked(), stamp_s)

    def layer_snapshot(self, name: str) -> "np.ndarray | None":
        """Copy of an extra layer's values (e.g. ``wifi_rssi`` for the page overlay), or None."""
        with self._lock:
            lay = getattr(self._mapper, "layers", None)
            if lay is None or name not in lay.extra:
                return None
            return lay.extra[name].values()

    def notes(self, now: float) -> list[str]:
        """Short facts for the LLM (glass/low obstacle, weak Wi-Fi), newest state only."""
        with self._lock:
            pose = self._pose_locked()
            out = []
            for src in (self._ultrasonic, self._wifi):
                n = src.note(pose, now) if src is not None else None
                if n:
                    out.append(n)
            return out

    def _pose_locked(self) -> Pose2D:
        traj = self._mapper.trajectory
        if traj:
            _, x, y, th = traj[-1]
            return Pose2D(x, y, th)
        return self.home

    @property
    def localizing(self) -> bool:
        return isinstance(self._mapper, LocalizingMapper)

    def relocalize(self, guess: Pose2D | None = None) -> None:
        """Localizing mode: drop the current track and re-initialize around ``guess`` (None =
        global search) on the next scan. No-op when building a new map."""
        with self._lock:
            if isinstance(self._mapper, LocalizingMapper):
                self._mapper.localizer.request_init(guess)

    def add_scan(self, scan: Scan) -> OdometryResult | LocalizeResult:
        """Integrate one cleaned-or-raw scan (Pi clock stamps, in time order)."""
        with self._lock:
            result = self._mapper.add_scan(scan)
            self._last_result = result
            self._latest = scan
            self._scans.add(scan)
            if result.accepted:
                try:
                    self._poses.add(scan.stamp_s, result.pose)
                except ValueError:
                    pass  # out-of-order stamp: keep the map, skip the history entry
            lay = getattr(self._mapper, "layers", None)
            if lay is not None:  # people from this scan's NEW clusters (none without a good pose)
                self.people_tracker.update(scan.stamp_s, lay.last_clusters if result.accepted else [])
            return result

    @property
    def pose(self) -> Pose2D:
        with self._lock:
            traj = self._mapper.trajectory
            if traj:
                _, x, y, th = traj[-1]
                return Pose2D(x, y, th)
            return self.home

    @property
    def latest_scan(self) -> Scan | None:
        with self._lock:
            return self._latest

    def pose_at(self, stamp_s: float) -> Pose2D | None:
        with self._lock:
            return self._poses.at(stamp_s)

    def scan_near(self, stamp_s: float) -> Scan | None:
        with self._lock:
            return self._scans.nearest(stamp_s)

    def grid_snapshot(self) -> OccupancyGrid:
        """Independent copy of the grid, safe to plan on while mapping continues. In layered
        mode: the planning grid (room + dynamic things + blocking extra layers)."""
        with self._lock:
            lay = getattr(self._mapper, "layers", None)
            if lay is not None:
                return lay.planning_grid()
            g = copy.copy(self._mapper.grid)
            g.log_odds = self._mapper.grid.log_odds.copy()
            return g

    def stats(self) -> dict:
        with self._lock:
            m = self._mapper
            r = self._last_result
            out = {"scans": m.n_scans, "integrated": m.n_integrated, "rejected": m.n_rejected,
                   "mode": "localizing" if isinstance(m, LocalizingMapper) else "mapping",
                   "lost": bool(getattr(r, "lost", False)), "ambiguous": bool(getattr(r, "ambiguous", False)),
                   "last_score": None if r is None else round(float(r.score), 3)}
            if isinstance(m, LocalizingMapper):
                loc = m.localizer
                out.update({"initialized": loc.initialized, "lost": loc.lost,
                            "ambiguous": loc.ambiguous and not loc.initialized,
                            "source": None if r is None else r.source,
                            "lost_events": loc.n_lost_events, "recoveries": loc.n_recoveries,
                            "layered": m.layers is not None,
                            "changes": len(m.layers.blobs(min_persistence=3)) if m.layers is not None else 0})
            return out

    def save(self, stem: str | Path, meta: dict | None = None) -> dict[str, Path]:
        with self._lock:
            return self._mapper.save(stem, meta=meta)
