#!/usr/bin/env python3
"""
Boustrophedon Coverage Path Planner 
=================================================
"""

import json
import math
import os
import pickle
import subprocess
import time
from collections import deque
from enum import Enum, auto
from pathlib import Path

import rclpy
import rclpy.qos
from geometry_msgs.msg import Point, TwistStamped
from geometry_msgs.msg import Point as GPoint
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker

# ── Tunable constants ────────────────────────────────────────────────────────
CRITICAL_DIST        = 0.40   # m – emergency stop / reverse
SLOW_DIST            = 0.70   # m – slow down in transit
SWEEP_SPEED          = 0.20   # m/s forward during lawnmower rows
TURN_SPEED           = 0.45   # rad/s for wall-follow turns  (↓ from 0.55)
TURN_SPEED_ESC       = 0.35   # rad/s during escape spins    (gentler → less map noise)
BACKUP_SPEED         = -0.18  # m/s
BACKUP_DURATION      = 1.4    # s
SPIN_DURATION        = 1.2    # s per escape spin attempt    (↓ from 1.6)
MAX_ESCAPE_RETRIES   = 3      # how many back+spin cycles before giving up

ROW_SPACING          = 0.40   # m between lawnmower rows
PLAN_CELL_SIZE       = 0.10   # m – CPP planning grid resolution
GOAL_TOL             = 0.35   # m – "close enough" to a waypoint
HEADING_TOL          = 0.18   # rad
HEADING_KP           = 1.20
MAX_TURN             = 0.60   # rad/s cap

STALL_CHECK_PERIOD   = 3.0    # s
STALL_DIST_THRESH    = 0.04   # m

OVERRIDE_TIMEOUT     = 2.0    # s

DIRT_FRESHNESS       = 0.8
DIRT_CONF_MIN        = 0.35
DIRT_AREA_STOP       = 0.08
TRASH_PICKUP_DIST    = 0.40
DIRT_CLEAN_DIST      = 1.50
SEM_MIN_AREA         = 400
WATER_MIN_AREA       = 500
CARPET_MULT          = 0.50

COVERAGE_THRESHOLD   = 0.95
VISITED_RADIUS       = 0.25   # m
COVERAGE_CHECK_EVERY = 0.5    # s

WALL_FOLLOW_DURATION = 30.0   # s
WALL_FOLLOW_SPEED    = 0.16
WALL_FOLLOW_DIST     = 0.55   # target right-wall distance

HOME_TOL             = 0.50   # m

# ── Persistent map memory ─────────────────────────────────────────────────────
MAP_MEMORY_DIR    = Path.home() / ".ros" / "coverage_maps"
MAP_MEMORY_DIR.mkdir(parents=True, exist_ok=True)
MAP_MEMORY_FILE   = MAP_MEMORY_DIR / "house_map.pkl"
MAP_META_FILE     = MAP_MEMORY_DIR / "house_map_meta.json"
MAP_SAVE_INTERVAL = 30.0
MAP_MATCH_THRESHOLD = 0.70

DIRT_ZONES = {
    "dirt_zone":   ( 0.3,  0.9),
    "dirt_zone_2": (-0.4,  0.8),
    "dirt_zone_3": ( 0.0, -0.9),
}
TRASH_ITEMS = {
    "trash_1":  (-3.5, -1.5), "trash_2":  (-3.8, -1.2), "trash_3":  (-3.2, -1.8),
    "trash_4":  (-3.5,  3.8), "trash_5":  (-3.8,  3.5),
    "trash_6":  ( 4.5, -3.5), "trash_7":  ( 4.7, -3.8),
    "trash_8":  ( 4.5,  4.2), "trash_9":  ( 4.2,  3.9),
    "trash_10": ( 0.5,  1.0), "trash_11": ( 0.8,  0.7),
}


class Phase(Enum):
    WALL_FOLLOW   = auto()
    PLAN          = auto()
    GOTO_START    = auto()
    SWEEP         = auto()
    ESCAPE_BACK   = auto()
    ESCAPE_SPIN   = auto()
    RETURN_HOME   = auto()
    DONE          = auto()


class CoverageExplorer(Node):

    def __init__(self):
        super().__init__("coverage_explorer")

        self.cmd_pub    = self.create_publisher(TwistStamped, "/cmd_vel",           10)
        self.viz_pub    = self.create_publisher(Marker,       "/coverage_path_viz", 10)
        self.status_pub = self.create_publisher(String,       "/coverage_status",   10)

        map_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(LaserScan,     "/scan",           self._scan_cb,      10)
        self.create_subscription(OccupancyGrid, "/map",            self._map_cb,       map_qos)
        self.create_subscription(Odometry,      "/odom",           self._odom_cb,      10)
        self.create_subscription(Point,         "/dirt_target",    self._dirt_cb,      10)
        self.create_subscription(String,        "/semantic_class", self._sem_class_cb, 10)
        self.create_subscription(Float32,       "/semantic_area",  self._sem_area_cb,  10)
        self.create_subscription(TwistStamped,  "/cmd_vel",        self._manual_cb,    10)

        self.create_timer(0.1,                  self._control_loop)
        self.create_timer(COVERAGE_CHECK_EVERY, self._coverage_check)

        # Sensor state
        self.ranges    : list[float] = []
        self.map       : OccupancyGrid | None = None
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.sem_class = "clear"
        self.sem_area  = 0.0

        # Home
        self.home_x : float | None = None
        self.home_y : float | None = None

        # Dirt / trash
        self.dirt_offset     = None
        self.dirt_confidence = 0.0
        self.dirt_area_val   = 0.0
        self.last_dirt_time  = None
        self.dirt_items      = dict(DIRT_ZONES)
        self.trash_items     = dict(TRASH_ITEMS)

        # Coverage
        self.visited_cells : set = set()
        self.coverage_pct  = 0.0

        # CPP state
        self.phase           : Phase = Phase.WALL_FOLLOW
        self.phase_start     : float = time.time()
        self.waypoints       : list[tuple[float,float]] = []
        self.wp_index        : int   = 0
        self.escape_phase_return : Phase = Phase.SWEEP

        # ── Escape state ──────────────────────────────────────────────────────
        self._spin_dir       : float = 1.0
        self._escape_attempts: int   = 0   # retries within one blockage event

        # Stall detection
        self.stall_check_t   = time.time()
        self.stall_ref_x     = 0.0
        self.stall_ref_y     = 0.0

        # Manual override
        self.last_manual_time = 0.0

        # Map memory
        self._last_map_save_t  : float = 0.0
        self._saved_map_loaded : bool  = False

        self.get_logger().info("Coverage Explorer ready – starting wall-follow phase.")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.ranges = [
            r if (msg.range_min < r < msg.range_max) else float('inf')
            for r in msg.ranges
        ]

    def _map_cb(self, msg: OccupancyGrid):
        self.map = msg
        if not self._saved_map_loaded:
            self._saved_map_loaded = True
            self._load_map()

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.home_x is None:
            self.home_x = self.robot_x
            self.home_y = self.robot_y
            self.stall_ref_x = self.robot_x
            self.stall_ref_y = self.robot_y
            self.get_logger().info(f"Home set at ({self.home_x:.2f}, {self.home_y:.2f})")

        # Only mark visited when actually moving forward (not during escapes or turns)
        if self.phase == Phase.SWEEP:
            self._mark_visited()

    def _dirt_cb(self, msg: Point):
        self.dirt_offset     = msg.x
        self.dirt_confidence = msg.y
        self.dirt_area_val   = msg.z
        self.last_dirt_time  = self.get_clock().now()

    def _sem_class_cb(self, msg: String): self.sem_class = msg.data
    def _sem_area_cb(self, msg: Float32): self.sem_area  = msg.data

    def _manual_cb(self, msg: TwistStamped):
        lx = msg.twist.linear.x
        az = msg.twist.angular.z
        auto_speeds = {SWEEP_SPEED, WALL_FOLLOW_SPEED, BACKUP_SPEED, 0.0}
        if (lx != 0.0 or az != 0.0) and lx not in auto_speeds:
            self.last_manual_time = time.time()

    # ── Persistent map memory ─────────────────────────────────────────────────

    def _save_map(self, reason: str = "autosave") -> bool:
        if self.map is None:
            self.get_logger().warn("Map memory: no map to save yet.")
            return False

        info = self.map.info
        payload = {
            "map_info": {
                "resolution": info.resolution,
                "width":      info.width,
                "height":     info.height,
                "origin_x":   info.origin.position.x,
                "origin_y":   info.origin.position.y,
            },
            "map_data":  bytearray(self.map.data),
            "visited":   list(self.visited_cells),
            "home":      [self.home_x, self.home_y],
            "saved_at":  time.strftime("%Y-%m-%dT%H:%M:%S"),
        }

        try:
            tmp = MAP_MEMORY_FILE.with_suffix(".tmp")
            with open(tmp, "wb") as f:
                pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)
            tmp.replace(MAP_MEMORY_FILE)

            meta = {k: v for k, v in payload.items() if k != "map_data"}
            meta["visited_count"] = len(self.visited_cells)
            meta["map_data"]      = f"<{len(payload['map_data'])} bytes>"
            with open(MAP_META_FILE, "w") as f:
                json.dump(meta, f, indent=2, default=str)

            self.get_logger().info(
                f"Map memory saved ({reason}): "
                f"{len(self.visited_cells)} visited cells → {MAP_MEMORY_FILE}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Map memory save failed: {e}")
            return False

    def _load_map(self) -> bool:
        if not MAP_MEMORY_FILE.exists():
            self.get_logger().info("Map memory: no saved map found on disk.")
            return False
        if self.map is None:
            self.get_logger().warn("Map memory: live map not ready yet, skipping load.")
            return False

        try:
            with open(MAP_MEMORY_FILE, "rb") as f:
                payload = pickle.load(f)
        except Exception as e:
            self.get_logger().error(f"Map memory load failed (corrupt file?): {e}")
            return False

        saved_info = payload["map_info"]
        live_info  = self.map.info

        if abs(saved_info["resolution"] - live_info.resolution) > 0.005:
            self.get_logger().warn("Map memory: resolution mismatch. Discarding.")
            return False

        s_res = saved_info["resolution"]
        s_ox  = saved_info["origin_x"]
        s_oy  = saved_info["origin_y"]
        s_w   = saved_info["width"]
        s_h   = saved_info["height"]
        s_data = payload["map_data"]

        l_res = live_info.resolution
        l_ox  = live_info.origin.position.x
        l_oy  = live_info.origin.position.y
        l_w   = live_info.width
        l_h   = live_info.height
        l_data = self.map.data

        saved_free = overlap_free = 0
        for cy in range(s_h):
            for cx in range(s_w):
                if s_data[cy * s_w + cx] != 0:
                    continue
                saved_free += 1
                wx = cx * s_res + s_ox + s_res * 0.5
                wy = cy * s_res + s_oy + s_res * 0.5
                lx = int((wx - l_ox) / l_res)
                ly = int((wy - l_oy) / l_res)
                if 0 <= lx < l_w and 0 <= ly < l_h and l_data[ly * l_w + lx] == 0:
                    overlap_free += 1

        if saved_free == 0:
            self.get_logger().warn("Map memory: saved map has no free cells.")
            return False

        overlap_ratio = overlap_free / saved_free
        self.get_logger().info(
            f"Map memory: overlap = {overlap_ratio*100:.1f}% "
            f"(threshold {MAP_MATCH_THRESHOLD*100:.0f}%)"
        )

        if overlap_ratio < MAP_MATCH_THRESHOLD:
            self.get_logger().warn("Map memory: different environment. Starting fresh.")
            return False

        restored = set()
        for (cx, cy) in payload["visited"]:
            wx = cx * s_res + s_ox + s_res * 0.5
            wy = cy * s_res + s_oy + s_res * 0.5
            lx = int((wx - l_ox) / l_res)
            ly = int((wy - l_oy) / l_res)
            if 0 <= lx < l_w and 0 <= ly < l_h:
                restored.add((lx, ly))
        self.visited_cells |= restored

        if self.home_x is None and payload.get("home") and payload["home"][0] is not None:
            self.home_x, self.home_y = payload["home"]
            self.get_logger().info(
                f"Map memory: home restored to ({self.home_x:.2f}, {self.home_y:.2f})"
            )

        self.get_logger().info(
            f"Map memory loaded ✔  saved={payload.get('saved_at','?')}  "
            f"restored {len(restored)} visited cells."
        )
        return True

    def _autosave_map(self):
        now = time.time()
        if (now - self._last_map_save_t) >= MAP_SAVE_INTERVAL:
            self._last_map_save_t = now
            self._save_map("autosave")

    # ── Map helpers ───────────────────────────────────────────────────────────

    def _mark_visited(self):
        if self.map is None:
            return
        res = self.map.info.resolution
        ox  = self.map.info.origin.position.x
        oy  = self.map.info.origin.position.y
        w   = self.map.info.width
        h   = self.map.info.height
        r   = max(1, int(VISITED_RADIUS / res))
        cx  = int((self.robot_x - ox) / res)
        cy  = int((self.robot_y - oy) / res)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx*dx + dy*dy <= r*r:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        self.visited_cells.add((nx, ny))

    def _free_cells(self) -> set[tuple[int,int]]:
        if self.map is None:
            return set()
        w, h = self.map.info.width, self.map.info.height
        data = self.map.data
        return {
            (cx, cy)
            for cy in range(h) for cx in range(w)
            if data[cy * w + cx] == 0
        }

    def _robot_cell(self) -> tuple[int, int]:
        if self.map is None:
            return 0, 0
        res = self.map.info.resolution
        ox  = self.map.info.origin.position.x
        oy  = self.map.info.origin.position.y
        return (int((self.robot_x - ox) / res),
                int((self.robot_y - oy) / res))

    # ── Boustrophedon CPP planner ─────────────────────────────────────────────

    def _build_boustrophedon_waypoints(self) -> list[tuple[float, float]]:
        if self.map is None:
            return []

        res = self.map.info.resolution
        ox  = self.map.info.origin.position.x
        oy  = self.map.info.origin.position.y
        w   = self.map.info.width
        h   = self.map.info.height
        data = self.map.data

        stride = max(1, int(ROW_SPACING / res))

        INF_R = 3
        occupied = set()
        for cy in range(h):
            for cx in range(w):
                if data[cy * w + cx] not in (0, -1) and data[cy * w + cx] > 50:
                    for dy in range(-INF_R, INF_R + 1):
                        for dx in range(-INF_R, INF_R + 1):
                            occupied.add((cx + dx, cy + dy))

        def is_free(cx, cy):
            return (0 <= cx < w and 0 <= cy < h
                    and data[cy * w + cx] == 0
                    and (cx, cy) not in occupied)

        waypoints = []
        left_to_right = True

        for row_cy in range(0, h, stride):
            row_cells = [cx for cx in range(w) if is_free(cx, row_cy)]
            if not row_cells:
                continue
            if not left_to_right:
                row_cells = list(reversed(row_cells))

            segments = []
            seg_start = row_cells[0]
            prev = row_cells[0]
            for cx in row_cells[1:]:
                if cx != prev + 1 and cx != prev - 1:
                    segments.append((seg_start, prev))
                    seg_start = cx
                prev = cx
            segments.append((seg_start, prev))

            for s, e in segments:
                wx_s = s * res + ox + res * 0.5
                wx_e = e * res + ox + res * 0.5
                wy   = row_cy * res + oy + res * 0.5
                if not left_to_right:
                    wx_s, wx_e = wx_e, wx_s
                waypoints.append((wx_s, wy))
                waypoints.append((wx_e, wy))

            left_to_right = not left_to_right

        self.get_logger().info(f"CPP planned {len(waypoints)} waypoints across {h // stride} rows.")
        return waypoints

    # ── Sector sensor helpers ─────────────────────────────────────────────────

    def _sector_min(self, d0: int, d1: int) -> float:
        if not self.ranges:
            return float("inf")
        n    = len(self.ranges)
        vals = []
        for d in range(d0, d1):
            r = self.ranges[int(d * n / 360) % n]
            if math.isfinite(r) and r > 0.0:
                vals.append(r)
        return min(vals) if vals else float("inf")

    def _front(self) -> float:
        return min(self._sector_min(345, 360), self._sector_min(0, 15))

    def _right(self) -> float:
        return self._sector_min(260, 290)

    def _left(self) -> float:
        return self._sector_min(70, 100)

    # ── Drive helpers ─────────────────────────────────────────────────────────

    def _pub(self, lx: float = 0.0, az: float = 0.0):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x  = lx
        cmd.twist.angular.z = az
        self.cmd_pub.publish(cmd)

    def _steer_to(self, tx: float, ty: float, speed: float) -> bool:
        dx   = tx - self.robot_x
        dy   = ty - self.robot_y
        dist = math.hypot(dx, dy)
        if dist < GOAL_TOL:
            return True

        desired_yaw = math.atan2(dy, dx)
        err_yaw     = math.atan2(math.sin(desired_yaw - self.robot_yaw),
                                 math.cos(desired_yaw - self.robot_yaw))

        if abs(err_yaw) > HEADING_TOL:
            az = max(-MAX_TURN, min(MAX_TURN, HEADING_KP * err_yaw))
            self._pub(lx=0.0, az=az)
        else:
            az = max(-MAX_TURN, min(MAX_TURN, HEADING_KP * err_yaw))
            self._pub(lx=speed, az=az)
        return False

    # ── Stall detection ───────────────────────────────────────────────────────

    def _check_stall(self) -> bool:
        now = time.time()
        if (now - self.stall_check_t) < STALL_CHECK_PERIOD:
            return False
        moved = math.hypot(self.robot_x - self.stall_ref_x,
                           self.robot_y - self.stall_ref_y)
        self.stall_check_t = now
        self.stall_ref_x   = self.robot_x
        self.stall_ref_y   = self.robot_y
        if moved < STALL_DIST_THRESH:
            self.get_logger().warn("Stall detected – escaping.")
            return True
        return False

    def _start_escape(self, return_to: Phase):
        """
        Begin escape sequence.
        - Resets retry counter only on the FIRST entry into an escape event.
        - If called while already escaping (re-trigger), keep the counter.
        """
        if self.phase not in (Phase.ESCAPE_BACK, Phase.ESCAPE_SPIN):
            # Fresh blockage event
            self._escape_attempts = 0
            self._spin_dir = 1.0 if self._left() < self._right() else -1.0

        self.escape_phase_return = return_to
        self.phase               = Phase.ESCAPE_BACK
        self.phase_start         = time.time()

    # ── Coverage tracking ─────────────────────────────────────────────────────

    def _coverage_check(self):
        if self.map is None or self.phase == Phase.DONE:
            return
        if self.phase not in (Phase.SWEEP, Phase.WALL_FOLLOW):
            self._publish_status()
            return

        free = self._free_cells()
        if not free:
            return

        self.coverage_pct = len(free & self.visited_cells) / len(free)
        self.get_logger().info(f"Coverage: {self.coverage_pct*100:.1f}%")
        self._publish_visited_viz(free)
        self._publish_status()

    # ── Dirt / trash helpers ──────────────────────────────────────────────────

    def _has_fresh_dirt(self) -> bool:
        if self.last_dirt_time is None or self.dirt_offset is None:
            return False
        age = (self.get_clock().now() - self.last_dirt_time).nanoseconds / 1e9
        return age < DIRT_FRESHNESS and self.dirt_confidence > DIRT_CONF_MIN

    def _vacuum_nearby_trash(self):
        for name, pos in list(self.trash_items.items()):
            if math.hypot(self.robot_x - pos[0], self.robot_y - pos[1]) < TRASH_PICKUP_DIST:
                self.get_logger().info(f"Vacuumed {name}")
                self._delete_gz_model(name)
                del self.trash_items[name]

    def _clean_nearby_dirt(self):
        for name, pos in list(self.dirt_items.items()):
            if math.hypot(self.robot_x - pos[0], self.robot_y - pos[1]) < DIRT_CLEAN_DIST:
                self.get_logger().info(f"Cleaned {name}")
                self._delete_gz_model(name)
                del self.dirt_items[name]

    def _delete_gz_model(self, name: str):
        subprocess.run([
            "gz", "service", "-s", "/world/my_house/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", f'name: "{name}", type: MODEL',
        ], capture_output=True)

    # ── Status / visualisation ────────────────────────────────────────────────

    def _publish_status(self):
        labels = {
            Phase.WALL_FOLLOW: "WALL-FOLLOW (mapping perimeter)",
            Phase.PLAN:        "PLANNING boustrophedon path",
            Phase.GOTO_START:  "GOTO start of CPP path",
            Phase.SWEEP:       f"SWEEP {self.coverage_pct*100:.1f}% wp={self.wp_index}/{len(self.waypoints)}",
            Phase.ESCAPE_BACK: f"ESCAPE – reversing (attempt {self._escape_attempts+1})",
            Phase.ESCAPE_SPIN: f"ESCAPE – turning (attempt {self._escape_attempts+1})",
            Phase.RETURN_HOME: "RETURNING HOME",
            Phase.DONE:        "DONE",
        }
        s = String()
        s.data = labels.get(self.phase, str(self.phase))
        self.status_pub.publish(s)

    def _publish_visited_viz(self, free_cells: set):
        if self.map is None:
            return
        res = self.map.info.resolution
        ox  = self.map.info.origin.position.x
        oy  = self.map.info.origin.position.y
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = "visited"; m.type = Marker.POINTS; m.action = Marker.ADD
        m.scale.x = res * 0.85; m.scale.y = res * 0.85
        m.color.r = 0.1; m.color.g = 0.85; m.color.b = 0.3; m.color.a = 0.55
        for cx, cy in (free_cells & self.visited_cells):
            p = GPoint()
            p.x = cx * res + ox + res * 0.5
            p.y = cy * res + oy + res * 0.5
            p.z = 0.01
            m.points.append(p)
        self.viz_pub.publish(m)

    # ── Main control loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        if (time.time() - self.last_manual_time) < OVERRIDE_TIMEOUT:
            return
        if not self.ranges:
            return

        front = self._front()
        left  = self._left()
        right = self._right()
        now   = time.time()

        # ── Passive tasks (always) ────────────────────────────────────────────
        self._vacuum_nearby_trash()
        self._clean_nearby_dirt()
        self._autosave_map()

        # Water avoidance
        if (self.sem_class == "water" and self.sem_area > WATER_MIN_AREA
                and self.phase not in (Phase.ESCAPE_BACK, Phase.ESCAPE_SPIN, Phase.DONE)):
            self._start_escape(self.phase)
            return

        speed_mult = CARPET_MULT if (self.sem_class == "carpet"
                                     and self.sem_area > SEM_MIN_AREA) else 1.0

        # ── Phase machine ─────────────────────────────────────────────────────

        if self.phase == Phase.DONE:
            self._pub()
            return

        # ── WALL_FOLLOW ───────────────────────────────────────────────────────
        if self.phase == Phase.WALL_FOLLOW:
            elapsed = now - self.phase_start
            if elapsed >= WALL_FOLLOW_DURATION:
                self.get_logger().info("Wall-follow done. Planning CPP path.")
                self.phase       = Phase.PLAN
                self.phase_start = now
                return

            if front < CRITICAL_DIST:
                self._start_escape(Phase.WALL_FOLLOW)
                return

            # Right-wall follower with gentler turn cap (preserves SLAM quality)
            err = right - WALL_FOLLOW_DIST
            az  = max(-TURN_SPEED, min(TURN_SPEED, 0.8 * err))
            if right > 1.5:   # no wall on right – turn right to find one
                az = -TURN_SPEED * 0.6
            self._pub(lx=WALL_FOLLOW_SPEED * speed_mult, az=az)
            # Mark visited only when moving straight-ish (|az| small)
            if abs(az) < 0.25:
                self._mark_visited()
            return

        # ── PLAN ──────────────────────────────────────────────────────────────
        if self.phase == Phase.PLAN:
            self.waypoints = self._build_boustrophedon_waypoints()
            if not self.waypoints:
                self.get_logger().warn("No waypoints generated, retrying in 2 s.")
                self.phase_start = now
                return

            best_i = 0
            best_d = float("inf")
            for i, (wx, wy) in enumerate(self.waypoints):
                d = math.hypot(wx - self.robot_x, wy - self.robot_y)
                if d < best_d:
                    best_d, best_i = d, i
            self.wp_index = best_i
            self.phase    = Phase.GOTO_START
            self.get_logger().info(f"Starting CPP at wp {best_i}: {self.waypoints[best_i]}")
            return

        # ── GOTO_START ────────────────────────────────────────────────────────
        if self.phase == Phase.GOTO_START:
            if front < CRITICAL_DIST:
                self._start_escape(Phase.GOTO_START)
                return
            tx, ty = self.waypoints[self.wp_index]
            arrived = self._steer_to(tx, ty, SWEEP_SPEED * speed_mult)
            if arrived:
                self.phase         = Phase.SWEEP
                self.phase_start   = now
                self.stall_check_t = now
                self.stall_ref_x   = self.robot_x
                self.stall_ref_y   = self.robot_y
                self.get_logger().info("At start of sweep. Executing CPP.")
            return

        # ── SWEEP ─────────────────────────────────────────────────────────────
        if self.phase == Phase.SWEEP:
            if front < CRITICAL_DIST:
                self._start_escape(Phase.SWEEP)
                return

            if self._has_fresh_dirt() and front > CRITICAL_DIST * 1.5:
                offset = max(-1.0, min(1.0, self.dirt_offset or 0.0))
                lx = max(0.08, 0.22 * (1.0 - abs(offset))) * speed_mult
                az = -0.9 * offset
                if self.dirt_area_val > DIRT_AREA_STOP:
                    lx = az = 0.0
                self._pub(lx=lx, az=az)
                return

            if self._check_stall():
                self._start_escape(Phase.SWEEP)
                return

            if self.wp_index >= len(self.waypoints):
                if self.home_x is not None:
                    self.get_logger().info("Sweep complete. Returning home.")
                    self.phase       = Phase.RETURN_HOME
                    self.phase_start = now
                else:
                    self.phase = Phase.DONE
                return

            tx, ty = self.waypoints[self.wp_index]
            arrived = self._steer_to(tx, ty, SWEEP_SPEED * speed_mult)
            if arrived:
                self.wp_index += 1
            return

        # ── ESCAPE_BACK ───────────────────────────────────────────────────────
        if self.phase == Phase.ESCAPE_BACK:
            self._pub(lx=BACKUP_SPEED)
            if (now - self.phase_start) >= BACKUP_DURATION:
                # If already clear, skip spin to preserve map quality
                if self._front() > SLOW_DIST:
                    self.get_logger().info("Obstacle cleared during backup – skipping spin.")
                    self._finish_escape(now)
                else:
                    self.phase       = Phase.ESCAPE_SPIN
                    self.phase_start = now
            return

        # ── ESCAPE_SPIN ───────────────────────────────────────────────────────
        if self.phase == Phase.ESCAPE_SPIN:
            self._pub(az=self._spin_dir * TURN_SPEED_ESC)   # gentler rate

            if (now - self.phase_start) >= SPIN_DURATION:
                if self._front() < CRITICAL_DIST and self._escape_attempts < MAX_ESCAPE_RETRIES:
                    # Still blocked – retry with reversed spin direction
                    self._escape_attempts += 1
                    self.get_logger().warn(
                        f"Still blocked after spin, retry {self._escape_attempts}/{MAX_ESCAPE_RETRIES}."
                    )
                    self._spin_dir  *= -1.0
                    self.phase       = Phase.ESCAPE_BACK
                    self.phase_start = now
                else:
                    self._finish_escape(now)
            return

        # ── RETURN_HOME ───────────────────────────────────────────────────────
        if self.phase == Phase.RETURN_HOME:
            if front < CRITICAL_DIST:
                self._start_escape(Phase.RETURN_HOME)
                return
            arrived = self._steer_to(self.home_x, self.home_y, SWEEP_SPEED)
            if arrived:
                self.get_logger().info("HOME reached. Mission complete.")
                self._pub()
                self._save_map("mission-complete")
                self.phase = Phase.DONE
            return

    # ── Escape completion helper ──────────────────────────────────────────────

    def _finish_escape(self, now: float):
        """
        Called when escape sequence ends (either clear or max retries hit).
        If returning to SWEEP and the current waypoint is still blocked,
        skip it so we don't immediately re-trigger the same escape.
        """
        returning_to = self.escape_phase_return

        # Skip the problematic waypoint if SWEEP and still narrow ahead
        if (returning_to == Phase.SWEEP
                and self.wp_index < len(self.waypoints)
                and self._front() < SLOW_DIST):
            self.get_logger().warn(
                f"Skipping blocked waypoint {self.wp_index} after escape."
            )
            self.wp_index += 1

        self.get_logger().info(f"Escape done (attempts={self._escape_attempts}), resuming {returning_to.name}.")
        self.phase         = returning_to
        self.phase_start   = now
        self.stall_check_t = now
        self.stall_ref_x   = self.robot_x
        self.stall_ref_y   = self.robot_y
        self._escape_attempts = 0


def main():
    rclpy.init()
    node = CoverageExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub()
        node._save_map("shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()