#!/usr/bin/env python3
"""Autonomous vacuum explorer node.

Handles obstacle avoidance, dirt/trash detection via YOLO offsets,
semantic surface classification, and Gazebo model deletion.
"""

import math
import random
import subprocess

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

# ---------------------------------------------------------------------------
# Tunable thresholds
# ---------------------------------------------------------------------------
DANGER_DIST  = 0.35   # m — immediate obstacle, must turn
CAUTION_DIST = 0.70   # m — slow down and steer
ROAM_DIST    = 1.50   # m — open space, full speed

FORWARD_MAX_TIME = 8.0   # s before forced wander
BACKUP_DURATION  = 1.2   # s
MIN_TURN_TIME    = 0.8   # s before switching back to forward

FULL_SPEED   = 0.35   # m/s
SLOW_SPEED   = 0.25   # m/s
CAUTION_SPEED = 0.15  # m/s
BACKUP_SPEED = -0.20  # m/s
TURN_RATE    = 1.00   # rad/s
WANDER_RATE  = 0.80   # rad/s
CARPET_MULT  = 0.50   # speed multiplier on carpet

DIRT_CONF_MIN  = 0.35   # minimum YOLO confidence to chase
DIRT_FRESHNESS = 0.8    # s — discard old detections
DIRT_AREA_STOP = 0.08   # area ratio at which we stop (very close)

TRASH_PICKUP_DIST = 0.40  # m
DIRT_CLEAN_DIST   = 1.50  # m
SEM_MIN_AREA      = 400   # px — min semantic pixel count
WATER_MIN_AREA    = 500   # px


# ---------------------------------------------------------------------------
# Initial world state  (move to a YAML config if the world changes often)
# ---------------------------------------------------------------------------
DIRT_ZONES = {
    "dirt_zone":   (0.3,  0.9),
    "dirt_zone_2": (-0.4, 0.8),
    "dirt_zone_3": (0.0, -0.9),
}

TRASH_ITEMS = {
    "trash_1":  (-3.5, -1.5), "trash_2":  (-3.8, -1.2), "trash_3":  (-3.2, -1.8),
    "trash_4":  (-3.5,  3.8), "trash_5":  (-3.8,  3.5),
    "trash_6":  ( 4.5, -3.5), "trash_7":  ( 4.7, -3.8),
    "trash_8":  ( 4.5,  4.2), "trash_9":  ( 4.2,  3.9),
    "trash_10": ( 0.5,  1.0), "trash_11": ( 0.8,  0.7),
}


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__("autonomous_explorer")

        # Publishers / subscribers
        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan",         self._scan_cb,     10)
        self.create_subscription(Point,     "/dirt_target",  self._dirt_cb,     10)
        self.create_subscription(Odometry,  "/odom",         self._odom_cb,     10)
        self.create_subscription(String,    "/semantic_class", self._sem_class_cb, 10)
        self.create_subscription(Float32,   "/semantic_area",  self._sem_area_cb,  10)
        self.create_timer(0.1, self._control_loop)

        # Sensor state
        self.ranges: list[float] = []
        self.odom_x = self.odom_y = 0.0
        self.sem_class = "clear"
        self.sem_area  = 0.0

        # YOLO dirt target
        self.dirt_offset: float | None = None
        self.dirt_confidence = 0.0
        self.dirt_area       = 0.0
        self.last_dirt_time  = None

        # Mutable world state (items removed when vacuumed)
        self.dirt_items  = dict(DIRT_ZONES)
        self.trash_items = dict(TRASH_ITEMS)

        # FSM
        self.state         = "forward"
        self.state_timer   = 0.0
        self.forward_time  = 0.0
        self.last_turn_dir = 1

        self.get_logger().info("AutonomousExplorer started.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        self.ranges = list(msg.ranges)

    def _dirt_cb(self, msg: Point):
        self.dirt_offset     = msg.x
        self.dirt_confidence = msg.y
        self.dirt_area       = msg.z
        self.last_dirt_time  = self.get_clock().now()

    def _odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def _sem_class_cb(self, msg: String):
        self.sem_class = msg.data

    def _sem_area_cb(self, msg: Float32):
        self.sem_area = msg.data

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _sector_min(self, deg_start: int, deg_end: int) -> float:
        """Return minimum valid range reading in [deg_start, deg_end)."""
        if not self.ranges:
            return float("inf")
        n = len(self.ranges)
        vals = []
        for d in range(deg_start, deg_end):
            r = self.ranges[int(d * n / 360) % n]
            if math.isfinite(r) and r > 0.0:
                vals.append(r)
        return min(vals) if vals else float("inf")

    def _has_fresh_dirt(self) -> bool:
        if self.last_dirt_time is None or self.dirt_offset is None:
            return False
        age = (self.get_clock().now() - self.last_dirt_time).nanoseconds / 1e9
        return age < DIRT_FRESHNESS and self.dirt_confidence > DIRT_CONF_MIN

    def _robot_dist(self, pos: tuple[float, float]) -> float:
        return math.hypot(self.odom_x - pos[0], self.odom_y - pos[1])

    def _delete_gz_model(self, name: str):
        """Remove a Gazebo model via gz service."""
        cmd = [
            "gz", "service", "-s", "/world/my_house/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", f'name: "{name}", type: MODEL',
        ]
        subprocess.run(cmd, capture_output=True)
        self.get_logger().info(f"Removed model: {name}")

    def _closest_item(self, items: dict) -> tuple[str | None, float]:
        """Return (name, distance) of the nearest item, or (None, inf)."""
        best_name, best_dist = None, float("inf")
        for name, pos in items.items():
            d = self._robot_dist(pos)
            if d < best_dist:
                best_dist, best_name = d, name
        return best_name, best_dist

    # ------------------------------------------------------------------
    # World interaction
    # ------------------------------------------------------------------
    def _vacuum_nearby_trash(self):
        name, dist = self._closest_item(self.trash_items)
        if name and dist < TRASH_PICKUP_DIST:
            self.get_logger().info(f"Vacuumed {name} (dist={dist:.2f} m)")
            self._delete_gz_model(name)
            del self.trash_items[name]

    def _clean_nearby_dirt(self):
        name, dist = self._closest_item(self.dirt_items)
        if name and dist < DIRT_CLEAN_DIST:
            self.get_logger().info(f"Cleaned dirt patch {name}")
            self._delete_gz_model(name)
            del self.dirt_items[name]
            self.state = "backup"
            self.state_timer = 0.0

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        cmd = Twist()
        if not self.ranges:
            self.cmd_pub.publish(cmd)
            return

        front       = min(self._sector_min(350, 360), self._sector_min(0, 10))
        front_left  = self._sector_min(10,  60)
        front_right = self._sector_min(300, 350)
        left        = self._sector_min(60,  120)
        right       = self._sector_min(240, 300)

        self.state_timer += 0.1
        speed_mult = CARPET_MULT if (self.sem_class == "carpet" and self.sem_area > SEM_MIN_AREA) else 1.0

        # --- world interactions ---
        self._vacuum_nearby_trash()

        if self.sem_class == "water" and self.sem_area > WATER_MIN_AREA:
            if self.state != "backup":
                self.get_logger().info("Water detected — avoiding.")
                self.state, self.state_timer = "backup", 0.0

        if self.sem_class == "dirt" and self.sem_area > SEM_MIN_AREA:
            self._clean_nearby_dirt()

        # --- state transitions ---
        if front < 0.2 and self.state != "backup":
            self.state, self.state_timer = "backup", 0.0

        if self.state not in ("backup",) and self._has_fresh_dirt() and front > DANGER_DIST:
            if self.state != "chase_dirt":
                self.state, self.state_timer = "chase_dirt", 0.0

        # --- FSM actions ---
        if self.state == "forward":
            self.forward_time += 0.1
            if front < DANGER_DIST:
                self.last_turn_dir = 1 if left > right else -1
                self.state, self.state_timer, self.forward_time = "turn", 0.0, 0.0
            elif front < CAUTION_DIST:
                cmd.linear.x  = CAUTION_SPEED * speed_mult
                cmd.angular.z = self.last_turn_dir * 0.4
            else:
                cmd.linear.x = (FULL_SPEED if front > ROAM_DIST else SLOW_SPEED) * speed_mult
                if self.forward_time > FORWARD_MAX_TIME:
                    self.last_turn_dir = random.choice([-1, 1])
                    self.state, self.state_timer, self.forward_time = "wander", 0.0, 0.0

        elif self.state == "turn":
            cmd.angular.z = self.last_turn_dir * TURN_RATE
            if front > CAUTION_DIST and self.state_timer > MIN_TURN_TIME:
                self.state, self.state_timer = "forward", 0.0

        elif self.state == "wander":
            cmd.linear.x  = 0.1 * speed_mult
            cmd.angular.z = self.last_turn_dir * WANDER_RATE
            if self.state_timer > random.uniform(0.8, 2.0):
                self.state, self.state_timer, self.forward_time = "forward", 0.0, 0.0

        elif self.state == "chase_dirt":
            offset = max(-1.0, min(1.0, self.dirt_offset or 0.0))
            cmd.angular.z = -0.9 * offset
            cmd.linear.x  = max(0.08, 0.24 * (1.0 - min(1.0, abs(offset)))) * speed_mult
            if self.dirt_area > DIRT_AREA_STOP:
                cmd.linear.x = cmd.angular.z = 0.0
            if not self._has_fresh_dirt() or front < DANGER_DIST:
                self.state = "backup" if front < DANGER_DIST else "forward"
                self.state_timer = 0.0

        elif self.state == "backup":
            cmd.linear.x = BACKUP_SPEED
            if self.state_timer > BACKUP_DURATION:
                self.last_turn_dir = 1 if left > right else -1
                self.state, self.state_timer = "turn", 0.0

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = AutonomousExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()