# task_allocator.py

import math
import logging
from typing import Dict, List, Optional, Set

import config as cfg
from detector import Pose

log = logging.getLogger("TaskAllocator")


class TaskAllocator:
    """
    Greedy Euclidean nearest-object task assignment.

    - Assigns uncollected, unassigned, visible objects to idle robots.
    - Handles robot offline: releases its assignment so another robot
      can pick up the task.
    - Handles robot rejoin: robot re-enters idle and gets a new assignment.
    - Scalable: works for any number of robots and objects from config.
    """

    def __init__(self):
        self._assignments: Dict[int, Optional[int]] = {
            rid: None for rid in cfg.ROBOT_IDS
        }
        self._collected: Set[int] = set()   # objects delivered to drop zone

    # ── Query ──────────────────────────────────────────────────────────────────
    def get_assignment(self, robot_id: int) -> Optional[int]:
        """Return object ID currently assigned to this robot, or None."""
        return self._assignments.get(robot_id)

    def is_all_done(self) -> bool:
        """True when every object that has appeared has been collected."""
        return len(self._collected) >= len(cfg.OBJECT_IDS)

    @property
    def collected(self) -> Set[int]:
        return set(self._collected)

    # ── Updates ───────────────────────────────────────────────────────────────
    def update(self,
               active_robots:  Dict[int, Pose],
               visible_objects: Dict[int, Pose],
               idle_robot_ids: List[int]) -> None:
        """
        Assign nearest uncollected unassigned object to each idle robot.
        Called every tick from main.py.

        active_robots:   currently detected robot poses
        visible_objects: currently detected object poses
        idle_robot_ids:  robot IDs in TO_OBJECT phase with no assignment
        """
        already_assigned = {
            v for v in self._assignments.values() if v is not None
        }
        free_objs = {
            oid: pose for oid, pose in visible_objects.items()
            if oid not in self._collected and oid not in already_assigned
        }

        for rid in sorted(idle_robot_ids):   # sorted = deterministic
            if not free_objs:
                break
            if rid not in active_robots:
                continue
            rpose = active_robots[rid]
            nearest = min(free_objs,
                          key=lambda oid: _dist(rpose, free_objs[oid]))
            self._assignments[rid] = nearest
            del free_objs[nearest]
            log.info(f"Allocated: Robot {rid} → Object {nearest} "
                     f"(dist={_dist(rpose, visible_objects[nearest]):.0f}px)")

    def mark_collected(self, robot_id: int) -> None:
        """Call when robot successfully picks up its object."""
        obj_id = self._assignments.get(robot_id)
        if obj_id is not None:
            self._collected.add(obj_id)
            self._assignments[robot_id] = None
            log.info(f"Object {obj_id} collected by Robot {robot_id} "
                     f"({len(self._collected)}/{len(cfg.OBJECT_IDS)} done)")

    def release_robot(self, robot_id: int) -> None:
        """
        Call when robot goes offline.
        Releases its assignment so another robot can take over.
        """
        obj_id = self._assignments.get(robot_id)
        if obj_id is not None:
            log.warning(f"Robot {robot_id} offline — releasing Object {obj_id}")
            self._assignments[robot_id] = None

    def reassign_on_rejoin(self, robot_id: int) -> None:
        """Call when robot comes back online — ensure it starts idle."""
        self._assignments[robot_id] = None
        log.info(f"Robot {robot_id} rejoined — marked idle")


def _dist(a: Pose, b: Pose) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
