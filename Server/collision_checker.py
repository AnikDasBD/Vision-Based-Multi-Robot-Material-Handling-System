# collision_checker.py

import math
import logging
from typing import Dict, Set

import config as cfg
from detector import Pose

log = logging.getLogger("Collision")

# Phases that mean the robot is carrying an object
_CARRYING_PHASES = {"AT_OBJECT", "TO_DROP", "AT_DROP"}


class CollisionChecker:
    """
    Geometric circle collision model.

    Each robot is a circle of radius ROBOT_RADIUS_PX + SAFETY_MARGIN_PX.
    If the distance between two robot centers <= sum of radii, a collision
    is predicted and the lower-priority robot is told to yield (stop + replan).

    Priority rules:
        1. Robot carrying object beats robot not carrying object.
        2. If both carry OR both are empty → robot further from its goal yields.
    """

    def check(self,
              robots:         Dict[int, Pose],
              phases:         Dict[int, str],
              dist_to_goals:  Dict[int, float]) -> Set[int]:
        """
        Returns set of robot_ids that must yield this tick.

        robots:        {robot_id: Pose}
        phases:        {robot_id: phase_name_str}
        dist_to_goals: {robot_id: pixels_to_current_goal}
        """
        yielding: Set[int] = set()
        ids = list(robots.keys())

        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                rid_a = ids[i]
                rid_b = ids[j]

                pa = robots[rid_a]
                pb = robots[rid_b]
                d  = math.hypot(pa.x - pb.x, pa.y - pb.y)

                if d > cfg.COLLISION_DIST_PX:
                    continue   # safe

                log.debug(f"Collision risk: R{rid_a} ↔ R{rid_b} "
                          f"dist={d:.0f}px (threshold={cfg.COLLISION_DIST_PX}px)")

                carrying_a = phases.get(rid_a, "") in _CARRYING_PHASES
                carrying_b = phases.get(rid_b, "") in _CARRYING_PHASES

                if carrying_a and not carrying_b:
                    yielding.add(rid_b)
                elif carrying_b and not carrying_a:
                    yielding.add(rid_a)
                else:
                    # Both carrying or both empty → further from goal yields
                    da = dist_to_goals.get(rid_a, 0)
                    db = dist_to_goals.get(rid_b, 0)
                    if da >= db:
                        yielding.add(rid_a)
                    else:
                        yielding.add(rid_b)

        if yielding:
            log.info(f"Collision: robots {yielding} yield this tick")

        return yielding
