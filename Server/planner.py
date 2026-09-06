# planner.py

import math
import logging
from heapq import heappush, heappop
from typing import List, Set, Tuple

import config as cfg
from detector import Pose

log = logging.getLogger("Planner")


class Planner:
    def __init__(self):
        self.cw = cfg.FRAME_W / cfg.GRID_COLS
        self.ch = cfg.FRAME_H / cfg.GRID_ROWS

    def _to_cell(self, px: float, py: float) -> Tuple[int, int]:
        c = int(px / self.cw)
        r = int(py / self.ch)
        return (max(0, min(cfg.GRID_COLS-1, c)),
                max(0, min(cfg.GRID_ROWS-1, r)))

    def _to_px(self, col: int, row: int) -> Tuple[int, int]:
        return (int((col+0.5)*self.cw), int((row+0.5)*self.ch))

    def _blocked(self, obstacles: List[Pose]) -> Set[Tuple[int,int]]:
        blocked = set()
        r = cfg.ROBOT_CELL_R
        for p in obstacles:
            cc, rc = self._to_cell(p.x, p.y)
            for dc in range(-r, r+1):
                for dr in range(-r, r+1):
                    nc, nr = cc+dc, rc+dr
                    if 0 <= nc < cfg.GRID_COLS and 0 <= nr < cfg.GRID_ROWS:
                        blocked.add((nc, nr))
        return blocked

    def plan(self, start: Pose, goal: Pose,
             obstacles: List[Pose] = []) -> List[Tuple[int,int]]:
        """A* from start to goal. obstacles = other robot poses."""
        s = self._to_cell(start.x, start.y)
        g = self._to_cell(goal.x,  goal.y)

        if s == g:
            return [self._to_px(*g)]

        blocked = self._blocked(obstacles)
        blocked.discard(s)
        blocked.discard(g)

        def h(c): return math.hypot(c[0]-g[0], c[1]-g[1])

        heap = [(h(s), 0.0, s, None)]
        came_from = {}
        g_score   = {s: 0.0}
        DIRS = [(-1,0),(1,0),(0,-1),(0,1),
                (-1,-1),(-1,1),(1,-1),(1,1)]

        while heap:
            f, gc, cur, parent = heappop(heap)
            if cur in came_from:
                continue
            came_from[cur] = parent
            if cur == g:
                path, node = [], cur
                while node:
                    path.append(self._to_px(*node))
                    node = came_from[node]
                path.reverse()
                log.debug(f"A* {len(path)} waypoints")
                return path[1:] if len(path) > 1 else path
            for dc, dr in DIRS:
                nb = (cur[0]+dc, cur[1]+dr)
                if not (0 <= nb[0] < cfg.GRID_COLS and
                        0 <= nb[1] < cfg.GRID_ROWS):
                    continue
                if nb in blocked or nb in came_from:
                    continue
                cost = 1.414 if (dc and dr) else 1.0
                ng   = gc + cost
                if ng < g_score.get(nb, float('inf')):
                    g_score[nb] = ng
                    heappush(heap, (ng+h(nb), ng, nb, cur))

        log.warning("A*: no path, direct fallback")
        return [self._to_px(*g)]
