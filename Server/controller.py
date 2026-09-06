# controller.py

import math
import time
import logging
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import config as cfg
from detector import Pose, DetectionResult
from planner import Planner
from task_allocator import TaskAllocator
import robot_comm as comm

log = logging.getLogger("Controller")


class Phase(Enum):
    TO_OBJECT    = auto()
    AT_OBJECT    = auto()   # magnet ON, 3.5s pause
    TO_DROP      = auto()   # magnet stays ON
    AT_DROP      = auto()   # magnet OFF, 2s pause
    TO_HOME_ZONE = auto()
    AT_HOME_ZONE = auto()
    DONE         = auto()


class Controller:
    """One controller instance per robot."""

    def __init__(self, robot_id: int, allocator: TaskAllocator):
        self.robot_id  = robot_id
        self.allocator = allocator
        self.planner   = Planner()
        self.phase     = Phase.TO_OBJECT
        self.t_pause   = 0.0
        self._estop    = False

        self._path:   List[Tuple[int,int]] = []
        self._wp_idx: int = 0

        self._current_obj_id: Optional[int] = None
        self._had_object:     bool = False
        self._had_drop_zone:  bool = False
        self._had_home_zone:  bool = False

        # Exposed for collision checker
        self.dist_to_goal: float = 0.0

        # ── NEW: PD heading controller state ──────────────────────────────────
        # Stores the heading error from the previous tick so we can compute
        # the derivative term (rate of change of error) each tick.
        # Reset whenever the robot resets or rejoins, so stale derivative
        # values from a previous run don't contaminate the new trajectory.
        self._prev_heading_err: float = 0.0

        # ── NEW: Stop hysteresis state ────────────────────────────────────────
        # True while the robot is in the stopped-near-goal dead-band.
        # Only cleared once distance exceeds RESUME_DISTANCE for this robot.
        # This prevents the rapid stop/start oscillation that occurs when
        # ArUco noise makes the measured distance flutter around STOP_DISTANCE.
        self._stopped: bool = False

    # ── Public ────────────────────────────────────────────────────────────────
    @property
    def has_object(self) -> bool:
        return self.phase in (Phase.AT_OBJECT, Phase.TO_DROP, Phase.AT_DROP)

    @property
    def is_idle(self) -> bool:
        """Idle = TO_OBJECT phase with no assignment."""
        return (self.phase == Phase.TO_OBJECT and
                self.allocator.get_assignment(self.robot_id) is None and
                self._current_obj_id is None)

    @property
    def done(self) -> bool:
        return self.phase == Phase.DONE

    def estop(self):
        self._estop = True
        comm.stop(self.robot_id)
        log.warning(f"R{self.robot_id} E-STOP")

    def resume(self):
        self._estop = False
        self._path   = []
        self._wp_idx = 0
        log.info(f"R{self.robot_id} resumed")

    def go_offline(self):
        """Called by main when robot disappears for too long."""
        comm.stop(self.robot_id)
        comm.magnet_off(self.robot_id)
        self.allocator.release_robot(self.robot_id)
        self._reset()
        log.warning(f"R{self.robot_id} declared offline — state reset")

    def rejoin(self):
        """Called by main when robot reappears after being offline."""
        self.allocator.reassign_on_rejoin(self.robot_id)
        self._reset()
        log.info(f"R{self.robot_id} rejoined — starting from idle")

    def get_path(self):
        return self._path, self._wp_idx

    # ── Main tick ─────────────────────────────────────────────────────────────
    def tick(self,
             det:               DetectionResult,
             should_yield:      bool = False,
             drop_zone_busy:    bool = False,
             other_robot_poses: List[Pose] = []) -> None:

        if self._estop or self.phase == Phase.DONE:
            return

        robot = det.robots.get(self.robot_id)
        if robot is None:
            log.debug(f"R{self.robot_id} not visible")
            return

        # Yield to higher-priority robot
        if should_yield:
            comm.stop(self.robot_id)
            self._path   = []   # force replan next tick
            self._wp_idx = 0
            # Reset derivative so we don't get a spike on the next steer call
            self._prev_heading_err = 0.0
            return

        # ── TO_OBJECT ─────────────────────────────────────────────────────────
        if self.phase == Phase.TO_OBJECT:

            # Get assignment from allocator
            if self._current_obj_id is None:
                self._current_obj_id = self.allocator.get_assignment(self.robot_id)

            if self._current_obj_id is None:
                # No assignment yet — wait and rotate slowly to find objects
                comm.drive(self.robot_id, 0.0, 0.25)
                self.dist_to_goal = 9999
                return

            target = det.objects.get(self._current_obj_id)

            if target is None:
                if self._had_object:
                    # Marker gone — robot is on top of object → trigger pickup
                    self._trigger_pickup()
                else:
                    log.debug(f"R{self.robot_id} obj {self._current_obj_id} "
                              f"not visible — searching")
                    comm.drive(self.robot_id, 0.05, 0.3)
                    self.dist_to_goal = 9999
                return

            self._had_object  = True
            self.dist_to_goal = _dist(robot, target)

            if self.dist_to_goal < cfg.AT_GOAL_OBJ_PX:
                self._trigger_pickup()
                return

            self._path   = self.planner.plan(robot, target, other_robot_poses)
            self._wp_idx = 0
            self._follow(robot)

        # ── AT_OBJECT ─────────────────────────────────────────────────────────
        elif self.phase == Phase.AT_OBJECT:
            self.dist_to_goal = 0
            if time.time() - self.t_pause >= cfg.PAUSE_AT_OBJECT:
                self.allocator.mark_collected(self.robot_id)
                log.info(f"R{self.robot_id} picked up obj — heading to drop")
                self._path   = []
                self._wp_idx = 0
                self.phase   = Phase.TO_DROP

        # ── TO_DROP ───────────────────────────────────────────────────────────
        elif self.phase == Phase.TO_DROP:

            if det.drop_zone is None:
                if self._had_drop_zone:
                    self._trigger_drop()
                else:
                    comm.drive(self.robot_id, 0.0, 0.25)
                    self.dist_to_goal = 9999
                return

            self._had_drop_zone  = True
            self.dist_to_goal    = _dist(robot, det.drop_zone)

            # Wait if another robot is currently dropping
            if drop_zone_busy and self.dist_to_goal < cfg.DROP_ZONE_HOLD_DIST:
                comm.stop(self.robot_id)
                self._path   = []
                self._wp_idx = 0
                log.debug(f"R{self.robot_id} waiting — drop zone busy")
                return

            if self.dist_to_goal < cfg.AT_GOAL_DROP_PX:
                self._trigger_drop()
                return

            self._path   = self.planner.plan(robot, det.drop_zone,
                                             other_robot_poses)
            self._wp_idx = 0
            self._follow(robot)

        # ── AT_DROP ───────────────────────────────────────────────────────────
        elif self.phase == Phase.AT_DROP:
            self.dist_to_goal = 0
            if time.time() - self.t_pause >= cfg.PAUSE_AT_DROP:
                remaining = [o for o in cfg.OBJECT_IDS
                             if o not in self.allocator.collected]
                if remaining:
                    log.info(f"R{self.robot_id} drop done — "
                             f"{len(remaining)} objects remain")
                    self._current_obj_id = None
                    self._had_object     = False
                    self._had_drop_zone  = False
                    self._path   = []
                    self._wp_idx = 0
                    self.phase   = Phase.TO_OBJECT
                else:
                    log.info(f"R{self.robot_id} all delivered — going home")
                    self._path   = []
                    self._wp_idx = 0
                    self.phase   = Phase.TO_HOME_ZONE

        # ── TO_HOME_ZONE ──────────────────────────────────────────────────────
        elif self.phase == Phase.TO_HOME_ZONE:
            home_id = cfg.CAR_ZONE_IDS.get(self.robot_id)
            home    = det.zones.get(home_id) if home_id else None

            if home is None:
                if self._had_home_zone:
                    comm.stop(self.robot_id)
                    self._had_home_zone = False
                    log.info(f"R{self.robot_id} home zone marker lost — parked")
                    self.t_pause = time.time()
                    self.phase   = Phase.AT_HOME_ZONE
                else:
                    comm.drive(self.robot_id, 0.0, 0.25)
                    self.dist_to_goal = 9999
                return

            self._had_home_zone = True
            self.dist_to_goal   = _dist(robot, home)

            if self.dist_to_goal < cfg.AT_GOAL_CAR_ZONE_PX:
                comm.stop(self.robot_id)
                self._had_home_zone = False
                log.info(f"R{self.robot_id} parked at home zone {home_id}")
                self.t_pause = time.time()
                self.phase   = Phase.AT_HOME_ZONE
                return

            self._path   = self.planner.plan(robot, home, other_robot_poses)
            self._wp_idx = 0
            self._follow(robot)

        # ── AT_HOME_ZONE ──────────────────────────────────────────────────────
        elif self.phase == Phase.AT_HOME_ZONE:
            self.dist_to_goal = 0
            if time.time() - self.t_pause >= cfg.PAUSE_AT_DROP:
                log.info(f"R{self.robot_id} mission complete")
                comm.stop(self.robot_id)
                self.phase = Phase.DONE

    # ── Triggers ──────────────────────────────────────────────────────────────
    def _trigger_pickup(self):
        comm.stop(self.robot_id)
        comm.magnet_on(self.robot_id)
        self._had_object  = False
        self._path        = []
        self._wp_idx      = 0
        log.info(f"R{self.robot_id} PICKUP obj {self._current_obj_id} "
                 f"— magnet ON, pausing {cfg.PAUSE_AT_OBJECT}s")
        self.t_pause      = time.time()
        self.phase        = Phase.AT_OBJECT

    def _trigger_drop(self):
        comm.stop(self.robot_id)
        comm.magnet_off(self.robot_id)
        self._had_drop_zone = False
        self._path          = []
        self._wp_idx        = 0
        log.info(f"R{self.robot_id} DROP — magnet OFF, pausing {cfg.PAUSE_AT_DROP}s")
        self.t_pause        = time.time()
        self.phase          = Phase.AT_DROP

    # ── Path follower ─────────────────────────────────────────────────────────
    def _follow(self, robot: Pose):
        if not self._path:
            comm.stop(self.robot_id)
            return
        wx, wy = self._path[0]
        self._steer(robot, wx, wy)

    # ── Heading controller (PD + trim + stop hysteresis) ─────────────────────
    def _steer(self, robot: Pose, gx: float, gy: float):
        dx   = gx - robot.x
        dy   = gy - robot.y
        dist = math.hypot(dx, dy)

        # ── Stop hysteresis ───────────────────────────────────────────────────
        # Read per-robot thresholds; fall back to sensible defaults if missing.
        stop_d, resume_d = cfg.STOP_HYSTERESIS.get(self.robot_id, (40, 60))

        if self._stopped:
            # Already stopped near goal — stay stopped until far enough away.
            if dist < resume_d:
                log.debug(f"R{self.robot_id} hysteresis HOLD "
                          f"dist={dist:.0f} < resume={resume_d}")
                comm.stop(self.robot_id)
                return
            # Distance grew back (e.g. goal moved, new waypoint) → un-stop.
            self._stopped = False
            log.debug(f"R{self.robot_id} hysteresis RELEASE dist={dist:.0f}")

        if dist < stop_d:
            # Enter the stopped dead-band.
            self._stopped          = True
            self._prev_heading_err = 0.0   # clear derivative memory
            log.debug(f"R{self.robot_id} hysteresis STOP dist={dist:.0f}")
            comm.stop(self.robot_id)
            return

        # ── Heading error ─────────────────────────────────────────────────────
        desired = (math.degrees(math.atan2(dy, dx)) + 360) % 360
        err = desired - robot.theta
        if err >  180: err -= 360
        if err < -180: err += 360

        # Apply deadzone — suppress small ArUco noise wobbles.
        if abs(err) < cfg.HEADING_DEADZONE_DEG:
            err = 0.0

        # ── PD angular velocity ───────────────────────────────────────────────
        # P term: proportional to current error (same as before).
        # D term: proportional to *change* in error since last tick.
        #   Positive derivative → error is growing  → dampen turn.
        #   Negative derivative → error is shrinking → allow faster correction.
        # KD_ANG is in the same unit space as KP_ANG (both operate on
        # err in degrees, converted to radians below).
        d_err = err - self._prev_heading_err
        self._prev_heading_err = err

        omega = cfg.KP_ANG * math.radians(err) + cfg.KD_ANG * math.radians(d_err)
        omega = max(-cfg.MAX_W, min(cfg.MAX_W, omega))

        # ── Adaptive forward speed ────────────────────────────────────────────
        # When the heading error is large the robot should mostly turn, not
        # drive forward (same taper factor as before — preserving existing logic).
        tf = max(0.05, 1.0 - abs(err) / 90.0)
        v  = cfg.KP_LIN * dist * tf
        v  = max(0.0, min(cfg.MAX_V, v))

        # ── Motor trim ────────────────────────────────────────────────────────
        # Convert v/omega to normalised wheel speeds exactly as robot_comm does,
        # then inject per-robot trim before clamping.  This keeps the trim effect
        # consistent regardless of speed, without duplicating the PWM maths.
        left_trim, right_trim = cfg.MOTOR_TRIM.get(self.robot_id, (0, 0))

        if left_trim != 0 or right_trim != 0:
            # Replicate robot_comm.drive() normalisation so we can add trim
            # at the PWM level and then pass the adjusted values directly.
            MAX_PWM = 120
            v_n     = v     / cfg.MAX_V if cfg.MAX_V > 0 else 0.0
            omega_n = omega / cfg.MAX_W if cfg.MAX_W > 0 else 0.0
            left    = v_n + omega_n
            right   = v_n - omega_n
            scale   = MAX_PWM / max(1.0, abs(left), abs(right))
            pwm_l   = int(left  * scale) + left_trim
            pwm_r   = int(right * scale) + right_trim
            pwm_l   = max(-255, min(255, pwm_l))
            pwm_r   = max(-255, min(255, pwm_r))
            DUR_MS  = int((1000 / cfg.CONTROL_HZ) * 2)
            log.debug(f"R{self.robot_id} TRIM dist={dist:.0f} "
                      f"err={err:.1f} d_err={d_err:.1f} "
                      f"v={v:.3f} w={omega:.3f} "
                      f"L={pwm_l}(+{left_trim}) R={pwm_r}(+{right_trim})")
            import robot_comm as _comm_mod
            _comm_mod._post(self.robot_id,
                            {"cmd": "MOVE",
                             "speed_l": pwm_l,
                             "speed_r": pwm_r,
                             "dur_ms":  DUR_MS})
        else:
            # No trim needed — use the standard comm path.
            log.debug(f"R{self.robot_id} dist={dist:.0f} "
                      f"err={err:.1f} d_err={d_err:.1f} "
                      f"v={v:.3f} w={omega:.3f}")
            comm.drive(self.robot_id, v, omega)

    def _reset(self):
        self.phase             = Phase.TO_OBJECT
        self._path             = []
        self._wp_idx           = 0
        self._current_obj_id   = None
        self._had_object       = False
        self._had_drop_zone    = False
        self._had_home_zone    = False
        self.dist_to_goal      = 0.0
        # Reset PD and hysteresis state so stale values don't carry over
        # into a new trajectory after an offline/rejoin cycle.
        self._prev_heading_err = 0.0
        self._stopped          = False


def _dist(a: Pose, b: Pose) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
