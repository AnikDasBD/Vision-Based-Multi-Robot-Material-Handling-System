#!/usr/bin/env python3
"""
main.py — Multi-robot swarm warehouse server.

Runs N robots simultaneously (defined in config.ROBOT_IDS).
Keys: [q] quit  [e] E-Stop all  [r] resume all
"""

import cv2
import logging
import time
from typing import Dict, Set

import config as cfg
from detector import Detector, DetectionResult, Pose
from controller import Controller, Phase
from task_allocator import TaskAllocator
from collision_checker import CollisionChecker
import robot_comm as comm

logging.basicConfig(
    level=logging.DEBUG,
    format="[%(asctime)s] %(levelname)-8s  %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("Main")

# Phase colours for overlay
PHASE_COLORS = {
    Phase.TO_OBJECT:    (0, 165, 255),
    Phase.AT_OBJECT:    (0, 255, 255),
    Phase.TO_DROP:      (255, 255, 0),
    Phase.AT_DROP:      (0, 255, 0),
    Phase.TO_HOME_ZONE: (255, 0, 255),
    Phase.AT_HOME_ZONE: (200, 200, 200),
    Phase.DONE:         (0, 200, 0),
}

ROBOT_DRAW_COLORS = {0: (0, 255, 0), 1: (0, 200, 255)}


def draw_overlay(frame, controllers: Dict[int, Controller],
                 allocator: TaskAllocator,
                 det: DetectionResult,
                 offline: Set[int]):

    # Per-robot phase + path
    for rid, ctrl in controllers.items():
        color = ROBOT_DRAW_COLORS.get(rid, (255,255,255))
        phase_name = ctrl.phase.name
        status = "OFFLINE" if rid in offline else phase_name

        y_off = 35 + rid * 70
        cv2.putText(frame, f"R{rid}: {status}",
                    (10, y_off), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, color, 2)

        # A* path
        path, wp_idx = ctrl.get_path()
        for i in range(wp_idx, len(path)-1):
            cv2.line(frame, path[i], path[i+1], color, 1)
        if wp_idx < len(path):
            cv2.circle(frame, path[wp_idx], 6, (0,255,255), -1)

        # Target circle
        if ctrl.phase == Phase.TO_OBJECT and det:
            oid = ctrl._current_obj_id
            if oid and oid in det.objects:
                p = det.objects[oid]
                cv2.circle(frame, (int(p.x), int(p.y)), 20, color, 2)
                cv2.putText(frame, f"R{rid}->O{oid}",
                            (int(p.x)-30, int(p.y)-28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        elif ctrl.phase == Phase.TO_DROP and det and det.drop_zone:
            p = det.drop_zone
            cv2.circle(frame, (int(p.x), int(p.y)), 22, color, 2)

        elif ctrl.phase == Phase.TO_HOME_ZONE and det:
            hid  = cfg.CAR_ZONE_IDS.get(rid)
            home = det.zones.get(hid) if hid else None
            if home:
                cv2.circle(frame, (int(home.x), int(home.y)), 22, color, 2)

    # Collected count
    n_done  = len(allocator.collected)
    n_total = len(cfg.OBJECT_IDS)
    cv2.putText(frame, f"Collected: {n_done}/{n_total}",
                (10, 35 + len(cfg.ROBOT_IDS)*70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)


def main():
    log.info("=" * 55)
    log.info(f"  Fleet  : {cfg.ROBOT_IDS}")
    log.info(f"  IPs    : {cfg.ROBOT_IPS}")
    log.info(f"  Objects: {cfg.OBJECT_IDS}")
    log.info("  Keys: [q] quit  [e] E-Stop  [r] resume")
    log.info("=" * 55)

    # Ping all robots
    for rid in cfg.ROBOT_IDS:
        comm.ping(rid)

    detector  = Detector()
    allocator = TaskAllocator()
    checker   = CollisionChecker()

    controllers: Dict[int, Controller] = {
        rid: Controller(rid, allocator) for rid in cfg.ROBOT_IDS
    }

    # Offline tracking
    offline_counters: Dict[int, int] = {rid: 0 for rid in cfg.ROBOT_IDS}
    offline_robots:   Set[int]       = set()

    # Drop zone occupancy (only one robot drops at a time)
    drop_zone_occupant: int = -1   # -1 = nobody

    dt = 1.0 / cfg.CONTROL_HZ

    try:
        while True:
            t0 = time.time()

            det, frame = detector.read()
            if det is None or frame is None:
                log.warning("Bad frame")
                time.sleep(0.05)
                continue

            # ── Offline detection ─────────────────────────────────────────────
            for rid in cfg.ROBOT_IDS:
                if rid in det.robots:
                    if rid in offline_robots:
                        # Robot came back
                        offline_robots.discard(rid)
                        offline_counters[rid] = 0
                        controllers[rid].rejoin()
                        log.info(f"Robot {rid} back online")
                    else:
                        offline_counters[rid] = 0
                else:
                    offline_counters[rid] += 1
                    if (offline_counters[rid] >= cfg.OFFLINE_FRAMES
                            and rid not in offline_robots):
                        offline_robots.add(rid)
                        controllers[rid].go_offline()

            # ── Task allocation ───────────────────────────────────────────────
            idle_ids = [
                rid for rid, ctrl in controllers.items()
                if ctrl.is_idle and rid not in offline_robots
            ]
            allocator.update(det.robots, det.objects, idle_ids)

            # ── Drop zone queue ───────────────────────────────────────────────
            # Only one robot in AT_DROP at a time
            at_drop_robots = [
                rid for rid, ctrl in controllers.items()
                if ctrl.phase == Phase.AT_DROP
            ]
            drop_zone_occupant = at_drop_robots[0] if at_drop_robots else -1

            # ── Collision check ───────────────────────────────────────────────
            active_robots = {rid: pose for rid, pose in det.robots.items()
                             if rid not in offline_robots}
            phases        = {rid: ctrl.phase.name
                             for rid, ctrl in controllers.items()}
            dist_to_goals = {rid: ctrl.dist_to_goal
                             for rid, ctrl in controllers.items()}
            yield_set = checker.check(active_robots, phases, dist_to_goals)

            # ── Tick each controller ──────────────────────────────────────────
            for rid, ctrl in controllers.items():
                if rid in offline_robots:
                    continue

                other_poses = [pose for r, pose in det.robots.items()
                               if r != rid]
                drop_busy   = (drop_zone_occupant != -1 and
                               drop_zone_occupant != rid)

                ctrl.tick(
                    det,
                    should_yield      = rid in yield_set,
                    drop_zone_busy    = drop_busy,
                    other_robot_poses = other_poses,
                )

            # ── Check mission complete ────────────────────────────────────────
            all_done = all(ctrl.done for ctrl in controllers.values()
                           if ctrl.robot_id not in offline_robots)
            if all_done and len(offline_robots) < len(cfg.ROBOT_IDS):
                log.info("MISSION COMPLETE")

            # ── Draw & display ────────────────────────────────────────────────
            draw_overlay(frame, controllers, allocator, det, offline_robots)

            fps = 1.0 / max(time.time() - t0, 1e-9)
            cv2.putText(frame, f"FPS:{fps:.0f}",
                        (cfg.FRAME_W - 120, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            display = cv2.resize(frame, (1280, 720))

            if all_done:
                cv2.putText(display, "MISSION COMPLETE",
                            (300, 360), cv2.FONT_HERSHEY_SIMPLEX,
                            1.5, (0,255,0), 3)

            cv2.imshow("Swarm", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('e'):
                for rid, ctrl in controllers.items():
                    ctrl.estop()
                    comm.emergency_stop(rid)
            elif key == ord('r'):
                for rid, ctrl in controllers.items():
                    comm.clear_emergency(rid)
                    ctrl.resume()

            time.sleep(max(0.0, dt - (time.time() - t0)))

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        comm.stop_all()
        detector.release()
        log.info("Stopped.")


if __name__ == "__main__":
    main()
