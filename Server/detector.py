# detector.py

import cv2
import numpy as np
import math
import logging
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import config as cfg

log = logging.getLogger("Detector")


@dataclass
class Pose:
    x: float
    y: float
    theta: float   # degrees, 0-360


@dataclass
class DetectionResult:
    robots:    Dict[int, Pose]   # {robot_id: Pose}  all detected robots
    objects:   Dict[int, Pose]   # {marker_id: Pose}
    drop_zone: Optional[Pose]
    zones:     Dict[int, Pose]   # {marker_id: Pose} all zone markers


def _yaw_from_rvec(rvec) -> float:
    R, _ = cv2.Rodrigues(rvec)
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return (yaw + 360) % 360


def _smooth_angle(prev: float, new: float, alpha: float) -> float:
    diff = (new - prev + 180) % 360 - 180
    return (prev + alpha * diff) % 360


def _apply_robot_offset(pose: Pose, robot_id: int) -> Pose:
    offset = cfg.ROBOT_MARKER_OFFSETS.get(robot_id, 0)
    if offset == 0:
        return pose
    theta = math.radians(pose.theta)
    return Pose(pose.x + offset * math.cos(theta),
                pose.y + offset * math.sin(theta),
                pose.theta)


def _apply_obj_offset(pose: Pose) -> Pose:
    offset = cfg.OBJ_CENTER_OFFSET_PX
    if offset == 0:
        return pose
    theta = math.radians(pose.theta)
    return Pose(pose.x - offset * math.cos(theta),
                pose.y - offset * math.sin(theta),
                pose.theta)


class Detector:
    def __init__(self):
        self.cap = cv2.VideoCapture(cfg.CAM_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg.FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.FRAME_H)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {cfg.CAM_INDEX}")
        log.info(f"Camera {cfg.FRAME_W}x{cfg.FRAME_H} opened")

        self._smooth: Dict[int, Pose] = {}
        self._ref_positions: Dict[int, Tuple[float, float]] = {}
        self._stab_ready = False

    def _apply_smooth(self, mid: int, raw: Pose) -> Pose:
        a = cfg.SMOOTH_ALPHA
        if mid not in self._smooth:
            self._smooth[mid] = Pose(raw.x, raw.y, raw.theta)
            return Pose(raw.x, raw.y, raw.theta)
        s = self._smooth[mid]
        s.x     = s.x + a * (raw.x - s.x)
        s.y     = s.y + a * (raw.y - s.y)
        s.theta = _smooth_angle(s.theta, raw.theta, a)
        return Pose(s.x, s.y, s.theta)

    def _get_drift(self, raw_zones: Dict[int, Tuple[float, float]]):
        """Compute mean camera drift using fixed reference markers."""
        refs = [m for m in cfg.REFERENCE_MARKER_IDS if m in raw_zones]
        if not refs:
            return 0.0, 0.0
        if not self._stab_ready:
            if len(refs) >= 2:
                for m in refs:
                    self._ref_positions[m] = raw_zones[m]
                self._stab_ready = True
                log.info(f"Stabilisation ready ({len(refs)} refs: {refs})")
            return 0.0, 0.0
        drifts = [(raw_zones[m][0] - self._ref_positions[m][0],
                   raw_zones[m][1] - self._ref_positions[m][1])
                  for m in refs if m in self._ref_positions]
        if not drifts:
            return 0.0, 0.0
        return (sum(d[0] for d in drifts) / len(drifts),
                sum(d[1] for d in drifts) / len(drifts))

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, cfg.ARUCO_DICT, parameters=cfg.ARUCO_PARAMS)

        raw_zones: Dict[int, Tuple[float, float]] = {}
        raw_det   = {}  # mid → (raw_x, raw_y, raw_t, rvec, tvec)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, mid in enumerate(ids.flatten()):
                if mid in cfg.ZONE_IDS:
                    mlen = cfg.MARKER_LEN_ZONE
                elif mid in cfg.OBJECT_IDS:
                    mlen = cfg.MARKER_LEN_OBJECT
                elif mid in cfg.ROBOT_IDS:
                    mlen = cfg.MARKER_LEN_ROBOT
                else:
                    continue

                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[i]], mlen, cfg.CAM_MTX, cfg.DIST)

                rx = float(np.mean(corners[i][0][:, 0]))
                ry = float(np.mean(corners[i][0][:, 1]))
                rt = _yaw_from_rvec(rvec[0])
                raw_det[mid] = (rx, ry, rt, rvec, tvec)

                if mid in cfg.ZONE_IDS:
                    raw_zones[mid] = (rx, ry)

        dx, dy = self._get_drift(raw_zones)

        robots, objects, drop_zone, zones = {}, {}, None, {}

        for mid, (rx, ry, rt, rvec, tvec) in raw_det.items():
            cx, cy = rx - dx, ry - dy

            if mid in cfg.ROBOT_IDS:
                yaw_off = cfg.YAW_OFFSETS.get(mid, 0)
                rt = (rt + yaw_off) % 360

            pose = self._apply_smooth(mid, Pose(cx, cy, rt))

            if mid in cfg.ROBOT_IDS:
                pose = _apply_robot_offset(pose, mid)
                robots[mid] = pose
                color = (0, 255, 0)
            elif mid in cfg.OBJECT_IDS:
                pose = _apply_obj_offset(pose)
                objects[mid] = pose
                color = (0, 165, 255)
            elif mid in cfg.ZONE_IDS:
                zones[mid] = pose
                if mid == cfg.DROP_ID:
                    drop_zone = pose
                color = (255, 0, 0)

            cv2.drawFrameAxes(frame, cfg.CAM_MTX, cfg.DIST,
                              rvec[0], tvec[0], 0.04)
            name = cfg.MARKER_NAMES.get(mid, f"ID{mid}")
            cv2.putText(frame,
                        f"{name}({pose.x:.0f},{pose.y:.0f})t{pose.theta:.0f}",
                        (int(rx)-50, int(ry)-14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

        stab = f"Stab:{'ON' if self._stab_ready else 'wait'}"
        cv2.putText(frame, stab, (10, cfg.FRAME_H-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,0), 1)

        return DetectionResult(robots, objects, drop_zone, zones), frame

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
