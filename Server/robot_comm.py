# robot_comm.py

import requests
import logging
import config as cfg

log = logging.getLogger("RobotComm")

MAX_PWM = 120
DUR_MS  = int((1000 / cfg.CONTROL_HZ) * 2)


def _url(robot_id: int) -> str:
    return f"http://{cfg.ROBOT_IPS[robot_id]}:{cfg.ROBOT_PORT}/cmd"


def ping(robot_id: int) -> bool:
    base = f"http://{cfg.ROBOT_IPS[robot_id]}:{cfg.ROBOT_PORT}"
    for attempt in range(1, 4):
        try:
            r = requests.get(f"{base}/", timeout=2.0)
            log.info(f"Robot {robot_id} reachable (attempt {attempt}) "
                     f"HTTP {r.status_code}")
            return True
        except requests.exceptions.ConnectionError:
            log.warning(f"  Robot {robot_id} attempt {attempt}: ConnectionError")
        except requests.exceptions.Timeout:
            log.warning(f"  Robot {robot_id} attempt {attempt}: Timeout")
    log.warning(f"Robot {robot_id} NOT reachable")
    return False


def drive(robot_id: int, v: float, omega: float):
    v_n     = v     / cfg.MAX_V if cfg.MAX_V > 0 else 0.0
    omega_n = omega / cfg.MAX_W if cfg.MAX_W > 0 else 0.0
    left    = v_n + omega_n
    right   = v_n - omega_n
    scale   = MAX_PWM / max(1.0, abs(left), abs(right))
    pwm_l   = max(-255, min(255, int(left  * scale)))
    pwm_r   = max(-255, min(255, int(right * scale)))
    log.debug(f"R{robot_id} drive v={v:.2f} w={omega:.2f} "
              f"L={pwm_l} R={pwm_r}")
    _post(robot_id, {"cmd":"MOVE","speed_l":pwm_l,
                     "speed_r":pwm_r,"dur_ms":DUR_MS})


def stop(robot_id: int):
    log.debug(f"R{robot_id} STOP")
    _post(robot_id, {"cmd": "STOP"})


def magnet_on(robot_id: int):
    log.info(f"R{robot_id} MAGNET ON")
    _post(robot_id, {"cmd": "MAGNET_ON"})


def magnet_off(robot_id: int):
    log.info(f"R{robot_id} MAGNET OFF")
    _post(robot_id, {"cmd": "MAGNET_OFF"})


def emergency_stop(robot_id: int):
    log.warning(f"R{robot_id} EMERGENCY_STOP")
    _post(robot_id, {"cmd": "EMERGENCY_STOP"})


def clear_emergency(robot_id: int):
    log.info(f"R{robot_id} CLEAR_EMERGENCY")
    _post(robot_id, {"cmd": "CLEAR_EMERGENCY"})


def stop_all():
    for rid in cfg.ROBOT_IDS:
        stop(rid)


def _post(robot_id: int, payload: dict):
    url = _url(robot_id)
    try:
        r = requests.post(url, json=payload, timeout=0.4)
        if r.status_code != 200:
            log.warning(f"R{robot_id} HTTP {r.status_code} "
                        f"body={r.text[:40]}")
    except requests.exceptions.ConnectionError:
        log.warning(f"R{robot_id} ConnectionError")
    except requests.exceptions.Timeout:
        log.warning(f"R{robot_id} Timeout")
    except Exception as e:
        log.warning(f"R{robot_id} {e}")
