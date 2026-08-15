import time

import cv2
import numpy as np
import requests

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


ROBOT_IP = "192.168.1.10"

SPEED = 0.05
ACC = 0.2

URL = f"http://{ROBOT_IP}:4242/current.jpg?annotations=off"
CAMERA_TIMEOUT_S = 2.0
MAX_CAMERA_FAILURES = 10
CAMERA_TO_TOOL_DEG = 0.0

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

MARKER_SIZE = 50.0
MARKER_ID = None
FX = FY = 800.0

TABLE_Z_BASE_MM = None
SAFE_OFFSET_MM = 150.0
Z_LIMITS_MM = (50.0, 900.0)
MAX_Z_STEP_MM = 10.0

MAX_STEP_MM = 20.0
DEADBAND_MM = 3.0
KP = 0.5

X_LIMITS_MM = None
Y_LIMITS_MM = None

MAX_TILT_DEG = 25.0

BOX_HSV_LOWER = np.array([5, 40, 20])
BOX_HSV_UPPER = np.array([35, 255, 255])
BOX_MIN_AREA = 800.0


def get_frame():
    try:
        r = requests.get(URL, timeout=CAMERA_TIMEOUT_S)
        r.raise_for_status()
    except requests.RequestException as e:
        print(f"camera: {e}")
        return None

    buf = np.frombuffer(r.content, np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def detect_aruco(frame, display):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None or len(ids) == 0:
        return None

    ids = ids.flatten()

    if MARKER_ID is None:
        idx = int(np.argmin(ids))
    else:
        match = np.where(ids == MARKER_ID)[0]
        if len(match) == 0:
            return None
        idx = int(match[0])

    pts = corners[idx].reshape(4, 2).astype(np.float32)

    sides = [float(np.linalg.norm(pts[i] - pts[(i + 1) % 4])) for i in range(4)]
    side_px = float(np.mean(sides))
    if side_px < 5.0:
        return None
    mm_per_px = MARKER_SIZE / side_px

    h = MARKER_SIZE / 2.0
    object_points = np.array([
        [-h,  h, 0],
        [ h,  h, 0],
        [ h, -h, 0],
        [-h, -h, 0],
    ], dtype=np.float32)

    cx, cy = frame.shape[1] / 2.0, frame.shape[0] / 2.0
    camera_matrix = np.array([
        [FX,  0, cx],
        [ 0, FY, cy],
        [ 0,  0,  1],
    ], dtype=np.float32)
    dist = np.zeros((5, 1), dtype=np.float32)

    distance_mm = None
    ok, _rvec, tvec = cv2.solvePnP(
        object_points, pts, camera_matrix, dist,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if ok:
        distance_mm = float(tvec[2][0])

    cv2.polylines(display, [pts.astype(np.int32)], True, (0, 255, 0), 2)

    return mm_per_px, distance_mm


def detect_box(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BOX_HSV_LOWER, BOX_HSV_UPPER)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_area = BOX_MIN_AREA

    for c in contours:
        area = cv2.contourArea(c)
        if area <= best_area:
            continue

        x, y, w, h = cv2.boundingRect(c)
        best_area = area
        best = (x + w // 2, y + h // 2)

    return best, mask


def tool_tilt_deg(tcp):
    R, _ = cv2.Rodrigues(np.array(tcp[3:6], dtype=np.float64))
    tool_z = R[:, 2]
    return float(np.degrees(np.arccos(np.clip(-tool_z[2], -1.0, 1.0))))


def move_robot(rtde_c, tcp, dx_mm, dy_mm, z_target_mm):
    a = np.radians(CAMERA_TO_TOOL_DEG)
    dx_tool = dx_mm * np.cos(a) - dy_mm * np.sin(a)
    dy_tool = dx_mm * np.sin(a) + dy_mm * np.cos(a)

    R, _ = cv2.Rodrigues(np.array(tcp[3:6], dtype=np.float64))
    delta_base = R @ np.array([dx_tool, dy_tool, 0.0])

    x_mm = tcp[0] * 1000.0 + delta_base[0]
    y_mm = tcp[1] * 1000.0 + delta_base[1]

    if X_LIMITS_MM and not (X_LIMITS_MM[0] <= x_mm <= X_LIMITS_MM[1]):
        print(f"BLOCKED: x={x_mm:.0f} mm outside {X_LIMITS_MM}")
        return False
    if Y_LIMITS_MM and not (Y_LIMITS_MM[0] <= y_mm <= Y_LIMITS_MM[1]):
        print(f"BLOCKED: y={y_mm:.0f} mm outside {Y_LIMITS_MM}")
        return False

    new_pose = list(tcp)
    new_pose[0] = x_mm / 1000.0
    new_pose[1] = y_mm / 1000.0

    if z_target_mm is not None:
        z_now_mm = tcp[2] * 1000.0
        step = float(np.clip(z_target_mm - z_now_mm, -MAX_Z_STEP_MM, MAX_Z_STEP_MM))
        z_mm = z_now_mm + step
        if not (Z_LIMITS_MM[0] <= z_mm <= Z_LIMITS_MM[1]):
            print(f"BLOCKED: z={z_mm:.0f} mm outside {Z_LIMITS_MM}")
            return False
        new_pose[2] = z_mm / 1000.0

    rtde_c.moveL(new_pose, SPEED, ACC)
    return True


def poll_exit(delay_ms=1):
    return (cv2.waitKey(delay_ms) & 0xFF) == 27


def shutdown(rtde_c, rtde_r):
    if rtde_c is not None:
        try:
            rtde_c.stopScript()
        except Exception as e:
            print(f"stopScript: {e}")
        try:
            rtde_c.disconnect()
        except Exception as e:
            print(f"control disconnect: {e}")

    if rtde_r is not None:
        try:
            rtde_r.disconnect()
        except Exception as e:
            print(f"receive disconnect: {e}")

    cv2.destroyAllWindows()


def main():
    z_target_mm = None
    if TABLE_Z_BASE_MM is not None:
        z_target_mm = TABLE_Z_BASE_MM + SAFE_OFFSET_MM
        if not (Z_LIMITS_MM[0] <= z_target_mm <= Z_LIMITS_MM[1]):
            raise SystemExit(
                f"Z target {z_target_mm:.0f} mm is outside {Z_LIMITS_MM}. "
                "TABLE_Z_BASE_MM must be a base-frame height, not a distance "
                "measured from the camera."
            )
        print(f"Z control ON, holding {z_target_mm:.0f} mm (base frame)")
    else:
        print("Z control OFF, holding current height.")

    rtde_c = None
    rtde_r = None
    failures = 0

    try:
        rtde_c = RTDEControl(ROBOT_IP)
        rtde_r = RTDEReceive(ROBOT_IP)

        print("Starting UR5e ArUco visual servo. ESC in a window to stop.")

        while True:
            frame = get_frame()
            if frame is None:
                failures += 1
                if failures >= MAX_CAMERA_FAILURES:
                    print("Camera unreachable, stopping.")
                    break
                if poll_exit(100):
                    break
                time.sleep(0.1)
                continue
            failures = 0

            display = frame.copy()

            marker = detect_aruco(frame, display)
            box, mask = detect_box(frame)

            if box is not None:
                cv2.circle(display, box, 5, (0, 0, 255), -1)

            if marker is not None and box is not None:
                mm_per_px, distance_mm = marker

                err_x_mm = (box[0] - frame.shape[1] / 2.0) * mm_per_px
                err_y_mm = (box[1] - frame.shape[0] / 2.0) * mm_per_px
                err_mm = float(np.hypot(err_x_mm, err_y_mm))

                tcp = rtde_r.getActualTCPPose()
                tilt = tool_tilt_deg(tcp)

                if tilt > MAX_TILT_DEG:
                    print(f"BLOCKED: tool tilted {tilt:.0f} deg from vertical")
                elif err_mm < DEADBAND_MM:
                    d = f", marker {distance_mm:.0f} mm" if distance_mm is not None else ""
                    print(f"On target ({err_mm:.1f} mm{d})")
                else:
                    scale = min(1.0, MAX_STEP_MM / (err_mm * KP))
                    dx_mm = err_x_mm * KP * scale
                    dy_mm = err_y_mm * KP * scale
                    print(f"err={err_mm:5.1f} mm  ->  dx={dx_mm:6.1f} dy={dy_mm:6.1f}")
                    move_robot(rtde_c, tcp, dx_mm, dy_mm, z_target_mm)

            cv2.imshow("frame", display)
            cv2.imshow("mask", mask)

            if poll_exit():
                break

    finally:
        shutdown(rtde_c, rtde_r)
        print("Stopped.")


if __name__ == "__main__":
    main()
