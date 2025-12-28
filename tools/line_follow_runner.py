#!/usr/bin/env python3
import argparse
import math
import subprocess
import time
import re

from pymavlink import mavutil


def enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
    return x_enu, y_enu, -z_enu


def extract_pose(text: str, target: str):
    block = re.search(
        r'pose\s*\{\s*name:\s*"' + re.escape(target) + r'".*?\n\}',
        text,
        re.S,
    )
    if not block:
        return None

    b = block.group(0)

    def g(k: str) -> float:
        m = re.search(rf'{k}\s*:\s*([-\deE\.]+)', b)
        return float(m.group(1)) if m else 0.0

    x = g("x")
    y = g("y")
    z = g("z")
    return x, y, z


def parse_offsets(spec: str):
    offsets = []
    chunks = [c.strip() for c in spec.split(";") if c.strip()]
    for idx, chunk in enumerate(chunks, start=1):
        parts = [p.strip() for p in chunk.split(",") if p.strip()]
        if len(parts) != 2:
            raise ValueError(f"Invalid offset #{idx}: '{chunk}'")
        offsets.append((float(parts[0]), float(parts[1])))
    return offsets


def wait_heartbeat(master):
    master.wait_heartbeat()
    print(f"Heartbeat OK ({master.address})")


def arm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0, 0, 0, 0, 0, 0,
    )


def set_mode_offboard(master):
    master.set_mode("OFFBOARD")


def send_setpoint(master, x_ned, y_ned, z_ned, time_boot_ms):
    mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        x_ned,
        y_ned,
        z_ned,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def get_local_position_ned(master, timeout_s=1.0):
    return master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout_s)


def main():
    parser = argparse.ArgumentParser(description="Fly UAVs in a line above a moving ground vehicle.")
    parser.add_argument("--world", default="multi_quad7")
    parser.add_argument("--model", default="ground_vehicle")
    parser.add_argument("--gz-topic", default="/world/multi_quad7/pose/info")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--alt", type=float, default=10.0)
    parser.add_argument("--offsets", default="0,0; 0,5; 0,-5")
    parser.add_argument("--uav0", default="udp:127.0.0.1:14540")
    parser.add_argument("--uav1", default="udp:127.0.0.1:14541")
    parser.add_argument("--uav2", default="udp:127.0.0.1:14542")
    parser.add_argument("--auto-arm", action="store_true")
    parser.add_argument("--auto-takeoff", action="store_true")
    args = parser.parse_args()

    offsets = parse_offsets(args.offsets)
    if len(offsets) != 3:
        raise ValueError("Provide exactly 3 offsets (for UAV0, UAV1, UAV2).")

    masters = [
        mavutil.mavlink_connection(args.uav0),
        mavutil.mavlink_connection(args.uav1),
        mavutil.mavlink_connection(args.uav2),
    ]
    for m in masters:
        wait_heartbeat(m)

    t0 = time.monotonic()
    if args.auto_arm:
        for m in masters:
            print("Arming...")
            arm(m)
        time.sleep(1.0)

    if args.auto_takeoff:
        for m in masters:
            msg = get_local_position_ned(m, timeout_s=2.0)
            if msg is None:
                raise RuntimeError("No LOCAL_POSITION_NED available; cannot auto-takeoff.")
            takeoff_target = (msg.x, msg.y, -args.alt)
            for _ in range(20):
                t_ms = int((time.monotonic() - t0) * 1000)
                send_setpoint(m, *takeoff_target, t_ms)
                time.sleep(0.1)
            set_mode_offboard(m)

    dt = 1.0 / max(args.rate_hz, 1.0)
    print("Tracking ground vehicle...")
    while True:
        proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", args.gz_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        time.sleep(0.15)
        proc.terminate()
        out = proc.stdout.read() if proc.stdout else ""
        pose = extract_pose(out, args.model)
        if pose is None:
            time.sleep(dt)
            continue
        x, y, _ = pose
        for m, (dx, dy) in zip(masters, offsets):
            x_t = x + dx
            y_t = y + dy
            z_t = args.alt
            x_ned, y_ned, z_ned = enu_to_ned(x_t, y_t, z_t)
            t_ms = int((time.monotonic() - t0) * 1000)
            send_setpoint(m, x_ned, y_ned, z_ned, t_ms)
        time.sleep(dt)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
