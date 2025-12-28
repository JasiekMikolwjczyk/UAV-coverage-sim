#!/usr/bin/env python3
import argparse
import math
import time

from pymavlink import mavutil


def enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
    return x_enu, y_enu, -z_enu


def ned_to_enu(x_ned: float, y_ned: float, z_ned: float):
    return y_ned, x_ned, -z_ned


def build_zigzag_points(
    start_x: float,
    start_y: float,
    alt: float,
    first_dx: float,
    y_amp: float,
    step_x: float,
    count: int,
):
    points = []
    x = start_x + first_dx
    y = y_amp
    points.append((x, y, alt))
    for i in range(1, count):
        x = start_x + first_dx + i * step_x
        y = y_amp if i % 2 == 0 else -y_amp
        points.append((x, y, alt))
    return points


def parse_waypoints(spec: str, default_alt: float):
    points = []
    chunks = [c.strip() for c in spec.split(";") if c.strip()]
    for idx, chunk in enumerate(chunks, start=1):
        parts = [p.strip() for p in chunk.split(",") if p.strip()]
        if len(parts) == 2:
            x, y = float(parts[0]), float(parts[1])
            z = default_alt
        elif len(parts) == 3:
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        else:
            raise ValueError(f"Invalid waypoint #{idx}: '{chunk}'")
        points.append((x, y, z))
    return points


def wait_heartbeat(master):
    master.wait_heartbeat()
    print("Heartbeat OK")


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


def send_velocity_setpoint(master, vx, vy, vz, time_boot_ms):
    mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
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
        0.0,
        0.0,
        0.0,
        vx,
        vy,
        vz,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def get_local_position_ned(master, timeout_s=1.0):
    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout_s)
    return msg


def main():
    parser = argparse.ArgumentParser(description="Run a route via PX4 offboard.")
    parser.add_argument("--udp", default="udp:127.0.0.1:14540")
    parser.add_argument("--alt", type=float, default=10.0)
    parser.add_argument("--first-dx", type=float, default=5.0)
    parser.add_argument("--step-x", type=float, default=5.0)
    parser.add_argument("--y-amp", type=float, default=10.0)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--accept", type=float, default=0.5)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--auto-arm", action="store_true")
    parser.add_argument("--auto-takeoff", action="store_true")
    parser.add_argument("--start-current", action="store_true")
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--waypoints", type=str, default="")
    args = parser.parse_args()

    t0 = time.monotonic()
    master = mavutil.mavlink_connection(args.udp)
    wait_heartbeat(master)

    if args.auto_arm:
        print("Arming...")
        arm(master)
        time.sleep(1.0)

    if args.auto_takeoff:
        msg = get_local_position_ned(master, timeout_s=2.0)
        if msg is None:
            raise RuntimeError("No LOCAL_POSITION_NED available; cannot auto-takeoff.")

        takeoff_target = (msg.x, msg.y, -args.alt)
        print(f"Auto-takeoff target NED: {takeoff_target}")

        for _ in range(20):
            t_ms = int((time.monotonic() - t0) * 1000)
            send_setpoint(master, *takeoff_target, t_ms)
            time.sleep(0.1)

        set_mode_offboard(master)

        while True:
            t_ms = int((time.monotonic() - t0) * 1000)
            send_setpoint(master, *takeoff_target, t_ms)
            msg = get_local_position_ned(master, timeout_s=0.2)
            if msg:
                if abs(msg.z - takeoff_target[2]) <= args.accept:
                    break
            time.sleep(0.1)

    if args.start_current:
        msg = get_local_position_ned(master, timeout_s=2.0)
        if msg is None:
            raise RuntimeError("No LOCAL_POSITION_NED available; cannot use --start-current.")
        start_x, start_y, _ = ned_to_enu(msg.x, msg.y, msg.z)
        print(f"Using current position as start (ENU): ({start_x:.2f}, {start_y:.2f})")
    else:
        start_x, start_y = args.start_x, args.start_y

    if args.waypoints:
        points_enu = parse_waypoints(args.waypoints, default_alt=args.alt)
        if args.start_current:
            points_enu = [(x + start_x, y + start_y, z) for (x, y, z) in points_enu]
            print("Applying current-position offset to waypoints.")
        print(f"Waypoints (ENU): {points_enu}")
    else:
        points_enu = build_zigzag_points(
            start_x=start_x,
            start_y=start_y,
            alt=args.alt,
            first_dx=args.first_dx,
            y_amp=args.y_amp,
            step_x=args.step_x,
            count=args.count,
        )
        print(f"Zig-zag points (ENU): {points_enu}")

    print("Streaming setpoints...")

    if not args.auto_takeoff:
        first_ned = enu_to_ned(*points_enu[0])
        for _ in range(20):
            t_ms = int((time.monotonic() - t0) * 1000)
            send_setpoint(master, *first_ned, t_ms)
            time.sleep(0.1)

        set_mode_offboard(master)

    dt = 1.0 / max(args.rate_hz, 1.0)
    use_velocity = args.speed is not None and args.speed > 0.0
    if use_velocity:
        print(f"Velocity control enabled: speed={args.speed:.3f} m/s")

    for idx, p_enu in enumerate(points_enu):
        target_ned = enu_to_ned(*p_enu)
        print(f"Waypoint {idx + 1}/{len(points_enu)} -> ENU {p_enu}")
        while True:
            t_ms = int((time.monotonic() - t0) * 1000)
            msg = get_local_position_ned(master, timeout_s=dt)
            if msg:
                dx = target_ned[0] - msg.x
                dy = target_ned[1] - msg.y
                dz = target_ned[2] - msg.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if use_velocity and dist > 1e-3:
                    scale = args.speed / dist
                    vx = dx * scale
                    vy = dy * scale
                    vz = dz * scale
                    send_velocity_setpoint(master, vx, vy, vz, t_ms)
                else:
                    send_setpoint(master, *target_ned, t_ms)
                if dist <= args.accept:
                    break
            time.sleep(dt)

        time.sleep(0.2)

    print("Route complete.")


if __name__ == "__main__":
    main()
