from __future__ import annotations

from typing import TYPE_CHECKING

import serial

from scripts.common import get_station_module

if TYPE_CHECKING:
    from SDCStation import SerialController


def script_demo2_refactored(
    serial_ports: dict[str, serial.Serial],
    active_port_keys: list[str],
    control: SerialController,
    *_,
):
    """Demo 2025-02-20: home/move stages, run pump sequence, then reset."""
    _ = active_port_keys

    station = get_station_module()
    s_name = "demo2"
    try:
        if not station.check_ports(serial_ports, ["x", "y", "z", "p"]):
            print(station.Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
            return

        control.enter_context(s_name)

        x_retract = 0
        y_retract = 0
        z_retract = 0  # or 100?

        x_measure = 200
        y_measure = 95
        z_measure = 60
        z_premeasure_offset = 15
        z_premeasure = z_measure - z_premeasure_offset

        ## "Home" the stages
        control.queue_command("x", "<home>", 2, None)
        control.queue_command("y", "<home>", 2, None)
        control.queue_command("z", "<home>", 2, None)
        _ = control.execute_queued_commands()

        ## Move stages into retracted position
        control.queue_command(
            "x",
            f"<goto {x_retract}>",
            None,
            x_retract / station.LS_SPEED_DEFAULT + station.DEFAULT_SLEEP,
        )
        control.queue_command(
            "y",
            f"<goto {y_retract}>",
            None,
            y_retract / station.LS_SPEED_DEFAULT + station.DEFAULT_SLEEP,
        )
        control.queue_command(
            "z",
            f"<goto {z_retract}>",
            None,
            z_retract / station.LS_SPEED_DEFAULT + station.DEFAULT_SLEEP,
        )
        _ = control.execute_queued_commands()

        ## Move sample into position and configure pumps
        # Synchronize by changing speeds to arrive at the same time
        x_move_dist = abs(x_measure - x_retract)
        y_move_dist = abs(y_measure - y_retract)
        z_move_dist = abs(z_premeasure - z_retract)
        time_move_sample_to_measure = (
            max(x_move_dist, y_move_dist, z_move_dist) / station.LS_SPEED_DEFAULT
        )
        x_speed = x_move_dist / time_move_sample_to_measure
        y_speed = y_move_dist / time_move_sample_to_measure
        z_speed = z_move_dist / time_move_sample_to_measure
        z_s_approach = z_speed / 1  # /2

        # Move all stages
        control.queue_command(
            "x",
            f"<goto {x_measure} {x_speed}>",
            None,
            time_move_sample_to_measure,
        )
        control.queue_command(
            "y",
            f"<goto {y_measure} {y_speed}>",
            None,
            time_move_sample_to_measure,
        )
        control.queue_command(
            "z",
            f"<goto {z_premeasure} {z_speed}>",
            None,
            time_move_sample_to_measure,
        )
        # Move z into final position and wait to settle
        control.queue_command(
            "z",
            f"<goto {z_measure} {z_s_approach}>",
            None,
            z_premeasure_offset / z_s_approach + station.DEFAULT_SLEEP_LONG,
            s_name,
        )

        # Configure pump
        P_RETURN = 2
        P_SEND = 4
        P_CW = "J"
        P_CCW = "K"
        P_MODE_TIME = "N"
        P_SET_RPM = "S"
        P_SET_RUNTIME = "V"
        P_START = "H"
        RPM = 30
        RPM_DT3 = f"{RPM * 100:06d}"  # Discrete Type 3: width=6 0.01RPM
        RUNTIME = 15  # seconds
        RUNTIME_TT2 = f"{RUNTIME * 10:04d}"  # Time Type 2: width=4, 0.1 sec

        control.queue_command("p", "@1", 1, None)
        control.queue_command("p", "1~1", 1, None)
        # Run channel send CCW, fast, 5 seconds
        # Run channel return CW, slowly, 5 seconds
        control.queue_command("p", f"{P_SEND}{P_CCW}", 1, None)
        control.queue_command("p", f"{P_RETURN}{P_CW}", 1, None)
        # Set mode: Time
        control.queue_command("p", f"{P_SEND}{P_MODE_TIME}", 1, None)
        control.queue_command("p", f"{P_RETURN}{P_MODE_TIME}", 1, None)
        # Set RPM mode flow rate setting
        control.queue_command("p", f"{P_SEND}{P_SET_RPM}{RPM_DT3}", 1, None)
        control.queue_command("p", f"{P_RETURN}{P_SET_RPM}{RPM_DT3}", 1, None)
        # Set run time
        control.queue_command("p", f"{P_SEND}{P_SET_RUNTIME}{RUNTIME_TT2}", 1, None)
        control.queue_command("p", f"{P_RETURN}{P_SET_RUNTIME}{RUNTIME_TT2}", 1, None)

        _ = control.execute_queued_commands()

        ## Run pumps
        control.queue_command("p", f"{P_SEND}{P_START}", 0, None)
        control.queue_command(
            "p", f"{P_RETURN}{P_START}", None, RUNTIME + station.DEFAULT_SLEEP_LONG
        )
        _ = control.execute_queued_commands()

        # Disengage head (z stage)
        control.queue_command(
            "z",
            f"<goto {z_premeasure} {z_s_approach}>",
            None,
            z_premeasure_offset / z_s_approach + station.DEFAULT_SLEEP_LONG,
        )
        _ = control.execute_queued_commands()

        ## Reset stages
        control.queue_command(
            "x", f"<goto {x_retract} {x_speed}>", None, time_move_sample_to_measure
        )
        control.queue_command(
            "y", f"<goto {y_retract} {y_speed}>", None, time_move_sample_to_measure
        )
        control.queue_command(
            "z", f"<goto {z_retract} {z_speed}>", None, time_move_sample_to_measure
        )
        _ = control.execute_queued_commands()

        print(f"({s_name}) Successful!")

    finally:
        control.exit_context()
        print(f"({s_name}) exited")
