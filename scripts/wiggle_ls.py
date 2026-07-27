from __future__ import annotations

from typing import TYPE_CHECKING

import serial

from scripts.common import get_station_module

if TYPE_CHECKING:
    from SDCStation import SerialController


def script_wiggle_ls(
    serial_ports: dict[str, serial.Serial],
    active_port_keys: list[str],
    serial_controller: SerialController,
    *args,
):
    """Move the linear stage back and forth"""
    _ = active_port_keys
    _ = serial_controller

    station = get_station_module()
    s_name = "(wiggle_ls) "
    try:
        port_code = args[0]
        dist = int(args[1])
        if not station.check_port(serial_ports, port_code):
            print(station.Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
            return

        prefix = s_name + port_code
        serial_port = serial_ports[port_code]

        timeout = dist / station.LS_SPEED_DEFAULT

        station.send_and_listen(serial_port, f"<move {dist}>", 2, timeout, prefix)
        station.send_and_listen(serial_port, f"<move -{dist}>", 2, timeout, prefix)

        print(s_name + f"Wiggled {port_code} {dist}mm !")

    finally:
        print(s_name + "Exited")
