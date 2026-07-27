from __future__ import annotations

from typing import TYPE_CHECKING

import serial

from scripts.common import get_station_module

if TYPE_CHECKING:
    from SDCStation import SerialController


def script_demo(
    serial_ports: dict[str, serial.Serial],
    active_port_keys: list[str],
    serial_controller: SerialController,
    *args,
):
    """Demo 2025-02-03: Move linear stage, run pump, move stage back"""
    _ = active_port_keys
    _ = serial_controller

    station = get_station_module()
    s_name = "(demo) "
    try:
        if not all([station.check_port(serial_ports, key) for key in ("p", "z")]):
            print(station.Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
            return

        dist = int(args[0])
        ser_ls_z = serial_ports["z"]
        ser_pump = serial_ports["p"]

        timeout_home = 10
        timeout_move = dist / station.LS_SPEED_DEFAULT

        station.send_and_listen(ser_ls_z, "<home>", 2, timeout_home, s_name + "z")
        station.send_and_listen(ser_pump, "@1", 1, 2, s_name + "p")
        station.send_and_listen(ser_ls_z, f"<move {dist}>", 2, timeout_move, s_name + "z")
        station.send_and_listen(ser_pump, "4H", 2, 5, s_name + "p")
        station.send_and_listen(ser_ls_z, f"<move -{dist}>", 2, timeout_move, s_name + "z")

        print(s_name + "Successful!")

    finally:
        print(s_name + "Exited")
