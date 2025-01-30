from dataclasses import dataclass
import serial
import sys
import threading
import time

## Constants
VERSION = "0.2"
ONE_FRAME = 1 / 60  # 0.016666 s
DEFAULT_SLEEP = 0.1  # s
DEFAULT_SLEEP_LONG = 1
LINE_TERMINATION = "\r\n"


@dataclass(frozen=True)
class Msg:
    I_WELCOME = "Welcome to SDC corrosion demo v0.2"
    I_KEYBOARD_INTERRUPT = "KeyboardInterrupt: Exiting..."
    I_CLEAN_EXIT = "Clean exit: Closed all ports"
    I_START = "Enter command: "
    E_DEVICE_NOT_FOUND = "ERROR: One of the serial devices was not found; Check that you've set the correct device identifier for your platform. E.g. COM<x> on Windows, /dev/ttyACM<x> on Mac & Linux"
    W_LISTENER_THREAD_ALVIE = "WARNING: lisener_thread is still alive"
    E_INPUT_NOT_VALID = "ERROR: Input is not valid"
    E_SERIAL_PORT_NOT_REGISTERED = (
        "ERROR: The serial port for this device is not registered"
    )
    E_PORT_NOT_AVAILABLE = "ERROR: Port is labeled active but is either not configured or not currently open."
    E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE = (
        "ERROR: Required devices for script are not active"
    )


## Configure Serial connection

ACTIVE_PORT_KEYS = ["p", "z"]

# X Stage: Ossila
ser_ls_x = None

# Y Stage: Ossila
ser_ls_y = None

# Z Stage: Ossila 100mm
ser_port = serial.Serial()
ser_port.port = "/dev/ttyACM0"
ser_port.baudrate = 9600  # 9600
ser_port.bytesize = serial.EIGHTBITS
ser_port.parity = serial.PARITY_NONE
ser_port.stopbits = serial.STOPBITS_ONE
ser_port.timeout = 1
assert (ser_port.rts is True) and (ser_port.dtr is True)

# Peristaltic pump: Reglo ICC
ser_pump = serial.Serial()
ser_pump.port = "/dev/ttyACM1"
ser_pump.baudrate = 9600  # 9600
ser_pump.bytesize = serial.EIGHTBITS
ser_pump.parity = serial.PARITY_NONE
ser_pump.stopbits = serial.STOPBITS_ONE
ser_pump.timeout = 1
assert (ser_pump.rts is True) and (ser_pump.dtr is True)

SERIAL_PORTS: dict[str, serial.Serial | None] = {
    "p": ser_pump,
    "x": ser_ls_x,
    "y": ser_ls_y,
    "z": ser_port,
}


## Classess


class StoppableThread(threading.Thread):
    def __init__(self, target, *args, **kwargs):
        super().__init__()
        self._target = target
        self._args = args
        self._kwargs = kwargs
        self._stop_event = threading.Event()  # Setting terminates the thread
        self._resume_event = threading.Event()  # Clearing pauses the thread
        self._resume_event.set()  # Initially set to allow execution

    def run(self):
        """Execute the target function until stopped"""
        self._target(self._stop_event, self._resume_event, *self._args, **self._kwargs)

    def stop(self):
        """Signal the thread to stop (terminate)"""
        self._stop_event.set()
        self._resume_event.set()  # Continue if waiting, so it can exit

    def pause(self):
        """Pause the thread."""
        self._resume_event.clear()

    def resume(self):
        """Resume the thread."""
        self._resume_event.set()


## Scripts


def script_demo(serial_ports: dict[str, serial.Serial], active_port_keys, *args):
    if not all([check_port(serial_ports, key) for key in ("p", "z")]):
        print(Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
        return
    try:
        dist = int(args[0])
        speed = 15  # mm/s
        ser_ls_z = serial_ports["z"]
        ser_pump = serial_ports["p"]

        timeout_home = 10
        timeout_move = dist / speed
        s_name = "(demo) "

        send_listen_print(ser_ls_z, "<home>", 2, timeout_home, s_name + "z")
        send_listen_print(ser_pump, "@1", 1, 2, s_name + "p")
        send_listen_print(ser_ls_z, f"<move {dist}>", 2, timeout_move, s_name + "z")
        send_listen_print(ser_pump, "4H", 2, 5, s_name + "p")
        send_listen_print(ser_ls_z, f"<move -{dist}>", 2, timeout_move, s_name + "z")

        print(s_name + "Successful!")

    finally:
        print(s_name + "Exited")


def script_wiggle_ls(serial_ports: dict[str, serial.Serial], active_port_keys, *args):
    try:
        port_code = args[0]
        dist = int(args[1])
        if not check_port(serial_ports, port_code):
            print(Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
            return

        serial_port = serial_ports[port_code]

        timeout = 1.5
        send_listen_print(serial_port, f"<move {dist}>", 2, timeout, "(script) ls")

        send_listen_print(serial_port, f"<move -{dist}>", 2, timeout, "(script) ls")

        time.sleep(DEFAULT_SLEEP_LONG)
        print(f"wiggled {dist}mm !")

    finally:
        print("script_wiggle_ls exited")


def program_exit(*args):
    """Exit the program"""
    sys.exit()


scripts = {
    "demo": script_demo,
    "wiggle_ls": script_wiggle_ls,
    "exit": program_exit,
    "quit": program_exit,
}

## Functions


def send_command(serial_port: serial.Serial, command: str):
    serial_port.write((command + LINE_TERMINATION).encode())
    # print(f"Sent command: {command}")


def check_port(serial_ports: dict[str, serial.Serial], serial_port_code: str):
    """Check that the port is configured and open"""

    port = serial_ports[serial_port_code]
    if port is None:  # Not configured
        return False

    else:  # Configured; Check whether port is open
        return port.is_open


def listen_for(serial_port: serial.Serial, number_of_lines: int, timeout: float):
    # wait until first data byte arrives
    start_time = time.time()

    current_time = time.time()
    lines = []
    while current_time - start_time < timeout and len(lines) < number_of_lines:
        if serial_port.in_waiting > 0:
            data = serial_port.readline().decode().strip()
            lines.append(data)
        time.sleep(ONE_FRAME)
        current_time = time.time()

    return lines


def send_and_listen(
    serial_port: serial.Serial, command: str, number_of_lines: int, timeout: float
):
    send_command(serial_port, command)
    return listen_for(serial_port, number_of_lines, timeout)


def print_lines(prefix: str, lines: list):
    for line in lines:
        print(f"{prefix}: {line}")


def send_listen_print(
    serial_port: serial.Serial,
    command: str,
    number_of_lines: int,
    timeout: float,
    prefix: str,
):
    response_lines = send_and_listen(serial_port, command, number_of_lines, timeout)
    print_lines(prefix, response_lines)
    return response_lines


## Listener thread


def listen_for_data(
    stop_event: threading.Event,
    resume_event: threading.Event,
    all_serial_ports: dict[str, serial.Serial] = None,
    active_port_keys: list[str] = None,
):
    """Reads serial data in a loop until stop_event is set. Pauses when pause_event is cleared."""
    serial_ports = [all_serial_ports[port] for port in active_port_keys]

    while not stop_event.is_set():
        resume_event.wait()  # Pause execution while resuume_event is cleared

        for serial_port, port_key in zip(serial_ports, active_port_keys):
            if serial_port.in_waiting > 0:
                data = serial_port.readline().decode("utf-8").strip()
                print(f"{port_key}: {data}")

        time.sleep(ONE_FRAME)  # Prevent high CPU usage


## Main function


def setup(
    serial_ports: dict[str, serial.Serial],
    active_ports: list[str],
    listener: StoppableThread,
) -> None:
    """Open the serial ports and start listening."""
    try:
        for port_code in active_ports:
            serial_ports[port_code].open()

    except serial.SerialException:
        print(Msg.E_DEVICE_NOT_FOUND)
        raise

    listener.start()


def clean_up(
    serial_ports: dict[str, serial.Serial],
    active_ports: list[str],
    listener: StoppableThread,
) -> None:
    """Clean up serial port connections"""
    listener.stop()
    if listener.is_alive():
        listener.join(timeout=5)
        if listener.is_alive():
            print(Msg.W_LISTENER_THREAD_ALVIE)

    for port_code in active_ports:
        serial_ports[port_code].close


def main():
    try:
        print(Msg.I_WELCOME)
        # serial_ports = [SERIAL_PORTS[port_code] for port_code in ACTIVE_PORTS]
        listener_thread = StoppableThread(
            target=listen_for_data,
            all_serial_ports=SERIAL_PORTS,
            active_port_keys=ACTIVE_PORT_KEYS,
        )
        setup(SERIAL_PORTS, ACTIVE_PORT_KEYS, listener_thread)
        print(Msg.I_START)

        while True:
            user_input = input()

            # Invalid input
            if len(user_input) < 1:
                print(Msg.E_INPUT_NOT_VALID)

            user_input = user_input.strip()  # .split(' ')
            input_code = user_input[0].lower()

            # If input_code is a valid active port: Send commands directly
            if input_code in ACTIVE_PORT_KEYS:
                # Check that the port is open
                if check_port(SERIAL_PORTS, input_code):
                    ser_port = SERIAL_PORTS[input_code]
                    send_command(ser_port, user_input[2:])
                else:
                    raise LookupError(Msg.E_PORT_NOT_AVAILABLE)

            # If input_code is valid but port is not configured: Inform user
            elif input_code in SERIAL_PORTS:
                print(Msg.E_SERIAL_PORT_NOT_REGISTERED)

            # Otherwise, input must either be a script or invalid
            else:
                user_input_list = user_input.split(" ")
                keyword = user_input_list[0]
                arguments = user_input_list[1:]

                if keyword in scripts:
                    try:
                        listener_thread.pause()
                        scripts[keyword](SERIAL_PORTS, ACTIVE_PORT_KEYS, *arguments)

                    finally:
                        listener_thread.resume()

                else:
                    print(Msg.E_INPUT_NOT_VALID)

            time.sleep(DEFAULT_SLEEP)

    except KeyboardInterrupt:
        print("")
        program_exit()

    finally:
        clean_up(SERIAL_PORTS, ACTIVE_PORT_KEYS, listener_thread)
        print(Msg.I_CLEAN_EXIT)


if __name__ == "__main__":
    main()
