# basedpyright: recommended
import queue
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from queue import SimpleQueue
from typing import Concatenate, ParamSpec, final, override

import numpy as np
import serial

P = ParamSpec("P")  # Represents the parameter types of a function

## Constants
VERSION = "0.3"
ONE_FRAME = 1 / 600  # 0.0016666 s
DEFAULT_SLEEP = 0.1  # s
DEFAULT_SLEEP_LONG = 1
LINE_TERMINATION = "\r\n"
LS_SPEED_DEFAULT = 15  # mm/s Default max speed of linear stage
INF_INT = 999999


@final
@dataclass(frozen=True)
class Msg:
    I_WELCOME = f"Welcome to SDC corrosion demo v{VERSION}"
    I_KEYBOARD_INTERRUPT = "KeyboardInterrupt: Exiting..."
    I_CLEAN_EXIT = "Clean exit: Closed all ports"
    I_START = "Enter command: "
    E_DEVICE_NOT_FOUND = "ERROR: One of the serial devices was not found; Check that you've set the correct device identifier for your platform. E.g. COM<x> on Windows, /dev/ttyACM<x> on Mac & Linux"
    W_LISTENER_THREAD_ALIVE = "WARNING: listener_thread is still alive"
    W_SERIAL_WORKERS_ALIVE = "WARNING: some serial workers are still alive"
    E_INPUT_NOT_VALID = "ERROR: Input is not valid"
    E_SERIAL_PORT_NOT_REGISTERED = (
        "ERROR: The serial port for this device is not registered"
    )
    E_PORT_NOT_AVAILABLE = "ERROR: Port is labeled active but is either not configured or not currently open."
    E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE = (
        "ERROR: Required devices for script are not active"
    )


## Configure Serial connection

# Tell the script which ports to activate and use
ACTIVE_PORT_KEYS = [
    "x",
    "y",
    "z",
    "p",
]

# X Stage: Ossila 200mm
ser_ls_x = serial.Serial()
ser_ls_x.port = "/dev/ttyACM0"
ser_ls_x.baudrate = 9600  # 9600
ser_ls_x.bytesize = serial.EIGHTBITS
ser_ls_x.parity = serial.PARITY_NONE
ser_ls_x.stopbits = serial.STOPBITS_ONE
ser_ls_x.timeout = 1
assert (ser_ls_x.rts is True) and (ser_ls_x.dtr is True)

# Y Stage: Ossila 200mm
ser_ls_y = serial.Serial()
ser_ls_y.port = "/dev/ttyACM1"
ser_ls_y.baudrate = 9600  # 9600
ser_ls_y.bytesize = serial.EIGHTBITS
ser_ls_y.parity = serial.PARITY_NONE
ser_ls_y.stopbits = serial.STOPBITS_ONE
ser_ls_y.timeout = 1
assert (ser_ls_y.rts is True) and (ser_ls_y.dtr is True)

# Z Stage: Ossila 100mm
ser_ls_z = serial.Serial()
ser_ls_z.port = "/dev/ttyACM2"
ser_ls_z.baudrate = 9600  # 9600
ser_ls_z.bytesize = serial.EIGHTBITS
ser_ls_z.parity = serial.PARITY_NONE
ser_ls_z.stopbits = serial.STOPBITS_ONE
ser_ls_z.timeout = 1
assert (ser_ls_z.rts is True) and (ser_ls_z.dtr is True)

# Peristaltic pump: Reglo ICC
ser_pump = serial.Serial()
ser_pump.port = "/dev/ttyACM3"
ser_pump.baudrate = 9600  # 9600
ser_pump.bytesize = serial.EIGHTBITS
ser_pump.parity = serial.PARITY_NONE
ser_pump.stopbits = serial.STOPBITS_ONE
ser_pump.timeout = 1
assert (ser_pump.rts is True) and (ser_pump.dtr is True)

SERIAL_PORTS: dict[str, serial.Serial] = {
    "p": ser_pump,
    "x": ser_ls_x,
    "y": ser_ls_y,
    "z": ser_ls_z,
}


## Classess


@final
class StoppableThread(threading.Thread):
    def __init__(
        self,
        target: Callable[Concatenate[threading.Event, threading.Event, P], None],
        *args: P.args,
        **kwargs: P.kwargs,
    ):
        """Thread that checks for a stop and a resume event.

        Terminates on stop and toggles on pause/resume.
        """

        super().__init__()
        self._target: Callable[..., None] = target
        self._args = args
        self._kwargs = kwargs
        self._stop_event = threading.Event()  # Setting terminates the thread
        self._resume_event = threading.Event()  # Clearing pauses the thread
        self._resume_event.set()  # Initially set to allow execution

    @override
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


@final
class SerialWorker(threading.Thread):
    def __init__(
        self,
        device_name: str,
        serial_port: serial.Serial,
        line_termination: str = LINE_TERMINATION,
        response_check_period: float = ONE_FRAME,
        print_old_responses: bool = True,
        # *args,
        # **kwargs,
    ):
        super().__init__()
        self.device_name = device_name
        self.serial_port = serial_port

        self.response_queue: SimpleQueue[list[str]] = SimpleQueue()
        self.command_queue: SimpleQueue[
            tuple[str, int | None, float | None, str, bool]
        ] = SimpleQueue()

        # Options
        self.line_termination = line_termination
        self.print_old_responses = print_old_responses
        self.response_check_period = response_check_period

        self._stop_event = threading.Event()
        self._command_event = threading.Event()

    def queue_command(
        self,
        command: str,
        num_lines: int | None,
        timeout: float | None,
        context: str = "",
        print_response: bool = True,
    ):
        """Prepare a command to be sent"""
        self.command_queue.put((command, num_lines, timeout, context, print_response))

    def process_command_queue(self):
        """Notify the thread to start processing commands from queue"""
        self._command_event.set()

    @override
    def run(self):
        """Thread execution loop"""
        while not self._stop_event.is_set():
            _ = self._command_event.wait()  # Wait until a command is available
            if self._stop_event.is_set():
                break

            self._process_command()

            if self.command_queue.empty():  # if all commands have been processed
                self._command_event.clear()  # Reset the event

    def _process_command(self):
        # Retreieve queued command and parameters
        try:
            command, num_lines, timeout, context, print_response = (
                self.command_queue.get_nowait()
            )
        except queue.Empty:
            print(
                f"W: {self.device_name} SerialWorker entered _process_command but command_queue is empty"
            )
            return

        # Clear queued responses
        old_responses: list[str] = []
        while self.serial_port.in_waiting > 0:
            old_responses.append(self.serial_port.readline().decode().strip())

        if self.print_old_responses and len(old_responses) > 0:
            print(f"({context}) {self.device_name}: *** BEGIN OLD RESPONSES")
            for response in old_responses:
                print(f"({context}) {self.device_name}: {response}")
            print(f"({context}) {self.device_name}: *** END OLD RESPONSES")

        # Send command and get response back
        lines: list[str] = send_and_listen(
            self.serial_port,
            command,
            num_lines,
            timeout,
            prefix="(" + context + ") " + self.device_name,
            print_lines=print_response,
            read_period=self.response_check_period,
        )

        # Put set of lines all at once when listener returns
        self.response_queue.put(lines)

    def stop(self):
        """Stops the thread gracefully"""
        self._stop_event.set()
        self._command_event.set()  # Unblock the thread if waiting
        # self.serial_port.close() # Don't close here, might be used elsewhere


@final
class SerialController:
    def __init__(
        self,
        serial_ports: dict[str, serial.Serial],
        response_check_period: float = ONE_FRAME,
    ):
        # self.workers = SerialWorker()
        # self.serial_ports = serial_ports
        self.workers = {
            device_name: SerialWorker(
                device_name,
                serial_port,
                line_termination=LINE_TERMINATION,
                response_check_period=response_check_period,
                print_old_responses=True,
            )
            for device_name, serial_port in serial_ports.items()
        }
        self.response_check_period = response_check_period
        self.num_expected_responses = 0

        self.context: str | None = None

    def start(self):
        """Start all worker threads"""
        for worker in self.workers.values():
            worker.start()

    # def send_commands(self, command: str, num_lines: int, timeout: float):
    #     """Send the same command to all devices and wait """
    def queue_command(
        self,
        device_name: str,
        command: str,
        num_lines: int | None,
        timeout: float | None,
        context: str | None = None,
        print_response: bool = True,
    ):
        """Prepare a command to be sent to a SerialWorker"""
        if context is None:
            if self.context is None:
                raise ValueError("Context is None, and SerialController has context")
            context = self.context

        if device_name in self.workers:
            # TODO: This is perhaps redundant. Can simply put onto the queue directly
            self.workers[device_name].queue_command(
                command,
                num_lines,
                timeout,
                context=context,
                print_response=print_response,
            )
            self.num_expected_responses += 1
        else:
            raise ValueError(
                f"Supplied device name '{device_name}' does not map to any managed serial device"
            )

    def execute_queued_commands(self) -> dict[str, list[list[str]]]:
        """Signals all workers to start processing their respective command queues"""
        for worker in self.workers.values():
            if worker.command_queue.qsize() > 0:
                worker.process_command_queue()

        # while sum of length of response queues is less than expected: wait
        while (
            sum([worker.response_queue.qsize() for worker in self.workers.values()])
            < self.num_expected_responses
        ):
            time.sleep(self.response_check_period)

        self.num_expected_responses = 0

        responses: dict[str, list[list[str]]] = {}
        for worker_name, worker in self.workers.items():
            worker_responses: list[list[str]] = []
            while worker.response_queue.qsize() > 0:
                worker_response: list[str] = worker.response_queue.get_nowait()
                worker_responses.append(worker_response)
            responses[worker_name] = worker_responses

        return responses

    def enter_context(self, context: str):
        """set up new context"""
        self.context = context

    def exit_context(self):
        """Uninitializes context"""
        self.context = None

    def stop_workers(self) -> None:
        """Stops all worker threads"""
        for worker in self.workers.values():
            worker.stop()

    def join_workers(self, timeout: float | None = None) -> None:
        """Join all workers (wait for thread to terminate) one at a time"""
        for worker in self.workers.values():
            worker.join(timeout=timeout)

    def is_any_worker_alive(self):
        """Check whether any worker is still alive"""
        for worker in self.workers.values():
            if worker.is_alive():
                return True
        return False


## Scripts


def script_demo2_refactored(
    serial_ports: dict[str, serial.Serial],
    active_port_keys: list[str],
    control: SerialController,
    *_,
):
    """Demo2 using SerialController"""
    s_name = "demo2"
    try:
        if not check_ports(serial_ports, ["x", "y", "z", "p"]):
            print(Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
            return

        control.enter_context(s_name)

        timeout_home = 18

        x_retract = 0
        y_retract = 0
        z_retract = 0  # or 100?

        x_measure = 200
        y_measure = 95
        z_measure = 60
        z_premeasure_offset = 15
        z_premeasure = z_measure - z_premeasure_offset

        # z_s_approach = 5

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
            x_retract / LS_SPEED_DEFAULT + DEFAULT_SLEEP,
        )
        control.queue_command(
            "y",
            f"<goto {y_retract}>",
            None,
            y_retract / LS_SPEED_DEFAULT + DEFAULT_SLEEP,
        )
        control.queue_command(
            "z",
            f"<goto {z_retract}>",
            None,
            z_retract / LS_SPEED_DEFAULT + DEFAULT_SLEEP,
        )
        # time.sleep(DEFAULT_SLEEP_LONG)
        _ = control.execute_queued_commands()

        ## Move sample into position and configure pumps
        # Synchronize by changing speeds to arrive at the same time
        x_move_dist = abs(x_measure - x_retract)
        y_move_dist = abs(y_measure - y_retract)
        z_move_dist = abs(z_premeasure - z_retract)
        time_move_sample_to_measure = (
            max(x_move_dist, y_move_dist, z_move_dist) / LS_SPEED_DEFAULT
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
            z_premeasure_offset / z_s_approach + DEFAULT_SLEEP_LONG,
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
            "p", f"{P_RETURN}{P_START}", None, RUNTIME + DEFAULT_SLEEP_LONG
        )
        _ = control.execute_queued_commands()

        # Disenage head (z stage)
        control.queue_command(
            "z",
            f"<goto {z_premeasure} {z_s_approach}>",
            None,
            z_premeasure_offset / z_s_approach + DEFAULT_SLEEP_LONG,
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
    # try:
    #     if not all([check_])


def script_demo2(
    serial_ports: dict[str, serial.Serial],
    active_port_keys: list[str],
    serial_controller,
    *_,
):
    """Demo 2025-02-20: Move linear stage, run pump, move stage back"""
    s_name = "(demo2) "
    try:
        if not check_ports(serial_ports, ["x", "y", "z", "p"]):
            print(Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
            return

        ser_ls_x = serial_ports["x"]
        ser_ls_y = serial_ports["y"]
        ser_ls_z = serial_ports["z"]
        ser_pump = serial_ports["p"]

        timeout_home = 18

        x_retract = 0
        y_retract = 0
        z_retract = 0  # or 100?

        x_measure = 200
        y_measure = 85
        z_measure = 50

        z_s_approach = 5

        ## "Home" the stages
        send_command(ser_ls_x, "<home>")
        send_command(ser_ls_y, "<home>")
        send_command(ser_ls_z, "<home>")

        time.sleep(timeout_home)

        ## Move stages into retracted position
        send_command(ser_ls_x, f"<goto {x_retract}>")
        send_command(ser_ls_y, f"<goto {y_retract}>")
        send_command(ser_ls_z, f"<goto {z_retract}>")

        # time.sleep(z_retract / LS_SPEED_DEFAULT)
        time.sleep(1)

        ## Move sample into position
        send_command(ser_ls_x, f"<goto {x_measure}>")
        send_command(ser_ls_y, f"<goto {y_measure}>")

        time.sleep(max(x_measure, y_measure) / LS_SPEED_DEFAULT)

        # Move z into position
        send_command(ser_ls_z, f"<goto {z_measure} {z_s_approach}>")

        time.sleep(abs(z_measure - z_retract) / z_s_approach)

        P_RETURN = 1
        P_SEND = 3
        P_CW = "J"
        P_CCW = "K"
        P_MODE_TIME = "N"
        P_SET_RPM = "S"
        P_SET_RUNTIME = "V"
        P_START = "H"

        # def RPM_DT3(rpm: float):

        ## Prep pumps
        # Run channel 2 CCW, fast, 5 seconds
        # Run channel 4 CW, slowly, 5 seconds
        send_and_listen(
            ser_pump, "@1", 1, 2, s_name + "p"
        )  # Assign address 1 to the pump
        send_and_listen(
            ser_pump, "1~1", 1, 2, s_name + "p"
        )  # Configure independent channel control, pump 1

        # Set run direction: J = CW, K = CCW
        send_and_listen(ser_pump, f"{P_RETURN}{P_CCW}", 1, 2, s_name + "p")  # CCW
        send_and_listen(ser_pump, f"{P_SEND}{P_CW}", 1, 2, s_name + "p")  # CW

        # Set mode: Time
        send_and_listen(
            ser_pump, f"{P_RETURN}{P_MODE_TIME}", 1, 2, s_name + "p"
        )  # Time mode
        send_and_listen(
            ser_pump, f"{P_SEND}{P_MODE_TIME}", 1, 2, s_name + "p"
        )  # Time mode

        # Set RPM outside of RPM mode: 6, 0.01RPM
        send_and_listen(
            ser_pump, f"{P_RETURN}{P_SET_RPM}006000", 1, 2, s_name + "p"
        )  # RPM
        send_and_listen(
            ser_pump, f"{P_SEND}{P_SET_RPM}000600", 1, 2, s_name + "p"
        )  # RPM

        # Set run time: Time Type 1: 1-8, 0.1 sec
        send_and_listen(
            ser_pump, f"{P_RETURN}{P_SET_RUNTIME}0050", 1, 2, s_name + "p"
        )  # RPM
        send_and_listen(
            ser_pump, f"{P_SEND}{P_SET_RUNTIME}0050", 1, 2, s_name + "p"
        )  # RPM

        ## Run pumps
        send_command(ser_pump, f"{P_RETURN}{P_START}")
        send_and_listen(ser_pump, f"{P_SEND}{P_START}", 2, 5, s_name + "p")

        ## Reset stages
        send_command(ser_ls_z, f"<goto {z_retract} {z_s_approach}>")
        send_command(ser_ls_x, f"<goto {x_retract}>")
        send_command(ser_ls_y, f"<goto {y_retract}>")

        time.sleep(
            max(
                abs(z_measure - z_retract) / z_s_approach,
                abs(x_measure - x_retract) / LS_SPEED_DEFAULT,
                abs(y_measure - y_retract) / LS_SPEED_DEFAULT,
            )
        )

        print(s_name + "Successful!")

    finally:
        print(s_name + "exited")


def script_demo(
    serial_ports: dict[str, serial.Serial],
    active_port_keys: list[str],
    serial_controller: SerialController,
    *args,
):
    """Demo 2025-02-03: Move linear stage, run pump, move stage back"""

    s_name = "(demo) "
    try:
        if not all([check_port(serial_ports, key) for key in ("p", "z")]):
            print(Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
            return

        dist = int(args[0])
        ser_ls_z = serial_ports["z"]
        ser_pump = serial_ports["p"]

        timeout_home = 10
        timeout_move = dist / LS_SPEED_DEFAULT

        send_and_listen(ser_ls_z, "<home>", 2, timeout_home, s_name + "z")
        send_and_listen(ser_pump, "@1", 1, 2, s_name + "p")
        send_and_listen(ser_ls_z, f"<move {dist}>", 2, timeout_move, s_name + "z")
        send_and_listen(ser_pump, "4H", 2, 5, s_name + "p")
        send_and_listen(ser_ls_z, f"<move -{dist}>", 2, timeout_move, s_name + "z")

        print(s_name + "Successful!")

    finally:
        print(s_name + "Exited")


def script_wiggle_ls(
    serial_ports: dict[str, serial.Serial],
    active_port_keys: list[str],
    serial_controller: SerialController,
    *args,
):
    """Move the linear stage back and forth"""
    s_name = "(wiggle_ls) "
    try:
        port_code = args[0]
        dist = int(args[1])
        if not check_port(serial_ports, port_code):
            print(Msg.E_SCRIPT_REQUIRED_PORT_NOT_ACTIVE)
            return

        prefix = s_name + port_code
        serial_port = serial_ports[port_code]

        timeout = dist / LS_SPEED_DEFAULT

        send_and_listen(serial_port, f"<move {dist}>", 2, timeout, prefix)
        send_and_listen(serial_port, f"<move -{dist}>", 2, timeout, prefix)

        print(s_name + f"Wiggled {port_code} {dist}mm !")

    finally:
        print(s_name + "Exited")


def program_exit(*_):
    """Exit the program"""
    sys.exit()


scripts: dict[str, Callable[..., None]] = {
    "demo": script_demo,
    "demo2": script_demo2_refactored,
    "wiggle_ls": script_wiggle_ls,
    "exit": program_exit,
    "quit": program_exit,
}

## Functions


def check_port(serial_ports: dict[str, serial.Serial], serial_port_code: str):
    """Check that the port is configured and open"""

    port = serial_ports[serial_port_code]
    # if port is None:  # Not configured: This should not be possible anymore. All serial ports are defined
    #     return False
    # else:  # Configured; Check whether port is open
    return port.is_open


def check_ports(serial_ports: dict[str, serial.Serial], serial_port_codes: list[str]):
    """Check that all the port are configured and open"""
    return all([check_port(serial_ports, key) for key in serial_port_codes])


def send_command(serial_port: serial.Serial, command: str):
    """Encode and send the given string over a serial port"""
    _ = serial_port.write((command + LINE_TERMINATION).encode())
    # print(f"Sent command: {command}")


def listen_for(
    serial_port: serial.Serial,
    number_of_lines: int | None,
    timeout: float | None,
    print_lines: bool = True,
    prefix: str = "",
    read_period: float = ONE_FRAME,
) -> list[str]:
    """Read a specified number of lines with a timeout from serial"""
    if number_of_lines is None:
        number_of_lines = INF_INT
    if timeout is None:
        timeout = np.inf

    start_time = time.time()

    lines: list[str] = []
    while time.time() - start_time < timeout and len(lines) < number_of_lines:
        if serial_port.in_waiting > 0:
            data = serial_port.readline().decode().strip()
            lines.append(data)
            if print_lines:
                print(f"{prefix}: {data}")
        time.sleep(read_period)

    return lines


def send_and_listen(
    serial_port: serial.Serial,
    command: str,
    number_of_lines: int | None,
    timeout: float | None,
    prefix: str = "",
    print_lines: bool = True,
    read_period: float = ONE_FRAME,
) -> list[str]:
    """Send command over serial and read a specified number of lines from serial"""
    send_command(serial_port, command)
    return listen_for(
        serial_port,
        number_of_lines,
        timeout,
        print_lines=print_lines,
        prefix=prefix,
        read_period=read_period,
    )


## Listener thread


def listen_for_data(
    stop_event: threading.Event,
    resume_event: threading.Event,
    all_serial_ports: dict[str, serial.Serial],
    active_port_keys: list[str],
    response_check_period: float = ONE_FRAME,
):
    """Reads serial data in a loop until stop_event is set. Pauses when pause_event is cleared."""
    serial_ports = [all_serial_ports[port] for port in active_port_keys]

    while not stop_event.is_set():
        _ = resume_event.wait()  # Pause execution while resuume_event is cleared

        for serial_port, port_key in zip(serial_ports, active_port_keys):
            if serial_port.in_waiting > 0:
                data = serial_port.readline().decode("utf-8").strip()
                print(f"{port_key}: {data}")

        time.sleep(response_check_period)  # Prevent high CPU usage


## Main function


def setup() -> tuple[StoppableThread, SerialController]:
    """Open the serial ports and start listening."""

    # Open ports
    try:
        for port_code in ACTIVE_PORT_KEYS:
            SERIAL_PORTS[port_code].open()

    except serial.SerialException:
        print(Msg.E_DEVICE_NOT_FOUND)
        raise

    # start listener thread
    listener_thread = StoppableThread(
        target=listen_for_data,
        all_serial_ports=SERIAL_PORTS,
        active_port_keys=ACTIVE_PORT_KEYS,
        response_check_period=ONE_FRAME,
    )

    listener_thread.start()

    # Initialize SerialController
    serial_controller = SerialController(
        {device_name: SERIAL_PORTS[device_name] for device_name in ACTIVE_PORT_KEYS},
        response_check_period=ONE_FRAME,
    )
    serial_controller.start()

    return listener_thread, serial_controller


def clean_up(
    serial_ports: dict[str, serial.Serial],
    active_ports: list[str],
    listener: StoppableThread,
    serial_controller: SerialController,
) -> None:
    """Clean up serial port connections"""

    # Stop listener thread
    listener.stop()
    listener.join(timeout=5)
    if listener.is_alive():
        print(Msg.W_LISTENER_THREAD_ALIVE)

    # Stop SerialController and Serial Workers
    serial_controller.stop_workers()
    serial_controller.join_workers(timeout=5)
    if serial_controller.is_any_worker_alive():
        print(Msg.W_SERIAL_WORKERS_ALIVE)

    # Close serial ports
    for port_code in active_ports:
        serial_ports[port_code].close


def main():
    try:
        print(Msg.I_WELCOME)
        # setup(SERIAL_PORTS, ACTIVE_PORT_KEYS, listener_thread)
        listener_thread, serial_controller = setup()

        print(Msg.I_START)

        while True:
            user_input = input()

            # preprocess input
            user_input = user_input.strip()

            # Invalid input
            if len(user_input) < 2:
                print(Msg.E_INPUT_NOT_VALID)
                continue

            # get input code
            input_code = user_input[0].lower()

            # If input_code is a valid active port: Send commands directly
            if input_code in SERIAL_PORTS and user_input[1] == ":":
                user_input_list = user_input.split(";")

                try:
                    for p_user_input in user_input_list:
                        # Check that all ports are active
                        if not check_port(SERIAL_PORTS, p_user_input[0].lower()):
                            print(Msg.E_PORT_NOT_AVAILABLE)
                            raise LookupError

                        # Check that each command is formatted correctly
                        if p_user_input[1] != ":":
                            print(Msg.E_INPUT_NOT_VALID)
                            raise ValueError

                except (ValueError, LookupError) as _:
                    # Return to prompt
                    continue

                # If no issues, send all commands
                for p_user_input in user_input_list:
                    p_input_code = p_user_input[0].lower()
                    ser_port = SERIAL_PORTS[p_input_code]
                    send_command(ser_port, p_user_input[2:])

            # If input_code is valid but port is not configured: Inform user
            # elif input_code in SERIAL_PORTS and user_input[1] == ":":
            #     print(Msg.E_SERIAL_PORT_NOT_REGISTERED)

            # Otherwise, input must either be a script or invalid
            else:
                user_input_list = user_input.split(" ")
                keyword = user_input_list[0]
                arguments = user_input_list[1:]

                if keyword in scripts:
                    try:
                        listener_thread.pause()
                        scripts[keyword](
                            SERIAL_PORTS,
                            ACTIVE_PORT_KEYS,
                            serial_controller,
                            *arguments,
                        )

                    finally:
                        listener_thread.resume()

                else:
                    print(Msg.E_INPUT_NOT_VALID)

            time.sleep(DEFAULT_SLEEP)

    except KeyboardInterrupt:
        print("")
        program_exit()

    finally:
        clean_up(SERIAL_PORTS, ACTIVE_PORT_KEYS, listener_thread, serial_controller)
        print(Msg.I_CLEAN_EXIT)


if __name__ == "__main__":
    main()
