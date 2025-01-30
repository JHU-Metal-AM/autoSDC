# from concurrent.futures import thread
from dataclasses import dataclass
import serial
import threading
import time

## Constants
ONE_FRAME = 1 / 60 # 0.016666 s
DEFAULT_SLEEP = 0.1 # s
DEFAULT_SLEEP_LONG = 1

@dataclass(frozen=True)
class Msg:
    I_WELCOME = "Welcome to SDC corrosion demo v0.1"
    I_KEYBOARD_INTERRUPT = "KeyboardInterrupt: Exiting..."
    I_CLEAN_EXIT = "Clean exit: Closed all ports"
    E_DEVICE_NOT_FOUND = "ERROR: One of the serial devices was not found; Check that you've set the correct device identifier for your platform. E.g. COM<x> on Windows, /dev/ttyACM<x> on Mac & Linux"
    W_LISTENER_THREAD_ALVIE = "WARNING: lisener_thread is still alive"
    E_INPUT_NOT_VALID = "ERROR: Input is not valid"
    E_SERIAL_PORT_NOT_REGISTERED = "ERROR: The serial port for this device is not registered"

## Configure Serial connection

ser_ls_z = serial.Serial()
ser_ls_z.port = "/dev/ttyACM0"
ser_ls_z.baudrate = 9600  # 9600
ser_ls_z.bytesize = serial.EIGHTBITS
ser_ls_z.parity = serial.PARITY_NONE
ser_ls_z.stopbits = serial.STOPBITS_ONE
ser_ls_z.timeout = 1
assert (ser_ls_z.rts is True) and (ser_ls_z.dtr is True)

ser_pump = serial.Serial()
ser_pump.port = "/dev/ttyACM1"
ser_pump.baudrate = 9600  # 9600
ser_pump.bytesize = serial.EIGHTBITS
ser_pump.parity = serial.PARITY_NONE
ser_pump.stopbits = serial.STOPBITS_ONE
ser_pump.timeout = 1
assert (ser_pump.rts is True) and (ser_pump.dtr is True)

serial_ports : dict[str, serial.Serial | None ] = {
    'P:' : ser_pump,
    'X:' : None,
    'Y:' : None,
    'Z:' : ser_ls_z
}
## Classess

class StoppableThread(threading.Thread):
    def __init__(self, target, *args, **kwargs):
        super().__init__()
        self._target = target
        self._args = args
        self._kwargs = kwargs
        self._stop_event = threading.Event()  # Setting terminates the thread
        self._resume_event = threading.Event() # Clearing pauses the thread
        self._resume_event.set() # Initially set to allow execution

    def run(self):
        """Execute the target function until stopped"""
        self._target(self._stop_event, self._resume_event, *self._args, **self._kwargs)

    def stop(self):
        """Signal the thread to stop (terminate)"""
        self._stop_event.set()
        self._resume_event.set() # Continue if waiting, so it can exit
        
    def pause(self):
        """Pause the thread."""
        self._resume_event.clear()
        
    def resume(self):
        """Resume the thread."""
        self._resume_event.set()

## Scripts

def script_demo(*args, stop_event: threading.Event = None):
    print("Not Implemented!")

def script_wiggle_ls(serial_ports: dict[str, serial.Serial], *args, thread=StoppableThread):
    try:
        thread.pause()
        port_code = args[0]
        dist = int(args[1])
        serial_port = serial_ports[port_code]

        send_command(serial_port, f'<move {dist}>')

        while serial_port.in_waiting < 1:
            time.sleep(DEFAULT_SLEEP)
        data = ser_ls_z.read_until
        for line in serial_port.readlines():
            print(f"(script) Received: {line.decode()}")
        # time.sleep(DEFAULT_SLEEP_LONG)
        
        send_command(serial_port, f'<move -{dist}>')

        # while serial_port.in_waiting < 1:
        #     time.sleep(DEFAULT_SLEEP)

        # for line in serial_port.readlines():
        #     print(f"(script) Received: {line.decode()}")
        
        time.sleep(DEFAULT_SLEEP_LONG)
        print(f'wiggled {dist}mm !')

    finally:
        thread.resume()

scripts = {'demo': script_demo,
           'wiggle_ls': script_wiggle_ls}

## Functions

def send_command(serial_port, command):
    serial_port.write(command.encode())
    print(f"Sent command: {command}")


def check_port(serial_ports : dict[str, serial.Serial], serial_port_code : str):
    port = serial_ports[serial_port_code]
    if port is None:
        return False
    else:
        return port.is_open
    
## Listener thread

def listen_for_data(stop_event: threading.Event, resume_event: threading.Event): 
    """Reads serial data in a loop until stop_event is set. Pauses when pause_event is cleared."""
    while not stop_event.is_set():
        resume_event.wait() # Pause execution while resuume_event is cleared

        if ser_ls_z.in_waiting > 0:
            data = ser_ls_z.readline().decode('utf-8')  #.strip()
            print(f"Received: {data}")

        time.sleep(DEFAULT_SLEEP)  # Prevent high CPU usage

# listener_stop_event = threading.Event()
# listener_thread = threading.Thread(target=listen_for_data, args=(listener_stop_event,))

listener_thread = StoppableThread(target=listen_for_data)

## Main function

def main():
    try:
        print(Msg.I_WELCOME)

        try:
            ser_ls_z.open()
            ser_pump.open()

        except serial.SerialException:
            print(Msg.E_DEVICE_NOT_FOUND)
            raise

        listener_thread.start()

        while True:
            user_input = input("Enter command: ")
            
            # Invalid input
            if len(user_input) < 2:
                print(Msg.E_INPUT_NOT_VALID)

            # Check for port codes
            elif (port_code:=user_input[0:2]) in serial_ports:
                if check_port(serial_ports, port_code):
                    ser_port = serial_ports[port_code]
                    send_command(ser_port, user_input[2:])
                else:
                    print(Msg.E_SERIAL_PORT_NOT_REGISTERED)

            else:
                user_input_list = user_input.split(' ')
                keyword = user_input_list[0]
                arguments = user_input_list[1:]

                if keyword in scripts:
                    scripts[keyword](serial_ports, *arguments, threads=[listener_thread])

                else:
                    print(Msg.E_INPUT_NOT_VALID)

            time.sleep(DEFAULT_SLEEP)

    except KeyboardInterrupt:
        print(Msg.I_KEYBOARD_INTERRUPT)
    
    finally:
        # listener_stop_event.set()
        listener_thread.stop()
        if listener_thread.is_alive():
            listener_thread.join(timeout=5)
            if listener_thread.is_alive():
                print(Msg.W_LISTENER_THREAD_ALVIE)

        ser_ls_z.close()
        ser_pump.close()
        print(Msg.I_CLEAN_EXIT)

        

if __name__ == '__main__':
    main()