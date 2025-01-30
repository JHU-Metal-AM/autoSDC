Python CLI tool to script and interact with a pump and multiple linear stage controllers over serial for performing SDC corrosion measurements

Developed and tested on Python 3.12

## How to use
- Connect the hardware to your computer
- Identify the serial ports assigned by your system to each device (`COM` ports on Windows, `/dev/ttyXXXY` devices on Mac/Linux) and set each serial port to the correct device in the script
- Run `python autoSDC.py`!