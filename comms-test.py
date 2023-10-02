import time

import serial

from control_comms import ControlComms, StatusCode, DebugLevel

# Communication settings
SERIAL_PORT = "COM4"    # Check your devices
BAUD_RATE = 115200      # Must match what's in the Arduino code!
CTRL_TIMEOUT = 5.0      # Seconds
DEBUG_LEVEL = DebugLevel.DEBUG_ERROR

# Communication constants
CMD_SET_HOME = 0        # Set current stepper position as home (0 deg)
CMD_MOVE_TO = 1         # Move stepper to a particular position (deg)
CMD_MOVE_BY = 2         # Move stepper by a given amount (deg)
CMD_SET_STEP_MODE = 3   # Set step mode
CMD_SET_BLOCK_MODE = 4  # Set blocking mode
CMD_NOP = 5             # Take no action, just receive observation
STEP_MODE_1 = 0         # 1 division per step
STEP_MODE_2 = 1         # 2 divisions per step
STEP_MODE_4 = 2         # 4 divisions per step
STEP_MODE_8 = 3         # 8 divisions per step
STEP_MODE_16 = 4        # 16 divisions per step
STATUS_OK = 0           # Stepper idle
STATUS_STP_MOVING = 1   # Stepper is currently moving

# # Connect to Arduino board
# controller = ControlComms(timeout=CTRL_TIMEOUT, debug_level=DEBUG_LEVEL)
# ret = controller.connect(SERIAL_PORT, BAUD_RATE)
# if ret is not StatusCode.OK:
#     print("ERROR: Could not connect to board")
    
# # Comms stress test
# counter = 0
# for i in range(100000):
#     resp = controller.step(CMD_MOVE_BY, [counter])
#     print(resp)
#     counter += 1
#     if counter > 100:
#         counter = 0
#     time.sleep(0.001)


ser = serial.Serial(dsrdtr=False)
ser.rts = False
ser.dtr = False

# ser.writeTimeout = 0
ser.timeout = CTRL_TIMEOUT
ser.port = SERIAL_PORT
ser.baudrate = BAUD_RATE

ser.open()

# TEST
print("---SERIAL TEST---")
while True:

    # Transmit
    msg = '{"command": 2, "action": [90]}\n'
    print(f"TX: {msg}")
    ser.write(bytes(msg, encoding='utf-8'))

    # Wait for a response
    msg = ""
    try:
        
        # Test my own implementation
        timestamp = time.time()
        while time.time() - timestamp < CTRL_TIMEOUT:
            print(f"{ser.in_waiting} ", end='')
            c = ser.read(1).decode('ascii')
            print(c, end='')
            if c == '\n':
                break
            msg += c

        if time.time() - timestamp >= CTRL_TIMEOUT:
            print()
            print(f"---TIMEOUT---")
            print()
        
    except Exception as e:
        if self.debug_level >= DEBUG_LEVEL:
            print(f"Error receiving message: {e}")

    print(f"RX: {msg}")