import os
import re
import time
import serial
from enum import Enum, auto
# import rospy

import ras_low_level_bridge.global_state as globs
from ras_low_level_bridge.global_state import GlobalState


class MsgReceiverState(Enum):
    receiving = auto()
    idle = auto()


class MsgReceiver:
    """
    Class to collect serial data. obj.run returns the message string if one has successfully been received.
    """
    timeout = 25E6  # 25 ms
    char_timeout = 10E6  # 10 ms

    def __init__(self, serial: serial.Serial):
        self._serial = serial
        self.received_string = ""
        self.receive_start = 0
        self.last_received = 0
        self.state = MsgReceiverState.idle

    def reset(self):
        self.received_string = ""
        self.receive_start = 0
        self.last_received = 0
        self.state = MsgReceiverState.idle

    def run(self):
        try:

            read_chars = self._serial.read_all().decode('utf-8')

        except:
            read_chars = ''

        if self.state == MsgReceiverState.idle and read_chars:
            self.state = MsgReceiverState.receiving
            self.receive_start = time.time_ns()

        # If there is a timeout in either the whole message or some characters, reset.
        if self.state == MsgReceiverState.receiving and \
                (time.time_ns() - self.receive_start > self.timeout or
                 time.time_ns() - self.last_received > self.char_timeout):
            reason = "message timeout" if time.time_ns() - self.receive_start > self.timeout else "char timeout"
            print(f"[WARN] MsgReceiver.run: Unfinished message: {self.received_string}\n\tdue to {reason}")
            self.reset()
        else:
            self.last_received = time.time_ns()
            self.received_string += read_chars

        # Find indices of newline characters in received_string
        newline_index = ([pos for pos, char in enumerate(self.received_string) if char == '\n'])

        # If we received the right format, return it
        if len(self.received_string) > 2 and len(
                newline_index) > 0:  # Minimum length && at least one newline character must be found

            return_data = self.received_string[
                          0:newline_index[0]]  # Select the string up until the first newline character

            if len(self.received_string) > (
                    newline_index[0] + 1):  # there are other characters remaining that need to be processed
                self.received_string = self.received_string[(newline_index[
                                                                 0] + 1):]  # Keep the remaining part of the string to be handled in the next iteration
                self.receive_start = time.time_ns()  # Also timestamp the beginning of reading the next command which is apparently already (partially) waiting.
            else:
                self.reset()
            return return_data
        return None


def reset_arduino(serial: serial.Serial):
    """
    Reset the Arduino using the Serial connection.

    :param serial: Serial object
    :return:
    """
    # Not sure if this works
    # serial.setDTR(False)
    # time.sleep(0.5)
    # serial.flushInput()
    # serial.setDTR(True)
    # rospy.loginfo("Resetting Arduino")
    print("[INFO] Resetting Arduino")
    serial.close()
    globs.global_state = GlobalState.disconnected
    open_serial(serial)


def open_serial(serial: serial.Serial):
    """
    Open the serial connection.

    :param serial: Serial object
    :return:
    """
    serial.open()
    # rospy.loginfo(f"Opening {serial.name}")  # Checking the port
    print(f"[INFO] Opening {serial.name}")  # Checking the port
    while not serial.is_open:
        print('.', end='')
        time.sleep(0.1)  # Necessary to start up the serial connection
    # rospy.loginfo(f"\nConnected to {serial.name}")
    print(f"\nConnected to {serial.name}")
    globs.global_state = GlobalState.waiting
    # rospy.loginfo("Waiting to receive a signal...")
    print("[INFO] Waiting to receive a signal...")


def auto_detect_serial():
    selected = None
    # Try to automatically select the correct serial port for the Arduino
    try:
        serial_path = "/dev/serial/by-id/"
        serial_ports = os.listdir(serial_path)
        match_string = r'usb-([A-Za-z]+).*'
        for p in serial_ports:
            match = re.match(match_string, p)
            if match is not None and match.group(1) == "Arduino":
                selected = os.path.realpath(os.path.join(serial_path, p))
                # rospy.loginfo(f"Found Arduino at {selected} with id {p}.")
                print(f"[INFO] Found Arduino at {selected} with id {p}.")
                break
    except FileNotFoundError:
        pass
    if selected is None:
        selected = '/dev/ttyACM0'
        # rospy.logwarn(f"Could not autodetect Arduino in the system! Trying default {selected}.")
        print(f"[INFO] Could not autodetect Arduino in the system! Trying default {selected}.")
    return selected
