from enum import Enum, auto
import time


class GlobalState(Enum):
    """
    Class for managing object states in a readable manner
    :param: none
    :return: the created object
    """
    disconnected = auto()
    waiting = auto()
    ready = auto()
    shuttingDown = auto()


class Diagnostics:
    DIAGNOSTICS_LOOP_FREQ_TRACKER_TIMEOUT = 2000 * 1E6
    DIAGNOSTICS_REPORT_TIMEOUT = 2 * 1E9  # nanoseconds

    report_timestamp = time.time_ns()
    track_num_telemetry = 0
    track_num_actuation = 0
    track_num_mainLoop = 0

    def evaluate(self):
        self.track_num_mainLoop += 1
        now_time = time.time_ns()

        if (now_time - self.report_timestamp) >= self.DIAGNOSTICS_REPORT_TIMEOUT:
            # Calculate passed time
            time_passed = now_time - self.report_timestamp

            # Calculate task occurrence (hz)
            mainloop_freq = self.track_num_mainLoop / time_passed * 1E9
            self.track_num_mainLoop = 0  # reset tracker
            actuation_freq = self.track_num_actuation / time_passed * 1E9
            self.track_num_actuation = 0  # reset tracker
            telemetry_freq = self.track_num_telemetry / time_passed * 1E9
            self.track_num_telemetry = 0  # reset tracker

            # Share relevant information
            print(f"script diagnostics: mainloop runs at: {mainloop_freq} hz, "
                  f"sending actuation: {actuation_freq} hz, "
                  f"acquiring telemetry: {telemetry_freq} hz")

            # Finish up. Set last timestamp
            self.report_timestamp = now_time


global_state = GlobalState.disconnected
