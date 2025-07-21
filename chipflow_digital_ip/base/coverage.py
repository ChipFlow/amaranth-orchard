from abc import ABCMeta
from amaranth.sim import *
# from chipflow_digital_ip.base.vcd_writer import _VCDWriter

class ToggleCoverageObserver(metaclass=ABCMeta):
    def __init__(self, state, **kwargs):
        self.state = state
        self._prev_values = {}
        self._toggles = {}
        self._signal_names = {}

    def update_signal(self, timestamp, signal):
        sig_id = id(signal)
        try:
            val = eval_value(self.state, signal)
        except Exception:
            val = int(self.state.get_signal(signal))

        curr_val = int(val)

        if sig_id not in self._prev_values:
            self._prev_values[sig_id] = curr_val
            self._toggles[sig_id] = {"0->1": 0, "1->0": 0}
            self._signal_names[sig_id] = signal.name
            return

        prev_val = self._prev_values[sig_id]
        if prev_val == 0 and curr_val == 1:
            self._toggles[sig_id]["0->1"] += 1
        elif prev_val == 1 and curr_val == 0:
            self._toggles[sig_id]["1->0"] += 1
        self._prev_values[sig_id] = curr_val

    def update_memory(self, timestamp, memory, addr):
        pass

    def close(self, timestamp):
        pass

    def get_results(self):
        return {
            self._signal_names[sig_id]: toggles
            for sig_id, toggles in self._toggles.items()
        }
