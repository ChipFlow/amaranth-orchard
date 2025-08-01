import unittest
from amaranth import *
from amaranth.sim import *
from amaranth.sim._coverage import ToggleCoverageObserver, ToggleDirection
from chipflow_digital_ip.io import GPIOPeripheral
import re


def collect_all_signals(obj):
    signals = []

    def _collect(obj):
        for attr_name in dir(obj):
            if attr_name.startswith("_"):
                continue
            try:
                attr = getattr(obj, attr_name)
            except Exception:
                continue
            if isinstance(attr, Signal):
                signals.append(attr)
        if hasattr(obj, 'submodules'):
            submodules = getattr(obj, 'submodules')
            if isinstance(submodules, dict):
                for subm in submodules.values():
                    _collect(subm)
            elif hasattr(submodules, '__iter__'): 
                for subm in submodules:
                    _collect(subm)
    _collect(obj)
    return signals

    
def get_signal_full_paths(design):
    signal_path_map = {}
    for fragment, fragment_info in design.fragments.items():
        fragment_name = ("bench", *fragment_info.name)
        for signal, signal_name in fragment_info.signal_names.items():
            path = "/".join(fragment_name + (signal_name,))
            signal_path_map[id(signal)] = path 
    return signal_path_map
