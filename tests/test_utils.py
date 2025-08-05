import unittest
from amaranth import *
from amaranth.sim import *
from amaranth.sim._coverage import ToggleCoverageObserver, ToggleDirection
from amaranth.sim._coverage import StatementCoverageObserver
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



def get_assign_name(domain, stmt):
    lhs = getattr(stmt.lhs, "name", repr(stmt.lhs))
    rhs = repr(stmt.rhs)
    return f"{domain}:{lhs} = {rhs}"

def tag_assign_statements(fragment, coverage_id=0, parent_path=()):
    from amaranth.hdl._ast import Assign, Switch
    for domain, stmts in fragment.statements.items():
        for stmt in stmts:
            if isinstance(stmt, Assign):
                stmt._coverage_id = (parent_path, domain, coverage_id)
                stmt._coverage_name = get_assign_name(domain, stmt)
                coverage_id += 1
            elif isinstance(stmt, Switch):
                for _patterns, sub_stmts, _src_loc in stmt.cases:
                    for sub_stmt in sub_stmts:
                        if isinstance(sub_stmt, Assign):
                            sub_stmt._coverage_id = (parent_path, domain, coverage_id)
                            sub_stmt._coverage_name = get_assign_name(domain, sub_stmt)
                            coverage_id += 1
                        elif isinstance(sub_stmt, Switch):
                            for _patterns2, sub_stmts2, _src_loc2 in sub_stmt.cases:
                                for sub_sub_stmt in sub_stmts2:
                                    if isinstance(sub_sub_stmt, Assign):
                                        sub_sub_stmt._coverage_id = (parent_path, domain, coverage_id)
                                        sub_sub_stmt._coverage_name = get_assign_name(domain, sub_sub_stmt)
                                        coverage_id += 1
    for subfragment, name, _src_loc in getattr(fragment, "subfragments", []):
        coverage_id = tag_assign_statements(subfragment, coverage_id, parent_path + (name,))
    return coverage_id

def insert_coverage_signals(fragment):
    from amaranth.hdl._ast import Assign, Const, Signal
    coverage_signals = {}
    for domain, stmts in fragment.statements.items():
        for stmt in stmts:
            if hasattr(stmt, "_coverage_id"):
                sig_name = f"cov_stmt_{stmt._coverage_id[-1]}"
                coverage_signals[stmt._coverage_id] = Signal(name=sig_name, reset=0)
    for domain, stmts in fragment.statements.items():
        new_stmts = []
        for stmt in stmts:
            if hasattr(stmt, "_coverage_id"):
                cov_sig = coverage_signals[stmt._coverage_id]
                new_stmts.append(Assign(cov_sig, Const(1)))
            new_stmts.append(stmt)
        fragment.statements[domain] = new_stmts
    return coverage_signals