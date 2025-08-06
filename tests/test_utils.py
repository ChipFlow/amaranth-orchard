import unittest
from amaranth import *
from amaranth.sim import *
from amaranth.sim._coverage import ToggleCoverageObserver, ToggleDirection
from amaranth.sim._coverage import StatementCoverageObserver
from chipflow_digital_ip.io import GPIOPeripheral
import re

def collect_all_signals(obj):
    signals = []
    def _collect(o):
        for attr_name in dir(o):
            if attr_name.startswith("_"):
                continue
            try:
                attr = getattr(o, attr_name)
            except Exception:
                continue
            if isinstance(attr, Signal):
                signals.append(attr)
        if hasattr(o, 'submodules'):
            submodules = getattr(o, 'submodules')
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
    def expr_name(expr):
        if hasattr(expr, "value") and hasattr(expr, "start") and hasattr(expr, "stop"):
            base = expr_name(expr.value)
            if expr.start == expr.stop - 1:
                return f"{base}[{expr.start}]"
            else:
                return f"{base}[{expr.start}:{expr.stop}]"
        if hasattr(expr, "name"):
            return expr.name
        if hasattr(expr, "value"):
            return str(expr.value)
        return str(expr)
    src_loc = getattr(stmt, "src_loc", None)
    if src_loc:
        filename = src_loc[0]
        lineno = src_loc[1]
        anchor = "chipflow-digital-ip"
        idx = filename.find(anchor)
        if idx != -1:
            filename = filename[idx:]
        else:
            filename = filename.split("/")[-1]
        loc_str = f"{filename}:{lineno}"
    else:
        loc_str = "unknown"
    lhs = expr_name(stmt.lhs)
    rhs = expr_name(stmt.rhs)
    return f"{loc_str} | {domain}:{lhs} = {rhs}"

def get_switch_case_name(domain, switch_stmt, patterns, src_loc=None):
    if src_loc is None:
        src_loc = getattr(switch_stmt, "src_loc", None)
    if src_loc:
        filename = src_loc[0]
        lineno = src_loc[1]
        anchor = "chipflow-digital-ip"
        idx = filename.find(anchor)
        if idx != -1:
            filename = filename[idx:]
        else:
            filename = filename.split("/")[-1]
        loc_str = f"{filename}:{lineno}"
    else:
        loc_str = "unknown"
    patterns_str = str(patterns) if patterns is not None else "default"
    return f"{loc_str} | {domain}:switch_case({patterns_str})"

def tag_all_statements(fragment, coverage_id=0, parent_path=(), stmtid_to_info=None):
    from amaranth.hdl._ast import Assign, Switch
    from amaranth.hdl._ir import Fragment as AmaranthFragment

    if stmtid_to_info is None:
        stmtid_to_info = {}

    if not isinstance(fragment, AmaranthFragment):
        return coverage_id, stmtid_to_info

    for domain, stmts in fragment.statements.items():
        for stmt in stmts:
            if isinstance(stmt, Assign):
                stmt._coverage_id = (parent_path, domain, coverage_id)
                stmt._coverage_name = get_assign_name(domain, stmt)
                stmt._coverage_type = "assign"
                stmtid_to_info[stmt._coverage_id] = (stmt._coverage_name, stmt._coverage_type)
                coverage_id += 1
            elif isinstance(stmt, Switch):
                stmt._coverage_id = (parent_path, domain, coverage_id)
                stmt._coverage_name = f"{domain}:switch at {getattr(stmt, 'src_loc', 'unknown')}"
                stmt._coverage_type = "switch"
                stmtid_to_info[stmt._coverage_id] = (stmt._coverage_name, stmt._coverage_type)
                coverage_id += 1
                for patterns, sub_stmts, src_loc in stmt.cases:
                    for sub_stmt in sub_stmts:
                        if isinstance(sub_stmt, Assign):
                            sub_stmt._coverage_id = (parent_path, domain, coverage_id)
                            sub_stmt._coverage_name = get_assign_name(domain, sub_stmt)
                            sub_stmt._coverage_type = "assign"
                            stmtid_to_info[sub_stmt._coverage_id] = (sub_stmt._coverage_name, sub_stmt._coverage_type)
                            coverage_id += 1
                        elif isinstance(sub_stmt, Switch):
                            coverage_id, stmtid_to_info = tag_all_statements(
                                sub_stmt, coverage_id, parent_path, stmtid_to_info)

    for subfragment, name, _src_loc in getattr(fragment, "subfragments", []):
        if isinstance(subfragment, AmaranthFragment):
            coverage_id, stmtid_to_info = tag_all_statements(
                subfragment, coverage_id, parent_path + (name,), stmtid_to_info)
    return coverage_id, stmtid_to_info

def insert_coverage_signals(fragment):
    from amaranth.hdl._ast import Assign, Const, Signal

    coverage_signals = {}
    for domain, stmts in fragment.statements.items():
        for stmt in stmts:
            if hasattr(stmt, "_coverage_id"):
                cov_id = stmt._coverage_id
                typ = getattr(stmt, "_coverage_type", "unknown")
                sig_name = f"cov_{typ}_{cov_id[-1]}"
                if cov_id not in coverage_signals:
                    coverage_signals[cov_id] = Signal(name=sig_name, reset=0)

    for domain, stmts in fragment.statements.items():
        new_stmts = []
        for stmt in stmts:
            if hasattr(stmt, "_coverage_id"):
                cov_sig = coverage_signals[stmt._coverage_id]
                new_stmts.append(Assign(cov_sig, Const(1)))
            new_stmts.append(stmt)
        fragment.statements[domain] = new_stmts
    return coverage_signals