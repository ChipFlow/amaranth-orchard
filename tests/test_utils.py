import unittest
from amaranth import *
from amaranth.sim import *
from amaranth.sim._coverage import ToggleCoverageObserver, ToggleDirection
from amaranth.sim._coverage import *
from chipflow_digital_ip.io import GPIOPeripheral
import re
from collections import Counter


##### TOGGLE COVERAGE #####
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


##### STATEMENT COVERAGE #####
AGG_STMT_HITS = Counter()
AGG_STMT_INFO = {} 

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
        filename, lineno = src_loc[0], src_loc[1]
        anchor = "chipflow-digital-ip"
        idx = filename.find(anchor)
        filename = filename[idx:] if idx != -1 else filename.split("/")[-1]
        loc_str = f"{filename}:{lineno}"
    else:
        loc_str = "unknown"
    cov_id = getattr(switch_stmt, "_coverage_id", None)
    parent_path = cov_id[0] if cov_id else ()
    if parent_path:
        safe_parts = [("anon" if (p is None or p == "") else str(p)) for p in parent_path]
        path_str = "/".join(safe_parts)
    else:
        path_str = "top"
    patterns_str = "default" if patterns is None else str(patterns)
    return f"{loc_str} | {path_str} | {domain}:switch_case({patterns_str})"

def tag_all_statements(fragment, coverage_id=0, parent_path=(), stmtid_to_info=None):
    from amaranth.hdl._ast import Assign, Switch
    if stmtid_to_info is None:
        stmtid_to_info = {}
    if not hasattr(fragment, "statements"):
        return coverage_id, stmtid_to_info
    for domain, stmts in fragment.statements.items():
        for stmt in stmts:
            if hasattr(stmt, "_coverage_id"):
                continue
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
                case_ids = []
                for patterns, sub_stmts, case_src_loc in stmt.cases:
                    case_cov_id = (parent_path, domain, coverage_id)
                    case_name = get_switch_case_name(domain, stmt, patterns, src_loc=case_src_loc)
                    stmtid_to_info[case_cov_id] = (case_name, "switch_case")
                    case_ids.append(case_cov_id)
                    coverage_id += 1
                stmt._coverage_case_ids = tuple(case_ids)
                for patterns, sub_stmts, case_src_loc in stmt.cases:
                    for sub_stmt in sub_stmts:
                        tmp = type("TempFrag", (), {"statements": {domain: [sub_stmt]}, "subfragments": []})()
                        coverage_id, stmtid_to_info = tag_all_statements(
                            tmp, coverage_id, parent_path, stmtid_to_info
                        )
    for subfragment, name, _src_loc in getattr(fragment, "subfragments", []):
        if hasattr(subfragment, "statements"):
            coverage_id, stmtid_to_info = tag_all_statements(
                subfragment, coverage_id, parent_path + (name,), stmtid_to_info
            )
    return coverage_id, stmtid_to_info

def insert_coverage_signals(fragment):
    from amaranth.hdl._ast import Assign, Const, Signal, Switch as AstSwitch
    from amaranth.hdl._ir import Fragment as AmaranthFragment
    coverage_signals = {}
    def cov_name(cov_id, typ):
        parent_path, domain, serial = cov_id
        if parent_path:
            path = "_".join("anon" if p is None else str(p) for p in parent_path)
        else:
            path = "top"
        return f"cov_{path}_{domain}_{typ}_{serial}"
    def inject_in_stmt_list(domain, stmts):
        new = []
        for stmt in stmts:
            if hasattr(stmt, "_coverage_id") and not getattr(stmt, "_coverage_injected", False):
                cov_id = stmt._coverage_id
                typ = getattr(stmt, "_coverage_type", "unknown")
                sig = coverage_signals.get(cov_id)
                if sig is None:
                    sig = Signal(name=cov_name(cov_id, typ), init=0)
                    coverage_signals[cov_id] = sig
                new.append(Assign(sig, Const(1)))
                stmt._coverage_injected = True
            new.append(stmt)
            if isinstance(stmt, AstSwitch):
                case_ids = getattr(stmt, "_coverage_case_ids", ())
                for idx, (patterns, sub_stmts, case_src_loc) in enumerate(stmt.cases):
                    if idx < len(case_ids):
                        case_cov_id = case_ids[idx]
                        case_sig = coverage_signals.get(case_cov_id)
                        if case_sig is None:
                            case_sig = Signal(name=cov_name(case_cov_id, "switch_case"), init=0)
                            coverage_signals[case_cov_id] = case_sig
                        sub_stmts[:0] = [Assign(case_sig, Const(1))]
                    instrumented = inject_in_stmt_list(domain, list(sub_stmts))
                    sub_stmts[:] = instrumented
        return new
    def walk_fragment(frag):
        if not isinstance(frag, AmaranthFragment):
            return
        for domain, stmts in list(frag.statements.items()):
            frag.statements[domain] = inject_in_stmt_list(domain, list(stmts))
        for subfrag, _name, _sloc in getattr(frag, "subfragments", []):
            if hasattr(subfrag, "statements"):
                walk_fragment(subfrag)
    walk_fragment(fragment)
    return coverage_signals

def mk_sim_with_stmtcov(dut, verbose=False):
    mod = dut.elaborate(platform=None)
    fragment = Fragment.get(mod, platform=None)
    _, stmtid_to_info = tag_all_statements(fragment)
    coverage_signals = insert_coverage_signals(fragment)
    signal_to_stmtid = {id(sig): stmt_id for stmt_id, sig in coverage_signals.items()}
    sim = Simulator(fragment)
    stmt_cov = StatementCoverageObserver(signal_to_stmtid, sim._engine.state, stmtid_to_info=stmtid_to_info)
    sim._engine.add_observer(stmt_cov)
    if verbose:
        total_stmts = len(stmtid_to_info)
        print(f"[mk_sim_with_stmtcov] Instrumented {total_stmts} statements for coverage.")
    return sim, stmt_cov, stmtid_to_info, fragment

def merge_stmtcov(results, stmtid_to_info):
    for sid, info in stmtid_to_info.items():
        if sid not in AGG_STMT_INFO:
            AGG_STMT_INFO[sid] = info
    for sid, hits in results.items():
        AGG_STMT_HITS[sid] += hits

def emit_agg_summary(json_path="i2c_statement_cov.json", label="test_i2c.py"):
    total = len(AGG_STMT_INFO)
    hit = sum(1 for sid in AGG_STMT_INFO if AGG_STMT_HITS.get(sid, 0) > 0)
    pct = 100.0 if total == 0 else (hit / total) * 100.0
    print(f"\n[Statement coverage for {label}] {hit}/{total} = {pct:.1f}%")
    # items = []
    # for sid, (name, typ) in AGG_STMT_INFO.items():
    #     items.append((sid, name, typ, int(AGG_STMT_HITS.get(sid, 0))))
    # items.sort(key=lambda x: x[1])  # sort by name
    #
    # print("\nStatements HIT:")
    # for _, name, _, h in items:
    #     if h > 0:
    #         print(f"  {name} ({h} hits)")
    #
    # print("\nStatements NOT hit:")
    # for _, name, _, h in items:
    #     if h == 0:
    #         print(f"  {name}")
    try:
        import json
        report = []
        for sid, (name, typ) in AGG_STMT_INFO.items():
            report.append({
                "id": str(sid),
                "name": name,
                "type": typ,
                "hits": int(AGG_STMT_HITS.get(sid, 0)),
            })
        with open(json_path, "w") as f:
            json.dump({"summary": {"hit": hit, "total": total, "percent": pct},
                       "statements": report}, f, indent=2)
        print(f"Wrote {json_path}")
    except Exception as e:
        print(f"(could not write JSON report: {e})")


##### BRANCH COVERAGE #####
AGG_BLOCK_HITS = Counter()
AGG_BLOCK_INFO = {}

def get_block_name(domain, parent_path, tag, src_loc=None):
    if parent_path:
        safe_parts = [("anon" if (p is None or p == "") else str(p)) for p in parent_path]
        path_str = "/".join(safe_parts)
    else:
        path_str = "top"
    if src_loc:
        filename, lineno = src_loc[0], src_loc[1]
        anchor = "chipflow-digital-ip"
        idx = filename.find(anchor)
        filename = filename[idx:] if idx != -1 else filename.split("/")[-1]
        loc_str = f"{filename}:{lineno}"
    else:
        loc_str = "unknown"
    return f"{loc_str} | {path_str} | {domain}:block({tag})"

def tag_all_blocks(fragment, coverage_id=0, parent_path=(), blockid_to_info=None):
    from amaranth.hdl._ast import Switch
    if blockid_to_info is None:
        blockid_to_info = {}
    if not hasattr(fragment, "_block_cov_ids"):
        fragment._block_cov_ids = []
    for domain, stmts in getattr(fragment, "statements", {}).items():
        first_src = getattr(stmts, "0", None)
        first_src = getattr(stmts[0], "src_loc", None) if stmts else None
        blk_id = (parent_path, domain, coverage_id)
        blk_name = get_block_name(domain, parent_path, "root", src_loc=first_src)
        blockid_to_info[blk_id] = (blk_name, "block")
        fragment._block_cov_ids.append((id(stmts), blk_id))
        coverage_id += 1
    for domain, stmts in getattr(fragment, "statements", {}).items():
        for stmt in stmts:
            if isinstance(stmt, Switch):
                for patterns, sub_stmts, case_src_loc in stmt.cases:
                    blk_id = (parent_path, domain, coverage_id)
                    pat_str = "default" if patterns is None else str(patterns)
                    blk_name = get_block_name(domain, parent_path, f"case:{pat_str}", src_loc=case_src_loc)
                    blockid_to_info[blk_id] = (blk_name, "block")
                    if not hasattr(stmt, "_block_cov_ids"):
                        stmt._block_cov_ids = []
                    stmt._block_cov_ids.append((id(sub_stmts), blk_id))
                    coverage_id += 1
    for subfragment, name, _src_loc in getattr(fragment, "subfragments", []):
        if hasattr(subfragment, "statements"):
            coverage_id, blockid_to_info = tag_all_blocks(
                subfragment, coverage_id, parent_path + (name,), blockid_to_info
            )
    return coverage_id, blockid_to_info

def insert_block_coverage_signals(fragment, blockid_to_info):
    from amaranth.hdl._ast import Assign, Const, Signal, Switch as AstSwitch
    from amaranth.hdl._ir import Fragment as AmaranthFragment
    coverage_signals = {}
    listid_to_blockid = {}
    def cov_name(cov_id):
        parent_path, domain, serial = cov_id
        path = "_".join("anon" if p is None else str(p) for p in parent_path) if parent_path else "top"
        return f"blk_{path}_{domain}_{serial}"
    def harvest_block_ids(frag):
        if hasattr(frag, "_block_cov_ids"):
            for list_id, blk_id in frag._block_cov_ids:
                listid_to_blockid[list_id] = blk_id
        for domain, stmts in getattr(frag, "statements", {}).items():
            for stmt in stmts:
                if isinstance(stmt, AstSwitch) and hasattr(stmt, "_block_cov_ids"):
                    for list_id, blk_id in stmt._block_cov_ids:
                        listid_to_blockid[list_id] = blk_id
        for subfrag, _name, _sloc in getattr(frag, "subfragments", []):
            if hasattr(subfrag, "statements"):
                harvest_block_ids(subfrag)
    harvest_block_ids(fragment)
    def inject_head(stmts, blk_id):
        if blk_id is None:
            return
        sig = coverage_signals.get(blk_id)
        if sig is None:
            sig = Signal(name=cov_name(blk_id), init=0)
            coverage_signals[blk_id] = sig
        stmts[:0] = [Assign(sig, Const(1))]
    def walk_fragment(frag):
        if not isinstance(frag, AmaranthFragment):
            return
        for domain, stmts in list(frag.statements.items()):
            inject_head(stmts, listid_to_blockid.get(id(stmts)))
            for stmt in stmts:
                if isinstance(stmt, AstSwitch):
                    for _patterns, sub_stmts, _case_src_loc in stmt.cases:
                        inject_head(sub_stmts, listid_to_blockid.get(id(sub_stmts)))
        for subfrag, _name, _sloc in getattr(frag, "subfragments", []):
            if hasattr(subfrag, "statements"):
                walk_fragment(subfrag)
    walk_fragment(fragment)
    return coverage_signals

def mk_sim_with_blockcov(dut, verbose=False):
    mod = dut.elaborate(platform=None)
    fragment = Fragment.get(mod, platform=None)
    _, blockid_to_info = tag_all_blocks(fragment)
    coverage_signals = insert_block_coverage_signals(fragment, blockid_to_info)
    signal_to_blockid = {id(sig): blk_id for blk_id, sig in coverage_signals.items()}
    sim = Simulator(fragment)
    blk_cov = BlockCoverageObserver(signal_to_blockid, sim._engine.state, blockid_to_info=blockid_to_info)
    sim._engine.add_observer(blk_cov)
    if verbose:
        total_blocks = len(blockid_to_info)
        print(f"[mk_sim_with_blockcov] Instrumented {total_blocks} blocks for coverage.")
    return sim, blk_cov, blockid_to_info, fragment

def merge_blockcov(results, blockid_to_info):
    for bid, info in blockid_to_info.items():
        if bid not in AGG_BLOCK_INFO:
            AGG_BLOCK_INFO[bid] = info
    for bid, hits in results.items():
        AGG_BLOCK_HITS[bid] += hits
        
def emit_agg_block_summary(json_path="i2c_block_cov.json", label="test_i2c.py",
                           show_hits=True, show_misses=True, max_print=None, sort_by="name"):
    total = len(AGG_BLOCK_INFO)
    hit = sum(1 for bid in AGG_BLOCK_INFO if AGG_BLOCK_HITS.get(bid, 0) > 0)
    pct = 100.0 if total == 0 else (hit / total) * 100.0
    print(f"\n[Block coverage for {label}] {hit}/{total} = {pct:.1f}%")

    # items = []
    # for bid, (name, typ) in AGG_BLOCK_INFO.items():
    #     items.append((bid, name, typ, int(AGG_BLOCK_HITS.get(bid, 0))))

    # if sort_by == "hits":
    #     items.sort(key=lambda x: (x[3], x[1]))
    # else:
    #     items.sort(key=lambda x: x[1])

    # if show_hits:
    #     hits_list = [(bid, name, typ, h) for (bid, name, typ, h) in items if h > 0]
    #     if max_print is not None:
    #         hits_list = hits_list[:max_print]
    #     print("\nBlocks HIT:")
    #     for _, name, _, h in hits_list:
    #         print(f"  {name} ({h} hits)")

    # if show_misses:
    #     miss_list = [(bid, name, typ, h) for (bid, name, typ, h) in items if h == 0]
    #     if max_print is not None:
    #         miss_list = miss_list[:max_print]
    #     print("\nBlocks NOT hit:")
    #     for _, name, _, _ in miss_list:
    #         print(f"  {name}")

    try:
        import json
        report = []
        for bid, (name, typ) in AGG_BLOCK_INFO.items():
            report.append({
                "id": str(bid),
                "name": name,
                "type": typ,
                "hits": int(AGG_BLOCK_HITS.get(bid, 0)),
            })
        with open(json_path, "w") as f:
            json.dump({"summary": {"hit": hit, "total": total, "percent": pct},
                       "blocks": report}, f, indent=2)
        print(f"\nWrote {json_path}")
    except Exception as e:
        print(f"(could not write JSON report: {e})")
