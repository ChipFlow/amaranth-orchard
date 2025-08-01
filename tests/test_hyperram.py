# amaranth: UnusedElaboratable=no

# SPDX-License-Identifier: BSD-2-Clause

from amaranth import *
from amaranth.sim import *
from chipflow_digital_ip.memory import HyperRAM
from amaranth.sim._coverage import ToggleCoverageObserver, ToggleDirection
from tests.test_utils import get_signal_full_paths, collect_all_signals

def test_hyperram_smoke():
    m = Module()
    m.submodules.hram = hram = HyperRAM()
    sim = Simulator(m)
    sim.add_clock(1e-6)

    def process():
        yield hram.data_bus.adr.eq(0x5A5A5A)
        yield hram.data_bus.dat_w.eq(0xF0F0F0F0)
        yield hram.data_bus.sel.eq(1)
        yield hram.data_bus.we.eq(1)
        yield hram.data_bus.stb.eq(1)
        yield hram.data_bus.cyc.eq(1)
        for i in range(100):
            if (yield hram.data_bus.ack):
                yield hram.data_bus.stb.eq(0)
                yield hram.data_bus.cyc.eq(0)
            yield
        yield hram.data_bus.adr.eq(0x5A5A5A)
        yield hram.data_bus.sel.eq(1)
        yield hram.data_bus.we.eq(0)
        yield hram.data_bus.stb.eq(1)
        yield hram.data_bus.cyc.eq(1)
        yield hram.pins.rwds.i.eq(1)
        yield hram.pins.dq.i.eq(0xFF)
        for i in range(100):
            if (yield hram.data_bus.ack):
                yield hram.data_bus.stb.eq(0)
                yield hram.data_bus.cyc.eq(0)
            yield
    design = sim._engine._design
    signal_path_map = get_signal_full_paths(design)
    toggle_cov = ToggleCoverageObserver(sim._engine.state, signal_path_map=signal_path_map)
    sim._engine.add_observer(toggle_cov)
    sim.add_sync_process(process)
    all_signals = collect_all_signals(m)
    with sim.write_vcd("hyperram.vcd", "hyperram.gtkw"):
        sim.run()

    results = toggle_cov.get_results()
    print("=== Toggle Coverage Report ===")

    for signal_name, bit_toggles in results.items():
        print(f"{signal_name}:")
        for bit, counts in bit_toggles.items():
            zero_to_one = counts[ToggleDirection.ZERO_TO_ONE]
            one_to_zero = counts[ToggleDirection.ONE_TO_ZERO]
            print(f"  Bit {bit}: 0→1={zero_to_one}, 1→0={one_to_zero}")





