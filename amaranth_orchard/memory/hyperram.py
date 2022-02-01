#
# This file is part of LiteHyperBus
#
# Copyright (c) 2019 Antti Lukats <antti.lukats@gmail.com>
# Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2021-2022 gatecat <gatecat@ds0.me> [amaranth-soc port]
# SPDX-License-Identifier: BSD-2-Clause

from amaranth import *

from amaranth_soc import wishbone
from amaranth_soc.memory import MemoryMap
from ..base.peripheral import Peripheral

from math import ceil, log2

# for Migen compat
def timeline(m, trigger, events):
    lastevent = max([e[0] for e in events])
    counter = Signal(range(lastevent+1))

    # insert counter reset if it doesn't naturally overflow
    # (test if lastevent+1 is a power of 2)
    with m.If(((lastevent & (lastevent + 1)) != 0) & (counter == lastevent)):
        m.d.sync += counter.eq(0)
    with m.Elif(counter != 0):
        m.d.sync += counter.eq(counter + 1)
    with m.Elif(trigger):
        m.d.sync += counter.eq(1)

    def get_cond(e):
        if e[0] == 0:
            return trigger & (counter == 0)
        else:
            return counter == e[0]
    for ev in events:
        with m.If(get_cond(ev)):
            m.d.sync += ev[1]

# HyperRAM -----------------------------------------------------------------------------------------

class HyperRAMPins(Record):
    def __init__(self, cs_count=1):
        layout = [
            ("clk_o", 1),
            ("csn_o", cs_count),
            ("rstn_o", 1),
            ("rwds_o", 1),
            ("rwds_oe", 1),
            ("rwds_i", 1),
            ("dq_o", 8),
            ("dq_oe", 8),
            ("dq_i", 8),
        ]
        super().__init__(layout)

class HyperRAM(Peripheral, Elaboratable):
    """HyperRAM

    Provides a very simple/minimal HyperRAM core that should work with all FPGA/HyperRam chips:
    - FPGA vendor agnostic.
    - no setup/chip configuration (use default latency).

    This core favors portability and ease of use over performance.
    """
    def __init__(self, *, pins, latency=7, index=0):
        super().__init__()
        self.pins = pins
        self.cs_count = len(self.pins.csn_o)
        self.size = 2**23 * self.cs_count # 8MB per CS pin
        self.latency = latency
        assert self.latency in (6, 7) # TODO: anything else possible ?
        self.bus = wishbone.Interface(addr_width=ceil(log2(self.size / 4)),
                                      data_width=32, granularity=8)
        map = MemoryMap(addr_width=ceil(log2(self.size)), data_width=8)
        map.add_resource(name=f"hyperram{index}", size=self.size, resource=self)
        self.bus.memory_map = map
        # # #

    def elaborate(self, platform):
        m = Module()
        clk       = Signal()
        clk_phase = Signal(2)
        cs        = Signal(self.cs_count)
        ca        = Signal(48)
        sr        = Signal(48)

        dq_o = Signal(8)
        dq_i = Signal(8)
        dq_oe = Signal()

        rwds_o = Signal()
        rwds_oe = Signal()

        m.d.comb += [
            self.pins.csn_o.eq(~cs),
            self.pins.clk_o.eq(clk),
            self.pins.rwds_o.eq(rwds_o),
            self.pins.rwds_oe.eq(rwds_oe),
            self.pins.dq_o.eq(dq_o),
            self.pins.dq_oe.eq(Repl(dq_oe, 8)),
            self.pins.rstn_o.eq(~ResetSignal()),
            dq_i.eq(self.pins.dq_i),
        ]

        # Clock Generation (sys_clk/4) -------------------------------------------------------------
        m.d.sync += clk_phase.eq(clk_phase + 1)
        with m.Switch(clk_phase):
            with m.Case(1):
                m.d.sync += clk.eq(cs.any())
            with m.Case(3):
                m.d.sync += clk.eq(0)

        # Data Shift Register (for write and read) -------------------------------------------------
        dqi = Signal(8)
        m.d.sync += dqi.eq(dq_i) # Sample on 90° and 270°
        with m.Switch(clk_phase):
            with m.Case(0, 2):
                m.d.sync += sr.eq(Cat(dqi, sr[:-8]))

        m.d.comb += [
            self.bus.dat_r.eq(sr), # To Wisbone
            dq_o.eq(sr[-8:]), # To HyperRAM
        ]

        # Command generation -----------------------------------------------------------------------
        m.d.comb += [
            ca[47].eq(~self.bus.we),          # R/W#
            ca[45].eq(1),                     # Burst Type (Linear)
            ca[16:35].eq(self.bus.adr[2:21]), # Row & Upper Column Address
            ca[1:3].eq(self.bus.adr[0:2]),    # Lower Column Address
            ca[0].eq(0),                      # Lower Column Address
        ]

        # Sequencer --------------------------------------------------------------------------------
        dt_seq = [
            # DT,  Action
            (3,    []),
            (12,   [cs.eq(1 << (self.bus.adr[21:])), dq_oe.eq(1), sr.eq(ca)]),    # Command: 6 clk
            (self.latency * 8 - 4,   [dq_oe.eq(0)]),        # Latency(default): 2*6 clk
            (2,    [dq_oe.eq(self.bus.we),                  # Write/Read data byte: 2 clk
                    sr[:16].eq(0),
                    sr[16:].eq(self.bus.dat_w),
                    rwds_oe.eq(self.bus.we),
                    rwds_o.eq(~self.bus.sel[3])]),
            (2,    [rwds_o.eq(~self.bus.sel[2])]),              # Write/Read data byte: 2 clk
            (2,    [rwds_o.eq(~self.bus.sel[1])]),              # Write/Read data byte: 2 clk
            (2,    [rwds_o.eq(~self.bus.sel[0])]),              # Write/Read data byte: 2 clk
            (2,    [cs.eq(0), rwds_oe.eq(0), dq_oe.eq(0)]),
            (1,    [self.bus.ack.eq(1)]),
            (1,    [self.bus.ack.eq(0)]),
            (0,    []),
        ]
        # Convert delta-time sequencer to time sequencer
        t_seq = []
        t_seq_start = (clk_phase == 1)
        t = 0
        for dt, a in dt_seq:
            t_seq.append((t, a))
            t += dt
        timeline(m, self.bus.cyc & self.bus.stb & t_seq_start, t_seq)
        return m

