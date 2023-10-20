# Original litehyperbus version:
# Copyright (c) 2019 Antti Lukats <antti.lukats@gmail.com>
# Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Improved aramanth-soc port:
# Copyright (c) 2021-2022 gatecat <gatecat@ds0.me>
# SPDX-License-Identifier: BSD-2-Clause

from amaranth import *

from amaranth.sim import Simulator, Delay, Settle

from amaranth_soc import wishbone
from amaranth_soc.memory import MemoryMap
from ..base.peripheral import Peripheral

from math import ceil, log2

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
    def __init__(self, *, pins, init_latency=7, index=0):
        super().__init__()
        self.pins = pins
        self.cs_count = len(self.pins.csn_o)
        self.size = 2**23 * self.cs_count # 8MB per CS pin
        self.init_latency = init_latency
        assert self.init_latency in (6, 7) # TODO: anything else possible ?
        memory_map = MemoryMap(addr_width=ceil(log2(self.size)), data_width=8)
        memory_map.add_resource(name=f"hyperram{index}", size=self.size, resource=self)
        self.data_bus = wishbone.Interface(addr_width=ceil(log2(self.size / 4)), data_width=32,
                                           granularity=8, memory_map=memory_map)

        bank               = self.csr_bank(addr_width=3)
        self._ctrl_cfg     = bank.csr(32, "rw", name=f"ctrl_cfg")
        self._hram_cfg     = bank.csr(32, "w", name=f"hram_cfg")

        self._bridge    = self.bridge(addr_width=3, data_width=32, granularity=8)
        self.ctrl_bus = self._bridge.bus

        # Control registers
        self.latency = Signal(4, reset=self.init_latency)

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge  = self._bridge

        is_ctrl_write = Signal()
        latched_adr = Signal(len(self.data_bus.adr))
        latched_we = Signal()
        latched_cfg = Signal(16)

        counter = Signal(8)
        wait_count = Signal(4)
        clk = Signal()
        csn = Signal(self.cs_count)

        # Data shift register
        sr = Signal(48)
        sr_shift = Signal()

        # Whether or not we need to apply x2 latency
        x2_lat = Signal()

        # Drive out clock on negedge while active
        m.domains += ClockDomain("neg", clk_edge="neg")
        m.d.comb += [
            ClockSignal("neg").eq(ClockSignal()),
            ResetSignal("neg").eq(ResetSignal()),
        ]
        with m.If(csn.all()):
            # Reset clock if nothing active
            m.d.neg += clk.eq(0)
        with m.Elif(counter.any()):
            m.d.neg += clk.eq(~clk)
            m.d.sync += counter.eq(counter-1)
        with m.If(counter.any()):
            # move shift register (sample/output data) on posedge
            m.d.sync += sr.eq(Cat(self.pins.dq_i, sr[:-8]))


        m.d.comb += [
            self.pins.clk_o.eq(clk),
            self.pins.csn_o.eq(csn),
            self.pins.rstn_o.eq(~ResetSignal()),
            self.pins.dq_o.eq(sr[-8:]),
            self.data_bus.dat_r.eq(sr[:32]),
        ]

        with m.FSM() as fsm:
            with m.State("IDLE"):
                m.d.sync += [
                    counter.eq(0),
                    self.pins.rwds_oe.eq(0),
                    csn.eq((1 << self.cs_count) - 1), # all disabled
                ]
                with m.If(self.data_bus.stb & self.data_bus.cyc): # data bus activity
                    m.d.sync += [
                        csn.eq(~(1 << (self.data_bus.adr[21:]))),
                        self.pins.dq_oe.eq(1),
                        counter.eq(6),
                        # Assign CA
                        sr[47].eq(~self.data_bus.we), # R/W#
                        sr[46].eq(0), # memory space
                        sr[45].eq(1), # linear burst
                        sr[16:45].eq(self.data_bus.adr[2:21]), # upper address
                        sr[4:16].eq(0), # RFU
                        sr[1:3].eq(self.data_bus.adr[0:2]), # lower address
                        sr[0].eq(0), # address LSB (0 for 32-bit xfers)
                        latched_adr.eq(self.data_bus.adr),
                        latched_we.eq(self.data_bus.we),
                        is_ctrl_write.eq(0),
                    ]
                    m.next = "WAIT_CA"
                with m.If(self._hram_cfg.w_stb): # config register write
                    m.d.sync += [
                        csn.eq(~(1 << (self._hram_cfg.w_data[16 : 16+ceil(log2(self.cs_count))]))),
                        self.pins.dq_oe.eq(1),
                        counter.eq(6),
                        # Assign CA
                        sr[47].eq(0), # R/W#
                        sr[46].eq(1), # memory space
                        sr[45].eq(1), # linear burst
                        sr[24:45].eq(1), # upper address
                        sr[16:24].eq(0), # 
                        sr[4:16].eq(0), # RFU
                        sr[1:3].eq(0), # lower address
                        sr[0].eq(0), # address LSB (0 for 32-bit xfers)
                        latched_cfg.eq(self._hram_cfg.w_data[0:16]),
                        is_ctrl_write.eq(1),
                    ]
                    m.next = "WAIT_CA"
            with m.State("WAIT_CA"):
                # Waiting to shift out CA
                with m.If(counter == 3):
                    # RWDS tells us if we need 2x latency or not
                    #   sample at an arbitrary midpoint in CA
                    m.d.sync += x2_lat.eq(self.pins.rwds_i)
                with m.If(counter == 1):
                    # (almost) done shifting CA
                    with m.If(is_ctrl_write):
                        # no latency for control register writes
                        m.d.sync += [
                            counter.eq(2),
                            sr[32:].eq(latched_cfg),
                        ]
                        m.next = "SHIFT_DAT_CTRL"
                    with m.Else():
                        # wait for the specified latency period
                        with m.If(x2_lat):
                            m.d.sync += counter.eq(4 * self.latency - 2)
                        with m.Else():
                            m.d.sync += counter.eq(2 * self.latency - 2)
                        m.next = "WAIT_LAT"
            with m.State("WAIT_LAT"):
                m.d.sync += self.pins.dq_oe.eq(0)
                with m.If(counter == 1):
                    # About to shift data
                    m.d.sync += [
                        sr[:16].eq(0),
                        sr[16:].eq(self.data_bus.dat_w),
                        self.pins.dq_oe.eq(self.data_bus.we),
                        self.pins.rwds_oe.eq(self.data_bus.we),
                        self.pins.rwds_o.eq(~self.data_bus.sel[3]),
                        counter.eq(4),
                    ]
                    m.next = "SHIFT_DAT"
            with m.State("SHIFT_DAT"):
                with m.If(counter == 4):
                    m.d.sync += self.pins.rwds_o.eq(~self.data_bus.sel[2])
                with m.If(counter == 3):
                    m.d.sync += self.pins.rwds_o.eq(~self.data_bus.sel[1])
                with m.If(counter == 2):
                    m.d.sync += self.pins.rwds_o.eq(~self.data_bus.sel[0])
                with m.If(counter == 1):
                    m.next = "ACK_XFER"
            with m.State("ACK_XFER"):
                m.d.sync += [
                    self.pins.rwds_oe.eq(0),
                    self.pins.dq_oe.eq(0),
                    self.data_bus.ack.eq(1),
                    wait_count.eq(9)
                ]
                m.next = "WAIT_NEXT"
            with m.State("WAIT_NEXT"):
                m.d.sync += [
                    self.data_bus.ack.eq(0),
                    wait_count.eq(wait_count-1),
                ]
                with m.If(self.data_bus.stb & self.data_bus.cyc & ~self.data_bus.ack):
                    # Is a valid continuation within same page
                    with m.If((self.data_bus.adr[6:] == latched_adr[6:]) & (self.data_bus.adr[:6] == latched_adr[:6] + 1) & (self.data_bus.we == latched_we)):
                        m.d.sync += [
                            sr[:16].eq(0),
                            sr[16:].eq(self.data_bus.dat_w),
                            self.pins.dq_oe.eq(self.data_bus.we),
                            self.pins.rwds_oe.eq(self.data_bus.we),
                            self.pins.rwds_o.eq(~self.data_bus.sel[3]),
                            latched_adr.eq(self.data_bus.adr),
                            counter.eq(4),
                        ]
                        m.next = "SHIFT_DAT"
                    with m.Else():
                        # start a new xfer
                        m.d.sync += csn.eq((1 << self.cs_count) - 1)
                        m.next = "IDLE"
                with m.Elif(wait_count == 0):
                    m.d.sync += csn.eq((1 << self.cs_count) - 1)
                    m.next = "IDLE"
            with m.State("SHIFT_DAT_CTRL"):
                with m.If(counter == 1):
                    m.next = "CTRL_DONE"
            with m.State("CTRL_DONE"):
                m.d.sync += [
                    self.pins.rwds_oe.eq(0),
                    self.pins.dq_oe.eq(0),
                    csn.eq((1 << self.cs_count) - 1),
                ]
                m.next = "IDLE"
        # Controller config register
        m.d.comb += [
            self._ctrl_cfg.r_data.eq(self.latency)
        ]
        with m.If(self._ctrl_cfg.w_stb):
            m.d.sync += self.latency.eq(self._ctrl_cfg.w_data[0:4])
        return m

def sim():
    pins = HyperRAMPins(cs_count=1)
    m = Module()
    m.submodules.hram = hram = HyperRAM(pins=pins)
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
        yield pins.rwds_i.eq(1)
        yield pins.dq_i.eq(0xFF)
        for i in range(100):
            if (yield hram.data_bus.ack):
                yield hram.data_bus.stb.eq(0)
                yield hram.data_bus.cyc.eq(0)
            yield
        yield hram.ctrl_bus.adr.eq(1)
        yield hram.ctrl_bus.dat_w.eq(0x55AA)
        yield hram.ctrl_bus.sel.eq(0xF)
        yield hram.ctrl_bus.we.eq(1)
        yield hram.ctrl_bus.stb.eq(1)
        yield hram.ctrl_bus.cyc.eq(1)
        for i in range(100):
            if (yield hram.ctrl_bus.ack):
                yield hram.ctrl_bus.stb.eq(0)
                yield hram.ctrl_bus.cyc.eq(0)
            yield
    sim.add_sync_process(process)
    with sim.write_vcd("hyperram.vcd", "hyperram.gtkw"):
        sim.run()

if __name__ == '__main__':
    sim()
