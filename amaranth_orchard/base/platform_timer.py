from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, flipped, connect

from amaranth_soc import csr

from .field import StorageLessRW


__all__ = ["PlatformTimer"]


class PlatformTimer(wiring.Component):
    class Time(csr.Register, access="rw"):
        def __init__(self, width):
            super().__init__({"val": csr.Field(StorageLessRW, unsigned(width))})

    """Platform timer device for SoCs.

    Two CSRs forming a 48-bit timer that can be read to get tick count (upper 16 bits unused).

    Writing to CSRs, starting with high, then low, schedules an interrupt at the compare value
    written.
    """
    def __init__(self, *, name):
        self.width = 48

        regs = csr.Builder(addr_width=4, data_width=8, name=name)

        self._time_l = regs.add("time_l", self.Time(32), offset=0x0)
        self._time_h = regs.add("time_h", self.Time(16), offset=0x4)

        self._bridge = csr.Bridge(regs.as_memory_map())

        super().__init__({
            "bus": In(csr.Signature(addr_width=regs.addr_width, data_width=regs.data_width)),
            "irq": Out(unsigned(1)),
        })
        self.bus.memory_map = self._bridge.bus.memory_map

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        connect(m, flipped(self.bus), self._bridge.bus)

        timer = Signal(self.width)
        time_cmp = Signal(self.width)
        time_cmp_valid = Signal(reset=0)

        m.d.sync += [
            timer.eq(timer + 1),
            self.irq.eq(time_cmp_valid & (timer >= time_cmp)),
        ]

        m.d.comb += Cat(self._time_l.f.val.r_data, self._time_h.f.val.r_data).eq(timer)

        with m.If(self._time_h.f.val.w_stb):
             m.d.sync += [
                time_cmp[32:].eq(self._time_h.f.val.w_data),
                time_cmp_valid.eq(0),
            ]
        with m.If(self._time_l.f.val.w_stb):
             m.d.sync += [
                time_cmp[:32].eq(self._time_l.f.val.w_data),
                time_cmp_valid.eq(1),
            ]

        return m
