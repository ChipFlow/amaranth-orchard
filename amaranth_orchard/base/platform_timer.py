from amaranth import *

from .peripheral import Peripheral

class PlatformTimer(Peripheral, Elaboratable):
    """Platform timer device for SoCs
    Two CSRs forming a 48-bit timer that can be read to get tick count
        (upper 16 bits unused)

    Writing to CSRs, starting with high, then low,
    schedules an interrupt at the compare value written
    """

    def __init__(self, width=48, **kwargs):
        super().__init__()

        bank            = self.csr_bank()
        self._time_low  = bank.csr(32, "rw")
        self._time_high = bank.csr(32, "rw")

        self._bridge    = self.bridge(data_width=32, granularity=8, alignment=2)
        self.bus        = self._bridge.bus

        self.width = width
        self.timer_irq  = Signal()

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge  = self._bridge

        timer = Signal(self.width)
        time_cmp = Signal(self.width)
        time_cmp_valid = Signal(reset=0)

        m.d.comb += [
            self._time_high.r_data.eq(timer[32:self.width]),
            self._time_low.r_data.eq(timer[0:32]),
        ]
        m.d.sync += [
            timer.eq(timer + 1),
            self.timer_irq.eq(time_cmp_valid & (timer >= time_cmp)),
        ]

        with m.If(self._time_high.w_stb.any()):
             m.d.sync += [
                time_cmp[32:self.width].eq(self._time_high.w_data),
                time_cmp_valid.eq(0),
            ]
        with m.If(self._time_low.w_stb.any()):
             m.d.sync += [
                time_cmp[0:32].eq(self._time_low.w_data),
                time_cmp_valid.eq(1),
            ]

        return m
