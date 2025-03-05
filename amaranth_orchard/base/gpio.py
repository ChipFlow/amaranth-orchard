from amaranth import Module, unsigned
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, flipped, connect

from amaranth_soc import csr

from chipflow_lib.platforms import BidirPinSignature

__all__ = ["GPIOPins", "GPIOPeripheral"]


class GPIOPeripheral(wiring.Component):
    class Signature(wiring.Signature):
        def __init__(self, width):
            if width > 32:
                raise ValueError(f"Pin width must be lesser than or equal to 32, not {width}")
            self._width = width
            super().__init__({
                "gpio": Out(BidirPinSignature(width, all_have_oe=True))
            })

        @property
        def width(self):
            return self._width

    class DO(csr.Register, access="rw"):
        """output data (R/W, ignored for pins configured as inputs)"""

        def __init__(self, width):
            super().__init__({"pins": csr.Field(csr.action.RW, unsigned(width))})

    class OE(csr.Register, access="rw"):
        """output enable (R/W) 1=output, 0=input"""

        def __init__(self, width):
            super().__init__({"pins": csr.Field(csr.action.RW, unsigned(width))})

    class DI(csr.Register, access="r"):
        """input data (R)"""

        def __init__(self, width):
            super().__init__({"pins": csr.Field(csr.action.R, unsigned(width))})

    def __init__(self, *, width: int):
        """Simple GPIO peripheral.

        All pins default to input at power up.
        """
        self.width = width

        regs = csr.Builder(addr_width=4, data_width=8)

        self._do = regs.add("do", self.DO(self.width), offset=0x0)
        self._oe = regs.add("oe", self.OE(self.width), offset=0x4)
        self._di = regs.add("di", self.DI(self.width), offset=0x8)

        self._bridge = csr.Bridge(regs.as_memory_map())

        super().__init__({
            "bus": In(csr.Signature(addr_width=regs.addr_width, data_width=regs.data_width)),
            "pins": Out(self.Signature(width))
        })
        self.bus.memory_map = self._bridge.bus.memory_map

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        connect(m, flipped(self.bus), self._bridge.bus)

        m.d.comb += self.pins.gpio.o.eq(self._do.f.pins.data)
        m.d.comb += self.pins.gpio.oe.eq(self._oe.f.pins.data)
        m.d.comb += self._di.f.pins.r_data.eq(self.pins.gpio.i)

        return m
