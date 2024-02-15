from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, flipped, connect

from amaranth_soc import csr


__all__ = ["GPIOPins", "GPIOPeripheral"]


class GPIOPins(wiring.PureInterface):
    class Signature(wiring.Signature):
        def __init__(self, width):
            super().__init__({
                "o":  Out(unsigned(width)),
                "oe": Out(unsigned(width)),
                "i":  In(unsigned(width)),
            })

        def create(self, *, path=()):
            return GPIOPins(path=path)

    def __init__(self, width, *, path=()):
        super().__init__(self.Signature(width), path=path)


class GPIOPeripheral(wiring.Component):
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

    """Simple GPIO peripheral.

    All pins default to input at power up.
    """
    def __init__(self, *, name, pins):
        if len(pins.o) > 32:
            raise ValueError(f"Pin width must be lesser than or equal to 32, not {len(pins.o)}")

        self.width = len(pins.o)
        self.pins  = pins

        regs = csr.Builder(addr_width=4, data_width=8, name=name)

        self._do = regs.add("do", self.DO(self.width), offset=0x0)
        self._oe = regs.add("oe", self.OE(self.width), offset=0x4)
        self._di = regs.add("di", self.DI(self.width), offset=0x8)

        self._bridge = csr.Bridge(regs.as_memory_map())

        super().__init__({
            "bus": In(csr.Signature(addr_width=regs.addr_width, data_width=regs.data_width)),
        })
        self.bus.memory_map = self._bridge.bus.memory_map

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        connect(m, flipped(self.bus), self._bridge.bus)

        m.d.comb += [
            self.pins.o .eq(self._do.f.pins.data),
            self.pins.oe.eq(self._oe.f.pins.data),
        ]
        m.d.sync += self._di.f.pins.r_data.eq(self.pins.i)

        return m
