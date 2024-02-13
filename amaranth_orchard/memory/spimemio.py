from pathlib import Path

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, connect, flipped
from amaranth.utils import exact_log2

from amaranth_soc import csr, wishbone
from amaranth_soc.memory import MemoryMap

from ..base.field import StorageLessRW


__all__ = ["QSPIPins", "SPIMemIO"]


class QSPIPins(wiring.PureInterface):
    class Signature(wiring.Signature):
        def __init__(self):
            super().__init__({
                "clk_o": Out(1),
                "csn_o": Out(1),
                "d_o":   Out(4),
                "d_oe":  Out(4),
                "d_i":   In(4),
            })

        def create(self, *, path=()):
            return QSPIPins(path=path)

    def __init__(self, *, path=()):
        super().__init__(self.Signature(), path=path)


class SPIMemIO(wiring.Component):
    class ConfigReg(csr.Register, access="rw"):
        val: csr.Field(StorageLessRW, unsigned(8))

    """A wrapper around the memory-mapped SPI flash interface from picosoc,
    suitable for XIP.

    - ctrl_bus is the original 32-bit control register
    - data_bus is a bus peripheral that directly maps the 16MB of read-only flash memory.
    """
    def __init__(self, *, name, flash):
        self.flash = flash
        self.size  = 2**24
        size_words = (self.size * 8) // 32

        regs = csr.Builder(addr_width=2, data_width=8, name=name)

        self._cfgreg = {}
        for n in range(4):
            with regs.Index(n):
                self._cfgreg[n] = regs.add("cfgreg", self.ConfigReg())

        self._bridge = csr.Bridge(regs.as_memory_map())

        data_memory_map = MemoryMap(addr_width=exact_log2(self.size), data_width=8)
        data_memory_map.add_resource(name=(name,), size=self.size, resource=self)

        super().__init__({
            "ctrl_bus": In(csr.Signature(addr_width=regs.addr_width, data_width=regs.data_width)),
            "data_bus": In(wishbone.Signature(addr_width=exact_log2(size_words), data_width=32,
                                              granularity=8)),
        })
        self.ctrl_bus.memory_map = self._bridge.bus.memory_map
        self.data_bus.memory_map = data_memory_map

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        connect(m, flipped(self.ctrl_bus), self._bridge.bus)

        spi_ready = Signal()
        # TODO : QSPI

        m.submodules.spimemio = Instance(
            "spimemio",
            i_clk=ClockSignal(),
            i_resetn=~ResetSignal(),
            i_valid=self.data_bus.cyc & self.data_bus.stb,
            o_ready=spi_ready,
            i_addr=Cat(Const(0, 2), self.data_bus.adr), # Hack to force a 1MB offset
            o_rdata=self.data_bus.dat_r,
            o_flash_csb=self.flash.csn_o,
            o_flash_clk=self.flash.clk_o,
            o_flash_io0_oe=self.flash.d_oe[0],
            o_flash_io1_oe=self.flash.d_oe[1],
            o_flash_io2_oe=self.flash.d_oe[2],
            o_flash_io3_oe=self.flash.d_oe[3],
            o_flash_io0_do=self.flash.d_o[0],
            o_flash_io1_do=self.flash.d_o[1],
            o_flash_io2_do=self.flash.d_o[2],
            o_flash_io3_do=self.flash.d_o[3],
            i_flash_io0_di=self.flash.d_i[0],
            i_flash_io1_di=self.flash.d_i[1],
            i_flash_io2_di=self.flash.d_i[2],
            i_flash_io3_di=self.flash.d_i[3],
            i_cfgreg_we=Cat(self._cfgreg[n].f.val.w_stb  for n in range(4)),
            i_cfgreg_di=Cat(self._cfgreg[n].f.val.w_data for n in range(4)),
            o_cfgreg_do=Cat(self._cfgreg[n].f.val.r_data for n in range(4)),
        )
        # From https://github.com/im-tomu/foboot/blob/master/hw/rtl/picorvspi.py
        read_active = Signal()
        with m.If(self.data_bus.stb & self.data_bus.cyc & ~read_active):
            m.d.sync += read_active.eq(1)
            m.d.sync += self.data_bus.ack.eq(0)
        with m.Elif(read_active & spi_ready):
            m.d.sync += read_active.eq(0)
            m.d.sync += self.data_bus.ack.eq(1)
        with m.Else():
            m.d.sync += self.data_bus.ack.eq(0)

        path = Path(__file__).parent / f"verilog/spimemio.v"
        with open(path, 'r') as f:
            platform.add_file(path.name, f)

        return m
