from pathlib import Path

from amaranth import *
from amaranth.lib import coding, wiring
from amaranth.lib.wiring import In, Out, connect, flipped
from amaranth.utils import exact_log2

from amaranth_soc import csr, wishbone
from amaranth_soc.memory import MemoryMap

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
    class _ControlBridge(wiring.Component):
        bus: In(csr.Signature(addr_width=exact_log2(4), data_width=8))
        cfgreg_we: Out(unsigned(4))
        cfgreg_di: Out(unsigned(32))
        cfgreg_do: In(unsigned(32))

        def elaborate(self, platform):
            m = Module()
            m.submodules.addr_dec = addr_dec = coding.Decoder(width=4)
            m.d.comb += addr_dec.i.eq(self.bus.addr)
            m.d.sync += [
                self.cfgreg_we.eq(Mux(self.bus.w_stb, addr_dec.o, 0)),
                self.cfgreg_di.eq(self.bus.w_data.replicate(4)),
                # The CSR bus interface must output zero when idle.
                self.bus.r_data.eq(Mux(self.bus.r_stb,
                                       self.cfgreg_do.word_select(self.bus.addr, 8),
                                       0)),
            ]
            return m

    """A wrapper around the memory-mapped SPI flash interface from picosoc,
    suitable for XIP.

    - ctrl_bus is the original 32-bit control register
    - data_bus is a bus peripheral that directly maps the 16MB of read-only flash memory.
    """
    def __init__(self, *, name, flash):
        self.flash = flash
        self.size  = 2**24
        size_words = (self.size * 8) // 32

        super().__init__({
            "ctrl_bus": In(csr.Signature(addr_width=exact_log2(4), data_width=8)),
            "data_bus": In(wishbone.Signature(addr_width=exact_log2(size_words), data_width=32,
                                              granularity=8)),
        })

        ctrl_memory_map = MemoryMap(addr_width=exact_log2(4), data_width=8)
        ctrl_memory_map.add_resource(name=(name,), size=4, resource=self)
        self.ctrl_bus.memory_map = ctrl_memory_map

        data_memory_map = MemoryMap(addr_width=exact_log2(self.size), data_width=8)
        data_memory_map.add_resource(name=(name,), size=self.size, resource=self)
        self.data_bus.memory_map = data_memory_map

    def elaborate(self, platform):
        m = Module()
        m.submodules.ctrl_bridge = ctrl_bridge = self._ControlBridge()

        connect(m, flipped(self.ctrl_bus), ctrl_bridge.bus)

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
            i_cfgreg_we=ctrl_bridge.cfgreg_we,
            i_cfgreg_di=ctrl_bridge.cfgreg_di,
            o_cfgreg_do=ctrl_bridge.cfgreg_do,
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
