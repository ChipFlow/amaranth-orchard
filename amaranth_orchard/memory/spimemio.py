from pathlib import Path

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, connect, flipped
from amaranth.utils import exact_log2

from amaranth_soc import csr, wishbone
from amaranth_soc.memory import MemoryMap
from chipflow_lib.platforms import BidirPinSignature,OutputPinSignature

__all__ = ["QSPIPins", "SPIMemIO"]


class QSPIPins(wiring.PureInterface):
    class Signature(wiring.Signature):
        def __init__(self):
            super().__init__({
                "clk": Out(OutputPinSignature(1)),
                "csn": Out(OutputPinSignature(1)),
                "d": Out(BidirPinSignature(4, all_have_oe=True)),
            })

        def create(self, *, path=(), src_loc_at=0):
            return QSPIPins(path=path, src_loc_at=1 + src_loc_at)

    def __init__(self, *, path=(), src_loc_at=0):
        super().__init__(self.Signature(), path=path, src_loc_at=1 + src_loc_at)


class SPIMemIO(wiring.Component):
    class _ControlBridge(wiring.Component):
        bus: In(csr.Signature(addr_width=exact_log2(4), data_width=8))
        cfgreg_we: Out(unsigned(4))
        cfgreg_di: Out(unsigned(32))
        cfgreg_do: In(unsigned(32))

        def elaborate(self, platform):
            m = Module()
            cfgreg_we_next = Signal.like(self.cfgreg_we)
            m.d.comb += cfgreg_we_next.bit_select(self.bus.addr, 1).eq(1)
            m.d.sync += [
                self.cfgreg_we.eq(Mux(self.bus.w_stb, cfgreg_we_next, 0)),
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

    def __init__(self, mem_name=("mem",), cfg_name=("cfg",), *, flash):
        self.flash = flash
        self.size  = 2**24
        size_words = (self.size * 8) // 32

        super().__init__({
            "qspi": Out(QSPIPins.Signature()),
            "ctrl_bus": In(csr.Signature(addr_width=exact_log2(4), data_width=8)),
            "data_bus": In(wishbone.Signature(addr_width=exact_log2(size_words), data_width=32,
                                              granularity=8)),
        })

        ctrl_memory_map = MemoryMap(addr_width=exact_log2(4), data_width=8)
        ctrl_memory_map.add_resource(name=cfg_name, size=4, resource=self)
        self.ctrl_bus.memory_map = ctrl_memory_map

        data_memory_map = MemoryMap(addr_width=exact_log2(self.size), data_width=8)
        data_memory_map.add_resource(name=mem_name, size=self.size, resource=self)
        self.data_bus.memory_map = data_memory_map

    def elaborate(self, platform):
        m = Module()
        m.submodules.ctrl_bridge = ctrl_bridge = self._ControlBridge()

        connect(m, flipped(self.ctrl_bus), ctrl_bridge.bus)

        spi_ready = Signal()
        # TODO : QSPI

        verilog_map = {
            "i_clk": ClockSignal(),
            "i_resetn": ~ResetSignal(),
            "i_valid": self.data_bus.cyc & self.data_bus.stb,
            "o_ready": spi_ready,
            "i_addr": Cat(Const(0, 2), self.data_bus.adr), # Hack to force a 1MB offset
            "o_rdata": self.data_bus.dat_r,
            "o_flash_csb": self.qspi.csn.o,
            "o_flash_clk": self.qspi.clk.o,
            "i_cfgreg_we": ctrl_bridge.cfgreg_we,
            "i_cfgreg_di": ctrl_bridge.cfgreg_di,
            "o_cfgreg_do": ctrl_bridge.cfgreg_do,
        } | {
            f"o_flash_io{n}_oe": self.qspi.d.oe[n] for n in range(4)
        } | {
            f"o_flash_io{n}_do": self.qspi.d.o[n] for n in range(4)
        } | {
            f"o_flash_io{n}_di": self.qspi.d.i[n] for n in range(4)
        }
 
        m.submodules.spimemio = Instance("spimemio", **verilog_map)

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

        if platform is not None:
            path = Path(__file__).parent / "verilog/spimemio.v"
            with open(path, 'r') as f:
                platform.add_file(path.name, f)

        return m
