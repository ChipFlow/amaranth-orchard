from amaranth import *
from amaranth.utils import log2_int

from amaranth_soc import wishbone
from amaranth_soc.memory import MemoryMap

from ..base.peripheral import Peripheral

from pathlib import Path

class QSPIPins(Record):
    def __init__(self):
        layout = [
            ("clk_o", 1),
            ("csn_o", 1),
            ("d_o",   4),
            ("d_oe",  4),
            ("d_i",   4),
        ]
        super().__init__(layout)

class SPIMemIO(Peripheral, Elaboratable):
    """A wrapper around the memory-mapped SPI flash interface from picosoc,
    suitable for XIP.

    data_bus is a bus peripheral that directly maps the 16MB of read-only flash memory.

    ctrl_bus is the original 32-bit control register
    """
    def __init__(self, *, flash, **kwargs):
        super().__init__()

        self.data_bus = wishbone.Interface(addr_width=22,
                                      data_width=32, granularity=8)
        self.flash = flash
        map = MemoryMap(addr_width=24, data_width=8)
        map.add_resource(name="flash", size=2**24, resource=self)
        self.data_bus.memory_map = map

        bank            = self.csr_bank()
        self.ctrl_bus = wishbone.Interface(addr_width=0,
                                      data_width=32, granularity=8)
        map = MemoryMap(addr_width=2, data_width=8)
        map.add_resource(name="flash_ctrl", size=4, resource=self)
        self.ctrl_bus.memory_map = map
        self.size = 2**24

    def elaborate(self, platform):
        m = Module()
        spi_ready = Signal()
        # TODO : QSPI
        oe = Signal(4)
        cfgreg_do = Signal(32)
        ctrl_enable = Signal()

        m.submodules.spimemio = Instance(
            "spimemio",
            i_clk=ClockSignal(),
            i_resetn=~ResetSignal(),
            i_valid=self.data_bus.cyc&self.data_bus.stb,
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
            i_cfgreg_we=self.ctrl_bus.sel & Repl(ctrl_enable & self.ctrl_bus.we, 4),
            i_cfgreg_di=self.ctrl_bus.dat_w,
            o_cfgreg_do=self.ctrl_bus.dat_r,
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

        prev_ctrl = Signal()
        m.d.comb += ctrl_enable.eq(self.ctrl_bus.stb & self.ctrl_bus.cyc & ~self.ctrl_bus.ack)
        m.d.sync += self.ctrl_bus.ack.eq(ctrl_enable)

        path = Path(__file__).parent / f"verilog/spimemio.v"
        with open(path, 'r') as f:
            platform.add_file(path.name, f)

        return m
