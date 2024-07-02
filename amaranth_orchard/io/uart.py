from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, flipped, connect

from amaranth_soc import csr
from amaranth_stdio.serial import AsyncSerialRX, AsyncSerialTX


__all__ = ["UARTPins", "UARTPeripheral"]


class UARTPins(wiring.PureInterface):
    class Signature(wiring.Signature):
        def __init__(self):
            super().__init__({
                "tx_o": Out(1),
                "rx_i": In(1),
            })

        def create(self, *, path=(), src_loc_at=0):
            return UARTPins(path=path, src_loc_at=1 + src_loc_at)

    def __init__(self, *, path=(), src_loc_at=0):
        super().__init__(self.Signature(), path=path, src_loc_at=1 + src_loc_at)


class UARTPeripheral(wiring.Component):
    class TxData(csr.Register, access="w"):
        """valid to write to when tx_rdy is 1, will trigger a transmit"""
        val: csr.Field(csr.action.W, unsigned(8))

    class RxData(csr.Register, access="r"):
        """valid to read from when rx_avail is 1, last received byte"""
        val: csr.Field(csr.action.R, unsigned(8))

    class TxReady(csr.Register, access="r"):
        """is '1' when 1-byte transmit buffer is empty"""
        val: csr.Field(csr.action.R, unsigned(1))

    class RxAvail(csr.Register, access="r"):
        """is '1' when 1-byte receive buffer is full; reset by a read from rx_data"""
        val: csr.Field(csr.action.R, unsigned(1))

    class Divisor(csr.Register, access="rw"):
        """baud divider, defaults to init_divisor"""
        def __init__(self, init_divisor):
            super().__init__({
                "val": csr.Field(csr.action.RW, unsigned(24), init=init_divisor),
            })

    """
    A custom, minimal UART.

    TODO: Interrupts support, perhaps mimic something with upstream Linux kernel support...
    """
    def __init__(self, *, init_divisor, pins):
        self.init_divisor = init_divisor
        self.pins = pins

        regs = csr.Builder(addr_width=5, data_width=8)

        self._tx_data  = regs.add("tx_data",  self.TxData(),  offset=0x00)
        self._rx_data  = regs.add("rx_data",  self.RxData(),  offset=0x04)
        self._tx_rdy   = regs.add("tx_rdy",   self.TxReady(), offset=0x08)
        self._rx_avail = regs.add("rx_avail", self.RxAvail(), offset=0x0c)
        self._divisor  = regs.add("divisor",  self.Divisor(init_divisor), offset=0x10)

        self._bridge = csr.Bridge(regs.as_memory_map())

        super().__init__({
            "bus": In(csr.Signature(addr_width=regs.addr_width, data_width=regs.data_width)),
        })
        self.bus.memory_map = self._bridge.bus.memory_map

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        connect(m, flipped(self.bus), self._bridge.bus)

        m.submodules.tx = tx = AsyncSerialTX(divisor=self.init_divisor, divisor_bits=24)
        m.d.comb += [
            self.pins.tx_o.eq(tx.o),
            tx.data.eq(self._tx_data.f.val.w_data),
            tx.ack.eq(self._tx_data.f.val.w_stb),
            self._tx_rdy.f.val.r_data.eq(tx.rdy),
            tx.divisor.eq(self._divisor.f.val.data)
        ]

        rx_buf = Signal(unsigned(8))
        rx_avail = Signal()

        m.submodules.rx = rx = AsyncSerialRX(divisor=self.init_divisor, divisor_bits=24)

        with m.If(self._rx_data.f.val.r_stb):
            m.d.sync += rx_avail.eq(0)

        with m.If(rx.rdy):
            m.d.sync += [
                rx_buf.eq(rx.data),
                rx_avail.eq(1)
            ]

        m.d.comb += [
            rx.i.eq(self.pins.rx_i),
            rx.ack.eq(~rx_avail),
            rx.divisor.eq(self._divisor.f.val.data),
            self._rx_data.f.val.r_data.eq(rx_buf),
            self._rx_avail.f.val.r_data.eq(rx_avail)
        ]

        return m
