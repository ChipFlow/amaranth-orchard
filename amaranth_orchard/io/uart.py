from amaranth import *

from amaranth_stdio.serial import AsyncSerialRX, AsyncSerialTX

from ..base.peripheral import Peripheral

class UARTPins(Record):
    def __init__(self):
        layout = [
            ("tx_o", 1),
            ("rx_i", 1),
        ]
        super().__init__(layout)

class UARTPeripheral(Peripheral, Elaboratable):
    """
    A custom, minimal UART. TODO: Interrupts support, perhaps mimic something with upstream Linux kernel support...

    CSRs:
        tx_data: valid to write to when tx_rdy is 1, will trigger a transmit
        rx_data: valid to read from when rx_avail is 1, last received byte
        tx_rdy: is '1' when 1-byte transmit buffer is empty
        rx_avail: is '1' when 1-byte receive buffer is full; reset by a read from rx_data
        divisor: baud divider, defaults to init_diviser
    """
    def __init__(self, init_divisor, pins, **kwargs):
        super().__init__()

        self.init_divisor = init_divisor
        self.pins = pins

        bank            = self.csr_bank(addr_width=5)
        self.tx_data    = bank.csr(8, "w")
        self.rx_data    = bank.csr(8, "r")

        self.tx_rdy     = bank.csr(1, "r")
        self.rx_avail   = bank.csr(1, "r")

        self.divisor    = bank.csr(24, "rw")

        self._bridge    = self.bridge(addr_width=3, data_width=32, granularity=8, alignment=2)
        self.bus        = self._bridge.bus

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge  = self._bridge

        m.submodules.tx = tx = AsyncSerialTX(divisor=self.init_divisor)
        m.d.comb += [
            self.pins.tx_o.eq(tx.o),
            tx.data.eq(self.tx_data.w_data),
            tx.ack.eq(self.tx_data.w_stb),
            self.tx_rdy.r_data.eq(tx.rdy),
            self.divisor.r_data.eq(tx.divisor),
        ]

        with m.If(self.divisor.w_stb.any()):
            m.d.sync += tx.divisor.eq(self.divisor.w_data)

        rx_buf = Signal(8)
        rx_avail = Signal()

        m.submodules.rx = rx = AsyncSerialRX(divisor=self.init_divisor)

        with m.If(self.rx_data.r_stb):
            m.d.sync += rx_avail.eq(0)

        with m.If(rx.rdy):
            m.d.sync += [
                rx_buf.eq(rx.data),
                rx_avail.eq(1)
            ]

        m.d.comb += [
            rx.i.eq(self.pins.rx_i),
            rx.ack.eq(~rx_avail),
            rx.divisor.eq(tx.divisor),
            self.rx_data.r_data.eq(rx_buf),
            self.rx_avail.r_data.eq(rx_avail)
        ]

        return m
