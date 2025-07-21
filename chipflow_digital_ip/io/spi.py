from amaranth import Module, Signal, Cat, C, unsigned
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, connect, flipped

from amaranth_soc import csr
from chipflow_lib.platforms import InputIOSignature, OutputIOSignature

__all__ = ["SPISignature", "SPIPeripheral"]

SPISignature = wiring.Signature({
    "sck": Out(OutputIOSignature(1)),
    "copi": Out(OutputIOSignature(1)),
    "cipo": Out(InputIOSignature(1)),
    "csn": Out(OutputIOSignature(1)),
})

class SPIController(wiring.Component):
    def __init__(self):
        super().__init__({
            "spi": Out(SPISignature),
            "sck_idle": In(1),
            "sck_edge": In(1),
            "cs": In(1),
            "start_xfer": In(1),
            "width": In(5),
            "divider": In(8),
            "d_send": In(32),
            "d_recv": Out(32),
            "busy": Out(1),
            "done": Out(1),
        })

    def elaborate(self, platform):
        m = Module()
        sck = Signal()

        setup = Signal()
        latch = Signal()

        div_ctr = Signal(8)
        sr_o = Signal(32)
        sr_i = Signal(32)
        bit_count = Signal(6)

        m.d.comb += [
            self.spi.sck.o.eq(sck ^ self.sck_idle),
            self.spi.csn.o.eq(~self.cs),
        ]

        # defaults for strobes
        m.d.sync += [
            self.done.eq(0),
        ]

        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.start_xfer):
                    m.next = "ACTIVE"
                    m.d.sync += [
                        sr_i.eq(0),
                        sr_o.eq(self.d_send),
                    ]
                m.d.sync += [
                    bit_count.eq(0),
                    div_ctr.eq(0),
                    sck.eq(0),
                ]
            with m.State("ACTIVE"):
                with m.If(div_ctr == self.divider):
                    with m.If(sck):
                        # second half phase, SCK about to fall
                        with m.If(bit_count == self.width):
                            m.next = "DONE"
                        with m.Else():
                            m.d.sync += bit_count.eq(bit_count + 1)
                        m.d.comb += [
                            setup.eq(self.sck_edge),
                            latch.eq(~self.sck_edge),
                        ]
                    with m.Else():
                        # first half phase, SCK about to rise
                        m.d.comb += [
                            setup.eq(~self.sck_edge),
                            latch.eq(self.sck_edge),
                        ]
                    m.d.sync += [
                        sck.eq(~sck),
                        div_ctr.eq(0),
                    ]
                with m.Else():
                    m.d.sync += div_ctr.eq(div_ctr + 1)
                m.d.comb += self.busy.eq(1)
            with m.State("DONE"):
                # one extra clock cycle of busy, because on some phases it might take that to latch
                m.d.comb += self.busy.eq(1)
                m.d.sync += self.done.eq(1)
                m.next = "IDLE"

        # shift registers
        with m.If(setup):
            m.d.sync += sr_o.eq(Cat(C(0, 1), sr_o))
        with m.If(latch):
            m.d.sync += sr_i.eq(Cat(self.spi.cipo.i, sr_i))

        m.d.comb += [
            self.d_recv.eq(sr_i),
            self.spi.copi.o.eq(sr_o[-1])
        ]

        return m


class SPIPeripheral(wiring.Component):
    """
    A custom, minimal SPI controller

    Transfers up to a width of 32 bits are supported, as are all SPI modes.
    """

    class Config(csr.Register, access="rw"):
        """Config register.

        This :class:`Register` is written to in order to configure the SPI mode and transaction.

        It has the following fields:

        .. bitfield::
            :bits: 8

                [
                    { "name": "sck_idle", "bits": 1, "attr": "RW" },
                    { "name": "sck_edge", "bits": 1, "attr": "RW" },
                    { "name": "chip_select", "bits": 1, "attr": "RW" },
                    { "name": "width", "bits": 5, "attr": "RW" },
                ]

        - The ``sck_idle`` field is used to invert the SCK polarity, ``0b0`` for an idle low SCK, ``0b1`` for an idle high SCK
        - The ``sck_edge`` field selects which edge is used for input and output data.
            - ``0b0`` to read input on rising SCK edge, latch output on falling SCK edge
            - ``0b1`` to latch output on rising SCK edge, read input on falling SCK edge
        - The ``chip_select`` field controls the CS pin; setting it to ``0b1`` asserts (brings low) chip select.
        - The ``width`` field configures the width of the transfer. It is set to the width of the transfer minus 1,
        ``31`` gives the maximum width of 32.
        """
        sck_idle: csr.Field(csr.action.RW, unsigned(1))
        sck_edge: csr.Field(csr.action.RW, unsigned(1))
        chip_select: csr.Field(csr.action.RW, unsigned(1))
        width: csr.Field(csr.action.RW, unsigned(5))

    class Divider(csr.Register, access="rw"):
        """Divider register.

        This :class:`Register` is used to configure the clock frequency of the I2C peripheral.

        The SCK frequency is the input clock frequency divided by 4 times the value in this register.
        For example, for a SCK of 1/12 the system clock, this register would be set to 3.
        """
        val: csr.Field(csr.action.RW, unsigned(8))

    class SendData(csr.Register, access="w"):
        """SendData register.

        Writing to this :class:`Register` starts a read/write transfer on the SPI bus, the width of which is configured in `Config`.

        It has the following fields:

        .. bitfield::
            :bits: 32

                [
                    { "name": "val", "bits": 32, "attr": "W" },
                ]

        - The `val` field must be left-justified, so for a transfer of ``N`` bits, bits ``[31..32-N]`` are used.
        """

        val: csr.Field(csr.action.W, unsigned(32))

    class ReceiveData(csr.Register, access="r"):
        """ReceiveData register.

        This :class:`Register` contains the read data of the last transfer started with a write to ``SendData``.

        It has the following fields:

        .. bitfield::
            :bits: 8

                [
                    { "name": "val", "bits": 32, "attr": "R" },
                ]

        - The `val` field is right-justified, so for a transfer of ``N`` bits, bits ``[N-1..0]`` are used.
        """
        val: csr.Field(csr.action.R, unsigned(32))

    class Status(csr.Register, access="r"):
        """Status register.

        This :class:`Register` contains the status of the peripheral.

        It has the following fields:

        .. bitfield::
            :bits: 8

                [
                    { "name": "recv_full", "bits": 1, "attr": "R" },
                    { "bits": 7, "attr": "ResR0" },
                ]

        - The ``recv_full`` field is set to ``0b1`` when a transfer is completed. It is reset to zero by reading ``ReceiveData``.
        """
        recv_full: csr.Field(csr.action.R, unsigned(1))

    def __init__(self):
        regs = csr.Builder(addr_width=5, data_width=8)

        self._config       = regs.add("config",       self.Config(),      offset=0x00)
        self._divider      = regs.add("divider",      self.Divider(),     offset=0x04)
        self._send_data    = regs.add("send_data",    self.SendData(),    offset=0x08)
        self._receive_data = regs.add("receive_data", self.ReceiveData(), offset=0x0C)
        self._status       = regs.add("status",       self.Status(),      offset=0x10)

        self._bridge = csr.Bridge(regs.as_memory_map())

        super().__init__({
            "spi_pins": Out(SPISignature),
            "bus": In(csr.Signature(addr_width=regs.addr_width, data_width=regs.data_width)),
        })
        self.bus.memory_map = self._bridge.bus.memory_map

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        connect(m, flipped(self.bus), self._bridge.bus)

        m.submodules.spi = spi = SPIController()
        connect(m, flipped(self.spi_pins), spi.spi)

        with m.If(self._receive_data.f.val.r_stb):
            m.d.sync += self._status.f.recv_full.r_data.eq(0)
        with m.Elif(spi.done):
            m.d.sync += self._status.f.recv_full.r_data.eq(1)

        m.d.comb += [
            spi.sck_idle.eq(self._config.f.sck_idle.data),
            spi.sck_edge.eq(self._config.f.sck_edge.data),
            spi.cs.eq(self._config.f.chip_select.data),
            spi.width.eq(self._config.f.width.data),
            spi.divider.eq(self._divider.f.val.data),
            spi.start_xfer.eq(self._send_data.f.val.w_stb),
            spi.d_send.eq(self._send_data.f.val.w_data),
            self._receive_data.f.val.r_data.eq(spi.d_recv),
        ]

        return m
