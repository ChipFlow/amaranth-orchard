from amaranth import *
from amaranth.lib.cdc import FFSynchronizer


# I2C Controller Implementation from Glasgow
# https://github.com/GlasgowEmbedded/glasgow/blob/5e1f94dde6896e3adf2e8ebcfc0157d1fb950823/software/glasgow/gateware/i2c.py

class I2CBus(Elaboratable):
    """
    I2C bus.

    Decodes bus conditions (start, stop, sample and setup) and provides synchronization.
    """
    def __init__(self, pins):

        self.pins = pins

        self.scl_i = Signal()
        self.scl_o = Signal(init=1)
        self.sda_i = Signal()
        self.sda_o = Signal(init=1)

        self.sample = Signal(name="bus_sample")
        self.setup  = Signal(name="bus_setup")
        self.start  = Signal(name="bus_start")
        self.stop   = Signal(name="bus_stop")

    def elaborate(self, platform):
        m = Module()

        scl_r = Signal(init=1)
        sda_r = Signal(init=1)

        m.d.comb += [
            self.pins.scl.o.eq(0),
            self.pins.scl.oe.eq(~self.scl_o),
            self.pins.sda.o.eq(0),
            self.pins.sda.oe.eq(~self.sda_o),

            self.sample.eq(~scl_r & self.scl_i),
            self.setup.eq(scl_r & ~self.scl_i),
            self.start.eq(self.scl_i & sda_r & ~self.sda_i),
            self.stop.eq(self.scl_i & ~sda_r & self.sda_i),
        ]
        m.d.sync += [
            scl_r.eq(self.scl_i),
            sda_r.eq(self.sda_i),
        ]
        m.submodules += [
            FFSynchronizer(self.pins.scl.i, self.scl_i, init=1),
            FFSynchronizer(self.pins.sda.i, self.sda_i, init=1),
        ]

        return m


class I2CInitiator(Elaboratable):
    """
    Simple I2C transaction initiator.

    Generates start and stop conditions, and transmits and receives octets.
    Clock stretching is supported.

    :param period_cyc:
        Bus clock period, as a multiple of system clock period.
    :type period_cyc: int
    :param clk_stretch:
        If true, SCL will be monitored for devices stretching the clock. Otherwise,
        only internally generated SCL is considered.
    :type clk_stretch: bool

    :attr busy:
        Busy flag. Low if the state machine is idle, high otherwise.
    :attr start:
        Start strobe. When ``busy`` is low, asserting ``start`` for one cycle generates
        a start or repeated start condition on the bus. Ignored when ``busy`` is high.
    :attr stop:
        Stop strobe. When ``busy`` is low, asserting ``stop`` for one cycle generates
        a stop condition on the bus. Ignored when ``busy`` is high.
    :attr write:
        Write strobe. When ``busy`` is low, asserting ``write`` for one cycle receives
        an octet on the bus and latches it to ``data_o``, after which the acknowledge bit
        is asserted if ``ack_i`` is high. Ignored when ``busy`` is high.
    :attr data_i:
        Data octet to be transmitted. Latched immediately after ``write`` is asserted.
    :attr ack_o:
        Received acknowledge bit.
    :attr read:
        Read strobe. When ``busy`` is low, asserting ``read`` for one cycle latches
        ``data_i`` and transmits it on the bus, after which the acknowledge bit
        from the bus is latched to ``ack_o``. Ignored when ``busy`` is high.
    :attr data_o:
        Received data octet.
    :attr ack_i:
        Acknowledge bit to be transmitted. Latched immediately after ``read`` is asserted.
    """
    def __init__(self, pins, clk_stretch=True):
        self.clk_stretch = clk_stretch

        self.period_cyc = Signal(12)

        self.busy   = Signal(init=1)
        self.start  = Signal()
        self.stop   = Signal()
        self.read   = Signal()
        self.data_i = Signal(8)
        self.ack_o  = Signal()
        self.write  = Signal()
        self.data_o = Signal(8)
        self.ack_i  = Signal()

        self.bus = I2CBus(pins)

    def elaborate(self, platform):
        m = Module()

        m.submodules.bus = self.bus

        timer = Signal(12)
        stb   = Signal()

        with m.If((timer == 0) | ~self.busy):
            m.d.sync += timer.eq(self.period_cyc)
        with m.Elif((not self.clk_stretch) | (self.bus.scl_o == self.bus.scl_i)):
            m.d.sync += timer.eq(timer - 1)
        m.d.comb += stb.eq(timer == 0)

        bitno   = Signal(range(8))
        r_shreg = Signal(8)
        w_shreg = Signal(8)
        r_ack   = Signal()

        with m.FSM() as fsm:
            self._fsm = fsm
            def scl_l(state, next_state, *exprs):
                with m.State(state):
                    with m.If(stb):
                        m.d.sync += self.bus.scl_o.eq(0)
                        m.next = next_state
                        m.d.sync += exprs

            def scl_h(state, next_state, *exprs):
                with m.State(state):
                    with m.If(stb):
                        m.d.sync += self.bus.scl_o.eq(1)
                    with m.Elif(self.bus.scl_o == 1):
                        with m.If((not self.clk_stretch) | (self.bus.scl_i == 1)):
                            m.next = next_state
                            m.d.sync += exprs

            def stb_x(state, next_state, *exprs, bit7_next_state=None):
                with m.State(state):
                    with m.If(stb):
                        m.next = next_state
                        if bit7_next_state is not None:
                            with m.If(bitno == 7):
                                m.next = bit7_next_state
                        m.d.sync += exprs

            with m.State("IDLE"):
                m.d.sync += self.busy.eq(1)
                with m.If(self.start):
                    with m.If(self.bus.scl_i & self.bus.sda_i):
                        m.next = "START-SDA-L"
                    with m.Elif(~self.bus.scl_i):
                        m.next = "START-SCL-H"
                    with m.Elif(self.bus.scl_i):
                        m.next = "START-SCL-L"
                with m.Elif(self.stop):
                    with m.If(self.bus.scl_i & ~self.bus.sda_o):
                        m.next = "STOP-SDA-H"
                    with m.Elif(~self.bus.scl_i):
                        m.next = "STOP-SCL-H"
                    with m.Elif(self.bus.scl_i):
                        m.next = "STOP-SCL-L"
                with m.Elif(self.write):
                    m.d.sync += w_shreg.eq(self.data_i)
                    m.next = "WRITE-DATA-SCL-L"
                with m.Elif(self.read):
                    m.d.sync += r_ack.eq(self.ack_i)
                    m.next = "READ-DATA-SCL-L"
                with m.Else():
                    m.d.sync += self.busy.eq(0)

            # start
            scl_l("START-SCL-L", "START-SDA-H")
            stb_x("START-SDA-H", "START-SCL-H",
                self.bus.sda_o.eq(1)
            )
            scl_h("START-SCL-H", "START-SDA-L")
            stb_x("START-SDA-L", "IDLE",
                self.bus.sda_o.eq(0)
            )
            # stop
            scl_l("STOP-SCL-L",  "STOP-SDA-L")
            stb_x("STOP-SDA-L",  "STOP-SCL-H",
                self.bus.sda_o.eq(0)
            )
            scl_h("STOP-SCL-H",  "STOP-SDA-H")
            stb_x("STOP-SDA-H",  "IDLE",
                self.bus.sda_o.eq(1)
            )
            # write data
            scl_l("WRITE-DATA-SCL-L", "WRITE-DATA-SDA-X")
            stb_x("WRITE-DATA-SDA-X", "WRITE-DATA-SCL-H",
                self.bus.sda_o.eq(w_shreg[7])
            )
            scl_h("WRITE-DATA-SCL-H", "WRITE-DATA-SDA-N",
                w_shreg.eq(Cat(C(0, 1), w_shreg[0:7]))
            )
            stb_x("WRITE-DATA-SDA-N", "WRITE-DATA-SCL-L",
                bitno.eq(bitno + 1),
                bit7_next_state="WRITE-ACK-SCL-L"
            )
            # write ack
            scl_l("WRITE-ACK-SCL-L", "WRITE-ACK-SDA-H")
            stb_x("WRITE-ACK-SDA-H", "WRITE-ACK-SCL-H",
                self.bus.sda_o.eq(1)
            )
            scl_h("WRITE-ACK-SCL-H", "WRITE-ACK-SDA-N",
                self.ack_o.eq(~self.bus.sda_i)
            )
            stb_x("WRITE-ACK-SDA-N", "IDLE")
            # read data
            scl_l("READ-DATA-SCL-L", "READ-DATA-SDA-H")
            stb_x("READ-DATA-SDA-H", "READ-DATA-SCL-H",
                self.bus.sda_o.eq(1)
            )
            scl_h("READ-DATA-SCL-H", "READ-DATA-SDA-N",
                r_shreg.eq(Cat(self.bus.sda_i, r_shreg[0:7]))
            )
            stb_x("READ-DATA-SDA-N", "READ-DATA-SCL-L",
                bitno.eq(bitno + 1),
                bit7_next_state="READ-ACK-SCL-L"
            )
            # read ack
            scl_l("READ-ACK-SCL-L", "READ-ACK-SDA-X")
            stb_x("READ-ACK-SDA-X", "READ-ACK-SCL-H",
                self.bus.sda_o.eq(~r_ack)
            )
            scl_h("READ-ACK-SCL-H", "READ-ACK-SDA-N",
                self.data_o.eq(r_shreg)
            )
            stb_x("READ-ACK-SDA-N", "IDLE")

        return m
