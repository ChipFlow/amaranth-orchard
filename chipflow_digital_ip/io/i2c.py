from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out, connect, flipped

from amaranth_soc import csr
from chipflow_lib.platforms import BidirIOSignature
from .glasgow_i2c import I2CInitiator

__all__ = ["I2CPeripheral", "I2CSignature"]

I2CSignature = wiring.Signature({
    "scl": Out(BidirIOSignature(1)),
    "sda": Out(BidirIOSignature(1))
    })


class I2CPeripheral(wiring.Component):
    """
    A minimal I2C controller wrapping the Glasgow core

    Support for customisable clock frequency and byte-wise transfers.
    """

    class Divider(csr.Register, access="rw"):
        """Divider register.

        This :class:`Register` is used to configure the clock frequency of the I2C peripheral.

        The SCK frequency is the input clock frequency divided by 4 times the value in this register.
        For example, for a SCK of 1/12 the system clock, this register would be set to 3.
        """
        val: csr.Field(csr.action.RW, unsigned(12))

    class Action(csr.Register, access="w"):
        """Action register.

        This :class:`Register` is written to in order to perform various actions on the I2C bus.
        Writing ``0b1`` to a field performs that action, it is only valid to set one field at a time.

        It has the following fields:

        .. bitfield::
            :bits: 8

                [
                    { "name": "reset", "bits": 1, "attr": "W" },
                    { "name": "start", "bits": 1, "attr": "W" },
                    { "name": "stop", "bits": 1, "attr": "W" },
                    { "name": "read_ack", "bits": 1, "attr": "W" },
                    { "name": "read_nack", "bits": 1, "attr": "W" },
                    { "bits": 3, "attr": "ResR0W0" },
                ]

        - The ``reset`` field is used to reset the PHY in case of a lock-up (e.g. SCK stuck low)
        - The ``start`` field sends an I2C start
        - The ``stop`` field sends an I2C stop
        - The ``read_ack`` field begins an I2C read, followed by an ACK
        - The ``read_nack`` field begins an I2C read, followed by a NACK
        """
        reset: csr.Field(csr.action.W, unsigned(1))
        start: csr.Field(csr.action.W, unsigned(1))
        stop: csr.Field(csr.action.W, unsigned(1))
        read_ack: csr.Field(csr.action.W, unsigned(1))
        read_nack: csr.Field(csr.action.W, unsigned(1))


    class SendData(csr.Register, access="w"):
        """SendData register.

        Writing to this :class:`Register` sends a byte on the I2C bus. 

        It has the following fields:

        .. bitfield::
            :bits: 8

                [
                    { "name": "val", "bits": 8, "attr": "W" },
                ]

        """
        val: csr.Field(csr.action.W, unsigned(8))

    class ReceiveData(csr.Register, access="r"):
        """ReceiveData register.

        This :class:`Register` contains the result of the last read started using `read_ack` or `read_nack`.

        It has the following fields:

        .. bitfield::
            :bits: 8

                [
                    { "name": "val", "bits": 8, "attr": "R" },
                ]

        """
        val: csr.Field(csr.action.R, unsigned(8))

    class Status(csr.Register, access="r"):
        """Status register.

        This :class:`Register` contains the status of the peripheral.

        It has the following fields:

        .. bitfield::
            :bits: 8

                [
                    { "name": "busy", "bits": 1, "attr": "R" },
                    { "name": "ack", "bits": 1, "attr": "R" },
                    { "bits": 6, "attr": "ResR0" },
                ]

        - The ``busy`` field is set when the PHY is currently performing an action, and unable to accept any requests.
        - The ``ack`` field contains the ACK/NACK value of the last write.
        """
        busy: csr.Field(csr.action.R, unsigned(1))
        ack: csr.Field(csr.action.R, unsigned(1))

    def __init__(self):
        regs = csr.Builder(addr_width=5, data_width=8)

        self._divider      = regs.add("divider",      self.Divider(),     offset=0x00)
        self._action       = regs.add("action",       self.Action(),      offset=0x04)
        self._send_data    = regs.add("send_data",    self.SendData(),    offset=0x08)
        self._receive_data = regs.add("receive_data", self.ReceiveData(), offset=0x0C)
        self._status       = regs.add("status",       self.Status(),      offset=0x10)

        self._bridge = csr.Bridge(regs.as_memory_map())

        super().__init__({
            "i2c_pins": Out(I2CSignature),
            "bus": In(csr.Signature(addr_width=regs.addr_width, data_width=regs.data_width)),
        })
        self.bus.memory_map = self._bridge.bus.memory_map

    def elaborate(self, platform):
        m = Module()

        m.submodules.bridge = self._bridge

        connect(m, flipped(self.bus), self._bridge.bus)

        i2c_rst = Signal()

        m.submodules.i2c = i2c = ResetInserter(i2c_rst)(I2CInitiator(self.i2c_pins))

        m.d.comb += [
            i2c.period_cyc.eq(self._divider.f.val.data),

            i2c_rst.eq(self._action.f.reset.w_data & self._action.f.reset.w_stb),
            i2c.start.eq(self._action.f.start.w_data & self._action.f.start.w_stb),
            i2c.stop.eq(self._action.f.stop.w_data & self._action.f.stop.w_stb),
            i2c.read.eq((self._action.f.read_ack.w_data & self._action.f.read_ack.w_stb) | \
                (self._action.f.read_nack.w_data & self._action.f.read_nack.w_stb)),
            i2c.ack_i.eq(self._action.f.read_ack.w_data),

            i2c.data_i.eq(self._send_data.f.val.w_data),
            i2c.write.eq(self._send_data.f.val.w_stb),

            self._receive_data.f.val.r_data.eq(i2c.data_o),

            self._status.f.busy.r_data.eq(i2c.busy),
            self._status.f.ack.r_data.eq(i2c.ack_o),
        ]
        return m
