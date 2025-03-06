# amaranth: UnusedElaboratable=no
import logging
from pytest import raises

from amaranth import *
from amaranth.sim import *

from amaranth_orchard.io import GPIOPeripheral

logger = logging.getLogger(__name__)


def test_init():
    dut_1 = GPIOPeripheral(pin_count=4, addr_width=2, data_width=8)
    assert dut_1.pin_count == 4
    assert dut_1.input_stages == 2
    assert dut_1.bus.addr_width == 2
    assert dut_1.bus.data_width == 8
    dut_2 = GPIOPeripheral(pin_count=1, addr_width=8, data_width=16, input_stages=3)
    assert dut_2.pin_count == 1
    assert dut_2.input_stages == 3
    assert dut_2.bus.addr_width == 8
    assert dut_2.bus.data_width == 16


def test_init_wrong_pin_count():
    with raises(TypeError, match=r"Pin count must be a positive integer, not 'foo'"):
        GPIOPeripheral(pin_count="foo", addr_width=2, data_width=8)
    with raises(TypeError, match=r"Pin count must be a positive integer, not 0"):
        GPIOPeripheral(pin_count=0, addr_width=2, data_width=8)

def test_init_wrong_input_stages():
    with raises(TypeError, match=r"Input stages must be a non-negative integer, not 'foo'"):
        GPIOPeripheral(pin_count=1, addr_width=2, data_width=8, input_stages="foo")
    with raises(TypeError, match=r"Input stages must be a non-negative integer, not -1"):
        GPIOPeripheral(pin_count=1, addr_width=2, data_width=8, input_stages=-1)


async def _csr_access(ctx, dut, addr, r_stb=0, w_stb=0, w_data=0, r_data=0):
    ctx.set(dut.bus.addr, addr)
    ctx.set(dut.bus.r_stb, r_stb)
    ctx.set(dut.bus.w_stb, w_stb)
    ctx.set(dut.bus.w_data, w_data)
    await ctx.tick()
    if r_stb:
        assert ctx.get(dut.bus.r_data) == r_data
    ctx.set(dut.bus.r_stb, 0)
    ctx.set(dut.bus.w_stb, 0)


def test_sim(caplog):
    caplog.set_level(logging.CRITICAL, logger="root.amaranth_orchard.io.gpio")
    dut = GPIOPeripheral(pin_count=4, addr_width=2, data_width=8)

    mode_addr = 0x0
    input_addr = 0x1
    output_addr = 0x2
    setclr_addr = 0x3

    async def testbench(ctx):
        # INPUT_ONLY mode =====================================================================

        # - read Mode:
        await _csr_access(ctx, dut, mode_addr, r_stb=1, r_data=0b00000000)
        assert ctx.get(dut.alt_mode) == 0b0000
        assert ctx.get(dut.pins.gpio.oe) == 0b0000
        assert ctx.get(dut.pins.gpio.o) == 0b0000

        # - read Input:
        ctx.set(dut.pins.gpio.i[1], 1)
        ctx.set(dut.pins.gpio.i[3], 1)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        ctx.set(dut.pins.gpio.i[1], 0)
        ctx.set(dut.pins.gpio.i[3], 0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0xa)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)

        # - write 0xf to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0x0, w_stb=1, w_data=0xf)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b0000
        assert ctx.get(dut.pins.gpio.o) == 0b1111

        # - write 0x22 to SetClr (clear pins[0] and pins[2]):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0x22)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b0000
        assert ctx.get(dut.pins.gpio.o) == 0b1010

        # - write 0x0 to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0xa, w_stb=1, w_data=0x0)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b0000
        assert ctx.get(dut.pins.gpio.o) == 0b0000

        # - write 0x44 to SetClr (set pins[1] and pins[3]):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0x44)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b0000
        assert ctx.get(dut.pins.gpio.o) == 0b1010

        # - write 0x0 to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0xa, w_stb=1, w_data=0x0)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b0000
        assert ctx.get(dut.pins.gpio.o) == 0b0000

        # - write 0xff to SetClr (no-op):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0xff)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b0000
        assert ctx.get(dut.pins.gpio.o) == 0b0000

        # PUSH_PULL mode ======================================================================

        # - write Mode:
        await _csr_access(ctx, dut, mode_addr, w_stb=1, w_data=0b01010101)
        await ctx.tick()
        assert ctx.get(dut.alt_mode) == 0b0000
        assert ctx.get(dut.pins.gpio.oe) == 0b1111
        assert ctx.get(dut.pins.gpio.o) == 0b0000

        # - read Input:
        ctx.set(dut.pins.gpio.i[1], 1)
        ctx.set(dut.pins.gpio.i[3], 1)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        ctx.set(dut.pins.gpio.i[1], 0)
        ctx.set(dut.pins.gpio.i[3], 0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0xa)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)

        # - write 0xf to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0x0, w_stb=1, w_data=0xf)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b1111
        assert ctx.get(dut.pins.gpio.o) == 0b1111

        # - write 0x22 to SetClr (clear pins[0] and pins[2]):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0x22)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b1111
        assert ctx.get(dut.pins.gpio.o) == 0b1010

        # - write 0x0 to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0xa, w_stb=1, w_data=0x0)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b1111
        assert ctx.get(dut.pins.gpio.o) == 0b0000

        # - write 0x44 to SetClr (set pins[1] and pins[3]):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0x44)
        await ctx.tick()
        assert ctx.get(dut.pins.gpio.oe) == 0b1111
        assert ctx.get(dut.pins.gpio.o) == 0b1010

        """
        # - write 0x0 to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0xa, w_stb=1, w_data=0x0)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 1
            assert ctx.get(dut.pins[n].o) == 0

        # - write 0xff to SetClr (no-op):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0xff)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 1
            assert ctx.get(dut.pins[n].o) == 0

        # OPEN_DRAIN mode =====================================================================

        # - write Mode:
        await _csr_access(ctx, dut, mode_addr, w_stb=1, w_data=0b10101010)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.alt_mode[n]) == 0
            assert ctx.get(dut.pins[n].oe) == 1
            assert ctx.get(dut.pins[n].o) == 0

        # - read Input:
        ctx.set(dut.pins[1].i, 1)
        ctx.set(dut.pins[3].i, 1)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        ctx.set(dut.pins[1].i, 0)
        ctx.set(dut.pins[3].i, 0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0xa)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)

        # - write 0xf to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0x0, w_stb=1, w_data=0xf)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 0
            assert ctx.get(dut.pins[n].o) == 0

        # - write 0x22 to SetClr (clear pins[0] and pins[2]):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0x22)
        await ctx.tick()
        assert ctx.get(dut.pins[0].oe) == 1
        assert ctx.get(dut.pins[1].oe) == 0
        assert ctx.get(dut.pins[2].oe) == 1
        assert ctx.get(dut.pins[3].oe) == 0
        for n in range(4):
            assert ctx.get(dut.pins[n].o) == 0

        # - write 0x0 to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0xa, w_stb=1, w_data=0x0)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 1
            assert ctx.get(dut.pins[n].o) == 0

        # - write 0x44 to SetClr (set pins[1] and pins[3]):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0x44)
        await ctx.tick()
        assert ctx.get(dut.pins[0].oe) == 1
        assert ctx.get(dut.pins[1].oe) == 0
        assert ctx.get(dut.pins[2].oe) == 1
        assert ctx.get(dut.pins[3].oe) == 0
        for n in range(4):
            assert ctx.get(dut.pins[n].o) == 0

        # - write 0x0 to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0xa, w_stb=1, w_data=0x0)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 1
            assert ctx.get(dut.pins[n].o) == 0

        # - write 0xff to SetClr (no-op):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0xff)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 1
            assert ctx.get(dut.pins[n].o) == 0

        # ALTERNATE mode ======================================================================

        # - write Mode:
        await _csr_access(ctx, dut, mode_addr, w_stb=1, w_data=0b11111111)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.alt_mode[n]) == 1
            assert ctx.get(dut.pins[n].oe) == 0
            assert ctx.get(dut.pins[n].o) == 0

        # - read Input:
        ctx.set(dut.pins[1].i, 1)
        ctx.set(dut.pins[3].i, 1)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        ctx.set(dut.pins[1].i, 0)
        ctx.set(dut.pins[3].i, 0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0xa)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)

        # - write 0xf to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0x0, w_stb=1, w_data=0xf)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 0
            assert ctx.get(dut.pins[n].o) == 1

        # - write 0x22 to SetClr (clear pins[0] and pins[2]):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0x22)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 0
        assert ctx.get(dut.pins[0].o) == 0
        assert ctx.get(dut.pins[1].o) == 1
        assert ctx.get(dut.pins[2].o) == 0
        assert ctx.get(dut.pins[3].o) == 1

        # - write 0x0 to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0xa, w_stb=1, w_data=0x0)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 0
            assert ctx.get(dut.pins[n].o) == 0

        # - write 0x44 to SetClr (set pins[1] and pins[3]):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0x44)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 0
        assert ctx.get(dut.pins[0].o) == 0
        assert ctx.get(dut.pins[1].o) == 1
        assert ctx.get(dut.pins[2].o) == 0
        assert ctx.get(dut.pins[3].o) == 1

        # - write 0x0 to Output:
        await _csr_access(ctx, dut, output_addr, r_stb=1, r_data=0xa, w_stb=1, w_data=0x0)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 0
            assert ctx.get(dut.pins[n].o) == 0

        # - write 0xff to SetClr (no-op):
        await _csr_access(ctx, dut, setclr_addr, w_stb=1, w_data=0xff)
        await ctx.tick()
        for n in range(4):
            assert ctx.get(dut.pins[n].oe) == 0
            assert ctx.get(dut.pins[n].o) == 0
        """
    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(testbench)
    with sim.write_vcd(vcd_file="test.vcd"):
        sim.run()


def test_sim_without_input_sync():
    dut = GPIOPeripheral(pin_count=4, addr_width=2, data_width=8, input_stages=0)
    input_addr = 0x1

    async def testbench(ctx):
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)
        ctx.set(dut.pins.gpio.i[1], 1)
        ctx.set(dut.pins.gpio.i[3], 1)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0xa)
        ctx.set(dut.pins.gpio.i[1], 0)
        ctx.set(dut.pins.gpio.i[3], 0)
        await _csr_access(ctx, dut, input_addr, r_stb=1, r_data=0x0)

    sim = Simulator(dut)
    sim.add_clock(1e-6)
    sim.add_testbench(testbench)
    with sim.write_vcd(vcd_file="test.vcd"):
        sim.run()
