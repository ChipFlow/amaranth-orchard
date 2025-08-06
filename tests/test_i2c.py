# amaranth: UnusedElaboratable=no

# SPDX-License-Identifier: BSD-2-Clause
import unittest
from amaranth import *
from amaranth.sim import *
from amaranth.sim._coverage import ToggleCoverageObserver, ToggleDirection, StatementCoverageObserver
from chipflow_digital_ip.io import I2CPeripheral
from tests.test_utils import *
from amaranth.hdl._ir import Fragment


class _I2CHarness(Elaboratable):
    def __init__(self):
        self.i2c = I2CPeripheral()
        self.sda = Signal()
        self.scl = Signal()
        self.sda_i = Signal(init=1)
        self.scl_i = Signal(init=1)
    def elaborate(self, platform):
        m = Module()
        m.submodules.i2c = self.i2c
        # Simulate the open-drain I2C bus
        m.d.comb += [
            self.sda.eq(~self.i2c.i2c_pins.sda.oe & self.sda_i),
            self.scl.eq(~self.i2c.i2c_pins.scl.oe & self.scl_i),

            self.i2c.i2c_pins.sda.i.eq(self.sda),
            self.i2c.i2c_pins.scl.i.eq(self.scl),
        ]
        return m

class TestI2CPeripheral(unittest.TestCase):

    REG_DIVIDER   = 0x00
    REG_ACTION    = 0x04
    REG_SEND_DATA = 0x08
    REG_RECEIVE_DATA = 0x0C
    REG_STATUS = 0x10

    async def _write_reg(self, ctx, dut, reg, value, width=4):
        for i in range(width):
            ctx.set(dut.bus.addr, reg + i)
            ctx.set(dut.bus.w_data, (value >> (8 * i)) & 0xFF)
            ctx.set(dut.bus.w_stb, 1)
            await ctx.tick()
        ctx.set(dut.bus.w_stb, 0)

    async def _check_reg(self, ctx, dut, reg, value, width=4):
        result = 0
        for i in range(width):
            ctx.set(dut.bus.addr, reg + i)
            ctx.set(dut.bus.r_stb, 1)
            await ctx.tick()
            result |= ctx.get(dut.bus.r_data) << (8 * i)
        ctx.set(dut.bus.r_stb, 0)
        self.assertEqual(result, value)

    def test_start_stop(self):
        """Test I2C start and stop conditions"""
        dut = _I2CHarness()
        mod = dut.elaborate(platform=None)
        fragment = Fragment.get(mod, platform=None)
        _, stmtid_to_info = tag_all_statements(fragment)
        coverage_signals = insert_coverage_signals(fragment) 
        signal_to_stmtid = { id(sig): stmt_id for stmt_id, sig in coverage_signals.items() }

        stmtid_to_name = {}
        for domain, stmts in fragment.statements.items():
            for stmt in stmts:
                if hasattr(stmt, "_coverage_id") and hasattr(stmt, "_coverage_name"):
                    stmtid_to_name[stmt._coverage_id] = stmt._coverage_name

        sim = Simulator(fragment)
        statement_cov = StatementCoverageObserver(signal_to_stmtid, sim._engine.state, stmtid_to_info=stmtid_to_info)
        sim._engine.add_observer(statement_cov)

        async def testbench(ctx):
            await self._write_reg(ctx, dut.i2c, self.REG_DIVIDER, 1, 4)
            await ctx.tick()
            await self._write_reg(ctx, dut.i2c, self.REG_ACTION, 1<<1, 1) # START
            await ctx.tick()
            await self._check_reg(ctx, dut.i2c, self.REG_STATUS, 1, 1) # busy
            self.assertEqual(ctx.get(dut.sda), 1)
            self.assertEqual(ctx.get(dut.scl), 1)
            await ctx.tick()
            self.assertEqual(ctx.get(dut.sda), 0)
            self.assertEqual(ctx.get(dut.scl), 1)
            await ctx.tick()
            await self._check_reg(ctx, dut.i2c, self.REG_STATUS, 0, 1) # not busy
            await self._write_reg(ctx, dut.i2c, self.REG_ACTION, 1<<2, 1) # STOP
            for i in range(3):
                await ctx.tick()
            self.assertEqual(ctx.get(dut.sda), 1)
            self.assertEqual(ctx.get(dut.scl), 1)
            await ctx.tick()
            await self._check_reg(ctx, dut.i2c, self.REG_STATUS, 0, 1) # not busy

        # sim = Simulator(dut)
        # design = sim._engine._design
        # signal_path_map = get_signal_full_paths(design)
        # toggle_cov = ToggleCoverageObserver(sim._engine.state, signal_path_map=signal_path_map)
        # sim._engine.add_observer(toggle_cov)

        sim.add_clock(1e-5)
        sim.add_testbench(testbench)
        # all_signals = collect_all_signals(dut)
        all_signals = collect_all_signals(fragment)
        with sim.write_vcd("i2c_start_test.vcd", "i2c_start_test.gtkw"):
            sim.run()
        # results = toggle_cov.get_results()
        # toggle_cov.close(0)
        results = statement_cov.get_results()
        statement_cov.close(0)

    def test_write(self):
        dut = _I2CHarness()
        async def testbench(ctx):
            await self._write_reg(ctx, dut.i2c, self.REG_DIVIDER, 1, 4)
            await ctx.tick()
            await self._write_reg(ctx, dut.i2c, self.REG_ACTION, 1<<1, 1) # START
            for i in range(10):
                await ctx.tick() # wait for START to be completed
            for data in (0xAB, 0x63):
                await self._write_reg(ctx, dut.i2c, self.REG_SEND_DATA, data, 1) # write
                for i in range(3):
                    await ctx.tick()
                for bit in reversed(range(-1, 8)):
                    self.assertEqual(ctx.get(dut.scl), 0)
                    for i in range(4):
                        await ctx.tick()
                    if bit == -1: # ack
                        ctx.set(dut.sda_i, 0)
                    else:
                        self.assertEqual(ctx.get(dut.sda), (data >> bit) & 0x1)
                    for i in range(2):
                        await ctx.tick()
                    self.assertEqual(ctx.get(dut.scl), 1)
                    for i in range(6):
                        await ctx.tick()
                ctx.set(dut.sda_i, 1) # reset bus
                for i in range(20):
                    await ctx.tick()
                await self._check_reg(ctx, dut.i2c, self.REG_STATUS, 2, 1) # not busy, acked
        sim = Simulator(dut)
        sim.add_clock(1e-5)
        sim.add_testbench(testbench)
        with sim.write_vcd("i2c_write_test.vcd", "i2c_write_test.gtkw"):
            sim.run()

    def test_read(self):
        dut = _I2CHarness()
        data = 0xA3
        async def testbench(ctx):
            await self._write_reg(ctx, dut.i2c, self.REG_DIVIDER, 1, 4)
            await ctx.tick()
            await self._write_reg(ctx, dut.i2c, self.REG_ACTION, 1<<1, 1) # START
            for i in range(10):
                await ctx.tick() # wait for START to be completed
            await self._write_reg(ctx, dut.i2c, self.REG_ACTION, 1<<3, 1) # READ, ACK
            for i in range(3):
                await ctx.tick()
            for bit in reversed(range(-1, 8)):
                self.assertEqual(ctx.get(dut.scl), 0)
                for i in range(4):
                    await ctx.tick()
                if bit == -1: # ack
                    self.assertEqual(ctx.get(dut.sda), 0)
                else:
                    ctx.set(dut.sda_i, (data >> bit) & 0x1)
                for i in range(2):
                    await ctx.tick()
                self.assertEqual(ctx.get(dut.scl), 1)
                for i in range(6):
                    await ctx.tick()
                if bit == 0:
                    ctx.set(dut.sda_i, 1) # reset bus

            for i in range(20):
                await ctx.tick()
            await self._check_reg(ctx, dut.i2c, self.REG_STATUS, 0, 1) # not busy
            await self._check_reg(ctx, dut.i2c, self.REG_RECEIVE_DATA, data, 1) # data
        sim = Simulator(dut)
        sim.add_clock(1e-5)
        sim.add_testbench(testbench)
        with sim.write_vcd("i2c_read_test.vcd", "i2c_read_test.gtkw"):
            sim.run()



