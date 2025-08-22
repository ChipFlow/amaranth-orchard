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
        # sim, statement_cov, stmtid_to_info, fragment = mk_sim_with_stmtcov(dut)
        # sim, blk_cov, blk_info, _ = mk_sim_with_blockcov(dut, verbose=True)
        sim, expr_cov, expr_info, _ = mk_sim_with_exprcov(dut, verbose=True)

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


        sim.add_clock(1e-5)
        sim.add_testbench(testbench)
        with sim.write_vcd("i2c_start_test.vcd", "i2c_start_test.gtkw"):
            sim.run()

        # results = statement_cov.get_results()
        # statement_cov.close(0)
        # merge_stmtcov(results, stmtid_to_info)
        # results = blk_cov.get_results()
        # merge_blockcov(results, blk_info)
        results = expr_cov.get_results()
        merge_exprcov(results, expr_info)

    def test_write(self):
        dut = _I2CHarness()
        # sim, statement_cov, stmtid_to_info, fragment = mk_sim_with_stmtcov(dut)
        # sim, blk_cov, blk_info, _ = mk_sim_with_blockcov(dut, verbose=True)
        sim, expr_cov, expr_info, _ = mk_sim_with_exprcov(dut, verbose=True)

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
        sim.add_clock(1e-5)
        sim.add_testbench(testbench)
        with sim.write_vcd("i2c_write_test.vcd", "i2c_write_test.gtkw"):
            sim.run()

        # results = statement_cov.get_results()
        # statement_cov.close(0)
        # merge_stmtcov(results, stmtid_to_info)
        # results = blk_cov.get_results()
        # merge_blockcov(results, blk_info)
        results = expr_cov.get_results()
        merge_exprcov(results, expr_info)

    def test_read(self):
        dut = _I2CHarness()
        # sim, statement_cov, stmtid_to_info, fragment = mk_sim_with_stmtcov(dut)
        # sim, blk_cov, blk_info, _ = mk_sim_with_blockcov(dut, verbose=True)
        sim, expr_cov, expr_info, _ = mk_sim_with_exprcov(dut, verbose=True)

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
        sim.add_clock(1e-5)
        sim.add_testbench(testbench)
        with sim.write_vcd("i2c_read_test.vcd", "i2c_read_test.gtkw"):
            sim.run()
        # results = statement_cov.get_results()
        # statement_cov.close(0)
        # merge_stmtcov(results, stmtid_to_info)
        # results = blk_cov.get_results()
        # merge_blockcov(results, blk_info)
        results = expr_cov.get_results()
        merge_exprcov(results, expr_info)

    @classmethod
    def tearDownClass(cls):
    #     emit_agg_summary("i2c_statement_cov.json", label="tests/test_i2c.py")
    #     emit_agg_block_summary("i2c_block_cov.json", label=__name__)
        emit_expr_summary("i2c_expression_cov.json", label="test_i2c.py")

