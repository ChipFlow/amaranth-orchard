# SPDX-License-Identifier: BSD-2-Clause
import os
import pytest
from amaranth import *
from amaranth.lib import wiring
from amaranth.back.cxxrtl import convert, convert_fragment

from amaranth_orchard.io.uart import UART
from amaranth_orchard.platforms import SimPlatform

class TestUARTIntegration:
    def test_uart_simulation(self):
        """Test that we can create a UART and simulate it with the model"""
        class Top(Elaboratable):
            def __init__(self):
                self.uart = UART()
                self.counter = Signal(24)
                self.tx_data = Signal(8, reset=ord('A'))
                self.tx_strobe = Signal()
                
            def elaborate(self, platform):
                m = Module()
                
                m.submodules.uart = self.uart
                
                # Tick counter
                m.d.sync += self.counter.eq(self.counter + 1)
                
                # Send character every 1000 cycles
                with m.If(self.counter == 1000):
                    m.d.sync += self.tx_strobe.eq(1)
                    
                with m.If(self.tx_strobe & self.uart.tx_ready):
                    m.d.sync += [
                        self.tx_strobe.eq(0),
                        self.tx_data.eq(self.tx_data + 1),
                        self.counter.eq(0)
                    ]
                    
                # Connect the UART
                m.d.comb += [
                    self.uart.tx_data.eq(self.tx_data),
                    self.uart.tx_ready.eq(self.tx_strobe)
                ]
                
                # Add the UART simulation model
                if isinstance(platform, SimPlatform):
                    iface = wiring.Signature({
                        "tx_o": 1,
                        "rx_i": 1
                    }).create()
                    
                    m.d.comb += [
                        iface.tx_o.eq(self.uart.tx),
                        self.uart.rx.eq(iface.rx_i)
                    ]
                    
                    m.submodules.uart_model = platform.add_model("uart_model", iface)
                
                return m
                
        # Create simulation platform
        platform = SimPlatform()
        dut = Top()
        
        # Build for simulation
        platform.build(dut)
        
        # This is just a test of the wiring, in a real test we would run the simulation