# amaranth: UnusedElaboratable=no

import unittest
import os.path

from amaranth import *
from amaranth.lib.io import pin_layout
from amaranth_orchard.io.uart import UARTPeripheral

class TestModels(unittest.TestCase):
    def setUp(self):
        pins = Record([("rx_data", pin_layout(1, dir="i")),
                       ("tx_data", pin_layout(1, dir="o")),
                       ("tx_rdy", pin_layout(1, dir="o")),
                       ("rx_avail", pin_layout(1, dir="o"))])
 
        self.uart = UARTPeripheral(
            init_divisor=(25000000//115200),
            pins=pins)
 
    def test_includes(self):
       self.assertRegex(self.uart.model_includepaths[0], "amaranth_orchard/io/models/$" )
       self.assertRegex(self.uart.model_includepaths[1], "amaranth_orchard/base/models/$" )

    def test_sources(self):
        l = []
        for s in self.uart.model_sources:
            self.assertRegex(s, "amaranth_orchard/io/models/" )
            l.append(os.path.basename(s))
        self.assertEqual(l, ["uart.cc"])


if __name__ == '__main__':
    unittest.main()
