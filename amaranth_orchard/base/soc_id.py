import subprocess
from amaranth import *

from .peripheral import Peripheral

class SoCID(Peripheral, Elaboratable):
    """
    SoC identifier peripheral

    Two read-only CSRs; the first contains a SoC-defined type ID and the second the git hash of the
    repo being used to build the SoC.
    """
    def __init__(self, type_id=0xbadca77e, **kwargs):
        super().__init__()

        bank            = self.csr_bank()
        self.type_id = type_id
        self.soc_type  = bank.csr(32, "r")
        self.soc_version = bank.csr(32, "r")

        self._bridge    = self.bridge(data_width=32, granularity=8, alignment=2)
        self.bus        = self._bridge.bus
        self.git_hash = int(subprocess.check_output('git rev-parse --verify HEAD'.split(' ')).strip()[0:8], 16)

    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge  = self._bridge

        m.d.comb += [
            self.soc_type.r_data.eq(self.type_id),
            self.soc_version.r_data.eq(self.git_hash),
        ]

        return m
