from amaranth import *
from amaranth.lib.wiring import In, Out

from amaranth_soc import csr


__all__ = ["StorageLessRW"]


class StorageLessRW(csr.FieldAction):
    """A read/write field action without built-in storage."""
    def __init__(self, shape):
        super().__init__(shape, access="rw", members={
            "r_data": In(shape),
            "w_data": Out(shape),
            "w_stb":  Out(1),
        })

    def elaborate(self, platform):
        m = Module()
        m.d.comb += [
            self.port.r_data.eq(self.r_data),
            self.w_data.eq(self.port.w_data),
            self.w_stb .eq(self.port.w_stb),
        ]
        return m
