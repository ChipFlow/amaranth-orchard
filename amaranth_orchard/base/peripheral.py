# From lambdasoc, with some small updates
#
# Copyright (C) 2021-2 ChipFlow Ltd.
# Copyright (C) 2020 LambdaConcept
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys, os
from abc import ABC, abstractmethod

from amaranth import *
from amaranth import tracer
from amaranth.utils import log2_int

from amaranth_soc import csr, wishbone
from amaranth_soc.memory import MemoryMap
from amaranth_soc.csr.wishbone import WishboneCSRBridge

class Peripheral(ABC):
    """Wishbone peripheral.

    A helper class to reduce the boilerplate needed to control a peripheral with a Wishbone interface.
    It provides facilities for instantiating CSR registers, requesting windows to subordinate busses
    and sending interrupt requests to the CPU.

    The ``Peripheral`` class is not meant to be instantiated as-is, but rather as a base class for
    actual peripherals.

    Usage example
    -------------

    ```
    class ExamplePeripheral(Peripheral, Elaboratable):
        def __init__(self):
            super().__init__()
            bank         = self.csr_bank()
            self._foo    = bank.csr(8, "r")
            self._bar    = bank.csr(8, "w")

            self._rdy    = self.event(mode="rise")

            self._bridge = self.bridge(data_width=32, granularity=8, alignment=2)
            self.bus     = self._bridge.bus
            self.irq     = self._bridge.irq

        def elaborate(self, platform):
            m = Module()
            m.submodules.bridge = self._bridge
            # ...
            return m
    ```

    Arguments
    ---------
    name : str
        Name of this peripheral. If ``None`` (default) the name is inferred from the variable
        name this peripheral is assigned to.

    Properties
    ----------
    name : str
        Name of the peripheral.
    """
    def __init__(self, name=None, src_loc_at=1):
        if name is not None and not isinstance(name, str):
            raise TypeError("Name must be a string, not {!r}".format(name))
        self.name      = name or tracer.get_var_name(depth=2 + src_loc_at).lstrip("_")

        self._csr_banks = []
        self._windows   = []
        self._events    = []

        self._bus       = None
        self._irq       = None

    @property
    def model_sources(self):
        # list absolute paths of all c++ sources for simulating this peripheral
        class_dir = os.path.dirname(os.path.abspath(sys.modules[self.__module__].__file__))
        return [ os.path.join(class_dir, self._models_path(),  model) for model in self._models() ]

    @property
    def model_includepaths(self):
        # list of absolute paths where to find headers for simulating  this peripheral
        class_dir = os.path.dirname(os.path.abspath(sys.modules[self.__module__].__file__))
        return [ os.path.join(class_dir, self._models_path()) ]

    @abstractmethod
    def _models(self):
        # list of simulation model c++ sources for simulating this peripheral
        pass

    def _models_path(self):
        # path relative to the class source where c++ simulation models can be found
        return "models/"

    @property
    def bus(self):
        """Wishbone bus interface.

        Return value
        ------------
        An instance of :class:`Interface`.

        Exceptions
        ----------
        Raises :exn:`NotImplementedError` if the peripheral does not have a Wishbone bus.
        """
        if self._bus is None:
            raise NotImplementedError("Peripheral {!r} does not have a bus interface"
                                      .format(self))
        return self._bus

    @bus.setter
    def bus(self, bus):
        if not isinstance(bus, wishbone.Interface):
            raise TypeError("Bus interface must be an instance of wishbone.Interface, not {!r}"
                            .format(bus))
        self._bus = bus

    @property
    def irq(self):
        """Interrupt request line.

        Return value
        ------------
        An instance of :class:`IRQLine`.

        Exceptions
        ----------
        Raises :exn:`NotImplementedError` if the peripheral does not have an IRQ line.
        """
        if self._irq is None:
            raise NotImplementedError("Peripheral {!r} does not have an IRQ line"
                                      .format(self))
        return self._irq

    @irq.setter
    def irq(self, irq):
        if not isinstance(irq, IRQLine):
            raise TypeError("IRQ line must be an instance of IRQLine, not {!r}"
                            .format(irq))
        self._irq = irq

    def csr_bank(self, *, name=None, addr=None, alignment=None):
        """Request a CSR bank.

        Arguments
        ---------
        name : str
            Optional. Bank name.
        addr : int or None
            Address of the bank. If ``None``, the implicit next address will be used.
            Otherwise, the exact specified address (which must be a multiple of
            ``2 ** max(alignment, bridge_alignment)``) will be used.
        alignment : int or None
            Alignment of the bank. If not specified, the bridge alignment is used.
            See :class:`amaranth_soc.csr.Multiplexer` for details.

        Return value
        ------------
        An instance of :class:`CSRBank`.
        """
        bank = CSRBank(name=name)
        self._csr_banks.append((bank, addr, alignment))
        return bank

    def window(self, *, addr_width, data_width, granularity=None, features=frozenset(),
               name=None, addr=None, sparse=None):
        """Request a window to a subordinate bus.

        See :meth:`amaranth_soc.wishbone.Decoder.add` for details.

        Return value
        ------------
        An instance of :class:`amaranth_soc.wishbone.Interface`.
        """
        window = wishbone.Interface(addr_width=addr_width, data_width=data_width,
                                    granularity=granularity, features=features, name=name)
        self._windows.append((window, addr, sparse))
        return window

    def event(self, *, mode="level", name=None, src_loc_at=0):
        """Request an event source.

        See :class:`EventSource` for details.

        Return value
        ------------
        An instance of :class:`EventSource`.
        """
        event = EventSource(mode=mode, name=name, src_loc_at=1 + src_loc_at)
        self._events.append(event)
        return event

    def bridge(self, *, data_width=8, granularity=None, features=frozenset(), alignment=0):
        """Request a bridge to the resources of the peripheral.

        See :class:`PeripheralBridge` for details.

        Return value
        ------------
        A :class:`PeripheralBridge` providing access to local resources.
        """
        return PeripheralBridge(self, data_width=data_width, granularity=granularity,
                                features=features, alignment=alignment)

    def iter_csr_banks(self):
        """Iterate requested CSR banks and their parameters.

        Yield values
        ------------
        A tuple ``bank, addr, alignment`` describing the bank and its parameters.
        """
        for bank, addr, alignment in self._csr_banks:
            yield bank, addr, alignment

    def iter_windows(self):
        """Iterate requested windows and their parameters.

        Yield values
        ------------
        A tuple ``window, addr, sparse`` descr
        given to :meth:`Peripheral.window`.
        """
        for window, addr, sparse in self._windows:
            yield window, addr, sparse

    def iter_events(self):
        """Iterate requested event sources.

        Yield values
        ------------
        An instance of :class:`EventSource`.
        """
        for event in self._events:
            yield event


class CSRBank:
    """CSR register bank.

    Parameters
    ----------
    name : str
        Optional. Name prefix of the bank registers.
    """
    def __init__(self, *, name=None):
        if name is not None and not isinstance(name, str):
            raise TypeError("Name must be a string, not {!r}".format(name))

        self.name      = name
        self._csr_regs = []

    def csr(self, width, access, *, addr=None, alignment=None, name=None,
            src_loc_at=0):
        """Request a CSR register.

        Parameters
        ----------
        width : int
            Width of the register. See :class:`amaranth_soc.csr.Element`.
        access : :class:`Access`
            Register access mode. See :class:`amaranth_soc.csr.Element`.
        addr : int
            Address of the register. See :meth:`amaranth_soc.csr.Multiplexer.add`.
        alignment : int
            Register alignment. See :class:`amaranth_soc.csr.Multiplexer`.
        name : str
            Name of the register. If ``None`` (default) the name is inferred from the variable
            name this register is assigned to.

        Return value
        ------------
        An instance of :class:`amaranth_soc.csr.Element`.
        """
        if name is not None and not isinstance(name, str):
            raise TypeError("Name must be a string, not {!r}".format(name))
        name = name or tracer.get_var_name(depth=2 + src_loc_at).lstrip("_")

        if any(elem.name == name for (elem, _, _) in self._csr_regs):
            raise Exception("CSR \"{}\" has already been defined".format(name))
        elem = csr.Element(width, access, name=name)
        self._csr_regs.append((elem, addr, alignment))
        return elem

    def iter_csr_regs(self):
        """Iterate requested CSR registers and their parameters.

        Yield values
        ------------
        A tuple ``elem, addr, alignment`` describing the register and its parameters.
        """
        for elem, addr, alignment in self._csr_regs:
            yield elem, addr, alignment


class PeripheralBridge(Elaboratable):
    """Peripheral bridge.

    A bridge providing access to the registers and windows of a peripheral, and support for
    interrupt requests from its event sources.

    Event managment is performed by an :class:`InterruptSource` submodule.

    Parameters
    ---------
    periph : :class:`Peripheral`
        The peripheral whose resources are exposed by this bridge.
    data_width : int
        Data width. See :class:`amaranth_soc.wishbone.Interface`.
    granularity : int or None
        Granularity. See :class:`amaranth_soc.wishbone.Interface`.
    features : iter(str)
        Optional signal set. See :class:`amaranth_soc.wishbone.Interface`.
    alignment : int
        Resource alignment. See :class:`amaranth_soc.memory.MemoryMap`.

    Attributes
    ----------
    bus : :class:`amaranth_soc.wishbone.Interface`
        Wishbone bus providing access to the resources of the peripheral.
    irq : :class:`IRQLine`, out
        Interrupt request. It is raised if any event source is enabled and has a pending
        notification.
    """
    def __init__(self, periph, *, data_width, granularity, features, alignment):
        if not isinstance(periph, Peripheral):
            raise TypeError("Peripheral must be an instance of Peripheral, not {!r}"
                            .format(periph))

        self._wb_decoder = wishbone.Decoder(addr_width=1, data_width=data_width,
                                            granularity=granularity,
                                            features=features, alignment=alignment,
                                            name=periph.name)

        self._csr_subs = []

        for bank, bank_addr, bank_alignment in periph.iter_csr_banks():
            if bank_alignment is None:
                bank_alignment = alignment
            csr_mux = csr.Multiplexer(
                addr_width=1, data_width=granularity, alignment=bank_alignment,
                name=bank.name,
            )
            for elem, elem_addr, elem_alignment in bank.iter_csr_regs():
                if elem_alignment is None:
                    elem_alignment = alignment
                csr_mux.add(elem, addr=elem_addr, alignment=elem_alignment, extend=True)

            csr_bridge = WishboneCSRBridge(csr_mux.bus, data_width=data_width)
            self._wb_decoder.add(csr_bridge.wb_bus, addr=bank_addr, extend=True)
            self._csr_subs.append((csr_mux, csr_bridge))

        for window, window_addr, window_sparse in periph.iter_windows():
            self._wb_decoder.add(window, addr=window_addr, sparse=window_sparse, extend=True)

        events = list(periph.iter_events())
        if len(events) > 0:
            self._int_src = InterruptSource(events, name=periph.name)
            self.irq      = self._int_src.irq

            csr_mux = csr.Multiplexer(addr_width=1, data_width=8, alignment=alignment, name="ev")
            csr_mux.add(self._int_src.status,  extend=True)
            csr_mux.add(self._int_src.pending, extend=True)
            csr_mux.add(self._int_src.enable,  extend=True)

            csr_bridge = WishboneCSRBridge(csr_mux.bus, data_width=data_width)
            self._wb_decoder.add(csr_bridge.wb_bus, extend=True)
            self._csr_subs.append((csr_mux, csr_bridge))
        else:
            self._int_src = None
            self.irq      = None

        self.bus = self._wb_decoder.bus

    def elaborate(self, platform):
        m = Module()

        for i, (csr_mux, csr_bridge) in enumerate(self._csr_subs):
            m.submodules[   "csr_mux_{}".format(i)] = csr_mux
            m.submodules["csr_bridge_{}".format(i)] = csr_bridge

        if self._int_src is not None:
            m.submodules._int_src = self._int_src

        m.submodules.wb_decoder = self._wb_decoder

        return m



class EventSource:
    """Event source.
    Parameters
    ----------
    mode : ``"level"``, ``"rise"``, ``"fall"``
        Trigger mode. If ``"level"``, a notification is raised when the ``stb`` signal is high.
        If ``"rise"`` (or ``"fall"``) a notification is raised on a rising (or falling) edge
        of ``stb``.
    name : str
        Name of the event. If ``None`` (default) the name is inferred from the variable
        name this event source is assigned to.
    Attributes
    ----------
    name : str
        Name of the event
    mode : ``"level"``, ``"rise"``, ``"fall"``
        Trigger mode.
    stb : Signal, in
        Event strobe.
    """
    def __init__(self, *, mode="level", name=None, src_loc_at=0):
        if name is not None and not isinstance(name, str):
            raise TypeError("Name must be a string, not {!r}".format(name))

        choices = ("level", "rise", "fall")
        if mode not in choices:
            raise ValueError("Invalid trigger mode {!r}; must be one of {}"
                             .format(mode, ", ".join(choices)))

        self.name = name or tracer.get_var_name(depth=2 + src_loc_at)
        self.mode = mode
        self.stb  = Signal(name="{}_stb".format(self.name))


class IRQLine(Signal):
    """Interrupt request line."""
    def __init__(self, *, name=None, src_loc_at=0):
        super().__init__(name=name, src_loc_at=1 + src_loc_at)

    __hash__ = object.__hash__


class InterruptSource(Elaboratable):
    """Interrupt source.
    A mean of gathering multiple event sources into a single interrupt request line.
    Parameters
    ----------
    events : iter(:class:`EventSource`)
        Event sources.
    name : str
        Name of the interrupt source. If ``None`` (default) the name is inferred from the
        variable name this interrupt source is assigned to.
    Attributes
    ----------
    name : str
        Name of the interrupt source.
    status : :class:`csr.Element`, read-only
        Event status register. Each bit displays the level of the strobe of an event source.
        Events are ordered by position in the `events` parameter.
    pending : :class:`csr.Element`, read/write
        Event pending register. If a bit is 1, the associated event source has a pending
        notification. Writing 1 to a bit clears it.
        Events are ordered by position in the `events` parameter.
    enable : :class:`csr.Element`, read/write
        Event enable register. Writing 1 to a bit enables its associated event source.
        Writing 0 disables it.
        Events are ordered by position in the `events` parameter.
    irq : :class:`IRQLine`, out
        Interrupt request. It is raised if any event source is enabled and has a pending
        notification.
    """
    def __init__(self, events, *, name=None, src_loc_at=0):
        if name is not None and not isinstance(name, str):
            raise TypeError("Name must be a string, not {!r}".format(name))
        self.name = name or tracer.get_var_name(depth=2 + src_loc_at)

        for event in events:
            if not isinstance(event, EventSource):
                raise TypeError("Event source must be an instance of EventSource, not {!r}"
                                .format(event))
        self._events = list(events)

        width = len(events)
        self.status  = csr.Element(width, "r",  name="status")
        self.pending = csr.Element(width, "rw", name="pending")
        self.enable  = csr.Element(width, "rw", name="enable")

        self.irq = IRQLine(name="{}_irq".format(self.name))

    def elaborate(self, platform):
        m = Module()

        with m.If(self.pending.w_stb):
            m.d.sync += self.pending.r_data.eq(self.pending.r_data & ~self.pending.w_data)

        with m.If(self.enable.w_stb):
            m.d.sync += self.enable.r_data.eq(self.enable.w_data)

        for i, event in enumerate(self._events):
            m.d.sync += self.status.r_data[i].eq(event.stb)

            if event.mode in ("rise", "fall"):
                event_stb_r = Signal.like(event.stb, name_suffix="_r")
                m.d.sync += event_stb_r.eq(event.stb)

            event_trigger = Signal(name="{}_trigger".format(event.name))
            if event.mode == "level":
                m.d.comb += event_trigger.eq(event.stb)
            elif event.mode == "rise":
                m.d.comb += event_trigger.eq(~event_stb_r & event.stb)
            elif event.mode == "fall":
                m.d.comb += event_trigger.eq(event_stb_r & ~event.stb)
            else:
                assert False # :nocov:

            if event.mode == "level":
                m.d.sync += self.pending.r_data[i].eq(event_trigger)
            else:
                with m.If(event_trigger):
                    m.d.sync += self.pending.r_data[i].eq(1)

        m.d.comb += self.irq.eq((self.pending.r_data & self.enable.r_data).any())

        return m
