# SPDX-License-Identifier: BSD-2-Clause

import enum
import itertools
import logging
import pathlib
from collections import OrderedDict, deque
from typing import Set, List, Dict, Optional, Union, Literal

from amaranth.lib import wiring, io, meta
from amaranth.lib.wiring import In, Out

__all__ = ['PIN_ANNOTATION_SCHEMA', 'PinSignature',
           'OutputPinSignature', 'InputPinSignature', 'BidirPinSignature']

logger = logging.getLogger(__name__)


def _chipflow_schema_uri(name: str, version: int) -> str:
    return f"https://api.chipflow.com/schemas/{version}/{name}"


class _PinAnnotationModel:
    def __init__(self, direction, width, options=None):
        self.direction = direction
        self.width = width
        self.options = options or {}
        
    def model_dump(self):
        return {
            "direction": self.direction,
            "width": self.width,
            "options": self.options
        }


class _PinAnnotation(meta.Annotation):
    def __init__(self, **kwargs):
        self.model = _PinAnnotationModel(**kwargs)

    @property
    def origin(self):  # type: ignore
        return self.model

    def as_json(self):  # type: ignore
        return self.model.model_dump()


PIN_ANNOTATION_SCHEMA = str(_chipflow_schema_uri("pin-annotation", 0))


class PinSignature(wiring.Signature):
    """Amaranth Signature used to decorate wires that would
    usually be brought out onto a port on the package.

    direction: Input, Output or Bidir
    width: width of port
    all_have_oe: For Bidir ports, should Output Enable be per wire or for the whole port
    init: a  :ref:`const-castable object <lang-constcasting>` for the initial values of the port
    """

    def __init__(self, direction: io.Direction, width: int = 1, all_have_oe: bool = False, init = None):
        self._direction = direction
        self._width = width
        self._init = init
        match direction:
            case io.Direction.Bidir:
                sig = {
                    "o": Out(width),
                    "oe": Out(width if all_have_oe else 1),
                    "i": In(width)
                }
            case io.Direction.Input:
                sig = {"i": In(width)}
            case io.Direction.Output:
                sig = {"o": Out(width)}
            case _:
                assert False
        self._options = {
                "all_have_oe": all_have_oe,
                "init": init,
                }

        super().__init__(sig)

    @property
    def direction(self):
        return self._direction

    def width(self):
        return self._width

    def options(self):
        return self._options

    def annotations(self, *args):
        annotations = wiring.Signature.annotations(self, *args)
        pin_annotation = _PinAnnotation(direction=self._direction, width=self._width, options=self._options)
        return annotations + (pin_annotation,)

    def __repr__(self):
        opts = ', '.join(f"{k}={v}" for k, v in self._options.items())
        return f"PinSignature({self._direction}, {self._width}, {opts})"


def OutputPinSignature(width=1, **kwargs):
    return PinSignature(io.Direction.Output, width=width, **kwargs)


def InputPinSignature(width=1, **kwargs):
    return PinSignature(io.Direction.Input, width=width, **kwargs)


def BidirPinSignature(width=1, **kwargs):
    return PinSignature(io.Direction.Bidir, width=width, **kwargs)


Pin = Union[tuple, str]
PinSet = Set[Pin]
PinList = List[Pin]
Pins = Union[PinSet, PinList]