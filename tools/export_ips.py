import inspect
import json
import sys

from amaranth import Elaboratable
from amaranth.lib import wiring

import amaranth_orchard

def children(mod):
    stack = [(attr, getattr(mod, attr)) for attr in dir(mod) if not attr.startswith("_")]
    done = []
    while stack:
        i = stack.pop()
        e,o = i
        if inspect.isclass(o) and o.__module__.startswith('chipflow_digital_ip') and issubclass(o, wiring.Component):
            yield o
        if inspect.ismodule(o):
            if e in done:
                continue
            done.append(e)
            stack.extend([(attr, getattr(o, attr)) for attr in dir(o) if not attr.startswith("_")])

gen = children(amaranth_orchard)
output={}
for cls in gen:
    output[f"{cls.__module__}.{cls.__qualname__}"]=cls.__doc__

json.dump(output, sys.stdout, indent=2)
