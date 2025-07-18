import inspect
import json
import sys

from amaranth import Elaboratable
from amaranth.lib import wiring
from docstring_parser import parse_from_object

import chipflow_digital_ip

def children(mod):
    level = "chipflow_digital_ip"
    stack = [(f"{level}.{attr}", getattr(mod, attr)) for attr in dir(mod) if not attr.startswith("_")]
    done = []
    while stack:
        i = stack.pop()
        e,o = i
        if inspect.isclass(o) and o.__module__.startswith('chipflow_digital_ip') and issubclass(o, wiring.Component):
            yield e,o
        if inspect.ismodule(o):
            if e in done:
                continue
            done.append(e)
            stack.extend([(f"{e}.{attr}", getattr(o, attr)) for attr in dir(o) if not attr.startswith("_")])

gen = children(chipflow_digital_ip)
output={}
for name, cls in gen:
    docstring = parse_from_object(cls)
    d = {
        'short_description': docstring.short_description,
        'long_description': docstring.long_description,
        'params': [p.__dict__ for p in docstring.params],
        'examples': [ e.__dict__ for e in docstring.examples],
        }
    output[name] = d

json.dump(output, sys.stdout, indent=2)
