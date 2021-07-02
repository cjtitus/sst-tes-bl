from bluesky import RunEngine
from bluesky.plans import scan, count, list_scan, rel_scan
from bluesky.plan_stubs import mv, abs_set
from bluesky.callbacks import LiveTable
from databroker import Broker
from ophyd.sim import det, motor
from .motors import Manipulator
from .detectors import SynI1

import matplotlib.pyplot as plt
import numpy as np
plt.ion()

RE = RunEngine({})
db = Broker.named('temp')
RE.subscribe(db.insert)


p1 = vec(1, 0, 0)
p2 = vec(0, 0, 1)
p3 = vec(0, 0, 0)
frame = Frame(p1, p2, p3)

man = Manipulator(frame)

samplex = man.x
sampley = man.y
samplez = man.z
sampler = man.r

framex = man.sx
framey = man.sy
framez = man.sz
framer = man.sr

i1 = SynI1("i1", man)
