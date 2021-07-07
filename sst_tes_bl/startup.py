from bluesky import RunEngine
from databroker import Broker
from ophyd.sim import det, motor
from motors import Manipulator
from detectors import SynI1
from frames import vec, Bar

import matplotlib.pyplot as plt
import numpy as np
plt.ion()

RE = RunEngine({})
db = Broker.named('temp')
RE.subscribe(db.insert)

# Slightly misaligned manipulator
p1 = vec(10, 10, 0)
p2 = vec(10, 10, 1)
p3 = vec(0, 9, 0)
bar = Bar(p1, p2, p3, width=19.5, height=130, nsides=4)

man = Manipulator(bar, name='manipulator')

samplex = man.x
sampley = man.y
samplez = man.z
sampler = man.r

framex = man.sx
framey = man.sy
framez = man.sz
framer = man.sr

i1 = SynI1("i1", man)
