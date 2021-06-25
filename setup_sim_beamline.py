from bluesky import RunEngine
from bluesky.plans import scan, count, list_scan
from bluesky.callbacks import LiveTable
from databroker import Broker
from ophyd.sim import det, motor

RE = RunEngine({})
db = Broker.named('temp')
RE.subscribe(db.insert)

