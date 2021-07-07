from bluesky import RunEngine
from databroker import Broker

RE = RunEngine({})
db = Broker.named('temp')
RE.subscribe(db.insert)

# Slightly misaligned manipulator
