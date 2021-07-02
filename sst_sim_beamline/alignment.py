import bluesky.preprocessors as bpp
from bluesky.plan_stubs import mv
from databroker.core import SingleRunCache
import numpy as np

#need to fix imports, test, etc, actually hook up the max logic, and do a derivative scan version
#figure out if I can use bluesky-live instead? See thread

def move_to(f, *args):
    pass
    # start composing functions -- move to, alignment, etc. Figure out how to save alignment?

def find_max(plan, dets, *args):
    src = SingleRunCache()
    @bpp.subs_decorator(src.callback)
    def inner_maximizer():
        yield from plan(dets, *args)
        run = src.retrieve()
        table = run.primary.read()
        motor_names = run.metadata['start']['motors']
        motors = [m for m in args if getattr(m, 'name', None) in motor_names]
        detname = dets[0].name
        max_idx = table[detname].argmax()
        print(f"Maximum found at step {max_idx} for detector {detname}")
        ret = []
        for m in motors:
            max_val = np.asarray(table[m.name][max_idx])
            print(f"setting {m.name} to {max_val}")
            ret.append([m, max_val])
            yield from mv(m, max_val)
        return ret
    return (yield from inner_maximizer())

def find_max_deriv(plan, dets, *args):
    src = SingleRunCache()
    @bpp.subs_decorator(src.callback)
    def inner_maximizer():
        yield from plan(dets, *args)
        run = src.retrieve()
        table = run.primary.read()
        motor_names = run.metadata['start']['motors']
        motors = [m for m in args if getattr(m, 'name', None) in motor_names]
        if len(motors) > 1:
            print("Derivative with multiple motors unsupported, taking first motor")
            #raise NotImplementedError("Derivative with multiple motors unsupported")
        
        detname = dets[0].name
        motname = motors[0].name
        max_idx = np.argmax(np.abs(np.gradient(table[detname], table[motname])))
        print(f"Maximum derivative found at step {max_idx} for detector {detname}")
        ret = []
        for m in motors:
            max_val = np.asarray(table[m.name][max_idx])
            print(f"setting {m.name} to {max_val}")
            ret.append([max_val, m])
            yield from mv(m, max_val)
        return ret
    return (yield from inner_maximizer())
        
def find_z_offset(zstart=0, xstart=0):
    yield from mv(samplex, xstart)
    yield from mv(samplez, zstart)
    ret = yield from find_max_deriv(rel_scan, [i1], samplez, -5, 5, 26)
    zoffset = ret[0][0]
    print(zoffset)
