import bluesky.preprocessors as bpp
from bluesky.plan_stubs import mv
from bluesky.plans import rel_scan
from databroker.core import SingleRunCache
from sst_core.api import samplex, samplez, sampler, i1
from sst_base.frames import deg_to_rad, rad_to_deg
import numpy as np

#need to fix imports, test, etc, actually hook up the max logic, and do a derivative scan version
#figure out if I can use bluesky-live instead? See thread

def move_to(f, *args):
    pass
    # start composing functions -- move to, alignment, etc. Figure out how to save alignment?

def find_max(plan, dets, *args, invert=False):
    """
    invert turns find_max into find_min
    """
    src = SingleRunCache()
    @bpp.subs_decorator(src.callback)
    def inner_maximizer():
        yield from plan(dets, *args)
        run = src.retrieve()
        table = run.primary.read()
        motor_names = run.metadata['start']['motors']
        motors = [m for m in args if getattr(m, 'name', None) in motor_names]
        detname = dets[0].name
        if invert:
            max_idx = table[detname].argmin()
            print(f"Minimum found at step {max_idx} for detector {detname}")
        else:
            max_idx = table[detname].argmax()
            print(f"Maximum found at step {max_idx} for detector {detname}")
        ret = []
        for m in motors:
            max_val = float(table[m.name][max_idx])
            print(f"setting {m.name} to {max_val}")
            ret.append([m, max_val])
            yield from mv(m, max_val)
        return ret
    return (yield from inner_maximizer())

def find_min(plan, dets, *args):
    return (yield from find_max(plan, dets, *args, invert=True))

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
            max_val = float(table[m.name][max_idx])
            print(f"setting {m.name} to {max_val}")
            ret.append([max_val, m])
            yield from mv(m, max_val)
        return ret
    return (yield from inner_maximizer())
        
def find_z_offset(zstart=None, xstart=None):
    if zstart is not None:
        yield from mv(samplez, zstart)
    if xstart is not None:
        yield from mv(samplex, xstart)
    ret = yield from find_max_deriv(rel_scan, [i1], samplez, -5, 5, 26)
    zoffset = ret[0][0]
    print(zoffset)
    return zoffset

def scan_r_offset(rstart, rstop, step_size):
    """
    Relative scan, find r that maximizes signal
    """
    nsteps = int(np.abs(rstop - rstart)/step_size) + 1
    ret = yield from find_max(rel_scan, [i1], sampler, rstart, rstop, nsteps)
    roffset = ret[0][0]
    print(roffset)
    return roffset

def scan_r_coarse():
    return (yield from scan_r_offset(-10, 10, 1))

def scan_r_medium():
    return (yield from scan_r_offset(-2, 2, 0.1))

def scan_r_fine():
    return (yield from scan_r_offset(-0.5, 0.5, 0.05))
    
def scan_x_offset(xstart, xstop, step_size):
    nsteps = int(np.abs(xstop - xstart)/step_size) + 1
    ret = yield from find_max_deriv(rel_scan, [i1], samplex, xstart, xstop, nsteps)
    xoffset = ret[0][0]
    print(xoffset)
    return xoffset

def scan_x_coarse():
    return (yield from scan_x_offset(-10, 10, 0.5))

def scan_x_medium():
    return (yield from scan_x_offset(-2, 2, 0.2))

def scan_x_fine():
    return (yield from scan_x_offset(-0.5, 0.5, 0.05))
    
def find_x_offset():
    yield from scan_x_medium()
    ret = yield from scan_x_coarse()
    print(f"Found edge at {ret}")
    return ret

def find_r_offset():
    yield from scan_r_offset(-10, 10, 1)
    ret = yield from scan_r_offset(-1, 1, 0.1)
    print(f"Found angle at {ret}")
    return ret

def find_corner_x_theta():
    """
    finds the rotation that makes a side parallel to the beam, and the
    x-coordinate of the surface
    """
    yield from scan_x_coarse()
    yield from scan_r_coarse()
    yield from scan_x_medium()
    theta = yield from scan_r_medium()
    x = yield from scan_x_fine()
    return x, theta

def find_corner_coordinates(nsides=4):
    x1, theta1 = yield from find_corner_x_theta()
    #rotation angle to next side
    ra = 360.0/nsides
    yield from mvr(sampler, ra)
    x2, theta2 = yield from find_corner_x_theta()
    #interior angle for a regular polygon
    ia = 180.0*(nsides - 2)/nsides
    phi2 =  np.arctan(np.sin(ia)/(x1/x2 - np.cos(ia)))
    y2 = x2/np.tan(phi2)
    corner_vector = rotz(vec(x2, y2, 0), -1*theta2)
    print(corner_vector)
    return corner_vector
    
    
def find_radius_theta(rstart=35, dtheta=25):
    yield from mv(sampler, rstart)
    d1 = yield from find_x_offset()
    yield from mv(sampler, rstart + dtheta)
    d2 = yield from find_x_offset()
    tantheta = np.sin(deg_to_rad(dtheta))/(d2/d1 - np.cos(deg_to_rad(dtheta)))
    theta = np.arctan(tantheta)
    radius = d1/np.sin(theta)
    print(rad_to_deg(theta), radius)
