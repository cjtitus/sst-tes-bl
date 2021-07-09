import bluesky.preprocessors as bpp
from bluesky.plan_stubs import mv, mvr
from bluesky.plans import rel_scan
from databroker.core import SingleRunCache
from sst_core.api import samplex, samplez, sampler, i1
from sst_base.frames import deg_to_rad, rad_to_deg, rotz, vec
import numpy as np

#need to fix imports, test, etc, actually hook up the max logic, and do a derivative scan version
#figure out if I can use bluesky-live instead? See thread

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
            max_idx = int(table[detname].argmin())
            print(f"Minimum found at step {max_idx} for detector {detname}")
        else:
            max_idx = int(table[detname].argmax())
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
            ret.append([m, max_val])
            yield from mv(m, max_val)
        return ret
    return (yield from inner_maximizer())
        
def scan_z_offset(zstart, zstop, step_size):
    nsteps = int(np.abs(zstop - zstart)/step_size) + 1
    ret = yield from find_max_deriv(rel_scan, [i1], samplez, zstart, zstop, nsteps)
    _, zoffset = ret[0]
    print(zoffset)
    return zoffset

def scan_z_coarse():
    return (yield from scan_z_offset(-10, 10, 1))

def scan_z_medium():
    return (yield from scan_z_offset(-2, 2, 0.2))

def scan_z_fine():
    return (yield from scan_z_offset(-0.5, 0.5, 0.05))
    
def scan_r_offset(rstart, rstop, step_size):
    """
    Relative scan, find r that maximizes signal
    """
    nsteps = int(np.abs(rstop - rstart)/step_size) + 1
    ret = yield from find_max(rel_scan, [i1], sampler, rstart, rstop, nsteps)
    _, roffset = ret[0]
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
    _, xoffset = ret[0]
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

def find_z_offset():
    yield from scan_z_medium()
    zoffset = yield from scan_z_fine()
    print(f"Found edge at {zoffset}")
    return zoffset

def find_r_offset():
    yield from scan_r_offset(-10, 10, 1)
    ret = yield from scan_r_offset(-1, 1, 0.1)
    print(f"Found angle at {ret}")
    return ret

def find_corner_x_r():
    """
    finds the rotation that makes a side parallel to the beam, and the
    x-coordinate of the surface
    """
    yield from scan_x_coarse()
    yield from scan_r_coarse()
    yield from scan_x_medium()
    theta = yield from scan_r_medium()
    x = yield from scan_x_fine()
    return np.abs(x), theta

def find_corner_coordinates(nsides=4):
    #rotation angle to next side
    ra = 360.0/nsides
    
    x1, r1 = yield from find_corner_x_r()
    yield from mvr(sampler, ra)
    x2, r2 = yield from find_corner_x_r()
    
    y1 = calculate_corner_y(x1, r1, x2, r2, nsides)
    print(f"x1: {x1}, y1: {y1}, r1: {r1}")
    return x1, y1, r1, r2

def calculate_corner_y(x1, r1, x2, r2, nsides=4):
    x1 = np.abs(x1)
    x2 = np.abs(x2)
    theta1 = deg_to_rad(r1)
    #interior angle for a regular polygon
    ia = deg_to_rad(180.0*(nsides - 2)/nsides)
    phi1 =  np.arctan(np.sin(ia)/(x2/x1 + np.cos(ia)))
    y1 = x1/np.tan(phi1)
    return y1

def find_corner_known_rotation(r1, r2, nsides=4):
    yield from mv(sampler, r2)
    x2 = yield from find_x_offset()
    yield from mv(sampler, r1)
    x1 = yield from find_x_offset()
    y1 = calculate_corner_y(x1, r1, x2, r2, nsides)
    x2 = np.abs(x2)
    x1 = np.abs(x1)
    y1 = np.abs(y1)
    print(f"Corners known rotation x1: {x1}, y1: {y1}")
    return x1, y1
    
def find_side_basis(nsides=4):
    z = yield from find_z_offset()
    yield from mvr(samplez, -5)
    x1, y1, r1, r2 = yield from find_corner_coordinates(nsides)
    yield from mvr(samplez, -90)
    (x3, y3) = yield from find_corner_known_rotation(r1, r2, nsides)
    yield from mv(samplez, z)
    #fudging origin a bit so that it is z, not z + 5
    p1 = vec(x1, -y1, z)
    #still want correct height difference between points
    p2 = vec(x3, -y3, z + 95)
    p3 = vec(x1, -y1 + 10, z)

    theta1 = -1*deg_to_rad(r1)
    p1r = rotz(theta1, p1)
    p2r = rotz(theta1, p2)
    p3r = rotz(theta1, p3)
    print("Points ", p1, p2, p3)
    print("Rotated points ", p1r, p2r, p3r)
    return p1r, p2r, p3r
    
def 
