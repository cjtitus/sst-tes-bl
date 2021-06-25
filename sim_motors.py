from ophyd.sim import SynAxis, SynSignal, EnumSignal
from ophyd import Device, Component as Cpt
from ophyd.signal import Signal
import numpy as np
from scipy.special import erf

def rotzMat(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta),  np.cos(theta), 0],
                     [            0,              0, 1]])

def rotz(theta, v):
    """
    Rotate a vector by theta around the z axis
    """
    rz = rotzMat(theta)
    return np.dot(rz, v)
    
def getXVal(borders, x, z, r):
    n2 = borders['n2']
    o1 = borders['o1']
    o2 = borders['o2']
    a2 = (-z - o1[-1])/n2[-1]
    a1 = (-z - o2[-1])/n2[-1]
    v2 = rotz(r, a2*n2 + o1)
    v1 = rotz(r, a1*n2 + o2)
    #return (v2, v1)
    return max(v1[0], v2[0]) + x

def getZVal(borders, x, z, r):
    n1 = borders['n1']
    o1 = borders['o1']
    n_r = rotz(r, n1)
    o_r = rotz(r, o1)
    a = (-x - o_r[0])/n_r[0]
    v = a*n_r + o_r
    return v[2] + z

class SynXDetector(Device):
    val = Cpt(SynSignal, kind='hinted')
    Imax = Cpt(Signal, value=10, kind='config')
    center = Cpt(Signal, value=0, kind='config')
    sigma = Cpt(Signal, value=1, kind='config')
    noise = Cpt(EnumSignal, value='none', kind='config',
                enum_strings=('none', 'poisson', 'uniform'))
    noise_multiplier = Cpt(Signal, value=1, kind='config')

    def _compute(self):
        x = self._samplex.read()[self._samplex_field]['value']
        z = self._samplez.read()[self._samplez_field]['value']
        r = self._sampler.read()[self._sampler_field]['value']
        xval = getXVal(self.borders, x, z, r)
        v = self.Imax.get()*(erf(-xval*self.sigma.get()) + 1)
        return v
        
    def __init__(self, name, samplex, samplez, sampler, 
                 borders, sigma=1, noise="none", noise_multiplier=1,
                 random_state=None, **kwargs):
        super().__init__(name=name, **kwargs)
        self._samplez = samplez
        self._samplex = samplex
        self._sampler = sampler
        self._samplez_field = 'samplez'
        self._samplex_field = 'samplex'
        self._sampler_field = 'sampler'
        self.borders = borders
        
        self.center.put(0)
        self.Imax.put(1)
        self.sigma.put(sigma)
        self.noise.put(noise)
        self.noise_multiplier.put(noise_multiplier)

        if random_state is None:
            random_state = np.random
        self.random_state = random_state
        self.val.name = self.name
        self.val.sim_set_func(self._compute)

        self.trigger()

    def trigger(self, *args, **kwargs):
        return self.val.trigger(*args, **kwargs)

samplex = SynAxis(name='samplex')
samplez = SynAxis(name='samplez')
sampler = SynAxis(name='sampler')

def vec(x, y, z):
    return np.array([x, y, z])

def normVector(v):
    n = np.sqrt(np.dot(v, v))
    return v/n

def findOrthonormal(v1, v2):
    v3 = np.cross(v1, v2)
    return normVector(v3)

def constructBasis(p1, p2, p3):
    v1 = p3 - p1
    v2 = p2 - p1
    n2 = normVector(v2)
    n3 = findOrthonormal(v1, n2)
    n1 = findOrthonormal(n2, n3)
    return n1, n2, n3

def constructBorders(p1, p2, p3, sidelength=1):
    n1, n2, n3 = constructBasis(p1, p2, p3)
    origin = p1
    origin2 = n1*sidelength + origin
    borders = {'n1': n1, 'n2': n2,
               'o1': origin, 'o2': origin2}
    return borders

#need to fix imports, test, etc, actually hook up the max logic, and do a derivative scan version
#figure out if I can use bluesky-live instead? See thread
def find_max(plan, dets, *args):
    src = SingleRunCache()
    @bpp.subs_decorator(src.callback)
    def inner_maximizer():
        yield from plan(dets, *args)
        run = src.retrieve()
        table = run.primary.read()
    
    
p1 = vec(1, 0, 0)
p2 = vec(0, 0, 1)
p3 = vec(0, 0, 0)

borders = constructBorders(p1, p2, p3)

detX = SynXDetector("detX", samplex, samplez, sampler, borders)
