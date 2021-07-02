from ophyd import Device, Signal, Component as Cpt
from ophyd.sim import SynSignal, EnumSignal
import numpy as np
from scipy.special import erf

class SynI1(Device):
    val = Cpt(SynSignal, kind='hinted')
    Imax = Cpt(Signal, value=10, kind='config')
    center = Cpt(Signal, value=0, kind='config')
    sigma = Cpt(Signal, value=1, kind='config')
    noise = Cpt(EnumSignal, value='none', kind='config',
                enum_strings=('none', 'poisson', 'uniform'))
    noise_multiplier = Cpt(Signal, value=1, kind='config')

    def _compute(self):
        x, y, _, _ = self._manipulator.position
        #xval chosen to be inverted due to axis definition
        # finish updating once I've tested Manipulator class
        dist = -1*min(x, y)
        v = self.Imax.get()*self._norm_erf(dist)
        return v

    def _norm_erf(self, x, sigma=1):
        return 0.5*(erf(x*sigma) + 1)

    # need to fix detector to use positioner
    def __init__(self, name, manipulator, sigma=1, noise="none", noise_multiplier=1,
                 random_state=None, **kwargs):
        super().__init__(name=name, **kwargs)
        self._manipulator = manipulator        
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

