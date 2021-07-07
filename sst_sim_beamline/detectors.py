from ophyd import Device, Signal, Component as Cpt
from ophyd.sim import SynSignal, EnumSignal
import numpy as np
from scipy.special import erf
from frames import vec

def norm_erf(x, width=1):
    return 0.5*(erf(2.0*x/width) + 1)

class SynI1(Device):
    val = Cpt(SynSignal, kind='hinted')
    Imax = Cpt(Signal, value=1, kind='config')
    center = Cpt(Signal, value=0, kind='config')
    width = Cpt(Signal, value=1, kind='config')
    noise = Cpt(EnumSignal, value='none', kind='config',
                enum_strings=('none','uniform', 'normal'))
    noise_multiplier = Cpt(Signal, value=0.1, kind='config')
    noise_sigma = Cpt(Signal, value=0.1, kind='config')

    def _compute(self):
        dist = -1*self._manipulator.distance_to_beam()
        width = self.width.get()
        center = self.center.get()
        Imax = self.Imax.get()
        noise = self.noise.get()

        v = Imax*norm_erf(dist, width)
        if noise == "normal":
            noise_sigma = self.noise_sigma.get()
            v = self.random_state.normal(v, noise_sigma)
        elif noise == "uniform":
            noise_multiplier = self.noise_multiplier.get()
            v += self.random_state.uniform(-1, 1)*noise_multiplier
        return v

    # need to fix detector to use positioner
    def __init__(self, name, manipulator, width=1, noise="none", noise_sigma=0.1, noise_multiplier=0.1,
                 random_state=None, **kwargs):
        super().__init__(name=name, **kwargs)
        self._manipulator = manipulator        
        self.center.put(0)
        self.Imax.put(1)
        self.width.put(width)
        self.noise.put(noise)
        self.noise_sigma.put(noise_sigma)
        self.noise_multiplier.put(noise_multiplier)

        if random_state is None:
            random_state = np.random
        self.random_state = random_state
        self.val.name = self.name
        self.val.sim_set_func(self._compute)
        self.trigger()

    def trigger(self, *args, **kwargs):
        return self.val.trigger(*args, **kwargs)

