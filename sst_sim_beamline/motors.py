from ophyd.sim import SynAxis
from ophyd.positioner import SoftPositioner
from ophyd import PseudoPositioner, PseudoSingle
from ophyd import Component as Cpt
from ophyd.pseudopos import pseudo_position_argument, real_position_argument, _to_position_tuple

class Manipulator(PseudoPositioner):
    # Find some way to record all motors that moved!
    sx = Cpt(PseudoSingle)
    sy = Cpt(PseudoSingle)
    sz = Cpt(PseudoSingle)
    sr = Cpt(PseudoSingle)
    
    x = Cpt(SoftPositioner, name='x', init_pos=0.0)
    y = Cpt(SoftPositioner, name='y', init_pos=0.0)
    z = Cpt(SoftPositioner, name='z', init_pos=0.0)
    r = Cpt(SoftPositioner, name='r', init_pos=0.0)
    def __init__(self, bar, *args, **kwargs):
        self.bar = bar
        super().__init__(*args, **kwargs)

    @pseudo_position_argument
    def forward(self, pp):
        rx, ry, rz, rr = self.bar.frame_to_beam(pp.sx, pp.sy, pp.sz, pp.sr)
        return self.RealPosition(x=rx, y=ry, z=rz, r=rr)

    @real_position_argument
    def inverse(self, rp):
        sx, sy, sz, sr = self.bar.beam_to_frame(rp.x, rp.y, rp.z, rp.r)
        return self.PseudoPosition(sx=sx, sy=sy, sz=sz, sr=sr)

    def to_pseudo_tuple(self, *args, **kwargs):
        return _to_position_tuple(self.PseudoPosition, *args, **kwargs, _cur=lambda: self.position)

    def distance_to_beam(self):
        x, y, z, r = self.real_position
        return self.bar.distance_to_beam(x, y, z, r)
