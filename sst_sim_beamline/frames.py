import numpy as np

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

def changeBasisMatrix(n1, n2, n3):
    return np.vstack([n1, n2, n3]).T

def constructBorders(p1, p2, p3, sidelength=1):
    n1, n2, n3 = constructBasis(p1, p2, p3)
    origin = p1
    origin2 = n1*sidelength + origin
    borders = {'n1': n1, 'n2': n2,
               'o1': origin, 'o2': origin2}
    return borders

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

def deg_to_rad(d):
    return d*np.pi/180.0

def rad_to_deg(d):
    return d*180.0/np.pi

class Frame:
    def __init__(self, p1, p2, p3):
        self.p0 = p1
        self._basis = constructBasis(p1, p2, p3)
        # r_offset
        self.r0 = self._roffset()
        self.A = changeBasisMatrix(*self._basis)
        self.Ainv = self.A.T

    def _roffset(self):
        n3 = self._basis[-1]
        theta = np.arctan(n3[1]/n3[0])
        return theta
        
    def frame_to_global(self, v_frame, manip=vec(0, 0, 0), r=0):
        """
        Find the global coordinates of a point in the frame, given the
        rotation of the frame, and the manipulator position
        v_frame: frame vector
        manip: current position of manipulator
        r: rotation of the frame (0 = grazing incidence, 90 = normal)
        """
        
        theta = deg_to_rad(r) - self.r0
        Rz = rotzMat(theta)
        v_global = np.dot(Rz, np.dot(self.A, v_frame) + self.p0) + manip
        return v_global

    def global_to_frame(self, v_global, manip=vec(0, 0, 0), r=0):
        """
        Find the frame coordinates of a point in the global system, 
        given the manipulator position and rotation

        v_global: global vector
        manip: current position of manipulator
        r: rotation of the manipulator
        """
        theta = deg_to_rad(r)
        Rz_inv = rotzMat(-theta)
        v_frame = np.dot(self.Ainv, np.dot(Rz_inv, v_global - manip) - self.p0)
        return v_frame

    def frame_to_beam(self, fx, fy, fz, fr=0):
        """
        Given a frame coordinate, and rotation, find the manipulator position and rotation
        that places the frame coordinate in the beam path
        
        return coordinate tuple
        """
        v_frame = vec(fx, fy, fz)
        v_global = -1*self.frame_to_global(v_frame, r=fr)
        gr = fr - rad_to_deg(self.r0)
        gx, gy, gz = (v_global[0], v_global[1], v_global[2])
        return gx, gy, gz, gr

    def beam_to_frame(self, gx, gy, gz, gr=0):
        """
        Given a manipulator coordinate and rotation, find the beam intersection position and 
        incidence angle in the frame coordinates.

        return coordinate tuple
        """
        manip = vec(gx, gy, gz)
        v_frame = self.project_beam_to_frame_xy(manip, gr)
        fx, fy, fz = (v_frame[0], v_frame[1], v_frame[2])
        fr = gr + rad_to_deg(self.r0)
        return fx, fy, fz, fr
        
    def origin_to_frame(self, manip=vec(0, 0, 0), r=0):
        return self.global_to_frame(vec(0, 0, 0), manip, r)
        
    def project_beam_to_frame_xy(self, manip=vec(0, 0, 0), r=0):
        op = self.origin_to_frame(manip, r)
        theta = deg_to_rad(r)
        Rz_inv = rotzMat(-theta)
        vp = np.dot(self.Ainv, np.dot(Rz_inv, vec(0, 1, 0)))
        a = op[-1]/vp[-1]
        proj = op - a*vp
        return proj
