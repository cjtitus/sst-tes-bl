import numpy as np

def vec(*args):
    return np.array(args)

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
        x = n3[0]
        y = n3[1]
        theta = np.arctan(y/x)
        if x > 0 and y > 0:
            quad = 1
        elif x < 0 and y > 0:
            quad = 2
        elif x < 0 and y < 0:
            quad = 3
        else:
            quad = 4
        if quad == 1:
            return theta
        elif quad == 2 or quad == 3:
            return theta + np.pi
        elif quad == 4:
            return theta + 2*np.pi
        
    def frame_to_global(self, v_frame, manip=vec(0, 0, 0), r=0, rotation="frame"):
        """
        Find the global coordinates of a point in the frame, given the
        rotation of the frame, and the manipulator position
        v_frame: frame vector
        manip: current position of manipulator
        r: rotation of the frame (0 = grazing incidence, 90 = normal)
        """
        if rotation == 'frame':
            theta = deg_to_rad(r) - self.r0
        else:
            theta = deg_to_rad(r)
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
        v_frame = self.origin_to_frame(manip, gr)
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

class Bar():
    def __init__(self, *args, width, height, nsides):
        self.interior_angle = 360.0/nsides
        self.width = width
        self.height = height
        self.sides = []
        current_side = Side(*args, width=width, height=height)
        self.sides.append(current_side)
        for n in range(nsides - 1):
            new_side = self._newSideFromSide(current_side, self.interior_angle)
            self.sides.append(new_side)
            current_side = new_side
        self.current_side = self.sides[0]
        
    def _newSideFromSide(self, side, angle):
        prev_edges = side.real_edges(vec(0, 0, 0), 0)
        new_vector = vec(np.cos(np.pi - deg_to_rad(angle)), 0, -np.sin(np.pi - deg_to_rad(angle)))
        p1 = prev_edges[1]
        p2 = prev_edges[2]
        p3 = side.frame_to_global(new_vector + side.edges[1], r=0, rotation="global")
        return Side(p1, p2, p3, width=self.width, height=self.height)

    def distance_to_beam(self, x, y, z, r):
        distances = [side.distance_to_beam(x, y, z, r) for side in self.sides]
        return np.max(distances)

    def beam_to_frame(self, *args):
        return self.current_side.beam_to_frame(*args)

    def frame_to_beam(self, *args):
        return self.current_side.frame_to_beam(*args)

    def change_side(self, n):
        self.current_side = self.sides[n]
    
class Side(Frame):
    def __init__(self, *args, width=19.5, height=130):
        super().__init__(*args)
        self.width=width
        self.height=height
        self.edges = [vec(0, 0, 0), vec(width, 0, 0), vec(width, height, 0), vec(0, height, 0)]

    def real_edges(self, manip, r_manip):
        re = []
        for edge in self.edges:
            real_coord = self.frame_to_global(edge, manip, r_manip, rotation='global')
            re.append(real_coord)
        return re

    def project_real_edges(self, manip, r_manip):
        re = self.real_edges(manip, r_manip)
        ret = []
        for edge in re:
            ret.append(np.array([edge[0], edge[2]]))
        return ret

    def distance_to_beam(self, x, y, z, r):
        """
        r manipulator
        """
        manip = vec(x, y, z)
        real_edges = self.project_real_edges(manip, r)
        inPoly = isInPoly(vec(0, 0), *real_edges)
        distance = getMinDist(vec(0, 0), *real_edges)
        if inPoly:
            return distance
        else:
            return -1*distance
        
def triarea(p1, p2, p3):
    n1 = p1 - p2
    n2 = p3 - p2
    return 0.5*(n1[0]*n2[1] - n1[1]*n2[0])

def vec_len(v):
    return np.sqrt(np.dot(v, v))

def vec_angle(v1, v2):
    return np.arccos(np.dot(v1, v2)/(vec_len(v1)*vec_len(v2)))

def getPointAreas(p, *args):
    areas = []
    for n in range(len(args)):
        areas.append(triarea(p, args[n-1], args[n]))
    return areas

def distFromTri(p, a, b):
    area = np.abs(triarea(p, a, b))
    d = vec_len(a - b)
    s1 = vec_len(a - p)
    s2 = vec_len(b - p)
    if np.isclose(d, 0):
        return min(s1, s2)
    elif s1**2 > d**2 + s2**2:
        return s2
    elif s2**2 > d**2 + s1**2:
        return s1
    else:
        return 2.0*area/d

def getMinDist(p, *args):
    distances = []
    for n in range(len(args)):
        distances.append(distFromTri(p, args[n-1], args[n]))
    return np.min(distances)

def prunePoints(*args):
    pruned = []
    for n in range(len(args)):
        if not np.all(np.isclose(args[n-1] - args[n], 0.0)):
            pruned.append(args[n])
    return pruned
        
def isInPoly(p, *args):
    polyPoints = prunePoints(*args)
    areas = np.array(getPointAreas(p, *polyPoints))
    return (np.all(areas < 0) or np.all(areas > 0))
        
