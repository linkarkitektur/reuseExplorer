from typing import List, Optional
from specklepy.objects.geometry import Brep as Speckle_Brep
from specklepy.objects.geometry import Polyline, Box, Point, Base, BrepLoop, BrepEdge, BrepFace, BrepTrim, Surface
from specklepy.objects.primitive import Interval
from specklepy.objects.units import Units

from .aabb import AABBToBase
from .mesh import MeshToBase

from .._core import  Pos, Pos2D


def ListPosToPolyline(points: List[Pos]) -> Polyline:

    ply = Polyline()

    value = [value for p in points for value in [p.x, p.z, p.y]], # Swap y and z because of the point cloud is in a Y-up coordinate system
    closed =  True,
    domain = Interval(start = 0, end = 1),
    bbox = None
    area = None
    length = None 

    return ply


def ListPos2DToPolyline( points: List[Pos2D]) -> Polyline:
    ply = Polyline()

    value = [value for p in points for value in [p.x, p.y, 0]],
    closed =  True,
    domain = Interval(start = 0, end = 1),
    bbox = None
    area = None
    length = None 

    return ply