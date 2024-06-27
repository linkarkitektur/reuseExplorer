from typing import List, Optional
from specklepy.objects.geometry import Brep as Speckle_Brep
from specklepy.objects.geometry import  Point, Base, BrepEdge, BrepLoop, BrepFace, BrepTrim, Surface, Line, BrepTrimType, BrepLoopType
from specklepy.objects.primitive import Interval
from specklepy.objects.units import Units

from .aabb import AABBToBase
from .mesh import MeshToBase
from .polyline import ListPosToPolyline, ListPos2DToPolyline

from .._core import Brep

def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def BrepToBase(brep: Brep ) -> Base:

    sp_brep = Speckle_Brep()
    sp_brep.provenance = "LinkML"
    sp_brep.units = Units.m

    sp_brep.bbox = AABBToBase(brep.bbox())
    sp_brep.area = brep.area()
    sp_brep.volume = brep.volume()
    sp_brep.IsClosed = brep.is_closed()
    sp_brep.Orientation = brep.Orientation()

    sp_brep.displayValue = [MeshToBase(brep.get_mesh())]

    sp_brep.Curve3D = [Line(
            # Swap y and z because of the point cloud is in a Y-up coordinate system
            start = Point(x = points[0].x, y = points[0].z, z = points[0].y),
            end = Point(x = points[-1].x, y= points[-1].z, z = points[-1].y),
            domain = Interval(start = 0, end = 1),
            # bbox: Optional[Box] = None
            # length: Optional[float] = None
        ) for points in brep.Curves3D()]

    # Swap y and z because of the point cloud is in a Y-up coordinate system
    sp_brep.Vertices = [ Point(x=p.x, y=p.z,z=p.y) for p in brep.Vertices() ] 

    # Surfaces be generated before the 2D curves
    sp_brep.Surfaces = [
        Surface(
            degreeU = s.degreeU,
            degreeV = s.degreeV,
            rational = s.rational,
            # area = s.area,
            # Swap y and z because of the point cloud is in a Y-up coordinate system
            countU = s.countU,
            countV = s.countV,
            # bbox = AABBToBase(s.bbox),
            closedU = s.closedU,
            closedV = s.closedV,
            domainU = Interval(start = s.domainU[0], end = s.domainU[1]),
            domainV = Interval(start = s.domainV[0], end = s.domainV[1]),
            pointData = [ v for p in chunks(s.pointData,4) for v in [p[0], p[2], p[1], p[3]]],
            knotsU = s.knotsU,
            knotsV = s.knotsV
        ) for s in brep.Surfaces()]
    
    # Be sure to fist construct the surfaces before the 2D curves
    sp_brep.Curve2D = [ Line(
            start = Point(x = points[0].x, y = points[0].y, z = 0),
            end = Point(x = points[-1].x, y= points[-1].y, z = 0),
            domain = Interval(start = 0, end = 1),
            # bbox: Optional[Box] = None
            # length: Optional[float] = None
        ) for points in brep.Curves2D()]
    
    sp_brep.Edges = [BrepEdge(
        Curve3dIndex = e.Curve3dIndex,
        TrimIndices = e.TrimIndices,
        StartIndex = e.StartIndex,
        EndIndex = e.EndIndex,
        ProxyCurveIsReversed = e.ProxyCurveIsReversed,
        Domain = Interval(
            start = e.Domain[0],
            end = e.Domain[1])
    ) for e in brep.Edges()]

    sp_brep.Loops = [
            BrepLoop(
            FaceIndex = l.FaceIndex,
            TrimIndices = l.TrimIndices,
            Type = BrepLoopType.Unknown ,#l.Type    
        ) for l in brep.Loops()]

    sp_brep.Faces = [
        BrepFace(
            SurfaceIndex = f.SurfaceIndex,
            OuterLoopIndex = f.OuterLoopIndex,
            OrientationReversed = f.OrientationReversed,
            LoopIndices = [f.LoopIndices]
        ) for f in brep.Faces()]
    
    sp_brep.Trims = [
        BrepTrim(
            EdgeIndex = t.EdgeIndex,
            StartIndex = t.StartIndex,
            EndIndex = t.EndIndex,
            FaceIndex = t.FaceIndex,
            LoopIndex = t.LoopIndex,
            CurveIndex = t.CurveIndex,
            IsoStatus = t.IsoStatus,
            TrimType = BrepTrimType.Unknown,# t.TrimType,
            IsReversed = t.IsReversed
            # Domain = Interval( start = t.Domain[0], end = t.Domain[0])
        ) for t in brep.Trims()]

    return sp_brep