
from specklepy.objects.geometry import  Base
from specklepy.objects.geometry import Mesh as Speckle_Mesh
from specklepy.objects.units import Units

from .aabb import AABBToBase

from .._core import Mesh

def MeshToBase(mesh:Mesh) -> Base:
    return Speckle_Mesh.create(
        # Swap y and z because of the point cloud is in a Y-up coordinate system
        vertices= [ v for point in mesh.vertices() for v in [ point.x, point.z, point.y ] ],
        faces= mesh.faces()
    )
    # return Speckle_Mesh(
    #     # Swap y and z because of the point cloud is in a Y-up coordinate system
    #     units = Units.m,
    #     area = mesh.area(),
    #     volume = mesh.volume(),
    #     bbox = AABBToBase(mesh.bbox()),
    #     vertices = [ v for point in mesh.vertices() for v in [ point.x, point.z, point.y ] ],
    #     faces = mesh.faces(),
    #     colors = [], # mesh.colors(),
    #     texture_coordinates = [] # None, #mesh.textrueCoords()
    # )