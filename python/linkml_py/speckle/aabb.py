from specklepy.objects import Base 
from specklepy.objects.geometry import Box, Interval
from .WORLDXY import WORLDXY


from .._core import AABB


def AABBToBase(aabb: AABB) -> Base:

    xS, xE = aabb.xInterval()
    yS, yE = aabb.yInterval()
    zS, zE = aabb.zInterval()

    box = Box(
                basePlane = WORLDXY,
                # Swap y and z because of the point cloud is in a Y-up coordinate system
                xSize = Interval(start = xS, end = xE),
                # ySize = Interval(start = yS, end = yE),
                # zSize = Interval(start = zS, end = zE),
                ySize = Interval(start = zS, end = zE),
                zSize = Interval(start = yS, end = yE),
                volume = aabb.volume()
            )

    return box