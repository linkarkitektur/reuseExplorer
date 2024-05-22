from specklepy.objects.geometry import Box, Plane, Point, Vector, Base, Pointcloud
from specklepy.objects.primitive import Interval
from specklepy.objects.units import Units
from ._core import PointCloud

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured

WORLDXY = Plane(
    origin=Point(x=0.0,y=0.0, z=0.0),
    normal=Vector(x=0.0,y=0.0, z=1.0),
    xdir=Vector(x=1.0,y=0.0, z=0.0),
    ydir=Vector(x=0.0,y=1.0, z=0.0))


def PointCloudtoBase(cloud: PointCloud) -> Base:

    # Acessing the point cloud as structured numpy array
    data = np.asarray(cloud)

    # Convert the structured numpy array to unstructured numpy array
    xyz = structured_to_unstructured(data[["x","y","z"]])

    # Transform and rotate the point cloud to have Z point upwards
    # I've had issues with the python kernal crashing due to a missmatch between openMP version.
    # Though I am not sure where those versions are defined.
    # To me it seems that this should be a numpy operation and have nothing to do with linkML
    # But I also don't think that I am statically linking the openMP library.
    xyz = xyz @ np.array([
        [1,0,0],
        [0,0,1],
        [0,1,0]])
    
    # Flatten the array and convert it to a list
    # This is the format that Specklepy expects
    xyz = xyz.flatten().tolist()

    # Extract the color information as integers
    rgb = data["rgb"].view("u4").tolist()

    bbox = cloud.bbox()
    xS, xE = bbox.xInterval()
    yS, yE = bbox.yInterval()
    zS, zE = bbox.zInterval()

        
    bbox = Box(
                basePlane = WORLDXY,
                xSize = Interval(start = xS, end = xE),
                ySize = Interval(start = yS, end = yE),
                zSize = Interval(start = zS, end = zE),
                volume = bbox.volume()
            )
    
    sp_cloud =  Pointcloud(
        points = xyz,
        colors = rgb,
        sizes = [ 3 for _ in range(int(len(xyz)/3))],
        bbox = bbox,
        units = Units.m,
    )
    sp_cloud.add_chunkable_attrs(points=31250, colors=62500, sizes=62500)
    sp_cloud.add_detachable_attrs(["points", "colors", "sizes"])    

    return sp_cloud