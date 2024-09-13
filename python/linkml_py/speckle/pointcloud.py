from specklepy.objects.geometry import Base
from specklepy.objects.geometry import Pointcloud as Speckle_Pointcloud
from specklepy.objects.units import Units


from .aabb import AABBToBase

from .._core import PointCloud

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured


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

    
    sp_cloud =  Speckle_Pointcloud(
        points = xyz,
        colors = rgb,
        sizes = [ 3 for _ in range(int(len(xyz)/3))],
        bbox = AABBToBase(cloud.bbox()),
        units = Units.m,
    )
    sp_cloud.add_chunkable_attrs(points=31250, colors=62500, sizes=62500)
    sp_cloud.add_detachable_attrs(["points", "colors", "sizes"])    

    return sp_cloud



