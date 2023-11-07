import os
import sys
import importlib

sys.path.insert(
    0, '/home/mephisto/repos/linkml_cpp/build')

try:
    import linkml_py
    import numpy as np
except Exception as e:
    print(e)
    exit()

# points = np.load("./Test Points.npy")
# normals = np.load("./Test Normals.npy")

print("ID: ", os.getpid())

points = np.random.random((100,3))
normals = np.random.random((100,3))

cloud = linkml_py.PointCloud.from_numpy(points, normals)
print(cloud)

result = linkml_py.fit_planes(
    cloud, linkml_py.PlaneFittingParams(
        cosalpha=0.96592583,
        normal_distance_threshhold=1,  # normal_distance_threshhold=13.
        distance_threshhold=0.15,
        plane_size_threshhold=500))
print(result)

result = linkml_py.create_cell_complex(cloud, result)
print(result)