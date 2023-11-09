import os
import sys
import importlib

from linkml.visualize import polyscope

sys.path.insert(
    0, '/home/mephisto/repos/linkml_cpp/build/')

try:
    import linkml_py
    import numpy as np
except Exception as e:
    print(e)
    exit()

print("ID: ", os.getpid())

points = np.load("/home/mephisto/repos/LinkML/Data/points.npy")
normals = np.load("/home/mephisto/repos/LinkML/Data/normals.npy")


# points = points[::5000,:]
# normals = normals[::5000, :]


planes = np.load("/home/mephisto/repos/LinkML/Data/planes2.npy")
planes = [ linkml_py.Plane(p[0],p[1],p[2],p[3]) for p in planes.reshape((-1,4))]
# points = []

# for p,d,fs in os.walk("/home/mephisto/repos/LinkML/Notebooks/Plane Fitting/points2"):
#     fs.sort()

#     for f in fs:
#         points.append(np.load(os.path.join(p,f)))
# for pts, p in zip(points, planes):
#     p._points = pts


print(points.shape, normals.shape)
cloud = linkml_py.PointCloud.from_numpy(points, normals)
print(cloud)

# result = linkml_py.fit_planes(
#     cloud, linkml_py.PlaneFittingParams(
#         cosalpha=0.96592583,
#         normal_distance_threshhold=1,  # normal_distance_threshhold=13.
#         distance_threshhold=0.15,
#         plane_size_threshhold=500))
# print(result)


result = linkml_py.create_cell_complex(cloud, planes)
print(len(result))


with polyscope() as ps:

    ps.register_point_cloud("Cloud", points, radius=0.001)
    
    for idx, cell in enumerate(result):
        vertices = np.array(cell.vertecies)
        faces = np.array(cell.faces)
        ps.register_surface_mesh(f"Cell {idx:03.0f}", vertices,faces , transparency=1)