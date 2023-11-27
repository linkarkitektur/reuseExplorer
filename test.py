import os
import sys
import importlib


print(os.getpid())


sys.path.insert(
    0, '/home/mephisto/repos/linkml_cpp/build/')

try:
    import linkml_py
    import numpy as np
except Exception as e:
    print(e)
    exit()

# points = np.load("/home/mephisto/repos/LinkML/Data/points.npy")
# normals = np.load("/home/mephisto/repos/LinkML/Data/normals.npy")

# points =  np.concatenate((points[::5000,:],  np.atleast_2d(np.min(points, axis=0)),  np.atleast_2d(np.max(points, axis=0))))
# normals = np.concatenate((normals[::5000,:], np.atleast_2d(np.min(normals, axis=0)), np.atleast_2d(np.max(normals, axis=0))))


# planes = np.load("/home/mephisto/repos/LinkML/Data/planes2.npy")
# planes = [ linkml_py.Plane(p[0],p[1],p[2],p[3]) for p in planes.reshape((-1,4))]
# points = []

# for p,d,fs in os.walk("/home/mephisto/repos/LinkML/Notebooks/Plane Fitting/points2"):
#     fs.sort()

#     for f in fs:
#         points.append(np.load(os.path.join(p,f)))
# for pts, p in zip(points, planes):
#     p._points = pts


# print(points.shape, normals.shape)
# cloud = linkml_py.PointCloud.from_numpy(points, normals)
# print(cloud)

# result = linkml_py.fit_planes(
#     cloud, linkml_py.PlaneFittingParams(
#         cosalpha=0.96592583,
#         normal_distance_threshhold=1,  # normal_distance_threshhold=13.n
#         distance_threshhold=0.15,
#         plane_size_threshhold=500))
# print(result)


# result = linkml_py.create_cell_complex(cloud, planes)
# print(len(result))


# with polyscope() as ps:

#     ps.set_ground_plane_mode("shadow_only")

#     ps.register_point_cloud("Cloud", points, radius=0.001)
    
#     for idx, cell in enumerate(result):
#         vertices = np.array(cell.vertecies)
#         faces = np.array(cell.faces)
#         ps.register_surface_mesh(f"Cell {idx:03.0f}", vertices,faces , transparency=0.8)




## Rifinement Test

points = np.load("/home/mephisto/server_data/Amtssygehus/points.npy")
normals = np.load("/home/mephisto/server_data/Amtssygehus/normals.npy")
colors = np.load("/home/mephisto/server_data/Amtssygehus/colors.npy")
planes = np.load("/home/mephisto/server_data/Amtssygehus/planes.npy")

# Fix, scale & shape
points = points*0.1
planes = planes.reshape((-1,4))

cloud = linkml_py.PointCloud.from_numpy(points, normals)
planes = [linkml_py.Plane(plane[0], plane[1], plane[2], plane[3]) for plane in planes]

points_pp = []
indecies = []
for path, folders, files in os.walk("/home/mephisto/server_data/Amtssygehus/planes/"):

    # Sorting
    keys = [int(name.split(".")[0].split("-")[1]) for name in files]
    files = [x for _, x in sorted(zip(keys, files))]


    for file in files:
        if "points" in file:
            points_pp.append(np.load(os.path.join(path, file)))
        elif "indecies" in file:
            indecies.append(np.load(os.path.join(path, file)))


plane_results = linkml_py.PlaneFittingResults.from_numpy(planes, indecies)

print("Plans before refinement:", plane_results)

params = linkml_py.Refinement_Parameters()
res = linkml_py.refine_planes(cloud, plane_results, params)

print("N Plans after refinement:", len(res))
